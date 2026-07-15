#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

try:
    from .recorded_course import (
        PoseSample,
        RecordedCourseConfig,
        build_recorded_course,
    )
except ImportError:
    from recorded_course import (
        PoseSample,
        RecordedCourseConfig,
        build_recorded_course,
    )


COURSE_COLUMNS = (
    "s",
    "x",
    "y",
    "z",
    "yaw",
    "curvature",
    "left_offset",
    "right_offset",
    "target_velocity",
    "target_acceleration",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_odometry_bag(path: Path, topic: str, source_frame: str) -> list[PoseSample]:
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topic_type = topic_types.get(topic)
    if topic_type != "nav_msgs/msg/Odometry":
        raise ValueError(
            f"{topic} type must be nav_msgs/msg/Odometry, got {topic_type!r}"
        )

    poses = []
    while reader.has_next():
        current_topic, data, _ = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(data, Odometry)
        if message.header.frame_id != source_frame:
            raise ValueError(
                f"odometry frame mismatch: expected {source_frame!r}, "
                f"got {message.header.frame_id!r}"
            )
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        poses.append(
            PoseSample(
                stamp=stamp,
                x=position.x,
                y=position.y,
                z=position.z,
                qx=orientation.x,
                qy=orientation.y,
                qz=orientation.z,
                qw=orientation.w,
            )
        )
    if not poses:
        raise ValueError(f"odometry bag has no messages on {topic}")
    return poses


def _required_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def write_asset(
    output_dir: Path,
    map_id: str,
    source: dict,
    data_root: Path,
    poses: list[PoseSample],
) -> dict:
    if output_dir.exists():
        raise FileExistsError(f"refusing to replace existing output: {output_dir}")

    config = RecordedCourseConfig(**source["processing"])
    samples, validation = build_recorded_course(poses, config)
    map_path = data_root / source["map_path"]
    bag_metadata = _required_file(data_root / source["source_bag"] / "metadata.yaml")
    replay_metadata = _required_file(data_root / source["odometry_bag"] / "metadata.yaml")
    super_lio_config = _required_file(data_root / source["super_lio_config"])
    map_metadata = _required_file(map_path / "pointcloud_map_metadata.yaml")
    projector = _required_file(map_path / "map_projector_info.yaml")

    temporary = output_dir.with_name(f".{output_dir.name}.tmp-{os.getpid()}")
    if temporary.exists():
        raise FileExistsError(f"temporary output already exists: {temporary}")
    temporary.mkdir(parents=True)
    try:
        course_path = temporary / "course.csv"
        with course_path.open("w", encoding="ascii", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=COURSE_COLUMNS)
            writer.writeheader()
            for sample in samples:
                writer.writerow(
                    {name: f"{getattr(sample, name):.9f}" for name in COURSE_COLUMNS}
                )

        validation.update(
            {
                "status": "PASS",
                "map_frame": "map",
                "source_frame": source["source_frame"],
                "single_direction_recording": True,
            }
        )
        (temporary / "validation.json").write_text(
            json.dumps(validation, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        manifest = {
            "schema_version": 3,
            "production_method": "rc_recorded_super_lio",
            "map_id": map_id,
            "frame_id": "map",
            "source_frame": source["source_frame"],
            "assets": {
                "course.csv": {
                    "rows": len(samples),
                    "sha256": sha256(course_path),
                },
                "validation.json": {
                    "sha256": sha256(temporary / "validation.json"),
                },
            },
            "map": {
                "id": map_id,
                "pointcloud_map_metadata_sha256": sha256(map_metadata),
                "map_projector_info_sha256": sha256(projector),
            },
            "source": {
                "bag_metadata_sha256": sha256(bag_metadata),
                "odometry_bag_metadata_sha256": sha256(replay_metadata),
                "super_lio_config_sha256": sha256(super_lio_config),
            },
            "processing": asdict(config),
            "validation": validation,
            "limitations": [
                "The course is the recorded driven line, not a surveyed drivable-area model.",
                "Offsets are configured conservative metadata and are not online planning bounds.",
            ],
        }
        (temporary / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        temporary.replace(output_dir)
        return manifest
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build an RC fixed course from replayed Super-LIO odometry"
    )
    parser.add_argument("map_id")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--data-root",
        type=Path,
        default=Path(os.environ.get("AUTORACER_RC_DATA_ROOT", Path.cwd().parent)),
    )
    parser.add_argument(
        "--sources",
        type=Path,
        default=Path(__file__).with_name("recorded_course_sources.json"),
    )
    parser.add_argument("--odometry-topic", default="/lio/odom")
    args = parser.parse_args()

    sources = json.loads(args.sources.read_text(encoding="utf-8"))
    if args.map_id not in sources:
        parser.error(f"unknown map_id: {args.map_id}")
    source = sources[args.map_id]
    odometry_bag = args.data_root / source["odometry_bag"]
    poses = read_odometry_bag(
        odometry_bag, args.odometry_topic, source["source_frame"]
    )
    manifest = write_asset(args.output, args.map_id, source, args.data_root, poses)
    print(
        f"[recorded-course] {args.map_id}: "
        f"{manifest['assets']['course.csv']['rows']} points"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
