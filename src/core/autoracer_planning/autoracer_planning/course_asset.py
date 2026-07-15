from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path


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


@dataclass(frozen=True)
class RuntimeCourseSample:
    s: float
    x: float
    y: float
    z: float
    yaw: float
    curvature: float
    left_offset: float
    right_offset: float
    target_velocity: float
    target_acceleration: float


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_runtime_course_asset(asset_dir: Path, map_path: Path):
    manifest = json.loads((asset_dir / "manifest.json").read_text(encoding="utf-8"))
    schema_version = manifest.get("schema_version")
    if schema_version == 2:
        from .fixed_course import load_course_asset, validate_course_map_contract

        legacy_manifest, samples = load_course_asset(asset_dir)
        validate_course_map_contract(legacy_manifest, map_path)
        return legacy_manifest, samples
    if schema_version != 3:
        raise ValueError(f"unsupported course manifest schema: {schema_version!r}")
    if manifest.get("production_method") != "rc_recorded_super_lio":
        raise ValueError("unsupported schema-3 production method")
    if manifest.get("frame_id") != "map":
        raise ValueError("recorded course frame must be map")
    if (
        manifest.get("map_id") != map_path.name
        or manifest.get("map", {}).get("id") != map_path.name
    ):
        raise ValueError("recorded course/map ID mismatch")
    validation = manifest.get("validation", {})
    if validation.get("status") != "PASS":
        raise ValueError("recorded course validation status is not PASS")
    if validation.get("terminal_stop") is not True:
        raise ValueError("recorded course has no validated terminal stop")

    assets = manifest.get("assets", {})
    for filename in ("course.csv", "validation.json"):
        _validate_hash(asset_dir / filename, assets.get(filename, {}), "asset")
    validation_asset = json.loads(
        (asset_dir / "validation.json").read_text(encoding="utf-8")
    )
    if validation_asset.get("status") != "PASS" or validation_asset.get(
        "terminal_stop"
    ) is not True:
        raise ValueError("recorded course validation artifact is not PASS")

    map_contract = manifest.get("map", {})
    for key, filename in (
        ("pointcloud_map_metadata_sha256", "pointcloud_map_metadata.yaml"),
        ("map_projector_info_sha256", "map_projector_info.yaml"),
    ):
        path = map_path / filename
        expected = map_contract.get(key)
        if not path.is_file() or expected != sha256_file(path):
            raise ValueError(f"recorded course map contract mismatch for {filename}")

    samples = _load_course_csv(asset_dir / "course.csv")
    expected_rows = assets["course.csv"].get("rows")
    if expected_rows != len(samples):
        raise ValueError("recorded course row count mismatch")
    if samples[-1].target_velocity != 0.0:
        raise ValueError("recorded course terminal speed is not zero")
    return manifest, samples


def _validate_hash(path: Path, contract: dict, label: str) -> None:
    if not path.is_file():
        raise ValueError(f"recorded course {label} is missing: {path.name}")
    expected = contract.get("sha256")
    actual = sha256_file(path)
    if expected != actual:
        raise ValueError(
            f"recorded course {label} hash mismatch for {path.name}: "
            f"expected {expected!r}, got {actual!r}"
        )


def _load_course_csv(path: Path) -> list[RuntimeCourseSample]:
    samples = []
    with path.open("r", encoding="ascii", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != COURSE_COLUMNS:
            raise ValueError("recorded course CSV schema mismatch")
        for row in reader:
            values = {name: float(row[name]) for name in COURSE_COLUMNS}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("recorded course contains non-finite values")
            samples.append(RuntimeCourseSample(**values))
    if len(samples) < 2:
        raise ValueError("recorded course has fewer than two samples")
    if samples[0].s != 0.0 or not all(
        current.s > previous.s for previous, current in zip(samples, samples[1:])
    ):
        raise ValueError("recorded course distance is not strictly increasing from zero")
    if not all(
        sample.left_offset > 0.0
        and sample.right_offset > 0.0
        and sample.target_velocity >= 0.0
        for sample in samples
    ):
        raise ValueError("recorded course contains invalid offsets or speed")
    return samples
