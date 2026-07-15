from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable

from .map_manifest import sha256_file, validate_map_manifest


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
class CourseSample:
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


def write_course_csv(path: Path, samples: Iterable[CourseSample]) -> None:
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=COURSE_COLUMNS)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {name: f"{getattr(sample, name):.9f}" for name in COURSE_COLUMNS}
            )


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
    if manifest.get("runtime_contract") != "fixed_course_v1":
        raise ValueError("unsupported fixed-course runtime contract")
    if manifest.get("frame_id") != "map":
        raise ValueError("fixed course frame must be map")
    if (
        manifest.get("map_id") != map_path.name
        or manifest.get("map", {}).get("id") != map_path.name
    ):
        raise ValueError("fixed course/map ID mismatch")
    validation = manifest.get("validation", {})
    if validation.get("status") != "PASS":
        raise ValueError("fixed course validation status is not PASS")
    if validation.get("terminal_stop") is not True:
        raise ValueError("fixed course has no validated terminal stop")

    assets = manifest.get("assets", {})
    for filename in ("course.csv", "validation.json"):
        _validate_hash(asset_dir / filename, assets.get(filename, {}), "asset")
    validation_asset = json.loads(
        (asset_dir / "validation.json").read_text(encoding="utf-8")
    )
    if validation_asset.get("status") != "PASS" or validation_asset.get(
        "terminal_stop"
    ) is not True:
        raise ValueError("fixed course validation artifact is not PASS")

    map_manifest_path = map_path / "map_manifest.json"
    map_contract = manifest.get("map", {})
    if (
        map_contract.get("id") != map_path.name
        or not map_manifest_path.is_file()
        or map_contract.get("manifest_sha256") != sha256_file(map_manifest_path)
    ):
        raise ValueError("fixed course map manifest contract mismatch")
    map_manifest = json.loads(map_manifest_path.read_text(encoding="utf-8"))
    try:
        validate_map_manifest(map_path, map_manifest)
    except ValueError as exc:
        raise ValueError(f"fixed course map contract mismatch: {exc}") from exc

    samples = load_course_csv(asset_dir / "course.csv")
    expected_rows = assets["course.csv"].get("rows")
    if expected_rows != len(samples):
        raise ValueError("fixed course row count mismatch")
    if samples[-1].target_velocity != 0.0:
        raise ValueError("fixed course terminal speed is not zero")
    return manifest, samples


def _validate_hash(path: Path, contract: dict, label: str) -> None:
    if not path.is_file():
        raise ValueError(f"fixed course {label} is missing: {path.name}")
    expected = contract.get("sha256")
    actual = sha256_file(path)
    if expected != actual:
        raise ValueError(
            f"fixed course {label} hash mismatch for {path.name}: "
            f"expected {expected!r}, got {actual!r}"
        )


def load_course_csv(path: Path) -> list[CourseSample]:
    samples = []
    with path.open("r", encoding="ascii", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != COURSE_COLUMNS:
            raise ValueError("fixed course CSV schema mismatch")
        for row in reader:
            values = {name: float(row[name]) for name in COURSE_COLUMNS}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("fixed course contains non-finite values")
            samples.append(CourseSample(**values))
    if len(samples) < 2:
        raise ValueError("fixed course has fewer than two samples")
    if samples[0].s != 0.0 or not all(
        current.s > previous.s for previous, current in zip(samples, samples[1:])
    ):
        raise ValueError("fixed course distance is not strictly increasing from zero")
    if not all(
        sample.left_offset > 0.0
        and sample.right_offset > 0.0
        and sample.target_velocity >= 0.0
        for sample in samples
    ):
        raise ValueError("fixed course contains invalid offsets or speed")
    return samples
