import csv
import hashlib
import json
from pathlib import Path

import pytest

from autoracer_planning.course_asset import load_runtime_course_asset


COLUMNS = (
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


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_rc_asset(tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    map_path = tmp_path / "test_map"
    map_path.mkdir()
    metadata = map_path / "pointcloud_map_metadata.yaml"
    projector = map_path / "map_projector_info.yaml"
    metadata.write_text("x_resolution: 20\ny_resolution: 20\n", encoding="utf-8")
    projector.write_text("projector_type: Local\n", encoding="utf-8")

    asset = tmp_path / "course"
    asset.mkdir()
    course = asset / "course.csv"
    with course.open("w", newline="", encoding="ascii") as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerow(dict(zip(COLUMNS, (0, 0, 0, 0, 0, 0, 0.4, 0.4, 0.1, 0))))
        writer.writerow(dict(zip(COLUMNS, (1, 1, 0, 0, 0, 0, 0.4, 0.4, 0, -0.005))))
    validation = asset / "validation.json"
    validation.write_text('{"status":"PASS","terminal_stop":true}\n', encoding="utf-8")
    manifest = {
        "schema_version": 3,
        "production_method": "rc_recorded_super_lio",
        "map_id": "test_map",
        "frame_id": "map",
        "assets": {
            "course.csv": {"sha256": digest(course), "rows": 2},
            "validation.json": {"sha256": digest(validation)},
        },
        "map": {
            "id": "test_map",
            "pointcloud_map_metadata_sha256": digest(metadata),
            "map_projector_info_sha256": digest(projector),
        },
        "validation": {"status": "PASS", "terminal_stop": True},
    }
    (asset / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return asset, map_path


def test_recorded_rc_asset_loads_without_carmaker_roadeval(tmp_path):
    asset, map_path = make_rc_asset(tmp_path)
    manifest, samples = load_runtime_course_asset(asset, map_path)
    assert manifest["production_method"] == "rc_recorded_super_lio"
    assert [sample.x for sample in samples] == [0.0, 1.0]


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda manifest, asset, map_path: manifest.update(frame_id="base_link"), "frame"),
        (lambda manifest, asset, map_path: manifest.update(map_id="other"), "map ID"),
        (
            lambda manifest, asset, map_path: manifest["validation"].update(status="FAIL"),
            "validation",
        ),
        (
            lambda manifest, asset, map_path: manifest["validation"].update(
                terminal_stop=False
            ),
            "terminal stop",
        ),
    ],
)
def test_recorded_rc_manifest_rejects_invalid_contract(tmp_path, mutation, message):
    asset, map_path = make_rc_asset(tmp_path)
    path = asset / "manifest.json"
    manifest = json.loads(path.read_text())
    mutation(manifest, asset, map_path)
    path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_runtime_course_asset(asset, map_path)


def test_recorded_rc_asset_rejects_modified_course_or_map(tmp_path):
    asset, map_path = make_rc_asset(tmp_path)
    with (asset / "course.csv").open("a", encoding="ascii") as stream:
        stream.write("corrupt\n")
    with pytest.raises(ValueError, match="hash mismatch"):
        load_runtime_course_asset(asset, map_path)

    asset, map_path = make_rc_asset(tmp_path / "second")
    (map_path / "pointcloud_map_metadata.yaml").write_text("changed\n")
    with pytest.raises(ValueError, match="map contract mismatch"):
        load_runtime_course_asset(asset, map_path)
