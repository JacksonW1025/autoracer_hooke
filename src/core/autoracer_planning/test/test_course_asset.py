import csv
import hashlib
import json

import pytest

from autoracer_planning.course_asset import RuntimeCourseSample, load_runtime_course_asset
from autoracer_planning.fixed_course_publisher import course_asset_label, course_to_markers


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


def test_validated_course_markers_show_line_start_and_finish():
    def sample(s, x, y, z):
        return RuntimeCourseSample(s, x, y, z, 0, 0, 0.4, 0.4, 0.1, 0)

    markers = course_to_markers(
        {"frame_id": "map"},
        [sample(0, 1, 2, 3), sample(1, 4, 5, 6), sample(2, 7, 8, 9)],
    ).markers

    assert [marker.ns for marker in markers] == [
        "autoracer_fixed_course",
        "autoracer_fixed_course_start",
        "autoracer_fixed_course_finish",
    ]
    assert [marker.header.frame_id for marker in markers] == ["map"] * 3
    assert [(point.x, point.y, point.z) for point in markers[0].points] == [
        (1.0, 2.0, 3.15),
        (4.0, 5.0, 6.15),
        (7.0, 8.0, 9.15),
    ]
    assert (markers[1].pose.position.x, markers[1].pose.position.y) == (1.0, 2.0)
    assert (markers[2].pose.position.x, markers[2].pose.position.y) == (7.0, 8.0)
    assert (markers[1].color.r, markers[1].color.g, markers[1].color.b) == (0.0, 1.0, 0.0)
    assert (markers[2].color.r, markers[2].color.g, markers[2].color.b) == (0.0, 0.25, 1.0)


def test_recorded_course_log_label_uses_map_id_without_carmaker_version():
    assert course_asset_label({"map_id": "floor1_mapping_101"}) == "floor1_mapping_101"
    assert course_asset_label({"version": "urbanroad-v2", "map_id": "urbanroad"}) == "urbanroad-v2"
