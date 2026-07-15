import json

import pytest

from autoracer_planning.map_manifest import build_map_manifest, validate_map_manifest


def _write_pcd(path, points):
    path.write_bytes(
        (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {points}\n"
            "HEIGHT 1\n"
            f"POINTS {points}\n"
            "DATA binary\n"
        ).encode("ascii")
        + b"\0" * (points * 16)
    )


def make_map(tmp_path, map_id="test_map"):
    map_path = tmp_path / map_id
    tiles = map_path / "pointcloud_map.pcd"
    tiles.mkdir(parents=True)
    (map_path / "pointcloud_map_metadata.yaml").write_text(
        "x_resolution: 20\ny_resolution: 20\n", encoding="utf-8"
    )
    (map_path / "map_projector_info.yaml").write_text(
        "projector_type: Local\n", encoding="utf-8"
    )
    _write_pcd(tiles / "tile_0_0.pcd", 2)
    _write_pcd(tiles / "tile_20_0.pcd", 3)
    return map_path


def test_map_manifest_binds_every_pointcloud_tile(tmp_path):
    map_path = make_map(tmp_path)
    manifest = build_map_manifest(map_path)

    assert manifest["map_id"] == "test_map"
    assert manifest["frame_id"] == "map"
    assert manifest["pointcloud"]["tile_count"] == 2
    assert manifest["pointcloud"]["total_points"] == 5
    assert [tile["path"] for tile in manifest["pointcloud"]["tiles"]] == [
        "pointcloud_map.pcd/tile_0_0.pcd",
        "pointcloud_map.pcd/tile_20_0.pcd",
    ]
    validate_map_manifest(map_path, manifest)


def test_map_manifest_rejects_tile_change_or_unlisted_tile(tmp_path):
    map_path = make_map(tmp_path)
    manifest = build_map_manifest(map_path)
    (map_path / "pointcloud_map.pcd" / "tile_0_0.pcd").write_bytes(b"changed")
    with pytest.raises(ValueError, match="tile contract mismatch"):
        validate_map_manifest(map_path, manifest)

    map_path = make_map(tmp_path / "second")
    manifest = build_map_manifest(map_path)
    _write_pcd(map_path / "pointcloud_map.pcd" / "unlisted.pcd", 1)
    with pytest.raises(ValueError, match="tile set mismatch"):
        validate_map_manifest(map_path, manifest)


def test_map_manifest_round_trips_as_stable_json(tmp_path):
    manifest = build_map_manifest(make_map(tmp_path))
    encoded = json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    assert json.loads(encoded) == manifest
