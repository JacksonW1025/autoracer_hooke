from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path


MAP_ASSET_FILES = ("map_projector_info.yaml", "pointcloud_map_metadata.yaml")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pcd_point_count(path: Path) -> int:
    with path.open("rb") as stream:
        for _ in range(128):
            line = stream.readline()
            if not line:
                break
            if line.startswith(b"POINTS "):
                try:
                    points = int(line.split()[1])
                except (IndexError, ValueError) as exc:
                    raise ValueError(f"invalid PCD POINTS header: {path}") from exc
                if points < 0:
                    raise ValueError(f"negative PCD point count: {path}")
                return points
            if line.startswith(b"DATA "):
                break
    raise ValueError(f"PCD has no POINTS header: {path}")


def build_map_manifest(map_path: Path) -> dict:
    map_path = Path(map_path)
    if not map_path.is_dir():
        raise ValueError(f"map directory does not exist: {map_path}")
    assets = {}
    for filename in MAP_ASSET_FILES:
        path = map_path / filename
        if not path.is_file():
            raise ValueError(f"map asset is missing: {path}")
        assets[filename] = {
            "size_bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }

    tiles = []
    for path in sorted(map_path.rglob("*.pcd")):
        if not path.is_file():
            continue
        points = pcd_point_count(path)
        tiles.append(
            {
                "path": path.relative_to(map_path).as_posix(),
                "size_bytes": path.stat().st_size,
                "points": points,
                "sha256": sha256_file(path),
            }
        )
    if not tiles:
        raise ValueError(f"map contains no PCD tiles: {map_path}")
    return {
        "schema_version": 1,
        "status": "PASS",
        "map_id": map_path.name,
        "frame_id": "map",
        "assets": assets,
        "pointcloud": {
            "tile_count": len(tiles),
            "total_points": sum(tile["points"] for tile in tiles),
            "tiles": tiles,
        },
    }


def validate_map_manifest(map_path: Path, manifest: dict) -> None:
    map_path = Path(map_path)
    if (
        manifest.get("schema_version") != 1
        or manifest.get("status") != "PASS"
        or manifest.get("map_id") != map_path.name
        or manifest.get("frame_id") != "map"
    ):
        raise ValueError("invalid map manifest identity or status")
    assets = manifest.get("assets", {})
    for filename in MAP_ASSET_FILES:
        path = map_path / filename
        contract = assets.get(filename, {})
        if (
            not path.is_file()
            or contract.get("size_bytes") != path.stat().st_size
            or contract.get("sha256") != sha256_file(path)
        ):
            raise ValueError(f"map asset contract mismatch: {filename}")

    pointcloud = manifest.get("pointcloud", {})
    tile_contracts = pointcloud.get("tiles")
    if not isinstance(tile_contracts, list) or not tile_contracts:
        raise ValueError("map manifest has no pointcloud tile contracts")
    actual_paths = {
        path.relative_to(map_path).as_posix()
        for path in map_path.rglob("*.pcd")
        if path.is_file()
    }
    expected_paths = {contract.get("path") for contract in tile_contracts}
    if actual_paths != expected_paths:
        raise ValueError("pointcloud tile set mismatch")
    total_points = 0
    for contract in tile_contracts:
        relative = Path(contract["path"])
        path = (map_path / relative).resolve()
        try:
            path.relative_to(map_path.resolve())
        except ValueError as exc:
            raise ValueError(f"pointcloud tile escapes map directory: {relative}") from exc
        if (
            contract.get("size_bytes") != path.stat().st_size
            or contract.get("sha256") != sha256_file(path)
        ):
            raise ValueError(f"pointcloud tile contract mismatch: {relative}")
        points = pcd_point_count(path)
        if contract.get("points") != points:
            raise ValueError(f"pointcloud tile contract mismatch: {relative}")
        total_points += points
    if (
        pointcloud.get("tile_count") != len(tile_contracts)
        or pointcloud.get("total_points") != total_points
    ):
        raise ValueError("pointcloud aggregate contract mismatch")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a portable pointcloud map manifest")
    parser.add_argument("map_path", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()
    output = args.output or args.map_path / "map_manifest.json"
    if output.exists() and not args.replace:
        parser.error(f"output already exists: {output}")
    manifest = build_map_manifest(args.map_path)
    validate_map_manifest(args.map_path, manifest)
    temporary = output.with_name(f".{output.name}.tmp-{os.getpid()}")
    temporary.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, output)
    print(
        f"[map-manifest] {manifest['map_id']}: "
        f"{manifest['pointcloud']['tile_count']} tiles, "
        f"{manifest['pointcloud']['total_points']} points"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
