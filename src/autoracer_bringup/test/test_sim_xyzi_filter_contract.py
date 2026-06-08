from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CROP = PROJECT_ROOT / (
    "src/external/autoware/core/sensing/autoware_crop_box_filter/src/crop_box_filter_node.cpp"
)
VOXEL = PROJECT_ROOT / (
    "src/external/autoware/core/sensing/autoware_downsample_filters/src/"
    "voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.cpp"
)
FASTER_VOXEL = PROJECT_ROOT / (
    "src/external/autoware/core/sensing/autoware_downsample_filters/src/"
    "voxel_grid_downsample_filter/faster_voxel_grid_downsample_filter.cpp"
)


def test_migrated_crop_box_accepts_sim_xyzi_clouds():
    text = CROP.read_text(encoding="utf-8")

    assert "!is_data_layout_compatible_with_point_xyzi(*cloud)" in text
    assert "PointXYZI, PointXYZIRC, or PointXYZIRCAEDT" in text


def test_migrated_voxel_filter_accepts_sim_xyzi_clouds():
    text = VOXEL.read_text(encoding="utf-8")

    assert "!utils::is_data_layout_compatible_with_point_xyzi(*cloud)" in text
    assert "PointXYZI, PointXYZIRC, or PointXYZIRCAEDT" in text


def test_migrated_voxel_filter_preserves_float32_intensity():
    text = FASTER_VOXEL.read_text(encoding="utf-8")

    assert "PointField::UINT8" in text
    assert "PointField::FLOAT32" in text
    assert "reinterpret_cast<const float *>" in text
    assert "reinterpret_cast<float *>" in text
    assert "UINT8 or FLOAT32" in text
