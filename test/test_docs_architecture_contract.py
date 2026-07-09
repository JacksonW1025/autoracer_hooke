from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"


FORMAL_DOCS = [
    DOCS / "README_zh.md",
    DOCS / "architecture" / "platform_and_stack_zh.md",
    DOCS / "architecture" / "official_launch_structure_zh.md",
    DOCS / "architecture" / "official_migration_zh.md",
    DOCS / "architecture" / "runtime_alignment_audit_zh.md",
    DOCS / "operations" / "rc_full_chain_execution_zh.md",
    DOCS / "operations" / "rc_runbook_zh.md",
    DOCS / "operations" / "mapping_workflow_zh.md",
    DOCS / "reference" / "interfaces_and_topics_zh.md",
    DOCS / "reference" / "calibration_zh.md",
]


OLD_DOC_NAMES = [
    "rc_hooke_platform_boundary_zh.md",
    "rc_run_readiness_checklist_zh.md",
    "rc_autoware_full_workflow_zh.md",
    "autoracer_hooke_chain_audit",
    "rc_official_autoware_diff_audit_zh.md",
    "sensing_feedback_topics.md",
    "hooke2_chassis_chain.md",
    "calibration_checklist.md",
    "minimal_stack.md",
]


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_docs_readme_indexes_formal_docs():
    text = read(DOCS / "README_zh.md")
    for path in FORMAL_DOCS:
        if path.name == "README_zh.md":
            continue
        rel = path.relative_to(DOCS).as_posix()
        assert f"`{rel}`" in text
    assert "`superpowers/" not in text


def test_no_reader_role_table_or_old_doc_names():
    combined = "\n".join(read(path) for path in FORMAL_DOCS if path.exists())
    assert "| 文档 | 角色 | 什么时候读 |" not in combined
    for name in OLD_DOC_NAMES:
        assert name not in combined


def test_platform_doc_has_two_architecture_diagrams():
    text = read(DOCS / "architecture" / "platform_and_stack_zh.md")
    assert "## Hooke 底盘架构图" in text
    assert "## RC 底盘架构图" in text
    assert text.count("```mermaid") == 2
    assert text.count("```") % 2 == 0
    assert "Shared official Autoware upper stack" in text
    assert "rc_serial_interface" in text
    assert "hooke2_interface" in text


def test_current_runtime_docs_default_to_official_planning_control_boundary():
    platform = read(DOCS / "architecture" / "platform_and_stack_zh.md")
    runbook = read(DOCS / "operations" / "rc_runbook_zh.md")
    interfaces = read(DOCS / "reference" / "interfaces_and_topics_zh.md")
    current_runtime_docs = "\n".join((platform, runbook, interfaces))

    required_terms = [
        "official Autoware planning/control",
        "自研 planning/control 候选",
        "/control/command/control_cmd",
        "/autoracer/control/safe_control_cmd",
        "rc_serial_interface",
    ]
    for term in required_terms:
        assert term in current_runtime_docs

    stale_default_terms = [
        "lanelet_route_planner",
        "pure_pursuit_controller",
        "/autoracer/control/raw_control_cmd",
    ]
    for term in stale_default_terms:
        assert term not in current_runtime_docs


def test_formal_docs_do_not_keep_stale_host_or_old_control_language():
    combined = "\n".join(read(path) for path in FORMAL_DOCS if path.exists())

    stale_terms = [
        "树莓派",
        "192.168.1.136",
        "raw control",
        "/autoracer/control/raw_control_cmd",
        "后续实现计划",
    ]
    for term in stale_terms:
        assert term not in combined


def test_current_phase_does_not_describe_future_state_fusion():
    combined = "\n".join(read(path) for path in FORMAL_DOCS if path.exists())
    forbidden_terms = ["卡尔曼", "Kalman", "EKF", "ekf"]
    for term in forbidden_terms:
        assert term not in combined


def test_runtime_audit_records_pointcloud_filter_contract():
    text = read(DOCS / "architecture" / "runtime_alignment_audit_zh.md")
    assert "pointcloud_voxel_filter" in text
    assert "/sensing/lidar/concatenated/pointcloud" in text
    assert "/sensing/lidar/filtered/pointcloud" in text
    assert "official localization 默认消费 `/sensing/lidar/concatenated/pointcloud`" in text


def test_official_launch_structure_doc_explains_old_and_new_boundaries():
    text = read(DOCS / "architecture" / "official_launch_structure_zh.md")
    required_terms = [
        "旧结构图",
        "官方结构图",
        "autoracer_bringup/track.launch.py",
        "autoware_launch/autoware.launch.xml",
        "vehicle_model:=autoracer_hooke",
        "sensor_model:=autoracer_hooke_sensor_kit",
        "autoracer_hooke_description",
        "autoracer_hooke_launch",
        "autoracer_hooke_sensor_kit_description",
        "autoracer_hooke_sensor_kit_launch",
        "command_gate -> rc_serial_interface",
        "旧链路已从本分支移除",
        "模板分层",
        "src/external/autoware",
        "自研 planning 候选",
        "不在 `src/external/autoware` 里做隐形魔改",
    ]
    for term in required_terms:
        assert term in text
    assert text.count("```mermaid") >= 2
    assert "旧 `run_track.sh` 路径保留" not in text
    assert "旧链路回退" not in text


def test_full_chain_doc_preserves_audit_to_runtime_order():
    text = read(DOCS / "operations" / "rc_full_chain_execution_zh.md")
    required_terms = [
        "架构/Launch 差距审计",
        "车端传感器和 TF",
        "工作机 bag 检查和 Foxglove 查看",
        "Super-LIO 生成 PCD",
        "NDT localization-only",
        "planning/control/gate dry-run",
        "低速动态验证",
        "docs/architecture/runtime_alignment_audit_zh.md",
        "docs/operations/mapping_workflow_zh.md",
        "docs/operations/rc_runbook_zh.md",
    ]
    for term in required_terms:
        assert term in text
    assert "没有 Lanelet2 地图时只验证 localization-only" not in text
    assert "完整官方地图目录" in text


def test_static_architecture_png_is_documented_as_preview():
    assert (DOCS / "architecture" / "image.png").exists()
    readme = read(DOCS / "README_zh.md")
    platform = read(DOCS / "architecture" / "platform_and_stack_zh.md")
    assert "`architecture/image.png`" in readme
    assert "`docs/architecture/image.png`" in platform
    assert "静态预览图" in platform
    assert "Mermaid" in platform
    assert "runtime_alignment_audit_zh.md" in platform


def test_stale_superpowers_docs_are_removed_from_formal_docs():
    assert not (DOCS / "superpowers").exists()
