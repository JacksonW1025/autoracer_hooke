from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
RC_LIDAR_HOST_IP = "192.168.1.102"
OBSOLETE_LIDAR_HOST_IP = ".".join(("192", "168", "1", str(100 + 20)))


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_rc_c32_host_defaults_use_reserved_link_address():
    expected_defaults = {
        "defaults.env": f'LIDAR_HOST_IP:={RC_LIDAR_HOST_IP}',
        "scripts/configure_rc_lidar_link.sh": f'LIDAR_HOST_IP:-{RC_LIDAR_HOST_IP}',
        "scripts/rc/rc_configure_lidar.sh": "configure_rc_lidar_link.sh",
    }

    for relative_path, expected in expected_defaults.items():
        assert expected in read(relative_path)

    link_script = read("scripts/configure_rc_lidar_link.sh")
    assert "LIDAR_IFACE:-enP8p1s0" in link_script
    assert "default: eth0" not in link_script

    run_script = read("scripts/run_track.sh")
    assert "require_lidar_link_ready" in run_script
    assert "LIDAR_LINK_WAIT_SEC" in run_script
    assert "ip route get" in run_script
    assert "/sys/class/net/${LIDAR_IFACE}/carrier" in run_script
    assert (
        "if lidar_route_ready && lidar_carrier_ready; then\n"
        "    return 0\n"
        "  fi\n\n"
        "  try_configure_lidar_link"
    ) in run_script
    assert "sudo -E ./scripts/rc/rc_configure_lidar.sh" in run_script


def test_rc_user_docs_do_not_recommend_obsolete_host_address():
    canonical_link_docs = [
        "README.md",
        "docs/reference/interfaces_and_topics_zh.md",
    ]

    for relative_path in canonical_link_docs:
        content = read(relative_path)
        assert RC_LIDAR_HOST_IP in content

    current_user_docs = [
        "README.md",
        "docs/README_zh.md",
        "docs/architecture/platform_and_stack_zh.md",
        "docs/architecture/runtime_alignment_audit_zh.md",
        "docs/operations/rc_full_chain_execution_zh.md",
        "docs/operations/rc_runbook_zh.md",
        "docs/operations/mapping_workflow_zh.md",
        "docs/reference/interfaces_and_topics_zh.md",
        "docs/reference/calibration_zh.md",
    ]

    for relative_path in current_user_docs:
        content = read(relative_path)
        assert OBSOLETE_LIDAR_HOST_IP not in content


def test_repo_does_not_carry_obsolete_lidar_host_address():
    tracked_files = subprocess.check_output(
        ["git", "ls-files"],
        cwd=ROOT,
        text=True,
    ).splitlines()
    forbidden = OBSOLETE_LIDAR_HOST_IP.encode()

    offenders = []
    for relative_path in tracked_files:
        path = ROOT / relative_path
        if forbidden in path.read_bytes():
            offenders.append(relative_path)

    assert offenders == []
