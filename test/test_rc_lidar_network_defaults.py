from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RC_LIDAR_HOST_IP = "192.168.1.120"
CONFLICTING_HOST_IP = "192.168.1.102"


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_rc_c32_host_defaults_use_reserved_link_address():
    expected_defaults = {
        "defaults.env": f'LIDAR_HOST_IP:={RC_LIDAR_HOST_IP}',
        "scripts/configure_rc_lidar_link.sh": f'LIDAR_HOST_IP:-{RC_LIDAR_HOST_IP}',
        "scripts/check_sensor_status.sh": f'LIDAR_HOST_IP:-{RC_LIDAR_HOST_IP}',
        "scripts/verify_sensing_feedback.sh": f'DEFAULT_LIDAR_HOST_IP="{RC_LIDAR_HOST_IP}"',
    }

    for relative_path, expected in expected_defaults.items():
        assert expected in read(relative_path)


def test_rc_user_docs_do_not_recommend_conflicting_host_address():
    docs = [
        "README.md",
        "docs/sensing_feedback_topics.md",
        "docs/rc_run_readiness_checklist_zh.md",
    ]

    for relative_path in docs:
        content = read(relative_path)
        assert RC_LIDAR_HOST_IP in content
        assert CONFLICTING_HOST_IP not in content
