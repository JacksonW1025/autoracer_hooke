from pathlib import Path


def _repository_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "core").is_dir():
            return parent
    raise RuntimeError("repository root not found")


REPOSITORY_ROOT = _repository_root()
CORE_ROOT = REPOSITORY_ROOT / "src" / "core"
FORBIDDEN_CORE_TOKENS = (
    "hooke2",
    "fixposition",
    "pandar",
    "nebula",
    "lslidar",
    "hipnuc",
    "rc_serial",
    "safe_control_cmd",
)


def _product_sources(root: Path) -> str:
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in root.rglob("*")
        if path.suffix in {".py", ".xml", ".yaml", ".yml"}
        and "test" not in path.parts
    ).lower()


def test_core_sources_have_no_platform_tokens():
    sources = _product_sources(CORE_ROOT)

    for token in FORBIDDEN_CORE_TOKENS:
        assert token not in sources


def test_core_manifests_do_not_depend_on_platform_packages():
    manifests = "\n".join(
        path.read_text(encoding="utf-8")
        for path in CORE_ROOT.glob("*/package.xml")
    ).lower()

    assert "autoracer_hooke2" not in manifests
    assert "autoracer_rc" not in manifests
    assert "hooke2_" not in manifests
