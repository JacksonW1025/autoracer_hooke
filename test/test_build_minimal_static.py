from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = REPO_ROOT / "scripts" / "build_minimal.sh"


def test_build_minimal_defaults_to_release_build_type() -> None:
    script = BUILD_SCRIPT.read_text()

    assert 'CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"' in script
    assert '-DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"' in script


def test_build_minimal_uses_cmake_release_type_not_manual_o3_flags() -> None:
    script = BUILD_SCRIPT.read_text()

    cxx_flags_lines = [
        line for line in script.splitlines() if line.startswith("CMAKE_CXX_FLAGS=")
    ]
    assert cxx_flags_lines
    assert all("-O3" not in line and "-DNDEBUG" not in line for line in cxx_flags_lines)
