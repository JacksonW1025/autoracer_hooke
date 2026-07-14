# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[5]
RC_DEPENDENCIES = Path(__file__).resolve().parents[1]
RESOLVER = RC_DEPENDENCIES / "resolve_rc_vendor.py"
ROSDEP_ENTRY = RC_DEPENDENCIES / "install_rc_rosdeps.sh"
VENDOR_ENTRY = RC_DEPENDENCIES / "build_rc_vendor.sh"
PRODUCT_ENTRY = RC_DEPENDENCIES / "build_rc_product.sh"
ENTRY_POINTS = (ROSDEP_ENTRY, VENDOR_ENTRY, PRODUCT_ENTRY)


def resolved_records():
    result = subprocess.run(
        [sys.executable, str(RESOLVER), "--format", "records"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    return [tuple(line.split("\t")) for line in result.stdout.splitlines()]


def write_executable(path, body):
    path.write_text(f"#!{sys.executable}\n{body}", encoding="utf-8")
    path.chmod(0o755)


def make_fake_tools(tmp_path, colcon_names=None):
    tools = tmp_path / "fake-bin"
    tools.mkdir()
    log = tmp_path / "tool-log.jsonl"
    write_executable(
        tools / "rosdep",
        """import json
import os
from pathlib import Path
import sys

with Path(os.environ["RC_TEST_TOOL_LOG"]).open("a", encoding="utf-8") as stream:
    stream.write(json.dumps({"tool": "rosdep", "argv": sys.argv[1:]}) + "\\n")
raise SystemExit(int(os.environ.get("FAKE_ROSDEP_EXIT", "0")))
""",
    )
    write_executable(
        tools / "colcon",
        """import json
import os
from pathlib import Path
import sys

with Path(os.environ["RC_TEST_TOOL_LOG"]).open("a", encoding="utf-8") as stream:
    stream.write(json.dumps({"tool": "colcon", "argv": sys.argv[1:]}) + "\\n")
if sys.argv[1:2] == ["list"]:
    sys.stdout.write(os.environ.get("FAKE_COLCON_NAMES", ""))
raise SystemExit(int(os.environ.get("FAKE_COLCON_EXIT", "0")))
""",
    )
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": str(tools) + os.pathsep + environment.get("PATH", ""),
            "RC_TEST_TOOL_LOG": str(log),
        }
    )
    if colcon_names is not None:
        environment["FAKE_COLCON_NAMES"] = "".join(
            f"{name}\n" for name in colcon_names
        )
    return {"log": log, "env": environment}


def make_vendor_workspace(tmp_path):
    workspace = tmp_path / "rc-vendor-ws"
    source_root = workspace / "src"
    source_root.mkdir(parents=True)
    (workspace / "rc-vendor-filtered.repos").write_text(
        "repositories: {}\n", encoding="utf-8"
    )
    for name, package_path in resolved_records():
        package = source_root / package_path
        package.mkdir(parents=True)
        (package / "package.xml").write_text(
            f'<package format="3"><name>{name}</name></package>\n',
            encoding="utf-8",
        )
    return workspace


def run_entry(entry_point, workspace, tools):
    return subprocess.run(
        ["bash", str(entry_point), "--workspace", str(workspace)],
        cwd=ROOT,
        env=tools["env"],
        text=True,
        capture_output=True,
    )


def read_tool_log(tools):
    if not tools["log"].exists():
        return []
    return [json.loads(line) for line in tools["log"].read_text().splitlines()]


def option_values(argv, option, next_option):
    start = argv.index(option) + 1
    end = argv.index(next_option, start)
    return argv[start:end]


def test_entry_points_are_rc_owned_and_have_no_platform_dispatch():
    for entry_point in ENTRY_POINTS:
        assert entry_point.is_file()
        assert os.access(entry_point, os.X_OK)
        text = entry_point.read_text(encoding="utf-8")
        lowered = text.lower()
        assert "autoracer_platform" not in lowered
        assert "src/platform/hooke" not in lowered
        assert "autoracer_hooke" not in lowered
        assert "/scripts/" not in lowered
        assert "scripts/build" not in lowered
        assert "scripts/install" not in lowered


def test_rosdep_receives_only_resolved_vendor_core_and_rc_paths(tmp_path):
    records = resolved_records()
    workspace = make_vendor_workspace(tmp_path)
    tools = make_fake_tools(tmp_path)

    result = run_entry(ROSDEP_ENTRY, workspace, tools)

    assert result.returncode == 0, result.stderr
    log = read_tool_log(tools)
    assert [entry["tool"] for entry in log] == ["rosdep"]
    assert log[0]["argv"] == [
        "install",
        "--from-paths",
        *[str(workspace / "src" / path) for _, path in records],
        str(ROOT / "src/core"),
        str(ROOT / "src/platform/rc"),
        "--ignore-src",
        "-y",
        "-r",
    ]


def test_vendor_build_verifies_then_selects_the_exact_82_package_closure(tmp_path):
    records = resolved_records()
    package_names = [name for name, _ in records]
    workspace = make_vendor_workspace(tmp_path)
    tools = make_fake_tools(tmp_path, colcon_names=package_names)

    result = run_entry(VENDOR_ENTRY, workspace, tools)

    assert result.returncode == 0, result.stderr
    assert len(package_names) == 82
    log = read_tool_log(tools)
    assert [entry["tool"] for entry in log] == ["colcon", "colcon"]
    assert log[0]["argv"] == [
        "list",
        "--base-paths",
        str(workspace / "src"),
        "--names-only",
    ]
    build = log[1]["argv"]
    assert build[0] == "build"
    assert option_values(build, "--base-paths", "--symlink-install") == [
        str(workspace / "src")
    ]
    assert option_values(build, "--packages-select", "--cmake-args") == package_names
    assert "--packages-up-to" not in build
    assert "-DBUILD_TESTING=OFF" in build


def test_product_build_uses_only_frozen_core_and_rc_and_targets_rc_bringup(tmp_path):
    workspace = tmp_path / "rc-product-ws"
    workspace.mkdir()
    tools = make_fake_tools(tmp_path)

    result = run_entry(PRODUCT_ENTRY, workspace, tools)

    assert result.returncode == 0, result.stderr
    log = read_tool_log(tools)
    assert [entry["tool"] for entry in log] == ["colcon"]
    build = log[0]["argv"]
    assert build[0] == "build"
    assert option_values(build, "--base-paths", "--symlink-install") == [
        str(ROOT / "src/core"),
        str(ROOT / "src/platform/rc"),
    ]
    assert option_values(build, "--packages-up-to", "--cmake-args") == [
        "autoracer_rc_bringup"
    ]
    assert "--packages-select" not in build
    assert "-DBUILD_TESTING=OFF" in build


@pytest.mark.parametrize("entry_point", ENTRY_POINTS)
def test_invalid_workspace_stops_before_external_tools(entry_point, tmp_path):
    workspace = tmp_path / "ordinary-workspace"
    workspace.mkdir()
    tools = make_fake_tools(tmp_path)

    result = run_entry(entry_point, workspace, tools)

    assert result.returncode != 0
    assert "rc" in result.stderr.lower()
    assert read_tool_log(tools) == []


@pytest.mark.parametrize("entry_point", (ROSDEP_ENTRY, VENDOR_ENTRY))
def test_incomplete_vendor_workspace_stops_before_external_tools(entry_point, tmp_path):
    workspace = make_vendor_workspace(tmp_path)
    _, missing_path = resolved_records()[0]
    shutil.rmtree(workspace / "src" / missing_path)
    tools = make_fake_tools(tmp_path)

    result = run_entry(entry_point, workspace, tools)

    assert result.returncode != 0
    assert "rc" in result.stderr.lower()
    assert read_tool_log(tools) == []


@pytest.mark.parametrize("entry_name", ("install_rc_rosdeps.sh", "build_rc_vendor.sh"))
def test_empty_resolver_selection_stops_before_external_tools(entry_name, tmp_path):
    fake_root = tmp_path / "fake-product"
    fake_dependencies = fake_root / "src/platform/rc/dependencies"
    fake_dependencies.mkdir(parents=True)
    shutil.copyfile(RC_DEPENDENCIES / entry_name, fake_dependencies / entry_name)
    (fake_dependencies / "resolve_rc_vendor.py").write_text("", encoding="utf-8")
    workspace = tmp_path / "rc-empty-ws"
    (workspace / "src").mkdir(parents=True)
    (workspace / "rc-vendor-filtered.repos").write_text(
        "repositories: {}\n", encoding="utf-8"
    )
    tools = make_fake_tools(tmp_path)

    result = run_entry(fake_dependencies / entry_name, workspace, tools)

    assert result.returncode != 0
    assert "empty" in result.stderr.lower()
    assert read_tool_log(tools) == []


def test_failed_exact_verification_stops_before_vendor_build(tmp_path):
    package_names = [name for name, _ in resolved_records()]
    workspace = make_vendor_workspace(tmp_path)
    tools = make_fake_tools(tmp_path, colcon_names=package_names[:-1])

    result = run_entry(VENDOR_ENTRY, workspace, tools)

    assert result.returncode != 0
    assert "verification" in result.stderr.lower()
    log = read_tool_log(tools)
    assert len(log) == 1
    assert log[0]["tool"] == "colcon"
    assert log[0]["argv"][0] == "list"


def test_product_build_requires_a_fresh_empty_workspace(tmp_path):
    workspace = tmp_path / "rc-product-ws"
    workspace.mkdir()
    (workspace / "stale-build").mkdir()
    tools = make_fake_tools(tmp_path)

    result = run_entry(PRODUCT_ENTRY, workspace, tools)

    assert result.returncode != 0
    assert "fresh" in result.stderr.lower()
    assert read_tool_log(tools) == []
