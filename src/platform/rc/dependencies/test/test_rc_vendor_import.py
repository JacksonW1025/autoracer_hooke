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
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[5]
RC_DEPENDENCIES = Path(__file__).resolve().parents[1]
IMPORTER = RC_DEPENDENCIES / "import_rc_vendor.py"

SELECTED_PACKAGES = {
    "alpha_pkg": "upstream/alpha/alpha_pkg",
    "beta_pkg": "upstream/beta/nested/beta_pkg",
}


def write_metadata(tmp_path):
    metadata = tmp_path / "metadata"
    metadata.mkdir()
    fixtures = {
        "rc_manifest": metadata / "rc-vendor-packages.txt",
        "package_manifest": metadata / "vendor-packages.tsv",
        "repositories": metadata / "autoracer.repos",
        "lock_file": metadata / "versions.lock.yaml",
    }
    fixtures["rc_manifest"].write_text("beta_pkg\nalpha_pkg\n", encoding="utf-8")
    fixtures["package_manifest"].write_text(
        "alpha_pkg\tupstream/alpha/alpha_pkg\n"
        "alpha_unused\tupstream/alpha/alpha_unused\n"
        "beta_pkg\tupstream/beta/nested/beta_pkg\n"
        "unused_pkg\tupstream/unused/unused_pkg\n",
        encoding="utf-8",
    )
    fixtures["repositories"].write_text(
        """repositories:
  upstream/alpha:
    type: git
    url: https://example.invalid/alpha.git
    version: 1111111111111111111111111111111111111111
  upstream/beta:
    type: git
    url: https://example.invalid/beta.git
    version: 2222222222222222222222222222222222222222
  upstream/unused:
    type: git
    url: https://example.invalid/unused.git
    version: 3333333333333333333333333333333333333333
""",
        encoding="utf-8",
    )
    fixtures["lock_file"].write_text(
        """schema_version: 1
repositories:
  - path: upstream/alpha
    url: https://example.invalid/alpha.git
    revision: 1111111111111111111111111111111111111111
  - path: upstream/beta
    url: https://example.invalid/beta.git
    revision: 2222222222222222222222222222222222222222
  - path: upstream/unused
    url: https://example.invalid/unused.git
    revision: 3333333333333333333333333333333333333333
patches:
""",
        encoding="utf-8",
    )
    return fixtures


def write_package(source_root, package_path, package_name):
    package = source_root / package_path
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        f"<package format=\"3\"><name>{package_name}</name></package>\n",
        encoding="utf-8",
    )
    (package / "payload.txt").write_text(f"{package_name}\n", encoding="utf-8")


def write_executable(path, body):
    path.write_text(f"#!{sys.executable}\n{body}", encoding="utf-8")
    path.chmod(0o755)


def make_fake_tools(tmp_path):
    tools = tmp_path / "fake-bin"
    tools.mkdir()
    log = tmp_path / "tool-log.jsonl"
    source_root = tmp_path / "fake-vcs-source"
    for name, package_path in SELECTED_PACKAGES.items():
        write_package(source_root, package_path, name)
    write_package(source_root, "upstream/alpha/alpha_unused", "alpha_unused")
    write_package(source_root, "upstream/unused/unused_pkg", "unused_pkg")

    write_executable(
        tools / "vcs",
        """import json
import os
from pathlib import Path
import shutil
import sys

document = sys.stdin.read()
with Path(os.environ["RC_TEST_TOOL_LOG"]).open("a", encoding="utf-8") as stream:
    stream.write(json.dumps({"tool": "vcs", "argv": sys.argv[1:], "stdin": document}) + "\\n")
exit_code = int(os.environ.get("FAKE_VCS_EXIT", "0"))
if exit_code:
    raise SystemExit(exit_code)
destination = Path(sys.argv[-1])
destination.mkdir(parents=True, exist_ok=True)
shutil.copytree(
    os.environ["FAKE_VCS_SOURCE"], destination, dirs_exist_ok=True, symlinks=True
)
if "FAKE_VCS_UNREADABLE" in os.environ:
    (destination / os.environ["FAKE_VCS_UNREADABLE"]).chmod(0)
""",
    )
    write_executable(
        tools / "colcon",
        """import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

with Path(os.environ["RC_TEST_TOOL_LOG"]).open("a", encoding="utf-8") as stream:
    stream.write(json.dumps({"tool": "colcon", "argv": sys.argv[1:]}) + "\\n")
exit_code = int(os.environ.get("FAKE_COLCON_EXIT", "0"))
if exit_code:
    raise SystemExit(exit_code)
if "FAKE_COLCON_NAMES" in os.environ:
    names = os.environ["FAKE_COLCON_NAMES"].splitlines()
else:
    base_path = Path(sys.argv[sys.argv.index("--base-paths") + 1])
    names = []
    for package_xml in sorted(base_path.rglob("package.xml")):
        names.append(ET.parse(package_xml).getroot().findtext("name"))
sys.stdout.write("".join(f"{name}\\n" for name in names))
""",
    )
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": str(tools) + os.pathsep + environment.get("PATH", ""),
            "RC_TEST_TOOL_LOG": str(log),
            "FAKE_VCS_SOURCE": str(source_root),
        }
    )
    return {"bin": tools, "log": log, "source": source_root, "env": environment}


def importer_command(workspace=None, fixtures=None):
    command = [sys.executable, str(IMPORTER)]
    if workspace is not None:
        command.extend(["--workspace", str(workspace)])
    if fixtures:
        command.extend(
            [
                "--rc-manifest",
                str(fixtures["rc_manifest"]),
                "--package-manifest",
                str(fixtures["package_manifest"]),
                "--repositories",
                str(fixtures["repositories"]),
                "--lock-file",
                str(fixtures["lock_file"]),
            ]
        )
    return command


def run_importer(workspace=None, fixtures=None, tools=None, extra_environment=None):
    environment = tools["env"].copy() if tools else os.environ.copy()
    if extra_environment:
        environment.update(extra_environment)
    return subprocess.run(
        importer_command(workspace, fixtures),
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
    )


def read_tool_log(tools):
    if not tools["log"].exists():
        return []
    return [json.loads(line) for line in tools["log"].read_text().splitlines()]


def assert_failed_rc_only(result):
    assert result.returncode != 0
    assert result.stdout == ""
    assert "rc vendor import" in result.stderr.lower()


def test_importer_requires_an_explicit_workspace(tmp_path):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)

    result = run_importer(fixtures=fixtures, tools=tools)

    assert result.returncode != 0
    assert "workspace" in result.stderr.lower()
    assert read_tool_log(tools) == []


def test_importer_rejects_non_rc_relative_and_legacy_workspaces(tmp_path):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)
    product_link = tmp_path / "product-link"
    product_link.symlink_to(ROOT, target_is_directory=True)
    invalid_workspaces = [
        Path("rc_vendor_ws"),
        tmp_path / "vendor_ws",
        tmp_path / "hooke2" / "rc_vendor_ws",
        tmp_path / "autoware" / "rc_vendor_ws",
        product_link / "rc_vendor_ws",
    ]

    for workspace in invalid_workspaces:
        result = run_importer(workspace, fixtures, tools)
        assert_failed_rc_only(result)
        assert not workspace.exists()

    assert read_tool_log(tools) == []


def test_resolver_metadata_is_validated_before_workspace_mutation(tmp_path):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)
    workspace = tmp_path / "rc_vendor_ws"
    fixtures["rc_manifest"].write_text("alpha_pkg\nalpha_pkg\n", encoding="utf-8")

    result = run_importer(workspace, fixtures, tools)

    assert_failed_rc_only(result)
    assert "duplicate" in result.stderr.lower()
    assert not workspace.exists()
    assert read_tool_log(tools) == []


def test_network_import_uses_filtered_repositories_and_exact_package_copies(tmp_path):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)
    workspace = tmp_path / "rc_vendor_ws"

    result = run_importer(workspace, fixtures, tools)

    assert result.returncode == 0, result.stderr
    assert result.stderr == ""
    filtered_repositories = workspace / "rc-vendor-filtered.repos"
    document = filtered_repositories.read_text()
    assert document == """repositories:
  upstream/alpha:
    type: git
    url: https://example.invalid/alpha.git
    version: 1111111111111111111111111111111111111111
  upstream/beta:
    type: git
    url: https://example.invalid/beta.git
    version: 2222222222222222222222222222222222222222
"""

    log = read_tool_log(tools)
    assert [entry["tool"] for entry in log] == ["vcs", "colcon"]
    assert log[0]["argv"][0] == "import"
    assert Path(log[0]["argv"][1]).is_relative_to(workspace)
    assert log[0]["stdin"] == document
    assert log[1]["argv"][:2] == ["list", "--base-paths"]
    assert len(log[1]["argv"]) == 4
    assert Path(log[1]["argv"][2]).is_relative_to(workspace)
    assert log[1]["argv"][3:] == ["--names-only"]

    copied_package_xml = {
        path.relative_to(workspace / "src").as_posix()
        for path in (workspace / "src").rglob("package.xml")
    }
    assert copied_package_xml == {
        f"{package_path}/package.xml" for package_path in SELECTED_PACKAGES.values()
    }
    for name, package_path in SELECTED_PACKAGES.items():
        assert (workspace / "src" / package_path / "payload.txt").read_text() == f"{name}\n"
    assert not (workspace / "src" / "upstream/alpha/alpha_unused").exists()
    assert not (workspace / "src" / "upstream/unused").exists()


def test_importer_requires_package_xml_before_copying(tmp_path):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)
    workspace = tmp_path / "rc_vendor_ws"
    (tools["source"] / SELECTED_PACKAGES["beta_pkg"] / "package.xml").unlink()

    result = run_importer(workspace, fixtures, tools)

    assert_failed_rc_only(result)
    assert "package.xml" in result.stderr
    assert not (workspace / "src").exists()
    assert [entry["tool"] for entry in read_tool_log(tools)] == ["vcs"]


def test_copy_failure_stops_before_colcon_or_publishing_sources(tmp_path):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)
    workspace = tmp_path / "rc_vendor_ws"
    unreadable = SELECTED_PACKAGES["alpha_pkg"] + "/payload.txt"

    result = run_importer(
        workspace,
        fixtures,
        tools,
        {"FAKE_VCS_UNREADABLE": unreadable},
    )

    assert_failed_rc_only(result)
    assert "copy" in result.stderr.lower()
    assert not (workspace / "src").exists()
    assert [entry["tool"] for entry in read_tool_log(tools)] == ["vcs"]


@pytest.mark.parametrize(
    ("discovered", "error_fragment"),
    [
        ("alpha_pkg\n", "missing"),
        ("alpha_pkg\nbeta_pkg\nstale_pkg\n", "stale"),
    ],
)
def test_importer_rejects_missing_and_stale_colcon_package_sets(
    tmp_path, discovered, error_fragment
):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)
    workspace = tmp_path / "rc_vendor_ws"

    result = run_importer(
        workspace,
        fixtures,
        tools,
        {"FAKE_COLCON_NAMES": discovered},
    )

    assert_failed_rc_only(result)
    assert error_fragment in result.stderr.lower()
    assert not (workspace / "src").exists()
    assert [entry["tool"] for entry in read_tool_log(tools)] == ["vcs", "colcon"]


@pytest.mark.parametrize(
    ("failure_environment", "expected_tools"),
    [
        ({"FAKE_VCS_EXIT": "19"}, ["vcs"]),
        ({"FAKE_COLCON_EXIT": "23"}, ["vcs", "colcon"]),
    ],
)
def test_tool_failures_stop_without_retry_or_fallback(
    tmp_path, failure_environment, expected_tools
):
    fixtures = write_metadata(tmp_path)
    tools = make_fake_tools(tmp_path)
    workspace = tmp_path / "rc_vendor_ws"

    result = run_importer(
        workspace,
        fixtures,
        tools,
        failure_environment,
    )

    assert_failed_rc_only(result)
    assert not (workspace / "src").exists()
    log = read_tool_log(tools)
    assert [entry["tool"] for entry in log] == expected_tools
    if log and log[0]["tool"] == "vcs":
        assert "upstream/unused" not in log[0]["stdin"]
