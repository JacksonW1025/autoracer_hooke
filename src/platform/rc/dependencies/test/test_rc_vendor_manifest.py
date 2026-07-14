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

import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[5]
RC_DEPENDENCIES = Path(__file__).resolve().parents[1]
RC_MANIFEST = RC_DEPENDENCIES / "rc-vendor-packages.txt"
RESOLVER = RC_DEPENDENCIES / "resolve_rc_vendor.py"
PACKAGE_MANIFEST = ROOT / "dependencies" / "vendor-packages.tsv"
REPOSITORIES = ROOT / "dependencies" / "autoracer.repos"
LOCK_FILE = ROOT / "dependencies" / "versions.lock.yaml"

RC_HARDWARE_PACKAGES = {
    "hipnuc_imu",
    "hipnuc_lib_package",
    "lslidar_driver",
    "lslidar_msgs",
}
HOOKE_ONLY_PACKAGES = {
    "autoware_gnss_poser",
    "fixposition_driver_lib",
    "fixposition_driver_msgs",
    "fixposition_driver_ros2",
    "fpsdk_common",
    "fpsdk_ros2",
    "nebula_core_common",
    "nebula_core_decoders",
    "nebula_core_hw_interfaces",
    "nebula_core_ros",
    "nebula_hesai",
    "nebula_hesai_common",
    "nebula_hesai_decoders",
    "nebula_hesai_hw_interfaces",
    "nebula_msgs",
    "pandar_msgs",
    "rtcm_msgs",
    "sync_tooling_msgs",
    "tier4_api_msgs",
    "tier4_debug_msgs",
}
TEST_ONLY_PACKAGES = {"autoware_lint_common"}
RC_EXCLUDED_PACKAGES = HOOKE_ONLY_PACKAGES | TEST_ONLY_PACKAGES


def package_records(path=PACKAGE_MANIFEST):
    return [tuple(line.split("\t")) for line in path.read_text().splitlines()]


def manifest_names(path=RC_MANIFEST):
    return [line.strip() for line in path.read_text().splitlines()]


def parse_repositories(text):
    repositories = {}
    current = None
    for line in text.splitlines():
        if line == "repositories:":
            continue
        if line.startswith("  ") and not line.startswith("    ") and line.endswith(":"):
            current = line.strip()[:-1]
            repositories[current] = {}
            continue
        if line.startswith("    ") and current is not None:
            key, value = line.strip().split(": ", 1)
            repositories[current][key] = value
    return repositories


def parse_lock_repositories(text):
    repositories = {}
    current = None
    in_repositories = False
    for line in text.splitlines():
        if line == "repositories:":
            in_repositories = True
            continue
        if in_repositories and line and not line.startswith(" "):
            break
        if not in_repositories:
            continue
        if line.startswith("  - path: "):
            current = line.removeprefix("  - path: ")
            repositories[current] = {}
        elif line.startswith("    ") and current is not None:
            key, value = line.strip().split(": ", 1)
            repositories[current][key] = value
    return repositories


def canonical_package_names():
    return [name for name, _ in package_records()]


def expected_rc_names():
    return [
        name for name in canonical_package_names() if name not in RC_EXCLUDED_PACKAGES
    ]


def copy_inputs(tmp_path):
    fixtures = {
        "rc_manifest": tmp_path / "rc-vendor-packages.txt",
        "package_manifest": tmp_path / "vendor-packages.tsv",
        "repositories": tmp_path / "autoracer.repos",
        "lock_file": tmp_path / "versions.lock.yaml",
    }
    shutil.copyfile(RC_MANIFEST, fixtures["rc_manifest"])
    shutil.copyfile(PACKAGE_MANIFEST, fixtures["package_manifest"])
    shutil.copyfile(REPOSITORIES, fixtures["repositories"])
    shutil.copyfile(LOCK_FILE, fixtures["lock_file"])
    return fixtures


def run_resolver(output="packages", fixtures=None):
    command = [sys.executable, str(RESOLVER), "--format", output]
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
    return subprocess.run(command, cwd=ROOT, text=True, capture_output=True)


def assert_fails_before_output(fixtures, error_fragment):
    result = run_resolver(fixtures=fixtures)
    assert result.returncode != 0
    assert result.stdout == ""
    assert error_fragment in result.stderr.lower()


def test_repository_manifest_is_unchanged_two_column_inventory():
    lines = PACKAGE_MANIFEST.read_text().splitlines()
    records = package_records()

    assert len(lines) == 103
    assert all(lines)
    assert all(len(record) == 2 and all(record) for record in records)
    assert len({name for name, _ in records}) == 103
    assert len({path for _, path in records}) == 103


def test_rc_manifest_is_the_exact_name_only_closure():
    names = manifest_names()
    expected = expected_rc_names()

    assert len(names) == 82
    assert len(set(names)) == 82
    assert names == expected
    assert "tier4_localization_launch" in names
    assert RC_HARDWARE_PACKAGES <= set(names)
    assert {
        "autoware_map_height_fitter",
        "autoware_pure_pursuit",
        "autoware_vehicle_velocity_converter",
    } <= set(names)
    assert HOOKE_ONLY_PACKAGES.isdisjoint(names)
    assert TEST_ONLY_PACKAGES.isdisjoint(names)
    assert set(canonical_package_names()) - set(names) == RC_EXCLUDED_PACKAGES
    assert all(re.fullmatch(r"[a-z][a-z0-9_]*", name) for name in names)


def test_resolver_preserves_canonical_order_not_manifest_order(tmp_path):
    fixtures = copy_inputs(tmp_path)
    fixtures["rc_manifest"].write_text(
        "\n".join(reversed(expected_rc_names())) + "\n", encoding="utf-8"
    )

    first = run_resolver(fixtures=fixtures)
    second = run_resolver(fixtures=fixtures)

    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    assert first.stdout == second.stdout
    assert first.stdout.splitlines() == expected_rc_names()


def test_resolver_emits_records_paths_and_canonical_repository_pins():
    expected_records = [
        record for record in package_records() if record[0] in set(expected_rc_names())
    ]

    records_result = run_resolver("records")
    paths_result = run_resolver("paths")
    repositories_result = run_resolver("repositories")

    for result in (records_result, paths_result, repositories_result):
        assert result.returncode == 0, result.stderr
    resolved_records = [
        tuple(line.split("\t")) for line in records_result.stdout.splitlines()
    ]
    assert resolved_records == expected_records
    assert paths_result.stdout.splitlines() == [path for _, path in expected_records]

    canonical_repositories = parse_repositories(REPOSITORIES.read_text())
    locked_repositories = parse_lock_repositories(LOCK_FILE.read_text())
    resolved_repositories = parse_repositories(repositories_result.stdout)
    expected_owners = {
        owner
        for _, package_path in expected_records
        for owner in canonical_repositories
        if package_path == owner or package_path.startswith(owner + "/")
    }

    assert list(resolved_repositories) == [
        owner for owner in canonical_repositories if owner in expected_owners
    ]
    for owner, repository in resolved_repositories.items():
        assert repository == canonical_repositories[owner]
        assert repository["url"] == locked_repositories[owner]["url"]
        assert repository["version"] == locked_repositories[owner]["revision"]


@pytest.mark.parametrize("unsafe_path", ["/tmp/escape", "../escape", "autoware/../escape"])
def test_resolver_rejects_unsafe_package_paths_before_output(tmp_path, unsafe_path):
    fixtures = copy_inputs(tmp_path)
    text = fixtures["package_manifest"].read_text()
    text = text.replace(
        "autoware_cmake\tautoware/autoware_cmake/autoware_cmake",
        f"autoware_cmake\t{unsafe_path}",
    )
    fixtures["package_manifest"].write_text(text, encoding="utf-8")

    assert_fails_before_output(fixtures, "unsafe")


def test_resolver_rejects_duplicate_rc_names_before_output(tmp_path):
    fixtures = copy_inputs(tmp_path)
    with fixtures["rc_manifest"].open("a", encoding="utf-8") as manifest:
        manifest.write("autoware_cmake\n")

    assert_fails_before_output(fixtures, "duplicate")


def test_resolver_rejects_unknown_rc_names_before_output(tmp_path):
    fixtures = copy_inputs(tmp_path)
    with fixtures["rc_manifest"].open("a", encoding="utf-8") as manifest:
        manifest.write("unknown_rc_package\n")

    assert_fails_before_output(fixtures, "unknown")


def test_resolver_rejects_ambiguous_repository_owners_before_output(tmp_path):
    fixtures = copy_inputs(tmp_path)
    repositories = fixtures["repositories"].read_text()
    repositories += """
  autoware/core/localization:
    type: git
    url: https://example.invalid/ambiguous.git
    version: 1111111111111111111111111111111111111111
"""
    fixtures["repositories"].write_text(repositories, encoding="utf-8")
    lock = fixtures["lock_file"].read_text()
    lock = lock.replace(
        "patches:",
        """  - path: autoware/core/localization
    url: https://example.invalid/ambiguous.git
    revision: 1111111111111111111111111111111111111111
patches:""",
    )
    fixtures["lock_file"].write_text(lock, encoding="utf-8")

    assert_fails_before_output(fixtures, "ambiguous")


@pytest.mark.parametrize("source", ["repositories", "lock_file"])
def test_resolver_rejects_missing_revisions_before_output(tmp_path, source):
    fixtures = copy_inputs(tmp_path)
    path = fixtures[source]
    text = path.read_text()
    if source == "repositories":
        text = text.replace(
            "    version: 0e0794e034fe1b8fea6e3a0bbcb9d9b8cdba03ad\n", "", 1
        )
    else:
        text = text.replace(
            "    revision: 0e0794e034fe1b8fea6e3a0bbcb9d9b8cdba03ad\n", "", 1
        )
    path.write_text(text, encoding="utf-8")

    assert_fails_before_output(fixtures, "revision")


@pytest.mark.parametrize("field", ["url", "revision"])
def test_resolver_rejects_lock_disagreement_before_output(tmp_path, field):
    fixtures = copy_inputs(tmp_path)
    text = fixtures["lock_file"].read_text()
    if field == "url":
        text = text.replace(
            "https://github.com/autowarefoundation/autoware_cmake.git",
            "https://example.invalid/disagrees.git",
            1,
        )
    else:
        text = text.replace(
            "0e0794e034fe1b8fea6e3a0bbcb9d9b8cdba03ad",
            "2222222222222222222222222222222222222222",
            1,
        )
    fixtures["lock_file"].write_text(text, encoding="utf-8")

    assert_fails_before_output(fixtures, "disagree")


def test_resolver_rejects_an_empty_closure_before_output(tmp_path):
    fixtures = copy_inputs(tmp_path)
    fixtures["rc_manifest"].write_text("\n", encoding="utf-8")

    assert_fails_before_output(fixtures, "empty")
