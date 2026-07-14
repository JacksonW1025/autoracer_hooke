#!/usr/bin/env python3
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

"""Import the resolved vendor closure into an isolated RC workspace."""

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Sequence

from resolve_rc_vendor import (
    DEFAULT_LOCK_FILE,
    DEFAULT_PACKAGE_MANIFEST,
    DEFAULT_RC_MANIFEST,
    DEFAULT_REPOSITORIES,
    PACKAGE_NAME,
    PRODUCT_ROOT,
    Resolution,
    ResolutionError,
    render,
    resolve,
)


RC_WORKSPACE_TOKEN = re.compile(r"(?:^|[-_.])rc(?:[-_.]|$)")
FILTERED_REPOSITORIES_NAME = "rc-vendor-filtered.repos"


class RCImportError(RuntimeError):
    """Raised when the RC vendor closure cannot be imported safely."""


class RCArgumentParser(argparse.ArgumentParser):
    """Keep command-line failures within the RC-specific error boundary."""

    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(2, f"RC vendor import failed: {message}\n")


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _is_safe_relative_path(value: str) -> bool:
    if not value or value != value.strip() or "\\" in value:
        return False
    path = PurePosixPath(value)
    return not path.is_absolute() and all(part not in ("", ".", "..") for part in path.parts)


def _validate_resolution(resolution: Resolution) -> None:
    """Validate every import-relevant resolver field before workspace mutation."""

    if not resolution.records or not resolution.repositories:
        raise RCImportError("resolved RC closure is empty")

    package_names = [record.name for record in resolution.records]
    package_paths = [record.path for record in resolution.records]
    repository_paths = [repository.path for repository in resolution.repositories]
    if len(package_names) != len(set(package_names)):
        raise RCImportError("resolved RC package names are not unique")
    if len(package_paths) != len(set(package_paths)):
        raise RCImportError("resolved RC package paths are not unique")
    if len(repository_paths) != len(set(repository_paths)):
        raise RCImportError("resolved RC repository paths are not unique")

    repositories = set(repository_paths)
    owners = set()
    posix_package_paths = []
    for record in resolution.records:
        if not PACKAGE_NAME.fullmatch(record.name):
            raise RCImportError(f"invalid resolved RC package name: {record.name!r}")
        if not _is_safe_relative_path(record.path):
            raise RCImportError(f"unsafe resolved RC package path: {record.path!r}")
        if not _is_safe_relative_path(record.repository):
            raise RCImportError(f"unsafe resolved RC repository owner: {record.repository!r}")
        if record.repository not in repositories:
            raise RCImportError(f"RC package {record.name} has an unresolved repository owner")
        if not (
            record.path == record.repository
            or record.path.startswith(record.repository + "/")
        ):
            raise RCImportError(f"RC package {record.name} escapes its repository owner")
        owners.add(record.repository)
        posix_package_paths.append(PurePosixPath(record.path))

    if owners != repositories:
        raise RCImportError("filtered RC repositories do not exactly match package owners")
    for index, package_path in enumerate(posix_package_paths):
        for other_path in posix_package_paths[index + 1 :]:
            if package_path in other_path.parents or other_path in package_path.parents:
                raise RCImportError("overlapping resolved RC package paths are not allowed")


def _validate_workspace(workspace: Path) -> Path:
    if not workspace.is_absolute():
        raise RCImportError("RC workspace must be an explicit absolute path")

    try:
        resolved_workspace = workspace.resolve(strict=False)
    except (OSError, RuntimeError) as error:
        raise RCImportError(f"cannot resolve RC workspace path: {error}") from error
    product_root = PRODUCT_ROOT.resolve()
    if resolved_workspace == product_root or _is_relative_to(resolved_workspace, product_root):
        raise RCImportError("RC workspace must be outside the product source tree")

    if not RC_WORKSPACE_TOKEN.search(workspace.name.lower()) or not RC_WORKSPACE_TOKEN.search(
        resolved_workspace.name.lower()
    ):
        raise RCImportError("workspace name must contain an explicit RC token")

    path_parts = [part.lower() for part in (*workspace.parts, *resolved_workspace.parts)]
    if any("hooke" in part or "autoware" in part or part == "vendor_ws" for part in path_parts):
        raise RCImportError("RC import refuses Hooke/Autoware legacy workspace paths")

    try:
        if resolved_workspace.exists() and not resolved_workspace.is_dir():
            raise RCImportError("RC workspace path is not a directory")
        if resolved_workspace.exists() and any(resolved_workspace.iterdir()):
            raise RCImportError("RC import requires a fresh empty workspace")
    except OSError as error:
        raise RCImportError(f"cannot inspect RC workspace path: {error}") from error
    return resolved_workspace


def _find_tool(name: str) -> str:
    executable = shutil.which(name)
    if executable is None:
        raise RCImportError(f"required RC import tool is unavailable: {name}")
    try:
        executable_path = Path(executable).resolve()
    except (OSError, RuntimeError) as error:
        raise RCImportError(f"cannot resolve RC import tool {name}: {error}") from error
    scripts_root = (PRODUCT_ROOT / "scripts").resolve()
    if executable_path == scripts_root or _is_relative_to(executable_path, scripts_root):
        raise RCImportError(f"RC import refuses repository-level script as {name}")
    return str(executable_path)


def _run_vcs(vcs: str, checkout: Path, repository_document: str) -> None:
    try:
        result = subprocess.run(
            [vcs, "import", str(checkout)],
            input=repository_document,
            text=True,
            capture_output=True,
            check=False,
        )
    except OSError as error:
        raise RCImportError(f"could not execute vcs for RC import: {error}") from error
    if result.returncode != 0:
        raise RCImportError(f"vcs import stopped with exit status {result.returncode}")


def _safe_package_source(checkout: Path, package_path: str, package_name: str) -> Path:
    source = checkout.joinpath(*PurePosixPath(package_path).parts)
    if source.is_symlink() or not source.is_dir():
        raise RCImportError(f"selected RC package directory is missing: {package_name}")

    checkout_root = checkout.resolve()
    source_root = source.resolve()
    if not _is_relative_to(source_root, checkout_root):
        raise RCImportError(f"selected RC package path escapes checkout: {package_name}")

    package_xml = source / "package.xml"
    if package_xml.is_symlink() or not package_xml.is_file():
        raise RCImportError(f"selected RC package has no package.xml: {package_name}")
    if not _is_relative_to(package_xml.resolve(), source_root):
        raise RCImportError(f"selected RC package.xml escapes its package: {package_name}")

    for entry in source.rglob("*"):
        if entry.is_symlink():
            raise RCImportError(f"selected RC package contains an unsafe symlink: {package_name}")

    try:
        declared_name = ET.parse(package_xml).getroot().findtext("name")
    except (ET.ParseError, OSError) as error:
        raise RCImportError(f"selected RC package.xml is invalid: {package_name}") from error
    if declared_name != package_name:
        raise RCImportError(
            f"selected RC package.xml name mismatch: expected {package_name}, got {declared_name!r}"
        )
    return source


def _copy_exact_packages(resolution: Resolution, checkout: Path, destination: Path) -> None:
    sources = [
        (record, _safe_package_source(checkout, record.path, record.name))
        for record in resolution.records
    ]
    destination.mkdir()
    for record, source in sources:
        target = destination.joinpath(*PurePosixPath(record.path).parts)
        if not _is_relative_to(target.resolve(strict=False), destination.resolve()):
            raise RCImportError(f"selected RC package destination is unsafe: {record.name}")
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(source, target)
        except OSError as error:
            raise RCImportError(f"could not copy selected RC package {record.name}: {error}") from error


def _verify_exact_package_set(colcon: str, source_root: Path, expected: Sequence[str]) -> None:
    try:
        result = subprocess.run(
            [colcon, "list", "--base-paths", str(source_root), "--names-only"],
            text=True,
            capture_output=True,
            check=False,
        )
    except OSError as error:
        raise RCImportError(f"could not execute colcon for RC verification: {error}") from error
    if result.returncode != 0:
        raise RCImportError(f"colcon verification stopped with exit status {result.returncode}")

    discovered = result.stdout.splitlines()
    if any(not name or name != name.strip() or not PACKAGE_NAME.fullmatch(name) for name in discovered):
        raise RCImportError("colcon returned an invalid RC package name")
    if len(discovered) != len(set(discovered)):
        raise RCImportError("colcon returned duplicate RC packages")

    expected_set = set(expected)
    discovered_set = set(discovered)
    missing = sorted(expected_set - discovered_set)
    stale = sorted(discovered_set - expected_set)
    if missing or stale:
        details = []
        if missing:
            details.append("missing packages: " + ", ".join(missing))
        if stale:
            details.append("stale packages: " + ", ".join(stale))
        raise RCImportError("exact RC package-set verification failed; " + "; ".join(details))


def import_vendor(workspace: Path, resolution: Resolution) -> Path:
    """Import a fully resolved closure and publish it only after exact verification."""

    _validate_resolution(resolution)
    repository_document = render(resolution, "repositories")
    rc_workspace = _validate_workspace(workspace)
    vcs = _find_tool("vcs")
    colcon = _find_tool("colcon")

    try:
        rc_workspace.mkdir(parents=True, exist_ok=True)
        filtered_repositories = rc_workspace / FILTERED_REPOSITORIES_NAME
        filtered_repositories.write_text(repository_document, encoding="utf-8")
        with tempfile.TemporaryDirectory(prefix=".rc-import-", dir=rc_workspace) as temporary:
            staging_root = Path(temporary)
            checkout = staging_root / "checkout"
            checkout.mkdir()
            _run_vcs(vcs, checkout, repository_document)
            selected_source = staging_root / "src"
            _copy_exact_packages(resolution, checkout, selected_source)
            _verify_exact_package_set(colcon, selected_source, resolution.packages)
            selected_source.rename(rc_workspace / "src")
    except RCImportError:
        raise
    except OSError as error:
        raise RCImportError(f"RC workspace mutation failed: {error}") from error
    return rc_workspace / "src"


def _argument_parser() -> argparse.ArgumentParser:
    parser = RCArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--rc-manifest", type=Path, default=DEFAULT_RC_MANIFEST)
    parser.add_argument("--package-manifest", type=Path, default=DEFAULT_PACKAGE_MANIFEST)
    parser.add_argument("--repositories", type=Path, default=DEFAULT_REPOSITORIES)
    parser.add_argument("--lock-file", type=Path, default=DEFAULT_LOCK_FILE)
    return parser


def main(argv=None) -> int:
    arguments = _argument_parser().parse_args(argv)
    try:
        resolution = resolve(
            rc_manifest=arguments.rc_manifest,
            package_manifest=arguments.package_manifest,
            repositories_file=arguments.repositories,
            lock_file=arguments.lock_file,
        )
        imported_source = import_vendor(arguments.workspace, resolution)
    except (ResolutionError, RCImportError) as error:
        print(f"RC vendor import failed: {error}", file=sys.stderr)
        return 2
    print(f"RC vendor import complete: {imported_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
