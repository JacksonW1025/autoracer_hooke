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

"""Resolve the isolated RC vendor closure from canonical read-only metadata."""

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Dict, List, Mapping, Sequence


PACKAGE_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
PRODUCT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_RC_MANIFEST = Path(__file__).resolve().with_name("rc-vendor-packages.txt")
DEFAULT_PACKAGE_MANIFEST = PRODUCT_ROOT / "dependencies" / "vendor-packages.tsv"
DEFAULT_REPOSITORIES = PRODUCT_ROOT / "dependencies" / "autoracer.repos"
DEFAULT_LOCK_FILE = PRODUCT_ROOT / "dependencies" / "versions.lock.yaml"


class ResolutionError(ValueError):
    """Raised when RC dependency metadata cannot be resolved safely."""


@dataclass(frozen=True)
class Repository:
    path: str
    repository_type: str
    url: str
    revision: str


@dataclass(frozen=True)
class PackageRecord:
    name: str
    path: str
    repository: str


@dataclass(frozen=True)
class Resolution:
    records: Sequence[PackageRecord]
    repositories: Sequence[Repository]

    @property
    def packages(self) -> List[str]:
        return [record.name for record in self.records]

    @property
    def paths(self) -> List[str]:
        return [record.path for record in self.records]


def _read_text(path: Path, description: str) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as error:
        raise ResolutionError(f"cannot read {description} {path}: {error}") from error


def _is_safe_relative_path(value: str) -> bool:
    if not value or value != value.strip() or "\\" in value:
        return False
    path = PurePosixPath(value)
    parts = value.split("/")
    return not path.is_absolute() and all(part not in ("", ".", "..") for part in parts)


def _require_safe_path(value: str, description: str) -> None:
    if not _is_safe_relative_path(value):
        raise ResolutionError(f"unsafe {description}: {value!r}")


def _parse_rc_manifest(path: Path) -> List[str]:
    names = []
    seen = set()
    for line_number, raw_line in enumerate(_read_text(path, "RC manifest").splitlines(), 1):
        name = raw_line.strip()
        if not name or name.startswith("#"):
            continue
        if not PACKAGE_NAME.fullmatch(name):
            raise ResolutionError(
                f"invalid RC package name on line {line_number}; the manifest accepts names only"
            )
        if name in seen:
            raise ResolutionError(f"duplicate RC package name on line {line_number}: {name}")
        seen.add(name)
        names.append(name)
    if not names:
        raise ResolutionError("empty RC dependency closure")
    return names


def _parse_package_manifest(path: Path) -> List[tuple]:
    records = []
    names = set()
    paths = set()
    for line_number, line in enumerate(_read_text(path, "package manifest").splitlines(), 1):
        if not line:
            raise ResolutionError(f"blank package manifest record on line {line_number}")
        fields = line.split("\t")
        if len(fields) != 2 or not all(fields):
            raise ResolutionError(
                f"package manifest line {line_number} must contain exactly two nonblank fields"
            )
        name, package_path = fields
        if not PACKAGE_NAME.fullmatch(name):
            raise ResolutionError(f"invalid canonical package name on line {line_number}: {name}")
        _require_safe_path(package_path, f"package path on line {line_number}")
        if name in names:
            raise ResolutionError(f"duplicate canonical package name on line {line_number}: {name}")
        if package_path in paths:
            raise ResolutionError(
                f"duplicate canonical package path on line {line_number}: {package_path}"
            )
        names.add(name)
        paths.add(package_path)
        records.append((name, package_path))
    if not records:
        raise ResolutionError("empty canonical package manifest")
    return records


def _parse_repositories(path: Path) -> Dict[str, Mapping[str, str]]:
    repositories: Dict[str, Dict[str, str]] = {}
    current = None
    found_root = False
    for line_number, raw_line in enumerate(_read_text(path, "repository file").splitlines(), 1):
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if raw_line == "repositories:":
            if found_root:
                raise ResolutionError("duplicate repositories mapping")
            found_root = True
            continue
        if not found_root:
            raise ResolutionError(f"unexpected repository data on line {line_number}")
        if raw_line.startswith("  ") and not raw_line.startswith("    ") and stripped.endswith(":"):
            repository_path = stripped[:-1]
            _require_safe_path(repository_path, f"repository path on line {line_number}")
            if repository_path in repositories:
                raise ResolutionError(f"duplicate repository path: {repository_path}")
            repositories[repository_path] = {}
            current = repository_path
            continue
        if raw_line.startswith("    ") and current is not None and ": " in stripped:
            key, value = stripped.split(": ", 1)
            if key in repositories[current]:
                raise ResolutionError(f"duplicate {key} for repository {current}")
            if not value:
                raise ResolutionError(f"blank {key} for repository {current}")
            repositories[current][key] = value
            continue
        raise ResolutionError(f"malformed repository data on line {line_number}")

    if not repositories:
        raise ResolutionError("empty canonical repository mapping")
    for repository_path, metadata in repositories.items():
        if metadata.get("type") != "git":
            raise ResolutionError(f"repository {repository_path} must have type git")
        if not metadata.get("url"):
            raise ResolutionError(f"repository {repository_path} has no URL")
        if not metadata.get("version"):
            raise ResolutionError(f"repository {repository_path} has no pinned revision/version")
    return repositories


def _parse_lock_file(path: Path) -> Dict[str, Mapping[str, str]]:
    repositories: Dict[str, Dict[str, str]] = {}
    current = None
    in_repositories = False
    for line_number, raw_line in enumerate(_read_text(path, "lock file").splitlines(), 1):
        if raw_line == "repositories:":
            if in_repositories:
                raise ResolutionError("duplicate repositories list in lock file")
            in_repositories = True
            continue
        if not in_repositories:
            continue
        if raw_line and not raw_line.startswith(" "):
            break
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        if raw_line.startswith("  - path: "):
            repository_path = raw_line.removeprefix("  - path: ")
            _require_safe_path(repository_path, f"lock repository path on line {line_number}")
            if repository_path in repositories:
                raise ResolutionError(f"duplicate lock repository path: {repository_path}")
            repositories[repository_path] = {}
            current = repository_path
            continue
        if raw_line.startswith("    ") and current is not None and ": " in raw_line.strip():
            key, value = raw_line.strip().split(": ", 1)
            if key in repositories[current]:
                raise ResolutionError(f"duplicate lock {key} for repository {current}")
            if not value:
                raise ResolutionError(f"blank lock {key} for repository {current}")
            repositories[current][key] = value
            continue
        raise ResolutionError(f"malformed lock repository data on line {line_number}")

    if not repositories:
        raise ResolutionError("empty repository list in lock file")
    for repository_path, metadata in repositories.items():
        if not metadata.get("url"):
            raise ResolutionError(f"lock repository {repository_path} has no URL")
        if not metadata.get("revision"):
            raise ResolutionError(f"lock repository {repository_path} has no pinned revision")
    return repositories


def resolve(
    rc_manifest: Path = DEFAULT_RC_MANIFEST,
    package_manifest: Path = DEFAULT_PACKAGE_MANIFEST,
    repositories_file: Path = DEFAULT_REPOSITORIES,
    lock_file: Path = DEFAULT_LOCK_FILE,
) -> Resolution:
    """Resolve and fully validate the RC closure without emitting output."""

    selected_names = _parse_rc_manifest(rc_manifest)
    canonical_records = _parse_package_manifest(package_manifest)
    repositories = _parse_repositories(repositories_file)
    locked_repositories = _parse_lock_file(lock_file)

    canonical_names = {name for name, _ in canonical_records}
    unknown_names = [name for name in selected_names if name not in canonical_names]
    if unknown_names:
        raise ResolutionError(f"unknown RC package name: {unknown_names[0]}")

    selected = set(selected_names)
    resolved_records = []
    selected_owners = set()
    for name, package_path in canonical_records:
        if name not in selected:
            continue
        owners = [
            repository_path
            for repository_path in repositories
            if package_path == repository_path or package_path.startswith(repository_path + "/")
        ]
        if not owners:
            raise ResolutionError(f"package {name} has no repository owner")
        if len(owners) != 1:
            raise ResolutionError(
                f"package {name} has ambiguous repository owners: {', '.join(owners)}"
            )
        owner = owners[0]
        selected_owners.add(owner)
        resolved_records.append(PackageRecord(name=name, path=package_path, repository=owner))

    if not resolved_records:
        raise ResolutionError("empty resolved RC dependency closure")

    resolved_repositories = []
    for repository_path, metadata in repositories.items():
        if repository_path not in selected_owners:
            continue
        locked = locked_repositories.get(repository_path)
        if locked is None:
            raise ResolutionError(f"repository {repository_path} has no lock revision")
        if metadata["url"] != locked["url"]:
            raise ResolutionError(f"repository URL sources disagree for {repository_path}")
        if metadata["version"] != locked["revision"]:
            raise ResolutionError(f"repository revision sources disagree for {repository_path}")
        resolved_repositories.append(
            Repository(
                path=repository_path,
                repository_type=metadata["type"],
                url=metadata["url"],
                revision=metadata["version"],
            )
        )

    return Resolution(records=resolved_records, repositories=resolved_repositories)


def render(resolution: Resolution, output_format: str) -> str:
    if output_format == "packages":
        lines = resolution.packages
    elif output_format == "records":
        lines = [f"{record.name}\t{record.path}" for record in resolution.records]
    elif output_format == "paths":
        lines = resolution.paths
    elif output_format == "repositories":
        lines = ["repositories:"]
        for repository in resolution.repositories:
            lines.extend(
                [
                    f"  {repository.path}:",
                    f"    type: {repository.repository_type}",
                    f"    url: {repository.url}",
                    f"    version: {repository.revision}",
                ]
            )
    else:
        raise ResolutionError(f"unknown output format: {output_format}")
    return "\n".join(lines) + "\n"


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--format",
        choices=("packages", "records", "paths", "repositories"),
        default="packages",
        help="deterministic RC resolver view to write",
    )
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
        output = render(resolution, arguments.format)
    except ResolutionError as error:
        print(f"RC vendor resolution failed: {error}", file=sys.stderr)
        return 2
    sys.stdout.write(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
