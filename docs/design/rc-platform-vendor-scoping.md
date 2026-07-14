# ADR: Platform-scoped vendor dependencies

- Status: Accepted
- Date: 2026-07-15
- Applies to: `dependencies/`, dependency scripts, and the generated `vendor_ws`

## Context

`vendor_ws` is a reproducible ROS 2 underlay for pinned third-party source packages.
It keeps Autoware and hardware-driver source out of the product tree while allowing
the product overlay to build against an exact, reviewable dependency set.

The current `dependencies/vendor-packages.tsv` already inventories 103 packages.
The number 103 is the existing canonical inventory, not 103 packages to add. The
platform-scoping work classifies those rows and selects subsets; it does not create
a second inventory, change upstream revisions, or introduce dependencies.

The product now has one shared race core and two hardware platforms. Building RC
should not require Hooke2-only LiDAR, GNSS, or vehicle-interface packages, while
the established Hooke2 and CarMaker path must continue to work when no selector is
provided.

## Decision

Keep one canonical package manifest and add explicit scope metadata to each existing
row. Add one selector and make import, verification, rosdep, and vendor build consume
its output. Do not maintain `hooke2.repos`, `rc.repos`, copied package lists, or
script-local allowlists.

`dependencies/autoracer.repos` remains the pinned repository transport definition.
`dependencies/versions.lock.yaml` remains lock evidence. Neither is a competing
package inventory: package membership, relative path, scope, and owning repository
are authoritative only in `dependencies/vendor-packages.tsv`.

### Canonical manifest

Each non-comment TSV row has four fields:

```text
package_name    package_relative_path    scope    repository_key
```

The allowed categorical scopes are:

- `shared`: required by the platform-independent product graph.
- `hooke2`: required only by Hooke2 hardware or its CarMaker-compatible path.
- `rc`: required only by RC hardware.
- `test-only`: retained in the canonical inventory but unreachable from either
  production platform closure.

The repository key must exactly match a key under `repositories:` in
`dependencies/autoracer.repos`. A package has exactly one name, path, scope, and
repository owner. Comments and one schema header are allowed; blank or partially
populated data rows are not.

### Baseline counts

These are acceptance-test assertions, not manually trusted runtime facts:

| Set | Expected packages | Selection rule |
|---|---:|---|
| Shared | 78 | `shared` |
| Hooke2-only | 20 | `hooke2` |
| RC-only | 4 | `rc` |
| Test-only/unreachable | 1 | `test-only` |
| Hooke2 product | 98 | `shared + hooke2` |
| RC product | 82 | `shared + rc` |
| Full canonical union | 103 | all four categories |

The arithmetic is deliberate: 78 + 20 = 98, 78 + 4 = 82, and the full
78 + 20 + 4 + 1 inventory is 103. `all` is the full audit/compatibility selection
and therefore includes the test-only/unreachable row. Normal `hooke2` and `rc`
selections exclude it. Automated tests must validate every count and membership
rule before the baseline can be called verified.

### Selector contract

`scripts/select_vendor_dependencies.py` is the only scope parser. It uses the
Python standard library and produces deterministic, newline-delimited output for:

- selected package names for vendor build and verification;
- selected package/path records for import;
- selected relative paths for rosdep;
- a filtered `.repos` document for network import.

The selector resolves the platform from an explicit option first, then
`AUTORACER_PLATFORM`, and otherwise defaults to `hooke2`. Accepted values are
`hooke2`, `rc`, and `all`. Existing scripts pass through the same environment value
instead of interpreting scope independently.

Output preserves canonical manifest order for copying and building. Verification
may sort both sides solely to compare sets. Repeated output, duplicate manifest
records, implicit scope expansion, and platform aliases are prohibited.

### Workflow integration

`scripts/import_dependencies.sh` asks the selector for package/path records before
copying from a verified `PILOT_REPO` or a network checkout. `--refresh` is usable
only when `PILOT_REPO` points to a verified matching source checkout.
`--verify-only` compares the selected package names with `colcon list` for
`vendor_ws/src`; missing and extra packages both fail. Changing platforms in an
existing underlay therefore requires a new import rather than silently reusing a
stale superset.

For `--network`, the selector filters `dependencies/autoracer.repos` to repository
keys that own at least one selected package, and `vcs import` receives only that
document. A selected package still causes its complete upstream Git repository to
be checked out temporarily; only selected package directories are copied into
`vendor_ws/src`.

On the current Orin, `/home/wheeltec/work/pilot-auto.x1` is absent. The old Desktop
repository is dirty and reference-only; it must not be used as an import source.
Therefore `--refresh` must not be used here unless `PILOT_REPO` is explicitly set
to a separately verified checkout that matches the required source. Use
`--network` once the required credentials and repository access exist. Full import
and build verification is currently blocked because private
`autoware_launch.x1` access is unavailable; unavailable inputs must not be reported
as an expected or observed green result.

`scripts/install_rosdeps.sh` passes only selected vendor package paths, `src/core`,
and the selected platform source root to rosdep. `all` passes both platform roots.
It must not scan the unselected platform tree.

`scripts/build_vendor.sh` obtains `--packages-select` from the selector.
`scripts/build_product.sh` retains its existing product targets and the same
`AUTORACER_PLATFORM` values. `scripts/build.sh` carries one platform choice through
verify, vendor build, and product build.

### Shared core and platform boundary

Dependency scoping changes the underlay only. It does not move behavior between
`src/core` and `src/platform` and requires no core source, launch, topic, parameter,
map, course, or algorithm changes.

The following remain shared and identical for Hooke2 and RC:

- localization, planning, control, safety, command gating, and runtime management;
- the normalized sensing and vehicle-status topics consumed by core;
- the shared control-command topics emitted toward platform adapters;
- launch argument names and parameter-file injection contracts;
- effective core topic remappings, QoS expectations, frames, and parameter values.

`tier4_localization_launch` is part of the shared dependency closure and must be
selected for both `hooke2` and `rc`. It must not be removed, replaced, forked, or
reclassified as a Hooke2 detail. Existing boundary and launch-contract tests are
regression gates for this decision.

Platform code continues to own hardware drivers, raw protocols, normalization,
static sensor transforms, vehicle geometry, and platform parameter overlays.
There is no `if rc` or `if hooke2` branch in core.

### CarMaker compatibility

CarMaker remains compatible through the established Hooke2-facing contract. An
unset selector still produces the 98-package Hooke2 underlay, so existing CarMaker
entry points do not need a new environment variable. Platform scoping must not
rename or change CarMaker bridge topics, timestamps, frames, launch arguments,
parameters, vehicle-command behavior, or sourcing order.

`all` remains available for full-inventory rebuilds and cross-platform maintenance,
but it is not the default and must not be required for CarMaker. The dependency
selector does not make CarMaker a third hardware scope.

## Fail-closed rules

The selector or consuming script exits nonzero before destructive copying, rosdep,
or build work when any of the following is true:

- the platform value is empty after explicit resolution, unknown, or ambiguous;
- the manifest is missing, unreadable, empty, malformed, or has an unknown scope;
- package names or relative paths are duplicated;
- a package path is absolute, contains `..`, or escapes an expected workspace root;
- a repository key is missing, duplicated, unknown, or does not own the package path;
- a selected package has no `package.xml` in the import source;
- repository filtering produces invalid YAML or cannot map every selected package;
- the selected set is empty or differs from the imported `colcon list` set;
- a required patch cannot be applied cleanly to the selected underlay.

Errors name the platform, manifest row or package, and corrective action. Scripts
must not fall back from `rc` to `hooke2`, from a subset to `all`, or from a filtered
repository document to the full repository manifest.

## Verification levels and current status

Verification claims are separated so source evidence cannot be mistaken for live
hardware evidence:

| Level | Evidence | Current Orin status |
|---|---|---|
| Source/contract | Selector, manifest, boundary, launch, topic, and parameter tests | May run |
| Import/build | Exact package verification, rosdep scope, Hooke2/RC builds | **Blocked** pending private `autoware_launch.x1` credentials and repository access |
| Live sensing | Pointcloud presence, rate, frame, fields, and timestamp behavior | **Not-tested** |
| Full RC sensing | LiDAR and IMU together through normalized core inputs | **Not-tested** |
| Full RC vehicle/race | UART, safety response, localization, control, and low-speed run | **Not-tested** |

On the current Orin, other devices are enabled but the LiDAR has not been started.
Source and contract tests may therefore run. No result from those tests can promote
live pointcloud presence, rate, frame, field layout, timestamp behavior, or full RC
sensing validation: each remains **Not-tested** until the LiDAR is started and the
live procedures in `docs/rc_bench_validation.md` produce recorded evidence.

## Consequences

RC import, rosdep, and vendor build work becomes smaller while one pinned inventory
and one core behavior remain authoritative. Hooke2 stays backward-compatible by
default. Full-inventory maintenance remains possible with `all`.

The manifest gains classification metadata and the dependency scripts gain a strict
selector dependency. This is intentional: a visible failure is safer than silently
building the wrong platform closure. Hardware readiness remains an independent
verification concern and is not inferred from dependency or build success.
