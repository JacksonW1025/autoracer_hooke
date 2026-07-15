# Repository and Workspace Convergence Design

## Purpose

Make the RC product repository understandable from one checkout and prevent local development, Orin hardware validation, legacy Autoware sources, and generated assets from becoming competing sources of truth.

## Git contract

The user fork keeps exactly two branches:

- `pilot-localization-sync-20260707` is the immutable, CarMaker-validated baseline mirrored from `upstream/98064d3`.
- `rc-platform-integration` is the only active integration branch and remains a linear descendant of that baseline.

The tips of the abandoned `feature/official-autoware-launch` and `rc-car-migration` branches are retained as dated archive tags before their branches are deleted. The fork's obsolete `main` branch is deleted; upstream remains available through the `upstream` remote.

Each development machine has only the two corresponding local branches. Neither machine may create unpublished commits on `rc-platform-integration` while the other machine is editing it.

## Machine roles

The x86 workstation is the primary development surface because it owns the complete reference set, ROS bags, mapping workspace, generated map assets, and code-analysis tooling. Its main checkout is `/home/milesli/Desktop/RC/autoracer_hooke` on `rc-platform-integration`; the active branch must not live in a hidden worktree.

The Orin is the deployment and hardware-validation surface. Its workspace is `/home/wheeltec/Desktop/work`, with the product checkout at `/home/wheeltec/Desktop/work/autoracer_hooke` and map assets at `/home/wheeltec/Desktop/work/rc-map-assets`. Device-specific work uses a short-lived remote branch and returns to the integration branch after workstation review; the Orin is otherwise a clean consumer of the integration branch.

CAN on Hooke and serial on the RC car remain platform adapter details. Workspace convergence does not move chassis transport code into Core.

## Local workspace contract

The x86 workspace has explicit roles:

```text
/home/milesli/Desktop/RC/
├── autoracer_hooke/       # canonical product checkout
├── RCCar-Firmware/        # independent chassis firmware repository
├── rc_mapping_ws/         # mapping tool workspace
├── rc_mapping_data/       # bags, PCD maps, and generated validation assets
├── references/            # read-only legacy repositories
└── tooling/
    └── codebase-memory-mcp/
```

Generated `build`, `install`, and `log` trees are disposable and are removed during convergence. Large maps and bags stay outside the product repository; tracked manifests and course CSV files preserve their runtime contracts.

## Orin cleanup contract

Before removing the dirty legacy desktop checkout, export its binary Git diff and status into `/home/wheeltec/archive/autoracer-hooke-official-20260715`. Then remove:

- `/home/wheeltec/Desktop/quickstart`, because its old launch contract is retired;
- `/home/wheeltec/Desktop/autoracer_hooke`, after the dirty-state export;
- `/home/wheeltec/autoware`, because it is an obsolete broad underlay and must not replace the isolated RC vendor closure.

Move `/home/wheeltec/work` atomically to `/home/wheeltec/Desktop/work`. Do not claim the RC runtime is complete: the isolated vendor build remains blocked by the unavailable `tier4/autoware_launch.x1` repository, and the vehicle serial device is not currently present.

## Verification

Convergence is complete when:

- origin exposes exactly the pilot baseline and RC integration branches;
- both machines have exactly those two local branches;
- the local main checkout and Orin product checkout are clean at the same RC commit;
- no active Orin process or script references the deleted legacy desktop repository;
- archive tags resolve to the former abandoned branch tips;
- generated build/install/log directories covered by this cleanup are absent.
