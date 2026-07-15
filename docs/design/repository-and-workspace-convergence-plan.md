# Repository and Workspace Convergence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Converge origin, the x86 development workspace, and the Orin deployment workspace on one pilot baseline and one RC integration branch without losing the dirty legacy Orin changes.

**Architecture:** GitHub is the shared source of truth, the x86 checkout is the primary development surface, and the Orin checkout is a clean deployment/hardware-validation consumer. Legacy branch tips and the Orin dirty diff are archived before destructive cleanup; generated products remain disposable.

**Tech Stack:** Git, Bash, SSH, ROS 2 colcon workspace conventions.

---

### Task 1: Record and publish the management contract

**Files:**
- Create: `docs/design/repository-and-workspace-convergence.md`
- Create: `docs/design/repository-and-workspace-convergence-plan.md`

- [ ] **Step 1:** Run `git diff --check` and confirm no whitespace errors.
- [ ] **Step 2:** Commit both documents with a Lore-format commit whose intent is preserving a single development and deployment source of truth.
- [ ] **Step 3:** Push `rc-platform-integration` and verify its remote tip equals the local tip.

### Task 2: Reduce the fork to two branches

**Refs:**
- Create branch: `origin/pilot-localization-sync-20260707` at `98064d37638d1d515b5db2ffd68ba078c35df7b2`
- Create tag: `archive/official-autoware-launch-20260715` at `24f4ed561bf0f8bdb97fd3236c6ec84fac4e65cc`
- Create tag: `archive/rc-car-migration-20260715` at `641c40afc84df0c0b315946a8bfcfd9eacd761c2`
- Delete branches: `origin/main`, `origin/feature/official-autoware-launch`, `origin/rc-car-migration`

- [ ] **Step 1:** Verify each expected source SHA with `git cat-file -e <sha>^{commit}`.
- [ ] **Step 2:** Push the pilot SHA as the named baseline branch.
- [ ] **Step 3:** Create annotated archive tags that record the former branch name and tip, then push the tags.
- [ ] **Step 4:** Delete the three obsolete origin branches.
- [ ] **Step 5:** Run `git ls-remote --heads origin` and confirm exactly two heads remain.

### Task 3: Make the x86 main checkout canonical

**Paths:**
- Keep: `/home/milesli/Desktop/RC/autoracer_hooke`
- Remove worktree: `/home/milesli/Desktop/RC/autoracer_hooke/.worktrees/rc-platform-integration`
- Rename: `/home/milesli/Desktop/RC/legacy-refference` to `/home/milesli/Desktop/RC/references`
- Move: `/home/milesli/Desktop/RC/codebase-memory-mcp` to `/home/milesli/Desktop/RC/tooling/codebase-memory-mcp`
- Delete: `/home/milesli/Desktop/RC/rc_quickstart_wrapper.sh`
- Delete: empty `/home/milesli/Desktop/RC/.git`

- [ ] **Step 1:** Confirm both product worktrees are clean except the main checkout's untracked `.codex/config.toml`.
- [ ] **Step 2:** Remove generated `build`, `install`, and `log` directories from the product worktree before removing it.
- [ ] **Step 3:** Remove the RC worktree, switch the main checkout to `rc-platform-integration`, and verify `.codex/config.toml` remains present.
- [ ] **Step 4:** Remove generated product, mapping, visualization, reference, and third-party build/install/log directories.
- [ ] **Step 5:** Rename and move the reference/tooling directories and remove the temporary wrapper and empty false Git root.
- [ ] **Step 6:** Fetch/prune origin, create the local pilot branch tracking origin, and confirm exactly two local branches remain.

### Task 4: Preserve and remove the old Orin environment

**Paths:**
- Create: `/home/wheeltec/archive/autoracer-hooke-official-20260715/status.txt`
- Create: `/home/wheeltec/archive/autoracer-hooke-official-20260715/working-tree.patch`
- Delete: `/home/wheeltec/Desktop/quickstart`
- Delete: `/home/wheeltec/Desktop/autoracer_hooke`
- Delete: `/home/wheeltec/autoware`

- [ ] **Step 1:** Verify no process command line references the legacy checkout.
- [ ] **Step 2:** Export `git status --porcelain=v1`, `git diff --binary`, the legacy HEAD, and remote URLs into the archive directory.
- [ ] **Step 3:** Verify the patch is nonempty and lists the 12 known modified tracked files.
- [ ] **Step 4:** Remove the legacy Quick Start, legacy desktop checkout, and obsolete broad Autoware underlay.
- [ ] **Step 5:** Verify all three paths are absent and the archive remains readable.

### Task 5: Move and normalize the Orin workspace

**Paths:**
- Move: `/home/wheeltec/work` to `/home/wheeltec/Desktop/work`
- Product: `/home/wheeltec/Desktop/work/autoracer_hooke`
- Maps: `/home/wheeltec/Desktop/work/rc-map-assets`

- [ ] **Step 1:** Confirm `/home/wheeltec/Desktop/work` does not exist and the active product worktree is clean.
- [ ] **Step 2:** Move the complete work directory to the desktop without copying or rewriting Git history.
- [ ] **Step 3:** Remove the failed `/home/wheeltec/Desktop/work/rc-vendor-ws` import shell and temporary quick-start wrapper.
- [ ] **Step 4:** Fetch/prune origin, create the pilot tracking branch, and keep `rc-platform-integration` checked out.
- [ ] **Step 5:** Remove generated build/install/log directories under the migrated work directory.

### Task 6: Verify convergence

**Evidence:** Git refs, filesystem paths, status output, archive hashes, and process inventory.

- [ ] **Step 1:** Confirm origin has exactly two branch refs and both archive tags resolve to their recorded tips.
- [ ] **Step 2:** Confirm each machine has exactly two local branches.
- [ ] **Step 3:** Confirm both RC checkouts and origin resolve to the same `rc-platform-integration` commit and both worktrees are clean.
- [ ] **Step 4:** Confirm Orin map manifests remain present under the migrated path and their four SHA-256 hashes match the x86 copies.
- [ ] **Step 5:** Search active Orin process command lines and remaining executable scripts for the deleted legacy product path; expect no matches.
- [ ] **Step 6:** Report the unavailable private launcher repository and absent vehicle serial device as runtime blockers, not cleanup failures.
