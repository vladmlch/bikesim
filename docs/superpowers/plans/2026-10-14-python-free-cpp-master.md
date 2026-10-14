# Complete Python-to-C++ Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make bike-sim fully usable, buildable, testable and distributable without Python while retaining every required source capability.

**Architecture:** Extract the already-native kernels/runtime behind typed owned interfaces, then add native setup, missing modes and application/tooling layers. Freeze a reference oracle before changes, qualify each source file and test assertion, and delete Python only after clean native release gates.

**Tech Stack:** C++23, CMake >=3.24, pinned native MuJoCo 3.12.0/DuckDB 1.5.6, existing numeric kernels, native IO/GUI/test tools.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`. Also read `migration/interface-contracts.md`, `migration/api-index.md`, and the assigned source cards.

## Global Constraints

- Preserve all required first-party behavior; no Python fallback, embedded interpreter, subprocess interpreter or mandatory Python build/test tool in the final product.
- Preserve existing C++ kernels and the single authoritative physical stepping path; boundary extraction is not permission to change equations or operation order.
- Use C++23 and CMake >=3.24. Retain strict warnings, stack protection, numerical flags, hardening and native safety checks.
- Initially retain MuJoCo 3.12.0 with the verified private TLS error ABI and DuckDB 1.5.6; lock actual native dependency hashes before use.
- Maintain named schema/config/CLI/recording/replay contracts and every supported mode, or obtain explicit user approval for a documented compatibility break.
- Keep existing same-provider bitwise guarantees; no global epsilon, golden regeneration to hide failures, or silent cross-provider equivalence claims.
- Recording remains opt-in, RAM-retained during execution and final-only publication; no streaming redesign, silent history loss or automatic recording from --out.
- Every Python deletion requires a ledger owner, native replacement or approved retirement, assertion/capability coverage and passing evidence; final removal is T63.
- Changes to shared headers/CMake/registries go through one integration owner. Run full native gates serially per machine and record the exact build identity.
- Source examples and API snippets in this plan are proposed implementation contracts, not a claim that code was compiled or tests passed during planning.

---

The named execution sub-skills refer to the downstream agent's environment. When unavailable, follow the same task-by-task red/green/independent-review protocol; no capability is assumed in this handoff. All statuses start planned. The code examples are contracts/regressions to implement and validate, not patches already applied to the application.

## Start here

Read the specification first, then `migration/interface-contracts.md`, then the assigned subsystem plan and source cards. Use the actual repository checkout as implementation context, not this plan as a substitute for source. The delivered unmixed source is a text snapshot; recover omitted lock/binary assets and reconcile Git identity in T00.

**Do not start by deleting src/bike_sim or by porting already-native kernels again.** The first deliverables are a complete source/removal ledger, a frozen reference baseline and a Python-disabled native build. The biggest hidden dependency is native startup constructed from a Python simulation; the next are binding-shaped C++ configs/state and Python-owned recording/publication/tooling.

This handoff has 64 work packages (T00-T63), split across 10 subsystem plans, with 459 per-file work units. Work packages are reviewable milestones, not single unbroken coding sessions. Their source cards are the smaller units: red regression, one focused port/reuse proof, green comparison, evidence and commit. All source/test/native-counterpart references distinguish inspected facts from planned replacements.

## Deliverables and execution order

| Stage | Tasks | Exit gate |
|---|---|---|
| Reference/native foundation | T00-T06 | Actual checkout reconciled; captured oracles; native dependency/build/IO foundations |
| Native core isolation | T07-T11 | Typed standalone ride/research from frozen bootstrap; no Python in core headers/link closure |
| Native inputs/models | T12-T18 | Complete defaults/configs/geometry/MJCF/topology projection |
| Fresh native startup | T19-T25 | Compatible RNG/terrain/equilibrium/cache; uncached native initialization |
| Missing physics/legacy | T26-T33 | Existing cores reused and remaining modes implemented rather than dropped |
| Research/policies | T34-T37 | Native environment, policy ABI, sensor/pending-window/validity semantics |
| Recording/replay | T38-T46 | Full schema/catalog, native ingestion/publication/reader/exports/replay/analyst CLI |
| User-facing application | T47-T51 | Nine native entrypoints, isolated GUI, plots and deterministic batch |
| Tests/tools/install | T52-T60 | Every assertion mapped; validation/check/coverage/native packages/docs |
| Qualification/removal | T61-T63 | Full capability/compatibility/no-Python evidence, then reviewed deletion |

Phase numbers group related work; they are not absolute serial barriers. For example, catalog definitions can begin from frozen schema/model inputs while research/legacy work continues. The machine-readable graph and producer APIs control dependencies. Never merge speculative support flags before the corresponding implementation and tests exist.

## Subsystem plans

| Plan | Tasks |
|---|---|
| `docs/superpowers/plans/2026-10-14-python-free-cpp-00-foundation.md` | T00, T01, T02, T03, T04, T05, T06 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-01-core-boundaries.md` | T07, T08, T09, T10, T11 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-02-configuration-models.md` | T12, T13, T14, T15, T16, T17, T18 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-03-terrain-initialization.md` | T19, T20, T21, T22, T23, T24, T25 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-04-remaining-physics.md` | T26, T27, T28, T29, T30, T31, T32, T33 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-05-research-policies.md` | T34, T35, T36, T37 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-06-recording-replay.md` | T38, T39, T40, T41, T42, T43, T44, T45, T46 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-07-cli-viewer-batch.md` | T47, T48, T49, T50, T51 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-08-tests-tools-packaging.md` | T52, T53, T54, T55, T56, T57, T58, T59, T60 |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-09-qualification-removal.md` | T61, T62, T63 |

## Milestones the coordinator can demonstrate

**M0: Reference authority.** The source manifest matches the real checkout, missing inputs are resolved, baseline commands/exit codes are retained and fixtures have real hashes/provider identity. A historically failing reference case remains documented rather than relabeled successful.

**M1: Standalone core.** A bindings-OFF native executable loads a valid frozen bootstrap and runs/reset/snapshots/close/research windows without Python. This does not yet prove native model creation or equilibrium.

**M2: Fresh native simulation.** Standard user configs and tracks generate a model, reach the required initial state and produce a native runtime/presentation with no Python-created cache/model prerequisite. Match t=0 and subsequent control/step outputs.

**M3: Full application behavior.** Required physical and legacy modes, nine CLIs, GUI, policies, plots, batch, recording/replay and analysis pass their feature matrix. A narrow native physical profile is insufficient.

**M4: Native development/distribution.** Native checks, all ported tests, validation/benchmarks, coverage/sanitizers, source builds and offline installed packages pass on supported macOS/Linux targets with actual binary provenance.

**M5: Removal complete.** Final Python deletion, clean-tree rebuild, no-Python source/package/dynamic audit and qualified compatibility evidence. Retain immutable reference history and data, but no active interpreter path.

## Agent division of labor

Use one coordinator/integrator and focused workers for core boundaries, configuration/model/geometry, terrain/randomness/initialization, physics/rider modes, research/policies, recording/replay, CLI/viewer/plots/batch, and tests/tooling/packages. A worker owns only its assigned task/card target files. The integrator owns shared CMake/registries/public API merges and schedules the expensive full gates.

The coordinator assigns a task only after its required producer interfaces and baseline fixtures are available. A worker may prepare a leaf source card early when its actual dependencies are ready, but cannot mark the work package complete before all declared dependencies and owned cards pass. Use isolated worktrees/branches per worker; record exact source/base revisions. Do not run multiple full native sweeps on the same machine concurrently.

Worker context packet: this master, the specification, interface-contracts.md, the assigned task section, its `migration/cards/Txx.md`, relevant source files, input fixture records, dependency commits and the test-port rows. Do not send every worker the entire repository history or ask it to infer shared types independently.

Review twice: specification/behavior review first, then native implementation/ownership/numerical review. The reviewer must inspect actual diffs and test evidence. A passing output produced by regenerated goldens is not sufficient. Failed required checks, absent fixtures or unavailable platform evidence remain explicit failures/blockers, not approved skips.

## Integration and rollback

Keep an immutable pre-migration reference tag and a separate reference worktree with pinned legacy dependencies. The active branch introduces standalone targets and optional transition adapters, with the C++ core shared. Never create a new parallel physics implementation merely to make the optional bindings work.

Commit one source card or independently testable seam at a time. Each commit records its task/test IDs and updates the ledger. If a change regresses parity, revert that focused change or repair it against the original oracle; do not regenerate expected data. Shared API changes are coordinated across consumers before merge.

During migration there are two explicit build identities: a bindings-OFF standalone tree and a bindings-ON transition tree. The old native loader requires an absolute `NATIVE_TEST_BUILD_PATH`; its provenance must identify the intended extension. Both old native/full profiles run native preflight. After T57 the native runner replaces the interpreter-based runner and preserves the same safety/check classes.

Final deletion is a distinct integration transaction in T63. Do not delete by suffix across arbitrary vendor/history directories. The 459-path ledger plus newly discovered first-party files defines the reviewed removal set. Before deletion, each row has a native replacement/retirement decision and passing evidence. After deletion, build from a clean tree so stale wheels/extensions cannot hide a dependency.

## Decision register: facts versus proposals

| Topic | Inspected fact | Planned decision |
|---|---|---|
| Existing native work | Runtime/kernels/sensors/retention/column pieces already exist | Reuse; extract Python conversions |
| Startup | Python reference simulation supplies bootstrap/initial frame | Native SetupOwner then immutable typed bootstrap |
| Numeric execution | Existing exact/provider-sensitive tests and helper semantics | Preserve bitwise guarantees; qualify new provider separately |
| Missing modes | Native setup rejects source-supported combinations | Implement them before final capability closure |
| Recording | RAM-retained, opt-in, final-only, manifest-last publication | Preserve lifecycle and schema; use native DuckDB API |
| Exports | Analyst CSV and JSONL | Preserve exact outputs; no unsolicited Parquet rewrite |
| Plugins | Python module/factory and built-in policy surfaces | Native built-ins plus explicit versioned C ABI; required user plugins ported |
| Replay | Different versions/backends/execution identities | Keep identity; certify historical profile or require explicit user-approved break |
| GUI | Python viewer/re-exec plus native snapshots | Native GLFW/MuJoCo viewer with independent replica |
| Packages | Linux wheel packaging, macOS-focused native source | Qualify Linux headless native and macOS; do not assume Windows support |
| New dependencies | Not locked in the source snapshot | T03 acquires verified exact archives/hashes/licenses; no invented pins |

## Non-negotiable deletion gates

The coordinator rejects completion when any required capability is silently dropped, any test assertion lacks a disposition, a runtime/config/schema field is zero-filled instead of ported, a historical replay identity is relabeled, recording becomes streaming without approval, RNG/solver tolerances are silently changed, or an installed/source build still needs Python.

All remaining uncertainty is converted into a concrete gate: recover the missing checkout artifacts; collect actual test nodes; capture real fixtures; implement exact solver/random contracts; qualify the supported platform/provider; port required custom policies; certify required replay profiles. Agents do not get permission to skip those gates because the current native subset is easier.

## Evidence schema and status changes

Task states: `planned -> ready -> in_progress -> review -> verified -> merged`. A blocked task keeps a concrete blocker record and does not advance its state. File removal state changes only after its row has actual replacement symbols/test IDs/commit/evidence; T63 records the deletion commit.

Every handoff includes: task/card IDs, base/implementation commits, files changed, public API changes, exact commands and exit codes, binary/fixture/provider identities, failing or passing comparisons, capability rows closed, remaining blockers and reviewer decision. Machine-readable manifests are updated alongside human notes.

The delivery bundle's inventory statuses intentionally remain `planned`: this is a plan and source audit, not an implementation report.
