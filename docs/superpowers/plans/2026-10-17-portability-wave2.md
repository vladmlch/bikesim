# Portability Wave 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. These skills are not installed in this session; the available collaboration agents implement the independent tasks with a separate static review.

**Goal:** Implement portability T19 (macOS `mjpython` and Linux interactive viewers) and T24 (offline bundle manifests) against the supplied source snapshot.

**Architecture:** Keep viewer platform preparation in one lightweight module used by the existing Python frontends, with a compatibility delegate for their current launcher helper. Build offline bundles from explicit project sources and supplied local wheels, describe their target and integrity with a versioned manifest, and preserve the existing source-versus-wheel inspection interface.

**Tech Stack:** Python standard library, existing MuJoCo/GLFW viewer interfaces, JSON Schema, shell wrapper using `uv`, pytest regression sources.

**Spec:** User-supplied portability reviews in the 2026-10-17 request; the reconstructed scope and limits below are the execution contract. The original portability specification, briefs and `task-index.json` are absent from this snapshot.

## Global Constraints

- User instruction: **do not compile or run anything**. Do not invoke the application, tests, project tools, build systems, syntax compilers, imports of project modules, or dependency installation. File extraction, reading, editing, diffing and archiving are the only local operations used in this session.
- Preserve macOS support while implementing Linux support.
- Requested task IDs refer to **portability**, not the unrelated C++ migration `migration/cards/T19.md` (random streams) and `migration/cards/T24.md` (equilibrium/state). Those cards and their task graph must not be changed.
- T19 depends on portability T18; T24 depends on portability T23. The supplied snapshot does not establish that those tasks are complete. Implement the local seams needed by wave 2, and record the missing original dependency evidence.
- T21 recording lifecycle, T22 checked replay qualification, T25 installation qualification, T26–T30 convergence/release gates are outside this wave.
- Do not change physics, controller behavior, native numerical operation order, selected native artifacts, or worker-selection policy.
- Preserve exact CLI arguments, current working directory and existing environment variables through macOS re-execution.
- Linux interactive viewers must not search for or execute `mjpython`.
- GUI errors must preserve their original exception as context, and cleanup errors must not replace an active exception or interruption.
- Full and slim bundle metadata must distinguish supplied files from external prerequisites. Source code is authoritative; stale project-wheel content must be rejected.
- Bundle paths must be relative, contained and unique; sizes/hashes must come from supplied bytes. Do not package virtual environments, native build outputs or machine-local absolute paths.
- No platform, GUI, package closure, performance or release qualification may be reported as passed from static inspection.
- No git history accompanies the Repomix export. Retain the unmodified extraction as the evidence baseline. Patch preparation reconstructs trailing line endings from historical hashes where possible and explicitly records inferred cases; do not invent a commit ID or import history.

## Snapshot preflight

- [x] Extract the attachment using `repomix-unmixer` and retain an unmodified baseline.
- [x] Read `AGENTS.md` and `docs/agents/native-engineering.md`.
- [x] Confirm the missing portability briefs and distinguish the separate C++ migration task IDs.
- [x] Confirm `tools/install_offline.py` and `tools/check_offline_sources.py` exist.
- [x] Confirm `tools/make_offline_bundle.sh`, `tools/install_offline.sh` and `packaging/` are absent. New producer files are new implementations, not a recovered copy of the missing untracked originals.
- [x] Assign disjoint ownership: viewer/runtime agent for T19; delivery agent for T24; controller owns this plan, handoff and integration review.

## Task T19: macOS launcher and Linux viewers

**Files:**

- Create: `src/bike_sim/viewer_runtime.py`, `tests/test_viewer_runtime.py`, `tests/reference/test_viewer_cli_portability.py`.
- Modify: `src/bike_sim/sim/playground.py`; `src/bike_sim/sim/ride/{viewer,playback_viewer,physical_frontend}.py`; `src/bike_sim/sim/research/{viewer,replay_viewer}.py`; graphical branches of `src/bike_sim/cli/{main,ride,replay,playground}.py`.
- Adjust only affected fake-viewer test seams in `tests/reference/{test_frontend_windows,test_runtime_single_path,test_ride_backend_cli}.py` if required.

**Interfaces:**

- Consumes existing model/data objects and passive viewer keyword arguments unchanged.
- Retains `bike_sim.sim.playground.ensure_macos_mjpython() -> None` as a compatibility delegate.
- Produces reusable viewer platform preparation and passive-launch handling, without changing the simulation loop API.

- [x] **Step 1: Author launcher and viewer failure regression sources.** Cover Linux no-op without launcher lookup, macOS already launched, absent launcher, failed re-exec, preservation of `-m` and interpreter flags, unchanged native-build selection, absent display, failed GLFW initialization, successful fake viewer, normal close and close during an active error/interruption. Keep tests isolated from real displays and re-execution. The regression sources were authored alongside/after implementation; no red/green execution or test-first sequence is claimed.

  Existing regression sources that must remain compatible:

  ```text
  tests/reference/test_frontend_windows.py
  tests/reference/test_runtime_single_path.py
  tests/reference/test_ride_backend_cli.py
  tests/test_replay_playback.py
  ```

- [x] **Step 2: Implement lightweight platform preparation.** Check the OS before resolving a launcher; use an already initialized MuJoCo UI dispatcher as evidence of `mjpython`; preserve the original Python invocation using `sys.orig_argv` when available; prefer the active interpreter's scripts directory; give actionable setup errors rather than silently continuing. Record a failed restart without contaminating the success state.
- [x] **Step 3: Route every graphical entry through preparation before setup/GLFW launch.** Preserve the old helper import seam where it is used by callers and tests. Add Linux display/backend diagnostics without advertising unverified Wayland support. Surface synchronous GLFW initialization failures through the caller. Later asynchronous window/context/render failures still require desktop acceptance.
- [x] **Step 4: Preserve viewer lifecycle semantics.** Route all seven passive-window sites through an owning context. Propagate startup/loop errors and `KeyboardInterrupt`; annotate a secondary close failure without masking the original failure. Playback quit callbacks request a stop instead of closing the handle themselves. Preview-CSV publication and recording finalization remain T21 work.
- [x] **Step 5: Statically review all launch sites and tests.** Trace graphical, headless, replay and compatibility paths. Do not invoke any of them in this session.
- [ ] **Step 6: Verification gate — deliberately not run.** On a later authorized host, run the focused regression files through the repository test entry point, then graphical ride/research/playback/replay checks on both macOS and Linux desktops. Keep this checkbox open until actual evidence exists.

## Task T24: offline bundle manifest and source integrity

**Files:**

- Create: `tools/offline_bundle.py`, `tools/make_offline_bundle.sh`, `packaging/offline-bundle.schema.json`, `packaging/offline-targets.json`, `packaging/README.md`, `tests/test_offline_bundle.py`.
- Modify: `tools/check_offline_sources.py`, retaining its legacy `--source` and `--wheel` interface.
- Modify: `tools/install_offline.py` only to validate a present manifest and reject targets/profiles outside its existing Linux x86_64 CPython 3.13 slim contract before mutation. Preserve its no-manifest legacy behavior. Multi-platform/full/slim installation remains T25.

**Interfaces:**

- Consumes explicit source root, local wheelhouse, target OS/architecture/Python and full/slim profile; never downloads, resolves, builds, imports project code or executes the installer.
- Produces the existing `project/`, `wheelhouse/`, `requirements.txt` layout plus a versioned `bundle-manifest.json`, retaining file paths/hashes and source provenance.
- Preserves `compare_sources(source_root: Path, wheel: Path) -> dict[str, list[str]]`, including `missing_source`, `missing_wheel` and `different` report keys.
- Produces a manifest validator consumable by the next delivery task and `check_offline_sources.py`.

- [x] **Step 1: Write fixture-based manifest regression sources.** Cover full/slim distinction; Linux/macOS target metadata; deterministic inventory; file digest changes; stale or missing project-wheel files; incompatible wheel tags; malformed/duplicate paths; absolute and traversal paths; symlinks; undeclared files; binary build-output/venv exclusions; preserved source dot-configs; invalid sizes/hashes; missing external requirements; legacy comparator behavior. Also cover project payload confinement, case-preserving entry points, Python bounds and pristine-bundle bytecode prevention. Test data is generated inside temporary directories by the tests only when later authorized to run.
- [x] **Step 2: Define the manifest contract.** Include a format/version, explicit target, bundle profile, project identity and source provenance, local wheel metadata and digests, external requirements, integrity inventory and unqualified acceptance status. Derive source versions and Python bounds from `pyproject.toml`; do not invent platform qualification or a source commit. Describe Ubuntu 24.04 x86_64 CPython 3.13 and macOS arm64/x86_64 CPython 3.13/3.14 targets separately from the producer.
- [x] **Step 3: Implement generation and validation from file bytes.** Use a deterministic relative-path inventory and SHA-256. Verify metadata against current files, source-versus-wheel consistency and known filename-tag mismatches. Reject unknown/inconsistent fields, path escapes, unsafe archive entries and nonportable bundled state before writing a final archive. Do not equate tag checks with ABI/deployment qualification.
- [x] **Step 4: Add the shell entry point and profile examples.** Delegate to the producer through `uv` in offline/no-project mode. Document supplied-wheel prerequisites and distinguish a manifest example from a qualified release bundle.
- [x] **Step 5: Extend source checking additively.** Retain the existing comparison command and add bundle-manifest validation with structured, actionable errors and nonzero failure status.
- [x] **Step 6: Statically inspect producer/consumer compatibility.** Confirm that emitted layout and hashed requirements match the existing consumer's data format. The existing installer validates a present manifest before mutation and remains Linux x86_64 CPython 3.13 slim-only; do not label cross-platform installation complete. Keep dependency closure and semantic Requires-Dist equivalence explicitly unverified. Direct script imports do not write sibling bytecode into the checked inventory; module/import forms need the documented startup bytecode suppression.
- [ ] **Step 7: Verification gate — deliberately not run.** On a later authorized host, execute the focused fixture tests, generate full/slim bundles from real local wheels, inspect manifests and exercise T25 installation offline on each target. Keep this checkbox open until actual evidence exists.

## Review and delivery

- [x] Have a separate agent statically review the final changes against this contract and the existing call sites/consumer.
- [x] Resolve material findings and record remaining execution/desktop requirements.
- [x] Write a source-supported handoff and machine-readable status with implementation and execution qualification represented separately.
- [x] Create a patch against the attachment's reconstructed source, an archive of all changed files, and the updated extracted source archive. Document Repomix line-ending recovery and the lack of a commit-backed baseline.
- [x] Inspect patch structure and archive inventory without running project code. Apply the patch only as a file transformation in an isolated baseline copy and compare resulting bytes with the delivery files.

The final status is **implemented, not execution-verified** when the source work and review are complete. The original portability tasks remain unqualified until their required dependency and runtime gates are supplied.
