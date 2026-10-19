# Portability Wave 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Those execution skills are unavailable in this session; use the existing task/review gates manually. The user's prohibition on compilation and execution overrides the plans' run/commit steps.

**Goal:** Implement the first numbered wave, T18, T20 and T23, from the supplied portability review and preserve an explicit unverified handoff.

**Architecture:** Keep rendering at interactive boundaries; select process launch policy at the async-worker boundary; keep native installation and artifact diagnostics in the existing source/install owners. Runtime changes share one owner, with independent T23 implementation and static review.

**Tech Stack:** Python >=3.13, uv, MuJoCo 3.12.0, NumPy, multiprocessing, pytest, CMake >=3.27, existing C++23 native bindings.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`; T18/T20 in `2026-10-14-linux-macos-05-runtime.md`; T23 in `2026-10-14-linux-macos-06-delivery.md`.

**Delivery status:** T18/T20/T23 are `implementation_ready`. Completed checkboxes
below record source work and static review only. Every execution/qualification
gate remains unrun under the user's instruction; see
`docs/superpowers/handoff/reports/2026-10-17-wave-1.md`.

## Global Constraints

- Add Linux support; retain macOS support and existing macOS behavior.
- Keep Python >=3.13, CMake >=3.27, and strict ISO C++23 without compiler language extensions.
- Keep MuJoCo exactly 3.12.0 for native execution until its private error ABI is deliberately migrated in a separate change.
- Preserve same-environment native/Python bitwise tests, exception safety, source/artifact identity, and fail-closed verification.
- Do not change physics, fixed integration periods, controller behavior, recording opt-in/final-only semantics, or replay tolerances for this port.
- Use uv for project Python environments. Never silently regenerate the production lock or upgrade dependencies during a build.
- No implicit native compilation on application import; no silent native-to-Python fallback; no required GUI for headless execution.
- Treat the supplied Repomix export as a source snapshot, not a complete Git checkout or a runnable distribution.
- User instruction for this delivery: no project execution, compilation, tests, probes, or environment installation. Source reading, editing, archive handling and patch inspection only.
- Do not claim T01–T17's externally reported results were reproduced here. Do not mark any unrun gate `verified`.

---

## Task 1: T18 display-free execution contracts

**Files:** Create `tests/portability/test_headless_imports.py` and `tests/reference/test_native_linux_headless.py`. Change production imports only for a source-demonstrated rendering side effect.

**Interfaces:** Consume the unchanged CLI `main` functions, `PhysicalRideDriver`, `RideControl`, `environment_factory` and selected native test loader. Preserve all CLI flags, including explicit headless `--time-scale 1` rejection.

- [x] Read each headless entry point and trace rendering imports to their actual call sites. Existing lazy/TYPE_CHECKING imports need no rewrite.
- [x] Add fresh-process import/help cases with `DISPLAY`, `WAYLAND_DISPLAY` and `MUJOCO_GL` removed. Render initialization raises explicitly; importing GLFW bindings alone is allowed.
- [x] Add both-backend driver coverage based on the existing valid control:

  ```python
  driver = PhysicalRideDriver(environment_factory().sim, backend=backend,
                              duration_s=.005, decimation=2, record=False)
  try:
      result = driver.advance(100, RideControl(0.))
      assert result.step == 4
      assert result.outcome == 'duration_reached'
      assert driver.snapshot().integration_state.size > 0
  finally:
      driver.close()
  ```

- [x] Cover the ordinary welded/savage CLI with selected native artifact, actual outcome and first failure, and no output without `--record`. Keep strict invalid results invalid.
- [x] Review failure and interrupt cleanup statically; leave actual subprocess/host results pending.

## Task 2: T20 platform workers and bounded cleanup

**Files:** Modify `src/bike_sim/cli/batch.py`, `src/bike_sim/sim/ride/async_close.py`; update existing async/batch tests and create `tests/portability/test_worker_platform.py` plus guarded `worker_entry_probe.py`. Where needed, fix worker resource ownership in the existing physical driver/runtime and research environment rather than adding another process engine.

**Interfaces:** Preserve `_performance_cores() -> int`, queue message shapes, sequence numbers and publication errors. Add `_worker_start_method(system: str) -> str` and keyword-only `AsyncCloseRunner(start_method: str | None = None)`.

- [x] Gate Apple `sysctl` on `sys.platform == 'darwin'`; retain `os.process_cpu_count() or os.cpu_count() or 1` fallback and explicit job counts.
- [x] Select `spawn` for Linux and retain `fork` for Darwin. Validate an explicit method against `multiprocessing.get_all_start_methods()` before starting resources; never silently fall back to fork.
- [x] Construct queues and process from one context. Preserve the module-scope worker and guarded CLI entry points.
- [x] Make termination bounded, reap a terminated child, and prevent undrained queue feeders from blocking interpreter shutdown. Owners release workers on normal, failed and interrupted close without adding a hidden physics step.
- [x] Cover multiple periods, context replacement, publication/serialization errors, out-of-order replies, worker loss, pending-close cleanup and repeat startup. Inspect recovery ownership so unacknowledged periods are neither lost nor applied twice.
- [x] Add a guarded subprocess probe with NumPy work in a parent thread before worker startup, finite timeout, and child-leak checks. Add real entry-point and serial/pool controls for the later host run.
- [x] Review queue/publication semantics independently; do not claim launcher or BLAS qualification without execution.

## Task 3: T23 source installation and loader diagnostics

**Files:** Modify `src/bike_sim/native/artifact.py`, `tools/dev.py`, `tests/reference/native_loader.py` and their existing tests. Create `tests/portability/test_install_modes.py` and `docs/development/install-modes.md`. A narrow additive `native/cmake/BikeCheckContext.cmake` diagnostic record is allowed only after checking all consumers.

**Interfaces:** Preserve absolute `BIKE_NATIVE_BUILD_PATH` / `NATIVE_TEST_BUILD_PATH`, `load_native_extension(build_dir: Path | None = None)`, supported extension suffixes, exact-one artifact selection and source identity checks. Preserve old context readers; new doctor diagnostics may ask for explicit reconfiguration.

- [x] Establish the package's own source root from its fixed layout and required source markers; never search cwd for an unrelated checkout.
- [x] Distinguish editable source installation from a Python-only wheel. Missing/stale/conflicting artifacts fail with selected build, interpreter/SOABI, platform and explicit remediation.
- [x] Keep dynamic-loader errors and their original exception causes. Preserve dependency, pre-imported module and source-stamp rejection before native use.
- [x] Make dev commands consistently select their uv environment and reject mismatched CMake source/interpreter state. Doctor must inspect without compiling ABI probes.
- [x] Add isolated artifact/install controls and document fresh-source commands, wheel limitations, loader errors and pending real-host installation checks.
- [x] Runtime owner co-reviews `artifact.py`; confirm additive `native_loader.py` behavior for future T21/T22 consumers.

## Handoff and later verification

- [x] Perform independent requirement/correctness and maintainability/resource-ownership reviews of the final diff.
- [x] Update only T18/T20/T23 implementation status and create task reports with source-snapshot provenance, changed files, no-run statement and exact outstanding gate commands.
- [x] Produce a patch against the unchanged extracted snapshot plus an archive of every added/modified file, preserving project-relative paths. Include a final-newline-normalized patch for an ordinary checkout; check both variants as file transformations only.

After execution is authorized on the actual selected environment, the original T18/T20/T23 commands and required `tools/run_tests.sh native` gate remain mandatory. Real Linux/macOS launcher, GUI, native install, sanitizer and numerical results must come from that host. T19 and T21–T30 are outside this numbered wave.
