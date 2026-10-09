# Native Frontends (Track C) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the Track C source changes without executing the project, a build, a test, a benchmark, or an analyzer.

**Architecture:** Owned frame values separate simulation from a render replica. Python and native physical drivers share frontend semantics; research retains its external control-window transaction. One incremental, same-backend verifier drives headless and visual replay.

**Tech Stack:** Existing Python/NumPy/MuJoCo frontends; C++23/nanobind native runtime.

**Spec:** `docs/superpowers/specs/2026-10-08-native-visual-runtime-design.md` and `docs/superpowers/plans/2026-10-08-native-visual-runtime-frontends.md`.

## Global Constraints

- Keep the Python backend and 1x presentation as defaults.
- Preserve physical timestep, numerical operation order, controller periods, physical checks and existing tolerances.
- Use one owner thread per runtime. Release the GIL only around code that touches no Python objects.
- Keep external `module:factory` motor policies in Python, called once per external policy boundary.
- The supplied welded profile uses `timestep_s=0.00125`; comparative measurements retain that effective value unless the user explicitly overrides it for both backends.
- User override: source inspection/editing/packaging only. Every executable verification gate remains unexecuted.
- The input omits `.git`, root packaging/lock files, upstream tools and most upstream tests. Preserve this source-overlay boundary instead of inventing an upstream checkout or test result.
- Checked source tasks below mean written/inspected files, not successful execution. See `docs/native-visual-track-c.md` for the handoff and unexecuted gates.

## Task 1: Playback and owned presentation (C1)

**Files:** create `src/bike_sim/sim/playback.py`, `src/bike_sim/sim/ride/physical_view.py`, `src/bike_sim/sim/ride/presentation.py`, `native/src/runtime/presentation.hpp` and `.cpp`; extend contracts, snapshot adapters and the HUD.

**Interfaces:** `PlaybackClock.target_step(now, current_step=...) -> int`; `apply_frame(model, data, frame) -> None`; `PhysicalStep::initialize_presentation(PresentationState &) const`, `capture_presentation(PresentationState &) const`, `capture_accounted_presentation(PresentationState &) const`; `PresentationState::as_wire() const -> WireObject`.

- [x] Add clock sources and deterministic test sources, including the debt regression:
  ```python
  clock = PlaybackClock(.00125, scale=2)
  clock.rebase(0., step=0)
  assert clock.target_step(.01, current_step=0) == 16
  assert clock.target_step(.01, current_step=8) == 16
  ```
- [x] Preserve the complete preview schema and historical HUD formatting behind immutable values. Cache native endpoint values at committed boundaries; use the NumPy archive's `npy_rad2deg` multiplication order and MuJoCo's complete integration-state contract.
- [x] Add replica ownership, complete-model-field and original-HUD comparison test sources. Do not execute them.

## Task 2: Physical drivers and CLI routes (C2)

**Files:** create `src/bike_sim/sim/backend.py`, `src/bike_sim/sim/ride/physical_driver.py`, `physical_frontend.py`; modify ride/research CLIs, physical session and research factory.

**Interfaces:** `build_physical_driver(track, args, rider, *, seed=None, strict=None)` returns an owning driver; driver methods are `advance`, `snapshot`, `flush`, `reset`, `close`, `make_render_model` and `export` with native-backed recording.

- [x] Add backend/default/explicit-scale parser regressions. Example rejection: `parse_args(['--physics-config', PROFILE, '--headless', '--duration', '1', '--time-scale', '2'])`.
- [x] Validate native capability and selected artifact before setup/output; retain fixed dt and strictness. Check native finish position inside advancement, not once per rendered frame.
- [x] Integrate keys, preview generation markers, summary/telemetry export, and cleanup. Add mocked-window lifecycle and native physical record parity sources. Do not execute CLI or GUI routes.

## Task 3: Budgeted research frontend (C1/C2)

**Files:** modify `src/bike_sim/sim/research/viewer.py`, `environment.py`, `policy_session.py`, `src/bike_sim/native/research.py` and native research runtime/bindings.

**Interfaces:** `PolicySession.begin_advance()` / `advance_pending(wall_budget_s=.008, target_step=...)`; `ResearchEnvironment.advance_control(*, wall_budget_s=None, target_step=None)` and native equivalent. One policy call per external window; an optional absolute physics target yields without completing that window.

- [x] Pace completed physics steps, render at most 60 Hz, queue brake/pause/stop changes at external boundaries, finish the active window before saving/resetting/closing.
- [x] Add policy-call-count and boundary test sources for Python/native without running them.

## Task 4: Checked replay (C3)

**Files:** create `src/bike_sim/sim/research/replay_session.py`, `replay_viewer.py`; modify replay reconstruction and CLI.

**Interfaces:** `ReplaySession.advance(wall_budget_s=None, *, target_step=None) -> bool`, `snapshot`, `restart`, `close`, `report`; schema 1 means Python, schema 2 uses the recorded execution identity.

- [x] Share initial/requested-command/transition/observation/applied/final/outcome/metrics comparisons across headless and viewer paths at existing `1e-9` tolerances. Never load a recorded policy factory.
- [x] Keep `report()` unavailable before all checks, including the final state. Preserve the last verified frame on mismatch; restart creates a fresh verified owner. Exit codes: pass 0, mismatch 1, incomplete viewer 2.
- [x] Add round-trip, tamper, restart, partial-budget, empty/saved-prefix and early-close regression sources without execution.

## Task 5: Measurement and source handoff (C4)

**Files:** create `tools/measure_realtime.py`, `src/bike_sim/sim/realtime_measurement.py`, measurement contract tests, `docs/RIDE.md`, and `docs/native-visual-track-c.md`; update `docs/TESTING.md`.

**Interfaces:** profile dt unless explicitly overridden; `--backend`, `--runs`, `--warmup`; full-loop wall time includes final flush, while setup and export remain separate. Measurement requires an existing selected extension and never builds one.

- [x] Report chunk latency as chunk latency, not inferred per-step p95. Record provenance, comparability keys, classified prefix, outcomes and validity alongside RTF. Require at least one warm-up and three valid matching runs before publishing the native Release headless threshold result.
- [x] Write usage, reference-source hashes, delivery scope and unexecuted acceptance gates.
- [x] Complete final diff/whitespace and archive-content inspection, then package source plus a patch against the supplied archive.
- [ ] Release build, full/native profiles, ASan/UBSan, RTSan, IDE inspection, numerical parity, replay execution, performance and macOS GUI gates. **Intentionally not executed; no runtime acceptance claim.**
