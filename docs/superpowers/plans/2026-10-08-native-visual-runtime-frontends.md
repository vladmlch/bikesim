# Viewer, Replay and Acceptance Implementation Plan — Phase C

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose native ride/research execution through the existing window and CLI, add checked visual replay, and measure the completed feature.

**Architecture:** Backend-neutral playback and view values drive the existing MuJoCo passive viewer. A shared incremental ReplaySession supplies either the verifier CLI or the new replay window. Backend identity, physical chronology and presentation timing are separate.

**Tech Stack:** Python, MuJoCo passive viewer, Phase A/B native runtime, existing camera/HUD/recorders, pytest and explicit Release measurement.

**Spec:** [Design](../specs/2026-10-08-native-visual-runtime-design.md), sections 1 and 5–7.

## Global Constraints

Apply [the master constraints](2026-10-08-native-visual-runtime.md#global-constraints). C1's pure clock work can start after interfaces are stable; integration and final acceptance require A4 and B3.

## C1. Shared playback and immutable physical presentation

**Files:**

- Create `src/bike_sim/sim/playback.py`, `src/bike_sim/sim/ride/physical_view.py`.
- Modify `src/bike_sim/sim/ride/hud.py`, `src/bike_sim/sim/ride/viewer.py`, `src/bike_sim/sim/research/viewer.py`, `src/bike_sim/native/contracts.py`, `src/bike_sim/native/runtime.py`.
- Create `tests/reference/test_playback_clock.py`, `tests/reference/test_physical_view.py`, `tests/reference/test_native_viewer_snapshot.py`.

**Interfaces:**

```text
PlaybackClock(timestep_s: float, *, scale: int = 1)
PlaybackClock.rebase(now: float, *, step: int) -> None
PlaybackClock.target_step(now: float, *, current_step: int) -> int
PlaybackClock.set_scale(scale: int, *, now: float, step: int) -> None
PlaybackClock.set_paused(paused: bool, *, now: float, step: int) -> None
speed_key(keycode: int, current_scale: int) -> int | None
make_physical_view(sim, *, requested_scale: int, achieved_rtf: float | None) -> PhysicalViewState
apply_frame(model, data, frame: FrameSnapshot) -> None
```

The clock remembers last observed completed step. On each request it subtracts actual completed work from debt, adds newly elapsed scaled wall time, caps debt and returns an absolute target. A budget-limited native call leaves uncompleted debt for the next request.

`PhysicalViewState` in contracts contains `drive_mode`, latest schema-2 `sample` or None, `endpoint`, and `preview_row`. Endpoint includes time, position, speed, pitch, torso pitch and named rider joint angles required by existing HUD geometry columns. `preview_row` preserves every existing preview_log_row key/value meaning. Its requested scale/achieved RTF are presentation values added by the caller.

- [ ] **Add deterministic playback regressions with a fake clock.**

```python
from bike_sim.sim.playback import PlaybackClock, speed_key

def test_budget_yield_preserves_uncompleted_pacing_debt():
    clock = PlaybackClock(0.00125, scale=2)
    clock.rebase(0.0, step=0)
    assert clock.target_step(0.01, current_step=0) == 16
    assert clock.target_step(0.01, current_step=8) == 16
    clock.set_scale(4, now=0.01, step=8)
    assert clock.target_step(0.02, current_step=8) == 40

def test_speed_keys_do_not_reuse_physical_controls():
    assert speed_key(295, 2) == 1   # GLFW_KEY_F6
    assert speed_key(296, 2) == 4   # GLFW_KEY_F7
    assert speed_key(297, 8) == 1   # GLFW_KEY_F8
    assert speed_key(32, 2) is None
```

Add negative elapsed handling, exact fractional remainder, catch-up cap, invalid timestep/scale, pause/resume, reset after large backlog, scale ladder ends and CPU-limited advancement.

Run:

```bash
uv run --frozen --group native python -m pytest tests/reference/test_playback_clock.py -q
```

- [ ] **Implement clock arithmetic and speed mapping.**

Use the following update order; the clock validates monotonic completed step/generation through rebase:

```text
completed = current_step - last_step
debt = max(0, debt - completed * timestep)
elapsed = max(0, now - previous_wall)
if running: debt = min(debt + elapsed * scale, 0.05 * scale)
last_step = current_step
previous_wall = now
target = current_step + floor(debt / timestep)
```

Rebase resets debt/time/step without changing physics. Keep sub-step remainder in debt. Paused clocks request current_step only. Scale changes rebase rather than applying the new scale retroactively.

Retain legacy RealTimePacer behavior for legacy ride. Use the new clock only for physical and research/replay paths.

- [ ] **Extract physical formatting and implement frame replication.**

Move physical HUD numeric reads and preview-row production behind PhysicalViewState. Python uses `make_physical_view`; native snapshots supply equivalent native-computed values. Reuse the existing formatting, units, labels and blank-channel rules. Test equality against captured current Python HUD/CSV output before changing its consumers.

Create render model/data replicas during setup from the same compiled model. Copy full required integration fields from an owned snapshot and recompute replica kinematics only. Native metadata supplies authoritative metrics; any replica diagnostic state is presentation-only. Retain the frame through rendering so its arrays cannot be invalidated.

Runtime loop shape:

```text
drain pending keys
compute requested absolute target using completed runtime step
advance at most one 8 ms compute slice toward target
if a 60 Hz render deadline is due:
    obtain one owned snapshot and update replica
    update camera under viewer lock; viewer.sync()
if HUD/preview deadline is due:
    format/export the current owned view values
yield briefly when ahead of the presentation schedule
```

For research, pump the B2 pending control interval and call policy act only at its next boundary. Pause/brake/reset/stop keys take effect at external boundaries; frame refresh can happen at an intermediate compute yield.

- [ ] **Verify C1 and checkpoint.**

Use fake viewer/context/clock objects to assert render calls are wall-clock limited, snapshot generation resets are handled and no native buffer is read concurrently. With a real supported native runtime, change replica qpos and assert the native integration state is unchanged. Retain an old snapshot across reset/close and verify it is still readable and unchanged.

```bash
uv run --frozen --group native python -m pytest tests/reference/test_playback_clock.py tests/reference/test_physical_view.py -q
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_viewer_snapshot.py -q
```

Completion: clock and fake-window tests pass; physical HUD/CSV values match the existing Python path; replica ownership is verified. Checkpoint: `feat: share playback clock and physical view snapshots`.

## C2. Wire ride/research CLI selection and native sessions

**Files:**

- Modify `src/bike_sim/cli/ride.py`, `src/bike_sim/cli/research.py`.
- Modify `src/bike_sim/sim/ride/physical_session.py`, `src/bike_sim/sim/ride/session.py`, `src/bike_sim/sim/ride/input.py`, `src/bike_sim/sim/ride/viewer.py`, `src/bike_sim/sim/ride/console.py`.
- Modify `src/bike_sim/sim/research/configuration.py`, `src/bike_sim/sim/research/viewer.py`.
- Create `src/bike_sim/sim/ride/physical_driver.py`.
- Create `tests/reference/test_ride_backend_cli.py`, `tests/reference/test_native_ride_frontend.py`, `tests/reference/test_native_research_frontend.py`.

**Interfaces:**

- `build_physical_driver(track, args, rider)` returns a Python or native driver with advance/snapshot/flush/reset/close and immutable setup metadata.
- `PythonRideDriver` in physical_driver implements the same physical driver boundary by using the existing Python simulation. NativeRideDriver remains in the native package.
- Physical session owns brake state, camera, terminator and presentation events; it does not reach into native writer objects.
- CLI `--backend` choices/default and `--time-scale` follow design section 1.

- [ ] **Add argument and dispatch regressions.**

```python
import pytest
from bike_sim.cli.ride import parse_args

PROFILE = "examples/research/viewer_physics_welded.toml"

def test_native_visual_arguments_preserve_physics():
    args = parse_args(["--physics-config", PROFILE, "--backend", "native",
                       "--time-scale", "4"])
    assert args.backend == "native"
    assert args.time_scale == 4
    assert args.resolved_physics.timestep_s == 0.00125

def test_default_backend_remains_python():
    assert parse_args(["--physics-config", PROFILE]).backend == "python"

def test_headless_rejects_explicit_playback_scale():
    with pytest.raises(SystemExit):
        parse_args(["--physics-config", PROFILE, "--headless",
                    "--duration", "1", "--time-scale", "2"])
```

Add mocked builder dispatch for all four live/headless ride/research routes, missing selected artifact and unsupported configuration. A plain Python command must work while native import is forced to fail.

- [ ] **Implement parser and factory wiring.**

Add:

```python
parser.add_argument("--backend", choices=("python", "native"), default="python")
parser.add_argument("--time-scale", type=int, choices=(1, 2, 4, 8), default=None)
```

Use None only to track explicit time-scale use, reject it for headless and legacy paths, then resolve absent scale to 1. `bike-research` stays headless and accepts only the backend flag.

Validate the native capability matrix after config/rider resolution and before native construction/output creation. Build Python setup once; initialize the selected driver. For research, use the existing initialized environment as the native bootstrap source before any running step. Keep configured dt/control/sensor periods, source metadata and default flags unchanged.

Call `ensure_macos_mjpython()` before expensive setup on each graphical route. Preserve new flags and `BIKE_NATIVE_BUILD_PATH` across re-exec. On missing artifact, print the exact selected directory and documented Release build commands; return the existing CLI argument/setup error code without silently selecting Python.

- [ ] **Integrate physical run/session behavior.**

Preserve physical Space brakes, comma/period strength, R reset, camera/view/telemetry/marker controls and existing fixed-configuration notices. Intercept F6/F7/F8 before the mode's ordinary dispatch. Research retains Space pause, B brakes, R save/reset and Q stop.

Ride termination retains its existing reason precedence and final frame. For wall-clock limits, check host time in the adapter; state/time/position/crash come from native values. Include finish/max-step checks inside bounded advancement so a render batch cannot overrun the terminal step. Keep physical `--duration` behavior aligned between backends and avoid adding a new default duration.

Preserve preview.csv, generation markers, final flush and optional ride.html. For headless physical native execution, export current telemetry/summary/plot inputs from native records rather than reconstructing forces through Python. All exits close the driver after evidence/output handling.

- [ ] **Verify C2 and checkpoint.**

Exercise parser/dispatch/fake-window tests; run a short native headless physical and research episode through their public CLI with temporary output directories. Compare outcomes and schemas to Python using identical arguments. Verify reset clears pacing/RTF state while preserving selected backend/configuration.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_ride_backend_cli.py tests/reference/test_native_ride_frontend.py tests/reference/test_native_research_frontend.py -q
```

Completion: all requested entry paths reach the correct runtime and keep CLI/output semantics. Real GUI smoke is reserved for C4. Checkpoint: `feat: launch native physical and research rides`.

## C3. Same-backend checked replay with an optional window

**Files:**

- Modify `src/bike_sim/sim/research/replay.py`, `src/bike_sim/cli/replay.py`.
- Create `src/bike_sim/sim/research/replay_session.py`, `src/bike_sim/sim/research/replay_viewer.py`.
- Extend native research adapter if incremental replay needs a read-only export accessor.
- Create `tests/reference/test_replay_session.py`, `tests/reference/test_native_replay.py`, `tests/reference/test_replay_viewer.py`.

**Interfaces:**

```text
ReplaySession(directory: Path)
ReplaySession.advance(*, wall_budget_s: float | None = None) -> bool
ReplaySession.snapshot() -> FrameSnapshot
ReplaySession.restart() -> None
ReplaySession.close() -> None
ReplaySession.report() -> dict
ReplaySession.done: bool
run_replay_viewer(session: ReplaySession, *, time_scale: int = 1) -> int
```

Advance returns true only when final verification is complete; false means more recorded execution remains. report before complete raises RuntimeError. Mismatch raises the existing replay error classes and latches failure for presentation. The session never evaluates a policy factory.

- [ ] **Add checked replay regression through a real recording.**

```python
import pytest
from _native_runtime_support import make_python_research
from bike_sim.native.research import create_native_research
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.replay_session import ReplaySession

@pytest.mark.slow
def test_native_recording_replays_with_recorded_backend(tmp_path):
    env = create_native_research(make_python_research(duration=0.02))
    while not env.done:
        env.step(RideControl())
    destination = tmp_path / "recording"
    env.save(destination)
    session = ReplaySession(destination)
    with pytest.raises(RuntimeError, match="incomplete"):
        session.report()
    while not session.advance(wall_budget_s=None):
        pass
    report = session.report()
    assert report["passed"] is True
    assert report["physics_steps"] == env.sim.steps
    session.close()
```

Add Python recording controls, schema-1 compatibility, a deliberately changed command with updated checksum (must fail transition comparison), changed observation/final state, changed artifact hash, unavailable selected extension and unexpected backend value. Test policies are not imported during replay.

- [ ] **Refactor verifier into the shared incremental session.**

Move current validate/rebuild/compare/control-loop/final-check logic into ReplaySession without loosening comparisons. Keep `replay_episode(directory)` as a synchronous convenience that advances to completion and returns report.

Schema-1 selects Python. Schema-2 selects its recorded backend; native reconstruction first verifies source/runtime/artifact identity and then builds the checked fresh Python setup followed by native import. Check initial integration state, terrain/configuration fingerprints and bundled rider data before the first control.

Maintain these independent cursors: requested command index, expected observation/transition index, applied-command comparison and initial/final state checks. A budget return resumes the same requested command through B2's active window. On the final command, finish current comparisons, normalize an explicit recorded operator_stop exactly as existing replay does, then compare final outcome/metrics/state before setting done.

restart reconstructs/reset-validates the recorded initial environment and resets all verification cursors. It does not reuse a partially consumed sensor tape or previous final-state cache.

- [ ] **Implement replay CLI and viewer.**

Add `--viewer` and explicit-only `--time-scale` to bike-replay. Headless behavior remains checked verification. No backend override is exposed.

Use the common playback clock and snapshot replica. Space pauses, R restarts, C changes camera, Q/Escape exits; F6/F7/F8 change rate. Rendering consumes snapshots; it never supplies altered controls to the replay environment.

End states:

```text
all comparisons pass -> report passed=true; exit 0
compatibility/state mismatch -> stop advancing; print mismatching path; exit 1
window closed before final verification -> print incomplete; exit 2
```

On a mismatch, preserve the last consistent snapshot and show its error until the window closes. On successful completion, hold the final frame for inspection; closing after success returns 0. Re-exec under mjpython before creating ReplaySession when --viewer is selected.

- [ ] **Verify C3 and checkpoint.**

Run native/Python replay round trips and fake-window tests. Verify speed/FPS/budget differences produce the same final report; early close never emits passed=true; reset repeats the same verification; checksum-valid semantic tampering is still detected.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_replay_session.py tests/reference/test_native_replay.py tests/reference/test_replay_viewer.py tests/reference/test_replay_manifest_v2.py -q
```

Completion: headless/visual drivers share the same verification state machine and same-backend recording contract. Checkpoint: `feat: add checked visual replay for recorded backends`.

## C4. Release performance, complete verification and user documentation

**Files:**

- Modify `tools/measure_realtime.py`, `tests/reference/test_measure_realtime_report.py`.
- Create `tests/reference/test_native_runtime_performance_contract.py`.
- Update `docs/RIDE.md` if present; otherwise create it as the user-facing physical ride/research/replay guide.
- Update `docs/TESTING.md` and the A0 audit.
- Update native manifests and `tools/native_checks.py` only if new target/source registration requires it.

**Interfaces:** measurement accepts `--backend python|native`, optional `--dt` defaulting to the profile, `--runs` and `--warmup`; output records selected runtime, effective clocks, complete wall time, setup/flush/export timings, outcome/validity and artifact provenance.

- [ ] **Add measurement-contract tests before changing the benchmark.**

```python
from bike_sim.cli.ride import parse_args

def test_profile_timestep_is_not_replaced_for_native_measurement():
    args = parse_args([
        "--backend", "native", "--physics-config",
        "examples/research/viewer_physics_welded.toml",
        "--track", "examples/research/rough_uphill_savage.toml",
        "--headless", "--duration", "1", "--no-plots",
    ])
    assert args.resolved_physics.timestep_s == 0.00125
```

Extend the existing measurement report tests with fake drivers/clocks: whole-loop wall time includes Python/native boundary and final flush; setup/export are separate; different dt/config/record settings are marked non-comparable; a budgeted packet is not mislabeled as a single-step timing sample.

- [ ] **Measure the full runtime through its normal boundary.**

Use monotonic time around the complete measured advancement loop plus flush, rather than summing only native kernel times. Python and native use identical controls/record settings and the supported profile. For native speed, advance in policy/internal-clock packets rather than invoking Python once per physics step.

Report chunk latency percentiles with their explicit units. Preserve per-step metrics only when measured directly; identify Python outer timing versus native internal instrumentation. Do not manufacture per-step p95 by dividing a packet percentile.

Run baseline/native sequentially, one warm-up then three measurements per backend:

```bash
uv run --frozen --group native python tools/measure_realtime.py --backend python --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 30 --dt .00125 --warmup 1 --runs 3 --out output/native-visual-acceptance/python
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python tools/measure_realtime.py --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 30 --dt .00125 --warmup 1 --runs 3 --out output/native-visual-acceptance/native
```

Record all measured prefixes/outcomes and median RTF. Compare the same valid or equally classified natural terminal prefix. If it is too short or outcomes diverge, report the limitation and investigate; do not change dt, tolerances or the physical model to make the speed gate pass.

- [ ] **Run required verification on final code.**

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
git diff --check
```

Run required IDEA file inspections with warnings enabled and resolve findings related to these changes. Verify targeted native ownership/engine-error tests under ASan/UBSan with actual loader/runtime provenance. Add new native warm regions to RTSan only where their no-allocation/blocking behavior has been established; record any excluded coefficient-changing path explicitly.

- [ ] **Exercise the real macOS windows.**

Launch these only during execution acceptance, one at a time:

```bash
uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 1
uv run --frozen --group native bike-ride --backend native --research --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 1 --out output/native-visual-acceptance/research-viewer
uv run --frozen --group native bike-replay output/native-visual-acceptance/research-viewer/episode-0000 --viewer --time-scale 2
```

Use new output directories if acceptance artifacts already exist. Check opening/re-exec, camera, physical brakes/strength, reset, telemetry/preview output, research pause/save/reset, replay pause/restart/early exit, and F6/F7/F8. Record actual requested/achieved RTF and UI behavior. A source read or fake viewer is not this GUI check.

- [ ] **Publish the usage and final audit.**

Document the one-time Release build, explicit backend selection, supported capabilities, fixed physics versus playback scale, speed keys, artifact selection, research outputs, same-backend replay and meaning of incomplete replay. Include the user's exact profile/track launch at 1x and 2x.

Completion requires measured native improvement, native unpaced median RTF >= 1.0, visual 1x mean RTF >= 0.95, preserved parity/replay contracts and passing required verification. Report higher-scale achieved RTF without promising 2x/4x/8x throughput. Preserve failed physical validity classifications as evidence.

Update the audit with commands, source/artifact IDs, test counts/failure IDs, inspections, GUI observations, timings and any unresolved requirements. A required unresolved gate prevents an unqualified complete/green claim. Checkpoint: `feat: verify and document native visual simulation`.
