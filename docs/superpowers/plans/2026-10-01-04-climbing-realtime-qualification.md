# Climbing Realtime Qualification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. The user selected sequential inline execution; do not reopen the execution-mode question.

**Goal:** Выбрать физически проверенный профиль сидячего подъёма с 34T/30T и подтвердить его realtime на компьютере пользователя, включая интерфейс моторной политики.

**Architecture:** Существующий distributed tire и geometric reduced drive сначала проверяются на коротких стендах. Измеряются отдельно physics, recording и GUI; ускоряются только измеренные hotspots с проверкой физического результата. Один новый opt-in профиль выпускается с точной областью применимости, performance evidence и воспроизводимым extreme outcome.

**Tech Stack:** Python через uv, MuJoCo, NumPy, pytest, cProfile/pstats, TOML/JSON; существующие fidelity/replay/research pipelines.

**Spec:** `docs/superpowers/specs/2026-10-01-seated-realtime-climbing-design.md`.

## Global Constraints

- Python запускать только через `uv`.
- Работать в текущем checkout; сохранять чужие dirty/staged/untracked файлы и старые результаты.
- Не создавать commits/branches и не менять существующие default-профили без отдельного запроса.
- Реализовывать последовательно inline; уже выбранный пользователем способ исполнения не спрашивать повторно.
- Не писать anti-wheelie алгоритм; моторная политика не управляет райдером.
- Не приклеивать стопы/таз, не добавлять внешнюю pitch stabilization, не переписывать running qpos/qvel.
- Не увеличивать мотор 85 Н·м / 600 Вт, не менять трассы/seeds и заднюю кассету ради прохождения.
- Сравнивать передние звёзды 34T и 30T; выбранный вариант выпускать отдельным opt-in профилем.
- Не ослаблять energy residual 0.05 и существующие model-scope gates.
- Отмечать параметры человека/шин как синтетические; достоверность реального велосипеда требует измерений.

---

## Dependencies and file map

Required predecessor: plan 03, Tasks C1–C4. Keep its unresolved invalid contact intervals visible; fixing their scope is D2, not a reason to fake a successful handoff. Existing B2/B3/B4 in the earlier plant-qualification plan remain obligations, not completed work.

| File | Responsibility |
|---|---|
| `src/bike_sim/validation/climbing_benchmark.py` (new) | Exact physical profile/track unpaced benchmark and profile identity |
| `src/bike_sim/validation/climbing_campaign.py` (new) | Controlled scenarios, gear comparison, validity and performance evidence manifest |
| `src/bike_sim/sim/ride/physical_session.py` | Preview provenance, unique run artifacts and configurable flushing |
| `src/bike_sim/sim/ride/viewer.py`, `src/bike_sim/sim/research/viewer.py` | Independent rendering frequency and measured lag/frame intervals |
| `src/bike_sim/cli/ride.py` | Explicit `--render-fps` / preview logging options |
| `src/bike_sim/validation/fidelity_sweep.py`, `antiwheelie_bench.py` | Existing numeric/mechanical gates; profile-aware brake hold |
| `src/bike_sim/sim/ride/distributed_tire_forces.py`, `src/bike_sim/terrain/profile_distance.py` | Only measured contact hotspots, if profiling identifies them |
| `examples/research/seated_climb_realtime.toml` (new, after acceptance) | Final opt-in physical profile; no misleading reference certification |
| `docs/RIDE.md`, `docs/ANTI_WHEELIE.md`, `docs/RESEARCH_COMPLETION.md`, `README.md` | Verified commands, scope, human/pedal assumptions, realtime evidence |

Before using a listed path, check it against the current checkout. Baseline source `2e93585` has `profile_distance.py` under `src/bike_sim/terrain/`; preserve existing module ownership when optimizing.

Commands from repository root:

```bash
export UV_CACHE_DIR=/private/tmp/uv-cache-antiwheelie-plan
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
```

### Task D1: Reproducible timing and preview provenance

**Files:** Create `src/bike_sim/validation/climbing_benchmark.py`, `tests/test_climbing_benchmark.py`; modify `src/bike_sim/sim/ride/physical_session.py`, `viewer.py`; extend `tests/test_physical_reproducibility.py`.

**Interfaces:**
- `timing_metrics(step_ns: list[int], dt_s: float) -> dict`: count, p50/p95/p99/max step milliseconds and unpaced real-time factor. Reject empty/nonpositive/nonfinite intervals and nonpositive dt; do not count setup as stepping.
- `run_benchmark(physics_profile, track_path, *, mode: str, duration_s: float, repeats: int, output_dir) -> dict`; `mode` is `preview`, `accounted`, or `research`. Same constructor/config/commands; preview selects existing `preview_mode()`, accounted uses immutable samples, research uses shared environment with passthrough policy and recording. Factory builds a fresh deterministic episode each repeat.
- Module CLI flags: `--physics-config`, `--track`, `--mode`, `--duration`, `--repeats`, `--output`. Reject occupied output directories. Save source/config/terrain/versions/hardware, warmup/setup/step/writing times, simulation horizon/progress, state hashes and failure reason. Only fully completed required windows count in throughput comparisons.
- Preview stores `metadata.json` before stepping, generation/reset records and termination details. Use `configuration_metadata(sim)` and a unique numbered run directory; never overwrite an older CSV with the same configuration hash.

- [ ] **1. Add red timing/provenance tests.**

```python
import pytest
from bike_sim.validation.climbing_benchmark import timing_metrics

def test_throughput_uses_actual_steps_and_elapsed_time():
    result = timing_metrics([1_000_000] * 100, .00125)
    assert result['real_time_factor'] == pytest.approx(1.25)
    assert result['p99_step_ms'] == pytest.approx(1.)

@pytest.mark.parametrize('samples', [[], [0], [-1], [float('nan')]])
def test_invalid_timing_is_not_a_speed_claim(samples):
    with pytest.raises(ValueError):
        timing_metrics(samples, .00125)
```

Add metadata test proving different source hashes do not reuse a run path, and an existing path is not truncated. Capture all candidate configuration overrides, including front teeth and seated intent.

- [ ] **2. Implement an unpaced benchmark.** Use `time.perf_counter_ns()` around actual stepping, no sleep or viewer pacing. Exclude initialization and report it separately. Streaming timing aggregates are preferable for long runs; a short array is sufficient for percentiles in bounded scenarios. Run uninstrumented repeats for throughput and separate cProfile runs for hotspot attribution. Save partial evidence on exception, model invalidity or crash; do not compute a passing full-window RTF from the partial prefix.

```python
from time import perf_counter_ns

step_ns = []
for interval_index in range(required_steps):
    start_ns = perf_counter_ns()
    sim.step(control=control)
    step_ns.append(perf_counter_ns() - start_ns)
```

The actual implementation obtains `required_steps` from an integer-multiple horizon, applies the same automatic `RideControl()`/runtime intent as C3, evaluates termination/gates after each step and records the final interval. Preview mode has no full energy account: pair it with the accounted trajectory, never mark its missing energy as verified.

- [ ] **3. Measure stable C3 candidate, not only the old fast profile.**

```bash
uv run --no-sync -m bike_sim.validation.climbing_benchmark --physics-config examples/research/seated_climb_candidate.toml --track examples/research/rough_uphill_extreme.toml --mode preview --duration 8 --repeats 3 --output verification/seated-climb-20261001/timing-preview
uv run --no-sync -m bike_sim.validation.climbing_benchmark --physics-config examples/research/seated_climb_candidate.toml --track examples/research/rough_uphill_extreme.toml --mode accounted --duration 8 --repeats 3 --output verification/seated-climb-20261001/timing-accounted
uv run --no-sync -m cProfile -o /private/tmp/seated-climb-profile.prof -m bike_sim.validation.climbing_benchmark --physics-config examples/research/seated_climb_candidate.toml --track examples/research/rough_uphill_extreme.toml --mode preview --duration 8 --repeats 1 --output verification/seated-climb-20261001/profile-preview
```

Eight seconds covers initial coast/restart, not all obstacles. Repeat measurement on full-course runs or declared local grade/root/step fixtures in D4; do not certify extreme performance from its flat beginning.
- [ ] **4. Test and record.** `uv run --no-sync pytest -q tests/test_climbing_benchmark.py tests/test_physical_reproducibility.py`. Deliverable: reproducible measurement identifying the dominant runtime costs and missing realtime margin.

### Task D2: Qualify tires, reduced drive and integration resolution

**Files:** Modify `src/bike_sim/validation/antiwheelie_bench.py`; create `src/bike_sim/validation/climbing_campaign.py`, `tests/test_climbing_campaign.py`; extend `tests/test_antiwheelie_bench.py`; use `contact_resolution.py`, `drive_suspension_rig.py`, `fidelity_sweep.py` and their existing tests.

**Interfaces:**
- Add profile-aware `brake_hold_report(physics_profile, *, grade=.05, duration_s=3.) -> dict` to campaign module. It consumes C3 profile, zero rider/motor effort, both brakes, and a freshly settled candidate initial state. Reports displacement, velocity, external/drive/brake work, energy, scope and source/config identity.
- Reuse `run_contact_resolution`, `run_comparison`, `run_fidelity_sweep`; no replacement numerical framework.
- The campaign manifest names each required scenario, horizon, profile, source, resolution axes and criteria before collecting accepted evidence. Store coarse failures rather than selecting only passing rows.

- [ ] **1. Repair qualification coverage, not the legacy elastic chain.** Existing `antiwheelie_bench` mandatory `brake_hold` uses a different lumped/elastic configuration and previously failed equilibrium. Keep that historical failure visible as legacy diagnostic; use the new profile-aware hold as the candidate's gate. Add meaningful tests rejecting empty runs, positive motor work during zero-command hold and early model-invalid termination. On the settled hold, require displacement ≤2 mm, peak speed ≤0.01 m/s and energy residual ≤0.05; document these as engineering fixture thresholds.
- [ ] **2. Compare existing contact/drive options.** Run local tests for two supports, 4/6 cm steps, recontact, suspension motion and load transfer. Prefer `distributed_2d_reference` with `geometric_ideal_mid_drive` when accepted; a single-contact result with `multi_support` is not a cheaper equivalent. Track physical profile parameters used by each fixture, including front teeth.
- [ ] **3. Freeze source and run orthogonal sweeps.**

```bash
uv run --no-sync pytest -q tests/test_distributed_tire_kernel.py tests/test_distributed_tire_integration.py tests/test_geometric_transmission.py tests/test_drive_suspension_rig.py tests/test_fidelity_sweep.py tests/test_antiwheelie_bench.py tests/test_climbing_campaign.py
uv run --no-sync -m bike_sim.validation.fidelity_sweep --physics-config examples/research/seated_climb_candidate.toml --backend distributed_2d_reference --transmission geometric_ideal_mid_drive --stations --jobs 1 --time-steps 0.00125 0.000625 0.0003125 --road-steps 0.01 0.005 0.0025 --station-counts 128 256 512 --output verification/seated-climb-20261001/fidelity
```

Ensure candidate actually declares the requested backend/transmission and record resolved values; CLI names must not mislabel a different profile. All axes share the existing penultimate-resolution anchor and same physical initial condition. Use existing station-state projection when contact station counts differ. If an axis fails, refine that axis and its anchor as necessary; don't relax tolerances or compare mismatched horizons. A full scenario can bifurcate near stall; use matched local obstacle entry states for numerical comparison and separately report scenario outcome sensitivity.
- [ ] **4. Select a tested operating tuple.** Record `(dt, dx, stations)` and successful comparison to finer resolution. Lower station count is allowed only with passing load/pitch/contact metrics. High stiffness contact may require smaller dt; seek cost reductions in D3 before declaring realtime impossible.

Deliverable: current-source mechanical/numerical evidence for a specific candidate tuple. Synthetic qualification is distinct from real-tire calibration.

### Task D3: Realtime optimizations preserving the physical trajectory

**Files:** Modify `src/bike_sim/cli/ride.py`, `sim/ride/viewer.py`, `sim/research/viewer.py`, `sim/ride/physical_session.py` under `src/bike_sim/`; extend `tests/test_ride_cli.py`, `test_ride_track.py`, `test_research_viewer.py`; create `tests/test_preview_io.py`. Modify a physical hotspot only if D1 profiling identifies it.

**Interfaces:**
- CLI `--render-fps` finite in `[1, 120]`, default retains current rendering behaviour; explicitly use 30 for the new documented command. Pass into both viewer implementations, never into timestep or control-period resolution.
- Preview-only `--preview-log-period` in simulated seconds, default `.01`; `--preview-flush-period` in wall seconds, default `0.` (old immediate behaviour), opt-in `.25` for realtime. Physical integration configuration hash is unchanged by I/O preferences; store them in run metadata separately.
- Extend constructor to `PhysicalPreviewCsv(path: Path, interval_s: float = .01, *, flush_period_s: float = 0., clock=time.monotonic)`; immediate flush for markers/close/reset, bounded wall-time flush for normal rows. Preserve the final data on graceful exception/termination.
- `RealTimePacer` additionally records cumulative capped wall debt without changing its integer-step behaviour. Distinguish sub-dt remainder from lost lag; never convert lost lag into completed simulation time.

- [ ] **1. Add failing IO/render-independence tests.** Use a controllable fake clock for periodic flush; test marker/close on an incomplete episode. Extend existing render-grouping equivalence to the C3 rider with 30/60 Hz schedules. Explicitly assert identical applied commands, states and contact metrics, not just identical number of frames.

```python
from bike_sim.sim.ride.viewer import RealTimePacer

def test_pacer_reports_unserved_wall_time():
    pacer = RealTimePacer(.00125, max_catchup_s=.05)
    steps = pacer.steps_for(.2)
    assert steps == 40
    assert abs(pacer.dropped_wall_s - .15) < 1e-12
```

For IO tests use an in-memory stream or temporary file and injected clock, checking flush count and final data preservation. Never change dt based on RTF or rendered FPS.

- [ ] **2. Implement the low-risk savings first.** Decouple render interval, retain all physics updates, buffer preview writes with reliable final flush, skip unchanged HUD formatting. Re-run unpaced benchmark and actual GUI. Logging savings alone must not be reported as a faster force solver.
- [ ] **3. Optimize the largest measured physical cost, one change at a time.** Candidates include repeated array allocation, identical geometry work, finite-sole solver fallback, road distance loops, distributed contact station evaluation. Reuse buffers or vectorize without changing station selection, material law, contact signs or force application points. Remove a MuJoCo forward pass only with an explicit dependency check proving all consumers still see the correct current derived state.
- [ ] **4. Verify each optimization before the next.** Snapshot pre/post command and state trajectories from the same initial state. For algebra-preserving operations require existing replay-level state tolerance `1e-9` where feasible; any larger ordering difference requires the D2 physical convergence criteria and is recorded as a numerical change. Changing low-level controller rate or tire approximation is a model change requiring D2 again, not a free optimization.
- [ ] **5. Repeat source-matched physical gates after the last code change.** Do not reuse pre-optimization source hashes as final certification. `uv run --no-sync pytest -q tests/test_preview_io.py tests/test_ride_cli.py tests/test_ride_track.py tests/test_research_viewer.py tests/test_research_replay.py tests/test_coasting_contact_recovery.py tests/test_system_momentum.py`; then rerun affected D2 sweeps. Avoid a broad suite after every tiny optimization; run it once at final integration.

Deliverable: measured speedup with retained numerical/physical quality, transparent lag reporting and unchanged original defaults.

### Task D4: 30T comparison, extreme outcome and end-to-end acceptance

**Files:** Extend `src/bike_sim/validation/climbing_campaign.py`, `tests/test_climbing_campaign.py`, `tests/test_long_episode.py`, `tests/test_geometric_transmission.py`; create final profile only after acceptance. Update the documentation/ledger listed in the file map.

**Interfaces:**
- `run_climbing_campaign(physics_profile, *, output_dir, duration_s=90.) -> dict` creates paired 34T/30T runs using `build_sim(physics_profile, track_path, physics_overrides={'drive': {'gearing': {'front_teeth': teeth}}})` and shared track/environment. Save each resolved TOML/config identity, equilibrium and all validity/performance outcomes.
- `compare_gearing(records: list[dict]) -> dict` retains failed/incomplete runs and reports progress, climb sections reached, useful human/motor work, cadence, shifts, support loss, front unload and rear slip. No reward/winner from absence of wheelie alone.
- Campaign module CLI flags: `--physics-config`, `--duration`, `--output`; command runs predeclared control cases and both authored uphill tracks for each front ring, plus manifests referencing D1–D3 evidence. It refuses stale source/config evidence and output overwrite.

- [ ] **1. Verify ratio and geometry using existing physical fixtures.** Extend the current shaft-ratio/freehub fixture to both `30/51` and `34/51`. Inspect gear radius/tendon geometry generation and shifting re-anchoring: changing teeth must not leave the old chainring radius or stale wheel-required cadence. Check power conservation, not only a property returning the ratio.

```python
import pytest
from bike_sim.validation.plant_torque_rig import shaft_ratio_rig

@pytest.mark.parametrize('front_teeth', [30, 34])
def test_candidate_lowest_gear_obeys_physical_shaft_ratio(front_teeth):
    metrics, bounds = shaft_ratio_rig(.000125, ratio=front_teeth/51)
    assert bounds
    for name, (lower, upper) in bounds.items():
        assert lower <= metrics[name] <= upper
```

- [ ] **2. Fix the long-episode acceptance contract.** Existing `tests/test_long_episode.py` must require actual 15 s articulated flat completion, ≥5 m progress and positive rider work, with no crash/model/energy failure. An early fall cannot count as a full-duration pass. Use the candidate and an explicit flat 100 m track; finite rider effort and configured assistance, no velocity servo. Slow physical tests are marked slow but must run for final acceptance.
- [ ] **3. Run the comparison without terrain tuning.** Same motor85/600, cassette, mass, human intention, starting rest, seeds and horizon; only front ring differs. Do not impose the exact same MuJoCo qpos vector across changed tendon geometry. Use each consistently settled initial condition and record that distinction from dt convergence.

```bash
uv run --no-sync -m bike_sim.validation.climbing_campaign --physics-config examples/research/seated_climb_candidate.toml --duration 90 --output verification/seated-climb-20261001/campaign
```

For a genuine stall, inspect a preceding valid interval with requested vs actual effort, motor stall timeout, wheel load/friction/slip, grade resistance and brake work. At35% grade the seating and reach limits may matter; demonstrate the limitation with bounded lean comparison, not a forced body pose. A profile that loses pedals on flat ground cannot claim a successful physical extreme stall.
- [ ] **4. Freeze chosen profile and run full research path.** Preserve existing `ResearchEnvironment`, policy factory, sensor delay/noise/reset and replay. Measure full `research` benchmark mode, including default `.005 s` sensor acquisition, `.01 s` policy period and `.005 s` motor transport delay at compatible dt. Use existing passthrough/zero/fixed-limit demos, not a new anti-wheelie law. Complete earlier B4 pairing/release work with the same rider rules and policy-independent experimental inputs; reactive rider trajectories may differ between policies and must be recorded rather than mislabelled as identical posture histories.
- [ ] **5. Perform actual GUI acceptance on the final source.** Launch on macOS through the project's `mjpython` handling; measure wall/sim progress and rendered-frame intervals on the full scenario. Exercise pause/reset/brake/close and replay a research recording. Record initialization separately, three warmed throughput repeats, frame p50/p95/p99, maximum lag and total capped wall debt. Required unpaced throughput ≥1×, target margin≥1.2×; paced GUI should track1× with the documented short-delay distribution and target30Hz. Averages must not conceal a sustained slowdown on roots/steps.
- [ ] **6. Publish the qualified opt-in profile and evidence.** Only when physical and timing gates pass create `examples/research/seated_climb_realtime.toml` from the tested candidate. Retain actual selected30T or34T, dt/road/contact resolution, human limits, auto shifting, and synthetic status. Do not create or label an `antiwheelie_reference` release while earlier B4 gates remain incomplete. If a necessary criterion fails, retain candidate status and report the exact remaining issue, not an unexplained «physics limit».

The final ordinary and research launch templates, executable only after the new flags/profile exist:

```bash
uv run bike-ride --physics-config examples/research/seated_climb_realtime.toml --rider articulated_planar --track examples/research/rough_uphill_extreme.toml --render-fps 30 --preview-flush-period 0.25
uv run bike-ride --research --physics-config examples/research/seated_climb_realtime.toml --rider articulated_planar --track examples/research/rough_uphill_extreme.toml --render-fps 30 --duration 90 --out output/seated-climb-research
```

- [ ] **7. Validate final integration and documentation.** Run `uv run --no-sync pytest -q` once, wait for final exit, preserve unrelated baseline failures rather than repairing them speculatively. Verify both documented commands and replay under the final source. Update new ledger with a requirement→artifact table; annotate remaining older B2–B4 tasks only where evidence actually closes them. No commits or default switch.

## Completion evidence

- [ ] Proven repaired feet/coast/resume and seated human behaviour from plan03.
- [ ] Accepted tire/drive/contact resolution with current-source numerical gates.
- [ ] 34T/30T physical comparison; selected ring justified with more than finish distance.
- [ ] Extreme completes or ends in a physically evidenced limit, with no unresolved pedal defect hidden in the label.
- [ ] Realtime measured for physics, GUI and full research-loop separately on the user's computer.
- [ ] All recordings have source/config/terrain identity, partial failure preservation and exact replay where applicable.
- [ ] Policy interface and earlier paired/release obligations retained; synthetic status explicit.
- [ ] New profile/commands/documentation are verified; existing default profile remains untouched.

## Decisions requiring the user, if actually encountered

Do not ask about routine timestep selection, buffer reuse, force diagnostics or known-approved30T. Ask only if measurements show the agreed scope cannot meet both accuracy and realtime after bounded optimization, if passing requires a different motor/cassette/rider strength, or if the user wants a default-profile switch. Explain the measured cost/physical limitation and concrete alternatives in ordinary language. Do not silently choose standing, clipped pedals, more motor power or a simpler invalid contact model.
