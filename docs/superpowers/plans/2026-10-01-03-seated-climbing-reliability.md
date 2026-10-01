# Seated Climbing Reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. The user selected sequential inline execution; do not reopen the execution-mode question.

**Goal:** Получить устойчивое физическое педалирование сидящего райдера, восстановление опоры после наката и воспроизводимый разбор подъёма extreme.

**Architecture:** Сначала причинная диагностика текущего physical runtime и минимальный regression fix. Затем отдельный opt-in уровень намерений сидящего человека поверх существующих суставных сил и контактов; один физический clock для viewer/headless/research. Производительность и окончательный профиль принимает следующий план 04.

**Tech Stack:** Python через uv, MuJoCo, NumPy, pytest, TOML, существующие physical samples/replay/validation.

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

## Исходная точка и структура

План подготовлен по `2e93585`; tracked tree был чистым. Проверенные по исходнику исправления coasting scratch-state и finite sole уже есть. Старые красные XML и короткий preview — материал для гипотез, не current-source baseline.

| Файл / группа | Ответственность |
|---|---|
| `validation/rider_replay.py` | Точный automatic baseline, полные интервалы и сохранение первой ошибки |
| `validation/climbing_evidence.py` (новый) | Чистая оценка потери опоры, восстановления работы и первого недостоверного интервала |
| `sim/ride/rider_control.py`, `support_geometry.py`, `rider_contacts.py` | Только доказанный локальный дефект целей/сил/контакта |
| `physics/pedaling.py`, `physics/shifting.py` | Только доказанный дефект переходов намерения coast/pedal/shift |
| `physics/seated_climb.py` (новый) | Immutable конфигурация, чистая модель намерений, конечные скорость реакции и усилие |
| `sim/ride/rider_intent.py` (новый) | Адаптер физических сигналов, clock/reset/probe/override |
| `sim/ride/physical_runtime.py` | Единственная точка применения намерений перед pedaling/forces |
| `physics/model_config.py`, `physics/resolution.py` | Опциональная TOML-секция `[seated_climb]`, round-trip и validation |
| `examples/research/seated_climb_candidate.toml` (новый) | Явный synthetic кандидат; статус unqualified до плана 04 |

Все пути в таблице относительны `src/bike_sim/`, кроме `examples/`. Новые модули не должны превращаться в второй simulation loop.

Команды выполнять из корня checkout. Для текущего окружения:

```bash
export UV_CACHE_DIR=/private/tmp/uv-cache-antiwheelie-plan
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
git status --short
git rev-parse HEAD
```

### Task C1: Свежая воспроизводимая причина потери педалей

**Files:** Modify `src/bike_sim/validation/rider_replay.py`; create `src/bike_sim/validation/climbing_evidence.py`, `tests/test_climbing_evidence.py`; extend `tests/test_rider_replay.py`; record in `verification/seated-climb-20261001/README.md` (new).

**Interfaces:**
- Consumes existing `build_sim(physics_path, track_path, timestep_s=None, *, physics_overrides=None)`, `automatic_schedule(time_s)`, `diagnostic_row(sample)`.
- Add `support_evidence(rows: list[dict], *, required_duration_s: float | None = None) -> dict`: `first_both_unloaded_s`, `max_both_unloaded_s`, `max_front_gap_m`, `max_rear_gap_m`, `first_invalid_s`, `invalid_reason`, `interval_count`, `complete`.
- Extend existing `replay` with optional keyword `row_sink=None`; call `row_sink(row)` immediately after obtaining each immutable diagnostic row. Existing callers continue receiving a list. Exceptions preserve accumulated rows in `run_replay`.
- Evidence availability is footprint AND load >1 N; persistence 0.20 s is diagnostic, never a force. `complete` requires intervals covering `[0, required_duration_s]` with no gaps; with no declared horizon it is false. Numerical/model validity is a separate result, never inferred from completeness.

- [ ] **1. Baseline before edits.** Run existing regressions and the exact automatic input at three dt values. Each physics run may take more than five minutes; poll to exit, do not abandon it on a turn boundary.

```bash
uv run --no-sync pytest -q tests/test_physical_pedaling_regression.py tests/test_pedal_contact_stability.py tests/test_finite_pedal_contact.py tests/test_physical_coasting.py
uv run --no-sync -m bike_sim.validation.rider_replay --physics-config examples/research/viewer_physics_fast.toml --track examples/research/rough_uphill_extreme.toml --mode automatic --duration 8 --timestep 0.00125 --output verification/seated-climb-20261001/baseline-dt1250.json.gz
uv run --no-sync -m bike_sim.validation.rider_replay --physics-config examples/research/viewer_physics_fast.toml --track examples/research/rough_uphill_extreme.toml --mode automatic --duration 8 --timestep 0.000625 --output verification/seated-climb-20261001/baseline-dt625.json.gz
uv run --no-sync -m bike_sim.validation.rider_replay --physics-config examples/research/viewer_physics_fast.toml --track examples/research/rough_uphill_extreme.toml --mode automatic --duration 8 --timestep 0.0003125 --output verification/seated-climb-20261001/baseline-dt312.json.gz
```

CLI exit 0 currently only means no exception; inspect interval energy and model validity. Independently settled dt runs diagnose sensitivity but do not constitute strict common-initial-state convergence. Use the existing `PhysicalInitialState`/fidelity runner for the latter in plan 04.

- [ ] **2. Add meaningful failing report tests.** Test irregular interval lengths, unloaded but in-footprint pads, late numerical invalidity, and partial runs. Example input and expectations:

```python
from bike_sim.validation.climbing_evidence import support_evidence

def test_geometric_overlap_does_not_count_as_loaded_support():
    rows = [{
        'time_s': 0., 'end_time_s': .25,
        'rider': {side + '_pedal': {
            'in_platform': True, 'normal_load_n': 0., 'gap_m': .03,
        } for side in ('front', 'rear')},
        'model_status': {'model_valid': True, 'numerically_valid': True},
    }]
    result = support_evidence(rows)
    assert result['first_both_unloaded_s'] == 0.
    assert result['max_both_unloaded_s'] == .25
    assert result['max_front_gap_m'] == .03
```

For numerical evidence require actual `energy_quality` on real rows. Missing energy is unknown, never a pass; the minimal fixture above tests only support metrics. Add exception preservation with a small fake sim that raises on the second step and retains the first row through `row_sink`.

- [ ] **3. Implement the evaluator and streaming preservation.** Accumulate durations from `end_time_s - time_s`, validate finite ordered intervals, retain every invalidity and the first failure. Write the report in `finally` after setting error/partial status. Do not silently filter invalid rows from support statistics; provide valid-prefix statistics separately.

```python
saved_rows = []
try:
    replay(sim, schedule, duration_s, row_sink=saved_rows.append)
finally:
    report['rows'] = saved_rows
    report['support_evidence'] = support_evidence(
        saved_rows, required_duration_s=duration_s)
```

- [ ] **4. Locate the first cause before choosing a fix.** Build a time-aligned window preceding first sustained loss: actual vs requested crank/sole state, pedal orientation, finite-foot footprint, tangential slip, leg/arm saturation, stance, measured torque, assist, gear relief and rollback brakes. Check whether loss precedes assist reduction. Separate planned 3 mm return clearance from centimetre gaps and underside contact. Save the hypothesis, counterexample and predicted distinguishing experiment in the ledger.
- [ ] **5. Verify reports.** `uv run --no-sync pytest -q tests/test_climbing_evidence.py tests/test_rider_replay.py`; retain failing physical outcomes. Deliverable: a source-identified current reproduction, not yet a repaired rider.

### Task C2: Fix the demonstrated contact/control failure

**Files:** Modify only implicated files among `src/bike_sim/sim/ride/rider_control.py`, `support_geometry.py`, `rider_contacts.py`, `src/bike_sim/physics/pedaling.py`, `shifting.py`; extend `tests/test_coasting_contact_recovery.py`, `tests/test_pedal_contact_stability.py`, `tests/test_physical_coasting.py`, `tests/test_physical_shifting.py`.

**Interfaces:** Preserve `RideControl`, `RiderCommand`, physical contact reaction and freehub contracts. `advance=False` probes must be non-mutating. C1 reports remain comparable across changes.

- [ ] **1. Write a failing test for the cause recorded by C1.** Do not presume the old frozen-target defect is still present. Candidate branches: unstable finite-pedal integration; discontinuous stance/return target; inability to brake reversed cranks through bounded legs; raw wheel cadence wrongly sustaining coast; infeasible posture. Each branch needs a prior failing mechanical or deterministic state-transition trace. Add the current first-failure input to its existing test family.
- [ ] **2. Run that test red, then change one mechanism.** Preserve platform friction and unilateral normals. If instability is limited to coarse dt, first select the smallest stable candidate dt; do not promise 1.25 ms or change contact stiffness to manufacture stability. Any later integration change must reproduce passive energy decay and load transfer. If smoothing coast inputs is necessary, test wheelspin/impact spikes and sustained true overspeed separately; do not suppress all legitimate coasting.
- [ ] **3. Use existing integration tests as behavioural acceptance.** Extend the recovery test to both existing resume tracks and 10 s:

```python
import pytest
from bike_sim.validation.rider_replay import run_replay

@pytest.mark.slow
@pytest.mark.parametrize('track', ['rider_resume_flat', 'rider_resume_incline'])
def test_resume_has_real_work_after_a_full_coast(track, tmp_path):
    report = run_replay(
        'examples/research/viewer_physics_fast.toml',
        f'examples/research/{track}.toml', mode='human-only',
        timestep_s=.000625, duration_s=10., output=tmp_path/'resume.json.gz')
    assert 'error' not in report
    assert report['completed_requested_duration']
    assert report['evidence']['window_s'] == [4., 10.]
    assert report['evidence']['diagnosis'] == 'resumed'
    assert report['evidence']['crank_rotation_rad'] >= 2. * 3.141592653589793
    assert report['evidence']['positive_crank_work_j'] > 0.
```

The demonstrated stable dt may be finer; record any selection change in the spec ledger with convergence evidence. Also assert each real row's energy gate and model validity; a stale `model_status.numerically_valid` boolean is insufficient.

- [ ] **4. Verify passive and internal-force invariants.**

```bash
uv run --no-sync pytest -q tests/test_pedal_contact_stability.py tests/test_finite_pedal_contact.py tests/test_rider_contact_laws.py tests/test_system_momentum.py tests/test_physical_coasting.py tests/test_physical_shifting.py tests/test_coasting_contact_recovery.py
```

- [ ] **5. Repeat the exact automatic baseline with a new output name.** Demonstrate repaired initial coast and useful resumed pedaling; preserve original coarse-dt rejection if it remains invalid. Deliverable: a causal regression fix with a stable working rider, before adding new behaviour.

### Task C3: Bounded seated rider intention, shared by all runtimes

**Files:** Create `src/bike_sim/physics/seated_climb.py`, `src/bike_sim/sim/ride/rider_intent.py`, `tests/test_seated_climb.py`, `tests/test_rider_intent.py`, `examples/research/seated_climb_candidate.toml`; modify `src/bike_sim/physics/model_config.py`, `resolution.py`, `src/bike_sim/sim/ride/physical_runtime.py`, `physical_observations.py`; extend `tests/test_research_configuration.py`, `test_rider_posture_tracking.py`, `test_research_replay.py`.

**Interfaces:**
- `SeatedClimbConfig`: immutable dataclass in `physics/seated_climb.py`; fields `enabled=False`, `period_s=.01`, `reaction_delay_s=.15`, `target_crank_power_w=225.`, `max_crank_torque_nm=60.`, `torque_slew_nm_s=300.`, `lean_gain=1.`, `max_forward_lean_rad=.35`, `max_backward_lean_rad=.10`, `lean_rate_rad_s=.5`, `orientation_tau_s=.5`. All finite with positive periods/power/limits; gains may be zero, delay nonnegative.
- Add `SimulationPhysicsConfig.seated_climb` with default factory and strict `CHILDREN` resolver entry. Existing profiles default disabled.
- `SeatedClimbSignals`: immutable `pitch_rate_up_rad_s`, `specific_force_body_mps2`, `crank_rate_rad_s`, `human_crank_torque_nm`; finite sensory-like inputs, no track or wheel-load truth.
- `SeatedClimbIntent`: immutable `posture: RiderPosture`, `effort_ceiling_nm: float`; never a motor command.
- `SeatedClimbPolicy(config).reset()`, `.update(signals, dt_s) -> SeatedClimbIntent`: pure bounded policy state, called on its own period with delayed signals.
- `RiderIntentResolver` in runtime adapter: `reset()`, `resolve(control, *, active, advance) -> RideControl`; at most one update per physics interval, explicit posture/effort win. Idle/equilibrium/probes cannot consume reaction time or mutate state.

- [ ] **1. Add red tests for configuration and pure policy.** A representative test specifies seat-only operation and bounded response:

```python
from math import cos, sin
from bike_sim.physics.seated_climb import SeatedClimbConfig, SeatedClimbPolicy, SeatedClimbSignals

def test_sustained_uphill_estimate_only_changes_seated_torso():
    config = SeatedClimbConfig(enabled=True)
    policy = SeatedClimbPolicy(config)
    signals = SeatedClimbSignals(
        pitch_rate_up_rad_s=0.,
        specific_force_body_mps2=(9.81*sin(.2), 0., 9.81*cos(.2)),
        crank_rate_rad_s=8., human_crank_torque_nm=20.)
    previous = 0.
    for index in range(400):
        intent = policy.update(signals, .01)
        assert intent.posture.use_saddle
        assert intent.posture.pelvis_offset_m is None
        assert intent.posture.pelvis_pitch_rad == 0.
        assert abs(intent.posture.torso_lean_rad - previous) <= .005 + 1e-12
        assert 0. <= intent.effort_ceiling_nm <= 60.
        previous = intent.posture.torso_lean_rad
    assert 0. < previous <= .35
```

Verify engine/body axis sign on a stationary inclined fixture before using the synthetic accelerometer tuple in this test; the test and adapter must describe the same forward/up convention. Add tests for reverse/zero crank speed, transient 2g impact, repeated reset, empty delay queue, finite input validation and explicit disable.

- [ ] **2. Implement finite intent, not direct force.** Use gyro integration plus bounded accelerometer correction to estimate inclination; reject accelerometer correction when norm is outside `[0.8g, 1.2g]`, and document acceleration ambiguity inside that band. Delay the input history by `reaction_delay_s`; neutral intent until a sample is available. Apply posture angle/rate limits. Start with constant 225 W demand and a 60 N·m ceiling, no sprint reservoir or fatigue subsystem.

```python
from math import pi

def crank_effort_ceiling(power_w, torque_limit_nm, crank_rate_rad_s):
    speed_floor = 20. * 2. * pi / 60.
    forward_rate = max(0., crank_rate_rad_s)
    return min(torque_limit_nm, power_w / max(forward_rate, speed_floor))
```

Apply the resulting ceiling **after** the existing low-cadence mash calculation, before leg effort distribution. Otherwise `mash_torque_nm` would silently override the human power intention. During intentional coast the human effort remains zero. The 20 rpm floor is a finite torque regularization, not a minimum prescribed crank speed. Record requested and delivered shaft power separately; never clip a measured contact reaction to satisfy a target.

- [ ] **3. Integrate one physical clock.** Read `sensor_channels`/current non-advancing initial probe using the same causal state for preview and accounted mode. Hold high-level intention between 10 ms ticks; low-level muscle/contact control still runs every physics tick. Validate `period_s / dt` is integral; model the 150 ms reaction delay with the high-level clock. Store an interval id to prevent double advancement during multiple force evaluations. Every override is fieldwise: explicit effort does not disable automatic posture, explicit posture does not change the motor request. New candidate uses both defaults; research program/behaviour provide explicit override values.

```python
from dataclasses import replace

def merge_intent(control, intent):
    return replace(control,
        posture=intent.posture if control.posture is None else control.posture,
        human_torque_nm=(intent.effort_ceiling_nm
                         if control.human_torque_nm is None else control.human_torque_nm))
```

Carry a separate diagnostic source/ceiling flag into pedaling preparation so explicit controlled experiments retain their declared effort and the automatic ceiling is applied after mash. The merge helper alone is not the full integration. Do not enqueue rider reaction behind motor actuator delay; the runtime owns its clock.

- [ ] **4. Verify physiology and physical posture separately.** Candidate includes finite existing joint envelope, 400 N grip-pair cap and 600 W total positive joint-power cap as explicitly synthetic safety ceilings, not measured human calibration. On a loaded flat/5% incline fixture, 30 s of settled pedaling should deliver 200–250 W mean positive shaft power when grip/leg geometry permits; record negative crank work, joint positive work and saturation alongside it. If a limit prevents this, diagnose reach/contact/coordination before changing caps. Startup and impact overshoot remain visible; repeated sustained >250 W fails this rider profile.
- [ ] **5. Replay/reset/probe checks.** Add tests that probe calls do not advance intent, reset reproduces it, different render groupings preserve the physical trajectory, and replay reconstructs the same high-level intent from the same initial state. Verify explicit `RiderProgram` / `RiderBehavior` precedence and preservation of motor-policy-only authority. Use `tests/test_research_replay.py`, `tests/test_research_viewer.py`, `tests/test_rider_behavior_hook.py` as existing coverage.
- [ ] **6. Run focused checks.**

```bash
uv run --no-sync pytest -q tests/test_seated_climb.py tests/test_rider_intent.py tests/test_research_configuration.py tests/test_rider_posture_tracking.py tests/test_rider_behavior_hook.py tests/test_research_replay.py tests/test_research_viewer.py tests/test_system_momentum.py
```

Deliverable: opt-in bounded seated behaviour with common runtime and no motor algorithm. Fine-dt correctness precedes realtime claims.

### Task C4: Isolate climbing limits and hand off a physics candidate

**Files:** Extend `src/bike_sim/validation/rider_factor_matrix.py`, `tests/test_rider_replay.py`; update new `verification/seated-climb-20261001/README.md` and candidate TOML. Read `src/bike_sim/validation/reference_release.py` classification, do not equate its label alone with causal proof.

**Interfaces:** Consume C1 evidence and C3 candidate; return a per-run record with `outcome`, `first_invalid_s`, `first_support_loss_s`, `progress_m`, `human_positive_work_j`, `motor_work_j`, `brake_work_j`, `rear_slip`, `gear_history`, `rider_intent_config` and profile/source hashes. `physical_stall` requires a valid supported interval with continuing demand and insufficient measured forward traction; sustained near-zero speed alone yields `unresolved_stall`.

- [ ] **1. Run bounded-factor experiments.** First automatic full candidate, then change only rollback enable, only automatic shifting enable, only seated lean enable; preserve effort/motor/track and horizon. Start with the first troublesome interval established by C1, then run the 100 m scenario with an explicit 90 s observation horizon. If 90 s expires while still moving, record timeout and calculate a justified extension; do not label a stall.
- [ ] **2. Check the causal sequence.** Track loaded feet → measured human torque → motor request/delivery → wheel traction → speed. Measure brake work during hold/restart. A 1 s assist stall cutoff is synthetic and must be reported; changing it requires evidence about the intended generic assistance behaviour, not just improved distance.
- [ ] **3. Preserve invalidity.** Single-contact `multi_support` or bad energy ends the accepted prefix. A continued diagnostic trajectory is still invalid. Handoff to plan 04 for the existing distributed tire instead of calling later stop physically explained.
- [ ] **4. Record a review checkpoint.** List the exact fixed cause, unresolved model restrictions, successful flat/incline/coast/resume results, current-source identity, and candidate performance not yet measured. Keep the older B2/B3/B4 tasks open where no new evidence closes them.

## Completion gate for plan 03

- [ ] Current-source support-loss regression is reproduced and causally repaired.
- [ ] Flat/incline scripted coast/resume complete their full horizon with useful human work and numerical validity.
- [ ] Seated-only lean and finite human effort work through physical forces; overrides/reset/replay are deterministic.
- [ ] Extreme progress/stop has first-failure evidence; any unresolved tire scope is explicitly handed to plan 04.
- [ ] No existing default profile, user file, terrain seed or motor rating was changed.

Next: `docs/superpowers/plans/2026-10-01-04-climbing-realtime-qualification.md`. Do not ask again whether to execute inline; that preference is already settled. Do not mark this document complete merely because it has been written.
