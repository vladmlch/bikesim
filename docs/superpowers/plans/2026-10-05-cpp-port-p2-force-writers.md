# C++ Port P2: Force-Writer Equivalence Framework + First Ports Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Построить per-state equivalence-харнесс для force writers (state из golden → `mj_forward` → writer → побитовое сравнение компонентов) и портировать первый блок писателей: suspension, brake, resistance, tire, rider_forces.

**Architecture:** P1 дал oracle+scaffold; P2 расширяет golden-артефакт per-step `mjData`-снимками и матрицей force-компонентов (`acc.add` — уже точка сбора), а native `Stepper` получает `set_state`/`forward`/writers API. Каждый ported writer проверяется на каждом записанном шаге побитово против хранимых компонентов — horizon-независимо (вход зафиксирован, а не реинтегрирован). Trajectory-level проверка принадлежит P4 (step loop); здесь — только layer-1 на writer-выходах.

**Tech Stack:** Python 3.13 + uv, MuJoCo 3.12 (та же dylib), nanobind, CMake + Apple clang 21 (C++23), GCC 16 вторым фронтендом, pytest, numpy.

**Spec:** `docs/adr/0001-native-port-mujoco-core.md` (планка, флаги, hardening); план-основание: `docs/superpowers/plans/2026-10-05-cpp-port-p1-correctness-infra.md` (уже исполнен); `.superpowers/sdd/.../progress.md` — forward-constraints (ep.initial, flatten).

## Global Constraints

- Python — всегда `uv run`; тесты `uv run python -m pytest` из корня репо.
- Нативный код — C++23, clang primary + GCC 16 sweep (`tools/check_native_frontends.sh`), флаги и SDK-pin как в P1 (`native/CMakeLists.txt` — не дублировать, расширять).
- Планка: per-state writer-эквивалентность **bitwise** (`np.array_equal`) где математика реализуема тем же порядком FP-операций; где numpy-трансценденталка не воспроизводима в libm — задокументировать per-component max-ulp (≤4 ulp принимаемо: бюджет уже «сгорел» горизонтом h≈26–40, измеренным в P1). Любое ослабление bitwise→ulp — с записью в тест-комментарии и отчёте.
- Суммирование компонентов воспроизводит порядок `acc.add` вызовов (порядок суммы влияет на FP) — матрица `forces` хранит имена в insertion-order, native `total()` обязан суммировать в том же порядке.
- Никаких изменений поведения `src/bike_sim/` — capture v2 инструментирует через обёртки/monkeypatch, не правки рантайма. Разрешены чистые расширения (новые файлы), не меняющие существующие пути.
- `apply_forces` вызывает `mj_forward` трижды между writer'ами (physical_runtime.py:243,300,318) — per-state тест НЕ воспроизводит промежуточные форварды: он проверяет writer на записанном *входном* состоянии. Контракт writer'а — «те же входы → те же выходы», входы берутся из артефакта.
- Canonical сценарий и env-фабрика — как в P1 (`_pinned_config`, `--diagnostic-model-limits`, savage для тестов, extreme для CLI).

## Writer Surface (фактура, проверена по коду)

Порядок вызовов в `apply_forces` (physical_runtime.py:236-330) = порядок `acc.add`:

| # | Writer | Компонент(ы) в acc | Входы | Stateful? | План |
|---|---|---|---|---|---|
| 0 | `brake.apply` → ctrl | не в acc (пишет `d.ctrl`) | wheel omegas, demands | нет | T4 |
| 1 | `applier.compute_qfrc_components` | `fork_spring/fork_damper/shock_coil/shock_bumper/shock_damper/shock_top_out/shock_upper_stop[/shock_hbo]` | qpos/qvel + config | нет | T3 |
| 2 | `tire.compute_qfrc` | `tires` | contacts post-forward, brush state | **да** (`_BrushState`) | T5 |
| 3 | `resistance.compute_components` | `road_rolling`, `aerodynamic` | tire snapshots + mj_jac | нет (вход — snapshots) | T4 |
| 4 | `rider_forces.apply` | `seated_interfaces` | qpos/qvel | проверить | T6 |
| 5 | `rider_contacts.compute_qfrc` | `rider_interfaces` | contacts + enabled/diagnostics | **да** | P3 |
| 6 | `drive.compute_components` | chain/drive components | `pedaling` state + sensed | **да** | P3 |
| 7 | `rider_control.*` | `rider_joint_envelope` + torque write | QP controller | **да** | P3 |
| 8 | `cruise` → ctrl | не в acc | contacts | да | P3 |

`prepare_pedaling` (drive), `rider_intent.resolve`, `rider_control.compute` — stateful-цепочка контроллера, P3. `external` компонент — при `external is not None` (в golden нет).

---

### Task 1: Golden capture v2 — per-step state + force components

Расширяет `tools/golden_episode.py`: помимо channel-строк, артефакт хранит per-physics-step снимки состояния и матрицу force-компонентов — входы/выходы для per-call проверки writer'ов. Имена компонентов — insertion-order `acc.add`.

**Files:**
- Modify: `tools/golden_episode.py` (расширение `capture_episode`/`save`/`load_episode`)
- Test: `tests/reference/test_golden_episode_v2.py`

**Interfaces:**
- Consumes: существующий `capture_episode`; `sim.force_accumulator` (обёртка `add`), `runtime.apply_forces` (обёртка для state-снимка на входе и `d.ctrl` на выходе), `sim.tire.states` (`_BrushState` полей xi/tangent/point/segment/center), `sim.tire.snapshots` (per-side: `patches[].normal_load_n/.working_surface`, `effective_radius_m` — сериализуемые маленькие dicts).
- Produces: в `episode.npz` дополнительно `state_qpos[K,nq]`, `state_qvel[K,nv]`, `state_act[K,na]`, `state_warmstart[K,nv]`, `state_time[K]`, `ctrl_written[K,nu]`, `force_names[C]` (строки, insertion-order), `forces[K,C,nv]`, `tire_state[K,S]` + `tire_state_names[S]`; в `manifest.json` — `tire_snapshots[K]` (JSON-able per-side структуры). `load_episode` возвращает их в `EpisodeArtifact` (новые поля с дефолтами для старых артефактов). `K` = число physics-step'ов (может отличаться от числа channel-строк — sample'ы выходят по period-close).

- [ ] **Step 1: Failing test**

```python
"""Capture v2 stores per-step states and the full force-component matrix."""
import numpy as np
import pytest

def _ep(tmp_path, steps=40):
    from bike_sim.cli import research as research_cli
    from test_pinned_topology import _pinned_config
    from tools.golden_episode import capture_episode
    from bike_sim.sim.ride.control import RideControl
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--diagnostic-model-limits',
        '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    return env, capture_episode(env, steps, RideControl(human_torque_nm=35.))

@pytest.mark.slow
def test_v2_state_and_forces_recorded(tmp_path):
    env, ep = _ep(tmp_path)
    nv = env.sim.model.nv
    assert ep.state_qpos.shape == (40, env.sim.model.nq)
    assert ep.state_qvel.shape == (40, nv)
    assert ep.state_warmstart.shape == (40, nv)
    assert ep.forces.shape == (40, len(ep.force_names), nv)
    # suspension components must be present under their acc names
    assert {'fork_spring', 'shock_coil', 'shock_damper'} <= set(ep.force_names)
    # insertion-order sum of components is finite (ordering is the contract)
    total = np.zeros(nv)
    for i in range(len(ep.force_names)):
        total += ep.forces[10][i]
    assert np.isfinite(total).all()

@pytest.mark.slow
def test_v2_roundtrip_and_determinism(tmp_path):
    env, ep = _ep(tmp_path)
    _, other = _ep(tmp_path)
    for f in ('state_qpos', 'state_qvel', 'forces', 'ctrl_written'):
        assert np.array_equal(getattr(ep, f), getattr(other, f))
    from tools.golden_episode import save, load_episode
    save(ep, tmp_path/'g', env.sim.model)
    loaded = load_episode(tmp_path/'g')
    for f in ('state_qpos', 'state_qvel', 'forces', 'ctrl_written'):
        assert np.array_equal(getattr(loaded, f), getattr(ep, f))
    assert loaded.force_names == ep.force_names
```

- [ ] **Step 2: Run — verify fail**

Run: `uv run python -m pytest tests/reference/test_golden_episode_v2.py -v -m slow`
Expected: FAIL — `AttributeError: 'EpisodeArtifact' object has no attribute 'state_qpos'` (или аналог — поля ещё не существуют).

- [ ] **Step 3: Implement capture v2**

В `capture_episode`: до цикла шагов — инструментация:

```python
# Wrap acc.add: record every named component in insertion order.
acc = env.sim.force_accumulator
orig_add = acc.add
step_components: list[dict[str, np.ndarray]] = []
def spied_add(name, qfrc):
    orig_add(name, qfrc)
    step_components.setdefault(-1, {})   # see ordering note below
    step_components[-1][name] = np.asarray(qfrc, float).copy()
acc.add = spied_add
# Wrap apply_forces: entry state snapshot + exit ctrl snapshot.
orig_apply = rt.apply_forces
def spied_apply(**kw):
    snap = (d.qpos.copy(), d.qvel.copy(), d.act.copy(),
            d.qacc_warmstart.copy(), float(d.time))
    out = orig_apply(**kw)
    return snap, out, d.ctrl.copy()
```

Точные формы — дело имплементора (главное — ordering: state на входе apply_forces, components по мере `acc.add` внутри него, ctrl на выходе). Tire: `sim.tire.states`/`snapshots` копируются после каждого шага (deepcopy маленьких dicts). Cleanup: `acc.add`/`rt.apply_forces` восстанавливаются в `finally`. `EpisodeArtifact` получает поля; `save`/`load_episode` — симметрично. Нестейтфул детали: имена `force_names` — union в порядке первого появления (компоненты могут появляться не с шага 0 — `shock_hbo` только в physical mode и т.п.).

- [ ] **Step 4: Run — verify pass + regression**

Run: `uv run python -m pytest tests/reference/test_golden_episode_v2.py tests/reference/test_golden_episode.py -v -m slow`
Expected: PASS; старые capture-тесты зелёные (совместимость).

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(tools): golden capture v2 — per-step state + force-component matrix (cpp-port P2)"
```

---

### Task 2: Native state-restore + forward bitwise gate

`Stepper` учится восстанавливать произвольное `mjData`-состояние и делать `mj_forward` — после чего решённые величины обязаны побитово совпасть с Python `mj_forward` на том же состоянии. Это gate самой machinery перед writer-портами.

**Files:**
- Modify: `native/src/stepper.hpp`, `native/src/stepper.cpp`, `native/src/binding.cpp`
- Test: `tests/reference/test_native_state_restore.py`

**Interfaces:**
- Consumes: golden `episode.npz` (`state_*` матрицы из Task 1 — тест генерирует артефакт capture'ом или грузит fixture).
- Produces: `Stepper.set_state(qpos, qvel, act, warmstart, time)`, `Stepper.forward()`, views `qacc`, `qfrc_constraint`, `efc_force` (read-only numpy views, как `qpos`). Контракт: `set_state` + `forward` на записанном состоянии даёт побитово то же, что `mj_resetData`+запись+`mj_forward` в Python.

- [ ] **Step 1: Failing test**

```python
"""Native forward on a stored golden state is bitwise-equal to Python forward."""
import sys
from pathlib import Path
import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'native' / 'build'))

def _golden(tmp_path, steps=8):
    from bike_sim.cli import research as research_cli
    from test_pinned_topology import _pinned_config
    from tools.golden_episode import capture_episode, save
    from bike_sim.sim.ride.control import RideControl
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--diagnostic-model-limits',
        '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    ep = capture_episode(env, steps, RideControl(human_torque_nm=35.))
    save(ep, tmp_path/'g', env.sim.model)
    return tmp_path/'g'

@pytest.mark.slow
def test_forward_on_stored_state_is_bitwise(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    from tools.golden_episode import load_episode
    ep = load_episode(_golden(tmp_path))
    ref = mujoco.MjModel.from_binary_path(str(tmp_path/'g'/'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(tmp_path/'g'/'model.mjb'))
    for k in range(len(ep.state_qpos)):
        mujoco.mj_resetData(ref, dref)
        dref.qpos[:] = ep.state_qpos[k]; dref.qvel[:] = ep.state_qvel[k]
        dref.act[:] = ep.state_act[k]
        dref.qacc_warmstart[:] = ep.state_warmstart[k]
        dref.time = float(ep.state_time[k])
        mujoco.mj_forward(ref, dref)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        assert np.array_equal(st.qacc, dref.qacc)
        assert np.array_equal(st.qfrc_constraint, dref.qfrc_constraint)
        assert np.array_equal(st.efc_force, dref.efc_force)
```

- [ ] **Step 2: Run — verify fail**

Run: `uv run python -m pytest tests/reference/test_native_state_restore.py -v -m slow`
Expected: FAIL — `AttributeError`/`TypeError` на `set_state`/`forward`.

- [ ] **Step 3: Implement `set_state`/`forward` + views**

```cpp
// stepper.hpp additions:
void set_state(std::span<const double> qpos, std::span<const double> qvel,
               std::span<const double> act, std::span<const double> warmstart,
               double time);
void forward() { mj_forward(m_, d_); }
[[nodiscard]] std::span<const double> qacc() const;
[[nodiscard]] std::span<const double> qfrc_constraint() const;
[[nodiscard]] std::span<const double> efc_force() const;   // size m_->nefc
```

`set_state`: `mj_resetData`, затем `std::copy` каждого span в соответствующий буфер с проверкой размеров (`qpos.size()==m_->nq` и т.д.; mismatch → `std::invalid_argument`). binding: принимать `nb::ndarray<const double, nb::shape<-1>>` и конвертировать в `span` (size/shape check на границе, не в ядре). Views — как `qpos` (empty owner, non-owning).

- [ ] **Step 4: Build + run**

```bash
cd native && uv run cmake --build build && cd ..
uv run python -m pytest tests/reference/test_native_state_restore.py -v -m slow
bash tools/check_native_frontends.sh native/src .venv/lib/python3.14/site-packages/mujoco/include
```

Expected: bitwise PASS; sweeps clean. Если `efc_force` не совпадает побитово — первое расследование: warmstart-путь (`mj_forward` использует `qacc_warmstart` для солвера); проверить что `mj_resetData`+записи воспроизводят состояние capture-точки (state снимается на ВХОДЕ apply_forces — до первого mj_forward шага).

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(native): Stepper set_state/forward + solved-quantity views — bitwise restore gate (cpp-port P2)"
```

---

### Task 3: Config bridge + suspension writer port

Первый ported writer — suspension (stateless, 7–8 компонентов). Заодно появляется config-bridge: nanobind `dict` → typed C++ config — паттерн для всех последующих writer'ов.

**Files:**
- Create: `native/src/config.hpp` (читатель `nb::dict` → struct'ов)
- Create: `native/src/writers/suspension.hpp`, `native/src/writers/suspension.cpp`
- Create: `tools/native_config.py` (проектор: Python-configs → dict для native)
- Modify: `native/src/binding.cpp`, `native/CMakeLists.txt`
- Test: `tests/reference/test_native_suspension.py`

**Interfaces:**
- Consumes: `SuspensionForceApplier.compute_qfrc_components` (forces.py:123-222) — источник истины семантики; `CoilShock`/`CoilShockSpecs` (physics/coil_shock.py), `SuspensionController`/`BikeSuspensionSystem`/`Charger3Damper`/`SuperDeluxeDamper` (sim/controllers.py, physics/damper.py:82-395), `end_stop` (physics/stops.py), `SimulationPhysicsConfig.end_stops/physics_mode`.
- Produces: `Stepper(path, config: dict)` — ctor принимает dict от `tools.native_config.project(env)`; `stepper.suspension_components() -> dict[str, np.ndarray]` — имена как в `force_names` (`fork_spring` … `shock_hbo`), каждый — nv-вектор (копия или non-owning view; dtype float64).
- Config dict-ключи (Task фиксирует финальный список после чтения damper.py): coil (`rate_n_m,preload_mm,stroke_mm,bumper_length_mm,bumper_peak_n`), fork air spring + Charger3 clicks/params, SuperDeluxe clicks/HBO, `end_stops{stiffness_n_m,damping_n_s_m}`, `physics_mode`, joint names (`fork_travel`,`shock_stroke` — адреса резолвятся в C++ через `mj_name2id` с той же range-валидацией, что `_resolve_compression_joint`).

- [ ] **Step 1: Failing test**

```python
"""Native suspension components are bitwise-equal to recorded acc components."""
@pytest.mark.slow
def test_suspension_components_bitwise(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    # load golden; build config via tools.native_config.project(env) — env нужен
    # только для проекции конфига (те же поля, что capture-оружие).
    st = bike_native.Stepper(str(mjb), native_config.project(env))
    sus_names = [n for n in ep.force_names if n.startswith(('fork_', 'shock_'))]
    for k in range(len(ep.state_qpos)):
        st.set_state(*states_k); st.forward()
        comp = st.suspension_components()
        for i, name in enumerate(ep.force_names):
            if name in sus_names:
                assert np.array_equal(comp[name], ep.forces[k][i]), f'{name} step {k}'
```

- [ ] **Step 2: Run — verify fail** → `TypeError: Stepper() takes 1 argument` / `AttributeError`.

- [ ] **Step 3: Implement**

- `tools/native_config.py::project(env) -> dict` — вытаскивает ТОЛЬКО используемые поля (dataclass → dict через публичные attrs; без pickle). Versioning: ключ `schema: 1`.
- `native/src/config.hpp` — `struct SuspensionConfig` + `from_dict(const nb::dict&)`; обязательные ключи отсутствуют → `std::invalid_argument` с именем ключа.
- `suspension.cpp` — порядок FP-операций = forces.py:127-202 (сложение компонентов в том же порядке; `vector()`→ запись `-force` в dofadr). Damper-математика — expression-for-expression порт `compute_damping_force`/`compute_damping_components` (damper.py), `end_stop` (stops.py), `compute_spring_force`/`compute_bumper_force` (coil_shock.py). Трансценденталки если есть → см. Global Constraints (bitwise приоритет, ulp-бюджет документируется).
- Stepper ctor: `Stepper(path)` остаётся (config = empty dict → suspension disabled, метод кинет `std::logic_error`); `Stepper(path, dict)` — с config.

- [ ] **Step 4: Run** — build, test PASS, sweep clean.

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(native): suspension writer port + config bridge — bitwise per-state equivalence (cpp-port P2)"
```

---

### Task 4: Brake + resistance ports

Два маленьких stateless writer'а. Brake пишет `d.ctrl` (не acc) — проверяется через `ctrl_written` матрицу. Resistance требует tire-snapshots — берутся из манифеста (записаны в Task 1).

**Files:**
- Create: `native/src/writers/brake.{hpp,cpp}`, `native/src/writers/resistance.{hpp,cpp}`
- Modify: `config.hpp` (секции brake/resistance), `native_config.py`, `binding.cpp`, `CMakeLists.txt`
- Test: `tests/reference/test_native_brake_resistance.py`

**Interfaces:**
- Consumes: `BrakeController.compute`/`_wheel_torque`/`opposing_torque` (braking.py:67-105 — clamp demand, ceiling, taper); `PhysicalResistance.compute_components` (physical_resistance.py:20-42 — `mj_jac` по wheel axis, rolling-moment, drag-force; snapshots из manifest'а).
- Produces: `stepper.brake_torques(front: float, rear: float) -> tuple[float, float]` + `stepper.apply_brake(front, rear)` (пишет в ctrl — verify через `ctrl_written[k]`); `stepper.resistance_components(snapshots_k) -> dict[str, np.ndarray]` ('road_rolling','aerodynamic'). Snapshot dict реконструируется из manifest на Python-стороне и передаётся как nb::dict/pybstruct — наиболее простой контракт: плоские массивы (`patch_loads_front`, `eff_radius_front`, …) — имплементор фиксирует схему.

- [ ] **Step 1: Failing test** — per-state: `brake_torques(f,r)` vs записанными `front/rear` demands (controls в артефакте) и `ctrl_written`; `resistance_components` vs `forces[k][road_rolling|aerodynamic]` bitwise.
- [ ] **Step 2-4:** TDD-цикл, build+test+sweep, commit `"feat(native): brake + resistance writers (cpp-port P2)"`.

---

### Task 5: Tire writer port (stateful)

Самый тяжёлый writer P2: brush-state эволюционирует между вызовами. Per-call проверка: `set_state` + восстановление brush-state из `tire_state[k]` → `compute_qfrc` → compare `forces[k]['tires']`. Источники: `tire_forces.py` + `physics/tire.py` (`_brush_step`, `_normal_contact`, `TireSpec`) + `tyre/` (brush/carcass/geometry) + `ProfileQuery`. Road-профиль — из `model.hfield_*` (как `compiled_profile_vertices` — та же копипаста геометрии, ноль re-распила).

**Files:**
- Create: `native/src/writers/tire.{hpp,cpp}` (+ `native/src/tyre/` при необходимости приватных helper'ов)
- Modify: `config.hpp`, `native_config.py` (TireSpec + surface_map сериализация), `binding.cpp`, `CMakeLists.txt`
- Test: `tests/reference/test_native_tire.py`

**Interfaces:**
- Produces: `stepper.tire_state()` / `stepper.set_tire_state(arr)` (roundtrip с `tire_state` матрицей артефакта); `stepper.tire_qfrc(dt: float) -> np.ndarray` — nv-вектор, bitwise vs `forces[k]['tires']` на каждом шаге; snapshots native-аналог для downstream (resistance) — совместимая структура.

- [ ] **Step 1-4:** TDD: failing test (per-state bitwise `tire_qfrc`), implement, build+test+sweep, commit `"feat(native): tire writer port — brush-state restore + bitwise components (cpp-port P2)"`.
- Замечание: brush-интеграция использует `last_time_s`-проверки (`tire_forces.py:130`) — native-порт сохраняет ту же дисциплину (double-advance на том же timestamp = error).

---

### Task 6: rider_forces port + accumulator-order gate

`rider_forces.apply` пишет прямо в `qfrc_applied`, а acc получает копию как `seated_interfaces`. Порт должен вернуть тот же nv-вектор. Финальный gate P2: сумма в insertion-order записанных компонентов == записанный `qfrc_applied`-итог (добавить `qfrc_total[K,nv]` в capture v2 при необходимости — или проверять что sum(forces[k]) совпадает с известным итогом).

**Files:**
- Create: `native/src/writers/rider_forces.{hpp,cpp}`
- Modify: config/binding/CMake; возможно `golden_episode.py` (`qfrc_total` запись — если ещё не покрыто каналами)
- Test: `tests/reference/test_native_rider_forces.py` + accumulator-order тест (внутри него)

**Interfaces:**
- Consumes: `rider_forces.py` (apply:143, compute:50 — rider tissue/interface springs).
- Produces: `stepper.rider_forces_qfrc() -> np.ndarray` bitwise vs `forces[k]['seated_interfaces']`; тест `total()`: `np.add.reduce` в порядке `force_names` == native `total()` порядок.

- [ ] **Step 1-4:** TDD: failing test, implement, build+test+sweep, commit `"feat(native): rider_forces writer + accumulator ordering gate (cpp-port P2)"`.

---

### Deferred (явно вне P2)

- `rider_contacts` (682 LOC, enabled/diagnostics state, QP-куплеты), `rider_control` (ProxQP), `drive` (prepare_pedaling chain + sensed-torque feedback), `cruise` → **P3** (controller+stateful writers plan).
- Step-loop orchestration (native владеет порядком apply_forces + mj_step), intent-seam, recorder/telemetry → **P4**.
- `ep.initial` применение в native replay — уже зафиксировано в коде (`episode_compare.py` docstring).
- 4-й leaf-walk consumer → при появлении вынести `tools/_flatten.py`.

## Self-review выполнен

- Spec coverage: layer-1 (per-call bitwise) на writer-выходах — Tasks 2-6; входная machinery (capture v2 + set_state/forward) — Tasks 1-2; config bridge — Task 3. Layer-2 trajectory-проверка осознанно НЕ здесь (h≈26-40 уже измерен, ей место в P4 step-loop). Stateful writers требующие controller-контекста → P3 явно.
- Placeholder scan: тест-скетчи Task 4/5/6 описаны прозой (интерфейсы зафиксированы точно); скетчи Task 1/2/3 — runnable-код.
- Type consistency: `EpisodeArtifact` расширяется новыми полями с дефолтами (старые артефакты грузятся); `Stepper(path)` без config сохраняет P1-контракт; writer-методы возвращают `dict[str, np.ndarray]`/nv-вектора float64; `forces` матрица — `(K, C, nv)` float64, `force_names` insertion-order.
