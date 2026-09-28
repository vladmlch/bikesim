# Task A5 brief — exact requirements

## Global Constraints

- Базовая ревизия: 70623815b98788018bcdbd8eef347d778f9bb3f3.
- Python >=3.12; использовать существующий uv.lock, не обновлять зависимости в PR физики.
- Новые численные зависимости не добавлять: NumPy, SciPy, MuJoCo и pytest уже объявлены проектом.
- Новая физика использует SI: m, s, kg, N, N*m, rad; mm и km/h допустимы только на совместимых внешних границах.
- Новая физика остаётся плоской X-Z; боковое сцепление и баланс по крену не заявляются.
- В режиме physical запрещены внешняя стабилизация тангажа, ручной перенос веса и присваивание qpos/qvel после шага для исправления физики.
- legacy и physical имеют разные physics_revision и разные численные эталоны.
- Параметры без измерений помечаются synthetic; прохождение синтетических тестов не считается экспериментальной валидацией.
- Все новые численные допуски являются критериями приёмки, а не результатами уже выполненных испытаний.

## Task body

### Task A5: coil без растяжения и отдельный top-out

**Files:** Modify `src/bike_sim/physics/coil_shock.py:59-83`, `src/bike_sim/sim/ride/forces.py:106-124`; Create `src/bike_sim/physics/stops.py`; Test `tests/test_end_stops.py`.

**Interfaces:** Produces `end_stop(q,v,lo,hi,k,c)->tuple[float,float]` = generalized force и potential energy. Coil получает `legacy_behavior=False`.

- [ ] **Step 1: failing tests.**

```python
import pytest
from bike_sim.physics.coil_shock import CoilShock
from bike_sim.physics.stops import end_stop

def test_unloaded_compression_coil_does_not_pull():
    assert CoilShock().compute_spring_force(-4.0) == 0.0

def test_top_out_pushes_into_range_and_stores_energy():
    force, energy = end_stop(-0.002, -0.1, 0.0, 0.065, 200000.0, 500.0)
    assert force == pytest.approx(450.0)
    assert energy == pytest.approx(0.4)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_end_stops.py -q`; сначала отсутствует module, затем legacy coil-тест должен воспроизводимо краснеть до исправления.
- [ ] **Step 3: реализовать два разных элемента.**

```python
# Inside CoilShock.compute_spring_force, physical branch:
compression_m = max(0.0, (float(stroke_mm)+self.specs.preload_mm)/1000.0)
return self.specs.rate_n_m * compression_m
```

```python
from math import isfinite

def end_stop(q, v, lo, hi, k, c):
    if not all(isfinite(x) for x in (q,v,lo,hi,k,c)) or hi <= lo or k <= 0 or c < 0:
        raise ValueError("invalid end-stop parameters")
    low_depth = max(lo-q, 0.0)
    high_depth = max(q-hi, 0.0)
    force = 0.0
    if low_depth > 0:
        force += max(0.0, k*low_depth-c*v)
    if high_depth > 0:
        force -= max(0.0, k*high_depth+c*v)
    return force, 0.5*k*(low_depth**2+high_depth**2)
```

Не прибавлять новый upper stop поверх существующего bumper в том же диапазоне. В writer использовать отдельно top-out ниже нуля, bumper в своей зоне и только аварийный upper stop за рабочим ходом. Изменить пределы MJCF physical совместно с аварийной деформацией; legacy ranges не менять. Записать каждый компонент энергии/силы в named contribution. Проверять preload>=0, rate>0, stroke>0 и `0<bumper_length<=stroke`.

- [ ] **Step 4:** `uv run --locked pytest tests/test_end_stops.py tests/test_coil_shock.py -q`. Добавить release-сценарий с negative stroke и обратной скоростью, проверяя отсутствие генерации энергии.
- [ ] **Step 5:** `git add src/bike_sim/physics/coil_shock.py src/bike_sim/physics/stops.py src/bike_sim/sim/ride/forces.py tests/test_end_stops.py && git commit -m "fix: separate compression spring and end-stop forces"`.

## Execution addendum from current-source/spec review

- The source compiles `shock_stroke` range exactly `[0, BikeSpecs.shock_stroke]` in `src/bike_sim/mujoco/rear_linkage.py`; the force-path validator currently requires range start `0`. The planned top-out (`q<0`) and emergency compression stop are unreachable without integration changes. Extend task scope to `src/bike_sim/physics/model_config.py`, `src/bike_sim/mujoco/builder.py`, `src/bike_sim/mujoco/rear_linkage.py`, and `src/bike_sim/sim/ride_sim.py`; physical mode gets the configured stop travel, legacy range remains unchanged. The validator may allow negative lower travel only for `shock_stroke`, never for `fork_travel`.
- Store end-stop inputs as an immutable synthetic config shared by model compilation and force application. Use the initial reference `F_ref=7000.0 N`, `delta_ref=0.010 m`, `c_stop=500.0 N*s/m`, with provenance `synthetic`; derive `k_stop=F_ref/delta_ref=700000.0 N/m`. Permit 10 mm overtravel at each physical end so the solver limit lies behind the declared stop deformation. These are uncalibrated V1 defaults derived from the existing synthetic bumper's 7000 N peak and 10 mm length.
- A2 owns `ForceAccumulator`; the physical step currently adds one combined `suspension` vector. Expose separate qfrc contributions for coil, bumper, damper, top-out and solver-limit and register them as distinct named components; preserve A2's aggregate `compute_qfrc` and legacy `apply` API.
- Avoid double force/energy in the bumper-to-upper-stop transition. In physical mode, the quadratic bumper force acts through working stroke `s<=stroke`; at the boundary, hand off at zero rate to a preloaded upper emergency stop with boundary force `F_bumper_peak` and energy offset equal to the bumper's fully compressed energy `F_bumper_peak * bumper_length_m / 3`. Above stroke, do not add the progressive bumper force again. Extend `end_stop` with optional boundary force/energy offset if needed; the standalone `end_stop(-0.002,-0.1,0,0.065,200000,500)` example must still return 450 N / 0.4 J. Legacy bumper behavior stays unchanged. The damping term may activate when the stop is compressed, but must remain passive and must not erase stored energy.
- Use the resolved reference defaults from the main plan: `F_ref=7000.0 N`, `delta_ref=0.010 m`, `c_stop=500.0 N*s/m`, `k_stop=700000.0 N/m`, all marked `synthetic`; physical joint range extends to `[-0.010 m, stroke+0.010 m]`, with solver limits behind this declared deformation. The zero-rate upper handoff must preserve static force and potential energy, and lower top-out/contact damping must pass passivity tests.
- Preserve the old CoilShock law when `legacy_behavior=True`; A4 completed the matching damper flag. Physical callers use the corrected law.
- Fix-round reviewer requirements: reject a supplied `CoilShockSpecs.stroke_mm` that conflicts with `BikeSpecs.shock_stroke` before compilation; when no coil is injected, build it using BikeSpecs' active stroke and rate. If a CoilShock instance/subclass is supplied, preserve that instance and its custom force behavior rather than rebuilding a base `CoilShock` from `.specs`.
- The explicit upper stop is not the MuJoCo hard joint limit. After `mj_step`, extract only rows where `data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT` and `data.efc_id` is the `shock_stroke` joint; map the selected multipliers through `mujoco.mj_mulJacTVec` into a distinct `shock_solver_limit` generalized-force vector. Store/report it as a post-step constraint channel, separate from pre-step `ForceSample.components` and from terrain contacts. Add an isolated MuJoCo test that activates the joint limit and verifies this named channel. F1's recorder must accumulate this interval's solver-limit power/work separately when present.
