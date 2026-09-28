# Task A6 brief — exact requirements

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

### Task A6: исправить приведённую массу и численно подбирать sag

**Files:** Modify `src/bike_sim/physics/tuning.py:75-94`; Create `src/bike_sim/sim/ride/sag_fit.py`; Test `tests/test_sag_physics.py`; существующий `tests/test_mass_distribution.py` менять только в assertions исправленного аналитического расчёта.

**Interfaces:** Produces `reflected_shock_mass(mass_kg,leverage_ratio)->float`; `fit_sag(evaluate,target_mm,initial,bounds)->np.ndarray`, где `evaluate(psi,rate)->tuple[float,float]` возвращает фактические front/rear wheel travel в mm.

- [ ] **Step 1: failing tests с независимыми ожидаемыми значениями.**

```python
import numpy as np
import pytest
from bike_sim.physics.tuning import reflected_shock_mass
from bike_sim.sim.ride.sag_fit import fit_sag

def test_reflected_mass_follows_kinetic_energy():
    assert reflected_shock_mass(50.0, 3.0) == 450.0

def test_sag_fit_solves_two_positive_parameters():
    def evaluate(psi, rate):
        return 5000.0/psi, 5e6/rate
    result = fit_sag(evaluate, (50.0,50.0), (80.0,90000.0),
                     ((20.0,20000.0),(200.0,300000.0)))
    np.testing.assert_allclose(result, (100.0,100000.0), rtol=1e-4)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_sag_physics.py -q`; ожидается отсутствующий API.
- [ ] **Step 3: реализовать преобразование и ограниченный поиск.**

```python
from math import isfinite

def reflected_shock_mass(mass_kg, leverage_ratio):
    if not all(isfinite(v) for v in (mass_kg, leverage_ratio)) or mass_kg <= 0 or leverage_ratio <= 0:
        raise ValueError("mass and leverage ratio must be positive")
    return mass_kg * leverage_ratio**2
```

```python
import numpy as np
from scipy.optimize import least_squares

def fit_sag(evaluate, target_mm, initial, bounds):
    target = np.asarray(target_mm, dtype=float)
    lower, upper = map(lambda x: np.asarray(x, dtype=float), bounds)
    x0 = np.asarray(initial, dtype=float)
    arrays = (target, lower, upper, x0)
    if any(a.shape != (2,) or not np.isfinite(a).all() for a in arrays):
        raise ValueError("expected finite two-element sag arrays")
    if np.any(target <= 0) or np.any(lower <= 0) or np.any(upper <= lower):
        raise ValueError("invalid sag target or bounds")
    if np.any(x0 < lower) or np.any(x0 > upper):
        raise ValueError("initial sag parameters lie outside bounds")
    def residual(log_values):
        measured = np.asarray(evaluate(*np.exp(log_values)), dtype=float)
        if measured.shape != (2,) or not np.isfinite(measured).all():
            raise ValueError("invalid equilibrium output")
        return measured-target
    result = least_squares(residual, np.log(x0), bounds=(np.log(lower),np.log(upper)))
    if not result.success or np.max(np.abs(residual(result.x))) > 0.5:
        raise RuntimeError("requested sag is not achievable within parameter bounds")
    return np.exp(result.x)
```

Заменить division by LR² в `_compute_shock_tuning` на новый helper. В реальном `evaluate` создавать конфигурацию с пробными pressure/rate, активной A3-фабрикой, теми же массами/райдером/terrain и читать результат `solve_static_equilibrium`. Все остальные настройки фиксировать. Не подставлять целевой sag в `qpos`.

Проверки NaN/inf для helper и массивов target/initial/bounds выполняются в приведённом ядре до логарифма; добавить тесты, передающие NaN и отрицательные значения в каждый из этих аргументов и ожидающие ValueError. Зафиксировать отдельно тест недостижимого target. Старое ожидаемое damping-число заменить расчётом из энергии, не увеличивать tolerance.

- [ ] **Step 4:** `uv run --locked pytest tests/test_sag_physics.py tests/test_mass_distribution.py tests/test_ride_equilibrium.py -q`.
- [ ] **Step 5:** `git add src/bike_sim/physics/tuning.py src/bike_sim/sim/ride/sag_fit.py tests/test_sag_physics.py tests/test_mass_distribution.py && git commit -m "fix: reflect shock mass correctly and solve requested sag"`.

## Execution addendum — physical CLI hookup belongs to F4

- Keep the current legacy CLI `--sag` route unchanged in A6; F4 owns the new mode-aware CLI. The physical `--sag` route in F4 must call A6's actual equilibrium-based fitter; it must not continue using the analytic estimate as a final answer.
- A6 must provide the real evaluator from its own scope in `sag_fit.py` (for example, `build_equilibrium_evaluator`): close over the same base specs, track/terrain, rider and resolved mass/config; for each trial pressure/rate, construct the physical system through `RideSimulation`/A3 and read its `solve_static_equilibrium` outputs for fork and rear-wheel travel. It may use the existing analytic helper only for initial guesses. Never assign target sag into `qpos`.
- Test the evaluator against a small, deterministic flat-track fixture and verify it returns measured equilibrium travel from the resulting system. F4 will route physical `--sag` to this helper and assert achieved error <=0.5 mm; the legacy CLI path stays on its prior behavior.

## Execution addendum from current CLI/source audit

- The active `bike-ride --sag` path is `src/bike_sim/cli/ride.py::_fit_sag`, and it currently uses `compute_suspension_tuning_for_sag` directly. A6 is incomplete unless this user-facing path calls the numerical equilibrium fitter.
- Extend scope to `src/bike_sim/cli/ride.py` and `tests/test_ride_cli.py`. Use the existing analytical helper only for bounded initial guesses; the fit evaluator must construct candidate fork pressure and coil rate through `build_suspension_components`/the same resolved config, and call `solve_static_equilibrium` for the actual selected specs/rider/terrain. Do not write the target travel into `qpos`. Keep all other run settings fixed across candidates.
- Replace fixed expected pressure/rate CLI assertions with checks against the fitted result and achieved front/rear travel (`<=0.5 mm`), without widening tolerance. Keep legacy and physical sag paths explicit and preserve the calibrated/synthetic status.

## Controller ruling on conflicting CLI addenda

The latest CLI-routing request conflicts with the preceding addendum and the 06 validation/release plan: F4 introduces the mode-aware CLI/resolved configuration, owns `ride.py` and `tests/test_ride_cli.py`, and runs after B1 supplies mass-spec plumbing. Resolve in favor of F4 ownership. A6 implements the numerical evaluator only; F4 must route physical `--sag` to it, retain the legacy analytical route, and verify achieved error <=0.5 mm. Cost if this resolution is wrong: the physical user-facing sag path remains analytic until F4, so this is a mandatory release gate rather than a discarded requirement.
