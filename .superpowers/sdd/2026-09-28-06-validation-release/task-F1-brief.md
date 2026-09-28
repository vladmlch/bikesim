# Task F1 brief — exact requirements

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

### Task F1: согласованные samples, энергия и полный Ly

**Files:** Create `src/bike_sim/sim/ride/energy.py`, `src/bike_sim/sim/ride/telemetry_v2.py`; Modify `src/bike_sim/sim/ride/recorder.py:149-203`, `src/bike_sim/sim/ride_sim.py:192-223`; Test `tests/test_physics_telemetry.py`.

**Interfaces:** Produces `ForceSample(time_s,qpos,qvel,components)`, `component_powers(sample)->dict[str,float]`, `EnergyLedger(initial_energy_j).residual(energy_j,active_work_j,external_work_j,loss_j)->float`, `system_momentum(model,data)->tuple[linear_momentum,angular_momentum_about_com]`.

- [ ] **Step 1: failing tests временной независимости и знака баланса.**

```python
import numpy as np
import pytest
from bike_sim.sim.ride.telemetry_v2 import ForceSample, component_powers
from bike_sim.sim.ride.energy import EnergyLedger

def test_force_sample_does_not_read_poststep_velocity():
    qvel = np.array([2.0])
    sample = ForceSample(0.0,np.array([0.0]),qvel,{"motor":np.array([3.0])})
    qvel[0] = 100.0
    assert component_powers(sample)["motor"] == 6.0

def test_loss_is_counted_once():
    ledger = EnergyLedger(10.0)
    assert ledger.residual(8.0,0.0,0.0,2.0) == pytest.approx(0.0)
    assert ledger.residual(13.0,5.0,0.0,2.0) == pytest.approx(0.0)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_physics_telemetry.py -q`.
- [ ] **Step 3: immutable snapshots и механический ledger.**

```python
from dataclasses import dataclass
from types import MappingProxyType
import numpy as np

@dataclass(frozen=True)
class ForceSample:
    time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    components: dict

    def __post_init__(self):
        for name in ("qpos","qvel"):
            value = np.array(getattr(self,name),dtype=float,copy=True)
            value.setflags(write=False)
            object.__setattr__(self,name,value)
        copied = {}
        for name,force in self.components.items():
            value = np.array(force,dtype=float,copy=True)
            if value.shape != self.qvel.shape or not np.isfinite(value).all():
                raise ValueError("force sample shape or value mismatch")
            value.setflags(write=False)
            copied[name] = value
        object.__setattr__(self,"components",MappingProxyType(copied))

def component_powers(sample):
    return {name:float(force@sample.qvel) for name,force in sample.components.items()}
```

```python
from math import isfinite

class EnergyLedger:
    def __init__(self,initial_energy_j):
        if not isfinite(initial_energy_j):
            raise ValueError("invalid initial mechanical energy")
        self.initial = initial_energy_j

    def residual(self,energy_j,active_work_j,external_work_j,loss_j):
        if not all(isfinite(x) for x in (energy_j,active_work_j,external_work_j,loss_j)):
            raise ValueError("non-finite energy ledger")
        if loss_j < 0:
            raise ValueError("dissipation cannot be negative")
        return energy_j-self.initial-active_work_j-external_work_j+loss_j
```

Mechanical energy = kinetic + gravitational + explicitly modelled elastic energies. Battery storage не входит в этот ledger: motor shaft work — active source. Отдельный electrical ledger проверяет расход батареи против электрической работы. При построении общего баланса battery+mechanics motor shaft work взаимно сокращается, а motor losses остаются диссипацией.

Выбрать по одному способу учёта каждого воздействия. Например, aerodynamic world work — signed external work, поэтому её нельзя ещё раз добавить как positive loss. Для tire springs включить U_tire и tire dissipation, не добавляя дополнительно всю работу этих же контактных сил как внешний источник. Native contact без известной упругой энергии допускает отдельный контактный work/residual канал, но не точное разложение на материальные потери.

Полный момент импульса рассчитывать через CoM Jacobians:

```python
import mujoco
import numpy as np

def system_momentum(model,data):
    masses = np.asarray(model.body_mass)
    total = float(masses.sum())
    if total <= 0:
        raise ValueError("no physical system mass")
    com = (masses[:,None]*data.xipos).sum(axis=0)/total
    linear = np.zeros(3)
    angular = np.zeros(3)
    jp,jr = np.zeros((3,model.nv)),np.zeros((3,model.nv))
    for body,mass in enumerate(masses):
        if mass == 0:
            continue
        mujoco.mj_jacBodyCom(model,data,jp,jr,body)
        velocity,omega = jp@data.qvel,jr@data.qvel
        R = data.ximat[body].reshape(3,3)
        inertia = R@np.diag(model.body_inertia[body])@R.T
        momentum = mass*velocity
        linear += momentum
        angular += inertia@omega+np.cross(data.xipos[body]-com,momentum)
    return linear,angular
```

В Ly-стенде не использовать неявный rotor armature без физического body: приведённая инерция требует отдельного вклада в полную энергию/момент. В рассматриваемой версии двигатель не добавляет скрытый armature. Перед momentum helper актуализировать кинематику. Сохранять в CSV значения pre-step и длительность интервала; интеграторы работы обновлять на каждом подшаге даже при decimate>1.

- [ ] **Step 4:** `uv run --locked pytest tests/test_physics_telemetry.py tests/test_ride_telemetry.py -q`. Проверить invariance Ly к общему переносу и одинаковой добавленной поступательной скорости, равенство decimate=1/10 по накопленной работе и schema_version=1 legacy adapter.
- [ ] **Step 5:** `git add src/bike_sim/sim/ride/energy.py src/bike_sim/sim/ride/telemetry_v2.py src/bike_sim/sim/ride/recorder.py src/bike_sim/sim/ride_sim.py tests/test_physics_telemetry.py && git commit -m "feat: record synchronized physical power and conservation ledgers"`.


