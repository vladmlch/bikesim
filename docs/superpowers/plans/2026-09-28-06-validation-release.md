# 06 — наблюдаемость, валидация и выпуск Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Сделать физические выводы проверяемыми по независимым стендам, согласованной телеметрии и данным с известным происхождением.

**Architecture:** Состояние и силы записываются в одном временном соглашении. Механический и электрический балансы разделены. Pure tests, engine rigs, полная модель и holdout-измерения образуют разные уровни проверки; golden snapshots не заменяют ни один из них.

**Tech Stack:** Python >=3.12, MuJoCo, NumPy, SciPy, pytest; existing uv.lock.

**Spec:** [2026-09-28-physics-correctness.md](../specs/2026-09-28-physics-correctness.md)

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

---

## Файлы и зависимости

F1 начинать после A2 и расширять вместе с каждым силовым компонентом. F2 требует реализованных backend/drive/rider для соответствующих сценариев; отсутствие подсистемы не считается пройденным тестом. F3 определяет данные, но не создаёт измерения. F4 публикует только реально прошедшие gates.

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

### Task F2: отдельные стенды и матрица сходимости

**Files:** Create `src/bike_sim/validation/__init__.py`, `src/bike_sim/validation/benchmarks.py`, `tools/validate_physics.py`; Test `tests/test_physics_benchmarks.py`, `tests/test_physics_conservation.py`.

**Interfaces:** Produces `radial_rig(load_n,dt,duration_s=2.0)->dict[str,float]`, `relative_change(a,b,scale_floor)->float`, `run_suite(output_dir,dt_values)->dict`. Suite записывает измеренные метрики, критерии, версии и pass/fail по каждому сценарию, не только общий bool.

- [ ] **Step 1: конкретный failing engine-rig test.**

```python
import pytest
from bike_sim.validation.benchmarks import radial_rig

@pytest.mark.parametrize("load_n",[100.0,300.0,600.0,1000.0])
def test_isolated_radial_contact_matches_material_stiffness(load_n):
    result = radial_rig(load_n,0.00025)
    assert result["deflection_m"] == pytest.approx(load_n/130000.0,rel=0.02,abs=0.0001)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_physics_benchmarks.py -q`.
- [ ] **Step 3: реализовать стенд без подвески, чтобы она не скрывала ошибку покрышки.**

```python
import mujoco
import numpy as np
from bike_sim.physics.tire import normal_contact

def radial_rig(load_n,dt,duration_s=2.0):
    if not np.isfinite([load_n,dt,duration_s]).all() or min(load_n,dt,duration_s) <= 0:
        raise ValueError("invalid radial rig settings")
    model = mujoco.MjModel.from_xml_string(f'''<mujoco>
      <option timestep="{dt:.17g}" gravity="0 0 0"/>
      <worldbody><body><joint type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="2.4" diaginertia="0.1 0.2 0.1"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    for _ in range(round(duration_s/dt)):
        normal,_ = normal_contact(-float(data.qpos[0]),-float(data.qvel[0]),130000.,800.)
        data.qfrc_applied[0] = normal-load_n
        mujoco.mj_step(model,data)
    return {"load_n":load_n,"dt_s":dt,"deflection_m":-float(data.qpos[0]),
            "speed_mps":float(data.qvel[0]),"expected_deflection_m":load_n/130000.0}

def relative_change(a,b,scale_floor):
    if scale_floor <= 0:
        raise ValueError("positive comparison scale required")
    return abs(a-b)/max(abs(b),scale_floor)
```

Реестр suite обязан включать следующие случаи с указанной постановкой; это отдельные функции/pytest cases, не один road golden:

| Имя случая | Постановка | Проверяемые величины |
|---|---|---|
| `wheel_inertia` | Одна закреплённая ось, gravity/friction=0, известный torque | A04, rotational energy |
| `radial_100_300_600_1000` | Приведённый выше стенд, k/c неизменны | A06, settling, три dt |
| `radial_native_reference` | Та же нагрузка, native sphere/plane вместо C3 | Эффективная осадка, не автоматический PASS материальной модели |
| `flat_static` | Full physical bike, без привода, решённый sag | A02–A05 и A12 |
| `incline_30_deg` | Плоскость 30°, statically constrained longitudinal motion | Fn, полная вертикальная сила, A10 |
| `brake_hold` | Уклон 10°, достаточный brake ceiling, assist off, 5 s | A15; фактическое смещение и brake power |
| `drive_low_mu` | Flat, mu=0.1, положительный crank input | A09, ненулевой slip, torque не обрезан traction-controller |
| `airborne_passive` | Bike+rider над поверхностью, no aero/assist/root torque, 2 s | A18, ballistic CoM |
| `released_rider` | Все опоры разомкнуты, impulse только bike | A19, independent root response |
| `suspension_cycle` | Известные sinusoidal stroke/velocity на pure damper | A08, A11, отдельные HBO/bumper |
| `chain_locked_geometry` | Замороженный linkage, engaged hub, losses off | A13–A14 |
| `chain_suspension_motion` | Фиксированные углы crank/cassette, движение linkage | Extension derivative и chain reaction |
| `smooth_coast` | Гладкий профиль, no drive, no abrupt stops, 5 s | A17, breakdown диссипации |
| `single_edge` | Один edge с одинаковой начальной скоростью | Импульс, событие отрыва, multi_support; не один raw force peak |
| `road_worn` | Один seed и resolved config | Mean/RMS/travel/event metrics и A20–A22 |

Для `run_suite` реализовать явный registry имени к функции; каждый результат содержит `case_id`, `physics_mode`, `backend`, `dt`, `metrics`, `criteria`, `passed`. Исключение стенда фиксируется как `passed=false`, не как пропуск. `native_reference` может быть диагностическим case без утверждения V2; его статус должен отличаться от принятого physical backend.

`tools/validate_physics.py` принимает `--out` и `--dt 0.0005 0.00025 0.000125`, вызывает suite, сохраняет JSON `allow_nan=False` и возвращает ненулевой exit code при падении обязательных cases. Для полного bike benchmark применять ту же внешнюю команду/initial condition во времени, не одинаковое число шагов. При изменении terrain resolution сохранять физические размеры и профиль, менять только дискретизацию.

- [ ] **Step 4:** `uv run --locked pytest tests/test_physics_benchmarks.py tests/test_physics_conservation.py -q`; затем `uv run --locked python tools/validate_physics.py --out output/physics-validation --dt 0.0005 0.00025 0.000125`. Ожидается полный отчёт; заявлять PASS только после фактического прогона.
- [ ] **Step 5:** `git add src/bike_sim/validation tools/validate_physics.py tests/test_physics_benchmarks.py tests/test_physics_conservation.py && git commit -m "test: add independent physics rigs and convergence report"`.

### Task F3: происхождение измерений и независимый holdout

**Files:** Create `src/bike_sim/validation/datasets.py`; Test `tests/test_calibration_dataset.py`.

**Interfaces:** Produces `validate_dataset(payload,allow_synthetic=False)->dict` и `holdout_metrics(measured,predicted)->dict[str,float]`. Формат соответствует разделу 11 спецификации.

- [ ] **Step 1: failing test утечки данных между fit/holdout.**

```python
import pytest
from bike_sim.validation.datasets import validate_dataset

def test_one_experiment_cannot_be_in_fit_and_holdout():
    payload = {
      "dataset_id":"tire-rig-1","source":"synthetic","measured_at":"2026-09-28",
      "units":{"force":"N","deflection":"m"},"bike_config_hash":"fixture",
      "sensor_uncertainty":{"force":1.0},"conditions":{"pressure_pa_gauge":200000.0},
      "samples":[{"experiment_id":"a","force":100.0,"deflection":0.001}],
      "split":{"fit":["a"],"holdout":["a"]}}
    with pytest.raises(ValueError,match="overlap"):
        validate_dataset(payload,allow_synthetic=True)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_calibration_dataset.py -q`.
- [ ] **Step 3: валидировать схему и разделение целых экспериментов.**

```python
import copy
import numpy as np

def validate_dataset(payload,allow_synthetic=False):
    required = {"dataset_id","source","measured_at","units","bike_config_hash",
                "sensor_uncertainty","conditions","samples","split"}
    if not required <= payload.keys():
        raise ValueError("missing dataset metadata")
    fit,holdout = set(payload["split"]["fit"]),set(payload["split"]["holdout"])
    if fit & holdout:
        raise ValueError("fit/holdout experiment overlap")
    if not fit or not holdout:
        raise ValueError("fit and holdout experiments are both required")
    if payload["source"] == "synthetic" and not allow_synthetic:
        raise ValueError("synthetic data cannot validate a measured model")
    ids = {row["experiment_id"] for row in payload["samples"]}
    if fit|holdout != ids:
        raise ValueError("every experiment must have exactly one split")
    for row in payload["samples"]:
        numeric = [value for key,value in row.items() if key != "experiment_id"]
        if not np.isfinite(np.asarray(numeric,dtype=float)).all():
            raise ValueError("non-finite measurement")
    return copy.deepcopy(payload)

def holdout_metrics(measured,predicted):
    y,p = np.asarray(measured,dtype=float),np.asarray(predicted,dtype=float)
    if y.shape != p.shape or y.size == 0 or not np.isfinite(y).all() or not np.isfinite(p).all():
        raise ValueError("invalid holdout vectors")
    error = p-y
    return {"rmse":float(np.sqrt(np.mean(error**2))),"max_abs":float(np.max(np.abs(error))),
            "bias":float(np.mean(error))}
```

Дополнить строгой схемой единиц для конкретного вида стенда: tire load-deflection принимает force N и deflection m; damper принимает velocity m/s, stroke m и force N; motor принимает torque N*m, angular speed rad/s и electrical power W. Неподдержанные единицы отклонять, а не конвертировать догадкой. Для измеренных данных требовать непустой source identifier, дату, uncertainty и условия эксперимента. Synthetic fixture не называть измеренным даже при хорошей ошибке.

Отчёт fit/holdout сохраняет параметры, границы, dataset hash и ошибки отдельно по экспериментам. Порог допустимой ошибки берётся из заранее заданного model error budget и uncertainty, а не подбирается после чтения holdout. Перед V2 обязателен независимый файл отчёта; отсутствие данных оставляет статус parameterized_unvalidated.

- [ ] **Step 4:** `uv run --locked pytest tests/test_calibration_dataset.py -q`. Добавить tests missing units, NaN, пустой holdout, неизвестный experiment и запрет synthetic при allow_synthetic=False.
- [ ] **Step 5:** `git add src/bike_sim/validation/datasets.py tests/test_calibration_dataset.py && git commit -m "feat: validate calibration provenance and experiment holdout"`.

### Task F4: CLI, schema v2 и выпуск без ложной обратной совместимости

**Files:** Modify `src/bike_sim/cli/ride.py:50-102`, `src/bike_sim/sim/ride/session.py`, `src/bike_sim/sim/ride/recorder.py`, `src/bike_sim/sim/ride/hud.py`, `src/bike_sim/viz/ride_plots.py` (наличие проверено в дереве базовой ревизии). Test `tests/test_physics_cli.py`. Обновить `docs/RIDE.md` только после принятого поведения, не использовать его как источник physics.

**Interfaces:** Produces CLI parameters из DATA-01, единый resolved config для API/CLI, `summary.json.schema_version=2`, сохраняемый configuration hash. Legacy CSV/JSON сохраняют version 1 через явный adapter.

- [ ] **Step 1: failing tests новых аргументов и недопустимых комбинаций.**

```python
import pytest
from bike_sim.cli.ride import parse_args

def test_explicit_physical_effort_arguments():
    args = parse_args(["--physics","physical","--drive","crank_effort",
                       "--initial-speed","0","--human-torque","20","--assist-gain","2"])
    assert args.physics == "physical"
    assert args.drive == "crank_effort"
    assert args.initial_speed == 0.0

def test_speed_target_is_not_silently_used_for_coast():
    with pytest.raises(SystemExit):
        parse_args(["--physics","physical","--drive","coast","--speed","25"])
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_physics_cli.py -q`; ожидается unrecognized arguments до новой схемы.
- [ ] **Step 3: явный parsing и precedence.**

```python
parser.add_argument("--physics",choices=("legacy","physical"),default=None)
parser.add_argument("--drive",choices=("coast","ideal_speed_control","crank_effort","articulated_effort"),default=None)
parser.add_argument("--physics-config",default=None)
parser.add_argument("--initial-speed",type=float,default=None)
parser.add_argument("--human-torque",type=float,default=None)
parser.add_argument("--assist-gain",type=float,default=None)
parser.add_argument("--timestep",type=float,default=None)
```

Изменить существующий `--speed` на `default=None` и применять прежний default 25 km/h только после определения legacy/ideal mode. Иначе невозможно отличить пользовательский speed target от default при проверке coast. Precedence: schema defaults → TOML из `tomllib` → только явно переданные CLI значения. После разрешения режима вернуть `args.physics/drive` заполненными, как ожидает тест.

При физическом effort `--human-torque` — средний torque request, а `--initial-speed` — начальное условие. При установке initial speed задать согласованные поступательные скорости bike и независимого rider root, а также абсолютные wheel speeds с вычитанием parent angular speed; не запускать их с несовместимыми скоростями и затем считать начальный удар свойством покрышки.

Метаданные сериализовать canonical JSON с sort_keys=True и allow_nan=False; hash вычислять по resolved physical config и фактическим вершинам/масштабу terrain. Сохранять фактические версии из runtime, seed и идентификатор модели. Новые силы/скорости/мощности в HUD и plots читать из snapshot v2, не вычислять параллельно другим способом.

Команды приёмки API/CLI:

```bash
uv run --locked bike-ride --physics physical --drive coast --initial-speed 20 --rider lumped --headless --no-plots --out output/physics-coast
uv run --locked bike-ride --physics physical --drive crank_effort --human-torque 20 --assist-gain 2 --rider lumped --headless --no-plots --out output/physics-effort
uv run --locked python tools/validate_physics.py --out output/physics-validation --dt 0.0005 0.00025 0.000125
uv run --locked pytest -q
```

Неподвижный coast без initial speed разрешён как стенд с лимитом времени; CLI-заезд до финиша без входной энергии должен объяснять отсутствие движения, а не включать cruise автоматически. Golden physical создаётся после A01–A23, отдельным коммитом с метриками; legacy golden не заменять новым физическим поведением.

- [ ] **Step 4:** `uv run --locked pytest tests/test_physics_cli.py tests/test_ride_cli.py tests/test_physics_telemetry.py -q`; затем указанные команды полного gate. Сравнить resolved config и метрики одинакового сценария из API и CLI.
- [ ] **Step 5:** `git add src/bike_sim/cli/ride.py src/bike_sim/sim/ride/session.py src/bike_sim/sim/ride/recorder.py src/bike_sim/sim/ride/hud.py src/bike_sim/viz/ride_plots.py tests/test_physics_cli.py docs/RIDE.md && git commit -m "feat: expose verified physical modes and versioned run metadata"`.

## Финальная проверка покрытия

Сопоставить все SYS/SUS/MASS/TIRE/DRIVE/RIDER/DATA требования с выполненными задачами ведущего плана. Для каждого A01–A23 указать test node и сохранённый результат. Непроверенный сценарий остаётся непроверенным; он не исчезает из отчёта и не получает зелёный статус по прохождению соседних тестов.

Отдельно прочитать diff на наличие ручных qpos/qvel corrections, внешнего pitch moment в physical, двойного контакта, двойного torque/energy и неподписанных синтетических коэффициентов. Только после этой проверки разрешён статус V1; V2 требует F3 с реальными holdout-данными.