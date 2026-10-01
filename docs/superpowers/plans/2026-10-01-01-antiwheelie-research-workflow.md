# Anti-Wheelie Research Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Дать пользователю единый воспроизводимый viewer/headless-стенд для подключения собственного ограничителя момента, сохранив реальный путь сил и независимую оценку качества симуляции.

**Architecture:** Переиспользовать `ResearchEnvironment` как единственный исследовательский цикл. Подключить существующие TOML resolver, sensor/replay modules и добавить тонкую policy session; исследовательский viewer только отображает этот же цикл, не входит в preview mode. Физическая квалификация выполняется отдельно по плану B.

**Tech Stack:** Python >=3.13, uv, MuJoCo, NumPy, pytest, существующий passive viewer; без новых runtime dependencies.

**Spec:** `docs/superpowers/specs/2026-10-01-antiwheelie-development-bench-design.md`

## Global Constraints

- Python >=3.13; every Python command uses `uv`; no new runtime dependencies.
- Work only in this checkout; preserve existing dirty/staged/untracked files and previous verification artifacts.
- Do not commit, change branches, or change default profiles without explicit user authorization.
- Do not implement or tune the user's anti-wheelie algorithm.
- Retain planar X-Z dynamics, articulated rider, mid-drive torque units, and the sensor/truth boundary.
- Do not extend detailed-chain physics; do not add external pitch stabilization or runtime qpos/qvel corrections.
- Keep energy residual limit 0.05 and existing model-scope rejection; failed/incomplete runs cannot pass release gates.
- Preserve the existing preview command; research runs opt in explicitly and use the same core as headless runs.
- All newly created evidence uses a new output directory, source/config hashes and runtime versions.
- Never describe synthetic verification as measured real-bicycle validation.

---

## Порядок и текущий статус

План создан по статическому аудиту HEAD `3f94874`. Ни один checkbox здесь не означает выполненную работу. Перед выполнением проверить текущий HEAD и dirty state; не использовать старые отчёты как свежий baseline.

**Первым выполнить Task B1 из соседнего плана:** пользователь сообщил об отрыве ног от педалей и остановке на подъёме. B1 использует существующий headless/replay diagnostic и не зависит от новых интерфейсов A. Затем A1–A5, затем B2–B4. Нельзя объявить готовым интерфейс управления, если райдер систематически теряет педали в базовом режиме.

Каждая задача заканчивается проверяемым изменением и review. Git commit только по отдельному разрешению пользователя; checkpoint не означает автоматический commit.

## Карта файлов

| Файл | Ответственность |
|---|---|
| `sim/research/sensors.py` | Измерения, acquisition parameters, шум, bias, dropout, age |
| `sim/research/environment.py` | Физические шаги, command delay, отдельные sensor clocks, оценки и сохранение |
| `sim/research/configuration.py` | TOML/explicit CLI precedence и общая сборка environment |
| `cli/research.py`, `cli/ride.py` | Разбор CLI, выбор viewer/headless, перевод единиц на границе |
| `sim/research/replay.py` | Проверенная запись и реконструкция входов/состояния |
| новый `sim/research/policy_session.py` | Только policy lifecycle, вызов act, operator events |
| новый `sim/research/viewer.py` | Rendering, pause/reset/stop, отображение validity |
| `sim/research/policies.py` | Прозрачные демонстрационные политики, без anti-wheelie |
| `tools/research_batch.py` | Серии запусков через ту же policy factory/configuration |

Пути в таблице начинаются с `src/bike_sim/`, кроме `tools/`. Не разбирать `PhysicalRuntime` на новые подсистемы ради этой задачи; полный accounted path уже существует.

### Task A1: Независимые часы и корректная validity датчиков

**Files:**
- Modify: `src/bike_sim/sim/research/sensors.py:9`
- Modify: `src/bike_sim/sim/research/environment.py:122`
- Test: `tests/test_sensor_acquisition.py`
- Test: `tests/test_research_sensors.py`
- Test: `tests/test_sensor_no_imu.py`

**Interfaces:**
- Consumes: `raw_observation(sim, sample=None) -> SensorObservation`; `sample.time_s`/`sample.dt_s` описывают входящий физический интервал.
- Produces: новые поля `SensorConfig.sample_period_s: float = .001`, `acceleration_bias_mps2: tuple[float, float, float] = (0., 0., 0.)`, `gyro_bias_rad_s: float = 0.`, `dropout_probability: float = 0.`, `maximum_age_s: float = .1`.
- Produces: `SensorPipeline.samples_attempted: int`, `samples_dropped: int`; существующие `reset`, `push`, `read` сохраняют сигнатуры.
- Produces: `ResearchEnvironment.sensor_steps: int`; sensor acquisition внутри физического цикла, независимо от control ticks.

- [ ] **Step 1: Запустить уже написанные regression tests до изменения кода.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_sensor_acquisition.py tests/test_research_sensors.py tests/test_sensor_no_imu.py
```

Ожидаемый red по текущим исходникам: отсутствующие параметры `SensorConfig` и counters, неверное сохранение `valid=False`. Зафиксировать фактический результат, не подменять его этим прогнозом.

- [ ] **Step 2: Добавить один тест кратности для fast-профиля и один на повторный reset.**

В `tests/test_sensor_acquisition.py` уже есть `observation()` и fixture `sim`. Добавить:

```python
def test_reset_restarts_sensor_random_stream_and_counters():
    config = SensorConfig(sample_period_s=.001, dropout_probability=.25)
    pipeline = SensorPipeline(config, seed=71)
    outputs = []
    for attempt in range(2):
        pipeline.reset(observation())
        for sample_index in range(1, 20):
            pipeline.push(observation(sample_index * .001))
        outputs.append((pipeline.read(.03), pipeline.samples_attempted,
                        pipeline.samples_dropped))
    assert outputs[0] == outputs[1]
    assert outputs[0][1] == 20


def test_fast_profile_requires_an_aligned_sensor_period():
    from bike_sim.sim.research.environment import _integer_steps
    assert _integer_steps(.005, .00125, 'sensor period') == 4
    with pytest.raises(ValueError, match='sensor period'):
        _integer_steps(.001, .00125, 'sensor period')
```

- [ ] **Step 3: Реализовать параметры и causal pipeline.**

Добавлять поля после существующего `imu_enabled`, сохраняя старые positional arguments. В `__post_init__` проверять scalar durations, конечные bias, форму acceleration tuple, `0 <= dropout_probability <= 1`; `sample_period_s` строго положителен. `SensorConfig.ideal()` продолжает выключать noise/latency, но не clock.

Отдельные streams создать через NumPy `SeedSequence(seed).spawn(2)`: один для noise, второй для dropout. В `reset()` пересоздать оба stream, обнулить counters и queue; затем один раз вызвать `push(initial)`. При каждом `push` увеличить attempts; noise draw всегда выполнять одинаково, независимо от IMU/dropout; если dropout — увеличить drops и не ставить измерение в queue. Для пустой queue хранить нулевой invalid startup sample, не выдавать отброшенное измерение как доступное.

Правило доставки после выбора newest eligible sample:

```python
age_s = now - selected.source_time_s
valid = (selected.valid
         and selected.source_time_s <= now - self.config.latency_s + 1e-12
         and age_s <= self.config.maximum_age_s + 1e-12)
return replace(selected, time_s=now, valid=valid)
```

Bias добавляется один раз при acquisition, до delivery. `read` не расходует RNG, не увеличивает counters. Выключенный IMU сохраняет точные нули для acceleration/gyro и не изменяет encoder/torque noise.

- [ ] **Step 4: Перенести acquisition в физический цикл без двойного стартового сэмпла.**

При создании environment:

```python
self.sensor_steps = _integer_steps(
    self.sensor_config.sample_period_s, self.dt_s, 'sensor period')
if self.sensor_steps < 1:
    raise ValueError('sensor period must span at least one physics step')
```

После стартового `pipeline.reset(initial)` задать следующий source step `self.sensor_steps`. Внутри `step()`, после получения полного `sample`, но до возможного terminal break:

```python
raw = raw_observation(self.sim, sample)
source_step = round(raw.source_time_s / self.dt_s)
if source_step >= self._next_sensor_step:
    if source_step != self._next_sensor_step:
        raise RuntimeError('sensor acquisition skipped a physical sample')
    self.pipeline.push(raw)
    self._sensor_time = raw.source_time_s
    self._next_sensor_step += self.sensor_steps
```

Убрать старый одинарный `pipeline.push` после control loop; там оставить только `read(sim.time_s)` и сохранение observation. Начальный source `t=0` уже учтён reset; 40 мс run с периодом 1 мс имеет 40 попыток (`0..39` мс), как требует имеющийся тест. Не использовать endpoint force после `mj_forward` вместо frozen incoming sensor values.

- [ ] **Step 5: Повторить тесты Step 1; проверить diff и зафиксировать review checkpoint.**

```bash
git diff --check
```

Deliverable: одинаковые измерения на общих delivery times при control periods 10/20 мс, воспроизводимый reset, явный отказ несовместимого sensor period. Предлагаемое сообщение разрешённого впоследствии commit: `fix: decouple research sensor acquisition from policy timing`.

### Task A2: Одна конфигурация для пользовательского профиля и research

**Files:**
- Modify: `src/bike_sim/sim/research/configuration.py:12`
- Modify: `src/bike_sim/cli/research.py:26`
- Modify: `src/bike_sim/sim/ride/physical_session.py:163`
- Test: `tests/test_research_configuration.py`
- Test: `tests/test_research_examples.py`
- Test: `tests/test_research_cli.py`
- Test: `tests/test_research_transmission.py`

**Interfaces:**
- Consumes: `resolve_research_physics(default_config, args)` and `research_field(track, resolution_m)` уже существуют.
- Produces: `build_environment(*, track, rider, physics_config, experiment, sensors, road_resolution_m=.005, demand=None, rider_program=None) -> ResearchEnvironment` в `sim/research/configuration.py`.
- Produces CLI: `--physics-config PATH`, `--road-resolution M`, `--sensor-period S`, `--rider-program PATH` у `bike-research`.
- Produces precedence: defaults < TOML < явно переданные physics flags; default CLI human torque не перекрывает TOML или RiderProgram.

- [ ] **Step 1: Добавить regression для точного пользовательского TOML.**

В `tests/test_research_configuration.py` использовать существующий `capture_config`:

```python
def test_fast_profile_keeps_pedaling_assist_and_shifting(monkeypatch):
    captured, result = capture_config(monkeypatch, [
        '--physics-config', 'examples/research/viewer_physics_fast.toml',
        '--track-file', 'examples/research/rough_uphill_extreme.toml',
        '--assist', '--sensor-period', '.005',
    ])
    config = captured['physics_config']
    assert config.initial_speed_mps == 0.
    assert config.timestep_s == .00125
    assert config.drive.human_torque_nm == 20.
    assert config.drive.pedaling.enabled
    assert config.drive.shifting.enabled
    assert config.drive.assist.max_torque == 85.
    assert config.drive.assist.max_power == 600.


def test_assist_command_does_not_erase_configured_human_effort():
    from types import SimpleNamespace
    arguments = cli.parser().parse_args([
        '--physics-config', 'examples/research/viewer_physics_fast.toml',
        '--assist', '--sensor-period', '.005'])
    environment = SimpleNamespace(rider_program=None, demand_nm=None,
                                  sim=SimpleNamespace(time_s=0.))
    command = cli.command_for(environment, arguments)
    assert command.motor_torque_nm is None
    assert command.human_torque_nm is None
    assert command.posture is None
```

При переносе construction в `configuration.py` обновить monkeypatch targets `capture_config` на место фактического lookup, а не возвращать production aliases ради теста. `result` здесь только возвращаемый capture result; физику этот тест не запускает.

- [ ] **Step 2: Запустить targeted red.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_research_configuration.py tests/test_research_examples.py tests/test_research_cli.py tests/test_research_transmission.py
```

Ожидаемо parser пока не знает новых flags. Тест explicit zero должен сохраниться: `0` — override, не «не задано».

- [ ] **Step 3: Соединить parser, resolver и factory.**

Добавить argparse action, сохраняющий явность physics аргумента; назначить его существующим physics flags и новым transmission/stiffness flags. Пример полностью определённого action:

```python
class ExplicitPhysicsValue(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        supplied = set(getattr(namespace, '_explicit_physics', ()))
        supplied.add(self.dest)
        setattr(namespace, '_explicit_physics', supplied)
        setattr(namespace, self.dest, values)
```

Расширить `PHYSICS_FLAGS` следующими mappings:

```python
PHYSICS_FLAGS.update({
    'human_torque': ('drive', 'human_torque_nm'),
    'transmission': ('drive', 'transmission_model'),
    'chain_stiffness': ('drive', 'chain_k_n_m'),
    'freehub_stiffness': ('drive', 'freehub_k_nm_rad'),
})
```

Сначала создать прежний default config, затем вызвать `resolve_research_physics`, затем проверить соответствие effective transmission и chain-specific overrides. Проверять запрет chain stiffness после разрешения TOML, а не по parser default. Если явно меняется transmission без явного `dt`, выбирать прежний per-transmission default только при отсутствии TOML `timestep_s`; не перезаписывать время из файла.

Factory использует существующие типы, их импортировать в `configuration.py`:

```python
def build_environment(*, track, rider, physics_config, experiment, sensors,
                      road_resolution_m=.005, demand=None, rider_program=None):
    sim = RideSimulation(
        track=track, rider=rider, physics_config=physics_config,
        field=research_field(track, road_resolution_m))
    return ResearchEnvironment(
        sim, experiment, sensors, demand=demand, rider_program=rider_program)
```

`make_environment(args)` сохраняет публичную сигнатуру и делегирует factory. `--rider-program` загружает существующий `RiderProgram.load`, конфликт с `--rider-random` и явно заданными `--posture`/`--human-torque` завершает запуск с понятной ошибкой до construction. `--sensor-period` по умолчанию `.005`; поправить старое CLI expectation `.001` на новое documented default, оставив все explicit 1 мс API tests.

В `command_for` выбирать human command так:

```python
explicit_human = 'human_torque' in getattr(args, '_explicit_physics', ())
human = (args.human_torque
         if explicit_human or args.physics_config is None else None)
if env.rider_program is not None:
    human = None
```

Оставить прежнюю семантику старых вызовов без файла. Posture не задавать при neutral. `--assist` не подменять числовым нулём.

- [ ] **Step 4: Подтвердить совпадение effective config.**

Повторить Step 2. Добавить параметризацию `--initial-speed 0`, `--motor-max-torque 0`, `--transmission geometric_ideal_mid_drive`, `--dt .000625`; проверять изменённое поле и сохранение остальных через `asdict`. Входные единицы `bike-ride --initial-speed` остаются км/ч, `bike-research --initial-speed` — м/с; сравнивать разрешённый `SimulationPhysicsConfig`, а не Namespace.

- [ ] **Step 5: Review checkpoint.**

```bash
git diff --check
```

Deliverable: exact fast physics TOML можно исследовать без молчаливого изменения двигателя, cadence policy, shifting или человеческого усилия. Это совместимость конфигураций, ещё не квалификация физики.

### Task A3: Закончить self-contained replay и запись отказов

**Files:**
- Modify: `src/bike_sim/sim/research/environment.py:130`
- Modify: `src/bike_sim/sim/research/replay.py:39`
- Modify: `src/bike_sim/sim/ride/physical_session.py:96`
- Test: `tests/test_research_replay.py`
- Test: `tests/test_demand_program.py`
- Test: `tests/test_research_quality.py`

**Interfaces:**
- Consumes: `integration_state(sim)`, `save_replay_files(env, path)`, `DemandProgram.from_dict`, `RiderProgram.from_dict` уже существуют.
- Produces: `env.initial_integration_state: numpy.ndarray`, `replay.json`, `states.npz`, `transitions.jsonl` в `env.save`.
- Produces: replay восстановит demand и независимый rider program, сохранит прежний version/source/hash rejection.

- [ ] **Step 1: Запустить существующий replay regression.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_research_replay.py
```

После A1 ожидается отказ отсутствующей записи bundle/state, а не отсутствующего sensor parameter.

- [ ] **Step 2: Добавить тест непостоянного demand.**

В `tests/test_research_replay.py`:

```python
def test_replay_restores_time_varying_demand(tmp_path):
    from bike_sim.sim.research.demand import DemandProgram
    sim = RideSimulation(
        track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig(
            'physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))
    demand = DemandProgram(((0., 10.), (.01, 30.)))
    env = ResearchEnvironment(
        sim, ExperimentConfig(duration_s=.02, actuator_delay_s=0.),
        SensorConfig.ideal(), demand=demand)
    while not env.done:
        env.step(RideControl(motor_torque_nm=env.demand_nm, human_torque_nm=0.))
    destination = env.save(tmp_path / 'demand')
    rebuilt = rebuild_environment(destination)
    assert rebuilt.demand.to_dict() == demand.to_dict()
    assert replay_episode(destination)['passed']
```

- [ ] **Step 3: Соединить существующие функции в правильном порядке.**

В конце `_begin_episode`, после reset forces/sensors и до первого управляющего шага:

```python
from bike_sim.sim.research.replay import integration_state
self.initial_integration_state = integration_state(self.sim).copy()
```

В `save()` после сохранения всех файлов, входящих в `replay.FILES`:

```python
from bike_sim.sim.research.replay import save_replay_files
save_replay_files(self, path)
```

В `_rebuild` восстановить `demand` до construction:

```python
from bike_sim.sim.research.demand import DemandProgram
recorded_demand = research.get('demand_program')
demand = (None if recorded_demand is None
          else DemandProgram.from_dict(recorded_demand))
```

Передать `demand=demand` в существующий `ResearchEnvironment`. Runtime-dependent joint envelope сейчас является внешним path: включить его JSON в bundle и manifest при наличии, восстановить относительный path через bundle. Проверять checksum содержимого; canonical configuration hash для envelope должен зависеть от contents hash, а не абсолютного пути машины. Добавить tampering test на этот файл, чтобы «самодостаточная запись» не зависела от старого checkout path.

При simulation exception сохранять `summary.error` и доступные артефакты, затем повторно поднимать исключение; `replay_episode` продолжает отклонять запись незавершённого solve. Не добавлять фиктивную успешную transition.

- [ ] **Step 4: Проверить повторный reset и program ownership.**

Расширить demand test вторым `env.reset(seed=23)`, новым output path и проверкой нулевого времени initial state. Для rider program использовать `examples/research/rider_shift.toml`, чтобы replay не применял программу дважды: commands already store post-program control. Bundle хранит program для provenance/rebuild paired runs, а `replay_episode` повторяет recorded merged commands при выключенном `env.rider_program`; observations/trace сравниваются полностью. `rebuild_environment` для новых парных экспериментов, напротив, сохраняет program. Добавить реальный regression с двумя keyframes и explicit effort.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_research_replay.py tests/test_demand_program.py tests/test_research_quality.py
```

- [ ] **Step 5: Review checkpoint.**

```bash
git diff --check
```

Deliverable: сохранённый успешный прогон воспроизводится с проверкой каждого observation/transition и final state; испорченные/неполные/несовместимые записи отвергаются до заявления о replay success.

### Task A4: Policy lifecycle без доступа к truth и без управления райдером

**Files:**
- Create: `src/bike_sim/sim/research/policy_session.py`
- Modify: `src/bike_sim/sim/research/policies.py`
- Modify: `src/bike_sim/cli/research.py`
- Modify: `tools/research_batch.py`
- Create: `tests/test_policy_session.py`
- Modify: `tests/test_research_batch.py`
- Modify: `examples/research/controller_loop.py`

**Interfaces:**
- Produces: `load_policy(reference: str) -> object` для `module:factory`; factory без аргументов, объект с `reset(seed)` и `act(observation, demand_nm)`.
- Produces: `PolicySession(env, policy)`, `advance(*, front_brake_demand=0., rear_brake_demand=0.) -> ResearchStep`, `reset(*, seed=None) -> SensorObservation`.
- Produces: `passthrough_factory()` и `zero_factory()` в `sim/research/policies.py`. Existing callable demos остаются совместимыми; новый pass-through сохраняет `None` как pedelec.

- [ ] **Step 1: Написать unit test на границу полномочий без MuJoCo.**

```python
from types import SimpleNamespace
import pytest
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.policy_session import PolicySession


class FakeEnvironment:
    def __init__(self):
        self.seed = 17
        self.observation = object()
        self.demand_nm = None
        self.done = False
        self.commands = []
        self.sim = SimpleNamespace(time_s=0.)

    def step(self, control, **brakes):
        self.commands.append((control, brakes))
        self.sim.time_s += .01
        return 'advanced'

    def reset(self, *, seed=None):
        if seed is not None:
            self.seed = seed
        self.commands.clear()
        self.sim.time_s = 0.
        return self.observation


class RecordingPolicy:
    def __init__(self, control=None):
        self.control = RideControl() if control is None else control
        self.resets = []
        self.inputs = []

    def reset(self, seed):
        self.resets.append(seed)

    def act(self, observation, demand_nm):
        self.inputs.append((observation, demand_nm))
        return self.control


def test_assist_and_rider_intent_survive_policy_session():
    env = FakeEnvironment()
    policy = RecordingPolicy(RideControl(motor_limit_nm=40.))
    session = PolicySession(env, policy)
    assert session.advance(rear_brake_demand=.4) == 'advanced'
    assert policy.inputs == [(env.observation, None)]
    command, brakes = env.commands[0]
    assert command.motor_torque_nm is None
    assert command.human_torque_nm is None
    assert brakes['rear_brake_demand'] == .4
    session.reset(seed=23)
    assert policy.resets == [17, 23]


@pytest.mark.parametrize('control', [
    RideControl(human_torque_nm=0.), RideControl(rider_enabled=False)])
def test_motor_policy_cannot_change_rider(control):
    env = FakeEnvironment()
    session = PolicySession(env, RecordingPolicy(control))
    with pytest.raises(ValueError, match='rider'):
        session.advance()
    assert env.commands == []
```

- [ ] **Step 2: Запустить red, затем реализовать тонкую session.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_policy_session.py
```

Минимальная реализация session:

```python
class PolicySession:
    def __init__(self, env, policy):
        self.env = env
        self.policy = policy
        self.operator_events = []
        self.policy.reset(self.env.seed)

    def advance(self, *, front_brake_demand=0., rear_brake_demand=0.):
        if self.env.done:
            raise RuntimeError('episode has ended')
        command = self.policy.act(self.env.observation, self.env.demand_nm)
        if not isinstance(command, RideControl):
            raise ValueError('policy must return RideControl')
        if (command.human_torque_nm is not None or command.posture is not None
                or not command.rider_enabled):
            raise ValueError('motor policy cannot own rider inputs')
        if front_brake_demand or rear_brake_demand:
            self.operator_events.append({
                'time_s': self.env.sim.time_s,
                'front_brake_demand': front_brake_demand,
                'rear_brake_demand': rear_brake_demand,
            })
        return self.env.step(command, front_brake_demand=front_brake_demand,
                             rear_brake_demand=rear_brake_demand)

    def reset(self, *, seed=None):
        observation = self.env.reset(seed=seed)
        self.operator_events.clear()
        self.policy.reset(self.env.seed)
        return observation
```

Импортировать существующий `RideControl`; не выдавать объект `env` в аргументы factory/act. Python plugin не является security sandbox: это API boundary для корректных пользовательских алгоритмов.

- [ ] **Step 3: Реализовать loader, passthrough и обработку ошибок.**

```python
from importlib import import_module


def load_policy(reference):
    module_name, separator, factory_name = reference.partition(':')
    if not separator or not module_name or not factory_name:
        raise ValueError('policy must be module:factory')
    factory = getattr(import_module(module_name), factory_name)
    policy = factory()
    if not callable(getattr(policy, 'reset', None)):
        raise ValueError('policy must implement reset(seed)')
    if not callable(getattr(policy, 'act', None)):
        raise ValueError('policy must implement act(observation, demand_nm)')
    return policy


class PassthroughPolicy:
    def reset(self, seed):
        self.seed = seed

    def act(self, observation, demand_nm):
        return RideControl(motor_torque_nm=demand_nm)


def passthrough_factory():
    return PassthroughPolicy()
```

`zero_factory` возвращает объект того же интерфейса, у которого `act` всегда `RideControl(motor_torque_nm=0.)`. Никаких threshold/controller heuristics добавлять не нужно.

В CLI и batch перед construction/load output проверить policy reference, затем каждый run получает новый экземпляр. Ошибка `act` помечает `policy_error`, сохраняет последнее валидное состояние/причину и завершает run; stale command после ошибки не продолжает двигать симуляцию. В batch остальные независимые runs продолжаются и ошибка остаётся отдельной строкой. Добавить tests factory-per-run, wrong return type, invalid import, act exception.

Metadata политики: module/factory, source file SHA256 если доступен, seed, отсутствие source file обозначается `null` и explicit reproducibility limitation. Никогда не выполнять код из replay bundle: replay использует записанные команды.

- [ ] **Step 4: Проверить фактический actuator path.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_policy_session.py tests/test_research_control.py tests/test_research_environment.py tests/test_research_batch.py
```

Расширить существующий control test случаем `RideControl(motor_limit_nm=0.)` в assist mode: после transport delay фактический motor torque равен нулю, человеческое усилие и program не меняются; после снятия потолка восстанавливается ограниченный ramp. Ceiling расположен после lag, но получает обычную задержку доставки всей команды.

- [ ] **Step 5: Review checkpoint.**

Deliverable: пользователь подключает factory без знания `MjData`, а GUI/headless/batch могут одинаково управлять моментом. Ни один demo не объявляется anti-wheelie решением.

### Task A5: Исследовательский viewer и headless как две оболочки одной session

**Files:**
- Create: `src/bike_sim/sim/research/viewer.py`
- Modify: `src/bike_sim/cli/ride.py:169`
- Modify: `src/bike_sim/cli/ride.py:713`
- Modify: `src/bike_sim/sim/research/configuration.py`
- Modify: `src/bike_sim/sim/research/environment.py:259`
- Create: `tests/test_research_viewer.py`
- Modify: `tests/test_physics_cli.py`
- Modify: `docs/RIDE.md`
- Modify: `docs/ANTI_WHEELIE.md`

**Interfaces:**
- Produces CLI `bike-ride --research --policy module:factory --control-period .01 --sensor-period .005 --sensor-delay .01 --actuator-delay .005 --road-resolution .005 --demand NM --rider-program PATH`.
- `--demand` и `--rider-program` необязательны; без demand режим pedelec; default policy `bike_sim.sim.research.policies:passthrough_factory`.
- Produces `advance_control_ticks(session: PolicySession, count: int) -> int` в `sim/research/viewer.py` для тестирования frame grouping.
- Produces `run_research_viewer(session: PolicySession, output_dir: Path) -> int`; macOS launcher переиспользует `ensure_macos_mjpython`.

- [ ] **Step 1: Написать regression, что viewer batching не меняет входы и результаты.**

В `tests/test_research_viewer.py` использовать реальную короткую flat simulation; полностью определить helper:

```python
import numpy as np
from bike_sim.cli.research import parser, make_environment
from bike_sim.sim.research.policy_session import PolicySession
from bike_sim.sim.research.policies import passthrough_factory
from bike_sim.sim.research.viewer import advance_control_ticks


def run_grouped(groups):
    arguments = parser().parse_args([
        '--scenario', 'flat', '--rider', 'lumped', '--duration', '.04',
        '--demand', '20', '--initial-speed', '0', '--seed', '7'])
    env = make_environment(arguments)
    session = PolicySession(env, passthrough_factory())
    for count in groups:
        advance_control_ticks(session, count)
    return env


def test_render_grouping_does_not_change_physics_or_policy_samples():
    regular = run_grouped([1, 1, 1, 1])
    delayed = run_grouped([0, 3, 0, 1])
    assert regular.commands_applied == delayed.commands_applied
    assert regular.observations == delayed.observations
    assert regular.tracker.metrics == delayed.tracker.metrics
    assert regular.reason == delayed.reason
    assert np.array_equal(regular.sim.data.qpos, delayed.sim.data.qpos)
    assert regular.sim.physical.research_accounting_valid
    assert delayed.sim.physical.sample is not None
```

- [ ] **Step 2: Запустить red и реализовать batching без второго physical loop.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_research_viewer.py
```

```python
def advance_control_ticks(session, count):
    if type(count) is not int or count < 0:
        raise ValueError('control tick count must be a nonnegative integer')
    completed = 0
    for tick_index in range(count):
        if session.env.done:
            break
        session.advance()
        completed += 1
    return completed
```

Viewer использует `RealTimePacer` с периодом **control**, а не physics; один tick вызывает `env.step`, который делает целое число physics steps. Rendering и camera update происходят между ticks. При wall-clock lag замедлить отображение simulated time, не пропускать команды/физику. Не входить в `preview_mode`. Показывать simulated time, real-time factor, torque, validity и terminal reason.

- [ ] **Step 3: Подключить CLI и lifecycle.**

После существующего resolve `bike-ride` передать `args.resolved_physics`, `resolve_track`, `resolve_rider` в `build_environment`; не преобразовывать TOML в другую CLI строку. `--research` допустим только для physical effort drive. Для legacy режимов новые research flags отвергать. Ветка без `--research` остаётся существующим viewer/headless.

В обеих research ветках обязательно применять `--duration` (default 90 с), seed, одинаковые terrain field/experiment/sensors/policy. Headless loop:

```python
while not session.env.done:
    session.advance()
```

В viewer поддержать pause, reset, camera, stop и operator brake. Reset сначала сохраняет предыдущий episode в отдельный numbered directory, затем `session.reset`; для следующего эпизода не перезаписывать предыдущие файлы. Стоп окна/клавиши записывает `operator_stop`, время и partial status. Терминальное состояние остаётся видимым без продолжения интеграции. Изменение spring/damper/motor mode клавишами во время research run отвергать с сообщением «измените конфигурацию и reset»; camera keys не меняют физику.

Перед `env.save` добавить policy metadata и operator events к сохраняемому research metadata через новое `env.run_metadata: dict`, инициализируемое пустым в `_begin_episode`. Политика не получает этот dict. `PolicySession` заполняет metadata, а CLI сохраняет session-owned event list. Summary помечает `operator_intervention: true`, если применён ручной тормоз; такой run исключается из unattended acceptance.

Exit status: `0` — корректно завершённый физический эпизод (включая физический crash/stall); `2` — invalid model/numerics/config; `3` — policy/simulation exception. Batch success считается по результатам, не только exit code. Досрочно закрытый viewer — partial, не completed.

- [ ] **Step 4: Проверить headless parity и ручной GUI.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_research_viewer.py tests/test_physics_cli.py tests/test_research_configuration.py tests/test_research_replay.py tests/test_policy_session.py
```

Будущая smoke-команда после реализации:

```bash
uv run bike-ride --research --headless \
  --physics-config examples/research/viewer_physics_fast.toml \
  --rider articulated_planar \
  --track examples/research/rough_uphill_extreme.toml \
  --duration 15 --sensor-period .005 --seed 17 \
  --out output/antiwheelie-workflow-smoke
```

Повторить с новым `--out` и без `--headless`; проверить pause/reset/close, одинаковые commands/observations до одинакового simulated time, final accounting. Fast-профиль может завершиться invalid — это корректная демонстрация gates, но не прохождение плана B. GUI smoke выполнять на доступной macOS/display-сессии; headless test не заменяет эту проверку.

- [ ] **Step 5: Документировать один рабочий путь и review checkpoint.**

Добавить в `docs/RIDE.md` и `docs/ANTI_WHEELIE.md` существующий preview command, research variant, factory interface, единицы Н·м на валу, правила sensor/truth и latency. Указать, что физический профиль ещё квалифицируется планом B. В описании старого refocus-plan status ссылаться на новые планы; пользовательский dirty plan не изменять.

```bash
git diff --check
```

## Финальная проверка плана A

- [ ] B1 имеет воспроизведение отрыва ног, причинную диагностику и проверенный regression fix либо доказанный физический предел с рабочим базовым rider case; необъяснённый отрыв не закрыт.
- [ ] A1–A5 targeted checks действительно выполнены; результаты и команды записаны в новом verification directory.
- [ ] Exact physics TOML воспроизводится, human effort не становится нулевым из-за parser defaults.
- [ ] Политика не читает truth и не меняет райдера; generic environment по-прежнему допускает независимый RiderProgram.
- [ ] Rendering не меняет timestep, acquisition, delay и sequence controls.
- [ ] Replay проверен с demand, rider program и внешним joint envelope; отказ сохраняется честно.
- [ ] Перейти к плану B; не объявлять стенд физически квалифицированным только по green unit tests.
