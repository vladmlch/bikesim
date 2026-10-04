# Realistic Pedelec Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Педалирование и ассист ведут себя как у реального mid-drive педелека: шатун жёстко связан со звездой, мотор толкает через обгонную муфту, ассист — Bosch-подобный множитель измеренного момента с лагом 40 мс, райдер держит мощность и каденс, шифтер держит полосу 75–110.

**Architecture:** Удаляется односторонняя муфта «шатун → вал мотора» (`motor_clutch`) как рабочая топология; актуатор мотора садится на `crank_spin` (безынерционный ротор) либо, опционально, на тело `motor_rotor` с одностороннним тендоном `motor_freewheel` (ротор ведёт, шатун может обгонять). `AssistController` получает профиль `bosch_cx_gen4` и режим (`turbo` по умолчанию), теряет stall-cut, boost и stop_delay, и применяет pedelec-gate как бинарное разрешение после фильтра. Райдер остаётся мощность-целевым (`SeatedClimbPolicy`), reposition удаляется, решение о накате фильтруется EMA. Legacy `motor_clutch=true` остаётся как регрессионная топология до приёмки.

**Tech Stack:** Python 3.14 (`uv run`), numpy, MuJoCo 3.12, pytest (маркер `slow`), TOML-конфиги через `bike_sim.physics.resolution`.

**Spec:** `docs/superpowers/specs/2026-10-03-v2-seated-plant.md` (S4, S6; правится в Task 0). Решения пользователя — интервью 2026-10-04 (18 вопросов), зафиксированы в Task 0 как строки S0-changelog.

## Global Constraints

- Все числа Bosch — `recalled from memory, unverified`; сверка в сети запрещена пользователем.
- 85 Н·м пик; 250 Вт номинал, 600 Вт пик; лаг момента 1-го порядка 40 мс; Eco 0.6 / Tour 1.4 / eMTB 1.4→3.4 линейно до 40 Н·м момента райдера / Turbo 3.4; дефолт Turbo; `torque_curve` `[[0,85],[120,85],[120.1,47.7],[180,0]]`; спад 23→25 км/ч, ноль на отсечке.
- Pedelec gate: ассист только при шатуне вперёд быстрее 5°/с и измеренном моменте выше `engage_torque_nm`; `control.motor_torque_nm` — потолок, не газ. Бинарное разрешение применяется ПОСЛЕ фильтра и обнуляет и состояние фильтра.
- Нет секундной stall-защиты мотора, нет boost, нет stop_delay. Тепловой дерейтинг вне объёма.
- Мотор при переключении НЕ снижается (решение пользователя Q17 подтверждает спеку). Срез усилия райдера ×0.3 на 0.2 с остаётся.
- Σ max(τ·q̇, 0) ≤ 450 Вт на райдера и 250 Вт/сустав — без изменений. Целевая мощность на шатуне 250 Вт, потолок 60 Н·м.
- Stall — легитимный исход прогона. Никаких искусственных манёвров шатуна (reposition удаляется). `rollback_brake` остаётся до D4.
- Энергия входит только от мышц, мотора и гравитации. Трассы не правятся. Новых зависимостей нет.
- Никогда не запускать два теста-прогона плant'а параллельно (память: параллельные прогоны расходятся).
- Команды из корня: `uv run python -m pytest ...`. Медленные тесты: `-m slow`; обычный прогон: `-m 'not slow'`.

---

## Карта файлов

| Файл | Ответственность в этом плане |
|---|---|
| `docs/superpowers/specs/2026-10-03-v2-seated-plant.md` | S0-строки changelog, S4-абзац усилия, S6-топология/stall/полоса |
| `docs/superpowers/plans/2026-10-03-v2-04-mid-drive.md` | D1 — часть «ассист» перенесена сюда; D3 → D3' |
| `.superpowers/sdd/2026-10-03-v2-seated-plant/progress.md` | ruling + статус слайса |
| `src/bike_sim/physics/motor_profile.py` (new) | `MotorProfile`, `BOSCH_CX_GEN4`, `PROFILES`, `assist_gain`, `pedelec_cap` |
| `src/bike_sim/physics/motor.py` | `AssistController`: профиль/режим, фильтр 40 мс, gate; без stall/boost/stop_delay |
| `src/bike_sim/physics/physical_config.py` | `AssistConfig` (profile, mode, gate), `PhysicalDriveConfig.rotor_inertia_kgm2`, `PedalingConfig.coast_cadence_tau_s`, `ArticulatedConfig.pedal_torque_ripple`, удаление reposition-полей |
| `src/bike_sim/physics/model_config.py` | валидации rotor/clutch, удаление reposition-проверки |
| `src/bike_sim/mujoco/physical_topology.py` | опциональное тело `motor_rotor` + тендон `motor_freewheel`; актуатор на `crank_spin`/`rotor_spin` |
| `src/bike_sim/sim/ride/drivetrain_forces.py` | freewheel-диагностика, вал = ротор/шатун, удаление motor-cap при шифте и reposition-аргумента |
| `src/bike_sim/sim/ride/physical_runtime.py` | начальная скорость `rotor_spin` |
| `src/bike_sim/physics/pedaling.py` | удаление `ProgressStallDetector` и reposition; EMA на coast |
| `src/bike_sim/physics/shifting.py` | решение только по каденсу шатуна |
| `src/bike_sim/sim/ride/rider_control.py` | `pedal_torque_waveform`, ripple из конфига |
| `src/bike_sim/sim/ride/control.py`, `session.py`, `input.py`, `hud.py`, `console.py`, `sim/research/{environment,policies,policy_session,rider_program,viewer}.py` | удаление `crank_reposition` |
| `src/bike_sim/cli/ride.py` | `--assist <mode>` в physical-режиме |
| `src/bike_sim/validation/rider_replay.py` | без `assist_stalled` |
| `examples/research/rider_strength_reference.json` | vmax/hill_c ног |
| `examples/research/viewer_physics_welded.toml` | новый дефолт |
| `tests/reference/test_motor_profile.py`, `test_assist_controller.py`, `test_motor_freewheel_topology.py`, `test_cadence_shifter.py`, `test_pedaling_policy.py`, `test_welded_profile_config.py`, `test_realistic_pedelec_acceptance.py` (new) | тесты |
| `tests/test_pedaling_stall.py` | удаляется вместе с reposition |

Порядок задач: 0 → 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10. Task 8 (TOML) намеренно после всех кодовых задач: до него welded-конфиг остаётся валидным на старом коде, и каждая задача проверяется на `-m 'not slow'`.

---

### Task 0: Правка спеки, D-плана и ledger

**Files:**
- Modify: `docs/superpowers/specs/2026-10-03-v2-seated-plant.md:7-27` (таблица S0 и строка «Не изменились»), `:70-73` (S4.1), `:111-119` (S6)
- Modify: `docs/superpowers/plans/2026-10-03-v2-04-mid-drive.md:14-22` (Global Constraints), `:26-36` (D1 header), `:438-450` (D3)
- Modify: `.superpowers/sdd/2026-10-03-v2-seated-plant/progress.md` (append)

- [x] **Step 1: Добавить строки в таблицу S0** (после строки `| Мотор/датчики | ... |`):

```markdown
| Муфты трансмиссии | две односторонние: шатун → вал мотора, вал/кассета → заднее колесо | одна односторонняя: вал/кассета → заднее колесо (freehub). Шатун жёстко связан со звездой через торк-сенсор. Мотор толкает звезду через обгонную муфту ротор → звезда; по умолчанию ротор безынерционный (муфта вырождается в τ ≥ 0 на шатуне), инерция ротора — опциональный параметр `drive.rotor_inertia_kgm2` | реализм (Bosch CX: звезда на шпинделе, freewheel на выходе редуктора), решение пользователя 2026-10-04 (Q1, Q8, Q16) |
| Stall-защита мотора | секундная отсечка при валу < 15 об/мин | удалена; остаётся только pedelec gate; тепловой дерейтинг вне объёма | реализм, решение пользователя 2026-10-04 (Q13) |
| Полоса каденса шифтера | 70–90 | 75–110 на любом рельефе | решение пользователя 2026-10-04 (Q18) |
| Усилие райдера | 225 Вт на шатуне | 250 Вт на шатуне, потолок 60 Н·м; накат выше 120, возврат ниже 105 об/мин; решение о накате через EMA | решение пользователя 2026-10-04 (Q12) |
| Reposition шатуна | рефлекс/команда backpedal при stall | удалён; stall — легитимный исход | W3 вынесена вперёд; решение пользователя 2026-10-04 (Q10) |
```

- [x] **Step 2: Исправить строку «Не изменились»** (строка 27): убрать `две односторонние муфты`, оставить остальное:

```markdown
Не изменились: 450 Вт, 300 Н на кисть, COP-проверка weld стопы, pedelec gate, информационная граница `SensorObservation`, energy ≤1% для работы связей, две исходные трассы без правок.
```

- [x] **Step 3: Переписать абзац муфт в S6** (строка 115 `Две односторонние муфты: ...`):

```markdown
Топология привода как у Bosch CX: шпиндель шатуна жёстко связан со звездой через торк-сенсор; ротор мотора через редуктор и обгонную муфту (ротор → звезда) добавляет момент, но звезда может обгонять ротор; вал/кассета → заднее колесо через freehub. Трансмиссия не передаёт отрицательный ведущий момент на колесо. Инерция ротора — объявленный параметр `drive.rotor_inertia_kgm2` (по умолчанию 0: ротор безынерционный, муфта вырождается в ограничение τ_motor ≥ 0 на шатуне). Секундной stall-защиты нет: ассист разрешён, пока шатун вращается вперёд и момент стопы положителен. Legacy-топология `drive.motor_clutch=true` (муфта шатун → вал) сохраняется только для регрессионного A/B и удаляется после приёмки.
```

В абзаце «Переключение» заменить `полосы каденса` на `полосы каденса 75–110 об/мин`.

- [x] **Step 4: Добавить абзац в S4.1** после предложения про 450 Вт:

```markdown
Усилие на шатуне задаёт `SeatedClimbPolicy`: момент = min(60 Н·м, 250 Вт / ω_шатуна); ниже ~40 об/мин райдер «гриндит» на потолке момента. Каденс — следствие скорости и передачи при жёстко связанном шатуне; райдер держит его шифтером (полоса 75–110) и разгоном, никогда — записью скорости. Накат выше 120 об/мин, возврат ниже 105; решение о накате принимается по EMA каденса (τ 0.35 с). Форма момента по углу — `mean·(1 + ripple·cos 2φ)`, `ripple` = 0.5.
```

- [x] **Step 5: D-план.** В Global Constraints заменить `- Две односторонние муфты; отрицательного ведущего момента на колесо нет.` на `- Одна односторонняя муфта (freehub) и обгонная муфта мотора (ротор → звезда); отрицательного ведущего момента на колесо нет. См. план 2026-10-04-realistic-pedelec-drive.md.` В заголовке D1 добавить строку: `> Часть «ассист» (motor_profile, AssistController, AssistConfig, TOML [drive.assist]) выполнена планом 2026-10-04-realistic-pedelec-drive.md Tasks 1–2; здесь остаётся часть «датчики с маской» (sensor_profile, sensors, observations).` Заменить D3 целиком:

```markdown
### Task D3': Характеризация freehub и обгонной муфты мотора

Выполнена в составе плана `2026-10-04-realistic-pedelec-drive.md` (Task 3: `tests/reference/test_motor_freewheel_topology.py`). Остаётся: `engagement_energy_loss` оракул для неупругого включения freehub при уточнении dt — добавить в тот же тестовый модуль. `crank_clutch` ratio 1 больше не характеризуется: legacy-топология удаляется после приёмки.
```

- [x] **Step 6: Ledger.** Добавить в конец `progress.md`:

```markdown
Ruling (2026-10-04): crank→motor-shaft one-way clutch is NOT the real Bosch topology (chainring is on the crank spindle; the freewheel sits between motor output and chainring). Spec S6 and S0 amended; D3 rewritten as D3'. Slice "realistic pedelec drive" (plan docs/superpowers/plans/2026-10-04-realistic-pedelec-drive.md) = D1-assist + D3' + W3-reposition + new R9 (power/cadence rider envelope), executed BEFORE B/R and WITHOUT waiting for G5 acceptance — user decision Q15; G5's constraint-work gate is orthogonal and loses one unilateral constraint. Cost if wrong: G5 first-period attribution must be re-run on the new topology before W6.
Slice status: Task 0 docs — in progress.
```

- [x] **Step 7: Проверить, что ни один документ не противоречит**: `grep -n "две односторонние\|Две односторонние" docs/superpowers/specs/2026-10-03-v2-seated-plant.md docs/superpowers/plans/*.md` — должно быть пусто, кроме строки `| v1 ... |` таблицы S0 (колонка v1 допустима).

- [x] **Step 8: Commit**

```bash
git add docs/superpowers/specs/2026-10-03-v2-seated-plant.md docs/superpowers/plans/2026-10-03-v2-04-mid-drive.md docs/superpowers/plans/2026-10-04-realistic-pedelec-drive.md .superpowers/sdd/2026-10-03-v2-seated-plant/progress.md
git commit -m "docs: amend v2 spec to the real mid-drive clutch topology"
```

---

### Task 1: Профиль мотора и pedelec-gate как чистые функции

**Files:**
- Create: `src/bike_sim/physics/motor_profile.py`
- Test: `tests/reference/test_motor_profile.py`

**Interfaces:**
- Produces: `MotorProfile` (frozen dataclass), `BOSCH_CX_GEN4: MotorProfile`, `PROFILES: dict[str, MotorProfile]` с ключом `'bosch_cx_gen4'`, `assist_gain(profile, mode, human_nm) -> float`, `pedelec_cap(human_nm, crank_rad_s, external_cap_nm, *, braking, gate_min_crank_rad_s) -> float` (возвращает `0.`, когда ассист запрещён, иначе `external_cap_nm` или `math.inf`).

- [x] **Step 1: Написать падающий тест**

```python
"""Bosch-like assist profile as declared, provenance-tagged data; gate as a binary permission."""
import math

import pytest

from bike_sim.physics.motor_profile import (
    BOSCH_CX_GEN4, PROFILES, MotorProfile, assist_gain, pedelec_cap,
)


def test_bosch_profile_values_are_the_recalled_unverified_set():
    p = PROFILES['bosch_cx_gen4']
    assert p is BOSCH_CX_GEN4
    assert (p.peak_torque_nm, p.rated_power_w, p.peak_power_w) == (85., 250., 600.)
    assert p.torque_tau_s == .04
    assert p.cutoff_mps == pytest.approx(25/3.6)
    assert p.taper_width_mps == pytest.approx(2/3.6)
    assert p.gate_min_crank_rad_s == pytest.approx(math.radians(5.))
    assert 'unverified' in p.provenance and 'no web lookup' in p.provenance


@pytest.mark.parametrize('mode,human,expected', [
    ('eco', 30., .6), ('tour', 30., 1.4), ('turbo', 30., 3.4), ('turbo', 0., 3.4),
    ('emtb', 0., 1.4), ('emtb', 20., 2.4), ('emtb', 40., 3.4), ('emtb', 80., 3.4),
    ('emtb', -5., 1.4),
])
def test_mode_gains(mode, human, expected):
    assert assist_gain(BOSCH_CX_GEN4, mode, human) == pytest.approx(expected)


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError):
        assist_gain(BOSCH_CX_GEN4, 'sport', 10.)


def test_profile_requires_provenance_and_all_four_modes():
    base = dict(name='x', peak_torque_nm=1., rated_power_w=1., peak_power_w=1., torque_tau_s=.01,
                mode_gains={'eco': .5, 'tour': 1., 'emtb': (1., 2.), 'turbo': 2.},
                emtb_full_gain_at_nm=10., cadence_support_max_rpm=100., cutoff_mps=5.,
                taper_width_mps=1., gate_min_crank_rad_s=.1, provenance='test fixture')
    MotorProfile(**base)
    with pytest.raises(ValueError):
        MotorProfile(**{**base, 'provenance': '  '})
    with pytest.raises(ValueError):
        MotorProfile(**{**base, 'mode_gains': {'eco': .5}})
    with pytest.raises(ValueError):
        MotorProfile(**{**base, 'taper_width_mps': 6.})
    with pytest.raises(ValueError):
        MotorProfile(**{**base, 'peak_torque_nm': 0.})


def test_pedelec_cap_is_a_permission_not_a_throttle():
    gate = math.radians(5.)
    allowed = dict(human_nm=10., crank_rad_s=1., external_cap_nm=None, braking=False,
                   gate_min_crank_rad_s=gate)
    assert pedelec_cap(**allowed) == math.inf
    assert pedelec_cap(**{**allowed, 'external_cap_nm': 30.}) == 30.
    assert pedelec_cap(**{**allowed, 'external_cap_nm': 0.}) == 0.
    assert pedelec_cap(**{**allowed, 'braking': True}) == 0.
    assert pedelec_cap(**{**allowed, 'human_nm': 0.}) == 0.
    assert pedelec_cap(**{**allowed, 'human_nm': -3.}) == 0.
    assert pedelec_cap(**{**allowed, 'crank_rad_s': gate}) == 0.
    assert pedelec_cap(**{**allowed, 'crank_rad_s': -1.}) == 0.
    assert pedelec_cap(**{**allowed, 'crank_rad_s': gate*1.01}) == math.inf


def test_pedelec_cap_rejects_nonfinite_or_negative_inputs():
    for bad in (math.nan, math.inf):
        with pytest.raises(ValueError):
            pedelec_cap(bad, 1., None, braking=False, gate_min_crank_rad_s=.1)
    with pytest.raises(ValueError):
        pedelec_cap(1., 1., -1., braking=False, gate_min_crank_rad_s=.1)
    with pytest.raises(ValueError):
        pedelec_cap(1., 1., None, braking=False, gate_min_crank_rad_s=-.1)
```

- [x] **Step 2: Запустить, убедиться что падает**

Run: `uv run python -m pytest tests/reference/test_motor_profile.py -q`
Expected: FAIL — `ModuleNotFoundError: bike_sim.physics.motor_profile`

- [x] **Step 3: Реализация**

```python
"""Mid-drive assist profiles as declared data, and the pedelec permission.

Every number here is recalled from memory and unverified; the user forbade a
web lookup. The profile is data with provenance, not a constant in code.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotorProfile:
    name: str
    peak_torque_nm: float
    rated_power_w: float
    peak_power_w: float
    torque_tau_s: float
    mode_gains: dict
    emtb_full_gain_at_nm: float
    cadence_support_max_rpm: float
    cutoff_mps: float
    taper_width_mps: float
    gate_min_crank_rad_s: float
    provenance: str

    def __post_init__(self):
        values = (self.peak_torque_nm, self.rated_power_w, self.peak_power_w, self.torque_tau_s,
                  self.emtb_full_gain_at_nm, self.cadence_support_max_rpm, self.cutoff_mps,
                  self.taper_width_mps, self.gate_min_crank_rad_s)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in values):
            raise ValueError('motor profile numbers must be positive and finite')
        if self.taper_width_mps > self.cutoff_mps:
            raise ValueError('speed taper exceeds the cutoff speed')
        if set(self.mode_gains) != {'eco', 'tour', 'emtb', 'turbo'}:
            raise ValueError('profile must declare eco, tour, emtb and turbo')
        for mode, gain in self.mode_gains.items():
            bounds = gain if isinstance(gain, tuple) else (gain,)
            if len(bounds) not in (1, 2) or not all(math.isfinite(g) and g >= 0 for g in bounds):
                raise ValueError(f'invalid gain for mode {mode!r}')
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('motor profile requires provenance')


BOSCH_CX_GEN4 = MotorProfile(
    name='bosch_performance_line_cx_gen4',
    peak_torque_nm=85., rated_power_w=250., peak_power_w=600., torque_tau_s=.04,
    mode_gains={'eco': .6, 'tour': 1.4, 'emtb': (1.4, 3.4), 'turbo': 3.4},
    emtb_full_gain_at_nm=40., cadence_support_max_rpm=120.,
    cutoff_mps=25/3.6, taper_width_mps=2/3.6, gate_min_crank_rad_s=math.radians(5.),
    provenance='Bosch Performance Line CX Gen 4 (no ABS); recalled from memory 2026-10-03, '
               'unverified, no web lookup by user instruction')

PROFILES = {'bosch_cx_gen4': BOSCH_CX_GEN4}


def assist_gain(profile: MotorProfile, mode: str, human_nm: float) -> float:
    """Support factor for the mode; eMTB rises linearly with rider torque."""
    if mode not in profile.mode_gains:
        raise ValueError(f'unknown assist mode {mode!r}')
    gain = profile.mode_gains[mode]
    if isinstance(gain, tuple):
        low, high = gain
        # The real eMTB law is proprietary; a linear ramp to a declared
        # saturation torque is the approximation the spec records.
        fraction = min(1., max(0., human_nm)/profile.emtb_full_gain_at_nm)
        return low+(high-low)*fraction
    return float(gain)


def pedelec_cap(human_nm, crank_rad_s, external_cap_nm, *, braking, gate_min_crank_rad_s):
    """Binary permission plus the external ceiling: 0 when assist is forbidden.

    Forbidden means braking, a crank not turning forward faster than the gate,
    or no positive measured rider torque. Positive unloading is NOT handled
    here; it flows through the controller's lag. This only hard-zeroes.
    """
    if not all(math.isfinite(x) for x in (human_nm, crank_rad_s, gate_min_crank_rad_s)):
        raise ValueError('nonfinite pedelec state')
    if gate_min_crank_rad_s < 0:
        raise ValueError('invalid pedelec gate')
    if external_cap_nm is not None and (not math.isfinite(external_cap_nm) or external_cap_nm < 0):
        raise ValueError('invalid external motor ceiling')
    if not isinstance(braking, bool):
        raise ValueError('braking must be a bool')
    if braking or crank_rad_s <= gate_min_crank_rad_s or human_nm <= 0:
        return 0.
    return math.inf if external_cap_nm is None else float(external_cap_nm)
```

- [x] **Step 4: Запустить тесты**

Run: `uv run python -m pytest tests/reference/test_motor_profile.py -q`
Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add src/bike_sim/physics/motor_profile.py tests/reference/test_motor_profile.py
git commit -m "feat: declare the Bosch-like assist profile and pedelec permission"
```

---

### Task 2: AssistController — профиль, лаг 40 мс, gate; без stall/boost/stop_delay

**Files:**
- Modify: `src/bike_sim/physics/motor.py` (весь файл)
- Modify: `src/bike_sim/physics/physical_config.py:66-86` (`AssistConfig`)
- Modify: `src/bike_sim/sim/ride/drivetrain_forces.py:406` (диагностика), `:399-402` (без `shift_limited` трогать не надо — это Task 5)
- Modify: `src/bike_sim/sim/ride/hud.py:466-469`
- Modify: `src/bike_sim/validation/rider_replay.py:184-185`
- Modify: `src/bike_sim/cli/ride.py:206,227`
- Test: `tests/reference/test_assist_controller.py`

**Interfaces:**
- Consumes: `PROFILES`, `assist_gain`, `pedelec_cap` из Task 1.
- Produces: `AssistController(*, gain=2., max_torque=80., max_power=500., tau=.05, slew=400., engage_torque_nm=4., gate_min_crank_rad_s=radians(5), cutoff_mps=25/3.6, taper_width_mps=2/3.6, torque_curve=None, profile=None, mode='turbo')`; атрибуты `.torque`, `.pedaling` (bool: момент > 0), `.profile` (`MotorProfile | None`), `.mode`, `.last_gain`; метод `step(human_nm, cadence_rpm, speed_mps, braking, dt, *, torque_request_nm=None, shaft_rpm=None) -> float`. `AssistConfig` поля: `profile, mode, gain, max_torque, max_power, tau, slew, engage_torque_nm, gate_min_crank_rad_s, cutoff_mps, taper_width_mps, torque_curve`. Удалены: `stop_delay, spin_rpm, stall_timeout_s, boost_s` и атрибуты `age, stall_s, stalled`.

- [x] **Step 1: Написать падающий тест**

```python
"""AssistController: Bosch-like support factor through a 40 ms lag, gated as a permission."""
import math

import pytest

from bike_sim.physics.motor import AssistController

RPM = 2*math.pi/60


def _turbo(**kw):
    return AssistController(profile='bosch_cx_gen4', mode='turbo', **kw)


def _run(ctrl, human, cadence_rpm, speed, seconds, dt=.001, request=None):
    torque = 0.
    for _ in range(round(seconds/dt)):
        torque = ctrl.step(human, cadence_rpm, speed, False, dt, torque_request_nm=request)
    return torque


def test_profile_owns_limits_and_lag():
    c = _turbo()
    assert (c.max_torque, c.max_power, c.tau) == (85., 600., .04)
    assert c.cutoff == pytest.approx(25/3.6) and c.width == pytest.approx(2/3.6)
    assert c.gate_min_crank_rad_s == pytest.approx(math.radians(5.))
    assert c.profile.name.startswith('bosch') and c.mode == 'turbo'


def test_turbo_multiplies_rider_torque_through_a_40ms_first_order_lag():
    c = _turbo()
    after_40ms = _run(c, 20., 60., 3., .04)
    assert after_40ms == pytest.approx(68.*(1-math.exp(-1.)), rel=.03)
    settled = _run(c, 20., 60., 3., .5)
    assert settled == pytest.approx(68., rel=1e-3)
    assert c.last_gain == pytest.approx(3.4)


def test_emtb_gain_rises_with_rider_torque():
    c = AssistController(profile='bosch_cx_gen4', mode='emtb')
    assert _run(c, 10., 60., 3., .5) == pytest.approx((1.4+.5)*10., rel=1e-3)


def test_stationary_or_backward_crank_gets_nothing_even_with_pedal_pressure():
    c = _turbo()
    for cadence in (0., .5, -30.):
        assert _run(c, 50., cadence, 0., .2) == 0.
    # 5 deg/s is the gate itself; just above it assist is allowed
    assert _run(_turbo(), 50., 5./6.+1e-3, 0., .2) > 0.
    assert _run(_turbo(), 50., 5./6.-1e-3, 0., .2) == 0.


def test_torque_below_engage_threshold_is_not_pedalling():
    c = _turbo(engage_torque_nm=4.)
    assert _run(c, 4., 60., 3., .2) == 0.
    assert _run(c, 4.01, 60., 3., .2) > 0.


def test_release_of_rider_torque_hard_zeroes_and_forgets():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    assert c.step(0., 60., 3., False, .001) == 0.
    assert c.torque == 0. and not c.pedaling
    # Permission restored: the lag restarts from zero, not from the old 68 N.m
    assert c.step(20., 60., 3., False, .001) < 5.


def test_positive_unloading_keeps_the_lag():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    first = c.step(10., 60., 3., False, .001)
    assert 34. < first < 68.


def test_speed_taper_and_cutoff():
    kmh = lambda v: v/3.6
    assert _run(_turbo(), 20., 60., kmh(22.), .5) == pytest.approx(68., rel=1e-3)
    assert _run(_turbo(), 20., 60., kmh(24.), .5) == pytest.approx(34., rel=1e-2)
    assert _run(_turbo(), 20., 60., kmh(25.), .5) == 0.
    assert _run(_turbo(), 20., 60., kmh(30.), .5) == 0.


def test_torque_curve_and_power_ceiling_bind_on_shaft_rpm():
    curve = [[0., 85.], [120., 85.], [120.1, 47.7], [180., 0.]]
    c = _turbo(torque_curve=curve)
    # 60 rpm: 600 W / 6.283 rad/s = 95.5 > 85 -> peak torque binds
    assert _run(c, 100., 60., 3., .5) == pytest.approx(85., rel=1e-3)
    # 150 rpm: curve 23.85 N.m vs power 38.2 N.m -> curve binds
    assert _run(_turbo(torque_curve=curve), 100., 150., 3., .5) == pytest.approx(23.85, rel=1e-2)
    # shaft_rpm overrides cadence for the ceiling (legacy clutch topology)
    c = _turbo(torque_curve=curve)
    for _ in range(500):
        t = c.step(100., 60., 3., False, .001, shaft_rpm=150.)
    assert t == pytest.approx(23.85, rel=1e-2)


def test_external_request_is_a_ceiling_not_a_throttle():
    assert _run(_turbo(), 0., 60., 3., .5, request=100.) == 0.
    assert _run(_turbo(), 20., 60., 3., .5, request=100.) == pytest.approx(68., rel=1e-3)
    assert _run(_turbo(), 20., 60., 3., .5, request=30.) == pytest.approx(30., rel=1e-3)
    assert _run(_turbo(), 20., 60., 3., .5, request=0.) == 0.


def test_braking_resets_everything():
    c = _turbo()
    _run(c, 20., 60., 3., .5)
    assert c.step(20., 60., 3., True, .001) == 0.
    assert c.torque == 0.


def test_without_profile_the_plain_gain_is_used():
    c = AssistController(gain=4., tau=.3, max_torque=85., max_power=600.)
    assert c.profile is None
    assert _run(c, 10., 60., 3., 3.) == pytest.approx(40., rel=1e-3)


def test_rejects_unknown_profile_mode_and_removed_knobs():
    with pytest.raises(ValueError):
        AssistController(profile='shimano_ep8')
    with pytest.raises(ValueError):
        AssistController(profile='bosch_cx_gen4', mode='sport')
    for knob in ('stall_timeout_s', 'spin_rpm', 'boost_s', 'stop_delay'):
        with pytest.raises(TypeError):
            AssistController(**{knob: 1.})


def test_assist_config_accepts_profile_and_mode():
    from bike_sim.physics.physical_config import AssistConfig
    cfg = AssistConfig(profile='bosch_cx_gen4', mode='emtb')
    assert cfg.mode == 'emtb'
    with pytest.raises(ValueError):
        AssistConfig(profile='bosch_cx_gen4', mode='off')
    with pytest.raises(TypeError):
        AssistConfig(stall_timeout_s=1.)
```

- [x] **Step 2: Запустить, убедиться что падает**

Run: `uv run python -m pytest tests/reference/test_assist_controller.py -q`
Expected: FAIL — `TypeError: unexpected keyword argument 'profile'`.

- [x] **Step 3: Переписать `motor.py`**

```python
"""Torque-sensing mid-drive assist: support factor through a first-order lag,
bounded by instantaneous shaft limits and gated as a binary permission.

With a profile (``profile='bosch_cx_gen4'``) the numbers are the recalled,
unverified Bosch CX Gen 4 set from ``motor_profile``; without one the plain
``gain`` and explicit limits form a synthetic test motor.

Order per step: demand = gain(mode, human) x human  ->  first-order lag (tau)
and slew  ->  min with the instantaneous ceiling (peak torque, torque curve
over shaft rpm, peak power / omega, speed taper)  ->  min with the pedelec
permission (0 when braking, crank not turning forward past the gate, or no
measured rider torque above ``engage_torque_nm``; the external request is a
ceiling). A zero permission also zeroes the lag state so that a restored
permission ramps up from zero instead of restoring a stale torque.

There is no stall timer, no boost hold and no stop delay: a real Gen 4 keeps
assisting as long as the cranks turn forward under load; thermal derating is
out of scope.
"""
from math import expm1, pi, radians
import numpy as np
from bike_sim.physics.checks import array, scalar
from bike_sim.physics.motor_profile import PROFILES, MotorProfile, assist_gain, pedelec_cap


class AssistController:
    def __init__(self,*,gain=2.,max_torque=80.,max_power=500.,tau=.05,slew=400.,
                 engage_torque_nm=4.,gate_min_crank_rad_s=radians(5.),
                 cutoff_mps=25/3.6,taper_width_mps=2/3.6,torque_curve=None,
                 profile=None,mode='turbo'):
        self.profile = None
        if profile is not None:
            if isinstance(profile, MotorProfile):
                self.profile = profile
            elif profile in PROFILES:
                self.profile = PROFILES[profile]
            else:
                raise ValueError(f'unknown motor profile {profile!r}')
            p = self.profile
            max_torque,max_power,tau = p.peak_torque_nm,p.peak_power_w,p.torque_tau_s
            cutoff_mps,taper_width_mps = p.cutoff_mps,p.taper_width_mps
            gate_min_crank_rad_s = p.gate_min_crank_rad_s
            if mode not in p.mode_gains:
                raise ValueError(f'unknown assist mode {mode!r} for profile {p.name}')
        if not isinstance(mode, str):
            raise ValueError('assist mode must be a string')
        self.mode = mode
        for name,value in (('gain',gain),('max_torque',max_torque),('max_power',max_power),
                           ('engage_torque_nm',engage_torque_nm),
                           ('gate_min_crank_rad_s',gate_min_crank_rad_s),('cutoff_mps',cutoff_mps)):
            scalar(value,name,minimum=0)
        if tau <= 0 or slew <= 0 or taper_width_mps <= 0:
            raise ValueError('invalid assist time constants or thresholds')
        if taper_width_mps > cutoff_mps:
            raise ValueError('speed taper exceeds the cutoff speed')
        if self.profile is not None:
            # The declared 40 ms lag must dominate: a 400 N.m/s slew would turn
            # the first-order response into a 170 ms ramp to 68 N.m.
            slew = max(slew, self.profile.peak_torque_nm/self.profile.torque_tau_s)
        self.gain,self.max_torque,self.max_power = gain,max_torque,max_power
        self.tau,self.slew,self.engage_torque_nm = tau,slew,engage_torque_nm
        self.gate_min_crank_rad_s = gate_min_crank_rad_s
        self.cutoff,self.width = cutoff_mps,taper_width_mps
        self.torque_curve = None
        if torque_curve is not None:
            curve = array(torque_curve,'torque curve',readonly=True)
            if curve.ndim != 2 or curve.shape[1] != 2 or len(curve) < 2:
                raise ValueError('torque curve needs at least two rpm/torque points')
            if np.any(curve < 0) or np.any(np.diff(curve[:,0]) <= 0):
                raise ValueError('torque curve rpm must increase strictly; limits must be nonnegative')
            self.torque_curve = curve
        self.reset()

    def reset(self):
        self.torque = 0.
        self.pedaling = False
        self.last_gain = 0.

    def ceiling(self, shaft_rpm, speed_mps):
        """Instantaneous shaft limit: peak, curve over shaft rpm, power, speed taper."""
        omega = shaft_rpm*2*pi/60
        ceiling = self.max_torque
        if self.torque_curve is not None:
            ceiling = min(ceiling,float(np.interp(shaft_rpm,self.torque_curve[:,0],self.torque_curve[:,1])))
        if omega > 0:
            ceiling = min(ceiling,self.max_power/omega)
        taper = max(0.,min(1.,(self.cutoff-abs(speed_mps))/self.width))
        return ceiling*taper, taper

    def step(self,human_nm,cadence_rpm,speed_mps,braking,dt, *,
             torque_request_nm=None,shaft_rpm=None):
        human = scalar(human_nm,'human torque')
        rpm = scalar(cadence_rpm,'cadence')
        speed = scalar(speed_mps,'road speed')
        dt = scalar(dt,'assist timestep',positive=True)
        if not isinstance(braking,(bool,np.bool_)):
            raise ValueError('braking must be a bool')
        if torque_request_nm is not None:
            torque_request_nm = scalar(torque_request_nm,'motor setpoint',minimum=0.)
        # Rigid crank/chainring: the shaft is the crank. The legacy crank-side
        # clutch passes its own shaft rpm for the ceiling and power accounting.
        shaft = rpm if shaft_rpm is None else scalar(shaft_rpm,'motor shaft rpm')
        if braking:
            self.reset()
            return 0.
        sensed = human if human > self.engage_torque_nm else 0.
        gain = assist_gain(self.profile,self.mode,sensed) if self.profile is not None else self.gain
        self.last_gain = gain
        ceiling,taper = self.ceiling(shaft,speed)
        # Support itself fades toward the cutoff, not only the hard ceiling.
        target = min(gain*sensed*taper,ceiling)
        candidate = self.torque-expm1(-dt/self.tau)*(target-self.torque)
        candidate = max(self.torque-self.slew*dt,min(self.torque+self.slew*dt,candidate))
        cap = pedelec_cap(sensed,rpm*2*pi/60,torque_request_nm,braking=bool(braking),
                          gate_min_crank_rad_s=self.gate_min_crank_rad_s)
        torque = scalar(max(0.,min(candidate,ceiling,cap)),'delivered assist torque')
        self.torque = torque
        self.pedaling = torque > 0.
        return torque
```

- [x] **Step 4: `AssistConfig`** в `physical_config.py` (заменить класс целиком):

```python
@dataclass(frozen=True)
class AssistConfig:
    # profile='bosch_cx_gen4' owns max_torque, max_power, tau, cutoff, taper
    # and gate; explicit values below are then ignored. Without a profile the
    # explicit numbers form a synthetic test motor.
    profile: str | None = None
    mode: str = 'turbo'
    gain: float = 2.
    max_torque: float = 80.
    max_power: float = 500.
    tau: float = .05
    slew: float = 400.
    engage_torque_nm: float = 4.
    gate_min_crank_rad_s: float = radians(5.)
    cutoff_mps: float = 25/3.6
    taper_width_mps: float = 2/3.6
    torque_curve: tuple[tuple[float,float],...] | None = None

    def __post_init__(self):
        from dataclasses import asdict
        from bike_sim.physics.motor import AssistController
        if self.torque_curve is not None:
            object.__setattr__(self,'torque_curve',tuple(tuple(p) for p in self.torque_curve))
        AssistController(**asdict(self))
```

Добавить `from math import radians` к импортам файла (там уже есть `from math import pi`).

- [x] **Step 5: Потребители удалённых полей.**
  - `drivetrain_forces.py:406`: заменить `'assist_stall_s':self.assist.stall_s, 'assist_stalled':self.assist.stalled,` на `'assist_mode':self.assist.mode, 'assist_gain':self.assist.last_gain,`.
  - `hud.py:468-469`: удалить строки `"assist_age_s"` и `"assist_stall_s"`; выполнить `grep -n "assist_age_s\|assist_stall_s" src/bike_sim/sim/ride/hud.py` и удалить соответствующие заголовки колонок, если есть.
  - `rider_replay.py:185`: `bool(d.get('assist_demand_gated',False))`.
  - `cli/ride.py:206`: после строки с `assist_gain` добавить `if args.assist is not None and args.physics=="physical": drive_values["assist"]={**drive_values.get("assist",{}),"mode":args.assist}`; в строке 227 убрать `or args.assist is not None`. Убедиться, что ветки 256–266 (дефолт `tour`/`turbo` для legacy) не выполняются при `args.physics=="physical"` — иначе `mode` всегда будет подставлен; если выполняются, обернуть их в `if args.physics!="physical":`.

- [x] **Step 6: Тесты**

Run: `uv run python -m pytest tests/reference/test_assist_controller.py tests/reference/test_motor_profile.py -q`
Expected: PASS.

Run: `uv run python -m pytest tests -m 'not slow' -q -x`
Expected: PASS (welded toml всё ещё содержит `stop_delay`, `boost_s`, `spin_rpm`, `stall_timeout_s` → загрузка упадёт на `unknown AssistConfig parameter(s)`). **Поэтому** в этом же шаге убрать из `examples/research/viewer_physics_welded.toml` строки `tau = 0.3`, `stop_delay = 0.3`, `spin_rpm = 15.0`, `stall_timeout_s = 1.0`, `boost_s = 0.4` и их комментарии; `gain = 4.0` пока оставить (профиль включается в Task 8). Повторить прогон — PASS.

- [x] **Step 7: Commit**

```bash
git add src/bike_sim/physics/motor.py src/bike_sim/physics/physical_config.py src/bike_sim/sim/ride/drivetrain_forces.py src/bike_sim/sim/ride/hud.py src/bike_sim/validation/rider_replay.py src/bike_sim/cli/ride.py examples/research/viewer_physics_welded.toml tests/reference/test_assist_controller.py
git commit -m "feat: profile-driven assist with 40 ms lag and binary pedelec gate"
```

---

### Task 3: Топология — шатун ≡ звезда, мотор через обгонную муфту

**Files:**
- Modify: `src/bike_sim/mujoco/physical_topology.py:57-88,156-181`
- Modify: `src/bike_sim/physics/physical_config.py:225-262` (`PhysicalDriveConfig`)
- Modify: `src/bike_sim/physics/model_config.py:119-120`
- Modify: `src/bike_sim/sim/ride/drivetrain_forces.py:28-70,133-137,213-234,296-302,333-337`
- Modify: `src/bike_sim/sim/ride/physical_runtime.py:459-464`
- Modify: `src/bike_sim/sim/ride/ideal_freehub.py:10-17` (docstring)
- Test: `tests/reference/test_motor_freewheel_topology.py`

**Interfaces:**
- Produces: `PhysicalDriveConfig.rotor_inertia_kgm2: float = 0.`; тело `motor_rotor`, шарнир `rotor_spin`, тендон `motor_freewheel` (только при `rotor_inertia_kgm2 > 0`); актуатор `mid_drive` на `rotor_spin` (ротор) / `drive_shaft_spin` (legacy clutch) / `crank_spin` (дефолт). `DrivetrainForceApplier.freewheel: IdealFreehubConstraint | None`; диагностика `drive['motor_freewheel_engaged']` (bool), `drive['motor_freewheel_torque_nm']`, `drive['motor_freewheel_dissipation_power_w']`.

Семантика: в дефолте (ротор безынерционный) обгонная муфта вырождается в `ctrlrange 0..max_torque` на `crank_spin` — ротор без состояния не может ни обогнать, ни быть обогнан. Это **уже существующий** путь `motor_clutch=false`; задача добавляет опциональный инерционный ротор и диагностику.

- [x] **Step 1: Падающий тест — unit-оракул обгонной муфты на двух инерциях**

```python
"""Motor freewheel: the rotor drives the crank, the crank may overrun the rotor, never the reverse."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint

TWO_INERTIAS = """
<mujoco>
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="rotor"><joint name="rotor_spin" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="0.3"/>
      <inertial pos="0 0 0" mass="0.3" diaginertia="0.075 0.15 0.075"/></body>
    <body name="crank"><joint name="crank_spin" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.5 1.0 0.5"/></body>
  </worldbody>
  <tendon><fixed name="motor_freewheel" limited="true" range="-1e12 0" margin="0" solreflimit="0.001 1">
    <joint joint="rotor_spin" coef="1"/><joint joint="crank_spin" coef="-1"/></fixed></tendon>
  <actuator><motor name="mid_drive" joint="rotor_spin" gear="1" ctrllimited="true" ctrlrange="0 85"/></actuator>
</mujoco>
"""


def _plant():
    model = mujoco.MjModel.from_xml_string(TWO_INERTIAS)
    data = mujoco.MjData(model)
    hub = IdealFreehubConstraint(model, 1., tendon_name='motor_freewheel',
                                 driver='rotor_spin', driven='crank_spin')
    mujoco.mj_forward(model, data)
    hub.reset(model, data)
    return model, data, hub


def _advance(model, data, hub, steps, ctrl):
    forces = []
    for _ in range(steps):
        hub.prepare(model, data)
        data.ctrl[0] = ctrl
        mujoco.mj_step(model, data)
        forces.append(float(hub.solved_qfrc(model, data)[1]))
    return forces


def test_rotor_torque_drives_the_crank_through_the_engaged_freewheel():
    model, data, hub = _plant()
    torques = _advance(model, data, hub, 200, 10.)
    # Combined inertia 1.15: both spin up together, crank sees the tendon force
    assert data.qvel[0] == pytest.approx(data.qvel[1], rel=1e-3)
    assert data.qvel[1] == pytest.approx(10./1.15*.2, rel=.02)
    assert np.mean(torques[20:]) > 0.


def test_crank_overruns_a_coasting_rotor_without_backdriving_it():
    model, data, hub = _plant()
    _advance(model, data, hub, 200, 10.)
    rotor_before = float(data.qvel[0])
    data.qvel[1] += 5.   # the legs spin the crank faster than the rotor
    forces = _advance(model, data, hub, 200, 0.)
    assert max(forces) < 1e-6              # freewheel open: no force at all
    assert data.qvel[0] == pytest.approx(rotor_before, abs=1e-9)   # rotor keeps coasting
    assert data.qvel[1] == pytest.approx(rotor_before+5., abs=1e-9)


def test_rotor_catches_up_and_reengages_without_lash():
    model, data, hub = _plant()
    _advance(model, data, hub, 200, 10.)
    data.qvel[1] += 2.
    _advance(model, data, hub, 50, 0.)
    assert hub.relative_rate(data) < 0.
    _advance(model, data, hub, 400, 30.)
    assert data.qvel[0] == pytest.approx(data.qvel[1], rel=1e-3)
    assert hub.boundary == pytest.approx(hub._relative_angle(data), abs=1e-6)
```

- [x] **Step 2: Запустить**

Run: `uv run python -m pytest tests/reference/test_motor_freewheel_topology.py -q`
Expected: PASS уже сейчас (класс существует) — это оракул семантики, он фиксирует контракт до правки топологии. Если падает — чинить `IdealFreehubConstraint`, не тест.

- [x] **Step 3: Падающий тест — построение плant'а с ротором и без**

Добавить в тот же файл:

```python
ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT/'examples'/'research'/'viewer_physics_welded.toml'


def _config(tmp_path, drive_lines):
    text = WELDED.read_text()
    start = text.index('[drive]\n')
    end = text.index('[drive.pedaling]')
    text = text[:start]+'[drive]\n'+drive_lines+'\n\n'+text[end:]
    for name in ('joint_envelope_path', 'joint_strength_path'):
        text = text.replace(f'{name} = "', f'{name} = "{WELDED.parent}/')
    path = tmp_path/'physics.toml'
    path.write_text(text)
    return path


def _model(tmp_path, drive_lines):
    from bike_sim.cli import research as research_cli
    track = tmp_path/'track.toml'
    track.write_text('name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(_config(tmp_path, drive_lines)), '--track-file', str(track),
        '--duration', '0.01', '--initial-speed', '0', '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    return env.sim.model, env.sim.physical.drive


def _actuator_joint(model):
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'mid_drive')
    return model.joint(int(model.actuator_trnid[aid, 0])).name


@pytest.mark.slow
def test_default_topology_is_rigid_crank_with_motor_on_the_crank(tmp_path):
    model, drive = _model(tmp_path, 'transmission_model = "ideal_mid_drive"')
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'crank_clutch') == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'motor_freewheel') == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'drive_shaft_spin') == -1
    assert _actuator_joint(model) == 'crank_spin'
    assert drive.clutch is None and drive.freewheel is None
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'mid_drive')
    assert model.actuator_ctrlrange[aid, 0] == 0.


@pytest.mark.slow
def test_rotor_inertia_adds_a_freewheeled_rotor_body(tmp_path):
    model, drive = _model(tmp_path, 'transmission_model = "ideal_mid_drive"\nrotor_inertia_kgm2 = 0.15')
    rotor = model.body('motor_rotor')
    assert max(rotor.inertia) == pytest.approx(.15)   # principal axes may be permuted
    assert float(rotor.mass[0]) < 1.
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'motor_freewheel') >= 0
    assert _actuator_joint(model) == 'rotor_spin'
    assert drive.freewheel is not None and drive.freewheel.driver_dof == int(model.joint('rotor_spin').dofadr[0])


@pytest.mark.slow
def test_legacy_crank_clutch_still_builds_for_regression_ab(tmp_path):
    model, drive = _model(tmp_path, 'transmission_model = "ideal_mid_drive"\nmotor_clutch = true')
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'crank_clutch') >= 0
    assert _actuator_joint(model) == 'drive_shaft_spin'
    assert drive.clutch is not None


def test_rotor_and_legacy_clutch_are_exclusive():
    from bike_sim.physics.physical_config import PhysicalDriveConfig
    with pytest.raises(ValueError):
        PhysicalDriveConfig(transmission_model='ideal_mid_drive', motor_clutch=True, rotor_inertia_kgm2=.1)
    with pytest.raises(ValueError):
        PhysicalDriveConfig(transmission_model='elastic_chain', rotor_inertia_kgm2=.1)
    with pytest.raises(ValueError):
        PhysicalDriveConfig(rotor_inertia_kgm2=-.1)
```

Run: `uv run python -m pytest tests/reference/test_motor_freewheel_topology.py -q -m slow` и без `-m` для unit-части.
Expected: FAIL — `unknown PhysicalDriveConfig parameter(s): rotor_inertia_kgm2`.

- [x] **Step 4: Конфиг.** В `PhysicalDriveConfig` после `motor_clutch`:

```python
    # Reflected inertia of the motor rotor at the crank, kg.m^2. 0 (default)
    # means a stateless rotor: the output freewheel then degenerates to
    # tau_motor >= 0 on the crank. >0 builds a rotor body with a one-way
    # tendon rotor -> crank. Recalled order of magnitude for a mid-drive:
    # 0.1-0.3; unverified.
    rotor_inertia_kgm2: float = 0.
```

В `__post_init__` после проверки `motor_clutch`:

```python
        scalar(self.rotor_inertia_kgm2,'rotor_inertia_kgm2',minimum=0)
        if self.rotor_inertia_kgm2 > 0 and self.motor_clutch:
            raise ValueError('rotor freewheel and the legacy crank clutch are exclusive')
        if self.rotor_inertia_kgm2 > 0 and self.transmission_model not in ('ideal_mid_drive','geometric_ideal_mid_drive'):
            raise ValueError('a motor rotor requires an ideal mid-drive transmission')
```

Обновить комментарий над `motor_clutch`: `# Legacy regression topology (crank -[one-way]-> drive_shaft with motor). Not the real mid-drive; kept for A/B until acceptance.`

В `model_config.py:119` добавить рядом: `if self.drive.rotor_inertia_kgm2 > 0 and self.drive_mode not in ('crank_effort','articulated_effort'): raise ValueError('drive.rotor_inertia_kgm2 requires an effort drive mode')`.

- [x] **Step 5: Топология.** В `physical_topology.py` после блока `if clutch:` (строки ~77–88, тело `drive_shaft`) добавить:

```python
    rotor_inertia=physics_config.drive.rotor_inertia_kgm2
    if rotor_inertia > 0.:
        # The rotor is coaxial with the crank: its reflected inertia is the
        # declared number; its translational mass is a nominal 0.3 kg so the
        # bike mass is not inflated by a fictitious ring. The disc tensor
        # (I/2, I, I/2) satisfies MuJoCo's triangle inequality.
        frame_body=_find(root,'body','frame')
        rotor=ET.SubElement(frame_body,'body',name='motor_rotor',pos='0 0 0')
        ET.SubElement(rotor,'joint',name='rotor_spin',type='hinge',pos='0 0 0',axis='0 1 0',damping='0')
        _set_inertia(rotor,.3,np.zeros(3),np.diag([rotor_inertia/2,rotor_inertia,rotor_inertia/2]))
```

В блоке актуаторов (строки ~156–181):

```python
        driver_joint='drive_shaft_spin' if clutch else 'crank_spin'
        motor_joint=('rotor_spin' if physics_config.drive.rotor_inertia_kgm2 > 0.
                     else driver_joint)
        ...
        if physics_config.drive.rotor_inertia_kgm2 > 0.:
            if tendons is None:
                tendons = ET.SubElement(root, 'tendon')
            freewheel = ET.SubElement(tendons, 'fixed', name='motor_freewheel',
                limited='true', range='-1e12 0', margin='0',
                solreflimit=f'{physics_config.closure_time_constant_s:.17g} 1')
            ET.SubElement(freewheel, 'joint', joint='rotor_spin', coef='1')
            ET.SubElement(freewheel, 'joint', joint='crank_spin', coef='-1')
        ...
        ET.SubElement(actuators,'motor',name='mid_drive',joint=motor_joint,gear='1',
                      ctrllimited='true',ctrlrange=f'0 {physics_config.drive.assist.max_torque:.17g}')
```

Комментарий над `driver_joint` переписать: `# Real mid-drive: the chainring is on the crank spindle, so the freehub driver is crank_spin. The legacy clutch topology keeps the chainring on drive_shaft.`

- [x] **Step 6: `drivetrain_forces.py`.**
  - `__init__`: `self.rotor = config.rotor_inertia_kgm2 > 0.`; `if self.rotor: joint_names += ('rotor_spin',)`; после создания `self.clutch`: `self.freewheel = None` и

```python
            if self.rotor:
                self.freewheel = IdealFreehubConstraint(model, 1.,
                    tendon_name='motor_freewheel', driver='rotor_spin', driven='crank_spin')
```
  - `reset` (~136): `if self.freewheel is not None: self.freewheel.reset(model, data)`.
  - `prepare` вызов (~300): рядом с `self.clutch.prepare` добавить `if self.freewheel is not None: self.freewheel.prepare(model, data)`.
  - `settle_actuation` (~217): после блока clutch:

```python
        if self.freewheel is not None:
            fw_force = self.freewheel.solved_qfrc(model, data)
            transmission = transmission + fw_force
            fw_torque = float(fw_force[self.freewheel.driven_dof])
            self.last.update(
                motor_freewheel_torque_nm=fw_torque,
                motor_freewheel_engaged=bool(fw_torque > 1e-8),
                motor_freewheel_dissipation_power_w=max(0., fw_torque*self.freewheel.relative_rate(data)))
```
  - В `compute_components` (~333): `shaft = self.joints.get('drive_shaft_spin') or self.joints.get('rotor_spin')` вместо `self.joints.get('drive_shaft_spin')`; комментарий: `# The motor acts on its own coordinate only under the legacy clutch or with an inertial rotor; otherwise the shaft is the crank.`
  - В словаре `self.last` (~380–406) добавить `'motor_freewheel_engaged': delivered > 0., 'motor_freewheel_torque_nm': delivered,` — безынерционный ротор: «муфта замкнута» ⇔ мотор даёт момент. В роторной топологии `settle_actuation` перезаписывает оба ключа решённым тендонным моментом (см. выше).
  - `physical_runtime.py:489` уже суммирует `crank_clutch_dissipation_power_w`; добавить `+ self.drive.last.get('motor_freewheel_dissipation_power_w',0.)*dt`.

- [x] **Step 7: `physical_runtime.py:459-464`**: заменить одиночный `shaft = mj_name2id(..., 'drive_shaft_spin')` на цикл по `('drive_shaft_spin','rotor_spin')` с тем же присваиванием `rate`. Комментарий: `# Any motor-side coordinate (legacy shaft or inertial rotor) starts engaged at the crank rate.`

- [x] **Step 8: Docstring `ideal_freehub.py:10-17`** заменить вторую половину на: `Defaults keep the cassette freehub (crank_spin drives rear_wheel_spin); the motor freewheel reuses the class with rotor_spin -> crank_spin at ratio 1; the legacy crank clutch with crank_spin -> drive_shaft_spin.`

- [x] **Step 9: Тесты**

Run: `uv run python -m pytest tests/reference/test_motor_freewheel_topology.py -q` затем `-m slow` для того же файла (три сборки плant'а, ~1 мин).
Expected: PASS.

Run: `uv run python -m pytest tests -m 'not slow' -q -x`
Expected: PASS.

- [x] **Step 10: Commit**

```bash
git add src/bike_sim/mujoco/physical_topology.py src/bike_sim/physics/physical_config.py src/bike_sim/physics/model_config.py src/bike_sim/sim/ride/drivetrain_forces.py src/bike_sim/sim/ride/physical_runtime.py src/bike_sim/sim/ride/ideal_freehub.py tests/reference/test_motor_freewheel_topology.py
git commit -m "feat: optional inertial motor rotor behind a one-way freewheel"
```

---

### Task 4: Удалить reposition и stall-детектор

**Files:**
- Modify: `src/bike_sim/physics/pedaling.py` (удалить `ProgressStallDetector`, reposition-логику в `PedalingPolicy`)
- Modify: `src/bike_sim/physics/physical_config.py` (`PedalingConfig`: 9 полей `reposition_*` и их валидации; `PhysicalDriveConfig.__post_init__` строка `reposition_on_stall requires motor_clutch`)
- Modify: `src/bike_sim/physics/model_config.py:121-122`
- Modify: `src/bike_sim/sim/ride/control.py:24-27,38-39,50-54`
- Modify: `src/bike_sim/sim/ride/drivetrain_forces.py` (`policy.update(..., reposition=control.crank_reposition)` → без аргумента)
- Modify: `src/bike_sim/sim/ride/session.py:93,175-192,288-291`, `input.py:100-102`, `hud.py:557`, `console.py:16`
- Modify: `src/bike_sim/sim/research/rider_program.py` (поле `crank_reposition`, `owns_crank_reposition`, парсер строки 119–125), `environment.py:114-118,302`, `policy_session.py:55,66-67,73,78`, `policies.py:65-101` (класс `RepositionOnStallPolicy` и `reposition_on_stall_factory`; найти регистрацию: `grep -rn "RepositionOnStall\|reposition_on_stall" src`), `viewer.py:34-42,64,75,104-124`
- Delete: `tests/test_pedaling_stall.py`
- Delete (untracked scratch, проверить `git ls-files examples/research | grep _diag` пуст): `examples/research/_diag_*.toml`
- Test: `tests/reference/test_pedaling_policy.py` (создать; дополняется в Task 6)

**Interfaces:**
- Produces: `PedalingPolicy.update(phase_rad, rate_rad_s, required_cadence_rpm, effort_nm, dt, *, enabled=True, braking=False)`; `PedalingState.mode ∈ {'disabled','pedaling','coasting'}`; `RideControl` без `crank_reposition`; `PolicySession.step(...)` без `crank_reposition`.

- [x] **Step 1: Падающий тест**

```python
"""PedalingPolicy after the reposition removal: a stall is an outcome, not a manoeuvre."""
import math

import pytest

from bike_sim.physics.physical_config import PedalingConfig
from bike_sim.physics.pedaling import PedalingPolicy, PedalingState


def test_reposition_knobs_and_mode_are_gone():
    with pytest.raises(TypeError):
        PedalingConfig(reposition_on_stall=True)
    assert 'reposition' not in ' '.join(PedalingConfig.__dataclass_fields__)
    policy = PedalingPolicy(PedalingConfig(enabled=True))
    with pytest.raises(TypeError):
        policy.update(0., 0., 0., 30., .001, reposition=True)


def test_a_rocking_loaded_crank_simply_keeps_pedalling():
    policy = PedalingPolicy(PedalingConfig(enabled=True, effort_slew_nm_s=0.))
    modes = set()
    for step in range(3000):
        rate = 3.*math.sin(step*.02)      # rocks around a dead spot for 3 s
        state = policy.update(math.pi/2, rate, 0., 40., .001)
        modes.add(state.mode)
    assert modes == {'pedaling'}


def test_ride_control_has_no_reposition_field():
    from bike_sim.sim.ride.control import RideControl
    assert 'crank_reposition' not in RideControl.__dataclass_fields__
```

Run: `uv run python -m pytest tests/reference/test_pedaling_policy.py -q`
Expected: FAIL (`PedalingConfig(reposition_on_stall=True)` принимается).

- [x] **Step 2: `pedaling.py`.** Удалить класс `ProgressStallDetector` и импорт, если он остаётся неиспользованным. В `PedalingPolicy.reset` удалить `_reposition_*`, `_stall`, `_APPROACH_TAU_S`. В `update` удалить параметр `reposition`, блоки от `self._reposition_cooldown = ...` до `return PedalingState('reposition', ...)` включительно, и упростить `latch`:

```python
        if not reason:
            previous = self._effort
            self.reset()
            self._effort = previous
```

Валидацию булевых оставить: `if not isinstance(enabled, bool) or not isinstance(braking, bool): raise ValueError('pedaling enable and braking must be booleans')`.

- [x] **Step 3: Конфиги.** Удалить девять `reposition_*` полей и их валидации из `PedalingConfig`, строку `if self.pedaling.reposition_on_stall and not self.motor_clutch` из `PhysicalDriveConfig.__post_init__`, строки 121–122 из `model_config.py`. Комментарий про «Crank reposition maneuver» удалить.

- [x] **Step 4: Управление.** `control.py`: удалить поле `crank_reposition`, его проверку в `__post_init__` и блок в `validate_for`. `drivetrain_forces.py`: `policy.update(..., enabled=enabled)`. `session.py`: удалить `_reposition_pulse`, метод `request_crank_reposition` и блок 288–291 (оставить обычный `control`). `input.py:100-102`: удалить привязку клавиши V. `hud.py:557` и `console.py:16`: убрать упоминание V. `research/rider_program.py`: удалить поле `crank_reposition` из `RiderKeyframe`, свойство `owns_crank_reposition`, обработку в блендинге (строки 81–85, 94–103) и ключ `'crank_reposition'` в парсере (119–125). `environment.py`: удалить проверку 114–118 и `crank_reposition=` в 302. `policy_session.py`: удалить параметр и его использование. `policies.py`: удалить `RepositionOnStallPolicy`, `reposition_on_stall_factory` и регистрацию (найти grep'ом), импорт `ProgressStallDetector`. `viewer.py`: удалить параметр `crank_reposition` из `advance_control_ticks`, переменную `reposition`, ветку клавиши V (104–110) и аргумент в 121–124; строку подсказки 75 без «V crank reposition».

- [x] **Step 5: Удалить файлы**

```bash
git rm tests/test_pedaling_stall.py
rm -f examples/research/_diag_boost.toml examples/research/_diag_cadence.toml examples/research/_diag_cadence2.toml examples/research/_diag_highcoast.toml examples/research/_diag_noclutch.toml examples/research/_diag_nocoast.toml examples/research/_diag_rigid.toml
```

Из `examples/research/viewer_physics_welded.toml` удалить блок `reposition_on_stall = true` … `reposition_noop_rad = 0.12` с комментариями (иначе конфиг не загрузится).

- [x] **Step 6: Проверка отсутствия**

Run: `grep -rn "reposition\|ProgressStallDetector" src tests examples docs/superpowers/plans/2026-10-04-realistic-pedelec-drive.md --include='*.py' --include='*.toml' | grep -v "2026-10-04-realistic"`
Expected: пусто.

Run: `uv run python -m pytest tests -m 'not slow' -q -x`
Expected: PASS.

- [x] **Step 7: Commit**

```bash
git add -A src tests examples/research/viewer_physics_welded.toml
git commit -m "refactor: remove crank reposition; a stall is a legitimate outcome"
```

---

### Task 5: Шифтер — только каденс шатуна, мотор при переключении не режется

**Files:**
- Modify: `src/bike_sim/physics/shifting.py:29-56`
- Modify: `src/bike_sim/sim/ride/drivetrain_forces.py:73-74,131-133,159-160,183-192,349-352,399-402`
- Test: `tests/reference/test_cadence_shifter.py`

**Interfaces:**
- Produces: `CadenceShifter.update(cadence_rpm, required_cadence_rpm, dt, *, pedaling, braking, rear_in_contact, rear_slip_mps)` — решение по EMA `cadence_rpm` (шатун); `required_cadence_rpm` используется только для landing-проверки и для отказа при отрицательном значении. Удалены `DrivetrainForceApplier.shift_motor_limit_nm`, диагностики `shift_motor_limit_nm`, `shift_limited`.

- [x] **Step 1: Падающий тест**

```python
"""CadenceShifter decides on the crank cadence alone; the motor is never cut by a shift."""
import pytest

from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.physical_config import ShiftingConfig
from bike_sim.physics.shifting import CadenceShifter

CASSETTE = (10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51)


def _shifter(rear=28, **kw):
    cfg = ShiftingConfig(enabled=True, cassette=CASSETTE, target_cadence_min_rpm=75.,
                         target_cadence_max_rpm=110., cadence_smoothing_tau_s=0., **kw)
    return CadenceShifter(DrivetrainSpecs(front_teeth=22, rear_teeth=rear), cfg)


def test_low_crank_cadence_downshifts_to_the_next_bigger_cog():
    s = _shifter(28)
    assert s.update(60., 60., .01)
    assert s.rear_teeth == 33 and s.direction == 'down'


def test_a_wheel_implied_spike_alone_never_upshifts():
    # Regression: the old decision used max(crank, wheel-implied); a slipping or
    # bouncing wheel then upshifted a rider grinding at 85 rpm.
    s = _shifter(28)
    for _ in range(50):
        assert not s.update(85., 140., .01)
    assert s.rear_teeth == 28


def test_high_crank_cadence_upshifts_when_the_landing_stays_in_band():
    s = _shifter(28)
    assert s.update(115., 115., .01)
    assert s.rear_teeth == 24 and s.direction == 'up'


def test_hunting_upshift_is_rejected_when_landing_falls_below_the_band():
    s = _shifter(51)
    # 112 rpm on 51T lands at 112*45/51 = 98.8 -> allowed; on 10T -> 12T impossible (smallest)
    assert s.update(112., 112., .01) and s.rear_teeth == 45
    s = _shifter(12)
    # 112 on 12T -> 10T lands at 93 (in band) -> allowed; then no smaller cog
    assert s.update(112., 112., .01) and s.rear_teeth == 10
    s.cooldown_s = 0.
    assert not s.update(130., 130., .01)


def test_cooldown_and_not_pedalling_block_shifts():
    s = _shifter(28)
    assert s.update(60., 60., .01)
    assert not s.update(60., 60., .01)          # cooldown
    s = _shifter(28)
    assert not s.update(60., 60., .01, pedaling=False)
    assert not s.update(60., 60., .01, braking=True)
    assert not s.update(60., -5., .01)          # rollback


def test_wheelspin_blocks_an_upshift_but_not_a_downshift():
    s = _shifter(28)
    assert not s.update(115., 115., .01, rear_slip_mps=.8)
    assert s.update(60., 60., .01, rear_slip_mps=.8)


def test_ema_filters_a_single_stroke_spike():
    cfg = ShiftingConfig(enabled=True, cassette=CASSETTE, target_cadence_min_rpm=75.,
                         target_cadence_max_rpm=110., cadence_smoothing_tau_s=.35)
    s = CadenceShifter(DrivetrainSpecs(front_teeth=22, rear_teeth=28), cfg)
    s.update(90., 90., .01)
    for _ in range(5):
        assert not s.update(130., 90., .01)
```

Run: `uv run python -m pytest tests/reference/test_cadence_shifter.py -q`
Expected: FAIL на `test_a_wheel_implied_spike_alone_never_upshifts` (старый `max`).

- [x] **Step 2: `shifting.py`.** Строку `cadence = max(self.cadence_ema, self.required_ema)` заменить на `cadence = self.cadence_ema`. Комментарий над EMA переписать:

```python
        # The decision follows the crank: with a rigid crank/chainring the
        # crank cadence IS the wheel-implied cadence whenever the freehub is
        # engaged, and a slipping or bouncing wheel must not read as a
        # spin-up. The wheel-implied value is kept only to predict the
        # landing cadence of a candidate gear.
```

- [x] **Step 3: `drivetrain_forces.py`.** Удалить `self.shift_motor_limit_nm` (init 74, reset 133, строка 191, `_shift_diagnostics` 159–160), блок 350–352 (`limited_request = min(..., self.shift_motor_limit_nm)`) → `limited_request = safety_request`, и диагностику `'shift_limited'` (402). Комментарий у `torque_factor` в `prepare_pedaling` (200): `# Only the rider unloads for the shift; the motor follows the rider's torque through its own lag (spec S6, Gen 4 with a mechanical derailleur).`

Run: `grep -rn "shift_motor_limit_nm\|shift_limited" src tests` → пусто.

- [x] **Step 4: Тесты**

Run: `uv run python -m pytest tests/reference/test_cadence_shifter.py tests -m 'not slow' -q -x`
Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add src/bike_sim/physics/shifting.py src/bike_sim/sim/ride/drivetrain_forces.py tests/reference/test_cadence_shifter.py
git commit -m "fix: shift on crank cadence only and never cut the motor"
```

---

### Task 6: Райдер — EMA на накате, ripple из конфига

**Files:**
- Modify: `src/bike_sim/physics/physical_config.py` (`PedalingConfig.coast_cadence_tau_s`, `ArticulatedConfig.pedal_torque_ripple`)
- Modify: `src/bike_sim/physics/pedaling.py` (EMA в `PedalingPolicy`)
- Modify: `src/bike_sim/sim/ride/rider_control.py:65-74,711-712`
- Test: `tests/reference/test_pedaling_policy.py` (дополнить), `tests/reference/test_pedal_waveform.py` (создать)

**Interfaces:**
- Produces: `PedalingConfig.coast_cadence_tau_s: float = 0.` (0 = сырой каденс); `ArticulatedConfig.pedal_torque_ripple: float = .35` (0 ≤ r < 1); `ArticulatedConfig.return_foot_preload_n: float = 0.`; `pedal_torque_waveform(mean_nm, phase_rad, ripple) -> float` в `rider_control.py`; `pedaling_force_requests(phase, mean_nm, crank_m, normals, loads, mu, *, ripple=.35)`.

О `return_foot_preload_n`. Это остаточная нагрузка возвратной ноги на поднимающуюся педаль. У живого велосипедиста-любителя на плоской педали она составляет порядка 40–100 Н (часть веса ноги) и даёт отрицательный момент на возвратной стороне — отсюда глубокие провалы двуногого момента в мёртвых точках. Кроме того, стопа на плоской педали не может тянуть, а weld может: заданный прижим держит сварку в физически допустимом сжатии вместо численного блуждания вокруг нуля. Значение 40 Н — инженерная оценка (нижняя граница диапазона), помечается как таковая. Реализация — по ветке `wip/pedaling-rhythm` (`rider_control.py`, блок `if not stance[side] and cfg.return_foot_preload_n > 0.`), переписывается заново, не cherry-pick.

- [ ] **Step 1: Падающие тесты.** Добавить в `test_pedaling_policy.py`:

```python
def test_coast_decision_filters_a_single_stroke_spike_with_ema():
    cfg = PedalingConfig(enabled=True, coast_above_rpm=120., resume_below_rpm=105.,
                         coast_cadence_tau_s=.35, effort_slew_nm_s=0.)
    policy = PedalingPolicy(cfg)
    rpm = 2*math.pi/60
    for _ in range(100):
        assert policy.update(0., 90.*rpm, 90., 30., .001).mode == 'pedaling'
    for _ in range(50):                                   # 50 ms spike to 160 rpm
        assert policy.update(0., 160.*rpm, 90., 30., .001).mode == 'pedaling'
    for _ in range(1500):                                 # sustained 160 rpm
        state = policy.update(0., 160.*rpm, 160., 30., .001)
    assert state.mode == 'coasting' and state.reason == 'cadence'
    for _ in range(1500):                                 # back below resume
        state = policy.update(0., 95.*rpm, 95., 30., .001)
    assert state.mode == 'pedaling'


def test_zero_tau_keeps_the_raw_hysteresis():
    cfg = PedalingConfig(enabled=True, coast_above_rpm=120., resume_below_rpm=105.,
                         coast_cadence_tau_s=0., effort_slew_nm_s=0.)
    policy = PedalingPolicy(cfg)
    rpm = 2*math.pi/60
    assert policy.update(0., 90.*rpm, 90., 30., .001).mode == 'pedaling'
    assert policy.update(0., 121.*rpm, 90., 30., .001).mode == 'coasting'


def test_coast_tau_must_be_nonnegative():
    with pytest.raises(ValueError):
        PedalingConfig(enabled=True, coast_cadence_tau_s=-.1)
```

Создать `tests/reference/test_pedal_waveform.py`:

```python
"""Two-leg crank torque waveform: mean x (1 + ripple cos 2phi), ripple from config."""
import math

import numpy as np
import pytest

from bike_sim.sim.ride.rider_control import pedal_torque_waveform, pedaling_force_requests


def test_waveform_mean_and_ripple():
    phases = np.linspace(0., 2*math.pi, 3600, endpoint=False)
    values = np.array([pedal_torque_waveform(40., p, .5) for p in phases])
    assert values.mean() == pytest.approx(40., rel=1e-6)
    assert values.max() == pytest.approx(60.) and values.min() == pytest.approx(20.)
    assert pedal_torque_waveform(40., 0., 0.) == 40.


def test_waveform_rejects_invalid_ripple():
    for bad in (-.1, 1., 1.5, math.nan):
        with pytest.raises(ValueError):
            pedal_torque_waveform(40., 0., bad)


def test_force_requests_honour_the_ripple_argument():
    normals = {'front': (0., 0., 1.), 'rear': (0., 0., 1.)}
    loads = {'front': 1e4, 'rear': 1e4}
    lo, _ = pedaling_force_requests(0., 40., .175, normals, loads, 5., ripple=0.)
    hi, _ = pedaling_force_requests(0., 40., .175, normals, loads, 5., ripple=.5)
    total = lambda r: sum(np.linalg.norm(r[s]) for s in r)
    assert total(hi) > 1.3*total(lo)


def test_articulated_config_exposes_the_ripple():
    from bike_sim.physics.physical_config import ArticulatedConfig
    assert ArticulatedConfig().pedal_torque_ripple == .35
    assert ArticulatedConfig(pedal_torque_ripple=.5).pedal_torque_ripple == .5
    with pytest.raises(ValueError):
        ArticulatedConfig(pedal_torque_ripple=1.)


def test_return_foot_preload_is_a_nonnegative_force():
    from bike_sim.physics.physical_config import ArticulatedConfig
    assert ArticulatedConfig().return_foot_preload_n == 0.
    assert ArticulatedConfig(return_foot_preload_n=40.).return_foot_preload_n == 40.
    with pytest.raises(ValueError):
        ArticulatedConfig(return_foot_preload_n=-1.)
```

Добавить в `tests/reference/test_seated_pedaling_cycle.py` рядом с медленным `test_full_crank_revolutions_keep_both_feet_loaded` проверку, что при `return_foot_preload_n = 40` возвратная педаль несёт не меньше 30 Н нормальной нагрузки во всех интервалах (`attachments['foot_*']['normal_n'] >= 30.` для стороны, у которой `stance` в `rider_allocation` ложен) — это и есть «стопа в сжатии».

Run: `uv run python -m pytest tests/reference/test_pedaling_policy.py tests/reference/test_pedal_waveform.py -q`
Expected: FAIL (`unknown parameter coast_cadence_tau_s`, `ImportError pedal_torque_waveform`).

- [ ] **Step 2: Конфиг.** `PedalingConfig`: после `stop_time_s` добавить

```python
    # EMA time constant for the coast/resume decision. The crank rate ripples
    # twice per revolution; a rider coasts on a sustained spin-up, not on one
    # strong stroke. 0 keeps the raw hysteresis.
    coast_cadence_tau_s: float = 0.
```
и в `__post_init__`: `scalar(self.coast_cadence_tau_s, 'coast cadence tau', minimum=0.)`.

`ArticulatedConfig`: рядом с `pedal_scrape_fraction` добавить

```python
    # Two-leg crank torque waveform depth: mean x (1 + ripple cos 2phi).
    # 0.35 is the legacy synthetic value; ~0.5 is closer to measured seated
    # pedalling (peak ~1.5x mean, dead spots ~0.5x). Engineering choice.
    pedal_torque_ripple: float = .35
```
и в `__post_init__`: `ripple = scalar(self.pedal_torque_ripple,'pedal torque ripple',minimum=0.)` + `if ripple >= 1.: raise ValueError('pedal torque ripple must be below one')`.

Там же добавить

```python
    # Residual load of the recovery leg on the rising pedal, N. A recreational
    # rider leaves ~40-100 N of leg weight on the upstroke pedal (engineering
    # estimate, lower bound used); a flat-pedal foot cannot pull, so this also
    # keeps the welded foot in physically admissible compression. 0 disables.
    return_foot_preload_n: float = 0.
```
и `scalar(self.return_foot_preload_n,'return foot preload',minimum=0.)`.

- [ ] **Step 3: `pedaling.py`.** В `reset`: `self._cadence_ema = None`. В `update` заменить

```python
        cadence = max(abs(rate_rad_s) * 60. / (2. * pi), required_cadence_rpm)
        threshold = ...
        excessive = self.config.enabled and cadence >= threshold
```
на

```python
        cadence = max(abs(rate_rad_s) * 60. / (2. * pi), required_cadence_rpm)
        tau = self.config.coast_cadence_tau_s
        if tau > 0. and self._cadence_ema is not None:
            self._cadence_ema += min(1., dt/tau)*(cadence-self._cadence_ema)
        else:
            self._cadence_ema = cadence
        threshold = (self.config.resume_below_rpm if self.coasting
                     else self.config.coast_above_rpm)
        excessive = self.config.enabled and self._cadence_ema >= threshold
```
и в блоке `if not reason:` сохранить EMA через latch: `previous, ema = self._effort, self._cadence_ema; self.reset(); self._effort, self._cadence_ema = previous, ema`. Mash-рампу оставить (legacy-путь без seated_climb).

- [ ] **Step 4: `rider_control.py`.** Перед `pedaling_force_requests` добавить

```python
def pedal_torque_waveform(mean_nm, phase_rad, ripple):
    """Two-leg crank torque: mean x (1 + ripple cos 2phi); ripple in [0, 1)."""
    mean = scalar(mean_nm, 'mean pedaling torque', minimum=0.)
    phase = scalar(phase_rad, 'pedal phase')
    depth = scalar(ripple, 'pedal torque ripple', minimum=0.)
    if depth >= 1.:
        raise ValueError('pedal torque ripple must be below one')
    return mean*(1.+depth*cos(2.*phase))
```
Сигнатуру `pedaling_force_requests(phase, mean_nm, crank_m, normals, loads, mu, *, ripple=.35)` и строку 74 → `total_torque = pedal_torque_waveform(mean, phase, ripple)`. Вызов (711–712): добавить `ripple=cfg.pedal_torque_ripple`.

Прижим возвратной ноги: в цикле `for side,offset in (('front',0.),('rear',pi))` (~716–748), в ветке `else:` (педалирование) после присваивания `requests[side] = pedaling_requests[side]` добавить

```python
                if not stance[side] and cfg.return_foot_preload_n > 0.:
                    # Recovery leg: a declared residual load, pressed along the
                    # pedal normal; the weld stays in compression like a real
                    # foot on a flat pedal. Force on the bike is -normal.
                    requests[side] = feasible_pedal_force(
                        -cfg.return_foot_preload_n*normal, normal, cfg.foot_mu, load)
```
(выше по циклу `stance[side]` уже вычислен как «доля стороны в раздаче момента > 1e-6»). Проверить, что ветка scrape-ограничения ниже (`if pedaling_weights.get(side,0.) < .5`) по-прежнему применяется после прижима.

- [ ] **Step 5: Тесты**

Run: `uv run python -m pytest tests/reference/test_pedaling_policy.py tests/reference/test_pedal_waveform.py -q && uv run python -m pytest tests -m 'not slow' -q -x`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/bike_sim/physics/physical_config.py src/bike_sim/physics/pedaling.py src/bike_sim/sim/ride/rider_control.py tests/reference/test_pedaling_policy.py tests/reference/test_pedal_waveform.py
git commit -m "feat: filtered coast decision and configurable pedal torque ripple"
```

---

### Task 7: Мышечный конверт ног под 120 об/мин

**Files:**
- Modify: `examples/research/rider_strength_reference.json` (12 кривых: `rider_{hip,knee,ankle}_{front,rear}` × positive/negative)
- Test: `tests/reference/test_joint_strength.py` (дополнить), `tests/reference/test_seated_pedaling_cycle.py:154-156` (параметр 120)

**Interfaces:**
- Produces: у ног `vmax_rad_s = 22.0`, `hill_c = 0.35`; `source` каждой кривой дополнен `; vmax/hill_c set 2026-10-04 to a 120 rpm pedalling envelope (engineering choice, unverified)`. Торс/плечо/локоть без изменений.

Арифметика: на 120 об/мин шатун 12.57 рад/с; колено с размахом ~70° → амплитуда 0.61 рад → пик 7.7 рад/с; при vmax 22, c 0.35: s = 0.35 → фактор (1−0.35)/(1+0.35/0.35) = 0.325 → 140 × 0.325 ≈ 45 Н·м разгибания колена. Нужно для 20 Н·м на шатуне (~114 Н на педали, плечо ≲ 0.35 м) ≤ 40 Н·м. Бедро на 0.9 рад при 5 рад/с: s 0.23 → 0.77/1.65 = 0.47 → 85 Н·м.

- [ ] **Step 1: Падающий тест** (добавить в `test_joint_strength.py`):

```python
def test_leg_curves_span_a_120_rpm_pedalling_envelope():
    """At 120 rpm the knee peaks near 7.7 rad/s and must still hold ~40 N.m; the hip ~60 N.m at 5 rad/s."""
    # {joint: {+1: curve, -1: curve}} keyed by the sign of the bounded q-torque;
    # JSON "positive" -> +1, "negative" -> -1.
    profile = load_strength_profile(str(STRENGTH), JOINTS, require_verified=False)
    for side in ('front', 'rear'):
        for joint in ('hip', 'knee', 'ankle'):
            for direction in (1, -1):
                curve = profile[f'rider_{joint}_{side}'][direction]
                assert curve.vmax_rad_s >= 20., (joint, side, direction)
                assert curve.hill_c >= .3, (joint, side, direction)
                assert 'unverified' in curve.source
        knee = profile[f'rider_knee_{side}'][-1]          # knee extension ("negative")
        assert directional_capacity(knee, 1.0, -7.7, -1) >= 40.
        hip = profile[f'rider_hip_{side}'][1]             # hip extension ("positive")
        assert directional_capacity(hip, .9, 5., 1) >= 60.
    torso = profile['rider_torso_hinge'][1]
    assert torso.vmax_rad_s == 8.                          # untouched
```

Run: `uv run python -m pytest tests/reference/test_joint_strength.py -q -k envelope`
Expected: FAIL (`vmax 10 < 20`).

- [ ] **Step 2: JSON.** Для шести суставов ног и обоих направлений: `"vmax_rad_s": 22.0`, `"hill_c": 0.35`, `source` дополнить строкой выше. Сделать скриптом, чтобы не ошибиться:

```bash
uv run python - <<'EOF'
import json, pathlib
p = pathlib.Path('examples/research/rider_strength_reference.json')
d = json.loads(p.read_text())
note = '; vmax/hill_c set 2026-10-04 to a 120 rpm pedalling envelope (engineering choice, unverified)'
for joint, entry in d['joints'].items():
    if any(k in joint for k in ('hip', 'knee', 'ankle')):
        for curve in entry['directions'].values():
            curve['vmax_rad_s'] = 22.0
            curve['hill_c'] = 0.35
            if note not in curve['source']:
                curve['source'] += note
p.write_text(json.dumps(d, indent=1)+'\n')
EOF
```
Проверить `git diff --stat examples/research/rider_strength_reference.json` — если файл был в компактном формате (одна кривая на строку), привести к прежнему форматированию вручную или оставить `indent=1`, но убедиться, что тесты `test_joint_strength.py` не сравнивают текст побайтово.

- [ ] **Step 3: Параметр 120 об/мин** в `test_seated_pedaling_cycle.py:155`: добавить `120.` в `@pytest.mark.parametrize('cadence_rpm', ...)`.

- [ ] **Step 4: Тесты**

Run: `uv run python -m pytest tests/reference/test_joint_strength.py -q`
Expected: PASS.

Run: `uv run python -m pytest tests/reference/test_seated_pedaling_cycle.py -q -m slow -k "120"`
Expected: PASS (один прогон ~1–2 мин). Если падает по `rider_strength_violations` — это сигнал, что конверт всё ещё связывающий; не ослаблять тест, а поднять `vmax` до 25 и зафиксировать это в S0-строке «Усилие райдера» (Task 0 — дописать).

- [ ] **Step 5: Commit**

```bash
git add examples/research/rider_strength_reference.json tests/reference/test_joint_strength.py tests/reference/test_seated_pedaling_cycle.py
git commit -m "feat: widen the leg force-velocity envelope to 120 rpm pedalling"
```

---

### Task 8: Welded TOML — реализм как дефолт

**Files:**
- Modify: `examples/research/viewer_physics_welded.toml:27-164`
- Test: `tests/reference/test_welded_profile_config.py`

- [ ] **Step 1: Падающий тест**

```python
"""The welded preview profile is the realistic pedelec default."""
from pathlib import Path

import pytest

from bike_sim.physics.resolution import load_physics_config

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT/'examples'/'research'/'viewer_physics_welded.toml'


@pytest.fixture(scope='module')
def cfg():
    return load_physics_config(str(WELDED), {})


def test_rigid_crank_topology_is_the_default(cfg):
    assert cfg.drive.transmission_model == 'ideal_mid_drive'
    assert cfg.drive.motor_clutch is False
    assert cfg.drive.rotor_inertia_kgm2 == 0.
    assert cfg.drive.human_torque_nm == 0.     # effort comes from seated_climb


def test_assist_is_bosch_turbo_with_the_declared_envelope(cfg):
    a = cfg.drive.assist
    assert a.profile == 'bosch_cx_gen4' and a.mode == 'turbo'
    assert a.torque_curve == ((0., 85.), (120., 85.), (120.1, 47.7), (180., 0.))
    assert a.engage_torque_nm == 4.


def test_rider_numbers(cfg):
    p = cfg.drive.pedaling
    assert (p.coast_above_rpm, p.resume_below_rpm) == (120., 105.)
    assert p.coast_cadence_tau_s == .35
    assert p.rollback_brake is True
    s = cfg.drive.shifting
    assert (s.target_cadence_min_rpm, s.target_cadence_max_rpm) == (75., 110.)
    assert s.torque_factor == .3
    assert cfg.seated_climb.enabled and cfg.seated_climb.target_crank_power_w == 250.
    assert cfg.seated_climb.max_crank_torque_nm == 60.
    assert cfg.articulated.pedal_torque_ripple == .5
    assert cfg.articulated.return_foot_preload_n == 40.
    assert cfg.articulated.active_positive_power_limit_w == 450.
```

Run: `uv run python -m pytest tests/reference/test_welded_profile_config.py -q`
Expected: FAIL (`motor_clutch is True`, `profile None`, ...).

- [ ] **Step 2: Переписать блоки TOML** (строки от `[drive]` до `[drive.shifting]` включительно и `[seated_climb]`, `[articulated]` дополнить). Итоговый текст:

```toml
[drive]
# Real mid-drive topology: the chainring sits on the crank spindle (rigid
# crank/chainring through the torque sensor); the motor pushes through a
# one-way freewheel; the cassette freehub is the only other one-way element.
# RU: шатун жёстко со звездой, мотор через обгонную муфту. motor_clutch=true —
# legacy-топология (муфта шатун→вал) только для регрессионного A/B.
transmission_model = "ideal_mid_drive"
motor_clutch = false
# Reflected rotor inertia at the crank, kg*m^2. 0 = stateless rotor (the
# freewheel degenerates to tau_motor >= 0). ~0.1-0.3 is the recalled order of
# magnitude for a mid-drive (unverified); set >0 for an inertial-rotor A/B.
rotor_inertia_kgm2 = 0.0

[drive.pedaling]
enabled = true
# Coast/resume hysteresis on the EMA-filtered cadence (tau below): above
# coast_above_rpm the rider stops pressing, pedalling resumes below
# resume_below_rpm. RU: накат выше 120, возврат ниже 105; EMA убирает
# дёрганье от двух пульсаций момента за оборот.
coast_above_rpm = 120.0
resume_below_rpm = 105.0
coast_cadence_tau_s = 0.35
stop_time_s = 0.35
# Legacy low-cadence ramp; inactive while seated_climb supplies the effort.
mash_cadence_rpm = 45.0
mash_torque_nm = 60.0
effort_slew_nm_s = 300.0
# Rider's hand on the brake during rollback (until hill-hold D4 replaces it).
# RU: при откате быстрее 0.25 м/с райдер зажимает тормоза до остановки отката.
rollback_brake = true
rollback_engage_mps = 0.25
rollback_release_mps = 0.0
rollback_demand = 1.0

[drive.assist]
# Bosch Performance Line CX Gen 4 (no ABS), recalled from memory, unverified:
# 85 N*m, 600 W peak, 40 ms torque lag, Eco 0.6 / Tour 1.4 / eMTB 1.4-3.4 /
# Turbo 3.4, taper 23-25 km/h. The profile owns those numbers; only the
# cadence envelope and the engagement threshold are declared here.
# RU: режим ассиста: eco | tour | emtb | turbo. Мотор = множитель x момент
# на педалях через лаг 40 мс; ассист только при шатуне вперёд > 5 град/с и
# моменте стопы > engage_torque_nm. Stall-отсечки и boost нет.
profile = "bosch_cx_gen4"
mode = "turbo"
# Torque envelope vs crank rpm (crank == motor shaft with the rigid
# chainring). Plateau documents capability; above 120 rpm support declines
# to zero at 180 rpm.
torque_curve = [[0.0, 85.0], [120.0, 85.0], [120.1, 47.7], [180.0, 0.0]]
# Sensor noise floor: measured pedal torque below this is "not pedalling".
engage_torque_nm = 4.0

[drive.battery]
# No energy store: electrical power is still metered, but torque is never
# energy-limited and the reserve is never depleted.
enabled = false

[drive.gearing]
front_teeth = 22
rear_teeth = 51

[drive.shifting]
enabled = true
cassette = [10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51]
# RU: полоса каденса шатуна. Ниже 75 — на большую звезду (легче), выше 110 —
# на меньшую (тяжелее). Решение только по каденсу шатуна (EMA ниже);
# колёсный каденс используется лишь для проверки посадки на новую передачу.
target_cadence_min_rpm = 75.0
target_cadence_max_rpm = 110.0
shift_cooldown_s = 0.4
# RU: на время переключения райдер сбрасывает усилие до 30%; мотор НЕ
# режется — он следует за моментом райдера через свой лаг (спека S6).
shift_cut_duration_s = 0.2
torque_factor = 0.3
cadence_smoothing_tau_s = 0.35
upshift_slip_limit_mps = 0.5
```

В `[articulated]` добавить после `active_positive_power_limit_w = 450.0`:

```toml
# Two-leg torque waveform depth mean*(1 + 0.5 cos 2phi): peak ~1.5x mean,
# dead spots ~0.5x. The bike's flywheel carries the cranks through the dead
# spots now that crank and chainring are rigid.
pedal_torque_ripple = 0.5
# Residual recovery-leg load on the rising pedal (recreational rider ~40-100 N,
# lower bound used; engineering estimate). Keeps the welded flat-pedal foot in
# compression -- a real foot cannot pull a flat pedal.
return_foot_preload_n = 40.0
```

`[seated_climb]`:

```toml
[seated_climb]
enabled = true
# Rider holds ~250 W at the crank, torque-capped at 60 N*m (grinds below ~40 rpm).
target_crank_power_w = 250.0
max_crank_torque_nm = 60.0
```

Удалить `human_torque_nm = 20.0` и `gain = 4.0` (если ещё остались).

- [ ] **Step 3: Тесты**

Run: `uv run python -m pytest tests/reference/test_welded_profile_config.py -q && uv run python -m pytest tests -m 'not slow' -q`
Expected: PASS. Затем один медленный смоук: `uv run python -m pytest tests/test_rider_welds.py -q -m slow -k torque_sensor` — PASS (сенсор остался weld-моментом с лагом в шаг).

- [ ] **Step 4: Commit**

```bash
git add examples/research/viewer_physics_welded.toml tests/reference/test_welded_profile_config.py
git commit -m "feat: make the realistic pedelec drive the welded default"
```

---

### Task 9: Приёмочные прогоны

**Files:**
- Test: `tests/reference/test_realistic_pedelec_acceptance.py`

Критерии из интервью (Q6): (1) ровный старт 0→20 км/ч: каденс в полосе, сенсор > 0 на ≥ 80 % цикла; (2) 15 % подъём: скорость не ниже ~5 км/ч, без rocking stall; (3) момент мотора без пилы 3↔45 Н·м. 34 % вне скоупа. Запускать строго по одному (`-p no:xdist`, не параллелить с другими прогонами).

- [ ] **Step 1: Написать тесты**

```python
"""Acceptance: the rider stays engaged and the motor follows, on the flat and on 15 %."""
from pathlib import Path

import numpy as np
import pytest

from bike_sim.cli import research as research_cli
from bike_sim.sim.ride.control import RideControl

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT/'examples'/'research'/'viewer_physics_welded.toml'
SAVAGE = ROOT/'examples'/'research'/'rough_uphill_savage.toml'
KMH = 1/3.6


def _config(tmp_path):
    """Welded profile with absolute asset paths. Every ride starts in 22/51: on the
    flat the rider spins out and must shift up — that is the shifter's job to prove."""
    text = WELDED.read_text()
    for name in ('joint_envelope_path', 'joint_strength_path'):
        text = text.replace(f'{name} = "', f'{name} = "{WELDED.parent}/')
    path = tmp_path/'physics.toml'
    path.write_text(text)
    return path


def _environment(tmp_path, track, duration_s):
    args = research_cli.parser().parse_args([
        '--physics-config', str(_config(tmp_path)), '--track-file', str(track),
        '--duration', str(duration_s), '--dt', '.00125', '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--record-decimation', '8',
        '--initial-speed', '0', '--out', str(tmp_path/'out')])
    return research_cli.make_environment(args)


def _ride(env):
    """Automatic rider (seated_climb) and pedelec assist; returns per-step rows."""
    rows = []
    distance = 0.
    dt = env.control_steps*env.dt_s      # env.step advances one control period
    while not env.done:
        env.step(RideControl())
        drive = env.sim.physical.sample.channels['drive']
        speed = float(env.sim.speed_mps)
        distance += speed*dt
        rows.append(dict(t=env.sim.time_s, v=speed, x=distance,
                         cadence=drive['cadence_rpm'], human=drive['human_sensor_nm'],
                         motor=drive['motor_torque_nm'], mode=drive['rider_mode'],
                         shift=drive['shift_active'], gear=drive['gear_rear_teeth'],
                         shifts=drive['shift_count'], engaged=drive['motor_freewheel_engaged']))
    return rows


def _track(tmp_path, knots, length):
    track = tmp_path/'track.toml'
    track.write_text(f'name = "probe"\nlength_m = {length}\nsurface = "hardpack"\n'
                     f'grade_profile = {{ knots = {knots} }}\n')
    return track


@pytest.mark.slow
def test_flat_start_reaches_20_kmh_with_the_rider_engaged(tmp_path):
    env = _environment(tmp_path, _track(tmp_path, [[0., 0.], [300., 0.]], 300.), 20.)
    rows = _ride(env)
    assert env.reason == 'duration', env.reason
    v = np.array([r['v'] for r in rows])
    assert v.max() >= 20.*KMH
    # Launching in 22/51 the cranks spin out within a second; the rider must
    # answer with a run of upshifts (one per 0.4 s cooldown), coasting between
    # clicks like a real rider. By 6 s the gear has to be landed.
    launch = [r for r in rows if r['t'] <= 6.]
    assert launch[-1]['shifts'] >= 5, launch[-1]['shifts']
    assert launch[-1]['gear'] <= 24, launch[-1]['gear']
    settled = [r for r in rows if r['t'] >= 6.]
    assert np.mean([r['mode'] == 'coasting' for r in settled]) <= .15
    # ~100 rows/s (10 ms control period): judge pedalling below the assist taper.
    steady = [r for r in settled if r['v'] < 23.*KMH and r['mode'] == 'pedaling']
    assert len(steady) > 200
    human = np.array([r['human'] for r in steady])
    cadence = np.array([r['cadence'] for r in steady])
    assert np.mean(human > 0.) >= .8, np.mean(human > 0.)
    assert np.mean((cadence >= 60.) & (cadence <= 115.)) >= .8, (cadence.min(), cadence.max())
    assert np.mean(cadence) >= 70.
    # Motor smoothness: pulses with the legs, no 3<->45 N.m sawtooth
    motor = np.array([r['motor'] for r in steady if r['v'] < 20.*KMH])
    assert motor.mean() > 20.
    assert np.std(motor)/motor.mean() < .6


@pytest.mark.slow
def test_motor_follows_the_rider_through_a_shift_without_an_extra_cut(tmp_path):
    """The rider unloads to 30 % for 0.2 s; the motor follows that through its lag
    (spec S6) but is never cut on top of it: motor/sensor stays near the Turbo gain."""
    env = _environment(tmp_path, _track(tmp_path, [[0., 0.], [300., 0.]], 300.), 12.)
    rows = _ride(env)
    shifts = [i for i in range(1, len(rows)) if rows[i]['shift'] and not rows[i-1]['shift']]
    assert len(shifts) >= 5, 'the 22/51 flat launch must produce a run of upshifts'
    checked = 0
    for i in shifts:
        window = [r for r in rows[i:i+20] if r['shift'] and r['v'] < 20.*KMH]   # the 0.2 s cut
        human = np.mean([max(r['human'], 0.) for r in window]) if window else 0.
        if human < 4.:
            continue
        motor = np.mean([r['motor'] for r in window])
        assert motor/human >= 2.5, (rows[i]['t'], human, motor)   # 3.4 nominal, lag only
        checked += 1
    assert checked >= 1


@pytest.mark.slow
def test_fifteen_percent_climb_holds_speed_without_a_dead_spot_stall(tmp_path):
    knots = [[0., 0.], [10., 0.], [14., .15], [150., .15]]
    env = _environment(tmp_path, _track(tmp_path, knots, 150.), 25.)
    rows = _ride(env)
    assert env.reason == 'duration', env.reason
    climbing = [r for r in rows if r['t'] >= 10.]
    v = np.array([r['v'] for r in climbing])
    assert v.min() >= 5.*KMH, v.min()/KMH
    cadence = np.array([r['cadence'] for r in climbing])
    assert cadence.min() > 20., cadence.min()
    assert np.mean(np.array([r['human'] for r in climbing]) > 0.) >= .8


@pytest.mark.slow
def test_savage_fifteen_percent_plateau_is_ridden_above_5_kmh(tmp_path):
    env = _environment(tmp_path, SAVAGE, 22.)
    rows = _ride(env)
    plateau = [r for r in rows if 17. <= r['x'] <= 33.]
    assert plateau, 'did not reach the 15 % plateau'
    v = np.array([r['v'] for r in plateau])
    assert v.min() >= 4.*KMH, v.min()/KMH          # rough surface: 1 km/h margin
    cadence = np.array([r['cadence'] for r in plateau])
    assert np.mean(cadence > 20.) >= .95
```

Если какой-то ключ канала называется иначе (`shift_active`, `rider_mode`, `motor_freewheel_engaged`), сверить с `DrivetrainForceApplier.last` в `drivetrain_forces.py` и `_shift_diagnostics`, не придумывать новых.

- [ ] **Step 2: Запустить по одному**

```bash
uv run python -m pytest tests/reference/test_realistic_pedelec_acceptance.py -q -m slow -k flat_start
uv run python -m pytest tests/reference/test_realistic_pedelec_acceptance.py -q -m slow -k extra_cut
uv run python -m pytest tests/reference/test_realistic_pedelec_acceptance.py -q -m slow -k fifteen_percent_climb
uv run python -m pytest tests/reference/test_realistic_pedelec_acceptance.py -q -m slow -k savage
```
Expected: PASS каждый (1–3 мин на прогон). Любой FAIL — диагностировать по CSV в `tmp_path/out` (`preview.csv`: `cadence_rpm`, `human_sensor_nm`, `motor_torque_nm`, `motor_freewheel_engaged`, `gear_rear_teeth`). Пороги не ослаблять без явной записи причины в ledger; типичные причины и куда смотреть:
  - сенсор ≈ 0 при вращении → ноги не давят: проверить `human_command_nm` (SeatedClimb выдал 0?) и `rider_joints_saturated`;
  - каденс ниже 60 на ровном → шифтер не переключает вверх: `shift_count`, `upshift_slip_limit`;
  - шифтер не успевает за разгоном в 22/51 (меньше 5 апшифтов к 6 с, передача > 24T) → проверить, что `prepare_pedaling` передаёт `pedaling=True` во время наката по каденсу (командное усилие от seated_climb > 0) и что landing-проверка не отвергает апшифт из-за `required_ema`, завышенного стартовой передачей; cooldown 0.4 с не трогать без записи в ledger;
  - пила мотора → `assist_gain`, `engage_torque_nm` против амплитуды сенсора в мёртвых точках (при ripple 0.5 минимум сенсора = 0.5×среднего > 4 Н·м при среднем > 8 Н·м).

- [ ] **Step 3: Commit**

```bash
git add tests/reference/test_realistic_pedelec_acceptance.py
git commit -m "test: acceptance rides for the realistic pedelec drive"
```

---

### Task 10: Закрыть слайс в ledger и пересчитать базу

**Files:**
- Modify: `.superpowers/sdd/2026-10-03-v2-seated-plant/progress.md`
- Modify: `docs/superpowers/plans/2026-10-03-v2-05-release.md` (W3: пометить reposition-часть выполненной)

- [ ] **Step 1: Полный быстрый прогон и медленные приёмки**

```bash
uv run python -m pytest tests -m 'not slow' -q
uv run python -m pytest tests/reference/test_motor_freewheel_topology.py tests/reference/test_realistic_pedelec_acceptance.py tests/reference/test_seated_pedaling_cycle.py -q -m slow
```
Записать числа passed/failed.

- [ ] **Step 2: Ledger.** Заменить строку `Slice status: Task 0 docs — in progress.` на:

```markdown
Slice status (2026-10-04): Tasks 0–9 committed: <hashes>. Non-slow suite: <N passed>. Slow acceptance: flat 0→20 km/h, shift-no-cut, synthetic 15 %, savage 15 % plateau — <results>. D1-assist DONE; D1-sensors PENDING; D3' DONE (engagement_energy_loss oracle pending); W3-reposition DONE; W3-initializer PENDING; R9 DONE. Legacy motor_clutch=true kept until W6; delete there. G5 constraint-work attribution must be re-run on the rigid topology before W6.
```

- [ ] **Step 3: W3 в `2026-10-03-v2-05-release.md`**: у пункта про удаление `reposition_*` добавить `(выполнено планом 2026-10-04-realistic-pedelec-drive.md Task 4)`.

- [ ] **Step 4: Commit**

```bash
git add .superpowers/sdd/2026-10-03-v2-seated-plant/progress.md docs/superpowers/plans/2026-10-03-v2-05-release.md
git commit -m "docs: record the realistic pedelec slice status"
```

---

## Self-review

**Покрытие решений интервью (Q1–Q18):** Q1/Q8/Q16 топология — Task 3 + Task 8; Q2 профиль/режим/Turbo — Tasks 1, 2, 8; Q3 лаг 40 мс + gate — Task 2; Q4/Q12 райдер (250 Вт, 60 Н·м, 120/105, ripple 0.5) — Tasks 6, 8; Q5/Q12 конверт — Task 7; Q6 приёмка — Task 9; Q7 дефолт welded, legacy за флагом — Tasks 3, 8; Q9 правка спеки — Task 0; Q10 reposition/rollback/legacy — Tasks 4, 8, 10; Q11 coast EMA + остаточная нагрузка возвратной ноги 40 Н — Tasks 6, 8; Q13 stall-cut — Task 2; Q14/Q18 шифтер 75–110, мотор не режется — Tasks 5, 8; Q15 место в v2 — Tasks 0, 10; Q17 мотор при шифте по спеке — Task 5.

**Согласованность имён:** `rotor_inertia_kgm2`, `motor_rotor`, `rotor_spin`, `motor_freewheel`, `DrivetrainForceApplier.freewheel`, `motor_freewheel_engaged` — Task 3, Task 8, Task 9. `coast_cadence_tau_s` — Tasks 6, 8. `pedal_torque_ripple` / `pedal_torque_waveform` — Tasks 6, 8. `AssistController(profile, mode, engage_torque_nm, gate_min_crank_rad_s)` / `.last_gain` — Tasks 2, 8, 9 (`assist_gain` диагностика). `pedelec_cap(human_nm, crank_rad_s, external_cap_nm, *, braking, gate_min_crank_rad_s)` — Tasks 1, 2.

**Не входит в этот план (остаётся в v2):** D1-датчики с маской, D2 interlock/превью, D4 hill-hold, W3 инициализатор, тепловая модель мотора, удаление legacy `motor_clutch` (W6).
