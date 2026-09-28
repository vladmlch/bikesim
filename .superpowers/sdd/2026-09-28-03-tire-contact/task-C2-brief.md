# Task C2 brief — timestamp-based grounded filter

Source: `docs/superpowers/plans/2026-09-28-03-tire-contact.md`, Task C2. The physics-correctness specification is authoritative when a conflict appears.

## Global constraints

- Базовая ревизия: 70623815b98788018bcdbd8eef347d778f9bb3f3.
- Python >=3.12; использовать существующий uv.lock, не обновлять зависимости в PR физики.
- Новые численные зависимости не добавлять: NumPy, SciPy, MuJoCo и pytest уже объявлены проектом.
- Новая физика использует SI: m, s, kg, N, N*m, rad; mm и km/h допустимы только на совместимых внешних границах.
- Новая физика остаётся плоской X-Z; боковое сцепление и баланс по крену не заявляются.
- В режиме physical запрещены внешняя стабилизация тангажа, ручной перенос веса и присваивание qpos/qvel после шага для исправления физики.
- legacy и physical имеют разные physics_revision и разные численные эталоны.
- Параметры без измерений помечаются synthetic; прохождение синтетических тестов не считается экспериментальной валидацией.
- Все новые численные допуски являются критериями приёмки, а не результатами уже выполненных испытаний.

## Task C2: time-based grounded filter without substituting load

**Files:** Create `src/bike_sim/sim/ride/contact_filter.py`; modify `src/bike_sim/sim/ride/contacts.py:234-269`; test `tests/test_contact_filter_time.py`.

**Interfaces:** Produce `GroundedFilter(hold_s).update(raw_grounded,time_s)->bool` and `reset()`. The filter neither accepts nor returns normal load.

### Required behavior and acceptance

1. Add the idempotent timestamp example:

```python
from bike_sim.sim.ride.contact_filter import GroundedFilter

def test_filter_does_not_advance_when_read_twice():
    f = GroundedFilter(0.005)
    assert f.update(True, 0.0)
    assert f.update(False, 0.004)
    assert f.update(False, 0.004)
    assert not f.update(False, 0.006)
```

2. Validate `hold_s` as finite and nonnegative. `reset()` clears timestamps, last-loaded time, and output. `update` rejects non-finite time and backward time (caller must reset first). A repeated equal timestamp returns the previous filter value without advancing state. New timestamps preserve grounded while raw-grounded or for less than `hold_s` after the last loaded timestamp.

Reference behavior:

```python
class GroundedFilter:
    def __init__(self, hold_s):
        if not isfinite(hold_s) or hold_s < 0:
            raise ValueError("hold_s must be finite and nonnegative")
        self.hold_s = hold_s
        self.reset()

    def reset(self):
        self.last_time = None
        self.last_loaded = None
        self.value = False

    def update(self, raw_grounded, time_s):
        if not isfinite(time_s):
            raise ValueError("non-finite timestamp")
        if self.last_time is not None and time_s < self.last_time:
            raise ValueError("timestamp moved backwards; reset required")
        if time_s == self.last_time:
            return self.value
        self.last_time = time_s
        if raw_grounded:
            self.last_loaded = time_s
        self.value = bool(raw_grounded or (self.last_loaded is not None and
                          time_s-self.last_loaded < self.hold_s))
        return self.value
```

3. Physical rolling resistance and all tire contact forces use only raw normal load/contact. The filter's bool may gate native-reference controllers only and must be clearly named `controller_grounded`; it never replaces/creates a load or physical force. `catch_plane` crash detection stays outside the grounded filter. Resetting a simulation must reset the filter.

## Controller ruling on C2 integration scope

Keep the existing legacy `TerrainContacts.front_load_n` / `rear_load_n`, `front_support_n` / `rear_support_n`, `front_in_contact` / `rear_in_contact`, HUD, recorder, and pneumatic behavior unchanged for compatibility. Add separate `front_controller_grounded` / `rear_controller_grounded` outputs from raw working-road contact and time. The physical native-reference cruise/drivetrain gates must explicitly consume these booleans; physical tire force, Crr, raw load/support, and airtime metrics must not. C2 may extend to the narrow `RideSimulation`/`CruiseController` call sites needed to pass the bool; do not redirect legacy consumers or broad telemetry. `TerrainContactQuery.reset()` resets the time filters as well as its compatibility bridge.

Construct raw controller-grounded input from working-road normal load above the existing `CONTACT_LOAD_THRESHOLD_N` (`1.0 N`), not from a geometric row alone. This preserves the existing margin/noise threshold; the timer may bridge short false gaps but may not synthesize load.

The current legacy recorder/summary contact columns remain compatible in C2. They are not valid physical airtime metrics: F1/F4 must record physical raw `road_loaded_contact` plus threshold sensitivity using schema-v2/physical channels. Do not claim TIRE-01 airtime compliance from the legacy bridged fields.

4. Verify one time sequence at `dt=0.5`, `0.25`, and `0.125` ms; hold duration agrees within one step.

### Verification and commit

- Red: `uv run --locked pytest tests/test_contact_filter_time.py -q`; expected missing module.
- Focused: `uv run --locked pytest tests/test_contact_filter_time.py -q`.
- Commit only C2 implementation/tests with message: `fix: separate time-based grounded filter from physical loads`.

## Prior interfaces and decisions

- C1 provides raw immutable contact snapshots including distinct geometric and loaded-contact signals; C2 may consume only the raw grounded bool/time, never contact-force values.
- Keep controller debounce independent from physical metrics/airtime. Spec TIRE-01 requires raw airtime and separate threshold sensitivity.
- Preserve `physics_revision` and legacy behavior; do not let a filtered signal alter physical tyre forces, load, resistance, or contact geometry.
