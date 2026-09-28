# Task C1 brief — immutable physical contact snapshot and absolute point velocity

Source: `docs/superpowers/plans/2026-09-28-03-tire-contact.md`, Task C1. The physics-correctness specification is authoritative when a conflict appears.

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

## Task C1: physical snapshot and absolute point velocity

**Files:** Create `src/bike_sim/sim/ride/contact_state.py`, `src/bike_sim/sim/ride/wheel_kinematics.py`; modify `src/bike_sim/sim/ride/contacts.py:217-231`; test `tests/test_contact_kinematics.py`.

**Interfaces:** Produce `ContactPatch(point_m,normal,normal_load_n,tangent_force_n,slip_mps)` and `point_velocity(v_center,omega_world,offset)`. `WheelContactSnapshot` has a timestamp, patches, `geometric_contact`, and computed aggregates. A frozen snapshot must not retain references to mutable MuJoCo buffers.

### Required behavior and acceptance

1. Add the requested independent sign/decomposition tests:

```python
import numpy as np
import pytest
from bike_sim.sim.ride.wheel_kinematics import point_velocity
from bike_sim.sim.ride.contact_state import ContactPatch

def test_no_slip_uses_world_wheel_velocity():
    v = point_velocity(np.array([3.5,0.,0.]), np.array([0.,10.,0.]), np.array([0.,0.,-0.35]))
    np.testing.assert_allclose(v, np.zeros(3), atol=1e-12)

def test_slope_keeps_normal_and_vertical_force_distinct():
    n = np.array([-0.5,0.,np.sqrt(3)/2])
    patch = ContactPatch(np.zeros(3), n, 100.0, 20.0, 0.0)
    assert patch.normal_load_n == 100.0
    assert patch.world_force_n[2] == pytest.approx(100*np.sqrt(3)/2+10)
```

2. `point_velocity(v_center,omega_world,offset)` returns `v_center + cross(omega_world,offset)` after validating three finite shape-(3,) vectors.

3. Implement immutable `ContactPatch` semantics. Copy input point/normal into finite shape-(3,) arrays, require a unit normal within `1e-8`, require `normal_y` within `1e-8` of zero, require finite scalar load/force/slip and nonnegative normal load, then mark copied arrays read-only. `tangent` is `[normal_z,0,-normal_x]`; `world_force_n` is `normal_load_n*normal + tangent_force_n*tangent`.

4. `WheelContactSnapshot` must satisfy **all** TIRE-01 fields, including the plan's base fields and the phase-ledger additions:
   - finite `time_s >= 0` and a stable interval ID;
   - backend identifier;
   - `patches` as an immutable tuple of `ContactPatch`;
   - `geometric_contact` and separate loaded-contact state;
   - aggregate `normal_load_n`, `world_force_n`, `vertical_force_n`, `normal_vertical_n`, and `tangent_force_n`;
   - moment about wheel axis and effective radius;
   - касательная скорость/slip signal.

   Required aggregate definitions: `normal_load_n=sum(Fn_i)`; `world_force_n=sum(F_i)`; `vertical_force_n=world_force_n[2]`; `normal_vertical_n=sum(Fn_i*n_i[2])`; longitudinal `tangent_force_n` follows the defined X-Z tangent. Never collapse normal load and vertical resultant into one `support_n` value. A patch tuple input is copied/coerced to a tuple; empty patches return zero force/load aggregates. All exposed vectors are independent from mutable MuJoCo storage and read-only.

5. In native contact querying, read all components of `mj_contactForce`, transform the contact-frame force into world coordinates with `contact.frame.reshape(3,3).T @ force[:3]`, and account for tracked wheel geom being either geom1 or geom2 so force sign is correct. Preserve the vertical component of tangential force on slopes. The wheel-point velocity must be absolute/world velocity: use `mj_objectVelocity` with correct world-frame semantics and known wheel center, then transport it to the actual contact/axis point when centers differ. Keep relative hinge speed separately named; do not use it as ground slip.

6. Add or adapt legacy callers with compatible snapshot defaults/adapters so existing native controller behavior does not break. C2 alone owns timestamp-based controller filtering; C1 does not let a filtered bool change physical normal load/force.

7. Verification must separately exercise:
   - swapped `geom1` / `geom2` contact ordering and force sign;
   - a wheel parent rotating together with the wheel at zero relative hinge speed, proving nonzero absolute point velocity is used;
   - existing controller regressions.

## Controller resolution of C1 review findings

- Keep a copied exact `world_force_n` vector from each native MuJoCo row; do not reconstruct/alter the required TIRE-01 world resultant from a projected X-Z normal. The projected normal and longitudinal tangent remain separate planar decomposition fields. Add a regression with a materially out-of-plane raw normal/force to prove the snapshot resultant equals the complete transformed wrench.
- Preserve whether a patch came from the working `terrain` or the emergency `catch_plane` (or an equivalent explicit working-surface flag). Expose road-only loaded contact separately so C2/physical drive gating cannot treat catch-plane load as ordinary road. Do not change legacy bridged `TerrainContacts` load behavior in this C1 fix round; preserve legacy output while exposing the classification for physical consumers.
- Keep the two snapshot construction paths as-is unless the fix naturally requires a shared helper; the reviewer called that a nonblocking duplication observation.

### Verification and commit

- Red: `uv run --locked pytest tests/test_contact_kinematics.py -q`; expected missing modules.
- Focused: `uv run --locked pytest tests/test_contact_kinematics.py tests/test_ride_controllers.py -q`.
- Commit only C1 implementation/tests with message: `feat: expose physical contact wrench and wheel kinematics`.

## Prior interfaces and decisions

- Phase 01 A1/A2 are reviewed; the physical step uses one named force accumulator and preserves separate writer ownership.
- Phase 02 B1/B2 are reviewed. `physics_revision` is `legacy-v1` vs `physical-v1`; wheel mass/inertia is one explicit owner with synthetic provenance.
- Phase 03 keeps `native_reference` separate from the future `compliant_2d` backend. C1 describes contact state and native querying only; C5 owns compliant geometry and force application.
- The phase ledger additionally requires interval ID, backend, wheel-axis moment, effective radius, and loaded-contact state because TIRE-01 lists these fields even though the minimal plan dataclass omitted them.
