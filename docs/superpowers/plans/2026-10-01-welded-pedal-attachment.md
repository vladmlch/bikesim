# Welded Pedal Attachment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the `articulated_planar` rider an optional clipless-pedal mode where both feet are permanently welded to the pedal bodies, replacing the unilateral pad contacts.

**Architecture:** A new `articulated.pedal_attachment` config key (`"flat"` default, `"weld"` new). In weld mode the MJCF builder emits two `weld` equalities (`rider_foot_*` ↔ `pedal_*`); a new `PedalWelds` reader maps solved constraint multipliers into the existing pedal diagnostics and `delivered_crank_torque_nm`; the contact applier skips foot pads and the controller aims soles at the weld datum instead of footprint projections.

**Tech Stack:** Python 3 + MuJoCo (`mujoco` package), `attrs`-style dataclasses for config, ElementTree MJCF generation, pytest. All commands via `uv run`.

**Spec:** `docs/superpowers/specs/2026-10-01-welded-pedal-attachment-design.md`

## Global Constraints

- Every Python/pytest invocation uses `uv run` (per `AGENTS.md`).
- Config key: `[articulated] pedal_attachment = "flat" | "weld"`, default `"flat"`; unknown values rejected with `ValueError`.
- Weld emission only for `physics_mode="physical"` + `rider="articulated_planar"`; otherwise raise `ValueError`.
- Weld is non-breakable; `body1=rider_foot_{side}`, `body2=`pedal_{side}`; no `relpose` (datum = qpos0 relative pose).
- Weld `solref` = `max(2*timestep_s, closure_time_constant_s)` with damping ratio `1` — numerical stabilization only; add no physical damping, springs, or dampers.
- Sole-force requests stay push-only: `feasible_pedal_force` friction-cone clamp unchanged; the weld supplies retention as a reaction.
- Weld mode keeps the rider seated; saddle and grip pads are unchanged.
- `PedalRecovery` must never advance past `'none'` in weld mode.
- Diagnostics schema is preserved: `front_pedal`/`rear_pedal` keys keep `enabled`, `in_platform`, `normal_load_n`, `gap_m`, `force_on_rider_n`, `force_on_bike_n`, `patches`; `delivered_crank_torque_nm` stays the assist torque-sensor feed.
- Acceptance: weld translation residual < 3 mm while riding; zero pedal recovery events; `flat` mode bit-for-bit unchanged.
- Commit after every task; never commit generated `verification/` artifacts.

---

## File Structure

**Create:**

| File | Responsibility |
|---|---|
| `src/bike_sim/sim/ride/weld_pedals.py` | `PedalWelds` — reads solved weld equality rows (`efc_*`) and exposes world-frame force on the foot, translation residual, and crank torque. Stateless reader; one instance per model. |
| `tests/test_welded_pedal_attachment.py` | All weld-mode tests + the shared `welded_rig` helper. |
| `examples/research/viewer_physics_welded.toml` | Copy of `viewer_physics_fast.toml` with `pedal_attachment = "weld"` — the user's interactive/headless welded profile. `viewer_physics_fast.toml` itself is NOT modified (keeps a flat-mode A/B baseline). |

**Modify:**

| File | What changes |
|---|---|
| `src/bike_sim/physics/physical_config.py` | `ArticulatedConfig`: new `pedal_attachment` field, excluded from the numeric `scalar()` sweep, membership-validated in `__post_init__`. |
| `src/bike_sim/mujoco/builder.py` | `generate_mujoco_xml` (~lines 315-334): guard + emit two `weld` equalities inside the existing `if articulated_pose is not None:` physical block. |
| `src/bike_sim/sim/ride/rider_contacts.py` | `RiderContactApplier`: `__init__` stores `welded_pedals`/`PedalWelds`; `set_enabled` refuses pedal release; `compute_qfrc` pedal branch reads weld reactions instead of pads; `stored_energy` skips welded pedals; `delivered_crank_torque_nm` sourced from welds. |
| `src/bike_sim/sim/ride/rider_control.py` | `ArticulatedRiderController`: `__init__` computes per-side sole offsets in the pedal frame from `target_data`; `_targets` weld branch replaces `project_sole_goal`/clearance/recovery-goal logic with the weld anchor point; `compute` forces `clearance=0` and skips `PedalRecovery.observe`. |

**Deliberately untouched:** `physical_equilibrium.py` (the `pedal_spin = -phase` write stays weld-consistent because the weld datum keeps platform and sole level together), `physical_runtime.py` (all interfaces preserved), `drivetrain_forces.py`, `ideal_freehub.py`, `pedal_recovery.py`, `recorder.py`, `physical_observations.py`.

**Subsystem boundary:** builder owns topology; `weld_pedals.py` owns everything about reading constraint reactions; `rider_contacts.py` owns the diagnostics schema contract; `rider_control.py` owns goals/actuation. `physical_runtime.py` needs no changes because `RiderContactApplier`'s public surface (`compute_qfrc`, `diagnostics`, `probe_diagnostics`, `delivered_crank_torque_nm`, `set_enabled`, `release_all`, `stored_energy`) is unchanged.

---

### Task 1: `pedal_attachment` config field

**Files:**
- Modify: `src/bike_sim/physics/physical_config.py` (`ArticulatedConfig`, ~lines 396-439)
- Test: `tests/test_welded_pedal_attachment.py` (create)

**Interfaces:**
- Produces: `ArticulatedConfig.pedal_attachment: str` (`'flat'` or `'weld'`). Every later task reads `config.pedal_attachment == 'weld'`. `load_physics_config` exposes it as `[articulated] pedal_attachment` in TOML automatically (dataclass-driven resolution in `src/bike_sim/physics/resolution.py`).

- [ ] **Step 1: Write the failing tests**

Create `tests/test_welded_pedal_attachment.py`:

```python
"""Tests for articulated pedal_attachment='weld' (clipless pedal mode)."""
import mujoco
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import (
    ArticulatedConfig, PedalingConfig, PhysicalDriveConfig)
from bike_sim.physics.resolution import load_physics_config
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.rider_control import ArticulatedRiderController


def _config(attachment='weld', **kwargs):
    values = dict(drive_mode='articulated_effort', timestep_s=.00125,
                  articulated=ArticulatedConfig(pedal_attachment=attachment),
                  drive=PhysicalDriveConfig(
                      transmission_model='ideal_mid_drive',
                      human_torque_nm=20.))
    values.update(kwargs)
    return SimulationPhysicsConfig('physical', **values)


def welded_rig(transmission='ideal_mid_drive', human=20., speed=0.):
    """Compiled physical model + posed controller with welded feet."""
    specs = BikeSpecs()
    cfg = _config(initial_speed_mps=speed,
                  drive=PhysicalDriveConfig(transmission_model=transmission,
                                            human_torque_nm=human))
    rider = RiderSpecs(variant='articulated_planar')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=rider, physics_config=cfg))
    data = mujoco.MjData(model)
    controller = ArticulatedRiderController(
        model, geometry_pose(rider, specs), cfg.articulated,
        specs.crank_length / 1000.)
    controller.initialize(model, data)
    mujoco.mj_forward(model, data)
    return model, data, controller, cfg


def test_pedal_attachment_defaults_to_flat():
    assert ArticulatedConfig().pedal_attachment == 'flat'


def test_pedal_attachment_accepts_weld():
    assert ArticulatedConfig(pedal_attachment='weld').pedal_attachment == 'weld'


def test_pedal_attachment_rejects_unknown_values():
    with pytest.raises(ValueError, match='pedal_attachment'):
        ArticulatedConfig(pedal_attachment='clipless')


def test_physics_toml_loads_articulated_pedal_attachment(tmp_path):
    path = tmp_path / 'physics.toml'
    path.write_text(
        'physics_mode = "physical"\n'
        'drive_mode = "articulated_effort"\n'
        'timestep_s = 0.00125\n'
        '[articulated]\n'
        'pedal_attachment = "weld"\n')
    cfg = load_physics_config(str(path), {})
    assert cfg.articulated.pedal_attachment == 'weld'
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -x -q`
Expected: FAIL — `ArticulatedConfig() got an unexpected keyword argument 'pedal_attachment'` (and the TOML test fails on the same field being unknown).

- [ ] **Step 3: Add the field and validation**

In `src/bike_sim/physics/physical_config.py`, inside `ArticulatedConfig` add as the last field (after `foot_sensors_enabled`):

```python
    pedal_attachment: str = 'flat'
```

In `ArticulatedConfig.__post_init__`, extend the string-field skip:

```python
        for key in self.__dataclass_fields__:
            if key in ('joint_envelope_path', 'pedal_attachment'):
                continue
```

then append after the loop (before `self._envelopes = ...` if present, else at the end of `__post_init__`):

```python
        if self.pedal_attachment not in ('flat', 'weld'):
            raise ValueError(
                f"pedal_attachment must be 'flat' or 'weld', got {self.pedal_attachment!r}")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -x -q`
Expected: the four config tests PASS; rig-based tests are not present yet.

- [ ] **Step 5: Commit**

```bash
git add tests/test_welded_pedal_attachment.py src/bike_sim/physics/physical_config.py
git commit -m "feat: add articulated pedal_attachment config (flat|weld)"
```

---

### Task 2: Emit foot↔pedal welds in the MJCF

**Files:**
- Modify: `src/bike_sim/mujoco/builder.py` (`generate_mujoco_xml`, physical block ~lines 315-334)
- Test: `tests/test_welded_pedal_attachment.py` (append)

**Interfaces:**
- Consumes: `ArticulatedConfig.pedal_attachment` (Task 1); `physics_config.timestep_s`, `physics_config.closure_time_constant_s`; `root`/`equality`/`frame`/`articulated_pose` already in `generate_mujoco_xml`.
- Produces: equalities named `weld_foot_front`/`weld_foot_rear` of type `mjEQ_WELD` joining `rider_foot_{side}` (body1) to `pedal_{side}` (body2). Task 3 resolves these names.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_welded_pedal_attachment.py`:

```python
def _equality_names(model):
    return {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
            for i in range(model.neq)}


def test_weld_mode_emits_foot_pedal_equalities():
    model, _, _, _ = welded_rig()
    for side in ('front', 'rear'):
        eq = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                   f'weld_foot_{side}'))
        assert eq >= 0
        assert model.eq_type[eq] == mujoco.mjtEq.mjEQ_WELD
        foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                 f'rider_foot_{side}')
        pedal = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                  f'pedal_{side}')
        assert {int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])} == {foot, pedal}
        # solref[0] = closure time constant: >= 2*dt, critically damped row.
        assert model.eq_solref[eq][0] == pytest.approx(
            max(2. * model.opt.timestep, .0025))
        assert model.eq_solref[eq][1] == pytest.approx(1.)


def test_flat_mode_keeps_physical_feet_unwelded():
    cfg = _config('flat')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=RiderSpecs(variant='articulated_planar'),
        physics_config=cfg))
    assert not any(name.startswith('weld_foot_')
                   for name in _equality_names(model) if name)


def test_weld_mode_rejects_non_articulated_riders():
    with pytest.raises((ValueError, RuntimeError)):
        generate_mujoco_xml(mode='ride', rider=RiderSpecs(variant='lumped'),
                            physics_config=_config('weld'))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -k 'weld_mode or flat_mode' -x -q`
Expected: FAIL — `mj_name2id` returns `-1` (no `weld_foot_*` equalities emitted yet).

- [ ] **Step 3: Emit the welds in the builder**

In `src/bike_sim/mujoco/builder.py`, inside `generate_mujoco_xml`, at the start of `if physical:` (line ~315):

```python
    if physical:
        if (physics_config.articulated.pedal_attachment == 'weld'
                and articulated_pose is None):
            raise ValueError(
                "pedal_attachment='weld' requires rider='articulated_planar'")
```

Then, still inside `if articulated_pose is not None:` (line ~318), immediately after `add_rider_actuators(root, physics_config.articulated)`:

```python
            if physics_config.articulated.pedal_attachment == 'weld':
                solref = max(2. * physics_config.timestep_s,
                             physics_config.closure_time_constant_s)
                equality = root.find('equality')
                assert equality is not None
                for side in ('front', 'rear'):
                    ET.SubElement(equality, 'weld', {
                        'name': f'weld_foot_{side}',
                        'body1': f'rider_foot_{side}',
                        'body2': f'pedal_{side}',
                        'solref': f'{solref:.17g} 1',
                    })
```

(`build_equality_constraints` at line 280 always creates the `<equality>` element; `root.find` reuses it.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -x -q`
Expected: all seven tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/mujoco/builder.py tests/test_welded_pedal_attachment.py
git commit -m "feat: weld rider feet to pedals when pedal_attachment=weld"
```

---

### Task 3: `PedalWelds` — weld reaction reader

**Files:**
- Create: `src/bike_sim/sim/ride/weld_pedals.py`
- Test: `tests/test_welded_pedal_attachment.py` (append)

**Interfaces:**
- Consumes: weld equalities from Task 2; `data.efc_type`/`efc_id`/`efc_pos`/`efc_force` after `mj_forward`/`mj_step`.
- Produces (used by Task 4):
  - `PedalWelds(model)` — raises `ValueError` if a `weld_foot_*` equality is missing.
  - `force_on_rider_n(model, data, side) -> np.ndarray` — world-frame linear force the weld applies to the foot body (body1); `+z` = pedal pushes foot up.
  - `translation_residual_m(model, data, side) -> float` — norm of the weld's 3 positional residual rows, metres.
  - `delivered_crank_torque_nm(model, data) -> float` — total weld torque about `crank_spin`; positive spins the cranks forward.

Verified MuJoCo conventions (checked experimentally on a scratch two-body model): a weld equality produces 6 EFC rows of type `mjCNSTR_EQUALITY` with `efc_id` = equality index; rows 0-2 are the translation residual/force along world axes, rows 3-5 rotational; `efc_pos` equals `-(xpos_body2 - datum)` i.e. world-frame; the translational multipliers `λ[:3]` are the world force applied ON body1, so `-λ[:3]` acts on body2.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_welded_pedal_attachment.py`:

```python
def test_pedal_welds_reader_reports_forces_and_residual():
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    model, data, _, _ = welded_rig()
    welds = PedalWelds(model)
    foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                             'rider_foot_front')
    data.xfrc_applied[foot] = [0., 0., -300., 0., 0., 0.]
    for _ in range(20):
        mujoco.mj_step(model, data)
    force_on_rider = welds.force_on_rider_n(model, data, 'front')
    assert force_on_rider[2] > 0.          # weld pushes the loaded foot up
    assert welds.translation_residual_m(model, data, 'front') < .003
    # Downward force on the front pedal (arm at 3 o'clock at design pose)
    # must read as positive forward crank torque — the torque-sensor feed.
    assert welds.delivered_crank_torque_nm(model, data) > 0.


def test_pedal_welds_reader_is_quiet_when_unloaded():
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    model, data, _, _ = welded_rig()
    welds = PedalWelds(model)
    for side in ('front', 'rear'):
        assert welds.translation_residual_m(model, data, side) < .001
    assert abs(welds.delivered_crank_torque_nm(model, data)) < 5.
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -k welds_reader -x -q`
Expected: FAIL — `ModuleNotFoundError: bike_sim.sim.ride.weld_pedals`.

- [ ] **Step 3: Implement the module**

Create `src/bike_sim/sim/ride/weld_pedals.py`:

```python
"""Solved weld reactions for feet rigidly attached to pedals.

With articulated.pedal_attachment = 'weld' the MJCF builder emits one `weld`
equality per foot (body1 = rider_foot_*, body2 = pedal_*). This module reads
the solved constraint multipliers and reports them in the conventions
RiderContactApplier uses for pad supports.

Conventions (pinned by tests): a weld's six EFC rows are [x,y,z translation,
3 rotational]; the translational multipliers are the world-frame force applied
BY the weld ON body1 (the foot); the equal-and-opposite force acts on body2
(the pedal). efc_pos rows 0-2 are the world-frame translation residual.
"""
import numpy as np
import mujoco

from bike_sim.sim.ride.physical_mapping import resolve_id

SIDES = ('front', 'rear')
_EQUALITY = mujoco.mjtConstraint.mjCNSTR_EQUALITY


class PedalWelds:
    """Per-side weld reaction reader; reads the last solved constraint state."""

    def __init__(self, model):
        self.eq_ids = {side: resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                        f'weld_foot_{side}') for side in SIDES}
        self.crank_dof = int(model.joint('crank_spin').dofadr[0])

    @staticmethod
    def _mask(data, eq_id):
        n = data.nefc
        return ((data.efc_type[:n] == _EQUALITY)
                & (data.efc_id[:n] == eq_id))

    def force_on_rider_n(self, model, data, side):
        """World force the weld applies to the foot, in Newtons.

        +z is the pedal pushing the foot up (compressive). A negative z is the
        weld retaining the foot — the clipless pull the pad model cannot make.
        """
        mask = self._mask(data, self.eq_ids[side])
        if not np.any(mask):
            return np.zeros(3)
        lam = data.efc_force[:data.nefc][mask]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data, side):
        """Norm of the weld's positional residual — actual foot/pedal mismatch."""
        mask = self._mask(data, self.eq_ids[side])
        if not np.any(mask):
            return 0.
        pos = data.efc_pos[:data.nefc][mask]
        return float(np.linalg.norm(pos[:3]))

    def delivered_crank_torque_nm(self, model, data):
        """Total weld torque about crank_spin; + spins the cranks forward."""
        if data.nefc == 0:
            return 0.
        multipliers = np.zeros(data.nefc)
        nefc = data.nefc
        for eq in self.eq_ids.values():
            mask = self._mask(data, eq)
            multipliers[mask] = data.efc_force[:nefc][mask]
        qfrc = np.zeros(model.nv)
        mujoco.mj_mulJacTVec(model, data, qfrc, multipliers)
        return float(qfrc[self.crank_dof])
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -x -q`
Expected: PASS. If `force_on_rider[2]` comes out negative the λ sign convention is flipped on this MuJoCo version — swap the sign in `force_on_rider_n` and rerun; do not change the test.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/sim/ride/weld_pedals.py tests/test_welded_pedal_attachment.py
git commit -m "feat: read weld pedal reactions as rider contact forces"
```

---

### Task 4: `RiderContactApplier` weld mode

**Files:**
- Modify: `src/bike_sim/sim/ride/rider_contacts.py` (`__init__` ~line 52-77, `set_enabled` ~line 98, `compute_qfrc` pedal loop ~lines 205-260 and tail ~303, `stored_energy` ~line 172)
- Test: `tests/test_welded_pedal_attachment.py` (append)

**Interfaces:**
- Consumes: `PedalWelds` (Task 3) — `force_on_rider_n`, `translation_residual_m`, `delivered_crank_torque_nm`.
- Produces: identical public surface as before — `diagnostics`/`probe_diagnostics` keep the same keys; `delivered_crank_torque_nm` and `probe_delivered_crank_torque_nm` now weld-derived; `enabled` unchanged; pedal `set_enabled(False)` becomes a no-op returning `True`.

- [ ] **Step 1: Write the failing tests**

Append:

```python
def test_weld_mode_applier_reports_weld_diagnostics_and_no_pad_qfrc():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model, data, controller, cfg = welded_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    applier.initialize_settled_state(model, data)
    qfrc = applier.compute_qfrc(model, data, model.opt.timestep)
    diag = applier.diagnostics['front_pedal']
    assert diag['in_platform'] and diag['enabled']
    assert diag['normal_load_n'] >= 0.
    assert diag['gap_m'] < .003          # weld translation residual
    # The weld is solver-side: the applier must not double-apply its force.
    crank_dof = int(model.joint('crank_spin').dofadr[0])
    assert qfrc[crank_dof] == 0.
    # delivered_crank_torque_nm is weld-derived; saddle/grip entries survive.
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    assert applier.delivered_crank_torque_nm == pytest.approx(
        PedalWelds(model).delivered_crank_torque_nm(model, data))
    assert 'saddle' in applier.diagnostics and 'grip' in applier.diagnostics


def test_weld_mode_pedal_release_is_a_noop():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model, data, controller, cfg = welded_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    assert applier.set_enabled('front_pedal', False) is True
    assert applier.enabled['front_pedal']
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -k applier or -k pedal_release -x -q`
Expected: FAIL — `diag['gap_m']` is the pad gap (≈8 mm or penetration), `in_platform`/`qfrc`/`set_enabled` behave as in flat mode.

- [ ] **Step 3: Implement the weld branch**

In `src/bike_sim/sim/ride/rider_contacts.py`:

1. Top of file: `from bike_sim.sim.ride.weld_pedals import PedalWelds`.
2. In `__init__`, just before `self.reset(model,None)`:

```python
        self.welded_pedals = config.pedal_attachment == 'weld'
        self._welds = PedalWelds(model) if self.welded_pedals else None
```

3. In `set_enabled`, right after the `name not in self.CONTACTS` validation:

```python
        if not enabled and self.welded_pedals and name.endswith('_pedal'):
            return True   # a weld cannot be released
```

4. In `compute_qfrc`, at the top of the `for name,entry in self.supports.items():` loop:

```python
        for name,entry in self.supports.items():
            body,site,bike,geom=entry
            if self.welded_pedals and name.endswith('_pedal'):
                side=name.split('_')[0]
                force_on_rider=self._welds.force_on_rider_n(model,data,side)
                toward_foot=data.site_xpos[site]-data.geom_xpos[geom]
                normal_hat=toward_foot/max(np.linalg.norm(toward_foot),1e-9)
                normal_load=float(max(0.,force_on_rider@normal_hat))
                diagnostics[name]={'enabled':True,'in_platform':True,
                    'normal_load_n':normal_load,
                    'gap_m':self._welds.translation_residual_m(model,data,side),
                    'vertical_force_on_rider_n':float(force_on_rider[2])}
                if detailed:
                    diagnostics[name].update({'tangent_force_n':np.zeros(3),
                        'patches':[],
                        'force_on_rider_n':force_on_rider.tolist(),
                        'force_on_bike_n':(-force_on_rider).tolist(),
                        'moment_about_rider_origin_nm':np.zeros(3).tolist(),
                        'radial_energy_j':0.,'shear_energy_j':0.,
                        'relative_power_w':0.})
                new_states[f"{name}:0"]=_SupportState()
                new_states[f"{name}:1"]=_SupportState()
                continue
```

5. Still in `compute_qfrc`, before `self.delivered_crank_torque_nm=delivered`:

```python
        if self.welded_pedals:
            delivered=self._welds.delivered_crank_torque_nm(model,data)
```

6. In `stored_energy`, first line inside the supports loop:

```python
        for name,entry in self.supports.items():
            if self.welded_pedals and name.endswith('_pedal'):
                continue
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -x -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/sim/ride/rider_contacts.py tests/test_welded_pedal_attachment.py
git commit -m "feat: source pedal contact diagnostics from weld reactions"
```

---

### Task 5: Controller weld targets + inert recovery

**Files:**
- Modify: `src/bike_sim/sim/ride/rider_control.py` (`__init__` ~lines 246-250, `compute` ~lines 370-465, `_targets` ~lines 280-310)
- Test: `tests/test_welded_pedal_attachment.py` (append)

**Interfaces:**
- Consumes: `ArticulatedConfig.pedal_attachment`; weld datum implicitly via `target_data` at qpos0.
- Produces: `self.welded: bool`, `self._weld_pedal_bodies: dict[str,int]`, `self._weld_sole_offset: dict[str,np.ndarray]` — sole-site offset in the pedal body frame; `sole_targets[side]` returns the weld anchor (pedal pose ∘ offset). `support_diagnostics['feet'][side]['recovery_stage']` stays `'none'`.

- [ ] **Step 1: Write the failing tests**

Append:

```python
def test_weld_mode_sole_targets_track_the_welded_anchor():
    model, data, controller, _ = welded_rig()
    from bike_sim.sim.ride.rider_control import RiderCommand
    controller.compute(model, data, RiderCommand(20.),
        contact_loads={'front':100., 'rear':100., 'saddle':400., 'grip':True})
    for side in ('front', 'rear'):
        # The target must be the welded sole position, not a footprint
        # projection: it coincides with the actual sole site (< 3 mm residual).
        target = np.asarray(controller.sole_targets[side])
        actual = np.asarray(data.site_xpos[controller.soles[side]])
        assert np.linalg.norm(target - actual) < .003


def test_weld_mode_never_enters_pedal_recovery():
    model, data, controller, _ = welded_rig()
    from bike_sim.sim.ride.rider_control import RiderCommand
    # Force the sole below the pedal platform — the flat-mode recovery trigger.
    data.qpos[model.joint('rider_root_z').qposadr[0]] -= .04
    mujoco.mj_forward(model, data)
    controller.compute(model, data, RiderCommand(20.),
        contact_loads={'front':100., 'rear':100., 'saddle':400., 'grip':True},
        support_states={'front_pedal':{'normal_load_n':200.,'in_platform':True,
                                       'force_on_rider_n':[0.,0.,-50.]},
                        'rear_pedal':{'normal_load_n':200.,'in_platform':True,
                                      'force_on_rider_n':[0.,0.,-50.]}})
    feet = controller.support_diagnostics['feet']
    assert all(entry['recovery_stage'] == 'none' for entry in feet.values())


def test_weld_mode_feet_stay_pinned_when_crank_is_kicked():
    model, data, controller, _ = welded_rig()
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    welds = PedalWelds(model)
    data.qvel[controller.crank_spin_dof] = 8.   # sudden cadence jump
    for _ in range(100):
        mujoco.mj_step(model, data)
        assert welds.translation_residual_m(model, data, 'front') < .003
        assert welds.translation_residual_m(model, data, 'rear') < .003
        for side in ('front', 'rear'):
            # mj_objectVelocity fills res[0:3] = angular, res[3:6] = linear:
            # welded bodies must share angular velocity (residual covers linear).
            foot_v = np.zeros(6); pedal_v = np.zeros(6)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                    f'rider_foot_{side}')), foot_v, 0)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                    f'pedal_{side}')), pedal_v, 0)
            np.testing.assert_allclose(foot_v[:3], pedal_v[:3], atol=.05)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -k 'sole_targets or recovery or pinned' -x -q`
Expected: FAIL — sole targets still come from `project_sole_goal` (several mm off the welded anchor or hover offset), and a below-pedal sole triggers `recovery_stage == 'release'`.

- [ ] **Step 3: Implement the weld branch in the controller**

In `src/bike_sim/sim/ride/rider_control.py`:

1. In `__init__`, after `mujoco.mj_kinematics(model,self.target_data)` (line ~246):

```python
        self.welded=config.pedal_attachment=='weld'
        self._weld_pedal_bodies={}
        self._weld_sole_offset={}
        if self.welded:
            for side in ('front','rear'):
                body=int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,
                    f'pedal_{side}'))
                self._weld_pedal_bodies[side]=body
                rel=(np.asarray(self.target_data.site_xpos[self.soles[side]])
                     -np.asarray(self.target_data.xpos[body]))
                self._weld_sole_offset[side]=np.asarray(
                    self.target_data.xmat[body]).reshape(3,3).T@rel
```

2. In `_targets` (signature `(self,model,data,side,*,compression_m=None,shear_m=0.,clearance_m=0.,posture=None)`), replace the recovery/projection block at lines 296-304:

```python
        if self.welded:
            # The welded sole cannot hover or separate: aim at the exact point
            # the weld equality is already holding (datum = qpos0 relative pose).
            body=self._weld_pedal_bodies[side]
            sole_goal=(np.asarray(data.xpos[body])
                +np.asarray(data.xmat[body]).reshape(3,3)
                 @self._weld_sole_offset[side])
            goal_diagnostic={'saturated':False,'limiting_reasons':[],'weld':True}
        else:
            recovery_goal = self._active_recovery[side].goal(data.geom_xpos[geom],
                data.geom_xmat[geom].reshape(3,3), model.geom_size[geom])
            if recovery_goal is None:
                sole_goal, goal_diagnostic = project_sole_goal(data.geom_xpos[geom],
                    data.geom_xmat[geom].reshape(3,3), model.geom_size[geom],
                    self.config.pedal_patch_half_length_m, radius, depth, shear)
            else:
                sole_goal = recovery_goal
                goal_diagnostic = {'saturated': False, 'limiting_reasons': [], 'recovery': True}
```

Everything after (`ankle_goal = sole_goal+ankle_offset-...`, `self.sole_targets`, the 2-link IK and joint targets) is unchanged — the weld branch just supplies a different `sole_goal`.

3. In `compute`, guard recovery observation (line ~475):

```python
        if support_states is not None and not steady_state and not self.welded:
```

4. In `compute`, force zero hover clearance (line ~609):

```python
            clearance=0. if (self.welded or stance[side]) else cfg.swing_clearance_m
```

`_predict_target_state`/`_coasting_target_state`/`initialize_velocity` are unchanged: writing `pedal_spin` qvel `-rate` keeps the platform level in world, which is exactly the pose the weld datum preserves.

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_welded_pedal_attachment.py -x -q`
Expected: PASS.

- [ ] **Step 5: Run the flat-mode regression file**

Run: `uv run pytest tests/test_physical_pedaling_regression.py -x -q`
Expected: PASS — the `flat` path must be behaviorally identical (same pad branch, same recovery, same targets).

- [ ] **Step 6: Commit**

```bash
git add src/bike_sim/sim/ride/rider_control.py tests/test_welded_pedal_attachment.py
git commit -m "feat: track weld anchors and disable pedal recovery in weld mode"
```

---

### Task 6: Equilibrium + closed-loop integration test

**Files:**
- Test: `tests/test_welded_pedal_attachment.py` (append)
- Modify only if the test exposes it: `src/bike_sim/sim/ride/physical_equilibrium.py` or `physical_runtime.py` `_initial_speed`/`initialize`

**Interfaces:**
- Consumes: everything above; `RideSimulation` full stack (`sim.physical` runtime with `rider_control`, `rider_contacts`, `drive`).
- Produces: green closed-loop evidence — the acceptance bars from the spec.

Why no planned source change: at init, crank phase sets `pedal_spin = -phase`, keeping the platform level; the weld datum was built with a level sole on a level platform, so IK-posed feet arrive weld-consistent (residual ≈ 0) and the 2.5 ms constraint absorbs sub-mm leftovers. `_initial_speed` likewise writes `pedal_spin qvel = -crank_rate`, which matches a welded level platform. This task *verifies* that reasoning; only if the residual test fails do you touch equilibrium.

- [ ] **Step 1: Write the failing integration test**

Append:

```python
@pytest.mark.slow
def test_welded_ride_holds_feet_through_pedaling_and_coast():
    from bike_sim.terrain import get_preset
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    cfg = SimulationPhysicsConfig(
        'physical', drive_mode='articulated_effort', timestep_s=.00125,
        initial_speed_mps=4.,
        articulated=ArticulatedConfig(pedal_attachment='weld'),
        drive=PhysicalDriveConfig(
            transmission_model='ideal_mid_drive', human_torque_nm=20.,
            pedaling=PedalingConfig(enabled=True, coast_above_rpm=110.,
                                    resume_below_rpm=90., stop_time_s=.35)))
    sim = RideSimulation(track=get_preset('flat'), rider='articulated_planar',
                         physics_config=cfg)
    rt = sim.physical
    welds = PedalWelds(sim.model)
    worst_residual = 0.
    saw_coast = False
    for _ in range(int(6. / cfg.timestep_s)):
        sim.step()
        for side in ('front', 'rear'):
            worst_residual = max(worst_residual,
                welds.translation_residual_m(sim.model, sim.data, side))
        feet = rt.rider_control.support_diagnostics['feet']
        assert all(entry['recovery_stage'] == 'none' for entry in feet.values())
        saw_coast = saw_coast or rt.drive.last.get('rider_mode') == 'coasting'
    assert worst_residual < .003
    # The weld-mode torque sensor still feeds the drivetrain observer.
    assert 'human_sensor_nm' in rt.drive.last
```

(If `RideSimulation`'s constructor already resets/solves equilibrium, no extra call is needed; `sim.reset()` exists at `ride_sim.py:357` if state must be rebuilt.)

- [ ] **Step 2: Run the test — expect failures to diagnose**

Run: `uv run pytest tests/test_welded_pedal_attachment.py::test_welded_ride_holds_feet_through_pedaling_and_coast -x -q`
Expected: PASS, or FAIL showing one of:
- residual > 3 mm at init → IK initial pose vs weld datum mismatch: inspect `data.efc_pos` right after `rider_control.initialize` and adjust `physical_equilibrium.py` pedal writes (see fallback below).
- solver divergence/`mjError` → closed loop is fighting: reduce weld `solref` to `max(4*dt, closure)` and re-run before deeper changes.

Fallback (only if the residual fails): after `runtime.rider_control.initialize` inside `solve_physical_equilibrium`, align each pedal body to the posed foot instead of `-phase`: read the foot's world pitch `atan2(-xmat[foot][2,0], xmat[foot][0,0])` and write `pedal_{side}_spin = foot_pitch - crank_pitch` (the datum keeps pedal parallel to foot). Only implement this if the test proves the need.

- [ ] **Step 3: Commit**

```bash
git add tests/test_welded_pedal_attachment.py
git commit -m "test: welded rider holds feet through pedaling and coasting"
```

---

### Task 7: Welded profile + acceptance verification

**Files:**
- Create: `examples/research/viewer_physics_welded.toml`
- No source changes.

**Interfaces:**
- Produces: the user-facing run command `uv run bike-ride --physics-config examples/research/viewer_physics_welded.toml ...`.

- [ ] **Step 1: Create the welded physics profile**

```bash
cp examples/research/viewer_physics_fast.toml examples/research/viewer_physics_welded.toml
```

Then in `viewer_physics_welded.toml`, inside the `[articulated]` section add:

```toml
pedal_attachment = "weld"
```

- [ ] **Step 2: Headless flat acceptance run**

```bash
uv run bike-ride --headless --no-plots \
  --physics-config examples/research/viewer_physics_welded.toml \
  --track examples/research/rough_uphill.toml \
  --rider articulated_planar --research --duration 15 \
  --out verification/welded_ride
```

Expected: the run completes (writes `telemetry.csv` + `summary.json`), no MuJoCo warning storm. Then check weld health in-process:

```bash
uv run python - <<'EOF'
import json
summary = json.load(open('verification/welded_ride/summary.json'))
assert summary.get('model_valid', True), summary
print('duration_s:', summary.get('duration_s'))
EOF
```

- [ ] **Step 3: Regression sweep — flat mode untouched**

```bash
uv run pytest tests/test_physical_pedaling_regression.py tests/test_coasting_contact_recovery.py tests/test_rider_balance.py tests/test_physics_config.py -x -q
```

Expected: PASS (slow tests included; they exercise the flat path end-to-end).

- [ ] **Step 4: Commit**

```bash
git add examples/research/viewer_physics_welded.toml
git commit -m "feat: add welded-pedal research physics profile"
```

- [ ] **Step 5: Hand off for visual check**

Tell the user: `uv run bike-ride --physics-config examples/research/viewer_physics_welded.toml --rider articulated_planar --track examples/research/rough_uphill.toml` — feet must never leave the pedals, including during coasting and cadence jumps.

---

## Self-Review notes (applied while writing)

- **Spec coverage**: config (T1), topology+solref (T2), weld telemetry + torque sensor (T3, T4), pad-disable + release no-op (T4), sole anchor + no recovery + no hover (T5), equilibrium/closed-loop (T6), profile + acceptance + flat regression (T7). Seated coasting and push-only requests are *preserved behavior* — verified by running the unchanged flat regression file in T5/T7, no new code.
- **Deliberate non-changes**: `physical_runtime.py`, `physical_equilibrium.py`, `pedal_recovery.py`, drivetrain files — the spec's invariants are upheld by keeping their interfaces intact.
- **Sign conventions pinned by experiment**, not assumed: weld rows [pos3, rot3], world-frame, force on foot = `+λ[:3]`, on pedal = `-λ[:3]`; `delivered_crank_torque_nm` = `qfrc[crank_dof]` via `mj_mulJacTVec` restricted to weld rows.
