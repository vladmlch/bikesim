"""Parity between native StaticBrake and Python StaticBrakeApplier.

Static brakes are model frictionloss writes plus solved
mjCNSTR_FRICTION_DOF generalized forces — never an actuator torque. The
probe model is a falling body carrying two spin hinges; an external
qfrc_applied on a wheel DOF gives the friction row real work to hold.
"""
import mujoco
import numpy as np
import pytest
from _golden import record as record_golden

from native_loader import load_native

from bike_sim.sim.ride.static_braking import StaticBrakeApplier

_XML = """<mujoco>
  <worldbody>
    <body name="bike" pos="0 0 1">
      <freejoint/>
      <geom type="box" size=".1 .1 .1" mass="20"/>
      <body name="front_wheel" pos=".6 0 -.6">
        <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size=".2" mass="2"/>
      </body>
      <body name="rear_wheel" pos="-.6 0 -.6">
        <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size=".2" mass="2"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

CEILING = 200.0


def _dof(model, name):
    return int(model.joint(name).dofadr[0])


@pytest.fixture
def model(tmp_path):
    model = mujoco.MjModel.from_xml_string(_XML)
    path = tmp_path / "brake.mjb"
    mujoco.mj_saveModel(model, str(path))
    return model, path


@pytest.fixture
def probe(model):
    model, path = model
    native = load_native()
    return native.NativeTestAdapter(
        str(path),
        {"static_brake": {"front_dof": _dof(model, "front_wheel_spin"),
                          "rear_dof": _dof(model, "rear_wheel_spin"),
                          "ceiling_nm": CEILING}})


def _python_solved(model, front_demand, rear_demand, load_front, load_rear):
    """The Python applier's exact model writes and solved rows."""
    data = mujoco.MjData(model)
    brake = StaticBrakeApplier(_dof(model, "front_wheel_spin"),
                               _dof(model, "rear_wheel_spin"), CEILING)
    brake.apply(model, data, front_demand, rear_demand)
    data.qfrc_applied[_dof(model, "front_wheel_spin")] = load_front
    data.qfrc_applied[_dof(model, "rear_wheel_spin")] = load_rear
    mujoco.mj_forward(model, data)
    return model.dof_frictionloss.copy(), brake.solved_components(model, data)


def test_frictionloss_writes_match_python(model, probe):
    model, _ = model
    front_dof = _dof(model, "front_wheel_spin")
    rear_dof = _dof(model, "rear_wheel_spin")
    probe.apply_static_brake(0.4, 0.7)
    written = probe.dof_frictionloss
    assert written[front_dof] == pytest.approx(80.0)
    assert written[rear_dof] == pytest.approx(140.0)
    expected = np.zeros(model.nv)
    StaticBrakeApplier(front_dof, rear_dof, CEILING).apply(
        model, mujoco.MjData(model), 0.4, 0.7)
    record_golden(model.dof_frictionloss, "static brake frictionloss")
    np.testing.assert_array_equal(written, model.dof_frictionloss)


def test_solved_components_match_python(model, probe):
    model, _ = model
    front_dof = _dof(model, "front_wheel_spin")
    rear_dof = _dof(model, "rear_wheel_spin")
    ctrl = np.zeros(model.nu)
    loads = np.zeros(model.nv)
    loads[front_dof], loads[rear_dof] = 60.0, -45.0
    frictionloss, expected = _python_solved(model, 0.4, 0.7, 60.0, -45.0)
    probe.apply_static_brake(0.4, 0.7)
    probe.set_inputs(ctrl, loads)
    probe.forward()
    solved = probe.static_brake_solved()
    record_golden(expected["front_static_brake"], "front static brake solved",
                  kind="close", rtol=1e-12, atol=1e-12)
    record_golden(expected["rear_static_brake"], "rear static brake solved",
                  kind="close", rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(
        solved["front_static_brake"], expected["front_static_brake"],
        rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(
        solved["rear_static_brake"], expected["rear_static_brake"],
        rtol=1e-12, atol=1e-12)
    # The friction row must actually be holding the external load.
    assert abs(solved["front_static_brake"][front_dof]) > 1.0


def test_demands_clamp_and_sides_are_independent(model, probe):
    model, _ = model
    front_dof = _dof(model, "front_wheel_spin")
    rear_dof = _dof(model, "rear_wheel_spin")
    probe.apply_static_brake(1.5, -0.5)
    written = probe.dof_frictionloss
    assert written[front_dof] == pytest.approx(CEILING)
    assert written[rear_dof] == 0.0
    ctrl = np.zeros(model.nu)
    loads = np.zeros(model.nv)
    loads[front_dof] = 60.0
    probe.set_inputs(ctrl, loads)
    probe.forward()
    solved = probe.static_brake_solved()
    # Rear row unwritten -> no friction row for that DOF -> all-zero force.
    assert not np.any(solved["rear_static_brake"])
    assert abs(solved["front_static_brake"][front_dof]) > 1.0


def test_brake_validation_matches_python(model):
    model, path = model
    native = load_native()
    front_dof = _dof(model, "front_wheel_spin")
    rear_dof = _dof(model, "rear_wheel_spin")
    with pytest.raises(Exception):
        StaticBrakeApplier(front_dof, front_dof, CEILING)
    with pytest.raises(Exception):
        native.NativeTestAdapter(
            str(path), {"static_brake": {"front_dof": front_dof,
                                         "rear_dof": front_dof,
                                         "ceiling_nm": CEILING}})
    with pytest.raises(Exception):
        StaticBrakeApplier(-1, rear_dof, CEILING)
    with pytest.raises(Exception):
        native.NativeTestAdapter(
            str(path), {"static_brake": {"front_dof": -1,
                                         "rear_dof": rear_dof,
                                         "ceiling_nm": CEILING}})
    with pytest.raises(Exception):
        StaticBrakeApplier(front_dof, rear_dof, -1.0)
    with pytest.raises(Exception):
        native.NativeTestAdapter(
            str(path), {"static_brake": {"front_dof": front_dof,
                                         "rear_dof": rear_dof,
                                         "ceiling_nm": -1.0}})


def test_apply_rejects_nonfinite_demands(probe):
    with pytest.raises(Exception):
        probe.apply_static_brake(float("nan"), 0.0)
    with pytest.raises(Exception):
        probe.apply_static_brake(0.0, float("inf"))


# --- standstill bite (brake_hold_*) -------------------------------------------
#
# A held wheel at rest latches its angle and a clipped spring-damper pins it —
# the pad stiction that parks a real bike on a grade, which the regularized
# friction bound alone cannot express (it creeps under sustained load).

HOLD = {"hold_torque_nm": 50.0, "hold_stiffness_nm_rad": 1000.0,
        "hold_damping_nms_rad": 5.0, "hold_engage_radps": .3,
        "hold_release_radps": .6}


@pytest.fixture
def hold_probe(model, tmp_path):
    model, _ = model
    path = tmp_path / "hold_brake.mjb"
    mujoco.mj_saveModel(model, str(path))
    native = load_native()
    return native.NativeTestAdapter(
        str(path), {"static_brake": {"front_dof": _dof(model, "front_wheel_spin"),
                                     "rear_dof": _dof(model, "rear_wheel_spin"),
                                     "ceiling_nm": CEILING, **HOLD}})


def _hold_applier(model):
    return StaticBrakeApplier(
        _dof(model, "front_wheel_spin"), _dof(model, "rear_wheel_spin"), CEILING,
        hold_torque_nm=HOLD["hold_torque_nm"],
        hold_stiffness_nm_rad=HOLD["hold_stiffness_nm_rad"],
        hold_damping_nms_rad=HOLD["hold_damping_nms_rad"],
        hold_engage_radps=HOLD["hold_engage_radps"],
        hold_release_radps=HOLD["hold_release_radps"])


def _wheel_state(model, front_qpos, front_qvel, rear_qpos=0., rear_qvel=0.):
    q = np.zeros(model.nq)
    v = np.zeros(model.nv)
    q[model.joint("front_wheel_spin").qposadr[0]] = front_qpos
    q[model.joint("rear_wheel_spin").qposadr[0]] = rear_qpos
    v[_dof(model, "front_wheel_spin")] = front_qvel
    v[_dof(model, "rear_wheel_spin")] = rear_qvel
    return q, v


def _python_hold(model, data, brake, front_qpos, front_qvel, front, rear, advance):
    q, v = _wheel_state(model, front_qpos, front_qvel)
    data.qpos[:] = q
    data.qvel[:] = v
    return brake.hold_qfrc(model, data, front, rear, advance=advance)


def _native_hold(model, probe, front_qpos, front_qvel, front, rear, advance):
    q, v = _wheel_state(model, front_qpos, front_qvel)
    probe.set_wheel_state(q, v)
    return np.asarray(probe.static_brake_hold_qfrc(front, rear, advance))


def test_hold_bite_parity_sequence(model, hold_probe):
    """Python applier and native twin produce identical hold forces through an
    engage/wind/clip/release/reset sequence."""
    model, _ = model
    front_dof = _dof(model, "front_wheel_spin")
    data = mujoco.MjData(model)
    py = _hold_applier(model)
    sequence = [
        # rolling wheel: |omega| past release never latches — friction bound only
        (1.0, -2.0, 0.8, 0.8, True),
        # standstill inside the engage band: latch the angle
        (0.02, 0.0, 0.8, 0.8, True),
        # wound spring below the clip
        (0.03, 0.0, 0.8, 0.8, True),
        # wound past the clip: capped at torque*demand
        (0.5, 0.0, 0.8, 0.8, True),
        # a probe pass never moves the latch even past the release speed
        (0.5, 1.0, 0.8, 0.8, False),
        # advancing past release speed forgets the captured angle
        (0.5, 1.0, 0.8, 0.8, True),
        # hysteresis: inside the band but above engage, still free
        (0.5, 0.4, 0.8, 0.8, True),
        # back inside the engage band: re-bites at the new angle
        (0.5, 0.1, 0.8, 0.8, True),
        (0.6, 0.1, 0.8, 0.8, True),
        # lever freed: the bite releases
        (0.6, 0.1, 0.0, 0.8, True),
        # lever re-grabbed at the moved angle: new latch, no memory
        (0.6, 0.0, 0.5, 0.8, True),
        (0.7, 0.0, 0.5, 0.8, True),
    ]
    for front_qpos, front_qvel, front, rear, advance in sequence:
        expected = _python_hold(model, data, py, front_qpos, front_qvel,
                                front, rear, advance)
        got = _native_hold(model, hold_probe, front_qpos, front_qvel,
                           front, rear, advance)
        np.testing.assert_array_equal(got, expected)
    # The last advancing pair latched at theta=.6 with demand .5: the clip
    # caps the spring at 50*.5=25 and the .1 rad windup is past it.
    got = _native_hold(model, hold_probe, 0.7, 0.0, 0.5, 0.8, False)
    assert got[front_dof] == pytest.approx(-25.0)
    # reset() forgets the captured angle — the next standstill re-bites.
    py.reset()
    hold_probe.static_brake_reset()
    expected = _python_hold(model, data, py, 0.9, 0.0, 0.5, 0.8, True)
    got = _native_hold(model, hold_probe, 0.9, 0.0, 0.5, 0.8, True)
    np.testing.assert_array_equal(got, expected)
    assert expected[front_dof] == pytest.approx(0.0)


def test_hold_bite_disabled_by_default(model):
    """A zero hold torque or stiffness disables the bite — the pre-feature
    behavior and the absent-key wire default."""
    model, _ = model
    data = mujoco.MjData(model)
    front_dof = _dof(model, "front_wheel_spin")
    brake = StaticBrakeApplier(front_dof, _dof(model, "rear_wheel_spin"), CEILING)
    out = brake.hold_qfrc(model, data, 1., 1., advance=True)
    assert not np.any(out)
    # Non-finite demands still reject on the hold path like apply() does.
    with pytest.raises(Exception):
        brake.hold_qfrc(model, data, float("nan"), 0., advance=True)


def test_hold_validation_matches_python(model):
    model, path = model
    front_dof = _dof(model, "front_wheel_spin")
    rear_dof = _dof(model, "rear_wheel_spin")
    # Release speed below the engage band start rejects on both sides.
    bad = dict(HOLD, hold_engage_radps=.6, hold_release_radps=.3)
    with pytest.raises(Exception):
        StaticBrakeApplier(front_dof, rear_dof, CEILING, **bad)
    native = load_native()
    with pytest.raises(Exception):
        native.NativeTestAdapter(
            str(path), {"static_brake": {"front_dof": front_dof,
                                         "rear_dof": rear_dof,
                                         "ceiling_nm": CEILING, **bad}})
