"""Parity between native StaticBrake and Python StaticBrakeApplier.

Static brakes are model frictionloss writes plus solved
mjCNSTR_FRICTION_DOF generalized forces — never an actuator torque. The
probe model is a falling body carrying two spin hinges; an external
qfrc_applied on a wheel DOF gives the friction row real work to hold.
"""
import mujoco
import numpy as np
import pytest

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
