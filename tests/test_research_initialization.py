"""A real research-step startup regression, not a prebuilt equilibrium fixture."""
import mujoco
import numpy as np
import pytest
from bike_sim.cli.research import make_environment, parser
from bike_sim.sim.ride.control import RideControl


def test_fine_step_articulated_startup_is_supported_and_probe_is_current(monkeypatch):
    # The supplied implementation passed component tests but exhausted its
    # 20-second settling budget for this public constructor at 125 microseconds.
    env = make_environment(parser().parse_args([
        '--scenario', 'flat', '--rider', 'articulated_planar', '--dt', '.000125',
        '--initial-speed', '0', '--motor-torque', '0', '--duration', '.02',
        '--ideal-sensors']))
    sim = env.sim
    runtime = sim.physical
    assert sim.equilibrium['residual_qacc'] <= .05
    runtime.apply_forces(active=False, advance=False, front=1., rear=1.)
    mujoco.mj_forward(sim.model, sim.data)
    # Canonicalizing the material state must not undo the solved equilibrium.
    assert np.max(np.abs(sim.data.qacc)) <= .05
    assert sim.equilibrium['total_vertical_force_n'] == pytest.approx(
        float(sim.model.body_mass.sum()*9.81), abs=1.)
    np.testing.assert_array_equal(sim.data.qvel, 0.)
    assert all(runtime.rider_contacts.enabled.values())

    # A force probe must use the current probed foot/crank torque, not a stale
    # value from a previous advancing interval. The live contact state is not
    # mutated by the probe.
    contact = runtime.rider_contacts
    old = contact.delivered_crank_torque_nm
    contact.delivered_crank_torque_nm = 9876.
    sensed = []
    original = type(runtime.drive).compute_components

    def capture(drive, *args, **kwargs):
        sensed.append(kwargs['sensed_human_nm'])
        return original(drive, *args, **kwargs)

    with monkeypatch.context() as patch:
        patch.setattr(type(runtime.drive), 'compute_components', capture)
        runtime.apply_forces(active=False, advance=False, front=1., rear=1.)
    assert sensed[-1] == contact.probe_delivered_crank_torque_nm
    assert sensed[-1] != 9876.
    assert contact.delivered_crank_torque_nm == 9876.
    contact.delivered_crank_torque_nm = old

    for _ in range(2):
        result = env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
        assert result.valid_for_learning
    assert env.reason == 'duration'
    assert np.isfinite(sim.data.qpos).all()
