"""Pinned commands obey virtual work; launch delivery is measured by native solves."""
from dataclasses import replace

import mujoco
import numpy as np
import pytest

from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.rider_control import RiderCommand
from test_pinned_topology import _compiled


def _controller(tmp_path):
    model, controller = _compiled(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, controller


def test_pinned_control_never_calls_allocator_and_publishes_final_commands(tmp_path):
    model, data, c = _controller(tmp_path)
    def forbidden(*args, **kwargs):
        raise AssertionError('closed form called allocator')
    c.allocate = forbidden
    result = c.compute(model, data, RiderCommand(40.), advance=False)
    assert set(result) == set(c.joints)
    assert c.joint_torques_nm == result
    assert set(c.joint_capacity_nm) == set(c.joints)
    assert np.isfinite(list(result.values())).all()
    assert not c.allocation_diagnostics['invalid_controller']
    assert c.strength_violations(result, data.qpos, data.qvel) == ()


def test_upper_pose_target_accounts_for_pelvis_and_frame_pitch(tmp_path):
    model, data, c = _controller(tmp_path)
    pose = RiderPosture(torso_lean_rad=.1)
    neutral = c._upper_targets(data, pose)
    assert neutral['rider_torso_hinge'] == pytest.approx(.1)
    data.qpos[model.joint('rider_root_pitch').qposadr[0]] = .08
    data.qpos[model.joint('root_pitch').qposadr[0]] = -.03
    mujoco.mj_forward(model, data)
    changed = c._upper_targets(data, pose)
    assert changed['rider_torso_hinge']-neutral['rider_torso_hinge'] == pytest.approx(-.11)


def test_native_torso_torque_has_restoring_pelvis_reaction(tmp_path):
    model, data, c = _controller(tmp_path)
    root = int(model.joint('rider_root_pitch').dofadr[0])
    actuator = c.joints['rider_torso_hinge'][2]
    data.ctrl[actuator] = -1.
    mujoco.mj_forward(model, data)
    negative = float(data.qacc[root])
    data.ctrl[actuator] = 1.
    mujoco.mj_forward(model, data)
    assert data.qacc[root] < negative-1.


@pytest.mark.parametrize('speed', [4., -4., 25., -25.])
def test_final_torques_obey_strength_power_and_accelerating_speed_limits(tmp_path, speed):
    model, data, c = _controller(tmp_path)
    for _, dof, _ in c.joints.values():
        data.qvel[dof] = speed
    mujoco.mj_forward(model, data)
    result = c.compute(model, data, RiderCommand(1000.), advance=False)
    power = [result[name]*data.qvel[dof] for name,(_,dof,_) in c.joints.items()]
    assert max(power) <= 250.+1e-9
    assert sum(max(p,0.) for p in power) <= 450.+1e-9
    if abs(speed) >= 20.:
        assert max(power) <= 0.
    assert c.strength_violations(result,data.qpos,data.qvel) == ()


def test_optional_activation_is_bounded_and_probes_do_not_advance_state(tmp_path):
    model, data, c = _controller(tmp_path)
    c.config = replace(c.config, activation_tau_s=.05)
    before = c.active_state.copy()
    probe = c.compute(model, data, RiderCommand(40.), advance=False, dt_s=.005)
    assert np.array_equal(c.active_state, before)
    actual = c.compute(model, data, RiderCommand(40.), advance=True, dt_s=.005)
    assert actual == pytest.approx(probe)
    assert np.array_equal(c.active_state, [actual[n] for n in c.joints])
    with pytest.raises(ValueError, match='once per timestamp'):
        c.compute(model, data, RiderCommand(40.), advance=True, dt_s=.005)


def test_disabled_activation_probe_returns_zero_without_advancing_state(tmp_path):
    model, data, c = _controller(tmp_path)
    c.config = replace(c.config, activation_tau_s=.05)
    c.compute(model,data,RiderCommand(40.),advance=True,dt_s=.005)
    before = c.active_state.copy()
    torques = c.compute(model,data,RiderCommand(0.,enabled=False),advance=False,dt_s=.005)
    assert all(torque == 0. for torque in torques.values())
    assert np.array_equal(c.active_state,before)
    data.time += .005
    c.compute(model,data,RiderCommand(0.,enabled=False),advance=True,dt_s=.005)
    assert not c.active_state.any()


@pytest.mark.slow
def test_flat_launch_delivers_crank_torque(tmp_path):
    from test_realistic_pedelec_acceptance import _environment, _ride, _track
    env = _environment(tmp_path, _track(tmp_path, [[0.,0.],[300.,0.]],300.), 6.)
    rows = _ride(env)
    late = [r for r in rows if r['t'] > 2.]
    assert np.mean([r['human'] for r in late]) >= 10.
    assert np.mean([r['motor'] > 0. for r in late]) >= .8
    assert all(not r['allocation_invalid'] for r in rows)
    assert max(r['v'] for r in rows) >= 15/3.6


@pytest.mark.slow
def test_runtime_produces_front_load_share_without_changing_external_sensors(tmp_path):
    from test_pinned_topology import _model
    from bike_sim.sim.ride.control import RideControl
    _, env = _model(tmp_path)
    runtime = env.sim.physical
    for _ in range(4):
        runtime.step(front=1.,rear=1.,control=RideControl(motor_torque_nm=0.,human_torque_nm=0.))
    channels = runtime.sample.channels
    tires = channels['tires']
    front,rear = (tires[s]['normal_load_n'] for s in ('front','rear'))
    assert runtime.rider_intent_signals.front_load_share == pytest.approx(front/(front+rear))
    assert 'tires' not in channels['sensors']
