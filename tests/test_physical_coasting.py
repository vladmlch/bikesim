from dataclasses import replace
from math import pi

import mujoco
import numpy as np
import pytest

from bike_sim.physics.pedaling import PedalingPolicy
from bike_sim.physics.physical_config import PedalingConfig
from bike_sim.physics.resolution import load_physics_config, resolve_physics_config
from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint


def rpm_rate(rpm):
    return rpm * 2. * pi / 60.


def test_cadence_coasting_uses_wheel_required_cadence_and_hysteresis():
    policy = PedalingPolicy(PedalingConfig(enabled=True))
    assert policy.update(0., rpm_rate(80.), 80., 20., .01).mode == 'pedaling'
    coast = policy.update(.5, rpm_rate(115.), 115., 20., .01)
    assert coast.mode == 'coasting' and coast.effort_nm == 0.
    assert coast.reason == 'cadence'
    assert policy.update(.6, 0., 100., 20., .01).mode == 'coasting'
    resumed = policy.update(.6, 0., 85., 20., .01)
    assert resumed.mode == 'pedaling' and resumed.effort_nm == 20.
    assert resumed.target_phase_rad is None


def test_coasting_goal_stops_smoothly_without_following_actual_crank_forever():
    policy = PedalingPolicy(PedalingConfig(enabled=True, stop_time_s=.2))
    policy.update(1., 10., 120., 20., .01)
    state = None
    for _ in range(30):
        state = policy.update(2., 10., 120., 20., .01)
    assert state.target_rate_rad_s == 0.
    phase = state.target_phase_rad
    assert 1. < phase < 2.1
    assert policy.update(3., 10., 120., 20., .01).target_phase_rad == phase


def test_zero_effort_braking_disabled_and_reset():
    policy = PedalingPolicy(PedalingConfig(enabled=True))
    assert policy.update(0., 0., 60., 0., .01).reason == 'no_effort'
    assert policy.update(0., 0., 60., 20., .01, braking=True).reason == 'braking'
    assert policy.update(0., 0., 60., 20., .01, enabled=False).mode == 'disabled'
    policy.reset()
    assert policy.update(0., 0., 60., 20., .01).mode == 'pedaling'


def test_cadence_policy_is_opt_in_and_has_validated_toml_parameters():
    policy = PedalingPolicy(PedalingConfig())
    assert policy.update(0., rpm_rate(150.), 150., 20., .01).mode == 'pedaling'
    config = load_physics_config('examples/research/viewer_physics_fast.toml')
    assert config.drive.pedaling.enabled
    assert config.drive.pedaling.resume_below_rpm < config.drive.pedaling.coast_above_rpm
    with pytest.raises(ValueError):
        replace(config.drive.pedaling, resume_below_rpm=120.)
    with pytest.raises(ValueError):
        resolve_physics_config({'drive': {'pedaling': {'stop_time_s': 0.}}})


def hub_rig():
    model = mujoco.MjModel.from_xml_string('''
    <mujoco>
      <option gravity="0 0 0" timestep="0.00125"/>
      <worldbody>
        <body name="crank">
          <joint name="crank_spin" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        </body>
        <body name="rear_wheel">
          <joint name="rear_wheel_spin" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        </body>
      </worldbody>
      <tendon>
        <fixed name="ideal_mid_drive_freehub" limited="true" range="-1e12 0"
               solreflimit="0.0025 1" margin="0">
          <joint joint="crank_spin" coef="0.5"/>
          <joint joint="rear_wheel_spin" coef="-1"/>
        </fixed>
      </tendon>
    </mujoco>''')
    data = mujoco.MjData(model)
    hub = IdealFreehubConstraint(model, .5)
    hub.reset(model, data)
    return model, data, hub


def test_ideal_freehub_overrun_does_not_spin_cranks_or_apply_reverse_torque():
    model, data, hub = hub_rig()
    data.qvel[1] = 10.
    for _ in range(30):
        hub.prepare(model, data)
        mujoco.mj_step(model, data)
        force = hub.solved_qfrc(model, data)
        np.testing.assert_array_equal(force, 0.)
    assert data.qvel[0] == pytest.approx(0.)
    assert data.qvel[1] == pytest.approx(10.)


def test_ideal_freehub_reengages_without_position_or_velocity_rewrites():
    model, data, hub = hub_rig()
    data.qpos[1] = 2.
    hub.prepare(model, data)
    data.qvel[0] = 6.
    data.qvel[1] = 1.
    qpos, qvel = data.qpos.copy(), data.qvel.copy()
    hub.prepare(model, data)
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)
    peak_torque = 0.
    for _ in range(100):
        hub.prepare(model, data)
        mujoco.mj_step(model, data)
        force = hub.solved_qfrc(model, data)
        assert force[1] >= -1e-12
        assert force[0] == pytest.approx(-.5 * force[1])
        peak_torque = max(peak_torque, force[1])
    assert peak_torque > 0.
