"""Small compiled-model regressions; no expensive equilibrium/convergence sweep."""
from dataclasses import replace
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import PhysicalDriveConfig, PedalingConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
from bike_sim.sim.ride.rider_control import ArticulatedRiderController, RiderCommand
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier


def rolling_rig(transmission='ideal_mid_drive', human=20., speed=2.):
    specs = BikeSpecs()
    cfg = SimulationPhysicsConfig('physical', drive_mode='articulated_effort',
        initial_speed_mps=speed, drive=PhysicalDriveConfig(
            transmission_model=transmission, human_torque_nm=human))
    rider = RiderSpecs(variant='articulated_planar')
    m = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=rider, physics_config=cfg))
    d = mujoco.MjData(m)
    c = ArticulatedRiderController(m, geometry_pose(rider, specs), cfg.articulated,
                                  specs.crank_length / 1000.)
    c.initialize(m, d)
    r = PhysicalRuntime.__new__(PhysicalRuntime)
    r.cfg, r.rider_control = cfg, c
    geoms = {}
    for side in ('front', 'rear'):
        wheel = m.body(side + '_wheel').id
        geoms[side] = next(i for i in range(m.ngeom)
            if m.geom_bodyid[i] == wheel and m.geom_size[i, 0] > .3)
    r.sim = SimpleNamespace(model=m, data=d, contact_query=SimpleNamespace(
        front_id=geoms['front'], rear_id=geoms['rear']))
    r.snapshots = {side: SimpleNamespace(effective_radius_m=.35, patches=[
        SimpleNamespace(normal_load_n=500., working_surface=True, tangent=np.array([1., 0., 0.]))])
        for side in geoms}
    return m, d, r


@pytest.mark.parametrize('transmission', ['ideal_mid_drive', 'elastic_chain'])
def test_powered_rolling_start_initializes_crank_and_pedals(transmission):
    m, d, r = rolling_rig(transmission)
    q = d.qpos.copy()
    r._initial_speed()
    ratio = r.cfg.drive.gearing.front_teeth / r.cfg.drive.gearing.rear_teeth
    crank = d.qvel[r.address('crank_spin')[1]]
    wheel = d.qvel[r.address('rear_wheel_spin')[1]]
    assert crank == pytest.approx(wheel / ratio)
    for side in ('front', 'rear'):
        assert d.qvel[r.address('pedal_' + side + '_spin')[1]] == pytest.approx(-crank)
    np.testing.assert_array_equal(d.qpos, q)


def test_powered_rolling_start_feet_follow_moving_spindles():
    m, d, r = rolling_rig()
    r._initial_speed()
    mujoco.mj_forward(m, d)
    c = r.rider_control
    assert d.qvel[r.address('crank_spin')[1]] > 0.
    for side in ('front', 'rear'):
        foot = np.zeros((3, m.nv)); pedal = np.zeros_like(foot)
        mujoco.mj_jacSite(m, d, foot, None, c.soles[side])
        mujoco.mj_jacSite(m, d, pedal, None, c.pedals[side])
        np.testing.assert_allclose(foot @ d.qvel, pedal @ d.qvel, atol=.02)
        angular = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, None, angular, c.feet[side])
        np.testing.assert_allclose(angular @ d.qvel, 0., atol=1e-6)


@pytest.mark.parametrize('transmission', ['ideal_mid_drive', 'elastic_chain'])
def test_unpowered_start_keeps_freewheeling_crank_at_rest(transmission):
    m, d, r = rolling_rig(transmission, human=0.)
    r._initial_speed()
    assert d.qvel[r.address('rear_wheel_spin')[1]] > 0.
    assert d.qvel[r.address('crank_spin')[1]] == 0.


@pytest.mark.parametrize('transmission', ['ideal_mid_drive', 'elastic_chain'])
def test_zero_speed_start_does_not_inject_motion(transmission):
    m, d, r = rolling_rig(transmission, speed=0.)
    r._initial_speed()
    np.testing.assert_array_equal(d.qvel, 0.)


def test_excessive_startup_cadence_leaves_feet_and_cranks_stationary():
    model, data, runtime = rolling_rig(speed=5.)
    runtime.cfg = replace(runtime.cfg, drive=replace(runtime.cfg.drive, pedaling=PedalingConfig(enabled=True)))
    runtime._initial_speed()
    assert data.qvel[runtime.address('rear_wheel_spin')[1]] > 0.
    assert data.qvel[runtime.address('crank_spin')[1]] == 0.
    for side in ('front', 'rear'):
        assert data.qvel[runtime.address('pedal_' + side + '_spin')[1]] == 0.


def test_coasting_rider_holds_both_feet_using_only_internal_actuators():
    model, data, runtime = rolling_rig(speed=0.)
    controller = runtime.rider_control
    qpos, qvel = data.qpos.copy(), data.qvel.copy()
    command = RiderCommand(0., crank_target_phase_rad=.1, crank_target_rate_rad_s=0.)
    effort = controller.compute(model, data, command,
        contact_loads={'front':100., 'rear':100., 'saddle':400., 'grip':True})
    assert controller.support_diagnostics['stance'] == {'front':True, 'rear':True}
    assert controller.support_diagnostics['coasting']
    assert all(name.startswith('rider_') for name in effort)
    assert all(abs(value) <= controller.config.joint_limit_nm for value in effort.values())
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)


@pytest.mark.parametrize('motor_request', [None, 20.])
def test_coasting_gates_passive_pedal_torque_but_preserves_external_motor_command(motor_request):
    model, data, runtime = rolling_rig(speed=0.)
    config = replace(runtime.cfg.drive, pedaling=PedalingConfig(enabled=True))
    drive = DrivetrainForceApplier(model, config, 'articulated_effort')
    drive.reset(model, data)
    ratio = config.gearing.front_teeth / config.gearing.rear_teeth
    data.qvel[runtime.address('rear_wheel_spin')[1]] = 120. * 2. * np.pi / 60. * ratio
    drive.compute_components(model, data, model.opt.timestep, speed_mps=2.,
        sensed_human_nm=40., control=RideControl(motor_torque_nm=motor_request))
    assert drive.last['rider_mode'] == 'coasting'
    assert drive.last['human_sensor_nm'] == 40.
    assert drive.last['assist_sensor_nm'] == 0.
    assert (drive.last['motor_request_nm'] > 0.) == (motor_request is not None)
    motor = model.actuator('mid_drive').id
    assert model.actuator_trnid[motor, 0] == model.joint('crank_spin').id
    assert data.ctrl[motor] == pytest.approx(drive.last['motor_torque_nm'])


def test_coasting_probe_does_not_advance_rider_policy_or_ratchet_boundary():
    model, data, runtime = rolling_rig(speed=0.)
    config = replace(runtime.cfg.drive, pedaling=PedalingConfig(enabled=True))
    drive = DrivetrainForceApplier(model, config, 'articulated_effort')
    drive.reset(model, data)
    data.qpos[runtime.address('rear_wheel_spin')[0]] += 2.
    boundary = drive.ideal_hub.boundary
    ranges = model.tendon_range.copy()
    policy = vars(drive.pedaling).copy()
    drive.compute_components(model, data, model.opt.timestep, speed_mps=2.,
        sensed_human_nm=40., advance=False)
    assert drive.ideal_hub.boundary == boundary
    np.testing.assert_array_equal(model.tendon_range, ranges)
    assert vars(drive.pedaling) == policy
    assert drive.pending_actuation is None


def test_unloaded_stance_foot_can_press_but_cannot_request_friction():
    m, d, r = rolling_rig(speed=0.)
    c = r.rider_control
    q, v, applied = d.qpos.copy(), d.qvel.copy(), d.qfrc_applied.copy()
    efforts = c.compute(m, d, RiderCommand(20.),
        contact_loads={'front':0., 'rear':0., 'saddle':400., 'grip':True},
        support_available=dict.fromkeys(('front', 'rear', 'saddle', 'grip'), True))
    force = c.support_diagnostics['feasible_pedal_force_on_bike_n']['front']
    assert force[2] < -1.
    np.testing.assert_allclose(force[:2], 0., atol=1e-12)
    np.testing.assert_array_equal(d.qpos, q)
    np.testing.assert_array_equal(d.qvel, v)
    np.testing.assert_array_equal(d.qfrc_applied, applied)
    assert all(abs(value) <= c.config.joint_limit_nm for value in efforts.values())


def test_grip_ik_allows_the_deflection_needed_for_its_support_force():
    m, d, r = rolling_rig()
    c = r.rider_control
    force = np.array([30., 0., -200.])  # Force ON THE BIKE, not on the rider.
    targets = c._upper_targets(d, grip_force_n=force)
    d.qpos[c.joints['rider_torso_hinge'][0]] = targets['rider_torso_hinge']
    mujoco.mj_forward(m, d)
    for name, angle in c._upper_targets(d, grip_force_n=force).items():
        d.qpos[c.joints[name][0]] = angle
    mujoco.mj_forward(m, d)
    bar = d.xpos[c.frame] + d.xmat[c.frame].reshape(3, 3) @ c.pose.grip
    np.testing.assert_allclose(d.site_xpos[c.grip_site],
                               bar + force/c.config.grip_k_n_m, atol=1e-10)


def test_running_arm_goals_receive_the_support_request(monkeypatch):
    m, d, r = rolling_rig()
    c = r.rider_control
    seen = []
    original = c._upper_targets
    def observe(data, posture=None, *, grip_force_n=None):
        seen.append(grip_force_n)
        return original(data, posture, grip_force_n=grip_force_n)
    monkeypatch.setattr(c, '_upper_targets', observe)
    c.compute(m, d, RiderCommand(20.),
              contact_loads={'front':100., 'rear':100., 'saddle':400., 'grip':True})
    assert seen and all(force is not None for force in seen)
    assert any(np.linalg.norm(force) > 1. for force in seen)


def test_inertial_tracking_is_even_in_cadence_and_quadratic_in_speed():
    m, d, r = rolling_rig()
    c = r.rider_control
    # Isolate the feedforward before clipping; other tests exercise real caps.
    c.config = replace(c.config, joint_limit_nm=1e6, joint_power_limit_w=1e9)
    def tracking(rate):
        d.qvel.fill(0.)
        d.qvel[r.address('crank_spin')[1]] = rate
        mujoco.mj_forward(m, d)
        q, v = d.qpos.copy(), d.qvel.copy()
        c.compute(m, d, RiderCommand(20.),
                  contact_loads={'front':100., 'rear':100., 'saddle':400., 'grip':True})
        np.testing.assert_array_equal(d.qpos, q)
        np.testing.assert_array_equal(d.qvel, v)
        return np.array([c.last_terms[name]['tracking_nm'] for name in c.joints])
    zero, one, two, reverse = [tracking(rate) for rate in (0., 1., 2., -2.)]
    np.testing.assert_array_equal(zero, 0.)
    assert np.linalg.norm(one) > .01
    np.testing.assert_allclose(two, 4.*one, rtol=2e-4, atol=1e-5)
    np.testing.assert_allclose(reverse, two, rtol=2e-4, atol=1e-5)
