"""Tests for articulated grip_attachment='weld' (hand pinned to the bar)."""
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


def _config(grip='weld', **kwargs):
    values = dict(drive_mode='articulated_effort', timestep_s=.00125,
                  closure_time_constant_s=.0025,
                  articulated=ArticulatedConfig(grip_attachment=grip),
                  drive=PhysicalDriveConfig(
                      transmission_model='ideal_mid_drive',
                      human_torque_nm=20.))
    values.update(kwargs)
    physics_mode = values.pop('physics_mode', 'physical')
    return SimulationPhysicsConfig(physics_mode, **values)


def grip_rig(grip='weld', speed=0.):
    """Compiled physical model + posed controller with a pinned hand."""
    specs = BikeSpecs()
    cfg = _config(grip, initial_speed_mps=speed)
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


def test_grip_attachment_defaults_to_spring():
    assert ArticulatedConfig().grip_attachment == 'spring'


def test_grip_attachment_accepts_weld():
    assert ArticulatedConfig(grip_attachment='weld').grip_attachment == 'weld'


def test_grip_attachment_rejects_unknown_values():
    with pytest.raises(ValueError, match='grip_attachment'):
        ArticulatedConfig(grip_attachment='glued')


def test_physics_toml_loads_articulated_grip_attachment(tmp_path):
    path = tmp_path / 'physics.toml'
    path.write_text(
        'physics_mode = "physical"\n'
        'drive_mode = "articulated_effort"\n'
        'timestep_s = 0.00125\n'
        'closure_time_constant_s = 0.0025\n'
        '[articulated]\n'
        'grip_attachment = "weld"\n')
    cfg = load_physics_config(str(path), {})
    assert cfg.articulated.grip_attachment == 'weld'


def test_weld_mode_emits_forearm_steer_connect():
    model, _, _, _ = grip_rig()
    eq = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                             'connect_grip'))
    assert eq >= 0
    assert model.eq_type[eq] == mujoco.mjtEq.mjEQ_CONNECT
    forearm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                              'rider_forearm_pair')
    steer = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'steer')
    assert {int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])} == {forearm, steer}
    assert model.eq_solref[eq][0] == pytest.approx(
        max(2. * model.opt.timestep, .0025))
    assert model.eq_solref[eq][1] == pytest.approx(1.)


def test_spring_mode_keeps_hand_unpinned():
    cfg = _config('spring')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=RiderSpecs(variant='articulated_planar'),
        physics_config=cfg))
    assert int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                               'connect_grip')) < 0


def test_weld_mode_rejects_non_articulated_riders():
    cfg = _config(drive_mode='coast')
    with pytest.raises((ValueError, RuntimeError)):
        generate_mujoco_xml(mode='ride', rider=RiderSpecs(variant='lumped'),
                            physics_config=cfg)


def test_weld_mode_rejects_nonphysical_mode():
    cfg = _config(physics_mode='legacy', drive_mode='coast')
    with pytest.raises((ValueError, RuntimeError)):
        generate_mujoco_xml(mode='ride',
                            rider=RiderSpecs(variant='lumped'),
                            physics_config=cfg)


def test_grip_connect_reader_reports_force_and_residual():
    from bike_sim.sim.ride.weld_pedals import GripConnect
    model, data, _, _ = grip_rig()
    pin = GripConnect(model)
    forearm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                              'rider_forearm_pair')
    data.xfrc_applied[forearm] = [200., 0., 0., 0., 0., 0.]
    for _ in range(20):
        mujoco.mj_step(model, data)
    force_on_rider = pin.force_on_rider_n(model, data)
    assert abs(force_on_rider[0]) > 1.      # pin pulls back on the pushed hand
    assert pin.translation_residual_m(model, data) < .003


def test_weld_mode_applier_reports_grip_diagnostics():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    from bike_sim.sim.ride.weld_pedals import GripConnect
    model, data, controller, cfg = grip_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    applier.initialize_settled_state(model, data)
    applier.compute_qfrc(model, data, model.opt.timestep)
    diag = applier.diagnostics['grip']
    assert diag['enabled'] and diag['reachable']
    assert diag['hand_gap_m'] < .003        # pin residual, not a spring stretch
    pin = GripConnect(model)
    assert np.allclose(diag['force_on_rider_n'],
                       pin.force_on_rider_n(model, data))


def test_weld_mode_grip_release_is_a_noop():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model, data, controller, cfg = grip_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    assert applier.set_enabled('grip', False) is True
    assert applier.enabled['grip']


def _push_torso_until_release_or(model, data, applier, seconds):
    """Drag the torso forward past arm reach; the spring grip would release."""
    torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'rider_torso')
    released = False
    for _ in range(int(seconds / model.opt.timestep)):
        data.xfrc_applied[torso] = [300., 0., -300., 0., 0., 0.]
        mujoco.mj_step(model, data)
        applier.compute_qfrc(model, data, model.opt.timestep)
        released |= not applier.enabled['grip']
        data.xfrc_applied[torso] = np.zeros(6)
    return released


def test_spring_grip_releases_when_torso_forced_past_reach():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model, data, controller, cfg = grip_rig('spring')
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    applier.initialize_settled_state(model, data)
    assert _push_torso_until_release_or(model, data, applier, 1.5)


def test_welded_grip_never_releases_when_torso_forced_past_reach():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    from bike_sim.sim.ride.weld_pedals import GripConnect
    model, data, controller, cfg = grip_rig('weld')
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    applier.initialize_settled_state(model, data)
    assert not _push_torso_until_release_or(model, data, applier, 1.5)
    assert applier.diagnostics['grip']['enabled']
    pin = GripConnect(model)
    assert pin.translation_residual_m(model, data) < .01


@pytest.mark.slow
def test_welded_ride_keeps_hands_on_bar_through_pedaling():
    from bike_sim.terrain import get_preset
    from bike_sim.sim.ride_sim import RideSimulation
    cfg = SimulationPhysicsConfig(
        'physical', drive_mode='articulated_effort', timestep_s=.00125,
        closure_time_constant_s=.0025, initial_speed_mps=4.,
        articulated=ArticulatedConfig(grip_attachment='weld'),
        drive=PhysicalDriveConfig(
            transmission_model='ideal_mid_drive', human_torque_nm=20.,
            pedaling=PedalingConfig(enabled=True)))
    sim = RideSimulation(track=get_preset('flat'), rider='articulated_planar',
                         physics_config=cfg)
    torso = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, 'rider_torso')
    steer = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, 'steer')
    grip = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE,
                           'site_rider_grip')
    rel0 = (sim.data.xmat[steer].reshape(3, 3).T
            @ (sim.data.site_xpos[grip] - sim.data.xpos[steer]))
    worst_offset = 0.
    for _ in range(int(6. / cfg.timestep_s)):
        sim.step()
        rel = (sim.data.xmat[steer].reshape(3, 3).T
               @ (sim.data.site_xpos[grip] - sim.data.xpos[steer]))
        worst_offset = max(worst_offset, float(np.linalg.norm(rel - rel0)))
    assert worst_offset < .01
    assert sim.physical.rider_contacts.enabled['grip']
