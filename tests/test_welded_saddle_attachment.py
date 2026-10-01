"""Tests for articulated saddle_attachment='weld' (pelvis pinned to frame)."""
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


def _config(saddle='weld', **kwargs):
    values = dict(drive_mode='articulated_effort', timestep_s=.00125,
                  closure_time_constant_s=.0025,
                  articulated=ArticulatedConfig(saddle_attachment=saddle),
                  drive=PhysicalDriveConfig(
                      transmission_model='ideal_mid_drive',
                      human_torque_nm=20.))
    values.update(kwargs)
    physics_mode = values.pop('physics_mode', 'physical')
    return SimulationPhysicsConfig(physics_mode, **values)


def welded_rig(speed=0.):
    """Compiled physical model + posed controller with a welded pelvis."""
    specs = BikeSpecs()
    cfg = _config(initial_speed_mps=speed)
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


def test_saddle_attachment_defaults_to_flat():
    assert ArticulatedConfig().saddle_attachment == 'flat'


def test_saddle_attachment_accepts_weld():
    assert ArticulatedConfig(saddle_attachment='weld').saddle_attachment == 'weld'


def test_saddle_attachment_rejects_unknown_values():
    with pytest.raises(ValueError, match='saddle_attachment'):
        ArticulatedConfig(saddle_attachment='glued')


def test_physics_toml_loads_articulated_saddle_attachment(tmp_path):
    path = tmp_path / 'physics.toml'
    path.write_text(
        'physics_mode = "physical"\n'
        'drive_mode = "articulated_effort"\n'
        'timestep_s = 0.00125\n'
        'closure_time_constant_s = 0.0025\n'
        '[articulated]\n'
        'saddle_attachment = "weld"\n')
    cfg = load_physics_config(str(path), {})
    assert cfg.articulated.saddle_attachment == 'weld'


def test_weld_mode_emits_pelvis_frame_equality():
    model, _, _, _ = welded_rig()
    eq = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                               'weld_saddle'))
    assert eq >= 0
    assert model.eq_type[eq] == mujoco.mjtEq.mjEQ_WELD
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'rider_pelvis')
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'frame')
    assert {int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])} == {pelvis, frame}
    assert model.eq_solref[eq][0] == pytest.approx(
        max(2. * model.opt.timestep, .0025))
    assert model.eq_solref[eq][1] == pytest.approx(1.)


def test_flat_mode_keeps_pelvis_unwelded():
    cfg = _config('flat')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=RiderSpecs(variant='articulated_planar'),
        physics_config=cfg))
    assert int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                 'weld_saddle')) < 0


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


def test_saddle_weld_reader_reports_force_and_residual():
    from bike_sim.sim.ride.weld_pedals import SaddleWeld
    model, data, _, _ = welded_rig()
    weld = SaddleWeld(model)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'rider_pelvis')
    data.xfrc_applied[pelvis] = [0., 0., -400., 0., 0., 0.]
    for _ in range(20):
        mujoco.mj_step(model, data)
    force_on_rider = weld.force_on_rider_n(model, data)
    assert force_on_rider[2] > 0.          # weld pushes the loaded pelvis up
    assert weld.translation_residual_m(model, data) < .003


def test_weld_mode_applier_reports_weld_diagnostics():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    from bike_sim.sim.ride.weld_pedals import SaddleWeld
    model, data, controller, cfg = welded_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    applier.initialize_settled_state(model, data)
    applier.compute_qfrc(model, data, model.opt.timestep)
    diag = applier.diagnostics['saddle']
    assert diag['in_platform'] and diag['enabled']
    assert diag['gap_m'] < .003           # weld translation residual, not a pad gap
    weld = SaddleWeld(model)
    expected = max(0., weld.force_on_rider_n(model, data)[2])
    assert diag['normal_load_n'] == pytest.approx(expected, abs=1.)


def test_weld_mode_saddle_release_is_a_noop():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model, data, controller, cfg = welded_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    assert applier.set_enabled('saddle', False) is True
    assert applier.enabled['saddle']


@pytest.mark.slow
def test_welded_ride_keeps_pelvis_pinned_through_pedaling():
    from bike_sim.terrain import get_preset
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.sim.ride.weld_pedals import SaddleWeld
    cfg = SimulationPhysicsConfig(
        'physical', drive_mode='articulated_effort', timestep_s=.00125,
        closure_time_constant_s=.0025, initial_speed_mps=4.,
        articulated=ArticulatedConfig(saddle_attachment='weld'),
        drive=PhysicalDriveConfig(
            transmission_model='ideal_mid_drive', human_torque_nm=20.,
            pedaling=PedalingConfig(enabled=True)))
    sim = RideSimulation(track=get_preset('flat'), rider='articulated_planar',
                         physics_config=cfg)
    weld = SaddleWeld(sim.model)
    frame = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, 'frame')
    pelvis = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, 'rider_pelvis')
    rel0 = (sim.data.xmat[frame].reshape(3, 3).T
            @ (sim.data.xpos[pelvis] - sim.data.xpos[frame]))
    worst_residual = 0.
    worst_offset = 0.
    for _ in range(int(6. / cfg.timestep_s)):
        sim.step()
        worst_residual = max(worst_residual,
            weld.translation_residual_m(sim.model, sim.data))
        rel = (sim.data.xmat[frame].reshape(3, 3).T
               @ (sim.data.xpos[pelvis] - sim.data.xpos[frame]))
        # Standing up would need ~0.2 m of frame-relative lift; the soft weld
        # only ever grazes millimetres.
        worst_offset = max(worst_offset, float(np.linalg.norm(rel - rel0)))
    assert worst_residual < .003
    assert worst_offset < .02
