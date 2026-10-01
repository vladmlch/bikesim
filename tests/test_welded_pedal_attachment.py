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
                  closure_time_constant_s=.0025,
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
        'closure_time_constant_s = 0.0025\n'
        '[articulated]\n'
        'pedal_attachment = "weld"\n')
    cfg = load_physics_config(str(path), {})
    assert cfg.articulated.pedal_attachment == 'weld'


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
