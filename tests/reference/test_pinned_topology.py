"""The seated spindle plant keeps nine hinges and force-only attachments."""
import re
from pathlib import Path

import mujoco
import pytest

from bike_sim.cli import research as research_cli

WELDED = Path('examples/research/viewer_physics_welded.toml')


def _pinned_config(tmp_path):
    text = WELDED.read_text()
    for key, value in [('pedal_attachment', 'spindle'), ('saddle_attachment', 'pin'), ('grip_attachment', 'connect')]:
        text = re.sub(rf'{key} = "\w+"', f'{key} = "{value}"', text)
    for key in ('joint_envelope_path', 'joint_strength_path'):
        text = re.sub(rf'{key} = "([^"]+)"', lambda m: f'{key} = "{(WELDED.parent / m.group(1)).resolve()}"', text)
    path = tmp_path / 'physics.toml'
    path.write_text(text)
    return path


def _model(tmp_path):
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)), '--track-file',
        'examples/research/rough_uphill_savage.toml', '--duration', '.01',
        '--dt', '.00125', '--diagnostic-model-limits', '--out', str(tmp_path / 'out')])
    env = research_cli.make_environment(args)
    return env.sim.model, env


def _compiled(tmp_path):
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.physics.resolution import load_physics_config
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.physics.rider_segments import geometry_pose
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    cfg = load_physics_config(_pinned_config(tmp_path))
    specs = BikeSpecs()
    rider = RiderSpecs(variant='articulated_planar')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        specs, mode='ride', rider=rider, crank_joint=True, physics_config=cfg))
    controller = ArticulatedRiderController(model, geometry_pose(rider, specs), cfg.articulated, specs.crank_length/1000.)
    return model, controller


@pytest.mark.slow
def test_pinned_plant_topology(tmp_path):
    model, controller = _compiled(tmp_path)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i): model.eq_type[i] for i in range(model.neq)}
    for name in ('connect_foot_front', 'connect_foot_rear', 'connect_saddle', 'connect_grip_left', 'connect_grip_right'):
        assert names[name] == mujoco.mjtEq.mjEQ_CONNECT
    assert not [n for n in names if n and n.startswith('weld_')]
    for side in ('front', 'rear'):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'rider_ankle_{side}') == -1
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name and name.startswith('act_rider_'):
            assert not model.actuator_forcelimited[i]
    assert set(controller.joints) == {'rider_torso_hinge', 'rider_shoulder_left', 'rider_shoulder_right', 'rider_elbow_left', 'rider_elbow_right', 'rider_hip_front', 'rider_hip_rear', 'rider_knee_front', 'rider_knee_rear'}
    assert set(controller.locked_joints) == {'rider_ankle_front', 'rider_ankle_rear'}
    eq = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'connect_foot_front')
    assert model.eq_data[eq, 2] == pytest.approx(-.115)


def test_joint_profiles_select_present_joints_and_require_them(tmp_path):
    import json
    from bike_sim.mujoco.reference_rider import reference_joint_names
    from bike_sim.physics.rider_envelope import load_joint_envelopes
    from bike_sim.physics.joint_strength import load_strength_profile, load_strength_coordinates
    present = tuple(n for n in reference_joint_names() if 'ankle' not in n)
    folder = WELDED.parent
    assert set(load_joint_envelopes(str(folder / 'rider_joint_envelope_anatomical.json'), present=present)) == set(present)
    assert set(load_strength_profile(str(folder / 'rider_strength_reference.json'), present, require_verified=False)) == set(present)
    assert set(load_strength_coordinates(str(folder / 'rider_strength_reference.json'), present)) == set(present)
    for filename, loader in [('rider_joint_envelope_anatomical.json', lambda p: load_joint_envelopes(p, present=present)), ('rider_strength_reference.json', lambda p: load_strength_profile(p, present, require_verified=False))]:
        payload = json.loads((folder / filename).read_text())
        del payload['joints']['rider_knee_front']
        path = tmp_path / filename
        path.write_text(json.dumps(payload))
        with pytest.raises(ValueError):
            loader(str(path))
