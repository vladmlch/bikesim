"""Motor freewheel: the rotor drives the crank, the crank may overrun the rotor, never the reverse."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint

TWO_INERTIAS = """
<mujoco>
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="rotor"><joint name="rotor_spin" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="0.3"/>
      <inertial pos="0 0 0" mass="0.3" diaginertia="0.075 0.15 0.075"/></body>
    <body name="crank"><joint name="crank_spin" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.5 1.0 0.5"/></body>
  </worldbody>
  <tendon><fixed name="motor_freewheel" limited="true" range="-1e12 0" margin="0" solreflimit="0.001 1">
    <joint joint="rotor_spin" coef="1"/><joint joint="crank_spin" coef="-1"/></fixed></tendon>
  <actuator><motor name="mid_drive" joint="rotor_spin" gear="1" ctrllimited="true" ctrlrange="0 85"/></actuator>
</mujoco>
"""


def _plant():
    model = mujoco.MjModel.from_xml_string(TWO_INERTIAS)
    data = mujoco.MjData(model)
    hub = IdealFreehubConstraint(model, 1., tendon_name='motor_freewheel',
                                 driver='rotor_spin', driven='crank_spin')
    mujoco.mj_forward(model, data)
    hub.reset(model, data)
    return model, data, hub


def _advance(model, data, hub, steps, ctrl):
    forces = []
    for _ in range(steps):
        hub.prepare(model, data)
        data.ctrl[0] = ctrl
        mujoco.mj_step(model, data)
        forces.append(float(hub.solved_qfrc(model, data)[1]))
    return forces


def test_rotor_torque_drives_the_crank_through_the_engaged_freewheel():
    model, data, hub = _plant()
    torques = _advance(model, data, hub, 200, 10.)
    # Combined inertia 1.15: both spin up together, crank sees the tendon force
    assert data.qvel[0] == pytest.approx(data.qvel[1], rel=1e-3)
    assert data.qvel[1] == pytest.approx(10./1.15*.2, rel=.02)
    assert np.mean(torques[20:]) > 0.


def test_crank_overruns_a_coasting_rotor_without_backdriving_it():
    model, data, hub = _plant()
    _advance(model, data, hub, 200, 10.)
    rotor_before = float(data.qvel[0])
    data.qvel[1] += 5.   # the legs spin the crank faster than the rotor
    forces = _advance(model, data, hub, 200, 0.)
    assert max(abs(force) for force in forces) < 1e-6   # no backdrive either
    assert data.qvel[0] == pytest.approx(rotor_before, abs=1e-9)   # rotor keeps coasting
    assert data.qvel[1] == pytest.approx(rotor_before+5., abs=1e-9)


def test_rotor_catches_up_and_reengages_without_lash():
    model, data, hub = _plant()
    _advance(model, data, hub, 200, 10.)
    data.qvel[1] += 2.
    _advance(model, data, hub, 50, 0.)
    assert hub.relative_rate(data) < 0.
    # In overrun the ratchet follows the angle; no lost angular travel remains.
    hub.prepare(model, data)
    assert hub.boundary == pytest.approx(hub._relative_angle(data), abs=1e-6)
    torques = _advance(model, data, hub, 400, 30.)
    assert data.qvel[0] == pytest.approx(data.qvel[1], rel=1e-3)
    assert np.mean(torques[-20:]) == pytest.approx(30./1.15, rel=.02)
    # A loaded soft tendon has finite penetration, which is not backlash.
    # Unload, let it open, then synchronize the ratchet at this sampling time.
    _advance(model, data, hub, 50, 0.)
    hub.prepare(model, data)
    assert hub.boundary == pytest.approx(hub._relative_angle(data), abs=1e-6)
    assert max(_advance(model, data, hub, 20, 30.)) > 20.
ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT/'examples'/'research'/'viewer_physics_welded.toml'


def _config(tmp_path, drive_lines):
    text = WELDED.read_text().replace('reposition_on_stall = true', 'reposition_on_stall = false')
    text = text.replace('gain = 4.0', 'profile = "bosch_cx_gen4"').replace('max_torque = 85.0', 'max_torque = 1.0')
    # Task 8 removes explicit profile-owned limits from the default TOML. Keep
    # this intentionally conflicting raw limit in the probe regardless.
    assist = text[text.index('[drive.assist]'):text.index('[drive.battery]')]
    if '\nmax_torque = ' not in assist:
        text = text.replace('[drive.assist]\n', '[drive.assist]\nmax_torque = 1.0\n')
    start = text.index('[drive]\n')
    end = text.index('[drive.pedaling]')
    text = text[:start]+'[drive]\n'+drive_lines+'\n\n'+text[end:]
    for name in ('joint_envelope_path', 'joint_strength_path'):
        text = text.replace(f'{name} = "', f'{name} = "{WELDED.parent}/')
    path = tmp_path/'physics.toml'
    path.write_text(text)
    return path


def _model(tmp_path, drive_lines, initial_speed='0'):
    from bike_sim.cli import research as research_cli
    track = tmp_path/'track.toml'
    track.write_text('name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(_config(tmp_path, drive_lines)), '--track-file', str(track),
        '--duration', '0.01', '--initial-speed', initial_speed, '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    return env.sim.model, env.sim.physical.drive, env.sim.data


def _actuator_joint(model):
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'mid_drive')
    return model.joint(int(model.actuator_trnid[aid, 0])).name


@pytest.mark.slow
def test_default_topology_is_rigid_crank_with_motor_on_the_crank(tmp_path):
    model, drive, data = _model(tmp_path, 'transmission_model = "ideal_mid_drive"')
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'crank_clutch') == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'motor_freewheel') == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'drive_shaft_spin') == -1
    assert _actuator_joint(model) == 'crank_spin'
    assert drive.clutch is None and drive.freewheel is None
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'mid_drive')
    assert model.actuator_ctrlrange[aid, 0] == 0.
    assert model.actuator_ctrlrange[aid, 1] == 85.
    assert drive.config.assist.max_torque == 1.   # raw field must not cap a profile


@pytest.mark.slow
def test_rotor_inertia_adds_a_freewheeled_rotor_body(tmp_path):
    model, drive, data = _model(tmp_path, 'transmission_model = "ideal_mid_drive"\nhuman_torque_nm = 20.0\nrotor_inertia_kgm2 = 0.15', initial_speed='1')
    rotor = model.body('motor_rotor')
    assert max(rotor.inertia) == pytest.approx(.15)   # principal axes may be permuted
    assert float(rotor.mass[0]) < 1.
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'motor_freewheel') >= 0
    assert _actuator_joint(model) == 'rotor_spin'
    assert drive.freewheel is not None and drive.freewheel.driver_dof == int(model.joint('rotor_spin').dofadr[0])
    rotor_dof = int(model.joint('rotor_spin').dofadr[0])
    crank_dof = int(model.joint('crank_spin').dofadr[0])
    assert data.qvel[rotor_dof] == pytest.approx(data.qvel[crank_dof])
    assert data.qvel[rotor_dof] > 0.
    assert model.actuator_ctrlrange[model.actuator('mid_drive').id, 1] == 85.
    # Crank gates permission, rotor speed sets its own shaft-power ceiling.
    from bike_sim.physics.pedaling import PedalingState
    from bike_sim.sim.ride.control import RideControl
    data.qvel[rotor_dof] = 150.*2*np.pi/60.
    data.qvel[crank_dof] = 60.*2*np.pi/60.
    state = PedalingState('pedaling', '', 20., 60.)
    drive.assist.torque = 85.
    drive.compute_components(model, data, .001, speed_mps=3., sensed_human_nm=100.,
                             control=RideControl(), pedaling_state=state)
    expected = np.interp(150., drive.assist.torque_curve[:, 0], drive.assist.torque_curve[:, 1])
    assert drive.last['motor_torque_nm'] == pytest.approx(expected)
    assert drive.last['motor_shaft_power_w'] == pytest.approx(expected*data.qvel[rotor_dof])
    assert drive.last['motor_shaft_power_w'] <= 600.
    mujoco.mj_step(model, data)
    transmission = drive.settle_actuation(model, data)
    assert np.isfinite(transmission).all()
    assert drive.last['motor_freewheel_torque_nm'] >= 0.
    assert drive.last['motor_freewheel_dissipation_power_w'] >= 0.


@pytest.mark.slow
def test_legacy_crank_clutch_still_builds_for_regression_ab(tmp_path):
    model, drive, data = _model(tmp_path, 'transmission_model = "ideal_mid_drive"\nmotor_clutch = true')
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'crank_clutch') >= 0
    assert _actuator_joint(model) == 'drive_shaft_spin'
    assert drive.clutch is not None


def test_rotor_and_legacy_clutch_are_exclusive():
    from bike_sim.physics.physical_config import PhysicalDriveConfig
    with pytest.raises(ValueError):
        PhysicalDriveConfig(transmission_model='ideal_mid_drive', motor_clutch=True, rotor_inertia_kgm2=.1)
    with pytest.raises(ValueError):
        PhysicalDriveConfig(transmission_model='elastic_chain', rotor_inertia_kgm2=.1)
    with pytest.raises(ValueError):
        PhysicalDriveConfig(rotor_inertia_kgm2=-.1)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), True])
def test_rotor_inertia_requires_a_finite_nonnegative_scalar(bad):
    from bike_sim.physics.physical_config import PhysicalDriveConfig
    with pytest.raises(ValueError):
        PhysicalDriveConfig(transmission_model='ideal_mid_drive', rotor_inertia_kgm2=bad)


def test_rotor_requires_an_effort_drive_mode():
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.physical_config import PhysicalDriveConfig
    drive = PhysicalDriveConfig(transmission_model='ideal_mid_drive', rotor_inertia_kgm2=.15)
    with pytest.raises(ValueError, match='effort'):
        SimulationPhysicsConfig(physics_mode='physical', drive_mode='coast', drive=drive)
