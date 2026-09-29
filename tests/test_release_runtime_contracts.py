"""Engine-level regressions for final force, energy and public-interface review."""
import copy
from dataclasses import replace

import mujoco
import numpy as np
import pytest

from bike_sim.cli.ride import parse_args
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import BatteryConfig, PhysicalDriveConfig, TireBackendConfig
from bike_sim.sim.ride.physical_mapping import point_velocity
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder
from bike_sim.sim.ride.physical_session import configuration_metadata
from bike_sim.sim.ride.session import RideSession
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def make(**kwargs):
    cfg = SimulationPhysicsConfig('physical', tires=TireBackendConfig(backend='compliant_2d'), **kwargs)
    return RideSimulation(track=get_preset('flat'), rider='none', physics_config=cfg)


def test_compliant_virtual_work_in_full_bike_has_one_force_path():
    sim = make(initial_speed_mps=2.)
    m, d = sim.model, sim.data
    d.qvel[:] += np.random.default_rng(2).normal(0., .01, m.nv)
    mujoco.mj_forward(m, d)
    tire = copy.deepcopy(sim.physical.tire)
    force = tire.compute_qfrc(m, d, float(m.opt.timestep))
    power = sum(float(p.world_force_n @ point_velocity(m, d, tire.bodies[side], p.point_m))
                for side, snapshot in tire.snapshots.items() for p in snapshot.patches)
    assert float(force @ d.qvel) == pytest.approx(power, rel=1e-10, abs=1e-9)
    saved = {side: state.xi for side, state in tire.states.items()}
    time = tire.last_time_s
    tire.compute_qfrc(m, d, float(m.opt.timestep), advance=False)
    assert saved == {side: state.xi for side, state in tire.states.items()}
    assert tire.last_time_s == time
    with pytest.raises(ValueError, match='once'):
        tire.compute_qfrc(m, d, float(m.opt.timestep))
    wheel_bodies = set(tire.bodies.values())
    assert not any(int(m.geom_bodyid[c.geom1]) in wheel_bodies or
                   int(m.geom_bodyid[c.geom2]) in wheel_bodies for c in d.contact)


def test_reset_requires_new_recorder_even_after_only_interval_zero():
    sim = make()
    recorder = PhysicalRecorder(sim)
    sim.step()
    recorder.record(sim)
    assert recorder.rows == 1
    sim.reset()
    sim.step()
    with pytest.raises(ValueError, match='reset'):
        recorder.record(sim)
    new = PhysicalRecorder(sim)
    new.record(sim)
    assert new.rows == 1


def test_motor_battery_limit_and_airborne_drive_use_solved_torque():
    sim = make(drive_mode='crank_effort', drive=PhysicalDriveConfig(
        human_torque_nm=20., battery=BatteryConfig(energy_j=.003)))
    m, d = sim.model, sim.data
    d.qpos[sim.physical.address('root_z')[0]] += 2.
    d.qvel[:] = 0.
    d.qvel[sim.physical.address('crank_spin')[1]] = 10.
    sim.step()
    result = sim.physical.sample.channels['drive']
    assert result['motor_torque_nm'] > 0., 'airborne wheel must not disable motor assist'
    assert result['energy_limited']
    assert sim.contacts.rear_snapshot.normal_load_n == 0.
    spent = .003 - sim.physical.drive.battery.energy_j
    assert 0. < spent <= .003
    assert spent == pytest.approx(result['electrical_power_w'] * m.opt.timestep)
    assert abs(sim.physical.energy['electrical_residual_j']) < 1e-10
    sim.step(front_brake_demand=1.)
    assert sim.physical.sample.channels['drive']['motor_torque_nm'] == 0.


@pytest.mark.parametrize('flag,value', [('--tyre-tier','fast'), ('--tyre-pressure','1.5/1.7')])
def test_physical_cli_does_not_silently_ignore_legacy_tire_settings(flag, value):
    with pytest.raises(SystemExit):
        parse_args(['--physics','physical',flag,value])


def test_viewer_cannot_change_spring_energy_without_a_new_run(capsys):
    sim = make(initial_speed_mps=1.)
    session = RideSession(sim)
    before = configuration_metadata(sim)
    for key in (61, 93, 80, 69, 76, 78, 87):
        assert session.input.handle_key(key)
    after = configuration_metadata(sim)
    assert before['configuration_sha256'] == after['configuration_sha256']
    assert 'fixed per run' in capsys.readouterr().out
    session.input.handle_key(32)
    assert session.braking
    session.print_help()
    assert 'Physical ride' in capsys.readouterr().out
    sim.step()
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [sim.position_m, 0., .4]
    camera.distance = 3.5
    camera.azimuth = 90.
    camera.elevation = -10.
    with mujoco.Renderer(sim.model, height=120, width=160) as renderer:
        renderer.update_scene(sim.data, camera=camera)
        pixels = renderer.render()
    assert pixels.shape == (120, 160, 3)
    assert np.std(pixels.astype(float)) > 1.
    native = after['resolved_config']['compiled_contact_solver']
    assert not native['front']['native_enabled']
    assert 'solver parameters' in native['front']['units']
