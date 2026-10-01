from dataclasses import asdict
import pytest
import bike_sim.cli.research as cli
from bike_sim.sim.research.configuration import research_field
import bike_sim.sim.research.configuration as configuration
from bike_sim.terrain.research import build_research_track


def capture_config(monkeypatch, argv):
    captured = {}
    monkeypatch.setattr(configuration, 'RideSimulation', lambda **kw: captured.update(kw) or object())
    monkeypatch.setattr(configuration, 'ResearchEnvironment', lambda *args, **kw: (args, kw))
    result = cli.make_environment(cli.parser().parse_args(argv))
    return captured, result


def test_file_overrides_defaults_and_explicit_zero_overrides_file(monkeypatch, tmp_path):
    config = tmp_path/'physics.toml'
    config.write_text('initial_speed_mps=4.0\ntimestep_s=0.00025\n[drive]\nhuman_torque_nm=12.0\n[drive.assist]\nmax_torque=42.0\n')
    captured, result = capture_config(monkeypatch, ['--rider', 'lumped', '--physics-config', str(config),
        '--initial-speed', '0', '--motor-max-torque', '0'])
    cfg = captured['physics_config']
    assert cfg.initial_speed_mps == 0. and cfg.timestep_s == .00025
    assert cfg.drive.assist.max_torque == 0. and cfg.drive.human_torque_nm == 12.
    assert result[0][2].sample_period_s == .005


def test_fast_profile_preserves_the_user_drive_and_explicit_dt(monkeypatch):
    captured, result = capture_config(monkeypatch, [
        '--physics-config', 'examples/research/viewer_physics_fast.toml',
        '--track-file', 'examples/research/rough_uphill_extreme.toml',
        '--assist', '--dt', '.00125'])
    physics = captured['physics_config']
    assert physics.timestep_s == .00125
    assert physics.initial_speed_mps == 0.
    assert physics.drive.human_torque_nm == 20.
    assert physics.drive.assist.max_torque == 85.
    assert physics.drive.assist.max_power == 600.
    assert physics.drive.pedaling.enabled
    assert physics.drive.shifting.enabled
    assert result[0][2].sample_period_s == .005


def test_assist_command_preserves_configured_rider_effort():
    from types import SimpleNamespace
    arguments = cli.parser().parse_args([
        '--physics-config', 'examples/research/viewer_physics_fast.toml', '--assist'])
    environment = SimpleNamespace(rider_program=None, demand_nm=None,
                                  sim=SimpleNamespace(time_s=0.))
    control = cli.command_for(environment, arguments)
    assert control.motor_torque_nm is None
    assert control.human_torque_nm is None
    assert control.posture is None


def test_file_can_select_a_nonlinear_tire_without_mixing_linear_fields(monkeypatch, tmp_path):
    f = tmp_path/'curve.toml'
    f.write_text('[tires.front.material]\ndeflection_m=[0.0, 0.03]\nforce_n=[0.0, 5000.0]\n'
        'radial_c_ns_m=100.0\npressure_pa_gauge=200000.0\nprovenance="synthetic"\nvalid_load_range_n=[0.0, 5000.0]\n')
    captured, _ = capture_config(monkeypatch, ['--physics-config', str(f)])
    assert captured['physics_config'].tires.front.material.deflection_m == (0., .03)


def test_mesh_refinement_preserves_extent_and_exposes_real_spacing():
    track = build_research_track('rough_uphill')
    coarse, fine = research_field(track, .005), research_field(track, .0025)
    assert fine.ncol == 2*coarse.ncol-1
    assert fine.radius_x_m == coarse.radius_x_m
    assert fine.resolution_m == .0025
    for value in (0., -1., float('nan'), .007, 1e-12):
        with pytest.raises(ValueError):
            research_field(track, value)
