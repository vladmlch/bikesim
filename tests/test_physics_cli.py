import json
from dataclasses import replace
import pytest
from bike_sim.cli.ride import parse_args
from bike_sim.physics.resolution import load_physics_config


def test_explicit_physical_effort_arguments():
    args=parse_args(['--physics','physical','--drive','crank_effort','--initial-speed','0','--human-torque','20','--assist-gain','2'])
    assert args.physics=='physical' and args.drive=='crank_effort'
    assert args.initial_speed==0
    assert args.resolved_physics.drive.human_torque_nm==20
    assert args.speed is None


def test_speed_target_not_silently_used_for_coast():
    with pytest.raises(SystemExit):parse_args(['--physics','physical','--drive','coast','--speed','25'])


def test_toml_precedence_only_explicit_cli_overrides(tmp_path):
    path=tmp_path/'physics.toml'
    path.write_text('physics_mode="physical"\ndrive_mode="crank_effort"\ntimestep_s=0.00025\n[drive]\nhuman_torque_nm=30.0\ncrank_phase_rad=0.7\n[drive.assist]\ngain=3.0\n')
    args=parse_args(['--physics-config',str(path),'--human-torque','20'])
    assert args.resolved_physics.timestep_s==.00025
    assert args.resolved_physics.drive.crank_phase_rad==.7
    assert args.resolved_physics.drive.assist.gain==3
    assert args.resolved_physics.drive.human_torque_nm==20


def test_stationary_rig_requires_a_time_bound():
    with pytest.raises(SystemExit):parse_args(['--physics','physical','--headless'])
    assert parse_args(['--physics','physical','--headless','--duration','1']).duration==1

@pytest.mark.parametrize('flag,value',[('--initial-speed','nan'),('--human-torque','inf'),('--timestep','0'),('--duration','-1')])
def test_nonfinite_or_invalid_cli_is_rejected(flag,value):
    with pytest.raises(SystemExit):parse_args(['--physics','physical',flag,value])


def test_legacy_defaults_are_unchanged():
    args=parse_args([])
    assert args.physics=='legacy' and args.speed==25 and args.drive_mode=='motor'
    assert args.crank_phase==0


def test_mismatched_rider_effort_rejected():
    with pytest.raises(SystemExit):parse_args(['--physics','physical','--drive','articulated_effort','--rider','lumped'])
