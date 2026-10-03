"""Solved weld reactions: what a welded rider would feel on unilateral supports."""
from pathlib import Path

import numpy as np
import pytest

from bike_sim.cli import research as research_cli
from bike_sim.sim.ride.control import RideControl

ROOT = Path(__file__).resolve().parents[1]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'
SUPPORTS = ('saddle', 'front_pedal', 'rear_pedal', 'grip_left', 'grip_right')
DIAGNOSTIC_SUPPORTS = ('saddle', 'front_pedal', 'rear_pedal', 'grip')


def _environment(tmp_path, grade):
    track = tmp_path / 'track.toml'
    track.write_text(f'name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     f'grade_profile = {{ knots = [[0.0, {grade}], [30.0, {grade}]] }}\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(WELDED), '--track-file', str(track), '--duration', '1.5',
        '--energy-tolerance', '1e9', '--diagnostic-model-limits', '--initial-speed', '0',
        '--record-decimation', '400', '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _rider_weight(model):
    return 9.81*sum(model.body_mass[b] for b in range(model.nbody)
                    if model.body(b).name.startswith('rider_'))


def _held_episode(tmp_path, grade, channel='rider_welds'):
    env = _environment(tmp_path, grade)
    samples = []
    supports = DIAGNOSTIC_SUPPORTS if channel == 'rider' else SUPPORTS
    while not env.done:
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
        samples.append(env.sim.physical.sample.channels[channel])
    return samples[len(samples)//2:], _rider_weight(env.sim.model), supports


@pytest.mark.slow
def test_held_rider_weight_is_carried_by_the_welds(tmp_path):
    """At rest every attachment together carries exactly the rider's weight, the saddle in compression."""
    samples, weight, supports = _held_episode(tmp_path, 0.)
    vertical = np.mean([sum(w[k]['force_on_rider_n'][2] for k in supports) for w in samples])
    assert vertical == pytest.approx(weight, rel=.01)
    assert np.mean([w['saddle']['normal_n'] for w in samples]) > .3*weight
    assert not any(w['saddle']['would_separate'] or w['saddle']['would_slip'] for w in samples)


@pytest.mark.slow
def test_contact_diagnostics_carry_the_solved_weight(tmp_path):
    """The rider contact channel (and the controller's support loads) now report solved reactions.

    Regression: they were read from the zero-input forward pass at the top of
    apply_forces and summed to ~40 N for a 785 N rider, saddle in tension.
    """
    samples, weight, supports = _held_episode(tmp_path, 0., channel='rider')
    vertical = np.mean([sum(r[k]['force_on_rider_n'][2] for k in supports) for r in samples])
    assert vertical == pytest.approx(weight, rel=.01)
    assert np.mean([r['saddle']['normal_load_n'] for r in samples]) > .3*weight


@pytest.mark.slow
def test_torque_sensor_reads_the_previous_solved_crank_torque(tmp_path):
    """The pedelec torque sensor is the pedal-weld torque solved one step earlier, exactly."""
    env = _environment(tmp_path, 0.)
    physical = env.sim.physical
    physical.set_record_decimation(1)
    sensed, solved = [], []
    for _ in range(400):
        physical.step(control=RideControl(motor_torque_nm=0., human_torque_nm=40.))
        for sample in physical.completed_samples:
            channels = sample.channels
            sensed.append(channels['drive']['human_sensor_nm'])
            solved.append(channels['rider_welds']['crank_torque_nm'])
    assert len(sensed) == len(solved) == 400
    assert np.max(np.abs(np.array(sensed[1:]) - np.array(solved[:-1]))) < 1e-9
    assert np.mean(solved[200:]) > 10.


@pytest.mark.slow
def test_steep_grade_saddle_shear_exceeds_friction(tmp_path):
    """On a held 35 % grade the weld keeps the neutral rider from sliding back off the saddle."""
    samples, _, _ = _held_episode(tmp_path, .35)
    assert all(w['saddle']['would_slip'] for w in samples)
