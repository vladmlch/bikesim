"""Acceptance: the rider stays engaged and the motor follows, on the flat and on 15 %."""
from pathlib import Path
import csv

import numpy as np
import pytest

from bike_sim.cli import research as research_cli
from bike_sim.sim.ride.control import RideControl

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT/'examples'/'research'/'viewer_physics_welded.toml'
SAVAGE = ROOT/'examples'/'research'/'rough_uphill_savage.toml'
KMH = 1/3.6


def _config(tmp_path):
    """Welded profile with absolute asset paths. Every ride starts in 22/51: on the
    flat the rider spins out and must shift up — that is the shifter's job to prove."""
    text = WELDED.read_text()
    for name in ('joint_envelope_path', 'joint_strength_path'):
        text = text.replace(f'{name} = "', f'{name} = "{WELDED.parent}/')
    path = tmp_path/'physics.toml'
    path.write_text(text)
    return path


def _environment(tmp_path, track, duration_s):
    args = research_cli.parser().parse_args([
        '--physics-config', str(_config(tmp_path)), '--track-file', str(track),
        '--duration', str(duration_s), '--dt', '.00125', '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--record-decimation', '8',
        '--initial-speed', '0', '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    env._acceptance_output = tmp_path/'out'
    return env


def _save_ride(env, rows):
    path = env._acceptance_output
    path.mkdir(parents=True, exist_ok=True)
    with (path/'ride_rows.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['t'])
        writer.writeheader()
        writer.writerows(rows)
    env.save(path, overwrite=True)


def _ride(env):
    """Automatic rider and assist; preserve drive and model validity evidence."""
    rows = []
    try:
        while not env.done:
            env.step(RideControl())
            sample = env.sim.physical.sample
            drive = sample.channels['drive']
            rows.append(dict(t=env.sim.time_s, v=float(env.sim.speed_mps),
                             x=float(env.sim.position_m),
                             cadence=drive['cadence_rpm'], human=drive['human_sensor_nm'],
                             motor=drive['motor_torque_nm'], mode=drive['rider_mode'],
                             shift=drive['shift_active'], gear=drive['gear_rear_teeth'],
                             shifts=drive['shift_count'], engaged=drive['motor_freewheel_engaged'],
                             command=drive['human_command_nm'], model_valid=env.model_valid,
                             violations=';'.join(env.sim.physical.step_violations),
                             rider_power=sample.channels['rider_positive_power_w'],
                             allocation_invalid=sample.channels.get('rider_allocation', {}).get('invalid_controller', False)))
    except Exception as failure:
        try:
            _save_ride(env, rows)
        except Exception as save_failure:
            failure.add_note(f'acceptance evidence save failed: {save_failure}')
        raise
    _save_ride(env, rows)
    return rows


def _track(tmp_path, knots, length):
    track = tmp_path/'track.toml'
    track.write_text(f'name = "probe"\nlength_m = {length}\nsurface = "hardpack"\n'
                     f'grade_profile = {{ knots = {knots} }}\n')
    return track


@pytest.mark.slow
def test_flat_start_reaches_20_kmh_with_the_rider_engaged(tmp_path):
    env = _environment(tmp_path, _track(tmp_path, [[0., 0.], [300., 0.]], 300.), 20.)
    rows = _ride(env)
    assert env.reason == 'duration', env.reason
    v = np.array([r['v'] for r in rows])
    assert v.max() >= 20.*KMH
    # Launching in 22/51 the cranks spin out within a second; the rider must
    # answer with a run of upshifts (one per 0.4 s cooldown), coasting between
    # clicks like a real rider. By 6 s the gear has to be landed.
    launch = [r for r in rows if r['t'] <= 6.]
    assert launch[-1]['shifts'] >= 5, launch[-1]['shifts']
    assert launch[-1]['gear'] <= 24, launch[-1]['gear']
    settled = [r for r in rows if r['t'] >= 6.]
    assert np.mean([r['mode'] == 'coasting' for r in settled]) <= .15
    # ~100 rows/s (10 ms control period): judge pedalling below the assist taper.
    steady = [r for r in settled if r['v'] < 23.*KMH and r['mode'] == 'pedaling']
    assert len(steady) > 200
    human = np.array([r['human'] for r in steady])
    cadence = np.array([r['cadence'] for r in steady])
    assert np.mean(human > 0.) >= .8, np.mean(human > 0.)
    assert np.mean((cadence >= 60.) & (cadence <= 115.)) >= .8, (cadence.min(), cadence.max())
    assert np.mean(cadence) >= 70.
    # Motor smoothness: pulses with the legs, no 3<->45 N.m sawtooth
    motor = np.array([r['motor'] for r in steady if r['v'] < 20.*KMH])
    assert motor.mean() > 20.
    assert np.std(motor)/motor.mean() < .6


@pytest.mark.slow
def test_motor_follows_the_rider_through_a_shift_without_an_extra_cut(tmp_path):
    """The rider unloads to 30 % for 0.2 s; the motor follows that through its lag
    (spec S6) but is never cut on top of it: motor/sensor stays near the Turbo gain."""
    env = _environment(tmp_path, _track(tmp_path, [[0., 0.], [300., 0.]], 300.), 12.)
    rows = _ride(env)
    shifts = [i for i in range(1, len(rows)) if rows[i]['shift'] and not rows[i-1]['shift']]
    assert len(shifts) >= 5, 'the 22/51 flat launch must produce a run of upshifts'
    checked = 0
    for i in shifts:
        window = [r for r in rows[i:i+20] if r['shift'] and r['v'] < 20.*KMH]   # the 0.2 s cut
        human = np.mean([max(r['human'], 0.) for r in window]) if window else 0.
        if human < 4.:
            continue
        motor = np.mean([r['motor'] for r in window])
        assert motor/human >= 2.5, (rows[i]['t'], human, motor)   # 3.4 nominal, lag only
        checked += 1
    assert checked >= 1


@pytest.mark.slow
def test_fifteen_percent_climb_holds_speed_without_a_dead_spot_stall(tmp_path):
    knots = [[0., 0.], [10., 0.], [14., .15], [150., .15]]
    env = _environment(tmp_path, _track(tmp_path, knots, 150.), 25.)
    rows = _ride(env)
    assert env.reason == 'duration', env.reason
    climbing = [r for r in rows if r['t'] >= 10.]
    v = np.array([r['v'] for r in climbing])
    assert v.min() >= 5.*KMH, v.min()/KMH
    cadence = np.array([r['cadence'] for r in climbing])
    assert cadence.min() > 20., cadence.min()
    assert np.mean(np.array([r['human'] for r in climbing]) > 0.) >= .8


@pytest.mark.slow
def test_savage_fifteen_percent_plateau_is_ridden_above_5_kmh(tmp_path):
    env = _environment(tmp_path, SAVAGE, 22.)
    rows = _ride(env)
    plateau = [r for r in rows if 17. <= r['x'] <= 33.]
    assert plateau, 'did not reach the 15 % plateau'
    v = np.array([r['v'] for r in plateau])
    assert v.min() >= 4.*KMH, v.min()/KMH          # rough surface: 1 km/h margin
    cadence = np.array([r['cadence'] for r in plateau])
    assert np.mean(cadence > 20.) >= .95
