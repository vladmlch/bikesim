"""Acceptance: the rider stays engaged and the motor follows, on the flat and on 15 %."""
from pathlib import Path
import csv
import json

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
        '--diagnostic-model-limits', '--record-decimation', '1',
        '--initial-speed', '0', '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    env._acceptance_output = tmp_path/'out'
    return env


def _interval_violations(samples):
    return tuple((sample.interval_id,sample.time_s,tuple(sample.channels['attachment_violations']))
                 for sample in samples if sample.channels['attachment_violations'])


def _violations_after(rows,after_s=.5):
    return tuple(failure for row in rows for failure in row['interval_violations']
                 if failure[1] > after_s)


def _sample_row(env,sample):
    channels=sample.channels;drive=channels['drive'];allocation=channels.get('rider_allocation',{})
    attachments=channels.get('attachment_samples',{})
    return dict(interval_id=sample.interval_id,t=sample.time_s,
        v=float(sample.qvel[env.sim.root_x_dofadr]),x=float(sample.qpos[env.sim.root_x_qposadr]),
        cadence=drive['cadence_rpm'],human=drive['human_sensor_nm'],motor=drive['motor_torque_nm'],
        mode=drive['rider_mode'],shift=drive['shift_active'],gear=drive['gear_rear_teeth'],
        shifts=drive['shift_count'],engaged=drive['motor_freewheel_engaged'],
        command=drive['human_command_nm'],model_valid=channels['model_status']['model_valid'],
        violations=tuple(channels['attachment_violations']),
        rider_power=channels['rider_positive_power_w'],
        allocation_invalid=allocation.get('invalid_controller',False),
        crank_task_nm=allocation.get('crank_task_nm'),
        crank_task_shortfall_nm=allocation.get('crank_task_shortfall_nm'),
        foot_front_normal_n=attachments.get('foot_front',{}).get('normal_n'),
        foot_rear_normal_n=attachments.get('foot_rear',{}).get('normal_n'),
        saddle_normal_n=attachments.get('saddle',{}).get('normal_n'),
        front_slip_ratio=channels['tires']['front'].get('slip_ratio'),
        rear_slip_ratio=channels['tires']['rear'].get('slip_ratio'),
        balance_lost=channels['rider_balance']['balance_lost'],
        balance_lost_at_m=channels['rider_balance']['balance_lost_at_m'],
        constraint_work_ratio=channels['energy']['constraint_work_ratio'],
        joint_positive_power_w=dict(channels['rider_joint_positive_power_w']))


def _write_rows(path,rows):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]) if rows else ['t'])
        writer.writeheader()
        for row in rows:
            writer.writerow({key:json.dumps(value) if isinstance(value,(dict,list,tuple)) else value
                             for key,value in row.items()})


def _save_ride(env,rows):
    path=env._acceptance_output;path.mkdir(parents=True,exist_ok=True)
    _write_rows(path/'ride_rows.csv',rows)
    _write_rows(path/'interval_rows.csv',env._acceptance_intervals)
    env.save(path,overwrite=True)


def _ride(env):
    """Keep100Hz road/shift windows and capture every physical interval separately."""
    rows=[];interval_rows=[];pending=[];last_interval=-1
    env._acceptance_intervals=interval_rows
    runtime=env.sim.physical;original=runtime._close_period
    def close():
        nonlocal last_interval
        try:
            return original()
        finally:
            for sample in runtime.completed_samples:
                if sample.interval_id <= last_interval:
                    continue
                last_interval=sample.interval_id
                interval_rows.append(_sample_row(env,sample));pending.append(sample)
    runtime._close_period=close
    try:
        while not env.done:
            env.step(RideControl())
            row=_sample_row(env,runtime.sample)
            row['t']=env.sim.time_s;row['v']=float(env.sim.speed_mps);row['x']=float(env.sim.position_m)
            row['interval_violations']=_interval_violations(pending)
            rows.append(row);pending.clear()
        runtime.flush()
        if pending:
            row=_sample_row(env,pending[-1]);row['interval_violations']=_interval_violations(pending)
            rows.append(row);pending.clear()
    except Exception as failure:
        try:
            if pending:
                row=_sample_row(env,pending[-1]);row['interval_violations']=_interval_violations(pending)
                rows.append(row);pending.clear()
            _save_ride(env,rows)
        except Exception as save_failure:
            failure.add_note(f'acceptance evidence save failed: {save_failure}')
        raise
    finally:
        runtime._close_period=original
    _save_ride(env,rows)
    assert len(interval_rows)==env.sim.steps, (len(interval_rows),env.sim.steps)
    return rows


def _track(tmp_path, knots, length):
    track = tmp_path/'track.toml'
    track.write_text(f'name = "probe"\nlength_m = {length}\nsurface = "hardpack"\n'
                     f'grade_profile = {{ knots = {knots} }}\n')
    return track


@pytest.mark.slow
def test_flat_reaches_25_kmh_within_10_s(tmp_path):
    env=_environment(tmp_path,_track(tmp_path,[[0.,0.],[300.,0.]],300.),12.)
    rows=_ride(env)
    fast=[r['t'] for r in rows if r['v']>=25*KMH]
    assert fast and fast[0]<=10., max(r['v'] for r in rows)/KMH
    steady=[r for r in rows if r['t']>1. and r['v']<23*KMH and r['mode']=='pedaling']
    assert steady, 'no steady pedaling intervals'
    assert np.mean([60<=r['cadence']<=115 for r in steady])>=.8
    assert not any(r['balance_lost'] for r in rows)
    assert not _violations_after(rows), _violations_after(rows)[:3]


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
    assert not any(r['balance_lost'] for r in rows)
    assert not _violations_after(rows), _violations_after(rows)[:3]


@pytest.mark.slow
def test_fifteen_percent_climb_holds_12_kmh(tmp_path):
    env=_environment(tmp_path,_track(tmp_path,[[0.,0.],[10.,0.],[14.,.15],[150.,.15]],150.),25.)
    rows=_ride(env)
    late=[r for r in rows if r['t']>=12.]
    assert late, 'did not reach the steady climb interval'
    assert min(r['v'] for r in late)>=12*KMH
    assert not any(r['balance_lost'] for r in rows)
    assert not _violations_after(rows), _violations_after(rows)[:3]


@pytest.mark.slow
def test_savage_is_ridden_to_the_end(tmp_path):
    env=_environment(tmp_path,SAVAGE,110.)
    rows=_ride(env)
    assert max(r['x'] for r in rows)>=120.-1e-6 or env.reason=='finish'
    plateau=[r['v'] for r in rows if 38.<=r['x']<=55.]
    assert plateau, 'did not reach the25%plateau'
    assert np.mean(plateau)>=6*KMH
    assert not any(r['balance_lost'] for r in rows)
    assert not _violations_after(rows), _violations_after(rows)[:3]
