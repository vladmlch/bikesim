"""R5: continuous foot contact through the crank cycle, scrape intent, coast.

Both feet keep their support objective for the whole revolution; there is no
control-mode foot lift. Pedal intent is a bounded wish (cadence request,
phase-locked ankle offset, scrape fraction) consumed by the rider allocator --
never an externally applied crank moment and never a kinematic lock.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from bike_sim.physics.rider_program import PedalIntent, pedal_intent

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'


def test_ankle_program_is_periodic_and_scrape_is_only_an_intent():
    a = pedal_intent(.3, 8., ankle_amplitude_rad=.1, scrape_fraction=.2)
    b = pedal_intent(.3 + 2*math.pi, 8., ankle_amplitude_rad=.1,
                     scrape_fraction=.2)
    assert math.isclose(a.ankle_offset_rad, b.ankle_offset_rad, abs_tol=1e-12)
    assert 0 <= a.scrape_fraction <= 1
    stopped = pedal_intent(.3, 0., ankle_amplitude_rad=.1, scrape_fraction=.2)
    assert stopped.cadence_rad_s == 0.


def test_pedal_intent_validates_its_inputs():
    base = dict(ankle_amplitude_rad=.1, scrape_fraction=.2)
    for cadence in (-1., -1e-9):
        with pytest.raises(ValueError):
            pedal_intent(.3, cadence, **base)
    for kwargs in (dict(ankle_amplitude_rad=-.1),
                   dict(scrape_fraction=-.01),
                   dict(scrape_fraction=1.01)):
        with pytest.raises(ValueError):
            pedal_intent(.3, 8., **{**base, **kwargs})
    for bad in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            pedal_intent(bad, 8., **base)
        with pytest.raises(ValueError):
            pedal_intent(.3, bad, **base)


def test_pedal_intent_carries_no_torque_or_force_field():
    # Scrape is only a strategy wish: the intent type has no channel through
    # which it could apply a physical moment to the crank or the wheel.
    assert set(PedalIntent.__dataclass_fields__) == {
        'cadence_rad_s', 'ankle_offset_rad', 'scrape_fraction'}


def _environment(tmp_path, config=WELDED, duration_s=8.):
    from bike_sim.cli import research as research_cli
    tmp_path.mkdir(parents=True, exist_ok=True)
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 200.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(config), '--track-file', str(track),
        '--duration', str(duration_s), '--dt', '.0005',
        '--record-decimation', '1', '--initial-speed', '0',
        '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _flat_config(tmp_path):
    text = WELDED.read_text().replace('pedal_attachment = "weld"',
                                      'pedal_attachment = "flat"')
    for name in ('joint_envelope_path', 'joint_strength_path'):
        text = text.replace(f'{name} = "',
                            f'{name} = "{WELDED.parent}/')
    path = tmp_path / 'flat_physics.toml'
    path.write_text(text)
    return path


def _observe_all_intervals(runtime):
    rows=[]
    original=runtime._close_period
    def close():
        try:
            return original()
        finally:
            rows.extend(runtime.completed_samples)
    runtime._close_period=close
    return rows


def _assert_physical_intervals(rows):
    assert rows
    for index,sample in enumerate(rows):
        assert sample.interval_id == index
        assert not sample.channels['attachment_violations'], (index,sample.channels['attachment_violations'])
        assert sample.channels['rider_positive_power_w'] <= 450.+1e-9
        assert not sample.channels['rider_strength_violations']
        # The work criterion is integral at a closing 5 ms period/tail; every
        # incoming interval still contributes its signed and absolute work.
        if (sample.interval_id+1)%10==0 or index==len(rows)-1:
            assert sample.channels['energy']['constraint_work_ok']
        attachments=sample.channels['attachment_samples']
        for key in ('foot_front','foot_rear'):
            assert key in attachments, (index,key)
            assert attachments[key]['normal_n'] >= 20., (index,key,attachments[key])


def test_cadence_control_validates_a_finite_nonnegative_wish():
    from bike_sim.sim.ride.control import RideControl
    assert RideControl(crank_target_rate_rad_s=8.).crank_target_rate_rad_s == 8.
    for bad in (-1.,math.nan,math.inf):
        with pytest.raises(ValueError):
            RideControl(crank_target_rate_rad_s=bad)


def test_first_pedal_command_and_restart_respect_configured_effort_slew():
    from bike_sim.physics.physical_config import PedalingConfig
    from bike_sim.physics.pedaling import PedalingPolicy
    policy=PedalingPolicy(PedalingConfig(enabled=True,effort_slew_nm_s=300.))
    policy.update(0.,0.,0.,0.,.0005,enabled=False)
    first=policy.update(0.,0.,0.,30.,.0005)
    assert 0. <= first.effort_nm <= .15+1e-12
    for _ in range(10):policy.update(0.,0.,0.,30.,.0005)
    coast=policy.update(0.,0.,0.,0.,.0005)
    assert coast.effort_nm == 0.
    resumed=policy.update(0.,0.,0.,30.,.0005)
    assert 0. <= resumed.effort_nm <= .15+1e-12


def test_cadence_wish_is_forwarded_with_immediate_rider_fields_past_motor_queue():
    from collections import deque
    from types import SimpleNamespace
    from bike_sim.sim.research.environment import ResearchEnvironment
    from bike_sim.sim.ride.control import RideControl
    env=ResearchEnvironment.__new__(ResearchEnvironment)
    env.terminated=env.truncated=False;env.reason=env.error=None
    env.demand_nm=None;env.rider_program=env.rider_behavior=None
    env._queue=deque();env.delay_steps=2;env.control_steps=env.max_steps=1
    env._motor_applied=env._applied=RideControl(motor_torque_nm=0.)
    env.commands_requested=[];env.commands_applied=[]
    class CommandObserved(Exception):pass
    def step(*args,control):
        assert control.crank_target_rate_rad_s == 8.
        assert control.motor_torque_nm == 0.
        raise CommandObserved
    env.sim=SimpleNamespace(steps=0,time_s=0.,rider=SimpleNamespace(variant='articulated_planar'),
        physics_config=SimpleNamespace(physics_mode='physical',drive_mode='articulated_effort'),
        physical=SimpleNamespace(_buffer=SimpleNamespace(_raws=[])),step=step)
    env._consume_completed_samples=lambda **kwargs:None
    with pytest.raises(CommandObserved):
        env.step(RideControl(motor_torque_nm=50.,crank_target_rate_rad_s=8.))


@pytest.mark.slow
@pytest.mark.parametrize('cadence_rpm',[40.,80.,110.,120.])
def test_full_crank_revolutions_keep_both_feet_loaded(tmp_path,cadence_rpm,record_property):
    from bike_sim.sim.ride.control import RideControl
    env=_environment(tmp_path/f'{cadence_rpm:.0f}')
    runtime=env.sim.physical;controller=runtime.rider_control
    rows=_observe_all_intervals(runtime)
    while not env.done:
        env.step(RideControl(motor_torque_nm=0.,human_torque_nm=20. if cadence_rpm == 120. else 30.,
            crank_target_rate_rad_s=cadence_rpm*2*math.pi/60.))
    _assert_physical_intervals(rows)
    assert len(rows) == env.sim.steps
    assert env.reason == 'duration', env.reason
    phases=np.array([sample.qpos[controller.crank_spin_qpos] for sample in rows])
    cadences=np.array([sample.qvel[controller.crank_spin_dof]*60/(2*math.pi) for sample in rows])
    achieved_turns=float((phases[-1]-phases[0])/(2*math.pi))
    achieved_rpm=float(np.mean(cadences[len(cadences)//2:]))
    record_property('requested_cadence_rpm',cadence_rpm)
    record_property('achieved_mean_cadence_rpm',achieved_rpm)
    record_property('achieved_forward_revolutions',achieved_turns)
    assert achieved_turns >= 1., (cadence_rpm,achieved_rpm,achieved_turns)
    for name in ('rider_ankle_front','rider_ankle_rear'):
        positions=np.array([sample.qpos[controller.joints[name][0]] for sample in rows])
        lo,hi=controller.joint_ranges[name]
        assert positions.min() >= lo-1e-6 and positions.max() <= hi+1e-6


@pytest.mark.slow
def test_flat_pedals_keep_contact_through_the_cycle(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path, config=_flat_config(tmp_path), duration_s=4.)
    rows = _observe_all_intervals(env.sim.physical)
    recovery_calls = {'front': 0, 'rear': 0}
    infeasible = 0
    calls = 0
    for _ in range(3200):
        if env.done:
            break
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=30.),
                 front_brake_demand=0., rear_brake_demand=0.)
        calls += 1
        feet = env.sim.physical.rider_control.support_diagnostics.get('feet', {})
        for side in ('front', 'rear'):
            if feet.get(side, {}).get('recovery_stage', 'none') != 'none':
                recovery_calls[side] += 1
        if 'rider_controller.infeasible' in env.sim.physical.step_violations:
            infeasible += 1
    assert env.sim.steps > 2000
    _assert_physical_intervals(rows)
    # A flat sole may still physically separate under a bad stroke, but the
    # controller itself never commands a lift: the overwhelming share of the
    # cycle keeps both feet in their non-recovering contact state.
    for side in ('front', 'rear'):
        assert recovery_calls[side] <= .1*calls, (side, recovery_calls)
    # The 20 N pedal floor either held or was reported, never silently met by
    # relaxing the contact constraints.
    assert infeasible <= .1*calls


@pytest.mark.slow
def test_free_coast_is_an_intent_not_a_crank_lock(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    controller = env.sim.physical.rider_control
    rows = _observe_all_intervals(env.sim.physical)
    # Spin the cranks up, then stop requesting effort entirely: the coast
    # request is zero cadence, executed through bounded muscle moments while
    # the feet keep following the pedals. The 8 s episode gives ~3 s of
    # pedaling and the rest a free spin-down.
    for _ in range(400):
        if env.done:
            break
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=30.),
                 front_brake_demand=0., rear_brake_demand=0.)
    cadence_before = abs(float(env.sim.data.qvel[controller.crank_spin_dof]))
    coast_calls = 0
    coast_missing = 0
    for _ in range(1600):
        if env.done:
            break
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=0., rear_brake_demand=0.)
        coast_calls += 1
        samples = env.sim.physical.attachment_samples or {}
        coast_missing += sum(k not in samples for k in
                             ('foot_front', 'foot_rear'))
        # No external human torque may reach the crank in articulated mode:
        # the model has no human_crank actuator at all, and the drive's own
        # accounting must report zero delivered human crank effort.
        assert env.sim.physical.drive.actuators['human_crank'] == -1
        last = env.sim.physical.drive.last
        assert last['human_torque_nm'] == pytest.approx(0.)
        assert last['human_command_nm'] == pytest.approx(0.)
    assert coast_calls > 100
    assert coast_missing == 0
    _assert_physical_intervals(rows)
    assert cadence_before > 1.


@pytest.mark.slow
def test_return_foot_preload_keeps_recovery_foot_in_compression(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    import re
    text = WELDED.read_text()
    for name in ('joint_envelope_path', 'joint_strength_path'):
        text = text.replace(f'{name} = "', f'{name} = "{WELDED.parent}/')
    if 'return_foot_preload_n =' in text:
        text = re.sub(r'^return_foot_preload_n = .*$', 'return_foot_preload_n = 40.0', text, flags=re.M)
    else:
        text = text.replace('[articulated]\n', '[articulated]\nreturn_foot_preload_n = 40.0\n')
    path = tmp_path/'preload.toml'
    path.write_text(text)
    env = _environment(tmp_path/'ride', config=path, duration_s=2.)
    rows = _observe_all_intervals(env.sim.physical)
    while not env.done:
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=20.,
                            crank_target_rate_rad_s=80.*2*math.pi/60.))
    _assert_physical_intervals(rows)
    checked = 0
    for sample in rows:
        # Stance is the support intent, not an allocator solution field.
        stance = sample.channels['rider_support_targets']['stance']
        for side in ('front', 'rear'):
            if not stance[side]:
                normal = sample.channels['attachment_samples'][f'foot_{side}']['normal_n']
                assert normal >= 30., (sample.interval_id, side, normal)
                checked += 1
    assert checked > 0
    assert env.reason == 'duration', env.reason
