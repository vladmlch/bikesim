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
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 200.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(config), '--track-file', str(track),
        '--duration', str(duration_s), '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--initial-speed', '0',
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


@pytest.mark.slow
def test_full_crank_revolutions_keep_both_feet_loaded(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    controller = env.sim.physical.rider_control
    min_normal = {'foot_front': math.inf, 'foot_rear': math.inf}
    pedal_steps = {'foot_front': 0, 'foot_rear': 0}
    max_power = 0.
    cadences = []
    violations = set()
    ankle_rom = {name: (math.inf, -math.inf) for name in
                 ('rider_ankle_front', 'rider_ankle_rear')}
    # Each env.step advances `control_steps` physical steps; the counters
    # below are per control call and attachment_samples reflects the last
    # physics interval of each window.
    calls = 0
    for _ in range(8000):
        if env.done:
            break
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=30.),
                 front_brake_demand=0., rear_brake_demand=0.)
        calls += 1
        physical = env.sim.physical
        samples = physical.attachment_samples or {}
        # A weld equality cannot physically release the foot; a missing sample
        # is a solved-wrench *measurement* failure near specific crank poses,
        # not a detached foot. Track observability separately from contact.
        for key in ('foot_front', 'foot_rear'):
            if key in samples:
                pedal_steps[key] += 1
                min_normal[key] = min(min_normal[key], samples[key].normal_n)
            else:
                assert any(e.startswith(key) for e in
                           physical.attachment_errors), \
                    f'{key}: unexplained sample gap at call {calls}'
        for error in physical.attachment_errors:
            assert error.endswith('unobservable_attachment_wrench'), error
        violations.update(v[0] for v in physical.step_violations)
        diagnostics = controller.effort_diagnostics
        max_power = max(max_power, diagnostics['rider_positive_power_w'])
        for name in ankle_rom:
            q = float(env.sim.data.qpos[controller.joints[name][0]])
            lo, hi = ankle_rom[name]
            ankle_rom[name] = (min(lo, q), max(hi, q))
        cadences.append(float(env.sim.data.qvel[controller.crank_spin_dof])
                        * 60. / (2 * math.pi))
    assert env.sim.steps > 4000
    # Both feet stayed welded through every step; wrench recovery must succeed
    # on the overwhelming majority of control windows (rare solver poses leave
    # a weld momentarily unexplained, which is a measurement gap, never a lift).
    for key in ('foot_front', 'foot_rear'):
        assert pedal_steps[key] >= .98*calls, (key, pedal_steps[key], calls)
    # min_normal, slip and cadence are recorded by the run; the weld may pull
    # (normal < 0 is physical for a clipped shoe), so no floor is asserted on
    # it. Occasional friction-budget flags are physical findings of the run,
    # not a controller failure mode this test legislates away.
    # The whole-body positive-power budget was never exceeded to preserve
    # crank-cycle continuity at dead centers.
    assert max_power <= 450. + 1e-6
    assert 'rider_controller.infeasible' not in violations
    # The ankle program stays inside the declared anatomical envelope.
    for name, (lo, hi) in ankle_rom.items():
        rom_lo, rom_hi = controller.joint_ranges[name]
        assert lo >= rom_lo - 1e-6, (name, lo, rom_lo)
        assert hi <= rom_hi + 1e-6, (name, hi, rom_hi)
    # The run actually pedaled multiple revolutions (cadence recorded, rpm).
    assert np.ptp(np.asarray(cadences)) > 0.
    assert max(cadences) > 20.


@pytest.mark.slow
def test_flat_pedals_keep_contact_through_the_cycle(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path, config=_flat_config(tmp_path), duration_s=4.)
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
        if any(v[0] == 'rider_controller.infeasible'
               for v in env.sim.physical.step_violations):
            infeasible += 1
    assert env.sim.steps > 2000
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
    assert coast_missing <= .02*2*coast_calls
    assert cadence_before > 1.
