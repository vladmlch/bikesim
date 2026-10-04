"""R6: forward lean and a bounded body-thrust pulse as posture intent.

The posture program produces only a torso-lean wish: no instantaneous jump
(rate-limited, endpoint-free pulse), no wheelie feedback, no extra bar or
COM force. Anything the intent cannot achieve is reduced by the R4
allocator or reported invalid -- never granted extra grip budget.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from bike_sim.physics.rider_program import (
    SeatedPostureProgram, body_pulse,
)
from bike_sim.physics.seated_climb import SeatedClimbConfig

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'


def test_posture_program_reads_no_inertial_signal():
    import ast, inspect
    from bike_sim.physics import rider_program
    tree=ast.parse(inspect.getsource(rider_program))
    names={node.attr for node in ast.walk(tree) if isinstance(node,ast.Attribute)}
    names.update(node.id for node in ast.walk(tree) if isinstance(node,ast.Name))
    forbidden={'inclination_rad','pitch_rate_up_rad_s','specific_force_body_mps2',
               'efc_force','qfrc_constraint'}
    assert names.isdisjoint(forbidden), names & forbidden


def test_lean_follows_road_grade_not_body_pitch():
    program=SeatedPostureProgram(SeatedClimbConfig(enabled=True,lean_rate_rad_s=5.))
    for i in range(200):
        lean=program.update(i*.01,.20,.01)
    assert lean == pytest.approx(math.atan(.20),abs=1e-9)


def test_road_grade_uses_underwheel_and_preview_samples_uniformly():
    from bike_sim.sim.ride.rider_state import RoadSample,road_grade_for_posture
    samples=tuple(RoadSample(x,0.,g) for x,g in ((0.,.1),(1.,.2),(2.,.3)))
    assert road_grade_for_posture(samples) == pytest.approx(.2)
    with pytest.raises(ValueError,match='road sample'):
        road_grade_for_posture(())


def test_resolver_road_grade_survives_probe_and_ignores_imu():
    from bike_sim.sim.ride.rider_intent import RiderIntentResolver
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.physics.seated_climb import SeatedClimbSignals
    resolvers=[RiderIntentResolver(SeatedClimbConfig(enabled=True),.0005) for _ in range(2)]
    signals=[SeatedClimbSignals(),SeatedClimbSignals(pitch_rate_up_rad_s=4.,
                                                  specific_force_body_mps2=(5.,0.,8.))]
    for step in range(0,2000,20):
        results=[r.resolve(RideControl(),s,step=step,road_grade=.18)
                 for r,s in zip(resolvers,signals)]
        assert results[0].posture == results[1].posture
    assert results[0].posture.torso_lean_rad == pytest.approx(math.atan(.18))
    r=resolvers[0]
    before=r.policy.time_s
    probe=r.resolve(RideControl(),signals[1],step=2000,road_grade=.18,advance=False)
    assert r.policy.time_s == before
    assert probe.posture == results[0].posture


def test_runtime_passes_current_road_snapshot_before_intent_and_force_evaluation():
    from types import SimpleNamespace
    from bike_sim.physics.model_config import SimulationPhysicsConfig,ArticulatedConfig
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    from bike_sim.sim.ride.control_clock import ControlClock
    runtime=PhysicalRuntime.__new__(PhysicalRuntime)
    runtime.cfg=SimulationPhysicsConfig(physics_mode='physical',drive_mode='articulated_effort',
        articulated=ArticulatedConfig(road_lookahead_m=2.),
        seated_climb=SeatedClimbConfig(enabled=True))
    runtime.sim=SimpleNamespace(steps=0,rider=SimpleNamespace(variant='articulated_planar'),
        model=SimpleNamespace(nq=1,nv=1), data=SimpleNamespace(qpos=np.zeros(1),qvel=np.zeros(1),
        time=0.,xpos=np.array([[0.,0.,0.],[1.,0.,0.]])))
    runtime.control_clock=ControlClock(.0005,.005)
    runtime.rider_control=object()
    runtime._wheel_bodies=(0,1)
    runtime.vertices=np.array([[-1.,-.18],[4.,.72]])
    runtime.rider_intent_signals=object()
    class StopBeforeForces(Exception): pass
    def resolve(control,signals,**kwargs):
        assert kwargs['road_grade'] == pytest.approx(.18)
        assert runtime.rider_state.road[-1].x_m == pytest.approx(3.)
        raise StopBeforeForces
    runtime.rider_intent=SimpleNamespace(resolve=resolve)
    with pytest.raises(StopBeforeForces):
        runtime.apply_forces(advance=False)


def test_body_pulse_is_finite_and_has_no_impulsive_endpoint():
    assert body_pulse(0., 1., .5, .1) == 0.
    assert body_pulse(1., 1., .5, .1) == 0.
    assert abs(body_pulse(1.25, 1., .5, .1) - .1) < 1e-12
    assert body_pulse(1.5, 1., .5, .1) == 0.


def test_body_pulse_validates_its_inputs():
    for bad in (math.nan, math.inf):
        for args in ((bad, 1., .5, .1), (1., bad, .5, .1),
                     (1., 1., bad, .1), (1., 1., .5, bad)):
            with pytest.raises(ValueError):
                body_pulse(*args)
    with pytest.raises(ValueError):
        body_pulse(1., 1., 0., .1)
    with pytest.raises(ValueError):
        body_pulse(1., 1., -.5, .1)


def test_body_pulse_is_continuous_around_its_edges():
    amplitude = .1
    assert abs(body_pulse(1. + 1e-9, 1., .5, amplitude)) < 1e-6
    assert abs(body_pulse(1.5 - 1e-9, 1., .5, amplitude)) < 1e-6


def test_posture_program_rate_limits_and_bounds_the_pulse():
    config = SeatedClimbConfig(enabled=True, lean_gain=0.,
        max_forward_lean_rad=.35, max_backward_lean_rad=.1,
        lean_rate_rad_s=.5)
    program = SeatedPostureProgram(config)
    # A pulse amplitude above the lean envelope is clipped by the strategy
    # bound, not by widening the envelope.
    program.schedule_pulse(1., .5, 10.)
    dt = .01
    lean = 0.
    for i in range(80):  # to t = 0.8 s, before the pulse window
        lean = program.update(i*dt, 0., dt)
    assert lean == pytest.approx(0., abs=1e-12)
    # Mid-pulse the intent rises but never instantaneously and never beyond
    # the strategy envelope, even though amplitude 10 rad was requested.
    peak = 0.
    for i in range(80, 160):
        lean = program.update(i*dt, 0., dt)
        peak = max(peak, lean)
    assert 0. < peak <= config.max_forward_lean_rad + 1e-12
    # No instant jump: consecutive intents differ by at most the rate bound.
    program2 = SeatedPostureProgram(config)
    program2.schedule_pulse(.2, .6, .3)
    previous = 0.
    for i in range(200):
        current = program2.update(i*dt, 0., dt)
        assert current - previous <= config.lean_rate_rad_s*dt + 1e-12
        previous = current


def test_posture_program_rejects_nonfinite_or_reversed_schedules():
    program = SeatedPostureProgram(SeatedClimbConfig(enabled=True))
    with pytest.raises(ValueError):
        program.schedule_pulse(math.nan, .5, .1)
    with pytest.raises(ValueError):
        program.schedule_pulse(0., 0., .1)
    with pytest.raises(ValueError):
        program.schedule_pulse(0., .5, math.inf)


def _environment(tmp_path, grade=0.18, duration_s=6.):
    from bike_sim.cli import research as research_cli
    track = tmp_path / 'track.toml'
    track.write_text(
        'name = "probe"\nlength_m = 200.0\nsurface = "hardpack"\n'
        f'grade_profile = {{ knots = [[0.0, 0.0], [5.0, {grade}], '
        f'[200.0, {grade}]] }}\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(WELDED), '--track-file', str(track),
        '--duration', str(duration_s), '--dt', '.0005', '--initial-speed', '0',
        '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _run(tmp_path, pulse=None, duration_s=6.):
    """Run one graded episode; return per-call grip/power/chest channels."""
    from bike_sim.sim.ride.control import RideControl
    tmp_path.mkdir(parents=True, exist_ok=True)
    env = _environment(tmp_path, duration_s=duration_s)
    physical = env.sim.physical
    if pulse is not None:
        physical.rider_intent.schedule_pulse(*pulse)
    torso = None
    for i in range(env.sim.model.nbody):
        if env.sim.model.body(i).name in ('torso', 'rider_torso', 'rider_body'):
            torso = i
    grip_pull = []
    grip_pull_max = 0.
    power = []
    chest_z = []
    times = []
    infeasible = 0
    for _ in range(6000):
        if env.done:
            break
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=30.),
                 front_brake_demand=0., rear_brake_demand=0.)
        samples = physical.attachment_samples or {}
        grip = sum(max(0., s.pull_n) for k, s in samples.items()
                   if k.startswith('grip_'))
        for k, s in samples.items():
            if k.startswith('grip_'):
                grip_pull_max = max(grip_pull_max, s.pull_n)
        grip_pull.append(grip)
        power.append(
            physical.rider_control.effort_diagnostics['rider_positive_power_w'])
        if 'rider_controller.infeasible' in physical.step_violations:
            infeasible += 1
        if torso is not None:
            chest_z.append(float(env.sim.data.xpos[torso][2]))
        times.append(float(env.sim.data.time))
    return {'time': np.asarray(times), 'grip_pull_n': np.asarray(grip_pull),
            'power_w': np.asarray(power), 'chest_z_m': np.asarray(chest_z),
            'grip_pull_max_n': grip_pull_max,
            'infeasible': infeasible, 'env': env}


@pytest.mark.slow
def test_pulse_redistributes_load_through_inertia_only(tmp_path):
    pulse = (2.5, .5, .15)
    calm = _run(tmp_path / 'calm')
    surged = _run(tmp_path / 'pulse', pulse=pulse)
    # Identical runs until the pulse window: the trajectories match up to the
    # schedule point, so the comparison isolates the pulse contribution.
    start = pulse[0]
    window = (surged['time'] >= start) & (surged['time'] <= start + pulse[1])
    calm_window = np.interp(surged['time'][window], calm['time'],
                            calm['grip_pull_n'])
    delta = surged['grip_pull_n'][window] - calm_window
    # The bounded thrust moves segment inertia through the joints; the bar
    # reaction may only change through that inertia, so a difference must
    # exist yet each hand stays inside the declared 300 N pull budget.
    assert np.abs(delta).max() > 1e-6
    assert surged['grip_pull_max_n'] <= 300. + 1e-6
    assert surged['power_w'].max() <= 450. + 1e-6
    # Chest drops during the lean intent: geometric check that the posture
    # program produces a genuinely forward, low torso, not a cosmetic pose.
    if surged['chest_z_m'].size and calm['chest_z_m'].size:
        chest_delta = (np.interp(surged['time'][window], calm['time'],
                                 calm['chest_z_m'])
                       - surged['chest_z_m'][window])
        assert chest_delta.max() > 0.


def test_trim_is_delayed_and_increases_forward_lean_for_light_front():
    cfg = SeatedClimbConfig(enabled=True, lean_rate_rad_s=10., trim_dead_time_s=.2)
    program = SeatedPostureProgram(cfg)
    base = math.atan(.15)
    for i in range(20):
        lean = program.update(i*.01, .15, .01, front_load_share=.20)
    assert lean == pytest.approx(base)
    for i in range(20, 300):
        lean = program.update(i*.01, .15, .01, front_load_share=.20)
    assert base+.05 < lean <= base+.15+1e-9

def test_geometric_limit_caps_lean():
    program = SeatedPostureProgram(SeatedClimbConfig(enabled=True, lean_rate_rad_s=10., max_forward_lean_rad=.8))
    for i in range(200):
        lean = program.update(i*.01, .45, .01, lean_limit_rad=.2)
    assert lean == pytest.approx(.2)

def test_preview_is_last_sample_and_preserves_exact_endpoint():
    from bike_sim.sim.ride.rider_state import RoadSample, road_grade_preview, road_samples
    import numpy as np
    assert road_grade_preview((RoadSample(0.,0.,.1), RoadSample(2.,.4,.3))) == .3
    samples = road_samples(np.array([[0.,0.], [4.,.4]]), (0.,1.), 2.03)
    assert samples[-1].x_m == pytest.approx(3.03)
    with pytest.raises(ValueError):
        road_grade_preview(())
