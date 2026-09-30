import json
from bike_sim.cli.research import parser, make_environment
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.metrics import episode_metrics


def test_episode_metrics_schema(tmp_path):
    args = parser().parse_args(['--scenario', 'flat', '--rider', 'lumped', '--duration', '1',
        '--ideal-sensors', '--out', str(tmp_path/'run')])
    env = make_environment(args)
    while not env.done:
        env.step(RideControl(motor_torque_nm=40., human_torque_nm=0.))
    env.save(tmp_path/'run')
    rec = json.loads((tmp_path/'run'/'episode_metrics.json').read_text())
    for key in ('outcome', 'duration_s', 'progress_m', 'motor_pass_fraction',
                'loop_out', 'wheelie', 'numerically_valid', 'model_status'):
        assert key in rec
    assert rec['progress_m'] > 0.
    assert 'wheelie_episode_records' in rec['wheelie']


def test_episode_metrics_track_peak_suspension_travel_at_physics_rate(tmp_path):
    args = parser().parse_args(['--scenario', 'flat', '--rider', 'lumped', '--duration', '.5',
        '--ideal-sensors', '--out', str(tmp_path/'run')])
    env = make_environment(args)
    while not env.done:
        env.step(RideControl(motor_torque_nm=40., human_torque_nm=0.))
    rec = episode_metrics(env)
    # Peak |stroke| is tracked on every physics step, so it cannot be smaller
    # than the final (recorded) stroke and is nonzero under rider weight.
    final = abs(env.sim.physical.sample.channels['suspension']['shock_stroke_m'])
    assert rec['max_shock_stroke_m'] >= final > 0.
    assert rec['max_fork_travel_m'] >= 0.
