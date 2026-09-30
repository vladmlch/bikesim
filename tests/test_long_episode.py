import pytest
from bike_sim.cli.research import parser, make_environment
from bike_sim.sim.ride.control import RideControl


@pytest.mark.slow
def test_fifteen_second_episode_completes(tmp_path):
    args = parser().parse_args(['--scenario', 'generated', '--seed', '5', '--duration', '15',
        '--demand', '60', '--ideal-sensors', '--record-decimation', '80', '--out', str(tmp_path/'long')])
    env = make_environment(args)
    while not env.done:
        env.step(RideControl(motor_torque_nm=env.demand_nm or 60.))
    env.save(tmp_path/'long')
    # 'model_violation' and 'numerical_quality' are truncations (the plant left its
    # declared scope), not episode outcomes: a validated long episode must not end in them.
    assert env.reason in ('duration', 'finish', 'crash:loop_out', 'crash:endo')
    assert env.numerically_valid
    assert (tmp_path/'long'/'episode_metrics.json').is_file()
