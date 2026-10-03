"""The rider planner's information boundary: kinematics and bounded road only.

A planner that could read solved reactions would be optimizing against the
answer rather than predicting it. These tests pin the RiderKinematicState
contract, its independence from solver internals, the bounded road window,
and the unchanged external-sensor contract.
"""
import ast
import dataclasses
from pathlib import Path

import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride.rider_state import (
    RoadSample, RiderKinematicState, rider_kinematic_state, road_samples,
)
from bike_sim.sim.research.sensors import SensorObservation

SRC = Path(__file__).resolve().parents[2] / 'src' / 'bike_sim' / 'sim' / 'ride'
PRIVILEGED = ('efc_force', 'qfrc_constraint', 'efc_J', 'efc_id', 'efc_type',
              'contact_force', 'solver_niter')


def _free_rig():
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="b"><freejoint/>'
        '<geom size=".1" mass="1"/></body></worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _vertices():
    return np.array([[0., 0.], [1., .05], [2., .15], [3., .15]])


def test_kinematic_state_contract_fields():
    assert tuple(f.name for f in dataclasses.fields(RiderKinematicState)) == (
        'qpos', 'qvel', 'rider_time_s', 'road')
    assert tuple(f.name for f in dataclasses.fields(RoadSample)) == (
        'x_m', 'height_m', 'grade')


def test_state_snapshots_and_freezes_the_interval():
    model, data = _free_rig()
    state = rider_kinematic_state(model, data, vertices=_vertices(),
                                  wheel_x_m=(0., 1.), lookahead_m=.2)
    assert state.qpos == tuple(data.qpos) and state.qvel == tuple(data.qvel)
    assert state.rider_time_s == pytest.approx(0.)
    assert type(state.qpos) is tuple and type(state.road) is tuple
    data.qpos[0] = 5.
    assert state.qpos[0] != 5.
    with pytest.raises(dataclasses.FrozenInstanceError):
        object.__getattribute__(state, '__setattr__')('rider_time_s', 9.)


def test_state_is_blind_to_solver_truth():
    model, data = _free_rig()
    before = rider_kinematic_state(model, data, vertices=_vertices(),
                                   wheel_x_m=(0., 1.))
    data.efc_force[:] = 1e6
    data.qfrc_constraint[:] = -1e6
    data.qfrc_applied[:] = 1e6
    after = rider_kinematic_state(model, data, vertices=_vertices(),
                                  wheel_x_m=(0., 1.))
    assert before == after


def test_road_window_is_bounded():
    samples = road_samples(_vertices(), wheel_x_m=(.2, 1.2), lookahead_m=.5,
                           spacing_m=.1)
    xs = [s.x_m for s in samples]
    assert xs == sorted(xs)
    assert .2 in xs and 1.2 in xs
    assert min(xs) >= .2 and max(xs) <= 1.2+.5+1e-12
    by_x = {s.x_m: s for s in samples}
    assert by_x[1.2].height_m == pytest.approx(.07)
    assert by_x[1.2].grade == pytest.approx(.10)
    assert by_x[.2].grade == pytest.approx(.05)
    # Zero lookahead discloses only what is under the wheels.
    only = road_samples(_vertices(), wheel_x_m=(.2, 1.2), lookahead_m=0.)
    assert {s.x_m for s in only} == {.2, 1.2}
    with pytest.raises(ValueError):
        road_samples(_vertices(), wheel_x_m=(), lookahead_m=0.)


def test_planner_sources_do_not_read_solver_internals():
    forbidden = set(PRIVILEGED)
    for name in ('rider_state', 'rider_control', 'pedal_recovery',
                 'rider_support', 'rider_balance', 'rider_effort'):
        tree = ast.parse((SRC / f'{name}.py').read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Attribute):
                assert node.attr not in forbidden, f'{name}:{node.lineno} {node.attr}'
            elif isinstance(node, ast.Name):
                assert node.id not in forbidden, f'{name}:{node.lineno} {node.id}'


def test_sensor_observation_contract_unchanged():
    assert tuple(f.name for f in dataclasses.fields(SensorObservation)) == (
        'time_s', 'source_time_s', 'valid', 'specific_force_body_mps2',
        'pitch_rate_up_rad_s', 'front_wheel_rad_s', 'rear_wheel_rad_s',
        'crank_rad_s', 'motor_torque_nm', 'human_torque_nm')


def test_motor_policy_cannot_take_ownership_of_rider_cadence():
    from types import SimpleNamespace
    from bike_sim.sim.research.policy_session import PolicySession
    from bike_sim.sim.ride.control import RideControl
    session=PolicySession.__new__(PolicySession)
    session.policy=SimpleNamespace(act=lambda *args:RideControl(crank_target_rate_rad_s=8.))
    session.env=SimpleNamespace(done=False,observation=None,demand_nm=None,terminated=False,
        reason=None,error=None,step=lambda *args,**kwargs:None)
    with pytest.raises(ValueError,match='motor policy cannot own rider inputs'):
        session.advance()
    assert session.env.terminated and session.env.reason == 'policy_error'


@pytest.mark.slow
def test_same_kinematic_state_same_command(tmp_path):
    """Identical RiderKinematicState must produce an identical command even
    when the hidden solved reactions and support diagnostics differ."""
    from bike_sim.cli import research as research_cli
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.rider_control import RiderCommand

    root = Path(__file__).resolve().parents[2]
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(root / 'examples' / 'research' / 'viewer_physics_welded.toml'),
        '--track-file', str(track), '--duration', '1.', '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--initial-speed', '0', '--out', str(tmp_path / 'out')])
    env = research_cli.make_environment(args)
    env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
             front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    model, data = env.sim.model, env.sim.data
    controller = physical.rider_control
    state = physical.rider_state
    assert isinstance(state, RiderKinematicState)
    command = RiderCommand(0., enabled=True)
    availability = {'saddle': True, 'front': True, 'rear': True, 'grip': True}

    first = controller.compute(model, data, command, kinematic_state=state,
                               support_available=availability, dt_s=.001)
    # Corrupt every privileged channel the old path used to leak through.
    data.efc_force[:] = 1e9
    data.qfrc_constraint[:] = -1e9
    physical.rider_contacts.diagnostics['front_pedal']['normal_load_n'] = 1e9
    physical.rider_contacts.diagnostics['saddle']['normal_load_n'] = 1e9
    second = controller.compute(model, data, command, kinematic_state=state,
                               support_available=availability, dt_s=.001)
    assert first == pytest.approx(second)
