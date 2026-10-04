"""Experimental coupled task control for the fully welded rider (opt-in):
the crank torque is the allocator's task coordinate and tissue damping is
declared apart from the muscle gain. Physical acceptance is NOT claimed here;
see docs/superpowers/audits/2026-10-04-realistic-pedelec-drive.md."""
from pathlib import Path

import numpy as np
import pytest

from bike_sim.physics.physical_config import ArticulatedConfig
from bike_sim.sim.ride.control import RideControl

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT/'examples'/'research'/'viewer_physics_welded.toml'


def test_passive_damping_defaults_to_the_legacy_gain_coupling():
    """None keeps the historical DOF damping == joint_kd; an explicit value is its own."""
    assert ArticulatedConfig().passive_damping_nms_rad == ArticulatedConfig().joint_kd_nms_rad
    assert ArticulatedConfig(joint_passive_damping_nms_rad=.5).passive_damping_nms_rad == .5
    with pytest.raises(ValueError):
        ArticulatedConfig(joint_passive_damping_nms_rad=-1.)


def test_coupled_task_control_is_opt_in():
    assert ArticulatedConfig().coupled_task_control is False
    with pytest.raises(ValueError):
        ArticulatedConfig(coupled_task_control=1)


def test_crank_task_coordinate_is_appended_and_weighted():
    """The soft task is one extra optimisation coordinate in CRANK_TASK_SCALE_NM units,
    and the linear prefilter honours an optional equality block."""
    from bike_sim.sim.ride import rider_response_allocation as rra
    assert rra.CRANK_TASK_SCALE_NM == 1.
    g = np.zeros((1, 2)); g[0, 0] = 1.
    assert not rra.linear_region_infeasible(g, np.array([1.]), np.array([-1., -1.]), np.array([1., 1.]))
    assert rra.linear_region_infeasible(g, np.array([1.]), np.array([-1., -1.]), np.array([1., 1.]),
                                        np.array([[1., 0.]]), np.array([5.]))


def _environment(tmp_path, duration_s):
    """Welded profile with the experimental coupled task control and 0.5 N.m.s/rad tissue damping."""
    from bike_sim.cli import research as research_cli
    text = WELDED.read_text()
    for name in ('joint_envelope_path', 'joint_strength_path'):
        text = text.replace(f'{name} = "', f'{name} = "{WELDED.parent}/')
    text = text.replace('[articulated]\n', '[articulated]\ncoupled_task_control = true\njoint_passive_damping_nms_rad = 0.5\n')
    config = tmp_path/'physics.toml'
    config.write_text(text)
    track = tmp_path/'track.toml'
    track.write_text('name = "probe"\nlength_m = 100.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [100.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(config), '--track-file', str(track),
        '--duration', str(duration_s), '--dt', '.00125', '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--initial-speed', '0', '--out', str(tmp_path/'out')])
    return research_cli.make_environment(args)


@pytest.mark.slow
def test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step(tmp_path):
    """The task coordinate predicts exactly what the pedelec torque sensor will read
    for the first physics step of the control period (same multipliers, same J^T row)."""
    env = _environment(tmp_path, .6)
    physical = env.sim.physical
    controller = physical.rider_control
    physical.set_record_decimation(1)
    checked = 0
    while not env.done:
        env.step(RideControl())
        samples = list(physical.completed_samples)
        planned = controller.allocation_diagnostics.get('solution_crank_torque_nm')
        if planned is None or not samples or controller.allocation_diagnostics.get('effort_scale') == 0.:
            continue
        solved = samples[0].channels['rider_welds']['crank_torque_nm']
        assert abs(planned-solved) < 1e-3*max(1., abs(planned)), (env.sim.time_s, planned, solved)
        checked += 1
    assert checked >= 20


@pytest.mark.slow
def test_tissue_damping_does_not_eat_the_muscle_budget(tmp_path):
    """With 0.5 N.m.s/rad hinge viscosity the passive loss is a small fraction of
    positive muscle power while pedalling (it was ~160 of 160 W at the legacy 15)."""
    env = _environment(tmp_path, 1.5)
    controller = env.sim.physical.rider_control
    positive = []; passive = []
    while not env.done:
        env.step(RideControl())
        if env.sim.time_s < .5:
            continue
        d = controller.effort_diagnostics
        positive.append(d['rider_positive_power_w']); passive.append(-d['rider_passive_power_w'])
    assert np.mean(positive) > 20.
    assert np.mean(passive) < .3*np.mean(positive), (np.mean(positive), np.mean(passive))
