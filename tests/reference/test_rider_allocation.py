"""Constrained seated-effort allocation (R4).

The 0.75*rider_weight support heuristic is replaced by an explicit model:
rider inverse dynamics, attachment closures and budgets, directional joint
strength, a circular grip limit, and the summed positive-power cap. Intent is
only the objective; violating a budget is an invalid controller state, never
a legitimate stall.
"""
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import LinearConstraint, NonlinearConstraint

from bike_sim.physics.rider_allocation import (
    Allocation, allocate_effort, grip_constraints, inverse_dynamics_rows,
)

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'


def _capture_support_problem(monkeypatch, *, name='saddle', coupled=True, reserve=.15):
    """Exercise production constraint assembly on a declared one-DOF stand."""
    from types import SimpleNamespace
    import mujoco
    from bike_sim.physics.model_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    import bike_sim.physics.rider_allocation as allocation
    if coupled:
        from bike_sim.sim.ride.rider_response_allocation import support_rows
        force_map=np.zeros((3,5));force_map[0,2]=force_map[1,3]=300.
        g,h,floor=support_rows(force_map,np.array([0.,1.]),
            kind='saddle' if name=='saddle' else 'foot',
            config=ArticulatedConfig(saddle_reserve_weight_fraction=reserve),
            weight_n=80.*9.81,half_length_m=.05)
        return {'g':g,'h':h},{'saddle_normal_lower_bound_n':floor}
    controller = ArticulatedRiderController.__new__(ArticulatedRiderController)
    controller.config = ArticulatedConfig(saddle_reserve_weight_fraction=reserve)
    controller._rider_dofs, controller._bike_dofs = np.array([0]), np.array([], dtype=int)
    controller.joints = {'joint': (0,0,0)}
    controller._alloc_attachments = {name: {'eq':0 if coupled else -1}}
    controller._fullM = np.zeros((1,1))
    controller._envelope_dof_adrs = np.array([], dtype=int)
    controller.envelope_forces = lambda *args:(np.zeros(1), {})
    controller._attachment_frames = lambda *args:({name:np.zeros((2,1))}, [])
    controller._active_recovery = {side:SimpleNamespace(stage='none') for side in ('front','rear')}
    controller.rider_mass = 80.
    controller.strength = None
    controller._last_branch = controller._last_solution = None
    model = SimpleNamespace(nv=1, opt=SimpleNamespace(gravity=np.array([0.,0.,-9.81])),
        actuator_forcelimited=np.array([False]),actuator_ctrllimited=np.array([False]),
        actuator_forcerange=np.zeros((1,2)),actuator_ctrlrange=np.zeros((1,2)))
    data = SimpleNamespace(qacc=np.zeros(1), qvel=np.zeros(1), qfrc_bias=np.zeros(1))
    monkeypatch.setattr(mujoco,'mj_fullM',lambda m,d,out:out.__setitem__(slice(None), np.eye(1)))
    captured = {}
    def capture(target, ae, be, g, h, lo, hi, **kwargs):
        captured.update(g=g.copy(),h=h.copy(),lo=lo.copy(),hi=hi.copy())
        return Allocation(target.copy(),True,0.)
    monkeypatch.setattr(allocation,'allocate_effort',capture)
    _, diagnostic=controller.allocate(model,data,np.zeros(1),np.zeros(2),
        {name:np.array([0.,1.])},estimates={name:(1000.,0.)})
    return captured, diagnostic


@pytest.mark.parametrize('coupled',[True,False])
@pytest.mark.parametrize('reserve',[0.,.15,.5])
def test_saddle_reserve_is_an_actual_configured_lower_bound(monkeypatch,coupled,reserve):
    captured,diagnostic=_capture_support_problem(monkeypatch,coupled=coupled,reserve=reserve)
    # x=[qddot/500,tau/50,Fx/300,Fz/300,p/450]. Check generated
    # inequality, not only a diagnostic echo or a wish toward that force.
    row=np.array([0.,0.,0.,-300.,0.])
    selected=np.all(np.isclose(captured['g'],row),axis=1)
    assert selected.any(), 'no lower-normal bound in the coupled support problem'
    expected=reserve*80.*9.81
    guard=1e-5 if coupled else 0.
    assert np.min(captured['h'][selected]) == pytest.approx(-expected-guard)
    assert diagnostic['saddle_normal_lower_bound_n'] == pytest.approx(expected)
    from bike_sim.physics.attachment_budget import AttachmentSample, attachment_violations
    assert 'normal' in attachment_violations(AttachmentSample('saddle',0.,0.,0.,0.))


@pytest.mark.parametrize('name', ['front', 'rear', 'saddle'])
def test_coupled_support_cone_allows_compression_and_rejects_tension(monkeypatch,name):
    problem,_=_capture_support_problem(monkeypatch,name=name)
    def allowed(fx,fz):
        x=np.array([0.,0.,fx/300.,fz/300.,0.])
        return np.all(problem['g']@x <= problem['h']+1e-10)
    assert allowed(0.,200.), 'a compressive supported foot/saddle must be feasible'
    assert not allowed(0.,-200.)
    if name != 'saddle':
        assert allowed(.9*200.-1e-4,200.)
        assert not allowed(.901*200.,200.)
        assert not allowed(0.,19.99)
    else:
        assert allowed(.6*200.-1e-4,200.)
        assert not allowed(.601*200.,200.)


@pytest.mark.parametrize('activation_tau',[0.,.04])
def test_finalizer_delivers_allocated_torque_instead_of_rebuilding_raw_wish(activation_tau):
    from types import SimpleNamespace
    from bike_sim.physics.model_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_effort import finalize_effort
    c=SimpleNamespace(config=ArticulatedConfig(active_positive_power_limit_w=450.,activation_tau_s=activation_tau),
        joints={'joint':(0,0,0)},last_terms={'joint':{'requested_nm':50.}},
        strength=None, active_state=np.zeros(1),activation_time_s=None,
        strength_limited=lambda active,q,v:(active,()))
    data=SimpleNamespace(qvel=np.ones(1),qpos=np.zeros(1),time=0.)
    applied=finalize_effort(c,data,{'joint':10.},advance=True,dt_s=.005,steady_state=False)
    assert applied == {'joint':10.}
    assert c.last_terms['joint']['requested_nm'] == 50.
    assert c.effort_diagnostics['rider_active_delivered_nm'] == applied


def test_support_moment_constraints_reject_cop_outside_the_physical_patch():
    from bike_sim.physics.model_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_response_allocation import support_rows
    for kind,half,normal in (('foot',.05,100.),('saddle',.045,200.)):
        g,h,_=support_rows(np.eye(3),np.array([0.,1.]),kind=kind,
            config=ArticulatedConfig(),weight_n=80.*9.81,half_length_m=half)
        assert np.all(g@np.array([0.,normal,half*normal-1e-5]) <= h+1e-12)
        assert np.any(g@np.array([0.,normal,half*normal+1e-3]) > h)
    from bike_sim.physics.attachment_budget import AttachmentSample,attachment_violations
    assert attachment_violations(AttachmentSample('foot',100.,90.,5.,0.,half_patch_m=.05)) == ()


def test_activation_dynamics_shape_the_wish_before_support_allocation():
    from types import SimpleNamespace
    from bike_sim.physics.model_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_effort import activation_target
    c=SimpleNamespace(config=ArticulatedConfig(activation_tau_s=.04),active_state=np.array([10.]))
    wished,gain=activation_target(c,np.array([50.]),dt_s=.005,steady_state=False)
    expected=10.+(50.-10.)*(1.-math.exp(-.005/.04))
    np.testing.assert_allclose(wished,[expected],atol=1e-12)
    assert gain == pytest.approx(1.-math.exp(-.005/.04))
    np.testing.assert_array_equal(c.active_state,[10.])


@pytest.mark.parametrize('with_resistance',[False,True])
def test_allocation_forecast_includes_current_assembled_support_forces(with_resistance):
    from types import SimpleNamespace
    import mujoco
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    from bike_sim.sim.ride.control_clock import ControlClock
    from bike_sim.sim.ride.force_accumulator import ForceAccumulator
    model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint type="slide" axis="0 0 1"/>'
        '<inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody></mujoco>')
    data=mujoco.MjData(model); mujoco.mj_forward(model,data)
    runtime=PhysicalRuntime.__new__(PhysicalRuntime)
    runtime.cfg=SimulationPhysicsConfig(physics_mode='physical',drive_mode='articulated_effort')
    runtime.control_clock=ControlClock(.0005,.005)
    runtime.vertices=np.array([[-1.,0.],[3.,0.]])
    runtime._wheel_bodies=(1,1)
    runtime.tire=None
    if with_resistance:
        runtime.tire=SimpleNamespace(compute_qfrc=lambda *args,**kwargs:np.zeros(1),
            snapshots={},probe_snapshots={})
        runtime.resistance=SimpleNamespace(compute_components=lambda *args:{'wind':np.ones(1)})
    runtime.brake=SimpleNamespace(apply=lambda *args:None)
    runtime.drive=SimpleNamespace(prepare_pedaling=lambda *args,**kwargs:SimpleNamespace(
        target_phase_rad=None,target_rate_rad_s=0.,effort_nm=0.),ideal_hub=None,
        compute_components=lambda *args,**kwargs:{})
    runtime.rider_contacts=SimpleNamespace(compute_qfrc=lambda *args,**kwargs:np.zeros(1),
        probe_diagnostics={},probe_enabled={},delivered_crank_torque_nm=0.)
    class ForecastObserved(Exception): pass
    def compute(m,d,*args,**kwargs):
        np.testing.assert_allclose(d.qacc,[1. if with_resistance else 0.],atol=1e-12)
        assert args[0].crank_target_rate_rad_s == 8.
        raise ForecastObserved
    runtime.rider_control=SimpleNamespace(compute=compute,envelope_forces=lambda *args:(np.zeros(1),{}))
    runtime.sim=SimpleNamespace(model=model,data=data,steps=0,speed_mps=0.,rider=SimpleNamespace(variant='articulated_planar'),
        contacts=SimpleNamespace(rear_snapshot=None,rear_controller_grounded=True),
        force_accumulator=ForceAccumulator(model.nv),rider_forces=SimpleNamespace(active=False),
        applier=SimpleNamespace(compute_qfrc_components=lambda *args:{'support':np.array([9.81])}))
    with pytest.raises(ForecastObserved):
        from bike_sim.sim.ride.control import RideControl
        runtime.apply_forces(advance=False,control=RideControl(crank_target_rate_rad_s=8.))


# --- plan step-1 checks -----------------------------------------------------

def test_intent_is_relaxed_but_physical_limit_is_not():
    result = allocate_effort(np.array([100.]), np.zeros((0, 1)), np.zeros(0),
        np.array([[1.]]), np.array([80.]), np.array([0.]), np.array([200.]))
    assert result.feasible
    assert abs(result.solution[0] - 80.) < 1e-6


def test_infeasible_support_is_not_reported_as_valid_stall():
    result = allocate_effort(np.array([0.]), np.array([[1.]]), np.array([-1.]),
        np.zeros((0, 1)), np.zeros(0), np.array([20.]), np.array([100.]))
    assert not result.feasible


def test_scaled_grip_uses_a_circle_not_a_componentwise_square():
    # Optimization coordinates are dimensionless; force_map restores newtons.
    scale = 300.
    constraints = grip_constraints(scale*np.eye(2), np.ones(2)/np.sqrt(2),
                                   pulling=True)
    result = allocate_effort(np.array([-250., -250.])/scale,
        np.zeros((0, 2)), np.zeros(0), np.zeros((0, 2)), np.zeros(0),
        np.full(2, -1000.)/scale, np.full(2, 1000.)/scale,
        extra_constraints=constraints)
    assert result.feasible
    force = scale*result.solution
    assert np.linalg.norm(force) <= 300.
    assert np.linalg.norm(force-np.array([-250., -250.])) > 50.


# --- kernel rigour ----------------------------------------------------------

def test_diagonal_pull_beyond_the_circle_is_infeasible():
    # (250, 250) N has norm ~353 N: allowed by a 300 N per-component square,
    # rejected by the circle.
    scale = 300.
    constraints = grip_constraints(scale*np.eye(2), np.ones(2)/np.sqrt(2),
                                   pulling=True)
    result = allocate_effort(np.zeros(2),
        np.eye(2), np.array([-250., -250.])/scale,
        np.zeros((0, 2)), np.zeros(0),
        np.full(2, -1000.)/scale, np.full(2, 1000.)/scale,
        extra_constraints=constraints)
    assert not result.feasible


def test_grip_constraint_validation():
    with pytest.raises(ValueError, match='unit direction'):
        grip_constraints(np.eye(2), np.array([2., 0.]), pulling=True)
    with pytest.raises(ValueError, match='unit direction'):
        grip_constraints(np.eye(2), np.ones(2)/np.sqrt(2),
                         pulling=True, limit_n=0.)
    with pytest.raises(ValueError, match='planar grip map'):
        grip_constraints(np.ones((3, 3)), np.ones(2)/np.sqrt(2), pulling=True)


def test_pressing_branch_bounds_only_the_sign():
    # A pressing hand has no magnitude limit inside grip_constraints: the
    # projection sign is enforced, the size is left to joint budgets.
    scale = 300.
    constraints = grip_constraints(scale*np.eye(2), np.ones(2)/np.sqrt(2),
                                   pulling=False)
    result = allocate_effort(np.array([400., 400.])/scale,
        np.zeros((0, 2)), np.zeros(0), np.zeros((0, 2)), np.zeros(0),
        np.full(2, -1000.)/scale, np.full(2, 1000.)/scale,
        extra_constraints=constraints)
    assert result.feasible
    np.testing.assert_allclose(result.solution, np.array([400., 400.])/scale,
                               atol=1e-6)


def test_inverse_dynamics_rows_layout_and_sign():
    mass = np.array([[2., .1], [.1, 3.]])
    actuation = np.array([[1.], [0.]])
    support = np.array([[.5, 0.], [0., .5]])
    bias = np.array([9., -4.])
    known = np.array([1., 2.])
    aeq, beq = inverse_dynamics_rows(mass, actuation, support, bias, known)
    assert aeq.shape == (2, 5)
    np.testing.assert_allclose(aeq[:, :2], mass)
    np.testing.assert_allclose(aeq[:, 2], -actuation[:, 0])
    np.testing.assert_allclose(aeq[:, 3:], -support.T)
    np.testing.assert_allclose(beq, known - bias)


def test_allocation_rejects_bad_problem_data():
    with pytest.raises(ValueError, match='finite'):
        allocate_effort(np.array([np.nan]), np.zeros((0, 1)), np.zeros(0),
            np.zeros((0, 1)), np.zeros(0), np.array([0.]), np.array([1.]))
    with pytest.raises(ValueError, match='reversed'):
        allocate_effort(np.array([0.]), np.zeros((0, 1)), np.zeros(0),
            np.zeros((0, 1)), np.zeros(0), np.array([2.]), np.array([1.]))


def test_nonfinite_constraint_eval_reports_infinite_violation():
    bad = NonlinearConstraint(lambda x: np.array([np.nan]), -np.inf, 1.)
    result = allocate_effort(np.array([0.5]), np.zeros((0, 1)), np.zeros(0),
        np.zeros((0, 1)), np.zeros(0), np.array([0.]), np.array([1.]),
        extra_constraints=[bad])
    assert not result.feasible
    assert result.violation == float('inf')
    assert np.isfinite(result.solution).all()


def test_positive_power_cap_is_a_sum_not_a_net():
    # z = [tau(3), p(3)]; p_j >= tau_j*v_j, p_j >= 0, sum(p) <= 450.
    # Intent requests powers (400, -350, 100) W: the honest bound is 500 W of
    # positive work, so the cap must bite even though the net is only 150 W.
    speeds = np.array([2., -1., 1.])
    n = 6
    i_t, i_p = slice(0, 3), slice(3, 6)
    g_rows = []
    for j in range(3):
        row = np.zeros(n); row[j] = speeds[j]; row[3+j] = -1.
        g_rows.append(row)
    row = np.zeros(n); row[i_p] = 1.
    g_rows.append(row)
    g = np.vstack(g_rows)
    h = np.array([0., 0., 0., 450.])
    target = np.zeros(n)
    target[i_t] = np.array([400., -350., 100.])/speeds
    lo = np.full(n, -np.inf); hi = np.full(n, np.inf); lo[i_p] = 0.
    result = allocate_effort(target, np.zeros((0, n)), np.zeros(0),
        g, h, lo, hi)
    assert result.feasible
    tau = result.solution[i_t]
    positive = float(np.maximum(tau*speeds, 0.).sum())
    assert positive <= 450. + 1e-6
    assert positive < 500. - 1e-6


def test_isometric_joint_does_not_divide_by_zero_speed():
    # v == 0 must not produce a 0/0 power cap: a bounded isometric torque is
    # perfectly feasible.
    speeds = np.array([0., 1.])
    n = 4
    g = np.zeros((3, n))
    g[0, 0] = speeds[0]; g[0, 2] = -1.
    g[1, 1] = speeds[1]; g[1, 3] = -1.
    g[2, 2] = 1.; g[2, 3] = 1.
    h = np.array([0., 0., 450.])
    target = np.array([30., 5., 0., 0.])
    lo = np.array([-50., -50., 0., 0.]); hi = np.array([50., 50., 450., 450.])
    result = allocate_effort(target, np.zeros((0, n)), np.zeros(0),
        g, h, lo, hi)
    assert result.feasible
    assert result.solution[0] == pytest.approx(30., abs=1e-6)
    assert np.isfinite(result.solution).all()


def test_solution_is_deterministic_for_identical_inputs():
    args = (np.array([3., -2.]), np.array([[1., 1.]]), np.array([1.]),
            np.array([[1., 0.]]), np.array([2.]),
            np.array([-5., -5.]), np.array([5., 5.]))
    first = allocate_effort(*args)
    second = allocate_effort(*args)
    assert first.feasible and second.feasible
    np.testing.assert_array_equal(first.solution, second.solution)


def test_candidate_is_preserved_when_infeasible():
    result = allocate_effort(np.array([0.]), np.array([[1.]]), np.array([-1.]),
        np.zeros((0, 1)), np.zeros(0), np.array([20.]), np.array([100.]))
    assert not result.feasible
    assert result.solution.shape == (1,)
    assert np.isfinite(result.solution).all()


# --- adapter: whole-rider balance on the compiled plant ----------------------

def test_warm_start_does_not_change_the_answer():
    args = (np.array([100., -20.]), np.zeros((0, 2)), np.zeros(0),
            np.array([[1., 0.]]), np.array([80.]),
            np.array([0., -50.]), np.array([200., 50.]))
    cold = allocate_effort(*args)
    warm = allocate_effort(*args, x0=np.array([79., -19.]))
    assert cold.feasible and warm.feasible
    np.testing.assert_allclose(warm.solution, cold.solution, atol=1e-7)


def test_warm_start_from_feasible_point_solves_once(monkeypatch):
    from bike_sim.physics import rider_allocation
    calls = []
    original = rider_allocation.minimize
    def counting(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)
    monkeypatch.setattr(rider_allocation, 'minimize', counting)
    allocate_effort(np.array([100., -20.]), np.zeros((0, 2)), np.zeros(0),
                    np.array([[1., 0.]]), np.array([80.]),
                    np.array([0., -50.]), np.array([200., 50.]),
                    x0=np.array([80., -20.]))
    assert len(calls) == 1


@pytest.mark.parametrize('x0', [np.zeros(3), np.array([np.nan, 0.])])
def test_invalid_warm_start_is_rejected(x0):
    with pytest.raises(ValueError, match='warm start'):
        allocate_effort(np.ones(2), np.zeros((0, 2)), np.zeros(0),
                        np.zeros((0, 2)), np.zeros(0), -np.ones(2), np.ones(2), x0=x0)


def test_branch_memory_preserves_cold_ranking_and_full_fallback():
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    from bike_sim.physics.rider_allocation import Allocation
    controller = ArticulatedRiderController.__new__(ArticulatedRiderController)
    controller._last_branch = controller._last_solution = None
    target = np.array([0.])
    calls = []
    def solve(branch, x0):
        calls.append(branch)
        distance = {(True, True): 4., (True, False): 3.,
                    (False, True): 2., (False, False): 1.}[branch]
        return Allocation(np.array([distance]), True, 0.)
    _, branch = controller._select_allocation_branch(target, solve)
    assert len(calls) == 4 and branch == (False, False)
    calls.clear()
    _, branch = controller._select_allocation_branch(target, solve)
    assert calls == [(False, False)]
    calls.clear()
    def switched(branch, x0):
        result = solve(branch, x0)
        if branch == (False, False):
            return Allocation(result.solution, False, 1.)
        return result
    _, remembered = controller._select_allocation_branch(target, switched)
    assert len(calls) == 4
    controller._last_branch = controller._last_solution = None
    calls.clear()
    _, cold = controller._select_allocation_branch(target, switched)
    assert len(calls) == 4 and remembered == cold == (False, True)


def test_vector_nonlinear_bounds_and_extra_equalities_keep_their_semantics():
    extra = [LinearConstraint(np.array([[1., 1.]]), 1., 1.),
             NonlinearConstraint(lambda x: x, -np.inf, .8,
                                 jac=lambda x: np.eye(2))]
    result = allocate_effort(np.array([2., 0.]), np.zeros((0, 2)), np.zeros(0),
        np.zeros((0, 2)), np.zeros(0), np.zeros(2), np.ones(2), extra_constraints=extra)
    assert result.feasible
    np.testing.assert_allclose(result.solution, [.8, .2], atol=1e-7)


@pytest.mark.parametrize('x0',[None,np.array([1.])])
def test_nonlinear_constraint_is_evaluated_at_the_bounded_start(x0):
    # Nonlinear functions can have a domain narrower than the wish. The old
    # SLSQP conversion saw the clipped start, never an out-of-domain target.
    extra=NonlinearConstraint(lambda x: math.log(float(x[0])),-np.inf,1.,
                              jac=lambda x: np.array([1./x[0]]))
    result=allocate_effort(np.array([-5.]),np.zeros((0,1)),np.zeros(0),
        np.zeros((0,1)),np.zeros(0),np.array([.1]),np.array([2.]),
        extra_constraints=[extra],x0=x0)
    assert result.feasible
    np.testing.assert_allclose(result.solution,[.1],atol=1e-7)


def test_real_grip_problem_changes_from_remembered_pull_to_press():
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    controller = ArticulatedRiderController.__new__(ArticulatedRiderController)
    controller._last_branch = controller._last_solution = None
    calls = []
    def problem(target, prescribed=False):
        def solve(branch, x0):
            calls.append(branch)
            extra = []
            for i, pulling in enumerate(branch):
                force_map = np.zeros((2, 2)); force_map[0, i] = 1.
                extra.extend(grip_constraints(force_map, np.array([1., 0.]), pulling=pulling))
            return allocate_effort(target, np.eye(2) if prescribed else np.zeros((0, 2)),
                target if prescribed else np.zeros(0), np.zeros((0, 2)), np.zeros(0),
                np.full(2, -1000.), np.full(2, 1000.), extra_constraints=extra, x0=x0)
        return controller._select_allocation_branch(target, solve)
    first, branch = problem(np.array([-100., -100.]))
    assert first.feasible and branch == (True, True) and len(calls) == 4
    calls.clear()
    second, remembered = problem(np.array([100., 100.]), prescribed=True)
    assert second.feasible and remembered == (False, False) and len(calls) == 4
    controller._last_branch = controller._last_solution = None
    cold, branch = problem(np.array([100., 100.]), prescribed=True)
    assert branch == remembered and cold.feasible
    np.testing.assert_allclose(second.solution, cold.solution, atol=1e-7)

def _environment(tmp_path, config=WELDED):
    from bike_sim.cli import research as research_cli
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(config), '--track-file', str(track),
        '--duration', '1.', '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--initial-speed', '0',
        '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _controller(env):
    return env.sim.physical.rider_control


@pytest.mark.slow
def test_healthy_step_allocates_a_feasible_command(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(40):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    diagnostics = _controller(env).allocation_diagnostics
    assert diagnostics['feasible']
    assert not diagnostics['invalid_controller']
    assert 'rider_controller.infeasible' not in physical.step_violations
    channel = physical.sample.channels['rider_allocation']
    assert channel['feasible'] is True
    assert math.isfinite(channel['violation'])


@pytest.mark.slow
def test_allocated_wrenches_close_the_rider_dynamics(tmp_path):
    """The solved z must satisfy M*qddot + bias = B*tau + J.T*f + known.

    Force and moment balance about the saddle are rows of this same equality,
    so its residual is the honest whole-body check rather than a second
    statics add-on. The recorded z lives at the interval start: rewind the
    model there before rebuilding the rows.
    """
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(30):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    controller = _controller(env)
    diagnostics = controller.allocation_diagnostics
    assert diagnostics['feasible']
    model, data = env.sim.model, env.sim.data
    sample = physical.sample
    # Allocation is held between control ticks; rebuild dynamics at its actual
    # incoming tick state rather than the last interval of that period.
    step = round(diagnostics['solution_time_s']/physical.control_clock.timestep_s)
    assert physical.control_clock.is_tick(step)
    assert step == sample.interval_id-sample.interval_id % physical.control_clock.steps_per_period
    data.qpos[:] = diagnostics['solution_qpos']
    data.qvel[:] = diagnostics['solution_qvel']
    import mujoco
    mujoco.mj_forward(model, data)
    qddot = np.asarray(diagnostics['solution_qddot'])
    wrenches = np.asarray(diagnostics['solution_wrenches'])
    qacc_b = np.asarray(diagnostics['solution_base_qacc'])
    rd, bd = controller._rider_dofs, controller._bike_dofs
    attach = tuple(controller._alloc_attachments)
    full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, full)
    bias = np.asarray(data.qfrc_bias[rd]) + full[np.ix_(rd, bd)] @ qacc_b
    # The forward-response allocator retains each welded support moment at
    # the same sole/grip point as its recorded force, rather than omitting My.
    jac=[];expanded=[]
    moments=np.asarray(diagnostics['solution_moments'])
    for i,name in enumerate(attach):
        from bike_sim.sim.ride.rider_response_allocation import allocation_support_point
        point=allocation_support_point(model,data,controller,name,controller._alloc_attachments[name])
        jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jp,jr,point,controller._alloc_attachments[name]['rider_body'])
        jac.extend((jp[0,rd],jp[2,rd],jr[1,rd]))
        expanded.extend((wrenches[2*i],wrenches[2*i+1],moments[i]))
    jac=np.asarray(jac);wrenches=np.asarray(expanded)
    tau = np.asarray(diagnostics['solution_tau'])
    known = np.zeros(model.nv)
    envelope = controller.envelope_forces(model, data)[0]
    known[controller._envelope_dof_adrs] += envelope[controller._envelope_dof_adrs]
    hinge = np.asarray([controller.joints[n][1] for n in controller.joints])
    known[hinge] -= controller.config.joint_kd_nms_rad*data.qvel[hinge]
    b_torque = np.zeros(model.nv)
    for index, name in enumerate(controller.joints):
        b_torque[controller.joints[name][1]] += tau[index]
    residual = (full[np.ix_(rd, rd)] @ qddot + bias
                - b_torque[rd] - jac.T @ wrenches - known[rd])
    np.testing.assert_allclose(residual, np.zeros(len(rd)), atol=1e-4)


@pytest.mark.slow
def test_command_does_not_read_solved_reactions(tmp_path):
    """Same q/v, perturbed efc_force: the command must not change.

    Solved lambdas are diagnostics; the allocator must derive wrenches from
    its own model, never from the solver's answer.
    """
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.rider_control import RiderCommand
    env = _environment(tmp_path)
    for _ in range(20):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    controller = _controller(env)
    model, data = env.sim.model, env.sim.data
    command = RiderCommand(0., enabled=True,
                           posture=physical.applied_control.posture,
                           crank_target_phase_rad=None)
    state = physical.rider_state
    first = controller.compute(model, data, command, kinematic_state=state,
                               support_available=None, advance=False)
    perturbed = np.random.default_rng(7).normal(size=data.nefc)
    data.efc_force[:data.nefc] += perturbed
    second = controller.compute(model, data, command, kinematic_state=state,
                                support_available=None, advance=False)
    for name in first:
        assert first[name] == pytest.approx(second[name], abs=1e-9)


@pytest.mark.slow
def test_infeasible_intent_keeps_the_physical_bounds(tmp_path):
    """An absurd torque intent is relaxed, never exceeds strength bounds."""
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(20):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    controller = _controller(env)
    model, data = env.sim.model, env.sim.data
    n_t = len(controller.joints)
    names = tuple(controller.joints)
    speed = np.asarray([data.qvel[controller.joints[n][1]] for n in names])
    angle = np.asarray([controller.anatomical_joint_angle(
        n, data.qpos[controller.joints[n][0]]) for n in names])
    tau, diagnostics = controller.allocate(
        model, data, np.full(n_t, 1e4),
        np.zeros(2*len(controller._alloc_attachments)), {},
        estimates=None)
    assert diagnostics['feasible']
    for index, name in enumerate(names):
        cap = (controller.config.joint_limit_nm if controller.strength is None
               else max(
                   controller.strength_capacity(name, angle[index], speed[index], 1.),
                   controller.strength_capacity(name, angle[index], speed[index], -1.)))
        assert abs(tau[index]) <= cap + 1e-6


def test_allocate_effort_returns_target_without_slsqp_when_target_is_feasible(monkeypatch):
    from bike_sim.physics import rider_allocation as module
    target = np.array([1., -2.])
    zeros = np.zeros(0)
    lower = np.array([-10., -10.])
    upper = np.array([10., 10.])
    calls = []

    def fail_minimize(*args, **kwargs):
        calls.append(True)
        raise AssertionError('SLSQP must not run for an already-feasible target')

    monkeypatch.setattr(module, 'minimize', fail_minimize)
    result = module.allocate_effort(target, np.zeros((0, 2)), zeros,
        np.zeros((0, 2)), zeros, lower, upper)
    np.testing.assert_array_equal(result.solution, target)
    assert result.feasible and result.violation == 0.
    assert not calls
