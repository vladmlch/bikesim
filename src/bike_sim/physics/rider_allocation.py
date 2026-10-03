"""Constrained effort allocation for the seated rider.

A small quadratic program replaces heuristic support splits: the rider's own
inverse dynamics, welded-contact closures, attachment budgets, directional
joint-strength bounds and the whole-body positive-power cap are linear (or
one circle) constraints; the requested torques are only the objective. An
infeasible model means the command cannot hold physical budgets -- it is an
invalid controller state, never a legitimate stall.

Optimization coordinates are dimensionless: callers scale each variable
block by an explicit unit (torque, newton, watt) so ``force_map`` and bounds
restore physical magnitudes inside constraints.
"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, NonlinearConstraint, minimize


@dataclass(frozen=True)
class Allocation:
    solution: np.ndarray
    feasible: bool
    violation: float


def _residual(x, aeq, beq, g, h, extra_constraints, lower, upper):
    """Largest constraint violation at x; inf when any eval is nonfinite."""
    errors = [0., float(np.max(lower - x)), float(np.max(x - upper))]
    if aeq.shape[0]:
        errors.append(float(np.max(np.abs(aeq @ x - beq))))
    if g.shape[0]:
        errors.append(float(np.max(g @ x - h)))
    for constraint in extra_constraints:
        value = (constraint.A @ x if isinstance(constraint, LinearConstraint)
                 else np.atleast_1d(constraint.fun(x)))
        if not np.isfinite(value).all():
            return float('inf')
        errors.extend([float(np.max(constraint.lb - value)),
                       float(np.max(value - constraint.ub))])
    return max(errors)


def inverse_dynamics_rows(mass, actuation, support_jacobian, bias, known_qforce):
    """Rows of ``M*qddot = B*tau + J.T*f + known_qforce - bias`` for z stacking.

    ``mass``/``actuation`` are generalized-force matrices over the allocated
    DOF set, ``support_jacobian`` stacks one block-row pair per attachment
    wrench component, and ``bias`` is every modelled generalized force not
    represented as a variable (gravity, Coriolis, damping, prescribed base
    terms). The constraint acts on ``z=[qddot, tau, f]``.
    """
    return (np.column_stack((mass, -actuation, -support_jacobian.T)),
            np.asarray(known_qforce) - np.asarray(bias))


def allocate_effort(target, aeq, beq, g, h, lower, upper, *,
                    extra_constraints=(), x0=None) -> Allocation:
    """Nearest feasible point to ``target`` under equalities, bounds and extras.

    Feasibility is verified from the returned point itself: the optimizer's
    success flag is not trusted, residuals are measured against every
    constraint family, and a violated bound reports ``feasible=False``.
    """
    target, aeq, beq, g, h, lower, upper = [np.asarray(x, dtype=float)
        for x in (target, aeq, beq, g, h, lower, upper)]
    # +-inf is the honest spelling of "unbounded" inside lower/upper; the
    # coefficient arrays and the intent must be finite everywhere.
    if not all(np.isfinite(x).all() for x in (target, aeq, beq, g, h)):
        raise ValueError('allocation matrices must be finite')
    if np.isnan(lower).any() or np.isnan(upper).any():
        raise ValueError('allocation bounds must not be NaN')
    if np.any(lower > upper):
        raise ValueError('reversed allocation bounds')
    if x0 is not None:
        x0 = np.asarray(x0, dtype=float)
        if x0.shape != target.shape or not np.isfinite(x0).all():
            raise ValueError('warm start must have the target shape and finite values')
    x = np.clip(target if x0 is None else x0, lower, upper)
    # Equalities first, then linear inequalities, then nonlinear extras: the
    # active-set order affects SLSQP's path, and dynamics rows are the most
    # important to satisfy early.
    constraints = []
    if aeq.shape[0]:
        constraints.append({'type': 'eq', 'fun': lambda x: aeq @ x-beq,
                            'jac': lambda x: aeq})
    if g.shape[0]:
        constraints.append({'type': 'ineq', 'fun': lambda x: h-g @ x,
                            'jac': lambda x: -g})
    for constraint in extra_constraints:
        constraints.extend(_as_old_style(constraint, x))

    def objective(x):
        return .5 * np.dot(x - target, x - target)

    def gradient(x):
        return x - target

    # SLSQP occasionally stalls on a line-search step with a nearly feasible
    # iterate; a warm restart from the returned point pushes through without
    # relaxing any residual tolerance.
    for _ in range(4):
        result = minimize(objective, x, jac=gradient,
            bounds=Bounds(lower, upper), constraints=constraints,
            method='SLSQP', options={'ftol': 1e-12, 'maxiter': 500})
        x = np.asarray(result.x, dtype=float)
        if not np.isfinite(x).all():
            break
        candidate = np.clip(x, lower, upper)
        residual = _residual(candidate, aeq, beq, g, h, extra_constraints, lower, upper)
        x = candidate
        if residual <= 1e-7:
            break
    if not np.isfinite(x).all():
        return Allocation(np.clip(np.nan_to_num(x), lower, upper), False, float('inf'))
    violation = _residual(x, aeq, beq, g, h, extra_constraints, lower, upper)
    return Allocation(x.copy(), bool(violation <= 1e-7), violation)


def _as_old_style(constraint, start):
    """Avoid SciPy conversion while retaining equality and bound semantics."""
    if not isinstance(constraint, (LinearConstraint, NonlinearConstraint)):
        raise ValueError('unsupported allocation constraint')
    if isinstance(constraint, LinearConstraint):
        a = np.atleast_2d(np.asarray(constraint.A, dtype=float))
        fun = lambda x: a @ x
        jac = lambda x: a
        shape = (a.shape[0],)
    else:
        fun = lambda x: np.atleast_1d(constraint.fun(x))
        jac = (lambda x: np.atleast_2d(constraint.jac(x))) if callable(constraint.jac) else None
        shape = np.shape(fun(start))
    lb = np.broadcast_to(np.asarray(constraint.lb, dtype=float), shape).reshape(-1)
    ub = np.broadcast_to(np.asarray(constraint.ub, dtype=float), shape).reshape(-1)
    equal = np.isfinite(lb) & (lb == ub)
    result = []
    if equal.any():
        item = {'type': 'eq', 'fun': lambda x: fun(x)[equal]-lb[equal]}
        if jac is not None:
            item['jac'] = lambda x: jac(x)[equal]
        result.append(item)
    for mask, bound, sign in ((np.isfinite(lb) & ~equal, lb, 1.),
                               (np.isfinite(ub) & ~equal, ub, -1.)):
        if mask.any():
            item = {'type': 'ineq',
                'fun': lambda x, mask=mask, bound=bound, sign=sign: sign*(fun(x)[mask]-bound[mask])}
            if jac is not None:
                item['jac'] = lambda x, mask=mask, sign=sign: sign*jac(x)[mask]
            result.append(item)
    return result


def grip_constraints(force_map, pull_direction, *, pulling: bool, limit_n=300.):
    """One hand's bar-force branch: pull is circle-bounded, press is not.

    ``force_map`` maps the optimization vector to the planar force on the
    rider in newtons; ``pull_direction`` is the unit vector from the grip
    toward the rider. A pulling hand may exert at most ``limit_n`` in
    magnitude (the circle, not per-component squares); a pressing hand keeps
    only the unilateral sign, its size bounded elsewhere by joint strength.
    """
    force_map = np.asarray(force_map, dtype=float)
    direction = np.asarray(pull_direction, dtype=float)
    if force_map.ndim != 2 or force_map.shape[0] != 2 or direction.shape != (2,):
        raise ValueError('planar grip map and direction required')
    if not np.isfinite(force_map).all() or not np.isfinite(direction).all():
        raise ValueError('nonfinite grip map')
    if not np.isclose(direction @ direction, 1.) or not np.isfinite(limit_n) or limit_n <= 0:
        raise ValueError('unit direction and positive grip limit required')
    # force_map @ x is force on the rider; its negative acts on the bar.
    projection = -(direction @ force_map)
    branch = projection if pulling else -projection
    result = [LinearConstraint(branch.reshape(1, -1), 0., np.inf)]
    if pulling:
        result.append(NonlinearConstraint(
            lambda x: float(np.sum((force_map @ x)**2)/limit_n**2),
            -np.inf, 1.,
            jac=lambda x: 2.*force_map.T @ (force_map @ x)/limit_n**2))
    return result
