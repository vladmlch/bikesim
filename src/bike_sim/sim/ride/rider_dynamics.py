"""Model-based command response of the current soft constrained mechanism.

Inputs are M, h, applied/passive forces, constraint geometry and the declared
MuJoCo impedance/reference acceleration. No solved reaction or previous qacc
is an input. Each returned affine response includes the inequalities defining
its unilateral/friction mode; an optimizer cannot use it outside that domain.
"""
from dataclasses import dataclass
import mujoco
import numpy as np


@dataclass(frozen=True)
class ConstraintResponse:
    acceleration_offset: np.ndarray
    acceleration_matrix: np.ndarray
    force_offset: np.ndarray
    force_matrix: np.ndarray
    domain_matrix: np.ndarray
    domain_bound: np.ndarray
    mode: tuple


def attachment_force_map(model, data, dynamics, equality_id, body, point):
    """Map model multipliers to an observable [Fx,Fz,My] body wrench."""
    jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
    mujoco.mj_jac(model,data,jp,jr,np.asarray(point),body)
    spatial=np.vstack((jp,jr))
    columns=np.flatnonzero(np.max(np.abs(spatial),axis=0)>0.)
    if np.linalg.matrix_rank(spatial[[0,2,4]][:,columns],tol=1e-12) != 3:
        raise ValueError('model attachment planar wrench is not observable')
    selected=(dynamics.types==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)) & (dynamics.ids==equality_id)
    generalized=np.zeros_like(dynamics.jacobian.T)
    generalized[:,selected]=dynamics.jacobian[selected].T
    mapping=np.linalg.lstsq(spatial[:,columns].T,generalized[columns],rcond=1e-12)[0]
    if not np.allclose(spatial[:,columns].T@mapping,generalized[columns],rtol=1e-8,atol=1e-8):
        raise ValueError('model attachment wrench does not explain its constraint rows')
    return mapping[[0,2,4]]


class ConstraintDynamics:
    def __init__(self, model, data, actuator_dofs):
        n, nv = data.nefc, model.nv
        mass = np.empty((nv, nv))
        mujoco.mj_fullM(model, data, mass)
        # mj_mulJacVec supports both dense and sparse compiled Jacobians.
        if not mujoco.mj_isSparse(model):
            jac = np.asarray(data.efc_J[:n*nv]).reshape(n,nv).copy()
        else:
            jac = np.empty((n, nv), order='F')
            for column in range(nv):
                basis = np.zeros(nv); basis[column] = 1.
                mujoco.mj_mulJacVec(model, data, jac[:, column], basis)
        self.jacobian = jac
        self.types = np.asarray(data.efc_type[:n], dtype=int).copy()
        self.ids = np.asarray(data.efc_id[:n], dtype=int).copy()
        known = data.qfrc_applied + data.qfrc_passive + data.qfrc_actuator - data.qfrc_bias
        known[np.asarray(actuator_dofs)] -= data.qfrc_actuator[np.asarray(actuator_dofs)]
        actuation = np.zeros((nv, len(actuator_dofs)))
        actuation[np.asarray(actuator_dofs), np.arange(len(actuator_dofs))] = 1.
        inverse = np.linalg.solve(mass, np.column_stack((known, actuation, jac.T)))
        self.smooth_offset = inverse[:, 0]
        self.smooth_matrix = inverse[:, 1:1+len(actuator_dofs)]
        self.inverse_jacobian = inverse[:, 1+len(actuator_dofs):]
        self.matrix = jac @ self.inverse_jacobian + np.diag(data.efc_R[:n])
        self.rhs_offset = np.asarray(data.efc_aref[:n]).copy() - jac @ self.smooth_offset
        self.rhs_matrix = -jac @ self.smooth_matrix
        self.lower = np.full(n, -np.inf)
        self.upper = np.full(n, np.inf)
        equality = self.types == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
        unilateral = np.isin(self.types, [int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT),
            int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON),
            int(mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS),
            int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL)])
        friction = np.isin(self.types, [int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF),
                                      int(mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON)])
        if not np.all(equality | unilateral | friction):
            raise ValueError('model response requires scalar constraint rows; elliptic cones need a coupled model')
        self.lower[unilateral] = 0.
        self.lower[friction] = -data.efc_frictionloss[:n][friction]
        self.upper[friction] = data.efc_frictionloss[:n][friction]

    def linearize(self, torque):
        """Solve the convex constraint law, then retain its exact affine mode."""
        torque = np.asarray(torque, dtype=float)
        rhs = self.rhs_offset + self.rhs_matrix @ torque
        n = len(rhs)
        fixed = {}
        # Primal/dual active-set pivots on the strictly convex soft-constraint
        # dual. Equality forces stay free; limits and friction keep their bounds.
        for _ in range(4*n+1):
            free = np.array([i for i in range(n) if i not in fixed], dtype=int)
            force = np.zeros(n)
            for i, value in fixed.items(): force[i] = value
            if free.size:
                force[free] = np.linalg.solve(self.matrix[np.ix_(free,free)],
                                             rhs[free]-self.matrix[free]@force)
            violations = np.maximum(self.lower-force, force-self.upper)
            if free.size and np.max(violations[free]) > 1e-9:
                i = int(free[np.argmax(violations[free])])
                fixed[i] = self.lower[i] if force[i] < self.lower[i] else self.upper[i]
                continue
            gradient = self.matrix @ force-rhs
            release = [i for i, value in fixed.items()
                       if (value == self.lower[i] and gradient[i] < -1e-9)
                       or (value == self.upper[i] and gradient[i] > 1e-9)]
            if release:
                del fixed[max(release, key=lambda i:abs(gradient[i]))]
                continue
            break
        else:
            raise ArithmeticError('constraint model active set did not converge')
        return self.response_for_mode(tuple(sorted(fixed.items())))

    def response_for_mode(self, mode):
        fixed=dict(mode)
        n=len(self.rhs_offset)
        nt=self.rhs_matrix.shape[1]
        free=np.array([i for i in range(n) if i not in fixed],dtype=int)
        offset = np.zeros(n)
        derivative = np.zeros((n, nt))
        for i, value in fixed.items(): offset[i] = value
        if free.size:
            solved = np.linalg.solve(self.matrix[np.ix_(free,free)], np.column_stack((
                self.rhs_offset[free]-self.matrix[free]@offset, self.rhs_matrix[free])))
            offset[free], derivative[free] = solved[:,0], solved[:,1:]
        rows, bounds = [], []
        for i in free:
            if np.isfinite(self.lower[i]):
                rows.append(-derivative[i]); bounds.append(offset[i]-self.lower[i])
            if np.isfinite(self.upper[i]):
                rows.append(derivative[i]); bounds.append(self.upper[i]-offset[i])
        gradient_offset = self.matrix@offset-self.rhs_offset
        gradient_matrix = self.matrix@derivative-self.rhs_matrix
        for i,value in fixed.items():
            sign = -1. if value == self.lower[i] else 1.
            rows.append(sign*gradient_matrix[i]); bounds.append(-sign*gradient_offset[i])
        return ConstraintResponse(
            self.smooth_offset+self.inverse_jacobian@offset,
            self.smooth_matrix+self.inverse_jacobian@derivative,
            offset, derivative, np.asarray(rows).reshape(-1,nt),
            np.asarray(bounds), tuple(sorted(fixed.items())))

    def neighboring_modes(self, mode):
        fixed=dict(mode)
        for i in range(len(self.lower)):
            choices=[None]
            choices.extend(value for value in (self.lower[i],self.upper[i]) if np.isfinite(value))
            for value in choices:
                if value==fixed.get(i):continue
                candidate=dict(fixed)
                if value is None:candidate.pop(i,None)
                else:candidate[i]=value
                yield tuple(sorted(candidate.items()))


def search_constraint_modes(dynamics, wish, solve, *, torque_of=lambda result:result.solution,
                            max_candidates=256):
    """Cross mode boundaries when the wish's local mechanical mode is infeasible.

    A returned feasible point satisfies that mode's full domain. Exhausting
    the bounded search is a controller failure, not a proof of mechanical
    impossibility; the caller reports the count and completion flag explicitly.
    """
    from collections import deque
    first=dynamics.linearize(wish)
    pending=deque([first.mode]);seen=set();best=None
    while pending and len(seen)<max_candidates:
        mode=pending.popleft()
        if mode in seen:continue
        seen.add(mode)
        response=dynamics.response_for_mode(mode)
        result,payload=solve(response)
        candidate=(result,payload,response)
        if best is None or result.violation<best[0].violation:best=candidate
        if result.feasible:return *candidate,len(seen),True
        # The failed constrained iterate often already points at the missing
        # adjacent region; still retain neighboring regions for complete search.
        actual=dynamics.linearize(torque_of(result)).mode
        if actual not in seen:pending.appendleft(actual)
        pending.extend(m for m in dynamics.neighboring_modes(mode) if m not in seen)
    return *best,len(seen),not pending
