"""Zero-velocity refinement of a relaxed physical initial condition.

Optimizes actual compiled accelerations and initial shear strains. It does not
change parameters, relax acceptance tolerances, or run during a ride.
"""
import numpy as np
import mujoco
from scipy.optimize import least_squares


def refine_equilibrium(runtime, *, max_evaluations=80, acceleration_tolerance=.05):
    sim=runtime.sim; m,d=sim.model,sim.data
    if m.nq != m.nv:
        raise ValueError("physical equilibrium requires scalar planar coordinates")
    d.qvel.fill(0.)
    auxiliary=[]
    if runtime.tire is not None:
        for state in runtime.tire.states.values():
            auxiliary.append((state,'xi',None))
    if runtime.rider_contacts is not None:
        for state in runtime.rider_contacts.states.values():
            auxiliary.append((state,'xi',None))
        for axis in (0,2):
            auxiliary.append((runtime.rider_contacts,'grip_xi_local',axis))
    def get(obj,key,index):
        value=getattr(obj,key)
        return float(value if index is None else value[index])
    q0=d.qpos.copy()
    qids=np.array([i for i in range(m.nq) if i!=sim.root_x_qposadr])
    x0=np.r_[q0[qids],[get(*item) for item in auxiliary]]
    scale=np.r_[[.05 if m.jnt_type[i]==mujoco.mjtJoint.mjJNT_SLIDE else .2 for i in qids],np.full(len(auxiliary),.005)]
    mass_matrix=np.empty((m.nv,m.nv))
    mujoco.mj_fullM(m,d,mass_matrix)
    force_scale=np.sqrt(np.maximum(np.diag(mass_matrix),1e-12))
    best=[(1,float('inf')),x0.copy()]
    def place(x):
        d.qpos[:]=q0
        d.qpos[qids]=x[:len(qids)]
        d.qvel.fill(0.)
        for value,(obj,key,index) in zip(x[len(qids):],auxiliary):
            if index is None: setattr(obj,key,float(value))
            else: getattr(obj,key)[index]=value
    def residual(y):
        x=x0+scale*y
        place(x)
        runtime.apply_forces(active=False,advance=False, front=sim.physics_config.initial_front_brake, rear=sim.physics_config.initial_rear_brake)
        mujoco.mj_forward(m,d)
        # Raw acceleration weights a light pedal about a million times more
        # strongly than a loaded chassis. Solve the equivalent generalized
        # force balance instead; M is positive definite, so the zero is
        # unchanged. Acceptance still uses qacc.
        result=np.empty(m.nv)
        mujoco.mj_mulM(m,d,result,d.qacc)
        result/=force_scale
        objective=np.r_[result,1e-5*y]
        acceleration=float(np.max(np.abs(d.qacc)))
        norm=float(objective@objective)
        rank=(int(acceleration>acceleration_tolerance),norm)
        if np.isfinite(norm) and np.isfinite(acceleration) and rank<best[0]:
            best[:]=[rank,x.copy()]
        # Weak datum removes arbitrary whole-system translation without hiding
        # force residuals (acceptance below uses qacc alone).
        return objective
    def stop_when_converged(intermediate_result):
        if best[0][0]==0:
            raise StopIteration
    try:
        least_squares(residual,np.zeros_like(x0),jac='3-point',x_scale='jac',max_nfev=max_evaluations,
                      callback=stop_when_converged,
                      xtol=1e-10,ftol=1e-10,gtol=1e-10, bounds=(-np.ones_like(x0),np.ones_like(x0)))
    finally:
        place(best[1])
        runtime.apply_forces(active=False,advance=False, front=sim.physics_config.initial_front_brake, rear=sim.physics_config.initial_rear_brake)
        mujoco.mj_forward(m,d)
    return float(np.max(np.abs(d.qacc)))
