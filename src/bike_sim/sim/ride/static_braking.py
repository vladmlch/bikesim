"""Static friction constraints and their actual, isolated generalized forces."""
import contextlib

import numpy as np
from bike_sim.physics.checks import scalar


class StaticBrakeApplier:
    """Physical-mode brakes: a solved static-friction bound plus a latched
    elastic bite at standstill.

    ``apply`` writes ``dof_frictionloss`` bounds the constraint solver holds
    the wheel against. That bound is regularized, so under a sustained load
    just below the ceiling the wheel still creeps (docs/RIDE.md section 6:
    the standing-hold residual plateaus instead of settling). Real brakes do
    not creep at a standstill: once the pads bite, the wheel angle is pinned
    elastically. ``hold_qfrc`` models that bite -- while the wheel turns
    slower than ``hold_engage_radps`` and the lever stays pulled, the brake
    latches the wheel angle and applies a spring-damper about the captured
    angle, clipped to ``hold_torque_nm * demand``. The clip keeps stored
    energy bounded; releasing the lever or spinning past
    ``hold_release_radps`` (hysteresis, so a rolling wheel sees only the
    friction bound) forgets the captured angle and the bite dissipates.
    A zero hold torque or stiffness disables the bite entirely.
    """
    def __init__(self,front_dofadr,rear_dofadr,ceiling_nm=200.,*,
                 hold_torque_nm=0.,hold_stiffness_nm_rad=0.,hold_damping_nms_rad=0.,
                 hold_engage_radps=.5,hold_release_radps=1.):
        self.front,self.rear=int(front_dofadr),int(rear_dofadr)
        if min(self.front,self.rear)<0 or self.front==self.rear:
            raise ValueError('brakes require two distinct valid wheel DOF addresses')
        self.ceiling=scalar(ceiling_nm,'brake torque ceiling',minimum=0)
        self.hold_torque_nm=scalar(hold_torque_nm,'brake hold torque',minimum=0.)
        self.hold_stiffness_nm_rad=scalar(hold_stiffness_nm_rad,'brake hold stiffness',minimum=0.)
        self.hold_damping_nms_rad=scalar(hold_damping_nms_rad,'brake hold damping',minimum=0.)
        self.hold_engage_radps=scalar(hold_engage_radps,'brake hold engage speed',positive=True)
        self.hold_release_radps=scalar(hold_release_radps,'brake hold release speed',minimum=0.)
        if self.hold_release_radps<self.hold_engage_radps:
            raise ValueError('brake hold release must not precede engage')
        self._latch={'front':None,'rear':None}

    def apply(self,model,data,front_demand,rear_demand):
        front=scalar(front_demand,'front brake demand')
        rear=scalar(rear_demand,'rear brake demand')
        if max(self.front,self.rear)>=model.nv:
            raise ValueError('brake DOF is outside this model')
        # Validate both before writing either. These are bounds on solved static
        # friction, not a predetermined signed torque or a velocity reset.
        model.dof_frictionloss[self.front]=self.ceiling*float(np.clip(front,0.,1.))
        model.dof_frictionloss[self.rear]=self.ceiling*float(np.clip(rear,0.,1.))

    def hold_qfrc(self,model,data,front_demand,rear_demand,*,advance):
        """Bounded spring-damper about the latched wheel angle, per wheel.

        ``advance`` carries the same contract as ``apply_forces``: only an
        advancing evaluation may latch or release a bite -- a probe reads the
        committed latch without moving it, so a non-advancing pass reports
        exactly the torque the step would apply at this state.
        """
        front=scalar(front_demand,'front brake demand')
        rear=scalar(rear_demand,'rear brake demand')
        out=np.zeros(model.nv)
        for side,dof,demand in (('front',self.front,front),('rear',self.rear,rear)):
            u=float(np.clip(demand,0.,1.))
            theta=float(data.qpos[int(model.jnt_qposadr[int(model.dof_jntid[dof])])])
            omega=float(data.qvel[dof])
            latched=self._latch[side]
            if advance:
                if latched is None:
                    if u>0. and self.hold_torque_nm>0. and self.hold_stiffness_nm_rad>0. \
                            and abs(omega)<=self.hold_engage_radps:
                        latched=theta
                        self._latch[side]=latched
                elif u<=0. or abs(omega)>self.hold_release_radps:
                    latched=None
                    self._latch[side]=None
            if latched is None:
                continue
            cap=self.hold_torque_nm*u
            out[dof]=float(np.clip(
                -self.hold_stiffness_nm_rad*(theta-latched)-self.hold_damping_nms_rad*omega,
                -cap,cap))
        return out

    def reset(self):
        """Forgets both captured bite angles; the next standstill re-bites."""
        self._latch={'front':None,'rear':None}

    @contextlib.contextmanager
    def hold_suspended(self):
        """Equilibrium is not a standstill the latch should remember.

        A pose solve damps the wheels through the regularized friction bound;
        the bite's engage/release hysteresis ratchets them downhill instead --
        the captured angle drifts every cycle and the residual never lands.
        Suppress the bite for the duration of the solve and leave a clean
        latch: the episode re-bites at the settled angle on the first
        standstill either way.
        """
        torque,stiffness=self.hold_torque_nm,self.hold_stiffness_nm_rad
        self.hold_torque_nm=self.hold_stiffness_nm_rad=0.
        try:
            yield self
        finally:
            self.hold_torque_nm,self.hold_stiffness_nm_rad=torque,stiffness
            self.reset()

    def solved_components(self,model,data):
        import mujoco
        result={}
        for name,dof in (('front_static_brake',self.front),('rear_static_brake',self.rear)):
            qfrc=np.zeros(model.nv)
            nefc=int(data.nefc)
            multipliers=np.zeros(nefc)
            selected=((data.efc_type[:nefc]==mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF)
                      & (data.efc_id[:nefc]==dof))
            multipliers[selected]=data.efc_force[:nefc][selected]
            if np.any(selected):
                # Engine helper supports either dense or sparse EFC Jacobians.
                mujoco.mj_mulJacTVec(model,data,qfrc,multipliers)
            result[name]=qfrc
        return result
