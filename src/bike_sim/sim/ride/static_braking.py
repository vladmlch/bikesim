"""Static friction constraints and their actual, isolated generalized forces."""
import numpy as np
from bike_sim.physics.checks import scalar


class StaticBrakeApplier:
    def __init__(self,front_dofadr,rear_dofadr,ceiling_nm=200.):
        self.front,self.rear=int(front_dofadr),int(rear_dofadr)
        if min(self.front,self.rear)<0 or self.front==self.rear:
            raise ValueError('brakes require two distinct valid wheel DOF addresses')
        self.ceiling=scalar(ceiling_nm,'brake torque ceiling',minimum=0)

    def apply(self,model,data,front_demand,rear_demand):
        front=scalar(front_demand,'front brake demand')
        rear=scalar(rear_demand,'rear brake demand')
        if max(self.front,self.rear)>=model.nv:
            raise ValueError('brake DOF is outside this model')
        # Validate both before writing either. These are bounds on solved static
        # friction, not a predetermined signed torque or a velocity reset.
        model.dof_frictionloss[self.front]=self.ceiling*float(np.clip(front,0.,1.))
        model.dof_frictionloss[self.rear]=self.ceiling*float(np.clip(rear,0.,1.))

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
