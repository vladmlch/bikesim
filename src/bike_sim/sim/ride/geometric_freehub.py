"""Locally linearized one-way geometric tendon, with explicit defect metering.

The engine solves the ONLY reaction. ``constraint_reaction`` is an independent
sign/power oracle, never an additional applied force. Model constants are
updated using detached data, so a coefficient update cannot reset live state.
"""
from dataclasses import replace
import numpy as np
import mujoco
from bike_sim.physics.checks import scalar
from bike_sim.physics.transmission_constraint import transmission_geometry,linearized_upper_bound,constraint_reaction
from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint
from bike_sim.sim.ride.physical_mapping import resolve_id
from bike_sim.sim.ride.wheels import resolve_hinge


class GeometricFreehubConstraint(IdealFreehubConstraint):
    def __init__(self,model,gearing,*,driver='crank_spin',driver_body='crank'):
        gearing.__post_init__()
        self.gearing=gearing
        self.front_body=driver_body
        self.ratio=gearing.front_teeth/gearing.rear_teeth
        self.tendon_id=resolve_id(model,mujoco.mjtObj.mjOBJ_TENDON,'geometric_mid_drive_freehub')
        self.driver_qpos,self.driver_dof=resolve_hinge(model,driver)
        self.driven_qpos,self.driven_dof=resolve_hinge(model,'rear_wheel_spin')
        start=int(model.tendon_adr[self.tendon_id]);end=start+int(model.tendon_num[self.tendon_id])
        self.coefficients={}
        for index in range(start,end):
            if model.wrap_type[index]!=mujoco.mjtWrap.mjWRAP_JOINT:
                raise ValueError('geometric reduction must use one fixed scalar-joint tendon')
            joint=int(model.wrap_objid[index]);dof=int(model.jnt_dofadr[joint])
            if dof in self.coefficients:
                raise ValueError('duplicate geometric tendon coordinate')
            self.coefficients[dof]=index
        if set(self.coefficients)!=set(range(model.nv)) or model.nq!=model.nv:
            raise ValueError('geometric tendon must contain every scalar model coordinate exactly once')
        self.constant_data=mujoco.MjData(model)
        self.endpoint_data=mujoco.MjData(model)
        self.boundary=None;self.prepared=None;self.diagnostics={}
        self.shift_pending=False;self.shift_parameter_work_j=0.;self.shift_constraint_work_j=0.
        self.last_tension_n=0.

    def _geometry(self,model,data):
        mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
        return transmission_geometry(model,data,self.gearing,front_body=self.front_body)

    def _linearize(self,model,data,phi,jacobian):
        coefficients,upper=linearized_upper_bound(phi,jacobian,data.qpos,self.boundary)
        changed=False
        for dof,index in self.coefficients.items():
            if model.wrap_prm[index]!=coefficients[dof]:
                model.wrap_prm[index]=coefficients[dof];changed=True
        if changed:
            mujoco.mj_setConst(model,self.constant_data)
        model.tendon_range[self.tendon_id,1]=upper
        self.prepared=(float(phi),jacobian.copy(),data.qpos.copy(),float(data.time))

    def reset(self,model,data):
        phi,j=self._geometry(model,data)
        self.boundary=phi;self.diagnostics={};self.shift_pending=False
        self.shift_parameter_work_j=self.shift_constraint_work_j=self.last_tension_n=0.
        self._linearize(model,data,phi,j)

    def prepare(self,model,data):
        phi,j=self._geometry(model,data)
        self.boundary=phi if self.boundary is None else min(self.boundary,phi)
        self._linearize(model,data,phi,j)

    def prepare_initial_candidate(self,model,data,incoming_boundary):
        """Initialization ONLY: all optimizer trials share the same datum.

        The refiner owns permission to change q at t=0; temporal ratcheting
        across unordered least-squares trials would corrupt the initial pose.
        """
        phi,j=self._geometry(model,data)
        self.boundary=phi if incoming_boundary is None else min(float(incoming_boundary),phi)
        self._linearize(model,data,phi,j)

    def set_ratio(self,model,data,ratio):
        ratio=scalar(ratio,'geometric gear ratio',positive=True)
        if ratio==self.ratio:return
        teeth=round(self.gearing.front_teeth/ratio)
        if teeth<3 or not np.isclose(ratio,self.gearing.front_teeth/teeth,rtol=1e-12,atol=0):
            raise ValueError('geometric ratio must identify an integer rear sprocket')
        old_phi,_=self._geometry(model,data)
        old_gap=0. if self.boundary is None else self.boundary-old_phi
        self.gearing=replace(self.gearing,rear_teeth=teeth);self.ratio=ratio
        phi,j=self._geometry(model,data)
        self.boundary=phi+old_gap
        new_gap=self.boundary-phi
        # Virtual parameter work at fixed q. With a preserved physical gap
        # it is zero up to roundoff. The *whole* following solved interval's
        # work is logged separately; zero parameter work is not a claim that
        # a real shift is lossless or that solver impulses cannot change KE.
        self.shift_parameter_work_j+=self.last_tension_n*(new_gap-old_gap)
        self.shift_pending=True
        self._linearize(model,data,phi,j)

    def solved_qfrc(self,model,data):
        force=super().solved_qfrc(model,data)
        if self.prepared is None:
            raise RuntimeError('prepare the geometric transmission before solving')
        phi,j,q,time=self.prepared
        selected=((data.efc_type[:data.nefc]==mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)&
                  (data.efc_id[:data.nefc]==self.tendon_id))
        tension=float(np.sum(data.efc_force[:data.nefc][selected]))
        self.endpoint_data.qpos[:]=data.qpos
        endpoint,_=self._geometry(model,self.endpoint_data)
        displacement=data.qpos-q
        defect=endpoint-phi-float(j@displacement)
        work=float(force@displacement)
        if self.shift_pending:self.shift_constraint_work_j+=work
        self.diagnostics={'transmission_phi_m':phi,'transmission_boundary_m':self.boundary,
            'transmission_gap_m':self.boundary-phi,'transmission_tension_n':tension,
            'transmission_constraint_defect_m':defect,'transmission_interval_work_j':work,
            'transmission_reaction_error_n':float(np.max(np.abs(force-constraint_reaction(max(0.,tension),j)))),
            'shift_parameter_work_j':self.shift_parameter_work_j,
            'shift_interval_constraint_work_j':work if self.shift_pending else 0.,
            'shift_constraint_work_cumulative_j':self.shift_constraint_work_j,
            'transmission_reference_status':'experimental_geometric_reduction'}
        self.last_tension_n=tension;self.shift_pending=False
        return force
