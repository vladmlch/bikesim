"""Deferred duplicate validation keeps native inputs and published gates unchanged."""
from dataclasses import replace
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride.attachment_wrench import (
    attachment_raw_from_geometry, prepare_attachment_geometry)
from bike_sim.sim.ride.period_buffer import RawStep, evaluate_period
from test_pinned_topology import _compiled


def test_bulk_road_queries_match_two_hundred_scalar_windows():
    from bike_sim.sim.ride.rider_state import road_samples,_profile_height,_profile_grade
    vertices=np.array([[0.,0.],[.7,.08],[1.6,-.02],[2.8,.24],[5.,.3]])
    for i in range(200):
        samples=road_samples(vertices,(.1+i*.01,1.2+i*.01),.371)
        assert samples[-1].x_m==1.2+i*.01+.371
        for sample in samples:
            assert sample.height_m==_profile_height(vertices,sample.x_m)
            assert sample.grade==_profile_grade(vertices,sample.x_m)


def test_deferred_unexplained_wrench_is_rejected_at_its_original_interval(tmp_path):
    model,controller=_compiled(tmp_path)
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    eq=model.equality('connect_foot_front').id
    data.efc_force[:]=0.
    rows=np.flatnonzero((data.efc_type[:data.nefc]==mujoco.mjtConstraint.mjCNSTR_EQUALITY)&(data.efc_id[:data.nefc]==eq))
    data.efc_force[rows]=[0.,0.,100.]
    rider,bike=int(model.eq_obj1id[eq]),int(model.eq_obj2id[eq])
    point=data.xpos[rider]+data.xmat[rider].reshape(3,3)@model.eq_data[eq,:3]
    geometry=prepare_attachment_geometry(model,data,eq,rider,bike,point,np.array([0.,0.,1.]),'foot',rotational=False)
    bad=replace(geometry,bike_jac=np.zeros_like(geometry.bike_jac))
    with pytest.raises(ValueError,match='explain'):
        attachment_raw_from_geometry(model,data,bad)
    captured=attachment_raw_from_geometry(model,data,bad,validate_wrench=False)
    raw=RawStep(0,0.,.00125,data.qpos.copy(),data.qvel.copy(),{},
        {'foot_front':captured},np.zeros(model.nu),np.zeros(model.nv),{},
        {'attachment_errors':(),'numerical_constraint_power_w':{}})
    runtime=SimpleNamespace(rider_control=None,control_clock=SimpleNamespace(timestep_s=.00125))
    report=evaluate_period(runtime,[raw],controller.config.attachment_budget())
    assert report.first_failure==(.00125,('foot_front:unobservable_attachment_wrench',))


def _equal(left,right):
    if isinstance(left,dict):
        assert left.keys()==right.keys()
        for key in left:
            _equal(left[key],right[key])
    elif isinstance(left,(tuple,list)):
        assert len(left)==len(right)
        for a,b in zip(left,right):
            _equal(a,b)
    elif isinstance(left,(float,int)) and not isinstance(left,bool):
        assert np.isclose(left,right,rtol=1e-12,atol=1e-12)
    else:
        assert left==right


@pytest.mark.slow
def test_two_hundred_steps_match_scalar_validation_oracle(tmp_path,monkeypatch):
    from test_pinned_topology import _model
    from bike_sim.sim.ride import rider_contacts
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.initial_state import PhysicalInitialState
    _,env=_model(tmp_path)
    sim=env.sim;runtime=sim.physical
    sim.physical_initial_state=PhysicalInitialState.capture(sim)
    converter=rider_contacts.attachment_raw_from_geometry
    def scalar_converter(*args,**kwargs):
        kwargs['validate_wrench']=True
        return converter(*args,**kwargs)
    def collect():
        sim.reset();runtime.set_record_decimation(80)
        rows=[]
        for _ in range(200):
            runtime.step(control=RideControl(human_torque_nm=35.,motor_torque_nm=0.))
            rows.extend(sample.as_dict() for sample in runtime.completed_samples)
        rows.extend(sample.as_dict() for sample in runtime.flush())
        assert len(rows)==200
        return rows,runtime.reference_monitor.first_failure
    with monkeypatch.context() as patch:
        patch.setattr(rider_contacts,'attachment_raw_from_geometry',scalar_converter)
        baseline,first=collect()
    candidate,second=collect()
    _equal(baseline,candidate)
    assert first==second
