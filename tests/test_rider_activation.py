from dataclasses import replace
import mujoco
import numpy as np
import pytest
from bike_sim.physics.rider_activation import activation_step,limit_positive_power
from bike_sim.sim.ride.rider_control import RiderCommand
from bike_sim.sim.ride.rider_effort import solved_effort
from tests.test_rider_posture_tracking import rig


def test_shared_budget_does_not_cancel_braking():
    result=limit_positive_power([20.,20.,-5.],[10.,10.,10.],200.)
    assert np.maximum(result*10.,0.).sum()<=200.+1e-10
    assert result[2]==-5.


def test_filter_semigroup_and_zero_tau():
    zero,target=np.zeros(2),np.ones(2)
    half=activation_step(zero,target,.005,.05)
    np.testing.assert_allclose(activation_step(zero,target,.01,.05),activation_step(half,target,.005,.05))
    np.testing.assert_array_equal(activation_step(zero,target,.01,0.),target)


@pytest.mark.parametrize('dt,tau',[(0.,.1),(-1.,.1),(.1,-.1),(.1,float('nan'))])
def test_invalid_activation_time(dt,tau):
    with pytest.raises(ValueError):activation_step([0.],[1.],dt,tau)


def test_probe_no_advance_reset_and_solved_force_budget(rig):
    m,d,c=rig;c.config=replace(c.config,activation_tau_s=.05,active_positive_power_limit_w=100.)
    c.reset_activation()
    for _,dof,_ in c.joints.values():d.qvel[dof]=.1
    mujoco.mj_forward(m,d)
    before=c.active_state.copy();q,v=d.qpos.copy(),d.qvel.copy()
    command=RiderCommand(20.)
    c.compute(m,d,command,advance=False)
    np.testing.assert_array_equal(c.active_state,before)
    torques=c.compute(m,d,command)
    with pytest.raises(ValueError):c.compute(m,d,command)
    assert c.effort_diagnostics['rider_positive_power_w']<=100.+1e-9
    c.write(d,torques)
    mujoco.mj_step(m,d)
    info=solved_effort(c,d,v,float(m.opt.timestep))
    actual=np.array([d.actuator_force[aid] for _,_,aid in c.joints.values()])
    assert np.max(np.abs(actual))<=c.config.joint_limit_nm+1e-9
    assert info['rider_effort_observation'].startswith('solved')
    assert info['rider_passive_power_w']<=1e-9
    assert not any(n.startswith('rider_root_') for n in info['rider_active_delivered_nm'])
    c.reset_activation();assert np.linalg.norm(c.active_state)==0. and c.activation_time_s is None


def test_disabled_probe_does_not_clear_live_activation(rig):
    m,d,c=rig;c.active_state[:]=3.
    c.compute(m,d,RiderCommand(enabled=False),advance=False)
    np.testing.assert_array_equal(c.active_state,np.full(9,3.))
