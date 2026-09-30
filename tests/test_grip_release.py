from dataclasses import replace
import mujoco
import numpy as np
import pytest
from bike_sim.physics.grip_release import release_if_overloaded
from bike_sim.sim.ride.rider_contacts import RiderContactApplier
from tests.test_rider_posture_tracking import rig


def test_overload_breaks_and_accounts_old_energy_only():
    force,loss,released=release_if_overloaded(np.array([500.,0.,0.]),3.,400.)
    np.testing.assert_array_equal(force,np.zeros(3))
    assert loss==3. and released
    assert not release_if_overloaded([400.,0.,0.],3.,400.)[2]


@pytest.mark.parametrize('force,energy,limit',[([1.,2.],0.,1.),([float('nan'),0.,0.],0.,1.),([0.,0.,0.],-1.,1.),([0.,0.,0.],0.,0.)])
def test_invalid_release_arguments(force,energy,limit):
    with pytest.raises(ValueError):release_if_overloaded(force,energy,limit)


def test_probe_does_not_release_live_grip_and_release_latches(rig):
    m,d,c=rig;cfg=replace(c.config,grip_pair_force_limit_n=400.)
    a=RiderContactApplier(m,c.pose,cfg);a.reset(m,d)
    a.grip_xi_local[:]=[.2,0.,0.]
    old=.5*cfg.grip_k_n_m*.2**2
    q,v=d.qpos.copy(),d.qvel.copy()
    a.compute_qfrc(m,d,.0005,advance=False)
    assert a.enabled['grip'] and a.grip_xi_local[0]==.2
    a.compute_qfrc(m,d,.0005)
    assert not a.enabled['grip']
    assert a.diagnostics['grip']['release_loss_j']==pytest.approx(old)
    assert a.diagnostics['grip']['elastic_energy_j']==0.
    np.testing.assert_array_equal(a.diagnostics['grip']['force_on_rider_n'],[0.,0.,0.])
    np.testing.assert_array_equal(d.qpos,q);np.testing.assert_array_equal(d.qvel,v)
    d.time+=.0005;a.compute_qfrc(m,d,.0005)
    assert not a.enabled['grip'] and a.diagnostics['grip']['release_loss_j']==0.


def test_regrasp_requires_explicit_request_distance_speed_and_zero_energy(rig):
    m,d,c=rig;a=RiderContactApplier(m,c.pose,c.config);a.reset(m,d)
    a.set_enabled('grip',False)
    dof=m.joint('rider_root_x').dofadr[0]
    d.qvel[dof]=1.;mujoco.mj_forward(m,d)
    assert not a.set_enabled('grip',True)
    d.qvel[dof]=0.
    qa=m.joint('rider_root_x').qposadr[0];d.qpos[qa]+=.05
    assert not a.set_enabled('grip',True)
    d.qpos[qa]-=.05
    assert a.set_enabled('grip',True)
    assert np.linalg.norm(a.grip_xi_local)==0.
    force=a.compute_qfrc(m,d,.0005)
    g=a.diagnostics['grip']
    np.testing.assert_allclose(np.array(g['force_on_rider_n'])+g['force_on_bike_n'],0.)
