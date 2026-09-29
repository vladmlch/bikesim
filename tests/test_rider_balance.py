"""Internal balance intent, physical support feasibility and frame invariance."""
import mujoco
import numpy as np
import pytest
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.rider_balance import balance_force_request
from bike_sim.sim.ride.rider_control import ArticulatedRiderController
from bike_sim.sim.ride.rider_support import pedaling_support_targets


@pytest.fixture
def rig():
    cfg=SimulationPhysicsConfig('physical',drive_mode='articulated_effort')
    specs=BikeSpecs(); rider=RiderSpecs(variant='articulated_planar')
    m=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,physics_config=cfg))
    d=mujoco.MjData(m)
    c=ArticulatedRiderController(m,geometry_pose(rider,specs),cfg.articulated,specs.crank_length/1000.)
    c.initialize(m,d)
    return m,d,c


def test_displaced_pelvis_requests_restoring_support_without_applying_force(rig):
    m,d,c=rig
    d.qpos[m.joint('rider_root_x').qposadr[0]]+=.05
    d.qpos[m.joint('rider_root_z').qposadr[0]]+=.02
    mujoco.mj_forward(m,d)
    q,v,forces=d.qpos.copy(),d.qvel.copy(),d.qfrc_applied.copy()
    force=balance_force_request(c,m,d,RiderPosture())
    np.testing.assert_allclose(force,[-75.,0.,-30.],atol=1e-8)
    np.testing.assert_array_equal(d.qpos,q)
    np.testing.assert_array_equal(d.qvel,v)
    np.testing.assert_array_equal(d.qfrc_applied,forces)


def test_balance_request_is_invariant_to_common_translation_and_velocity(rig):
    m,d,c=rig
    pose=RiderPosture(pelvis_offset_m=(.03,.04))
    first=balance_force_request(c,m,d,pose)
    for root in ('root','rider_root'):
        d.qpos[m.joint(root+'_x').qposadr[0]]+=100.
        d.qpos[m.joint(root+'_z').qposadr[0]]+=2.
        d.qvel[m.joint(root+'_x').dofadr[0]]=3.
        d.qvel[m.joint(root+'_z').dofadr[0]]=.2
    mujoco.mj_forward(m,d)
    np.testing.assert_allclose(balance_force_request(c,m,d,pose),first,atol=1e-8)


def test_balance_force_request_has_finite_norm_limit(rig):
    m,d,c=rig
    d.qpos[m.joint('rider_root_x').qposadr[0]]+=.8
    d.qvel[m.joint('rider_root_z').dofadr[0]]=10.
    mujoco.mj_forward(m,d)
    f=balance_force_request(c,m,d,RiderPosture())
    assert np.linalg.norm(f)==pytest.approx(c.config.posture_translation_limit_n)
    assert f[0]<0 and f[2]<0 and f[1]==0


@pytest.mark.parametrize('grip',[True,False])
def test_balance_support_wrench_is_realized_or_reported_infeasible(grip):
    points=np.array([[-.2,0.,.7],[.165,0.,0.],[-.165,0.,0.],[.5,0.,.8]])
    feet={'front':np.zeros(3),'rear':np.zeros(3)}
    forces,info=pedaling_support_targets(800.,[.1,0.,.8],points,0.,.33,.12,
        [True,True,True,grip],feet,balance_force_on_rider_n=(100.,0.,100.))
    reactions=-np.array(list(forces.values()))
    if grip:
        np.testing.assert_allclose(reactions.sum(axis=0),[100.,0.,900.],atol=1e-9)
        assert info['feasible']
    else:
        assert reactions.sum(axis=0)[0]==0.
        assert not info['feasible']
    assert info['requested_balance_force_on_rider_n']==[100.,0.,100.]


def test_no_supports_cannot_realize_a_balance_request():
    points=np.zeros((4,3)); feet={'front':np.zeros(3),'rear':np.zeros(3)}
    forces,info=pedaling_support_targets(800.,np.zeros(3),points,0.,.33,.12,
        [False]*4,feet,balance_force_on_rider_n=(100.,0.,100.))
    assert not info['feasible']
    np.testing.assert_array_equal(list(forces.values()),np.zeros((4,3)))
