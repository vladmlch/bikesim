"""Actual articulated FK, bar following, and absence of running pose writes."""
import mujoco
import numpy as np
import pytest
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.rider_control import ArticulatedRiderController, RiderCommand


@pytest.fixture
def rig():
    cfg=SimulationPhysicsConfig('physical',drive_mode='articulated_effort')
    specs=BikeSpecs();rider=RiderSpecs(variant='articulated_planar')
    pose=geometry_pose(rider,specs)
    model=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,physics_config=cfg))
    data=mujoco.MjData(model)
    controller=ArticulatedRiderController(model,pose,cfg.articulated,specs.crank_length/1000.)
    controller.initialize(model,data)
    return model,data,controller


def test_initial_bent_arm_configuration_reaches_handlebar(rig):
    m,d,c=rig
    grip=d.xpos[c.frame]+d.xmat[c.frame].reshape(3,3)@c.pose.grip
    np.testing.assert_allclose(d.site_xpos[c.grip_site],grip,atol=1e-12)
    reach=np.linalg.norm(c.pose.elbow-c.pose.shoulder)+np.linalg.norm(c.pose.grip-c.pose.elbow)
    assert np.linalg.norm(grip-d.xpos[c.upper_arm])<.95*reach


@pytest.mark.parametrize('translation',[(.01,0.),(-.01,.005),(.005,-.005)])
def test_arm_targets_follow_actual_bar_after_pelvis_motion(rig,translation):
    m,d,c=rig
    d.qpos[m.joint('rider_root_x').qposadr[0]]+=translation[0]
    d.qpos[m.joint('rider_root_z').qposadr[0]]+=translation[1]
    mujoco.mj_forward(m,d)
    # Evaluate the torso target first, then the arms at that actual shoulder.
    d.qpos[c.joints['rider_torso_hinge'][0]]=c._upper_targets(d)['rider_torso_hinge']
    mujoco.mj_forward(m,d)
    for name,value in c._upper_targets(d).items():d.qpos[c.joints[name][0]]=value
    mujoco.mj_forward(m,d)
    assert not c.saturated_ik['arms']
    grip=d.xpos[c.frame]+d.xmat[c.frame].reshape(3,3)@c.pose.grip
    np.testing.assert_allclose(d.site_xpos[c.grip_site],grip,atol=1e-11)


def test_running_controller_only_returns_bounded_internal_effort(rig):
    m,d,c=rig
    d.qvel[:]=np.random.default_rng(19).normal(0.,.1,m.nv)
    mujoco.mj_forward(m,d)
    q,v=d.qpos.copy(),d.qvel.copy()
    efforts=c.compute(m,d,RiderCommand(20.),contact_loads={'front':100.,'rear':100.,'grip':True})
    np.testing.assert_array_equal(d.qpos,q)
    np.testing.assert_array_equal(d.qvel,v)
    assert not any('root' in name for name in efforts)
    assert all(abs(value)<=c.config.joint_limit_nm for value in efforts.values())


def test_posture_request_uses_contact_moments_not_fictitious_ik_frame(rig):
    m,d,c=rig
    d.qpos[m.joint('rider_root_pitch').qposadr[0]]+=.1
    mujoco.mj_forward(m,d)
    qa=[c.joints[f'rider_{joint}_front'][0] for joint in ('hip','knee','ankle')]
    d.qpos[qa]=c._targets(m,d,'front',compression_m=.003)
    mujoco.mj_forward(m,d)
    target=d.site_xpos[c.pedals['front']]+np.array([0.,0.,.008-.003])
    np.testing.assert_allclose(d.site_xpos[c.soles['front']],target,atol=1e-11)
    efforts=c.compute(m,d,RiderCommand(20.),contact_loads={'front':100.,'rear':100.,'grip':True},
        support_available={'front':True,'rear':True,'saddle':True,'grip':True})
    assert c.support_diagnostics['requested_pitch_moment_nm']<0.
    assert not any('root' in name for name in efforts)


def test_tangential_force_goal_displaces_sole_without_a_hidden_force(rig):
    m,d,c=rig
    qa=[c.joints[f'rider_{j}_front'][0] for j in ('hip','knee','ankle')]
    d.qpos[qa]=c._targets(m,d,'front',shear_m=0.)
    mujoco.mj_forward(m,d)
    zero=d.site_xpos[c.soles['front']].copy()
    d.qpos[qa]=c._targets(m,d,'front',shear_m=-.001)
    mujoco.mj_forward(m,d)
    np.testing.assert_allclose(d.site_xpos[c.soles['front']]-zero,[-.001,0.,0.],atol=1e-11)


def test_target_velocity_predictor_only_uses_detached_kinematics(rig):
    m,d,c=rig
    d.qvel[m.joint('crank_spin').dofadr[0]]=2.
    mujoco.mj_forward(m,d)
    q,v=d.qpos.copy(),d.qvel.copy()
    future=c._predict_target_state(m,d)
    np.testing.assert_array_equal(d.qpos,q)
    np.testing.assert_array_equal(d.qvel,v)
    assert future is not d
    assert np.linalg.norm(future.site_xpos[c.pedals['front']]-d.site_xpos[c.pedals['front']])>0.


def test_target_speed_does_not_follow_uncommanded_pelvis_motion(rig):
    m,d,c=rig
    d.qvel[m.joint('rider_root_z').dofadr[0]]=1.
    d.qvel[m.joint('rider_root_pitch').dofadr[0]]=1.
    mujoco.mj_forward(m,d)
    future=c._predict_target_state(m,d)
    np.testing.assert_array_equal(future.qpos,d.qpos)


def test_stance_transition_is_continuous_at_zero_normal_load(rig):
    m,d,c=rig
    d.qpos[m.joint('rider_root_pitch').qposadr[0]]+=.15
    mujoco.mj_forward(m,d)
    available={'saddle':True,'front':True,'rear':True,'grip':True}
    low=c.compute(m,d,RiderCommand(20.),contact_loads={'front':0.,'rear':100.,'grip':True},support_available=available)
    high=c.compute(m,d,RiderCommand(20.),contact_loads={'front':1e-8,'rear':100.,'grip':True},support_available=available)
    np.testing.assert_allclose(list(low.values()),list(high.values()),atol=1e-5,rtol=0.)


def test_disabled_controller_has_no_hidden_velocity_bias(rig):
    m,d,c=rig
    zero=c.compute(m,d,RiderCommand(enabled=False))
    c.write(d,zero)
    for _,dof,_ in c.joints.values():d.qvel[dof]=.1
    mujoco.mj_forward(m,d)
    np.testing.assert_array_equal(d.actuator_force[[a for _,_,a in c.joints.values()]],0.)
    c.write(d,c.compute(m,d,RiderCommand()))
    assert all(m.actuator_biasprm[a,2]==-c.config.joint_kd_nms_rad for _,_,a in c.joints.values())


def test_return_stroke_is_not_assigned_a_coasting_support_load(rig):
    m,d,c=rig
    c.compute(m,d,RiderCommand(20.),contact_loads={'saddle':400.,'front':140.,'rear':140.,'grip':True})
    assert c.support_diagnostics['stance']=={'front':True,'rear':False}
    assert c.support_diagnostics['requested_vertical_forces_n']['rear']==pytest.approx(0.,abs=1e-8)


def test_swing_target_lifts_sole_above_actual_platform(rig):
    m,d,c=rig
    target=c._targets(m,d,'rear',compression_m=0.,clearance_m=.003)
    for joint,value in zip(('hip','knee','ankle'),target):
        d.qpos[c.joints[f'rider_{joint}_rear'][0]]=value
    mujoco.mj_forward(m,d)
    sole=d.site_xpos[c.soles['rear']]
    geom=c.pedal_geoms['rear']
    assert sole[2]-(d.geom_xpos[geom,2]+m.geom_size[geom,2])==pytest.approx(.003,abs=1e-10)


@pytest.mark.parametrize('phase', np.linspace(-np.pi,np.pi,65))
def test_pedal_request_stays_inside_the_flat_pedal_friction_cone(phase):
    from bike_sim.sim.ride.rider_control import stance_force, feasible_pedal_force
    raw=stance_force(phase,20.,.165)
    force=feasible_pedal_force(raw,np.array([0.,0.,1.]),.8,200.)
    assert force[2]<=0.
    assert abs(force[0])<=.8*(-force[2])+1e-12
    assert abs(force[0])<=.8*200.+1e-12


def test_return_foot_never_requests_a_tensile_normal_force():
    from bike_sim.sim.ride.rider_control import feasible_pedal_force
    np.testing.assert_array_equal(feasible_pedal_force([30.,0.,20.],[0.,0.,1.],.8,200.),0.)
