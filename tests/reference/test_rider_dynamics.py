"""Current-state soft-constraint prediction against actual MuJoCo solves."""
import mujoco
import numpy as np
import pytest


def _model(kind):
    if kind == 'weld':
        xml='''<worldbody><body name="a"><freejoint/><inertial mass="2" pos=".1 0 0" diaginertia="1 1 1"/></body>
        <body name="b"><freejoint/><inertial mass="1" pos="0 0 .1" diaginertia=".5 .5 .5"/></body></worldbody>
        <equality><weld body1="a" body2="b" solref=".005 1"/></equality>'''
    else:
        friction='frictionloss="2"' if kind=='friction' else ''
        xml=f'''<worldbody><body><joint name="a" type="slide" axis="1 0 0" {friction}/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body>
        <body><joint name="b" type="slide" axis="1 0 0"/><inertial mass="2" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody>'''
        if kind=='tendon':xml+='<tendon><fixed limited="true" range="-10 0"><joint joint="a" coef="1"/><joint joint="b" coef="-1"/></fixed></tendon>'
    return mujoco.MjModel.from_xml_string('<mujoco><option gravity="0 0 0" timestep=".0005" tolerance="1e-12"/>'+xml+'</mujoco>')


@pytest.mark.parametrize('kind',['weld','tendon','friction'])
@pytest.mark.parametrize('torque',[-20.,0.,20.])
@pytest.mark.parametrize('impedance',[.95,.9999])
def test_model_response_predicts_current_acceleration_and_all_reactions(kind,torque,impedance):
    from bike_sim.sim.ride.rider_dynamics import ConstraintDynamics
    model=_model(kind);data=mujoco.MjData(model)
    model.eq_solimp[:,:2]=impedance
    model.tendon_solimp_lim[:,:2]=impedance
    if kind=='tendon':data.qpos[0]=1e-5
    data.qfrc_applied[-1]=.1
    mujoco.mj_forward(model,data)
    kernel=ConstraintDynamics(model,data,[0])
    response=kernel.linearize(np.array([torque]))
    predicted_a=response.acceleration_offset+response.acceleration_matrix[:,0]*torque
    predicted_f=response.force_offset+response.force_matrix[:,0]*torque
    assert np.all(response.domain_matrix@np.array([torque]) <= response.domain_bound+1e-8)
    data.qfrc_applied[0]+=torque
    mujoco.mj_forward(model,data)
    np.testing.assert_allclose(predicted_a,data.qacc,rtol=1e-10,atol=1e-9)
    np.testing.assert_allclose(predicted_f,data.efc_force[:data.nefc],rtol=1e-10,atol=1e-9)


def test_prediction_does_not_read_previous_reactions_or_acceleration():
    from bike_sim.sim.ride.rider_dynamics import ConstraintDynamics
    model=_model('weld');data=mujoco.MjData(model)
    data.qfrc_applied[0]=10.;mujoco.mj_forward(model,data)
    expected=ConstraintDynamics(model,data,[0]).linearize(np.array([5.]))
    data.efc_force[:data.nefc]=np.nan
    data.qfrc_constraint[:]=np.nan;data.qacc[:]=np.nan
    actual=ConstraintDynamics(model,data,[0]).linearize(np.array([5.]))
    np.testing.assert_array_equal(actual.acceleration_offset,expected.acceleration_offset)
    np.testing.assert_array_equal(actual.force_matrix,expected.force_matrix)


def test_unilateral_mode_domain_rejects_a_command_that_changes_contact_state():
    from bike_sim.sim.ride.rider_dynamics import ConstraintDynamics
    model=_model('tendon');data=mujoco.MjData(model);data.qpos[0]=1e-5
    mujoco.mj_forward(model,data)
    kernel=ConstraintDynamics(model,data,[0])
    engaged=kernel.linearize(np.array([20.]))
    free=kernel.linearize(np.array([-20.]))
    assert engaged.mode != free.mode
    assert np.any(engaged.domain_matrix@np.array([-20.]) > engaged.domain_bound)
    assert np.any(free.domain_matrix@np.array([20.]) > free.domain_bound)


def test_allocation_crosses_from_infeasible_sliding_wish_to_valid_sticking_mode():
    from bike_sim.sim.ride.rider_dynamics import ConstraintDynamics,search_constraint_modes
    from bike_sim.physics.rider_allocation import allocate_effort
    model=_model('friction');data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    kernel=ConstraintDynamics(model,data,[0]);wish=np.array([20.])
    def solve(response):
        # Declared support bound tau <= 1 conflicts with the wish's sliding
        # region tau >= 2; a physically valid sticking command still exists.
        g=np.vstack((response.domain_matrix,[[1.]]))
        h=np.r_[response.domain_bound,1.]
        result=allocate_effort(wish,np.zeros((0,1)),np.zeros(0),g,h,
            np.array([-50.]),np.array([50.]))
        return result,None
    assert not solve(kernel.linearize(wish))[0].feasible
    result,_,response,count,complete=search_constraint_modes(kernel,wish,solve)
    assert result.feasible and count>1
    assert result.solution[0] == pytest.approx(1.,abs=1e-7)
    data.qfrc_applied[0]=result.solution[0];mujoco.mj_forward(model,data)
    np.testing.assert_allclose(data.qacc,response.acceleration_offset+
                               response.acceleration_matrix@result.solution,atol=1e-9)
    assert abs(data.qacc[0]) < .1


def test_predicted_weld_wrench_includes_the_actual_support_moment():
    from bike_sim.sim.ride.rider_dynamics import ConstraintDynamics, attachment_force_map
    from bike_sim.sim.ride.attachment_wrench import attachment_sample
    model=_model('weld');data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    kernel=ConstraintDynamics(model,data,[4])
    response=kernel.linearize(np.array([10.]))
    point=data.xpos[model.body('a').id].copy()
    force_map=attachment_force_map(model,data,kernel,0,model.body('a').id,point)
    predicted=force_map@(response.force_offset+response.force_matrix[:,0]*10.)
    data.qfrc_applied[4]=10.;mujoco.mj_forward(model,data)
    sample=attachment_sample(model,data,0,model.body('a').id,model.body('b').id,
        point,np.array([0.,0.,1.]),'foot',rotational=True,half_patch_m=.05)
    np.testing.assert_allclose(predicted,[sample.tangent_n,sample.normal_n,sample.moment_nm],atol=1e-9)
    assert abs(predicted[2]) > 1.


def test_feasible_ten_watt_wish_is_not_suppressed_by_mixing_watts_and_scaled_torque():
    from bike_sim.physics.model_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    from bike_sim.sim.ride.rider_response_allocation import allocate_response
    model=mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
    <worldbody><body name="bike"><joint name="bx" type="slide" axis="1 0 0"/>
      <joint type="slide" axis="0 0 1"/><joint type="hinge" axis="0 1 0"/>
      <inertial mass="1000" pos="0 0 0" diaginertia="1000 1000 1000"/>
      <geom name="platform" type="box" size=".05 .025 .01" contype="0" conaffinity="0"/></body>
    <body name="rider"><joint name="rx" type="slide" axis="1 0 0"/>
      <joint type="slide" axis="0 0 1"/><joint type="hinge" axis="0 1 0"/>
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/><site name="sole"/></body></worldbody>
    <equality><weld body1="rider" body2="bike"/></equality>
    <actuator><motor joint="rx" gear="1" forcelimited="true" forcerange="-50 50"/></actuator></mujoco>''')
    data=mujoco.MjData(model);data.qfrc_applied[4]=-100.;data.qvel[[0,3]]=1.
    mujoco.mj_forward(model,data)
    c=ArticulatedRiderController.__new__(ArticulatedRiderController)
    c.config=ArticulatedConfig(active_positive_power_limit_w=450.)
    c.joints={'probe':(3,3,0)};c.strength=None;c.rider_mass=1.
    c._rider_dofs=np.array([3,4,5]);c._bike_dofs=np.array([0,1,2])
    c._alloc_attachments={'front':{'eq':0,'rider_body':model.body('rider').id}}
    c.soles={'front':model.site('sole').id};c.pedal_geoms={'front':model.geom('platform').id}
    c._last_branch=c._last_solution=None
    torque,diagnostic=allocate_response(c,model,data,np.array([10.]))
    assert diagnostic['feasible']
    assert 9. < torque[0] <= 10.
    assert diagnostic['solution_power'][0] == pytest.approx(torque[0],abs=1e-7)


@pytest.mark.parametrize('previous',[0.,35.])
def test_actual_allocation_cannot_jump_outside_bounded_activation_reach(previous):
    from bike_sim.physics.model_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    from bike_sim.sim.ride.rider_response_allocation import allocate_response
    from bike_sim.physics.rider_activation import activation_step
    model=mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0" timestep=".0005"/>
    <worldbody><body name="bike"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/>
      <joint type="hinge" axis="0 1 0"/><inertial mass="1000" pos="0 0 0" diaginertia="1000 1000 1000"/>
      <geom name="platform" type="box" size=".05 .025 .01" contype="0" conaffinity="0"/></body>
    <body name="rider"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/>
      <joint name="muscle" type="hinge" axis="0 1 0"/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
      <site name="sole"/></body></worldbody><equality><weld body1="rider" body2="bike"/></equality>
    <actuator><motor joint="muscle" gear="1" forcelimited="true" forcerange="-50 50"/></actuator></mujoco>''')
    data=mujoco.MjData(model);data.qfrc_applied[4]=-100.;data.qfrc_applied[5]=-35.
    mujoco.mj_forward(model,data)
    c=ArticulatedRiderController.__new__(ArticulatedRiderController)
    c.config=ArticulatedConfig(active_positive_power_limit_w=450.,activation_tau_s=.04)
    c.joints={'probe':(5,5,0)};c.strength=None;c.rider_mass=1.;c.active_state=np.array([previous])
    c._rider_dofs=np.array([3,4,5]);c._bike_dofs=np.array([0,1,2])
    c._alloc_attachments={'front':{'eq':0,'rider_body':model.body('rider').id}}
    c.soles={'front':model.site('sole').id};c.pedal_geoms={'front':model.geom('platform').id}
    c._last_branch=c._last_solution=None
    wished=activation_step(c.active_state,np.array([50.]),.005,.04)
    torque,diagnostic=allocate_response(c,model,data,wished,dt_s=.005)
    if previous==0.:
        assert not diagnostic['feasible']
        assert abs(torque[0]) <= 50.*(1.-np.exp(-.005/.04))+1e-7
    else:
        assert diagnostic['feasible']
        assert torque[0]>30.
    excitation=diagnostic['solution_excitation_nm']
    assert abs(excitation[0])<=50.
    np.testing.assert_allclose(torque,activation_step(c.active_state,excitation,.005,.04),atol=1e-7)
    from bike_sim.sim.ride.rider_effort import finalize_effort
    before=c.active_state.copy();c.activation_time_s=None;c.allocation_diagnostics=diagnostic
    c.last_terms={'probe':{'requested_nm':50.}}
    applied=finalize_effort(c,data,{'probe':float(torque[0])},advance=True,dt_s=.005,steady_state=False)
    assert applied['probe'] == pytest.approx(torque[0],abs=1e-9)
    np.testing.assert_allclose(c.active_state,activation_step(before,excitation,.005,.04),atol=1e-9)
    data.ctrl[0]=applied['probe'];mujoco.mj_forward(model,data)
    np.testing.assert_allclose(data.qacc[c._rider_dofs],diagnostic['solution_qddot'],atol=1e-8)


def test_activation_projection_and_jacobian_include_joint_and_whole_body_clamps():
    from bike_sim.sim.ride.rider_activation_allocation import ActivationLaw
    clamped=ActivationLaw([100.,100.],[10.,10.],[-40.,-40.],[40.,40.],
        dt_s=.005,tau_s=.04,power_limit=450.)
    np.testing.assert_allclose(clamped.delivered([40.,40.]),[22.5,22.5])
    assert np.all(clamped.latent([40.,40.])>40.)
    law=ActivationLaw([20.,20.],[20.,10.],[-50.,-50.],[50.,50.],
        dt_s=.005,tau_s=.04,power_limit=450.)
    excitation=np.array([10.,40.]);epsilon=1e-4
    numerical=np.column_stack([(law.delivered(excitation+epsilon*np.eye(2)[i])-
        law.delivered(excitation-epsilon*np.eye(2)[i]))/(2*epsilon) for i in range(2)])
    np.testing.assert_allclose(law.derivative(excitation),numerical,atol=1e-8)
    assert law.delivered(excitation)@law.speeds == pytest.approx(450.)


def test_linear_mode_rejection_respects_the_existing_feasibility_tolerance():
    from bike_sim.sim.ride.rider_response_allocation import linear_region_infeasible
    matrix=np.array([[1.],[-1.]])
    # The first interval has a witness under the existing 1e-7 residual
    # contract; an exact-LP precheck must not falsely reject it.
    assert not linear_region_infeasible(matrix,np.array([0.,-.5e-7]),np.array([-1.]),np.array([1.]))
    assert linear_region_infeasible(matrix,np.array([0.,-3e-7]),np.array([-1.]),np.array([1.]))


def test_grip_branch_uses_the_fixed_bar_anchor_when_the_soft_hand_moves():
    from types import SimpleNamespace
    from bike_sim.sim.ride.rider_response_allocation import allocation_support_point
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    from bike_sim.physics.rider_allocation import grip_constraints
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="bar"><freejoint/><inertial mass="1000" pos="0 0 0" diaginertia="1000 1000 1000"/></body>
      <body name="hand"><freejoint/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/><site name="grip"/></body>
      </worldbody><equality><connect body1="hand" body2="bar" anchor="0 0 0"/></equality></mujoco>''')
    data=mujoco.MjData(model);data.qpos[9]=.001;mujoco.mj_forward(model,data)
    c=SimpleNamespace(grip_sites={'left':model.site('grip').id})
    meta={'eq':0,'rider_body':model.body('hand').id,'other_body':model.body('bar').id}
    point=allocation_support_point(model,data,c,'grip_left',meta)
    np.testing.assert_array_equal(point,[0.,0.,0.])
    pelvis=np.array([-1.,0.]);force=np.array([.5,-1000.])
    direction=pelvis-point[[0,2]];direction/=np.linalg.norm(direction)
    press=grip_constraints(np.eye(2),direction,pulling=False)[0]
    assert (press.A@force)[0]<press.lb[0]
    # Sampling must keep that same compiled anchor, even if reset sees a
    # deformed hand site; otherwise the pull/press classification disagrees.
    contacts=RiderContactApplier.__new__(RiderContactApplier)
    contacts.supports={};contacts.steer=model.body('bar').id
    contacts.grip_sites={side:model.site('grip').id for side in ('left','right')}
    contacts.welded_grip=True
    contacts._grip_connect={side:SimpleNamespace(eq_id=0) for side in ('left','right')}
    contacts.reset(model,data)
    np.testing.assert_array_equal(contacts.grip_anchor_local['left'],[0.,0.,0.])
