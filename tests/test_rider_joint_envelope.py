from dataclasses import replace,asdict
from pathlib import Path
import json
import mujoco
import numpy as np
import pytest
from bike_sim.physics.rider_envelope import JointEnvelope,joint_q_range,load_joint_envelopes,soft_edge_response,RIDER_JOINTS


def test_coordinate_sign_and_neutral_are_explicit():
    a=JointEnvelope(1.,1,.1,2.4,'synthetic-test')
    b=replace(a,direction=-1)
    assert joint_q_range(a)==pytest.approx((-.9,1.4))
    assert joint_q_range(b)==pytest.approx((-1.4,.9))


@pytest.mark.parametrize('changes',[{'direction':0},{'direction':True},{'provenance':''},
    {'neutral_anatomical_rad':float('nan')},{'minimum_anatomical_rad':3.}])
def test_invalid_coordinate_contract(changes):
    with pytest.raises(ValueError):replace(JointEnvelope(1.,1,.1,2.4,'test'),**changes)


def test_exact_internal_joint_coverage(tmp_path):
    profile={name:asdict(JointEnvelope(1.,1,.1,2.4,'test')) for name in RIDER_JOINTS}
    path=tmp_path/'joints.json';path.write_text(json.dumps(profile))
    assert len(load_joint_envelopes(str(path)))==9
    profile['rider_root_pitch']=profile.pop('rider_shoulder')
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError):load_joint_envelopes(str(path))


def test_compiled_ranges_radians_root_free_and_mass_unchanged():
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.rider import RiderSpecs
    cfg=SimulationPhysicsConfig('physical',drive_mode='articulated_effort')
    rider=RiderSpecs(variant='articulated_planar')
    baseline=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,physics_config=cfg))
    path=str(Path('examples/research/rider_joint_envelope_synthetic.json').resolve())
    cfg=replace(cfg,articulated=replace(cfg.articulated,joint_envelope_path=path))
    model=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,physics_config=cfg))
    for name,env in load_joint_envelopes(path).items():
        assert model.joint(name).limited[0]
        np.testing.assert_allclose(model.joint(name).range,joint_q_range(env))
    for name in ('rider_root_x','rider_root_z','rider_root_pitch'):
        assert not model.joint(name).limited[0]
    np.testing.assert_array_equal(model.body_mass,baseline.body_mass)
    np.testing.assert_array_equal(model.body_inertia,baseline.body_inertia)


@pytest.mark.parametrize('q',[-1.1,-.95,0.,.95,1.1])
def test_soft_edge_force_is_negative_potential_gradient(q):
    tau,u=soft_edge_response(q,-1.,1.,100.,.1)
    eps=1e-7
    plus=soft_edge_response(q+eps,-1.,1.,100.,.1)[1]
    minus=soft_edge_response(q-eps,-1.,1.,100.,.1)[1]
    assert tau==pytest.approx(-(plus-minus)/(2*eps),abs=1e-6)
    assert u>=0 and float(tau*0.)==0.


def test_ik_clips_and_reports_joint_envelope():
    from tests.test_rider_posture_tracking import rig
    # Use the fixture constructor without a nested pytest request.
    model,data,controller=rig.__wrapped__()
    for side in ('front','rear'):
        controller.joint_ranges[f'rider_knee_{side}']=(.5,.6)
        targets=controller._targets(model,data,side)
        assert .5<=targets[1]<=.6
        assert controller.saturated_ik[side]
