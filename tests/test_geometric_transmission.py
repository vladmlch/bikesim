from dataclasses import replace
import numpy as np
import pytest
import mujoco
from bike_sim.physics.transmission_constraint import linearized_upper_bound,constraint_reaction,transmission_geometry
from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.physical_config import PhysicalDriveConfig,TireBackendConfig
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.sim.ride.geometric_freehub import GeometricFreehubConstraint


def model(rider='none'):
    gearing=DrivetrainSpecs(34,51)
    cfg=SimulationPhysicsConfig('physical',drive_mode='crank_effort',timestep_s=.0003125,closure_time_constant_s=.0025,
        drive=PhysicalDriveConfig(transmission_model='geometric_ideal_mid_drive',gearing=gearing),
        tires=TireBackendConfig(backend='compliant_2d'))
    m=mujoco.MjModel.from_xml_string(generate_mujoco_xml(specs=BikeSpecs(),mode='ride',rider=rider,physics_config=cfg))
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    return m,d,gearing


def forward(m,d):
    mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)


def test_linear_bound_and_full_work():
    q=np.array([.3,.5,.02]);j=np.array([.08,-.12,.4]);phi=.003;b=.005
    coeff,upper=linearized_upper_bound(phi,j,q,b)
    assert upper-coeff@q==pytest.approx(b-phi)
    v=np.array([2.,1.,.1]);f=constraint_reaction(100.,j)
    assert f@v==pytest.approx(-100.*(j@v)) and f[2]==pytest.approx(-40.)


@pytest.mark.parametrize('phase',[0.,3.13,3.15,20.*np.pi+.1])
def test_every_coordinate_analytic_jacobian_and_spin_unwrap(phase):
    m,d,g=model('articulated_planar');d.qpos[m.joint('crank_spin').qposadr[0]]=phase
    d.qpos[m.joint('root_pitch').qposadr[0]]=.2
    d.qpos[m.joint('horst_pivot').qposadr[0]]+=.03
    forward(m,d);phi,j=transmission_geometry(m,d,g)
    q=d.qpos.copy();oracle=np.empty(m.nv);eps=1e-7
    for i in range(m.nq):
        d.qpos[:]=q;d.qpos[i]+=eps;forward(m,d);plus=transmission_geometry(m,d,g)[0]
        d.qpos[:]=q;d.qpos[i]-=eps;forward(m,d);minus=transmission_geometry(m,d,g)[0]
        oracle[i]=(plus-minus)/(2*eps)
    np.testing.assert_allclose(j,oracle,rtol=3e-6,atol=1e-7)
    assert abs(j[m.joint('horst_pivot').dofadr[0]])>1e-4


def test_common_translation_rotation_invariance():
    m,d,g=model();p0,j0=transmission_geometry(m,d,g)
    d.qpos[m.joint('root_x').qposadr[0]]+=100.
    d.qpos[m.joint('root_z').qposadr[0]]+=30.
    d.qpos[m.joint('root_pitch').qposadr[0]]+=7.
    forward(m,d);p,j=transmission_geometry(m,d,g)
    assert p==pytest.approx(p0,abs=1e-12)
    np.testing.assert_allclose(j,j0,atol=1e-12)


def test_live_state_unchanged_and_gear_change_preserves_physical_gap():
    m,d,g=model('articulated_planar');hub=GeometricFreehubConstraint(m,g)
    hub.reset(m,d)
    d.qpos[hub.wheel_qpos]+=.2;forward(m,d)
    # An opened freehub gap, before a ratchet takes up the new boundary.
    old_phi,_=transmission_geometry(m,d,g);old_gap=hub.boundary-old_phi
    q=d.qpos.copy();v=d.qvel.copy();time=d.time
    hub.set_ratio(m,d,34/42)
    new_phi,_=transmission_geometry(m,d,hub.gearing)
    assert hub.boundary-new_phi==pytest.approx(old_gap,abs=1e-13)
    np.testing.assert_array_equal(q,d.qpos);np.testing.assert_array_equal(v,d.qvel);assert time==d.time
    assert m.ntendon==1
    assert m.tendon_num[hub.tendon_id]==m.nv
    assert abs(hub.shift_parameter_work_j)<1e-10


def test_initial_candidates_do_not_accumulate_a_temporal_ratchet():
    m,d,g=model();hub=GeometricFreehubConstraint(m,g);hub.reset(m,d);boundary=hub.boundary;q=d.qpos.copy()
    d.qpos[hub.wheel_qpos]+=1.;hub.prepare_initial_candidate(m,d,boundary)
    d.qpos[:]=q;hub.prepare_initial_candidate(m,d,boundary)
    assert hub.boundary==pytest.approx(boundary)


def test_geometric_freehub_has_no_tensile_force_when_wheel_overruns():
    from bike_sim.validation.drive_suspension_rig import _fixture
    m,d,drive,_,_= _fixture(.0003125,'geometric_ideal_mid_drive',.015)
    m.opt.gravity[:]=0.
    d.qvel[m.joint('rear_wheel_spin').dofadr[0]]=30.
    drive.ideal_hub.reset(m,d)
    for _ in range(20):
        drive.ideal_hub.prepare(m,d)
        mujoco.mj_step(m,d)
        force=drive.ideal_hub.solved_qfrc(m,d)
        assert np.linalg.norm(force)<1e-8
