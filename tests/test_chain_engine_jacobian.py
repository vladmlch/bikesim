import mujoco
import numpy as np
from bike_sim.physics.chain import chain_geometry,chain_center_gradient
from bike_sim.physics.physical_config import PhysicalDriveConfig
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier


def test_center_gradient_matches_both_external_tangent_branches():
    cf=np.array([.1,-.03])
    for cr in (np.array([-.5,.2]),np.array([.6,-.1])):
        for up in (np.array([0.,1.]),np.array([0.,-1.])):
            analytic=chain_center_gradient(cf,cr,.07,.04,up_xz=up)
            numeric=[]
            for i in range(2):
                h=np.eye(2)[i]*1e-6
                numeric.append((chain_geometry(cf,cr+h,.07,.04,up_xz=up)[0]-chain_geometry(cf,cr-h,.07,.04,up_xz=up)[0])/2e-6)
            np.testing.assert_allclose(analytic,numeric,rtol=1e-7,atol=1e-9)


def test_engine_analytic_jacobian_matches_read_only_finite_differences():
    m=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider='none',physics_config=SimulationPhysicsConfig('physical')))
    d=mujoco.MjData(m);drive=DrivetrainForceApplier(m,PhysicalDriveConfig(),'coast');mujoco.mj_forward(m,d);drive.reset(m,d)
    rng=np.random.default_rng(7)
    for _ in range(20):
        d.qpos[:]=rng.normal(0,.07,m.nq);mujoco.mj_forward(m,d)
        saved=d.qpos.copy();J=drive.jacobian(m,d);oracle=drive.finite_difference_jacobian(m,d)
        np.testing.assert_allclose(J,oracle,rtol=1e-7,atol=1e-9)
        np.testing.assert_array_equal(d.qpos,saved)
        assert abs(J[m.joint('root_x').dofadr[0]])<1e-12
        assert abs(J[m.joint('root_pitch').dofadr[0]])<1e-12
