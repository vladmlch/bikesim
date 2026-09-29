"""Initial contact losses must use the settled state, not stale relaxation rows."""
import copy
import mujoco
import numpy as np
import pytest
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.rider_contacts import RiderContactApplier
from bike_sim.sim.ride.rider_control import ArticulatedRiderController


def test_initial_contact_refresh_and_release_account_for_current_geometry():
    cfg=SimulationPhysicsConfig('physical',drive_mode='articulated_effort')
    specs=BikeSpecs();rider=RiderSpecs(variant='articulated_planar')
    pose=geometry_pose(rider,specs)
    m=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,physics_config=cfg))
    d=mujoco.MjData(m)
    controller=ArticulatedRiderController(m,pose,cfg.articulated,specs.crank_length/1000.)
    controller.initialize(m,d)
    contact=RiderContactApplier(m,pose,cfg.articulated)
    contact.reset(m,d)
    contact.compute_qfrc(m,d,m.opt.timestep)
    old=contact.stored_energy(m,d)
    # The equilibrium optimizer is allowed to change initial coordinates.
    # A final refresh must synchronize the loss datum before the ride starts.
    d.qpos[m.joint('rider_root_z').qposadr[0]]-=.001
    mujoco.mj_forward(m,d)
    contact.restart_clock()
    contact.initialize_settled_state(m,d)
    energy=contact.stored_energy(m,d)
    assert energy>old
    assert contact.last_time_s is None
    contact.release_all()
    assert contact.pending_release_loss_j==pytest.approx(energy,rel=1e-12,abs=1e-12)
    assert contact.stored_energy(m,d)==0.
    contact.compute_qfrc(m,d,m.opt.timestep)
    assert contact.loss_step_j==pytest.approx(energy,rel=1e-12,abs=1e-12)
    assert contact.pending_release_loss_j==0.


def test_numpy_contact_flags_remain_boolean_in_frozen_samples():
    from bike_sim.sim.ride.physical_samples import freeze,plain
    frozen=freeze({'feasible':np.bool_(True),'released':np.bool_(False)})
    assert plain(frozen)=={'feasible':True,'released':False}
    assert type(frozen['feasible']) is bool
