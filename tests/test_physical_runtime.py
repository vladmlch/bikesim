from dataclasses import replace
import mujoco
import numpy as np
import pytest
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig,PhysicalDriveConfig,ResistanceConfig
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def make(backend='compliant_2d',drive='coast',rider='none',speed=0.,torque=0.):
    return RideSimulation(track=get_preset('flat'),rider=rider,physics_config=SimulationPhysicsConfig(
        'physical',drive_mode=drive,tires=TireBackendConfig(backend=backend),
        drive=PhysicalDriveConfig(human_torque_nm=torque),initial_speed_mps=speed))

@pytest.mark.parametrize('backend',['native_reference','compliant_2d'])
def test_static_balance_uses_compiled_weight(backend):
    sim=make(backend)
    weight=sim.model.body_mass.sum()*9.81
    assert sim.equilibrium['total_vertical_force_n']==pytest.approx(weight,rel=.005)
    assert sim.equilibrium['residual_qacc'] <= .05


def test_compliant_has_no_native_wheel_contacts_and_single_mapping():
    sim=make(speed=2.)
    for _ in range(20): sim.step()
    assert sim.physical.sample is not None
    wheel_geoms={sim.contact_query.front_id,sim.contact_query.rear_id}
    assert not any(c.geom1 in wheel_geoms or c.geom2 in wheel_geoms for c in sim.data.contact)
    assert 'tires' in sim.last_force_sample.components
    assert abs(sim.physical.energy['residual_j'])/sim.physical.energy_scale_j<.01


def test_coast_reset_reproduces_initial_energy_and_clock():
    sim=make(speed=1.)
    initial=sim.data.qpos.copy()
    for _ in range(5): sim.step()
    sim.reset()
    np.testing.assert_allclose(sim.data.qpos,initial,atol=1e-10)
    assert sim.physical.history.duration_s == 0
    assert sim.time_s == 0
    sim.step()
    assert sim.physical.sample.interval_id == 0


def test_no_poststep_velocity_used_in_work():
    sim=make(speed=2.)
    sim.step()
    sample=sim.physical.sample
    frozen=sample.powers_w
    sim.data.qvel[:] *= 10
    assert sample.powers_w == frozen
    for key,force in sample.forces.items():
        assert sim.physical.history.work_j[key] == pytest.approx(float(force@sample.qvel)*sample.dt_s)
