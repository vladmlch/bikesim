from dataclasses import replace
import numpy as np
import pytest
from bike_sim.validation.ride_cases import make_sim
from bike_sim.sim.ride.initial_state import PhysicalInitialState
from bike_sim.sim.ride_sim import RideSimulation


def test_transfer_is_explicit_exact_and_only_before_first_step():
    sim=make_sim(.0005)
    seed=PhysicalInitialState.capture(sim)
    cfg=replace(sim.physics_config,timestep_s=.00025)
    target=RideSimulation(track=sim.track,rider=sim.rider,physics_config=cfg,physical_initial_state=seed)
    np.testing.assert_array_equal(sim.data.qpos,target.data.qpos)
    np.testing.assert_array_equal(sim.data.qvel,target.data.qvel)
    assert target.equilibrium['initial_state_sha256']==seed.sha256
    assert target.equilibrium['cache_hit'] is False
    assert abs(target.physical.initial_energy_j-sim.physical.initial_energy_j)<1e-10
    target.step()
    with pytest.raises(ValueError,match='first step'):seed.restore(target.physical)
    with pytest.raises(ValueError,match='first step'):PhysicalInitialState.capture(target)
    with pytest.raises(ValueError,match='contract'):
        RideSimulation(track=sim.track,rider=sim.rider,physics_config=replace(cfg,initial_speed_mps=2.),physical_initial_state=seed)


def test_quadrature_projection_keeps_generalized_initial_state_explicit():
    sim=make_sim(.0005,rider='none',backend='distributed_2d_reference')
    n=sim.physics_config.tires.distributed.station_count
    seed=PhysicalInitialState.project_stations(sim,n*2)
    cfg=replace(sim.physics_config,tires=replace(sim.physics_config.tires,
        distributed=replace(sim.physics_config.tires.distributed,station_count=n*2)))
    target=RideSimulation(track=sim.track,rider=sim.rider,physics_config=cfg,physical_initial_state=seed)
    np.testing.assert_array_equal(sim.data.qpos,target.data.qpos)
    np.testing.assert_array_equal(sim.data.qvel,target.data.qvel)
    assert seed.payload['station_projection']['target_count']==n*2
    assert target.equilibrium['cache_hit'] is False
    assert all(0<=i<n*2 for side,i in target.physical.tire.states)
    target.step()
    assert np.isfinite(target.data.qvel).all()
