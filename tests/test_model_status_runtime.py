"""The monitor observes once and cannot change a physical trajectory."""
import numpy as np
import pytest
from bike_sim.validation.ride_cases import make_sim
from bike_sim.sim.ride.initial_state import PhysicalInitialState
from bike_sim.sim.ride_sim import RideSimulation


def test_status_monitor_has_no_force_or_state_side_effect(monkeypatch):
    source=make_sim(.0005,rider='none')
    seed=PhysicalInitialState.capture(source)
    observed=RideSimulation(track=source.track,rider=source.rider,physics_config=source.physics_config,physical_initial_state=seed)
    silent=RideSimulation(track=source.track,rider=source.rider,physics_config=source.physics_config,physical_initial_state=seed)
    monkeypatch.setattr(silent.physical.model_status,'observe',lambda *args:None)
    for _ in range(12):
        observed.step();silent.step()
        np.testing.assert_array_equal(observed.data.qpos,silent.data.qpos)
        np.testing.assert_array_equal(observed.data.qvel,silent.data.qvel)
    assert observed.physical.model_status.last_interval==11
    assert observed.physical.sample.channels['model_status']['model_valid'] is True
    assert observed.physical.model_status.calibration_status=='parameterized_unvalidated'
    preview=RideSimulation(track=source.track,rider=source.rider,physics_config=source.physics_config,physical_initial_state=seed)
    preview.physical.interactive_preview=True
    for _ in range(12):preview.step()
    np.testing.assert_array_equal(preview.data.qpos,observed.data.qpos)
    np.testing.assert_array_equal(preview.data.qvel,observed.data.qvel)
    assert preview.physical.model_status.last_interval==11
    assert preview.physical.model_status.numerically_valid=='not_evaluated'
