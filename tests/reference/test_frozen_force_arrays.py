"""Force arrays are frozen once per step and reused by every sample type."""
import numpy as np
import pytest


def test_freeze_force_is_immutable_finite_and_recognised():
    from bike_sim.sim.ride.telemetry_v2 import freeze_force, is_frozen_array
    frozen = freeze_force(np.arange(3.), 3)
    assert is_frozen_array(frozen) and not frozen.flags.writeable
    with pytest.raises(ValueError):
        frozen[0] = 1.
    with pytest.raises(ValueError):
        freeze_force(np.array([1., np.nan, 0.]), 3)
    with pytest.raises(ValueError):
        freeze_force(np.zeros(2), 3)
    assert not is_frozen_array(np.zeros(3))
    assert not is_frozen_array(np.zeros(3).view()[:])  # writeable view


def test_samples_reuse_frozen_arrays_without_copying():
    from bike_sim.sim.ride.telemetry_v2 import ForceSample, freeze_force
    from bike_sim.sim.ride.physical_samples import PhysicalSample
    q, v = freeze_force(np.zeros(4), 4), freeze_force(np.ones(4), 4)
    force = freeze_force(np.full(4, 2.), 4)
    sample = ForceSample(0., q, v, {'a': force})
    assert sample.components['a'] is force
    published = PhysicalSample(0, 0., .00125, q, v, {'a': force}, {})
    assert published.forces['a'] is force and published.qpos is q
