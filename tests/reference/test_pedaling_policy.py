"""PedalingPolicy after the reposition removal: a stall is an outcome, not a manoeuvre."""
import math

import pytest

from bike_sim.physics.physical_config import PedalingConfig
from bike_sim.physics.pedaling import PedalingPolicy, PedalingState


def test_reposition_knobs_and_mode_are_gone():
    with pytest.raises(TypeError):
        PedalingConfig(reposition_on_stall=True)
    assert 'reposition' not in ' '.join(PedalingConfig.__dataclass_fields__)
    policy = PedalingPolicy(PedalingConfig(enabled=True))
    with pytest.raises(TypeError):
        policy.update(0., 0., 0., 30., .001, reposition=True)


def test_a_rocking_loaded_crank_simply_keeps_pedalling():
    policy = PedalingPolicy(PedalingConfig(enabled=True, effort_slew_nm_s=0.))
    modes = set()
    for step in range(3000):
        rate = 3.*math.sin(step*.02)      # rocks around a dead spot for 3 s
        state = policy.update(math.pi/2, rate, 0., 40., .001)
        modes.add(state.mode)
    assert modes == {'pedaling'}


def test_ride_control_has_no_reposition_field():
    from bike_sim.sim.ride.control import RideControl
    assert 'crank_reposition' not in RideControl.__dataclass_fields__


def test_rider_program_rejects_the_removed_reposition_command():
    from bike_sim.sim.research.rider_program import RiderProgram
    with pytest.raises(ValueError, match='unknown or malformed rider keyframe'):
        RiderProgram.from_dict({'keyframes': [{'time_s': 0., 'crank_reposition': True}]})
