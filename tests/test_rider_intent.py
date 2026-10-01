from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.physics.seated_climb import SeatedClimbConfig, SeatedClimbSignals
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.rider_intent import RiderIntentResolver
import pytest


SIGNALS = SeatedClimbSignals(0., (1.96, 0., 9.61), 8., 20.)


def resolver():
    return RiderIntentResolver(SeatedClimbConfig(enabled=True, reaction_delay_s=0.), .00125)


def test_repeated_force_evaluation_does_not_advance_intention_twice():
    intent = resolver()
    first = intent.resolve(RideControl(), SIGNALS, step=0)
    assert intent.resolve(RideControl(), SIGNALS, step=0) == first
    assert intent.resolve(RideControl(), SIGNALS, step=7) == first
    later = intent.resolve(RideControl(), SIGNALS, step=8)
    assert later.human_torque_nm > first.human_torque_nm


def test_nonadvancing_probe_predicts_without_consuming_reaction_state():
    intent = resolver()
    expected = intent.resolve(RideControl(), SIGNALS, step=0, advance=False)
    assert intent.resolve(RideControl(), SIGNALS, step=0, advance=False) == expected
    assert intent.resolve(RideControl(), SIGNALS, step=0) == expected


def test_motor_authority_and_explicit_rider_fields_are_preserved():
    intent = resolver()
    explicit = RideControl(motor_torque_nm=40., motor_limit_nm=20., human_torque_nm=0.,
                           posture=RiderPosture(torso_lean_rad=.1))
    assert intent.resolve(explicit, SIGNALS, step=0) == explicit
    partial = intent.resolve(RideControl(motor_torque_nm=40., human_torque_nm=0.), SIGNALS, step=8)
    assert partial.human_torque_nm == 0.
    assert partial.posture.torso_lean_rad > 0.
    assert partial.motor_torque_nm == 40.


def test_equilibrium_does_not_consume_high_level_clock():
    intent = resolver()
    control = RideControl()
    assert intent.resolve(control, SIGNALS, step=100, active=False) == control
    assert intent.resolve(control, SIGNALS, step=0) == resolver().resolve(control, SIGNALS, step=0)


def test_reset_restores_initial_intention():
    intent = resolver()
    first = intent.resolve(RideControl(), SIGNALS, step=0)
    intent.resolve(RideControl(), SIGNALS, step=8)
    intent.reset()
    assert intent.resolve(RideControl(), SIGNALS, step=0) == first


def test_high_level_clock_must_match_physics_steps():
    with pytest.raises(ValueError, match='multiple'):
        RiderIntentResolver(SeatedClimbConfig(enabled=True), .003)


def test_disabling_rider_does_not_break_the_clock_on_reenable():
    intent = resolver()
    intent.resolve(RideControl(), SIGNALS, step=0)
    disabled = RideControl(rider_enabled=False)
    assert intent.resolve(disabled, SIGNALS, step=8) == disabled
    resumed = intent.resolve(RideControl(), SIGNALS, step=16)
    assert resumed.rider_enabled
    assert resumed.human_torque_nm > 0.
