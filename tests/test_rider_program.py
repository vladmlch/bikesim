from dataclasses import replace
import pytest
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.rider_program import RiderKeyframe, RiderProgram


def program(delay=0.):
    return RiderProgram((
        RiderKeyframe(0., RiderPosture(pelvis_offset_m=(0., 0.)), 0.),
        RiderKeyframe(1., RiderPosture(torso_lean_rad=.2, pelvis_offset_m=(.03, .1), use_saddle=False), 30.),
        RiderKeyframe(2., RiderPosture(pelvis_offset_m=(-.03, .05), use_saddle=False), 10.),
    ), reaction_delay_s=delay)


def test_c2_interpolation_bounds_and_delay():
    p = program(.1)
    assert p.at(-1.).posture == p.at(.1).posture
    assert p.at(.6).posture.torso_lean_rad == pytest.approx(.1)
    assert p.at(.6).human_torque_nm == pytest.approx(15.)
    assert not p.at(1.1).posture.use_saddle
    assert p.at(99.).posture == p.keyframes[-1].posture
    for knot in (.1, 1.1, 2.1):
        left, middle, right = [p.at(knot+d).posture.torso_lean_rad for d in (-1e-5, 0., 1e-5)]
        assert abs(right-left) < 1e-10
        assert abs(left-2*middle+right) < 1e-10


def test_program_only_returns_internal_effort_commands_and_detects_conflicts():
    p = program()
    command = RideControl(motor_torque_nm=80., motor_limit_nm=30.)
    result = p.apply(command, .5)
    assert result.motor_torque_nm == 80. and result.motor_limit_nm == 30.
    assert result.human_torque_nm == pytest.approx(15.)
    assert command.posture is None and command.human_torque_nm is None
    for bad in (replace(command, posture=RiderPosture()), replace(command, human_torque_nm=0.)):
        with pytest.raises(ValueError, match='owns'):
            p.apply(bad, .5)


def test_roundtrip_and_unknown_keys():
    p = program()
    assert RiderProgram.from_dict(p.to_dict()) == p
    with pytest.raises(ValueError, match='unknown'):
        RiderProgram.from_dict(p.to_dict() | {'teleport': True})
    data = p.to_dict()
    data['keyframes'][0]['posture']['unknown_angle'] = .1
    with pytest.raises(ValueError):
        RiderProgram.from_dict(data)


@pytest.mark.parametrize('frames', [(), (RiderKeyframe(1.),),
    (RiderKeyframe(0.), RiderKeyframe(0.)),
    (RiderKeyframe(0.), RiderKeyframe(1., human_torque_nm=10.)),
    (RiderKeyframe(0.), RiderKeyframe(1., RiderPosture(pelvis_offset_m=(0., .1)))),
])
def test_invalid_or_discontinuous_keyframes(frames):
    with pytest.raises(ValueError):
        RiderProgram(frames)


def test_nonfinite_and_wrong_types():
    with pytest.raises(ValueError):
        RiderKeyframe(float('nan'))
    with pytest.raises(ValueError):
        RiderKeyframe(0., human_torque_nm=-1.)
    with pytest.raises(ValueError):
        RiderProgram(program().keyframes, reaction_delay_s=-1.)
    with pytest.raises(ValueError):
        RiderProgram.from_dict({'keyframes': 'not a list'})
