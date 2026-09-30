import pytest
from bike_sim.sim.research.rider_random import RiderRandomSpec, sample_rider


def test_deterministic_and_within_spec():
    spec = RiderRandomSpec()
    a, pa = sample_rider(spec, 5)
    b, pb = sample_rider(spec, 5)
    assert a == b and pa == pb
    c, pc = sample_rider(spec, 6)
    assert (a.mass_kg, pa) != (c.mass_kg, pc)
    for seed in range(20):
        rider, program = sample_rider(spec, seed)
        assert rider.variant == 'articulated_planar'
        assert spec.mass_kg[0] <= rider.mass_kg <= spec.mass_kg[1]
        assert spec.height_m[0] <= rider.height_m <= spec.height_m[1]
        key = program.keyframes[0]
        assert spec.torso_lean_rad[0] <= key.posture.torso_lean_rad <= spec.torso_lean_rad[1]
        assert spec.pelvis_pitch_rad[0] <= key.posture.pelvis_pitch_rad <= spec.pelvis_pitch_rad[1]
        assert spec.human_torque_nm[0] <= key.human_torque_nm <= spec.human_torque_nm[1]
        assert program.owns_human_effort


def test_program_is_a_static_posture_so_it_never_teleports():
    _, program = sample_rider(RiderRandomSpec(), 1)
    assert len(program.keyframes) == 1 and program.at(0.) == program.at(9.)


@pytest.mark.parametrize('bad', [dict(mass_kg=(100., 60.)), dict(mass_kg=(-1., 60.)),
                                 dict(torso_lean_rad=(0., 2.)), dict(human_torque_nm=(-1., 5.))])
def test_spec_validation(bad):
    with pytest.raises(ValueError):
        RiderRandomSpec(**bad)


def test_cli_rider_random_builds_sampled_rider_and_program():
    from bike_sim.cli.research import parser, build_rider
    args = parser().parse_args(['--rider-random', '--seed', '4', '--rider-seed', '9'])
    rider, program = build_rider(args)
    expected, expected_program = sample_rider(RiderRandomSpec(), 9)
    assert rider == expected and program == expected_program
    default_seed = build_rider(parser().parse_args(['--rider-random', '--seed', '4']))
    assert default_seed[0] == sample_rider(RiderRandomSpec(), 4)[0]
    fixed, none = build_rider(parser().parse_args(['--rider', 'lumped', '--rider-mass', '70']))
    assert fixed.mass_kg == 70. and fixed.variant == 'lumped' and none is None


def test_cli_rider_random_conflicts_are_explicit():
    from bike_sim.cli.research import parser, build_rider
    for extra in (['--rider', 'lumped'], ['--posture', 'forward'], ['--human-torque', '10']):
        with pytest.raises(ValueError, match='rider-random'):
            build_rider(parser().parse_args(['--rider-random', *extra]))
