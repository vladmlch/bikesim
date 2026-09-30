import pytest
from bike_sim.cli.research import parser, make_environment


def _transmission(*extra):
    # lumped rider: building the articulated plant dominates runtime
    env = make_environment(parser().parse_args(['--scenario', 'flat', '--rider', 'lumped', *extra]))
    return env.sim.physics_config.drive.transmission_model


def test_default_transmission_is_ideal():
    assert _transmission() == 'ideal_mid_drive'


def test_elastic_chain_selectable_and_accepts_chain_spring_overrides():
    assert _transmission('--transmission', 'elastic_chain', '--chain-stiffness', '2e5') == 'elastic_chain'


def test_geometric_transmission_selectable():
    assert _transmission('--transmission', 'geometric_ideal_mid_drive') == 'geometric_ideal_mid_drive'


@pytest.mark.parametrize('transmission', ['ideal_mid_drive', 'geometric_ideal_mid_drive'])
@pytest.mark.parametrize('flag', [['--chain-stiffness', '1e5'], ['--freehub-stiffness', '100']])
def test_chain_spring_overrides_rejected_for_ideal_modes(transmission, flag):
    with pytest.raises(ValueError, match='elastic_chain'):
        make_environment(parser().parse_args(['--scenario', 'flat', '--transmission', transmission, *flag]))


def test_default_dt_follows_the_validated_step_per_transmission():
    from bike_sim.cli.research import resolve_dt
    p = parser()
    assert resolve_dt(p.parse_args([])) == .0005                       # ideal_mid_drive (dt sweep)
    assert resolve_dt(p.parse_args(['--transmission', 'geometric_ideal_mid_drive'])) == .0005
    assert resolve_dt(p.parse_args(['--transmission', 'elastic_chain'])) == .000125   # chain oscillates at 0.5 ms
    assert resolve_dt(p.parse_args(['--transmission', 'elastic_chain', '--dt', '0.00025'])) == .00025
