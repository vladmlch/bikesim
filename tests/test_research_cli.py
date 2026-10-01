import pytest
import bike_sim.cli.research as cli
import bike_sim.sim.research.configuration as configuration


@pytest.mark.parametrize('variant, first, period', [('lumped', .1, .5), ('articulated_planar', 3., 3.)])
def test_cli_uses_appropriate_initial_refinement_and_fine_dynamics(monkeypatch, variant, first, period):
    captured = {}
    def capture(**kwargs):
        captured.update(kwargs)
        return object()
    monkeypatch.setattr(configuration, 'RideSimulation', capture)
    monkeypatch.setattr(configuration, 'ResearchEnvironment', lambda *args, **kwargs: args)
    result = cli.make_environment(cli.parser().parse_args(['--rider', variant]))
    config = captured['physics_config']
    assert config.equilibrium_refine_after_s == first
    assert config.equilibrium_refine_period_s == period
    assert config.timestep_s == .0005  # ideal_mid_drive default; elastic_chain keeps .000125
    assert config.pitch_assist is False
    assert config.tires.surface_mode == 'track'
    assert result[1].control_period_s == .01
