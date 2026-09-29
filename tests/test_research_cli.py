import pytest
import bike_sim.cli.research as cli


@pytest.mark.parametrize('variant, first, period', [('lumped', .1, .5), ('articulated_planar', 3., 3.)])
def test_cli_uses_appropriate_initial_refinement_and_fine_dynamics(monkeypatch, variant, first, period):
    captured = {}
    def capture(**kwargs):
        captured.update(kwargs)
        return object()
    monkeypatch.setattr(cli, 'RideSimulation', capture)
    monkeypatch.setattr(cli, 'ResearchEnvironment', lambda *args: args)
    result = cli.make_environment(cli.parser().parse_args(['--rider', variant]))
    config = captured['physics_config']
    assert config.equilibrium_refine_after_s == first
    assert config.equilibrium_refine_period_s == period
    assert config.timestep_s == .000125
    assert config.pitch_assist is False
    assert config.tires.surface_mode == 'track'
    assert result[1].control_period_s == .01
