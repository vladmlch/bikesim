"""A 1e-14 state perturbation diverges at a measurable, repeatable step."""
import pytest

@pytest.mark.slow
def test_perturbation_diverges_within_episode(tmp_path):
    from test_pinned_topology import _pinned_config
    from bike_sim.cli import research as research_cli
    from tools.divergence_horizon import measure_horizon
    def make():
        args = research_cli.parser().parse_args([
            '--physics-config', str(_pinned_config(tmp_path)),
            '--track-file', 'examples/research/rough_uphill_savage.toml',
            '--duration', '1', '--dt', '.00125',
            # Non-strict monitor: record violations in the channel rows
            # instead of dying at the first violated period close (strict
            # raises at t=0.005 on this track, before perturb_at=10).
            '--diagnostic-model-limits',
            '--out', str(tmp_path/'out')])
        return research_cli.make_environment(args)
    h = measure_horizon(make, 400, perturb={'qvel_idx': 0, 'eps': 1e-14})
    # Измерено h≈26 на savage/400 шагов (dt=1.25ms, tol=1e-9): первый канал за
    # tol — suspension.generalized_force_components_n.shock_damper; сам qvel.0
    # к этому шагу вырос лишь до ~1e-13. Число — граница слоя-2 для поздних планов.
    assert isinstance(h, int) and 0 < h < 400

@pytest.mark.slow
def test_unperturbed_never_diverges(tmp_path):
    from test_pinned_topology import _pinned_config
    from bike_sim.cli import research as research_cli
    from tools.divergence_horizon import measure_horizon
    def make():
        args = research_cli.parser().parse_args([
            '--physics-config', str(_pinned_config(tmp_path)),
            '--track-file', 'examples/research/rough_uphill_savage.toml',
            '--duration', '1', '--dt', '.00125',
            '--diagnostic-model-limits',
            '--out', str(tmp_path/'out')])
        return research_cli.make_environment(args)
    assert measure_horizon(make, 200, perturb={'qvel_idx': 0, 'eps': 0.}) == 200
