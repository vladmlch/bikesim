import pytest
from bike_sim.sim.research.quality import energy_quality


def test_large_artificial_losses_do_not_mask_energy_defect():
    energy = dict(energy_scale_j=50., active_work_j=100., external_work_j=-10.,
                  loss_j=100000., residual_j=50000.)
    assert not energy_quality(energy).acceptable
    assert energy_quality(energy).residual_ratio == pytest.approx(50000./160.)


def test_both_energy_accounts_must_pass():
    energy = dict(energy_scale_j=50., active_work_j=100., residual_j=.01,
                  electrical_residual_j=1e-9, electrical_work_j=150.)
    assert energy_quality(energy).acceptable
    energy['electrical_residual_j'] = 1.
    assert not energy_quality(energy).acceptable


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), True])
def test_invalid_energy_is_not_a_passing_run(bad):
    with pytest.raises(ValueError):
        energy_quality({'residual_j': bad})


@pytest.mark.parametrize('bad', [0., -.1, float('nan'), True])
def test_initialization_refinement_schedule_is_positive(bad):
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    with pytest.raises(ValueError):
        SimulationPhysicsConfig('physical', equilibrium_refine_after_s=bad)
