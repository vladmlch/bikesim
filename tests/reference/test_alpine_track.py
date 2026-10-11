"""Alpine scree climb: generator, preset, frozen files and CLI routing."""
from bike_sim.terrain.surface import SURFACES, get_surface


def test_scree_surface_is_registered_below_loose():
    scree, loose = get_surface('scree'), get_surface('loose')
    assert 'scree' in SURFACES
    assert scree.mu_slide <= scree.mu_peak
    assert scree.mu_peak < loose.mu_peak
    assert scree.mu_slide < loose.mu_slide
    assert scree.slip_stiffness_per_load < loose.slip_stiffness_per_load
    assert scree.stribeck_speed_mps == loose.stribeck_speed_mps
