"""Alpine scree climb: generator, preset, frozen files and CLI routing."""
import tomllib

import pytest

from bike_sim.terrain.alpine import ALPINE_PRESET_SEED, AlpineGenSpec
from bike_sim.terrain.surface import SURFACES, get_surface


def test_scree_surface_is_registered_below_loose():
    scree, loose = get_surface('scree'), get_surface('loose')
    assert 'scree' in SURFACES
    assert scree.mu_slide <= scree.mu_peak
    assert scree.mu_peak < loose.mu_peak
    assert scree.mu_slide < loose.mu_slide
    assert scree.slip_stiffness_per_load < loose.slip_stiffness_per_load
    assert scree.stribeck_speed_mps == loose.stribeck_speed_mps


def test_default_spec_matches_the_design_numbers():
    s = AlpineGenSpec()
    assert (s.length_m, s.lead_in_m, s.lead_out_m, s.transition_m) == (120., 10., 15., 2.)
    assert s.ramp_grade == (.15, .25) and s.wall_grade == (.26, .32) and s.n_walls == 1
    assert s.counter_grade == (-.08, -.03) and s.counter_prob == .3
    assert s.base_surface == 'hardpack' and s.n_scree == (2, 3) and s.scree_ramp_bias == .7
    assert s.n_sharp_edges == (0, 0) and s.single_contact_safe is True
    assert (s.min_contact_radius_m, s.max_kink_deg, s.contact_window_m) == (.45, 17., .25)
    assert isinstance(ALPINE_PRESET_SEED, int)


def test_spec_round_trips_through_dict_and_toml_lists():
    spec = AlpineGenSpec(ramp_grade=[.10, .20], n_boulders=[1, 2])
    assert spec.ramp_grade == (.10, .20) and spec.n_boulders == (1, 2)
    again = AlpineGenSpec.from_dict(tomllib.loads('ramp_grade = [0.10, 0.20]\nn_boulders = [1, 2]\n'))
    assert again == spec and again.sha256() == spec.sha256()
    assert AlpineGenSpec.from_dict(spec.to_dict()) == spec
    assert AlpineGenSpec().sha256() != spec.sha256()


@pytest.mark.parametrize('bad', [
    dict(unknown_key=1),
    dict(ramp_grade=(.25, .15)),
    dict(counter_grade=(-.05, .01)),
    dict(n_boulders=(1.5, 2)),
    dict(counter_prob=1.5),
    dict(n_walls=2),
    dict(wall_grade=(.20, .30)),
    dict(base_surface='ice'),
    dict(length_m=30.),
    dict(n_sharp_edges=(1, 1)),
    dict(single_contact_safe='yes'),
])
def test_spec_rejects_invalid_values(bad):
    with pytest.raises(ValueError):
        AlpineGenSpec.from_dict(bad)


def test_sharp_edges_need_the_multi_contact_opt_in():
    spec = AlpineGenSpec(single_contact_safe=False, n_sharp_edges=(1, 2))
    assert spec.n_sharp_edges == (1, 2)
