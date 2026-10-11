"""Alpine scree climb: generator, preset, frozen files and CLI routing."""
import tomllib

import numpy as np
import pytest

from bike_sim.terrain.alpine import ALPINE_PRESET_SEED, AlpineGenSpec, _rhythm, _segments
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


def test_spec_hash_is_stable_across_numeric_input_types():
    plain = AlpineGenSpec()
    assert AlpineGenSpec(length_m=120).sha256() == AlpineGenSpec(length_m=120.).sha256()
    mixed = AlpineGenSpec(length_m=np.int64(120), counter_prob=np.float64(.3))
    assert mixed.length_m == 120. and isinstance(mixed.length_m, float)
    assert mixed.counter_prob == .3
    assert mixed.sha256() == plain.sha256()
    # np.float32 validates too but widens to a different double; it must still hash cleanly.
    widened = AlpineGenSpec(counter_prob=np.float32(.3))
    assert isinstance(widened.counter_prob, float)
    widened.sha256()


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


def _kinds(seed, **overrides):
    spec = AlpineGenSpec(**overrides)
    return spec, _segments(spec, np.random.default_rng(seed))


@pytest.mark.parametrize('seed', range(8))
def test_rhythm_fills_the_body_with_ramp_shelf_cycles(seed):
    spec, segments = _kinds(seed)
    body_end, t = spec.length_m - spec.lead_out_m, spec.transition_m
    kinds = [s[0] for s in segments]
    assert kinds[0] in ('ramp', 'wall')
    assert kinds.count('wall') == 1
    assert all(k in ('ramp', 'wall', 'shelf', 'counter') for k in kinds)
    for (_, s, e, _g), (_, s2, _e2, _g2) in zip(segments, segments[1:]):
        assert s < e and s2 == pytest.approx(e + t, abs=1e-9)
    assert segments[0][1] == pytest.approx(spec.lead_in_m + t)
    assert segments[-1][2] <= body_end - t + 1e-9
    assert body_end - t - segments[-1][2] < spec.ramp_len_m[0] + t  # no room left for another ramp
    for kind, s, e, g in segments:
        lo, hi = {'ramp': spec.ramp_grade, 'wall': spec.wall_grade, 'shelf': spec.shelf_grade,
                  'counter': spec.counter_grade}[kind]
        assert lo <= g <= hi
        if kind == 'ramp':
            assert spec.ramp_len_m[0] <= e - s <= spec.ramp_len_m[1] + 1e-9


@pytest.mark.parametrize('seed', range(8))
def test_rhythm_grade_profile_is_flat_outside_the_body(seed):
    spec = AlpineGenSpec()
    grade, ramps = _rhythm(spec, np.random.default_rng(seed))
    x = np.linspace(0., spec.length_m, 2401)
    slope = grade.slope(x)
    assert np.all(slope[x <= spec.lead_in_m] == 0.)
    assert np.all(slope[x >= spec.length_m - spec.lead_out_m] == 0.)
    assert slope.max() <= spec.wall_grade[1] + 1e-12
    assert slope.min() >= spec.counter_grade[0] - 1e-12
    assert ramps and all(spec.lead_in_m < s < e < spec.length_m - spec.lead_out_m for s, e in ramps)
    assert grade.elevation(spec.length_m) > 8.  # a climb, not a flat


def test_rhythm_without_walls_or_counter_slopes():
    spec, segments = _kinds(3, n_walls=0, counter_prob=0.)
    assert {s[0] for s in segments} <= {'ramp', 'shelf'}
