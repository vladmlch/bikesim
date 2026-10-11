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


import tomllib as _tomllib

from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.terrain.alpine import alpine_scree, generate_alpine_track
from bike_sim.terrain.contact_profile import ProfileQuery
from bike_sim.terrain.contact_safety import single_contact_hazards
from bike_sim.terrain.heightfield import build_field_data
from bike_sim.terrain.obstacles import BowlPothole, Bump, RoadRoughness, RockGarden, SquareEdge
from bike_sim.terrain.profile import build_profile
from bike_sim.terrain.trackfile import dump_track, track_from_dict, track_to_dict
from bike_sim.terrain.wheelpath import FRONT_WHEEL_RADIUS_M, REAR_WHEEL_RADIUS_M

SEEDS = range(8)


def test_generate_rejects_bad_arguments():
    with pytest.raises(ValueError):
        generate_alpine_track(object(), seed=0)
    with pytest.raises(ValueError):
        generate_alpine_track(AlpineGenSpec(), seed=-1)
    with pytest.raises(ValueError):
        generate_alpine_track(AlpineGenSpec(), seed=1.0)


@pytest.mark.parametrize('seed', SEEDS)
def test_same_seed_same_bytes_and_toml_round_trip(seed):
    spec = AlpineGenSpec()
    first, second = dump_track(generate_alpine_track(spec, seed=seed)), dump_track(generate_alpine_track(spec, seed=seed))
    assert first == second
    doc = _tomllib.loads(first)
    assert track_to_dict(track_from_dict(doc)) == track_to_dict(generate_alpine_track(spec, seed=seed))


def test_different_seeds_differ():
    dumps = {dump_track(generate_alpine_track(AlpineGenSpec(), seed=s)) for s in SEEDS}
    assert len(dumps) == len(SEEDS)


def _body(track, spec):
    return spec.lead_in_m, spec.length_m - spec.lead_out_m


@pytest.mark.parametrize('seed', SEEDS)
def test_default_track_structure(seed):
    spec = AlpineGenSpec()
    track = generate_alpine_track(spec, seed=seed)
    track.validate()
    lo, hi = _body(track, spec)
    assert track.length_m == 120. and track.surface == 'hardpack' and track.name == f'alpine_{seed}'
    assert f'seed={seed}' in track.description and spec.sha256() in track.description
    assert all(lo <= o.start_m and o.end_m <= hi + 1e-9 for o in track.obstacles)
    assert all(lo <= s.start_m and s.end_m <= hi + 1e-9 for s in track.surface_sections)
    x = np.array([0., 5., 10., 105., 112., 120.])
    assert np.all(track.grade_profile.slope(x) == 0.)

    scree = [s for s in track.surface_sections if s.surface == 'scree']
    rough = [o for o in track.obstacles if isinstance(o, RoadRoughness)]
    assert len(scree) >= 2
    for section in scree:
        matches = [o for o in rough if abs(o.start_m - section.start_m) < 1e-6 and abs(o.end_m - section.end_m) < 1e-6]
        assert len(matches) == 1
        assert spec.scree_amp_m[0] * .8 ** 6 <= matches[0].amplitude_m <= spec.scree_amp_m[1]
    gardens = [o for o in track.obstacles if type(o) is RockGarden]
    boulders = [o for o in track.obstacles if isinstance(o, Bump)]
    bowls = [o for o in track.obstacles if isinstance(o, BowlPothole)]
    assert 1 <= len(gardens) <= spec.n_rock_gardens[1]
    assert 1 <= len(boulders) <= spec.n_boulders[1]
    assert 1 <= len(bowls) <= spec.n_water_bars[1]
    assert not any(isinstance(o, SquareEdge) for o in track.obstacles)
    assert all(spec.boulder_height_m[0] <= o.height_m <= spec.boulder_height_m[1] for o in boulders)
    assert all(o.depth_m <= spec.water_bar_depth_m[1] for o in bowls)
    assert build_field_data(track).shape[0] == 2


@pytest.mark.parametrize('seed', SEEDS)
def test_default_track_has_no_single_contact_hazards(seed):
    assert single_contact_hazards(generate_alpine_track(AlpineGenSpec(), seed=seed)) == []


def roll_multi_support_count(track, radius, *, delta_m=.010, step_m=.005):
    """Ground truth: roll a circle along the 5 mm raster through the production contact query."""
    cfg = TireBackendConfig()
    x = np.arange(0., track.length_m + 2. * radius + step_m / 2., step_m)
    z = build_profile(track, x)
    query = ProfileQuery(np.column_stack([x, z]), significant_delta_m=cfg.significant_delta_m,
                         significance_fraction=cfg.significance_fraction, normal_angle_deg=cfg.distinct_normal_deg)
    window = int(round(radius / step_m))
    count = 0
    for i in range(window + 1, len(x) - window - 1):
        lo, hi = i - window, i + window + 1
        dx = x[i] - x[lo:hi]
        lift = np.sqrt(np.maximum(radius * radius - dx * dx, 0.))
        centre = np.array([x[i], float(np.max(z[lo:hi] + lift)) - delta_m])
        count += query.contact(centre, radius).multi_support
    return count


@pytest.mark.parametrize('seed', range(3))
def test_default_track_rolls_clean_for_both_wheels(seed):
    track = generate_alpine_track(AlpineGenSpec(), seed=seed)
    assert roll_multi_support_count(track, FRONT_WHEEL_RADIUS_M) == 0
    assert roll_multi_support_count(track, REAR_WHEEL_RADIUS_M) == 0


def test_opting_out_of_single_contact_produces_detectable_hazards():
    spec = AlpineGenSpec(single_contact_safe=False, n_sharp_edges=(2, 2), rock_amp_m=(.10, .10), rock_corr_m=(.15, .15))
    track = generate_alpine_track(spec, seed=0)
    assert any(isinstance(o, SquareEdge) for o in track.obstacles)
    assert single_contact_hazards(track)
    assert roll_multi_support_count(track, FRONT_WHEEL_RADIUS_M) > 0


def test_redraw_keeps_a_marginal_rock_garden_clean():
    spec = AlpineGenSpec(rock_amp_m=(.07, .07), rock_corr_m=(.45, .45), n_rock_gardens=(2, 2))
    for seed in SEEDS:
        track = generate_alpine_track(spec, seed=seed)
        assert single_contact_hazards(track) == []
        assert all(o.amplitude_m <= .07 for o in track.obstacles if type(o) is RockGarden)


def test_alpine_scree_preset_builder():
    pinned, other = alpine_scree(), alpine_scree(seed=7)
    assert pinned.name == 'alpine_scree' and other.name == 'alpine_scree_7'
    assert dump_track(pinned) == dump_track(generate_alpine_track(AlpineGenSpec(), seed=ALPINE_PRESET_SEED, name='alpine_scree'))
