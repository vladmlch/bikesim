from dataclasses import replace
import numpy as np
import pytest
from bike_sim.terrain.generator import TerrainGenSpec, generate_track
from bike_sim.terrain.obstacles import Bump, Drop, RoadRoughness, Roots, SquareEdge
from bike_sim.terrain.profile import build_profile
from bike_sim.terrain.trackfile import load_track, save_track


def _profile(track):
    return build_profile(track, np.arange(0., track.length_m, .01))


def test_deterministic_per_seed():
    a = generate_track(TerrainGenSpec(), seed=42)
    b = generate_track(TerrainGenSpec(), seed=42)
    c = generate_track(TerrainGenSpec(), seed=43)
    assert np.array_equal(_profile(a), _profile(b))
    assert a.description == b.description
    assert a.surface_sections == b.surface_sections and a.surface == b.surface
    assert a.length_m != c.length_m or not np.array_equal(_profile(a)[:1000], _profile(c)[:1000])


@pytest.mark.parametrize('seed', range(12))
def test_spec_bounds_respected(seed):
    spec = TerrainGenSpec()
    t = generate_track(spec, seed=seed)
    t.validate()  # no overlap, inside track, known surface
    assert spec.length_m[0] <= t.length_m <= spec.length_m[1]
    lo, hi = spec.lead_in_m, t.length_m-spec.lead_out_m
    rough = [o for o in t.obstacles if isinstance(o, RoadRoughness)]
    assert spec.n_roughness[0] >= 0 and len(rough) <= spec.n_roughness[1]
    for o in t.obstacles:
        assert lo <= o.start_m and o.end_m <= hi, 'lead-in/out must stay clean'
        if isinstance(o, RoadRoughness):
            assert spec.roughness_amp_m[0]-1e-9 <= o.amplitude_m <= spec.roughness_amp_m[1]+1e-9
        elif isinstance(o, Bump):
            assert spec.bump_height_m[0]-1e-9 <= o.height_m <= spec.bump_height_m[1]+1e-9
        elif isinstance(o, SquareEdge):
            assert spec.edge_height_m[0]-1e-9 <= o.height_m <= spec.edge_height_m[1]+1e-9
        elif isinstance(o, Drop):
            assert spec.drop_height_m[0]-1e-9 <= o.height_m <= spec.drop_height_m[1]+1e-9
        elif isinstance(o, Roots):
            assert o.height_m <= spec.bump_height_m[1]+1e-9
    features = [o for o in t.obstacles if not isinstance(o, RoadRoughness)]
    assert len(features) <= spec.n_features[1]
    assert len(t.surface_sections) <= spec.n_surface_sections[1]
    assert t.surface in spec.base_surfaces
    assert all(s.surface in spec.section_surfaces and lo <= s.start_m and s.end_m <= hi
               for s in t.surface_sections)


@pytest.mark.parametrize('seed', range(12))
def test_grade_stays_in_envelope(seed):
    spec = TerrainGenSpec()
    t = generate_track(spec, seed=seed)
    x = np.arange(0., t.length_m, .05)
    slope = t.grade_profile.slope(x)
    assert slope.min() >= 0. and slope.max() <= spec.grade_peak+1e-9
    assert np.all(slope[x < spec.lead_in_m] == 0.)
    assert np.all(slope[x > t.length_m-spec.lead_out_m] == 0.)
    flat = generate_track(replace(spec, grade_peak_prob=0.), seed=seed)
    assert flat.grade_profile.slope(x).max() <= spec.grade_max+1e-9


def test_peak_grade_occurs_for_some_seed():
    spec = replace(TerrainGenSpec(), grade_peak_prob=1.)
    peaks = [generate_track(spec, seed=s).grade_profile.slope(np.arange(0., 60., .1)).max() for s in range(8)]
    assert max(peaks) > spec.grade_max


def test_generated_track_round_trips_through_trackfile(tmp_path):
    t = generate_track(TerrainGenSpec(), seed=9)
    save_track(t, tmp_path/'t.toml')
    back = load_track(tmp_path/'t.toml')
    assert back.length_m == t.length_m and back.surface == t.surface
    assert back.surface_sections == t.surface_sections
    assert np.allclose(_profile(back), _profile(t), atol=1e-6)


def test_description_records_seed_and_spec_hash():
    spec = TerrainGenSpec()
    d = generate_track(spec, seed=5).description
    assert 'seed=5' in d and spec.sha256()[:12] in d
    assert spec.sha256() != replace(spec, grade_max=.2).sha256()


def test_spec_from_dict_accepts_lists_and_rejects_unknown_keys():
    spec = TerrainGenSpec.from_dict({'length_m': [60, 70], 'grade_max': .1})
    assert spec.length_m == (60., 70.) and spec.grade_max == .1
    with pytest.raises(ValueError, match='unknown'):
        TerrainGenSpec.from_dict({'lenght_m': [60, 70]})


@pytest.mark.parametrize('bad', [dict(length_m=(120., 60.)), dict(grade_max=.4), dict(grade_peak_prob=1.5),
                                 dict(bump_height_m=(.1, .02)), dict(base_surfaces=('lava',)),
                                 dict(n_features=(3, 1)), dict(lead_in_m=-1.)])
def test_spec_validation(bad):
    with pytest.raises(ValueError):
        TerrainGenSpec(**bad)


def test_cli_generated_scenario_uses_seed_and_gen_spec(tmp_path):
    from bike_sim.cli.research import parser, build_track
    spec_file = tmp_path/'spec.toml'
    spec_file.write_text('length_m = [60.0, 61.0]\ngrade_peak_prob = 0.0\n')
    args = parser().parse_args(['--scenario', 'generated', '--seed', '11', '--gen-spec', str(spec_file)])
    track = build_track(args)
    assert track.name == 'generated_11' and 60. <= track.length_m <= 61.
    same = build_track(parser().parse_args(['--scenario', 'generated', '--seed', '11', '--gen-spec', str(spec_file)]))
    assert np.array_equal(_profile(track), _profile(same))
    default = build_track(parser().parse_args(['--scenario', 'generated', '--seed', '11']))
    assert default.length_m != track.length_m or default.description != track.description


def test_cli_track_file_beats_generated_scenario(tmp_path):
    from bike_sim.cli.research import parser, build_track
    save_track(generate_track(TerrainGenSpec(), seed=3, name='authored'), tmp_path/'a.toml')
    args = parser().parse_args(['--scenario', 'generated', '--track-file', str(tmp_path/'a.toml')])
    assert build_track(args).name == 'authored'


def test_cli_gen_spec_requires_generated_scenario(tmp_path):
    from bike_sim.cli.research import parser, build_track
    (tmp_path/'s.toml').write_text('grade_max = 0.1\n')
    with pytest.raises(ValueError, match='generated'):
        build_track(parser().parse_args(['--scenario', 'flat', '--gen-spec', str(tmp_path/'s.toml')]))


def _obstacles(spec, seeds=range(60)):
    return [o for s in seeds for o in generate_track(spec, seed=s).obstacles]


def test_default_spec_avoids_vertical_faces_the_tire_model_cannot_support():
    # SquareEdge/Drop faces end runs in model_violation (front:multi_support): opt-in only.
    kinds = {type(o) for o in _obstacles(TerrainGenSpec())}
    assert SquareEdge not in kinds and Drop not in kinds and Bump in kinds and Roots in kinds


def test_edges_and_drops_are_available_by_opting_in():
    spec = replace(TerrainGenSpec(), feature_types=('bump', 'edge', 'drop', 'roots'))
    kinds = {type(o) for o in _obstacles(spec)}
    assert SquareEdge in kinds and Drop in kinds


def test_bump_crest_radius_stays_above_the_wheel_contact_radius():
    from bike_sim.terrain.generator import MIN_CONTACT_RADIUS_M
    for o in _obstacles(TerrainGenSpec()):
        if isinstance(o, Bump):
            curvature = 2.*np.pi**2*o.height_m/o.bump_length_m**2   # peak of a raised cosine
            assert curvature <= 1./MIN_CONTACT_RADIUS_M+1e-9


def test_unknown_feature_type_rejected():
    with pytest.raises(ValueError, match='feature_types'):
        TerrainGenSpec(feature_types=('bump', 'volcano'))
