"""Reproducible longitudinal challenges for torque-control development."""
from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.surface import SurfaceSection
from bike_sim.terrain.obstacles import RoadRoughness, Bump, SquareEdge

RESEARCH_SCENARIOS = ('flat', 'uphill', 'rough_uphill', 'crest', 'low_grip', 'step_up')


def build_research_track(name='rough_uphill', *, seed=0):
    if name not in RESEARCH_SCENARIOS:
        raise ValueError(f'unknown research scenario {name!r}; choose {RESEARCH_SCENARIOS}')
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError('terrain seed must be a nonnegative integer')
    track = TrackSpec('research_'+name, 35., surface='asphalt',
                      description='Synthetic longitudinal torque-control challenge; seed='+str(seed))
    if name in ('uphill', 'rough_uphill', 'low_grip'):
        grade = .22 if name != 'low_grip' else .12
        track.grade_profile = GradeProfile(((0., 0.), (4., 0.), (8., grade),
                                           (24., grade), (30., 0.), (35., 0.)))
    if name == 'rough_uphill':
        track.surface = 'hardpack'
        track.obstacles = [
            RoadRoughness(start_m=4., section_length_m=4., amplitude_m=.015,
                          correlation_length_m=.35, seed=seed),
            Bump(start_m=8.5, height_m=.06, bump_length_m=.7),
            RoadRoughness(start_m=10., section_length_m=5., amplitude_m=.025,
                          correlation_length_m=.45, seed=seed+1),
            SquareEdge(start_m=16., height_m=.04, ledge_length_m=.25),
            RoadRoughness(start_m=18., section_length_m=11., amplitude_m=.018,
                          correlation_length_m=.3, seed=seed+2),
        ]
    elif name == 'crest':
        track.grade_profile = GradeProfile(((0., 0.), (4., 0.), (6., .28), (8., .28),
                                           (10., 0.), (12., -.35), (16., -.35), (20., 0.), (35., 0.)))
        track.obstacles = [RoadRoughness(start_m=5., section_length_m=3., amplitude_m=.008,
                                       correlation_length_m=.4, seed=seed)]
    elif name == 'low_grip':
        track.surface_sections = (SurfaceSection(6., 10., 'wet'), SurfaceSection(14., 21., 'loose'))
        track.obstacles = [RoadRoughness(start_m=4., section_length_m=23., amplitude_m=.008,
                                       correlation_length_m=.4, seed=seed)]
    elif name == 'step_up':
        track.obstacles = [SquareEdge(start_m=5., height_m=.07, ledge_length_m=.5),
                           Bump(start_m=9., height_m=.10, bump_length_m=1.)]
    track.validate()
    return track
