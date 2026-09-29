"""
Road Profile & Heightfield Package.

Pure geometry for the ride-mode road surface: an obstacle catalogue, profile assembly,
named track presets, and rasterization onto a fixed heightfield grid. Nothing in this
package imports MuJoCo or depends on the physics model, so the whole of it is testable
without compiling a simulation.
"""

from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.surface import SurfaceSection
from bike_sim.terrain.obstacles import (
    BUMP_TYPES,
    POTHOLE_TYPES,
    BowlPothole,
    Bump,
    Drop,
    GOut,
    Kicker,
    Obstacle,
    Pothole,
    RoadRoughness,
    RockGarden,
    Roots,
    SlopedPothole,
    SquareEdge,
    SteppedClimb,
    TrapezoidBump,
    Washboard,
)
from bike_sim.terrain.surface import (
    DEFAULT_SURFACE,
    ROAD_SURFACE,
    SURFACES,
    TRAIL_SURFACE,
    SurfaceMap,
    SurfaceSpec,
    available_surfaces,
    get_surface,
)
from bike_sim.terrain.profile import TrackSpec, build_profile, profile_extent
from bike_sim.terrain.wheelpath import (
    bridged_drop_m,
    effective_drop_m,
    effective_rise_m,
    wheel_centre_path,
)
from bike_sim.terrain.presets import (
    DEFAULT_PRESET,
    DEFAULT_ROAD_PRESET,
    PRESETS,
    available_presets,
    climb_steps,
    get_preset,
)
from bike_sim.terrain.road import (
    ROAD_LEVEL_SPECS,
    RoadGeneratorSpec,
    build_road,
    generate_road,
)
from bike_sim.terrain.trackfile import (
    TrackFileError,
    dump_track,
    load_track,
    save_track,
    track_from_dict,
    track_to_dict,
)
from bike_sim.terrain.heightfield import (
    FIELD,
    HeightFieldSpec,
    assert_track_fits,
    build_field_data,
)

__all__ = [
    "GradeProfile", "SurfaceSection",
    "Obstacle",
    "SquareEdge",
    "Pothole",
    "Washboard",
    "GOut",
    "Drop",
    "Kicker",
    "Roots",
    "RockGarden",
    "Bump",
    "TrapezoidBump",
    "SlopedPothole",
    "BowlPothole",
    "RoadRoughness",
    "SteppedClimb",
    "POTHOLE_TYPES",
    "BUMP_TYPES",
    "wheel_centre_path",
    "effective_drop_m",
    "effective_rise_m",
    "bridged_drop_m",
    "SurfaceSpec",
    "SurfaceMap",
    "SURFACES",
    "DEFAULT_SURFACE",
    "ROAD_SURFACE",
    "TRAIL_SURFACE",
    "available_surfaces",
    "get_surface",
    "TrackSpec",
    "build_profile",
    "profile_extent",
    "PRESETS",
    "DEFAULT_PRESET",
    "DEFAULT_ROAD_PRESET",
    "available_presets",
    "get_preset",
    "climb_steps",
    "RoadGeneratorSpec",
    "ROAD_LEVEL_SPECS",
    "generate_road",
    "build_road",
    "TrackFileError",
    "load_track",
    "dump_track",
    "save_track",
    "track_from_dict",
    "track_to_dict",
    "HeightFieldSpec",
    "FIELD",
    "build_field_data",
    "assert_track_fits",
]
