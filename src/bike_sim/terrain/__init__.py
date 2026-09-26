"""
Road Profile & Heightfield Package.

Pure geometry for the ride-mode road surface: an obstacle catalogue, profile assembly,
named track presets, and rasterization onto a fixed heightfield grid. Nothing in this
package imports MuJoCo or depends on the physics model, so the whole of it is testable
without compiling a simulation.
"""

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
    TrapezoidBump,
    Washboard,
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
    PRESETS,
    available_presets,
    get_preset,
)
from bike_sim.terrain.heightfield import (
    FIELD,
    HeightFieldSpec,
    assert_track_fits,
    build_field_data,
)

__all__ = [
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
    "POTHOLE_TYPES",
    "BUMP_TYPES",
    "wheel_centre_path",
    "effective_drop_m",
    "effective_rise_m",
    "bridged_drop_m",
    "TrackSpec",
    "build_profile",
    "profile_extent",
    "PRESETS",
    "DEFAULT_PRESET",
    "available_presets",
    "get_preset",
    "HeightFieldSpec",
    "FIELD",
    "build_field_data",
    "assert_track_fits",
]
