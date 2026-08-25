"""
Road Profile & Heightfield Package.

Pure geometry for the ride-mode road surface: an obstacle catalogue, profile assembly,
named track presets, and rasterization onto a fixed heightfield grid. Nothing in this
package imports MuJoCo or depends on the physics model, so the whole of it is testable
without compiling a simulation.
"""

from bike_sim.terrain.obstacles import (
    Drop,
    GOut,
    Kicker,
    Obstacle,
    Pothole,
    RockGarden,
    Roots,
    SquareEdge,
    Washboard,
)
from bike_sim.terrain.profile import TrackSpec, build_profile, profile_extent
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
