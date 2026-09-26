"""
Named Track Presets.

Tracks are defined in code rather than loaded from data files: a track is part of a
reproducible experiment, and a layout that lives in an external file stops being pinned to
the commit that produced a given run. Presets are exposed through a registry of factory
functions so that each caller receives a fresh, independent :class:`TrackSpec`.

Pseudo-random sections carry fixed seeds and are therefore properties of the preset, not
run-to-run noise.

The three ``road_*`` presets are procedurally generated (see :mod:`bike_sim.terrain.road`)
from a fixed seed, so they are just as pinned as the authored ones. A track loaded from a
TOML file (:mod:`bike_sim.terrain.trackfile`) is the user's own experiment and is not
registered here.
"""

from typing import Callable, Dict, List

from bike_sim.terrain.obstacles import (
    Drop,
    GOut,
    Kicker,
    Pothole,
    RockGarden,
    Roots,
    SquareEdge,
    Washboard,
)
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.road import road_broken, road_smooth, road_worn


def enduro_aggressive() -> TrackSpec:
    """
    Builds the default aggressive enduro track.

    Ordering is deliberate: periodic input first, then isolated impacts, then the
    energy-absorbing events (G-out, drop, kicker), then periodic input again on a
    suspension that is already working. The landing is placed late so that a bottom-out
    does not contaminate the sections measured before it.
    """
    return TrackSpec(
        name="enduro_aggressive",
        length_m=115.0,
        description="Braking bumps, square edges, rock gardens, pothole, G-out, drop and kicker.",
        obstacles=[
            # 12.0 - 20.1 m: braking bumps into the first corner
            Washboard(start_m=12.0, amplitude_m=0.035, wavelength_m=0.900, n_waves=9),
            # 22.0 m: isolated square-edge hit, the classic high-speed-compression test
            SquareEdge(start_m=22.0, height_m=0.090, ledge_length_m=0.250),
            # 26.0 - 34.0 m: rock garden
            RockGarden(
                start_m=26.0,
                section_length_m=8.0,
                amplitude_m=0.060,
                correlation_length_m=0.250,
                seed=1701,
            ),
            # 38.0 m: hole the front wheel drops into without reaching the bottom
            Pothole(start_m=38.0, depth_m=0.180, hole_length_m=0.600),
            # 42.0 - 47.0 m: G-out, loads the suspension without an impact
            GOut(start_m=42.0, depth_m=0.350, dip_length_m=5.0),
            # 52.0 m: step down
            Drop(start_m=52.0, height_m=0.600, ramp_m=0.0),
            # 57.0 - 65.9 m: kicker, gap and descending landing.
            # The landing runs 5 m rather than the 3 m the flight alone needs: at 3 m the
            # jump only works between 20.9 and 26.9 km/h, so a 20/25/30 speed sweep would
            # case at one end and overshoot at the other and the section would not be
            # comparable across the sweep. 5 m covers 20.9-30.6 km/h and leaves the
            # touchdown at the default speed exactly where it was.
            Kicker(
                start_m=57.0,
                height_m=0.350,
                ramp_m=1.400,
                gap_m=2.500,
                landing_angle_deg=20.0,
                landing_m=5.000,
            ),
            # 69.0 - 76.0 m: root section, held 3.1 m clear of the landing runout so the
            # suspension is not still recovering from touchdown when it arrives
            Roots(
                start_m=69.0,
                n_bumps=7,
                height_m=0.080,
                width_m=0.250,
                section_length_m=7.0,
                seed=2029,
            ),
            # 80.0 / 80.8 m: double edge, a compression straight into a second hit
            SquareEdge(start_m=80.0, height_m=0.070, ledge_length_m=0.250),
            SquareEdge(start_m=80.8, height_m=0.090, ledge_length_m=0.250),
            # 84.0 - 95.0 m: rougher rock garden
            RockGarden(
                start_m=84.0,
                section_length_m=11.0,
                amplitude_m=0.090,
                correlation_length_m=0.200,
                seed=3301,
            ),
        ],
    )


def flat() -> TrackSpec:
    """
    Builds a featureless track.

    Used to verify static sag, rolling resistance and coast-down without any terrain
    input, and as the control case against which a rough run is compared.
    """
    return TrackSpec(
        name="flat",
        length_m=112.0,
        description="No obstacles; sag, rolling resistance and coast-down reference.",
        obstacles=[],
    )


def single_edge() -> TrackSpec:
    """
    Builds a short track with one square edge.

    The debugging track for the suspension force model and the controllers: a single,
    fully characterized impact, reached after enough run-up to be at target speed.
    """
    return TrackSpec(
        name="single_edge",
        length_m=40.0,
        description="Run-up and one 90 mm square edge at 20 m.",
        obstacles=[SquareEdge(start_m=20.0, height_m=0.090, ledge_length_m=0.250)],
    )


def washboard_only() -> TrackSpec:
    """
    Builds a track containing only sustained periodic input.

    Isolates suspension pumping and rebound recovery from impact response.
    """
    return TrackSpec(
        name="washboard_only",
        length_m=60.0,
        description="Run-up and 20 waves of 35 mm braking bumps.",
        obstacles=[Washboard(start_m=15.0, amplitude_m=0.035, wavelength_m=0.900, n_waves=20)],
    )


PRESETS: Dict[str, Callable[[], TrackSpec]] = {
    "enduro_aggressive": enduro_aggressive,
    "flat": flat,
    "single_edge": single_edge,
    "washboard_only": washboard_only,
    "road_smooth": road_smooth,
    "road_worn": road_worn,
    "road_broken": road_broken,
}

DEFAULT_PRESET = "enduro_aggressive"
DEFAULT_ROAD_PRESET = "road_worn"


def available_presets() -> List[str]:
    """Returns the sorted names of every registered track preset."""
    return sorted(PRESETS)


def get_preset(name: str) -> TrackSpec:
    """
    Builds a track preset by name.

    Args:
        name: Registered preset name.

    Returns:
        A freshly constructed, validated TrackSpec.

    Raises:
        KeyError: If no preset is registered under that name.
    """
    if name not in PRESETS:
        raise KeyError(f"unknown track preset '{name}'; available: {', '.join(available_presets())}")
    track = PRESETS[name]()
    track.validate()
    return track


__all__ = [
    "PRESETS",
    "DEFAULT_PRESET",
    "DEFAULT_ROAD_PRESET",
    "available_presets",
    "get_preset",
    "enduro_aggressive",
    "flat",
    "single_edge",
    "washboard_only",
]
