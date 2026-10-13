"""
Road-Profile Assembly for the Pneumatic Tyre.

The tyre's rays are intersected against a 1-D world-frame polyline (`geometry.RoadProfile`)
sampled on the heightfield's longitudinal grid: the track profile raised to the static
ground plane. That is the same profile the rendered heightfield is rasterised from, in the
same coordinates, so the tyre and MuJoCo's collision geometry see the same road.

The arithmetic order here is pinned: the native backend reproduces this construction
bitwise, so the mm-to-m conversion of `compute_ground_z` is added to `build_profile`
exactly as written.
"""

from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.terrain.heightfield import HeightFieldSpec
from bike_sim.terrain.profile import TrackSpec, build_profile


def build_road_profile(track: TrackSpec, field: HeightFieldSpec, specs: BikeSpecs) -> RoadProfile:
    """
    Builds the world-frame road profile the pneumatic tyre rolls on.

    Args:
        track: Track layout the road is assembled from.
        field: Heightfield grid the profile is sampled on; ``field.track_x()`` fixes the
            sample positions in track (= world) X.
        specs: Bicycle geometry; only its flat-ground Z enters, via `compute_ground_z`.

    Returns:
        The road as a `RoadProfile` on the field's uniform grid.
    """
    track_x = field.track_x()
    road_z = (
        compute_ground_z(specs) / 1000.0
        + build_profile(track, track_x)
    )
    return RoadProfile.from_samples(track_x, road_z)


__all__ = ["build_road_profile"]
