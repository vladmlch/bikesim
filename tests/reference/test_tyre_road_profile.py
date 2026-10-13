# tests/reference/test_tyre_road_profile.py
"""The shared road-profile helper and the signed patch-slip telemetry of the pneumatic tyre."""
import numpy as np
import pytest

from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.tyre import FRONT_TYRE, TIERS
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.sim.ride.tyre.model import PneumaticTyre
from bike_sim.sim.ride.tyre.profile import build_road_profile
from bike_sim.terrain import DEFAULT_PRESET, HeightFieldSpec, get_preset
from bike_sim.terrain.profile import build_profile
from bike_sim.terrain.surface import SurfaceMap


def test_build_road_profile_matches_the_former_inline_construction_bitwise():
    track = get_preset(DEFAULT_PRESET)
    field = HeightFieldSpec.for_track(track)
    specs = BikeSpecs()

    # The statements `RideSimulation` ran inline before the helper was extracted; the
    # native port is bitwise-locked to this path, so the helper must reproduce it exactly.
    track_x = field.track_x()
    road_z = (
        compute_ground_z(specs) / 1000.0
        + build_profile(track, track_x)
    )
    expected = RoadProfile.from_samples(track_x, road_z)

    profile = build_road_profile(track, field, specs)
    assert profile.x0_m == expected.x0_m and profile.dx_m == expected.dx_m
    assert np.array_equal(profile.z_m, expected.z_m)


def _loaded_outputs(tyre, v_x_mps, omega_radps):
    """Evaluates the tyre on a flat road, loaded by a 6 mm hub drop."""
    radius_m = tyre.tyre.outer_radius_mm / 1000.0
    x_m = np.linspace(0.0, 20.0, 4001)
    road = RoadProfile.from_samples(x_m, np.zeros_like(x_m))
    return tyre.evaluate(
        np.array([5.0, 0.0, radius_m - 0.006]),
        np.array([v_x_mps, 0.0, 0.0]),
        omega_radps,
        road,
        SurfaceMap.uniform("asphalt"),
        dt_s=tyre.tier.timestep_s,
    )


@pytest.mark.parametrize("tier", [TIERS["fast"], TIERS["detailed"]], ids=lambda t: t.name)
@pytest.mark.parametrize(
    "v_x_mps, omega_radps, sign",
    ((5.0, 0.0, 1.0), (2.0, 20.0, -1.0)),
    ids=("forward-slide", "spinning-wheel"),
)
def test_patch_slip_speed_mps_is_the_signed_tread_slip(tier, v_x_mps, omega_radps, sign):
    # v_s = v_x - omega * R_eff: positive slides the tread forward over the road
    # (locked/slow wheel), negative spins the wheel faster than the travel speed.
    outputs = _loaded_outputs(PneumaticTyre(FRONT_TYRE, tier), v_x_mps, omega_radps)
    assert not outputs.airborne and outputs.patches
    velocity = np.array([v_x_mps, 0.0, 0.0])
    for patch in outputs.patches:
        v_x = float(np.dot(velocity, patch.tangent_world))
        effective_radius_m = max(
            FRONT_TYRE.outer_radius_mm / 1000.0 - patch.mean_deflection_m / 3.0,
            1e-6,
        )
        expected_v_s = v_x - float(omega_radps) * effective_radius_m
        assert patch.slip_speed_mps == expected_v_s
        assert patch.slip_speed_mps * sign > 0.0
        assert patch.sliding_speed_mps == abs(patch.slip_speed_mps)
