# tests/reference/test_distributed_tire_reference.py
"""The portable distributed tyre reference: pinned behaviour, scalar formulas, state wire."""
import numpy as np
import pytest

from bike_sim.physics.distributed_tire import HingeDensity, density_response, normal_station_response
from bike_sim.validation.contact_manifold_rigs import distributed_road_trace, wheel_stand

# Captured from the vectorised implementation before the portable rewrite (Task 1).
BEFORE_NORMAL = [632.674608364, 609.382103444, 613.516398494, 614.034816138, 613.869405735, 613.817389269, 613.801320645, 613.703552998, 613.341342092, 626.68744485]
BEFORE_VERTICAL = [632.674608364, 609.382103444, 613.516398494, 614.034816138, 613.869405735, 613.817389269, 613.801320645, 613.703552998, 613.341342092, 626.68744485]
BEFORE_PATCHES = [9, 8, 8, 8, 8, 8, 8, 8, 9, 9]
BEFORE_MAX_PATCHES = 9


def test_step_rig_matches_the_pinned_trace_to_reordering_tolerance():
    rows, m = distributed_road_trace(.00125, 128, .005, case='step', duration_s=.1)
    got_n = [r['normal_load_n'] for r in rows[::8]]
    got_v = [r['vertical_force_n'] for r in rows[::8]]
    assert [r['patch_count'] for r in rows[::8]] == BEFORE_PATCHES
    np.testing.assert_allclose(got_n, BEFORE_NORMAL, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(got_v, BEFORE_VERTICAL, rtol=1e-9, atol=1e-9)
    assert m['max_patch_count'] == BEFORE_MAX_PATCHES >= 2
    assert m['energy_residual_ratio'] <= .05
    assert m['native_wheel_contact_count'] == 0
