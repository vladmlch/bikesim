"""Bitwise oracle tests for the native pneumatic tyre geometry kernel.

Covers the Task-7 debug surfaces in ``bike_native``:

* ``_debug_pneumatic_ring``     — ``geometry.RayRing``
* ``_debug_pneumatic_intersect``— ``geometry.RoadProfile`` + ``geometry.intersect``
* ``_debug_pneumatic_patches``  — ``geometry.patches``

Every floating-point array is compared raw-byte-for-raw-byte against the
pure-Python oracle in ``src/bike_sim/sim/ride/tyre/geometry.py`` (NumPy
linspace/sin/cos/atan2/interp semantics).  ``tobytes()`` comparison keeps
signed zeros and NaN payloads distinguishable — ``np.array_equal`` would
hide both.

The corpus is constructed so every oracle branch in ``intersect`` is
taken at least once:

* ``early_window``    — ``index_range`` yields ``hi - lo < 2`` (wheel
  off either profile end, or ``radius == 0``).
* ``airborne``        — window present but no profile sample inside the
  disc (``inside.any()`` false).
* ``fast``            — monotone ``phi`` window, ``searchsorted`` row.
* ``brute``           — non-monotone ``phi`` window, O(n·m) row.
* coverage            — both ``|phi| > half_angle + 1e-9`` and
  ``pz >= cz`` causes, on fast and brute paths.

``_oracle_branch`` replicates the oracle's own window/phi computation so
each corpus entry can pin the branch it is meant to exercise; the real
verification is always the bitwise row comparison.
"""

import numpy as np
import pytest

from bike_sim.sim.ride.tyre import geometry

from native_loader import load_native
bike_native = load_native()
from _bits import assert_bitwise_equal
import _golden

RAD = np.pi / 180.0
HALF_DEG = 75.0
HALF = HALF_DEG * RAD
RADIUS = 0.33
RAYS = (64, 256)


def _road(x0, dx, z):
    return geometry.RoadProfile(x0_m=x0, dx_m=dx, z_m=np.asarray(z, float))


def _ring(n):
    return geometry.RayRing(n, HALF)


def _oracle_branch(road, cx, cz, radius, half=HALF):
    """Classify the intersect() path — mirrors geometry.py's control flow.

    Returns one of 'early_window', 'airborne', ('fast', cov), ('brute', cov).
    """
    lo, hi = road.index_range(cx - radius, cx + radius)
    if hi - lo < 2:
        return 'early_window'
    zs = road.z_m[lo:hi]
    xs = road.x0_m + road.dx_m * np.arange(lo, hi)
    dxs = xs - cx
    dzs = cz - zs
    inside = dxs * dxs + dzs * dzs < radius * radius
    if not inside.any():
        return 'airborne'
    idx = np.flatnonzero(inside)
    a = max(int(idx[0]) - 1, 0)
    b = min(int(idx[-1]) + 2, xs.size)
    px, pz = xs[a:b], zs[a:b]
    phi = np.arctan2(px - cx, cz - pz)
    cov = bool(
        np.any(np.abs(phi[inside[a:b]]) > half + 1e-9)
        or np.any(pz[inside[a:b]] >= cz)
    )
    if px.size >= 2 and np.all(np.diff(phi) > 0.0):
        return ('fast', cov)
    return ('brute', cov)


def _profile_dict(road):
    return {'x0_m': road.x0_m, 'dx_m': road.dx_m, 'z_m': road.z_m}


# --------------------------------------------------------------------
# Corpus
# --------------------------------------------------------------------

def _cases():
    """Return (id, road, cx, cz, radius, expected_branch) rows."""
    x = np.arange(1001) * 0.01
    x2 = np.arange(1601) * 0.005
    flat = _road(0.0, 0.01, np.zeros(1001))
    cases = [
        # --- monotone fast path, no coverage --------------------------
        ('flat-loaded', flat, 5.0, RADIUS - 0.006, RADIUS, ('fast', False)),
        ('flat-grazing', flat, 5.0, RADIUS - 1e-9, RADIUS, ('fast', False)),
        # --- fast path + coverage via pz >= cz ------------------------
        ('flat-sunk', flat, 5.0, 0.05, RADIUS, ('fast', True)),
        # --- airborne -------------------------------------------------
        ('flat-airborne', flat, 5.0, RADIUS + 0.5, RADIUS, 'airborne'),
        # --- hi - lo < 2 early-out (both ends + zero radius) ----------
        ('off-left', flat, -50.0, RADIUS - 0.01, RADIUS, 'early_window'),
        ('off-right', flat, 50.0, RADIUS - 0.01, RADIUS, 'early_window'),
        ('zero-radius', flat, 5.0, 0.0, 0.0, 'early_window'),
        # --- ramp (sloped road through the wheel) ---------------------
        ('ramp', _road(0.0, 0.01, 0.03 * x - 0.15), 5.0, 0.19, RADIUS,
         ('fast', False)),
        # --- step down just ahead of the hub --------------------------
        ('step-down', _road(0.0, 0.01, np.where(x < 5.02, 0.0, -0.12)),
         5.0, 0.30, RADIUS, ('fast', False)),
        # --- step-up cliff inside the disc: angle coverage ------------
        ('cliff', _road(0.0, 0.01, np.where(x < 5.32, 0.0, 0.27)),
         5.0, 0.32, RADIUS, ('fast', True)),
        # --- non-monotone phi -> brute-force --------------------------
        ('notch-left', _road(0.0, 0.01, np.where(x < 5.0, 0.0, -0.12)),
         4.95, 0.30, RADIUS, ('brute', False)),
        # sharp spike whose apex rises above the hub: non-monotone AND
        # coverage (interior pz >= cz and |phi| > half angle).
        ('spike-above-hub', _road(
            0.0, 0.01, np.maximum(0.0, 0.36 - np.abs(x - 5.02) * 12.0)),
         5.0, 0.30, RADIUS, ('brute', True)),
        # spike apex below the hub: non-monotone, no coverage.
        ('spike-below-hub', _road(
            0.0, 0.01, np.maximum(0.0, 0.27 - np.abs(x - 5.02) * 8.0)),
         5.0, 0.31, RADIUS, ('brute', False)),
        # sunk hub on flat road: every inside point has pz >= cz, and the
        # window wraps through +-pi -> non-monotone brute path.
        ('sunk-hub', flat, 5.0, -0.05, RADIUS, ('brute', True)),
        # steep ramp left of hub parallel to ray ~theta_19 (n=64) —
        # exercises the near-parallel denominator row of the brute mask.
        ('parallel-ramp', _road(
            0.0, 0.005,
            np.where(x2 < 4.5, 1.74879359546275 * (4.5 - 4.95) + 0.30,
                     np.where(x2 < 4.95,
                              1.74879359546275 * (x2 - 4.95) + 0.30, 0.0))),
         5.0, 0.30, RADIUS, ('brute', True)),
        # wedge bump just inside +half angle: patch reaches ray n-1.
        ('edge-front', _road(
            0.0, 0.01, np.maximum(0.0, 0.235 - np.abs(x - 5.30) * 6.0)),
         5.0, 0.30, RADIUS, None),
        # wedge bump just inside -half angle: patch starts at ray 0.
        ('edge-rear', _road(
            0.0, 0.01, np.maximum(0.0, 0.235 - np.abs(x - 4.70) * 6.0)),
         5.0, 0.30, RADIUS, None),
    ]
    return cases


CORPUS = _cases()


# --------------------------------------------------------------------
# RayRing
# --------------------------------------------------------------------

@pytest.mark.parametrize('n_rays', [3, 63, 64, 256])
def test_pneumatic_ring_bitwise(n_rays):
    want = _ring(n_rays)
    got = bike_native._debug_pneumatic_ring(n_rays, HALF_DEG)
    assert_bitwise_equal(np.asarray(got['theta_rad']), want.theta_rad)
    assert_bitwise_equal(np.asarray(got['ux']), want.ux)
    assert_bitwise_equal(np.asarray(got['uz']), want.uz)
    assert_bitwise_equal(got['dtheta_rad'], want.dtheta_rad)


def test_pneumatic_ring_narrow_angle():
    want = geometry.RayRing(64, 30.0 * RAD)
    got = bike_native._debug_pneumatic_ring(64, 30.0)
    assert_bitwise_equal(np.asarray(got['theta_rad']), want.theta_rad)
    assert_bitwise_equal(np.asarray(got['ux']), want.ux)
    assert_bitwise_equal(np.asarray(got['uz']), want.uz)


# --------------------------------------------------------------------
# intersect
# --------------------------------------------------------------------

@pytest.mark.parametrize(
    'road, cx, cz, radius, expected',
    [pytest.param(c[1], c[2], c[3], c[4], c[5], id=c[0]) for c in CORPUS],
)
@pytest.mark.parametrize('n_rays', RAYS)
def test_pneumatic_intersect_bitwise(road, cx, cz, radius, expected, n_rays):
    ring = _ring(n_rays)
    if expected is not None:
        assert _oracle_branch(road, cx, cz, radius) == expected
    want = geometry.intersect(ring, cx, cz, radius, road)
    got = bike_native._debug_pneumatic_intersect(
        _profile_dict(road), cx, cz, radius, n_rays, HALF_DEG)
    for name in ('r_m', 'delta_m', 'road_x_m', 'road_z_m'):
        assert_bitwise_equal(np.asarray(got[name]), getattr(want, name))
    _golden.record(np.asarray(want.coverage_event), 'coverage_event')
    assert got['coverage_event'] == bool(want.coverage_event)
    _golden.record(np.asarray(want.airborne), 'airborne')
    assert got['airborne'] == bool(want.airborne)


def test_pneumatic_intersect_coarse_and_fine_dx():
    """Different sampling densities hit different index_range windows."""
    for dx in (0.05, 0.002, 0.017):
        x = np.arange(int(10.0 / dx) + 1) * dx
        z = 0.02 * np.sin(x * 3.1) + np.maximum(0.0, 0.05 - np.abs(x - 5.0) * 4.0)
        road = _road(0.0, dx, z)
        for n in RAYS:
            ring = _ring(n)
            want = geometry.intersect(ring, 5.0, 0.28, RADIUS, road)
            got = bike_native._debug_pneumatic_intersect(
                _profile_dict(road), 5.0, 0.28, RADIUS, n, HALF_DEG)
            for name in ('r_m', 'delta_m', 'road_x_m', 'road_z_m'):
                assert_bitwise_equal(np.asarray(got[name]), getattr(want, name))
            _golden.record(np.asarray(want.coverage_event),
                           'coverage_event')
            assert got['coverage_event'] == bool(want.coverage_event)
            _golden.record(np.asarray(want.airborne), 'airborne')
            assert got['airborne'] == bool(want.airborne)


def test_pneumatic_intersect_hub_sweep():
    """Sweep the hub across a bumpy road — mixes all monotone paths."""
    x = np.arange(1201) * 0.01
    z = (0.015 * np.sin(x * 2.7)
         + np.maximum(0.0, 0.05 - np.abs(x - 4.80) * 5.0)
         + np.maximum(0.0, 0.04 - np.abs(x - 5.30) * 7.0))
    road = _road(0.0, 0.01, z)
    for cx in np.linspace(4.55, 5.45, 10):
        for cz in (0.26, 0.30, 0.34):
            for n in RAYS:
                ring = _ring(n)
                want = geometry.intersect(ring, float(cx), cz, RADIUS, road)
                got = bike_native._debug_pneumatic_intersect(
                    _profile_dict(road), float(cx), cz, RADIUS, n, HALF_DEG)
                for name in ('r_m', 'delta_m', 'road_x_m', 'road_z_m'):
                    assert_bitwise_equal(np.asarray(got[name]),
                                      getattr(want, name))
                _golden.record(np.asarray(want.coverage_event),
                               'coverage_event')
                assert got['coverage_event'] == bool(want.coverage_event)
                _golden.record(np.asarray(want.airborne), 'airborne')
                assert got['airborne'] == bool(want.airborne)


def test_pneumatic_intersect_edge_patches():
    """The wedge cases must load the boundary rays themselves."""
    x = np.arange(1001) * 0.01
    for n in RAYS:
        ring = _ring(n)
        front = _road(0.0, 0.01, np.maximum(0.0, 0.235 - np.abs(x - 5.30) * 6.0))
        hits = geometry.intersect(ring, 5.0, 0.30, RADIUS, front)
        patches = geometry.patches(hits.delta_m)
        assert patches[-1][1] == n          # rear-most +angle ray n-1 loaded
        rear = _road(0.0, 0.01, np.maximum(0.0, 0.235 - np.abs(x - 4.70) * 6.0))
        hits = geometry.intersect(ring, 5.0, 0.30, RADIUS, rear)
        patches = geometry.patches(hits.delta_m)
        assert patches[0][0] == 0           # ray 0 loaded


# --------------------------------------------------------------------
# patches
# --------------------------------------------------------------------

def _assert_patches(delta):
    want = geometry.patches(np.asarray(delta, float))
    got = np.asarray(bike_native._debug_pneumatic_patches(
        np.asarray(delta, float)))
    assert got.dtype == np.int32
    assert got.shape == (len(want), 2)
    _golden.record(np.asarray(want, dtype=np.int32).reshape(-1, 2),
                   'patches')
    assert got.tolist() == [[int(s), int(e)] for s, e in want]


def test_pneumatic_patches_empty():
    _assert_patches(np.zeros(64))
    _assert_patches(np.zeros(0))
    _assert_patches(np.array([-1e-3, -0.2]))


def test_pneumatic_patches_single_run():
    d = np.zeros(64)
    d[10:20] = 0.01
    _assert_patches(d)


def test_pneumatic_patches_multi_run():
    d = np.zeros(64)
    d[2:5] = 1e-4
    d[10] = 0.3
    d[11] = 0.1
    d[40:64:2] = 0.02
    _assert_patches(d)


def test_pneumatic_patches_boundaries():
    d = np.zeros(64)
    d[0] = 1.0          # run touching index 0
    d[62:] = 0.5        # run touching index n-1
    _assert_patches(d)
    _assert_patches(np.full(64, 0.1))   # single full-width run
    d = np.zeros(64)
    d[0] = 0.1
    _assert_patches(d)


def test_pneumatic_patches_tiny_positive():
    # The smallest representable positive must still count as loaded.
    d = np.zeros(8)
    d[3] = np.finfo(np.float64).tiny
    _assert_patches(d)
    d[5] = 1.0 / np.finfo(np.float64).max
    _assert_patches(d)


def test_pneumatic_patches_nonfinite():
    # Comparison semantics: +inf is loaded, nan and -inf are not —
    # whatever the oracle decides, the native side must reproduce.
    _assert_patches(np.array([np.inf, np.nan, 0.1, -np.inf, np.inf]))


def test_pneumatic_patches_from_intersect():
    """End-to-end: delta rows from both tiers split into the same runs."""
    x = np.arange(1001) * 0.01
    front = _road(0.0, 0.01, np.maximum(0.0, 0.235 - np.abs(x - 5.30) * 6.0))
    for n in RAYS:
        ring = _ring(n)
        hits = geometry.intersect(ring, 5.0, 0.30, RADIUS, front)
        _assert_patches(hits.delta_m)


# --------------------------------------------------------------------
# RoadProfile: index_range / height_at exercised through intersect, but
# pin the helper semantics directly as well.
# --------------------------------------------------------------------

def test_road_profile_helpers_match_oracle_contract():
    """Document the oracle contract the native RoadProfile is bound to:
    x_m is the endpoint-pinned x0 + dx*k row and height_at is np.interp
    over the FULL profile (end-clamped), not over a window."""
    road = _road(1.5, 0.007, np.sin(np.arange(500) * 0.013))
    full_x = road.x0_m + road.dx_m * np.arange(road.z_m.size)
    _golden.record(np.asarray(full_x), 'x_m')
    assert road.x_m.tobytes() == full_x.tobytes()
    for q in (-5.0, 0.0, 1.499, 1.5, 3.75, 4.995, 20.0):
        _golden.record(np.float64(np.interp(q, full_x, road.z_m)),
                       f'height_at({q!r})')
        assert np.float64(road.height_at(q)).tobytes() == np.float64(
            np.interp(q, full_x, road.z_m)).tobytes()
