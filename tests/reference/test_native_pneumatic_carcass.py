"""Bitwise oracle tests for the native pneumatic tyre carcass kernel.

Covers the Task-8 debug surfaces in ``bike_native``:

* ``_debug_pneumatic_carcass``           — ``carcass.evaluate_carcass`` +
  ``CarcassState`` state transitions
* ``_debug_pneumatic_advance_unloaded``  — ``CarcassState.advance_unloaded``
* ``_debug_pneumatic_material_rate``     — ``material_deflection_rate_mps``
* ``_debug_pneumatic_advect``            — ``_advect_material``
* ``_debug_pneumatic_math``              — the ``np.tanh`` / ``np.hypot``
  lowerings the kernel uses

Every floating-point array and scalar is compared raw-byte-for-raw-byte
against the pure-Python oracle in
``src/bike_sim/sim/ride/tyre/carcass.py`` across identical multi-step
sequences — ``tobytes()`` keeps signed zeros and NaN payloads
distinguishable.  The state dict carries every ``CarcassState`` field
(three history rows, ``initialized`` and the five rim-event slots), so
the post-call state is pinned bitwise alongside each step's result.

The corpus exercises every oracle branch: the ``initialized`` first-step
path, both upwind directions of ``material_deflection_rate_mps``, the
``omega == 0`` advect copy, fractional/out-of-range ``np.interp`` source
bins, the single-ray contact-length cell, ``np.maximum`` saturation at
zero, the rim-strike FSM start/track/release cycle, ``advance_unloaded``'s
early return, validation and ``np.any`` gate, and the four-level
invalid-input rejection order with failure atomicity.
"""

import numpy as np
import pytest

from bike_sim.physics.tyre import FRONT_TYRE, REAR_TYRE, TyreSpecs
from bike_sim.sim.ride.tyre import carcass, geometry

from native_loader import load_native
bike_native = load_native()
from _bits import assert_bitwise_equal
import _golden

RAD = np.pi / 180.0
HALF_DEG = 75.0
HALF = HALF_DEG * RAD
RAYS = (64, 256)

_SPEC_KEYS = (
    'name', 'outer_radius_mm', 'rim_radius_mm', 'section_height_mm',
    'pressure_bar', 'crr_reference', 'casing_width_mm',
    'compressed_casing_mm', 'area_factor', 'carcass_stiffness_n_mm2',
    'contact_length_factor', 'rate_stiffening', 'rate_relaxation_s',
    'loss_factor', 'hysteresis_rate_eps_mps', 'relaxation_length_mm',
    'rim_stiffness_n_mm2', 'tread_loss_crr',
)

_STATE_ARRAY_KEYS = ('delta_prev_m', 'elastic_prev_n', 'maxwell_force_n')
_STATE_BOOL_KEYS = ('initialized', 'rim_strike_active')
_STATE_FLOAT_KEYS = (
    'rim_event_x_m', 'rim_event_speed_mps', 'rim_event_peak_load_n',
    'rim_event_peak_force_n', 'rim_event_energy_j',
)
_RESULT_ARRAY_KEYS = (
    'pressure_force_n', 'elastic_force_n', 'maxwell_force_n',
    'hysteresis_force_n', 'rim_force_n', 'total_element_force_n',
    'material_rate_mps',
)
_RESULT_FLOAT_KEYS = ('force_x_n', 'force_z_n', 'normal_load_n',
                      'rim_energy_j')
_EVENT_KEYS = ('x_m', 'speed_mps', 'peak_load_n', 'peak_rim_force_n',
               'absorbed_energy_j')


def _spec_dict(tyre):
    """The 18-field TyreSpecs wire dict."""
    return {k: getattr(tyre, k) for k in _SPEC_KEYS}


def _hits_dict(hits):
    """A RayHits object as the six-field wire dict (plain float64 rows)."""
    return {
        'r_m': np.asarray(hits.r_m, dtype=float),
        'delta_m': np.asarray(hits.delta_m, dtype=float),
        'road_x_m': np.asarray(hits.road_x_m, dtype=float),
        'road_z_m': np.asarray(hits.road_z_m, dtype=float),
        'coverage_event': bool(hits.coverage_event),
        'airborne': bool(hits.airborne),
    }


def _synth_hits(n, delta, rng=None, road_x=None, road_z=None):
    """A synthetic hits dict — the kernel consumes delta/road rows
    directly, so manufactured rows are valid boundary inputs."""
    delta = np.asarray(delta, dtype=float)
    assert delta.shape == (n,)
    if rng is None:
        rng = np.random.default_rng(0x5eed)
    return {
        'r_m': np.full(n, np.inf),
        'delta_m': delta.copy(),
        'road_x_m': np.asarray(
            road_x if road_x is not None else
            rng.uniform(4.0, 6.0, n) + np.arange(n) * 0.001, dtype=float),
        'road_z_m': np.asarray(
            road_z if road_z is not None else rng.uniform(-0.4, 0.4, n),
            dtype=float),
        'coverage_event': False,
        'airborne': bool(np.all(delta <= 0.0)),
    }


def _oracle_state_dict(state):
    """Every CarcassState field (minus n_rays) as a wire dict."""
    d = {k: np.asarray(getattr(state, k), dtype=float)
         for k in _STATE_ARRAY_KEYS}
    d.update({k: bool(getattr(state, k)) for k in _STATE_BOOL_KEYS})
    d.update({k: float(getattr(state, k)) for k in _STATE_FLOAT_KEYS})
    return d


def _fresh_state_dict(n):
    return _oracle_state_dict(carcass.CarcassState(n))


def _assert_state_bitwise(native_state, oracle_state, msg=''):
    """Post-call state must be byte-identical: rows, flags, event slots."""
    for k in _STATE_ARRAY_KEYS:
        assert_bitwise_equal(np.asarray(native_state[k]),
                             getattr(oracle_state, k), f'{msg} state.{k}')
    for k in _STATE_BOOL_KEYS:
        _golden.record(np.asarray(bool(getattr(oracle_state, k))),
                       f'{msg} state.{k}')
        assert bool(native_state[k]) == bool(getattr(oracle_state, k)), \
            f'{msg} state.{k}'
    for k in _STATE_FLOAT_KEYS:
        assert_bitwise_equal(float(native_state[k]),
                             float(getattr(oracle_state, k)),
                             f'{msg} state.{k}')


def _assert_result_bitwise(got, want, msg=''):
    for k in _RESULT_ARRAY_KEYS:
        assert_bitwise_equal(np.asarray(got[k]), getattr(want, k),
                             f'{msg} result.{k}')
    want_patches = np.asarray(want.contact_patches, dtype=np.int32) \
        .reshape(-1, 2)
    got_patches = np.asarray(got['contact_patches'])
    assert got_patches.dtype == np.int32
    assert_bitwise_equal(got_patches, want_patches,
                         f'{msg} result.contact_patches')
    assert_bitwise_equal(np.asarray(got['contact_lengths_m']),
                         np.asarray(want.contact_lengths_m, dtype=float),
                         f'{msg} result.contact_lengths_m')
    for k in _RESULT_FLOAT_KEYS:
        assert_bitwise_equal(float(got[k]), float(getattr(want, k)),
                             f'{msg} result.{k}')
    _golden.record(np.asarray(bool(want.rim_strike_active)),
                   f'{msg} result.rim_strike_active')
    assert bool(got['rim_strike_active']) == bool(want.rim_strike_active), \
        f'{msg} result.rim_strike_active'
    _golden.record(np.asarray(want.rim_event is not None),
                   f'{msg} result.rim_event')
    if want.rim_event is None:
        assert got['rim_event'] is None, f'{msg} result.rim_event'
    else:
        assert got['rim_event'] is not None, f'{msg} result.rim_event'
        for k in _EVENT_KEYS:
            assert_bitwise_equal(float(got['rim_event'][k]),
                                 float(getattr(want.rim_event, k)),
                                 f'{msg} result.rim_event.{k}')


def _native_step(spec, n_rays, hits, state_dict, dt_s, omega, speed):
    return bike_native._debug_pneumatic_carcass(
        spec, n_rays, HALF_DEG, hits, state_dict, dt_s, omega, speed)


def _run_sequence(spec, tyre, n_rays, steps, check_oracle_state=None):
    """Drive one identical multi-step sequence through both
    implementations and compare every result field plus every post-call
    state byte-for-byte.

    ``steps`` is a list of ('evaluate', hits_dict, dt, omega, speed) or
    ('advance', omega, dt, dtheta, tau) rows — the latter exercises
    advance_unloaded mid-sequence.
    """
    ring = geometry.RayRing(n_rays, HALF)
    oracle_state = carcass.CarcassState(n_rays)
    native_state = _fresh_state_dict(n_rays)
    for idx, step in enumerate(steps):
        tag = f'step {idx} ({step[0]})'
        if step[0] == 'evaluate':
            _, hits_dict, dt, omega, speed = step
            hits = geometry.RayHits(
                r_m=hits_dict['r_m'], delta_m=hits_dict['delta_m'],
                road_x_m=hits_dict['road_x_m'],
                road_z_m=hits_dict['road_z_m'],
                coverage_event=hits_dict['coverage_event'],
                airborne=hits_dict['airborne'])
            want = carcass.evaluate_carcass(
                ring, hits, tyre, oracle_state, dt_s=dt,
                omega_radps=omega, speed_mps=speed)
            got = _native_step(spec, n_rays, hits_dict, native_state,
                               dt, omega, speed)
            _assert_result_bitwise(got, want, tag)
            native_state = got['state']
            _assert_state_bitwise(native_state, oracle_state, tag)
            if check_oracle_state is not None:
                check_oracle_state(want, oracle_state, tag)
        else:
            _, omega, dt, dtheta, tau = step
            oracle_state.advance_unloaded(
                omega_radps=omega, dt_s=dt, dtheta_rad=dtheta,
                relaxation_s=tau)
            native_state = bike_native._debug_pneumatic_advance_unloaded(
                native_state, omega, dt, dtheta, tau)
            _assert_state_bitwise(native_state, oracle_state, tag)
    return oracle_state, native_state


# --------------------------------------------------------------------
# Shared roads / hit drivers
# --------------------------------------------------------------------

def _flat_road():
    x = np.arange(1201) * 0.01
    return geometry.RoadProfile(x0_m=0.0, dx_m=0.01,
                                z_m=np.zeros(x.size))


def _bump_road():
    """A short rounded bump plus a dip — enough for multi-patch hits."""
    x = np.arange(1201) * 0.01
    z = (0.028 * np.exp(-((x - 5.05) / 0.16) ** 2)
         - 0.018 * np.exp(-((x - 4.72) / 0.10) ** 2)
         + 0.012 * np.exp(-((x - 5.55) / 0.09) ** 2))
    return geometry.RoadProfile(x0_m=0.0, dx_m=0.01, z_m=z)


def _real_hits(ring, road, cx, cz, radius):
    return geometry.intersect(ring, cx, cz, radius, road)


def _real_sequence(ring, road, radius, rows):
    """rows: (cx, cz, dt, omega, speed) → evaluate steps via real hits."""
    return [('evaluate',
             _hits_dict(_real_hits(ring, road, cx, cz, radius)),
             dt, omega, speed)
            for cx, cz, dt, omega, speed in rows]


# --------------------------------------------------------------------
# Multi-step evaluate_carcass sequences
# --------------------------------------------------------------------

@pytest.mark.parametrize('tyre', [FRONT_TYRE, REAR_TYRE])
@pytest.mark.parametrize('n_rays', RAYS)
def test_carcass_progressive_load_unload_bitwise(tyre, n_rays):
    """Hub descends through a bump, holds, rises again — exercises the
    initialized path, hysteresis sign flips and Maxwell
    relaxation/advection with a spinning wheel."""
    ring = geometry.RayRing(n_rays, HALF)
    road = _bump_road()
    radius = tyre.outer_radius_mm * 0.001
    cz_path = (0.46, 0.40, 0.355, 0.343, 0.339, 0.335, 0.333,
               0.336, 0.350, 0.385, 0.44)
    rows = [(5.0, cz, 0.004, 9.5, 4.4) for cz in cz_path]
    steps = _real_sequence(ring, road, radius, rows)
    _run_sequence(_spec_dict(tyre), tyre, n_rays, steps)


@pytest.mark.parametrize('n_rays', RAYS)
def test_carcass_rim_strike_bitwise(n_rays):
    """Deep deflection crosses rim_strike_deflection_mm: the FSM must
    start, track peaks and emit the completed RimStrikeEvent on release
    — with the peak ray's road x, |speed| and summed rim force."""
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    rim_defl = tyre.rim_strike_deflection_mm * 0.001
    # Hub heights that put the bottom ray's delta across the rim
    # threshold; the last two release it again.
    cz_below = radius - rim_defl - 0.004
    cz_below_deep = radius - rim_defl - 0.012
    rows = [(5.0, 0.36, 0.002, -7.0, -3.2),          # approach, omega<0
            (5.0, cz_below, 0.002, -7.5, -3.2),      # rim engage
            (5.0, cz_below_deep, 0.002, -8.0, -3.2), # deepen
            (5.0, cz_below, 0.002, -8.0, -3.2),      # hold
            (5.0, 0.34, 0.002, -8.0, -3.2),          # partial release
            (5.0, 0.36, 0.002, -7.0, -3.2)]          # full release → event
    steps = _real_sequence(ring, road, radius, rows)
    seen = []

    def watch(want, _state, tag):
        seen.append(want.rim_event)
        # Confirm the event actually engaged at some point.
        if tag.startswith('step 4'):
            assert any(e is not None for e in seen), \
                'rim strike never engaged'

    _run_sequence(_spec_dict(tyre), tyre, n_rays, steps, watch)
    # The completed event must exist and carry nonzero peaks.
    events = [e for e in seen if e is not None]
    assert len(events) == 1
    ev = events[0]
    assert ev.peak_load_n > 0.0 and ev.peak_rim_force_n > 0.0
    assert ev.absorbed_energy_j > 0.0 and ev.speed_mps == 3.2


@pytest.mark.parametrize('n_rays', RAYS)
def test_carcass_airborne_landed_bitwise(n_rays):
    """airborne (all-zero delta) → land → airborne again — the loaded
    mask, empty patches and zero totals must all match bitwise."""
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    rows = [(5.0, radius + 0.5, 0.002, 6.0, 2.0),   # airborne, first step
            (5.0, radius + 0.4, 0.002, 6.0, 2.0),   # still airborne
            (5.0, radius - 0.01, 0.002, 6.0, 2.0),  # touchdown
            (5.0, radius - 0.02, 0.002, 6.0, 2.0),
            (5.0, radius + 0.5, 0.002, 6.0, 2.0)]   # airborne again
    steps = _real_sequence(ring, road, radius, rows)
    _run_sequence(_spec_dict(tyre), tyre, n_rays, steps)


@pytest.mark.parametrize('n_rays', RAYS)
def test_carcass_advance_unloaded_sequence_bitwise(n_rays):
    """advance_unloaded inside the sequence: no-op while uninitialized,
    gate skip on zero maxwell, and the spinning advect+decay path once
    Maxwell memory exists."""
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    dtheta = ring.dtheta_rad
    steps = [
        # Not initialized yet — immediate return.
        ('advance', 7.0, 0.002, dtheta, tyre.rate_relaxation_s),
        # Land: first step initializes, maxwell stays zero → the
        # np.any(maxwell) gate must skip advection.
        ('evaluate',
         _hits_dict(_real_hits(ring, road, 5.0, radius - 0.012, radius)),
         0.002, 6.0, 2.5),
        ('advance', 7.0, 0.002, dtheta, tyre.rate_relaxation_s),
        # Load deeper while rolling: Maxwell memory becomes nonzero.
        ('evaluate',
         _hits_dict(_real_hits(ring, road, 5.0, radius - 0.02, radius)),
         0.002, 6.0, 2.5),
        # Spinning advance: decay * advected row.
        ('advance', 7.0, 0.002, dtheta, tyre.rate_relaxation_s),
        ('advance', -4.0, 0.003, dtheta, tyre.rate_relaxation_s),
        ('advance', 0.0, 0.002, dtheta, tyre.rate_relaxation_s),
        ('advance', 7.0, 0.002, dtheta, tyre.rate_relaxation_s),
        # Back on the ground: the history must have moved exactly like
        # the oracle's.
        ('evaluate',
         _hits_dict(_real_hits(ring, road, 5.0, radius - 0.018, radius)),
         0.002, 6.0, 2.5),
    ]
    _run_sequence(_spec_dict(tyre), tyre, n_rays, steps)


# --------------------------------------------------------------------
# material_deflection_rate_mps / _advect_material unit coverage
# --------------------------------------------------------------------

@pytest.mark.parametrize('omega', [0.0, 3.7, -5.2, 1e-9])
@pytest.mark.parametrize('n', [3, 17, 64, 256])
def test_carcass_material_rate_bitwise(omega, n):
    rng = np.random.default_rng(n * 7919 + int(abs(omega) * 1000))
    delta = rng.uniform(0.0, 0.09, n)
    prev = rng.uniform(0.0, 0.09, n)
    dt, dtheta = 0.004, 0.117
    want = carcass.material_deflection_rate_mps(delta, prev, omega, dt,
                                                dtheta)
    got = bike_native._debug_pneumatic_material_rate(delta, prev, omega,
                                                     dt, dtheta)
    assert_bitwise_equal(np.asarray(got), want)


@pytest.mark.parametrize('n', [3, 5, 64, 256])
def test_carcass_material_rate_edges(n):
    """The out-of-window taps at rays 0/n-1 in both upwind directions."""
    delta = np.linspace(0.0, 0.07, n)
    prev = np.linspace(0.04, 0.0, n)
    for omega in (12.0, -12.0):
        want = carcass.material_deflection_rate_mps(delta, prev, omega,
                                                    0.002, 0.09)
        got = bike_native._debug_pneumatic_material_rate(
            delta, prev, omega, 0.002, 0.09)
        assert_bitwise_equal(np.asarray(got), want)


def _advect_oracle(values, omega, dt, dtheta):
    # The private oracle is the contract — call it by name like the
    # production kernel does.
    return carcass._advect_material(np.asarray(values, float), omega, dt,
                                    dtheta)


@pytest.mark.parametrize('n', [3, 5, 17, 64, 256])
@pytest.mark.parametrize('shift', [0.0, 0.3, -0.6, 1.0, -2.7, 3.4, -4.9,
                                   63.9, -64.1])
def test_carcass_advect_bitwise(n, shift):
    """Fractional source bins, exact bins and the out-of-window
    left/right=0.0 sentinels — shift = omega*dt/dtheta reaches all of
    np.interp's branches."""
    rng = np.random.default_rng(n)
    values = rng.uniform(-1e3, 1e3, n)
    # rng.uniform's low + range*next transform contracts to fmadd on
    # the darwin wheel but not the linux one — replay the frozen row.
    _golden.record(values, 'input.values')
    dtheta, dt = 0.11, 0.004
    omega = shift * dtheta / dt if shift != 0.0 else 0.0
    want = _advect_oracle(values, omega, dt, dtheta)
    got = bike_native._debug_pneumatic_advect(values, omega, dt, dtheta)
    assert_bitwise_equal(np.asarray(got), want)


def test_carcass_advect_zero_omega_copy():
    values = np.array([1.5, -0.0, 4.25, np.nextafter(0.0, 1.0)])
    want = _advect_oracle(values, 0.0, 0.01, 0.5)
    got = bike_native._debug_pneumatic_advect(values, 0.0, 0.01, 0.5)
    assert_bitwise_equal(np.asarray(got), want)


def test_carcass_empty_rows():
    """Empty-row edge cases: np.interp returns [] on an empty query, and
    numpy 2.x broadcasts the length-1 concatenate() tap against the
    empty row — so both helpers return empty rows for every omega."""
    empty = np.empty(0)
    assert_bitwise_equal(
        np.asarray(bike_native._debug_pneumatic_advect(empty, 0.0,
                                                       0.01, 0.1)),
        _advect_oracle(empty, 0.0, 0.01, 0.1))
    assert_bitwise_equal(
        np.asarray(bike_native._debug_pneumatic_advect(empty, 3.0,
                                                       0.01, 0.1)),
        _advect_oracle(empty, 3.0, 0.01, 0.1))
    for omega in (0.0, 1.0, -2.0):
        assert_bitwise_equal(
            np.asarray(bike_native._debug_pneumatic_material_rate(
                empty, empty, omega, 0.01, 0.1)),
            carcass.material_deflection_rate_mps(empty, empty, omega,
                                                 0.01, 0.1))
    # Width mismatch — the shape check precedes the empty path.
    with pytest.raises(ValueError):
        carcass.material_deflection_rate_mps(empty, np.ones(2), 0.0,
                                             0.01, 0.1)
    with pytest.raises(ValueError):
        bike_native._debug_pneumatic_material_rate(empty, np.ones(2),
                                                   0.0, 0.01, 0.1)


# --------------------------------------------------------------------
# np.tanh / np.hypot transcendental probes
# --------------------------------------------------------------------

def test_carcass_math_probes_bitwise():
    """The two lowerings that are NOT the platform libm defaults:
    np.tanh (numpy's lut18x16 SIMD table) and np.hypot (libm hypot).
    Sweep structured values, the LUT's interval bases, subnormals,
    infinities and NaN."""
    xs = np.concatenate([
        np.linspace(-25.0, 25.0, 4001),
        np.linspace(-0.25, 0.25, 2001),
        np.array([0.0, -0.0, np.inf, -np.inf, np.nan,
                  np.finfo(np.float64).tiny, -np.finfo(np.float64).tiny,
                  np.nextafter(20.0, 0.0), np.nextafter(20.0, 30.0),
                  20.0, 9.011, 8.7,
                  float.fromhex('0x1.8p-3'), float.fromhex('0x1.3p-4'),
                  float.fromhex('0x1.cp-2')]),
    ])
    rng = np.random.default_rng(0xC0FFEE)
    raw = rng.integers(0, 1 << 63, 2048).astype(np.uint64)
    xs = np.concatenate([xs, raw.view(np.float64)])
    ys = rng.uniform(-1.0, 1.0, xs.size)
    ys[: xs.size // 7] = 0.0
    want_tanh = np.tanh(xs)
    want_hypot = np.hypot(xs, ys)
    # The uint64-derived probe rows are replayed verbatim by the C++
    # port (the rng.integers high bound does not fit the int64 API).
    _golden.record(xs, 'input.xs')
    _golden.record(ys, 'input.ys')
    got = bike_native._debug_pneumatic_math(xs, ys)
    assert_bitwise_equal(np.asarray(got['tanh']), want_tanh)
    assert_bitwise_equal(np.asarray(got['hypot']), want_hypot)


# --------------------------------------------------------------------
# Nonnegative saturation, single-ray patch, boundary patches
# --------------------------------------------------------------------

@pytest.mark.parametrize('n_rays', RAYS)
def test_carcass_saturation_and_patch_shapes_bitwise(n_rays):
    """* total clamps at 0 when the decayed Maxwell term is negative
      (rapid unload) while the ray stays loaded;
    * a single loaded ray uses the one-ray-span cell;
    * patches at rays 0 and n-1 are emitted; two disjoint bumps give two
      patches and two hypot lengths."""
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    spec = _spec_dict(tyre)
    rng = np.random.default_rng(n_rays)

    # --- step 1: deep wide load, rolling forward --------------------
    delta = np.zeros(n_rays)
    c = n_rays // 2
    delta[c - 4:c + 5] = 0.05 + 0.01 * rng.uniform(0.0, 1.0, 9)
    road_x = np.cumsum(np.full(n_rays, 0.001)) + 4.9
    road_z = rng.uniform(-0.05, -0.02, n_rays)
    # rng.uniform's low + range*next transform is platform-contracted —
    # replay the frozen rows (hits3's drawn road_z likewise).
    _golden.record(road_z, 'input.road_z')
    hits1 = _synth_hits(n_rays, delta, rng, road_x, road_z)
    oracle_state = carcass.CarcassState(n_rays)
    native_state = _fresh_state_dict(n_rays)
    want = carcass.evaluate_carcass(ring, _oracle_hits(hits1), tyre,
                                    oracle_state, dt_s=0.002,
                                    omega_radps=8.0, speed_mps=3.0)
    got = _native_step(spec, n_rays, hits1, native_state, 0.002, 8.0, 3.0)
    _assert_result_bitwise(got, want, 'load')
    native_state = got['state']
    _assert_state_bitwise(native_state, oracle_state, 'load')

    # --- step 2: rapid partial unload — strong negative Maxwell
    # contribution must clamp total at zero on rays still loaded.
    delta2 = delta * 0.06
    delta2[delta2 == 0] = 0.0
    hits2 = _synth_hits(n_rays, delta2, rng, road_x, road_z)
    want = carcass.evaluate_carcass(ring, _oracle_hits(hits2), tyre,
                                    oracle_state, dt_s=0.001,
                                    omega_radps=9.0, speed_mps=3.1)
    got = _native_step(spec, n_rays, hits2, native_state, 0.001, 9.0, 3.1)
    _assert_result_bitwise(got, want, 'unload-clamp')
    pre_clamp = (want.elastic_force_n + want.maxwell_force_n
                 + want.hysteresis_force_n + want.rim_force_n)
    assert np.any(pre_clamp < 0.0), 'the saturation branch never ran'
    _assert_state_bitwise(got['state'], oracle_state, 'unload-clamp')
    native_state = got['state']

    # --- step 3: single-ray patch + boundary rays --------------------
    delta3 = np.zeros(n_rays)
    delta3[0] = 0.02
    delta3[n_rays - 1] = 0.015
    delta3[n_rays // 3] = 0.03      # single-ray patch mid-ring
    hits3 = _synth_hits(n_rays, delta3, rng)
    _golden.record(np.asarray(hits3['road_z_m']),
                   'input.hits3 road_z_m')
    want = carcass.evaluate_carcass(ring, _oracle_hits(hits3), tyre,
                                    oracle_state, dt_s=0.002,
                                    omega_radps=0.0, speed_mps=0.0)
    got = _native_step(spec, n_rays, hits3, native_state, 0.002, 0.0, 0.0)
    _assert_result_bitwise(got, want, 'single-ray')
    assert len(want.contact_patches) == 3
    # The single-ray patches take the one_ray_span branch.
    assert len(want.contact_lengths_m) == 3


def _oracle_hits(hits_dict):
    return geometry.RayHits(
        r_m=hits_dict['r_m'], delta_m=hits_dict['delta_m'],
        road_x_m=hits_dict['road_x_m'], road_z_m=hits_dict['road_z_m'],
        coverage_event=hits_dict['coverage_event'],
        airborne=hits_dict['airborne'])


# --------------------------------------------------------------------
# Validation order + failure atomicity
# --------------------------------------------------------------------

def _valid_evaluate_args(n_rays):
    spec = _spec_dict(FRONT_TYRE)
    delta = np.zeros(n_rays)
    delta[n_rays // 2] = 0.01
    hits = _synth_hits(n_rays, delta)
    # The drawn road_z row is wheel-contraction divergent — freeze it.
    _golden.record(np.asarray(hits['road_z_m']), 'input.road_z_m')
    return spec, hits


@pytest.mark.parametrize('n_rays', RAYS)
def test_carcass_invalid_inputs_bitwise_atomicity(n_rays):
    """Every rejection class, in the oracle's check order, plus state
    preservation: the failed call must leave both states untouched so
    the follow-up valid call still matches."""
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    spec, hits = _valid_evaluate_args(n_rays)
    oracle_hits = _oracle_hits(hits)
    oracle_state = carcass.CarcassState(n_rays)
    # Establish a nonzero history so the failed call has something to
    # lose.
    carcass.evaluate_carcass(ring, oracle_hits, tyre, oracle_state,
                             dt_s=0.002, omega_radps=4.0, speed_mps=1.0)
    native_state = _native_step(
        spec, n_rays, hits, _fresh_state_dict(n_rays), 0.002, 4.0, 1.0)[
        'state']

    bad_cases = [
        # dt invalid — first check.
        dict(dt_s=0.0), dict(dt_s=-0.002), dict(dt_s=np.nan),
        dict(dt_s=np.inf),
        # omega/speed non-finite — second check.
        dict(omega_radps=np.nan), dict(omega_radps=np.inf),
        dict(speed_mps=-np.inf),
        # delta shape/content — fourth check.
        dict(delta='short'), dict(delta='nan'), dict(delta='neg'),
    ]
    for bad in bad_cases:
        dt = bad.get('dt_s', 0.002)
        omega = bad.get('omega_radps', 4.0)
        speed = bad.get('speed_mps', 1.0)
        o_hits = oracle_hits
        n_hits = hits
        if 'delta' in bad:
            delta = np.zeros(n_rays)
            delta[n_rays // 2] = 0.01
            if bad['delta'] == 'nan':
                delta[3] = np.nan
            elif bad['delta'] == 'neg':
                delta[3] = -1e-9
            elif bad['delta'] == 'short':
                delta = delta[:-1]
            o_hits = geometry.RayHits(
                r_m=o_hits.r_m, delta_m=delta,
                road_x_m=o_hits.road_x_m, road_z_m=o_hits.road_z_m,
                coverage_event=False, airborne=False)
            n_hits = {**hits, 'delta_m': delta}
        # Oracle side: raises and leaves its state untouched.
        snap = _oracle_state_dict(oracle_state)
        with pytest.raises(ValueError):
            carcass.evaluate_carcass(ring, o_hits, tyre, oracle_state,
                                     dt_s=dt, omega_radps=omega,
                                     speed_mps=speed)
        _assert_state_bitwise(snap, oracle_state, 'oracle atomicity')
        # Native side: same ValueError class (std::invalid_argument).
        with pytest.raises(ValueError):
            _native_step(spec, n_rays, n_hits, native_state, dt, omega,
                         speed)

    # State-mismatch check sits between the omega/speed and delta
    # checks (oracle order level 3): a state dict one ray too narrow
    # must fail even with a nan in delta AND a good dt.
    narrow_state = dict(native_state)
    narrow_state['delta_prev_m'] = native_state['delta_prev_m'][:-1]
    narrow_state['elastic_prev_n'] = native_state['elastic_prev_n'][:-1]
    narrow_state['maxwell_force_n'] = native_state['maxwell_force_n'][:-1]
    delta_nan = hits['delta_m'].copy()
    delta_nan[0] = np.nan
    n_hits = {**hits, 'delta_m': delta_nan}
    with pytest.raises(ValueError):
        _native_step(spec, n_rays, n_hits, narrow_state, 0.002, 4.0, 1.0)
    wrong_ring_state = carcass.CarcassState(n_rays - 1)
    with pytest.raises(ValueError):
        carcass.evaluate_carcass(ring, oracle_hits, tyre,
                                 wrong_ring_state, dt_s=0.002,
                                 omega_radps=4.0, speed_mps=1.0)

    # Ordering: dt wins over bad omega and bad delta; omega wins over
    # bad delta.
    bad_hits = {**hits, 'delta_m': np.full(n_rays, np.nan)}
    bad_o = geometry.RayHits(r_m=hits['r_m'], delta_m=np.full(n_rays, np.nan),
                             road_x_m=hits['road_x_m'],
                             road_z_m=hits['road_z_m'],
                             coverage_event=False, airborne=False)
    with pytest.raises(ValueError, match='dt_s'):
        carcass.evaluate_carcass(ring, bad_o, tyre, oracle_state,
                                 dt_s=0.0, omega_radps=np.nan,
                                 speed_mps=0.0)
    with pytest.raises(ValueError):
        _native_step(spec, n_rays, bad_hits, native_state, 0.0,
                     np.nan, 0.0)
    with pytest.raises(ValueError, match='finite'):
        carcass.evaluate_carcass(ring, bad_o, tyre, oracle_state,
                                 dt_s=0.002, omega_radps=np.inf,
                                 speed_mps=0.0)
    with pytest.raises(ValueError):
        _native_step(spec, n_rays, bad_hits, native_state, 0.002,
                     np.inf, 0.0)

    # Both states must still evolve identically on the next valid call.
    want = carcass.evaluate_carcass(ring, oracle_hits, tyre,
                                    oracle_state, dt_s=0.002,
                                    omega_radps=4.0, speed_mps=1.0)
    got = _native_step(spec, n_rays, hits, native_state, 0.002, 4.0, 1.0)
    _assert_result_bitwise(got, want, 'post-failure')
    _assert_state_bitwise(got['state'], oracle_state, 'post-failure')


def test_carcass_advance_unloaded_validation():
    """advance_unloaded: early return before validation, the
    positivity check, and the np.any gate — all bitwise."""
    n_rays = 64
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    spec = _spec_dict(tyre)
    # Uninitialized: even invalid dt returns silently.
    s0 = carcass.CarcassState(n_rays)
    s0.advance_unloaded(omega_radps=5.0, dt_s=-1.0, dtheta_rad=0.1,
                        relaxation_s=0.2)
    n0 = _fresh_state_dict(n_rays)
    out = bike_native._debug_pneumatic_advance_unloaded(n0, 5.0, -1.0,
                                                        0.1, 0.2)
    _assert_state_bitwise(out, s0, 'uninitialized early return')

    # Build a state with nonzero Maxwell memory.
    flat = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    hits_a = _hits_dict(_real_hits(ring, flat, 5.0, radius - 0.02,
                                   radius))
    state = carcass.CarcassState(n_rays)
    carcass.evaluate_carcass(ring, _oracle_hits(hits_a), tyre, state,
                             dt_s=0.002, omega_radps=6.0, speed_mps=2.0)
    hits_b = _hits_dict(_real_hits(ring, flat, 5.0, radius - 0.012,
                                   radius))
    carcass.evaluate_carcass(ring, _oracle_hits(hits_b), tyre, state,
                             dt_s=0.002, omega_radps=6.0, speed_mps=2.0)
    assert np.any(state.maxwell_force_n)
    native_state = _oracle_state_dict(state)
    # Invalid params raise on both sides; state untouched.
    for bad in (dict(dt_s=0.0), dict(dt_s=-1.0), dict(dtheta_rad=0.0),
                dict(dtheta_rad=-0.5), dict(relaxation_s=0.0),
                dict(relaxation_s=-3.0)):
        kw = dict(omega_radps=5.0, dt_s=0.002,
                  dtheta_rad=ring.dtheta_rad, relaxation_s=0.2)
        kw.update(bad)
        snap = _oracle_state_dict(state)
        with pytest.raises(ValueError):
            state.advance_unloaded(**kw)
        _assert_state_bitwise(snap, state, 'advance atomicity')
        with pytest.raises(ValueError):
            bike_native._debug_pneumatic_advance_unloaded(
                native_state, kw['omega_radps'], kw['dt_s'],
                kw['dtheta_rad'], kw['relaxation_s'])
    # dt=nan passes the oracle's positivity check (nan <= 0 is False)
    # and propagates nan through the advect — keep that quirk.
    snap = _oracle_state_dict(state)
    state.advance_unloaded(omega_radps=5.0, dt_s=np.nan,
                           dtheta_rad=ring.dtheta_rad, relaxation_s=0.2)
    out = bike_native._debug_pneumatic_advance_unloaded(
        native_state, 5.0, np.nan, ring.dtheta_rad, 0.2)
    _assert_state_bitwise(out, state, 'nan dt propagation')


def test_carcass_state_construction_validation():
    """CarcassState.__post_init__ rejects n < 3 — surfaced through the
    state dict intake."""
    with pytest.raises(ValueError, match='at least 3 rays'):
        carcass.CarcassState(2)
    with pytest.raises(ValueError):
        bike_native._debug_pneumatic_advance_unloaded(
            _fresh_state_dict(2), 0.0, 0.002, 0.1, 0.2)


def test_carcass_first_step_initialized_branch():
    """The first evaluate on a fresh state takes the uninitialized
    branch: material_rate and maxwell are exact +0.0 rows."""
    n_rays = 64
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    delta = np.zeros(n_rays)
    delta[30:35] = 0.02
    hits = _synth_hits(n_rays, delta)
    # The drawn road_z row is wheel-contraction divergent — freeze it.
    _golden.record(np.asarray(hits['road_z_m']), 'input.road_z_m')
    state = carcass.CarcassState(n_rays)
    want = carcass.evaluate_carcass(ring, _oracle_hits(hits), tyre,
                                    state, dt_s=0.002, omega_radps=5.0,
                                    speed_mps=2.0)
    _golden.record(np.zeros(n_rays), 'first-step material_rate_mps +0.0')
    assert want.material_rate_mps.tobytes() == np.zeros(
        n_rays).tobytes()
    _golden.record(np.zeros(n_rays), 'first-step maxwell_force_n +0.0')
    assert want.maxwell_force_n.tobytes() == np.zeros(n_rays).tobytes()
    got = _native_step(_spec_dict(tyre), n_rays, hits,
                       _fresh_state_dict(n_rays), 0.002, 5.0, 2.0)
    _assert_result_bitwise(got, want, 'first-step')
    _assert_state_bitwise(got['state'], state, 'first-step')
    # And the second step now takes the initialized branch.
    hits2 = _synth_hits(n_rays, delta * 1.1)
    _golden.record(np.asarray(hits2['road_z_m']),
                   'input.hits2 road_z_m')
    want = carcass.evaluate_carcass(ring, _oracle_hits(hits2), tyre,
                                    state, dt_s=0.002, omega_radps=5.0,
                                    speed_mps=2.0)
    got = _native_step(_spec_dict(tyre), n_rays, hits2, got['state'],
                       0.002, 5.0, 2.0)
    _assert_result_bitwise(got, want, 'second-step')
    _assert_state_bitwise(got['state'], state, 'second-step')


@pytest.mark.parametrize('n_rays', RAYS)
def test_carcass_multistep_mixed_speed_spin_bitwise(n_rays):
    """A longer mixed sequence: varying spin sign, speed (incl.
    negative for the abs() in the event record), dt and hub path —
    real intersect hits on the bump road."""
    tyre = REAR_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    road = _bump_road()
    radius = tyre.outer_radius_mm * 0.001
    rng = np.random.default_rng(n_rays)
    rows = []
    cx = 4.6
    cz = radius + 0.10
    for k in range(22):
        cx += 0.028
        cz = radius + 0.10 - min(k * 0.013, 0.125)
        omega = (8.0 if k % 5 < 3 else -6.5)
        speed = (3.4 + 0.1 * k) * (1.0 if k % 4 else -1.0)
        rows.append((float(cx), float(cz), 0.002 + 0.0002 * (k % 3),
                     omega, speed))
    steps = _real_sequence(ring, road, radius, rows)
    _run_sequence(_spec_dict(tyre), tyre, n_rays, steps)


def test_carcass_negative_zero_delta_and_inf_hits():
    """delta rows carrying -0.0 (accepted: not < 0.0) and inf r_m —
    signed-zero propagation through np.maximum/np.where stays bitwise."""
    n_rays = 64
    tyre = FRONT_TYRE
    ring = geometry.RayRing(n_rays, HALF)
    spec = _spec_dict(tyre)
    delta = np.zeros(n_rays)
    delta[10] = -0.0
    delta[11] = 0.01
    hits = _synth_hits(n_rays, delta)
    # The drawn road_z row is wheel-contraction divergent — freeze it.
    _golden.record(np.asarray(hits['road_z_m']), 'input.road_z_m')
    state = carcass.CarcassState(n_rays)
    want = carcass.evaluate_carcass(ring, _oracle_hits(hits), tyre,
                                    state, dt_s=0.002, omega_radps=3.0,
                                    speed_mps=1.0)
    got = _native_step(spec, n_rays, hits, _fresh_state_dict(n_rays),
                       0.002, 3.0, 1.0)
    _assert_result_bitwise(got, want, '-0.0 delta')
    _assert_state_bitwise(got['state'], state, '-0.0 delta')


# --------------------------------------------------------------------
# np.interp on the unit knot grid — the dedicated lenxp == 1 branch
# --------------------------------------------------------------------

def test_pneumatic_interp_single_knot_bitwise():
    """``_debug_pneumatic_interp`` with one value hits arr_interp's
    dedicated lenxp == 1 loop (compiled_base.c:671-682), which runs
    BEFORE any NaN check: x < 0 -> lval, x > 0 -> rval, everything
    else — x == 0, -0.0 AND NaN — returns values[0]."""
    xs = [0.0, -0.0, np.nan, -np.nan, 1e-300, -1e-300, 0.5, -0.5,
          np.inf, -np.inf, np.nextafter(0.0, 1.0),
          np.nextafter(0.0, -1.0)]
    for x in xs:
        want = float(np.interp(x, np.arange(1), np.array([7.25]),
                               left=-9.5, right=11.75))
        got = bike_native._debug_pneumatic_interp(
            float(x), np.array([7.25]), -9.5, 11.75)
        assert_bitwise_equal(got, want, f'len1 x={x!r}')


def test_pneumatic_interp_multi_knot_bitwise():
    """n >= 2 keeps the general path: NaN queries pass the payload
    through, interior/exact-knot/last-knot/out-of-range probes all
    match np.interp bitwise."""
    values = np.array([2.0, -4.5, 8.25, -1.0])
    xs = [np.nan, -np.nan, 0.0, -0.0, 1.0, 2.0, 3.0, 0.25, 1.5, 2.999,
          3.0000000001, -1e-9, 4.0, np.inf, -np.inf,
          np.nextafter(3.0, 0.0), np.nextafter(0.0, 1.0)]
    for x in xs:
        want = float(np.interp(x, np.arange(4), values, left=-9.5,
                               right=11.75))
        got = bike_native._debug_pneumatic_interp(
            float(x), values, -9.5, 11.75)
        assert_bitwise_equal(got, want, f'len4 x={x!r}')


def test_pneumatic_interp_two_knot_and_empty():
    """The n == 2 minimum-size general path, a zero-slope segment, and
    the empty-row rejection on both sides."""
    # Zero slope between equal knots — fma(0, x-x0, v) stays exact.
    values = np.array([1.5, 1.5])
    assert_bitwise_equal(
        bike_native._debug_pneumatic_interp(0.5, values, -1.0, 1.0),
        float(np.interp(0.5, np.arange(2), values, left=-1.0,
                        right=1.0)))
    # Two-element interpolation through the slope formula.
    values = np.array([-3.0, 9.0])
    for x in (0.25, 0.5, 0.75, np.nextafter(1.0, 0.0)):
        assert_bitwise_equal(
            bike_native._debug_pneumatic_interp(x, values, -7.0, 7.0),
            float(np.interp(x, np.arange(2), values, left=-7.0,
                            right=7.0)),
            f'len2 x={x!r}')
    # Empty values raise on both sides.
    with pytest.raises(ValueError):
        bike_native._debug_pneumatic_interp(0.5, np.empty(0), 0.0, 1.0)
    with pytest.raises(ValueError):
        np.interp(0.5, np.arange(0), np.empty(0), left=0.0, right=1.0)
