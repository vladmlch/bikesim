"""Bitwise oracle tests for the native pneumatic tyre brush kernels.

Covers the Task-9 debug surfaces in ``bike_native``:

* ``_debug_pneumatic_relax``             — ``brush.relax``
* ``_debug_pneumatic_sigma_x``           — ``brush._sigma_x``
* ``_debug_pneumatic_surface_mu``        — ``SurfaceSpec.mu`` Stribeck
* ``_debug_pneumatic_tread_stiffness``   — ``brush._tread_stiffness_n_m2``
* ``_debug_pneumatic_lumped_brush``      — ``brush.lumped_brush``
* ``_debug_pneumatic_discretised_brush`` — ``brush.DiscretisedBrush.step``
  plus the post-call state (bristle row + ``kappa_prime``)

Every floating-point scalar and array is compared raw-byte-for-raw-byte
against the pure-Python oracle in
``src/bike_sim/sim/ride/tyre/brush.py`` — ``tobytes()`` keeps signed
zeros, infinities and NaN payloads distinguishable.  The discretised
surface also returns the post-call state, so bristle evolution and the
transient slip filter are pinned bitwise across multi-step sequences.

The corpus exercises every oracle branch: all four ``relax`` return
paths (zero sigma moving, zero sigma parked, parked linear integration,
the exponential lag) and its two rejection levels; ``_sigma_x``'s
locked-wheel -inf limit, the +inf copysign and the ratio; both
``_tread_stiffness_n_m2`` resolutions and its invalid-value rejection;
``lumped_brush``'s staged finite gate (inf kappa allowed), zero-load
and zero-stiffness early returns, the saturated |z|>=1 copysign force
and the cubic region; ``DiscretisedBrush.step``'s shape/content/scalar
validation order, the zero-load reset mutation, advection under both
transport directions and out-of-window interp sentinels, the verbatim
copy at zero wheel speed, active masking, the exact |trial|==limit
clip boundary, partial and full sliding, and single-element patches.
"""

import numpy as np
import pytest

from bike_sim.sim.ride.tyre import brush
from bike_sim.terrain.surface import SURFACES, SurfaceSpec

from native_loader import load_native
bike_native = load_native()
from _bits import assert_bitwise_equal
import _golden

_SURFACE_KEYS = ('name', 'mu_peak', 'mu_slide', 'slip_stiffness_per_load',
                 'stribeck_speed_mps')


def _surface_dict(spec):
    """A SurfaceSpec as the five-field wire dict."""
    return {k: getattr(spec, k) for k in _SURFACE_KEYS}


def _state_dict(bristle_row, kappa_prime):
    return {'bristle_deflection_m': np.asarray(bristle_row, dtype=float),
            'kappa_prime': float(kappa_prime)}


def _oracle_state_dict(brush_state):
    # np.array(..., copy=True): np.asarray would alias the oracle's
    # bristle row, which step() mutates in place — the snapshot must be
    # a genuine pre-call copy.
    return _state_dict(np.array(brush_state.bristle_deflection_m,
                              dtype=float, copy=True),
                       brush_state.kappa_prime)


def _assert_result_bitwise(got, want, msg=''):
    assert_bitwise_equal(float(got['force_n']), float(want.force_n),
                         f'{msg} result.force_n')
    _golden.record(np.asarray(bool(want.fully_sliding)),
                   f'{msg} result.fully_sliding')
    assert bool(got['fully_sliding']) == bool(want.fully_sliding), \
        f'{msg} result.fully_sliding'
    assert_bitwise_equal(float(got['friction_coefficient']),
                         float(want.friction_coefficient),
                         f'{msg} result.friction_coefficient')


def _assert_brush_state_bitwise(native_state, oracle_brush, msg=''):
    assert_bitwise_equal(np.asarray(native_state['bristle_deflection_m']),
                         oracle_brush.bristle_deflection_m,
                         f'{msg} state.bristle_deflection_m')
    assert_bitwise_equal(float(native_state['kappa_prime']),
                         float(oracle_brush.kappa_prime),
                         f'{msg} state.kappa_prime')


def _native_step(loads, state, half, v_x, v_s, sigma, surface, dt):
    return bike_native._debug_pneumatic_discretised_brush(
        np.asarray(loads, dtype=float), state, half, v_x, v_s, sigma,
        surface, dt)


def _run_brush_sequence(n, steps, seed=0xB417):
    """Drive one identical multi-step DiscretisedBrush sequence through
    both implementations; compare the result and the post-call state
    byte-for-byte at every step.

    ``steps`` rows are (loads, half, v_x, v_s, sigma, surface_dict,
    surface_obj, dt).
    """
    oracle = brush.DiscretisedBrush(n)
    native_state = _state_dict(np.zeros(n), 0.0)
    for idx, (loads, half, v_x, v_s, sigma, sd, surface, dt) in \
            enumerate(steps):
        tag = f'step {idx}'
        want = oracle.step(np.asarray(loads, dtype=float),
                           half_length_m=half, v_x_mps=v_x, v_s_mps=v_s,
                           sigma_m=sigma, surface=surface, dt_s=dt)
        got = _native_step(loads, native_state, half, v_x, v_s, sigma,
                           sd, dt)
        _assert_result_bitwise(got, want, tag)
        native_state = got['state']
        _assert_brush_state_bitwise(native_state, oracle, tag)
    return oracle, native_state


ALL_SURFACES = [SURFACES[k] for k in
                ('asphalt', 'hardpack', 'loose', 'wet', 'scree')]

# A unit-law surface making the exact |trial| == limit clip boundary
# constructible: with loads=1.0 the friction limit equals mu == 1.0 at
# v_s == 0 and the element stiffness divides out cleanly.
UNIT_SURFACE = SurfaceSpec('unit', mu_peak=1.0, mu_slide=0.5,
                           slip_stiffness_per_load=0.5,
                           stribeck_speed_mps=1.0)


# --------------------------------------------------------------------
# relax
# --------------------------------------------------------------------

def test_relax_sweep_bitwise():
    """Grid over all four return paths plus near-degenerate values —
    bitwise at every node."""
    rng = np.random.default_rng(0xE1A0)
    kappas = [0.0, -0.0, 0.15, -0.85, 2.5, -12.0,
              *rng.uniform(-4.0, 4.0, 12).tolist()]
    vxs = [0.0, -0.0, 3.2, -7.5, 1e-9, 400.0]
    vss = [0.0, -0.0, 0.4, -2.2, 30.0]
    sigmas = [0.0, -0.0, 1e-6, 0.45, 2.0]
    dts = [0.0, 1e-4, 0.0005, 0.002, 0.25]
    for kappa in kappas:
        for vx in vxs:
            for vs in vss:
                for sigma in sigmas:
                    for dt in dts:
                        want = brush.relax(kappa, vx, vs, sigma, dt)
                        got = bike_native._debug_pneumatic_relax(
                            kappa, vx, vs, sigma, dt)
                        assert_bitwise_equal(
                            got, want,
                            f'relax({kappa},{vx},{vs},{sigma},{dt})')


def test_relax_extreme_and_transient_bitwise():
    """Overflowing products (exponent -> -inf), exact-zero exponents and
    a chained transient where each step feeds the next."""
    cases = [
        # |v_x|*dt overflows to inf -> exponent -inf -> exp(-inf) = +0.
        (0.5, 1e200, -3.0, 1e-300, 1e200),
        (-0.5, -1e200, 3.0, 1e-300, 1e200),
        # dt = 0 -> exponent -0.0 -> exp = 1.0 -> steady + (k - steady).
        (0.7, 3.0, -1.0, 0.4, 0.0),
        (0.7, -3.0, 1.0, 0.4, -0.0),
        # sigma huge -> exponent ~ -0.
        (0.2, 4.0, 0.5, 1e300, 0.001),
        # parked, sigma > 0: linear integration, both vs signs.
        (0.2, 0.0, 0.5, 0.4, 0.002),
        (0.2, -0.0, -0.5, 0.4, 0.002),
        # parked, sigma = 0: state holds regardless of vs.
        (-0.9, 0.0, 5.0, 0.0, 0.002),
        # sigma = 0, moving: algebraic jump to -vs/|vx|.
        (0.333, 2.0, 0.125, 0.0, 0.001),
    ]
    for args in cases:
        want = brush.relax(*args)
        got = bike_native._debug_pneumatic_relax(*args)
        assert_bitwise_equal(got, want, f'relax{args}')

    # Chained transient: kappa' feeds forward like the model drives it.
    k_o = k_n = 0.0
    for vx, vs, sigma, dt in [(4.0, -0.5, 0.4, 0.0005),
                              (4.0, -0.9, 0.4, 0.0005),
                              (4.0, 0.2, 0.4, 0.0005),
                              (0.0, 0.0, 0.4, 0.0005),
                              (0.0, -0.3, 0.4, 0.0005),
                              (-2.0, 0.1, 0.4, 0.001),
                              (6.5, 1.4, 0.0, 0.0005)]:
        k_o = brush.relax(k_o, vx, vs, sigma, dt)
        k_n = bike_native._debug_pneumatic_relax(k_n, vx, vs, sigma, dt)
        assert_bitwise_equal(k_n, k_o, f'transient ({vx},{vs},{sigma},{dt})')


@pytest.mark.parametrize('bad', [
    dict(kappa_prime=np.nan), dict(kappa_prime=np.inf),
    dict(kappa_prime=-np.inf),
    dict(v_x_mps=np.nan), dict(v_x_mps=np.inf),
    dict(v_s_mps=np.nan), dict(v_s_mps=-np.inf),
    dict(sigma_m=np.nan), dict(sigma_m=np.inf), dict(sigma_m=-1e-9),
    dict(dt_s=np.nan), dict(dt_s=-np.inf), dict(dt_s=-1e-9),
])
def test_relax_rejections(bad):
    """Finite gate first, then the nonnegative-length/dt gate."""
    kw = dict(kappa_prime=0.1, v_x_mps=2.0, v_s_mps=-0.3,
              sigma_m=0.5, dt_s=0.001)
    kw.update(bad)
    with pytest.raises(ValueError):
        brush.relax(**kw)
    with pytest.raises(ValueError):
        bike_native._debug_pneumatic_relax(kw['kappa_prime'],
                                           kw['v_x_mps'], kw['v_s_mps'],
                                           kw['sigma_m'], kw['dt_s'])


def test_relax_rejection_order():
    """A non-finite input beats a negative sigma; a negative sigma and a
    negative dt share the second message."""
    with pytest.raises(ValueError, match='finite'):
        brush.relax(np.nan, 2.0, 0.0, -1.0, -1.0)
    with pytest.raises(ValueError):
        bike_native._debug_pneumatic_relax(np.nan, 2.0, 0.0, -1.0, -1.0)
    with pytest.raises(ValueError, match='not be negative'):
        brush.relax(0.0, 2.0, 0.0, -1.0, 0.001)
    with pytest.raises(ValueError):
        bike_native._debug_pneumatic_relax(0.0, 2.0, 0.0, -1.0, 0.001)


# --------------------------------------------------------------------
# _sigma_x / _tread_stiffness_n_m2 / SurfaceSpec.mu probes
# --------------------------------------------------------------------

@pytest.mark.parametrize('kappa', [
    0.0, -0.0, 0.25, -0.5, 0.9999999999999999, 1.0, 5.0, 1e300,
    -0.9999999999999999, -1.0, -1.0000000000000002, -4.0, -1e300,
    np.inf, -np.inf, np.nan,
    np.nextafter(-1.0, -2.0), np.nextafter(-1.0, 0.0),
])
def test_sigma_x_bitwise(kappa):
    want = brush._sigma_x(kappa)
    got = bike_native._debug_pneumatic_sigma_x(kappa)
    assert_bitwise_equal(got, want, f'sigma_x({kappa!r})')


@pytest.mark.parametrize('surface', ALL_SURFACES)
@pytest.mark.parametrize('load,half', [(0.0, 0.05), (350.0, 0.041),
                                       (1e-9, 0.02), (800.0, 0.005),
                                       (5.0, 1e-9)])
@pytest.mark.parametrize('tread', [None, 0.0, 3.4e4, 1e-12])
def test_tread_stiffness_bitwise(surface, load, half, tread):
    sd = _surface_dict(surface)
    want = brush._tread_stiffness_n_m2(load, half, surface, tread)
    got = bike_native._debug_pneumatic_tread_stiffness(
        load, half, sd, tread)
    assert_bitwise_equal(got, want, f'{surface.name} {load} {half} {tread}')


@pytest.mark.parametrize('tread', [np.nan, np.inf, -np.inf, -1.0])
def test_tread_stiffness_rejections(tread):
    surface = SURFACES['asphalt']
    with pytest.raises(ValueError):
        brush._tread_stiffness_n_m2(400.0, 0.05, surface, tread)
    with pytest.raises(ValueError):
        bike_native._debug_pneumatic_tread_stiffness(
            400.0, 0.05, _surface_dict(surface), tread)


@pytest.mark.parametrize('surface', ALL_SURFACES)
def test_surface_mu_bitwise(surface):
    """The Stribeck curve plus saturation at large |v|."""
    sd = _surface_dict(surface)
    speeds = np.concatenate([
        np.linspace(0.0, 12.0, 121), np.linspace(-12.0, 0.0, 60),
        np.array([-0.0, 1e-9, 4.5, -4.5, 1e3, 1e30]),
    ])
    for v in speeds:
        want = surface.mu(float(v))
        got = bike_native._debug_pneumatic_surface_mu(sd, float(v))
        assert_bitwise_equal(got, want, f'{surface.name}.mu({v!r})')


def test_surface_mu_rejections():
    """array() rejects non-finite speeds; derived_array would raise
    OverflowError on a non-finite coefficient (unreachable for valid
    surfaces)."""
    sd = _surface_dict(SURFACES['wet'])
    for v in (np.nan, np.inf, -np.inf):
        with pytest.raises(ValueError):
            SURFACES['wet'].mu(v)
        with pytest.raises(ValueError):
            bike_native._debug_pneumatic_surface_mu(sd, v)


# --------------------------------------------------------------------
# lumped_brush
# --------------------------------------------------------------------

@pytest.mark.parametrize('surface', ALL_SURFACES)
def test_lumped_brush_sweep_bitwise(surface):
    """Slip sweep through the cubic region into saturation, both load
    magnitudes, explicit and derived stiffness, and Stribeck speeds."""
    sd = _surface_dict(surface)
    kappas = np.concatenate([
        np.linspace(-1.05, 1.05, 43),
        np.array([-0.0, np.nextafter(-1.0, 0.0), -1.0, np.inf, -np.inf,
                  1e6, -1e6]),
    ])
    for kappa in kappas:
        for load, half, tread, slide in [
                (0.0, 0.05, None, 0.0),
                (1e-9, 0.03, None, 1.0),
                (140.0, 0.021, None, 0.0),
                (520.0, 0.052, None, 3.7),
                (520.0, 0.052, 4.2e4, 0.0),
                (520.0, 0.052, 0.0, 8.0),
                (7.0, 1e-6, None, 0.25)]:
            want = brush.lumped_brush(
                float(kappa), load, half, surface,
                tread_stiffness_n_m2=tread, sliding_speed_mps=slide)
            got = bike_native._debug_pneumatic_lumped_brush(
                float(kappa), load, half, sd, tread, slide)
            _assert_result_bitwise(
                got, want, f'{surface.name} k={kappa!r} F={load}')


def test_lumped_brush_stribeck_speeds_bitwise():
    """sliding_speed sweeps decay mu from peak towards slide — force
    and friction_coefficient stay bitwise."""
    surface = SURFACES['loose']
    sd = _surface_dict(surface)
    for slide in (0.0, -0.0, 0.2, 2.0, 4.5, 9.0, 60.0, 1e6):
        for kappa in (0.05, -0.4, np.inf):
            want = brush.lumped_brush(kappa, 300.0, 0.04, surface,
                                      sliding_speed_mps=slide)
            got = bike_native._debug_pneumatic_lumped_brush(
                kappa, 300.0, 0.04, sd, None, slide)
            _assert_result_bitwise(got, want, f'slide={slide}')


def test_lumped_brush_early_returns_and_edges():
    """Zero load (returns before the half-length gate), the c_px == 0
    early return via explicit zero stiffness and via a huge half-length
    underflow, and nan-z propagation."""
    surface = SURFACES['asphalt']
    sd = _surface_dict(surface)
    # load == 0 wins over an invalid half-length (no raise).
    want = brush.lumped_brush(0.3, 0.0, -1.0, surface)
    got = bike_native._debug_pneumatic_lumped_brush(0.3, 0.0, -1.0, sd,
                                                    None, 0.0)
    _assert_result_bitwise(got, want, 'zero load bad half')
    # Explicit zero stiffness -> (0, False, mu) even mid-range.
    want = brush.lumped_brush(0.4, 400.0, 0.05, surface,
                              tread_stiffness_n_m2=0.0)
    got = bike_native._debug_pneumatic_lumped_brush(0.4, 400.0, 0.05,
                                                    sd, 0.0, 0.0)
    _assert_result_bitwise(got, want, 'c_px==0 explicit')
    # half**2 overflows: CPython float_pow raises OverflowError at the
    # `** 2` in _tread_stiffness_n_m2 (derived path) or in theta
    # (explicit stiffness path). The port throws std::overflow_error,
    # which nanobind maps to OverflowError — same class.
    for tread in (None, 5.0e4):
        with pytest.raises(OverflowError):
            brush.lumped_brush(0.4, 400.0, 1e200, surface,
                               tread_stiffness_n_m2=tread)
        with pytest.raises(OverflowError):
            bike_native._debug_pneumatic_lumped_brush(
                0.4, 400.0, 1e200, sd, tread, 0.0)
    # theta nan via a zero-mu-like path is unreachable for valid
    # surfaces; the z==±inf saturation boundary (kappa <= -1 exact)
    # lands on copysign.
    want = brush.lumped_brush(-1.0, 300.0, 0.05, surface)
    got = bike_native._debug_pneumatic_lumped_brush(-1.0, 300.0, 0.05,
                                                    sd, None, 0.0)
    _assert_result_bitwise(got, want, 'kappa=-1 locked')
    assert want.fully_sliding


def test_lumped_brush_validation_order():
    """The staged finite gate: kappa must be *infinite* to defer to the
    load/half/slide gate; then load<0, then load==0 return, then
    half<=0, then the stiffness check."""
    surface = SURFACES['hardpack']
    sd = _surface_dict(surface)
    base = dict(normal_load_n=400.0, half_length_m=0.05,
                sliding_speed_mps=1.0)
    cases = [
        # (kappa, overrides, tread)
        (np.nan, {}, None),                    # first message
        (np.nan, {'normal_load_n': np.inf}, None),
        (np.inf, {'normal_load_n': np.inf}, None),   # second message
        (-np.inf, {'half_length_m': np.nan}, None),
        (np.inf, {'sliding_speed_mps': np.nan}, None),
        (np.inf, {'normal_load_n': -3.0}, None),     # load<0
        (0.2, {'normal_load_n': -3.0}, None),        # load<0
        (0.2, {'half_length_m': 0.0}, None),         # loaded, half<=0
        (0.2, {'half_length_m': -0.5}, None),
        (0.2, {}, np.nan),                            # tread invalid
        (0.2, {}, np.inf),
        (0.2, {}, -2.0),
        (np.inf, {}, np.nan),                         # tread after inf-kappa path
    ]
    for kappa, over, tread in cases:
        kw = {**base, **over}
        try:
            brush.lumped_brush(kappa, kw['normal_load_n'],
                               kw['half_length_m'], surface,
                               tread_stiffness_n_m2=tread,
                               sliding_speed_mps=kw['sliding_speed_mps'])
            oracle_raised = None
        except Exception as exc:  # noqa: BLE001 - record the class only
            oracle_raised = type(exc)
        try:
            bike_native._debug_pneumatic_lumped_brush(
                kappa, kw['normal_load_n'], kw['half_length_m'], sd,
                tread, kw['sliding_speed_mps'])
            native_raised = None
        except Exception as exc:  # noqa: BLE001
            native_raised = type(exc)
        assert native_raised is oracle_raised, \
            f'kappa={kappa!r} {over} tread={tread}: ' \
            f'{native_raised} != {oracle_raised}'
        assert oracle_raised is ValueError


def test_lumped_brush_inf_kappa_zero_load():
    """inf kappa + zero load returns cleanly through the early return;
    +inf kappa loaded is positive saturation, -inf negative."""
    surface = SURFACES['scree']
    sd = _surface_dict(surface)
    want = brush.lumped_brush(np.inf, 0.0, -1.0, surface)
    got = bike_native._debug_pneumatic_lumped_brush(np.inf, 0.0, -1.0,
                                                    sd, None, 0.0)
    _assert_result_bitwise(got, want, 'inf kappa zero load')
    for kappa in (np.inf, -np.inf):
        want = brush.lumped_brush(kappa, 250.0, 0.05, surface)
        got = bike_native._debug_pneumatic_lumped_brush(
            kappa, 250.0, 0.05, sd, None, 0.0)
        _assert_result_bitwise(got, want, f'kappa={kappa}')
        assert want.fully_sliding


# --------------------------------------------------------------------
# DiscretisedBrush.step — single-step coverage
# --------------------------------------------------------------------

@pytest.mark.parametrize('n', [1, 2, 3, 5, 9, 17, 64])
@pytest.mark.parametrize('surface', ALL_SURFACES)
def test_discretised_step_sweep_bitwise(n, surface):
    """Randomized loads and kinematics over every registered surface and
    element count; both the result and the post-state are bitwise."""
    sd = _surface_dict(surface)
    rng = np.random.default_rng(n * 131 + int(surface.mu_peak * 1000))
    oracle = brush.DiscretisedBrush(n)
    native_state = _oracle_state_dict(oracle)
    for step_idx in range(6):
        loads = rng.uniform(0.0, 900.0, n)
        # Sprinkle zero-load (inactive) bristles in — edges included.
        mask = rng.uniform(0.0, 1.0, n) < 0.25
        loads[mask] = 0.0
        if not np.any(loads > 0.0):
            loads[rng.integers(0, n)] = 10.0
        half = float(rng.uniform(0.004, 0.09))
        v_x = float(rng.uniform(-9.0, 9.0))
        v_s = float(rng.uniform(-3.0, 3.0))
        sigma = float(rng.uniform(0.0, 0.8))
        dt = float(rng.uniform(0.0002, 0.004))
        # The nonzero-low uniform transforms are wheel-contraction
        # divergent — replay the frozen scalars.
        _golden.record(np.array([half, v_x, v_s, dt]),
                       f'input.step {step_idx} scalars')
        want = oracle.step(loads, half_length_m=half, v_x_mps=v_x,
                           v_s_mps=v_s, sigma_m=sigma, surface=surface,
                           dt_s=dt)
        got = _native_step(loads, native_state, half, v_x, v_s, sigma,
                           sd, dt)
        _assert_result_bitwise(got, want, f'step {step_idx}')
        native_state = got['state']
        _assert_brush_state_bitwise(native_state, oracle,
                                    f'step {step_idx}')


def test_discretised_multi_step_evolution_bitwise():
    """A long staged sequence: rolling -> harder slip -> stop -> parked
    hold -> reverse, exercising advection shifts in both directions and
    the sigma-a input-filter switch."""
    n = 12
    surface = SURFACES['hardpack']
    sd = _surface_dict(surface)
    rng = np.random.default_rng(0xD15C)
    base_loads = np.linspace(60.0, 340.0, n)
    base_loads[:2] = 0.0          # inactive leading bristles
    base_loads[-1] = 0.0          # inactive trailing bristle
    half = 0.045
    program = [
        # (v_x, v_s, sigma, dt) — rolling drive, both advect signs.
        (4.0, -0.30, 0.50, 0.0005), (4.0, -0.30, 0.50, 0.0005),
        (4.0, -0.85, 0.50, 0.0005), (4.2, -1.60, 0.50, 0.0005),
        (3.0, -2.60, 0.50, 0.0005),   # strong slip -> partial/full slide
        (0.0, -0.10, 0.50, 0.0005),   # wheel rest: filtered = v_s
        (0.0, 0.00, 0.50, 0.0005),    # parked hold
        (0.0, 0.40, 0.50, 0.0010),    # parked push
        (-2.5, 0.35, 0.50, 0.0005),   # reverse rolling
        (-2.5, 0.60, 0.50, 0.0005),
        (4.0, 4.00, 0.50, 0.0005),    # v_x == v_s: verbatim copy + filter
        (4.0, -0.30, 0.02, 0.0005),   # sigma < a -> input_sigma = 0
        (4.0, -0.30, 0.045, 0.0005),  # sigma == a -> input_sigma = 0
        (4.0, -0.30, 0.50, 0.0005),
    ]
    steps = [(base_loads * (1.0 + 0.05 * rng.uniform(-1.0, 1.0, n)),
              half, v_x, v_s, sigma, sd, surface, dt)
             for v_x, v_s, sigma, dt in program]
    _run_brush_sequence(n, steps)


def test_discretised_advection_extremes_bitwise():
    """Shifts that push every source index out of the window on each
    side (left=0/right=0 sentinels), fractional bins, exact knots and
    the zero-wheel-speed copy — against a populated bristle row."""
    n = 7
    surface = SURFACES['asphalt']
    sd = _surface_dict(surface)
    rng = np.random.default_rng(77)
    bristles = rng.uniform(-0.02, 0.02, n)
    bristles[0] = -0.0           # signed-zero payload survives the copy
    # Wheel-contraction divergent — replay the frozen row.
    _golden.record(bristles, 'input.bristles')
    kappa0 = 0.11
    loads = np.linspace(50.0, 300.0, n)
    half = 0.035                  # dx = 0.07/7 = 0.01
    for v_x, v_s, dt in [
            (4.0, -0.3, 0.0025),   # shift +0.357 elements
            (-4.0, 0.3, 0.0025),   # shift -0.357
            (20.0, -1.0, 0.01),    # shift huge positive -> right=0
            (-20.0, 1.0, 0.01),    # huge negative -> left=0
            (3.0, 3.0, 0.0025),    # wheel speed 0 -> verbatim copy
            (0.0, 0.0, 0.0025),    # parked: copy + v_s integration
            (1e300, 0.0, 1e-2),    # shift overflows to +inf -> right=0
    ]:
        oracle = brush.DiscretisedBrush(n)
        oracle.bristle_deflection_m[:] = bristles
        oracle.kappa_prime = kappa0
        native_state = _state_dict(bristles, kappa0)
        want = oracle.step(loads, half_length_m=half, v_x_mps=v_x,
                           v_s_mps=v_s, sigma_m=0.5, surface=surface,
                           dt_s=dt)
        got = _native_step(loads, native_state, half, v_x, v_s, 0.5,
                           sd, dt)
        _assert_result_bitwise(got, want, f'advect v_x={v_x} v_s={v_s}')
        _assert_brush_state_bitwise(got['state'], oracle,
                                    f'advect v_x={v_x} v_s={v_s}')


def test_discretised_clip_boundary_bitwise():
    """|trial| == limit exactly: on the unit-law surface with loads=1,
    bristle u = mu/k lands trial exactly on the friction limit — the
    sliding mask's >= edge — and both bristles saturate together."""
    n = 2
    surface = UNIT_SURFACE
    sd = _surface_dict(surface)
    loads = np.ones(n)
    half = 0.5            # dx = 0.5; c_px = 0.5*2/(2*0.25) = 2; k = 1.0
    # v_x = v_s = 0 -> copy advection; filtered = 0 -> damping = 0;
    # updated = bristle. mu(0) = 1.0; k = 1.0; limit_i = 1.0.
    bristles = np.array([1.0, -1.0])
    oracle = brush.DiscretisedBrush(n)
    oracle.bristle_deflection_m[:] = bristles
    want = oracle.step(loads, half_length_m=half, v_x_mps=0.0,
                       v_s_mps=0.0, sigma_m=1.0, surface=surface,
                       dt_s=0.001)
    got = _native_step(loads, _state_dict(bristles, 0.0), half, 0.0,
                       0.0, 1.0, sd, 0.001)
    _assert_result_bitwise(got, want, 'clip boundary')
    assert want.fully_sliding, 'the exact-boundary bristles must slide'
    _assert_brush_state_bitwise(got['state'], oracle, 'clip boundary')

    # One bristle exactly at the limit, the other just inside ->
    # partial sliding (fully_sliding False), still bitwise.
    bristles = np.array([1.0, np.nextafter(-1.0, 0.0)])
    oracle = brush.DiscretisedBrush(n)
    oracle.bristle_deflection_m[:] = bristles
    want = oracle.step(loads, half_length_m=half, v_x_mps=0.0,
                       v_s_mps=0.0, sigma_m=1.0, surface=surface,
                       dt_s=0.001)
    got = _native_step(loads, _state_dict(bristles, 0.0), half, 0.0,
                       0.0, 1.0, sd, 0.001)
    _assert_result_bitwise(got, want, 'partial clip boundary')
    assert not want.fully_sliding
    _assert_brush_state_bitwise(got['state'], oracle,
                                'partial clip boundary')


def test_discretised_clip_sign_and_zero_edges():
    """Clip's operand order: force at the -limit side and signed-zero
    trial/limit payloads keep umath-clip semantics (the -0.0 lane is
    where clip differs from maximum-then-minimum composition)."""
    n = 3
    surface = UNIT_SURFACE
    sd = _surface_dict(surface)
    loads = np.ones(n)
    half = 0.5
    # Saturate one bristle at +limit, one at -limit, leave one at -0.0:
    # updated = bristle (v_x = v_s = 0 -> copy, filtered = 0 -> damping
    # 0), so trial = bristle*1.0; -0.0 clips as -0.0 < -limit? no:
    # clip(-0.0, -1.0, 1.0) -> -0.0 through _NPY_MAX/_NPY_MIN ties.
    bristles = np.array([5.0, -5.0, -0.0])
    oracle = brush.DiscretisedBrush(n)
    oracle.bristle_deflection_m[:] = bristles
    want = oracle.step(loads, half_length_m=half, v_x_mps=0.0,
                       v_s_mps=0.0, sigma_m=1.0, surface=surface,
                       dt_s=0.001)
    got = _native_step(loads, _state_dict(bristles, 0.0), half, 0.0,
                       0.0, 1.0, sd, 0.001)
    _assert_result_bitwise(got, want, 'clip signs')
    _assert_brush_state_bitwise(got['state'], oracle, 'clip signs')
    # The saturated bristles rewrite to (force - damping)/k = ±1.0.
    assert_bitwise_equal(np.asarray(got['state']['bristle_deflection_m']),
                         np.array([1.0, -1.0, -0.0]),
                         'write-back')


def test_discretised_zero_load_reset_bitwise():
    """total_load <= 0 or half_length <= 0 resets state and returns
    (0, False, mu) — a mutation, unlike the rejections."""
    n = 6
    surface = SURFACES['wet']
    sd = _surface_dict(surface)
    for loads, half in [
            (np.zeros(n), 0.04),           # all-inactive patch
            (np.linspace(1.0, 3.0, n), 0.0),   # half == 0 -> reset path
            (np.linspace(1.0, 3.0, n), -0.0),
    ]:
        oracle = brush.DiscretisedBrush(n)
        # Prime the state so the reset has something to clear.
        oracle.step(np.full(n, 10.0), half_length_m=0.04, v_x_mps=3.0,
                    v_s_mps=-0.5, sigma_m=0.4, surface=surface,
                    dt_s=0.001)
        assert np.any(oracle.bristle_deflection_m != 0.0)
        native_state = _oracle_state_dict(oracle)
        want = oracle.step(np.asarray(loads, dtype=float),
                           half_length_m=half, v_x_mps=3.0,
                           v_s_mps=-0.5, sigma_m=0.4, surface=surface,
                           dt_s=0.001)
        got = _native_step(loads, native_state, half, 3.0, -0.5, 0.4,
                           sd, 0.001)
        _assert_result_bitwise(got, want, f'reset half={half}')
        _assert_brush_state_bitwise(got['state'], oracle,
                                    f'reset half={half}')
        assert_bitwise_equal(
            np.asarray(got['state']['bristle_deflection_m']),
            np.zeros(n), 'reset clears bristles')
        assert_bitwise_equal(float(got['state']['kappa_prime']), 0.0,
                             'reset clears kappa')


def test_discretised_state_nan_propagation():
    """A NaN bristle seeded into the state propagates through the
    unit-grid interp (including its forward-point NaN fallback) and the
    clip — bitwise, payload included."""
    n = 5
    surface = SURFACES['asphalt']
    sd = _surface_dict(surface)
    bristles = np.array([0.01, np.nan, -0.02, 0.015, 0.0])
    loads = np.linspace(80.0, 240.0, n)
    oracle = brush.DiscretisedBrush(n)
    oracle.bristle_deflection_m[:] = bristles
    oracle.kappa_prime = 0.05
    for step_idx, (v_x, v_s) in enumerate([(4.0, -0.3), (0.0, 0.0),
                                           (-3.0, 0.2)]):
        want = oracle.step(loads, half_length_m=0.05, v_x_mps=v_x,
                           v_s_mps=v_s, sigma_m=0.5, surface=surface,
                           dt_s=0.001)
        if step_idx == 0:
            got = _native_step(loads, _state_dict(bristles, 0.05), 0.05,
                               v_x, v_s, 0.5, sd, 0.001)
            native_state = got['state']
        else:
            got = _native_step(loads, native_state, 0.05, v_x, v_s,
                               0.5, sd, 0.001)
            native_state = got['state']
        _assert_result_bitwise(got, want, f'nan bristle step {step_idx}')
        _assert_brush_state_bitwise(native_state, oracle,
                                    f'nan bristle step {step_idx}')


# --------------------------------------------------------------------
# DiscretisedBrush.step — validation order and construction
# --------------------------------------------------------------------

def test_discretised_validation_order_and_atomicity():
    """Every rejection class in the oracle's check order; a failed call
    leaves the oracle's state untouched, and the follow-up valid step
    matches bitwise on both sides."""
    n = 8
    surface = SURFACES['hardpack']
    sd = _surface_dict(surface)
    loads = np.linspace(40.0, 300.0, n)
    kw = dict(half_length_m=0.04, v_x_mps=4.0, v_s_mps=-0.3,
              sigma_m=0.5, dt_s=0.001)
    oracle = brush.DiscretisedBrush(n)
    oracle.step(loads, surface=surface, **kw)   # nonzero state to lose
    native_state = _oracle_state_dict(oracle)
    snap = _oracle_state_dict(oracle)

    def oracle_step(l, **over):
        oracle.step(np.asarray(l, dtype=float),
                    **{**kw, **over}, surface=surface)

    def native_step(l, state, **over):
        args = {**kw, **over}
        return _native_step(l, state, args['half_length_m'],
                            args['v_x_mps'], args['v_s_mps'],
                            args['sigma_m'], sd, args['dt_s'])

    # Level 1 — loads shape.
    with pytest.raises(ValueError, match='normal loads'):
        oracle_step(loads[:-1])
    with pytest.raises(ValueError):
        native_step(loads[:-1], native_state)
    _assert_brush_state_bitwise(native_state, oracle,
                                'shape rejection atomicity')

    # Level 2 — loads content (finite + nonnegative share one message).
    for bad_loads in (np.where(np.arange(n) == 3, np.nan, loads),
                      np.where(np.arange(n) == 3, np.inf, loads),
                      np.where(np.arange(n) == 3, -1e-9, loads),
                      np.where(np.arange(n) == 3, -np.inf, loads)):
        with pytest.raises(ValueError, match='finite and nonnegative'):
            oracle_step(bad_loads)
        with pytest.raises(ValueError):
            native_step(bad_loads, native_state)
    _assert_brush_state_bitwise(native_state, oracle,
                                'loads rejection atomicity')

    # Level 3 — scalar finite gate beats the domain gate below it.
    for over in (dict(half_length_m=np.nan), dict(v_x_mps=np.inf),
                 dict(v_s_mps=-np.inf), dict(sigma_m=np.nan),
                 dict(dt_s=np.inf),
                 dict(half_length_m=np.nan, sigma_m=-1.0, dt_s=0.0)):
        with pytest.raises(ValueError, match='finite'):
            oracle_step(loads, **over)
        with pytest.raises(ValueError):
            native_step(loads, native_state, **over)

    # Level 4 — domain gate: half < 0, sigma < 0, dt <= 0.
    for over in (dict(half_length_m=-1e-9), dict(sigma_m=-0.25),
                 dict(dt_s=0.0), dict(dt_s=-0.0), dict(dt_s=-1.0)):
        with pytest.raises(ValueError, match='nonnegative|positive'):
            oracle_step(loads, **over)
        with pytest.raises(ValueError):
            native_step(loads, native_state, **over)
    _assert_brush_state_bitwise(native_state, oracle,
                                'domain rejection atomicity')

    # A non-finite kappa_prime injected into the state reaches relax's
    # own finite gate — after every step-level check has passed.
    corrupt = _state_dict(snap['bristle_deflection_m'], np.nan)
    with pytest.raises(ValueError):
        native_step(loads, corrupt)
    # Same on the oracle side.
    saved = oracle.kappa_prime
    oracle.kappa_prime = np.nan
    with pytest.raises(ValueError, match='finite'):
        oracle_step(loads)
    oracle.kappa_prime = saved

    # The follow-up valid step still matches bitwise.
    want = oracle.step(loads, surface=surface, **kw)
    got = native_step(loads, native_state)
    _assert_result_bitwise(got, want, 'post-failure')
    _assert_brush_state_bitwise(got['state'], oracle, 'post-failure')


def test_discretised_pow_overflow_mid_step_commit():
    """half=1e200 -> `half ** 2` raises OverflowError at the c_px line —
    AFTER the oracle already committed kappa_prime through relax() but
    before the bristle row commits. The oracle keeps the advanced
    kappa_prime; bristles stay stale. (The native binding cannot expose
    a mid-failure object — the raise class is what crosses the FFI.)"""
    n = 4
    surface = SURFACES['asphalt']
    sd = _surface_dict(surface)
    loads = np.linspace(60.0, 200.0, n)
    oracle = brush.DiscretisedBrush(n)
    oracle.step(loads, half_length_m=0.04, v_x_mps=3.0, v_s_mps=-0.2,
                sigma_m=0.4, surface=surface, dt_s=0.001)
    before = _oracle_state_dict(oracle)
    with pytest.raises(OverflowError):
        oracle.step(loads, half_length_m=1e200, v_x_mps=3.0,
                    v_s_mps=-0.2, sigma_m=0.4, surface=surface,
                    dt_s=0.001)
    # kappa_prime advanced through relax (input_sigma = max(0, .4-1e200)
    # = 0 -> algebraic branch), bristles byte-identical.
    assert_bitwise_equal(
        np.asarray(oracle.bristle_deflection_m),
        before['bristle_deflection_m'], 'bristles untouched')
    assert oracle.kappa_prime != before['kappa_prime']
    expected_kappa = brush.relax(before['kappa_prime'], 3.0, -0.2, 0.0,
                                 0.001)
    assert_bitwise_equal(oracle.kappa_prime, expected_kappa,
                         'kappa committed before the pow raise')
    with pytest.raises(OverflowError):
        _native_step(loads, before, 1e200, 3.0, -0.2, 0.4, sd, 0.001)


def test_discretised_construction_rejection():
    """n_elements < 1 raises on both sides (through the state dict
    intake for the native kernel)."""
    with pytest.raises(ValueError, match='at least 1 element'):
        brush.DiscretisedBrush(0)
    loads = np.empty(0)
    state = _state_dict(np.empty(0), 0.0)
    with pytest.raises(ValueError):
        _native_step(loads, state, 0.04, 3.0, -0.2, 0.4,
                     _surface_dict(SURFACES['asphalt']), 0.001)


def test_discretised_partial_inactive_patch_bitwise():
    """A lopsided patch: only interior bristles loaded. Inactive
    bristles are zeroed by the where() mask and contribute no force;
    fully_sliding is computed over the active subset only."""
    n = 9
    surface = SURFACES['loose']
    sd = _surface_dict(surface)
    loads = np.zeros(n)
    loads[3:6] = (120.0, 240.0, 90.0)
    oracle = brush.DiscretisedBrush(n)
    native_state = _oracle_state_dict(oracle)
    for step_idx, (v_x, v_s, dt) in enumerate(
            [(5.0, -0.4, 0.0005), (5.0, -1.2, 0.0005),
             (5.0, -4.0, 0.0005), (-3.0, 0.8, 0.0005),
             (0.0, 0.0, 0.0005)]):
        want = oracle.step(loads, half_length_m=0.05, v_x_mps=v_x,
                           v_s_mps=v_s, sigma_m=0.6, surface=surface,
                           dt_s=dt)
        got = _native_step(loads, native_state, 0.05, v_x, v_s, 0.6,
                           sd, dt)
        _assert_result_bitwise(got, want, f'partial {step_idx}')
        native_state = got['state']
        _assert_brush_state_bitwise(native_state, oracle,
                                    f'partial {step_idx}')
    # Hard slip: every active bristle saturates -> fully_sliding.
    want = oracle.step(loads, half_length_m=0.05, v_x_mps=5.0,
                       v_s_mps=-30.0, sigma_m=0.6, surface=surface,
                       dt_s=0.02)
    got = _native_step(loads, native_state, 0.05, 5.0, -30.0, 0.6, sd,
                       0.02)
    _assert_result_bitwise(got, want, 'partial full-slide')
    assert want.fully_sliding
    _assert_brush_state_bitwise(got['state'], oracle,
                                'partial full-slide')


def test_discretised_single_element_bitwise():
    """n=1: interp's j == n-1 last-knot branch and the single-element
    sliding decision."""
    surface = SURFACES['asphalt']
    sd = _surface_dict(surface)
    oracle = brush.DiscretisedBrush(1)
    native_state = _oracle_state_dict(oracle)
    for step_idx, (v_x, v_s, sigma, dt) in enumerate(
            [(4.0, -0.3, 0.5, 0.0005), (4.0, -3.0, 0.5, 0.001),
             (0.0, 0.2, 0.5, 0.0005), (-2.0, 0.5, 0.0, 0.0005),
             (8.0, -9.0, 0.5, 0.002)]):
        loads = np.array([260.0])
        want = oracle.step(loads, half_length_m=0.03, v_x_mps=v_x,
                           v_s_mps=v_s, sigma_m=sigma, surface=surface,
                           dt_s=dt)
        got = _native_step(loads, native_state, 0.03, v_x, v_s, sigma,
                           sd, dt)
        _assert_result_bitwise(got, want, f'n=1 step {step_idx}')
        native_state = got['state']
        _assert_brush_state_bitwise(native_state, oracle,
                                    f'n=1 step {step_idx}')


def test_discretised_zero_kappa_inf_products():
    """element_stiffness == 0.0 (via a zero slip_stiffness... not
    constructible on a valid surface) — instead pin k == 0 via the
    unreachable-check side: slip_stiffness stays positive, so drive the
    damping/trial arithmetic where filtered_v_s and dt make the growth
    exact zero, keeping IEEE -0.0 payloads bitwise."""
    n = 4
    surface = SURFACES['wet']
    sd = _surface_dict(surface)
    loads = np.linspace(50.0, 150.0, n)
    oracle = brush.DiscretisedBrush(n)
    native_state = _oracle_state_dict(oracle)
    # v_x nonzero, v_s = 0 -> filtered = -kappa*|v_x|; kappa stays 0 at
    # sigma 0 -> filtered -0.0 products; then v_x == 0 -> filtered = v_s.
    for v_x, v_s, sigma in [(4.0, 0.0, 0.0), (4.0, -0.0, 0.0),
                            (0.0, -0.0, 0.5), (0.0, 0.0, 0.5),
                            (-4.0, -0.0, 0.3)]:
        want = oracle.step(loads, half_length_m=0.04, v_x_mps=v_x,
                           v_s_mps=v_s, sigma_m=sigma, surface=surface,
                           dt_s=0.001)
        got = _native_step(loads, native_state, 0.04, v_x, v_s, sigma,
                           sd, 0.001)
        _assert_result_bitwise(got, want, f'v_x={v_x} v_s={v_s}')
        native_state = got['state']
        _assert_brush_state_bitwise(native_state, oracle,
                                    f'v_x={v_x} v_s={v_s}')
