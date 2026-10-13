"""Bitwise oracle tests for the native pneumatic tyre model.

Covers the Task-10 debug surfaces in ``bike_native``:

* ``_debug_pneumatic_evaluate``            — ``PneumaticTyre.evaluate``:
  intersect → ``evaluate_carcass`` → patch-state migration → lumped or
  discretised brush → wheel outputs, plus the post-call
  ``{carcass, patch_states}`` snapshot; the optional ``pressure`` key
  in the ``step`` dict runs ``set_pressure`` before the step
* ``_debug_pneumatic_model_set_pressure``  — ``set_pressure``'s clamp
* ``_debug_pneumatic_model_reset``         — ``reset`` on restored
  state: the ``WheelOutputs.empty`` snapshot plus cleared state

Every floating-point scalar and array is compared raw-byte-for-raw-byte
against the pure-Python oracle in ``src/bike_sim/sim/ride/tyre/model.py``
— ``tobytes()`` keeps signed zeros, infinities and NaN payloads
distinguishable.  The state dict carries every ``CarcassState`` field
plus the ordered ``_patch_states`` slots (``kappa_prime`` and, under the
discretised tier, the detailed brush's bristle row and its own
``kappa_prime``), so patch-state migration is pinned bitwise alongside
each step's outputs.

The corpus exercises every oracle path in ``evaluate``: the airborne
clean-state fast path (``advance_unloaded`` + patch-state clear), the
loaded-then-airborne no-patch path (unloaded outputs carrying rim
state), the loaded path end to end in both brush modes, exact-key and
max-overlap patch-state migration with bristle copying — including the
used-set drop and the slice-broadcast rejection — multi-patch roads,
rim-strike events, the tread rolling loss in every ``np.sign``
combination including the stationary fallback and the rest taper, the
pressure-dependent Crr power, ``set_pressure`` clamping, ``reset``, and
the input-validation order with failure atomicity.
"""

import numpy as np
import pytest
from dataclasses import replace

from bike_sim.physics.tyre import (
    FRONT_TYRE,
    PRESSURE_MAX_BAR,
    PRESSURE_MIN_BAR,
    REAR_TYRE,
    TIERS,
    TierSpec,
    clamp_pressure_bar,
)
from bike_sim.sim.ride.tyre import carcass, geometry
from bike_sim.sim.ride.tyre.brush import DiscretisedBrush
from bike_sim.sim.ride.tyre.model import (
    PneumaticTyre,
    _PatchBrushState,
    _tread_loss_crr_at_pressure,
    _tread_loss_force,
)
from bike_sim.terrain.surface import (
    SurfaceMap,
    SurfaceSection,
    get_surface,
)

from native_loader import load_native
bike_native = load_native()
from _bits import assert_bitwise_equal

FAST = TIERS['fast']
DETAIL = TIERS['detailed']
# A discretised tier at the fast ray count keeps the corpus cheap while
# still exercising per-element bristle migration.
DETAIL64 = TierSpec('detail64', n_rays=64, half_angle_deg=75.0,
                    timestep_s=0.0005, discretised_brush=True)
FAST256 = TierSpec('fast256', n_rays=256, half_angle_deg=75.0,
                   timestep_s=0.00025, discretised_brush=False)

_SPEC_KEYS = (
    'name', 'outer_radius_mm', 'rim_radius_mm', 'section_height_mm',
    'pressure_bar', 'crr_reference', 'casing_width_mm',
    'compressed_casing_mm', 'area_factor', 'carcass_stiffness_n_mm2',
    'contact_length_factor', 'rate_stiffening', 'rate_relaxation_s',
    'loss_factor', 'hysteresis_rate_eps_mps', 'relaxation_length_mm',
    'rim_stiffness_n_mm2', 'tread_loss_crr',
)
_TIER_KEYS = ('name', 'n_rays', 'half_angle_deg', 'timestep_s',
              'discretised_brush')
_SURFACE_KEYS = ('name', 'mu_peak', 'mu_slide', 'slip_stiffness_per_load',
                 'stribeck_speed_mps')

_CARCASS_ARRAY_KEYS = ('delta_prev_m', 'elastic_prev_n', 'maxwell_force_n')
_CARCASS_BOOL_KEYS = ('initialized', 'rim_strike_active')
_CARCASS_FLOAT_KEYS = (
    'rim_event_x_m', 'rim_event_speed_mps', 'rim_event_peak_load_n',
    'rim_event_peak_force_n', 'rim_event_energy_j',
)
_RIM_EVENT_KEYS = ('x_m', 'speed_mps', 'peak_load_n', 'peak_rim_force_n',
                   'absorbed_energy_j')
_WHEEL_ARRAY_KEYS = ('hub_position_world_m', 'hub_velocity_world_mps',
                     'force_world_n')
_WHEEL_FLOAT_KEYS = ('pressure_bar', 'omega_forward_radps',
                     'normal_load_n', 'support_n', 'mean_deflection_m',
                     'contact_length_m', 'slip_ratio',
                     'transient_slip_ratio', 'dissipated_power_w')
_WHEEL_BOOL_KEYS = ('fully_sliding', 'rim_strike_active',
                    'coverage_event', 'airborne')
_PATCH_ARRAY_KEYS = ('normal_force_world_n', 'tangent_world',
                     'force_world_n', 'centroid_world_m')
_PATCH_FLOAT_KEYS = ('normal_load_n', 'support_n', 'tangential_force_n',
                     'mean_deflection_m', 'contact_length_m', 'slip_ratio',
                     'transient_slip_ratio', 'sliding_speed_mps',
                     'slip_speed_mps', 'friction_coefficient',
                     'dissipated_power_w')


def _spec_dict(tyre):
    """The 18-field TyreSpecs wire dict."""
    return {k: getattr(tyre, k) for k in _SPEC_KEYS}


def _tier_dict(tier):
    return {k: getattr(tier, k) for k in _TIER_KEYS}


def _profile_dict(road):
    """A geometry.RoadProfile as the {x0_m, dx_m, z_m} wire dict."""
    return {'x0_m': float(road.x0_m), 'dx_m': float(road.dx_m),
            'z_m': np.asarray(road.z_m, dtype=float)}


def _surface_dict(spec):
    return {k: getattr(spec, k) for k in _SURFACE_KEYS}


def _surface_map_dict(surface_map):
    """A Python SurfaceMap as the {surface, sections} wire dict — the
    wire carries each section's resolved SurfaceSpec, not its registry
    name."""
    return {
        'surface': _surface_dict(surface_map._surface),
        'sections': [
            {'start_m': s.start_m, 'end_m': s.end_m,
             'surface': _surface_dict(get_surface(s.surface))}
            for s in surface_map.sections
        ],
    }


def _oracle_carcass_state_dict(state):
    d = {k: np.asarray(getattr(state, k), dtype=float)
         for k in _CARCASS_ARRAY_KEYS}
    d.update({k: bool(getattr(state, k)) for k in _CARCASS_BOOL_KEYS})
    d.update({k: float(getattr(state, k)) for k in _CARCASS_FLOAT_KEYS})
    return d


def _oracle_state_dict(model):
    """The model's full semantic state as a wire dict."""
    return {
        'carcass': _oracle_carcass_state_dict(model.carcass_state),
        'patch_states': [
            _patch_state_wire(s.start, s.stop, s.kappa_prime,
                              None if s.detailed is None else
                              s.detailed.bristle_deflection_m,
                              0.0 if s.detailed is None else
                              s.detailed.kappa_prime)
            for s in model._patch_states.values()],
    }


def _fresh_state_dict(n_rays):
    return {'carcass': _oracle_carcass_state_dict(
        carcass.CarcassState(n_rays)),
        'patch_states': []}


def _patch_state_wire(start, stop, kappa_prime=0.0, bristle=None,
                      detailed_kappa_prime=0.0):
    """A patch-state wire entry; ``bristle=None`` is the oracle's
    ``detailed is None``."""
    return {'start': int(start), 'stop': int(stop),
            'kappa_prime': float(kappa_prime),
            'detailed': None if bristle is None else {
                'bristle_deflection_m': np.asarray(bristle, dtype=float),
                'kappa_prime': float(detailed_kappa_prime)}}


def _injected_state(start, stop, kappa_prime=0.0, bristle=None,
                    detailed_kappa_prime=0.0):
    """The oracle-side twin of _patch_state_wire: a real
    _PatchBrushState with an optional free-standing DiscretisedBrush."""
    st = _PatchBrushState(start, stop, kappa_prime=kappa_prime)
    if bristle is not None:
        bristle = np.asarray(bristle, dtype=float)
        st.detailed = DiscretisedBrush(bristle.size)
        st.detailed.bristle_deflection_m[:] = bristle
        st.detailed.kappa_prime = detailed_kappa_prime
    return st


def _assert_rim_event(got_ev, want_ev, msg):
    if want_ev is None:
        assert got_ev is None, f'{msg} rim_event'
        return
    assert got_ev is not None, f'{msg} rim_event'
    for k in _RIM_EVENT_KEYS:
        assert_bitwise_equal(float(got_ev[k]), float(getattr(want_ev, k)),
                             f'{msg} rim_event.{k}')


def _assert_outputs_bitwise(got, want, msg=''):
    assert got['tyre_name'] == want.tyre_name, f'{msg} tyre_name'
    for k in _WHEEL_ARRAY_KEYS:
        assert_bitwise_equal(np.asarray(got[k]), getattr(want, k),
                             f'{msg} {k}')
    for k in _WHEEL_FLOAT_KEYS:
        assert_bitwise_equal(float(got[k]), float(getattr(want, k)),
                             f'{msg} {k}')
    for k in _WHEEL_BOOL_KEYS:
        assert bool(got[k]) == bool(getattr(want, k)), f'{msg} {k}'
    _assert_rim_event(got['rim_event'], want.rim_event, msg)
    assert len(got['patches']) == len(want.patches), \
        f'{msg} patches {len(got["patches"])} != {len(want.patches)}'
    for i, (gp, wp) in enumerate(zip(got['patches'], want.patches)):
        pmsg = f'{msg} patches[{i}]'
        assert gp['ray_start'] == wp.ray_start, f'{pmsg} ray_start'
        assert gp['ray_stop'] == wp.ray_stop, f'{pmsg} ray_stop'
        assert gp['surface_name'] == wp.surface_name, \
            f'{pmsg} surface_name'
        for k in _PATCH_ARRAY_KEYS:
            assert_bitwise_equal(np.asarray(gp[k]), getattr(wp, k),
                                 f'{pmsg} {k}')
        for k in _PATCH_FLOAT_KEYS:
            assert_bitwise_equal(float(gp[k]), float(getattr(wp, k)),
                                 f'{pmsg} {k}')
        assert bool(gp['fully_sliding']) == bool(wp.fully_sliding), \
            f'{pmsg} fully_sliding'


def _assert_carcass_state_bitwise(native_state, oracle_state, msg=''):
    for k in _CARCASS_ARRAY_KEYS:
        assert_bitwise_equal(np.asarray(native_state[k]),
                             getattr(oracle_state, k),
                             f'{msg} carcass.{k}')
    for k in _CARCASS_BOOL_KEYS:
        assert bool(native_state[k]) == bool(getattr(oracle_state, k)), \
            f'{msg} carcass.{k}'
    for k in _CARCASS_FLOAT_KEYS:
        assert_bitwise_equal(float(native_state[k]),
                             float(getattr(oracle_state, k)),
                             f'{msg} carcass.{k}')


def _assert_state_bitwise(native_state, oracle_model, msg=''):
    _assert_carcass_state_bitwise(native_state['carcass'],
                                  oracle_model.carcass_state, msg)
    got = native_state['patch_states']
    want = _oracle_state_dict(oracle_model)['patch_states']
    assert len(got) == len(want), \
        f'{msg} patch_states {len(got)} != {len(want)}'
    for i, (gs, ws) in enumerate(zip(got, want)):
        smsg = f'{msg} patch_states[{i}]'
        assert gs['start'] == ws['start'] and gs['stop'] == ws['stop'], \
            f'{smsg} key'
        assert_bitwise_equal(float(gs['kappa_prime']),
                             float(ws['kappa_prime']),
                             f'{smsg} kappa_prime')
        if ws['detailed'] is None:
            assert gs['detailed'] is None, f'{smsg} detailed'
        else:
            assert gs['detailed'] is not None, f'{smsg} detailed'
            assert_bitwise_equal(
                np.asarray(gs['detailed']['bristle_deflection_m']),
                ws['detailed']['bristle_deflection_m'],
                f'{smsg} detailed.bristle_deflection_m')
            assert_bitwise_equal(
                float(gs['detailed']['kappa_prime']),
                float(ws['detailed']['kappa_prime']),
                f'{smsg} detailed.kappa_prime')


def _native_evaluate(spec_d, tier_d, profile_d, smap_d, state,
                     hub, velocity, omega, dt, pressure=None):
    step = {
        'hub': np.asarray(hub, dtype=float),
        'velocity': np.asarray(velocity, dtype=float),
        'omega_forward_radps': float(omega),
        'dt_s': float(dt),
        'pressure': pressure,
    }
    return bike_native._debug_pneumatic_evaluate(
        spec_d, tier_d, profile_d, smap_d, state, step)


def _run_sequence(tyre, tier, road, surface_map, rows, check_step=None):
    """Drive one identical multi-step sequence through the oracle model
    and the native debug surface; compare every wheel-output field and
    every post-call state byte-for-byte.

    ``rows`` is a list of (hub, velocity, omega, dt) tuples, or
    (hub, velocity, omega, dt, pressure) to run set_pressure first.
    """
    oracle = PneumaticTyre(tyre, tier)
    native_state = _fresh_state_dict(tier.n_rays)
    spec_d, tier_d = _spec_dict(tyre), _tier_dict(tier)
    profile_d, smap_d = (_profile_dict(road),
                         _surface_map_dict(surface_map))
    for idx, row in enumerate(rows):
        hub, velocity, omega, dt = row[:4]
        pressure = row[4] if len(row) > 4 else None
        hub, velocity = (np.asarray(hub, dtype=float),
                         np.asarray(velocity, dtype=float))
        tag = f'step {idx}'
        if pressure is not None:
            oracle.set_pressure(pressure)
            # set_pressure persists on the oracle's tyre spec — the
            # native model is rebuilt per call, so the clamped value
            # has to persist through the spec dict too (the evaluate
            # binding still runs its own set_pressure, clamped again
            # to the same value).
            spec_d = dict(spec_d)
            spec_d['pressure_bar'] = clamp_pressure_bar(pressure)
        want = oracle.evaluate(hub, velocity, omega, road, surface_map,
                               dt)
        got = _native_evaluate(spec_d, tier_d, profile_d, smap_d,
                               native_state, hub, velocity, omega, dt,
                               pressure)
        _assert_outputs_bitwise(got, want, tag)
        native_state = got['state']
        _assert_state_bitwise(native_state, oracle, tag)
        if check_step is not None:
            check_step(got, want, oracle, tag)
    return oracle, native_state


# --------------------------------------------------------------------
# Shared roads / surface maps / kinematic drivers
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


def _twin_bump_road():
    """Two bumps split by a trough that falls below zero — a single
    patch splits in two while the trough sits under the hub."""
    x = np.arange(1201) * 0.01
    z = (0.030 * np.exp(-((x - 4.90) / 0.12) ** 2)
         + 0.030 * np.exp(-((x - 5.20) / 0.12) ** 2)
         - 0.020 * np.exp(-((x - 5.05) / 0.05) ** 2))
    return geometry.RoadProfile(x0_m=0.0, dx_m=0.01, z_m=z)


def _spike_road():
    """A narrow kerb taller than the hub — road inside the tyre circle
    but beyond the ±75° ray coverage (and above the hub centre):
    geometry.intersect's coverage_event triggers."""
    x = np.arange(1201) * 0.01
    z = 0.45 * np.exp(-((x - 5.30) / 0.05) ** 2)
    return geometry.RoadProfile(x0_m=0.0, dx_m=0.01, z_m=z)


def _uniform_map(name='asphalt'):
    return SurfaceMap.uniform(name)


def _zoned_map():
    """A hardpack base with a loose and a scree zone — the patch
    centroid crosses section boundaries mid-sequence."""
    return SurfaceMap(
        get_surface('hardpack'),
        [SurfaceSection(4.6, 5.1, 'loose'),
         SurfaceSection(5.1, 5.6, 'scree')])


def _rows(xs, zs, omegas, dts=None, speed=3.0):
    """(hub, velocity, omega, dt) rows at a constant forward velocity
    — the wheel slides when x advances slower than speed*dt."""
    dts = dts if dts is not None else [0.002] * len(xs)
    return [(np.array([x, 0.0, z]), np.array([speed, 0.0, 0.0]), o, dt)
            for x, z, o, dt in zip(xs, zs, omegas, dts)]


# --------------------------------------------------------------------
# Loaded sequences — both brush modes
# --------------------------------------------------------------------

@pytest.mark.parametrize('tyre', [FRONT_TYRE, REAR_TYRE])
@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_progressive_load_unload_bitwise(tyre, tier):
    """Hub descends over a bump, holds, rises again — the loaded path
    with patch keys held across steady frames and drifted across
    moving ones."""
    road = _bump_road()
    radius = tyre.outer_radius_mm * 0.001
    xs = [4.6 + 0.03 * k for k in range(16)]
    zs = [radius + 0.10 - min(k * 0.016, 0.140) for k in range(16)]
    zs += [radius - 0.030, radius - 0.028, radius - 0.020, radius + 0.02,
           radius + 0.09]
    xs += [xs[-1] + 0.03 * k for k in range(1, 6)]
    omegas = [9.5 if k % 5 < 3 else -6.0 for k in range(len(xs))]
    rows = _rows(xs, zs, omegas)
    _run_sequence(tyre, tier, road, _uniform_map(), rows)


@pytest.mark.parametrize('tier', [FAST256, DETAIL])
def test_model_authored_tiers_bitwise(tier):
    """The two authored TIERS entries end to end — 256 rays, including
    the discretised brush at full width."""
    tyre = FRONT_TYRE
    road = _bump_road()
    radius = tyre.outer_radius_mm * 0.001
    xs = [4.8 + 0.02 * k for k in range(10)]
    zs = [radius - 0.018 - 0.002 * (k % 4) for k in range(10)]
    rows = _rows(xs, zs, [8.0] * 10, dts=[tier.timestep_s] * 10)
    _run_sequence(tyre, tier, road, _uniform_map(), rows)


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_airborne_sequence_bitwise(tier):
    """Airborne on a clean state (the advance_unloaded fast path),
    landing, liftoff through evaluate_carcass, then the initialized
    airborne fast path that advects Maxwell memory."""
    tyre = FRONT_TYRE
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    hi, lo = radius + 0.40, radius - 0.018
    rows = _rows([5.0] * 8, [hi, hi, lo, lo, hi, hi, hi, lo],
                 [7.0] * 8, speed=4.0)

    def check(_got, want, oracle, tag):
        if tag in ('step 4', 'step 5', 'step 6'):
            # Liftoff through the carcass path (step 4), then the
            # initialized airborne fast path (5, 6).
            assert want.airborne and want.patches == ()
            assert not oracle._patch_states

    _run_sequence(tyre, tier, road, _uniform_map(), rows, check)


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_mixed_velocity_components_bitwise(tier):
    """Nonzero lateral and vertical hub velocity: v_y rides through
    the output echo and v_z enters speed_mps = hypot(v0, v2) and the
    v_x tangent projection."""
    tyre = FRONT_TYRE
    road = _bump_road()
    radius = tyre.outer_radius_mm * 0.001
    lo = radius - 0.025
    rows = [
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.4, 0.0]), 9.0, 0.002),
        (np.array([5.0, 0.1, lo]), np.array([2.0, -0.3, 0.8]), 9.0,
         0.002),
        (np.array([5.0, 0.0, lo]), np.array([1.5, 0.0, -1.2]), 9.0,
         0.002),
        (np.array([5.0, 0.0, lo]), np.array([0.0, 0.0, 2.0]), 9.0, 0.002),
        (np.array([5.0, 0.0, lo]), np.array([0.0, -0.7, 0.0]), -4.0,
         0.002),
    ]
    _run_sequence(tyre, tier, road, _uniform_map(), rows)


# --------------------------------------------------------------------
# Patch-state migration
# --------------------------------------------------------------------

def _patch_keys(model):
    return tuple(model._patch_states.keys())


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_patch_migration_drift_bitwise(tier):
    """Steady frames hold the patch key (exact reuse); a bumpy roll
    drifts it by a ray at a time — every migration takes the
    max-positive-overlap branch and, under the discretised tier,
    copies the bristle overlap by ray index."""
    tyre = FRONT_TYRE
    road = _bump_road()
    radius = tyre.outer_radius_mm * 0.001
    # Four identical frames guarantee exact-key reuse, then a bumpy
    # descent drifts the key.
    xs = [4.80] * 4 + [4.80 + 0.006 * k for k in range(1, 27)]
    zs = [radius - 0.030] * 4 + [radius - 0.030 - 0.001 * (k % 3)
                                 for k in range(1, 27)]
    rows = _rows(xs, zs, [10.0] * 30)

    keys = []

    def check(_got, _want, oracle, _tag):
        keys.append(_patch_keys(oracle))

    _run_sequence(tyre, tier, road, _uniform_map(), rows, check)
    shifts = sum(1 for a, b in zip(keys, keys[1:]) if a != b)
    same = sum(1 for a, b in zip(keys, keys[1:]) if a == b)
    assert shifts >= 2, f'patch never drifted: {keys}'
    assert same >= 1, f'patch never held: {keys}'


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_patch_split_and_merge_bitwise(tier):
    """The trough road splits one patch into two under a shallow load
    and re-merges it as the hub moves on — the second new patch that
    overlaps an already-used old state starts fresh."""
    tyre = FRONT_TYRE
    road = _twin_bump_road()
    radius = tyre.outer_radius_mm * 0.001
    xs = [4.55 + 0.02 * k for k in range(34)]
    # Shallow: the sub-zero trough unloads the central rays.
    zs = [radius - 0.006 - 0.001 * (k % 4) for k in range(34)]
    rows = _rows(xs, zs, [8.5] * 34, speed=1.0)

    counts = []

    def check(_got, want, _oracle, _tag):
        counts.append(len(want.patches))

    _run_sequence(tyre, tier, road, _uniform_map(), rows, check)
    assert 2 in counts, f'never two patches: {counts}'
    assert 1 in counts, f'never one patch: {counts}'


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_patch_no_overlap_drops_stale(tier):
    """An injected stale state at a disjoint key is dropped by the
    migration — the live patch starts fresh rather than migrating
    nothing-overlapping slip memory."""
    tyre = FRONT_TYRE
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    smap = _uniform_map()
    hub = np.array([5.0, 0.0, radius - 0.025])
    vel = np.array([3.0, 0.0, 0.0])

    oracle = PneumaticTyre(tyre, tier)
    oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    live = _patch_keys(oracle)
    assert len(live) == 1
    (s0, e0), = live
    stale = (max(0, s0 - 8), max(1, s0 - 5))
    assert stale[1] <= s0  # disjoint from the live patch

    oracle2 = PneumaticTyre(tyre, tier)
    oracle2._patch_states[stale] = _injected_state(
        *stale, kappa_prime=0.7,
        bristle=np.linspace(0.01, 0.02, stale[1] - stale[0])
        if tier.discretised_brush else None,
        detailed_kappa_prime=0.7)
    native_state = {
        'carcass': _oracle_carcass_state_dict(oracle2.carcass_state),
        'patch_states': [
            _patch_state_wire(
                *stale, kappa_prime=0.7,
                bristle=np.linspace(0.01, 0.02, stale[1] - stale[0])
                if tier.discretised_brush else None,
                detailed_kappa_prime=0.7)]}
    want = oracle2.evaluate(hub, vel, 9.0, road, smap, 0.002)
    got = _native_evaluate(_spec_dict(tyre), _tier_dict(tier),
                           _profile_dict(road), _surface_map_dict(smap),
                           native_state, hub, vel, 9.0, 0.002)
    _assert_outputs_bitwise(got, want, 'stale-drop')
    _assert_state_bitwise(got['state'], oracle2, 'stale-drop')
    # The stale key was dropped; only live patches remain.
    assert _patch_keys(oracle2) == live


def test_model_patch_migration_injected_candidates():
    """Crafted candidates pin the selection rules: exact-key reuse
    beats any overlap, equal overlaps break the tie on the larger
    (start, stop), and a duplicate key restores last-write-wins."""
    tyre, tier = FRONT_TYRE, DETAIL64
    road = _flat_road()
    smap = _uniform_map()
    radius = tyre.outer_radius_mm * 0.001
    hub = np.array([5.0, 0.0, radius - 0.025])
    vel = np.array([3.0, 0.0, 0.0])
    spec_d, tier_d = _spec_dict(tyre), _tier_dict(tier)
    prof_d, smap_d = _profile_dict(road), _surface_map_dict(smap)

    probe = PneumaticTyre(tyre, tier)
    probe.evaluate(hub, vel, 9.0, road, smap, 0.002)
    (s0, e0), = _patch_keys(probe)
    w = e0 - s0
    assert w >= 8

    def drive(oracle_injected, wire_states, tag):
        oracle = PneumaticTyre(tyre, tier)
        for st in oracle_injected:
            oracle._patch_states[(st.start, st.stop)] = st
        native_state = {
            'carcass': _oracle_carcass_state_dict(
                oracle.carcass_state),
            'patch_states': wire_states}
        want = oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
        got = _native_evaluate(spec_d, tier_d, prof_d, smap_d,
                               native_state, hub, vel, 9.0, 0.002)
        _assert_outputs_bitwise(got, want, tag)
        _assert_state_bitwise(got['state'], oracle, tag)
        return want, oracle

    # --- tie-break: both candidates overlap by 3 rays; the larger
    # (start, stop) key wins and seeds the new detailed brush with its
    # own kappa and bristle overlap.
    left = _injected_state(s0 - 2, s0 + 3, 0.5,
                           np.linspace(0.01, 0.02, 5), 0.5)
    right = _injected_state(e0 - 3, e0 + 2, 0.75,
                            np.linspace(-0.02, -0.01, 5), 0.75)
    want, oracle = drive(
        [left, right],
        [_patch_state_wire(s0 - 2, s0 + 3, 0.5,
                           np.linspace(0.01, 0.02, 5), 0.5),
         _patch_state_wire(e0 - 3, e0 + 2, 0.75,
                           np.linspace(-0.02, -0.01, 5), 0.75)],
        'tie-break')
    # `right` won: the migrated detailed brush carried 0.75 forward —
    # the stepped state differs from a 0.5 seeding, which the bitwise
    # state compare pins; this guard documents the intent.
    assert _patch_keys(oracle) == ((s0, e0),)

    # --- exact key wins regardless of a larger-overlap alternative.
    exact = _injected_state(s0, e0, 0.25,
                            np.linspace(0.03, 0.04, w), 0.25)
    wider = _injected_state(s0 - 4, e0 + 4, 0.9,
                            np.linspace(0.05, 0.06, w + 8), 0.9)
    drive([exact, wider],
          [_patch_state_wire(s0, e0, 0.25,
                             np.linspace(0.03, 0.04, w), 0.25),
           _patch_state_wire(s0 - 4, e0 + 4, 0.9,
                             np.linspace(0.05, 0.06, w + 8), 0.9)],
          'exact-key')

    # --- dict last-write-wins on a repeated (start, stop).
    oracle = PneumaticTyre(tyre, tier)
    oracle._patch_states[(s0, e0)] = _injected_state(s0, e0, -0.5)
    native_state = {
        'carcass': _oracle_carcass_state_dict(oracle.carcass_state),
        'patch_states': [
            _patch_state_wire(s0, e0, 0.125),
            _patch_state_wire(s0, e0, -0.5)]}
    want = oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    got = _native_evaluate(spec_d, tier_d, prof_d, smap_d,
                           native_state, hub, vel, 9.0, 0.002)
    _assert_outputs_bitwise(got, want, 'duplicate-key')
    _assert_state_bitwise(got['state'], oracle, 'duplicate-key')


def test_model_migration_bristle_broadcast_semantics():
    """The slice-clamped bristle copy follows NumPy's assignment
    broadcast: a clamped RHS of exactly one element fills the whole
    overlap (no error — bitwise), while a clamped RHS of two or more
    short elements raises the oracle's ValueError on both sides and
    leaves both patch pools untouched."""
    tyre, tier = FRONT_TYRE, DETAIL64
    road = _flat_road()
    smap = _uniform_map()
    radius = tyre.outer_radius_mm * 0.001
    spec_d, tier_d = _spec_dict(tyre), _tier_dict(tier)
    prof_d, smap_d = _profile_dict(road), _surface_map_dict(smap)
    hub = np.array([5.0, 0.0, radius - 0.025])
    vel = np.array([3.0, 0.0, 0.0])

    probe = PneumaticTyre(tyre, tier)
    probe.evaluate(hub, vel, 9.0, road, smap, 0.002)
    (s0, e0), = _patch_keys(probe)
    w = e0 - s0
    assert w >= 8

    # A state at an adjacent key: the overlap window is (s0, e0-2) —
    # w - 2 target slots sourced from prev[2 : w-2], clamped to the
    # injected row's length.
    def drive(bristle, tag):
        short = _injected_state(s0 - 2, e0 - 2, 0.4, bristle, 0.4)
        oracle = PneumaticTyre(tyre, tier)
        oracle._patch_states[(short.start, short.stop)] = short
        native_state = {
            'carcass': _oracle_carcass_state_dict(
                oracle.carcass_state),
            'patch_states': [
                _patch_state_wire(s0 - 2, e0 - 2, 0.4, bristle, 0.4)]}
        return oracle, native_state

    # len 3 → prev[2:3] gives ONE element — broadcast fills w-2 slots.
    oracle, native_state = drive((0.01, -0.02, 0.03), 'broadcast-fill')
    want = oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    got = _native_evaluate(spec_d, tier_d, prof_d, smap_d,
                           native_state, hub, vel, 9.0, 0.002)
    _assert_outputs_bitwise(got, want, 'broadcast-fill')
    _assert_state_bitwise(got['state'], oracle, 'broadcast-fill')

    # len 4 → prev[2:4] gives TWO elements — cannot broadcast into
    # w-2 >= 6 slots: the oracle's ValueError fires, and the failed
    # migration never rebound _patch_states.
    oracle, native_state = drive((0.01, -0.02, 0.03, 0.04),
                                 'broadcast-reject')
    with pytest.raises(ValueError):
        oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    assert _patch_keys(oracle) == ((s0 - 2, e0 - 2),)
    with pytest.raises(ValueError):
        _native_evaluate(spec_d, tier_d, prof_d, smap_d, native_state,
                         hub, vel, 9.0, 0.002)


def test_model_migration_from_detailed_none():
    """A migrated state whose predecessor carries ``detailed=None``
    under the discretised tier gets a fresh DiscretisedBrush — the
    ``previous.detailed is not None`` gate's false side — while the
    patch-level kappa_prime still migrates."""
    tyre, tier = FRONT_TYRE, DETAIL64
    road = _flat_road()
    smap = _uniform_map()
    radius = tyre.outer_radius_mm * 0.001
    hub = np.array([5.0, 0.0, radius - 0.025])
    vel = np.array([3.0, 0.0, 0.0])

    probe = PneumaticTyre(tyre, tier)
    probe.evaluate(hub, vel, 9.0, road, smap, 0.002)
    (s0, e0), = _patch_keys(probe)

    bare = _injected_state(s0 - 1, e0 - 1, 0.33)
    assert bare.detailed is None
    oracle = PneumaticTyre(tyre, tier)
    oracle._patch_states[(s0 - 1, e0 - 1)] = bare
    native_state = {
        'carcass': _oracle_carcass_state_dict(oracle.carcass_state),
        'patch_states': [_patch_state_wire(s0 - 1, e0 - 1, 0.33)]}
    want = oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    got = _native_evaluate(_spec_dict(tyre), _tier_dict(tier),
                           _profile_dict(road), _surface_map_dict(smap),
                           native_state, hub, vel, 9.0, 0.002)
    _assert_outputs_bitwise(got, want, 'detailed-none')
    _assert_state_bitwise(got['state'], oracle, 'detailed-none')
    assert oracle._patch_states[(s0, e0)].detailed is not None


# --------------------------------------------------------------------
# Rim events, rolling loss, surfaces
# --------------------------------------------------------------------

@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_rim_strike_bitwise(tier):
    """Deep deflection crosses the rim threshold — the completed event
    lands in the wheel outputs on release and rim_strike_active while
    engaged, in both brush modes."""
    tyre = FRONT_TYRE
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    rim_defl = tyre.rim_strike_deflection_mm * 0.001
    cz_below = radius - rim_defl - 0.004
    cz_deep = radius - rim_defl - 0.012
    rows = _rows([5.0] * 6,
                 [0.36, cz_below, cz_deep, cz_below, 0.34, 0.36],
                 [-7.0, -7.5, -8.0, -8.0, -8.0, -7.0],
                 speed=-3.2)

    events = []

    def check(_got, want, _o, _t):
        events.append(want.rim_event)

    _run_sequence(tyre, tier, road, _uniform_map(), rows, check)
    done = [e for e in events if e is not None]
    assert len(done) == 1
    assert done[0].peak_load_n > 0.0


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_rolling_loss_signs_bitwise(tier):
    """Every _tread_loss_force direction combination: v_x > 0 and
    v_x < 0 with spin, v_x == 0 falling back to the spin sign, both
    zero (no force), and the |omega| < 1 rad/s taper."""
    tyre = FRONT_TYRE  # tread_loss_crr = 0.00586 — nonzero.
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    lo = radius - 0.025
    rows = [
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 8.0, 0.002),
        (np.array([5.0, 0.0, lo]), np.array([-2.0, 0.0, 0.0]), -7.0,
         0.002),
        # v_x == 0 → the spin-sign fallback.
        (np.array([5.0, 0.0, lo]), np.array([0.0, 0.0, 0.0]), 8.0, 0.002),
        (np.array([5.0, 0.0, lo]), np.array([0.0, 0.0, 0.0]), -8.0,
         0.002),
        # v_x == 0 and omega == 0 → no direction, force stays +0.0.
        (np.array([5.0, 0.0, lo]), np.array([0.0, 0.0, 0.0]), 0.0, 0.002),
        # |omega| below the 1 rad/s taper: force scales linearly.
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 0.4, 0.002),
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 1.0, 0.002),
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 1.5, 0.002),
    ]

    losses, loads = [], []

    def check(_got, want, _o, tag):
        # Recover the oracle's exact tread-loss force from the patch
        # outputs: v_x = velocity . tangent_world is emitted per patch.
        p = want.patches[0]
        _, vel_row, omega_row, _ = rows[int(tag.split()[1])]
        v_x = float(np.dot(vel_row, p.tangent_world))
        loads.append(p.normal_load_n)
        losses.append(_tread_loss_force(
            _tread_loss_crr_at_pressure(tyre), p.normal_load_n,
            omega_row, v_x))

    _run_sequence(tyre, tier, road, _uniform_map(), rows, check)
    # Forward/reverse rolling produce opposite signed losses; the
    # v_x == 0 fallback uses the spin sign; the all-zero row returns
    # +0.0; the taper is linear below 1 rad/s and saturated at/above —
    # compare |F|/load = crr . taper so the evolving normal load cancels.
    assert losses[0] * losses[1] < 0.0
    assert np.sign(losses[2]) == -1.0 and np.sign(losses[3]) == 1.0
    assert losses[4] == 0.0
    assert abs(losses[5]) / loads[5] < abs(losses[6]) / loads[6]
    assert np.isclose(abs(losses[6]) / loads[6],
                      abs(losses[7]) / loads[7], rtol=1e-12)


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_tread_loss_disabled_bitwise(tier):
    """tread_loss_crr == 0 takes the crr<=0 early return — the patch
    tangential force is the brush's alone."""
    tyre = replace(FRONT_TYRE, tread_loss_crr=0.0)
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    rows = _rows([5.0, 5.0, 5.0], [radius - 0.025] * 3, [9.0] * 3)
    _run_sequence(tyre, tier, road, _uniform_map(), rows)


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_pressure_power_and_set_pressure_bitwise(tier):
    """set_pressure writes the clamped spec pressure; the following
    step's Crr shortfall scales by (p / 1.5 bar) ** -0.3 — CPython's
    float-pow ladder on a non-trivial exponent — and the carcass
    stiffness re-derives at the new pressure."""
    tyre = FRONT_TYRE
    road = _flat_road()
    radius = tyre.outer_radius_mm * 0.001
    lo = radius - 0.025
    rows = [
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 9.0, 0.002,
         2.4),
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 9.0, 0.002,
         None),
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 9.0, 0.002,
         0.2),    # clamps to PRESSURE_MIN_BAR
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 9.0, 0.002,
         9.9),    # clamps to PRESSURE_MAX_BAR
        (np.array([5.0, 0.0, lo]), np.array([3.0, 0.0, 0.0]), 9.0, 0.002,
         np.nan),  # max(0.8, nan) keeps the bound -> 0.8
    ]
    _run_sequence(tyre, tier, road, _uniform_map(), rows)


def test_model_set_pressure_clamp_bitwise():
    """The standalone set_pressure surface returns the live clamped
    pressure — including both clamp bounds and the NaN quirk."""
    spec_d, tier_d = _spec_dict(FRONT_TYRE), _tier_dict(FAST)
    for p in (0.0, 0.4, PRESSURE_MIN_BAR, 1.1, 1.5, 2.9,
              PRESSURE_MAX_BAR, 4.5, np.inf, -np.inf, np.nan,
              np.nextafter(PRESSURE_MIN_BAR, 0.0),
              np.nextafter(PRESSURE_MAX_BAR, 100.0)):
        want = clamp_pressure_bar(p)
        got = bike_native._debug_pneumatic_model_set_pressure(
            spec_d, tier_d, float(p))
        assert_bitwise_equal(float(got), float(want), f'clamp {p!r}')


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_surface_sections_bitwise(tier):
    """Patch centroids crossing material-zone boundaries resolve the
    half-open section scan — the surface_name and friction follow the
    section, then fall back to the default."""
    tyre = FRONT_TYRE
    road = _flat_road()
    smap = _zoned_map()
    radius = tyre.outer_radius_mm * 0.001
    # Roll from hardpack through 'loose' [4.6, 5.1) into 'scree'
    # [5.1, 5.6) and out the other side.
    xs = [4.3 + 0.05 * k for k in range(30)]
    rows = _rows(xs, [radius - 0.025] * 30, [9.0] * 30, speed=2.5)

    names = []

    def check(_g, want, _o, _t):
        names.extend(p.surface_name for p in want.patches)

    _run_sequence(tyre, tier, road, smap, rows, check)
    assert 'loose' in names and 'scree' in names and 'hardpack' in names


# --------------------------------------------------------------------
# reset / coverage events / validation
# --------------------------------------------------------------------

@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_reset_bitwise(tier):
    """reset() on a dirty state returns WheelOutputs.empty and clears
    carcass + patch state — exercised on real loaded history restored
    into the native model."""
    tyre = FRONT_TYRE
    road = _bump_road()
    radius = tyre.outer_radius_mm * 0.001
    rows = _rows([4.9, 4.95, 5.0, 5.05], [radius - 0.030] * 4,
                 [8.0] * 4, speed=2.5)
    oracle, native_state = _run_sequence(tyre, tier, road,
                                         _uniform_map(), rows)
    oracle.reset()
    got = bike_native._debug_pneumatic_model_reset(
        _spec_dict(tyre), _tier_dict(tier), native_state)
    _assert_outputs_bitwise(got['outputs'], oracle.last_outputs,
                            'reset')
    _assert_state_bitwise(got['state'], oracle, 'reset')
    assert len(got['state']['patch_states']) == 0


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_coverage_event_bitwise(tier):
    """A kerb taller than the hub puts road inside the tyre circle but
    outside ray coverage — coverage_event reaches the outputs on the
    loaded, no-contact and airborne paths."""
    tyre = FRONT_TYRE
    road = _spike_road()   # 0.45 m kerb at x = 5.30
    radius = tyre.outer_radius_mm * 0.001
    lo = radius - 0.02
    rows = _rows([5.02, 5.02, 5.02],
                 [lo, radius + 0.06, radius + 0.5],
                 [8.0, 8.0, 8.0], speed=6.0)

    cov = []

    def check(_g, want, _o, _t):
        cov.append(want.coverage_event)

    _run_sequence(tyre, tier, road, _uniform_map(), rows, check)
    assert any(cov), 'coverage_event never raised'


@pytest.mark.parametrize('tier', [FAST, DETAIL64])
def test_model_validation_bitwise_atomicity(tier):
    """Every rejection class in the oracle's order — 3-vector shape,
    hub/velocity finiteness, spin finiteness — plus state preservation
    on both sides and the carcass-level dt gate."""
    tyre = FRONT_TYRE
    road = _flat_road()
    smap = _uniform_map()
    radius = tyre.outer_radius_mm * 0.001
    spec_d, tier_d = _spec_dict(tyre), _tier_dict(tier)
    prof_d, smap_d = _profile_dict(road), _surface_map_dict(smap)
    hub = np.array([5.0, 0.0, radius - 0.025])
    vel = np.array([3.0, 0.0, 0.0])

    # Establish history on both sides.
    oracle = PneumaticTyre(tyre, tier)
    oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    native_state = _native_evaluate(
        spec_d, tier_d, prof_d, smap_d, _fresh_state_dict(tier.n_rays),
        hub, vel, 9.0, 0.002)['state']

    bad_inputs = [
        ('shape', np.zeros(4), vel, 9.0, 0.002),
        ('shape-vel', hub, np.zeros(2), 9.0, 0.002),
        ('nan-hub', np.array([5.0, np.nan, hub[2]]), vel, 9.0, 0.002),
        ('inf-hub', np.array([5.0, 0.0, np.inf]), vel, 9.0, 0.002),
        ('nan-vel', hub, np.array([np.nan, 0.0, 0.0]), 9.0, 0.002),
        ('inf-vel', hub, np.array([0.0, 0.0, -np.inf]), 9.0, 0.002),
        ('nan-omega', hub, vel, np.nan, 0.002),
        ('inf-omega', hub, vel, np.inf, 0.002),
        # carcass-level checks that fire on the loaded path.
        ('zero-dt', hub, vel, 9.0, 0.0),
        ('neg-dt', hub, vel, 9.0, -0.002),
        ('nan-dt', hub, vel, 9.0, np.nan),
        ('inf-dt', hub, vel, 9.0, np.inf),
    ]
    for tag, h, v, o, dt in bad_inputs:
        snap = _oracle_state_dict(oracle)
        with pytest.raises(ValueError):
            oracle.evaluate(h, v, o, road, smap, dt)
        _assert_state_bitwise(snap, oracle, f'{tag} atomicity')
        with pytest.raises(ValueError):
            _native_evaluate(spec_d, tier_d, prof_d, smap_d,
                             native_state, h, v, o, dt)

    # Ordering: the shape check precedes finiteness and spin.
    with pytest.raises(ValueError, match='3-vectors'):
        oracle.evaluate(np.zeros(4), np.array([np.nan]), np.nan,
                        road, smap, 0.002)
    with pytest.raises(ValueError):
        _native_evaluate(spec_d, tier_d, prof_d, smap_d, native_state,
                         np.zeros(4), np.array([np.nan]), np.nan, 0.002)
    # Finiteness precedes the spin check.
    with pytest.raises(ValueError, match='finite'):
        oracle.evaluate(np.array([np.inf, 0.0, 0.0]), vel, np.nan,
                        road, smap, 0.002)
    with pytest.raises(ValueError):
        _native_evaluate(spec_d, tier_d, prof_d, smap_d, native_state,
                         np.array([np.inf, 0.0, 0.0]), vel, np.nan,
                         0.002)

    # State-intake rejection: a carcass state one ray too narrow and a
    # patch-state bristle wider than the ring.
    narrow = dict(native_state)
    narrow['carcass'] = dict(native_state['carcass'])
    narrow['carcass']['delta_prev_m'] = \
        native_state['carcass']['delta_prev_m'][:-1]
    narrow['carcass']['elastic_prev_n'] = \
        native_state['carcass']['elastic_prev_n'][:-1]
    narrow['carcass']['maxwell_force_n'] = \
        native_state['carcass']['maxwell_force_n'][:-1]
    with pytest.raises(ValueError):
        _native_evaluate(spec_d, tier_d, prof_d, smap_d, narrow,
                         hub, vel, 9.0, 0.002)
    saved = oracle.carcass_state
    oracle.carcass_state = carcass.CarcassState(tier.n_rays - 1)
    with pytest.raises(ValueError):
        oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    oracle.carcass_state = saved   # untouched by the failed call

    wide_bristle = dict(native_state)
    wide_bristle['patch_states'] = [
        _patch_state_wire(10, 20, 0.1, np.zeros(tier.n_rays + 1))]
    with pytest.raises(ValueError):
        _native_evaluate(spec_d, tier_d, prof_d, smap_d, wide_bristle,
                         hub, vel, 9.0, 0.002)

    # The airborne fast path skips the carcass dt gate entirely on an
    # uninitialized state (advance_unloaded's early return) — but the
    # same call on an initialized state rejects it.
    air_hub = np.array([5.0, 0.0, radius + 0.5])
    fresh_oracle = PneumaticTyre(tyre, tier)
    silent = fresh_oracle.evaluate(air_hub, vel, 9.0, road, smap, 0.0)
    got_silent = _native_evaluate(
        spec_d, tier_d, prof_d, smap_d, _fresh_state_dict(tier.n_rays),
        air_hub, vel, 9.0, 0.0)
    _assert_outputs_bitwise(got_silent, silent, 'silent dt')
    # Drive a dirty loaded→airborne transition on both sides; the
    # second airborne call hits advance_unloaded's positivity check.
    oracle.evaluate(air_hub, vel, 9.0, road, smap, 0.002)
    native_state = _native_evaluate(
        spec_d, tier_d, prof_d, smap_d, native_state, air_hub, vel,
        9.0, 0.002)['state']
    with pytest.raises(ValueError):
        oracle.evaluate(air_hub, vel, 9.0, road, smap, 0.0)
    with pytest.raises(ValueError):
        _native_evaluate(spec_d, tier_d, prof_d, smap_d, native_state,
                         air_hub, vel, 9.0, 0.0)

    # Post-failure valid call still matches bitwise.
    want = oracle.evaluate(hub, vel, 9.0, road, smap, 0.002)
    got = _native_evaluate(spec_d, tier_d, prof_d, smap_d, native_state,
                           hub, vel, 9.0, 0.002)
    _assert_outputs_bitwise(got, want, 'post-failure')
    _assert_state_bitwise(got['state'], oracle, 'post-failure')
