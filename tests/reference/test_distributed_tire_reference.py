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


def test_scalar_station_formulas_equal_the_vectorised_ones_bitwise():
    material = HingeDensity()
    deltas = np.array([-.01, 0., 1e-6, .003, .004, .0041, .011, .012, .02])
    rates = np.array([-.3, 0., .2, -1., .5, 0., .1, -.1, 2.])
    weights = np.full(len(deltas), 2 * np.pi / 128)
    vec_force, vec_energy = normal_station_response(deltas, rates, weights, material)
    vec_density, vec_density_energy = density_response(deltas, material)
    from bike_sim.physics.distributed_tire import station_density, station_response
    for i in range(len(deltas)):
        f, e = station_response(float(deltas[i]), float(rates[i]), float(weights[i]), material)
        d, de = station_density(float(deltas[i]), material)
        assert (f, e) == (float(vec_force[i]), float(vec_energy[i]))
        assert (d, de) == (float(vec_density[i]), float(vec_density_energy[i]))


def test_station_count_ceiling():
    from bike_sim.physics.distributed_tire import station_angles, DistributedTireConfig
    station_angles(512)
    with pytest.raises(ValueError, match='512'):
        station_angles(513)
    with pytest.raises(ValueError):
        DistributedTireConfig(station_count=1024)


def _stand():
    m, d, tire = wheel_stand(.00125, 32)
    return m, d, tire


def test_state_wire_names_and_round_trip():
    from bike_sim.sim.ride.distributed_tire_forces import _StationState
    m, d, tire = _stand()
    names = tire.state_names()
    assert len(names) == 2 * 32 * 4
    assert names[:4] == ['front.s000.xi', 'front.s000.tangent.0', 'front.s000.tangent.1', 'front.s000.tangent.2']
    assert names[-1] == 'rear.s031.tangent.2'
    tire.states = {('front', 3): _StationState(.002, np.array([1., 0., 0.])), ('rear', 7): _StationState(-.001, None)}
    wire = tire.state_dict()
    assert wire['names'] == names and wire['row'].shape == (len(names),)
    assert wire['row'][names.index('front.s003.xi')] == .002
    assert np.isnan(wire['row'][names.index('rear.s007.tangent.0')])
    assert np.isnan(wire['row'][names.index('front.s000.xi')])
    tire.states = {}
    tire.set_state(wire['names'], wire['row'])
    assert set(tire.states) == {('front', 3), ('rear', 7)}
    assert tire.states[('front', 3)].xi == .002 and tire.states[('rear', 7)].tangent is None
    assert np.array_equal(tire.states[('front', 3)].tangent, [1., 0., 0.])
    with pytest.raises(ValueError, match='rear.s031.tangent.2'):
        tire.set_state(wire['names'][:-1], wire['row'][:-1])
    with pytest.raises(ValueError, match='front.s000.bogus'):
        tire.set_state(['front.s000.bogus'], [0.])


def test_diagnostics_are_scalar_only_and_multi_contact_is_informational():
    rows, m = distributed_road_trace(.00125, 64, .005, case='two_support', duration_s=.05)
    assert m['max_patch_count'] >= 2
    m_, d_, tire = wheel_stand(.00125, 64)
    tire.compute_qfrc(m_, d_, .00125)
    diag = tire.diagnostics['front']
    assert set(diag) == {'penetration_m', 'normal_load_n', 'multi_support', 'supports_multiple_contacts',
                         'capability_status', 'calibration_status', 'station_count', 'active_station_count',
                         'nonsmooth_station_count', 'max_patch_count', 'outside_material_load_range',
                         'outside_material_deflection_range', 'radial_energy_j', 'shear_energy_j', 'brush_loss_j'}
    assert diag['supports_multiple_contacts'] is True and diag['station_count'] == 64
    assert all(not isinstance(v, (list, tuple, np.ndarray)) for v in diag.values())


def test_compute_is_deterministic_and_states_are_station_states():
    from bike_sim.sim.ride.distributed_tire_forces import _StationState
    import mujoco
    a = wheel_stand(.00125, 64)
    b = wheel_stand(.00125, 64)
    for m, d, tire in (a, b):
        d.qpos[1] = -.006
        mujoco.mj_forward(m, d)
    fa, fb = a[2].compute_qfrc(a[0], a[1], .00125), b[2].compute_qfrc(b[0], b[1], .00125)
    assert np.array_equal(fa, fb)
    assert a[2].states and all(isinstance(s, _StationState) for s in a[2].states.values())
    # equal_nan: the airborne rear wheel keeps an all-NaN state block and NaN != NaN.
    assert np.array_equal(a[2].state_dict()['row'], b[2].state_dict()['row'], equal_nan=True)


def test_probe_leaves_state_and_clock_untouched():
    import mujoco
    m, d, tire = wheel_stand(.00125, 64)
    d.qpos[1] = -.006
    mujoco.mj_forward(m, d)
    tire.compute_qfrc(m, d, .00125)
    before = tire.state_dict()['row'].copy(); clock = tire.last_time_s
    tire.compute_qfrc(m, d, .00125, advance=False)
    assert np.array_equal(np.nan_to_num(tire.state_dict()['row']), np.nan_to_num(before))
    assert tire.last_time_s == clock and tire.probe_snapshots
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        tire.compute_qfrc(m, d, .00125)
