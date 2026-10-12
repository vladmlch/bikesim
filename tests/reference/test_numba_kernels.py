"""Equivalence of the numba-JIT kernels against preserved Python references.

Every ``@numba.njit(cache=True)`` kernel keeps its pre-JIT body importable as a
``*_py`` function. These tests draw 1000 random inputs per kernel and require
the compiled path to agree with the reference at ``rel=1e-12`` — a band two
orders of magnitude above measured codegen drift (~1 ulp from op ordering/FMA)
and six below the stage-2 aggregate gate (rtol=1e-6). ``abs=1e-12`` covers
last-ulp differences on results that cancel to ~0 from O(1e2-1e3) operands.
"""
import numpy as np
import pytest
from hypothesis import given, settings, strategies as st
from hypothesis.extra import numpy as hnp

from bike_sim.physics.joint_strength import (
    TorqueCurve, directional_capacity, _directional_capacity_py,
)
from bike_sim.physics.tire import (
    _brush_step, _brush_step_py, _normal_contact, _normal_contact_py,
)
from bike_sim.sim.ride.support_geometry import (
    _box_pad_contact, _box_pad_contact_py,
)

RTOL = 1e-12
ATOL = 1e-12


@st.composite
def _torque_curves(draw):
    n = draw(st.integers(min_value=2, max_value=8))
    angles = np.sort(draw(st.lists(
        st.floats(-2., 2., allow_nan=False, allow_infinity=False),
        min_size=n, max_size=n, unique=True)))
    torques = draw(st.lists(
        st.floats(0., 600., allow_nan=False, allow_infinity=False),
        min_size=n, max_size=n))
    return TorqueCurve(
        angles_rad=tuple(float(v) for v in angles),
        torques_nm=tuple(float(v) for v in torques),
        vmax_rad_s=draw(st.floats(1e-3, 50., allow_nan=False,
                                  allow_infinity=False)),
        hill_c=draw(st.floats(1e-3, 2., allow_nan=False, allow_infinity=False)),
        eccentric_ratio=draw(st.floats(1., 2., allow_nan=False,
                                       allow_infinity=False)),
        source='synthetic-equivalence-probe')


@given(_torque_curves(),
       st.floats(-2.5, 2.5, allow_nan=False),
       st.floats(-1e6, 1e6, allow_nan=False),
       st.sampled_from((-1, 1)))
@settings(max_examples=1000, deadline=None)
def test_directional_capacity_matches_python(curve, angle, velocity, direction):
    angle = min(max(angle, curve.angles_rad[0]), curve.angles_rad[-1])
    assert directional_capacity(curve, angle, velocity, direction) == \
        pytest.approx(
            _directional_capacity_py(curve, angle, velocity, direction),
            rel=RTOL, abs=ATOL)


def test_directional_capacity_error_paths_match():
    curve = TorqueCurve(angles_rad=(-1., 0., 1.), torques_nm=(50., 100., 50.),
                        vmax_rad_s=10., hill_c=.25, eccentric_ratio=1.4,
                        source='synthetic-equivalence-probe')
    for angle, velocity, direction in (
            (0., 0., 0), (0., 0., 2), (2., 0., 1),
            (0., float('inf'), 1), (0., float('nan'), -1)):
        with pytest.raises(ValueError):
            directional_capacity(curve, angle, velocity, direction)
        with pytest.raises(ValueError):
            _directional_capacity_py(curve, angle, velocity, direction)


_finite = dict(allow_nan=False, allow_infinity=False)


@given(st.floats(-1., 1., **_finite),                    # xi [m]
       st.floats(-50., 50., **_finite),                  # u [m/s]
       st.floats(-50., 50., **_finite),                  # v_roll [m/s]
       st.one_of(st.just(0.), st.floats(0., 2e3, **_finite)),   # Fn [N]
       st.floats(1e-3, 1e6, **_finite),                  # k [N/m]
       st.one_of(st.just(0.), st.floats(0., 3., **_finite)),    # mu
       st.floats(1e-6, 10., **_finite),                  # length [m]
       st.floats(1e-6, .1, **_finite))                   # dt [s]
@settings(max_examples=1000, deadline=None)
def test_brush_step_matches_python(xi, u, v_roll, Fn, k, mu, length, dt):
    expected = _brush_step_py(xi, u, v_roll, Fn, k, mu, length, dt)
    actual = _brush_step(xi, u, v_roll, Fn, k, mu, length, dt)
    assert len(actual) == len(expected) == 3
    for got, want in zip(actual, expected):
        assert got == pytest.approx(want, rel=RTOL, abs=ATOL)


def test_brush_step_error_paths_match():
    # old_energy overflows before the Fn==0/mu==0 early return in both paths.
    args = (1e200, 0., 0., 1., 1e200, 1., 1., 1.)
    with pytest.raises(OverflowError):
        _brush_step(*args)
    with pytest.raises(OverflowError):
        _brush_step_py(*args)
    # friction_limit (mu*Fn) overflows inside the later finite check.
    args = (0., 0., 1., 1e200, 1., 1e200, 1., 1.)
    with pytest.raises(OverflowError):
        _brush_step(*args)
    with pytest.raises(OverflowError):
        _brush_step_py(*args)


@given(st.floats(-1., 1., **_finite),                    # delta [m]
       st.floats(-1e3, 1e3, **_finite),                  # delta_dot [m/s]
       st.floats(1e-6, 1e7, **_finite),                  # k [N/m]
       st.floats(0., 1e4, **_finite))                    # c [N.s/m]
@settings(max_examples=1000, deadline=None)
def test_normal_contact_matches_python(delta, delta_dot, k, c):
    expected = _normal_contact_py(delta, delta_dot, k, c)
    actual = _normal_contact(delta, delta_dot, k, c)
    assert len(actual) == len(expected) == 2
    for got, want in zip(actual, expected):
        assert got == pytest.approx(want, rel=RTOL, abs=ATOL)


def test_normal_contact_error_paths_match():
    # k*delta overflows inside the finite check in both paths.
    args = (1e200, 0., 1e200, 0.)
    with pytest.raises(OverflowError):
        _normal_contact(*args)
    with pytest.raises(OverflowError):
        _normal_contact_py(*args)


_vec3 = hnp.arrays(np.float64, (3,), elements=st.floats(-1., 1., **_finite))
_half3 = hnp.arrays(np.float64, (3,), elements=st.floats(1e-3, 1., **_finite))


@st.composite
def _planar_rotations(draw):
    """Proper rotation about the model's out-of-plane Y axis."""
    a = draw(st.floats(-np.pi, np.pi, **_finite))
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def _assert_contact_close(got, want):
    np.testing.assert_allclose(got.point_m, want.point_m, rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(got.normal, want.normal, rtol=RTOL, atol=ATOL)
    assert got.gap_m == pytest.approx(want.gap_m, rel=RTOL, abs=ATOL)
    assert got.within_width == want.within_width
    assert got.within_footprint == want.within_footprint


@given(_vec3, st.floats(1e-4, .5, **_finite), _vec3,
       _planar_rotations(), _half3)
@settings(max_examples=1000, deadline=None)
def test_box_pad_contact_matches_python(center, radius, origin, rotation, half):
    _assert_contact_close(
        _box_pad_contact(center, radius, origin, rotation, half),
        _box_pad_contact_py(center, radius, origin, rotation, half))


def test_box_pad_contact_inside_and_tie_cases():
    # Deterministic coverage of the distance<=1e-14 inside branch, face-plane
    # boundaries, and the medial-axis sign==0 tie that random draws never hit.
    origin = np.zeros(3)
    rotation = np.eye(3)
    half = np.array([.15, .05, .02])
    for center in (
            np.array([0., 0., 0.]),        # exact medial tie -> upward face
            np.array([.15, 0., 0.]),       # on the +x face plane
            np.array([0., 0., .02]),       # on the +z face plane
            np.array([.05, 0., -.01]),     # strictly inside
            np.array([.2, 0., 0.]),        # outside the +x face
            np.array([.2, 0., .05]),       # outside beyond the corner
    ):
        for radius in (.01, .03):
            _assert_contact_close(
                _box_pad_contact(center, radius, origin, rotation, half),
                _box_pad_contact_py(center, radius, origin, rotation, half))
