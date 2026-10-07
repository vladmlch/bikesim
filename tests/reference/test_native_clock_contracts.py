"""F5: deterministic interval ids and the live cruise timestep.

``bike_sim.sim.ride.tire_forces.interval_id`` and its native twin
``interval_clock::interval_id`` implement the spec-S4 interval conversion:
binary64 ``time / dt`` rounded half-even — independent of the process
floating-point rounding mode — accepted on ``[0, 2**63 - 1]``. Domain
violations (nonfinite or negative time, nonpositive or nonfinite dt) are
``ValueError``; a finite input whose interval overflows int64 is
``OverflowError``. The native tire snapshot path must emit the same ids as
the Python applier, and the cruise integrators must read
``model.opt.timestep`` at compute time, not a constructor-cached copy.
"""
import ctypes
import math
import platform
import struct
import sys
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal
from bike_sim.sim.ride.cruise import CruiseController
from bike_sim.sim.ride.tire_forces import interval_id
from test_native_tire import CANONICAL_NAMES, MJB, _tire_cfg

INT64_MAX = (1 << 63) - 1
# 0x1p63 — the first binary64 value outside the int64 domain.
INT64_BOUND = float(1 << 63)
# binary64 spacing on [2**62, 2**63) is 1024: 2**63-1 is not representable
# and the largest admitted interval is 2**63-1024.
INT64_LARGEST = math.nextafter(INT64_BOUND, 0.)
assert INT64_LARGEST == 9223372036854774784.

CRUISE_CONFIG = dict(target_speed_kmh=25., kp_nm_per_mps=180.,
                     ki_nm_per_mps_s=150., torque_ceiling_nm=150.)

# (tonearest, upward, downward, towardzero) per architecture: arm64 carries
# the FPCR RMode bits, x86-64 the MXCSR encodings.
_ROUNDING_MODES = {
    'arm64': (0x0, 0x400000, 0x800000, 0xC00000),
    'arm64e': (0x0, 0x400000, 0x800000, 0xC00000),
    'aarch64': (0x0, 0x400000, 0x800000, 0xC00000),
    'x86_64': (0x0, 0x800, 0x400, 0xC00),
    'amd64': (0x0, 0x800, 0x400, 0xC00),
}


def _fenv():
    """fegetround/fesetround via ctypes; None when the platform lacks them."""
    for library in (None, 'libm.so.6', 'libSystem.B.dylib'):
        try:
            handle = (ctypes.CDLL(library) if library is not None
                      else ctypes.CDLL(None))
            get, set_ = handle.fegetround, handle.fesetround
        except (AttributeError, OSError):
            continue
        get.restype = ctypes.c_int
        set_.argtypes = [ctypes.c_int]
        set_.restype = ctypes.c_int
        return get, set_
    return None


def _rounding_controls():
    """(fegetround, fesetround, modes) or skip — the fenv gate for the
    rounding-mode sweeps."""
    fenv = _fenv()
    modes = _ROUNDING_MODES.get(platform.machine().lower())
    if fenv is None or modes is None:
        pytest.skip('fenv rounding-mode control unavailable on this platform')
    return *fenv, modes


def _modes_move_division(fesetround, modes):
    """Prove the mode bits reach the FP unit — a folded or ignored mode would
    make the sweeps below vacuous."""
    one, three = 1., 3.   # locals keep the division out of constant folding
    nearest = struct.pack('<d', one / three)
    for mode in modes[1:]:
        if fesetround(mode) == 0 and \
                struct.pack('<d', one / three) != nearest:
            return True
    return False


class TestIntervalId:
    @pytest.mark.parametrize('q,expected', [
        (0.5, 0), (1.5, 2), (2.5, 2), (3.5, 4), (4.5, 4), (11.5, 12),
        (0., 0), (-0., 0), (1., 1), (0.49999999999999994, 0),
        (0.5000000000000001, 1)])
    def test_half_even_ties(self, q, expected):
        result = interval_id(q, 1.)
        assert type(result) is int
        assert result == expected

    @pytest.mark.parametrize('tie,below,above', [
        (0.5, 0, 1), (1.5, 1, 2), (2.5, 2, 3), (3.5, 3, 4), (4.5, 4, 5)])
    def test_nextafter_neighbours_of_ties(self, tie, below, above):
        assert interval_id(math.nextafter(tie, 0.), 1.) == below
        assert interval_id(math.nextafter(tie, math.inf), 1.) == above

    @pytest.mark.parametrize('time,dt,expected', [
        (1., 2., 0), (3., 2., 2), (5., 2., 2), (7., 2., 4),      # x/2 ties
        (.25, .5, 0), (.75, .5, 2), (1.25, .5, 2), (1.75, .5, 4),
        (.3, .1, 3),               # 0.3/0.1 = 2.9999999999999996 → 3
        (1., 3., 0), (2., 3., 1), (1., .00125, 800)])
    def test_scaled_dt_quotients(self, time, dt, expected):
        assert interval_id(time, dt) == expected == round(time / dt)

    def test_matches_builtin_round_on_valid_domain(self):
        """Broad sweep against the CPython oracle on the same binary64
        quotient — the domain checks must not perturb representable ids."""
        rng = np.random.default_rng(41)
        cases = np.concatenate([
            np.array([0., 1., 1e6, INT64_LARGEST]),
            rng.uniform(0., 1e3, 300),
            np.floor(rng.uniform(0., 1e6, 300)) + .5])   # exact ties
        for dt in (1., .0005, .00125, 2.5, 37.):
            for t in cases:
                t = float(t)
                q = t / dt
                if not math.isfinite(q) or q >= INT64_BOUND:
                    continue
                assert interval_id(t, dt) == round(q)

    @pytest.mark.parametrize('time', [float('nan'), float('inf'),
                                      -float('inf'), -1e-300, -1.,
                                      -INT64_BOUND])
    def test_time_domain_rejects_with_valueerror(self, time):
        with pytest.raises(ValueError):
            interval_id(time, 1.)

    @pytest.mark.parametrize('dt', [0., -0., -1., float('nan'),
                                    float('inf'), -float('inf')])
    def test_dt_domain_rejects_with_valueerror(self, dt):
        with pytest.raises(ValueError):
            interval_id(1., dt)

    def test_huge_positive_dt_is_valid(self):
        assert interval_id(1., 1e300) == 0
        assert interval_id(1., np.finfo(np.float64).max) == 0

    @pytest.mark.parametrize('time,dt', [
        (INT64_BOUND, 1.),                        # quotient is exactly 0x1p63
        (float(9223372036854775807), 1.),          # literal rounds up to 0x1p63
        (math.nextafter(INT64_BOUND, math.inf), 1.),
        (INT64_BOUND, math.nextafter(1., 0.)),     # quotient just over 0x1p63
        (1e300, 1e-300),                            # quotient overflows to inf
        (np.finfo(np.float64).max, 1.)])
    def test_interval_overflow_is_overflowerror(self, time, dt):
        with pytest.raises(OverflowError):
            interval_id(time, dt)

    def test_int64_upper_edge_accepted(self):
        assert interval_id(INT64_LARGEST, 1.) == 9223372036854774784
        # The retired 9.2e18 approximation rejected this; the exact bound
        # admits every representable value below 0x1p63.
        assert interval_id(9.2e18, 1.) == 9200000000000000000
        assert interval_id(float(1 << 62), 1.) == 1 << 62

    def test_ids_are_rounding_mode_independent(self):
        fegetround, fesetround, modes = _rounding_controls()
        original = fegetround()
        cases = [(0., 0), (-0., 0), (0.5, 0), (1.5, 2), (2.5, 2), (3.5, 4),
                 (4.5, 4), (INT64_LARGEST, 9223372036854774784)]
        for tie, below, above in ((0.5, 0, 1), (1.5, 1, 2), (2.5, 2, 3),
                                  (3.5, 3, 4)):
            cases.append((math.nextafter(tie, 0.), below))
            cases.append((math.nextafter(tie, math.inf), above))
        try:
            if not _modes_move_division(fesetround, modes):
                pytest.skip('fesetround has no observable effect on '
                            'arithmetic on this platform')
            for mode in modes:
                assert fesetround(mode) == 0
                for q, expected in cases:
                    # dt=1.0: the division is exact under every mode, so the
                    # half-even conversion is observed in isolation.
                    assert interval_id(q, 1.) == expected
                # A mode-sensitive quotient shifts identically for both
                # conversions — the helper still mirrors builtin round().
                for t, dt in ((.3, .1), (1., 3.), (5., 2.)):
                    assert interval_id(t, dt) == round(t / dt)
                with pytest.raises(OverflowError):
                    interval_id(INT64_BOUND, 1.)
        finally:
            fesetround(original)
        assert fegetround() == original


def _tire_rig():
    """Native Stepper + live Python TireForceApplier on the pinned model,
    wheels slid over the road like test_native_tire's non-slow fixture."""
    from bike_sim.physics.physical_config import TireBackendConfig
    from bike_sim.sim.ride.tire_forces import (
        TireForceApplier, compiled_profile_vertices)
    model = mujoco.MjModel.from_binary_path(MJB)
    data = mujoco.MjData(model)
    data.qpos[0] += 0.6
    data.qpos[14] += 0.6
    mujoco.mj_forward(model, data)
    stepper = bike_native.Stepper(MJB, _tire_cfg())
    applier = TireForceApplier(
        model, compiled_profile_vertices(model, data),
        TireBackendConfig(backend='compliant_2d'))
    return model, data, stepper, applier


def _snapshot_ids(model, data, stepper, applier, time, dt):
    """Advance BOTH implementations once at (time, dt) from clean brush
    states; return {side: (native_id, python_id)}."""
    data.time = time
    stepper.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart,
                      float(time))
    stepper.forward()
    stepper.set_tire_state(CANONICAL_NAMES, np.zeros(len(CANONICAL_NAMES)))
    stepper.tire_qfrc(dt)
    applier.last_time_s = None
    applier.compute_qfrc(model, data, dt)
    native = stepper.tire_snapshots()
    return {side: (native[side]['interval_id'],
                   applier.snapshots[side].interval_id)
            for side in ('front', 'rear')}


class TestTireSnapshotIntervalPath:
    @pytest.mark.parametrize('time,dt,expected', [
        (0., .00125, 0),
        (-0., .00125, 0),
        (1., .00125, 800),
        (.25, .5, 0), (.75, .5, 2), (1.25, .5, 2), (1.75, .5, 4),
        (.3, .1, 3), (1., 3., 0),
        (INT64_LARGEST, 1., 9223372036854774784)])
    def test_ids_match_python_and_helper(self, time, dt, expected):
        model, data, stepper, applier = _tire_rig()
        ids = _snapshot_ids(model, data, stepper, applier, time, dt)
        for side, (native_id, python_id) in ids.items():
            assert type(native_id) is int and type(python_id) is int
            assert native_id == python_id == expected == \
                interval_id(time, dt), side

    def test_overflow_is_overflowerror_and_atomic(self):
        """An out-of-domain interval fails with OverflowError on both
        implementations and commits nothing — the retry then succeeds."""
        model, data, stepper, applier = _tire_rig()
        stepper.set_state(data.qpos, data.qvel, data.act,
                          data.qacc_warmstart, INT64_BOUND)
        stepper.forward()
        stepper.set_tire_state(CANONICAL_NAMES,
                               np.zeros(len(CANONICAL_NAMES)))
        before = stepper.tire_state().copy()
        with pytest.raises(OverflowError):
            stepper.tire_qfrc(1.)
        assert_bitwise_equal(stepper.tire_state(), before,
                             'rejected interval preserves brush state')
        data.time = INT64_BOUND
        applier.last_time_s = None
        with pytest.raises(OverflowError):
            applier.compute_qfrc(model, data, 1.)
        # The uncommitted attempt left no clock: dt=2 halves the quotient
        # back into the domain and advances cleanly.
        stepper.tire_qfrc(2.)
        assert stepper.tire_snapshots()['front']['interval_id'] == 1 << 62

    def test_snapshot_ids_under_foreign_rounding_modes(self):
        """Both sides divide under the active FE mode and agree; the mode is
        restored even on failure."""
        fegetround, fesetround, modes = _rounding_controls()
        original = fegetround()
        if not _modes_move_division(fesetround, modes):
            fesetround(original)
            pytest.skip('fesetround has no observable effect on this '
                        'platform')
        model, data, stepper, applier = _tire_rig()
        try:
            for mode in modes[1:]:
                if fesetround(mode) != 0:
                    continue
                for time, dt in ((1. / 3., .25), (2. / 7., 1.), (.7, .3)):
                    ids = _snapshot_ids(model, data, stepper, applier,
                                        time, dt)
                    for side, (native_id, python_id) in ids.items():
                        assert native_id == python_id == \
                            interval_id(time, dt), (mode, side, time, dt)
        finally:
            fesetround(original)


def _cruise_model(tmp_path, timestep):
    model = mujoco.MjModel.from_xml_string(f'''
        <mujoco><option timestep="{timestep}"/><worldbody><body>
        <joint name="root_x" type="slide" axis="1 0 0"/>
        <geom type="sphere" size=".1" mass="1"/>
        </body></worldbody></mujoco>''')
    path = tmp_path / f'cruise-{timestep}.mjb'
    mujoco.mj_saveModel(model, str(path))
    return model, path


def _cruise_push(native, data):
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart,
                     float(data.time))


class TestCruiseTimestep:
    def test_python_oracle_reads_timestep_live(self, tmp_path):
        """Same oracle instance: doubling model.opt.timestep between calls
        doubles the integral increment — a ctor-cached copy cannot do this."""
        model, _ = _cruise_model(tmp_path, 0.0005)
        data = mujoco.MjData(model)
        oracle = CruiseController(model, **CRUISE_CONFIG)
        contacts = SimpleNamespace(rear_in_contact=True)
        speed = oracle.target_speed_mps - .2  # unsaturated → integral runs
        data.qvel[oracle.root_x_dofadr] = speed
        oracle.compute(model, data, contacts)
        error = oracle.target_speed_mps - speed
        assert_bitwise_equal(oracle.integral_mps_s, error * 0.0005)
        model.opt.timestep = 0.001
        oracle.compute(model, data, contacts)
        assert_bitwise_equal(oracle.integral_mps_s,
                             error * 0.0005 + error * 0.001)

    def test_python_oracle_checks_timestep_only_when_integrating(self,
                                                                 tmp_path):
        """The read sits inside the integral update: disengaged or
        traction-limited calls never reach it — even with a broken model."""
        model, _ = _cruise_model(tmp_path, 0.0005)
        data = mujoco.MjData(model)
        oracle = CruiseController(model, **CRUISE_CONFIG)
        data.qvel[oracle.root_x_dofadr] = oracle.target_speed_mps - .2
        model.opt.timestep = 0.
        oracle.compute(model, data, SimpleNamespace(rear_in_contact=False))
        oracle.compute(model, data, SimpleNamespace(rear_in_contact=True),
                       traction_limited=True)
        with pytest.raises(ValueError, match='timestep'):
            oracle.compute(model, data, SimpleNamespace(rear_in_contact=True))

    def test_native_matches_python_at_each_serialized_timestep(self,
                                                             tmp_path):
        """Every native Stepper integrates with ITS model's opt.timestep and
        matches the Python oracle bitwise — the binding calls the same
        per-call compute the direct C++ contract case covers."""
        integrals = {}
        for ts in (0.0005, 0.001, 0.0025):
            model, path = _cruise_model(tmp_path, ts)
            data = mujoco.MjData(model)
            oracle = CruiseController(model, **CRUISE_CONFIG)
            native = bike_native.Stepper(
                str(path), {'schema': 1, 'cruise': CRUISE_CONFIG})
            contacts = SimpleNamespace(rear_in_contact=True)
            for offset in (-.2, -.05, .1, -.3, .0, -.15):
                data.qvel[oracle.root_x_dofadr] = \
                    oracle.target_speed_mps + offset
                _cruise_push(native, data)
                expected = oracle.compute(model, data, contacts)
                assert_bitwise_equal(native.cruise_compute(True), expected)
                assert_bitwise_equal(
                    native.cruise_state()['integral_mps_s'],
                    oracle.integral_mps_s)
            integrals[ts] = oracle.integral_mps_s
        # The serialized timestep really entered the integrator — identical
        # speed sequences accumulated different integrals per model.
        assert integrals[0.0005] != integrals[0.001]
        assert integrals[0.001] != integrals[0.0025]
