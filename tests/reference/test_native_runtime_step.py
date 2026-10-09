"""A3 — the owned physical step and bounded NativeRideRuntime.advance().

The native runtime steps the same compiled model the Python physical
runtime steps, in the same statement order. The integration vector
(mjSTATE_INTEGRATION: qpos/qvel/act/warmstart/time) is the strongest
per-step parity surface available before A4 drains sample rows — a
bitwise-equal vector certifies every force the solve consumed.
"""
import numpy as np
import pytest
import mujoco

from native_loader import load_native
from _native_runtime_support import make_python_ride, assert_tree_close
from bike_sim.native.runtime import create_native_ride
from bike_sim.sim.ride.control import RideControl


# The selected-artifact import doubles as the sanitizer-runtime check;
# create_native_ride reuses this module through load_native_extension.
bike_native = load_native()


@pytest.fixture
def sim():
    return make_python_ride()


def _integration_state(sim):
    expected = np.empty(
        mujoco.mj_stateSize(sim.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(sim.model, sim.data, expected,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    return expected


def test_physics_result_is_independent_of_call_chunks(sim):
    one = create_native_ride(sim, strict=False, record_decimation=1)
    chunks = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    one.advance(40, command)
    for target in (1, 3, 9, 17, 40):
        chunks.advance(target, command)
    a, b = one.snapshot(), chunks.snapshot()
    assert a.step == b.step == 40
    np.testing.assert_array_equal(a.integration_state, b.integration_state)


def test_native_first_step_uses_settled_material_state(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    sim.step(control=command)
    native.advance(1, command)
    expected = _integration_state(sim)
    np.testing.assert_allclose(
        native.snapshot().integration_state, expected,
        atol=1e-9, rtol=1e-9)


@pytest.mark.slow
def test_native_step_sequence_matches_python_state(sim):
    """Every committed interval lands on the oracle's integration vector."""
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    for target in range(1, 31):
        sim.step(control=command)
        result = native.advance(target, command)
        assert result.step == target
        assert result.reason == 'target'
        np.testing.assert_array_equal(
            native.snapshot().integration_state, _integration_state(sim))


@pytest.mark.slow
def test_native_force_fold_matches_python_recorded_inputs(sim):
    """probe_step_inputs replays apply_forces' staged fold without stepping.

    apply_forces(advance=False) leaves the ordered force accumulator, the
    actuator input row and the model brake bounds staged exactly like the
    step's pre-solve assembly; the native probe surfaces the same views.
    """
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    for front, rear in ((0., 0.), (.35, .2), (1., 1.)):
        probe = native.probe_step_inputs(
            command, front_brake_demand=front, rear_brake_demand=rear)
        sim.physical.apply_forces(active=True, advance=False, front=front,
                                  rear=rear, control=command)
        expected = sim.force_accumulator.components
        assert list(probe['components'].keys()) == list(expected.keys())
        for name, force in expected.items():
            np.testing.assert_allclose(
                probe['components'][name], force, atol=1e-9, rtol=1e-9,
                err_msg=name)
        np.testing.assert_allclose(probe['ctrl'], np.asarray(sim.data.ctrl),
                                   atol=1e-9, rtol=1e-9)
        assert probe['front_brake_bound_nm'] == pytest.approx(
            float(sim.model.dof_frictionloss[sim.physical.brake.front]),
            rel=0., abs=1e-12)
        assert probe['rear_brake_bound_nm'] == pytest.approx(
            float(sim.model.dof_frictionloss[sim.physical.brake.rear]),
            rel=0., abs=1e-12)
        # A probe is not a step: no committed state moves.
        assert native.snapshot().step == 0
        before = native.snapshot().integration_state
        native.reset()
        np.testing.assert_array_equal(
            native.snapshot().integration_state, before)


@pytest.mark.slow
def test_native_advance_tracks_python_under_load(sim):
    """Brake application and release reproduce the oracle interval-for-interval."""
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    demands = [(0., 0.)] * 10 + [(0.6, 0.6)] * 10 + [(0., 0.)] * 10
    for target, (front, rear) in enumerate(demands, start=1):
        sim.step(front, rear, control=command)
        native.advance(target, command, front_brake_demand=front,
                       rear_brake_demand=rear)
        np.testing.assert_array_equal(
            native.snapshot().integration_state, _integration_state(sim),
            err_msg=f'interval {target}')


def test_reset_after_advance_replays_the_bootstrap(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    replay = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    native.advance(15, command)
    native.reset()
    assert native.snapshot().step == 0
    native.advance(7, command)
    replay.advance(7, command)
    np.testing.assert_array_equal(
        native.snapshot().integration_state,
        replay.snapshot().integration_state)


def test_snapshot_is_an_owned_copy(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    native.advance(3, RideControl())
    snap = native.snapshot()
    # The contract is an owned, read-only copy (contracts.py): caller-held
    # storage can never alias the runtime's live buffers or its scratch.
    assert snap.integration_state.flags.owndata
    assert not snap.integration_state.flags.writeable
    native.advance(4, RideControl())
    later = native.snapshot()
    # A live view would show step-4 state; the held snapshot must still
    # report step 3 and share no memory with the new copy.
    assert snap.step == 3
    assert later.step == 4
    assert not np.array_equal(snap.integration_state,
                              later.integration_state)
    assert not np.shares_memory(snap.integration_state,
                                later.integration_state)
    # Mutating our own copy of the held data cannot reach the runtime.
    held = snap.integration_state.copy()
    held[:] = 0.
    np.testing.assert_array_equal(native.snapshot().integration_state,
                                  later.integration_state)


def test_probe_step_inputs_returns_staged_diagnostics(sim):
    native = create_native_ride(sim, strict=False, record_decimation=1)
    probe = native.probe_step_inputs(
        RideControl(), front_brake_demand=0., rear_brake_demand=0.)
    assert isinstance(probe['drive'], dict)
    assert probe['contact_probe'] is None or isinstance(
        probe['contact_probe'], dict)


def test_sensor_channels_read_sensordata_rows(sim):
    """frame_gyro/frame_accel come from sensordata[sensor_adr[id]...], not
    the sensor id — on the pinned model id=13 but adr=15, so an id-indexed
    read returns [accel_y, accel_z, gyro_x] instead of the gyro row."""
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    for target in range(1, 6):
        sim.step(control=command)
        native.advance(target, command)
        # probe_step_inputs stages apply_forces(advance=False), which
        # re-forwards sensordata with the staged forces — stage the same
        # inputs on the oracle first so both read post-staging rows.
        sim.physical.apply_forces(active=True, advance=False, front=0.,
                                  rear=0., control=command)
        probe = native.probe_step_inputs(command)
        # d.sensor(name).data is the oracle lane (physical_observations.py
        # :182-183); bitwise equal, not just close.
        np.testing.assert_array_equal(
            np.asarray(probe['sensors']['frame_gyro_body_rad_s']),
            sim.data.sensor('sensor_frame_gyro').data)
        np.testing.assert_array_equal(
            np.asarray(probe['sensors']['frame_specific_force_body_mps2']),
            sim.data.sensor('sensor_frame_accel').data)


@pytest.mark.slow
def test_native_intent_lanes_match_python_past_reaction_delay(sim):
    """The welded profile runs seated_climb (period .01 s = 8 steps at
    dt=.00125) with reaction_delay_s=.15: past ~step 120 the delayed
    samples drive inclination_rad. A corrupted gyro lane would integrate
    pitch_rate_up ~ -9.8 rad/s and diverge ~1 rad by then."""
    native = create_native_ride(sim, strict=False, record_decimation=1)
    command = RideControl()
    for target in range(1, 161):
        sim.step(control=command)
        native.advance(target, command)
    assert native.snapshot().step == 160
    # Snapshot before the probe: apply_forces staging rewrites
    # qacc_warmstart/sensordata lanes the integration vector carries.
    np.testing.assert_array_equal(
        native.snapshot().integration_state, _integration_state(sim))
    probe = native.probe_step_inputs(command)
    expected = sim.physical.rider_intent_signals
    signals = probe['intent_signals']
    assert signals['pitch_rate_up_rad_s'] == expected.pitch_rate_up_rad_s
    np.testing.assert_array_equal(
        np.asarray(signals['specific_force_body_mps2']),
        np.asarray(expected.specific_force_body_mps2))
    assert signals['crank_rate_rad_s'] == expected.crank_rate_rad_s
    assert signals['human_crank_torque_nm'] == expected.human_crank_torque_nm
    assert signals['front_load_share'] == expected.front_load_share
    assert (probe['intent_inclination_rad'] ==
            sim.physical.rider_intent.policy.inclination_rad)
