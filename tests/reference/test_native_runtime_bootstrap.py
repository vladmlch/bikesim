"""A1 — NativeRideRuntime construction, bootstrap ownership and restore.

The owning boundary: the runtime decodes the capture once, holds its own
copies, and replays them on reset(). Rejections name the state/config field
path; a failed construction leaves nothing partially restored.
"""
import copy
import gc

import numpy as np
import pytest

from native_loader import load_native
from _native_runtime_support import make_python_ride
from bike_sim.native.setup import capture_bootstrap


@pytest.fixture(scope='module')
def sim():
    return make_python_ride()


@pytest.fixture()
def setup(sim, tmp_path):
    return capture_bootstrap(sim, tmp_path)


@pytest.fixture()
def native_runtime(setup):
    return load_native().NativeRideRuntime(
        str(setup.model_path), setup.config, setup.state)


@pytest.mark.slow
def test_bootstrap_owns_input_and_restores_complete_start(setup, sim):
    native = load_native().NativeRideRuntime(
        str(setup.model_path), setup.config, setup.state)
    before = native.snapshot()
    setup.state["integration_state"][:] = 0.0
    sim.data.qpos[:] = sim.data.qpos + 0.01
    np.testing.assert_array_equal(native.snapshot().integration_state,
                                  before.integration_state)
    native.reset()
    after = native.snapshot()
    np.testing.assert_array_equal(after.integration_state,
                                  before.integration_state)
    assert after.step == 0
    assert after.generation == before.generation + 1


@pytest.mark.slow
def test_snapshot_reports_captured_counters(native_runtime, setup):
    snap = native_runtime.snapshot()
    assert snap.step == setup.state['runtime']['step'] == 0
    assert snap.generation == setup.state['runtime']['generation']
    assert snap.time_s == pytest.approx(0.0, abs=0.0)
    assert snap.integration_state.dtype == np.float64
    assert snap.integration_state.flags.owndata


@pytest.mark.slow
def test_bootstrap_survives_caller_destruction(setup):
    native = load_native().NativeRideRuntime(
        str(setup.model_path), copy.deepcopy(setup.config),
        copy.deepcopy(setup.state))
    before = native.snapshot()
    del setup
    gc.collect()
    after = native.snapshot()
    np.testing.assert_array_equal(after.integration_state,
                                  before.integration_state)


@pytest.mark.slow
def test_reset_twice_replays_the_same_bootstrap(native_runtime):
    first = native_runtime.snapshot().integration_state
    native_runtime.reset()
    native_runtime.reset()
    third = native_runtime.snapshot()
    np.testing.assert_array_equal(third.integration_state, first)
    assert third.generation == 3


@pytest.mark.slow
def test_close_releases_and_fails_fast(native_runtime):
    native_runtime.close()
    native_runtime.close()  # idempotent
    assert native_runtime.closed
    with pytest.raises(Exception, match='closed'):
        native_runtime.snapshot()
    with pytest.raises(Exception, match='closed'):
        native_runtime.reset()


@pytest.mark.slow
def test_config_mutation_after_construction_is_isolated(setup):
    state_vec = setup.state['integration_state'].copy()
    native = load_native().NativeRideRuntime(
        str(setup.model_path), setup.config, setup.state)
    setup.config['timestep_s'] = -1.0
    setup.config['monitors']['balance_floor_mps'] = -5.0
    setup.state['runtime']['step'] = 999
    np.testing.assert_array_equal(native.snapshot().integration_state,
                                  state_vec)
    assert native.snapshot().step == 0


# ---- validation surface -------------------------------------------------

def test_rejects_missing_state_key(setup):
    broken = copy.deepcopy(setup.state)
    del broken['drive']
    with pytest.raises(ValueError, match='state.drive'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_unknown_state_key(setup):
    broken = copy.deepcopy(setup.state)
    broken['mystery'] = 1
    with pytest.raises(ValueError, match='state.mystery'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_digest_mismatch(setup):
    broken = copy.deepcopy(setup.state)
    broken['model_digest'] = 'f' * 64
    with pytest.raises(ValueError, match='model_digest'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_model_dims_mismatch(setup):
    broken = copy.deepcopy(setup.state)
    broken['model_dims']['nq'] += 1
    with pytest.raises(ValueError, match='model_dims.nq'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_integration_width_mismatch(setup):
    broken = copy.deepcopy(setup.state)
    broken['integration_state'] = broken['integration_state'][:-1]
    with pytest.raises(ValueError, match='integration_state'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_nonfinite_integration_lane(setup):
    broken = copy.deepcopy(setup.state)
    broken['integration_state'][3] = np.nan
    with pytest.raises(ValueError, match='integration_state'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_malformed_drive_state(setup):
    broken = copy.deepcopy(setup.state)
    del broken['drive']['policies']
    with pytest.raises(ValueError, match='drive'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_unknown_joint_in_mutable_state(setup):
    broken = copy.deepcopy(setup.state)
    broken['model_mutable']['dof_frictionloss']['front']['joint'] = 'no_such'
    with pytest.raises(ValueError, match='dof_frictionloss'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_unknown_runtime_key(setup):
    broken = copy.deepcopy(setup.state)
    del broken['runtime']['balance']
    with pytest.raises(ValueError, match='runtime.balance'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_schema_version(setup):
    broken = copy.deepcopy(setup.state)
    broken['schema'] = 99
    with pytest.raises(ValueError, match='schema'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, broken)


def test_rejects_bad_runtime_config_envelope(setup):
    config = copy.deepcopy(setup.config)
    config['runtime_schema'] = 2
    with pytest.raises(ValueError, match='runtime_schema'):
        load_native().NativeRideRuntime(
            str(setup.model_path), config, setup.state)


def test_rejects_control_period_inconsistency(setup):
    config = copy.deepcopy(setup.config)
    config['control_period_steps'] += 1
    with pytest.raises(ValueError, match='control_period'):
        load_native().NativeRideRuntime(
            str(setup.model_path), config, setup.state)


def test_rejects_missing_model_file(setup):
    import os
    os.remove(setup.model_path)
    with pytest.raises((ValueError, RuntimeError), match='model'):
        load_native().NativeRideRuntime(
            str(setup.model_path), setup.config, setup.state)
