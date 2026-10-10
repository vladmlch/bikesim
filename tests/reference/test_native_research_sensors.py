from copy import deepcopy
from dataclasses import asdict, replace
import numpy as np
import pytest
from native_loader import load_native
from bike_sim.sim.research.sensor_noise import build_noise_tape
from bike_sim.sim.research.sensors import SensorConfig, SensorObservation, SensorPipeline


# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def native_pipeline(module, config, *, count=32, seed=41):
    tape = build_noise_tape(config, seed=seed, count=count)
    return module.NativeSensorPipeline(asdict(config), tape.noise, tape.dropout_uniform)


@pytest.mark.parametrize('latency', [0., .003, .02])
@pytest.mark.parametrize('dropout', [0., .4, 1.])
@pytest.mark.parametrize('imu', [False, True])
def test_noise_latency_dropout_and_stale_match_python(native_module, raw_sensor, assert_tree,
                                                     latency, dropout, imu):
    config = replace(SensorConfig(), latency_s=latency, maximum_age_s=.015,
        dropout_probability=dropout, imu_enabled=imu,
        acceleration_bias_mps2=(.5, -.25, .125), gyro_bias_rad_s=.02)
    reference, native = SensorPipeline(config, seed=41), native_pipeline(native_module, config)
    reference.reset(raw_sensor())
    native.reset(asdict(raw_sensor()))
    for index in range(24):
        time_s = index*.005
        if index:
            reference.push(raw_sensor(time_s))
            native.push(asdict(raw_sensor(time_s)))
        expected = reference.read(time_s)
        actual = SensorObservation(**native.read(time_s))
        assert_tree(actual, expected, atol=0., rtol=0.)
        before = native.state_dict()
        assert_tree(native.read(time_s), asdict(actual), atol=0., rtol=0.)
        assert_tree(native.state_dict(), before, atol=0., rtol=0.)
        assert_tree(native.state_dict(), reference.state_dict(), atol=0., rtol=0.)
    assert not native.read(1.)['valid']
    assert not reference.read(1.).valid
    assert_tree(native.state_dict(), reference.state_dict(), atol=0., rtol=0.)


def test_imported_startup_row_is_not_drawn_twice(native_module, raw_sensor, assert_tree):
    config = replace(SensorConfig(), latency_s=0.)
    reference = SensorPipeline(config, seed=41)
    reference.reset(raw_sensor())
    expected_initial = reference.read(0.)
    native = native_pipeline(native_module, config)
    native.import_state(reference.state_dict())
    assert_tree(native.read(0.), asdict(expected_initial), atol=0., rtol=0.)
    assert native.state_dict()['cursor'] == 1
    reference.push(raw_sensor(.005))
    native.push(asdict(raw_sensor(.005)))
    assert_tree(native.read(.005), asdict(reference.read(.005)), atol=0., rtol=0.)
    assert native.state_dict()['cursor'] == 2


def test_invalid_operations_and_exhaustion_preserve_state(native_module, raw_sensor, assert_tree):
    native = native_pipeline(native_module, SensorConfig.ideal(), count=2)
    native.reset(asdict(raw_sensor()))
    native.read(.01)
    before = native.state_dict()
    for raw in (asdict(raw_sensor()), dict(asdict(raw_sensor(.005)), valid=1),
                dict(asdict(raw_sensor(.005)), source_time_s=.01)):
        with pytest.raises(ValueError):
            native.push(raw)
        assert_tree(native.state_dict(), before, atol=0., rtol=0.)
    for time_s in (-1., .009, float('nan'), True):
        with pytest.raises(ValueError):
            native.read(time_s)
        assert_tree(native.state_dict(), before, atol=0., rtol=0.)
    invalid = deepcopy(before)
    invalid['cursor'] += 1
    with pytest.raises(ValueError):
        native.import_state(invalid)
    assert_tree(native.state_dict(), before, atol=0., rtol=0.)
    native.push(asdict(raw_sensor(.005)))
    before = native.state_dict()
    with pytest.raises(RuntimeError, match='exhausted'):
        native.push(asdict(raw_sensor(.01)))
    assert_tree(native.state_dict(), before, atol=0., rtol=0.)


def test_irregular_delivery_schedule_matches_python(native_module, raw_sensor, assert_tree):
    config = replace(SensorConfig(), latency_s=.004, maximum_age_s=.02,
        acceleration_bias_mps2=(.05, -.01, .02), gyro_bias_rad_s=.003,
        dropout_probability=.15)
    reference = SensorPipeline(config, seed=11)
    native = native_pipeline(native_module, config, count=8, seed=11)
    reference.reset(raw_sensor())
    native.reset(asdict(raw_sensor()))
    pushes = [.005, .010, .015, .025, .030]
    reads = [.0032, .0067, .0119, .0161, .0204, .0276, .0311, .045]
    for read_at in reads:
        while pushes and pushes[0] <= read_at:
            raw = raw_sensor(pushes.pop(0))
            reference.push(raw)
            native.push(asdict(raw))
        assert_tree(native.read(read_at), asdict(reference.read(read_at)), atol=0., rtol=0.)
        assert_tree(native.state_dict(), reference.state_dict(), atol=0., rtol=0.)


def test_same_seed_reset_repeats_and_different_seeds_differ(native_module, raw_sensor, assert_tree):
    config = replace(SensorConfig(), latency_s=0., dropout_probability=.3)

    def python_sequence(pipeline):
        pipeline.reset(raw_sensor())
        out = []
        for index in range(1, 6):
            raw = raw_sensor(index*.005)
            pipeline.push(raw)
            out.append(pipeline.read(index*.005))
        return out

    def native_sequence(pipeline):
        pipeline.reset(asdict(raw_sensor()))
        out = []
        for index in range(1, 6):
            pipeline.push(asdict(raw_sensor(index*.005)))
            out.append(pipeline.read(index*.005))
        return out

    reference = SensorPipeline(config, seed=7)
    native = native_pipeline(native_module, config, count=6, seed=7)
    first_reference = python_sequence(reference)
    first_native = native_sequence(native)
    # Reset replays the same seed: identical tape, identical deliveries.
    assert_tree(python_sequence(reference), first_reference, atol=0., rtol=0.)
    assert_tree(native_sequence(native), first_native, atol=0., rtol=0.)
    assert_tree(first_native, [asdict(o) for o in first_reference], atol=0., rtol=0.)
    other = python_sequence(SensorPipeline(config, seed=8))
    differences = sum(a != b for a, b in zip(first_reference, other))
    assert differences > 0


def test_native_owns_noise_and_validates_empty_shape(native_module, raw_sensor):
    config = asdict(SensorConfig.ideal())
    noise, dropout = np.zeros((2, 9)), np.full(2, .5)
    native = native_module.NativeSensorPipeline(config, noise, dropout)
    noise.fill(100.)
    dropout.fill(0.)
    native.reset(asdict(raw_sensor()))
    assert native.read(0.)['motor_torque_nm'] == 5.
    empty = native_module.NativeSensorPipeline(config, np.empty((0, 9)), np.empty(0))
    with pytest.raises(RuntimeError, match='exhausted'):
        empty.reset(asdict(raw_sensor()))
    assert empty.state_dict()['cursor'] == 0
    for bad in (np.empty((0, 8)), np.empty(0), np.empty((1, 9), dtype=bool)):
        with pytest.raises(ValueError):
            native_module.NativeSensorPipeline(config, bad, np.empty(0))
