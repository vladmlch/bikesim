from dataclasses import asdict
import numpy as np
import pytest
from native_loader import load_native
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.demand import DemandProgram
from bike_sim.sim.research.metrics import episode_metrics
from bike_sim.sim.research.replay import integration_state
from bike_sim.sim.research.rider_program import RiderKeyframe, RiderProgram


# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


@pytest.mark.slow
def test_native_research_matches_every_external_transition(assert_tree):
    """The plan's paired gate on the rough-track assist fixture."""
    from _native_runtime_support import make_python_research
    from bike_sim.native.research import create_native_research
    reference = make_python_research()
    native = create_native_research(reference)
    assert_tree(asdict(native.observation), asdict(reference.observation))
    while not reference.done and not native.done:
        wanted = reference.step(RideControl())
        got = native.step(RideControl())
        assert_tree(asdict(got), asdict(wanted))
        assert native.sim.steps == reference.sim.steps
        assert native.sim.time_s == reference.sim.time_s
    # Same outcome contract as the one-second gate: the oracle decides, the
    # native owner must agree step-for-step (both 'duration' at step 80 here).
    assert native.done and reference.done
    assert native.sim.steps == reference.sim.steps == reference.max_steps
    assert native.reason == reference.reason == 'duration'
    assert_tree(native.commands_applied, reference.commands_applied, atol=0., rtol=0.)
    assert_tree(native.commands_requested, reference.commands_requested, atol=0., rtol=0.)


def test_one_second_pair_with_one_e_minus_nine_gate(environment_pair, assert_tree):
    """Release gate, not a pre-existing result: do not shorten on early failure."""
    reference, native = environment_pair(duration=1., decimation=20)
    command = RideControl(motor_torque_nm=0.)
    assert_tree(native.observation, reference.observation, atol=0., rtol=0.)
    while not reference.done and not native.done:
        expected, actual = reference.step(command), native.step(command)
        assert_tree(actual, expected)
        assert native.sim.steps == reference.sim.steps
        assert native.sim.time_s == reference.sim.time_s
        np.testing.assert_allclose(integration_state(native.sim), integration_state(reference.sim),
                                   atol=1e-9, rtol=1e-9)
    # The Python oracle defines the outcome; this platform's documented
    # baseline physics (energy.constraint_work from the first intervals)
    # truncates this run near step 104 with 'numerical_quality'. The gate is
    # cross-backend equality of steps/reason, not a scripted outcome.
    assert reference.done and native.done
    assert native.sim.steps == reference.sim.steps
    assert native.reason == reference.reason
    assert_tree(native.commands_requested, reference.commands_requested, atol=0., rtol=0.)
    assert_tree(native.commands_applied, reference.commands_applied, atol=0., rtol=0.)
    assert_tree(episode_metrics(native), episode_metrics(reference))
    assert_tree(native.recorder.columns(), reference.recorder.columns())


@pytest.mark.parametrize('decimation', [1, 7, 20, 80])
def test_delay_full_commands_immediate_brakes_and_recording(environment_pair, assert_tree, decimation):
    reference, native = environment_pair(duration=.04, decimation=decimation)
    commands = [RideControl(motor_torque_nm=None, motor_limit_nm=0., human_torque_nm=0.,
                           crank_target_rate_rad_s=0., posture=RiderPosture()),
                RideControl(0., None, 1., None, RiderPosture(.01), True),
                RideControl(2., .5, 0., 0., None, False), RideControl()]
    for index, command in enumerate(commands):
        brakes = dict(front_brake_demand=.2 if index == 1 else 0., rear_brake_demand=.1 if index == 2 else 0.)
        assert_tree(native.step(command, **brakes), reference.step(command, **brakes))
        assert_tree(native.commands_applied, reference.commands_applied, atol=0., rtol=0.)
        assert_tree(native.commands_requested, reference.commands_requested, atol=0., rtol=0.)
        if reference.done or native.done:
            assert reference.done and native.done
            break
    assert_tree(native.recorder.columns(), reference.recorder.columns())
    assert_tree(episode_metrics(native), episode_metrics(reference))
    assert native.commands_requested[0]['control']['motor_torque_nm'] is None
    # Rider fields are immediate, while the motor transport starts with the
    # safe bootstrap command. Check the actual recorded interval commands.
    first = native.recorder.samples[0]['control']
    assert first['motor_torque_nm'] == 0.
    assert first['human_torque_nm'] == 0.


def test_scripted_rider_and_demand_are_sampled_at_physics_rate(environment_pair, assert_tree):
    program = RiderProgram((RiderKeyframe(0., RiderPosture(), 0.),
        RiderKeyframe(.021, RiderPosture(.01), 1.),
        RiderKeyframe(.04, RiderPosture(), 0.)), reaction_delay_s=.0025)
    demand = DemandProgram(((0., 0.), (.013, 5.), (.04, 1.)))
    reference, native = environment_pair(program=program, demand=demand, decimation=3)
    for _ in range(4):
        command = RideControl(motor_torque_nm=native.demand_nm)
        assert_tree(native.step(command), reference.step(command))
        if native.done or reference.done:
            break
    assert_tree(native.commands_applied, reference.commands_applied, atol=0., rtol=0.)
    assert len(native.commands_applied) > len(native.commands_requested)
    assert_tree(episode_metrics(native), episode_metrics(reference))


def test_zero_budget_retains_single_window_and_no_tail_flush(environment_pair, assert_tree):
    reference, native = environment_pair()
    command = RideControl(0.)
    before = native.snapshot()
    native.begin_control(command)
    for _ in range(5):
        assert native.advance_control(wall_budget_s=0.) is None
        assert native.control_pending
        assert native.sim.steps == 0
        assert native.recorder.rows == 0
        assert len(native.commands_requested) == 1
        assert len(native.observations) == 1
        assert len(native.trace) == 0
    np.testing.assert_array_equal(native.snapshot().integration_state, before.integration_state)
    assert_tree(native.advance_control(), reference.step(command))
    assert not native.control_pending
    assert len(native.trace) == 1
    with pytest.raises(RuntimeError):
        native.advance_control()


def test_publication_failure_retry_does_not_advance_twice(environment_pair, monkeypatch, assert_tree):
    import bike_sim.native.research as adapter
    reference, native = environment_pair()
    native.begin_control(RideControl(0.))
    with monkeypatch.context() as patch:
        def fail_transition(_):
            raise MemoryError('injected Python transition publication failure')
        patch.setattr(adapter, '_transition', fail_transition)
        with pytest.raises(MemoryError, match='publication'):
            native.advance_control()
    assert native.control_pending
    committed = native._native.status()['step']
    assert committed == native.control_steps
    assert_tree(native.advance_control(), reference.step(RideControl(0.)))
    assert native.sim.steps == committed
    assert len(native.trace) == len(native.commands_requested) == 1


def test_native_loop_never_calls_python_interval_or_rng(environment_pair, monkeypatch):
    reference, native = environment_pair()
    def forbidden(*args, **kwargs):
        raise AssertionError('Python entered the native interval loop')
    monkeypatch.setattr(reference.sim, 'step', forbidden)
    monkeypatch.setattr(reference.sim.physical, 'flush', forbidden)
    import bike_sim.sim.research.sensor_noise as noise
    monkeypatch.setattr(noise.SensorNoiseSource, 'draw', forbidden)
    assert native.step(RideControl(0.)).physics_steps == native.control_steps


def test_reset_seed_generation_snapshot_and_facade_ownership(environment_pair, assert_tree):
    reference, native = environment_pair()
    snapshot = native.snapshot()
    original = snapshot.integration_state.copy()
    with pytest.raises(ValueError):
        snapshot.integration_state[0] = 1.
    for name in ('model', 'data', 'step'):
        with pytest.raises(AttributeError):
            getattr(native.sim, name)
    with pytest.raises(AttributeError):
        native.sim.time_s = 1.
    with pytest.raises(AttributeError):
        native.terminated = True
    native.step(RideControl(0.))
    assert_tree(native.reset(seed=73), reference.reset(seed=73), atol=0., rtol=0.)
    assert native.snapshot().generation > snapshot.generation
    assert native.sim.steps == 0
    assert native.recorder.rows == 0
    assert len(native.commands_requested) == len(native.trace) == 0
    assert native.pipeline.noise_cursor == 1
    # Queues and accumulated metrics are cleared; the startup command is re-seated.
    assert native.torque_delivered_nms == native.torque_requested_nms == 0.
    assert native.max_energy_residual_ratio == 0.
    assert native.demand_integral_nms == reference.demand_integral_nms
    assert len(native.commands_applied) == len(reference.commands_applied) == 1
    assert_tree(episode_metrics(native), episode_metrics(reference))
    np.testing.assert_array_equal(snapshot.integration_state, original)
    assert_tree(native.metadata, reference.metadata, atol=0., rtol=0.)
    assert_tree(native.step(RideControl(0.)), reference.step(RideControl(0.)))


def test_invalid_boundary_calls_do_not_change_state(environment_pair, assert_tree):
    _, native = environment_pair()
    before = native.snapshot()
    with pytest.raises(ValueError):
        native.begin_control(RideControl(), front_brake_demand=True)
    assert not native.control_pending
    native.begin_control(RideControl(0.))
    with pytest.raises(RuntimeError):
        native.begin_control(RideControl(0.))
    for budget in (True, -.1, float('nan')):
        with pytest.raises(ValueError):
            native.advance_control(wall_budget_s=budget)
        assert native.control_pending
    with pytest.raises(RuntimeError):
        native.reset()
    with pytest.raises(RuntimeError):
        native.close()
    np.testing.assert_array_equal(native.snapshot().integration_state, before.integration_state)
    native.stop()
    assert not native.done
    native.advance_control()
    assert native.done and native.reason == 'operator_stop'


class BoundaryRider:
    def __init__(self):
        self.seeds, self.times = [], []

    def reset(self, seed):
        self.seeds.append(seed)
        self.times.clear()

    def act(self, time_s, signals):
        self.times.append(time_s)
        return RiderPosture()


def test_custom_rider_callback_runs_once_per_external_boundary(environment_pair, assert_tree):
    behavior = BoundaryRider()
    reference, native = environment_pair(behavior=behavior)
    assert native.rider_behavior is not behavior
    native.begin_control(RideControl(0.))
    for _ in range(3):
        assert native.advance_control(wall_budget_s=0.) is None
    assert native.rider_behavior.times == [0.]
    assert behavior.times == []
    assert_tree(native.advance_control(), reference.step(RideControl(0.)))
    assert behavior.times == native.rider_behavior.times == [0.]
    native.reset(seed=59)
    assert native.rider_behavior.seeds == [19, 59]
    assert native.rider_behavior.times == []


@pytest.mark.parametrize('strict', [False, True])
def test_strict_and_diagnostic_outcomes_remain_visible(environment_pair, assert_tree, strict):
    from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
    reference, native = environment_pair(strict=strict)
    for _ in range(4):
        results, errors = [], []
        for env in (reference, native):
            try:
                results.append(env.step(RideControl(0.)))
                errors.append(None)
            except InvalidReferenceRun as error:
                results.append(None)
                errors.append(error)
        assert (errors[0] is None) == (errors[1] is None)
        assert native.sim.steps == reference.sim.steps
        assert native.reason == reference.reason
        assert native.model_valid == reference.model_valid
        if errors[0] is not None:
            assert strict and native.reason == 'invalid_controller'
            assert native.reference_monitor.first_failure is not None
            assert_tree(native.reference_monitor.first_failure, reference.reference_monitor.first_failure)
            break
        assert_tree(results[1], results[0])
        if reference.done or native.done:
            assert native.done and reference.done
            break


def test_stop_on_ended_episode_preserves_original_reason(environment_pair):
    reference, native = environment_pair(duration=.02)
    while not reference.done:
        reference.step(RideControl(0.))
        native.step(RideControl(0.))
    assert reference.done and native.done
    assert native.reason == reference.reason == 'duration'
    reference.stop()
    native.stop()
    # An operator stop after the episode must not relabel its real outcome.
    assert native.reason == reference.reason == 'duration'
