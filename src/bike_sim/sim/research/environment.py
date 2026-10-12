"""Sample-and-hold research interface without Gym or additional dependencies.

Policies receive SensorObservation only. Truth/metrics are a separate evaluation
surface. This wrapper never adds forces or edits qpos/qvel to keep the bike up.
"""
from collections import deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from time import monotonic
import mujoco
from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder
from bike_sim.sim.ride.physical_session import configuration_metadata
from bike_sim.sim.ride.wheelie import WheelieTracker, WheelieTruth, truth_from_sample
from bike_sim.sim.research.observations import raw_observation
from bike_sim.sim.research.rider_behavior import RiderBehavior, signals_from_sample
from bike_sim.sim.research.rider_program import RiderProgram
from bike_sim.sim.research.sensors import SensorConfig, SensorObservation, SensorPipeline
from bike_sim.sim.research.demand import DemandProgram
from bike_sim.sim.research.quality import energy_quality


@dataclass(frozen=True)
class ExperimentConfig:
    control_period_s: float = .01
    actuator_delay_s: float = .005
    duration_s: float = 3.
    seed: int = 0
    record_decimation: int = 20
    wheelie_persistence_s: float = .02
    maximum_energy_residual_ratio: float = .05
    stop_on_model_violation: bool = True

    def __post_init__(self):
        for name in ('control_period_s', 'duration_s', 'maximum_energy_residual_ratio'):
            scalar(getattr(self, name), name, positive=True)
        for name in ('actuator_delay_s', 'wheelie_persistence_s'):
            scalar(getattr(self, name), name, minimum=0.)
        if type(self.stop_on_model_violation) is not bool:
            raise ValueError('stop_on_model_violation must be a bool')
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError('experiment seed must be a nonnegative integer')
        if type(self.record_decimation) is not int or self.record_decimation < 1:
            raise ValueError('record_decimation must be a positive integer')


@dataclass(frozen=True)
class ResearchStep:
    observation: SensorObservation
    truth: WheelieTruth
    contact_state: str
    terminated: bool
    truncated: bool
    reason: str | None
    physics_steps: int
    numerically_valid: bool = True
    demand_nm: float | None = None  # appended last: step() builds ResearchStep positionally
    model_valid: bool = True

    @property
    def valid_for_learning(self):
        return self.numerically_valid and self.model_valid and self.reason not in (
            'simulation_error', 'policy_error')


def crash_reason(event):
    """Map a latched CrashEvent to an outcome reason.

    pitch_over splits by sign because the two failures call for different
    policies: root pitch is negative when the nose is up (wheelie.py defines
    pitch_up = -qpos), so negative is a loop-out and positive is an endo.
    Other causes pass through; model_violation/numerical_quality are not crashes
    and never come through here.
    """
    if event.cause == 'pitch_over':
        return 'crash:loop_out' if event.pitch_rad < 0. else 'crash:endo'
    return 'crash:'+event.cause


def _integer_steps(seconds, dt, name):
    ratio = seconds/dt
    steps = round(ratio)
    if abs(ratio-steps) > 1e-8:
        raise ValueError(f'{name} must be an integer multiple of the physics timestep')
    return steps


class ResearchEnvironment:
    backend = "python"

    def __init__(self, sim, config=None, sensors=None, *, demand=None, rider_behavior=None, rider_program=None, capture_mode="legacy"):
        from bike_sim.recording.research import capture_mode as validate_capture_mode
        self.capture_mode = validate_capture_mode(capture_mode)
        self._keep_history = self.capture_mode in ('legacy', 'full')
        self._generation = -1
        self.sim = sim
        self.config = ExperimentConfig() if config is None else config
        self.sensor_config = SensorConfig() if sensors is None else sensors
        if not isinstance(self.config, ExperimentConfig) or not isinstance(self.sensor_config, SensorConfig):
            raise ValueError('expected ExperimentConfig and SensorConfig')
        if demand is not None and not isinstance(demand, DemandProgram):
            raise ValueError('demand must be a DemandProgram')
        if rider_program is not None and not isinstance(rider_program, RiderProgram):
            raise ValueError('rider_program must be a RiderProgram')
        if rider_behavior is not None and not isinstance(rider_behavior, RiderBehavior):
            raise ValueError('rider_behavior must implement the RiderBehavior protocol (reset, act)')
        if rider_behavior is not None and rider_program is not None:
            raise ValueError('rider_behavior and a posture-owning rider_program are exclusive')
        self.demand, self.rider_behavior, self.rider_program = demand, rider_behavior, rider_program
        cfg = sim.physics_config
        if cfg.physics_mode != 'physical' or cfg.drive_mode not in ('crank_effort', 'articulated_effort'):
            raise ValueError('research requires a physical effort drive, not a speed controller')
        if cfg.pitch_assist or cfg.tires.backend not in ('compliant_2d','distributed_2d_reference') or cfg.tires.surface_mode != 'track':
            raise ValueError('research requires no pitch assist and track-material compliant_2d tires')
        self.dt_s = float(sim.model.opt.timestep)
        self.control_steps = _integer_steps(self.config.control_period_s, self.dt_s, 'control period')
        self.delay_steps = _integer_steps(self.config.actuator_delay_s, self.dt_s, 'actuator delay')
        self.max_steps = _integer_steps(self.config.duration_s, self.dt_s, 'duration')
        self.sensor_steps = _integer_steps(self.sensor_config.sample_period_s, self.dt_s, 'sensor period')
        if self.sensor_steps < 1:
            raise ValueError('sensor period must span at least one physics step')
        if min(self.control_steps, self.max_steps) < 1:
            raise ValueError('control period and duration must span at least one physics step')
        if sim.steps != 0 or sim.time_s != 0.:
            raise ValueError('construct research environment from a freshly reset simulation')
        self.seed = self.config.seed
        self._begin_episode()

    def _begin_episode(self):
        self._generation += 1
        self._sealed_episode = None
        self._capture_samples = []
        self._capture_cursor = -1
        self._completed_count = self._requested_count = self._applied_base = 0
        self._last_requested = None
        self._window = None
        self._stop_requested = False
        self._closed = False
        from bike_sim.native.artifact import execution_provenance
        self.execution = execution_provenance("python")
        self.run_metadata = {}
        if self.rider_behavior is not None:
            self.rider_behavior.reset(self.seed)
        self.recorder = PhysicalRecorder(self.sim, decimate=self.config.record_decimation,
                                         enabled=self.capture_mode == 'legacy')
        self.sim.physical.set_recording_retention(
            'full' if self.capture_mode == 'full' else 'legacy' if self.capture_mode == 'legacy' else 'off')
        # Diagnostic mode may keep recording a violating episode; the run's
        # model_valid flag still goes false and stays false.
        self.sim.physical.set_strict(self.config.stop_on_model_violation)
        self.sim.physical.set_measurement_decimation(self.config.record_decimation)
        self.tracker = WheelieTracker(persistence_s=self.config.wheelie_persistence_s)
        self.pipeline = SensorPipeline(self.sensor_config, seed=self.seed)
        self.sim.data.qacc_warmstart.fill(0.)
        # Initial holding brakes belong to static equilibrium, not the policy
        # episode. Re-solve sensors for the same q/v with the safe startup input.
        self.sim.physical.apply_forces(active=True, advance=False,
            control=RideControl(motor_torque_nm=0., human_torque_nm=0.))
        mujoco.mj_forward(self.sim.model, self.sim.data)
        if self.sim.physics_config.seated_climb.enabled:
            from bike_sim.sim.ride.physical_observations import sensor_channels
            self.sim.physical.update_rider_intent_signals(sensor_channels(
                self.sim.physical, drive_channels=self.sim.physical.drive.probe_last))
        initial = raw_observation(self.sim)
        self.pipeline.reset(initial)
        self._sensor_time = initial.source_time_s
        self._next_sensor_step = self.sensor_steps
        self.observation = self.pipeline.read(self.sim.time_s)
        self._queue = deque()
        # Safe startup while a first controller command is in transport.
        self._applied = RideControl(motor_torque_nm=0.,
            human_torque_nm=None if self.sim.physics_config.seated_climb.enabled else 0.)
        self._motor_applied = self._applied
        self.commands_requested = []
        self.commands_applied = ([dict(time_s=0., step=0, control=asdict(self._applied))]
                                 if self.capture_mode != 'off' else [])
        self.observations = [self.observation] if self._keep_history else []
        self.trace = []
        self._sample_cursor = -1
        self.terminated = self.truncated = False
        self.reason = self.error = None
        self.last_truth = None
        self.numerically_valid = True
        self.max_energy_residual_ratio = 0.
        # N*m*s per control interval: delivered is the solved drive torque,
        # requested is the applied policy command (None = pedelec assist, no request).
        self.start_position_m = float(self.sim.position_m)
        self.torque_delivered_nms = 0.
        self.torque_requested_nms = 0.
        self.demand_integral_nms = None if self.demand is None else 0.
        # Peak |travel| at physics rate; the decimated recorder can miss the peak.
        self.max_shock_stroke_m = 0.
        self.max_fork_travel_m = 0.
        self.demand_nm = None if self.demand is None else self.demand.at(0.)
        self.metadata = configuration_metadata(self.sim, seed=self.seed)
        from bike_sim.sim.research.replay import integration_state
        from bike_sim.recording.contracts import freeze
        self.initial_integration_state = freeze(integration_state(self.sim))
        self._recording_assets = ()
        self._recording_catalog = None
        if self.capture_mode in ('full', 'legacy'):
            from bike_sim.recording.assets import snapshot_assets, research_recipe
            self._recording_assets = snapshot_assets(track=self.sim.track,
                vertices=self.sim.physical.vertices, metadata=self.metadata, recipe=research_recipe(self))
        if self.capture_mode == 'full':
            from bike_sim.recording.catalog import build_catalog
            from bike_sim.recording.native import model_joints
            self._recording_catalog = build_catalog(model_joints(self.sim.model))

    @property
    def done(self):
        return self.terminated or self.truncated

    @property
    def reference_monitor(self):
        return self.sim.physical.reference_monitor

    @property
    def model_valid(self):
        return self.sim.physical.model_status.as_dict()['model_valid']

    def reset(self, *, seed=None):
        if self._closed or self.control_pending:
            raise RuntimeError("reset requires an open environment at an external boundary")
        if seed is not None:
            if type(seed) is not int or seed < 0:
                raise ValueError('reset seed must be a nonnegative integer')
            self.seed = seed
        self.sim.reset()
        self._begin_episode()
        return self.observation

    def _retain_published_samples(self):
        if self.capture_mode != 'full':
            return
        from bike_sim.recording.research import OwnedSample
        # Retain all committed accounting rows before evaluation can stop on
        # an earlier outcome or fail. This does not create a control transition.
        for sample in self.sim.physical.completed_samples:
            if sample.interval_id > self._capture_cursor:
                owned = OwnedSample.capture(sample)
                self._capture_samples.append(owned)
                self._capture_cursor = sample.interval_id

    def _consume_completed_samples(self, *, stop_at_outcome=True):
        self._retain_published_samples()
        for sample in self.sim.physical.completed_samples:
            if sample.interval_id <= self._sample_cursor:
                continue
            # Claim before any external consumer side effect: a failed write
            # makes the run invalid and must never replay its partial effects.
            self._sample_cursor = sample.interval_id
            source_step = round(sample.time_s / self.dt_s)
            if source_step >= self._next_sensor_step:
                if source_step != self._next_sensor_step:
                    raise RuntimeError('sensor acquisition skipped a physical sample')
                raw = raw_observation(self.sim, sample)
                self.pipeline.push(raw)
                self._sensor_time = raw.source_time_s
                self._next_sensor_step += self.sensor_steps
            self.last_truth = truth_from_sample(self.sim, sample)
            suspension = sample.channels['suspension']
            self.max_shock_stroke_m = max(self.max_shock_stroke_m, abs(suspension['shock_stroke_m']))
            self.max_fork_travel_m = max(self.max_fork_travel_m, abs(suspension['fork_travel_m']))
            delivered = float(sample.channels['drive'].get('motor_torque_nm', 0.))
            if self.demand is not None:
                self.demand_integral_nms += self.demand.at(sample.time_s)*sample.dt_s
            applied = sample.channels['control']['motor_torque_nm']
            if applied is not None:
                self.torque_requested_nms += applied*sample.dt_s
            self.torque_delivered_nms += delivered*sample.dt_s
            self.tracker.update(self.last_truth, sample.dt_s, context={
                'delivered_motor_nm': delivered, 'applied_motor_nm': applied,
                'road_pitch_rad': self.last_truth.road_pitch_rad,
                'pitch_rate_up_rad_s': self.last_truth.pitch_rate_up_rad_s,
                'speed_mps': self.last_truth.speed_mps})
            self.recorder.record(self.sim, sample=sample)
            quality = energy_quality(sample.channels['energy'],
                maximum_ratio=self.config.maximum_energy_residual_ratio)
            self.max_energy_residual_ratio = max(self.max_energy_residual_ratio, quality.residual_ratio)
            if not quality.acceptable:
                self.numerically_valid = False
                self.truncated = True
                self.reason = 'numerical_quality'
                if stop_at_outcome:
                    break
            if self.config.stop_on_model_violation and not sample.channels['model_status']['model_valid']:
                self.truncated = True
                self.reason = 'model_violation'
                if stop_at_outcome:
                    break
            if self.sim.crash is not None and self.sim.crash.time_s <= sample.end_time_s:
                self.terminated = True
                self.reason = crash_reason(self.sim.crash)
                if stop_at_outcome:
                    break
            if self.last_truth.position_m >= self.sim.track.length_m:
                self.terminated = True
                self.reason = 'finish'
                if stop_at_outcome:
                    break

    @property
    def control_pending(self):
        return self._window is not None

    def step(self, control, *, front_brake_demand=0., rear_brake_demand=0.):
        self.begin_control(control, front_brake_demand=front_brake_demand,
                           rear_brake_demand=rear_brake_demand)
        return self.advance_control()

    def begin_control(self, control, *, front_brake_demand=0., rear_brake_demand=0.):
        if self._closed or self._sealed_episode is not None:
            raise RuntimeError('research environment is closed or sealed; reset before advancing')
        if self.control_pending:
            raise RuntimeError('an external control interval is already active')
        if self.done:
            raise RuntimeError('episode has ended; reset before stepping again')
        if not isinstance(control, RideControl):
            raise ValueError('expected an immutable RideControl')
        for value in (front_brake_demand, rear_brake_demand):
            if scalar(value, 'brake demand', minimum=0.) > 1.:
                raise ValueError('brake demand must lie in [0, 1]')
        # env.demand_nm is what the policy saw when it chose this command; it is
        # refreshed at the end of step() for the next call and never enforced.
        seen_demand_nm = self.demand_nm
        if self.rider_program is not None:
            self.rider_program.validate_control(control)
        elif self.rider_behavior is not None and control.posture is None:
            posture = self.rider_behavior.act(self.sim.time_s, signals_from_sample(self.sim.physical.sample))
            if posture is not None:
                control = replace(control, posture=posture)
        control.validate_for(self.sim.physics_config, self.sim.rider.variant)
        start_step = self.sim.steps
        request = dict(time_s=self.sim.time_s, step=start_step, control=asdict(control),
            front_brake_demand=front_brake_demand, rear_brake_demand=rear_brake_demand)
        target_step = min(start_step+self.control_steps, self.max_steps)
        window = (control, front_brake_demand, rear_brake_demand,
                  start_step, target_step, seen_demand_nm)
        next_count = self._requested_count + 1
        # Stage allocations before publishing the request/window pair.
        if self._keep_history:
            self.commands_requested.append(request)
        try:
            self._queue.append((start_step+self.delay_steps, control))
        except BaseException:
            if self._keep_history:
                self.commands_requested.pop()
            raise
        self._last_requested = request
        self._requested_count = next_count
        self._window = window

    def advance_control(self, *, wall_budget_s=None, target_step=None):
        if self._closed or self._sealed_episode is not None or not self.control_pending:
            raise RuntimeError("begin an external control interval before advancing")
        if wall_budget_s is not None:
            wall_budget_s = scalar(wall_budget_s, "wall_budget_s", minimum=0.)
        if target_step is not None and (type(target_step) is not int or target_step < self.sim.steps):
            raise ValueError('target_step must be an integer at or after the committed step')
        deadline = None if wall_budget_s is None else monotonic()+wall_budget_s
        control, front_brake_demand, rear_brake_demand, start_step, window_end_step, seen_demand_nm = self._window
        try:
            while self.sim.steps < window_end_step and not self.done:
                if target_step is not None and self.sim.steps >= target_step:
                    return None  # No accounting flush or new policy command on a pacing yield.
                if deadline is not None and monotonic() >= deadline:
                    return None
                while self._queue and self._queue[0][0] <= self.sim.steps:
                    _, self._motor_applied = self._queue.popleft()
                rider_control = (self.rider_program.apply(control, self.sim.time_s)
                                 if self.rider_program is not None else control)
                effective = replace(self._motor_applied,
                    human_torque_nm=rider_control.human_torque_nm,
                    crank_target_rate_rad_s=rider_control.crank_target_rate_rad_s,
                    posture=rider_control.posture, rider_enabled=rider_control.rider_enabled)
                if effective != self._applied:
                    if self.capture_mode != 'off':
                        self.commands_applied.append(dict(time_s=self.sim.time_s, step=self.sim.steps,
                                                           control=asdict(effective)))
                    self._applied = effective
                # Brakes are an immediate out-of-band safety input, not queued.
                self.sim.step(front_brake_demand, rear_brake_demand, control=self._applied)
                self._consume_completed_samples()
                if self.done:
                    break
            # End of the external policy window can leave a partial period.
            if self.sim.physical._buffer._raws:
                self.sim.physical.flush()
                self._consume_completed_samples()
        except Exception as exc:
            from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
            self.terminated = True
            primary_reason = ('invalid_controller' if isinstance(exc, InvalidReferenceRun)
                              else 'simulation_error')
            self.reason = primary_reason
            self.error = f'{type(exc).__name__}: {exc}'
            # Account a partial physical period even when integration or a
            # caller fails, without replacing the originating exception.
            try:
                if self.sim.physical._buffer._raws:
                    self.sim.physical.flush()
            except Exception as tail_error:
                exc.add_note(f'trailing interval check: {tail_error}')
            try:
                self._consume_completed_samples(stop_at_outcome=False)
            except Exception as consumption_error:
                exc.add_note(f'trailing sample consumption: {consumption_error}')
            finally:
                self.reason = primary_reason
            self._window = None
            raise  # Never manufacture a successful transition from invalid dynamics.
        if not self.done and self.sim.steps >= self.max_steps:
            self.truncated, self.reason = True, 'duration'
        self.demand_nm = None if self.demand is None else self.demand.at(self.sim.time_s)
        self.observation = self.pipeline.read(self.sim.time_s)
        result = ResearchStep(self.observation, self.last_truth, self.tracker.state,
                              self.terminated, self.truncated, self.reason, self.sim.steps-start_step, self.numerically_valid,
                              demand_nm=seen_demand_nm, model_valid=self.model_valid)
        if self._keep_history:
            self.trace.append(result)
            try:
                self.observations.append(self.observation)
            except BaseException:
                self.trace.pop()
                raise
        self._completed_count += 1
        self._window = None
        if self._stop_requested:
            self.stop()
        return result

    @property
    def last_requested(self):
        from bike_sim.recording.contracts import freeze
        return freeze(self._last_requested)

    def applied_since(self, cursor):
        from bike_sim.recording.contracts import _integer, freeze
        cursor = _integer(cursor, 'cursor')
        end = self._applied_base + len(self.commands_applied)
        if not self._applied_base <= cursor <= end:
            raise ValueError('applied cursor is stale or beyond the committed prefix')
        result = (freeze(self.commands_applied[cursor - self._applied_base:]), end)
        # Prepare the complete Python-owned result before consuming anything.
        if self.capture_mode == 'verify':
            del self.commands_applied[:end - self._applied_base]
            self._applied_base = end
        return result

    def seal_episode(self, episode_id, *, reason):
        if self.capture_mode != 'full':
            raise RuntimeError('sealing research evidence requires full capture')
        if self._sealed_episode is not None:
            if self._sealed_episode.episode_id != episode_id:
                raise ValueError('an episode cannot be sealed under two identities')
            return self._sealed_episode
        if self.sim.physical._buffer._raws:
            try:
                self.sim.physical.flush()
            except InvalidReferenceRun as error:
                # The monitor has published the failing prefix already. Own
                # that prefix without integrating or fabricating a transition.
                self.error = self.error or f'{type(error).__name__}: {error}'
                if not self.done:
                    self.terminated, self.reason = True, 'invalid_controller'
        self._retain_published_samples()
        from bike_sim.recording.research import OwnedPythonEpisode, ResearchHistory, seal_research
        from bike_sim.recording.native import NativePhysicalSource
        physical = NativePhysicalSource(OwnedPythonEpisode(self._generation, tuple(self._capture_samples)),
                                        self._recording_catalog)
        history = ResearchHistory(requested=self.commands_requested, applied=self.commands_applied,
                                  observations=self.observations, transitions=self.trace)
        evidence = seal_research(self, episode_id, reason=reason, physical=physical, history=history)
        self._capture_samples = physical.episode.samples
        self._sealed_episode = evidence
        return evidence

    def save(self, directory, *, overwrite=False):
        from bike_sim.native.recording import export_recording
        return export_recording(self, directory, overwrite=overwrite)

    def current_metadata(self):
        return configuration_metadata(self.sim, seed=self.seed)

    def stop(self):
        if self.done:
            return
        if self.control_pending:
            self._stop_requested = True
        else:
            self.truncated, self.reason = True, "operator_stop"

    def fail_policy(self, error):
        if self.control_pending:
            raise RuntimeError("cannot fail a policy inside an active control interval")
        if not self.done:
            self.terminated, self.reason = True, "policy_error"
            self.error = f"{type(error).__name__}: {error}"

    def close(self, *, discard_pending=False):
        if self.control_pending and not discard_pending:
            raise RuntimeError("finish the active interval before closing the environment")
        self._window = None
        self._closed = True
        owner = getattr(self, "_replay_assets_owner", None)
        if owner is not None:
            owner.cleanup()
            self._replay_assets_owner = None
