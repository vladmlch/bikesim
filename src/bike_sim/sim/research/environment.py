"""Sample-and-hold research interface without Gym or additional dependencies.

Policies receive SensorObservation only. Truth/metrics are a separate evaluation
surface. This wrapper never adds forces or edits qpos/qvel to keep the bike up.
"""
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
import csv
import hashlib
import json
import numpy as np
import mujoco
from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder
from bike_sim.sim.ride.physical_session import configuration_metadata, physical_summary
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.wheelie import WheelieTracker, WheelieTruth, truth_from_sample
from bike_sim.sim.research.observations import raw_observation
from bike_sim.sim.research.sensors import SensorConfig, SensorObservation, SensorPipeline
from bike_sim.terrain.trackfile import save_track
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


def _integer_steps(seconds, dt, name):
    ratio = seconds/dt
    steps = round(ratio)
    if abs(ratio-steps) > 1e-8:
        raise ValueError(f'{name} must be an integer multiple of the physics timestep')
    return steps


class ResearchEnvironment:
    def __init__(self, sim, config=None, sensors=None):
        self.sim = sim
        self.config = ExperimentConfig() if config is None else config
        self.sensor_config = SensorConfig() if sensors is None else sensors
        if not isinstance(self.config, ExperimentConfig) or not isinstance(self.sensor_config, SensorConfig):
            raise ValueError('expected ExperimentConfig and SensorConfig')
        cfg = sim.physics_config
        if cfg.physics_mode != 'physical' or cfg.drive_mode not in ('crank_effort', 'articulated_effort'):
            raise ValueError('research requires a physical effort drive, not a speed controller')
        if cfg.pitch_assist or cfg.tires.backend not in ('compliant_2d','distributed_2d_reference') or cfg.tires.surface_mode != 'track':
            raise ValueError('research requires no pitch assist and track-material compliant_2d tires')
        self.dt_s = float(sim.model.opt.timestep)
        self.control_steps = _integer_steps(self.config.control_period_s, self.dt_s, 'control period')
        self.delay_steps = _integer_steps(self.config.actuator_delay_s, self.dt_s, 'actuator delay')
        self.max_steps = _integer_steps(self.config.duration_s, self.dt_s, 'duration')
        if min(self.control_steps, self.max_steps) < 1:
            raise ValueError('control period and duration must span at least one physics step')
        if sim.steps != 0 or sim.time_s != 0.:
            raise ValueError('construct research environment from a freshly reset simulation')
        self.seed = self.config.seed
        self._begin_episode()

    def _begin_episode(self):
        self.recorder = PhysicalRecorder(self.sim, decimate=self.config.record_decimation)
        self.tracker = WheelieTracker(persistence_s=self.config.wheelie_persistence_s)
        self.pipeline = SensorPipeline(self.sensor_config, seed=self.seed)
        # Initial holding brakes belong to static equilibrium, not the policy
        # episode. Re-solve sensors for the same q/v with the safe startup input.
        self.sim.physical.apply_forces(active=True, advance=False,
            control=RideControl(motor_torque_nm=0., human_torque_nm=0.))
        mujoco.mj_forward(self.sim.model, self.sim.data)
        initial = raw_observation(self.sim)
        self.pipeline.reset(initial)
        self._sensor_time = initial.source_time_s
        self.observation = self.pipeline.read(self.sim.time_s)
        self._queue = deque()
        # Safe startup while a first controller command is in transport.
        self._applied = RideControl(motor_torque_nm=0., human_torque_nm=0.)
        self.commands_requested = []
        self.commands_applied = [dict(time_s=0., step=0, control=asdict(self._applied))]
        self.observations = [self.observation]
        self.trace = []
        self.terminated = self.truncated = False
        self.reason = self.error = None
        self.last_truth = None
        self.numerically_valid = True
        self.max_energy_residual_ratio = 0.
        self.metadata = configuration_metadata(self.sim, seed=self.seed)

    @property
    def done(self):
        return self.terminated or self.truncated

    def reset(self, *, seed=None):
        if seed is not None:
            if type(seed) is not int or seed < 0:
                raise ValueError('reset seed must be a nonnegative integer')
            self.seed = seed
        self.sim.reset()
        self._begin_episode()
        return self.observation

    def step(self, control, *, front_brake_demand=0., rear_brake_demand=0.):
        if self.done:
            raise RuntimeError('episode has ended; reset before stepping again')
        if not isinstance(control, RideControl):
            raise ValueError('expected an immutable RideControl')
        control.validate_for(self.sim.physics_config, self.sim.rider.variant)
        for value in (front_brake_demand, rear_brake_demand):
            if scalar(value, 'brake demand', minimum=0.) > 1.:
                raise ValueError('brake demand must lie in [0, 1]')
        start_step = self.sim.steps
        self._queue.append((start_step+self.delay_steps, control))
        self.commands_requested.append(dict(time_s=self.sim.time_s, step=start_step, control=asdict(control),
            front_brake_demand=front_brake_demand, rear_brake_demand=rear_brake_demand))
        count = min(self.control_steps, self.max_steps-self.sim.steps)
        try:
            for _ in range(count):
                while self._queue and self._queue[0][0] <= self.sim.steps:
                    _, self._applied = self._queue.popleft()
                    self.commands_applied.append(dict(time_s=self.sim.time_s, step=self.sim.steps,
                                                       control=asdict(self._applied)))
                # Brakes are an immediate out-of-band safety input, not queued.
                self.sim.step(front_brake_demand, rear_brake_demand, control=self._applied)
                sample = self.sim.physical.sample
                self.last_truth = truth_from_sample(self.sim, sample)
                self.tracker.update(self.last_truth, sample.dt_s)
                self.recorder.record(self.sim)
                quality = energy_quality(self.sim.physical.energy,
                    maximum_ratio=self.config.maximum_energy_residual_ratio)
                self.max_energy_residual_ratio = max(self.max_energy_residual_ratio, quality.residual_ratio)
                if not quality.acceptable:
                    self.numerically_valid = False
                    self.truncated = True
                    self.reason = 'numerical_quality'
                    break
                if self.config.stop_on_model_violation and not self.sim.physical.model_status.as_dict()['model_valid']:
                    self.truncated = True
                    self.reason = 'model_violation'
                    break
                if self.sim.crash is not None:
                    self.terminated = True
                    self.reason = 'crash:'+self.sim.crash.cause
                    break
                if self.sim.position_m >= self.sim.track.length_m:
                    self.terminated = True
                    self.reason = 'finish'
                    break
        except Exception as exc:
            self.terminated = True
            self.reason = 'simulation_error'
            self.error = f'{type(exc).__name__}: {exc}'
            raise  # Never manufacture a successful transition from invalid dynamics.
        if not self.done and self.sim.steps >= self.max_steps:
            self.truncated, self.reason = True, 'duration'
        raw = raw_observation(self.sim, self.sim.physical.sample)
        if raw.source_time_s > self._sensor_time:
            self.pipeline.push(raw)
            self._sensor_time = raw.source_time_s
        self.observation = self.pipeline.read(self.sim.time_s)
        self.observations.append(self.observation)
        result = ResearchStep(self.observation, self.last_truth, self.tracker.state,
                              self.terminated, self.truncated, self.reason, self.sim.steps-start_step, self.numerically_valid)
        self.trace.append(result)
        return result

    def save(self, directory, *, overwrite=False):
        path = Path(directory)
        if path.exists() and (not path.is_dir() or any(path.iterdir())) and not overwrite:
            raise FileExistsError(f'refusing to overwrite nonempty output: {path}')
        path.mkdir(parents=True, exist_ok=True)
        current_metadata = configuration_metadata(self.sim, seed=self.seed)
        research = dict(config=asdict(self.config), sensor_config=asdict(self.sensor_config),
            model_status=self.sim.physical.model_status.as_dict(),
            numerically_valid=self.numerically_valid, max_energy_residual_ratio=self.max_energy_residual_ratio,
            actual_sensor_seed=self.seed, control_steps=self.control_steps, actuator_delay_steps=self.delay_steps,
            metrics=self.tracker.metrics, terminated=self.terminated, truncated=self.truncated, error=self.error,
            source_changed_during_run=current_metadata['model_source_sha256'] != self.metadata['model_source_sha256'],
            policy_inputs='proper acceleration, pitch gyro, wheel/crank encoders, motor/human torque sensors',
            privileged_outputs='contact loads/clearances, road-relative pitch, CoM and simulator speed',
            timestep_alignment='truth/sensor source is the last incoming physical state, not the endpoint',
            parameters_validated_against_measurements=False)
        commands_json = json.dumps(plain(self.commands_requested), sort_keys=True, allow_nan=False)
        research['commands_sha256'] = hashlib.sha256(commands_json.encode()).hexdigest()
        summary = physical_summary(self.sim, self.metadata, self.reason or 'not_finished')
        summary['research'] = research
        (path/'summary.json').write_text(json.dumps(plain(summary), indent=2, sort_keys=True, allow_nan=False)+'\n')
        for name, values in (('commands_requested.jsonl', self.commands_requested),
                             ('commands_applied.jsonl', self.commands_applied),
                             ('observations.jsonl', [asdict(o) for o in self.observations])):
            with (path/name).open('w', encoding='utf-8') as stream:
                for value in values:
                    stream.write(json.dumps(plain(value), sort_keys=True, allow_nan=False)+'\n')
        rows = []
        for result in self.trace:
            obs = asdict(result.observation)
            acceleration = obs.pop('specific_force_body_mps2')
            row = {'sensor_'+k: v for k, v in obs.items()}
            row.update({f'sensor_specific_force_{axis}_mps2': acceleration[i] for i, axis in enumerate('xyz')})
            row.update({'truth_'+k: v for k, v in asdict(result.truth).items()})
            row.update(contact_state=result.contact_state, terminated=result.terminated,
                       truncated=result.truncated, reason=result.reason, numerically_valid=result.numerically_valid)
            rows.append(row)
        with (path/'trace.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['sensor_time_s'])
            writer.writeheader(); writer.writerows(rows)
        self.recorder.write_csv(path/'telemetry.csv')
        self.recorder.write_jsonl(path/'intervals.jsonl')
        np.save(path/'terrain_vertices.npy', self.sim.physical.vertices, allow_pickle=False)
        save_track(self.sim.track, path/'track.toml')
        return path
