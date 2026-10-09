"""Owned native research backend; Python work is confined to external boundaries.

No fallback to Python physics exists here. A fresh Python environment supplies
static equilibrium/setup, then the C++ runtime owns an independent model, data,
clock, accounting, sensor tape and recording. The facade deliberately exposes
neither a mutable mjData nor a simulation ``step`` method.
"""
from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import asdict, replace
from pathlib import Path
import csv
import hashlib
import json
import tempfile
import numpy as np

from bike_sim.native.contracts import FrameSnapshot, PhysicalViewState, _freeze, _plain
from bike_sim.native.runtime import _sample
from bike_sim.native.setup import capture_bootstrap, _control_dict, validate_supported
from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
from bike_sim.sim.ride.wheelie import WheelieTruth
from bike_sim.sim.research.environment import ResearchEnvironment, ResearchStep
from bike_sim.sim.research.observations import raw_observation
from bike_sim.sim.research.rider_behavior import signals_from_sample
from bike_sim.sim.research.sensor_noise import build_noise_tape
from bike_sim.sim.research.sensors import SensorObservation, SensorPipeline

__all__ = ['NativeResearchEnvironment', 'create_native_research']


class _Record(Mapping):
    """An owned, read-only mapping with the historical diagnostic attributes."""
    __slots__ = ('_values',)

    def __init__(self, values):
        object.__setattr__(self, '_values', _freeze(values))

    def __setattr__(self, name, value):
        raise AttributeError('native diagnostic views are read-only')

    def __getitem__(self, name):
        value = self._values[name]
        return _Record(value) if isinstance(value, Mapping) else value

    def __iter__(self):
        return iter(self._values)

    def __len__(self):
        return len(self._values)

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as error:
            raise AttributeError(name) from error

    def as_dict(self):
        return _plain(self._values)


class _SimulationView:
    __slots__ = ('_owner', '_static', '_vertices')
    backend = 'native'

    def __init__(self, owner, sim):
        object.__setattr__(self, '_owner', owner)
        object.__setattr__(self, '_static', {
            name: deepcopy(getattr(sim, name)) for name in
            ('physics_config', 'rider', 'track', 'field', 'start_x_m', 'equilibrium')})
        object.__setattr__(self, '_vertices', _freeze(np.asarray(sim.physical.vertices)))

    def __setattr__(self, name, value):
        raise AttributeError('native simulation facade is read-only')

    def __getattr__(self, name):
        if name in self._static:
            # Returning a copy prevents mutable nested recipe data from
            # changing the runtime or the recipe used by a later export.
            return deepcopy(self._static[name])
        if name in ('time_s', 'position_m'):
            return self._owner._status[name]
        if name == 'steps':
            return self._owner._status['step']
        if name == 'crash':
            value = self._owner._status['crash']
            return None if value is None else _Record(value)
        raise AttributeError(name)

    @property
    def physical(self):
        state = self._owner._status
        accounting = state['accounting']
        return _Record(dict(energy=accounting['energy'], history=accounting['history'],
            model_status=accounting['model_status'], reference_monitor=accounting['monitor'],
            drive=dict(battery=dict(energy_j=state['battery_energy_j'])),
            balance_monitor=dict(event=state['balance_event']), vertices=self._vertices))

    def snapshot(self):
        return self._owner.snapshot()

    def integration_state(self):
        return self.snapshot().integration_state.copy()


class _NativeRecorder:
    def __init__(self, owner):
        self._owner = owner

    @property
    def rows(self):
        return self._owner._status['recorded_rows']

    @property
    def samples(self):
        return tuple(_freeze(value) for value in self._owner._native.recorded_intervals())

    def columns(self):
        return _freeze(self._owner._native.recorded_columns())

    def write_csv(self, path):
        columns = self.columns()
        names = sorted(columns) if columns else ['schema_version', 'time_s']
        with Path(path).open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(names)
            for index in range(self.rows):
                writer.writerow(['' if np.isnan(columns[name][index])
                                 else repr(float(columns[name][index])) for name in names])
        return Path(path)

    def write_jsonl(self, path):
        with Path(path).open('w', encoding='utf-8') as stream:
            for row in self._owner._native.recorded_intervals():
                stream.write(json.dumps(plain(row), sort_keys=True, allow_nan=False)+'\n')
        return Path(path)


def _observation(value):
    return SensorObservation(**value)


def _transition(value):
    data = dict(value)
    data['observation'] = _observation(data['observation'])
    data['truth'] = None if data['truth'] is None else WheelieTruth(**data['truth'])
    return ResearchStep(**data)


def _metadata_seed(metadata, seed):
    from bike_sim.sim.ride.physical_session import canonical_json
    result = deepcopy(metadata)
    result['seed'] = seed
    result['resolved_config']['seed'] = seed
    hashed = deepcopy(result['resolved_config'])
    articulated = hashed['physics']['articulated']
    articulated['joint_envelope_path'] = hashed.get('joint_envelope_sha256')
    articulated['joint_strength_path'] = hashed.get('joint_strength_sha256')
    result['configuration_sha256'] = hashlib.sha256(canonical_json(hashed).encode()).hexdigest()
    return result


class NativeResearchEnvironment:
    backend = 'native'
    _STATUS_FIELDS = frozenset({
        'terminated', 'truncated', 'reason', 'error', 'numerically_valid', 'model_valid',
        'demand_nm', 'max_energy_residual_ratio', 'torque_delivered_nms',
        'torque_requested_nms', 'demand_integral_nms', 'max_shock_stroke_m', 'max_fork_travel_m'})

    def __init__(self, runtime, reference, bootstrap, directory, extension, execution):
        self._native, self._bootstrap, self._directory = runtime, bootstrap, directory
        self._reference_error = extension.InvalidReferenceRun
        self.config, self.sensor_config = deepcopy(reference.config), deepcopy(reference.sensor_config)
        self.demand, self.rider_program = deepcopy(reference.demand), deepcopy(reference.rider_program)
        # Clone the already reset callback. Sharing its mutable state with the
        # reference environment would break independent paired experiments.
        self.rider_behavior = deepcopy(reference.rider_behavior)
        self.seed = reference.seed
        for name in ('dt_s', 'control_steps', 'delay_steps', 'max_steps', 'sensor_steps', 'start_position_m'):
            setattr(self, name, getattr(reference, name))
        self.metadata = deepcopy(reference.metadata)
        self.execution = deepcopy(execution)
        self.run_metadata = deepcopy(reference.run_metadata)
        self.initial_integration_state = _freeze(reference.initial_integration_state)
        self._raw_initial = raw_observation(reference.sim)
        self._active = False
        self._closed = False
        self._status = _freeze(runtime.status())
        self.sim = _SimulationView(self, reference.sim)
        self.recorder = _NativeRecorder(self)

    def __setattr__(self, name, value):
        if name in type(self)._STATUS_FIELDS:
            raise AttributeError("native episode outcomes are read-only")
        object.__setattr__(self, name, value)

    def __getattr__(self, name):
        if name in self._STATUS_FIELDS:
            return self._status[name]
        raise AttributeError(name)

    @property
    def observation(self):
        return _observation(self._status['observation'])

    @property
    def last_truth(self):
        value = self._status['truth']
        return None if value is None else WheelieTruth(**value)

    @property
    def done(self):
        return self.terminated or self.truncated

    @property
    def control_pending(self):
        return self._active

    @property
    def tracker(self):
        return _Record(dict(state=self._status['contact_state'], metrics=self._status['metrics']))

    @property
    def pipeline(self):
        return _Record({name: self._status[name] for name in
                        ('samples_attempted', 'samples_dropped', 'noise_cursor')})

    @property
    def reference_monitor(self):
        return self.sim.physical.reference_monitor

    @property
    def commands_requested(self):
        return self._native.commands_requested()

    @property
    def commands_applied(self):
        return self._native.commands_applied()

    @property
    def observations(self):
        return [_observation(value) for value in self._native.observations()]

    @property
    def trace(self):
        return [_transition(value) for value in self._native.transitions()]

    def snapshot(self):
        value = self._native.snapshot()
        return FrameSnapshot(generation=int(value.generation), step=int(value.step),
            time_s=float(value.time_s), integration_state=value.integration_state,
            latest_sample=_sample(value.latest_sample), view=PhysicalViewState(value.view),
            outcome=value.outcome, first_failure=value.first_failure, model_status=value.model_status)

    def begin_control(self, control, *, front_brake_demand=0., rear_brake_demand=0.):
        if self._closed or self.done or self.control_pending:
            raise RuntimeError('begin_control requires an open idle research episode')
        if not isinstance(control, RideControl):
            raise ValueError('expected an immutable RideControl')
        for value in (front_brake_demand, rear_brake_demand):
            if scalar(value, 'brake demand', minimum=0.) > 1.:
                raise ValueError('brake demand must lie in [0, 1]')
        if self.rider_program is not None:
            self.rider_program.validate_control(control)
        elif self.rider_behavior is not None and control.posture is None:
            posture = self.rider_behavior.act(self.sim.time_s,
                signals_from_sample(self.snapshot().latest_sample))
            if posture is not None:
                control = replace(control, posture=posture)
        control.validate_for(self.sim.physics_config, self.sim.rider.variant)
        self._native.begin_control(_control_dict(control), front_brake_demand, rear_brake_demand)
        self._active = True

    def advance_control(self, *, wall_budget_s=None):
        if self._closed or not self.control_pending:
            raise RuntimeError('begin an external control interval before advancing')
        try:
            value = self._native.advance_control(wall_budget_s)
        except Exception as error:
            # Publication errors retain the completed native transaction for
            # retry. Cleanup/diagnostic failures must not mask the primary one.
            converted = InvalidReferenceRun(str(error)) if isinstance(error, self._reference_error) else error
            try:
                status = self._native.status()
                if not status['active'] and not status['delivery_pending']:
                    self._active = False
                    if status['terminated'] or status['truncated']:
                        message = f'{type(converted).__name__}: {converted}'
                        self._native.set_error_text(message)
                        status['error'] = message
                for note in status['cleanup_notes']:
                    converted.add_note(note)
                self._status = _freeze(status)
            except Exception as cleanup:
                converted.add_note(f'native failure-state publication failed: {type(cleanup).__name__}: {cleanup}')
            if converted is not error:
                raise converted from error
            raise
        status = self._native.status()
        if value is None:
            self._status = _freeze(status)
            return None
        # Complete all Python allocations/validation before acknowledgement.
        # If any of these fail, retry returns exactly this completed window.
        result = _transition(value)
        status['active'] = status['delivery_pending'] = False
        if status['stop_requested'] and not (status['terminated'] or status['truncated']):
            status['truncated'], status['reason'] = True, 'operator_stop'
        status["stop_requested"] = False
        owned_status = _freeze(status)
        self._native.acknowledge_control()
        self._status = owned_status
        self._active = False
        return result

    def step(self, control, *, front_brake_demand=0., rear_brake_demand=0.):
        self.begin_control(control, front_brake_demand=front_brake_demand,
                           rear_brake_demand=rear_brake_demand)
        return self.advance_control()

    def reset(self, *, seed=None):
        if self._closed or self.control_pending:
            raise RuntimeError('reset requires an open environment at an external boundary')
        chosen = self.seed if seed is None else seed
        if type(chosen) is not int or chosen < 0:
            raise ValueError('reset seed must be a nonnegative integer')
        from bike_sim.native.artifact import execution_provenance
        current_execution = execution_provenance('native')
        if current_execution != self.execution or self.current_metadata() != self.metadata:
            raise ValueError('source, binary or recipe changed; construct a fresh environment before reset')
        tape = build_noise_tape(self.sensor_config, seed=chosen, count=1+self.max_steps//self.sensor_steps)
        pipeline = SensorPipeline(self.sensor_config, seed=chosen)
        pipeline.reset(self._raw_initial)
        pipeline.read(0.)
        behavior = deepcopy(self.rider_behavior)
        if behavior is not None:
            behavior.reset(chosen)
        metadata = _metadata_seed(self.metadata, chosen)
        try:
            self._native.reset(tape.noise, tape.dropout_uniform, pipeline.state_dict())
        except self._reference_error as error:
            raise InvalidReferenceRun(str(error)) from error
        self.seed, self.metadata, self.rider_behavior = chosen, metadata, behavior
        self.run_metadata = {}
        self._status = _freeze(self._native.status())
        return self.observation

    def current_metadata(self):
        from bike_sim.native.artifact import _file_digest
        from bike_sim.validation.environment import source_fingerprint
        current = deepcopy(self.metadata)
        current['model_source_sha256'] = source_fingerprint(Path(__file__).resolve().parents[1])
        resolved = current['resolved_config']
        for key in ('joint_envelope', 'joint_strength'):
            path = resolved['physics']['articulated'][key+'_path']
            if path is not None:
                resolved[key+'_sha256'] = _file_digest(path)
        return _metadata_seed(current, self.seed)

    def stop(self):
        self._native.stop()
        self._status = _freeze(self._native.status())

    def fail_policy(self, error):
        self._native.fail_policy(f'{type(error).__name__}: {error}')
        self._status = _freeze(self._native.status())

    def save(self, directory, *, overwrite=False):
        from bike_sim.native.recording import export_recording
        return export_recording(self, directory, overwrite=overwrite)

    def close(self):
        if self._closed:
            return
        if self.control_pending:
            raise RuntimeError('finish the active interval before closing the environment')
        self._native.close()
        self._closed = True
        if self._directory is not None:
            self._directory.cleanup()


def create_native_research(reference: ResearchEnvironment, *, directory=None):
    """Capture a safe research t=0; do not replay startup noise or holding brakes."""
    if not isinstance(reference, ResearchEnvironment):
        raise ValueError('expected a fresh Python ResearchEnvironment as the setup reference')
    if reference.sim.steps != 0 or reference.sim.time_s != 0. or reference.control_pending or reference.done:
        raise ValueError('native research setup requires the initial external boundary')
    validate_supported(reference.sim.physics_config, reference.sim.rider)
    from bike_sim.native.artifact import load_native_extension, execution_provenance
    extension = load_native_extension()
    execution = execution_provenance('native', artifact=extension)
    if extension.runtime_identity()['native_source_sha256'] != execution['native_source_sha256']:
        raise ValueError('selected extension was not built from the current native source')
    tape = build_noise_tape(reference.sensor_config, seed=reference.seed,
                           count=1+reference.max_steps//reference.sensor_steps)
    sim = reference.sim
    geometry = dict(vertices=sim.physical.vertices,
        front_radius=float(sim.model.geom_size[sim.contact_query.front_id, 0]),
        rear_radius=float(sim.model.geom_size[sim.contact_query.rear_id, 0]),
        root_x_qpos=sim.root_x_qposadr, root_x_dof=sim.root_x_dofadr,
        root_pitch_qpos=sim.root_pitch_qposadr, root_pitch_dof=sim.physical.address("root_pitch")[1])
    research = {name: getattr(reference, name) for name in
                ('control_steps', 'delay_steps', 'max_steps', 'sensor_steps', 'start_position_m')}
    research.update(timestep_s=reference.dt_s, track_length_m=sim.track.length_m,
        record_decimation=reference.config.record_decimation,
        stop_on_model_violation=reference.config.stop_on_model_violation,
        maximum_energy_residual_ratio=reference.config.maximum_energy_residual_ratio,
        wheelie_persistence_s=reference.config.wheelie_persistence_s,
        sensors=asdict(reference.sensor_config), geometry=geometry,
        startup_control=_control_dict(reference._applied),
        rider_program=None if reference.rider_program is None else reference.rider_program.to_dict(),
        demand_program=None if reference.demand is None else reference.demand.to_dict())
    owned_directory = tempfile.TemporaryDirectory(prefix='bike_research_native_') if directory is None else None
    destination = Path(owned_directory.name if owned_directory is not None else directory)
    runtime = None
    try:
        bootstrap = capture_bootstrap(sim, destination)
        runtime = extension.NativeResearchRuntime(str(bootstrap.model_path), bootstrap.config,
            bootstrap.state, research, tape.noise, tape.dropout_uniform, reference.pipeline.state_dict())
        return NativeResearchEnvironment(runtime, reference, bootstrap, owned_directory, extension, execution)
    except BaseException as error:
        for resource in (runtime, owned_directory):
            if resource is None:
                continue
            try:
                resource.close() if resource is runtime else resource.cleanup()
            except BaseException as cleanup:
                error.add_note(f"native setup cleanup failed: {type(cleanup).__name__}: {cleanup}")
        raise
