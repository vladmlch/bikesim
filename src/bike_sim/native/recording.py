"""Explicit schema-1/2 compatibility projection from one owned snapshot.

Normal CLI recording never enters this module. Snapshotting precedes all I/O;
reset/close or a failed publication cannot change the bytes being exported.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import csv
import hashlib
import json
import tempfile

import numpy as np

from bike_sim.recording.contracts import freeze
from bike_sim.recording.json_codec import loads, thaw
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.physical_session import physical_summary
from bike_sim.sim.research.metrics import episode_metrics


@dataclass(frozen=True)
class LegacySnapshot:
    summary: object
    requested: tuple
    applied: tuple
    observations: tuple
    transitions: tuple
    metrics: object
    columns: object
    intervals: tuple
    assets: object
    initial_state: object
    final_state: object
    recipe: object

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, freeze(getattr(self, name)))


def _summary(self, requested):
    current_metadata = self.current_metadata()
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
    from bike_sim.native.artifact import execution_provenance
    current_execution = execution_provenance(self.backend)
    research["execution_changed_during_run"] = current_execution != self.execution
    research["execution"] = self.execution
    research['valid_for_learning'] = self.numerically_valid and self.model_valid and self.error is None
    research['run_metadata'] = plain(self.run_metadata)
    if self.demand is not None:
        research['demand_program'] = self.demand.to_dict()
    if self.rider_program is not None:
        research['rider_program'] = self.rider_program.to_dict()
        research['rider_command_recording'] = 'program_inputs'
    commands_json = json.dumps(plain(requested), sort_keys=True, allow_nan=False)
    research['commands_sha256'] = hashlib.sha256(commands_json.encode()).hexdigest()
    summary = physical_summary(self.sim, self.metadata, self.reason or 'not_finished')
    summary['research'] = research
    return summary


def _columns(rows):
    """Legacy numeric projection only: its historical NaN padding is explicit."""
    result, count = {}, 0
    for row in rows:
        for key in row.keys() - result.keys():
            result[key] = [float('nan')] * count
        for key, values in result.items():
            values.append(float(row.get(key, float('nan'))))
        count += 1
    return result


def capture_legacy(env):
    """Consume each growing native stream exactly once, then detach the owner."""
    if env.control_pending:
        raise RuntimeError('finish the external control interval before saving at its boundary')
    if env.capture_mode not in ('legacy', 'full'):
        raise RuntimeError('legacy export requires legacy or full capture; disabled history cannot be recovered')
    if env.backend == 'native':
        owner = env._native.legacy_snapshot()
        def stream(name):
            return tuple(value for start in range(0, int(owner.row_count(name)), 4096)
                         for value in owner.batch(name, start, 4096))
        requested, applied = stream('commands_requested'), stream('commands_applied')
        observations, transitions = stream('observations'), stream('transitions')
        physical = owner.physical
        intervals, flat = [], []
        # One owned sample batch supplies both old file views. No separate
        # native recorded_columns()/recorded_intervals() history conversion.
        for start in range(0, int(physical.row_count), 4096):
            batch = physical.batch(start, 4096)
            columns = batch.columns
            for index, sample in enumerate(batch.samples):
                if sample.interval_id % env.config.record_decimation:
                    continue
                intervals.append(sample.as_dict())
                flat.append({name: values[index] for name, values in columns.items()})
        columns = _columns(flat)
    else:
        requested, applied = tuple(env.commands_requested), tuple(env.commands_applied)
        observations = tuple(asdict(value) for value in env.observations)
        transitions = tuple(asdict(value) for value in env.trace)
        if env.capture_mode == 'full':
            env._retain_published_samples()
            samples = tuple(value for value in env._capture_samples
                            if value.interval_id % env.config.record_decimation == 0)
            columns = _columns(env.recorder._row(sample) for sample in samples)
            intervals = tuple(sample.as_dict() for sample in samples)
        else:
            columns, intervals = env.recorder.columns(), tuple(env.recorder.samples)
    from bike_sim.sim.research.replay import integration_state
    assets = {value.name: value.content for value in env._recording_assets}
    recipe = loads(assets['recipe.json'].decode('utf-8'))
    return LegacySnapshot(_summary(env, requested), tuple(requested), tuple(applied),
                          tuple(observations), tuple(transitions), episode_metrics(env),
                          columns, tuple(intervals), assets, env.initial_integration_state,
                          integration_state(env.sim), recipe)


def _write_json(path, value):
    path.write_text(json.dumps(thaw(value), indent=2, sort_keys=True, allow_nan=False) + '\n',
                    encoding='utf-8')


def _write_jsonl(path, values):
    with path.open('w', encoding='utf-8') as stream:
        for value in values:
            stream.write(json.dumps(thaw(value), sort_keys=True, allow_nan=False) + '\n')


def write_replay_files(snapshot, path):
    """The old schema-2 fixed file set and checksum rules remain unchanged."""
    from bike_sim.native.artifact import validate_execution
    from bike_sim.sim.research.replay import SCHEMA2_FILES, _sha256
    path = Path(path)
    recipe = thaw(snapshot.recipe)
    manifest = dict(schema_version=2, start_x_m=recipe['start_x_m'],
                    heightfield=recipe['heightfield'], state_signature='mjSTATE_INTEGRATION',
                    execution=validate_execution(recipe['execution']))
    np.savez_compressed(path / 'states.npz', initial=snapshot.initial_state, final=snapshot.final_state)
    _write_jsonl(path / 'transitions.jsonl', snapshot.transitions)
    hashes = {name: _sha256(path / name) for name in SCHEMA2_FILES}
    for key, name in (('joint_envelope_file', 'rider_joint_envelope.json'),
                      ('joint_strength_file', 'rider_joint_strength.json')):
        if key in recipe:
            contents = snapshot.assets[name]
            (path / name).write_bytes(contents)
            manifest[key] = name
            hashes[name] = hashlib.sha256(contents).hexdigest()
    manifest['file_sha256'] = hashes
    _write_json(path / 'replay.json', manifest)


def _write_episode(snapshot, directory, *, overwrite=False):
    path = Path(directory)
    if path.exists() and (not path.is_dir() or any(path.iterdir())) and not overwrite:
        raise FileExistsError(f'refusing to overwrite nonempty output: {path}')
    path.mkdir(parents=True, exist_ok=True)
    _write_json(path / 'summary.json', snapshot.summary)
    for name, values in (('commands_requested.jsonl', snapshot.requested),
                         ('commands_applied.jsonl', snapshot.applied),
                         ('observations.jsonl', snapshot.observations)):
        _write_jsonl(path / name, values)
    rows = []
    for result in snapshot.transitions:
        obs = dict(result['observation'])
        acceleration = obs.pop('specific_force_body_mps2')
        row = {'sensor_' + key: value for key, value in obs.items()}
        row.update({f'sensor_specific_force_{axis}_mps2': acceleration[index]
                    for index, axis in enumerate('xyz')})
        row.update({'truth_' + key: value for key, value in (result['truth'] or {}).items()})
        row.update({key: result[key] for key in ('contact_state', 'demand_nm', 'terminated',
                                               'truncated', 'reason', 'numerically_valid')})
        rows.append(row)
    with (path / 'trace.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['sensor_time_s'])
        writer.writeheader()
        writer.writerows(rows)
    _write_json(path / 'episode_metrics.json', snapshot.metrics)
    columns = snapshot.columns
    names = sorted(columns) if columns else ['schema_version', 'time_s']
    count = len(next(iter(columns.values()))) if columns else 0
    with (path / 'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index in range(count):
            writer.writerow(['' if np.isnan(columns[name][index])
                             else repr(float(columns[name][index])) for name in names])
    _write_jsonl(path / 'intervals.jsonl', snapshot.intervals)
    for name in ('terrain_vertices.npy', 'track.toml'):
        (path / name).write_bytes(snapshot.assets[name])
    write_replay_files(snapshot, path)
    return path


def export_recording(env, directory, *, overwrite=False):
    """Explicit, owned compatibility export, with replay.json published last."""
    destination = Path(directory)
    if destination.exists() and not destination.is_dir():
        raise FileExistsError(f'output is not a directory: {destination}')
    if destination.exists() and any(destination.iterdir()) and not overwrite:
        raise FileExistsError(f'refusing to overwrite nonempty output: {destination}')
    snapshot = capture_legacy(env)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.research-export-', dir=destination.parent) as temporary:
        staging = Path(temporary)
        _write_episode(snapshot, staging)
        if destination.exists() and any(destination.iterdir()) and not overwrite:
            raise FileExistsError(f'refusing to overwrite nonempty output: {destination}')
        destination.mkdir(parents=True, exist_ok=True)
        (destination / 'replay.json').unlink(missing_ok=True)
        for source in sorted(staging.iterdir()):
            if source.name != 'replay.json':
                source.replace(destination / source.name)
        (staging / 'replay.json').replace(destination / 'replay.json')
    return destination
