"""One explicit-export path for Python and native research recordings.

The native hot loop stores owned C++ intervals and columns. Conversion to
Python rows occurs only here, after a completed external control boundary.
"""
from dataclasses import asdict
from pathlib import Path
import csv
import hashlib
import json
import tempfile
import numpy as np
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.physical_session import physical_summary
from bike_sim.sim.research.metrics import episode_metrics
from bike_sim.terrain.trackfile import save_track

def _write_episode(self, directory, *, overwrite=False):
    path = Path(directory)
    if path.exists() and (not path.is_dir() or any(path.iterdir())) and not overwrite:
        raise FileExistsError(f'refusing to overwrite nonempty output: {path}')
    path.mkdir(parents=True, exist_ok=True)
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
        row.update({'truth_'+k: v for k, v in (asdict(result.truth) if result.truth is not None else {}).items()})
        row.update(contact_state=result.contact_state, demand_nm=result.demand_nm, terminated=result.terminated,
                   truncated=result.truncated, reason=result.reason, numerically_valid=result.numerically_valid)
        rows.append(row)
    with (path/'trace.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['sensor_time_s'])
        writer.writeheader(); writer.writerows(rows)
    (path/'episode_metrics.json').write_text(
        json.dumps(plain(episode_metrics(self)), indent=2, sort_keys=True, allow_nan=False)+'\n')
    self.recorder.write_csv(path/'telemetry.csv')
    self.recorder.write_jsonl(path/'intervals.jsonl')
    np.save(path/'terrain_vertices.npy', self.sim.physical.vertices, allow_pickle=False)
    save_track(self.sim.track, path/'track.toml')
    from bike_sim.sim.research.replay import save_replay_files
    save_replay_files(self, path)
    return path


def export_recording(env, directory, *, overwrite=False):
    """Stage complete output and publish replay.json after all other files.

    A failed pre-publication export cannot corrupt an older recording. If a
    filesystem error interrupts publication, no stale manifest remains to
    describe a partially replaced file set. Unrelated destination files are
    never removed.
    """
    if env.control_pending:
        raise RuntimeError('finish the external control interval before saving at its boundary')
    destination = Path(directory)
    if destination.exists() and not destination.is_dir():
        raise FileExistsError(f'output is not a directory: {destination}')
    if destination.exists() and any(destination.iterdir()) and not overwrite:
        raise FileExistsError(f'refusing to overwrite nonempty output: {destination}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.research-export-', dir=destination.parent) as temporary:
        staging = Path(temporary)
        _write_episode(env, staging)
        # Recheck the overwrite guard after staging, before touching output.
        if destination.exists() and any(destination.iterdir()) and not overwrite:
            raise FileExistsError(f'refusing to overwrite nonempty output: {destination}')
        destination.mkdir(parents=True, exist_ok=True)
        (destination/'replay.json').unlink(missing_ok=True)
        for source in sorted(staging.iterdir()):
            if source.name != 'replay.json':
                source.replace(destination/source.name)
        (staging/'replay.json').replace(destination/'replay.json')
    return destination
