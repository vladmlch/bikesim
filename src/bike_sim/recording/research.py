"""Owned research evidence and catalog projections, independent of live engines.

The native source retains an OwnedResearchEpisode, not a facade. The Python
source freezes its payloads on retention. Both export in bounded batches and
leave completed control transitions distinct from an incomplete physical tail.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from types import SimpleNamespace

import numpy as np

from bike_sim.recording.contracts import EpisodeEvidence, ResearchEvidence, freeze, _integer
from bike_sim.recording.encoding import encode_rows
from bike_sim.recording.json_codec import thaw
from bike_sim.recording.native import NativePhysicalSource
from bike_sim.recording.schema import research_tables

CAPTURE_MODES = frozenset(('off', 'full', 'legacy', 'verify'))


def capture_mode(value):
    if not isinstance(value, str) or value not in CAPTURE_MODES:
        raise ValueError('capture_mode must be off, full, legacy or verify')
    return value


def _data(value):
    return asdict(value) if is_dataclass(value) else value


@dataclass(frozen=True)
class OwnedSample:
    interval_id: int
    time_s: float
    end_time_s: float
    qpos: object
    qvel: object
    forces: object
    powers_w: object
    channels: object

    @classmethod
    def capture(cls, sample):
        return cls(sample.interval_id, sample.time_s, sample.end_time_s,
                   freeze(sample.qpos), freeze(sample.qvel), freeze(sample.forces),
                   freeze(sample.powers_w), freeze(sample.channels))

    @property
    def dt_s(self):
        return self.end_time_s - self.time_s

    def as_dict(self):
        return dict(schema_version=2, interval_id=self.interval_id,
                    time_s=self.time_s, interval_end_s=self.end_time_s, dt_s=self.dt_s,
                    qpos=thaw(self.qpos), qvel=thaw(self.qvel), powers_w=thaw(self.powers_w),
                    **thaw(self.channels))


@dataclass(frozen=True)
class OwnedPythonEpisode:
    generation: int
    samples: tuple

    @property
    def row_count(self):
        return len(self.samples)

    def batch(self, start, limit):
        start = _integer(start, 'start')
        limit = _integer(limit, 'limit', minimum=1)
        if start > self.row_count:
            raise ValueError('batch start exceeds owned physical prefix')
        return SimpleNamespace(samples=self.samples[start:start + limit])


class ResearchHistory:
    """Exactly one native owner or one immutable Python control journal."""
    __slots__ = ('_owner', '_streams', '_counts')

    def __init__(self, *, owner=None, requested=(), applied=(), observations=(), transitions=()):
        self._owner = owner
        self._streams = None if owner is not None else freeze({
            'commands_requested': tuple(_data(row) for row in requested),
            'commands_applied': tuple(_data(row) for row in applied),
            'observations': tuple(_data(row) for row in observations),
            'transitions': tuple(_data(row) for row in transitions)})
        self._counts = {spec.name: (int(owner.row_count(spec.name)) if owner is not None
                                   else len(self._streams[spec.name])) for spec in research_tables()}

    def row_count(self, name):
        return self._counts[name]

    def rows(self, name, start, limit):
        start = _integer(start, 'start')
        limit = _integer(limit, 'limit', minimum=1)
        if start > self._counts[name]:
            raise ValueError('research batch start exceeds owned prefix')
        if self._owner is not None:
            return self._owner.batch(name, start, limit)
        return self._streams[name][start:start + limit]

    def tables(self, *, batch_rows):
        batch_rows = _integer(batch_rows, 'batch_rows', minimum=1)
        for spec in research_tables():
            identity = spec.keys[1].key
            for start in range(0, self.row_count(spec.name), batch_rows):
                values = []
                for index, value in enumerate(self.rows(spec.name, start, batch_rows), start):
                    row = dict(value)
                    if spec.name == 'transitions':
                        # Observation is already stored once in its own stream.
                        row.pop('observation')
                        row.update(request_id=index, previous_observation_id=index,
                                   observation_id=index + 1)
                    row[identity] = index
                    values.append(row)
                yield encode_rows(spec, values, episode_id=0)


def research_summary(env, *, reason):
    """Snapshot outcomes and identity, with no read of full history properties."""
    from bike_sim.native.artifact import execution_provenance
    from bike_sim.sim.ride.physical_session import physical_summary
    from bike_sim.sim.research.metrics import episode_metrics
    from bike_sim.sim.ride.physical_samples import plain
    summary = physical_summary(env.sim, env.metadata, reason)
    research = dict(config=asdict(env.config), sensor_config=asdict(env.sensor_config),
        model_status=env.sim.physical.model_status.as_dict(),
        numerically_valid=env.numerically_valid, max_energy_residual_ratio=env.max_energy_residual_ratio,
        actual_sensor_seed=env.seed, control_steps=env.control_steps, actuator_delay_steps=env.delay_steps,
        metrics=plain(env.tracker.metrics), terminated=env.terminated, truncated=env.truncated, error=env.error,
        execution=plain(env.execution), run_metadata=plain(env.run_metadata),
        policy_inputs='proper acceleration, pitch gyro, wheel/crank encoders, motor/human torque sensors',
        privileged_outputs='contact loads/clearances, road-relative pitch, CoM and simulator speed',
        timestep_alignment='truth/sensor source is the last incoming physical state, not the endpoint',
        parameters_validated_against_measurements=False,
        capture_mode=env.capture_mode, measurement_decimation=env.config.record_decimation,
        physical_downsampling=False)
    try:
        current = env.current_metadata()
        research.update(
            source_changed_during_run=current['model_source_sha256'] != env.metadata['model_source_sha256'],
            configuration_changed_during_run=current['configuration_sha256'] != env.metadata['configuration_sha256'],
            execution_changed_during_run=execution_provenance(env.backend) != env.execution)
    except Exception as error:
        research.update(source_changed_during_run=None, configuration_changed_during_run=None,
                        execution_changed_during_run=None,
                        identity_check_error=f'{type(error).__name__}: {error}')
    research['valid_for_learning'] = (env.numerically_valid and env.model_valid and env.error is None
                                      and not env.run_metadata.get('launch_failure'))
    if env.demand is not None:
        research['demand_program'] = env.demand.to_dict()
    if env.rider_program is not None:
        research['rider_program'] = env.rider_program.to_dict()
        research['rider_command_recording'] = 'program_inputs'
    summary.update(research=research, episode_metrics=episode_metrics(env))
    return summary


def seal_research(env, episode_id, *, reason, physical, history):
    """Construct the evidence before the facade commits its one-time seal."""
    from bike_sim.sim.research.replay import integration_state
    _integer(episode_id, 'episode_id')
    if not isinstance(reason, str) or not reason:
        raise ValueError('episode reason must be a nonempty string')
    summary = research_summary(env, reason=env.reason or 'not_finished')
    summary['seal_reason'] = reason
    initial = np.asarray(env.initial_integration_state, dtype='float64')
    final = np.asarray(integration_state(env.sim), dtype='float64')
    research = summary['research']
    complete = history.row_count('transitions')
    eligible = (not env.control_pending and history.row_count('commands_requested') == complete
                and physical.row_count == env.sim.steps
                and env.error is None and env.numerically_valid and env.model_valid
                and np.isfinite(initial).all() and np.isfinite(final).all()
                and all(research.get(key) is False for key in ('source_changed_during_run',
                    'configuration_changed_during_run', 'execution_changed_during_run'))
                and not env.run_metadata.get('policy_source_changed_during_run', False)
                and not env.run_metadata.get('launch_failure')
                and reason not in ('simulation_error', 'policy_error', 'invalid_controller'))
    summary['recording_extents'] = dict(physical_rows=physical.row_count,
        requested_controls=history.row_count('commands_requested'), complete_control_steps=complete,
        observations=history.row_count('observations'), replay_eligible=bool(eligible))
    return EpisodeEvidence(episode_id=episode_id, generation=int(physical.episode.generation),
        physical=physical, summary=summary, assets=env._recording_assets,
        research=ResearchEvidence(history, initial, final, 3, complete, bool(eligible)))
