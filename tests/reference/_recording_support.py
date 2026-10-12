"""Shared synthetic fixtures for the run-recording contract tests.

``run_metadata`` carries the fixed identity of the recording contract tests;
``TreePhysicalSource`` converts hand-authored nested sample rows into typed
``TableBatch`` values strictly through the production field catalog and
encoder — never through the native adapter or a simulator.
"""
from datetime import datetime, timezone
from uuid import UUID

from bike_sim.recording.catalog import JointCoords, build_catalog
from bike_sim.recording.contracts import EpisodeEvidence, RunMetadata
from bike_sim.recording.encoding import encode_rows, force_rows


def run_metadata():
    """Deterministic synthetic run identity for recording tests."""
    return RunMetadata(
        run_id=UUID('00000000-0000-0000-0000-000000000001'),
        started_at=datetime(2026, 10, 12, 0, 0, 0, tzinfo=timezone.utc),
        backend='native',
        mode='ride',
        values={'provenance': {
            'kind': 'synthetic',
            'author': 'tests/reference/_recording_support.py',
            'purpose': 'deterministic capture and catalog regression tests',
            'physical_source': 'none (hand-authored trees)',
        }},
    )


# A small planar joint set shaped like the supported physical model's scalar
# joints: joint name -> (type, qposadr, dofadr). nq == nv == 8.
_SYNTHETIC_JOINTS = (
    ('root_x', 'slide'),
    ('root_z', 'slide'),
    ('root_pitch', 'hinge'),
    ('front_wheel_spin', 'hinge'),
    ('rear_wheel_spin', 'hinge'),
    ('crank_spin', 'hinge'),
    ('fork_slide', 'slide'),
    ('shock_slide', 'slide'),
)


def synthetic_joints():
    return tuple(
        JointCoords(name=name, joint_type=joint_type, qposadr=i, dofadr=i)
        for i, (name, joint_type) in enumerate(_SYNTHETIC_JOINTS))


def support_catalog():
    """The production catalog specialized on the synthetic joint map."""
    return build_catalog(synthetic_joints())


def sample_row(interval_id=0, *, start=0.0, dt=0.001, qpos=None, qvel=None,
               forces=None, powers=None, channels=None):
    """One hand-authored canonical sample tree (see catalog.TableSpec kinds)."""
    n = len(_SYNTHETIC_JOINTS)
    return {
        'interval_id': interval_id,
        'time_s': start,
        'end_time_s': start + dt,
        'dt_s': dt,
        'qpos': [0.0] * n if qpos is None else list(qpos),
        'qvel': [0.0] * n if qvel is None else list(qvel),
        'forces': {} if forces is None else dict(forces),
        'powers_w': {} if powers is None else dict(powers),
        'channels': {} if channels is None else dict(channels),
    }


def energy_channels(**overrides):
    """A complete authored ``channels`` row centred on the energy domain."""
    channels = {
        'energy': {
            'mechanical_energy_j': 12.5,
            'elastic_energy_j': {'fork_spring': 0.25},
            'active_work_j': 1.5,
            'external_work_j': -0.5,
            'loss_j': 0.25,
            'loss_step_j': 0.001,
            'solver_constraint_work_j': 0.0,
            'energy_scale_j': 10.0,
            'muscle_signed_j': 1.0,
            'muscle_positive_j': 1.25,
            'motor_signed_j': 0.5,
            'motor_positive_j': 0.5,
            'source_positive_work_j': 1.75,
            'constraint_signed_j': 0.0,
            'constraint_absolute_j': 0.0,
            'constraint_work_ratio': 0.0,
            'constraint_work_ok': True,
            'residual_j': -0.001,
            'electrical_work_j': 0.5,
            'electrical_residual_j': 0.0,
            'measured': True,
            'source_interval_id': 0,
            'source_time_s': 0.0,
        },
        'component_work_j': {'chain': 0.1, 'tires': -0.2},
    }
    for key, value in overrides.items():
        if key == 'energy':
            channels['energy'].update(value)
        else:
            channels[key] = value
    return channels


class TreePhysicalSource:
    """PhysicalSource over authored sample trees, encoded via the catalog.

    ``rows`` are canonical sample trees as produced by ``sample_row``. Every
    table in the bound catalog is emitted: ``physics`` and ``channel_*`` rows
    come straight from each sample, ``forces`` explodes the per-component
    generalized-force vectors into long rows. Conversion delegates to the
    production ``encoding.encode_rows``; nothing is flattened by hand here.
    """

    def __init__(self, rows=(), *, catalog=None):
        self._catalog = support_catalog() if catalog is None else catalog
        self._rows = tuple(rows)

    @property
    def catalog(self):
        return self._catalog

    @property
    def row_count(self):
        return len(self._rows)

    def _table_rows(self, spec):
        if spec.kind == 'physics':
            return [dict(row) for row in self._rows]
        if spec.kind == 'forces':
            return [row for sample in self._rows
                    for row in force_rows(sample)]
        if spec.kind == 'channels':
            return [{'interval_id': sample['interval_id'],
                     **{root: sample['channels'][root]
                        for root in spec.claims
                        if root in sample['channels']}}
                    for sample in self._rows]
        raise ValueError(f'unknown table kind {spec.kind!r}')

    def _check_channel_roots(self):
        claimed = self._catalog.channel_roots()
        for sample in self._rows:
            for root in sample['channels']:
                if root not in claimed:
                    from bike_sim.recording.encoding import UnknownFieldError
                    raise UnknownFieldError(
                        f'channels: unrecognized domain {root!r}')

    def tables(self, *, batch_rows):
        if type(batch_rows) is not int or batch_rows < 1:
            raise ValueError('batch_rows must be a positive integer')
        self._check_channel_roots()
        for spec in self._catalog.table_order():
            rows = self._table_rows(spec)
            for offset in range(0, max(len(rows), 1), batch_rows):
                yield encode_rows(spec, rows[offset:offset + batch_rows])


def episode(source, episode_id=0, *, generation=0, summary=None, assets=()):
    return EpisodeEvidence(
        episode_id=episode_id, generation=generation, physical=source,
        summary={} if summary is None else summary, assets=tuple(assets))
