"""Catalog-governed typed encoding of a sealed native episode (A2).

``NativePhysicalSource`` adapts an ``OwnedNativeEpisode`` — or any
duck-typed value exposing ``generation``, ``row_count`` and
``batch(start, limit)`` — to the ``PhysicalSource`` contract. Every table
in the bound catalog is emitted in catalog order through the reviewed
``encode_rows`` encoder; nothing is flattened or renamed by hand, unknown
channel roots are rejected before any batch is emitted, and absent
optional roots encode explicit NULLs rather than padding.
"""
from __future__ import annotations

from bike_sim.recording.catalog import Catalog, JointCoords
from bike_sim.recording.encoding import UnknownFieldError, encode_rows, force_rows

__all__ = ['NativePhysicalSource', 'model_joints']

# The eager channel-root sweep uses its own bounded window; the consumer's
# ``batch_rows`` governs only the emitted TableBatch sizes.
_SCAN_ROWS = 1024

_JOINT_TYPE_NAMES = {0: 'free', 1: 'ball', 2: 'slide', 3: 'hinge'}


def model_joints(model):
    """``JointCoords`` values in model order, for ``build_catalog``.

    ``model`` is a compiled ``mujoco.MjModel`` — the driver's
    ``make_render_model()`` value. mujoco stays a lazy import so the
    recording modules remain importable without the engine.
    """
    import mujoco
    joints = []
    for index in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        if name is None:
            raise ValueError(f'model joint {index} is unnamed')
        joint_type = _JOINT_TYPE_NAMES.get(int(model.jnt_type[index]))
        if joint_type is None:
            raise ValueError(f'joint {name}: unsupported joint type')
        joints.append(JointCoords(
            name=name, joint_type=joint_type,
            qposadr=int(model.jnt_qposadr[index]),
            dofadr=int(model.jnt_dofadr[index])))
    return tuple(joints)


class NativePhysicalSource:
    """``PhysicalSource`` over a sealed native episode.

    The consumed episode surface is deliberately small — ``generation``,
    ``row_count`` and ``batch(start, limit)`` returning batches whose
    ``samples`` expose ``interval_id``, ``time_s``, ``end_time_s``,
    ``qpos``, ``qvel``, ``forces``, ``powers_w`` and ``channels`` — so the
    binding-owned value and hand-built test doubles share one encoding
    path.
    """

    def __init__(self, episode, catalog):
        if not isinstance(catalog, Catalog):
            raise TypeError('catalog must be a Catalog')
        for member in ('generation', 'row_count', 'batch'):
            if not hasattr(episode, member):
                raise TypeError(
                    f'episode must expose {member} (an OwnedNativeEpisode '
                    'or duck-typed equivalent)')
        self._episode = episode
        self._catalog = catalog

    @property
    def row_count(self):
        return int(self._episode.row_count)

    @staticmethod
    def _canonical(sample):
        """One owned sample -> the canonical row tree encoding consumes."""
        end_time_s = float(sample.end_time_s)
        time_s = float(sample.time_s)
        channels = dict(sample.channels)
        # The model-status numeric verdict is a bool|str tri-state at
        # runtime; the reviewed catalog publishes it as lowercase verdict
        # text ('true' | 'false' | 'not_evaluated'). The mapping is
        # injective over the runtime domain, so nothing is lost.
        status = channels.get('model_status')
        if isinstance(status, dict):
            verdict = status.get('numerically_valid')
            if isinstance(verdict, bool):
                status = dict(status)
                status['numerically_valid'] = 'true' if verdict else 'false'
                channels['model_status'] = status
        return {
            'interval_id': int(sample.interval_id),
            'time_s': time_s,
            'end_time_s': end_time_s,
            'dt_s': end_time_s - time_s,
            'qpos': sample.qpos,
            'qvel': sample.qvel,
            'forces': dict(sample.forces),
            'powers_w': dict(sample.powers_w),
            'channels': channels,
        }

    def _check_channel_roots(self):
        claimed = self._catalog.channel_roots()
        for start in range(0, self.row_count, _SCAN_ROWS):
            batch = self._episode.batch(start, _SCAN_ROWS)
            for sample in batch.samples:
                for root in sample.channels:
                    if root not in claimed:
                        raise UnknownFieldError(
                            f'channels: unrecognized domain {root!r}')

    @staticmethod
    def _sample_rows(spec, canonical):
        if spec.kind == 'physics':
            yield {key: canonical[key] for key in
                   ('interval_id', 'time_s', 'end_time_s', 'dt_s',
                    'qpos', 'qvel')}
        elif spec.kind == 'forces':
            yield from force_rows(canonical)
        else:
            yield {'interval_id': canonical['interval_id'],
                   **{root: canonical['channels'][root]
                      for root in spec.claims
                      if root in canonical['channels']}}

    def tables(self, *, batch_rows):
        if type(batch_rows) is not int or batch_rows < 1:
            raise ValueError('batch_rows must be a positive integer')
        self._check_channel_roots()
        episode_id = int(self._episode.generation)
        count = self.row_count
        for spec in self._catalog.table_order():
            pending = []
            emitted = False
            for start in range(0, count, batch_rows):
                batch = self._episode.batch(start, batch_rows)
                for sample in batch.samples:
                    pending.extend(
                        self._sample_rows(spec, self._canonical(sample)))
                while len(pending) >= batch_rows:
                    yield encode_rows(spec, pending[:batch_rows],
                                      episode_id=episode_id)
                    emitted = True
                    pending = pending[batch_rows:]
            if pending:
                yield encode_rows(spec, pending, episode_id=episode_id)
            elif not emitted:
                # An empty table still emits one zero-row batch so the
                # publisher sees every catalog table, like
                # TreePhysicalSource.
                yield encode_rows(spec, [], episode_id=episode_id)
