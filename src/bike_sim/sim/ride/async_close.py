"""Off-path period publication: a separate process evaluates closes.

PhysicalRuntime accumulates RawStep payloads for one control period, then
publishes them through publish_close — pure data work (attachment wrenches,
effort channels, ledger bookkeeping, sample assembly) that never touches live
mjData. That makes it movable: this module runs the same publish_close inside
a platform-selected worker, so the stepping loop only serializes raws and applies
results at the next step boundary.

Semantics: identical code on identical data, applied one step later. Runs
where nothing consumes mid-period published state for control decisions are
bitwise identical to synchronous runs; the interactive viewer is such a
consumer-free path. Policy sessions that read ``physical.sample`` to act may
observe one-period-older observations under a slow worker.

Wire protocol (parent -> worker), all payloads pickled explicitly so a
serialization failure is a normal error on the stepping path, never a silent
drop inside the queue feeder thread:
  ('context', None, bytes) -- rebuild worker-side close state
  ('close', seq, bytes)  -- pickled raws for one drained period
  ('stop', None, None)   -- terminate
Worker -> parent:
  ('ok', seq, result_dict)  -- fields to replay onto the runtime
  ('err', seq, (type_name, message))
"""
from __future__ import annotations

import pickle
import platform
import queue as _queue
from collections import deque
from types import MappingProxyType, SimpleNamespace

_RESULT_FIELDS = ('period_violations', 'energy',
                  '_last_endpoint_mass', '_last_measurement_source',
                  'attachment_samples', 'attachment_errors', 'step_violations',
                  'loss_j', 'active_work_j', 'external_work_j', 'solver_work_j',
                  'muscle_signed_j', 'muscle_positive_j', 'motor_signed_j',
                  'motor_positive_j', 'constraint_absolute_j',
                  'electrical_work_j', 'history', 'model_status')


def _sample_transport(sample):
    """PhysicalSample holds a MappingProxyType, which pickle cannot carry.

    Ship the validated fields verbatim; the parent restores the frozen object
    without rerunning __post_init__ — the worker already ran it on these exact
    values, and rechecking would double the cost the pipeline exists to move.
    """
    return (sample.interval_id, sample.time_s, sample.end_time_s,
            sample.qpos, sample.qvel, dict(sample.forces), sample.channels)


def _sample_restore(spec):
    from bike_sim.sim.ride.physical_samples import PhysicalSample, freeze
    interval_id, time_s, end_time_s, qpos, qvel, forces, channels = spec
    sample = PhysicalSample.__new__(PhysicalSample)
    object.__setattr__(sample, 'interval_id', interval_id)
    object.__setattr__(sample, 'time_s', time_s)
    object.__setattr__(sample, 'end_time_s', end_time_s)
    object.__setattr__(sample, 'qpos', freeze(qpos))
    object.__setattr__(sample, 'qvel', freeze(qvel))
    object.__setattr__(sample, 'forces',
                     MappingProxyType({k: freeze(v) for k, v in forces.items()}))
    object.__setattr__(sample, 'channels', channels)
    return sample


def _build_rt(ctx):
    """Worker-side stand-in carrying every close-state field publish_close needs."""
    from bike_sim.sim.ride.model_status import ModelStatus
    from bike_sim.sim.ride.physical_samples import WorkHistory
    rt = SimpleNamespace(
        cfg=ctx['cfg'],
        control_clock=SimpleNamespace(timestep_s=ctx['timestep_s']),
        rider_control=None,
        rider_contacts=None,
        history=ctx['history'] if ctx.get('history') is not None else WorkHistory(),
        model_status=(ctx['model_status'] if ctx.get('model_status') is not None
                      else ModelStatus()),
        interval_constraints={}, completed_samples=(), sample=None,
        period_violations=(), energy=ctx.get('energy') or {},
        _last_endpoint_mass=ctx.get('_last_endpoint_mass'),
        _last_measurement_source=ctx.get('_last_measurement_source'),
        attachment_samples={},
        attachment_errors=(), step_violations=(),
        initial_energy_j=ctx['initial_energy_j'],
        energy_scale_j=ctx['energy_scale_j'],
        initial_battery_j=ctx['initial_battery_j'],
        electrical_work_j=ctx['electrical_work_j'],
        loss_j=ctx['loss_j'], active_work_j=ctx['active_work_j'],
        external_work_j=ctx['external_work_j'], solver_work_j=ctx['solver_work_j'],
        muscle_signed_j=ctx['muscle_signed_j'],
        muscle_positive_j=ctx['muscle_positive_j'],
        motor_signed_j=ctx['motor_signed_j'],
        motor_positive_j=ctx['motor_positive_j'],
        constraint_absolute_j=ctx['constraint_absolute_j'])
    rider = ctx['rider_control']
    if rider is not None:
        rt.rider_control = _RiderControlView(rider)
    if ctx['has_rider_contacts']:
        rt.rider_contacts = SimpleNamespace(last_attachment_samples={},
                                            last_attachment_errors=())
    return rt


class _RiderControlView:
    """Data-only view that satisfies publish_close's rider_control reads.

    The real controller owns live MjData and cannot cross a process boundary;
    the close path only reads joints/config and calls three pure methods, so
    the same unbound implementations run here against plain data.
    """
    def __init__(self, spec):
        self.joints = spec['joints']
        self.config = spec['config']
        self.strength = spec['strength']
        self.strength_coordinates = spec['strength_coordinates']
        self.effort_diagnostics = {}
        self.last_terms = {}

    def strength_violations(self, active_torques, qpos, qvel):
        from bike_sim.sim.ride.rider_control import ArticulatedRiderController
        return ArticulatedRiderController.strength_violations(self, active_torques, qpos, qvel)

    def strength_capacity(self, name, angle_rad, velocity_rad_s, torque_nm):
        from bike_sim.sim.ride.rider_control import ArticulatedRiderController
        return ArticulatedRiderController.strength_capacity(
            self, name, angle_rad, velocity_rad_s, torque_nm)

    def anatomical_joint_angle(self, name, qpos_value):
        from bike_sim.sim.ride.rider_control import ArticulatedRiderController
        return ArticulatedRiderController.anatomical_joint_angle(self, name, qpos_value)


def _extract_result(rt):
    rider = rt.rider_control
    contacts = rt.rider_contacts
    return {
        'fields': {name: getattr(rt, name) for name in _RESULT_FIELDS},
        'samples': [_sample_transport(s) for s in rt.completed_samples],
        'interval_constraints': rt.interval_constraints,
        'rider_control': None if rider is None else {
            'effort_diagnostics': rider.effort_diagnostics,
            'last_terms': rider.last_terms},
        'last_attachment_samples': (None if contacts is None
                                    else contacts.last_attachment_samples),
        'last_attachment_errors': (None if contacts is None
                                   else contacts.last_attachment_errors),
        'failures': rt.period_violations}


def _close_worker_main(in_q, out_q):
    """Spawn entry: rebuild context, then evaluate closes until 'stop'."""
    from bike_sim.sim.ride.period_buffer import publish_close
    rt = None
    while True:
        kind, seq, payload = in_q.get()
        if kind == 'stop':
            return
        if kind == 'context':
            rt = _build_rt(pickle.loads(payload))
            continue
        try:
            raws = pickle.loads(payload)
            publish_close(rt, raws)
            out_q.put(('ok', seq, _extract_result(rt)))
        except Exception as exc:
            out_q.put(('err', seq, (type(exc).__name__, str(exc))))


class AsyncWorkerLost(Exception):
    """Transport-level failure: worker died or answered out of order."""


class AsyncPublicationError(RuntimeError):
    """Publication failed after a prefix that the parent still must apply once."""

    def __init__(self, message, completed_results):
        super().__init__(message)
        self.completed_results = tuple(completed_results)


def _worker_start_method(system: str) -> str:
    """Linux must not inherit scientific-library locks; retain Darwin's baseline."""
    return 'fork' if system == 'Darwin' else 'spawn'


def _close_queues(queues):
    # Closing discards work; owners drain explicitly before this boundary when
    # they need publication. Never wait for a feeder whose reader was killed.
    primary = None
    for index, channel in enumerate(queues):
        for operation in ('cancel_join_thread', 'close'):
            try:
                getattr(channel, operation)()
            except BaseException as error:
                if primary is None:
                    primary = error
                elif error is not primary:
                    primary.add_note(f'async queue {index} {operation} failed: '
                                     f'{type(error).__name__}: {error}')
    if primary is not None:
        raise primary


_WORKER_IMPORTS = (
    'bike_sim.sim.ride.period_buffer',
    'bike_sim.sim.ride.model_status',
    'bike_sim.sim.ride.physical_samples',
    'bike_sim.sim.ride.rider_control',
    'bike_sim.sim.ride.attachment_wrench',
    'bike_sim.sim.ride.rider_effort',
    'bike_sim.physics.energy_ledger',
    'bike_sim.physics.attachment_budget',
    'bike_sim.physics.joint_strength',
    'bike_sim.physics.rider_envelope',
    'bike_sim.sim.research.validity',
    'bike_sim.sim.research.quality',
)


class AsyncCloseRunner:
    """Parent-side handle: submit drained periods, collect applied results.

    Linux uses spawn after scientific imports. Darwin retains its existing
    fork route until a separately qualified launcher migration. Every entry
    point must guard its main function; no launch failure falls back to fork.
    """

    def __init__(self, *, start_method: str | None = None):
        import importlib
        import multiprocessing as mp
        method = _worker_start_method(platform.system()) if start_method is None else start_method
        available = mp.get_all_start_methods()
        if not isinstance(method, str) or method not in available:
            raise ValueError(f'unsupported async close start_method {method!r}; '
                             f'available methods: {available}')
        for name in _WORKER_IMPORTS:
            importlib.import_module(name)
        ctx = mp.get_context(method)
        self._seq = 0
        self._pending = deque()  # (seq, raws) submitted but not yet applied
        self._closed = False
        queues = []
        self._proc = None
        try:
            self._in_q = ctx.Queue()
            queues.append(self._in_q)
            self._out_q = ctx.Queue()
            queues.append(self._out_q)
            self._proc = ctx.Process(target=_close_worker_main,
                                     args=(self._in_q, self._out_q), daemon=True)
            self._proc.start()
        except BaseException as error:
            try:
                if self._proc is not None and self._proc.pid is not None:
                    self.close()
                else:
                    _close_queues(queues)
            except BaseException as cleanup:
                error.add_note(f'async startup cleanup failed: {type(cleanup).__name__}: {cleanup}')
            raise

    def send_context(self, ctx):
        self._in_q.put(('context', None, pickle.dumps(ctx)))

    def submit(self, raws):
        """Queue one period; raises if the raws cannot be serialized."""
        payload = pickle.dumps(list(raws))
        self._in_q.put(('close', self._seq, payload))
        self._pending.append((self._seq, raws))
        self._seq += 1

    def collect(self, *, block=False):
        """Return finished results in submission order.

        With block=True waits until every pending close has answered. A
        worker-side publication failure raises RuntimeError like a synchronous
        publish would; transport loss raises AsyncWorkerLost so the caller can
        replay the still-pending periods on the synchronous path.
        """
        results = []
        while len(results) < len(self._pending):
            if not block and self._out_q.empty():
                break
            self._check_alive()
            try:
                status, seq, payload = (self._out_q.get(timeout=0.5) if block
                                        else self._out_q.get_nowait())
            except _queue.Empty:
                if block:
                    continue
                break
            pending_seq = self._pending[len(results)][0]
            if seq != pending_seq:
                raise AsyncWorkerLost(
                    f'async close out of order: expected {pending_seq}, got {seq}')
            if status not in ('ok', 'err'):
                raise AsyncWorkerLost(f'async close invalid result status: {status!r}')
            if status == 'err':
                # The episode aborts on this error; later queued periods would
                # never be applied, so they do not keep the runner busy.
                self._pending.clear()
                type_name, message = payload
                raise AsyncPublicationError(
                    f'async close worker failed: {type_name}: {message}', results)
            results.append(payload)
        # No caller has applied these results yet. If a later response in the
        # same batch fails, keep the entire unacknowledged batch for replay.
        for _ in results:
            self._pending.popleft()
        return results

    def _check_alive(self):
        if not self._proc.is_alive():
            raise AsyncWorkerLost(
                f'async close worker exited (code {self._proc.exitcode}) '
                f'with {len(self._pending)} unacknowledged periods')

    def drain(self, apply):
        """Block until all pending results are consumed; apply each."""
        try:
            results = self.collect(block=True)
        except AsyncPublicationError as error:
            for result in error.completed_results:
                apply(result)
            raise
        for result in results:
            apply(result)

    def detach_pending(self):
        """Hand back unacknowledged raws for synchronous replay."""
        pending = [raws for _, raws in self._pending]
        self._pending.clear()
        return pending

    def close(self):
        if self._closed:
            return
        self._closed = True
        primary = None

        def remember(error):
            nonlocal primary
            if primary is None:
                primary = error
            elif error is not primary:
                primary.add_note(f'async worker cleanup also failed: '
                                 f'{type(error).__name__}: {error}')

        try:
            try:
                self._in_q.put(('stop', None, None))
            except Exception:
                pass
            self._proc.join(timeout=2.)
        except BaseException as error:
            remember(error)
        try:
            for stop in (self._proc.terminate, self._proc.kill):
                if not self._proc.is_alive():
                    break
                try:
                    stop()
                except BaseException as error:
                    remember(error)
                try:
                    self._proc.join(timeout=2.)
                except BaseException as error:
                    remember(error)
            if self._proc.is_alive():
                remember(AsyncWorkerLost('async close worker did not exit after kill'))
        except BaseException as error:
            remember(error)
        finally:
            try:
                _close_queues((self._in_q, self._out_q))
            except BaseException as error:
                remember(error)
        if primary is not None:
            raise primary
