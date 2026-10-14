"""Off-path period publication: a spawned process evaluates closes.

PhysicalRuntime accumulates RawStep payloads for one control period, then
publishes them through publish_close — pure data work (attachment wrenches,
effort channels, ledger bookkeeping, sample assembly) that never touches live
mjData. That makes it movable: this module runs the same publish_close inside
a spawned worker, so the stepping loop only serializes raws and applies
results at the next step boundary.

Semantics: identical code on identical data, applied one step later. Runs
where nothing consumes mid-period published state for control decisions are
bitwise identical to synchronous runs; the interactive viewer is such a
consumer-free path. Policy sessions that read ``physical.sample`` to act may
observe one-period-older observations under a slow worker.

Wire protocol (parent -> worker), all payloads pickled explicitly so a
serialization failure is a normal error on the stepping path, never a silent
drop inside the queue feeder thread:
  ('context', ctx_dict)  -- rebuild worker-side close state
  ('close', seq, bytes)  -- pickled raws for one drained period
  ('stop',)              -- terminate
Worker -> parent:
  ('ok', seq, result_dict)  -- fields to replay onto the runtime
  ('err', seq, (type_name, message))
"""
from __future__ import annotations

import pickle
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

    Fork, not spawn: a spawned interpreter re-imports ``__main__``, which
    re-executes console-script entry points that lack an ``__main__`` guard
    (uv wrappers) or hangs on non-module mains. A forked worker inherits the
    parent's module table, does pure numpy/pickle work, and never touches
    mujoco or Objective-C, which is the safe fork pattern on macOS.
    """

    def __init__(self):
        import importlib
        for name in _WORKER_IMPORTS:
            importlib.import_module(name)
        from multiprocessing import get_context
        ctx = get_context('fork')
        self._in_q = ctx.Queue()
        self._out_q = ctx.Queue()
        self._proc = ctx.Process(target=_close_worker_main,
                                 args=(self._in_q, self._out_q), daemon=True)
        self._proc.start()
        self._seq = 0
        self._pending = deque()  # (seq, raws) submitted but not yet applied
        self._closed = False

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
        while self._pending:
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
            pending_seq = self._pending[0][0]
            if seq != pending_seq:
                raise AsyncWorkerLost(
                    f'async close out of order: expected {pending_seq}, got {seq}')
            self._pending.popleft()
            if status == 'err':
                # The episode aborts on this error; later queued periods would
                # never be applied, so they do not keep the runner busy.
                self._pending.clear()
                type_name, message = payload
                raise RuntimeError(f'async close worker failed: {type_name}: {message}')
            results.append(payload)
        return results

    def _check_alive(self):
        if not self._proc.is_alive():
            raise AsyncWorkerLost(
                f'async close worker exited (code {self._proc.exitcode}) '
                f'with {len(self._pending)} unacknowledged periods')

    def drain(self, apply):
        """Block until all pending results are consumed; apply each."""
        for result in self.collect(block=True):
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
        try:
            self._in_q.put(('stop', None, None))
        except Exception:
            pass
        self._proc.join(timeout=2.)
        if self._proc.is_alive():
            self._proc.terminate()
        self._in_q.close()
        self._out_q.close()
