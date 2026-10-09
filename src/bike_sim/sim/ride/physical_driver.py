"""Backend-independent physical ride owner, recorder and committed-frame adapter.

Only the Python backend calls RideSimulation.step. Native advancement, recording,
force accounting and termination stay in C++; setup metadata is captured once.
"""
from __future__ import annotations

from copy import copy, deepcopy
from dataclasses import replace
import csv
import json
import math
from numbers import Integral, Real
from pathlib import Path
import time

import numpy as np

from bike_sim.native.contracts import AdvanceResult
from bike_sim.sim.backend import require_backend
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.physical_view import present_view, snapshot_python


class NativePhysicalRecorder:
    schema_version = 2

    def __init__(self, driver):
        self.driver = driver

    def columns(self):
        return self.driver.recorded_columns

    @property
    def rows(self):
        columns = self.columns()
        return 0 if not columns else len(next(iter(columns.values())))

    @property
    def samples(self):
        return self.driver.recorded_intervals()

    def write_csv(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        columns = self.columns()
        names = sorted(columns) if columns else ['schema_version', 'time_s']
        length = 0 if not columns else len(next(iter(columns.values())))
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(names)
            for index in range(length):
                writer.writerow(['' if np.isnan(columns[name][index]) else
                                 repr(float(columns[name][index])) for name in names])
        return path

    def write_jsonl(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as stream:
            for row in self.samples:
                stream.write(json.dumps(plain(row), sort_keys=True, allow_nan=False)+'\n')
        return path


class PhysicalRideDriver:
    """Absolute-step advancement; wall pacing never changes the physics clock."""

    def __init__(self, sim, *, backend='python', strict=False, decimation=1,
                 duration_s=None, seed=None, clock=time.monotonic):
        require_backend(backend, sim.physics_config, sim.rider)
        if type(decimation) is not int or decimation < 1:
            raise ValueError('decimation must be a positive integer')
        if duration_s is not None and (isinstance(duration_s, bool) or
                not isinstance(duration_s, Real) or not math.isfinite(duration_s) or duration_s <= 0.):
            raise ValueError('duration_s must be finite and positive')
        from bike_sim.sim.ride.physical_session import configuration_metadata
        from bike_sim.native.artifact import execution_provenance
        self.backend, self._clock = backend, clock
        self.record_decimation, self.strict = decimation, bool(strict)
        self.dt_s = float(sim.model.opt.timestep)
        self.track = deepcopy(sim.track)
        self.equilibrium = deepcopy(sim.equilibrium)
        self.vertices = np.array(sim.physical.vertices, copy=True)
        self.vertices.flags.writeable = False
        self.metadata = configuration_metadata(sim, seed)
        self.metadata['execution'] = execution_provenance(backend)
        self.metadata['backend'] = backend
        self.limits = sim.default_limits()
        self.max_steps = (self.limits.max_steps if duration_s is None
                          else math.ceil(duration_s / self.dt_s))
        self.limit_reason = 'step_cap' if duration_s is None else 'duration_reached'
        self._decimation = decimation
        self._closed = False
        self.reason = None
        self.step, self.time_s = int(sim.steps), float(sim.time_s)
        self._native = None
        self._sim = None
        sim.physical.set_strict(strict)
        if backend == 'native':
            from bike_sim.native.runtime import create_native_ride
            native = create_native_ride(sim, strict=strict, record_decimation=decimation)
            try:
                native.set_finish_position(self.limits.finish_x_m)
            except BaseException:
                native.close()
                raise
            self._native = native
            self.recorder = NativePhysicalRecorder(native)
        else:
            self._sim = sim
            self.recorder = PhysicalRecorder(sim, decimation)
        self.start()

    def start(self):
        self._started = self._clock()

    def _require_open(self):
        if self._closed:
            raise RuntimeError('physical ride driver is closed')

    @property
    def wall_clock_s(self):
        return max(0., self._clock() - self._started)

    def _python_reason(self):
        if self._sim.crash is not None:
            return 'crash'
        if self._sim.position_m >= self.limits.finish_x_m:
            return 'end_of_track'
        if self.step >= self.max_steps:
            return self.limit_reason
        return None

    def advance(self, target_step, control=None, *, front_brake_demand=0.,
                rear_brake_demand=0., wall_budget_s=None):
        self._require_open()
        if isinstance(target_step, bool) or not isinstance(target_step, Integral) or target_step < self.step:
            raise ValueError('target_step must be an integer at or after the committed step')
        if wall_budget_s is not None and (isinstance(wall_budget_s, bool) or
                not isinstance(wall_budget_s, Real) or not math.isfinite(wall_budget_s) or wall_budget_s < 0.):
            raise ValueError('wall_budget_s must be finite and nonnegative')
        command = RideControl() if control is None else control
        if not isinstance(command, RideControl):
            raise ValueError('expected an immutable RideControl')
        for demand in (front_brake_demand, rear_brake_demand):
            if isinstance(demand, bool) or not isinstance(demand, Real) or not math.isfinite(demand):
                raise ValueError('brake demands must be finite real numbers')
        if self.reason is None and self.wall_clock_s >= self.limits.max_wall_clock_s:
            self.reason = 'wall_clock_cap'
        target = min(int(target_step), self.max_steps)
        result_reason = 'target'
        if self.reason is None and self._native is not None:
            try:
                result = self._native.advance(target, command,
                    front_brake_demand=front_brake_demand, rear_brake_demand=rear_brake_demand,
                    wall_budget_s=wall_budget_s)
            except BaseException as error:
                # Native checks can reject after publishing an owned prefix.
                # Keep frontend counters truthful even when no result is returned.
                try:
                    frame = self._native.snapshot()
                    self.step, self.time_s = frame.step, frame.time_s
                    self.reason = ('crash' if frame.outcome and frame.outcome.startswith('crash')
                                   else frame.outcome)
                except BaseException as snapshot_error:
                    error.add_note(f'committed snapshot unavailable: {type(snapshot_error).__name__}: {snapshot_error}')
                raise
            self.step, self.time_s = result.step, result.time_s
            result_reason = result.reason
            self.reason = ('crash' if result.outcome and result.outcome.startswith('crash')
                           else result.outcome)
            self._native.release_samples()
        elif self.reason is None:
            deadline = None if wall_budget_s is None else self._clock()+wall_budget_s
            while self.step < target:
                self.reason = self._python_reason()
                if self.reason is not None:
                    break
                if self.wall_clock_s >= self.limits.max_wall_clock_s:
                    self.reason = 'wall_clock_cap'
                    break
                if deadline is not None and self._clock() >= deadline:
                    result_reason = 'budget'
                    break
                try:
                    self._sim.step(front_brake_demand, rear_brake_demand, control=command)
                finally:
                    self.step, self.time_s = int(self._sim.steps), float(self._sim.time_s)
                    self.recorder.record(self._sim)
            if self.reason is None:
                self.reason = self._python_reason()
        if self.reason is None and self.step >= self.max_steps:
            self.reason = self.limit_reason
        return AdvanceResult(self.step, self.time_s,
                             'outcome' if self.reason is not None else result_reason, self.reason)

    def snapshot(self):
        self._require_open()
        frame = (self._native.snapshot() if self._native is not None
                 else snapshot_python(self._sim, outcome=self.reason))
        self.step, self.time_s = frame.step, frame.time_s
        return replace(frame, view=present_view(frame.view, track=self.track),
                       outcome=self.reason or frame.outcome)

    def make_render_model(self):
        self._require_open()
        return self._native.make_render_model() if self._native is not None else copy(self._sim.model)

    def flush(self):
        self._require_open()
        if self._native is not None:
            self._native.flush()
            self._native.release_samples()
        else:
            try:
                self._sim.physical.flush()
            finally:
                self.recorder.record(self._sim)

    def reset(self):
        self._require_open()
        if self._native is not None:
            self._native.reset()
        else:
            self._sim.reset()
            self.recorder = PhysicalRecorder(self._sim, self._decimation)
        self.step, self.time_s, self.reason = 0, 0., None
        self.start()
        return self.snapshot()

    def summary(self, reason=None):
        reason = self.reason if reason is None else reason
        if self._native is None:
            from bike_sim.sim.ride.physical_session import physical_summary
            return physical_summary(self._sim, self.metadata, reason)
        frame = self.snapshot()
        accounting = self._native.accounting_state
        endpoint = frame.view.endpoint
        history = accounting['history']
        balance = endpoint.get('balance_lost_at_m')
        return plain(dict(self.metadata, first_failure=frame.first_failure,
            outcome=dict(reason=reason, time_s=frame.time_s,
                position_m=endpoint['position_m'], steps=frame.step,
                crashed=bool(frame.outcome and frame.outcome.startswith('crash')),
                balance_lost=balance is not None, balance_lost_at_m=balance),
            equilibrium=self.equilibrium, energy=accounting['energy'],
            model_status=frame.model_status, battery_energy_j=endpoint['battery_energy_j'],
            component_work_j=history['work_j'], airtime_threshold_s=history['airtime_s'],
            duration_s=history['duration_s']))

    def export(self, directory, *, reason=None):
        """Export already-accounted values; callers explicitly flush before this."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.recorder.write_csv(directory/'telemetry.csv')
        self.recorder.write_jsonl(directory/'intervals.jsonl')
        np.save(directory/'terrain_vertices.npy', self.vertices, allow_pickle=False)
        summary = self.summary(reason)
        (directory/'summary.json').write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)+'\n', encoding='utf-8')
        return summary

    def close(self):
        if self._closed:
            return
        if self._native is not None:
            self._native.close()
        self._closed = True


def build_physical_driver(track, args, rider, *, seed=None, strict=None):
    # Validate capabilities/artifact BEFORE equilibrium, file output or capture.
    backend = getattr(args, 'backend', 'python')
    require_backend(backend, args.resolved_physics, rider)
    from bike_sim.sim.ride.physical_session import build_physical_simulation
    sim = build_physical_simulation(track, args, rider)
    return PhysicalRideDriver(sim, backend=backend,
        strict=args.headless if strict is None else strict, decimation=args.decimate,
        duration_s=args.duration, seed=seed)
