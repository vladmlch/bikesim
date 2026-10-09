"""Application adapter for the owned native ride runtime (A3/A4).

Advancement performs no per-interval Python accounting or sample boxing.
Explicit flush closes a partial physical period; snapshot, drain and wall
budget yields do not. Native samples are acknowledged only after the owned
Python batch has been constructed successfully.
"""
from __future__ import annotations

from numbers import Integral
from dataclasses import replace
from pathlib import Path
import tempfile

from bike_sim.native.contracts import (
    AdvanceResult, FrameSnapshot, SampleBatch,
    _freeze, _owned_array,
)
from bike_sim.native.setup import RuntimeBootstrap, capture_bootstrap, _control_dict
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import PhysicalSample
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun

__all__ = ['NativeRideDriver', 'create_native_ride']


def _integer(value, name, *, minimum=0):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ValueError(f'{name} must be an integer >= {minimum}')
    return int(value)


def _sample(native):
    if native is None:
        return None
    return PhysicalSample(
        interval_id=int(native.interval_id), time_s=float(native.time_s),
        end_time_s=float(native.end_time_s), qpos=native.qpos, qvel=native.qvel,
        forces=native.forces, channels=_freeze(native.channels))


class NativeRideDriver:
    """Facade over ``NativeRideRuntime`` returning owning contract values."""

    def __init__(self, runtime, bootstrap: RuntimeBootstrap, hold_dir=None,
                 *, native_reference_error=None, initial_frame=None) -> None:
        self._native = runtime
        self._bootstrap = bootstrap
        self._hold_dir = hold_dir
        self._initial_frame = initial_frame
        if native_reference_error is None:
            from bike_sim.native.artifact import load_native_extension
            native_reference_error = load_native_extension().InvalidReferenceRun
        self._native_reference_error = native_reference_error

    def _call(self, operation, *args, **kwargs):
        try:
            return operation(*args, **kwargs)
        except self._native_reference_error as error:
            # Keep the existing application exception family, without turning
            # engine failures or validation errors into monitor rejections.
            raise InvalidReferenceRun(str(error)) from error

    def advance(self, target_step: int, control=None, *,
                front_brake_demand: float = 0.0,
                rear_brake_demand: float = 0.0,
                wall_budget_s: float | None = None) -> AdvanceResult:
        """Advance to an absolute integer boundary, retaining a partial period."""
        target = _integer(target_step, 'target_step')
        command = RideControl() if control is None else control
        if not isinstance(command, RideControl):
            raise ValueError('expected an immutable RideControl')
        result = self._call(
            self._native.advance, target, _control_dict(command),
            front_brake_demand=front_brake_demand,
            rear_brake_demand=rear_brake_demand, wall_budget_s=wall_budget_s)
        return AdvanceResult(step=int(result.step), time_s=float(result.time_s),
                             reason=str(result.reason), outcome=result.outcome)

    def snapshot(self) -> FrameSnapshot:
        """The last committed engine boundary and latest published sample."""
        snap = self._native.snapshot()
        from bike_sim.sim.ride.physical_view import native_frame
        frame = native_frame(snap, _sample(snap.latest_sample))
        # Bootstrap restores the integration vector without mj_forward. The
        # already-settled setup owns the t=0 geometry/diagnostics; no extra
        # engine solve may be inserted merely to draw the initial frame.
        if frame.step == 0 and self._initial_frame is not None:
            frame = replace(frame, view=self._initial_frame.view,
                            model_fields=self._initial_frame.model_fields)
        return frame

    def make_render_model(self):
        import mujoco
        return mujoco.MjModel.from_binary_path(str(self._bootstrap.model_path))

    def set_finish_position(self, position_m):
        self._native.set_finish_position(position_m)

    def release_samples(self):
        """Release pending delivery, retaining native decimated recording."""
        self._native.release_samples()

    def recorded_intervals(self):
        return self._native.recorded_intervals()

    def probe_step_inputs(self, control=None, *,
                          front_brake_demand: float = 0.0,
                          rear_brake_demand: float = 0.0) -> dict:
        """Stage input diagnostics without moving committed runtime state."""
        command = RideControl() if control is None else control
        if not isinstance(command, RideControl):
            raise ValueError('expected an immutable RideControl')
        return self._native.probe_step_inputs(
            _control_dict(command), front_brake_demand=front_brake_demand,
            rear_brake_demand=rear_brake_demand)

    def drain_samples(self) -> SampleBatch:
        """Drain only closed intervals, with acknowledgement after boxing."""
        pending = self._native.prepare_samples()
        batch = SampleBatch.from_native(pending)
        self._native.acknowledge_samples(pending)
        return batch

    def flush(self) -> None:
        """Close/account a partial period, leaving its samples pending."""
        self._call(self._native.flush)

    @property
    def first_failure(self):
        return _freeze(self._native.accounting_state()['monitor']['first_failure'])

    @property
    def model_status(self):
        return _freeze(self._native.accounting_state()['model_status'])

    @property
    def accounting_state(self):
        """Owned accounting/history diagnostics, including unflushed counts."""
        return _freeze(self._native.accounting_state())

    @property
    def recorded_columns(self):
        """Decimated native recorder columns; missing fields are NaN."""
        return _freeze({name: _owned_array(values, name)
                        for name, values in self._native.recorded_columns().items()})

    def reset(self) -> FrameSnapshot:
        """Close the outgoing period, then replay t=0 in a new generation.

        A strict rejection leaves the old generation drainable. Retry reset
        after handling that rejection. A successful reset clears old output.
        """
        self._call(self._native.reset)
        return self.snapshot()

    def close(self) -> None:
        self._native.close()
        if self._hold_dir is not None:
            self._hold_dir.cleanup()
            self._hold_dir = None

    @property
    def closed(self) -> bool:
        return bool(self._native.closed)


def create_native_ride(sim, *, strict: bool | None = None,
                       record_decimation: int | None = None,
                       directory=None) -> NativeRideDriver:
    """Capture a settled t=0 ride and build its independent native owner."""
    physical = getattr(sim, 'physical', None)
    if physical is None:
        raise ValueError('create_native_ride requires a physical ride runtime')
    if strict is not None and not isinstance(strict, bool):
        raise ValueError('strict must be bool')
    decimation = None if record_decimation is None else _integer(
        record_decimation, 'record_decimation', minimum=1)
    if strict is not None:
        physical.set_strict(strict)
    if decimation is not None:
        physical.set_record_decimation(decimation)
    hold_dir = None
    if directory is None:
        hold_dir = tempfile.TemporaryDirectory(prefix='bike_sim_native_')
        directory = hold_dir.name
    native = None
    try:
        from bike_sim.sim.ride.physical_view import snapshot_python
        initial_frame = snapshot_python(sim)
        bootstrap = capture_bootstrap(sim, Path(directory))
        from bike_sim.native.artifact import load_native_extension
        extension = load_native_extension()
        native = extension.NativeRideRuntime(
            str(bootstrap.model_path), bootstrap.config, bootstrap.state)
        return NativeRideDriver(native, bootstrap, hold_dir,
            native_reference_error=extension.InvalidReferenceRun, initial_frame=initial_frame)
    except BaseException as error:
        for resource in (native, hold_dir):
            if resource is not None:
                try:
                    resource.close() if resource is native else resource.cleanup()
                except BaseException as cleanup:
                    error.add_note(f'native setup cleanup failed: {cleanup}')
        raise
