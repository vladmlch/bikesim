"""Application-side adapter for the owned native ride runtime (plan A3).

``create_native_ride`` freezes a freshly initialized Python physical ride
through ``capture_bootstrap`` and constructs ``NativeRideRuntime`` on the
artifact. The extension import stays inside this module's functions so
Python-only frontends never pay it; artifact loading remains the explicit
``bike_sim.native.artifact.load_native_extension`` boundary.

``NativeRideDriver.advance`` takes the command for every call — the native
side never holds a command across advance() invocations. Sample draining
(``drain_samples``/``flush``) is the A4 boundary: until period accounting
lands natively, both return an empty batch rather than a partial one.
"""
from __future__ import annotations

from pathlib import Path
import tempfile

from bike_sim.native.contracts import AdvanceResult, FrameSnapshot, SampleBatch
from bike_sim.native.setup import RuntimeBootstrap, capture_bootstrap, _control_dict
from bike_sim.sim.ride.control import RideControl

__all__ = ['NativeRideDriver', 'create_native_ride']


class NativeRideDriver:
    """Facade over the bound ``NativeRideRuntime`` returning contract types."""

    def __init__(self, runtime, bootstrap: RuntimeBootstrap,
                 hold_dir=None) -> None:
        self._native = runtime
        self._bootstrap = bootstrap
        # The bootstrap artifact must outlive construction only, but keeping
        # the TemporaryDirectory referenced also keeps later reset() calls on
        # the same captured bytes when callers dropped them.
        self._hold_dir = hold_dir

    def advance(self, target_step: int, control=None, *,
                front_brake_demand: float = 0.0,
                rear_brake_demand: float = 0.0,
                wall_budget_s: float | None = None) -> AdvanceResult:
        """Advance committed physics steps until ``target_step``.

        ``reason`` is 'target' when the target commits, 'budget' when the
        wall-clock budget stops the loop between steps, and 'outcome' when a
        terminal crash latches. A failure mid-range preserves the committed
        prefix — it is never rolled back.
        """
        command = RideControl() if control is None else control
        if not isinstance(command, RideControl):
            raise ValueError('expected an immutable RideControl')
        result = self._native.advance(
            int(target_step), _control_dict(command),
            front_brake_demand=float(front_brake_demand),
            rear_brake_demand=float(rear_brake_demand),
            wall_budget_s=wall_budget_s)
        return AdvanceResult(step=int(result.step), time_s=float(result.time_s),
                             reason=str(result.reason),
                             outcome=result.outcome)

    def snapshot(self) -> FrameSnapshot:
        """The last committed boundary — an owned copy, never a live view."""
        snap = self._native.snapshot()
        return FrameSnapshot(generation=int(snap.generation),
                             step=int(snap.step), time_s=float(snap.time_s),
                             integration_state=snap.integration_state,
                             outcome=snap.outcome)

    def probe_step_inputs(self, control=None, *,
                          front_brake_demand: float = 0.0,
                          rear_brake_demand: float = 0.0) -> dict:
        """apply_forces(advance=False): ordered components, actuator input,
        model brake bounds and staged diagnostics — without advancing."""
        command = RideControl() if control is None else control
        if not isinstance(command, RideControl):
            raise ValueError('expected an immutable RideControl')
        return self._native.probe_step_inputs(
            _control_dict(command),
            front_brake_demand=float(front_brake_demand),
            rear_brake_demand=float(rear_brake_demand))

    def drain_samples(self) -> SampleBatch:
        """A4's acknowledged batch boundary — empty until period close lands."""
        return SampleBatch()

    def flush(self) -> SampleBatch:
        return SampleBatch()

    def reset(self) -> None:
        """Replay the captured t=0 bootstrap (generation increments)."""
        self._native.reset()

    def close(self) -> None:
        self._native.close()

    @property
    def closed(self) -> bool:
        return bool(self._native.closed)


def create_native_ride(sim, *, strict: bool | None = None,
                       record_decimation: int | None = None,
                       directory=None) -> NativeRideDriver:
    """Capture ``sim``'s settled t=0 bootstrap and build the owned runtime.

    ``strict``/``record_decimation`` override the physical runtime's flags
    before capture (both are bootstrap fields). ``directory`` pins the model
    artifact location for inspection; a managed TemporaryDirectory is used
    otherwise.
    """
    physical = getattr(sim, 'physical', None)
    if physical is None:
        raise ValueError('create_native_ride requires a physical ride runtime')
    if strict is not None:
        physical.set_strict(bool(strict))
    if record_decimation is not None:
        physical.set_record_decimation(int(record_decimation))
    hold_dir = None
    if directory is None:
        hold_dir = tempfile.TemporaryDirectory(prefix='bike_sim_native_')
        directory = hold_dir.name
    bootstrap = capture_bootstrap(sim, Path(directory))
    from bike_sim.native.artifact import load_native_extension
    native = load_native_extension().NativeRideRuntime(
        str(bootstrap.model_path), bootstrap.config, bootstrap.state)
    return NativeRideDriver(native, bootstrap, hold_dir)
