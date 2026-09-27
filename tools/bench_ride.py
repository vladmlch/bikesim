"""
Ride-Mode Step-Cost Benchmark.

Measures what one ride-mode step costs on this machine, and where the time goes, so the
pneumatic tyre's `fast` tier has a budget to be held to (docs/superpowers/plans/
2026-09-26-pneumatic-tyre.md, Task 0; docs/RIDE.md section 10).

Each track is ridden headless from its solved equilibrium to its end, exactly as
`RideSimulation.run` does it. Every per-step writer, the contact query and `mj_step` are
wrapped in a `perf_counter` timer *after* construction and reset, so the equilibrium solve is
not counted. The wrappers cost a few hundred nanoseconds per call; `overhead` is what is left
of the step once the timed parts are subtracted, and includes them.

**The budget.** The interactive viewer paces the simulation to wall clock and spends part of
every real second outside the physics (`viewer.sync` at 120 Hz, HUD refresh, a 1 ms frame
sleep). A step must therefore cost well under one timestep of wall clock. With a safety
margin `m` (default 30 %), the time a new per-step writer may add is

    budget = timestep * (1 - m) - measured step cost

and the worst track decides it.

Run with:  uv run python -m tools.bench_ride [--tracks flat road_worn ...] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import mujoco

from bike_sim.physics.tyre import TYRE_MODELS, TYRE_TIERS, TyreConfig
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import available_presets, get_preset

DEFAULT_MARGIN = 0.30

# (label, attribute path on the simulation, method name). Order follows `RideSimulation.step`.
TIMED_PARTS = (
    ("tyre", "tyre_applier", "apply"),
    ("suspension", "applier", "apply"),
    ("rider", "rider_forces", "apply"),
    ("rolling_resistance", "resistance", "apply"),
    ("cruise", "cruise", "compute"),
    ("brakes", "brakes", "compute"),
    ("virtual_rider", "stabilizer", "apply"),
    ("crash_check", "crash_detector", "check"),
    ("contact_query", "contact_query", "query"),
)


@dataclass
class TrackResult:
    """Timing of one traverse."""

    track: str
    tyre_model: str
    tyre_tier: str
    outcome: str
    steps: int
    sim_time_s: float
    wall_time_s: float
    steps_per_s: float
    real_time_factor: float
    step_us: float
    parts_us: Dict[str, float] = field(default_factory=dict)


class _Timer:
    """Accumulates wall time spent inside a wrapped callable."""

    def __init__(self, fn: Callable) -> None:
        self.fn = fn
        self.total_s = 0.0

    def __call__(self, *args, **kwargs):
        t0 = time.perf_counter()
        try:
            return self.fn(*args, **kwargs)
        finally:
            self.total_s += time.perf_counter() - t0


def bench_track(
    name: str,
    rider: Optional[str] = None,
    tyre: Optional[TyreConfig] = None,
) -> TrackResult:
    """
    Rides one track headless and times every part of the step.

    Args:
        name: Track preset name.
        rider: Rider variant; None for the default (seated).
        tyre: Wheel contact model and fidelity tier; defaults to sphere.

    Returns:
        The traverse's timing.
    """
    config = tyre if tyre is not None else TyreConfig()
    sim = RideSimulation(track=get_preset(name), rider=rider, tyre=config)

    timers: Dict[str, _Timer] = {}
    for label, owner_name, method in TIMED_PARTS:
        owner = getattr(sim, owner_name)
        if owner is None:
            continue
        timers[label] = _Timer(getattr(owner, method))
        setattr(owner, method, timers[label])

    original_step = mujoco.mj_step
    timers["mj_step"] = _Timer(original_step)
    mujoco.mj_step = timers["mj_step"]
    try:
        t0 = time.perf_counter()
        outcome = sim.run()
        wall_s = time.perf_counter() - t0
    finally:
        mujoco.mj_step = original_step

    steps = max(outcome.steps, 1)
    parts_us = {
        label: 1e6 * timers[label].total_s / steps if label in timers else 0.0
        for label, _, _ in TIMED_PARTS
    }
    parts_us["mj_step"] = 1e6 * timers["mj_step"].total_s / steps
    step_us = 1e6 * wall_s / steps
    parts_us["overhead"] = step_us - sum(parts_us.values())
    return TrackResult(
        track=name,
        tyre_model=config.model,
        tyre_tier=config.tier,
        outcome=outcome.reason,
        steps=outcome.steps,
        sim_time_s=sim.time_s,
        wall_time_s=wall_s,
        steps_per_s=outcome.steps / wall_s,
        real_time_factor=sim.time_s / wall_s,
        step_us=step_us,
        parts_us=parts_us,
    )


def budget_us(results: Sequence[TrackResult], timestep_s: float, margin: float) -> float:
    """
    Per-step time a new writer may add while the viewer keeps real time.

    Args:
        results: Measured traverses.
        timestep_s: Integration timestep.
        margin: Fraction of each timestep reserved for the viewer's own work.

    Returns:
        The budget in microseconds, set by the slowest track.
    """
    worst = max(r.step_us for r in results)
    return 1e6 * timestep_s * (1.0 - margin) - worst


def format_table(results: Sequence[TrackResult], timestep_s: float, margin: float) -> str:
    """Formats the results as a Markdown table plus the budget line."""
    parts = list(results[0].parts_us)
    head = ["track", "tyres", "outcome", "steps", "RTF", "µs/step", *parts]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for r in results:
        tyre_label = r.tyre_model if r.tyre_model == "sphere" else f"{r.tyre_model}/{r.tyre_tier}"
        cells = [r.track, tyre_label, r.outcome, str(r.steps), f"{r.real_time_factor:.2f}×", f"{r.step_us:.1f}"]
        cells += [f"{r.parts_us[p]:.1f}" for p in parts]
        lines.append("| " + " | ".join(cells) + " |")
    lines.append("")
    sphere_results = [r for r in results if r.tyre_model == "sphere"]
    if sphere_results:
        lines.append(
            f"timestep {1e6 * timestep_s:.0f} µs, margin {100 * margin:.0f} % -> per-step budget for a "
            f"new writer: **{budget_us(sphere_results, timestep_s, margin):.0f} µs** "
            "(set by the slowest sphere track)"
        )
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--tracks", nargs="*", default=None, help="presets to ride (default: all)")
    parser.add_argument("--rider", default=None, help="rider variant (default: seated)")
    parser.add_argument("--margin", type=float, default=DEFAULT_MARGIN)
    parser.add_argument("--tyre-model", choices=TYRE_MODELS, default="sphere")
    parser.add_argument("--tyre-tier", choices=TYRE_TIERS, default="fast")
    parser.add_argument("--json", type=Path, default=None, help="also write the results as JSON")
    args = parser.parse_args(argv)

    names = args.tracks or available_presets()
    tyre = TyreConfig(model=args.tyre_model, tier=args.tyre_tier)
    results = []
    for name in names:
        r = bench_track(name, rider=args.rider, tyre=tyre)
        tyre_label = r.tyre_model if r.tyre_model == "sphere" else f"{r.tyre_model}/{r.tyre_tier}"
        print(f"{name:20s} {tyre_label:18s} {r.outcome:12s} {r.steps:7d} steps  "
              f"{r.real_time_factor:6.2f}x real time  "
              f"{r.step_us:6.1f} us/step", flush=True)
        results.append(r)

    timestep_s = float(
        RideSimulation(track=get_preset("flat"), rider=args.rider, tyre=tyre).model.opt.timestep
    )
    print()
    print(format_table(results, timestep_s, args.margin))
    if args.json is not None:
        args.json.write_text(json.dumps({
            "timestep_s": timestep_s,
            "margin": args.margin,
            "budget_us": budget_us(results, timestep_s, args.margin) if args.tyre_model == "sphere" else None,
            "tyre_model": args.tyre_model,
            "tyre_tier": args.tyre_tier,
            "results": [asdict(r) for r in results],
        }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
