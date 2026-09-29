"""Opt-in physical rider regression, without GUI or an anti-wheelie controller.

Run from the checkout with PYTHONPATH=src. This is intentionally not a default
pytest test: settling and an actual climb are more expensive than law tests.
Preview mode uses the same forces/contacts but skips scientific work accounting;
this check is not a timestep-convergence or energy-conservation certificate.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from bike_sim.physics.resolution import load_physics_config
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain.research import build_research_track


def check(track: str, duration_s: float, initial_speed_mps: float,
          config: Path) -> dict:
    """Run real initialization and contacts, with no saved-state injection."""
    if not math.isfinite(duration_s) or duration_s < 2.:
        raise ValueError('duration must be finite and at least 2 seconds')
    if not math.isfinite(initial_speed_mps) or initial_speed_mps < 0.:
        raise ValueError('initial speed must be finite and nonnegative')
    cfg = replace(load_physics_config(config), initial_speed_mps=initial_speed_mps)
    if cfg.physics_mode != 'physical' or cfg.drive_mode != 'articulated_effort':
        raise ValueError('this check requires physical/articulated_effort mode')
    sim = RideSimulation(track=build_research_track(track),
                         rider='articulated_planar', physics_config=cfg)
    runtime = sim.physical
    model, data = sim.model, sim.data
    crank_q, crank_v = runtime.address('crank_spin')
    x0 = float(data.qpos[sim.root_x_qposadr])
    crank0 = float(data.qpos[crank_q])
    dt = float(model.opt.timestep)
    minimum_speed = math.inf
    pedal_work = 0.
    minimum_load = dict.fromkeys(('front', 'rear'), math.inf)
    maximum_gap = 0.
    failures = []
    bad_warnings = [getattr(mujoco.mjtWarning, 'mjWARN_' + name)
                    for name in ('BADQPOS', 'BADQVEL', 'BADQACC', 'BADCTRL')]
    warning_counts = [int(data.warning[item].number) for item in bad_warnings]
    # Unlike a pose animation, every step goes through the real contact, limb,
    # transmission and tire force writers. No speed/pose correction is applied.
    with runtime.preview_mode():
        for _ in range(math.ceil(duration_s / dt)):
            runtime.step()
            if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
                failures.append('non-finite state')
                break
            if sim.crash_detector.event is not None:
                failures.append('crash: ' + str(sim.crash_detector.event))
                break
            if data.time >= .5:
                minimum_speed = min(minimum_speed, float(sim.speed_mps))
            pedal_work += (runtime.rider_contacts.delivered_crank_torque_nm
                           * float(data.qvel[crank_v]) * dt)
            for side, snapshot in runtime.snapshots.items():
                minimum_load[side] = min(minimum_load[side], float(snapshot.normal_load_n))
            for side in ('front', 'rear'):
                gap = runtime.rider_contacts.diagnostics[side + '_pedal']['gap_m']
                maximum_gap = max(maximum_gap, float(gap))
    if any(int(data.warning[item].number) > old
           for item, old in zip(bad_warnings, warning_counts)):
        failures.append('MuJoCo invalid-state warning')
    turns = float(data.qpos[crank_q] - crank0) / (2. * math.pi)
    distance = float(data.qpos[sim.root_x_qposadr]) - x0
    if data.time + dt / 2. < duration_s:
        failures.append('requested interval not completed')
    if minimum_speed < -.05:
        failures.append('bike rolled backwards')
    if turns < 1. or pedal_work <= 1.:
        failures.append('no sustained physical pedaling')
    if distance <= .25:
        failures.append('no forward progress')
    if any(load < -1e-8 for load in minimum_load.values()):
        failures.append('negative unilateral tire load')
    # A zero front load is deliberately allowed: actual wheel lift must remain
    # observable for the future anti-wheelie research, not be suppressed here.
    result = {
        'passed': not failures,
        'failures': failures,
        'track': track,
        'config': str(config),
        'initial_speed_mps': initial_speed_mps,
        'simulated_s': float(data.time),
        'distance_m': distance,
        'final_x_m': float(data.qpos[sim.root_x_qposadr]),
        'final_speed_mps': float(sim.speed_mps),
        'minimum_speed_after_0_5_s_mps': minimum_speed if math.isfinite(minimum_speed) else None,
        'crank_turns': turns,
        'measured_pedal_work_j': pedal_work,
        'maximum_sampled_sole_gap_m': maximum_gap,
        'minimum_tire_load_n': {side: load if math.isfinite(load) else None
                                for side, load in minimum_load.items()},
        'equilibrium_residual_qacc': float(sim.equilibrium['residual_qacc']),
        'energy_accounting': 'not evaluated (physical preview)',
    }
    # Preserve a readable failure report even after a non-finite state.
    return {key: None if isinstance(value, float) and not math.isfinite(value) else value
            for key, value in result.items()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track', choices=('uphill', 'rough_uphill'), default='uphill')
    parser.add_argument('--duration', type=float, default=5., help='simulated seconds')
    parser.add_argument('--initial-speed-mps', type=float, default=3.)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).with_name('viewer_physics_fast.toml'))
    args = parser.parse_args()
    try:
        result = check(args.track, args.duration, args.initial_speed_mps, args.config)
    except (OSError, ValueError, RuntimeError, FloatingPointError) as exc:
        print(json.dumps({'passed': False, 'error': str(exc)}, indent=2))
        return 1
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
