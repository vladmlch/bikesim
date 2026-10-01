"""Per-step wall-time benchmark for the viewer (_step_preview) path.

Runs the same headless preview steps under several physics configurations to
isolate what the recent rider changes cost: anatomical joint envelope, saddle
weld, grip connect weld. Reports ms/step split between apply_forces (Python
force pipeline + IK + contacts) and mj_step (solver), plus constraint stats.
"""
import dataclasses
import sys
import time

import mujoco
import numpy as np

from bike_sim.physics.resolution import load_physics_config
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain.trackfile import load_track

PROFILE = 'examples/research/viewer_physics_welded.toml'
TRACK = 'examples/research/rough_uphill.toml'
SIM_SECONDS = 4.


def variants(base):
    art = base.articulated
    yield 'pre(all flat, no envelope)', dataclasses.replace(
        base, articulated=dataclasses.replace(
            art, pedal_attachment='flat', saddle_attachment='flat',
            grip_attachment='spring', joint_envelope_path=None))
    yield '+envelope', dataclasses.replace(
        base, articulated=dataclasses.replace(
            art, pedal_attachment='flat', saddle_attachment='flat',
            grip_attachment='spring'))
    yield '+saddle weld', dataclasses.replace(
        base, articulated=dataclasses.replace(art, grip_attachment='spring'))
    yield 'current(all weld+env)', base


def bench(cfg, track, seconds):
    sim = RideSimulation(track=track, rider='articulated_planar',
                         physics_config=cfg)
    m, d = sim.model, sim.data
    phys = sim.physical
    t_apply = 0.0
    t_step = 0.0
    orig_apply = phys.apply_forces
    orig_mj_step = mujoco.mj_step

    def timed_apply(**kw):
        nonlocal t_apply
        t0 = time.perf_counter()
        try:
            return orig_apply(**kw)
        finally:
            t_apply += time.perf_counter() - t0

    def timed_step(mm, dd):
        nonlocal t_step
        t0 = time.perf_counter()
        try:
            return orig_mj_step(mm, dd)
        finally:
            t_step += time.perf_counter() - t0

    phys.apply_forces = timed_apply
    mujoco.mj_step = timed_step
    nefc, niter, warm = [], [], 0
    try:
        with phys.preview_mode():
            # Settle briefly so startup transients do not skew the sample.
            for _ in range(int(0.5 / m.opt.timestep)):
                phys.step()
            n = int(seconds / m.opt.timestep)
            t_all = time.perf_counter()
            for _ in range(n):
                phys.step()
                nefc.append(d.nefc)
                niter.append(int(d.solver_niter[0]))
            t_total = time.perf_counter() - t_all
    finally:
        phys.apply_forces = orig_apply
        mujoco.mj_step = orig_mj_step
    return {
        'ms_per_step': 1e3 * t_total / n,
        'ms_apply': 1e3 * t_apply / n,
        'ms_mjstep': 1e3 * t_step / n,
        'rtf': n * m.opt.timestep / t_total,
        'nefc_mean': float(np.mean(nefc)),
        'niter_mean': float(np.mean(niter)),
        'niter_max': int(np.max(niter)),
    }


def main():
    track = load_track(TRACK)
    base = load_physics_config(PROFILE)
    print(f"{'config':30} {'ms/step':>8} {'apply':>7} {'mjstep':>7} "
          f"{'rtf':>6} {'nefc':>6} {'niter':>6} {'maxit':>5}")
    for name, cfg in variants(base):
        r = bench(cfg, track, SIM_SECONDS)
        print(f"{name:30} {r['ms_per_step']:8.3f} {r['ms_apply']:7.3f} "
              f"{r['ms_mjstep']:7.3f} {r['rtf']:6.2f} {r['nefc_mean']:6.1f} "
              f"{r['niter_mean']:6.1f} {r['niter_max']:5d}", flush=True)


if __name__ == '__main__':
    sys.exit(main())
