"""Non-strict headless reproduction of a viewer run for diagnosis."""
import sys
import numpy as np

from bike_sim.cli.ride import parse_args, resolve_rider, resolve_track
from bike_sim.sim.ride.physical_session import build_physical_simulation
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder

argv = [
    "--physics-config", "examples/research/viewer_physics_welded.toml",
    "--track", "examples/research/rough_uphill_savage.toml",
    "--headless", "--duration", "14", "--no-plots",
] + sys.argv[1:]
args = parse_args(argv)
rider = resolve_rider(args)
track = resolve_track(args.track, seed=None, length_m=args.length)
sim = build_physical_simulation(track, args, rider)
sim.physical.set_strict(False)
recorder = PhysicalRecorder(sim, 8)
dt = float(sim.model.opt.timestep)
n = int(14.0 / dt)
for i in range(n):
    sim.step()
    recorder.record(sim)
    if sim.crash is not None:
        print("CRASH", sim.crash)
        break
sim.physical.flush()
recorder.record(sim)
cols = recorder.columns()
print("time:", cols.get("time_s", np.array([0]))[-1])
np.savez("/tmp/diag_telemetry.npz", **cols)
print("channels:", len(cols))
for k in sorted(cols):
    print(" ", k)
