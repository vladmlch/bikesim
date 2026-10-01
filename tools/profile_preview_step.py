"""cProfile the viewer _step_preview path for two physics configs."""
import cProfile
import dataclasses
import pstats
import sys
import io

from bike_sim.physics.resolution import load_physics_config
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain.trackfile import load_track

PROFILE = 'examples/research/viewer_physics_welded.toml'
TRACK = 'examples/research/rough_uphill.toml'
SIM_SECONDS = 4.


def run(cfg, track):
    sim = RideSimulation(track=track, rider='articulated_planar',
                         physics_config=cfg)
    phys = sim.physical
    with phys.preview_mode():
        for _ in range(int(0.5 / sim.model.opt.timestep)):
            phys.step()
        pr = cProfile.Profile()
        pr.enable()
        for _ in range(int(SIM_SECONDS / sim.model.opt.timestep)):
            phys.step()
        pr.disable()
    buf = io.StringIO()
    pstats.Stats(pr, stream=buf).sort_stats('cumulative').print_stats(30)
    return buf.getvalue()


def main():
    track = load_track(TRACK)
    base = load_physics_config(PROFILE)
    pre = dataclasses.replace(
        base, articulated=dataclasses.replace(
            base.articulated, pedal_attachment='flat',
            saddle_attachment='flat', grip_attachment='spring',
            joint_envelope_path=None))
    print('=========== PRE (flat, no envelope) ===========')
    print(run(pre, track))
    print('=========== CURRENT (all weld + envelope) ===========')
    print(run(base, track))


if __name__ == '__main__':
    sys.exit(main())
