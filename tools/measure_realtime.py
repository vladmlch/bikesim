"""Measure the common accounted physical path; startup and final flush are explicit."""
import argparse
import json
from math import isfinite
from pathlib import Path
import platform
import subprocess
import time

import numpy as np


def measure_realtime(track_path, physics_path, duration_s, *, out, dt_s=.0005):
    if not isfinite(duration_s) or duration_s <= 0.:
        raise ValueError('duration must be finite and positive')
    started=time.perf_counter()
    from bike_sim.cli import ride as ride_cli
    from bike_sim.physics.resolution import load_physics_config
    args=ride_cli.parse_args([
        '--track',str(track_path),'--physics-config',str(physics_path),'--headless',
        '--no-plots','--duration',str(duration_s),'--decimate','80',
        '--timestep',str(dt_s),'--out',str(out)])
    source_cfg=load_physics_config(physics_path)
    sim=ride_cli.build_physical_simulation_from_args(args)
    # Builder-only timing must still choose the real diagnostic demand; a
    # recorder would otherwise be the only owner setting this decimation.
    sim.physical.set_record_decimation(args.decimate)
    sim.physical.reference_monitor.strict=False
    startup=time.perf_counter()-started
    dt=float(sim.model.opt.timestep)
    steps=max(1,int(round(duration_s/dt)))
    costs=np.empty(steps)
    reason='duration'
    measured=0
    for i in range(steps):
        tick=time.perf_counter()
        sim.step()
        costs[i]=time.perf_counter()-tick
        measured=i+1
        if sim.crash is not None:
            reason='crash';break
        if sim.position_m >= sim.track.length_m:
            reason='finish';break
    costs=costs[:measured]
    tick=time.perf_counter()
    sim.physical.flush()
    flush_wall=time.perf_counter()-tick
    wall=float(costs.sum())+flush_wall
    root=Path(__file__).resolve().parents[1]
    git=subprocess.run(['git','rev-parse','HEAD'],cwd=root,capture_output=True,text=True,check=True)
    status=subprocess.run(['git','status','--porcelain','--','src/bike_sim','tools/measure_realtime.py'],
                          cwd=root,capture_output=True,text=True,check=True)
    from bike_sim.validation.environment import source_fingerprint
    model_status=sim.physical.model_status.as_dict()
    report={'steps':int(costs.size),'sim_seconds':float(costs.size*dt),
        'wall_seconds':wall,'startup_seconds':startup,'flush_wall_seconds':flush_wall,
        'factor':float(costs.size*dt/wall),'step_ms_mean':float(costs.mean()*1e3),
        'step_ms_p99':float(np.percentile(costs,99)*1e3),'git_sha':git.stdout.strip(),
        'machine':platform.platform(),'track':str(track_path),'physics':str(physics_path),
        'working_tree_dirty':bool(status.stdout.strip()),
        'source_sha256':source_fingerprint(root/'src/bike_sim'),
        'requested_duration_s':float(duration_s),'source_timestep_s':source_cfg.timestep_s,
        'effective_timestep_s':dt,'controller_interval_s':sim.physical.control_clock.period_s,
        'record_decimation':args.decimate,'monitor_strict':False,'measurement_path':'accounted_headless',
        'reason':reason,'model_valid':model_status['model_valid'],
        'numerically_valid':model_status.get('numerically_valid','not_evaluated'),
        'model_status':model_status,
        'first_failure':sim.physical.reference_monitor.first_failure}
    # Use JSON's canonical lists for first_failure in both return and artifact.
    report=json.loads(json.dumps(report,allow_nan=False))
    destination=Path(out);destination.mkdir(parents=True,exist_ok=True)
    (destination/'realtime.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track',required=True)
    parser.add_argument('--physics-config',required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--dt',type=float,default=.0005,
                        help='explicit physical timestep; v2 default .0005 (profile remains unchanged)')
    parser.add_argument('--out',default='output/realtime')
    args=parser.parse_args()
    print(json.dumps(measure_realtime(args.track,args.physics_config,args.duration,
                                     out=Path(args.out),dt_s=args.dt),indent=2))
