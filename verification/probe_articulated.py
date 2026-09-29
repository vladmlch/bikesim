import json, time, sys, pickle
from pathlib import Path
import numpy as np
from bike_sim.validation.ride_cases import make_sim

def main():
    dt=float(sys.argv[1]) if len(sys.argv)>1 else .0005
    duration=float(sys.argv[2]) if len(sys.argv)>2 else 6.
    start=time.monotonic()
    # Final acceptance uses a fresh constructor; diagnostic cache use must be explicit.
    cache = Path(sys.argv[3]) if len(sys.argv)>3 else None
    if cache is not None:
        with cache.open('rb') as f: sim=pickle.load(f)
        print('DIAGNOSTIC: cached equilibrium, not fresh acceptance', flush=True)
    else:
        sim=make_sim(dt,rider='articulated_planar',drive='articulated_effort',human=20.)
        print('FRESH constructor completed',flush=True)

    print('equilibrium',json.dumps(sim.equilibrium),flush=True)
    m,d=sim.model,sim.data
    j=m.joint('crank_spin'); cq=int(j.qposadr[0]); cv=int(j.dofadr[0])
    min_crank=max_crank=float(d.qpos[cq]); max_residual=0.
    for i in range(round(duration/dt)):
        sim.step()
        min_crank=min(min_crank,float(d.qpos[cq]));max_crank=max(max_crank,float(d.qpos[cq]))
        rel=sim.physical.energy['residual_j']/sim.physical.energy_scale_j
        max_residual=max(max_residual,abs(rel))
        if i%round(.25/dt)==0:
            c=sim.physical.rider_contacts
            row={'t':sim.time_s,'x':sim.position_m,'v':sim.speed_mps,'crank':float(d.qpos[cq]),
                 'cadence':float(d.qvel[cv])*60/(2*np.pi),'residual_fraction':rel,
                 'loss_j':sim.physical.loss_j,'active_work_j':sim.physical.active_work_j,
                 'contacts':{name:{k:values[k] for k in ('normal_load_n','force_on_rider_n','gap_m') if k in values}
                             for name,values in c.diagnostics.items()},
                 'pedal_pitch':{side:float(d.qpos[m.joint(f'pedal_{side}_spin').qposadr[0]]) for side in ('front','rear')},
                 'crash':str(sim.crash)}
            print(json.dumps(row),flush=True)
        if sim.crash is not None:
            print('CRASH',sim.crash,flush=True)
            break
    print('FINAL',json.dumps({'time':sim.time_s,'min_crank':min_crank,'max_crank':max_crank,
        'max_residual_fraction':max_residual,'wall_s':time.monotonic()-start}),flush=True)

if __name__=='__main__':main()
