"""Targeted trace around loss of pedal tracking; not a calibration experiment."""
import pickle,json
import numpy as np
from bike_sim.sim.ride.physical_samples import plain
with open('/mnt/data/articulated-stiff-continuation.pkl','rb') as f:s=pickle.load(f)
r=s.physical;m,d=s.model,s.data;c=r.rider_control
original=c._targets;targets={}
def trace_target(model,data,side,**kw):
    value=original(model,data,side,**kw)
    if data is d:targets[side]={'q':value.tolist(),**kw}
    return value
c._targets=trace_target
qcr=r.drive.joints['crank_spin'][0];dt=m.opt.timestep
with open('verification/pedal-detail.jsonl','w') as out:
    for i in range(round(3./dt)):
        s.step()
        if i%round(.01/dt)==0:
            sample=r.sample
            joints={name:{'q':float(sample.qpos[qa]),'v':float(sample.qvel[va]),**c.last_terms[name]}
                    for name,(qa,va,_) in c.joints.items()}
            row={'time':s.time_s,'crank':float(d.qpos[qcr]),'drive':r.drive.last,
                 'terms':joints,'targets':targets,'support':c.support_diagnostics,
                 'contacts':r.rider_contacts.diagnostics,'energy':r.energy}
            row['crank_force_components']={n:float(f[qcr]) for n,f in sample.force_components.items()} if hasattr(sample,'force_components') else {}
            out.write(json.dumps(plain(row),allow_nan=False)+'\n')
        if i in (round(2./dt),round(2.2/dt),round(2.4/dt)):
            del c.__dict__['_targets']
            with open(f'/mnt/data/pedal-state-{i}.pkl','wb') as f:pickle.dump(s,f)
            c._targets=trace_target
        if i%1000==0:print(s.time_s,float(d.qpos[qcr]),r.drive.last['chain_tension_n'],flush=True)
