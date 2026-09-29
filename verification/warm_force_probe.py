"""Diagnostic continuation; not an acceptance substitute for a fresh constructor."""
import dataclasses, pickle, time, json
from pathlib import Path
import mujoco
from force_refine import refine_equilibrium
from bike_sim.sim.ride.physical_observations import energy_state

start=time.monotonic()
with open('/mnt/data/articulated-initial-0.0005.pkl','rb') as f: sim=pickle.load(f)
r=sim.physical;m,d=sim.model,sim.data
cfg=dataclasses.replace(sim.physics_config,articulated=dataclasses.replace(sim.physics_config.articulated,support_k_n_m=100000.))
sim.physics_config=cfg;r.cfg=cfg;r.rider_contacts.config=cfg.articulated;r.rider_control.config=cfg.articulated
for i in range(8):
    residual=refine_equilibrium(r,max_evaluations=60)
    print('REFINE',i,residual,'wall',time.monotonic()-start,flush=True)
    if residual<=.05:break
print('WORST',[(m.joint(int(m.dof_jntid[i])).name,float(d.qacc[i])) for i in __import__('numpy').argsort(abs(d.qacc))[-5:]],flush=True)
with open('/mnt/data/force-continuation-debug.pkl','wb') as f:pickle.dump(sim,f)
if residual>.05: raise RuntimeError('diagnostic continuation not equilibrated')
d.time=0.;d.qvel.fill(0.)
r.rider_contacts.restart_clock();r.rider_contacts.initialize_settled_state(m,d)
r.tire.restart_clock() if hasattr(r.tire,'restart_clock') else None
r.apply_forces(active=False,advance=False)
mass,elastic,total=energy_state(r);r.initial_energy_j=total;r.energy_scale_j=max(1.,mass['kinetic_energy_j']+sum(elastic.values()))
sim.equilibrium['residual_qacc']=residual
with open('/mnt/data/articulated-stiff-continuation.pkl','wb') as f:pickle.dump(sim,f)
print('READY',r.energy_scale_j,flush=True)
