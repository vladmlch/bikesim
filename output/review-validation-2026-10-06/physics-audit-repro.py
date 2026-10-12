import sys, copy, pathlib, math, ctypes
sys.path[:0]=['.','native/build','tests/reference']
import bike_native, mujoco, numpy as np
from bike_sim.sim.ride_sim import RideSimulation
from tools.native_config import project
from bike_sim.physics.air_spring import ForkAirSpring, AirSpringSpecs
from bike_sim.physics.damper import Charger3Damper
from test_native_tire import _tire_cfg
sim=RideSimulation()
path='/private/tmp/physics-audit.mjb'
mujoco.mj_saveModel(sim.model,path)
cfg=project(sim)
sus={'schema':1,'suspension':cfg['suspension']}
def outcome(fn):
 try:
  val=fn(); return str(val)
 except Exception as e: return f'{type(e).__name__}: {e}'
print('fork legacy 150 ctor Python:',outcome(lambda: Charger3Damper(total_travel_mm=150.,legacy_behavior=True).hbo_start_mm))
c=copy.deepcopy(sus); c['suspension']['fork_damper']['total_travel_mm']=150.
print('fork legacy 150 native ctor:',outcome(lambda: bike_native.Stepper(path,c)))
c=copy.deepcopy(sus); c['suspension']['air_spring']['neg_chamber_length_mm']=-5.
st=bike_native.Stepper(path,c)
q=sim.data.qpos.copy(); q[sim.applier.fork_qposadr]=.010
st.set_state(q,np.zeros(sim.model.nv),np.zeros(sim.model.na),np.zeros(sim.model.nv),0.)
print('negative chamber Python:',outcome(lambda: ForkAirSpring(AirSpringSpecs(neg_chamber_length_mm=-5.)).compute_axial_force(10.)))
print('negative chamber native fork spring:',outcome(lambda: st.suspension_components()['fork_spring'][sim.applier.fork_dofadr]))
c=copy.deepcopy(sus); c['suspension']['fork_damper']['max_lsc']=0
st=bike_native.Stepper(path,c)
q=sim.data.qpos.copy(); v=np.zeros(sim.model.nv); v[sim.applier.fork_dofadr]=.1
st.set_state(q,v,np.zeros(sim.model.na),np.zeros(sim.model.nv),0.)
damper=Charger3Damper(); damper.max_lsc=0; damper.lsc_clicks=0
print('zero max_lsc Python:',outcome(lambda: damper.compute_damping_force(.1)))
print('zero max_lsc native fork damper:',outcome(lambda: st.suspension_components()['fork_damper'][sim.applier.fork_dofadr]))
st=bike_native.Stepper(path,{'schema':1,'brake':{'torque_ceiling_nm':float('nan'),'taper_radps':.5}})
st.set_state(sim.data.qpos,np.ones(sim.model.nv),np.zeros(sim.model.na),np.zeros(sim.model.nv),0.)
print('brake NaN ctor accepted; torques:',st.brake_torques(1.,1.))
p={'joint':'root_x','stiffness_n_m':-1.,'damping_ns_m':-1.,'preload_deflection_m':1.,'offset_m':0.,'unilateral':False}
st=bike_native.Stepper(path,{'schema':1,'rider_forces':{'paths':[p]}})
print('negative rider stiffness ctor accepted; finite qfrc:',np.isfinite(st.rider_forces_qfrc()).all())
# tire public set_state NaN time and integer size behavior
path='tools/proto_native_bench/artifacts/model.mjb'
m=mujoco.MjModel.from_binary_path(path); d=mujoco.MjData(m)
d.qpos[0]+=.6; d.qpos[14]+=.6; mujoco.mj_forward(m,d)

for t in [float('nan'),1e20,2.5*.00125]:
 st=bike_native.Stepper(path,_tire_cfg())
 st.set_state(d.qpos,d.qvel,d.act,np.zeros(m.nv),t); st.forward()
 print('tire time',t,':',outcome(lambda: (st.tire_qfrc(.00125), st.tire_snapshots()['front']['interval_id'])[1]))
# nearbyint rounding depends on active FENV unlike Python round
lib=ctypes.CDLL(None)
for mode in [0,0x400000]:
 print('fesetround',mode,lib.fesetround(mode))
 st=bike_native.Stepper(path,_tire_cfg()); st.set_state(d.qpos,d.qvel,d.act,np.zeros(m.nv),2.5*.00125); st.forward()
 print('native/Python interval',outcome(lambda:(st.tire_qfrc(.00125),st.tire_snapshots()['front']['interval_id'])[1]),round(2.5))
lib.fesetround(0)
