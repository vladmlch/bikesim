import sys,copy,ctypes
sys.path[:0]=['.','native/build','tests/reference']
import bike_native,mujoco,numpy as np
from test_native_tire import _tire_cfg
from bike_sim.terrain.surface import SurfaceSpec
from bike_sim.physics.physical_config import TireBackendConfig,TireParameters
from bike_sim.physics.tire import TireSpec
from bike_sim.sim.ride.tire_forces import TireForceApplier,compiled_profile_vertices
path='tools/proto_native_bench/artifacts/model.mjb'
def outcome(fn):
 try: return str(fn())
 except Exception as e: return f'{type(e).__name__}: {e}'
surface=dict(name='x',mu_peak=.8,mu_slide=.5,slip_stiffness_per_load=10.,stribeck_speed_mps=float('inf'))
print('surface inf Python:',outcome(lambda:SurfaceSpec(**surface)))
c=_tire_cfg();c['tire']['surface_mode']='track';c['tire']['surface_map']=dict(surface=surface,sections=[])
print('surface inf native:',outcome(lambda:bike_native.Stepper(path,c)))
m=mujoco.MjModel.from_binary_path(path);d=mujoco.MjData(m)
d.qpos[0]+=.6; d.qpos[14]+=.6; mujoco.mj_forward(m,d)
mat=TireSpec(130000.,800.,200000.,'synthetic',(0.,2000.))
param=TireParameters(mat)
cfg=TireBackendConfig(backend='compliant_2d',front=param,rear=param)
vertices=compiled_profile_vertices(m,d)
for t in [float('nan'),1e20]:
 d.time=t
 ap=TireForceApplier(m,vertices,cfg)
 print('Python tire time',t,outcome(lambda: (ap.compute_qfrc(m,d,.00125),ap.snapshots['front'].interval_id)[1]))
# Valid model scalar names accepting hinge longitudinal root.
m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="root_x" type="hinge"/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
mujoco.mj_saveModel(m,'/private/tmp/physics-cruise-hinge.mjb')
print('hinge cruise native',outcome(lambda:bike_native.Stepper('/private/tmp/physics-cruise-hinge.mjb',{'schema':1,'cruise':dict(target_speed_kmh=25.,kp_nm_per_mps=1.,ki_nm_per_mps_s=1.,torque_ceiling_nm=150.)})))
c=_tire_cfg();c['tire']['front']['material']['radial_k_n_m']=True
print('bool TireSpec Python',outcome(lambda:TireSpec(True,800.,200000.,'synthetic',(0.,2000.))))
print('bool TireSpec native',outcome(lambda:bike_native.Stepper(path,c)))
