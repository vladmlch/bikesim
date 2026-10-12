import sys, tempfile, copy, gc
from pathlib import Path
sys.path.insert(0, str(Path.cwd() / 'tests/reference'))
from test_native_drivetrain import pair, bike_native, model_xml
from test_native_drive_policies import oracle_config
from bike_sim.physics.physical_config import PedalingConfig, ShiftingConfig
import mujoco
import numpy as np

tmp=Path(tempfile.mkdtemp(prefix='drive-audit-'))
m,d,p,n=pair(tmp)
for prop in ('qfrc_applied','actuator_force'):
    refs=sys.getrefcount(n); a=getattr(n,prop)
    print('owner retention',prop,sys.getrefcount(n)-refs,'base',type(a.base))
    del a
pol=bike_native.DrivePolicies(oracle_config(p))
for val in (1,0,'true',[],None,np.bool_(True)):
    try: pol.assist_step(20.,80.,2.,val,.01); print('bool accepted',repr(val))
    except Exception as e: print('bool rejected',repr(val),type(e).__name__)
for where in ('control','last'):
    try:
        if where=='control': n.drive_components({1:0},.01,2.)
        else:
            state=n.drive_state(); state['last'][1]=0; n.set_drive_state(state)
    except Exception as e: print('nonstring',where,type(e).__name__,str(e))
before=n.drive_state()['ideal_hub']
after=n._drive_core('unknown','ratio',34/28)
print('unknown core kind accepted',after['ratio'],'live ratio',n.drive_state()['ideal_hub']['ratio'],'live coefficient',n.drive_state()['ideal_hub']['coefficients'])

m,d,p,n=pair(tmp,pedaling=PedalingConfig(effort_slew_nm_s=100))
before=n.drive_state()
try: n.drive_prepare_pedaling({},.01,effort_ceiling_nm=-1.)
except Exception as e: print('bad ceiling',type(e).__name__,'effort before/after',before['policies']['pedaling']['_effort'],n.drive_state()['policies']['pedaling']['_effort'])
n.drive_components({},.01,2.)
d.time=.1; n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward()
for i in range(2):
    before=n.drive_state()
    try: n.drive_components({},.01,2.)
    except Exception as e: print('unsettled attempt',i,type(e).__name__,'effort before/after',before['policies']['pedaling']['_effort'],n.drive_state()['policies']['pedaling']['_effort'])

Path('/private/tmp/drive-model.xml').write_text(model_xml('geometric_ideal_mid_drive'))
Path('/private/tmp/drive-ideal.xml').write_text(model_xml('ideal_mid_drive'))
for field in ('jacobian','qpos'):
    m,d,p,n=pair(tmp,'geometric_ideal_mid_drive')
    s=n.drive_state();s['ideal_hub']['prepared'][field]=[True]*m.nv
    try: n.set_drive_state(s); print('bool vector accepted',field,n.drive_state()['ideal_hub']['prepared'][field][0])
    except Exception as e: print('bool vector rejected',field,type(e).__name__)

m,d,p,n=pair(tmp,'geometric_ideal_mid_drive',shifting=ShiftingConfig(enabled=True,cassette=(24,1000),cadence_smoothing_tau_s=0.,target_cadence_min_rpm=100.,target_cadence_max_rpm=1000.))
d.qvel[m.joint('crank_spin').dofadr[0]]=4.;d.qvel[m.joint('rear_wheel_spin').dofadr[0]]=1.
n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward()
before=n.drive_state()
try: n.drive_prepare_pedaling({},.1)
except Exception as e:
    after=n.drive_state();print('geometric failed shift',type(e).__name__,str(e),'ratio',before['ideal_hub']['ratio'],after['ideal_hub']['ratio'],'rear',after['ideal_hub']['rear_teeth'],'boundary unchanged',before['ideal_hub']['boundary']==after['ideal_hub']['boundary'],'prepared phi unchanged',before['ideal_hub']['prepared']['phi']==after['ideal_hub']['prepared']['phi'])
