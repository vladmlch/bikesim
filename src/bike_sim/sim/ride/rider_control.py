"""Bounded internal rider actuation; no root force or direct human crank torque."""
from dataclasses import dataclass
from math import atan2, cos, pi, sin
import numpy as np
from bike_sim.physics.checks import array, scalar


def stance_force(phase,mean_nm,crank_m):
    phase = scalar(phase,'pedal phase')
    mean_nm = scalar(mean_nm,'requested mean torque',minimum=0)
    crank_m = scalar(crank_m,'crank length',positive=True)
    weight = max(cos(phase),0.)*pi/2
    return (mean_nm/crank_m)*weight*np.array([-sin(phase),0.,-cos(phase)])


def bounded_joint_torque(q,qd,target,kp,kd,limit):
    q,qd,target = (array(v,n) for v,n in zip((q,qd,target),('joint position','joint speed','joint target')))
    if q.ndim != 1 or q.shape != qd.shape or q.shape != target.shape:
        raise ValueError('invalid joint state shapes')
    kp,kd,limit = (scalar(v,n,minimum=0) for v,n in zip((kp,kd,limit),('joint kp','joint kd','joint limit')))
    value = kp*(target-q)-kd*qd
    if not np.isfinite(value).all():
        raise ValueError('joint controller overflow')
    return np.clip(value,-limit,limit)


def bounded_effort(qd, requested, torque_limit, speed_limit, power_limit):
    """Limit the final internal effort, retaining braking above the speed bound."""
    qd=array(qd,'joint speed')
    torque=array(requested,'requested joint torque')
    if qd.shape!=torque.shape or qd.ndim!=1:
        raise ValueError('effort state shape mismatch')
    torque_limit=scalar(torque_limit,'joint torque ceiling',minimum=0)
    speed_limit=scalar(speed_limit,'joint speed ceiling',positive=True)
    power_limit=scalar(power_limit,'joint power ceiling',minimum=0)
    result=np.clip(torque,-torque_limit,torque_limit)
    accelerating=result*qd>0
    result[accelerating & (np.abs(qd)>=speed_limit)]=0.
    accelerating=result*qd>0
    cap=np.full_like(qd,torque_limit)
    moving=np.abs(qd)>0
    cap[moving]=np.minimum(cap[moving],power_limit/np.abs(qd[moving]))
    result[accelerating]=np.clip(result[accelerating],-cap[accelerating],cap[accelerating])
    return result


def two_link_ik(target_xz,upper_m,lower_m,*,elbow_sign=1):
    """Geometric CCW angles plus explicit unreachable-target saturation status."""
    p = array(target_xz,'IK target',(2,))
    a,b = scalar(upper_m,'upper link',positive=True),scalar(lower_m,'lower link',positive=True)
    if elbow_sign not in (-1,1):
        raise ValueError('IK branch must be -1 or +1')
    distance = float(np.linalg.norm(p))
    lo,hi = abs(a-b)+1e-10,a+b-1e-10
    if hi <= lo:
        raise ValueError('IK links are too small')
    saturated = not lo <= distance <= hi
    d = float(np.clip(distance,lo,hi))
    direction = atan2(p[1],p[0]) if distance > 1e-15 else -pi/2
    cosine = np.clip((d*d-a*a-b*b)/(2*a*b),-1.,1.)
    knee = elbow_sign*float(np.arccos(cosine))
    hip = direction-atan2(b*sin(knee),a+b*cos(knee))
    return np.array([hip,knee]),saturated


@dataclass(frozen=True)
class RiderCommand:
    mean_crank_torque_nm: float = 0.
    enabled: bool = True

    def __post_init__(self):
        scalar(self.mean_crank_torque_nm,'rider effort',minimum=0)
        if not isinstance(self.enabled,bool):
            raise ValueError('rider controller enable must be a bool')


class ArticulatedRiderController:
    """Foot-force feedforward plus PD, clipped AFTER summation on limb DOFs only."""
    def __init__(self,model,pose,config,crank_length_m):
        import mujoco
        self.pose,self.config = pose,config
        self.crank_length_m = scalar(crank_length_m,'crank length',positive=True)
        self.enabled = True
        self.last_terms = {}
        self.saturated_ik = {'front':False,'rear':False}
        self.joints = {}
        for name in ('rider_torso_hinge','rider_shoulder','rider_elbow') + tuple(
            f'rider_{joint}_{side}' for side in ('front','rear') for joint in ('hip','knee','ankle')):
            jid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
            aid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'act_'+name)
            if jid < 0 or aid < 0:
                raise ValueError(f'missing articulated joint/actuator {name}')
            self.joints[name] = (int(model.jnt_qposadr[jid]),int(model.jnt_dofadr[jid]),aid)
        self.pelvis = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rider_pelvis')
        self.feet = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'rider_foot_{s}') for s in ('front','rear')}
        self.soles = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,f'site_rider_sole_{s}') for s in ('front','rear')}
        self.pedals = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,f'site_pedal_{s}') for s in ('front','rear')}
        self.crank = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'crank')
        if min(self.pelvis,self.crank,*self.feet.values(),*self.soles.values(),*self.pedals.values()) < 0:
            raise ValueError('incomplete rider interface topology')

    def _targets(self,model,data,side):
        hip = data.xpos[self.pelvis]
        R = data.xmat[self.pelvis].reshape(3,3)
        ankle_offset = getattr(self.pose,f'ankle_{side}')-getattr(self.pose,f'pedal_{side}')
        pedal = data.site_xpos[self.pedals[side]]
        target = R.T @ (pedal + ankle_offset-hip)
        upper = getattr(self.pose,f'knee_{side}')-self.pose.hip
        lower = getattr(self.pose,f'ankle_{side}')-getattr(self.pose,f'knee_{side}')
        a0,b0 = atan2(upper[2],upper[0]),atan2(lower[2],lower[0])
        branch = 1 if sin(b0-a0) >= 0 else -1
        angles,saturated = two_link_ik(target[[0,2]],np.linalg.norm(upper),np.linalg.norm(lower),elbow_sign=branch)
        self.saturated_ik[side] = saturated
        # MuJoCo +Y is clockwise in X-Z; the pure IK helper uses CCW.
        hip_q = a0-angles[0]
        knee_q = (b0-a0)-angles[1]
        pitch = atan2(-R[2,0],R[0,0])
        return np.array([hip_q,knee_q,-pitch-hip_q-knee_q])

    def initialize(self,model,data):
        """Initial-condition setup only, before static equilibrium, never in step()."""
        import mujoco
        mujoco.mj_forward(model,data)
        for side in ('front','rear'):
            targets = self._targets(model,data,side)
            if self.saturated_ik[side]:
                raise ValueError(f'initial {side} pedal is unreachable')
            for joint,value in zip(('hip','knee','ankle'),targets):
                qa,_,_ = self.joints[f'rider_{joint}_{side}']
                data.qpos[qa] = value
        mujoco.mj_forward(model,data)

    def compute(self,model,data,command):
        import mujoco
        if not isinstance(command,RiderCommand):
            raise ValueError('expected a RiderCommand')
        if not command.enabled or not self.enabled:
            self.last_terms = {name:{'posture_nm':0.,'pedaling_nm':0.,'command_nm':0.,'saturated':False} for name in self.joints}
            return {name:0. for name in self.joints}
        cfg = self.config
        result = {}
        terms = {}
        for name,(qa,va,_) in self.joints.items():
            if name.startswith(('rider_hip_','rider_knee_','rider_ankle_')):
                continue
            result[name] = float(bounded_joint_torque(
                [data.qpos[qa]],[data.qvel[va]],[0.],cfg.joint_kp_nm_rad,
                cfg.joint_kd_nms_rad,cfg.joint_limit_nm)[0])
            raw = cfg.joint_kp_nm_rad*(0.-data.qpos[qa])-cfg.joint_kd_nms_rad*data.qvel[va]
            terms[name] = (float(raw), 0.)
        rotation = data.xmat[self.crank].reshape(3,3)
        phase = atan2(-rotation[2,0],rotation[0,0])
        for side,offset in (('front',0.),('rear',pi)):
            names = [f'rider_{joint}_{side}' for joint in ('hip','knee','ankle')]
            qa = [self.joints[n][0] for n in names]
            va = [self.joints[n][1] for n in names]
            target = self._targets(model,data,side)
            pd = cfg.joint_kp_nm_rad*(target-data.qpos[qa])-cfg.joint_kd_nms_rad*data.qvel[va]
            jp,jr = np.zeros((3,model.nv)),np.zeros((3,model.nv))
            mujoco.mj_jac(model,data,jp,jr,data.site_xpos[self.soles[side]],self.feet[side])
            requested = stance_force(phase+offset,command.mean_crank_torque_nm,self.crank_length_m)
            feedforward = jp[:,va].T@requested
            torque = pd+feedforward
            terms.update({n:(float(p),float(f)) for n,p,f in zip(names,pd,feedforward)})
            if not np.isfinite(torque).all():
                raise ValueError('non-finite articulated command')
            result.update(zip(names,map(float,torque)))
        self.last_terms = {}
        for name, (posture, pedaling) in terms.items():
            _, va, _ = self.joints[name]
            command_torque = float(bounded_effort([data.qvel[va]],[posture+pedaling],
                cfg.joint_limit_nm,cfg.joint_speed_limit_rad_s,cfg.joint_power_limit_w)[0])
            scale = command_torque/(posture+pedaling) if posture+pedaling != 0 else 0.
            result[name] = command_torque
            self.last_terms[name] = {'posture_nm':posture*scale, 'pedaling_nm':pedaling*scale,
                                    'command_nm':command_torque, 'saturated':scale!=1.}
        return result

    def write(self,data,torques):
        if set(torques) != set(self.joints):
            raise ValueError('incomplete rider actuator command')
        values = {name:scalar(torque,'rider torque') for name,torque in torques.items()}
        for name,torque in values.items():
            data.ctrl[self.joints[name][2]] = torque
