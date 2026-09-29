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
        self.model=model
        self.command_enabled=True
        self.target_data = mujoco.MjData(model)
        self.target_difference_s = 1e-6
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
        self.frame = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'frame')
        self.torso = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rider_torso')
        self.upper_arm = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rider_upper_arm_pair')
        self.forearm = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rider_forearm_pair')
        self.grip_site = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'site_rider_grip')
        self.rider_bodies = [b for b in range(model.nbody)
            if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,b) or '').startswith('rider_')]
        self.rider_mass = float(np.sum(model.body_mass[self.rider_bodies]))
        self.support_diagnostics = {}
        self.pitch_dofs = tuple(int(model.joint(name).dofadr[0])
            for name in ('root_pitch', 'rider_root_pitch'))
        self.crank = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'crank')
        self.crank_spin_dof=int(model.jnt_dofadr[model.joint('crank_spin').id])
        self.pedal_spin_dofs=[int(model.jnt_dofadr[model.joint(f'pedal_{side}_spin').id]) for side in ('front','rear')]
        if min(self.pelvis,self.crank,*self.feet.values(),*self.soles.values(),*self.pedals.values()) < 0:
            raise ValueError('incomplete rider interface topology')

    def _targets(self,model,data,side, *, compression_m=None, shear_m=0.):
        hip = data.xpos[self.pelvis]
        # Solve in the actual parent frame so the foot target reaches the
        # actual platform. Posture is requested through a balanced contact
        # wrench, not a fictitious root frame in the leg IK.
        R = data.xmat[self.pelvis].reshape(3,3)
        ankle_offset = getattr(self.pose,f'ankle_{side}')-getattr(self.pose,f'pedal_{side}')
        pedal = data.site_xpos[self.pedals[side]]
        depth=self.config.posture_sole_depth_m if compression_m is None else scalar(compression_m,'target sole compression',minimum=0)
        normal=data.site_xmat[self.pedals[side]].reshape(3,3)[:,2]
        tangent=data.site_xmat[self.pedals[side]].reshape(3,3)[:,0]
        shear=scalar(shear_m,'target sole shear')
        # Target the actual platform top, with a deflection consistent with the
        # requested normal load. A fixed 3 mm position goal in parallel with an
        # unrelated force goal makes the PD oppose the very load it must hold.
        ankle_goal=pedal+ankle_offset+normal*(.008-depth)-np.array([0.,0.,.008])+shear*tangent
        target = R.T@(ankle_goal-hip)
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
        targets=np.array([hip_q,knee_q,-pitch-hip_q-knee_q])
        current=np.array([data.qpos[self.joints[f'rider_{joint}_{side}'][0]] for joint in ('hip','knee','ankle')])
        # Equivalent +/-2*pi IK representations must not cause torque impulses.
        return current + np.arctan2(np.sin(targets-current),np.cos(targets-current))

    def _upper_targets(self, data):
        """Bounded joint goals keep the hands reachable as the pelvis moves.

        A fixed shoulder/elbow angle is not a bar-following controller. Select
        the upper (upright) torso branch with explicit elbow flexion reserve,
        then solve the actual arm chain to the bar. Only actuator targets are
        produced; no running root or joint coordinate is prescribed.
        """
        pose = self.pose
        pelvis_R = data.xmat[self.pelvis].reshape(3,3)
        hip = data.xpos[self.pelvis]
        grip = data.xpos[self.frame] + data.xmat[self.frame].reshape(3,3) @ pose.grip
        trunk = pose.shoulder-pose.hip
        upper, lower = pose.elbow-pose.shoulder, pose.grip-pose.elbow
        # Keep a neutral relative torso/pelvis angle. Counter-rotating the
        # torso against the unactuated pelvis pitch pushes the pelvis further
        # in that direction through the equal actuator reaction. The hands
        # follow the actual bar separately, through the two arm joints.
        target = pose.grip-pose.hip
        reach = self.config.arm_reach_fraction*(np.linalg.norm(upper)+np.linalg.norm(lower))
        candidates = [two_link_ik(target[[0,2]], np.linalg.norm(trunk), reach, elbow_sign=sign)
                      for sign in (-1,1)]
        angles, saturated = max(candidates, key=lambda pair: sin(pair[0][0]))
        neutral_q = atan2(trunk[2],trunk[0])-angles[0]
        torso_q = neutral_q
        torso_R = data.xmat[self.torso].reshape(3,3)
        arm_target = torso_R.T @ (grip-data.xpos[self.upper_arm])
        a0,b0 = atan2(upper[2],upper[0]),atan2(lower[2],lower[0])
        branch = 1 if sin(b0-a0)>=0 else -1
        arm_angles, arm_saturated = two_link_ik(arm_target[[0,2]],np.linalg.norm(upper),np.linalg.norm(lower),elbow_sign=branch)
        targets = {'rider_torso_hinge':torso_q,'rider_shoulder':a0-arm_angles[0],
                   'rider_elbow':(b0-a0)-arm_angles[1]}
        self.saturated_ik.update(torso=saturated,arms=arm_saturated)
        for name,target_q in targets.items():
            current = float(data.qpos[self.joints[name][0]])
            targets[name] = current+atan2(sin(target_q-current),cos(target_q-current))
        return targets

    def _predict_target_state(self,model,data):
        """Directional derivative of moving IK goals on detached kinematics.

        Damping against zero joint speed brakes intended pedaling. Estimate the
        goal speed induced by crank spin, not a prescribed cadence. Root bob/pitch
        and other uncommanded motions remain damped disturbances, not desired
        target velocities that would cancel the postural feedback.
        No forces, contacts, or controller states are advanced on the live model.
        """
        import mujoco
        future=self.target_data
        future.qpos[:]=data.qpos
        future.qvel.fill(0.)
        rate=data.qvel[self.crank_spin_dof]
        future.qvel[self.crank_spin_dof]=rate
        # Keep the platform attitude fixed while its spindle follows the crank.
        future.qvel[self.pedal_spin_dofs]=-rate
        mujoco.mj_integratePos(model,future.qpos,future.qvel,self.target_difference_s)
        mujoco.mj_kinematics(model,future)
        return future

    def initialize(self,model,data):
        """Initial-condition setup only, before static equilibrium, never in step()."""
        import mujoco
        mujoco.mj_forward(model,data)
        # Pose the torso first, then solve the arms about that actual shoulder.
        upper = self._upper_targets(data)
        data.qpos[self.joints['rider_torso_hinge'][0]] = upper['rider_torso_hinge']
        mujoco.mj_forward(model,data)
        for name,value in self._upper_targets(data).items():
            data.qpos[self.joints[name][0]] = value
        mujoco.mj_forward(model,data)
        for side in ('front','rear'):
            targets = self._targets(model,data,side)
            if self.saturated_ik[side]:
                raise ValueError(f'initial {side} pedal is unreachable')
            for joint,value in zip(('hip','knee','ankle'),targets):
                qa,_,_ = self.joints[f'rider_{joint}_{side}']
                data.qpos[qa] = value
        mujoco.mj_forward(model,data)

    def compute(self,model,data,command, *, contact_loads=None, support_available=None):
        import mujoco
        if not isinstance(command,RiderCommand):
            raise ValueError('expected a RiderCommand')
        self.command_enabled=command.enabled and self.enabled
        if not self.command_enabled:
            self.last_terms = {name:{'posture_nm':0.,'pedaling_nm':0.,'command_nm':0.,'saturated':False} for name in self.joints}
            return {name:0. for name in self.joints}
        cfg = self.config
        from bike_sim.sim.ride.rider_support import pedaling_support_targets
        support = np.zeros(model.nv)
        weight = self.rider_mass*float(np.linalg.norm(model.opt.gravity))
        com = np.sum(model.body_mass[self.rider_bodies,None]*data.xipos[self.rider_bodies],axis=0)/self.rider_mass
        saddle = data.xpos[self.pelvis]+data.xmat[self.pelvis].reshape(3,3)@np.array([0.,0.,-.060])
        grip = data.site_xpos[self.grip_site]
        points = np.array([saddle,data.site_xpos[self.pedals['front']],data.site_xpos[self.pedals['rear']],grip])
        loads = {} if contact_loads is None else contact_loads
        availability=loads if support_available is None else support_available
        enabled = [bool(availability.get(name,False)) for name in ('saddle','front','rear','grip')]
        frame_R=data.xmat[self.frame].reshape(3,3)
        pelvis_R=data.xmat[self.pelvis].reshape(3,3)
        pitch_error=atan2(-frame_R[2,0],frame_R[0,0])-atan2(-pelvis_R[2,0],pelvis_R[0,0])
        pitch_error=atan2(sin(pitch_error),cos(pitch_error))
        frame_dof,pelvis_dof=self.pitch_dofs
        pitch_request=float(np.clip(cfg.posture_pitch_k_nm_rad*pitch_error
            +cfg.posture_pitch_d_nms_rad*(data.qvel[frame_dof]-data.qvel[pelvis_dof]),
            -cfg.posture_pitch_limit_nm,cfg.posture_pitch_limit_nm))
        rotation = data.xmat[self.crank].reshape(3,3)
        phase = atan2(-rotation[2,0],rotation[0,0])
        blends={}
        requests={}
        for side,offset in (('front',0.),('rear',pi)):
            load=max(float(loads.get(side,0.)),0.)
            fraction=min(load/cfg.stance_blend_load_n,1.)
            blends[side]=fraction*fraction*(3.-2.*fraction)
            requests[side]=stance_force(phase+offset,command.mean_crank_torque_nm,self.crank_length_m)*blends[side]
        support_forces,diagnostics=pedaling_support_targets(weight,com,points,data.xpos[self.crank,0],
            cfg.pedal_support_fraction,cfg.bar_support_fraction,enabled,requests,pitch_moment_nm=0.)
        support_targets=diagnostics['requested_vertical_forces_n']
        diagnostics['requested_pitch_moment_nm']=pitch_request
        self.support_diagnostics=diagnostics
        result = {}
        terms = {}
        upper_targets = self._upper_targets(data)
        future=self._predict_target_state(model,data) if data.qvel[self.crank_spin_dof] != 0. else data
        saturation=dict(self.saturated_ik)
        future_upper=self._upper_targets(future) if future is not data else upper_targets
        self.saturated_ik=saturation
        for name,(qa,va,_) in self.joints.items():
            if name.startswith(('rider_hip_','rider_knee_','rider_ankle_')):
                continue
            target_q = upper_targets[name]
            target_speed=atan2(sin(future_upper[name]-target_q),cos(future_upper[name]-target_q))/self.target_difference_s
            result[name] = float(bounded_joint_torque(
                [data.qpos[qa]],[data.qvel[va]-target_speed],[target_q],cfg.joint_kp_nm_rad,
                cfg.joint_kd_nms_rad,cfg.joint_limit_nm)[0])
            raw = cfg.joint_kp_nm_rad*(target_q-data.qpos[qa])+cfg.joint_kd_nms_rad*(target_speed-data.qvel[va])
            terms[name] = (float(raw), 0.)
        rotation = data.xmat[self.crank].reshape(3,3)
        phase = atan2(-rotation[2,0],rotation[0,0])
        for side,offset in (('front',0.),('rear',pi)):
            names = [f'rider_{joint}_{side}' for joint in ('hip','knee','ankle')]
            qa = [self.joints[n][0] for n in names]
            va = [self.joints[n][1] for n in names]
            requested=requests[side]
            blend=blends[side]
            normal=data.site_xmat[self.pedals[side]].reshape(3,3)[:,2]
            desired_down=support_forces[side]
            depth=(1.-blend)*cfg.posture_sole_depth_m+blend*max(0.,-float(desired_down@normal))/cfg.support_k_n_m
            tangent=data.site_xmat[self.pedals[side]].reshape(3,3)[:,0]
            shear=blend*float(desired_down@tangent)/cfg.support_tangent_k_n_m
            target = self._targets(model,data,side,compression_m=depth,shear_m=shear)
            saturation=dict(self.saturated_ik)
            future_target=self._targets(model,future,side,compression_m=depth,shear_m=shear) if future is not data else target
            self.saturated_ik=saturation
            target_speed=np.arctan2(np.sin(future_target-target),np.cos(future_target-target))/self.target_difference_s
            pd = cfg.joint_kp_nm_rad*(target-data.qpos[qa])+cfg.joint_kd_nms_rad*(target_speed-data.qvel[va])
            jp,jr = np.zeros((3,model.nv)),np.zeros((3,model.nv))
            mujoco.mj_jac(model,data,jp,jr,data.site_xpos[self.soles[side]],self.feet[side])
            feedforward = jp[:,va].T@requested
            torque = pd+feedforward
            terms.update({n:(float(p),float(f)) for n,p,f in zip(names,pd,feedforward)})
            if not np.isfinite(torque).all():
                raise ValueError('non-finite articulated command')
            result.update(zip(names,map(float,torque)))
        # Requested support reactions are realized by the bounded limb torques,
        # never by writing the actual contact force or a root contribution.
        for side in ('front','rear'):
            if enabled[('front','rear').index(side)+1]:
                jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
                mujoco.mj_jac(model,data,jp,jr,data.site_xpos[self.soles[side]],self.feet[side])
                support += jp.T@np.array([0.,0.,-support_targets[side]])
        if enabled[3]:
            jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
            mujoco.mj_jac(model,data,jp,jr,grip,self.forearm)
            support += jp.T@support_forces['grip']
        self.last_terms = {}
        for name, (posture, pedaling) in terms.items():
            _, va, _ = self.joints[name]
            # Joint-only inverse-dynamics bias compensation. No root column is
            # actuated; the reaction on the parent is part of the mechanism.
            posture += float(data.qfrc_bias[va]) + float(support[va])
            # A hip actuator's equal opposite reaction acts on the pelvis.
            # Split the posture request between the two internal hips; never
            # write the floating root. This remains internal even in flight.
            hip_posture=-.5*pitch_request if name.startswith('rider_hip_') else 0.
            posture += hip_posture
            command_torque = float(bounded_effort([data.qvel[va]],[posture+pedaling],
                cfg.joint_limit_nm,cfg.joint_speed_limit_rad_s,cfg.joint_power_limit_w)[0])
            scale = command_torque/(posture+pedaling) if posture+pedaling != 0 else 0.
            result[name] = command_torque
            self.last_terms[name] = {'posture_nm':posture*scale, 'pedaling_nm':pedaling*scale,
                                    'command_nm':command_torque, 'support_nm':float(support[va])*scale, 'hip_posture_nm':hip_posture*scale, 'saturated':scale!=1.}
        return result

    def write(self,data,torques):
        if set(torques) != set(self.joints):
            raise ValueError('incomplete rider actuator command')
        values = {name:scalar(torque,'rider torque') for name,torque in torques.items()}
        for name,torque in values.items():
            _, dof, aid = self.joints[name]
            # Disabled means no actuator force at *any* velocity, including
            # the implicit endpoint; cancelling only its incoming bias leaves
            # an unrequested damping response during the step.
            self.model.actuator_biasprm[aid,2]=-self.config.joint_kd_nms_rad if self.command_enabled else 0.
            if not self.command_enabled:
                data.ctrl[aid]=0.
                continue
            # The affine actuator supplies -kd*v inside MuJoCo's implicit
            # velocity solve. Offset its current-state bias so the bounded
            # requested torque is still exactly delivered at q_n, v_n.
            data.ctrl[aid] = torque+self.config.joint_kd_nms_rad*data.qvel[dof]
