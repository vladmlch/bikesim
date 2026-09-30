"""Bounded internal rider actuation; no root force or direct human crank torque."""
from dataclasses import dataclass, field, asdict
from bike_sim.physics.rider_posture import RiderPosture
from math import acos, atan2, cos, hypot, isfinite, pi, sin
import numpy as np
from bike_sim.physics.checks import array, scalar
from bike_sim.sim.ride.support_geometry import _upper_box_face, validate_planar_support_model


def stance_force(phase,mean_nm,crank_m):
    phase = scalar(phase,'pedal phase')
    mean_nm = scalar(mean_nm,'requested mean torque',minimum=0)
    crank_m = scalar(crank_m,'crank length',positive=True)
    weight = max(cos(phase),0.)*pi/2
    return (mean_nm/crank_m)*weight*np.array([-sin(phase),0.,-cos(phase)])


def feasible_pedal_force(request,normal,mu,measured_normal_n):
    """Project a limb force target onto the unilateral flat-pedal friction cone.

    This limits only the *requested human contact force*, not the tire force,
    crank motor, or a solver load. Actual contact is still solved independently.
    The pure crank-tangential request otherwise asks for infinite Fx/Fn near a
    dead center, which makes a flat-pedal controller slide its own foot off.
    """
    f=array(request,'pedal force request',(3,))
    n=array(normal,'pedal normal',(3,))
    friction=scalar(mu,'pedal friction',minimum=0.)
    measured=scalar(measured_normal_n,'measured pedal load',minimum=0.)
    if abs(np.linalg.norm(n)-1.)>1e-9 or abs(n[1])>1e-9 or abs(f[1])>1e-9:
        raise ValueError('pedal force request must use a planar unit normal')
    tangent=np.array([n[2],0.,-n[0]])
    compression=max(-float(f@n),0.)
    bound=friction*min(compression,measured)
    return -compression*n+np.clip(float(f@tangent),-bound,bound)*tangent


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
    return _two_link_ik(p,a,b,elbow_sign)


def _two_link_ik(p,a,b,elbow_sign):
    """Core of `two_link_ik` for callers that already validated inputs."""
    distance = hypot(float(p[0]),float(p[1]))
    lo,hi = abs(a-b)+1e-10,a+b-1e-10
    if hi <= lo:
        raise ValueError('IK links are too small')
    saturated = not lo <= distance <= hi
    d = min(max(distance,lo),hi)
    direction = atan2(p[1],p[0]) if distance > 1e-15 else -pi/2
    cosine = min(max((d*d-a*a-b*b)/(2*a*b),-1.),1.)
    knee = elbow_sign*acos(cosine)
    hip = direction-atan2(b*sin(knee),a+b*cos(knee))
    return np.array([hip,knee]),saturated


@dataclass(frozen=True)
class RiderCommand:
    mean_crank_torque_nm: float = 0.
    enabled: bool = True
    posture: RiderPosture = field(default_factory=RiderPosture)
    crank_target_phase_rad: float | None = None
    crank_target_rate_rad_s: float = 0.

    def __post_init__(self):
        scalar(self.mean_crank_torque_nm,'rider effort',minimum=0)
        if not isinstance(self.posture, RiderPosture):
            raise ValueError('rider posture must be a RiderPosture')
        if not isinstance(self.enabled,bool):
            raise ValueError('rider controller enable must be a bool')
        scalar(self.crank_target_rate_rad_s, 'coasting crank goal speed')
        if self.crank_target_phase_rad is not None:
            scalar(self.crank_target_phase_rad, 'coasting crank goal phase')
            if self.mean_crank_torque_nm > 0.:
                raise ValueError('coasting goals cannot request pedaling effort')


class ArticulatedRiderController:
    """Foot-force feedforward plus PD, clipped AFTER summation on limb DOFs only."""
    def __init__(self,model,pose,config,crank_length_m):
        import mujoco
        self.pose,self.config = pose,config
        self.model=model
        self.command_enabled=True
        self.target_data = mujoco.MjData(model)
        self.coasting_target_data = mujoco.MjData(model)
        # Central acceleration differences need a larger interval than the old
        # forward velocity-only difference (1e-6 s), to avoid cancellation.
        self.target_difference_s = 1e-4
        self.previous_target_data = mujoco.MjData(model)
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
        self.pedal_geoms = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,f'geom_pedal_{s}') for s in ('front','rear')}
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
        self.crank_spin_qpos=int(model.jnt_qposadr[model.joint('crank_spin').id])
        self.pedal_spin_dofs=[int(model.jnt_dofadr[model.joint(f'pedal_{side}_spin').id]) for side in ('front','rear')]
        self.pedal_spin_qpos=[int(model.jnt_qposadr[model.joint(f'pedal_{side}_spin').id]) for side in ('front','rear')]
        if min(self.pelvis,self.crank,*self.feet.values(),*self.soles.values(),*self.pedals.values()) < 0:
            raise ValueError('incomplete rider interface topology')
        mujoco.mj_kinematics(model,self.target_data)
        validate_planar_support_model(model,self.target_data,self.pedal_geoms.values())
        self.leg_geometry={}
        self.sole_jacobians={}
        for side in ('front','rear'):
            upper=getattr(pose,f'knee_{side}')-pose.hip
            lower=getattr(pose,f'ankle_{side}')-getattr(pose,f'knee_{side}')
            upper_angle=atan2(upper[2],upper[0])
            lower_angle=atan2(lower[2],lower[0])
            self.leg_geometry[side]=(float(np.linalg.norm(upper)),float(np.linalg.norm(lower)),
                upper_angle,lower_angle,1 if sin(lower_angle-upper_angle)>=0 else -1)
            self.sole_jacobians[side]=(np.empty((3,model.nv)),np.empty((3,model.nv)))
        trunk=pose.shoulder-pose.hip
        upper=pose.elbow-pose.shoulder
        lower=pose.grip-pose.elbow
        upper_length=float(np.linalg.norm(upper))
        lower_length=float(np.linalg.norm(lower))
        upper_angle=atan2(upper[2],upper[0])
        lower_angle=atan2(lower[2],lower[0])
        self.arm_geometry=(upper_length,lower_length,upper_angle,lower_angle,
            1 if sin(lower_angle-upper_angle)>=0 else -1)
        candidates=[two_link_ik((pose.grip-pose.hip)[[0,2]],np.linalg.norm(trunk),
            (upper_length+lower_length)*config.arm_reach_fraction,elbow_sign=sign) for sign in (-1,1)]
        neutral,self.neutral_torso_saturated=max(candidates,key=lambda pair:sin(pair[0][0]))
        self.neutral_torso_q=atan2(trunk[2],trunk[0])-neutral[0]
        self._sole_drop=np.array([0.,0.,.008+config.support_pad_radius_m])

    def _targets(self,model,data,side, *, compression_m=None, shear_m=0., clearance_m=0., posture=None):
        hip = data.xpos[self.pelvis]
        if posture is not None and posture.pelvis_offset_m is not None:
            x, z = posture.pelvis_offset_m
            hip = data.xpos[self.frame] + data.xmat[self.frame].reshape(3,3) @ (
                self.pose.hip + np.array([x, 0., z]))
            # This is only an inverse-kinematics goal. No data coordinate is
            # written: movement requires actual foot/hand reactions and effort.
        # Solve in the actual parent frame so the foot target reaches the
        # actual platform. Posture is requested through a balanced contact
        # wrench, not a fictitious root frame in the leg IK.
        R = data.xmat[self.pelvis].reshape(3,3)
        ankle_offset = getattr(self.pose,f'ankle_{side}')-getattr(self.pose,f'pedal_{side}')
        pedal = data.site_xpos[self.pedals[side]]
        depth=self.config.posture_sole_depth_m if compression_m is None else scalar(compression_m,'target sole compression',minimum=0)
        depth-=scalar(clearance_m,'target sole clearance',minimum=0)
        geom=self.pedal_geoms[side]
        surface,normal,tangent=_upper_box_face(data.geom_xpos[geom],
            data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])
        shear=scalar(shear_m,'target sole shear')
        # Target the actual platform top, with a deflection consistent with the
        # requested normal load. A fixed 3 mm position goal in parallel with an
        # unrelated force goal makes the PD oppose the very load it must hold.
        radius=self.config.support_pad_radius_m
        ankle_goal=(surface+ankle_offset+normal*(radius-depth)
                    -self._sole_drop+shear*tangent)
        target = R.T@(ankle_goal-hip)
        upper_length,lower_length,a0,b0,branch=self.leg_geometry[side]
        angles,saturated = _two_link_ik(target[[0,2]],upper_length,lower_length,elbow_sign=branch)
        self.saturated_ik[side] = saturated
        # MuJoCo +Y is clockwise in X-Z; the pure IK helper uses CCW.
        hip_q = a0-angles[0]
        knee_q = (b0-a0)-angles[1]
        pitch = atan2(-R[2,0],R[0,0])
        targets=np.array([hip_q,knee_q,-pitch-hip_q-knee_q])
        current=np.array([data.qpos[self.joints[f'rider_{joint}_{side}'][0]] for joint in ('hip','knee','ankle')])
        # Equivalent +/-2*pi IK representations must not cause torque impulses.
        return current + np.arctan2(np.sin(targets-current),np.cos(targets-current))

    def _upper_targets(self, data, posture=None, *, grip_force_n=None):
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
        if grip_force_n is not None:
            # The grip is compliant, not a weld. To push down on the bar, the
            # hand goal must allow its spring to deflect down. A zero-deflection
            # IK goal fights the very support wrench used to balance the rider.
            # This is only an actuator goal; the actual paired contact force
            # remains exclusively in RiderContactApplier.
            grip = grip + array(grip_force_n, 'grip force goal', (3,))/self.config.grip_k_n_m
        # Keep a neutral relative torso/pelvis angle. Counter-rotating the
        # torso against the unactuated pelvis pitch pushes the pelvis further
        # in that direction through the equal actuator reaction. The hands
        # follow the actual bar separately, through the two arm joints.
        torso_q = self.neutral_torso_q + (0. if posture is None else posture.torso_lean_rad)
        torso_R = data.xmat[self.torso].reshape(3,3)
        arm_target = torso_R.T @ (grip-data.xpos[self.upper_arm])
        upper_length,lower_length,a0,b0,branch=self.arm_geometry
        arm_angles, arm_saturated = _two_link_ik(arm_target[[0,2]],upper_length,lower_length,elbow_sign=branch)
        targets = {'rider_torso_hinge':torso_q,'rider_shoulder':a0-arm_angles[0],
                   'rider_elbow':(b0-a0)-arm_angles[1]}
        self.saturated_ik.update(torso=self.neutral_torso_saturated,arms=arm_saturated)
        for name,target_q in targets.items():
            current = float(data.qpos[self.joints[name][0]])
            targets[name] = current+atan2(sin(target_q-current),cos(target_q-current))
        return targets

    def _predict_target_state(self,model,data, *, reverse=False):
        """Directional derivative of moving IK goals on detached kinematics.

        Damping against zero joint speed brakes intended pedaling. Estimate the
        goal speed induced by crank spin, not a prescribed cadence. Root bob/pitch
        and other uncommanded motions remain damped disturbances, not desired
        target velocities that would cancel the postural feedback.
        No forces, contacts, or controller states are advanced on the live model.
        """
        import mujoco
        future=self.previous_target_data if reverse else self.target_data
        future.qpos[:]=data.qpos
        future.qvel.fill(0.)
        rate=data.qvel[self.crank_spin_dof]
        future.qvel[self.crank_spin_dof]=rate
        # Keep the platform attitude fixed while its spindle follows the crank.
        future.qvel[self.pedal_spin_dofs]=-rate
        mujoco.mj_integratePos(model,future.qpos,future.qvel,
                               -self.target_difference_s if reverse else self.target_difference_s)
        mujoco.mj_kinematics(model,future)
        return future

    def initialize_velocity(self, model, data):
        """Match moving pedal targets at startup, without moving the solved pose.

        Only reset calls this method. During a ride the same target derivative
        is a bounded actuator request, never a write to generalized velocity.
        """
        future = self._predict_target_state(model, data)
        previous = self._predict_target_state(model, data, reverse=True)
        for side in ('front', 'rear'):
            next_target = self._targets(model, future, side)
            previous_target = self._targets(model, previous, side)
            velocity = np.arctan2(np.sin(next_target-previous_target),
                                 np.cos(next_target-previous_target)) / (2.*self.target_difference_s)
            for joint, rate in zip(('hip', 'knee', 'ankle'), velocity):
                data.qvel[self.joints[f'rider_{joint}_{side}'][1]] = rate

    def _coasting_target_state(self, model, data, command):
        import mujoco
        target = self.coasting_target_data
        target.qpos[:] = data.qpos
        target.qvel.fill(0.)
        phase_change = command.crank_target_phase_rad - float(data.qpos[self.crank_spin_qpos])
        target.qpos[self.crank_spin_qpos] += phase_change
        target.qpos[self.pedal_spin_qpos] -= phase_change
        target.qvel[self.crank_spin_dof] = command.crank_target_rate_rad_s
        target.qvel[self.pedal_spin_dofs] = -command.crank_target_rate_rad_s
        mujoco.mj_kinematics(model, target)
        return target

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
        posture = command.posture
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
        enabled[0] = enabled[0] and posture.use_saddle
        frame_R=data.xmat[self.frame].reshape(3,3)
        pelvis_R=data.xmat[self.pelvis].reshape(3,3)
        pitch_error=atan2(-frame_R[2,0],frame_R[0,0])-atan2(-pelvis_R[2,0],pelvis_R[0,0])
        pitch_error += posture.pelvis_pitch_rad
        pitch_error=atan2(sin(pitch_error),cos(pitch_error))
        frame_dof,pelvis_dof=self.pitch_dofs
        pitch_request=float(np.clip(cfg.posture_pitch_k_nm_rad*pitch_error
            +cfg.posture_pitch_d_nms_rad*(data.qvel[frame_dof]-data.qvel[pelvis_dof]),
            -cfg.posture_pitch_limit_nm,cfg.posture_pitch_limit_nm))
        rotation = data.xmat[self.crank].reshape(3,3)
        phase = atan2(-rotation[2,0],rotation[0,0])
        blends={}
        requests={}
        stance={}
        for side,offset in (('front',0.),('rear',pi)):
            load=max(float(loads.get(side,0.)),0.)
            fraction=min(load/cfg.stance_blend_load_n,1.)
            blends[side]=fraction*fraction*(3.-2.*fraction)
            stance[side]=(command.mean_crank_torque_nm==0. or cos(phase+offset)>0.)
            if not stance[side]:
                # A flat-pedal return foot must be lifted rather than carrying
                # a coasting support load that brakes the rising crank arm.
                # This changes only the control target, never physical Fn.
                enabled[1+('front','rear').index(side)]=False
            geom=self.pedal_geoms[side]
            _,normal,_=_upper_box_face(data.geom_xpos[geom],data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])
            if command.mean_crank_torque_nm==0.:
                requests[side]=np.zeros(3)
            else:
                # A lost contact must still be approached/pressed normally.
                # Multiplying the entire request by measured Fn makes zero
                # load an absorbing state. Tangential demand remains bounded
                # by measured Fn in feasible_pedal_force; no adhesion is added.
                raw=stance_force(phase+offset,command.mean_crank_torque_nm,self.crank_length_m)
                requests[side]=feasible_pedal_force(raw,normal,cfg.support_mu,load)
        support_forces,diagnostics=pedaling_support_targets(weight,com,points,data.xpos[self.crank,0],
            cfg.pedal_support_fraction,cfg.bar_support_fraction,enabled,requests,pitch_moment_nm=pitch_request)
        support_targets=diagnostics['requested_vertical_forces_n']
        diagnostics['requested_pitch_moment_nm']=pitch_request
        diagnostics['stance']=stance.copy()
        diagnostics['feasible_pedal_force_on_bike_n']={s:requests[s].tolist() for s in requests}
        diagnostics['posture'] = asdict(posture)
        diagnostics['coasting'] = command.crank_target_phase_rad is not None
        self.support_diagnostics=diagnostics
        result = {}
        terms = {}
        upper_targets = self._upper_targets(data, posture, grip_force_n=support_forces['grip'])
        target_state = (data if command.crank_target_phase_rad is None
                        else self._coasting_target_state(model, data, command))
        future=self._predict_target_state(model,target_state) if target_state.qvel[self.crank_spin_dof] != 0. else target_state
        previous=self._predict_target_state(model,target_state,reverse=True) if future is not target_state else target_state
        desired_acceleration=np.zeros(model.nv)
        saturation=dict(self.saturated_ik)
        future_upper=self._upper_targets(future, posture, grip_force_n=support_forces['grip']) if future is not data else upper_targets
        self.saturated_ik=saturation
        for name,(qa,va,_) in self.joints.items():
            if name.startswith(('rider_hip_','rider_knee_','rider_ankle_')):
                continue
            target_q = upper_targets[name]
            target_speed=atan2(sin(future_upper[name]-target_q),cos(future_upper[name]-target_q))/self.target_difference_s
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
            geom=self.pedal_geoms[side]
            _,normal,tangent=_upper_box_face(data.geom_xpos[geom],
                data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])
            desired_down=support_forces[side]
            depth=(1.-blend)*cfg.posture_sole_depth_m+blend*max(0.,-float(desired_down@normal))/cfg.support_k_n_m
            shear=blend*float(desired_down@tangent)/cfg.support_tangent_k_n_m
            clearance=0. if stance[side] else cfg.swing_clearance_m
            if not stance[side]:
                depth=shear=0.
            target = self._targets(model,target_state,side,compression_m=depth,shear_m=shear,clearance_m=clearance,posture=posture)
            saturation=dict(self.saturated_ik)
            future_target=self._targets(model,future,side,compression_m=depth,shear_m=shear,clearance_m=clearance,posture=posture) if future is not target_state else target
            previous_target=self._targets(model,previous,side,compression_m=depth,shear_m=shear,clearance_m=clearance,posture=posture) if previous is not target_state else target
            self.saturated_ik=saturation
            ahead=np.arctan2(np.sin(future_target-target),np.cos(future_target-target))
            behind=np.arctan2(np.sin(previous_target-target),np.cos(previous_target-target))
            target_speed=(ahead-behind)/(2.*self.target_difference_s)
            desired_acceleration[va]=(ahead+behind)/self.target_difference_s**2
            pd = cfg.joint_kp_nm_rad*(target-data.qpos[qa])+cfg.joint_kd_nms_rad*(target_speed-data.qvel[va])
            jp,jr = self.sole_jacobians[side]
            mujoco.mj_jac(model,data,jp,None,data.site_xpos[self.soles[side]],self.feet[side])
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
                jp,_=self.sole_jacobians[side]
                support += jp.T@np.array([0.,0.,-support_targets[side]])
        if enabled[3]:
            jp=np.zeros((3,model.nv))
            mujoco.mj_jac(model,data,jp,None,grip,self.forearm)
            support += jp.T@support_forces['grip']
        # Convective acceleration of the moving crank targets. Gravity/Coriolis
        # compensation alone cannot track a circular pedal path at cadence.
        # Only limb rows are commanded, still inside the existing effort caps.
        tracking_inertia=np.zeros(model.nv)
        mujoco.mj_mulM(model,data,tracking_inertia,desired_acceleration)
        self.last_terms = {}
        for name, (posture, pedaling) in terms.items():
            _, va, _ = self.joints[name]
            # Joint-only inverse-dynamics bias compensation. No root column is
            # actuated; the reaction on the parent is part of the mechanism.
            posture += float(data.qfrc_bias[va]) + float(support[va]) + float(tracking_inertia[va])
            # A hip actuator's equal opposite reaction acts on the pelvis.
            # Split the posture request between the two internal hips; never
            # write the floating root. This remains internal even in flight.
            hip_posture=-.5*pitch_request if name.startswith('rider_hip_') else 0.
            posture += hip_posture
            joint_speed=float(data.qvel[va])
            requested_torque=posture+pedaling
            if not all(isfinite(value) for value in (posture,pedaling,requested_torque,joint_speed)):
                raise ValueError('non-finite articulated command')
            command_torque = min(max(requested_torque,-cfg.joint_limit_nm),cfg.joint_limit_nm)
            if command_torque*joint_speed>0.:
                if abs(joint_speed)>=cfg.joint_speed_limit_rad_s:
                    command_torque=0.
                else:
                    power_cap=min(cfg.joint_limit_nm,cfg.joint_power_limit_w/abs(joint_speed))
                    command_torque=min(max(command_torque,-power_cap),power_cap)
            scale = command_torque/requested_torque if requested_torque != 0 else 0.
            result[name] = command_torque
            self.last_terms[name] = {'posture_nm':posture*scale, 'pedaling_nm':pedaling*scale,
                                    'command_nm':command_torque, 'support_nm':float(support[va])*scale, 'tracking_nm':float(tracking_inertia[va])*scale, 'hip_posture_nm':hip_posture*scale, 'saturated':scale!=1.}
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
