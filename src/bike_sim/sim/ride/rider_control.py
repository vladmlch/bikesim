"""Bounded internal rider actuation; no root force or direct human crank torque."""
from dataclasses import dataclass, field, asdict
import copy
from bike_sim.physics.rider_posture import RiderPosture
from math import acos, atan2, cos, hypot, isfinite, pi, sin
import numpy as np
from bike_sim.physics.checks import array, scalar
from bike_sim.sim.ride.support_geometry import (
    _box_pad_contact, _upper_box_face, project_sole_goal,
    validate_planar_support_model,
)
from bike_sim.sim.ride.pedal_recovery import PedalRecovery
from bike_sim.sim.ride.physical_mapping import relative_point_jacobian
from bike_sim.sim.ride.rider_state import RiderKinematicState
from bike_sim.physics.tire import _normal_contact


_PRESSURE_BLEND_COSINE = .15


def _pressure_mean_scale():
    boundary = acos(_PRESSURE_BLEND_COSINE)
    sine_boundary = sin(boundary)
    third_integral = 2./3.-sine_boundary+sine_boundary**3/3.
    fourth_integral = (3.*(pi/2.-boundary)/8.-sin(2.*boundary)/4.-sin(4.*boundary)/32.)
    integral = (sine_boundary+3.*third_integral/_PRESSURE_BLEND_COSINE**2
                -2.*fourth_integral/_PRESSURE_BLEND_COSINE**3)
    return pi/(2.*integral)


_PRESSURE_MEAN_SCALE = _pressure_mean_scale()


def stance_force(phase,mean_nm,crank_m):
    """Mean shaft intention as compressive pressure with smooth stroke boundaries."""
    phase = scalar(phase,'pedal phase')
    mean_nm = scalar(mean_nm,'requested mean torque',minimum=0)
    crank_m = scalar(crank_m,'crank length',positive=True)
    fraction = min(max(cos(phase)/_PRESSURE_BLEND_COSINE, 0.), 1.)
    weight = fraction*fraction*(3.-2.*fraction)*_PRESSURE_MEAN_SCALE
    return np.array([0., 0., -(mean_nm/crank_m)*weight])


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


def pedaling_force_requests(phase, mean_nm, crank_m, normals, loads, mu, *, normal_limit_n=500.):
    """Realize a mean shaft waveform with compressive forces and friction reserve."""
    phase = scalar(phase, 'pedal phase')
    mean = scalar(mean_nm, 'mean pedaling torque', minimum=0.)
    radius = scalar(crank_m, 'crank length', positive=True)
    friction = scalar(mu, 'pedal friction', minimum=0.)
    normal_limit = scalar(normal_limit_n, 'pedal normal request ceiling', positive=True)
    fraction = min(max(.5+.5*cos(phase)/_PRESSURE_BLEND_COSINE, 0.), 1.)
    front_weight = fraction*fraction*(3.-2.*fraction)
    weights = {'front': front_weight, 'rear': 1.-front_weight}
    total_torque = mean*(1.+.35*cos(2.*phase))
    geometry = {}
    capacities = {}
    for side, offset in (('front', 0.), ('rear', pi)):
        normal = array(normals[side], 'pedal normal', (3,))
        lever_x = radius*cos(phase+offset)
        lever_z = -radius*sin(phase+offset)
        normal_moment = lever_x*normal[2]-lever_z*normal[0]
        tangent_moment = lever_z*normal[2]+lever_x*normal[0]
        ratio = .8*friction*min(abs(tangent_moment)/radius, 1.)
        capacity = normal_moment+ratio*abs(tangent_moment)
        measured = scalar(loads[side], 'measured pedal pressure', minimum=0.)
        compression_limit = normal_limit
        if normal_moment < 0. and ratio > 0.:
            compression_limit = min(normal_limit, friction*measured/ratio)
        torque_limit = (normal_moment*compression_limit+abs(tangent_moment)
            *min(ratio*compression_limit, friction*min(compression_limit, measured)))
        geometry[side] = normal, normal_moment, tangent_moment, ratio, capacity, compression_limit, measured
        capacities[side] = max(0., torque_limit) if capacity > 1e-10 and weights[side] > 1e-6 else 0.
    active_weights = {side: weights[side] if capacities[side] > 0. else 0. for side in weights}
    weight_sum = sum(active_weights.values())
    torque_targets = {side: (min(capacities[side], total_torque*active_weights[side]/weight_sum)
                            if weight_sum else 0.) for side in weights}
    remaining = max(0., total_torque-sum(torque_targets.values()))
    for side in sorted(weights, key=weights.get, reverse=True):
        addition = min(remaining, capacities[side]-torque_targets[side])
        torque_targets[side] += addition
        remaining -= addition
    requests = {}
    for side in ('front', 'rear'):
        normal, normal_moment, tangent_moment, ratio, capacity, compression_limit, measured = geometry[side]
        wanted = torque_targets[side]
        if wanted == 0.:
            requests[side] = np.zeros(3)
            continue
        threshold = min(compression_limit, friction*measured/ratio) if ratio > 0. else compression_limit
        if wanted <= capacity*threshold+1e-12:
            compression = wanted/capacity
        else:
            compression = (wanted-abs(tangent_moment)*friction*measured)/normal_moment
        compression = min(max(0., compression), compression_limit)
        tangent = np.array([normal[2], 0., -normal[0]])
        signed_ratio = -ratio if tangent_moment < 0. else ratio
        raw = compression*(-normal+signed_ratio*tangent)
        requests[side] = feasible_pedal_force(raw, normal, friction, loads[side])
    return requests, weights


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
        self.sole_targets = {}
        # Central acceleration differences need a larger interval than the old
        # forward velocity-only difference (1e-6 s), to avoid cancellation.
        self.target_difference_s = 1e-4
        self.previous_target_data = mujoco.MjData(model)
        self.crank_length_m = scalar(crank_length_m,'crank length',positive=True)
        self.enabled = True
        self.last_terms = {}
        self.saturated_ik = {'front':False,'rear':False}
        # Geometric reach failure only (two-link IK), before envelope clipping.
        # initialize() must reject genuinely unreachable pedals; a clipped
        # target merely starts the joint at its anatomical bound.
        self.ik_reach_limited = {'front':False,'rear':False}
        self.joints = {}
        from bike_sim.mujoco.reference_rider import reference_joint_names
        for name in reference_joint_names():
            jid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
            aid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'act_'+name)
            if jid < 0 or aid < 0:
                raise ValueError(f'missing articulated joint/actuator {name}')
            self.joints[name] = (int(model.jnt_qposadr[jid]),int(model.jnt_dofadr[jid]),aid)
        self.joint_ranges={name:tuple(model.joint(name).range) for name in self.joints
                           if model.joint(name).limited[0]}
        # Aligned envelope vectors let envelope_forces batch all joints into a
        # single soft_edge_response call instead of one numpy ritual per joint.
        self._sync_joint_envelope()
        self.reset_activation()
        self.pelvis = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rider_pelvis')
        self.feet = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'rider_foot_{s}') for s in ('front','rear')}
        self.soles = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,f'site_rider_sole_{s}') for s in ('front','rear')}
        self.pedal_geoms = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,f'geom_pedal_{s}') for s in ('front','rear')}
        self.pedals = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,f'site_pedal_{s}') for s in ('front','rear')}
        self.frame = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'frame')
        self.torso = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rider_torso')
        self.upper_arms = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,
            f'rider_upper_arm_{s}') for s in ('left','right')}
        self.forearms = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,
            f'rider_forearm_{s}') for s in ('left','right')}
        self.grip_sites = {s:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,
            f'site_rider_grip_{s}') for s in ('left','right')}
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
        if min(self.pelvis,self.crank,*self.feet.values(),*self.soles.values(),
               *self.pedals.values(),*self.upper_arms.values(),
               *self.forearms.values(),*self.grip_sites.values()) < 0:
            raise ValueError('incomplete rider interface topology')
        mujoco.mj_kinematics(model,self.target_data)
        self.welded=config.pedal_attachment=='weld'
        self._weld_pedal_bodies={}
        self._weld_sole_offset={}
        if self.welded:
            for side in ('front','rear'):
                body=int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,
                    f'pedal_{side}'))
                self._weld_pedal_bodies[side]=body
                rel=(np.asarray(self.target_data.site_xpos[self.soles[side]])
                     -np.asarray(self.target_data.xpos[body]))
                self._weld_sole_offset[side]=np.asarray(
                    self.target_data.xmat[body]).reshape(3,3).T@rel
        self.welded_grip = config.grip_attachment == 'weld'
        self._weld_grip_offset = {}
        if self.welded_grip:
            # The connect datum is the steer point each grip site occupies at
            # qpos0; aim the arm IK there instead of the frame-fixed design
            # point plus a spring deflection that no longer exists.
            self.steer = int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'steer'))
            steer_R = self.target_data.xmat[self.steer].reshape(3,3)
            for side in ('left','right'):
                rel = (np.asarray(self.target_data.site_xpos[self.grip_sites[side]])
                       - np.asarray(self.target_data.xpos[self.steer]))
                self._weld_grip_offset[side] = steer_R.T @ rel
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
        if self.welded:
            # The welded sole cannot hover or separate: aim at the exact point
            # the weld equality is already holding (datum = qpos0 relative pose).
            body=self._weld_pedal_bodies[side]
            sole_goal=(np.asarray(data.xpos[body])
                +np.asarray(data.xmat[body]).reshape(3,3)
                 @self._weld_sole_offset[side])
            goal_diagnostic={'saturated':False,'limiting_reasons':[],'weld':True}
        else:
            recovery_goal = self._active_recovery[side].goal(data.geom_xpos[geom],
                data.geom_xmat[geom].reshape(3,3), model.geom_size[geom])
            if recovery_goal is None:
                sole_goal, goal_diagnostic = project_sole_goal(data.geom_xpos[geom],
                    data.geom_xmat[geom].reshape(3,3), model.geom_size[geom],
                    self.config.pedal_patch_half_length_m, radius, depth, shear)
            else:
                sole_goal = recovery_goal
                goal_diagnostic = {'saturated': False, 'limiting_reasons': [], 'recovery': True}
        ankle_goal = sole_goal+ankle_offset-np.array([0.,0.,.008])
        self.sole_targets[side] = sole_goal
        self.sole_goal_diagnostics[side] = goal_diagnostic
        target = R.T@(ankle_goal-hip)
        upper_length,lower_length,a0,b0,branch=self.leg_geometry[side]
        angles,saturated = _two_link_ik(target[[0,2]],upper_length,lower_length,elbow_sign=branch)
        self.ik_reach_limited[side] = saturated
        self.saturated_ik[side] = saturated
        # MuJoCo +Y is clockwise in X-Z; the pure IK helper uses CCW.
        hip_q = a0-angles[0]
        knee_q = (b0-a0)-angles[1]
        pitch = atan2(-R[2,0],R[0,0])
        targets=np.array([hip_q,knee_q,-pitch-hip_q-knee_q])
        current=np.array([data.qpos[self.joints[f'rider_{joint}_{side}'][0]] for joint in ('hip','knee','ankle')])
        # Equivalent +/-2*pi IK representations must not cause torque impulses.
        result=current + np.arctan2(np.sin(targets-current),np.cos(targets-current))
        for i,joint in enumerate(('hip','knee','ankle')):
            name=f'rider_{joint}_{side}'
            if name in self.joint_ranges:
                lo,hi=self.joint_ranges[name]
                # NaN-preserving scalar clip: comparisons are False for NaN so
                # a non-finite IK result propagates exactly like np.clip, which
                # min(max(x,lo),hi) would silently swallow.
                value=float(result[i])
                limited=float(hi) if value>hi else float(lo) if value<lo else value
                self.saturated_ik[side]=bool(self.saturated_ik[side] or limited!=value)
                result[i]=limited
        return result

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
        from bike_sim.physics.rider_segments import ARM_LATERAL_OFFSET_M
        frame_R = data.xmat[self.frame].reshape(3,3)
        grips = {}
        for side,sign in (('left',-1.),('right',1.)):
            if self.welded_grip:
                grips[side] = (data.xpos[self.steer]
                        + data.xmat[self.steer].reshape(3,3) @ self._weld_grip_offset[side])
            else:
                grip = (data.xpos[self.frame]
                        + frame_R @ (pose.grip + np.array([0.,sign*ARM_LATERAL_OFFSET_M,0.])))
                if grip_force_n is not None:
                    # The grip is compliant, not a weld. To push down on the bar, each
                    # hand goal must allow its spring to deflect down; two springs in
                    # parallel split the requested pair force. A zero-deflection
                    # IK goal fights the very support wrench used to balance the rider.
                    # This is only an actuator goal; the actual contact force
                    # remains exclusively in RiderContactApplier.
                    grip = grip + array(grip_force_n, 'grip force goal', (3,))/(2.*self.config.grip_k_n_m)
                grips[side] = grip
        # Keep a neutral relative torso/pelvis angle. Counter-rotating the
        # torso against the unactuated pelvis pitch pushes the pelvis further
        # in that direction through the equal actuator reaction. The hands
        # follow the actual bar separately, through the two arm joints.
        torso_q = self.neutral_torso_q + (0. if posture is None else posture.torso_lean_rad)
        torso_R = data.xmat[self.torso].reshape(3,3)
        upper_length,lower_length,a0,b0,branch=self.arm_geometry
        targets = {'rider_torso_hinge':torso_q}
        arm_saturated = False
        for side in ('left','right'):
            arm_target = torso_R.T @ (grips[side]-data.xpos[self.upper_arms[side]])
            arm_angles, saturated = _two_link_ik(arm_target[[0,2]],upper_length,lower_length,elbow_sign=branch)
            arm_saturated |= saturated
            targets[f'rider_shoulder_{side}'] = a0-arm_angles[0]
            targets[f'rider_elbow_{side}'] = (b0-a0)-arm_angles[1]
        self.saturated_ik.update(torso=self.neutral_torso_saturated,arms=arm_saturated)
        for name,target_q in targets.items():
            current = float(data.qpos[self.joints[name][0]])
            targets[name] = current+atan2(sin(target_q-current),cos(target_q-current))
            if name in self.joint_ranges:
                lo,hi=self.joint_ranges[name]
                value=targets[name]
                limited=float(hi) if value>hi else float(lo) if value<lo else value
                self.saturated_ik['arms' if name!='rider_torso_hinge' else 'torso'] |= limited!=value
                targets[name]=limited
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

    def _coasting_target_state(self, model, data, command, *, follow_motion=True):
        """Follow actual moving supports; braking belongs to compressive limb effort."""
        import mujoco
        target = self.coasting_target_data
        target.qpos[:] = data.qpos
        target.qvel.fill(0.)
        actual_rate = (float(data.qvel[self.crank_spin_dof]) if follow_motion
                       else command.crank_target_rate_rad_s)
        target.qvel[self.crank_spin_dof] = actual_rate
        target.qvel[self.pedal_spin_dofs] = -actual_rate
        mujoco.mj_kinematics(model, target)
        return target

    def reset_activation(self):
        self.active_state=np.zeros(len(self.joints))
        self.activation_time_s=None
        self.kinematic_state=None
        self.effort_diagnostics={}
        self.sole_goal_diagnostics = {}
        self.pedal_recovery = {side: PedalRecovery(self.config.pedal_patch_half_length_m,
            self.config.support_pad_radius_m, .01) for side in ('front', 'rear')}
        self._active_recovery = self.pedal_recovery

    def _sync_joint_envelope(self):
        """Rebuild the aligned envelope vectors in joint_ranges order."""
        qpos_adrs=[];dof_adrs=[];lower=[];upper=[]
        for name,(lo,hi) in self.joint_ranges.items():
            qa,va,_=self.joints[name]
            qpos_adrs.append(qa);dof_adrs.append(va);lower.append(lo);upper.append(hi)
        self._envelope_qpos_adrs=np.asarray(qpos_adrs,dtype=np.intp)
        self._envelope_dof_adrs=np.asarray(dof_adrs,dtype=np.intp)
        self._envelope_lower=np.asarray(lower)
        self._envelope_upper=np.asarray(upper)
        # joint_ranges stays a plain dict (tests inject limits); realign only
        # when its contents or iteration order actually change.
        self._envelope_signature=tuple(self.joint_ranges.items())

    def envelope_forces(self, model, data):
        from bike_sim.physics.rider_envelope import soft_edge_response
        if not self.joint_ranges:
            return np.zeros(model.nv),0.
        if tuple(self.joint_ranges.items())!=self._envelope_signature:
            self._sync_joint_envelope()
        torque,stored=soft_edge_response(data.qpos[self._envelope_qpos_adrs],
            self._envelope_lower,self._envelope_upper,
            self.config.joint_envelope_soft_k_nm_rad,self.config.joint_envelope_soft_margin_rad)
        force=np.zeros(model.nv)
        force[self._envelope_dof_adrs]=torque
        # sum() over the ndarray iterates np.float64s and adds left-to-right:
        # the same sequential accumulation as the old per-joint loop (summing a
        # tolist() of exact floats would take CPython's compensated fast path
        # and drift by an ulp).
        return force,float(sum(stored))

    def initialize(self,model,data):
        """Initial-condition setup only, before static equilibrium, never in step()."""
        import mujoco
        mujoco.mj_forward(model,data)
        self.reset_activation()
        # Pose the torso first, then solve the arms about that actual shoulder.
        upper = self._upper_targets(data)
        data.qpos[self.joints['rider_torso_hinge'][0]] = upper['rider_torso_hinge']
        mujoco.mj_forward(model,data)
        for name,value in self._upper_targets(data).items():
            data.qpos[self.joints[name][0]] = value
        mujoco.mj_forward(model,data)
        # A weld has no pad to deflect: with pedal_attachment='weld' the sole
        # rests exactly on the pedal surface, so the weld datum at qpos0 is the
        # design pose itself and the equalities start residual-free.
        welded = self.config.pedal_attachment == 'weld'
        for side in ('front','rear'):
            targets = self._targets(model,data,side,
                                    compression_m=0. if welded else None)
            if self.ik_reach_limited[side]:
                raise ValueError(f'initial {side} pedal is unreachable')
            for joint,value in zip(('hip','knee','ankle'),targets):
                qa,_,_ = self.joints[f'rider_{joint}_{side}']
                data.qpos[qa] = value
        mujoco.mj_forward(model,data)

    def _pedal_contact_estimate(self, model, data, side):
        """Kinematic prediction of the pad support: normal load and vertical force.

        The flat-pad spring law is a declared material property evaluated on
        the sole/box geometry, so the controller can predict its own support
        without reading any solved reaction. The same evaluation also covers a
        sole trapped beneath the pedal (the bottom face then reports a
        downward force). Welded feet carry the shoe by the coupling, not by a
        friction cone, so their load prediction is unbounded compression.
        """
        cfg = self.config
        half = cfg.pedal_patch_half_length_m
        radius = cfg.support_pad_radius_m
        geom = self.pedal_geoms[side]
        origin = data.geom_xpos[geom]
        rotation = data.geom_xmat[geom].reshape(3, 3)
        foot_rotation = data.xmat[self.feet[side]].reshape(3, 3)
        sole = data.site_xpos[self.soles[side]]
        pedal_body = int(model.geom_bodyid[geom])
        jac_a = np.zeros((3, model.nv)); jac_b = np.zeros((3, model.nv))
        load = force_z = 0.
        for sign in (-1., 1.):
            center = sole + foot_rotation @ np.array([sign*half, 0., radius])
            contact = _box_pad_contact(center, radius, origin, rotation,
                                       model.geom_size[geom])
            if not (contact.within_footprint and contact.gap_m < 0.):
                continue
            jrel = relative_point_jacobian(model, data, self.feet[side],
                                           pedal_body, contact.point_m,
                                           jac_a, jac_b)
            normal_speed = float((jrel @ data.qvel) @ contact.normal)
            normal, _ = _normal_contact(-contact.gap_m, -normal_speed,
                                        cfg.support_k_n_m/2, cfg.pedal_c_ns_m/2)
            load += normal
            force_z += normal*float(contact.normal[2])
        return load, force_z

    def compute(self,model,data,command, *, kinematic_state=None, support_available=None,
                advance=True, dt_s=None, steady_state=False, pedal_recovery=False):
        import mujoco
        if not isinstance(command,RiderCommand):
            raise ValueError('expected a RiderCommand')
        if kinematic_state is not None and not isinstance(kinematic_state,RiderKinematicState):
            raise ValueError('expected a RiderKinematicState')
        self.kinematic_state = kinematic_state
        self.command_enabled=command.enabled and self.enabled
        if not self.command_enabled:
            if advance:
                self.reset_activation()
            self.effort_diagnostics={'rider_active_request_nm':{n:0. for n in self.joints},
                'rider_active_delivered_nm':{n:0. for n in self.joints},
                'rider_positive_power_w':0.,'rider_passive_power_w':0.,'rider_activation_saturated':False}
            self.last_terms = {name:{'posture_nm':0.,'pedaling_nm':0.,'command_nm':0.,'saturated':False} for name in self.joints}
            return {name:0. for name in self.joints}
        cfg = self.config
        self._active_recovery = self.pedal_recovery if advance else copy.deepcopy(self.pedal_recovery)
        estimates = {side: self._pedal_contact_estimate(model, data, side)
                     for side in ('front', 'rear')}
        if pedal_recovery and not steady_state and not self.welded:
            for side in ('front', 'rear'):
                geom = self.pedal_geoms[side]
                self._active_recovery[side].observe(data.geom_xpos[geom],
                    data.geom_xmat[geom].reshape(3, 3), model.geom_size[geom],
                    data.site_xpos[self.soles[side]],
                    (0., 0., estimates[side][1]))
        posture = command.posture
        from bike_sim.sim.ride.rider_support import pedaling_support_targets
        support = np.zeros(model.nv)
        weight = self.rider_mass*float(np.linalg.norm(model.opt.gravity))
        com = np.sum(model.body_mass[self.rider_bodies,None]*data.xipos[self.rider_bodies],axis=0)/self.rider_mass
        saddle = data.xpos[self.pelvis]+data.xmat[self.pelvis].reshape(3,3)@np.array([0.,0.,-.060])
        # The support polygon sees the pair of hands as one bar contact: its
        # midpoint, halfway between the two grip sites.
        grip = 0.5*(data.site_xpos[self.grip_sites['left']]
                    +data.site_xpos[self.grip_sites['right']])
        points = np.array([saddle,data.site_xpos[self.pedals['front']],data.site_xpos[self.pedals['rear']],grip])
        # A welded sole cannot measure its coupling: its load prediction is
        # the rider's own weight (a foot cannot press more than it carries),
        # and friction requests stay bounded by predicted compression.
        loads = {side: (weight if self.welded else estimates[side][0])
                 for side in ('front', 'rear')}
        availability={} if support_available is None else support_available
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
        coasting_torque = 0.
        if command.crank_target_phase_rad is not None and not steady_state:
            rate_error = command.crank_target_rate_rad_s-float(data.qvel[self.crank_spin_dof])
            coasting_torque = float(np.clip(cfg.coasting_brake_d_nm_s_rad*rate_error,
                -cfg.coasting_brake_limit_nm, cfg.coasting_brake_limit_nm))
        pedaling_requests = {}
        pedaling_weights = {}
        if command.mean_crank_torque_nm > 0.:
            normals = {side: _upper_box_face(data.geom_xpos[geom],
                data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])[1]
                for side, geom in self.pedal_geoms.items()}
            pedal_loads = {side: max(float(loads.get(side, 0.)), 0.) for side in ('front', 'rear')}
            pedaling_requests, pedaling_weights = pedaling_force_requests(phase,
                command.mean_crank_torque_nm, self.crank_length_m, normals, pedal_loads, cfg.support_mu,
                normal_limit_n=.75*weight)
        for side,offset in (('front',0.),('rear',pi)):
            load=max(float(loads.get(side,0.)),0.)
            fraction=min(load/cfg.stance_blend_load_n,1.)
            blends[side]=fraction*fraction*(3.-2.*fraction)
            stance[side]=(command.mean_crank_torque_nm==0. or pedaling_weights.get(side, 0.) > 1e-6)
            recovering = self._active_recovery[side].stage != 'none'
            if recovering:
                stance[side] = False
                blends[side] = 0.
            if not stance[side]:
                # A flat-pedal return foot must be lifted rather than carrying
                # a coasting support load that brakes the rising crank arm.
                # This changes only the control target, never physical Fn.
                enabled[1+('front','rear').index(side)]=False
            geom=self.pedal_geoms[side]
            _,normal,_=_upper_box_face(data.geom_xpos[geom],data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])
            if recovering:
                requests[side] = np.zeros(3)
            elif command.mean_crank_torque_nm==0.:
                brake_phase = phase+offset+(pi if coasting_torque < 0. else 0.)
                raw = stance_force(brake_phase, abs(coasting_torque), self.crank_length_m)/_PRESSURE_MEAN_SCALE
                requests[side] = feasible_pedal_force(raw,normal,cfg.support_mu,load)
            else:
                # A lost contact must still be approached/pressed normally.
                # Multiplying the entire request by measured Fn makes zero
                # load an absorbing state. Tangential demand remains bounded
                # by measured Fn in feasible_pedal_force; no adhesion is added.
                requests[side] = pedaling_requests[side]
        from bike_sim.sim.ride.rider_balance import balance_force_request
        balance=balance_force_request(self,model,data,posture)
        support_forces,diagnostics=pedaling_support_targets(weight,com,points,data.xpos[self.crank,0],
            cfg.pedal_support_fraction,cfg.bar_support_fraction,enabled,requests,
            pitch_moment_nm=pitch_request,balance_force_on_rider_n=balance)
        support_targets=diagnostics['requested_vertical_forces_n']
        diagnostics['requested_pitch_moment_nm']=pitch_request
        diagnostics['stance']=stance.copy()
        diagnostics['feasible_pedal_force_on_bike_n']={s:requests[s].tolist() for s in requests}
        diagnostics['posture'] = asdict(posture)
        diagnostics['coasting'] = command.crank_target_phase_rad is not None
        diagnostics['coasting_requested_crank_torque_nm'] = coasting_torque
        actual_phase = float(data.qpos[self.crank_spin_qpos])
        desired_phase = command.crank_target_phase_rad
        diagnostics['crank_tracking'] = {
            'actual_phase_rad': actual_phase,
            'target_phase_rad': desired_phase,
            'phase_error_rad': None if desired_phase is None else atan2(
                sin(desired_phase - actual_phase), cos(desired_phase - actual_phase)),
            'actual_rate_rad_s': float(data.qvel[self.crank_spin_dof]),
            'target_rate_rad_s': command.crank_target_rate_rad_s,
        }
        diagnostics['feet'] = {}
        self.support_diagnostics=diagnostics
        result = {}
        terms = {}
        upper_targets = self._upper_targets(data, posture, grip_force_n=support_forces['grip'])
        target_state = (data if command.crank_target_phase_rad is None
                        else self._coasting_target_state(model, data, command,
                                                         follow_motion=not steady_state))
        future=self._predict_target_state(model,target_state) if target_state.qvel[self.crank_spin_dof] != 0. else target_state
        previous=self._predict_target_state(model,target_state,reverse=True) if future is not target_state else target_state
        desired_acceleration=np.zeros(model.nv)
        saturation=dict(self.saturated_ik); reach=dict(self.ik_reach_limited)
        future_upper=self._upper_targets(future, posture, grip_force_n=support_forces['grip']) if future is not data else upper_targets
        self.saturated_ik=saturation; self.ik_reach_limited=reach
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
            clearance=0. if (self.welded or stance[side]) else cfg.swing_clearance_m
            if not stance[side]:
                depth=shear=0.
            target = self._targets(model,target_state,side,compression_m=depth,shear_m=shear,clearance_m=clearance,posture=posture)
            diagnostics['feet'][side] = {
                'actual_sole_position_m': data.site_xpos[self.soles[side]].tolist(),
                'target_sole_position_m': self.sole_targets[side].tolist(),
                'ik_saturated': bool(self.saturated_ik[side]),
                'recovery_stage': self._active_recovery[side].stage,
                'goal_geometry': self.sole_goal_diagnostics[side].copy(),
            }
            saturation=dict(self.saturated_ik); reach=dict(self.ik_reach_limited)
            future_target=self._targets(model,future,side,compression_m=depth,shear_m=shear,clearance_m=clearance,posture=posture) if future is not target_state else target
            previous_target=self._targets(model,previous,side,compression_m=depth,shear_m=shear,clearance_m=clearance,posture=posture) if previous is not target_state else target
            self.saturated_ik=saturation; self.ik_reach_limited=reach
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
            for side in ('left','right'):
                jp=np.zeros((3,model.nv))
                mujoco.mj_jac(model,data,jp,None,
                    data.site_xpos[self.grip_sites[side]],self.forearms[side])
                support += jp.T@(support_forces['grip']/2.)
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
            self.last_terms[name] = {'requested_nm':requested_torque, 'posture_nm':posture*scale, 'pedaling_nm':pedaling*scale,
                                    'command_nm':command_torque, 'support_nm':float(support[va])*scale, 'tracking_nm':float(tracking_inertia[va])*scale, 'hip_posture_nm':hip_posture*scale, 'saturated':scale!=1.}
        from bike_sim.sim.ride.rider_effort import finalize_effort
        return finalize_effort(self,data,result,advance=advance,
            dt_s=float(model.opt.timestep) if dt_s is None else dt_s,steady_state=steady_state)

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
