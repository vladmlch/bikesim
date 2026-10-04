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
from bike_sim.physics.rider_program import pedal_intent
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


def pedal_torque_waveform(mean_nm, phase_rad, ripple):
    """Two-leg crank torque: mean*(1+ripple*cos(2*phase)), ripple in [0,1)."""
    mean = scalar(mean_nm, 'mean pedaling torque', minimum=0.)
    phase = scalar(phase_rad, 'pedal phase')
    depth = scalar(ripple, 'pedal torque ripple', minimum=0.)
    if depth >= 1.:
        raise ValueError('pedal torque ripple must be below one')
    return mean*(1.+depth*cos(2.*phase))


def pedaling_force_requests(phase, mean_nm, crank_m, normals, loads, mu, *, ripple=.35,
                            return_foot_preload_n=0., preload_sides=('front', 'rear')):
    """Request a net shaft waveform, accounting for recovery-leg pressure.

    The preload's signed moment is removed from the active-leg target before
    allocation. Recovering feet may be excluded by the caller. Infeasible
    geometry still obeys the pressure/friction bounds, without a negative
    active torque request or a guarantee of exact net torque.
    """
    phase = scalar(phase, 'pedal phase')
    mean = scalar(mean_nm, 'mean pedaling torque', minimum=0.)
    radius = scalar(crank_m, 'crank length', positive=True)
    friction = scalar(mu, 'pedal friction', minimum=0.)
    preload = scalar(return_foot_preload_n, 'return foot preload', minimum=0.)
    try:
        preload_sides = frozenset(preload_sides)
    except TypeError as error:
        raise ValueError('preload sides must be a sequence of foot names') from error
    if not preload_sides <= {'front', 'rear'}:
        raise ValueError('unknown preload foot')
    fraction = min(max(.5+.5*cos(phase)/_PRESSURE_BLEND_COSINE, 0.), 1.)
    front_weight = fraction*fraction*(3.-2.*fraction)
    weights = {'front': front_weight, 'rear': 1.-front_weight}
    total_torque = pedal_torque_waveform(mean, phase, ripple)
    geometry = {}
    capacities = {}
    recovery_requests = {}
    recovery_torque = 0.
    for side, offset in (('front', 0.), ('rear', pi)):
        normal = array(normals[side], 'pedal normal', (3,))
        lever_x = radius*cos(phase+offset)
        lever_z = -radius*sin(phase+offset)
        normal_moment = lever_x*normal[2]-lever_z*normal[0]
        tangent_moment = lever_z*normal[2]+lever_x*normal[0]
        ratio = .8*friction*min(abs(tangent_moment)/radius, 1.)
        capacity = normal_moment+ratio*abs(tangent_moment)
        measured = scalar(loads[side], 'measured pedal pressure', minimum=0.)
        compression_limit = float('inf')
        if normal_moment < 0. and ratio > 0.:
            compression_limit = friction*measured/ratio
        if np.isinf(compression_limit):
            # Avoid 0*inf at a horizontal power stroke. A positive normal
            # lever allows an unbounded request; the allocator still enforces
            # actual joint/support budgets. With no normal lever, only the
            # measured-load friction can contribute torque.
            torque_limit = (float('inf') if normal_moment > 0. else
                            abs(tangent_moment)*friction*measured)
        else:
            torque_limit = (normal_moment*compression_limit+abs(tangent_moment)
                *min(ratio*compression_limit, friction*min(compression_limit, measured)))
        geometry[side] = normal, normal_moment, tangent_moment, ratio, capacity, compression_limit, measured
        capacities[side] = max(0., torque_limit) if capacity > 1e-10 and weights[side] > 1e-6 else 0.
        if preload > 0. and side in preload_sides and weights[side] <= 1e-6:
            recovery_force = feasible_pedal_force(-preload*normal, normal, friction, measured)
            recovery_requests[side] = recovery_force
            recovery_torque += lever_z*recovery_force[0]-lever_x*recovery_force[2]
    active_weights = {side: weights[side] if capacities[side] > 0. else 0. for side in weights}
    weight_sum = sum(active_weights.values())
    driving_torque = max(0., total_torque-recovery_torque)
    torque_targets = {side: (min(capacities[side], driving_torque*active_weights[side]/weight_sum)
                            if weight_sum else 0.) for side in weights}
    remaining = max(0., driving_torque-sum(torque_targets.values()))
    for side in sorted(weights, key=weights.get, reverse=True):
        addition = min(remaining, capacities[side]-torque_targets[side])
        torque_targets[side] += addition
        remaining -= addition
    requests = {}
    for side in ('front', 'rear'):
        normal, normal_moment, tangent_moment, ratio, capacity, compression_limit, measured = geometry[side]
        wanted = torque_targets[side]
        if wanted == 0.:
            requests[side] = recovery_requests.get(side, np.zeros(3))
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
        self.strength=None
        self.strength_coordinates={}
        if config.joint_strength_path is not None:
            from bike_sim.physics.joint_strength import (load_strength_coordinates,
                load_strength_profile)
            from bike_sim.physics.rider_envelope import load_joint_envelopes
            expected=tuple(self.joints)
            self.strength=load_strength_profile(config.joint_strength_path,expected,
                require_verified=False)
            self.strength_coordinates=load_strength_coordinates(
                config.joint_strength_path,expected)
            if config.joint_envelope_path is not None:
                # The strength file and the envelope file must agree on the
                # q->anatomical mapping, or a curve could silently bound the
                # wrong anatomical direction.
                envelopes=load_joint_envelopes(config.joint_envelope_path)
                for name,coordinate in self.strength_coordinates.items():
                    if name in envelopes:
                        envelope=envelopes[name]
                        if (envelope.direction!=coordinate.direction
                                or not np.isclose(envelope.neutral_anatomical_rad,
                                    coordinate.neutral_anatomical_rad)):
                            raise ValueError(
                                f'{name}: strength coordinate disagrees with the joint envelope')
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
                eq=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_EQUALITY,'connect_grip_'+side)
                if eq<0:raise ValueError('missing compiled grip connect equality')
                self._weld_grip_offset[side] = model.eq_data[eq,3:6].copy()
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
        self._allocation_setup(model)
        self.allocation_diagnostics={}

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
        # The pedaling program modulates foot pitch with crank phase. This is
        # a bounded ankle wish, applied before the joint-range clip so it can
        # never exceed the anatomical envelope. `data` is the target state
        # (current, future or previous), so each candidate gets its own
        # phase-consistent offset.
        rotation = data.xmat[self.crank].reshape(3,3)
        crank_phase = atan2(-rotation[2,0],rotation[0,0])
        side_offset = 0. if side == 'front' else pi
        targets[2] += pedal_intent(crank_phase+side_offset,
            abs(float(data.qvel[self.crank_spin_dof])),
            ankle_amplitude_rad=self.config.pedal_ankle_amplitude_rad,
            scrape_fraction=self.config.pedal_scrape_fraction).ankle_offset_rad
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
        self._last_branch = None
        self._last_solution = None
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
                'rider_positive_power_w':0.,'rider_passive_power_w':0.,'rider_activation_saturated':False,
                'rider_strength_limited':(),'rider_strength_violations':()}
            self.last_terms = {name:{'posture_nm':0.,'pedaling_nm':0.,'command_nm':0.,'saturated':False} for name in self.joints}
            self.allocation_diagnostics={'feasible':True,'violation':0.,
                'invalid_controller':False}
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
                ripple=cfg.pedal_torque_ripple, return_foot_preload_n=cfg.return_foot_preload_n,
                preload_sides=tuple(side for side in ('front', 'rear')
                                    if self._active_recovery[side].stage == 'none'))
        # Both feet keep their support objective for the whole cycle: the
        # return foot rides through its backstroke loaded at least the
        # declared minimum instead of being lifted by a control-mode switch.
        # Recovery (a genuinely lost contact) still overrides everything.
        for side,offset in (('front',0.),('rear',pi)):
            load=max(float(loads.get(side,0.)),0.)
            fraction=min(load/cfg.stance_blend_load_n,1.)
            blends[side]=fraction*fraction*(3.-2.*fraction)
            stance[side]=(command.mean_crank_torque_nm==0. or pedaling_weights.get(side, 0.) > 1e-6)
            recovering = self._active_recovery[side].stage != 'none'
            if recovering:
                stance[side] = False
                blends[side] = 0.
            geom=self.pedal_geoms[side]
            _,normal,tangent=_upper_box_face(data.geom_xpos[geom],
                data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])
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
                # In its backstroke the foot may scrape with at most the
                # declared fraction of its friction cone; the downstroke foot
                # keeps the full budget. This is an intent bound, not a
                # physical limit -- the QP still enforces the whole cone.
                if pedaling_weights.get(side,0.) < .5:
                    t_cap = cfg.pedal_scrape_fraction*cfg.foot_mu*load
                    t_part = float(requests[side]@tangent)
                    requests[side] = requests[side]+(min(max(t_part,-t_cap),t_cap)-t_part)*tangent
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
        if command.mean_crank_torque_nm > 0. and command.crank_target_rate_rad_s > 0.:
            # Cadence is a limb velocity wish in the IK/PD objective. Only
            # muscle torques act; the physical crank velocity is never set.
            target_state = self._coasting_target_state(model,data,command,follow_motion=False)
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
            # No separate swing-lift mode: the return foot keeps its sole
            # pressed to the pedal through the backstroke at the same depth
            # and shear law as the stance foot. Only a real recovery event
            # lifts it, through _active_recovery, not a control switch.
            clearance=0.
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
        names = tuple(self.joints)
        requested = {}
        pedaling_of = {}
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
            requested[name]=requested_torque
            pedaling_of[name]=pedaling
        from bike_sim.sim.ride.rider_effort import activation_target
        tau_intent,activation_gain = activation_target(self,np.asarray([requested[n] for n in names]),
            dt_s=float(model.opt.timestep) if dt_s is None else dt_s,steady_state=steady_state)
        pedaling_part = np.asarray([pedaling_of.get(n,0.) for n in names])
        # The allocation's wrench intent is the preferred on-rider share of the
        # support split: preferred, never authoritative. Pedal normals are the
        # only friction-cone axes; grips use their pull/press branch instead.
        f_intent=[]
        normals_alloc={}
        for name in self._alloc_attachments:
            if name=='saddle':
                vec=-np.asarray(support_forces['saddle'],dtype=float)
            elif name in ('front','rear'):
                vec=-np.asarray(support_forces[name],dtype=float)
                _,n3,_=_upper_box_face(data.geom_xpos[self.pedal_geoms[name]],
                    data.geom_xmat[self.pedal_geoms[name]].reshape(3,3),model.geom_size[self.pedal_geoms[name]])
                normals_alloc[name]=np.array([n3[0],n3[2]])
            else:
                vec=-np.asarray(support_forces['grip'],dtype=float)/2.
            f_intent.extend((float(vec[0]),float(vec[2])))
        tau_alloc = None; alloc_diagnostics = {}
        for effort_scale in (1.,.5,.25):
            intent = tau_intent-(1.-effort_scale)*activation_gain*pedaling_part
            tau_alloc,alloc_diagnostics = self.allocate(model,data,intent,
                np.asarray(f_intent,float),normals_alloc,effort_scale=effort_scale,
                estimates=estimates,dt_s=dt_s,steady_state=steady_state)
            if alloc_diagnostics['feasible']:
                break
        alloc_diagnostics['invalid_controller']=not alloc_diagnostics['feasible']
        alloc_diagnostics.update(solution_qpos=data.qpos.copy(), solution_qvel=data.qvel.copy(),
                                 solution_time_s=float(data.time))
        self.allocation_diagnostics=alloc_diagnostics
        for index,name in enumerate(names):
            _, va, _ = self.joints[name]
            requested_torque=requested[name]
            command_torque=float(tau_alloc[index])
            scale = command_torque/requested_torque if requested_torque != 0 else 0.
            result[name] = command_torque
            hip_posture = -.5*pitch_request if name.startswith('rider_hip_') else 0.
            self.last_terms[name] = {'requested_nm':requested_torque,
                'posture_nm':(requested_torque-pedaling_part[index])*scale,
                'pedaling_nm':pedaling_part[index]*scale,
                'command_nm':command_torque,
                'support_nm':float(support[va])*scale,
                'tracking_nm':float(tracking_inertia[va])*scale,
                'hip_posture_nm':hip_posture*scale, 'saturated':scale!=1.}
        from bike_sim.sim.ride.rider_effort import finalize_effort
        return finalize_effort(self,data,result,advance=advance,
            dt_s=float(model.opt.timestep) if dt_s is None else dt_s,steady_state=steady_state)

    def strength_capacity(self, name, angle_rad, velocity_rad_s, torque_nm):
        """Directional isometric*Hill bound for one joint torque, or inf."""
        if self.strength is None or torque_nm == 0.:
            return float('inf')
        from bike_sim.physics.joint_strength import directional_capacity
        direction = 1 if torque_nm > 0. else -1
        curve = self.strength[name][direction]
        # The soft ROM envelope permits small excursions past the declared
        # range; outside the documented knots the edge capacity applies --
        # never an extrapolation, never a crash mid-episode.
        angle = min(max(angle_rad, curve.angles_rad[0]), curve.angles_rad[-1])
        return directional_capacity(curve, angle, velocity_rad_s, direction)

    def anatomical_joint_angle(self, name, qpos_value):
        """Anatomical angle through the strength profile's own convention."""
        from bike_sim.physics.rider_envelope import anatomical_angle
        return anatomical_angle(self.strength_coordinates[name], qpos_value)

    def strength_limited(self, torques, qpos, qvel):
        """Clip active torques to each joint's directional capacity."""
        if self.strength is None:
            return np.asarray(torques, float), ()
        clipped = np.asarray(torques, float).copy()
        limited = []
        for index, (name, (qa, dof, _)) in enumerate(self.joints.items()):
            capacity = self.strength_capacity(name,
                self.anatomical_joint_angle(name, qpos[qa]), qvel[dof], clipped[index])
            if abs(clipped[index]) > capacity:
                clipped[index] = np.copysign(capacity, clipped[index])
                limited.append(name)
        return clipped, tuple(limited)

    def strength_violations(self, active_torques, qpos, qvel):
        """Delivered torques exceeding the directional capacity at a state."""
        if self.strength is None:
            return ()
        violated = []
        for name, (qa, dof, _) in self.joints.items():
            torque = active_torques.get(name, 0.)
            capacity = self.strength_capacity(name,
                self.anatomical_joint_angle(name, qpos[qa]), qvel[dof], torque)
            if abs(torque) > capacity * (1. + 1e-6) + 1e-9:
                violated.append(name)
        return tuple(violated)

    def write(self,data,torques):
        if set(torques) != set(self.joints):
            raise ValueError('incomplete rider actuator command')
        values = {name:scalar(torque,'rider torque') for name,torque in torques.items()}
        for name,torque in values.items():
            _, _, aid = self.joints[name]
            # The actuator is a pure motor: ctrl is the muscle torque itself.
            # Passive damping lives on the DOF, so a disabled command removes
            # only active force; tissue damping still acts physically.
            data.ctrl[aid] = torque if self.command_enabled else 0.

    def _allocation_setup(self, model):
        """Cache the DOF split and per-attachment metadata for the QP."""
        import mujoco
        hinge = [self.joints[n][1] for n in self.joints]
        root = [int(model.joint(n).dofadr[0]) for n in
                ('rider_root_x','rider_root_z','rider_root_pitch')]
        self._rider_dofs = np.asarray(root+hinge, dtype=np.intp)
        self._bike_dofs = np.asarray([d for d in range(model.nv)
                                      if d not in set(self._rider_dofs)], dtype=np.intp)
        self._alloc_attachments = {}
        site_of = {'saddle': 'site_rider_saddle',
                   'front': 'site_rider_sole_front', 'rear': 'site_rider_sole_rear'}
        for name, eq_name in (('saddle','weld_saddle'),
                              ('front','weld_foot_front'),('rear','weld_foot_rear'),
                              ('grip_left','connect_grip_left'),
                              ('grip_right','connect_grip_right')):
            eq = int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_EQUALITY,eq_name))
            if eq < 0 and name == 'saddle':
                # 'pin' saddle compiles to connect_saddle instead.
                eq = int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_EQUALITY,
                                           'connect_saddle'))
            if eq >= 0:
                obj1, obj2 = int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])
                self._alloc_attachments[name] = {
                    'eq': eq,
                    'weld': int(model.eq_type[eq]) == int(mujoco.mjtEq.mjEQ_WELD),
                    'rider_body': obj1, 'other_body': obj2,
                    'anchor_local': np.asarray(model.eq_data[eq][:3], dtype=float).copy()}
                continue
            if name in site_of:
                # A bare pad still applies a real wrench: keep an uncoupled
                # attachment row at the contact site with the unilateral cone
                # and the declared pad capacity, and no rigid closure.
                site_id = int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,
                                              site_of[name]))
                body = int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,
                    'rider_pelvis' if name == 'saddle' else f'rider_foot_{name}'))
                if site_id < 0 or body < 0:
                    continue
                self._alloc_attachments[name] = {
                    'eq': -1, 'weld': False, 'site': site_of[name],
                    'site_id': site_id, 'rider_body': body,
                    'anchor_local': np.zeros(3)}
        self._jacp = np.zeros((3,model.nv)); self._jacr = np.zeros((3,model.nv))
        self._jdp = np.zeros((3,model.nv)); self._jdr = np.zeros((3,model.nv))
        self._fullM = np.zeros((model.nv,model.nv))

    def _attachment_frames(self, model, data):
        """Current closure Jacobians and on-rider wrench Jacobians.

        Wrench rows are the x/z force-on-rider block of the rider-body
        Jacobian at each attach point. Closure rows are the relative
        Jacobian ``J_rider - J_bike`` pairs (position rows for every coupled
        attachment, rotation rows for welds) alongside their Jdot mates.
        Returns ``({name: (2,nv)}, [(jac_row, jacdot_row)])``.
        """
        import mujoco
        wrench = {}
        closures = []
        for name, meta in self._alloc_attachments.items():
            b1 = meta['rider_body']
            if meta['eq'] < 0:
                # Bare pad contact: the wrench acts at the declared contact
                # site; there is no rigid closure to bound.
                point = np.asarray(data.site_xpos[meta['site_id']])
                mujoco.mj_jac(model,data,self._jacp,self._jacr,point,b1)
                wrench[name] = self._jacp.copy()[[0,2],:]
                continue
            b2 = meta['other_body']
            # eq_data[:3] is the anchor in the rider-body frame for both weld
            # and connect rows; unanchored welds store zeros, i.e. the origin.
            point = (np.asarray(data.xpos[b1])
                     + data.xmat[b1].reshape(3,3) @ meta['anchor_local'])
            mujoco.mj_jac(model,data,self._jacp,self._jacr,point,b1)
            jacp1, jacr1 = self._jacp.copy(), self._jacr.copy()
            mujoco.mj_jac(model,data,self._jacp,self._jacr,point,b2)
            jacp2, jacr2 = self._jacp.copy(), self._jacr.copy()
            mujoco.mj_jacDot(model,data,self._jdp,self._jdr,point,b1)
            jdp1, jdr1 = self._jdp.copy(), self._jdr.copy()
            mujoco.mj_jacDot(model,data,self._jdp,self._jdr,point,b2)
            rows = [jacp1 - jacp2]
            dot_rows = [jdp1 - self._jdp]
            if meta['weld']:
                rows.append(jacr1 - jacr2)
                dot_rows.append(jdr1 - self._jdr)
            closures.extend(zip(np.vstack(rows), np.vstack(dot_rows)))
            # The wrench acts on the rider body only; the relative Jacobian is
            # for the closure rows, not for J^T f in the dynamics balance.
            wrench[name] = jacp1[[0,2],:]
        return wrench, closures

    def allocate(self, model, data, tau_intent, f_intent, normals, *,
                 effort_scale=1., estimates=None,dt_s=None,steady_state=False):
        """Solve z=[qddot_r,tau,f_attach,p] nearest the intent under dynamics.

        qddot over the rider DOFs, tau the 11 muscle torques, f the planar
        attachment wrenches on the rider, p the per-joint positive-work
        epigraph. Closures bound welded point accelerations to the estimated
        base acceleration window; budgets bound every wrench; the power cap
        is the engineering budget.
        """
        import mujoco
        from bike_sim.physics.rider_allocation import (
            allocate_effort, grip_constraints, inverse_dynamics_rows, Allocation, _residual)
        cfg = self.config
        if not hasattr(self,'_rider_dofs'):
            self._allocation_setup(model)
        if all(meta['eq'] >= 0 for meta in self._alloc_attachments.values()):
            from bike_sim.sim.ride.rider_response_allocation import allocate_response
            return allocate_response(self,model,data,tau_intent,effort_scale=effort_scale,
                                     dt_s=dt_s,steady_state=steady_state)
        rd, bd = self._rider_dofs, self._bike_dofs
        names = tuple(self.joints)
        n_r, n_t = len(rd), len(names)
        attach = tuple(self._alloc_attachments)
        n_f = 2*len(attach)
        qacc_b = np.asarray(data.qacc[bd], dtype=float)
        mujoco.mj_fullM(model,data,self._fullM)
        mass_r = self._fullM[np.ix_(rd,rd)]
        bias = np.asarray(data.qfrc_bias[rd]) + self._fullM[np.ix_(rd,bd)] @ qacc_b
        known = np.zeros(model.nv)
        known[self._envelope_dof_adrs] += self.envelope_forces(model,data)[0][self._envelope_dof_adrs]
        hinge_dofs = np.asarray([self.joints[n][1] for n in names], dtype=np.intp)
        known[hinge_dofs] -= cfg.joint_kd_nms_rad*data.qvel[hinge_dofs]
        wrench_rows, closures = self._attachment_frames(model,data)
        jac_att = (np.vstack([wrench_rows[n][:,rd] for n in attach])
                   if attach else np.zeros((0,n_r)))
        actuation = np.zeros((n_r,n_t))
        for i,(name,(qa,dof,aid)) in enumerate(self.joints.items()):
            actuation[int(np.flatnonzero(rd==dof)[0]),i] = 1.
        a_dyn,b_dyn = inverse_dynamics_rows(mass_r,actuation,jac_att,bias,known[rd])
        # z = [qddot_r(n_r), tau(n_t), f(n_f), p(n_t)]
        base_size = n_r+n_t+n_f+n_t
        activation_enabled=cfg.activation_tau_s>0. and not steady_state
        n_z = base_size+(n_t if activation_enabled else 0)
        i_q = slice(0,n_r); i_t = slice(n_r,n_r+n_t)
        i_f = slice(n_r+n_t,n_r+n_t+n_f); i_p = slice(n_r+n_t+n_f,base_size)
        excitation_indices=np.arange(base_size,n_z);torque_indices=np.arange(n_r,n_r+n_t)
        s_q,s_t,s_f,s_p = 500.,50.,300.,450.
        scales = np.concatenate([np.full(n_r,s_q),np.full(n_t,s_t),
                                 np.full(n_f,s_f),np.full(n_t,s_p),
                                 np.full(n_t if activation_enabled else 0,s_t)])
        speed = np.asarray([data.qvel[self.joints[n][1]] for n in names])
        angle = np.asarray([self.anatomical_joint_angle(n,data.qpos[self.joints[n][0]])
                            for n in names]) if self.strength is not None else None
        lo = np.full(n_z,-np.inf); hi = np.full(n_z,np.inf)
        for i,name in enumerate(names):
            v = speed[i]
            if self.strength is None:
                cap_pos = cap_neg = cfg.joint_limit_nm
            else:
                cap_pos = min(cfg.joint_limit_nm,self.strength_capacity(name,angle[i],speed[i],1.))
                cap_neg = min(cfg.joint_limit_nm,self.strength_capacity(name,angle[i],speed[i],-1.))
            # Positive-power torque (the accelerating direction) keeps the
            # per-joint speed/power budget; braking stays bounded by strength.
            if v > 0.:
                cap_pos = min(cap_pos,cfg.joint_power_limit_w/v)
                if v >= cfg.joint_speed_limit_rad_s: cap_pos = 0.
            elif v < 0.:
                cap_neg = min(cap_neg,-cfg.joint_power_limit_w/v)
                if -v >= cfg.joint_speed_limit_rad_s: cap_neg = 0.
            lo[n_r+i],hi[n_r+i] = -cap_neg,cap_pos
            aid=self.joints[name][2]
            if model.actuator_forcelimited[aid]:
                lo[n_r+i]=max(lo[n_r+i],model.actuator_forcerange[aid,0])
                hi[n_r+i]=min(hi[n_r+i],model.actuator_forcerange[aid,1])
            if model.actuator_ctrllimited[aid]:
                lo[n_r+i]=max(lo[n_r+i],model.actuator_ctrlrange[aid,0])
                hi[n_r+i]=min(hi[n_r+i],model.actuator_ctrlrange[aid,1])
        lo[i_p] = 0.
        cap = cfg.active_positive_power_limit_w
        hi[i_p] = cap if cap is not None else np.inf
        # Closure rows bound the rider's attachment acceleration to the
        # estimated base window: |J_r qddot_r + J_b qacc_b + Jdot qvel| <= tol.
        g_rows = []; g_hi = []
        for jrel, jdot in closures:
            rhs = -(jrel[bd]@qacc_b + jdot@data.qvel)
            tol = 50.+.5*abs(rhs)
            row = np.zeros(n_z); row[i_q] = jrel[rd]
            g_rows.append(row); g_hi.append(rhs+tol)
            g_rows.append(-row); g_hi.append(tol-rhs)
        # Every foot/saddle support obeys the unilateral physical cone even
        # when a computational equality enforces attachment geometry.
        # Grips take their pull/press branch instead of a friction cone.
        weight = self.rider_mass*float(np.linalg.norm(model.opt.gravity))
        for k,name in enumerate(attach):
            if name.startswith('grip'):
                continue
            n_hat = normals.get(name)
            if n_hat is None:
                n_hat = np.array([0.,1.])  # world +z on the planar (Fx,Fz)
            t_hat = np.array([-n_hat[1],n_hat[0]])
            mu = dict(saddle=cfg.saddle_mu,front=cfg.foot_mu,rear=cfg.foot_mu).get(name,cfg.support_mu)
            i0 = i_f.start+2*k
            coupled = self._alloc_attachments[name]['eq'] >= 0
            # -mu*fn <= ft <= mu*fn and the declared normal floor.
            for s_t_ in (1.,-1.):
                row = np.zeros(n_z)
                row[i0] = s_t_*t_hat[0]-mu*n_hat[0]
                row[i0+1] = s_t_*t_hat[1]-mu*n_hat[1]
                g_rows.append(row); g_hi.append(0.)
            row = np.zeros(n_z); row[i0] = -n_hat[0]; row[i0+1] = -n_hat[1]
            g_rows.append(row)
            predicted = 0. if estimates is None else float(estimates.get(name,(0.,0.))[0])
            # A pedal that is in contact keeps at least the declared minimum
            # normal through the whole crank cycle; a genuinely airborne or
            # recovering foot has no such floor, so infeasibility is reported
            # only when physics truly cannot meet the request.
            pad_lo = cfg.saddle_reserve_weight_fraction*weight if name=='saddle' else 0.
            if name in ('front','rear') and (coupled or (predicted > 0.
                    and self._active_recovery[name].stage == 'none')):
                pad_lo = cfg.pedal_min_normal_n
            g_hi.append(-pad_lo)
            if coupled:
                continue
            row = np.zeros(n_z); row[i0] = n_hat[0]; row[i0+1] = n_hat[1]
            g_rows.append(row)
            g_hi.append(predicted+.005*cfg.support_k_n_m)
        # Positive-work epigraph: p_j >= tau_j*v_j and p_j >= 0; sum(p) <= cap.
        for i in range(n_t):
            row = np.zeros(n_z); row[n_r+i] = speed[i]; row[i_p.start+i] = -1.
            g_rows.append(row); g_hi.append(0.)
        if cap is not None:
            row = np.zeros(n_z); row[i_p] = 1.
            g_rows.append(row); g_hi.append(cap)
        aeq = np.zeros((0,n_z)); beq = np.zeros(0)
        dyn = np.zeros((n_r,n_z)); dyn[:,i_q] = a_dyn[:,:n_r]
        dyn[:,i_t] = a_dyn[:,n_r:n_r+n_t]; dyn[:,i_f] = a_dyn[:,n_r+n_t:]
        aeq = np.vstack([dyn]); beq = b_dyn
        activation=None
        activated_target=np.asarray(tau_intent,float)
        excitation_target=np.zeros(0)
        if activation_enabled:
            from bike_sim.sim.ride.rider_activation_allocation import ActivationLaw
            lo[excitation_indices]=lo[i_t];hi[excitation_indices]=hi[i_t]
            activation=ActivationLaw(self.active_state,speed,lo[i_t],hi[i_t],
                dt_s=float(model.opt.timestep) if dt_s is None else dt_s,
                tau_s=cfg.activation_tau_s,power_limit=cap)
            excitation_target=np.clip((activated_target-(1.-activation.gain)*self.active_state)/activation.gain,
                                      lo[i_t],hi[i_t])
            activated_target=activation.delivered(excitation_target)
        target = np.concatenate([np.zeros(n_r),activated_target,
                                 np.asarray(f_intent,float),np.zeros(n_t),excitation_target])
        g = np.asarray(g_rows,float).reshape(-1,n_z); h = np.asarray(g_hi,float)
        # Dimensionless x = z/scale.
        S = np.diag(scales)
        aeq_x = aeq@S; beq_x = beq.copy()
        g_x = g@S; h_x = h.copy()
        lo_x = lo/scales; hi_x = hi/scales
        # Row-normalize equalities for conditioning.
        norms = np.maximum(np.linalg.norm(aeq_x,axis=1),1e-9)
        aeq_x = aeq_x/norms[:,None]; beq_x = beq_x/norms
        scaled_target = target/scales
        def solve(branch, x0):
            pull_left, pull_right = branch
            extra = []
            if activation is not None:
                extra.append(activation.constraint(torque_indices,excitation_indices,scales))
            for side,pulling in (('grip_left',pull_left),('grip_right',pull_right)):
                if side not in self._alloc_attachments:
                    continue
                from bike_sim.sim.ride.rider_response_allocation import allocation_support_point,allocation_grip_constraints
                origin = allocation_support_point(model,data,self,side,self._alloc_attachments[side])
                pelvis = np.asarray(data.xpos[self.pelvis],dtype=float)
                direction = pelvis-origin; direction[1]=0.
                direction = direction[[0,2]]/max(np.linalg.norm(direction[[0,2]]),1e-9)
                k = attach.index(side)
                force_map = np.zeros((2,n_z))
                force_map[0,i_f.start+2*k] = s_f; force_map[1,i_f.start+2*k+1] = s_f
                extra.extend(allocation_grip_constraints(force_map,direction,pulling=pulling,
                                              limit_n=cfg.grip_pull_per_hand_n))
            result=allocate_effort(scaled_target,aeq_x,beq_x,g_x,h_x,lo_x,hi_x,
                                   extra_constraints=extra,x0=x0)
            if activation is not None:
                projected=activation.project(result.solution,torque_indices,excitation_indices,scales)
                error=_residual(projected,aeq_x,beq_x,g_x,h_x,extra,lo_x,hi_x)
                result=Allocation(projected,error<=1e-7,error)
            return result
        result, branch = self._select_allocation_branch(scaled_target, solve)
        z = result.solution*scales
        excitation=z[i_t].copy() if activation is None else z[excitation_indices].copy()
        activation_state=z[i_t].copy() if activation is None else activation.latent(excitation)
        return z[i_t], {'feasible':bool(result.feasible),'violation':float(result.violation),
                        'saddle_normal_lower_bound_n':cfg.saddle_reserve_weight_fraction*weight,
                        'grip_branch':branch,'effort_scale':effort_scale,
                        'solution_qddot':z[i_q],'solution_wrenches':z[i_f],
                        'solution_tau':z[i_t].copy(),'solution_power':z[i_p],
                        'solution_excitation_nm':excitation,'solution_activation_state_nm':activation_state,
                        'solution_active_lower_nm':lo[i_t].copy(),'solution_active_upper_nm':hi[i_t].copy(),
                        'solution_base_qacc':qacc_b.copy()}

    def _select_allocation_branch(self, target, solve):
        """Try remembered mode first; retain full ranking on cold/fallback solves."""
        branches = [(p,q) for p in (True,False) for q in (True,False)]
        remembered = self._last_branch
        if remembered in branches:
            branches.remove(remembered)
            branches.insert(0, remembered)
        x0 = self._last_solution
        if x0 is not None and x0.shape != target.shape:
            x0 = None
        best = None
        for branch in branches:
            result = solve(branch, x0)
            error = float(np.dot(result.solution-target, result.solution-target))
            key = ((result.feasible, -error) if result.feasible
                   else (result.feasible, -result.violation))
            if best is None or key > best[0]:
                best = key, result, branch
            if result.feasible and branch == remembered:
                break
        _, result, branch = best
        if result.feasible:
            self._last_branch = branch
            self._last_solution = result.solution.copy()
        return result, branch
