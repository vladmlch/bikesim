"""Immutable, explicit SI parameters for the physical force components.

All supplied defaults describe synthetic rigs. Native solver settings are never
interpreted as tire stiffness, and neither a component name nor a low residual
upgrades these parameters to a measured/calibrated model.
"""
from dataclasses import dataclass, field
from math import pi, radians
from bike_sim.physics.checks import scalar
from bike_sim.physics.tire import TireSpec
from bike_sim.physics.tire_curve import TabulatedTireSpec
from bike_sim.physics.distributed_tire import DistributedTireConfig
from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.drivetrain import CASSETTE_12S_TEETH


def _material():
    return TireSpec(130000.,800.,200000.,'synthetic',(0.,2000.))


@dataclass(frozen=True)
class TireParameters:
    material: TireSpec | TabulatedTireSpec = field(default_factory=_material)
    tangent_k_n_m: float = 20000.
    mu: float = .8
    relaxation_length_m: float = .2

    def __post_init__(self):
        if not isinstance(self.material, (TireSpec, TabulatedTireSpec)):
            raise ValueError('tire material must be a TireSpec or TabulatedTireSpec')
        for key in ('tangent_k_n_m','relaxation_length_m'):
            scalar(getattr(self,key),key,positive=True)
        scalar(self.mu,'tire friction',minimum=0)


@dataclass(frozen=True)
class TireBackendConfig:
    backend: str = 'native_reference'
    front: TireParameters = field(default_factory=TireParameters)
    rear: TireParameters = field(default_factory=TireParameters)
    significant_delta_m: float = .0001
    significance_fraction: float = .05
    distinct_normal_deg: float = 20.
    surface_mode: str = 'configured'
    distributed: DistributedTireConfig = field(default_factory=DistributedTireConfig)

    def __post_init__(self):
        if self.backend not in ('native_reference','compliant_2d','distributed_2d_reference'):
            raise ValueError('unknown physical tire backend')
        if self.surface_mode not in ('configured', 'track'):
            raise ValueError('surface_mode must be configured or track')
        if self.surface_mode == 'track' and self.backend not in ('compliant_2d','distributed_2d_reference'):
            raise ValueError('track material resolution requires compliant_2d tires')
        if not isinstance(self.distributed,DistributedTireConfig):
            raise ValueError('distributed tire configuration must be explicit')
        if not isinstance(self.front,TireParameters) or not isinstance(self.rear,TireParameters):
            raise ValueError('front and rear tires need explicit TireParameters')
        scalar(self.significant_delta_m,'significant penetration',minimum=0)
        fraction = scalar(self.significance_fraction,'significance fraction',minimum=0)
        angle = scalar(self.distinct_normal_deg,'distinct normal angle',positive=True)
        if fraction > 1 or angle >= 180:
            raise ValueError('invalid multi-support thresholds')


@dataclass(frozen=True)
class AssistConfig:
    # A named profile owns torque/power/lag/cutoff/taper/gate; explicit values
    # describe a synthetic motor when no profile is selected.
    profile: str | None = None
    mode: str = 'turbo'
    gain: float = 2.
    max_torque: float = 80.
    max_power: float = 500.
    tau: float = .05
    slew: float = 400.
    engage_torque_nm: float = 4.
    gate_min_crank_rad_s: float = radians(5.)
    cutoff_mps: float = 25/3.6
    taper_width_mps: float = 2/3.6
    torque_curve: tuple[tuple[float,float],...] | None = None

    def __post_init__(self):
        from dataclasses import asdict
        from bike_sim.physics.motor import AssistController
        if self.profile is not None and not isinstance(self.profile, str):
            raise ValueError('assist profile must be a registered name')
        if self.torque_curve is not None:
            object.__setattr__(self,'torque_curve',tuple(tuple(p) for p in self.torque_curve))
        AssistController(**asdict(self))

    @property
    def effective_max_torque(self):
        """Actuator limit from the selected profile or the synthetic config."""
        from bike_sim.physics.motor_profile import PROFILES
        return self.max_torque if self.profile is None else PROFILES[self.profile].peak_torque_nm


@dataclass(frozen=True)
class BatteryConfig:
    # enabled=false removes the energy store: metering still runs but torque is
    # never energy-limited and the reserve never depletes.
    enabled: bool = True
    energy_j: float = 1800000.
    copper_w_per_nm2: float = .02
    speed_w_per_rad_s2: float = 0.
    idle_w: float = 5.

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('battery enable must be a bool')
        for key in self.__dataclass_fields__:
            if key != 'enabled':
                scalar(getattr(self,key),key,minimum=0)


@dataclass(frozen=True)
class PedalingConfig:
    enabled: bool = False
    coast_above_rpm: float = 110.
    resume_below_rpm: float = 90.
    stop_time_s: float = .35
    # Coast on a sustained cadence rise, filtering the two strokes per turn.
    # Zero retains raw hysteresis.
    coast_cadence_tau_s: float = 0.
    # Low-cadence effort ramp: a rider grinding to a stall presses harder on
    # the pedal, so commanded effort rises toward a low-cadence ceiling instead
    # of fading. The ceiling is capped near what the articulated leg drive can
    # actually transmit; commanding far beyond it just deepens the pedal-stroke
    # torque oscillation. The slew bound models force development; without it a
    # 0-rpm mash appears as an impulse that rips the modelled feet off the pedals.
    mash_cadence_rpm: float = 45.
    mash_torque_nm: float = 60.
    effort_slew_nm_s: float = 300.
    # Hill-hold reflex: sustained rollback grabs the wheel brakes and holds
    # until forward motion resumes. Braking here is a physical restraint, not
    # rider intent -- it does not gate pedaling or assist.
    rollback_brake: bool = True
    rollback_engage_mps: float = .25
    rollback_release_mps: float = 0.
    rollback_demand: float = 1.

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('cadence coasting enable must be a bool')
        scalar(self.coast_above_rpm, 'coasting cadence', positive=True)
        scalar(self.resume_below_rpm, 'resume cadence', minimum=0.)
        scalar(self.stop_time_s, 'coasting stop time', positive=True)
        scalar(self.coast_cadence_tau_s, 'coast cadence tau', minimum=0.)
        if self.resume_below_rpm >= self.coast_above_rpm:
            raise ValueError('resume cadence must be below coasting cadence')
        scalar(self.mash_torque_nm, 'mash torque', minimum=0.)
        scalar(self.mash_cadence_rpm, 'mash cadence', minimum=0.)
        scalar(self.effort_slew_nm_s, 'effort slew', minimum=0.)
        if self.mash_torque_nm > 0. and self.mash_cadence_rpm <= 0.:
            raise ValueError('a nonzero mash torque needs a positive mash cadence')
        if not isinstance(self.rollback_brake, bool):
            raise ValueError('rollback brake enable must be a bool')
        engage = scalar(self.rollback_engage_mps, 'rollback engage speed', positive=True)
        release = scalar(self.rollback_release_mps, 'rollback release speed', minimum=0.)
        if release >= engage:
            raise ValueError('rollback release must be slower than engage')
        demand = scalar(self.rollback_demand, 'rollback brake demand', minimum=0.)
        if demand > 1.:
            raise ValueError('rollback brake demand must not exceed one')


@dataclass(frozen=True)
class ShiftingConfig:
    enabled: bool = False
    cassette: tuple[int, ...] = CASSETTE_12S_TEETH
    target_cadence_min_rpm: float = 65.
    target_cadence_max_rpm: float = 85.
    shift_cooldown_s: float = .4
    shift_cut_duration_s: float = .2
    torque_factor: float = .3
    cadence_smoothing_tau_s: float = .35
    upshift_slip_limit_mps: float = .5
    upshift_slip_mode: str = 'legacy_signed'

    def __post_init__(self):
        if self.upshift_slip_mode not in ('legacy_signed', 'magnitude'):
            raise ValueError('unknown upshift slip mode')
        if not isinstance(self.enabled, bool):
            raise ValueError('automatic shifting enable must be a bool')
        try:
            cassette = tuple(self.cassette)
        except TypeError as exc:
            raise ValueError('cassette needs a sequence of integer tooth counts') from exc
        if not cassette or any(type(teeth) is not int or teeth < 3 for teeth in cassette):
            raise ValueError('cassette needs integer tooth counts of at least three')
        if len(set(cassette)) != len(cassette):
            raise ValueError('cassette tooth counts must be distinct')
        object.__setattr__(self, 'cassette', tuple(sorted(cassette)))
        scalar(self.target_cadence_min_rpm, 'minimum shift cadence', positive=True)
        scalar(self.target_cadence_max_rpm, 'maximum shift cadence', positive=True)
        if self.target_cadence_max_rpm <= self.target_cadence_min_rpm:
            raise ValueError('maximum shift cadence must exceed minimum shift cadence')
        scalar(self.shift_cooldown_s, 'shift cooldown', minimum=0.)
        scalar(self.shift_cut_duration_s, 'shift torque cut duration', minimum=0.)
        scalar(self.cadence_smoothing_tau_s, 'shift cadence smoothing', minimum=0.)
        scalar(self.upshift_slip_limit_mps, 'upshift rear slip limit', minimum=0.)
        if self.shift_cut_duration_s > self.shift_cooldown_s:
            raise ValueError('shift torque cut must not outlast the cooldown')
        if scalar(self.torque_factor, 'shift torque factor', minimum=0.) > 1.:
            raise ValueError('shift torque factor must not exceed one')


@dataclass(frozen=True)
class PhysicalDriveConfig:
    gearing: DrivetrainSpecs = field(default_factory=DrivetrainSpecs)
    transmission_model: str = 'elastic_chain'
    human_torque_nm: float = 0.
    torque_ripple: float = .35
    crank_phase_rad: float = 0.
    chain_k_n_m: float = 200000.
    chain_c_ns_m: float = 10.
    freehub_k_nm_rad: float = 1000.
    freehub_c_nms_rad: float = .5
    bearing_c_nms_rad: float = .03
    brake_ceiling_nm: float = 200.
    # Legacy regression topology: crank -[one-way]-> drive_shaft with motor.
    # Kept for A/B until acceptance; the real mid-drive chainring is on crank.
    motor_clutch: bool = False
    # Motor inertia reflected to the crank coordinate, kg.m^2. Zero means a
    # stateless rotor (tau_motor >= 0 on crank); positive builds rotor->crank
    # freewheel. 0.1–0.3 is an unverified recalled order of magnitude.
    rotor_inertia_kgm2: float = 0.
    assist: AssistConfig = field(default_factory=AssistConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    pedaling: PedalingConfig = field(default_factory=PedalingConfig)
    shifting: ShiftingConfig = field(default_factory=ShiftingConfig)

    def __post_init__(self):
        if not isinstance(self.gearing,DrivetrainSpecs) or not isinstance(self.assist,AssistConfig) or not isinstance(self.battery,BatteryConfig):
            raise ValueError('invalid drivetrain configuration object')
        if not isinstance(self.pedaling, PedalingConfig):
            raise ValueError('pedaling needs an immutable PedalingConfig')
        if self.transmission_model not in ('elastic_chain', 'ideal_mid_drive', 'geometric_ideal_mid_drive'):
            raise ValueError('unknown transmission model')
        if not isinstance(self.motor_clutch, bool):
            raise ValueError('motor clutch enable must be a bool')
        scalar(self.rotor_inertia_kgm2, 'rotor_inertia_kgm2', minimum=0.)
        if self.rotor_inertia_kgm2 > 0. and self.motor_clutch:
            raise ValueError('rotor freewheel and the legacy crank clutch are exclusive')
        if self.rotor_inertia_kgm2 > 0. and self.transmission_model not in ('ideal_mid_drive','geometric_ideal_mid_drive'):
            raise ValueError('a motor rotor requires an ideal mid-drive transmission')
        if self.motor_clutch and self.transmission_model not in ('ideal_mid_drive','geometric_ideal_mid_drive'):
            raise ValueError('motor clutch requires an ideal mid-drive transmission')
        if not isinstance(self.shifting, ShiftingConfig):
            raise ValueError('shifting needs an immutable ShiftingConfig')
        if self.shifting.enabled:
            if self.transmission_model not in ('ideal_mid_drive','geometric_ideal_mid_drive'):
                raise ValueError('automatic physical shifting requires ideal_mid_drive')
            if self.gearing.rear_teeth not in self.shifting.cassette:
                raise ValueError('initial rear sprocket must belong to the cassette')
        for key in ('chain_k_n_m','freehub_k_nm_rad'):
            scalar(getattr(self,key),key,positive=True)
        for key in ('human_torque_nm','torque_ripple','chain_c_ns_m','freehub_c_nms_rad','bearing_c_nms_rad','brake_ceiling_nm'):
            scalar(getattr(self,key),key,minimum=0)
        scalar(self.crank_phase_rad,'initial crank phase')
        if self.torque_ripple >= 1:
            raise ValueError('torque ripple must be less than one')


@dataclass(frozen=True)
class ResistanceConfig:
    crr: float = .015
    rolling_taper_rad_s: float = .2
    rho_kg_m3: float = 1.2
    cda_m2: float = .5
    wind_world_mps: tuple[float,float,float] = (0.,0.,0.)
    point_body_m: tuple[float,float,float] = (0.,0.,.5)

    def __post_init__(self):
        from bike_sim.physics.checks import array
        for key in ('crr','rho_kg_m3','cda_m2'):
            scalar(getattr(self,key),key,minimum=0)
        scalar(self.rolling_taper_rad_s,'rolling taper',positive=True)
        for key in ('wind_world_mps','point_body_m'):
            value = array(getattr(self,key),key,(3,))
            if value[1] != 0:
                raise ValueError('resistance configuration must be planar')
            object.__setattr__(self,key,tuple(float(x) for x in value))


@dataclass(frozen=True)
class ArticulatedConfig:
    # Desired support distribution belongs to posture control, never anatomy.
    pedal_support_fraction: float = .33
    bar_support_fraction: float = .12
    posture_sole_depth_m: float = .003
    swing_clearance_m: float = .003  # unstrapped return foot is not a brake
    coasting_brake_d_nm_s_rad: float = 2.
    coasting_brake_limit_nm: float = 60.
    stance_blend_load_n: float = 50.
    posture_pitch_k_nm_rad: float = 600.
    posture_pitch_d_nms_rad: float = 60.
    posture_pitch_limit_nm: float = 60.
    # Synthetic internal support feedback, not a force on the floating root.
    posture_translation_k_n_m: float = 1500.
    posture_translation_d_ns_m: float = 150.
    posture_translation_limit_n: float = 400.
    saddle_patch_half_length_m: float = .045
    pedal_patch_half_length_m: float = .025
    support_pad_radius_m: float = .020  # synthetic sole contact shape
    # Synthetic finite-pad stiffness: 1000 N gives 10 mm total deflection,
    # at most 20 mm if one pressure pad carries that entire load.
    support_k_n_m: float = 100000.
    support_c_ns_m: float = 500.  # saddle interface
    # A 0.175 kg pedal has Iyy ~= 1.50e-4 kg m^2. Reusing saddle damping
    # makes its explicit edge contact unstable at 0.5 ms; keep the synthetic
    # sole material separate and validate it on the unforced edge-pad rig.
    pedal_c_ns_m: float = 100.
    support_tangent_k_n_m: float = 20000.
    support_mu: float = .8
    support_length_m: float = .1
    grip_k_n_m: float = 4000.
    grip_c_ns_m: float = 150.
    grip_release_distance_m: float = .12
    grip_pair_force_limit_n: float | None = None
    grip_capture_distance_m: float = .02
    grip_capture_speed_mps: float = .2
    arm_reach_fraction: float = .92
    joint_kp_nm_rad: float = 600.
    joint_kd_nms_rad: float = 15.
    joint_limit_nm: float = 100.
    joint_speed_limit_rad_s: float = 20.
    joint_power_limit_w: float = 250.
    joint_envelope_path: str | None = None
    joint_envelope_soft_margin_rad: float = .1
    joint_envelope_soft_k_nm_rad: float = 100.
    activation_tau_s: float = 0.
    active_positive_power_limit_w: float | None = None
    # Reference seated plant budgets (spec S3). support_mu remains for legacy
    # comparison profiles; the reference gate fixes these per-attachment
    # values instead of one shared coefficient.
    foot_mu: float = .9
    saddle_mu: float = .6
    pedal_min_normal_n: float = 20.
    saddle_reserve_weight_fraction: float = .15
    grip_pull_per_hand_n: float = 300.
    # Seated pedaling-cycle strategy knobs (R5): the ankle rocks with the
    # crank phase inside the joint envelope, and the return foot may ask for
    # up to this fraction of its friction cone while scraping through the
    # backstroke. Both are intents, never physical guarantees.
    pedal_ankle_amplitude_rad: float = .1
    pedal_scrape_fraction: float = .5
    # Two-leg waveform: mean*(1+ripple*cos(2*phase)), an engineering choice.
    pedal_torque_ripple: float = .35
    # Residual recovery-leg load on the rising flat pedal; 40–100 N is
    # an unverified engineering estimate. Zero disables this intent.
    return_foot_preload_n: float = 0.
    link_max_gap_m: float = .005
    # Bounded road preview the rider planner may see ahead of the front wheel.
    road_lookahead_m: float = 0.
    joint_strength_path: str | None = None
    pedal_attachment: str = 'flat'
    saddle_attachment: str = 'flat'
    # 'spring' is the releasable compliant grip; 'weld' pins the hands to the
    # bar permanently through a connect equality (the wrist DOF stays free, so
    # the torso can still lean over locked hands).
    grip_attachment: str = 'spring'

    def __post_init__(self):
        if not 0 < scalar(self.arm_reach_fraction,'arm reach fraction',positive=True) < 1:
            raise ValueError('arm reach fraction must lie strictly between zero and one')
        if self.pedal_support_fraction + self.bar_support_fraction >= 1.:
            raise ValueError('postural support fractions must leave a saddle share')
        if self.joint_envelope_path is not None and (not isinstance(self.joint_envelope_path,str) or not self.joint_envelope_path.strip()):
            raise ValueError('joint_envelope_path must be a nonempty path or None')
        if self.joint_strength_path is not None and (not isinstance(self.joint_strength_path,str) or not self.joint_strength_path.strip()):
            raise ValueError('joint_strength_path must be a nonempty path or None')
        for key in self.__dataclass_fields__:
            if key in ('joint_envelope_path', 'joint_strength_path',
                       'pedal_attachment', 'saddle_attachment',
                       'grip_attachment'):
                continue
            if key in ('grip_pair_force_limit_n','active_positive_power_limit_w') and getattr(self,key) is None:
                continue
            scalar(getattr(self,key),key,minimum=0)
        if self.grip_pair_force_limit_n is not None:
            scalar(self.grip_pair_force_limit_n,'pair grip force limit',positive=True)
        for key in ('support_pad_radius_m','stance_blend_load_n','support_k_n_m','support_tangent_k_n_m','support_length_m','grip_k_n_m','grip_release_distance_m','joint_speed_limit_rad_s',
                    'pedal_min_normal_n','grip_pull_per_hand_n','link_max_gap_m'):
            scalar(getattr(self,key),key,positive=True)
        if self.saddle_reserve_weight_fraction > 1.:
            raise ValueError('saddle reserve fraction must not exceed one')
        if self.pedal_torque_ripple >= 1.:
            raise ValueError('pedal torque ripple must be below one')
        if self.pedal_scrape_fraction > 1.:
            raise ValueError('pedal scrape fraction must not exceed one')
        if self.pedal_attachment not in ('flat', 'weld'):
            raise ValueError(
                f"pedal_attachment must be 'flat' or 'weld', got {self.pedal_attachment!r}")
        if self.saddle_attachment not in ('flat', 'weld', 'pin'):
            raise ValueError(
                f"saddle_attachment must be 'flat', 'weld' or 'pin', got {self.saddle_attachment!r}")
        if self.grip_attachment not in ('spring', 'weld'):
            raise ValueError(
                f"grip_attachment must be 'spring' or 'weld', got {self.grip_attachment!r}")
