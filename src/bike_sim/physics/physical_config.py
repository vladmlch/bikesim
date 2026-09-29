"""Immutable, explicit SI parameters for the physical force components.

All supplied defaults describe synthetic rigs. Native solver settings are never
interpreted as tire stiffness, and neither a component name nor a low residual
upgrades these parameters to a measured/calibrated model.
"""
from dataclasses import dataclass, field
from bike_sim.physics.checks import scalar
from bike_sim.physics.tire import TireSpec
from bike_sim.physics.chain import DrivetrainSpecs


def _material():
    return TireSpec(130000.,800.,200000.,'synthetic',(0.,2000.))


@dataclass(frozen=True)
class TireParameters:
    material: TireSpec = field(default_factory=_material)
    tangent_k_n_m: float = 20000.
    mu: float = .8
    relaxation_length_m: float = .2

    def __post_init__(self):
        if not isinstance(self.material,TireSpec):
            raise ValueError('tire material must be a TireSpec')
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

    def __post_init__(self):
        if self.backend not in ('native_reference','compliant_2d'):
            raise ValueError('unknown physical tire backend')
        if self.surface_mode not in ('configured', 'track'):
            raise ValueError('surface_mode must be configured or track')
        if self.surface_mode == 'track' and self.backend != 'compliant_2d':
            raise ValueError('track material resolution requires compliant_2d tires')
        if not isinstance(self.front,TireParameters) or not isinstance(self.rear,TireParameters):
            raise ValueError('front and rear tires need explicit TireParameters')
        scalar(self.significant_delta_m,'significant penetration',minimum=0)
        fraction = scalar(self.significance_fraction,'significance fraction',minimum=0)
        angle = scalar(self.distinct_normal_deg,'distinct normal angle',positive=True)
        if fraction > 1 or angle >= 180:
            raise ValueError('invalid multi-support thresholds')


@dataclass(frozen=True)
class AssistConfig:
    gain: float = 2.
    max_torque: float = 80.
    max_power: float = 500.
    tau: float = .05
    slew: float = 400.
    stop_delay: float = .1
    on_rpm: float = 10.
    off_rpm: float = 5.
    cutoff_mps: float = 25/3.6
    taper_width_mps: float = 2/3.6
    torque_curve: tuple[tuple[float,float],...] | None = None

    def __post_init__(self):
        from dataclasses import asdict
        from bike_sim.physics.motor import AssistController
        if self.torque_curve is not None:
            object.__setattr__(self,'torque_curve',tuple(tuple(p) for p in self.torque_curve))
        AssistController(**asdict(self))


@dataclass(frozen=True)
class BatteryConfig:
    energy_j: float = 1800000.
    copper_w_per_nm2: float = .02
    speed_w_per_rad_s2: float = 0.
    idle_w: float = 5.

    def __post_init__(self):
        for key in self.__dataclass_fields__:
            scalar(getattr(self,key),key,minimum=0)


@dataclass(frozen=True)
class PhysicalDriveConfig:
    gearing: DrivetrainSpecs = field(default_factory=DrivetrainSpecs)
    human_torque_nm: float = 0.
    torque_ripple: float = .35
    crank_phase_rad: float = 0.
    chain_k_n_m: float = 200000.
    chain_c_ns_m: float = 10.
    freehub_k_nm_rad: float = 1000.
    freehub_c_nms_rad: float = .5
    bearing_c_nms_rad: float = .03
    brake_ceiling_nm: float = 200.
    assist: AssistConfig = field(default_factory=AssistConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)

    def __post_init__(self):
        if not isinstance(self.gearing,DrivetrainSpecs) or not isinstance(self.assist,AssistConfig) or not isinstance(self.battery,BatteryConfig):
            raise ValueError('invalid drivetrain configuration object')
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
    stance_blend_load_n: float = 50.
    posture_pitch_k_nm_rad: float = 600.
    posture_pitch_d_nms_rad: float = 60.
    posture_pitch_limit_nm: float = 60.
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
    arm_reach_fraction: float = .92
    joint_kp_nm_rad: float = 600.
    joint_kd_nms_rad: float = 15.
    joint_limit_nm: float = 100.
    joint_speed_limit_rad_s: float = 20.
    joint_power_limit_w: float = 250.

    def __post_init__(self):
        if not 0 < scalar(self.arm_reach_fraction,'arm reach fraction',positive=True) < 1:
            raise ValueError('arm reach fraction must lie strictly between zero and one')
        if self.pedal_support_fraction + self.bar_support_fraction >= 1.:
            raise ValueError('postural support fractions must leave a saddle share')
        for key in self.__dataclass_fields__:
            scalar(getattr(self,key),key,minimum=0)
        for key in ('support_pad_radius_m','stance_blend_load_n','support_k_n_m','support_tangent_k_n_m','support_length_m','grip_k_n_m','grip_release_distance_m','joint_speed_limit_rad_s'):
            scalar(getattr(self,key),key,positive=True)
