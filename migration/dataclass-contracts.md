# Source data contracts

These excerpts are from the supplied repository, not new implementation code. They preserve field/default/validation authority for the C++ ports. T12/T18/T34 and each owner retain equivalent semantics; native names live in interface-contracts.md. Line numbers refer to the unmixed snapshot.


## `src/bike_sim/cli/batch.py`

### `Job` (src/bike_sim/cli/batch.py:40-45)

```python
@dataclass(frozen=True)
class Job:
    index: int
    name: str
    params: dict  # merged flag values actually passed
    argv: list
```


## `src/bike_sim/cli/ride.py`

### `LegConfig` (src/bike_sim/cli/ride.py:506-517)

```python
class LegConfig(NamedTuple):
    legs: str
    visual_pedalling: bool
```


## `src/bike_sim/config.py`

### `BikeConfig` (src/bike_sim/config.py:19-31)

```python
@dataclass
class BikeConfig:
    geometry: FrameGeometrySpecs = field(default_factory=FrameGeometrySpecs)
    suspension: SuspensionHardwareSpecs = field(default_factory=SuspensionHardwareSpecs)
    air_spring: AirSpringSpecs = field(default_factory=AirSpringSpecs)
    damper_clicks: DamperClickConfig = field(default_factory=DamperClickConfig)
    mass: BikeMassSpecs = field(default_factory=BikeMassSpecs)
    rider: RiderSpecs = field(default_factory=RiderSpecs)
    drivetrain: DrivetrainSpecs = field(default_factory=DrivetrainSpecs)
```


## `src/bike_sim/geometry/cockpit.py`

### `SaddleGeometry` (src/bike_sim/geometry/cockpit.py:59-78)

```python
@dataclass(frozen=True)
class SaddleGeometry:
    seatpost_top: np.ndarray
    saddle_pos: np.ndarray
    top_center: np.ndarray
    top_z_m: float
    height_m: float
    exposed_post_m: float
```


## `src/bike_sim/geometry/specs.py`

### `FrameGeometrySpecs` (src/bike_sim/geometry/specs.py:10-23)

```python
@dataclass
class FrameGeometrySpecs:
    reach: float = 480.0
    stack: float = 646.0
    head_angle_deg: float = 64.0
    effective_seat_angle_deg: float = 77.0
    bb_drop: float = 22.5
    wheelbase: float = 1280.55  # Gives exact BB->P1 distance of 447.50 mm
    headtube_length: float = 120.0
    front_wheel_radius: float = 372.0  # 29" outer radius ~ 744mm diameter
    rear_wheel_radius: float = 352.0  # 27.5" outer radius ~ 704mm diameter
    wheel_radius: float = 372.0  # Legacy compatibility alias (Front 29")
```

### `SuspensionHardwareSpecs` (src/bike_sim/geometry/specs.py:26-38)

```python
@dataclass
class SuspensionHardwareSpecs:
    fork_travel: float = 180.0
    fork_offset: float = 44.0
    rear_wheel_travel: float = 180.0
    shock_eye_to_eye: float = 205.0  # Trunnion mount 205x65 (= 230x65 standard-mount equivalent)
    shock_stroke: float = 65.0
    fork_stiffness: float = 6000.0  # N/m (6.0 N/mm) -> 30% sag (54mm) under 80kg rider
    fork_damping: float = 450.0  # Ns/m -> damping ratio ~0.50 (linear baseline)
    shock_stiffness: float = 114600.0  # N/m (114.6 N/mm ~654 lbs/in) -> 30% rear sag
    shock_damping: float = 1200.0  # Ns/m -> damping ratio ~0.65 (linear baseline)
```

### `BikeSpecs` (src/bike_sim/geometry/specs.py:41-112)

```python
@dataclass
class BikeSpecs:
    reach: float = 480.0
    stack: float = 646.0
    head_angle_deg: float = 64.0
    effective_seat_angle_deg: float = 77.0
    bb_drop: float = 22.5
    wheelbase: float = 1280.55
    headtube_length: float = 120.0
    front_wheel_radius: float = 372.0
    rear_wheel_radius: float = 352.0
    wheel_radius: float = 372.0
    crank_length: float = 165.0  # Crank arm length, BB spindle to pedal spindle (mm)
    fork_travel: float = 180.0
    fork_offset: float = 44.0
    rear_wheel_travel: float = 180.0
    shock_eye_to_eye: float = 205.0  # Trunnion mount 205x65 (= 230x65 standard-mount equivalent)
    shock_stroke: float = 65.0
    fork_stiffness: float = 6000.0
    fork_damping: float = 450.0
    shock_stiffness: float = 114600.0
    shock_damping: float = 1200.0
    fork_hsc: int = 2
    fork_lsc: int = 7
    fork_rebound: int = 9
    fork_initial_psi: float = 85.2
    fork_air_tokens: int = 2
    shock_hsc: int = 2
    shock_lsc: int = 7
    shock_rebound: int = 7
    shock_hbo: int = 2
    shock_lockout: bool = False
```


## `src/bike_sim/native/contracts.py`

### `PhysicalViewState` (src/bike_sim/native/contracts.py:151-166)

```python
@dataclass(frozen=True)
class PhysicalViewState:
    channels: Mapping[str, Any] = field(default_factory=dict)
    drive_mode: str = 'articulated_effort'
    sample: Any = None
    endpoint: Mapping[str, Any] = field(default_factory=dict)
    preview_row: Mapping[str, Any] = field(default_factory=dict)
    requested_scale: int = 1
    achieved_rtf: float | None = None
    def __post_init__(self) -> None:
        object.__setattr__(self, 'channels', _freeze(self.channels))
        object.__setattr__(self, 'endpoint', _freeze(self.endpoint))
        object.__setattr__(self, 'preview_row', _freeze(self.preview_row))
```

### `FrameSnapshot` (src/bike_sim/native/contracts.py:169-191)

```python
@dataclass(frozen=True)
class FrameSnapshot:
    generation: int
    step: int
    time_s: float
    integration_state: np.ndarray
    latest_sample: Any = None
    view: PhysicalViewState | None = None
    outcome: str | None = None
    first_failure: Any = None
    model_status: Mapping[str, Any] = field(default_factory=dict)
    model_fields: Mapping[str, Any] = field(default_factory=dict)
    def __post_init__(self) -> None:
        object.__setattr__(self, 'integration_state',
                           _owned_array(self.integration_state, 'integration_state'))
        object.__setattr__(self, 'first_failure', _freeze(self.first_failure))
        object.__setattr__(self, 'model_status', _freeze(self.model_status))
        object.__setattr__(self, 'model_fields', _freeze(self.model_fields))
        if self.view is not None and not isinstance(self.view, PhysicalViewState):
            raise ValueError('view must be a PhysicalViewState')
```

### `AdvanceResult` (src/bike_sim/native/contracts.py:194-205)

```python
@dataclass(frozen=True)
class AdvanceResult:
    step: int
    time_s: float
    reason: str
    outcome: str | None = None
    def __post_init__(self) -> None:
        if self.reason not in ('target', 'budget', 'outcome'):
            raise ValueError(f'unknown advance reason {self.reason!r}')
```

### `SampleBatch` (src/bike_sim/native/contracts.py:208-272)

```python
@dataclass(frozen=True, init=False, eq=False)
class SampleBatch:
    interval_ids: tuple[int, ...]
    columns: Mapping[str, np.ndarray]
    generation: int
    _rows: tuple[Mapping[str, Any], ...] | None
    _native_batch: Any
```


## `src/bike_sim/native/recording.py`

### `LegacySnapshot` (src/bike_sim/native/recording.py:24-41)

```python
@dataclass(frozen=True)
class LegacySnapshot:
    summary: object
    requested: tuple
    applied: tuple
    observations: tuple
    transitions: tuple
    metrics: object
    columns: object
    intervals: tuple
    assets: object
    initial_state: object
    final_state: object
    recipe: object
    def __post_init__(self):
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, freeze(getattr(self, name)))
```


## `src/bike_sim/native/setup.py`

### `RuntimeBootstrap` (src/bike_sim/native/setup.py:327-339)

```python
@dataclass(frozen=True)
class RuntimeBootstrap:
    model_path: Path
    config: dict
    state: dict
```


## `src/bike_sim/physics/air_spring.py`

### `AirSpringSpecs` (src/bike_sim/physics/air_spring.py:23-61)

```python
@dataclass(frozen=True)
class AirSpringSpecs:
    stanchion_inner_diam_mm: float = 24.0   # Inner air shaft bore inside 38mm stanchion ~ 24.0 mm
    total_travel_mm: float = 180.0          # Max travel in mm
    pos_chamber_length_mm: float = 280.0    # Baseline positive chamber length in mm (280mm air column)
    neg_chamber_length_mm: float = 55.0     # Negative chamber length in mm (~20% of total)
    token_volume_cm3: float = 8.0           # Volume displaced by one volume spacer token (8 cm^3)
    max_tokens: int = 4                     # Maximum allowable volume spacers
    default_tokens: int = 2                 # Default recommended tokens
    gamma: float = 1.40                     # Adiabatic index for diatomic air
    atm_pressure_pa: float = ATM_PA         # Atmospheric reference pressure in Pa
    def __post_init__(self):
        air_specs(self)
```


## `src/bike_sim/physics/attachment_budget.py`

### `AttachmentSample` (src/bike_sim/physics/attachment_budget.py:11-19)

```python
@dataclass(frozen=True)
class AttachmentSample:
    kind: str
    normal_n: float
    tangent_n: float
    moment_nm: float
    gap_m: float
    pull_n: float = 0.0
    half_patch_m: float = 0.0
```

### `AttachmentBudget` (src/bike_sim/physics/attachment_budget.py:22-28)

```python
@dataclass(frozen=True)
class AttachmentBudget:
    foot_min_normal_n: float
    foot_mu: float
    saddle_mu: float
    grip_pull_n: float
    max_gap_m: float
```


## `src/bike_sim/physics/chain.py`

### `DrivetrainSpecs` (src/bike_sim/physics/chain.py:13-30)

```python
@dataclass(frozen=True)
class DrivetrainSpecs:
    front_teeth: int = 34
    rear_teeth: int = 24
    chain_pitch_m: float = .0127
    def __post_init__(self):
        for key in ('front_teeth', 'rear_teeth'):
            integer(getattr(self, key), f'DrivetrainSpecs.{key}', minimum=3)
        object.__setattr__(self,'chain_pitch_m',scalar(self.chain_pitch_m,'chain pitch',positive=True))
```


## `src/bike_sim/physics/coil_shock.py`

### `CoilShockSpecs` (src/bike_sim/physics/coil_shock.py:26-55)

```python
@dataclass
class CoilShockSpecs:
    rate_n_m: float = 114600.0
    preload_mm: float = 0.0
    stroke_mm: float = 65.0
    bumper_length_mm: float = 10.0
    bumper_peak_n: float = 7000.0
    def __post_init__(self):
        coil_specs(self)
```


## `src/bike_sim/physics/damper.py`

### `DamperPreset` (src/bike_sim/physics/damper.py:18-23)

```python
class DamperPreset(Enum):
    BASE = "Base Tune (Factory Neutral)"
    PLUSH = "Plush / Wet Traction"
    ENDURO = "Enduro Race (Supportive)"
    PARK = "Bike Park / Jumps (Firm)"
    CUSTOM = "Custom Rider Tune"
```

### `DamperClickConfig` (src/bike_sim/physics/damper.py:26-42)

```python
@dataclass
class DamperClickConfig:
    fork_hsc: int = 2          # 0..4 (0=Open, 2=Neutral, 4=Firm)
    fork_lsc: int = 7          # 0..14 (0=Open, 7=Neutral, 14=Firm)
    fork_rebound: int = 9      # 0..17 (0=Fast/Open, 9=Neutral, 17=Slow/Firm)
    shock_hsc: int = 2         # 0..4 (0=Open, 2=Neutral, 4=Firm)
    shock_lsc: int = 7         # 0..14 (0=Open, 7=Neutral, 14=Firm)
    shock_rebound: int = 7     # 0..14 (0=Fast/Open, 7=Neutral, 14=Slow/Firm)
    shock_hbo: int = 2         # 0..4 (0=Min HBO, 2=Neutral, 4=Max HBO)
    shock_lockout: bool = False  # False=Open, True=Firm Pedal Platform
    preset_name: str = DamperPreset.BASE.value
```


## `src/bike_sim/physics/distributed_tire.py`

### `HingeDensity` (src/bike_sim/physics/distributed_tire.py:11-29)

```python
@dataclass(frozen=True)
class HingeDensity:
    knots_m: tuple[float, ...] = (0., .004, .012)
    stiffness_n_m: tuple[float, ...] = (300000., 0., 0.)
    damping_ns_m: float = 2000.
    provenance: str = 'synthetic density; not fitted or measured'
    def __post_init__(self):
        a, k = np.asarray(self.knots_m, float), np.asarray(self.stiffness_n_m, float)
        if a.ndim != 1 or len(a) == 0 or a.shape != k.shape:
            raise ValueError('invalid density table shape')
        if not np.isfinite([*a, *k, self.damping_ns_m]).all():
            raise ValueError('density parameters must be finite')
        if a[0] != 0 or np.any(np.diff(a) <= 0) or np.any(k < 0) or self.damping_ns_m < 0:
            raise ValueError('density law must be monotone and dissipative')
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('material provenance is required')
        object.__setattr__(self, 'knots_m', tuple(float(v) for v in a))
        object.__setattr__(self, 'stiffness_n_m', tuple(float(v) for v in k))
```

### `DistributedWheelMaterial` (src/bike_sim/physics/distributed_tire.py:32-51)

```python
@dataclass(frozen=True)
class DistributedWheelMaterial:
    density: HingeDensity = field(default_factory=HingeDensity)
    fitting_dataset_id: str = 'none-synthetic'
    valid_load_range_n: tuple[float, float] = (0., 2000.)
    valid_deflection_range_m: tuple[float, float] = (0., .04)
    calibration_status: str = 'experimental_unvalidated'
    def __post_init__(self):
        if self.calibration_status != 'experimental_unvalidated':
            raise ValueError('a material record cannot certify the plant')
        if not isinstance(self.density, HingeDensity):
            raise ValueError('an explicit density is required')
        if not isinstance(self.fitting_dataset_id, str) or not self.fitting_dataset_id.strip():
            raise ValueError('fitting dataset ID is required')
        for name in ('valid_load_range_n', 'valid_deflection_range_m'):
            value = tuple(float(x) for x in getattr(self, name))
            if len(value) != 2 or not np.isfinite(value).all() or not 0 <= value[0] < value[1]:
                raise ValueError('invalid per-wheel material domain')
            object.__setattr__(self, name, value)
```

### `DistributedTireConfig` (src/bike_sim/physics/distributed_tire.py:54-91)

```python
@dataclass(frozen=True)
class DistributedTireConfig:
    station_count: int = 128
    density: HingeDensity = field(default_factory=HingeDensity)
    fitting_dataset_id: str = 'none-synthetic'
    valid_load_range_n: tuple[float, float] = (0., 2000.)
    valid_deflection_range_m: tuple[float, float] = (0., .04)
    calibration_status: str = 'experimental_unvalidated'
    front_material: DistributedWheelMaterial | None = None
    rear_material: DistributedWheelMaterial | None = None
    def __post_init__(self):
        station_angles(self.station_count)
        if not isinstance(self.density, HingeDensity):
            raise ValueError('distributed tire needs an explicit density law')
        for field_name in ('valid_load_range_n', 'valid_deflection_range_m'):
            v = tuple(float(x) for x in getattr(self, field_name))
            if len(v) != 2 or not np.isfinite(v).all() or not 0 <= v[0] < v[1]:
                raise ValueError('invalid distributed tire applicability range')
            object.__setattr__(self, field_name, v)
        if not self.fitting_dataset_id.strip():
            raise ValueError('a fitting dataset ID or explicit synthetic ID is required')
        # Status is produced by evidence, never user-upgraded via a TOML label.
        if self.calibration_status != 'experimental_unvalidated':
            raise ValueError('distributed tire acceptance is evaluated in the validation report')
        self.for_wheel('front')
        self.for_wheel('rear')
```


## `src/bike_sim/physics/drivetrain.py`

### `DrivetrainSpecs` (src/bike_sim/physics/drivetrain.py:63-189)

```python
@dataclass(frozen=True)
class DrivetrainSpecs:
    chainring_teeth: int = 32
    cog_teeth: int = 14
    cassette: Tuple[int, ...] = CASSETTE_12S_TEETH
    auto_shift: bool = True
    target_cadence_min_rpm: float = 65.0
    target_cadence_max_rpm: float = 85.0
    shift_cooldown_s: float = 0.40
    shift_cut_duration_s: float = 0.20
    ripple_depth: float = 0.85
    rider_torque_ceiling_nm: float = 60.0
    rider_power_ceiling_w: float = 300.0
    assist_torque_ceiling_nm: float = 85.0
    assist_power_ceiling_w: float = 600.0
    assist_response_s: float = 0.30
    cutoff_speed_kmh: float = 25.0
    cutoff_taper_kmh: float = 2.0
    crank_phase_deg: float = 0.0
    crank_hold_stiffness_nm_rad: float = 30.0
    crank_hold_damping_nms_rad: float = 3.0
    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If the gearing, a ceiling or the ripple depth is out of range.
        """
        if self.chainring_teeth < 20 or self.cog_teeth < 9:
            raise ValueError(
                f"implausible gearing {self.chainring_teeth}x{self.cog_teeth}; "
                f"a chainring has at least 20 teeth and a cog at least 9"
            )
        if self.auto_shift:
            if not self.cassette:
                raise ValueError("cassette cannot be empty when auto_shift is True")
            if any(t < 9 for t in self.cassette):
                raise ValueError(f"all cassette cogs must have at least 9 teeth, got {self.cassette}")
            if self.target_cadence_min_rpm <= 0 or self.target_cadence_max_rpm <= self.target_cadence_min_rpm:
                raise ValueError(
                    f"invalid cadence band [{self.target_cadence_min_rpm}, {self.target_cadence_max_rpm}]"
                )
            if self.shift_cooldown_s < 0.0 or self.shift_cut_duration_s < 0.0:
                raise ValueError("shift timings must be non-negative")
        if not 0.0 <= self.ripple_depth <= 1.0:
            raise ValueError(f"ripple_depth must be in [0, 1], got {self.ripple_depth}")
        for name in (
            "rider_torque_ceiling_nm",
            "rider_power_ceiling_w",
            "assist_torque_ceiling_nm",
            "assist_power_ceiling_w",
            "assist_response_s",
            "cutoff_speed_kmh",
            "cutoff_taper_kmh",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
        rider_peak = self.rider_torque_ceiling_nm * peak_shape(self.ripple_depth)
        if rider_peak + self.assist_torque_ceiling_nm > CRANK_TORQUE_CEILING_NM:
            raise ValueError(
                f"peak crank torque {rider_peak + self.assist_torque_ceiling_nm:.1f} N.m exceeds the "
                f"{CRANK_TORQUE_CEILING_NM:.0f} N.m crank actuator ceiling; lower a ceiling or the depth"
            )
```


## `src/bike_sim/physics/energy_ledger.py`

### `StepWork` (src/bike_sim/physics/energy_ledger.py:13-20)

```python
@dataclass(frozen=True)
class StepWork:
    muscle_signed_j: float
    muscle_positive_j: float
    motor_signed_j: float
    motor_positive_j: float
    constraint_signed_j: float
    constraint_absolute_j: float
```


## `src/bike_sim/physics/inertia.py`

### `WheelComponent` (src/bike_sim/physics/inertia.py:10-17)

```python
class WheelComponent(NamedTuple):
    mass_fraction: float
    inner_radius_m: float
    outer_radius_m: float
    width_m: float
    offset_m: tuple[float, float, float]
```


## `src/bike_sim/physics/joint_strength.py`

### `TorqueCurve` (src/bike_sim/physics/joint_strength.py:27-54)

```python
@dataclass(frozen=True)
class TorqueCurve:
    angles_rad: tuple[float, ...]
    torques_nm: tuple[float, ...]
    vmax_rad_s: float
    hill_c: float
    eccentric_ratio: float
    source: str
    def __post_init__(self):
        if len(self.angles_rad) < 2 or len(self.angles_rad) != len(self.torques_nm):
            raise ValueError('strength curve requires paired angle/torque knots')
        values = (*self.angles_rad, *self.torques_nm, self.vmax_rad_s,
                  self.hill_c, self.eccentric_ratio)
        if not np.isfinite(values).all() or any(t < 0 for t in self.torques_nm):
            raise ValueError('invalid strength data')
        if any(b <= a for a, b in zip(self.angles_rad, self.angles_rad[1:])):
            raise ValueError('angles must increase')
        if self.vmax_rad_s <= 0 or self.hill_c <= 0 or self.eccentric_ratio < 1:
            raise ValueError('invalid force-velocity parameters')
        if not self.source.strip():
            raise ValueError('missing strength provenance')
        # float64 copies for the compiled capacity core; the tuple fields stay
        # authoritative so equality/hash/repr keep their declared shape.
        object.__setattr__(self, '_angles',
                           np.array(self.angles_rad, dtype=float, copy=True))
        object.__setattr__(self, '_torques',
                           np.array(self.torques_nm, dtype=float, copy=True))
```


## `src/bike_sim/physics/mass.py`

### `BikeMassSpecs` (src/bike_sim/physics/mass.py:23-98)

```python
@dataclass
class BikeMassSpecs:
    motor_mass: float = 2.90          # Mid-drive motor casing & internals around BB
    battery_mass: float = 4.30        # 750 Wh integrated battery inside lower downtube
    frame_structure_mass: float = 3.20 # Front triangle tubes, headtube, pivot mounts
    saddle_post_mass: float = 0.90     # Dropper post, remote cable, enduro saddle
    crank_pedals_mass: float = 0.85    # Crank arms, spindle, chainring, platform pedals
    steer_assembly_mass: float = 1.10  # Handlebar, stem, steerer tube, crown, headset, controls
    stanchions_mass: float = 0.85      # 38mm upper fork stanchions
    fork_lowers_mass: float = 1.50     # Magnesium lowers, arch, axle, 4-piston caliper
    chainstay_mass: float = 1.25       # Chainstay arms, main pivot sleeve, Horst pivots
    seatstay_mass: float = 1.05        # Seatstay arms, rear dropouts, rear brake caliper
    rocker_mass: float = 0.40          # Forged aluminum rocker link
    shock_yoke_mass: float = 0.30      # Shock driving yoke
    shock_damper_mass: float = 0.60    # Metric trunnion 205x65mm air damper body & shaft
    front_wheel_mass: float = 2.40     # 29" front wheel, 2.4" enduro tire, sealant, 203mm rotor
    rear_wheel_mass: float = 2.80      # 27.5" rear wheel, 2.5" DH tire, 12-spd cassette, rotor
    wheel_rim_tire_fraction: float = 0.75
    wheel_hub_core_fraction: float = 0.25
```


## `src/bike_sim/physics/model_config.py`

### `EndStopConfig` (src/bike_sim/physics/model_config.py:15-37)

```python
@dataclass(frozen=True)
class EndStopConfig:
    reference_force_n: float = 7000.0
    reference_deflection_m: float = 0.010
    damping_n_s_m: float = 500.0
    overtravel_m: float = 0.010
    provenance: str = "synthetic"
    def __post_init__(self) -> None:
        scalar(self.reference_force_n, "EndStopConfig.reference_force_n", minimum=0.)
        scalar(self.reference_deflection_m, "EndStopConfig.reference_deflection_m", positive=True)
        scalar(self.damping_n_s_m, "EndStopConfig.damping_n_s_m", minimum=0.)
        scalar(self.overtravel_m, "EndStopConfig.overtravel_m", positive=True)
        if self.overtravel_m < self.reference_deflection_m:
            raise ValueError("EndStopConfig.overtravel_m: must reach reference deflection")
```

### `SimulationPhysicsConfig` (src/bike_sim/physics/model_config.py:40-136)

```python
@dataclass(frozen=True)
class SimulationPhysicsConfig:
    physics_mode: str = "legacy"
    drive_mode: str | None = None
    timestep_s: float = 0.0005
    control_period_s: float = 0.005
    pitch_assist: bool | None = None
    end_stops: EndStopConfig = field(default_factory=EndStopConfig)
    tires: TireBackendConfig = field(default_factory=TireBackendConfig)
    drive: PhysicalDriveConfig = field(default_factory=PhysicalDriveConfig)
    resistance: ResistanceConfig = field(default_factory=ResistanceConfig)
    articulated: ArticulatedConfig = field(default_factory=ArticulatedConfig)
    seated_climb: SeatedClimbConfig = field(default_factory=SeatedClimbConfig)
    initial_speed_mps: float = 0.0
    initial_front_brake: float = 0.0
    initial_rear_brake: float = 0.0
    closure_time_constant_s: float = 0.001
    equilibrium_refine_after_s: float = 1.0
    equilibrium_refine_period_s: float = 3.0
    equilibrium_cache_enabled: bool = False
    equilibrium_state_file: str | None = None
    def __post_init__(self) -> None:
        if self.physics_mode not in {"legacy", "physical"}:
            raise ValueError("unknown physics_mode")
        if not isfinite(self.timestep_s) or self.timestep_s <= 0:
            raise ValueError("timestep_s must be finite and positive")
        scalar(self.control_period_s, 'control period', positive=True)
        steps = round(self.control_period_s/self.timestep_s)
        if steps < 1 or abs(steps*self.timestep_s-self.control_period_s) > 1e-9:
            raise ValueError('control_period_s must be an integer multiple of timestep_s')

        for name in ('equilibrium_refine_after_s', 'equilibrium_refine_period_s'):
            scalar(getattr(self, name), name, positive=True)
        legacy = self.physics_mode == "legacy"
        scalar(self.closure_time_constant_s, "closure time constant", positive=True)
        if not legacy and self.closure_time_constant_s < 2.0*self.timestep_s:
            raise ValueError(
                f"closure_time_constant_s={self.closure_time_constant_s} must be at least "
                f"2*timestep_s={2.0*self.timestep_s}; otherwise MuJoCo silently changes closure stiffness"
            )
        if self.drive_mode is None:
            object.__setattr__(
                self, "drive_mode", "ideal_speed_control" if legacy else "coast"
            )
        if self.pitch_assist is None:
            object.__setattr__(self, "pitch_assist", legacy)
        if self.drive_mode not in {
            "coast",
            "ideal_speed_control",
            "crank_effort",
            "articulated_effort",
        }:
            raise ValueError("unknown drive_mode")
        if legacy and self.drive_mode in {'crank_effort','articulated_effort'}:
            raise ValueError('physical effort drive modes require physics_mode=physical')
        if not legacy and self.pitch_assist:
            raise ValueError("external pitch assist is forbidden in physical mode")

        scalar(self.initial_speed_mps, "initial speed")
        for name in ('initial_front_brake','initial_rear_brake'):
            value=scalar(getattr(self,name),name,minimum=0.)
            if value>1.: raise ValueError(f'{name} must lie in [0,1]')
        scalar(self.timestep_s, "timestep", positive=True)
        if not isinstance(self.pitch_assist, bool):
            raise ValueError("pitch_assist must be a bool")
        if not isinstance(self.equilibrium_cache_enabled, bool):
            raise ValueError("equilibrium_cache_enabled must be a bool")
        if (self.equilibrium_state_file is not None
                and (not isinstance(self.equilibrium_state_file, str)
                     or not self.equilibrium_state_file.strip())):
            raise ValueError("equilibrium_state_file must be a non-empty path or None")
        for name, cls in (("end_stops", EndStopConfig), ("tires", TireBackendConfig),
                          ("drive", PhysicalDriveConfig), ("resistance", ResistanceConfig),
                          ("articulated", ArticulatedConfig), ("seated_climb", SeatedClimbConfig)):
            if not isinstance(getattr(self, name), cls):
                raise ValueError(f"{name} needs an immutable {cls.__name__}")
        # The detailed discretised brush is written for its declared step: unlike the
        # legacy applier (which rewrote model.opt.timestep), the physical runtime owns
        # the step, so a mismatch would silently integrate at the wrong dt. The fast
        # tier declares 5e-4 too but stays ungated, same as the legacy stack.
        if (self.tires.backend == 'pneumatic'
                and isinstance(self.tires.pneumatic, PneumaticTireConfig)
                and self.tires.pneumatic.tier == 'detailed'
                and self.timestep_s != TIERS['detailed'].timestep_s):
            raise ValueError(
                f"detailed pneumatic tyre requires timestep_s={TIERS['detailed'].timestep_s}; "
                f"got {self.timestep_s}"
            )
        if self.seated_climb.enabled and (legacy or self.drive_mode != 'articulated_effort'):
            raise ValueError('seated climb requires the articulated physical effort drive')
        if self.drive.motor_clutch and self.drive_mode not in ('crank_effort','articulated_effort'):
            raise ValueError('drive.motor_clutch requires an effort drive mode')
        if self.drive.rotor_inertia_kgm2 > 0. and self.drive_mode not in ('crank_effort','articulated_effort'):
            raise ValueError('drive.rotor_inertia_kgm2 requires an effort drive mode')
```


## `src/bike_sim/physics/motor_profile.py`

### `MotorProfile` (src/bike_sim/physics/motor_profile.py:14-63)

```python
@dataclass(frozen=True)
class MotorProfile:
    name: str
    peak_torque_nm: float
    rated_power_w: float
    peak_power_w: float
    torque_tau_s: float
    mode_gains: Mapping[str, float | tuple[float, float]]
    emtb_full_gain_at_nm: float
    cadence_support_max_rpm: float
    cutoff_mps: float
    taper_width_mps: float
    gate_min_crank_rad_s: float
    provenance: str
    def __post_init__(self):
        object.__setattr__(self, "mode_gains", MappingProxyType(self.validate()))
    def validate(self):
        for name in ('peak_torque_nm', 'rated_power_w', 'peak_power_w', 'torque_tau_s',
                     'emtb_full_gain_at_nm', 'cadence_support_max_rpm', 'cutoff_mps',
                     'taper_width_mps', 'gate_min_crank_rad_s'):
            scalar(getattr(self, name), name, positive=True)
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('motor profile requires a name')
        if self.rated_power_w > self.peak_power_w:
            raise ValueError('rated motor power exceeds peak power')
        if self.taper_width_mps > self.cutoff_mps:
            raise ValueError('speed taper exceeds the cutoff speed')
        if not isinstance(self.mode_gains, Mapping) or set(self.mode_gains) != {'eco', 'tour', 'emtb', 'turbo'}:
            raise ValueError('profile must declare eco, tour, emtb and turbo')
        gains = {}
        for mode, gain in self.mode_gains.items():
            if mode == 'emtb':
                if not isinstance(gain, tuple) or len(gain) != 2:
                    raise ValueError('eMTB requires two ordered gain bounds')
                low, high = (scalar(value, 'eMTB gain', minimum=0.) for value in gain)
                if low > high:
                    raise ValueError('eMTB gain bounds must increase')
                gains[mode] = (low, high)
            else:
                gains[mode] = scalar(gain, f'{mode} gain', minimum=0.)
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('motor profile requires provenance')
        return gains
```


## `src/bike_sim/physics/pedaling.py`

### `PedalingState` (src/bike_sim/physics/pedaling.py:18-25)

```python
@dataclass(frozen=True)
class PedalingState:
    mode: str
    reason: str
    effort_nm: float
    required_cadence_rpm: float
    target_phase_rad: float | None = None
    target_rate_rad_s: float = 0.
```


## `src/bike_sim/physics/physical_config.py`

### `TireParameters` (src/bike_sim/physics/physical_config.py:23-37)

```python
@dataclass(frozen=True)
class TireParameters:
    material: TireSpec | TabulatedTireSpec = field(default_factory=_material)
    tangent_k_n_m: float = 20000.
    mu: float = .8
    relaxation_length_m: float = .2
    def __post_init__(self):
        if not isinstance(self.material, (TireSpec, TabulatedTireSpec)):
            raise ValueError('tire material must be a TireSpec or TabulatedTireSpec')
        if isinstance(self.material, TireSpec):
            self.material.validate()
        for key in ('tangent_k_n_m','relaxation_length_m'):
            scalar(getattr(self,key),key,positive=True)
        scalar(self.mu,'tire friction',minimum=0)
```

### `PneumaticTireConfig` (src/bike_sim/physics/physical_config.py:40-64)

```python
@dataclass(frozen=True)
class PneumaticTireConfig:
    tier: str = "fast"
    front: TyreSpecs = field(default=FRONT_TYRE)
    rear: TyreSpecs = field(default=REAR_TYRE)
    surface: str | None = None
    def __post_init__(self):
        if self.tier not in TIERS:
            raise ValueError(f"unknown pneumatic tyre tier '{self.tier}'; available: {', '.join(TIERS)}")
        if not isinstance(self.front,TyreSpecs) or not isinstance(self.rear,TyreSpecs):
            raise ValueError('pneumatic tires need explicit TyreSpecs')
        self.front.__post_init__()
        self.rear.__post_init__()
        if self.surface is not None:
            from bike_sim.terrain.surface import SURFACES  # pure data, no cycle at import time
            if not isinstance(self.surface,str) or self.surface not in SURFACES:
                raise ValueError(f"unknown surface '{self.surface}'; available: {', '.join(SURFACES)}")
```

### `TireBackendConfig` (src/bike_sim/physics/physical_config.py:67-101)

```python
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
    pneumatic: PneumaticTireConfig = field(default_factory=PneumaticTireConfig)
    def __post_init__(self):
        if self.backend not in ('native_reference','compliant_2d','distributed_2d_reference','pneumatic'):
            raise ValueError('unknown physical tire backend')
        if self.surface_mode not in ('configured', 'track'):
            raise ValueError('surface_mode must be configured or track')
        if self.surface_mode == 'track' and self.backend not in ('compliant_2d','distributed_2d_reference','pneumatic'):
            raise ValueError('track material resolution requires a compliant_2d, distributed_2d_reference or pneumatic tire backend')
        if not isinstance(self.distributed,DistributedTireConfig):
            raise ValueError('distributed tire configuration must be explicit')
        if not isinstance(self.pneumatic,PneumaticTireConfig):
            raise ValueError('pneumatic tire configuration must be explicit')
        self.pneumatic.__post_init__()
        if self.backend == 'pneumatic' and self.surface_mode != 'track' and self.pneumatic.surface is None:
            raise ValueError('pneumatic tires require surface_mode track or a named surface')
        if not isinstance(self.front,TireParameters) or not isinstance(self.rear,TireParameters):
            raise ValueError('front and rear tires need explicit TireParameters')
        self.front.__post_init__()
        self.rear.__post_init__()
        scalar(self.significant_delta_m,'significant penetration',minimum=0)
        fraction = scalar(self.significance_fraction,'significance fraction',minimum=0)
        angle = scalar(self.distinct_normal_deg,'distinct normal angle',positive=True)
        if fraction > 1 or angle >= 180:
            raise ValueError('invalid multi-support thresholds')
```

### `AssistConfig` (src/bike_sim/physics/physical_config.py:104-134)

```python
@dataclass(frozen=True)
class AssistConfig:
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
```

### `BatteryConfig` (src/bike_sim/physics/physical_config.py:137-152)

```python
@dataclass(frozen=True)
class BatteryConfig:
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
```

### `PedalingConfig` (src/bike_sim/physics/physical_config.py:155-203)

```python
@dataclass(frozen=True)
class PedalingConfig:
    enabled: bool = False
    coast_above_rpm: float = 110.
    resume_below_rpm: float = 90.
    stop_time_s: float = .35
    coast_cadence_tau_s: float = 0.
    mash_cadence_rpm: float = 45.
    mash_torque_nm: float = 60.
    effort_slew_nm_s: float = 300.
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
```

### `ShiftingConfig` (src/bike_sim/physics/physical_config.py:206-241)

```python
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
        cassette = tuple(integer(tooth, 'ShiftingConfig.cassette', minimum=3) for tooth in sequence(self.cassette, 'ShiftingConfig.cassette'))
        if not cassette:
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
```

### `PhysicalDriveConfig` (src/bike_sim/physics/physical_config.py:244-303)

```python
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
    motor_clutch: bool = False
    rotor_inertia_kgm2: float = 0.
    assist: AssistConfig = field(default_factory=AssistConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    pedaling: PedalingConfig = field(default_factory=PedalingConfig)
    shifting: ShiftingConfig = field(default_factory=ShiftingConfig)
    def __post_init__(self):
        if not isinstance(self.gearing,DrivetrainSpecs) or not isinstance(self.assist,AssistConfig) or not isinstance(self.battery,BatteryConfig):
            raise ValueError('invalid drivetrain configuration object')
        self.gearing.__post_init__()
        self.assist.__post_init__()
        self.battery.__post_init__()
        if not isinstance(self.pedaling, PedalingConfig):
            raise ValueError('pedaling needs an immutable PedalingConfig')
        self.pedaling.__post_init__()
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
        self.shifting.__post_init__()
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
```

### `ResistanceConfig` (src/bike_sim/physics/physical_config.py:306-324)

```python
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
```

### `ArticulatedConfig` (src/bike_sim/physics/physical_config.py:327-474)

```python
@dataclass(frozen=True)
class ArticulatedConfig:
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
    posture_translation_k_n_m: float = 1500.
    posture_translation_d_ns_m: float = 150.
    posture_translation_limit_n: float = 400.
    saddle_patch_half_length_m: float = .045
    pedal_patch_half_length_m: float = .025
    support_pad_radius_m: float = .020  # synthetic sole contact shape
    support_k_n_m: float = 100000.
    support_c_ns_m: float = 500.  # saddle interface
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
    joint_passive_damping_nms_rad: float | None = None
    coupled_task_control: bool = False
    joint_limit_nm: float = 100.
    joint_speed_limit_rad_s: float = 20.
    joint_power_limit_w: float = 250.
    joint_envelope_path: str | None = None
    joint_envelope_soft_margin_rad: float = .1
    joint_envelope_soft_k_nm_rad: float = 100.
    activation_tau_s: float = 0.
    active_positive_power_limit_w: float | None = None
    foot_mu: float = .9
    saddle_mu: float = .6
    pedal_min_normal_n: float = 20.
    saddle_reserve_weight_fraction: float = .15
    grip_pull_per_hand_n: float = 300.
    pedal_ankle_amplitude_rad: float = .1
    pedal_scrape_fraction: float = .5
    pedal_torque_ripple: float = .35
    return_foot_preload_n: float = 0.
    link_max_gap_m: float = .005
    road_lookahead_m: float = 0.
    joint_strength_path: str | None = None
    pedal_attachment: str = 'flat'
    saddle_attachment: str = 'flat'
    grip_attachment: str = 'spring'
    balance_floor_kmh: float = 4.
    balance_dwell_s: float = .5
    balance_grace_s: float = 3.
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
                       'grip_attachment', 'coupled_task_control'):
                continue
            if key in ('grip_pair_force_limit_n','active_positive_power_limit_w',
                       'joint_passive_damping_nms_rad') and getattr(self,key) is None:
                continue
            scalar(getattr(self,key),key,minimum=0)
        if self.grip_pair_force_limit_n is not None:
            scalar(self.grip_pair_force_limit_n,'pair grip force limit',positive=True)
        for key in ('support_pad_radius_m','stance_blend_load_n','support_k_n_m','support_tangent_k_n_m','support_length_m','grip_k_n_m','grip_release_distance_m','joint_speed_limit_rad_s',
                    'pedal_min_normal_n','grip_pull_per_hand_n','link_max_gap_m',
                    'balance_floor_kmh', 'balance_dwell_s'):
            scalar(getattr(self,key),key,positive=True)
        if self.saddle_reserve_weight_fraction > 1.:
            raise ValueError('saddle reserve fraction must not exceed one')
        if self.joint_passive_damping_nms_rad is not None:
            scalar(self.joint_passive_damping_nms_rad,'joint_passive_damping_nms_rad',minimum=0)
        if not isinstance(self.coupled_task_control, bool):
            raise ValueError('coupled_task_control must be a bool')
        if self.pedal_torque_ripple >= 1.:
            raise ValueError('pedal torque ripple must be below one')
        if self.pedal_scrape_fraction > 1.:
            raise ValueError('pedal scrape fraction must not exceed one')
        if self.pedal_attachment not in ('flat', 'weld', 'spindle'):
            raise ValueError(
                f"pedal_attachment must be 'flat', 'weld' or 'spindle', got {self.pedal_attachment!r}")
        if self.saddle_attachment not in ('flat', 'weld', 'pin'):
            raise ValueError(
                f"saddle_attachment must be 'flat', 'weld' or 'pin', got {self.saddle_attachment!r}")
        if self.grip_attachment == 'weld':
            object.__setattr__(self, 'grip_attachment', 'connect')
        if self.grip_attachment not in ('spring', 'connect'):
            raise ValueError(
                f"grip_attachment must be 'spring' or 'connect', got {self.grip_attachment!r}")
```


## `src/bike_sim/physics/rider.py`

### `RiderSpecs` (src/bike_sim/physics/rider.py:139-339)

```python
@dataclass
class RiderSpecs:
    variant: str = DEFAULT_RIDER_VARIANT
    legs: str = "rigid"
    mass_kg: float = 80.0
    height_m: float = 1.80
    inseam_m: Optional[float] = None
    helmet_mass_kg: float = 0.40
    saddle_share: float = 0.55
    pedal_share: float = 0.33
    bar_share: float = 0.12
    torso_resonance_hz: float = 7.0
    leg_resonance_hz: float = 5.0
    arm_resonance_hz: float = 4.0
    damping_ratio: float = 0.40
    saddle_interface_k_n_m: float = 101_000.0
    saddle_interface_c_ns_m: float = 2_762.0
    def __post_init__(self) -> None:
        if self.variant not in RIDER_VARIANTS:
            raise ValueError(f"rider variant must be one of {RIDER_VARIANTS}, got {self.variant!r}")
        if self.legs not in ("rigid", "articulated"):
            raise ValueError(f"legs must be 'rigid' or 'articulated', got {self.legs!r}")
        if self.mass_kg <= 0.0 and self.variant != "none":
            raise ValueError(f"rider mass must be positive, got {self.mass_kg}")
        if self.height_m <= 0.0:
            raise ValueError(f"rider height must be positive, got {self.height_m}")
        if self.inseam_m is not None and not 0.0 < self.inseam_m < self.height_m:
            raise ValueError(f"inseam {self.inseam_m} m must lie between 0 and the height {self.height_m} m")
        shares = self.saddle_share + self.pedal_share + self.bar_share
        if abs(shares - 1.0) > 1e-9:
            raise ValueError(f"saddle, pedal and bar shares must sum to 1, got {shares:.6f}")
        if self.helmet_mass_kg < 0.0 or self.helmet_mass_kg >= self.mass_kg:
            raise ValueError(f"helmet mass {self.helmet_mass_kg} kg must be non-negative and below the rider mass")
```

### `RiderGeom` (src/bike_sim/physics/rider.py:347-361)

```python
@dataclass(frozen=True)
class RiderGeom:
    name: str
    kind: str  # "capsule" | "sphere"
    p1: np.ndarray
    p2: np.ndarray
    radius: float
    mass: float
```

### `RiderBody` (src/bike_sim/physics/rider.py:364-430)

```python
@dataclass(frozen=True)
class RiderBody:
    name: str
    parent: str
    attach: np.ndarray
    joint: str
    geoms: Tuple[RiderGeom, ...]
    stiffness_n_m: float
    damping_ns_m: float
    supported_mass_kg: float
    unilateral: bool
    interface: str
    def __post_init__(self):
        scalar(self.stiffness_n_m, 'RiderBody.stiffness_n_m', minimum=0.)
        scalar(self.damping_ns_m, 'RiderBody.damping_ns_m', minimum=0.)
        scalar(self.supported_mass_kg, 'RiderBody.supported_mass_kg', minimum=0.)
        boolean(self.unilateral, 'RiderBody.unilateral')
```

### `SeatedPose` (src/bike_sim/physics/rider.py:433-495)

```python
@dataclass(frozen=True)
class SeatedPose:
    saddle: SaddleGeometry
    hip: np.ndarray
    shoulder: np.ndarray
    elbow: np.ndarray
    wrist: np.ndarray
    grip: np.ndarray
    head_center: np.ndarray
    knee_front: np.ndarray
    knee_rear: np.ndarray
    ankle_front: np.ndarray
    ankle_rear: np.ndarray
    pedal_front: np.ndarray
    pedal_rear: np.ndarray
    torso_lean_deg: float
    knee_flexion_front_deg: float
    knee_flexion_rear_deg: float
    knee_flexion_bdc_deg: float
    bodies: Tuple[RiderBody, ...]
    path_masses_kg: Dict[str, float] = field(default_factory=dict)
    leg_chains: Tuple["LegChain", ...] = ()
```

### `LegChain` (src/bike_sim/physics/rider.py:602-628)

```python
@dataclass(frozen=True)
class LegChain:
    side: str
    lateral_y_m: float
    crank_len_m: float
    hip: np.ndarray
    knee: np.ndarray
    ankle: np.ndarray
    pedal: np.ndarray
    thigh_len_m: float
    shank_len_m: float
    thigh_mass_kg: float
    shank_mass_kg: float
    foot_mass_kg: float
    pitch0_thigh_rad: float
    pitch0_shank_rad: float
    pitch0_foot_rad: float
```


## `src/bike_sim/physics/rider_allocation.py`

### `Allocation` (src/bike_sim/physics/rider_allocation.py:19-23)

```python
@dataclass(frozen=True)
class Allocation:
    solution: np.ndarray
    feasible: bool
    violation: float
```


## `src/bike_sim/physics/rider_envelope.py`

### `JointEnvelope` (src/bike_sim/physics/rider_envelope.py:19-34)

```python
@dataclass(frozen=True)
class JointEnvelope:
    neutral_anatomical_rad: float
    direction: int
    minimum_anatomical_rad: float
    maximum_anatomical_rad: float
    provenance: str
    def __post_init__(self):
        values=(self.neutral_anatomical_rad,self.minimum_anatomical_rad,self.maximum_anatomical_rad)
        if not all(isfinite(v) for v in values) or type(self.direction) is not int or self.direction not in (-1,1):
            raise ValueError('invalid joint coordinate convention')
        if not self.minimum_anatomical_rad < self.maximum_anatomical_rad:
            raise ValueError('empty joint range')
        if not isinstance(self.provenance,str) or not self.provenance.strip():
            raise ValueError('joint range provenance is required')
```


## `src/bike_sim/physics/rider_posture.py`

### `RiderPosture` (src/bike_sim/physics/rider_posture.py:6-39)

```python
@dataclass(frozen=True)
class RiderPosture:
    torso_lean_rad: float = 0.
    pelvis_pitch_rad: float = 0.
    pelvis_offset_m: tuple[float, float] | None = None
    use_saddle: bool = True
    def __post_init__(self):
        for name, limit in (('torso_lean_rad', .8), ('pelvis_pitch_rad', .5)):
            value = scalar(getattr(self, name), name)
            if abs(value) > limit:
                raise ValueError(f'{name} must lie in [-{limit}, {limit}]')
        if self.pelvis_offset_m is not None:
            value = array(self.pelvis_offset_m, 'hip offset', (2,))
            if abs(value[0]) > .25 or not -.15 <= value[1] <= .25:
                raise ValueError('hip offset exceeds the posture command envelope')
            object.__setattr__(self, 'pelvis_offset_m', tuple(map(float, value)))
        if not isinstance(self.use_saddle, bool):
            raise ValueError('use_saddle must be a bool')
```


## `src/bike_sim/physics/rider_program.py`

### `PedalIntent` (src/bike_sim/physics/rider_program.py:14-18)

```python
@dataclass(frozen=True)
class PedalIntent:
    cadence_rad_s: float
    ankle_offset_rad: float
    scrape_fraction: float
```


## `src/bike_sim/physics/rider_segments.py`

### `ArticulatedPose` (src/bike_sim/physics/rider_segments.py:68-86)

```python
@dataclass(frozen=True)
class ArticulatedPose:
    saddle: object
    hip: np.ndarray
    shoulder: np.ndarray
    elbow: np.ndarray
    grip: np.ndarray
    head_center: np.ndarray
    knee_front: np.ndarray
    knee_rear: np.ndarray
    ankle_front: np.ndarray
    ankle_rear: np.ndarray
    pedal_front: np.ndarray
    pedal_rear: np.ndarray
    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name != 'saddle':
                object.__setattr__(self,name,array(getattr(self,name),name,(3,),readonly=True))
```


## `src/bike_sim/physics/seated_climb.py`

### `SeatedClimbConfig` (src/bike_sim/physics/seated_climb.py:11-47)

```python
@dataclass(frozen=True)
class SeatedClimbConfig:
    enabled: bool = False
    period_s: float = .01
    reaction_delay_s: float = .15
    target_crank_power_w: float = 225.
    max_crank_torque_nm: float = 60.
    torque_slew_nm_s: float = 300.
    lean_gain: float = 1.
    max_forward_lean_rad: float = .35
    max_backward_lean_rad: float = .10
    lean_rate_rad_s: float = .5
    orientation_tau_s: float = .5
    surge_power_w: float = 400.
    surge_grade: float = .20
    surge_budget_s: float = 15.
    surge_recovery_rate: float = 1/3
    front_load_share_target: float = .30
    lean_trim_gain_rad_s: float = .3
    lean_trim_limit_rad: float = .15
    trim_dead_time_s: float = .2
    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('seated climb enable must be boolean')
        for name in ('period_s', 'target_crank_power_w', 'max_crank_torque_nm',
                     'torque_slew_nm_s', 'lean_rate_rad_s', 'orientation_tau_s',
                     'surge_power_w', 'surge_budget_s'):
            scalar(getattr(self, name), name, positive=True)
        for name in ('reaction_delay_s', 'lean_gain', 'max_forward_lean_rad', 'max_backward_lean_rad',
                     'surge_grade', 'surge_recovery_rate', 'front_load_share_target',
                     'lean_trim_gain_rad_s', 'lean_trim_limit_rad', 'trim_dead_time_s'):
            scalar(getattr(self, name), name, minimum=0.)
        if self.front_load_share_target > 1.:
            raise ValueError('front load share target must not exceed one')
        if max(self.max_forward_lean_rad, self.max_backward_lean_rad) > .8:
            raise ValueError('seated lean exceeds the posture envelope')
```

### `SeatedClimbSignals` (src/bike_sim/physics/seated_climb.py:50-66)

```python
@dataclass(frozen=True)
class SeatedClimbSignals:
    pitch_rate_up_rad_s: float = 0.
    specific_force_body_mps2: tuple[float, float, float] = (0., 0., 0.)
    crank_rate_rad_s: float = 0.
    human_crank_torque_nm: float = 0.
    front_load_share: float | None = None
    def __post_init__(self):
        for name in ('pitch_rate_up_rad_s', 'crank_rate_rad_s', 'human_crank_torque_nm'):
            scalar(getattr(self, name), name)
        acceleration = array(self.specific_force_body_mps2, 'rider specific force', (3,))
        object.__setattr__(self, 'specific_force_body_mps2', tuple(map(float, acceleration)))
        if self.front_load_share is not None:
            scalar(self.front_load_share, 'front load share', minimum=0.)
            if self.front_load_share > 1.:
                raise ValueError('front load share must not exceed one')
```

### `SeatedClimbIntent` (src/bike_sim/physics/seated_climb.py:69-72)

```python
@dataclass(frozen=True)
class SeatedClimbIntent:
    posture: RiderPosture
    effort_ceiling_nm: float
```


## `src/bike_sim/physics/tire.py`

### `TireSpec` (src/bike_sim/physics/tire.py:32-103)

```python
@dataclass(frozen=True)
class TireSpec:
    radial_k_n_m: float
    radial_c_ns_m: float
    pressure_pa_gauge: float
    provenance: str
    valid_load_range_n: tuple[float, float]
    def __post_init__(self) -> None:
        for name in ("radial_k_n_m", "radial_c_ns_m", "pressure_pa_gauge"):
            object.__setattr__(self, name, _scalar(getattr(self, name), name))
        if self.radial_k_n_m <= 0.0:
            raise ValueError("radial_k_n_m must be positive")
        if self.radial_c_ns_m < 0.0 or self.pressure_pa_gauge < 0.0:
            raise ValueError("radial damping and gauge pressure must be nonnegative")
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError("provenance must be a nonempty description, e.g. synthetic")
        try:
            lower, upper = self.valid_load_range_n
        except (TypeError, ValueError) as exc:
            raise ValueError("valid_load_range_n must contain two finite bounds") from exc
        lower = _scalar(lower, "minimum valid load")
        upper = _scalar(upper, "maximum valid load")
        if not 0.0 <= lower < upper:
            raise ValueError("valid load bounds must satisfy 0 <= minimum < maximum")
        object.__setattr__(self, "valid_load_range_n", (lower, upper))
    def validate(self) -> None:
        scalar(self.radial_k_n_m, 'TireSpec.radial_k_n_m', positive=True)
        scalar(self.radial_c_ns_m, 'TireSpec.radial_c_ns_m', minimum=0.)
        scalar(self.pressure_pa_gauge, 'TireSpec.pressure_pa_gauge', minimum=0.)
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('TireSpec.provenance: nonblank description required')
        from bike_sim.physics.checks import array
        lower, upper = array(self.valid_load_range_n, 'TireSpec.valid_load_range_n', (2,))
        if not 0. <= lower < upper:
            raise ValueError('TireSpec.valid_load_range_n: expected 0 <= lower < upper')
```


## `src/bike_sim/physics/tire_calibration.py`

### `TireCalibration` (src/bike_sim/physics/tire_calibration.py:15-35)

```python
@dataclass(frozen=True)
class TireCalibration:
    dataset_id: str
    source_kind: str
    tyre_label: str
    radius_m: float
    inflated_width_m: float
    rim_internal_width_m: float
    material: TabulatedTireSpec
    def __post_init__(self):
        for name in ('dataset_id', 'tyre_label'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'{name} is required')
        if self.source_kind not in ('synthetic', 'measured'):
            raise ValueError('source_kind must describe input data provenance')
        for name in ('radius_m', 'inflated_width_m', 'rim_internal_width_m'):
            scalar(getattr(self, name), name, positive=True)
        if not isinstance(self.material, TabulatedTireSpec):
            raise ValueError('an explicit radial curve is required')
```


## `src/bike_sim/physics/tire_curve.py`

### `TabulatedTireSpec` (src/bike_sim/physics/tire_curve.py:16-92)

```python
@dataclass(frozen=True)
class TabulatedTireSpec:
    deflection_m: tuple[float, ...]
    force_n: tuple[float, ...]
    radial_c_ns_m: float
    pressure_pa_gauge: float
    provenance: str
    valid_load_range_n: tuple[float, float]
    def __post_init__(self):
        try:
            x = tuple(_scalar(v, 'deflection') for v in self.deflection_m)
            y = tuple(_scalar(v, 'elastic force') for v in self.force_n)
        except TypeError as exc:
            raise ValueError('tire curve requires deflection and force sequences') from exc
        if len(x) != len(y) or len(x) < 2:
            raise ValueError('tire curve requires at least two paired samples')
        if x[0] != 0. or y[0] != 0.:
            raise ValueError('tire curve must start at zero deflection and force')
        if any(b <= a for a, b in zip(x, x[1:])):
            raise ValueError('tire deflection must strictly increase')
        if any(b < a for a, b in zip(y, y[1:])) or y[-1] <= 0.:
            raise ValueError('elastic tire force must be nondecreasing and positive at the end')
        for name in ('radial_c_ns_m', 'pressure_pa_gauge'):
            value = _scalar(getattr(self, name), name)
            if value < 0.:
                raise ValueError(f'{name} must be nonnegative')
            object.__setattr__(self, name, value)
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('tire curve requires explicit provenance')
        try:
            lo, hi = (_scalar(v, 'valid load') for v in self.valid_load_range_n)
        except (TypeError, ValueError) as exc:
            raise ValueError('valid load range requires two finite bounds') from exc
        if not 0. <= lo < hi <= y[-1]:
            raise ValueError('valid load range must fit within the supplied force table')
        slopes = tuple((b-a)/(v-u) for u, v, a, b in zip(x, x[1:], y, y[1:]))
        integral = [0.]
        for u, v, a, b in zip(x, x[1:], y, y[1:]):
            integral.append(integral[-1]+.5*(a+b)*(v-u))
        _finite_result(*slopes, *integral)
        object.__setattr__(self, 'deflection_m', x)
        object.__setattr__(self, 'force_n', y)
        object.__setattr__(self, 'valid_load_range_n', (lo, hi))
        # Derived caches are deliberately not dataclass fields: configuration
        # serialization contains only inputs, never redundant/rounded caches.
        object.__setattr__(self, '_slopes', slopes)
        object.__setattr__(self, '_integral', tuple(integral))
```


## `src/bike_sim/physics/tyre.py`

### `TyreSpecs` (src/bike_sim/physics/tyre.py:35-128)

```python
@dataclass(frozen=True)
class TyreSpecs:
    name: str
    outer_radius_mm: float
    rim_radius_mm: float
    section_height_mm: float
    pressure_bar: float
    crr_reference: float
    casing_width_mm: float = 60.0
    compressed_casing_mm: float = 6.0
    area_factor: float = 0.46
    carcass_stiffness_n_mm2: float = 0.10
    contact_length_factor: float = 0.86
    rate_stiffening: float = 0.25
    rate_relaxation_s: float = 0.2
    loss_factor: float = 0.07
    hysteresis_rate_eps_mps: float = 0.01
    relaxation_length_mm: float = 60.0
    rim_stiffness_n_mm2: float = 28.0
    tread_loss_crr: float = 0.0
    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If a dimension or constant is out of its physical range.
        """
        positive = (
            "outer_radius_mm", "rim_radius_mm", "section_height_mm", "pressure_bar",
            "crr_reference", "casing_width_mm", "compressed_casing_mm", "area_factor",
            "contact_length_factor", "rate_relaxation_s", "hysteresis_rate_eps_mps",
            "relaxation_length_mm", "rim_stiffness_n_mm2",
        )
        for name in positive:
            if not getattr(self, name) > 0.0:
                raise ValueError(f"tyre '{self.name}': {name} must be positive, got {getattr(self, name)}")
        for name in ("carcass_stiffness_n_mm2", "rate_stiffening", "loss_factor", "tread_loss_crr"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"tyre '{self.name}': {name} must not be negative, got {getattr(self, name)}")
        if self.rim_radius_mm >= self.outer_radius_mm:
            raise ValueError(f"tyre '{self.name}': rim radius {self.rim_radius_mm} mm is not inside "
                             f"the outer radius {self.outer_radius_mm} mm")
        if self.rim_strike_deflection_mm <= 0.0:
            raise ValueError(f"tyre '{self.name}': compressed casing {self.compressed_casing_mm} mm "
                             f"leaves no travel above the rim")
        if self.area_factor > 1.0 or self.contact_length_factor > 1.0:
            raise ValueError(f"tyre '{self.name}': area and contact-length factors are at most 1")
        if self.loss_factor >= 1.0:
            raise ValueError(f"tyre '{self.name}': loss factor must be below 1, got {self.loss_factor}")
```

### `TierSpec` (src/bike_sim/physics/tyre.py:163-180)

```python
@dataclass(frozen=True)
class TierSpec:
    name: str
    n_rays: int
    half_angle_deg: float
    timestep_s: float
    discretised_brush: bool
```

### `TyreConfig` (src/bike_sim/physics/tyre.py:192-238)

```python
@dataclass(frozen=True)
class TyreConfig:
    model: str = DEFAULT_TYRE_MODEL
    tier: str = DEFAULT_TYRE_TIER
    front: TyreSpecs = field(default=FRONT_TYRE)
    rear: TyreSpecs = field(default=REAR_TYRE)
    surface: Optional[str] = None
    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: On an unknown model, tier or surface.
        """
        if self.model not in TYRE_MODELS:
            raise ValueError(f"unknown tyre model '{self.model}'; available: {', '.join(TYRE_MODELS)}")
        if self.tier not in TIERS:
            raise ValueError(f"unknown tyre tier '{self.tier}'; available: {', '.join(TYRE_TIERS)}")
        if self.surface is not None:
            from bike_sim.terrain.surface import SURFACES  # pure data, no cycle at import time

            if self.surface not in SURFACES:
                raise ValueError(f"unknown surface '{self.surface}'; available: {', '.join(SURFACES)}")
```


## `src/bike_sim/recording/catalog.py`

### `FieldSpec` (src/bike_sim/recording/catalog.py:47-125)

```python
@dataclass(frozen=True)
class FieldSpec:
    key: str
    dtype: str
    unit: str
    phase: str
    optional: bool
    measurement: str
    source: str
    description: str
    frame: str = 'n/a'
    path: tuple = ()            # path inside the row tree; default (key,)
    aliases: tuple = ()         # legacy flat names this field replaces
    shape: tuple | None = None
    axes: tuple | None = None
    except_keys: tuple = ()     # map<struct>: members owned by sibling fields
    children: tuple = ()        # struct/struct[]/map<struct> child schema
    def __post_init__(self) -> None:
        if not isinstance(self.key, str) or not self.key:
            raise ValueError('field key must be a non-empty string')
        if self.dtype not in DTYPES:
            raise ValueError(f'{self.key}: unknown dtype {self.dtype!r}')
        if self.phase not in PHASES:
            raise ValueError(f'{self.key}: unknown phase {self.phase!r}')
        if self.measurement not in MEASUREMENTS:
            raise ValueError(
                f'{self.key}: unknown measurement {self.measurement!r}')
        if not isinstance(self.optional, bool):
            raise ValueError(f'{self.key}: optional must be a bool')
        if not self.unit or not self.source or not self.description:
            raise ValueError(
                f'{self.key}: unit, source and description are required')
        if self.shape is not None and self.axes is not None \
                and tuple(self.shape) != (len(self.axes),):
            raise ValueError(f'{self.key}: shape/axes mismatch')
        object.__setattr__(self, 'path',
                           tuple(self.path) if self.path else (self.key,))
        object.__setattr__(self, 'aliases', tuple(self.aliases))
        object.__setattr__(self, 'except_keys', tuple(self.except_keys))
        object.__setattr__(self, 'children', tuple(self.children))
        if self.dtype in ('struct', 'struct[]', 'map<struct>') \
                and not self.children:
            raise ValueError(f'{self.key}: structured dtype needs children')
```

### `TableSpec` (src/bike_sim/recording/catalog.py:128-170)

```python
@dataclass(frozen=True)
class TableSpec:
    name: str
    kind: str                   # 'physics' | 'forces' | 'channels'
    keys: tuple
    fields: tuple
    description: str
    source: str
    claims: frozenset = frozenset()   # owned top-level ``channels`` roots
    exempt_keys: tuple = ()           # row keys owned by other tables
    def __post_init__(self) -> None:
        if self.kind not in ('physics', 'forces', 'channels'):
            raise ValueError(f'{self.name}: unknown table kind {self.kind!r}')
        object.__setattr__(self, 'keys', tuple(self.keys))
        object.__setattr__(self, 'fields', tuple(self.fields))
        object.__setattr__(self, 'claims', frozenset(self.claims))
        object.__setattr__(self, 'exempt_keys', tuple(self.exempt_keys))
        names = [k.key for k in self.keys] + [f.key for f in self.fields]
        if len(names) != len(set(names)):
            raise ValueError(f'{self.name}: duplicate field keys')
        paths = [f.path for f in self.fields]
        if len(paths) != len(set(paths)):
            raise ValueError(f'{self.name}: duplicate field paths')
        containers = {'struct', 'struct[]', 'map<struct>'}
        for terminal in self.fields:
            depth = len(terminal.path)
            if any(len(p) > depth and p[:depth] == terminal.path
                   for p in paths):
                if not (terminal.dtype.startswith('map<')
                        or terminal.dtype in containers):
                    raise ValueError(
                        f'{self.name}: field {terminal.key} at '
                        f'{terminal.path} must be a container to share a '
                        'prefix with deeper fields')
        if self.kind == 'channels' and not self.claims:
            raise ValueError(f'{self.name}: channel table must claim roots')
```

### `JointCoords` (src/bike_sim/recording/catalog.py:173-180)

```python
@dataclass(frozen=True)
class JointCoords:
    name: str
    joint_type: str             # 'free' | 'ball' | 'hinge' | 'slide'
    qposadr: int
    dofadr: int
```

### `Coordinate` (src/bike_sim/recording/catalog.py:183-192)

```python
@dataclass(frozen=True)
class Coordinate:
    index: int
    kind: str                   # 'qpos' | 'dof'
    joint: str
    component: str              # member label ('' for scalar joints)
    dof_kind: str               # 'translation' | 'rotation' | 'quaternion'
    unit: str
```

### `CoordinateCatalog` (src/bike_sim/recording/catalog.py:195-209)

```python
@dataclass(frozen=True)
class CoordinateCatalog:
    qpos: tuple
    dof: tuple
```

### `AliasSpec` (src/bike_sim/recording/catalog.py:212-220)

```python
@dataclass(frozen=True)
class AliasSpec:
    alias: str
    canonical: str              # 'table.field' (dot may index a member)
    index: int | None = None    # vector element when the alias names one
    parameter: str | None = None  # map member captured by a template
    note: str = ''
```


## `src/bike_sim/recording/contracts.py`

### `RunMetadata` (src/bike_sim/recording/contracts.py:71-91)

```python
@dataclass(frozen=True)
class RunMetadata:
    run_id: UUID
    started_at: datetime
    backend: str
    mode: str
    values: Mapping[str, Any] = field(default_factory=dict)
    def __post_init__(self) -> None:
        if not isinstance(self.run_id, UUID):
            raise ValueError('run_id must be a UUID')
        if not isinstance(self.started_at, datetime):
            raise ValueError('started_at must be a datetime')
        if self.started_at.tzinfo is None or self.started_at.utcoffset() is None:
            raise ValueError('started_at must be timezone-aware (UTC)')
        object.__setattr__(self, 'started_at', self.started_at.astimezone(timezone.utc))
        _text(self.backend, 'backend')
        _text(self.mode, 'mode')
        object.__setattr__(self, 'values', freeze(self.values))
```

### `InputEvent` (src/bike_sim/recording/contracts.py:94-117)

```python
@dataclass(frozen=True)
class InputEvent:
    episode_id: int
    event_id: int
    kind: str
    received_wall_s: float
    processed_step: int
    applied_step: int | None
    values: Mapping[str, Any] = field(default_factory=dict)
    def __post_init__(self) -> None:
        _integer(self.episode_id, 'episode_id')
        _integer(self.event_id, 'event_id')
        _text(self.kind, 'kind')
        _finite(self.received_wall_s, 'received_wall_s')
        _integer(self.processed_step, 'processed_step')
        if self.applied_step is not None:
            applied = _integer(self.applied_step, 'applied_step')
            if applied < self.processed_step:
                raise ValueError(
                    'applied_step precedes the step the input was processed')
        object.__setattr__(self, 'values', freeze(self.values))
```

### `Asset` (src/bike_sim/recording/contracts.py:120-136)

```python
@dataclass(frozen=True)
class Asset:
    name: str
    media_type: str
    sha256: str
    content: bytes
    def __post_init__(self) -> None:
        _text(self.name, 'asset name')
        _text(self.media_type, 'media_type')
        if not isinstance(self.sha256, str) or len(self.sha256) != 64 \
                or any(c not in '0123456789abcdef' for c in self.sha256):
            raise ValueError('sha256 must be a 64-digit lowercase hex digest')
        if not isinstance(self.content, bytes):
            raise ValueError('asset content must be bytes')
```

### `TableBatch` (src/bike_sim/recording/contracts.py:139-166)

```python
@dataclass(frozen=True)
class TableBatch:
    table: str
    columns: Mapping[str, Sequence[Any] | np.ndarray]
    row_count: int
    def __post_init__(self) -> None:
        _text(self.table, 'table')
        _integer(self.row_count, 'row_count')
        frozen = {}
        for name, column in self.columns.items():
            if len(column) != self.row_count:
                raise ValueError(
                    f'{self.table}.{name}: column length {len(column)} '
                    f'does not match row_count {self.row_count}')
            if isinstance(column, np.ndarray):
                column = freeze(column)
            else:
                column = tuple(column)
            frozen[str(name)] = column
        object.__setattr__(self, 'columns', MappingProxyType(frozen))
```

### `PieceBatch` (src/bike_sim/recording/contracts.py:169-184)

```python
@dataclass(frozen=True)
class PieceBatch:
    table: str
    row_count: int
    pieces: Mapping[str, Any]
    def __post_init__(self):
        _text(self.table, 'table')
        _integer(self.row_count, 'row_count')
        object.__setattr__(self, 'pieces', MappingProxyType(dict(self.pieces)))
```

### `PhysicalSource` (src/bike_sim/recording/contracts.py:187-192)

```python
class PhysicalSource(Protocol):
    row_count: int
```

### `ResearchSource` (src/bike_sim/recording/contracts.py:195-198)

```python
class ResearchSource(Protocol):
    """Owned control-rate history. No method may call a live environment."""

    def tables(self, *, batch_rows: int) -> Iterator[TableBatch]: ...
```

### `ResearchEvidence` (src/bike_sim/recording/contracts.py:201-224)

```python
@dataclass(frozen=True)
class ResearchEvidence:
    history: ResearchSource
    initial_integration_state: np.ndarray
    final_integration_state: np.ndarray
    replay_contract_version: int
    complete_control_steps: int
    replay_eligible: bool
    def __post_init__(self) -> None:
        if not callable(getattr(self.history, 'tables', None)):
            raise ValueError('research history must implement tables')
        if self.replay_contract_version != 3:
            raise ValueError('new recordings require research replay contract 3')
        _integer(self.complete_control_steps, 'complete_control_steps')
        if type(self.replay_eligible) is not bool:
            raise ValueError('replay_eligible must be a bool')
        for name in ('initial_integration_state', 'final_integration_state'):
            array = np.asarray(getattr(self, name))
            if array.dtype != np.dtype('float64') or array.ndim != 1:
                raise ValueError(f'{name} must be a one-dimensional float64 array')
            object.__setattr__(self, name, freeze(array))
        if self.initial_integration_state.shape != self.final_integration_state.shape:
            raise ValueError('initial and final integration state shapes differ')
```

### `EpisodeEvidence` (src/bike_sim/recording/contracts.py:227-253)

```python
@dataclass(frozen=True)
class EpisodeEvidence:
    episode_id: int
    generation: int
    physical: PhysicalSource
    summary: Mapping[str, Any]
    assets: tuple[Asset, ...]
    research: ResearchEvidence | None = None
    def __post_init__(self) -> None:
        _integer(self.episode_id, 'episode_id')
        _integer(self.generation, 'generation')
        physical = self.physical
        rows = getattr(physical, 'row_count', None)
        _integer(rows, 'physical.row_count')
        if not callable(getattr(physical, 'tables', None)):
            raise ValueError('physical must implement tables(batch_rows=...)')
        object.__setattr__(self, 'summary', freeze(self.summary))
        assets = tuple(self.assets)
        for asset in assets:
            if not isinstance(asset, Asset):
                raise ValueError('assets must contain Asset values')
        object.__setattr__(self, 'assets', assets)
        if self.research is not None and not isinstance(self.research, ResearchEvidence):
            raise ValueError('research must be ResearchEvidence or None')
```


## `src/bike_sim/recording/replay_input.py`

### `ReplayInput` (src/bike_sim/recording/replay_input.py:24-48)

```python
@dataclass(frozen=True)
class ReplayInput:
    summary: object
    manifest: object
    assets: object
    requested: tuple
    applied: tuple
    observations: tuple
    transitions: tuple
    initial_state: np.ndarray
    final_state: np.ndarray
    terrain_vertices: np.ndarray
    metrics: object
    episode_id: int
    container_schema_version: int | None
    def __post_init__(self):
        for name in ('summary', 'manifest', 'assets', 'requested', 'applied',
                     'observations', 'transitions', 'initial_state', 'final_state',
                     'terrain_vertices', 'metrics'):
            object.__setattr__(self, name, freeze(getattr(self, name)))
```


## `src/bike_sim/recording/research.py`

### `OwnedSample` (src/bike_sim/recording/research.py:33-58)

```python
@dataclass(frozen=True)
class OwnedSample:
    interval_id: int
    time_s: float
    end_time_s: float
    qpos: object
    qvel: object
    forces: object
    powers_w: object
    channels: object
```

### `OwnedPythonEpisode` (src/bike_sim/recording/research.py:61-75)

```python
@dataclass(frozen=True)
class OwnedPythonEpisode:
    generation: int
    samples: tuple
```


## `src/bike_sim/sim/research/backend.py`

### `ResearchBackend` (src/bike_sim/sim/research/backend.py:10-49)

```python
class ResearchBackend(Protocol):
    backend: str
    seed: int
    sim: Any
    config: Any
    sensor_config: Any
    observation: SensorObservation
    demand_nm: float | None
    metadata: dict
    execution: dict
    run_metadata: dict
    terminated: bool
    truncated: bool
    reason: str | None
    error: str | None
    numerically_valid: bool
```


## `src/bike_sim/sim/research/demand.py`

### `DemandProgram` (src/bike_sim/sim/research/demand.py:16-65)

```python
@dataclass(frozen=True)
class DemandProgram:
    keyframes: tuple[tuple[float, float], ...]
    def __post_init__(self):
        try:
            frames = tuple(tuple(k) for k in self.keyframes)
        except TypeError as exc:
            raise ValueError('demand keyframes must be a sequence of (time_s, torque_nm)') from exc
        if not frames or any(len(k) != 2 for k in frames):
            raise ValueError('demand needs at least one (time_s, torque_nm) keyframe')
        frames = tuple((scalar(t, 'demand keyframe time', minimum=0.),
                        scalar(nm, 'demand torque', minimum=0.)) for t, nm in frames)
        if frames[0][0] != 0. or any(b[0] <= a[0] for a, b in zip(frames, frames[1:])):
            raise ValueError('demand keyframes must start at zero and strictly increase')
        object.__setattr__(self, 'keyframes', frames)
        object.__setattr__(self, '_times', tuple(t for t, _ in frames))
```


## `src/bike_sim/sim/research/environment.py`

### `ExperimentConfig` (src/bike_sim/sim/research/environment.py:25-49)

```python
@dataclass(frozen=True)
class ExperimentConfig:
    control_period_s: float = .01
    actuator_delay_s: float = .005
    duration_s: float = 3.
    seed: int = 0
    record_decimation: int = 20
    wheelie_persistence_s: float = .02
    maximum_energy_residual_ratio: float = .05
    stop_on_model_violation: bool = True
    async_telemetry: bool = False
    def __post_init__(self):
        for name in ('control_period_s', 'duration_s', 'maximum_energy_residual_ratio'):
            scalar(getattr(self, name), name, positive=True)
        for name in ('actuator_delay_s', 'wheelie_persistence_s'):
            scalar(getattr(self, name), name, minimum=0.)
        if type(self.stop_on_model_violation) is not bool:
            raise ValueError('stop_on_model_violation must be a bool')
        if type(self.async_telemetry) is not bool:
            raise ValueError('async_telemetry must be a bool')
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError('experiment seed must be a nonnegative integer')
        if type(self.record_decimation) is not int or self.record_decimation < 1:
            raise ValueError('record_decimation must be a positive integer')
```

### `ResearchStep` (src/bike_sim/sim/research/environment.py:52-68)

```python
@dataclass(frozen=True)
class ResearchStep:
    observation: SensorObservation
    truth: WheelieTruth
    contact_state: str
    terminated: bool
    truncated: bool
    reason: str | None
    physics_steps: int
    numerically_valid: bool = True
    demand_nm: float | None = None  # appended last: step() builds ResearchStep positionally
    model_valid: bool = True
```


## `src/bike_sim/sim/research/quality.py`

### `EnergyQuality` (src/bike_sim/sim/research/quality.py:6-10)

```python
@dataclass(frozen=True)
class EnergyQuality:
    residual_ratio: float
    electrical_residual_j: float
    acceptable: bool
```


## `src/bike_sim/sim/research/rider_behavior.py`

### `RiderSignals` (src/bike_sim/sim/research/rider_behavior.py:13-19)

```python
@dataclass(frozen=True)
class RiderSignals:
    pitch_rate_up_rad_s: float = 0.
    specific_force_body_mps2: tuple[float, float, float] = (0., 0., 0.)
    saddle_load_n: float = 0.
    bar_load_n: float = 0.
    pedal_load_n: float = 0.
```

### `RiderBehavior` (src/bike_sim/sim/research/rider_behavior.py:22-26)

```python
@runtime_checkable
class RiderBehavior(Protocol):
    def reset(self, seed: int) -> None: ...

    def act(self, time_s: float, signals: RiderSignals) -> RiderPosture | None: ...
```


## `src/bike_sim/sim/research/rider_program.py`

### `RiderKeyframe` (src/bike_sim/sim/research/rider_program.py:16-27)

```python
@dataclass(frozen=True)
class RiderKeyframe:
    time_s: float
    posture: RiderPosture = field(default_factory=RiderPosture)
    human_torque_nm: float | None = None
    def __post_init__(self):
        scalar(self.time_s, 'rider keyframe time', minimum=0.)
        if not isinstance(self.posture, RiderPosture):
            raise ValueError('rider keyframe requires a RiderPosture')
        if self.human_torque_nm is not None:
            scalar(self.human_torque_nm, 'rider effort', minimum=0.)
```

### `RiderProgram` (src/bike_sim/sim/research/rider_program.py:30-119)

```python
@dataclass(frozen=True)
class RiderProgram:
    keyframes: tuple[RiderKeyframe, ...]
    reaction_delay_s: float = 0.
    def __post_init__(self):
        try:
            frames = tuple(self.keyframes)
        except TypeError as exc:
            raise ValueError('rider keyframes must be a sequence') from exc
        if not frames or any(not isinstance(f, RiderKeyframe) for f in frames):
            raise ValueError('rider program requires RiderKeyframes')
        if frames[0].time_s != 0. or any(b.time_s <= a.time_s for a, b in zip(frames, frames[1:])):
            raise ValueError('rider keyframes must start at zero and strictly increase')
        if len({f.human_torque_nm is None for f in frames}) != 1:
            raise ValueError('specify human effort in all keyframes or in none')
        if len({f.posture.pelvis_offset_m is None for f in frames}) != 1:
            raise ValueError('hip offsets must be explicit in all keyframes or in none')
        scalar(self.reaction_delay_s, 'rider reaction delay', minimum=0.)
        object.__setattr__(self, 'keyframes', frames)
        object.__setattr__(self, '_times', tuple(f.time_s for f in frames))
```


## `src/bike_sim/sim/research/rider_random.py`

### `RiderRandomSpec` (src/bike_sim/sim/research/rider_random.py:24-38)

```python
@dataclass(frozen=True)
class RiderRandomSpec:
    mass_kg: tuple = (60., 100.)
    height_m: tuple = (1.60, 1.85)
    torso_lean_rad: tuple = (0., .25)
    pelvis_pitch_rad: tuple = (0., .08)
    human_torque_nm: tuple = (0., 25.)
    def __post_init__(self):
        # Limits: posture bounds are RiderPosture's envelope; mass/height are sanity caps.
        for name, lo, hi in (('mass_kg', 20., 200.), ('height_m', 1., 2.3), ('torso_lean_rad', -.8, .8),
                             ('pelvis_pitch_rad', -.5, .5), ('human_torque_nm', 0., 1000.)):
            object.__setattr__(self, name, _range(getattr(self, name), name, lo, hi))
```


## `src/bike_sim/sim/research/sensor_noise.py`

### `NoiseTape` (src/bike_sim/sim/research/sensor_noise.py:17-37)

```python
@dataclass(frozen=True)
class NoiseTape:
    noise: np.ndarray
    dropout_uniform: np.ndarray
    def __post_init__(self):
        for values in (self.noise, self.dropout_uniform):
            if np.asarray(values).dtype.kind not in "fiu":
                raise ValueError("noise tape must contain real numeric values, not booleans or objects")
        noise = np.array(self.noise, dtype=np.float64, order='C', copy=True)
        dropout = np.array(self.dropout_uniform, dtype=np.float64, order='C', copy=True)
        if noise.ndim != 2 or noise.shape[1] != 9 or dropout.shape != (noise.shape[0],):
            raise ValueError('noise tape requires (count, 9) noise and (count,) dropout')
        if not np.isfinite(noise).all() or not np.isfinite(dropout).all():
            raise ValueError('noise tape must be finite')
        if np.any((dropout < 0.) | (dropout >= 1.)):
            raise ValueError('dropout uniforms must lie in [0, 1)')
        noise.setflags(write=False)
        dropout.setflags(write=False)
        object.__setattr__(self, 'noise', noise)
        object.__setattr__(self, 'dropout_uniform', dropout)
```


## `src/bike_sim/sim/research/sensors.py`

### `SensorConfig` (src/bike_sim/sim/research/sensors.py:8-37)

```python
@dataclass(frozen=True)
class SensorConfig:
    latency_s: float = .01
    acceleration_std_mps2: float = .05
    gyro_std_rad_s: float = .002
    encoder_std_rad_s: float = .01
    torque_std_nm: float = .05
    imu_enabled: bool = True
    sample_period_s: float = .001
    acceleration_bias_mps2: tuple[float, float, float] = (0., 0., 0.)
    gyro_bias_rad_s: float = 0.
    dropout_probability: float = 0.
    maximum_age_s: float = .1
    def __post_init__(self):
        for name in ('latency_s', 'acceleration_std_mps2', 'gyro_std_rad_s',
                     'encoder_std_rad_s', 'torque_std_nm', 'maximum_age_s'):
            scalar(getattr(self, name), name, minimum=0.)
        scalar(self.sample_period_s, 'sensor period', positive=True)
        scalar(self.gyro_bias_rad_s, 'gyro bias')
        bias = array(self.acceleration_bias_mps2, 'acceleration bias', (3,))
        object.__setattr__(self, 'acceleration_bias_mps2', tuple(map(float, bias)))
        if scalar(self.dropout_probability, 'dropout probability', minimum=0.) > 1.:
            raise ValueError('dropout probability must lie in [0, 1]')
        if type(self.imu_enabled) is not bool:
            raise ValueError('imu_enabled must be a bool')
```

### `SensorObservation` (src/bike_sim/sim/research/sensors.py:40-64)

```python
@dataclass(frozen=True)
class SensorObservation:
    time_s: float
    source_time_s: float
    valid: bool
    specific_force_body_mps2: tuple[float, float, float]
    pitch_rate_up_rad_s: float
    front_wheel_rad_s: float
    rear_wheel_rad_s: float
    crank_rad_s: float
    motor_torque_nm: float
    human_torque_nm: float
    def __post_init__(self):
        scalar(self.time_s, 'observation time', minimum=0.)
        scalar(self.source_time_s, 'sensor source time', minimum=0.)
        if self.source_time_s > self.time_s+1e-12:
            raise ValueError('sensor observation cannot come from the future')
        if not isinstance(self.valid, bool):
            raise ValueError('sensor validity must be a bool')
        force = array(self.specific_force_body_mps2, 'proper acceleration', (3,))
        object.__setattr__(self, 'specific_force_body_mps2', tuple(map(float, force)))
        for name in ('pitch_rate_up_rad_s', 'front_wheel_rad_s', 'rear_wheel_rad_s',
                     'crank_rad_s', 'motor_torque_nm', 'human_torque_nm'):
            scalar(getattr(self, name), name)
```


## `src/bike_sim/sim/ride/attachment_wrench.py`

### `AttachmentGeometry` (src/bike_sim/sim/ride/attachment_wrench.py:14-27)

```python
@dataclass(frozen=True)
class AttachmentGeometry:
    eq_id: int
    rider_jac: np.ndarray
    rider_columns: np.ndarray
    bike_jac: np.ndarray
    bike_columns: np.ndarray
    observable: bool
    normal: np.ndarray
    kind: str
    rotational: bool
    half_patch_m: float
    pull_direction: np.ndarray | None
```

### `AttachmentRaw` (src/bike_sim/sim/ride/attachment_wrench.py:30-42)

```python
@dataclass(frozen=True)
class AttachmentRaw:
    rider_jac: np.ndarray
    rider_qfrc: np.ndarray
    bike_jac: np.ndarray
    bike_qfrc: np.ndarray
    observable: bool
    normal: np.ndarray
    kind: str
    rotational: bool
    half_patch_m: float
    gap_m: float
    pull_direction: np.ndarray | None
```


## `src/bike_sim/sim/ride/balance_monitor.py`

### `BalanceLostEvent` (src/bike_sim/sim/ride/balance_monitor.py:13-17)

```python
@dataclass(frozen=True)
class BalanceLostEvent:
    time_s: float
    position_m: float
    speed_mps: float
```


## `src/bike_sim/sim/ride/constraint_forces.py`

### `ConstraintForceSnapshot` (src/bike_sim/sim/ride/constraint_forces.py:34-85)

```python
@dataclass(frozen=True)
class ConstraintForceSnapshot:
    interval_start_s: float
    interval_end_s: float
    qvel_start: np.ndarray
    components: Mapping[str, np.ndarray]
    def __post_init__(self) -> None:
        if (
            not isfinite(self.interval_start_s)
            or not isfinite(self.interval_end_s)
            or self.interval_end_s <= self.interval_start_s
        ):
            raise ValueError("invalid constraint interval")
        qvel = np.array(self.qvel_start, dtype=float, copy=True)
        if qvel.ndim != 1 or not np.isfinite(qvel).all():
            raise ValueError("invalid constraint velocity")
        object.__setattr__(self, "qvel_start", _frozen_copy(qvel))
        copied: dict[str, np.ndarray] = {}
        for name, force in self.components.items():
            value = np.array(force, dtype=float, copy=True)
            if value.shape != qvel.shape or not np.isfinite(value).all():
                raise ValueError("invalid constraint force")
            copied[name] = _frozen_copy(value)
        object.__setattr__(self, "components", MappingProxyType(copied))
```


## `src/bike_sim/sim/ride/contact_state.py`

### `ContactPatch` (src/bike_sim/sim/ride/contact_state.py:19-80)

```python
@dataclass(frozen=True)
class ContactPatch:
    point_m: np.ndarray
    normal: np.ndarray
    normal_load_n: float
    tangent_force_n: float
    slip_mps: float
    couple_world_nm: np.ndarray | None = None
    native_world_force_n: np.ndarray | None = None
    source_geom: str = "terrain"
    _tangent: np.ndarray = field(init=False, repr=False, compare=False)
    _world_force_n: np.ndarray = field(init=False, repr=False, compare=False)
    def __post_init__(self) -> None:
        point = _frozen_vector(self.point_m, "contact point")
        normal = _frozen_vector(self.normal, "contact normal")
        if abs(np.linalg.norm(normal) - 1.0) > 1e-8 or abs(normal[1]) > 1e-8:
            raise ValueError("expected a unit normal in the X-Z plane")
        scalars = tuple(float(value) for value in (
            self.normal_load_n, self.tangent_force_n, self.slip_mps
        ))
        if not all(isfinite(value) for value in scalars) or scalars[0] < 0.0:
            raise ValueError("invalid contact load, force or slip")
        if self.source_geom not in ("terrain", "catch_plane"):
            raise ValueError("contact source must be terrain or catch_plane")
        couple = _frozen_vector(
            np.zeros(3) if self.couple_world_nm is None else self.couple_world_nm,
            "contact couple",
        )
        native_force = (
            None if self.native_world_force_n is None
            else _frozen_vector(self.native_world_force_n, "native world force")
        )
        object.__setattr__(self, "point_m", point)
        object.__setattr__(self, "normal", normal)
        object.__setattr__(self, "couple_world_nm", couple)
        object.__setattr__(self, "native_world_force_n", native_force)
        for name, value in zip(
            ("normal_load_n", "tangent_force_n", "slip_mps"), scalars
        ):
            object.__setattr__(self, name, value)
        # Tangent and the derived world force are fixed at validation; every
        # later access reuses these instead of a fresh frozen vector per call.
        tangent = _frozen_vector(np.array([normal[2], 0.0, -normal[0]]), "tangent")
        object.__setattr__(self, "_tangent", tangent)
        object.__setattr__(self, "_world_force_n",
            native_force if native_force is not None else _frozen_vector(
                scalars[0] * normal + scalars[1] * tangent, "contact force"))
```

### `WheelContactSnapshot` (src/bike_sim/sim/ride/contact_state.py:83-198)

```python
@dataclass(frozen=True)
class WheelContactSnapshot:
    time_s: float
    patches: tuple[ContactPatch, ...]
    geometric_contact: bool
    interval_id: int = 0
    backend: str = "native_reference"
    wheel_axis_m: np.ndarray | None = None
    _world_force_n: np.ndarray = field(init=False, repr=False, compare=False)
    def __post_init__(self) -> None:
        time_s = float(self.time_s)
        if not isfinite(time_s) or time_s < 0.0:
            raise ValueError("contact snapshot needs finite nonnegative time")
        if (
            isinstance(self.interval_id, bool)
            or not isinstance(self.interval_id, Integral)
            or self.interval_id < 0
        ):
            raise ValueError("contact snapshot needs a nonnegative interval ID")
        if not isinstance(self.backend, str) or not self.backend:
            raise ValueError("contact snapshot needs a backend")
        patches = tuple(self.patches)
        if not all(isinstance(patch, ContactPatch) for patch in patches):
            raise ValueError("contact snapshot needs ContactPatch entries")
        axis = _frozen_vector(
            np.zeros(3) if self.wheel_axis_m is None else self.wheel_axis_m,
            "wheel axis",
        )
        object.__setattr__(self, "time_s", time_s)
        object.__setattr__(self, "interval_id", int(self.interval_id))
        object.__setattr__(self, "patches", patches)
        object.__setattr__(self, "geometric_contact", bool(self.geometric_contact))
        object.__setattr__(self, "wheel_axis_m", axis)
        # The patch set is fixed at validation, so the resultant is too: keep
        # the same frozen sum instead of rebuilding it on every access.
        object.__setattr__(self, "_world_force_n", _frozen_vector(
            sum((patch.world_force_n for patch in patches), np.zeros(3)),
            "wheel force",
        ))
```


## `src/bike_sim/sim/ride/contacts.py`

### `WheelLoadProvider` (src/bike_sim/sim/ride/contacts.py:86-90)

```python
class WheelLoadProvider(Protocol):
    front_outputs: WheelOutputs
    rear_outputs: WheelOutputs
```

### `TerrainContacts` (src/bike_sim/sim/ride/contacts.py:93-145)

```python
@dataclass(frozen=True)
class TerrainContacts:
    front_load_n: float
    rear_load_n: float
    front_support_n: float
    rear_support_n: float
    handlebar_load_n: float
    front_snapshot: WheelContactSnapshot | None = None
    rear_snapshot: WheelContactSnapshot | None = None
    front_controller_grounded: bool | None = None
    rear_controller_grounded: bool | None = None
```


## `src/bike_sim/sim/ride/control.py`

### `RideControl` (src/bike_sim/sim/ride/control.py:7-43)

```python
@dataclass(frozen=True)
class RideControl:
    motor_torque_nm: float | None = None
    motor_limit_nm: float | None = None
    human_torque_nm: float | None = None
    crank_target_rate_rad_s: float | None = None
    posture: RiderPosture | None = None
    rider_enabled: bool = True
    def __post_init__(self):
        for name in ('motor_torque_nm', 'motor_limit_nm', 'human_torque_nm', 'crank_target_rate_rad_s'):
            value = getattr(self, name)
            if value is not None:
                scalar(value, name, minimum=0.)
        if self.posture is not None and not isinstance(self.posture, RiderPosture):
            raise ValueError('posture must be a RiderPosture')
        if not isinstance(self.rider_enabled, bool):
            raise ValueError('rider_enabled must be a bool')
    def validate_for(self, config, rider_variant):
        if config.physics_mode != 'physical':
            raise ValueError('RideControl requires physical mode')
        if any(v is not None for v in (self.motor_torque_nm, self.motor_limit_nm, self.human_torque_nm)):
            if config.drive_mode not in ('crank_effort', 'articulated_effort'):
                raise ValueError('torque commands require a physical effort drive mode')
        if (self.posture is not None or not self.rider_enabled
                or self.crank_target_rate_rad_s is not None) and rider_variant != 'articulated_planar':
            raise ValueError('rider commands require articulated_planar')
```


## `src/bike_sim/sim/ride/distributed_tire_forces.py`

### `_StationState` (src/bike_sim/sim/ride/distributed_tire_forces.py:25-29)

```python
@dataclass
class _StationState:
    xi: float = 0.0
    tangent: np.ndarray | None = None
```


## `src/bike_sim/sim/ride/drivetrain.py`

### `CrankCommand` (src/bike_sim/sim/ride/drivetrain.py:56-71)

```python
class CrankCommand(NamedTuple):
    crank_torque_nm: float
    rider_torque_nm: float
    assist_torque_nm: float
    phase_rad: float
    cadence_rpm: float
    rider_power_w: float
    motor_power_w: float
    support_factor: float
    freewheel: bool
    cutoff_active: bool
    rider_limit: str
    brake_demand: float
    gear_teeth: int = 14
```


## `src/bike_sim/sim/ride/initial_state.py`

### `PhysicalInitialState` (src/bike_sim/sim/ride/initial_state.py:58-213)

```python
@dataclass(frozen=True)
class PhysicalInitialState:
    payload_json: str
```


## `src/bike_sim/sim/ride/leg_loop.py`

### `LegLoopGeometry` (src/bike_sim/sim/ride/leg_loop.py:10-17)

```python
@dataclass(frozen=True)
class LegLoopGeometry:
    thigh_m: float
    link_m: float
    knee_sign: float
    thigh_angle0_rad: float
    link_angle0_rad: float
    side_sign: float
```


## `src/bike_sim/sim/ride/metrics.py`

### `TravelStats` (src/bike_sim/sim/ride/metrics.py:47-61)

```python
@dataclass
class TravelStats:
    end: str
    travel_limit_mm: float
    max_mm: float
    max_pct: float
    p95_mm: float
    mean_mm: float
    bottom_out_threshold_mm: float
    bottom_outs: int
    top_outs: int
    max_compression_mps: float
    max_rebound_mps: float
```

### `AccelStats` (src/bike_sim/sim/ride/metrics.py:64-73)

```python
@dataclass
class AccelStats:
    point: str
    filter_hz: float
    rms_filtered_mps2: float
    peak_filtered_mps2: float
    peak_raw_mps2: float
    raw_peak_note: str = "raw peak includes solver contact transients (1-2 ms); compare filtered values"
```

### `RiderStats` (src/bike_sim/sim/ride/metrics.py:76-94)

```python
@dataclass
class RiderStats:
    torso: AccelStats
    pelvis: AccelStats
    saddle_lift_events: int
    saddle_lift_time_s: float
    saddle_gap_max_mm: float
    mean_saddle_share: float
    mean_pedal_share: float
    mean_bar_share: float
    min_saddle_load_n: float
    max_saddle_load_n: float
```

### `PotholeReport` (src/bike_sim/sim/ride/metrics.py:97-107)

```python
@dataclass
class PotholeReport:
    label: str
    kind: str
    start_m: float
    length_m: float
    declared_depth_mm: float
    effective_drop_front_mm: float
    effective_drop_rear_mm: float
```

### `RimStrikeStats` (src/bike_sim/sim/ride/metrics.py:110-120)

```python
@dataclass
class RimStrikeStats:
    x_m: float
    speed_mps: float
    peak_load_n: float
    peak_rim_force_n: float
    absorbed_energy_j: float
    start_time_s: Optional[float] = None
    end_time_s: Optional[float] = None
```

### `TyreStats` (src/bike_sim/sim/ride/metrics.py:123-138)

```python
@dataclass
class TyreStats:
    wheel: str
    peak_fz_n: float
    max_deflection_mm: float
    rim_strikes: int
    rim_strike_events: List[RimStrikeStats]
    wheelspin_time_s: float
    locked_time_s: float
    mean_dissipated_power_w: float
    crr_equivalent: float
    completed_rim_impact_episodes: int = 0
    rim_episode_gap_s: float = 0.001
    rim_episode_distance_m: float = 0.01
```

### `PedalBobStats` (src/bike_sim/sim/ride/metrics.py:141-186)

```python
@dataclass
class PedalBobStats:
    mean_cadence_rpm: float
    bob_frequency_hz: float
    shock_bob_amplitude_mm: float
    fork_bob_amplitude_mm: float
    crank_torque_mean_nm: float
    crank_torque_peak_nm: float
    assist_torque_mean_nm: float
    wheel_torque_mean_nm: float
    gear_ratio_check: float
    freewheel_fraction: float
    cutoff_fraction: float
    traction_limited_fraction: float
    assist_switches: int
```

### `RideSummary` (src/bike_sim/sim/ride/metrics.py:280-404)

```python
@dataclass
class RideSummary:
    track: str
    track_length_m: float
    n_potholes: int
    n_bumps: int
    n_obstacles: int
    target_speed_kmh: float
    outcome: str
    completed: bool
    crash: Optional[str]
    steps: int
    sim_time_s: float
    wall_clock_s: float
    sample_interval_s: float
    window_start_m: float
    window_rows: int
    mean_speed_kmh: float
    airborne_events: int
    airborne_time_s: float
    fork: TravelStats
    shock: TravelStats
    bar: AccelStats
    saddle: AccelStats
    potholes: List[PotholeReport] = field(default_factory=list)
    extras: Dict[str, Union[str, float]] = field(default_factory=dict)
    rider_variant: str = "lumped"
    rider: Optional[RiderStats] = None
    tyres: Dict[str, TyreStats] = field(default_factory=dict)
    pedals: Optional[PedalBobStats] = None
```


## `src/bike_sim/sim/ride/model_status.py`

### `ModelStatus` (src/bike_sim/sim/ride/model_status.py:9-45)

```python
@dataclass
class ModelStatus:
    counts: Counter = field(default_factory=Counter)
    first: dict | None = None
    last_interval: int = -1
    maximum_compression_fraction: float = .15
    maximum_linkage_error_m: float = .002
    numerically_valid: bool | str = 'not_evaluated'
    calibration_status: str = 'parameterized_unvalidated'
```


## `src/bike_sim/sim/ride/period_buffer.py`

### `RawStep` (src/bike_sim/sim/ride/period_buffer.py:9-21)

```python
@dataclass(frozen=True)
class RawStep:
    interval_id: int
    time_s: float
    end_time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    components: dict
    attachment_raw: dict
    actuator_force: np.ndarray
    qfrc_passive: np.ndarray
    contact_truth: dict
    metadata: dict = field(default_factory=dict)
```

### `PeriodReport` (src/bike_sim/sim/ride/period_buffer.py:114-120)

```python
@dataclass(frozen=True)
class PeriodReport:
    violations_by_step: tuple
    works: tuple
    first_failure: tuple | None
    attachments: tuple
    efforts: tuple
```


## `src/bike_sim/sim/ride/physical_samples.py`

### `PhysicalSample` (src/bike_sim/sim/ride/physical_samples.py:39-83)

```python
@dataclass(frozen=True)
class PhysicalSample:
    interval_id: int
    time_s: float
    end_time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    forces: Mapping
    channels: Mapping
    def __post_init__(self):
        if isinstance(self.interval_id, bool) or not isinstance(self.interval_id, int) or self.interval_id < 0:
            raise ValueError('invalid interval ID')
        t = scalar(self.time_s, 'state time', minimum=0)
        end = scalar(self.end_time_s, 'interval end', minimum=0)
        if end <= t:
            raise ValueError('force interval must have positive duration')
        def frozen(value, name, shape=None):
            if is_frozen_array(value) and value.ndim == 1 and (shape is None or value.shape == shape):
                return value
            return array(value, name, shape, readonly=True)
        q, v = frozen(self.qpos, 'sample qpos'), frozen(self.qvel, 'sample qvel')
        if q.ndim != 1 or v.ndim != 1:
            raise ValueError('sample coordinates must be vectors')
        forces = {name: frozen(f, name, v.shape) for name, f in self.forces.items()}
        object.__setattr__(self, 'qpos', q)
        object.__setattr__(self, 'qvel', v)
        object.__setattr__(self, 'forces', MappingProxyType(forces))
        # Ownership is transferred by the runtime; it builds a fresh channels
        # mapping and never mutates the sample after publication.
        object.__setattr__(self, 'channels', self.channels if isinstance(self.channels, Mapping) else dict(self.channels))
```


## `src/bike_sim/sim/ride/reference_monitor.py`

### `ReferenceMonitor` (src/bike_sim/sim/ride/reference_monitor.py:16-31)

```python
@dataclass
class ReferenceMonitor:
    strict: bool
    first_failure: tuple[float, tuple[str, ...]] | None = None
```


## `src/bike_sim/sim/ride/rider_contacts.py`

### `_SupportState` (src/bike_sim/sim/ride/rider_contacts.py:47-50)

```python
@dataclass
class _SupportState:
    xi: float=0.
    tangent: np.ndarray | None=None
```


## `src/bike_sim/sim/ride/rider_control.py`

### `RiderCommand` (src/bike_sim/sim/ride/rider_control.py:216-234)

```python
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
```


## `src/bike_sim/sim/ride/rider_dynamics.py`

### `ConstraintResponse` (src/bike_sim/sim/ride/rider_dynamics.py:13-21)

```python
@dataclass(frozen=True)
class ConstraintResponse:
    acceleration_offset: np.ndarray
    acceleration_matrix: np.ndarray
    force_offset: np.ndarray
    force_matrix: np.ndarray
    domain_matrix: np.ndarray
    domain_bound: np.ndarray
    mode: tuple
```


## `src/bike_sim/sim/ride/rider_state.py`

### `RoadSample` (src/bike_sim/sim/ride/rider_state.py:14-24)

```python
@dataclass(frozen=True)
class RoadSample:
    x_m: float
    height_m: float
    grade: float
    def __post_init__(self):
        scalar(self.x_m, 'road sample position')
        scalar(self.height_m, 'road sample height')
        scalar(self.grade, 'road sample grade')
```

### `RiderKinematicState` (src/bike_sim/sim/ride/rider_state.py:41-57)

```python
@dataclass(frozen=True)
class RiderKinematicState:
    qpos: tuple[float, ...]
    qvel: tuple[float, ...]
    rider_time_s: float
    road: tuple[RoadSample, ...]
    def __post_init__(self):
        qpos = tuple(self.qpos); qvel = tuple(self.qvel)
        if not all(type(v) is float for v in qpos+qvel) or not np.isfinite(qpos+qvel).all():
            raise ValueError('rider state coordinates must be finite floats')
        object.__setattr__(self, 'qpos', qpos); object.__setattr__(self, 'qvel', qvel)
        object.__setattr__(self, 'road', tuple(self.road))
        scalar(self.rider_time_s, 'rider state time', minimum=0.)
        if not all(isinstance(s, RoadSample) for s in self.road):
            raise ValueError('rider state road must be RoadSample entries')
```


## `src/bike_sim/sim/ride/support_geometry.py`

### `BoxPadContact` (src/bike_sim/sim/ride/support_geometry.py:18-33)

```python
@dataclass(frozen=True)
class BoxPadContact:
    point_m: np.ndarray
    normal: np.ndarray
    gap_m: float
    within_width: bool
    within_footprint: bool
    def __post_init__(self):
        for name in ('point_m', 'normal'):
            value = np.asarray(getattr(self, name), dtype=float)
            object.__setattr__(self, name, np.frombuffer(value.tobytes(), dtype=float))
```


## `src/bike_sim/sim/ride/telemetry_v2.py`

### `ForceSample` (src/bike_sim/sim/ride/telemetry_v2.py:11-40)

```python
@dataclass(frozen=True)
class ForceSample:
    time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    components: Mapping[str, np.ndarray]
    def __post_init__(self) -> None:
        if not isfinite(self.time_s):
            raise ValueError("force sample time is not finite")
        for name in ("qpos", "qvel"):
            value = getattr(self, name)
            if is_frozen_array(value) and value.ndim == 1:
                continue
            value = np.array(value, dtype=float, copy=True)
            if value.ndim != 1 or not np.isfinite(value).all():
                raise ValueError("force sample state shape or value mismatch")
            object.__setattr__(self, name, _frozen_copy(value))
        copied: dict[str, np.ndarray] = {}
        for name, force in self.components.items():
            if is_frozen_array(force) and force.shape == self.qvel.shape:
                copied[name] = force
                continue
            value = np.array(force, dtype=float, copy=True)
            if value.shape != self.qvel.shape or not np.isfinite(value).all():
                raise ValueError("force sample shape or value mismatch")
            copied[name] = _frozen_copy(value)
        object.__setattr__(self, "components", MappingProxyType(copied))
```


## `src/bike_sim/sim/ride/termination.py`

### `RunLimits` (src/bike_sim/sim/ride/termination.py:46-126)

```python
@dataclass(frozen=True)
class RunLimits:
    finish_x_m: float
    max_steps: int
    max_wall_clock_s: float = DEFAULT_MAX_WALL_CLOCK_S
    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If any bound is not positive, which would terminate every run
                immediately and report a cap as though the bike had failed.
        """
        if self.finish_x_m <= 0.0:
            raise ValueError(f"finish_x_m must be positive, got {self.finish_x_m}")
        if self.max_steps <= 0:
            raise ValueError(f"max_steps must be positive, got {self.max_steps}")
        if self.max_wall_clock_s <= 0.0:
            raise ValueError(f"max_wall_clock_s must be positive, got {self.max_wall_clock_s}")
```

### `RunOutcome` (src/bike_sim/sim/ride/termination.py:129-164)

```python
@dataclass(frozen=True)
class RunOutcome:
    reason: str
    steps: int
    sim_time_s: float
    wall_clock_s: float
    position_m: float
    crash: Optional[CrashEvent] = None
```


## `src/bike_sim/sim/ride/tire_forces.py`

### `_BrushState` (src/bike_sim/sim/ride/tire_forces.py:48-54)

```python
@dataclass
class _BrushState:
    xi: float = 0.
    tangent: np.ndarray | None = None
    point: np.ndarray | None = None
    segment: int | None = None
    center: np.ndarray | None = None
```


## `src/bike_sim/sim/ride/tyre/applier.py`

### `_WheelBinding` (src/bike_sim/sim/ride/tyre/applier.py:20-27)

```python
@dataclass(frozen=True)
class _WheelBinding:
    name: str
    body_id: int
    spin_dofadr: int
    contact_geom_id: int
```


## `src/bike_sim/sim/ride/tyre/brush.py`

### `BrushResult` (src/bike_sim/sim/ride/tyre/brush.py:21-27)

```python
@dataclass(frozen=True)
class BrushResult:
    force_n: float
    fully_sliding: bool
    friction_coefficient: float
```


## `src/bike_sim/sim/ride/tyre/carcass.py`

### `RimStrikeEvent` (src/bike_sim/sim/ride/tyre/carcass.py:22-30)

```python
@dataclass(frozen=True)
class RimStrikeEvent:
    x_m: float
    speed_mps: float
    peak_load_n: float
    peak_rim_force_n: float
    absorbed_energy_j: float
```

### `CarcassState` (src/bike_sim/sim/ride/tyre/carcass.py:33-98)

```python
@dataclass
class CarcassState:
    n_rays: int
    delta_prev_m: np.ndarray = field(init=False)
    elastic_prev_n: np.ndarray = field(init=False)
    maxwell_force_n: np.ndarray = field(init=False)
    initialized: bool = field(default=False, init=False)
    rim_strike_active: bool = field(default=False, init=False)
    rim_event_x_m: float = field(default=0.0, init=False)
    rim_event_speed_mps: float = field(default=0.0, init=False)
    rim_event_peak_load_n: float = field(default=0.0, init=False)
    rim_event_peak_force_n: float = field(default=0.0, init=False)
    rim_event_energy_j: float = field(default=0.0, init=False)
    def __post_init__(self) -> None:
        if self.n_rays < 3:
            raise ValueError(f"carcass needs at least 3 rays, got {self.n_rays}")
        self.delta_prev_m = np.zeros(self.n_rays, dtype=float)
        self.elastic_prev_n = np.zeros(self.n_rays, dtype=float)
        self.maxwell_force_n = np.zeros(self.n_rays, dtype=float)
```

### `CarcassResult` (src/bike_sim/sim/ride/tyre/carcass.py:101-124)

```python
@dataclass(frozen=True)
class CarcassResult:
    pressure_force_n: np.ndarray
    elastic_force_n: np.ndarray
    maxwell_force_n: np.ndarray
    hysteresis_force_n: np.ndarray
    rim_force_n: np.ndarray
    total_element_force_n: np.ndarray
    material_rate_mps: np.ndarray
    contact_patches: Tuple[Tuple[int, int], ...]
    contact_lengths_m: Tuple[float, ...]
    force_x_n: float
    force_z_n: float
    normal_load_n: float
    rim_energy_j: float
    rim_strike_active: bool
    rim_event: Optional[RimStrikeEvent]
```


## `src/bike_sim/sim/ride/tyre/geometry.py`

### `RoadProfile` (src/bike_sim/sim/ride/tyre/geometry.py:38-85)

```python
@dataclass(frozen=True)
class RoadProfile:
    x0_m: float
    dx_m: float
    z_m: np.ndarray
```

### `RayHits` (src/bike_sim/sim/ride/tyre/geometry.py:120-140)

```python
@dataclass
class RayHits:
    r_m: np.ndarray
    delta_m: np.ndarray
    road_x_m: np.ndarray
    road_z_m: np.ndarray
    coverage_event: bool
    airborne: bool
```


## `src/bike_sim/sim/ride/tyre/model.py`

### `TyrePatchOutput` (src/bike_sim/sim/ride/tyre/model.py:34-58)

```python
@dataclass(frozen=True)
class TyrePatchOutput:
    ray_start: int
    ray_stop: int
    surface_name: str
    normal_load_n: float
    support_n: float
    tangential_force_n: float
    normal_force_world_n: np.ndarray
    tangent_world: np.ndarray
    force_world_n: np.ndarray
    centroid_world_m: np.ndarray
    mean_deflection_m: float
    contact_length_m: float
    slip_ratio: float
    transient_slip_ratio: float
    sliding_speed_mps: float
    slip_speed_mps: float
    friction_coefficient: float
    fully_sliding: bool
    dissipated_power_w: float
```

### `WheelOutputs` (src/bike_sim/sim/ride/tyre/model.py:61-108)

```python
@dataclass(frozen=True)
class WheelOutputs:
    tyre_name: str
    pressure_bar: float
    hub_position_world_m: np.ndarray
    hub_velocity_world_mps: np.ndarray
    omega_forward_radps: float
    force_world_n: np.ndarray
    normal_load_n: float
    support_n: float
    mean_deflection_m: float
    contact_length_m: float
    slip_ratio: float
    transient_slip_ratio: float
    fully_sliding: bool
    rim_strike_active: bool
    rim_event: Optional[RimStrikeEvent]
    dissipated_power_w: float
    coverage_event: bool
    airborne: bool
    patches: Tuple[TyrePatchOutput, ...]
```

### `_PatchBrushState` (src/bike_sim/sim/ride/tyre/model.py:111-118)

```python
@dataclass
class _PatchBrushState:
    start: int
    stop: int
    kappa_prime: float = 0.0
    detailed: Optional[DiscretisedBrush] = None
```


## `src/bike_sim/sim/ride/virtual_rider.py`

### `CrashEvent` (src/bike_sim/sim/ride/virtual_rider.py:146-168)

```python
@dataclass(frozen=True)
class CrashEvent:
    cause: str
    time_s: float
    position_m: float
    pitch_rad: float
```


## `src/bike_sim/sim/ride/wheelie.py`

### `WheelieTruth` (src/bike_sim/sim/ride/wheelie.py:35-64)

```python
@dataclass(frozen=True)
class WheelieTruth:
    time_s: float
    front_load_n: float = 0.
    rear_load_n: float = 0.
    front_clearance_m: float = 0.
    rear_clearance_m: float = 0.
    relative_pitch_rad: float = 0.
    position_m: float = 0.
    speed_mps: float = 0.
    pitch_up_rad: float = 0.
    pitch_rate_up_rad_s: float = 0.
    axle_pitch_rad: float = 0.
    road_pitch_rad: float = 0.
    front_slip_mps: float = 0.
    rear_slip_mps: float = 0.
    com_x_m: float = 0.
    com_z_m: float = 0.
    def __post_init__(self):
        for f in fields(self):
            scalar(getattr(self, f.name), f.name)
        for name in ('time_s', 'front_load_n', 'rear_load_n', 'front_clearance_m', 'rear_clearance_m'):
            scalar(getattr(self, name), name, minimum=0.)
```


## `src/bike_sim/sim/ride/wheels.py`

### `WheelSpin` (src/bike_sim/sim/ride/wheels.py:20-39)

```python
@dataclass(frozen=True)
class WheelSpin:
    name: str
    dofadr: int
    radius_m: float
```


## `src/bike_sim/sim/telemetry.py`

### `SimModelHandles` (src/bike_sim/sim/telemetry.py:21-45)

```python
@dataclass
class SimModelHandles:
    jnt_main_pivot: int = -1
    jnt_shock: int = -1
    jnt_fork: int = -1
    site_p3_ss: int = -1
    site_p3_roc: int = -1
    site_bb: int = -1
    site_pfa: int = -1
    site_pra: int = -1
```


## `src/bike_sim/terrain/alpine.py`

### `AlpineGenSpec` (src/bike_sim/terrain/alpine.py:56-154)

```python
@dataclass(frozen=True)
class AlpineGenSpec:
    length_m: float = 120.
    lead_in_m: float = 10.
    lead_out_m: float = 15.
    transition_m: float = 2.
    ramp_len_m: tuple = (15., 30.)
    ramp_grade: tuple = (.15, .25)
    shelf_len_m: tuple = (4., 8.)
    shelf_grade: tuple = (.02, .06)
    wall_grade: tuple = (.26, .32)
    n_walls: int = 1
    counter_len_m: tuple = (3., 6.)
    counter_grade: tuple = (-.08, -.03)
    counter_prob: float = .3
    base_surface: str = 'hardpack'
    n_scree: tuple = (2, 3)
    scree_len_m: tuple = (6., 15.)
    scree_ramp_bias: float = .7
    scree_amp_m: tuple = (.010, .020)
    scree_corr_m: tuple = (.15, .20)
    n_wet: tuple = (0, 0)
    wet_len_m: tuple = (2., 5.)
    background_amp_m: tuple = (.005, .010)
    background_corr_m: tuple = (.20, .30)
    n_boulders: tuple = (2, 5)
    boulder_height_m: tuple = (.05, .15)
    boulder_len_m: tuple = (.6, 1.2)
    n_rock_gardens: tuple = (1, 2)
    rock_len_m: tuple = (4., 8.)
    rock_amp_m: tuple = (.03, .06)
    rock_corr_m: tuple = (.30, .45)
    n_water_bars: tuple = (1, 3)
    water_bar_depth_m: tuple = (.08, .15)
    water_bar_len_m: tuple = (1.0, 1.5)
    n_sharp_edges: tuple = (0, 0)
    sharp_edge_height_m: tuple = (.03, .08)
    single_contact_safe: bool = True
    min_contact_radius_m: float = .45
    max_kink_deg: float = 17.
    contact_window_m: float = .25
    def __post_init__(self):
        # JSON/TOML give lists; freeze them so the dataclass stays hashable/immutable.
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, list):
                object.__setattr__(self, f.name, tuple(value))
        for name in _RANGE_FIELDS:
            lo, hi = check_range(getattr(self, name), name,
                                 minimum=MIN_GRID_M if name in _GRID_RANGE_FIELDS else 0.)
            object.__setattr__(self, name, (float(lo), float(hi)))
        lo, hi = check_range(self.counter_grade, 'counter_grade', minimum=-1.)
        if hi >= 0.:
            raise ValueError('counter_grade must be negative: it is a short descent')
        object.__setattr__(self, 'counter_grade', (float(lo), float(hi)))
        for name in _COUNT_FIELDS:
            check_range(getattr(self, name), name, integer=True)
        for name in _POSITIVE_FIELDS:
            object.__setattr__(self, name, scalar(getattr(self, name), name, positive=True))
        for name in ('counter_prob', 'scree_ramp_bias'):
            value = scalar(getattr(self, name), name, minimum=0.)
            if value > 1.:
                raise ValueError(f'{name} must lie in [0, 1]')
            object.__setattr__(self, name, value)
        if type(self.n_walls) is not int or not 0 <= self.n_walls <= 1:
            raise ValueError('n_walls must be 0 or 1')
        if type(self.single_contact_safe) is not bool:
            raise ValueError('single_contact_safe must be a bool')
        if self.wall_grade[0] < self.ramp_grade[1]:
            raise ValueError('wall_grade must start at or above the top of ramp_grade')
        if self.base_surface not in SURFACES:
            raise ValueError(f'unknown base_surface {self.base_surface!r}; available: {", ".join(SURFACES)}')
        if self.transition_m < MIN_GRID_M:
            raise ValueError(f'transition_m must be at least {MIN_GRID_M} m: segments are spaced on a centimetre grid')
        # _segments walks the body on the rounded centimetre grid, so the fit is checked
        # on the same rounded terms rather than on the raw spec values.
        cursor = _r(self.lead_in_m + self.transition_m, 2)
        limit = _r(_r(self.length_m - self.lead_out_m, 2) - self.transition_m, 2)
        if _r(limit - cursor, 2) < self.ramp_len_m[0]:
            raise ValueError('the body must hold at least one shortest ramp between lead-in and lead-out')
        if self.single_contact_safe and self.n_sharp_edges[1] > 0:
            raise ValueError('sharp edges always put the 2D tyre on two supports; '
                             'set single_contact_safe = false to opt in')
```


## `src/bike_sim/terrain/contact_profile.py`

### `ProfileContact` (src/bike_sim/terrain/contact_profile.py:15-29)

```python
@dataclass(frozen=True)
class ProfileContact:
    point: np.ndarray
    normal: np.ndarray
    delta: float
    segment_id: int
    multi_support: bool
    def __post_init__(self):
        object.__setattr__(self, 'point', array(self.point, 'contact point', (2,), readonly=True))
        n = array(self.normal, 'contact normal', (2,), readonly=True)
        if not isclose(hypot(float(n[0]),float(n[1])),1.,rel_tol=0.,abs_tol=1e-10):
            raise ValueError('contact normal must be a unit vector')
        object.__setattr__(self, 'normal', n)
        object.__setattr__(self, 'delta', scalar(self.delta, 'penetration'))
```


## `src/bike_sim/terrain/generator.py`

### `TerrainGenSpec` (src/bike_sim/terrain/generator.py:34-98)

```python
@dataclass(frozen=True)
class TerrainGenSpec:
    length_m: tuple = (60., 120.)
    grade_max: float = .25          # nominal knots uniform in [0, grade_max]
    grade_peak: float = .30         # one optional steep stretch
    grade_peak_prob: float = .35
    roughness_amp_m: tuple = (.005, .030)
    roughness_len_m: tuple = (3., 10.)
    roughness_corr_m: tuple = (.25, .5)
    bump_height_m: tuple = (.02, .10)
    bump_length_m: tuple = (.5, 1.)
    edge_height_m: tuple = (.02, .08)
    drop_height_m: tuple = (.02, .10)
    roots_len_m: tuple = (3., 6.)
    base_surfaces: tuple = ('hardpack', 'asphalt', 'loose')
    section_surfaces: tuple = ('wet', 'loose')
    feature_types: tuple = ('bump', 'roots')
    section_len_m: tuple = (3., 15.)
    n_roughness: tuple = (2, 6)
    n_features: tuple = (1, 4)
    n_surface_sections: tuple = (0, 2)
    lead_in_m: float = 4.           # flat, clean start for equilibrium
    lead_out_m: float = 5.          # flat finish
    def __post_init__(self):
        # JSON/TOML give lists; freeze them so the dataclass stays hashable/immutable.
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, list):
                object.__setattr__(self, f.name, tuple(value))
        for name in ('length_m', 'roughness_amp_m', 'roughness_len_m', 'roughness_corr_m', 'bump_height_m',
                     'bump_length_m', 'edge_height_m', 'drop_height_m', 'roots_len_m', 'section_len_m'):
            lo, hi = _range(getattr(self, name), name)
            object.__setattr__(self, name, (float(lo), float(hi)))
        for name in ('n_roughness', 'n_features', 'n_surface_sections'):
            _range(getattr(self, name), name, integer=True)
        for name in ('grade_max', 'grade_peak', 'lead_in_m', 'lead_out_m'):
            scalar(getattr(self, name), name, minimum=0.)
        if self.grade_peak < self.grade_max:
            raise ValueError('grade_peak must not be below grade_max')
        if scalar(self.grade_peak_prob, 'grade_peak_prob', minimum=0.) > 1.:
            raise ValueError('grade_peak_prob must lie in [0, 1]')
        for name, allowed in (('base_surfaces', SURFACES), ('section_surfaces', SURFACES),
                              ('feature_types', FEATURE_TYPES)):
            value = getattr(self, name)
            if not isinstance(value, tuple) or not value or any(v not in allowed for v in value):
                raise ValueError(f'{name} must be a nonempty tuple of known surfaces')
        if self.length_m[0] < self.lead_in_m+self.lead_out_m+2.*CLEARANCE_M:
            raise ValueError('shortest track must leave a usable body after lead-in/out')
```


## `src/bike_sim/terrain/grade.py`

### `GradeProfile` (src/bike_sim/terrain/grade.py:8-55)

```python
@dataclass(frozen=True)
class GradeProfile:
    knots: tuple[tuple[float, float], ...]
    def __post_init__(self):
        try:
            raw = tuple(tuple(p) for p in self.knots)
            if len(raw) < 2 or any(len(p) != 2 for p in raw):
                raise ValueError('grade needs at least two station/grade pairs')
            if any(isinstance(v, bool) or not isinstance(v, Real) or not isfinite(v)
                   for p in raw for v in p):
                raise ValueError('grade knots must be finite real numbers')
            knots = tuple((float(x), float(g)) for x, g in raw)
            if knots[0][0] != 0. or any(b[0] <= a[0] for a, b in zip(knots, knots[1:])):
                raise ValueError('grade stations must start at zero and strictly increase')
        except TypeError as exc:
            raise ValueError('grade knots must be station/grade pairs') from exc
        object.__setattr__(self, 'knots', knots)
```


## `src/bike_sim/terrain/heightfield.py`

### `HeightFieldSpec` (src/bike_sim/terrain/heightfield.py:37-233)

```python
@dataclass
class HeightFieldSpec:
    nrow: int = 2
    ncol: int = 24001
    radius_x_m: float = 60.0
    radius_y_m: float = 0.5
    elevation_m: float = 3.4
    base_m: float = 0.5
    datum_z_m: float = 2.8
```


## `src/bike_sim/terrain/obstacles.py`

### `Obstacle` (src/bike_sim/terrain/obstacles.py:29-74)

```python
@dataclass
class Obstacle(ABC):
    start_m: float
```

### `SquareEdge` (src/bike_sim/terrain/obstacles.py:77-94)

```python
@dataclass
class SquareEdge(Obstacle):
    height_m: float = 0.090
    ledge_length_m: float = 0.250
```

### `Pothole` (src/bike_sim/terrain/obstacles.py:97-115)

```python
@dataclass
class Pothole(Obstacle):
    depth_m: float = 0.180
    hole_length_m: float = 0.600
```

### `Washboard` (src/bike_sim/terrain/obstacles.py:118-135)

```python
@dataclass
class Washboard(Obstacle):
    amplitude_m: float = 0.035
    wavelength_m: float = 0.900
    n_waves: int = 9
```

### `GOut` (src/bike_sim/terrain/obstacles.py:138-154)

```python
@dataclass
class GOut(Obstacle):
    depth_m: float = 0.350
    dip_length_m: float = 5.0
```

### `Drop` (src/bike_sim/terrain/obstacles.py:157-181)

```python
@dataclass
class Drop(Obstacle):
    height_m: float = 0.600
    ramp_m: float = 0.0
```

### `Kicker` (src/bike_sim/terrain/obstacles.py:184-233)

```python
@dataclass
class Kicker(Obstacle):
    height_m: float = 0.350
    ramp_m: float = 1.400
    gap_m: float = 2.500
    landing_angle_deg: float = 20.0
    landing_m: float = 3.000
```

### `Roots` (src/bike_sim/terrain/obstacles.py:236-273)

```python
@dataclass
class Roots(Obstacle):
    n_bumps: int = 7
    height_m: float = 0.080
    width_m: float = 0.250
    section_length_m: float = 7.0
    jitter: float = 0.35
    seed: int = 0
```

### `RockGarden` (src/bike_sim/terrain/obstacles.py:276-328)

```python
@dataclass
class RockGarden(Obstacle):
    section_length_m: float = 8.0
    amplitude_m: float = 0.060
    correlation_length_m: float = 0.250
    seed: int = 0
```

### `Bump` (src/bike_sim/terrain/obstacles.py:336-354)

```python
@dataclass
class Bump(Obstacle):
    height_m: float = 0.050
    bump_length_m: float = 0.400
```

### `TrapezoidBump` (src/bike_sim/terrain/obstacles.py:357-381)

```python
@dataclass
class TrapezoidBump(Obstacle):
    height_m: float = 0.050
    ramp_m: float = 0.150
    plateau_m: float = 0.300
```

### `SlopedPothole` (src/bike_sim/terrain/obstacles.py:384-415)

```python
@dataclass
class SlopedPothole(Obstacle):
    depth_m: float = 0.080
    hole_length_m: float = 0.500
    edge_m: float = 0.100
```

### `BowlPothole` (src/bike_sim/terrain/obstacles.py:418-435)

```python
@dataclass
class BowlPothole(Obstacle):
    depth_m: float = 0.080
    hole_length_m: float = 0.500
```

### `RoadRoughness` (src/bike_sim/terrain/obstacles.py:438-455)

```python
@dataclass
class RoadRoughness(RockGarden):
    section_length_m: float = 10.0
    amplitude_m: float = 0.003
    correlation_length_m: float = 0.200
    seed: int = 0
```

### `SteppedClimb` (src/bike_sim/terrain/obstacles.py:458-527)

```python
@dataclass
class SteppedClimb(Obstacle):
    steps: Tuple[Tuple[float, float], ...] = (
        (5.0, 15.0),
        (10.0, 15.0),
        (15.0, 15.0),
        (20.0, 15.0),
        (25.0, 15.0),
    )
    transition_m: float = 3.0
```


## `src/bike_sim/terrain/profile.py`

### `TrackSpec` (src/bike_sim/terrain/profile.py:24-104)

```python
@dataclass
class TrackSpec:
    name: str
    length_m: float
    obstacles: List[Obstacle] = field(default_factory=list)
    description: str = ""
    surface: str = DEFAULT_SURFACE
    grade_profile: GradeProfile | None = None
    surface_sections: tuple[SurfaceSection, ...] = ()
    def validate(self) -> None:
        """
        Checks that the layout is well formed.

        Raises:
            ValueError: If the track length is non-positive, the surface is not registered,
                an obstacle falls outside the track, or two obstacles overlap.
        """
        if not np.isfinite(self.length_m) or self.length_m <= 0.0:
            raise ValueError(f"track '{self.name}' has non-positive length {self.length_m}")
        if self.surface not in SURFACES:
            raise ValueError(
                f"track '{self.name}': unknown surface '{self.surface}'; "
                f"available: {', '.join(SURFACES)}"
            )

        if self.grade_profile is not None:
            if not isinstance(self.grade_profile, GradeProfile):
                raise ValueError('grade_profile must be a GradeProfile')
            if self.grade_profile.knots[-1][0] > self.length_m:
                raise ValueError('grade stations must fit inside the track')
        mapping = self.surface_map
        if any(s.end_m > self.length_m for s in mapping.sections):
            raise ValueError('material intervals must fit inside the track')
        ordered = self.sorted_obstacles
        for obs in ordered:
            if obs.length_m < 0.0:
                raise ValueError(f"track '{self.name}': {obs.label} has negative length")
            if obs.start_m < 0.0 or obs.end_m > self.length_m:
                raise ValueError(
                    f"track '{self.name}': {obs.label} spans "
                    f"[{obs.start_m:.3f}, {obs.end_m:.3f}] m, outside [0, {self.length_m:.3f}] m"
                )

        for prev, nxt in zip(ordered, ordered[1:]):
            if nxt.start_m < prev.end_m:
                raise ValueError(
                    f"track '{self.name}': {prev.label} ends at {prev.end_m:.3f} m but "
                    f"{nxt.label} starts at {nxt.start_m:.3f} m -- obstacles may not overlap"
                )
```


## `src/bike_sim/terrain/profile_distance.py`

### `ProfileDistance` (src/bike_sim/terrain/profile_distance.py:12-18)

```python
@dataclass(frozen=True)
class ProfileDistance:
    distance: np.ndarray
    normal: np.ndarray
    point: np.ndarray
    segment: np.ndarray
    nonsmooth: np.ndarray
```


## `src/bike_sim/terrain/road.py`

### `RoadGeneratorSpec` (src/bike_sim/terrain/road.py:47-95)

```python
@dataclass
class RoadGeneratorSpec:
    seed: int = 0
    runup_m: float = 10.0
    runout_m: float = 5.0
    min_gap_m: float = 1.0
    potholes_per_100m: float = 4.0
    pothole_depth_m: Range = (0.040, 0.120)
    pothole_length_m: Range = (0.300, 0.800)
    pothole_edge: Dict[str, float] = field(default_factory=lambda: {"sharp": 1.0})
    pothole_edge_m: float = 0.100
    bumps_per_100m: float = 6.0
    bump_height_m: Range = (0.030, 0.080)
    bump_length_m: Range = (0.300, 0.600)
    bump_shape: Dict[str, float] = field(default_factory=lambda: {"cosine": 1.0})
    bump_ramp_fraction: float = 0.3
    roughness_m: float = 0.003
    roughness_correlation_m: float = 0.200
    def validate(self) -> None:
        """
        Checks the spec for values the generator cannot honour.

        Raises:
            ValueError: On negative rates, inverted ranges, unknown shape names or
                weights that are all zero.
        """
        if self.potholes_per_100m < 0.0 or self.bumps_per_100m < 0.0:
            raise ValueError("defect rates must be non-negative")
        for name in ("pothole_depth_m", "pothole_length_m", "bump_height_m", "bump_length_m"):
            lo, hi = getattr(self, name)
            if lo < 0.0 or hi < lo:
                raise ValueError(f"{name} range ({lo}, {hi}) is not 0 <= low <= high")
        _check_weights("pothole_edge", self.pothole_edge, POTHOLE_EDGES)
        _check_weights("bump_shape", self.bump_shape, BUMP_SHAPES)
        if self.roughness_m < 0.0:
            raise ValueError("roughness_m must be non-negative")
        if not 0.0 < self.bump_ramp_fraction <= 0.5:
            raise ValueError("bump_ramp_fraction must be in (0, 0.5]")
```


## `src/bike_sim/terrain/surface.py`

### `SurfaceSpec` (src/bike_sim/terrain/surface.py:31-92)

```python
@dataclass(frozen=True)
class SurfaceSpec:
    name: str
    mu_peak: float
    mu_slide: float
    slip_stiffness_per_load: float
    stribeck_speed_mps: float = DEFAULT_STRIBECK_SPEED_MPS
    description: str = ""
    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If a coefficient is not positive, or sliding friction exceeds peak.
        """
        for label, value in (
            ("mu_peak", self.mu_peak),
            ("mu_slide", self.mu_slide),
            ("slip_stiffness_per_load", self.slip_stiffness_per_load),
            ("stribeck_speed_mps", self.stribeck_speed_mps),
        ):
            scalar(value, f"SurfaceSpec.{label}", positive=True)
            if not value > 0.0:
                raise ValueError(f"surface '{self.name}': {label} must be positive, got {value}")
        if self.mu_slide > self.mu_peak:
            raise ValueError(
                f"surface '{self.name}': mu_slide {self.mu_slide} exceeds mu_peak {self.mu_peak}"
            )
```

### `SurfaceSection` (src/bike_sim/terrain/surface.py:146-164)

```python
@dataclass(frozen=True)
class SurfaceSection:
    start_m: float
    end_m: float
    surface: str
    def __post_init__(self):
        from math import isfinite
        from numbers import Real
        for name in ('start_m', 'end_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(value):
                raise ValueError('surface interval stations must be finite real numbers')
            object.__setattr__(self, name, float(value))
        if not 0. <= self.start_m < self.end_m:
            raise ValueError('surface interval must satisfy 0 <= start < end')
        if not isinstance(self.surface, str) or self.surface not in SURFACES:
            raise ValueError('unknown surface material')
```


## `src/bike_sim/validation/plant_prefix.py`

### `ValidPrefix` (src/bike_sim/validation/plant_prefix.py:6-43)

```python
@dataclass
class ValidPrefix:
    seen: int = 0
    accepted: int = 0
    valid_until_s: float = 0.
    first_bad: dict | None = None
    front_normal_impulse_ns: float = 0.
    rear_normal_impulse_ns: float = 0.
```


## `src/bike_sim/validation/ride_cases.py`

### `Incline` (src/bike_sim/validation/ride_cases.py:15-23)

```python
@dataclass
class Incline(Obstacle):
    extent_m: float = 8.
    angle_deg: float = 10.
```
