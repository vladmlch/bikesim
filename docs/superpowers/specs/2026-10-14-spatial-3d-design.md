# Spatial (3D) ride model for anti-wheelie research — design

Date: 2026-10-14. Status: design settled in a structured interview with the user (every decision below was put to the user and confirmed); awaiting file review. Execution plan: `docs/superpowers/plans/2026-10-14-spatial-3d-master.md` and the phase plans `2026-10-14-spatial-3d-p0…p8-*.md`. Correctness method: `docs/adr/0002-spatial-3d-correctness-method.md`.

## 1. Goal and non-goals

**Goal.** Turn the planar (sagittal-plane) e-bike ride simulator into a full spatial simulation — free 6-DOF chassis, working steering, roll and yaw, lateral tyre forces and camber, laterally varying terrain, a rider that balances by steering — so that it can serve as the test bench for a future motor-torque controller that prevents front-wheel lift on rough climbs. The bench must expose what a real motor controller would see (a 6-axis IMU on the frame, wheel and crank speeds, torque sensor) and must keep the project's engineering discipline (zero-allocation native tick-path, deterministic bytes, energy accounting, full telemetry contract).

**Priority of 3D effects** (confirmed): (1) realistic 6-axis IMU where roll/yaw/lateral acceleration contaminate the pitch signal; (2) lateral terrain (rock under one tyre shoulder, rut, side slope) coupling roll and pitch and redistributing front load; (3) combined slip — cornering/side-slope grip reduces the longitudinal grip available before the rear spins up; (4) curved trails and path following — **optional later track**, not in the main line.

**Non-goals (explicit).**
- The anti-wheelie algorithm itself. The policy interface stays Python `act(observation, demand_nm) -> RideControl` (`src/bike_sim/sim/research/policy_session.py`); no C++ policy plug-in.
- The legacy stack (`physics_mode = "legacy"`, `RideSimulation`, sphere/pneumatic legacy tyres, lumped/seated rider, 65-channel recorder). Untouched.
- Any change to the planar mode's physics, bytes, tests or TOML files. `spatial = "planar"` is the default and existing configs do not change by one byte.
- A laterally articulated rider (hip/torso lateral DOFs). Deferred (stage 3, §7.4).
- In-process batched episodes across threads (optional track after the performance gate, §11.3).
- Standalone CLI frontend (does not exist today; not added).

## 2. Decisions ledger

| Branch of the design tree | Decision |
|---|---|
| Meaning of "3D" | Full spatial dynamics (6-DOF root, steering, balance), staged through a constrained 4-DOF root |
| Modes in scope | `physics_mode = "physical"` only |
| Planar mode | Preserved byte-for-byte as `spatial = "planar"` (default); shared code refactored only if every planar bitwise gate stays green |
| Language | All new tick-path physics, equilibrium/bootstrap: C++23. Python keeps builder, terrain generation, research orchestration, noise tape, policy, CLI, viewer window |
| Correctness | ADR 0002: external benchmarks + invariants + planar regression bridge + engineering tests; self-golden sha256 manifests in git |
| Tyre | New backend `pneumatic_3d`: ray grid over a torus (circular section, tread ignored), reusing the native pneumatic carcass/brush/relaxation kernels; fast tier = geometric grid + lumped 2D brush with camber; detailed tier = discretised 2D bristles at dt 0.25 ms. Both tyres (Magic Mary / Hans Dampf); one lateral coefficient set scaled by load and pressure |
| Terrain | Straight corridor ±1.5 m, grid z(x, y) at 5 mm × 10 mm, bicubic (Catmull-Rom) interpolation; features: 2D roughness, rocks with lateral offset, ruts, side slope up to ±15°; generated in Python, consumed by C++ from a binary file; MJCF hfield for rendering only |
| Added by the planner (not in the interview; confirm) | Steady-cornering benchmark (§9.2) with a test-only `yaw_rate_ref` controller input; `topology::Spatial` descriptor name; `kMaxPatches3d = 8`; viewer key `V` for the rider toggle |
| Chassis staging | Stage 1 constrained root (x, y, z, pitch; steer removed) → Stage 2 free root + steer joint + steer-torque actuator + balance controller → Stage 3 lateral rider (out of plan) |
| Physical-mode principle | No forces from the world during the scored episode. Bootstrap hold during settle only (§8.2). **Deviation from the interview:** the interview said `eq_active` weld; a MuJoCo weld constrains all six DOFs and would block suspension sag and pitch settling, so the hold is a bootstrap-only writer with the same status as `initial_front_brake = 1.0`. Needs the user's confirmation |
| Balance controller | Rider steering torque (internal force), roll/roll-rate + heading/lateral-offset feedback; reaction delay and torque cap are config fields, defaults 0 s / strength file |
| Equilibrium | Ported to C++ with an explicit settle time budget and the equilibrium cache fingerprint extended with 3D fields |
| Sensors | Observation v2: 6-axis IMU with `imu_mount`, existing latency/noise/bias/dropout model; `pitch_rate_up_rad_s` channel removed from the observation |
| Telemetry | Raw `qpos/qvel` in MuJoCo convention (`nq = nv + number of free joints`); derived roll/pitch/yaw as right-handed ZYX Euler angles of the frame quaternion in the world frame (x forward, y left, z up — so positive pitch is nose-**down**, positive roll is lean **right**), steer angle, lateral offset; `WheelieTruth.pitch_up_rad = −pitch` keeps its semantics (fact: `wheelie.py:101`, `observations.py:16`) |
| Config key | Top-level `spatial = "planar" | "3d"` plus `[spatial_3d]` table |
| Performance | Viewer ≥ 1× fast tier; headless ≥ 2× fast tier; detailed tier ungated. Intra-step multithreading planned from the start with bitwise invariance to thread count; GPU feasibility memo (Metal only, 2 agent-days, ADR 0003) |
| Frontends | Viewer (chase camera, roll/steer HUD), research/batch, checked replay 1e-9 — all three |
| Organisation | Base: `main` after `feat/native-pneumatic-tire` (and `feat/alpine-scree-track` if ready) merge. Phase 0 freezes five interface contracts; tracks then run in parallel. Agent-neutral plans. ≈45–55 agent-days |

## 3. Facts the design rests on

Gathered 2026-10-14 from this worktree (branch `feat/native-pneumatic-tire`, HEAD 26af002, with uncommitted native pneumatic edits).

| Fact | Where |
|---|---|
| Ride-mode root = `root_x`, `root_z`, `root_pitch`; no free-joint mode exists | `src/bike_sim/mujoco/frame.py:252-257` |
| Physical mode removes `steer_joint` ("stiff oblique hinge … leaks small lateral velocities") | `src/bike_sim/mujoco/physical_topology.py:38-49` |
| Legacy ride modes lock steering with ±0.01° range, stiffness 50000 | `src/bike_sim/mujoco/steering_fork.py:36-39` |
| Pitch assist is forbidden in physical mode (no world forces) | `src/bike_sim/physics/model_config.py:96-97` |
| Viewer "2D/3D" is a camera preset only | `src/bike_sim/sim/camera.py:23-38` |
| All tyre backends compute normal + longitudinal only; contact frame hard-coded `n = [nx, 0, nz]` | `sim/ride/tire_forces.py:208-210`, `pneumatic_tire_forces.py:371-378`, `native/src/writers/pneumatic_tire.cpp:136-139` |
| Pneumatic tyre = world-fixed ray fan in the x-z plane, radial carcass per ray (pressure × chord + k_c·δ, Maxwell branch, hysteresis, rim strike), lumped or discretised longitudinal brush with relaxation; tiers fast (64 rays) / detailed (256 rays, dt 0.25 ms) | `sim/ride/tyre/{geometry,carcass,brush,model}.py`, `physics/tyre.py:183-187` |
| Native pneumatic port is a bitwise port of that Python model | `native/src/pneumatic/*`, `native/src/writers/pneumatic_tire.cpp` |
| Terrain is `z = h(x)`; hfield has 2 rows; `compiled_profile_vertices` rejects lateral variation | `terrain/profile.py:4`, `terrain/heightfield.py:42-58`, `sim/ride/tire_forces.py:35-43` |
| Rider `articulated_planar`: root x/z/pitch + 11 hinges about Y; attachments are MJCF equalities (connect/weld) | `mujoco/articulated_rider.py`, `mujoco/reference_rider.py` |
| Native planar invariants: hinge axis {0,1,0}, `nq == nv`, "attachment wrench leaves the planar model", rider supports require planar ancestors, resistance `[1] == 0`, terrain Y axis, spring accounting "scalar planar joints", named root joints | `writers/drivetrain.cpp:103-105`, `rider/support_geometry.cpp:275-317`, `rider/attachment_wrench.cpp:313-327`, `writers/resistance.cpp:100-120`, `tyre/profile.cpp:233`, `runtime/step.cpp:710, 1028-1030` |
| `validate_supported` whitelist | `src/bike_sim/native/setup.py:34-55` |
| Research: policy `act()` once per control period (10 ms) in Python; native runs the window (`advance_control`); FIFO actuator delay 5 ms owned by native; noise tape pre-drawn in Python | `sim/research/{environment,policy_session,sensors}.py`, `native/src/runtime/research.cpp:130-147` |
| `SensorObservation` has 3-axis specific force but only `pitch_rate_up_rad_s` | `sim/research/sensors.py:40-51` |
| `WheelieTruth` / `classify_contact` thresholds: load 5 N, clearance 0.01 m, pitch 0.035 rad; persistence 20 ms | `sim/ride/wheelie.py:13-52, 108-190` |
| Motor: actuator `mid_drive` on `crank_spin`, 40 ms first-order lag, slew, Bosch CX Gen4 profile "recalled, unverified" | `physics/motor.py`, `physics/motor_profile.py:231-238`, `native/src/writers/drivetrain.cpp:141-144` |
| Equilibrium: Python only, damped relaxation + `scipy.optimize.least_squares`; `nq != nv → ValueError`; detailed-tier settle needs ≈60 s simulated time (tread creep) | `sim/ride/physical_equilibrium.py`, `equilibrium_refine.py:18-19`, `.worktrees/pneumatic-perf/output/pneumatic_perf_findings.md` |
| Native uses `mj_step`/`mj_forwardSkip`, `mj_jac`, `efc_*` introspection, `mj_contactForce`; no `mj_step1/2`; integrator `implicitfast` | `native/src/engine_call.cpp:211-247`, `rider/equality_reactions.cpp:36-52`, `mujoco/builder.py:183` |
| An IMU already exists: `accelerometer sensor_frame_accel` + `gyro sensor_frame_gyro` on `site_BB` (bottom bracket); native resolves both by name and **throws if missing**; channels `frame_specific_force_body_mps2` (3) and `frame_gyro_body_rad_s` (3); planar observation uses `−frame_gyro[1]` as `pitch_rate_up` | `mujoco/sensors.py:42-43`, `mujoco/frame.py:189`, `native/src/runtime/step.cpp:1042-1043, 1997-2000`, `sim/research/observations.py:16` |
| MuJoCo 3.12.0 thread API is `mju_threadpool(mjData*, int nthread)` (no `mju_threadPoolCreate`) | `.venv/…/mujoco/include/mujoco/mujoco.h:1629` |
| Native line counts: writers 10 541, runtime 10 681, rider 8 836, pneumatic 3 226, drivetrain 3 030, tyre 1 272, contact 75 | `wc -l native/src/**` |
| Research scenarios include a `flat` track: `RESEARCH_SCENARIOS = ('flat', 'uphill', 'rough_uphill', 'crest', 'low_grip', 'step_up')` | `terrain/research.py:7` |
| Viewer key `B` is deliberately unbound in ride mode; `N`/`M` adjust tyre pressure; `V` is free | `sim/ride/input.py:98-99, 126-127` |
| Performance (native, dt 1.25 ms, nv 26): compliant_2d RTF 8.9, distributed 7.4; no numbers for pneumatic; no threads in native | `output/rtf-ab/*/report.json`, `docs/superpowers/audits/2026-10-11-native-distributed-tire.md:150-152` |
| Telemetry: 924 flattened columns on a pneumatic welded run (count is backend-dependent), 147 of them 3-vector components with y ≈ 0; no roll/yaw/steer channels; catalog `recording/catalog.py` | real `generation-0001/telemetry.csv` |
| Golden artifacts are not committed; regenerated in-test; `tools/golden_episode.py` | `tests/reference/test_native_suspension.py:17-30` |
| Machine: Apple M4 Max, 16 CPU / 40 GPU cores, 64 GB, Metal 4; no CUDA | `sysctl`, `system_profiler` |
| Documentation drift: `CONTEXT.md` says C++17 (actual C++23) and "~100 channels" (actual 924); code cites `RIDE.md §3.1/§11.2` which do not exist in the tree | `CONTEXT.md`, `physics/tyre.py:4,13,166` |

## 4. Configuration

### 4.1 Keys

```toml
# physics TOML (examples/research/*.toml)
physics_mode = "physical"
spatial = "3d"                      # "planar" (default, unchanged) | "3d"

[spatial_3d]
root = "free"                       # "constrained" (x, y, z, pitch; steer removed) | "free" (freejoint + steer_joint)
corridor_half_width_m = 1.5         # off-corridor crash threshold; must match the terrain grid half-width
capsize_roll_rad = 1.0472           # |roll| above this ends the episode as crash:capsize

[spatial_3d.steer]                  # only with root = "free"
range_deg = 60.0
damping_nms_rad = 0.5
torque_cap_nm = "strength"          # number (N*m) or "strength" -> from joint_strength_path (new key "steer")
reaction_delay_s = 0.0              # neuromuscular delay of the balance controller
controller = "roll_heading"         # "off" (benchmarks) | "roll_heading"

[spatial_3d.steer.gains]            # speed-scheduled; see §8.3
roll_nm_rad = 60.0
roll_rate_nms_rad = 12.0
heading_nm_rad = 8.0
lateral_nm_m = 4.0
steer_rate_nms_rad = 1.0
schedule_speed_mps = [1.0, 3.0, 6.0, 10.0]
schedule_scale = [2.5, 1.5, 1.0, 0.8]

[spatial_3d.bootstrap]
settle_time_s = 80.0                # replaces max(40000, 20/dt); fast tier may lower it
hold_roll_nm_rad = 2000.0           # bootstrap stabiliser (settle only, §8.2)
hold_yaw_nm_rad = 2000.0
hold_lateral_n_m = 20000.0

[spatial_3d.imu]
mount_pos_m = "site_BB"            # frame-body coordinates, or the string "site_BB" = the existing bottom-bracket IMU site (default)
mount_rpy_rad = [0.0, 0.0, 0.0]
axis_misalignment_rad = [0.0, 0.0, 0.0]

[tires]
backend = "pneumatic_3d"            # new backend; "pneumatic" stays planar-only

[tires.pneumatic_3d]
tier = "fast"                       # "fast": 64 x 7 rays, lumped 2D brush, any dt | "detailed": 256 x 15 rays, discretised 2D bristles, dt 0.00025
lateral_coefficients = "default"    # name in tyre_lateral_reference.json
knife_edge = false                  # benchmark setting: lateral stiffness x 1000, relaxation lengths / 100
# [tires.pneumatic_3d.front] / [tires.pneumatic_3d.rear]: same per-tyre overrides as tires.pneumatic (pressure_bar, ...)
```

Rules:
- `spatial` defaults to `"planar"`; `[spatial_3d]` is rejected unless `spatial = "3d"`; `tires.backend = "pneumatic_3d"` requires `spatial = "3d"`; `spatial = "3d"` requires `tires.backend = "pneumatic_3d"` (the planar backends assert a planar contact frame and must not be reachable from 3D).
- `root = "constrained"` forbids `[spatial_3d.steer]`.
- The `[terrain]`/track file must be a 3D grid (`.terrain3d`) when `spatial = "3d"`; a planar `.toml` track is accepted and auto-extruded laterally flat (this is how the regression bridge runs).
- `src/bike_sim/native/setup.py::validate_supported` adds `spatial ∈ {planar, 3d}` and the backend pairing above; `src/bike_sim/native/schema.py` validates the new tables with `exact_keys`.

### 4.2 Builder

`generate_mujoco_xml(..., spatial="planar")`. For `"3d"`:
- Frame root: `root = "constrained"` → joints `root_x` (slide 1 0 0), `root_y` (slide 0 1 0), `root_z` (slide 0 0 1), `root_pitch` (hinge 0 1 0), in that order. `root = "free"` → `<freejoint name="root_free"/>`.
- Rider pelvis root (`articulated_rider.py:45-47`): same two variants (`rider_root_x/y/z/pitch` or `rider_root_free`). Rider internal hinges unchanged (axis 0 1 0 in body frame).
- Steering (`steering_fork.py`): `root = "free"` keeps `steer_joint` as a hinge on the steering axis with `range = ±range_deg`, `damping = damping_nms_rad`, no stiffness; adds actuator `<motor name="steer_torque" joint="steer_joint" ctrlrange="-cap cap"/>`. `root = "constrained"` removes it exactly as physical planar does.
- Wheels: `pneumatic_3d` sets wheel geoms `contype = conaffinity = 0` (as `pneumatic`); the wheel bodies keep their spin hinges.
- IMU: the existing `site_BB` sensors (`sensor_frame_accel`, `sensor_frame_gyro`) stay exactly as they are — native requires them and the planar channels `frame_*` keep their bytes. For `spatial = "3d"` the builder adds a second site `site_imu` (pose from `[spatial_3d.imu]`; with the default `"site_BB"` it coincides with `site_BB`) and sensors `sensor_imu_accel`, `sensor_imu_gyro`; these feed the `imu.*` truth channels and observation v2. With the default mount `imu.*` equals `frame_*` bitwise (test).
- Terrain: hfield `road` becomes `nrow = ny_render`, `ncol = nx_render` at render resolution (default 2 cm), filled from the 3D grid; `radius_y_m = corridor_half_width_m`. `catch_plane` unchanged.
- Benchmark model (§9.1): a new `mode = "benchmark_whipple"` that builds the Meijaard bicycle from its 27 parameters (rear frame + rigid rider as one body, front assembly, two wheels), `spatial = "3d"`, `root = "free"`, no suspension, no rider tree, no drivetrain, `controller = "off"`, `knife_edge = true`.

## 5. Interface contracts frozen in Phase 0

These five contracts are written as headers + stub tests in Phase 0 so tracks B–G can proceed in parallel against them. Changing a contract afterwards requires a note in the master plan's "Contract changes" table.

### 5.1 `TerrainQuery3D` (C++ `native/src/terrain/grid3d.hpp`)

```cpp
namespace terrain {
struct Sample { double z; std::array<double,3> normal; std::uint16_t surface_id; bool inside; };
struct RayHit { bool hit; double t; std::array<double,3> point, normal; std::uint16_t surface_id; };
class Grid3D {                     // immutable after load; shared by all writers
 public:
  static std::expected<Grid3D, std::string> load(std::span<const std::byte>);   // .terrain3d (§6.2)
  static Grid3D from_profile_extrusion(const nativecfg::PneumaticProfile&, double half_width_m); // 3 rows (−W, 0, +W), regression-bridge path
  Sample sample(double x, double y) const noexcept;                               // bicubic Catmull-Rom, analytic gradient
  RayHit ray_cast(std::span<const double,3> origin, std::span<const double,3> dir, double t_max) const noexcept; // DDA + bracketed refine
  double x0() const noexcept; double y0() const noexcept; double dx() const noexcept; double dy() const noexcept;
  std::size_t nx() const noexcept; std::size_t ny() const noexcept;
  const SurfaceTable& surfaces() const noexcept;    // friction zones, as today's surface map, keyed by surface_id
};
}
```
Guarantees: `noexcept`, allocation-free, thread-safe (const), deterministic. `sample` outside the grid returns `inside = false` with the clamped border value. Ray cast terminates in ≤ `ceil(t_max / min(dx, dy)) + 2` cells.

### 5.2 Tyre writer output and state wire

```cpp
struct PatchWrench {               // one per contact patch, world frame
  std::array<double,3> point_m, force_n, couple_nm;   // couple is the moment about point_m (brush/camber twisting)
  double normal_load_n, contact_length_m, camber_rad, slip_ratio, slip_angle_rad;
  std::uint16_t surface_id; bool valid;
};
struct TyreOutput { std::span<const PatchWrench> patches; bool airborne; bool rim_strike; /* diagnostics as in pneumatic */ };
```
Injection: `qfrc += J(point)ᵀ · force + J_rotᵀ · couple` via `mj_jac` on the wheel body (as `pneumatic_tire_forces.py:383-386`). The wheel spin for slip is `ω_spin = (ω_wheel − ω_hub) · axle_world`, never a world-axis component.

State wire (canonical names, `{side} ∈ {front, rear}`): `{side}.el.{i}.{j}.{delta_prev_m|elastic_prev_n|maxwell_n}`, `{side}.el.{i}.{j}.bristle.{x_m|y_m}` (detailed only), `{side}.carcass.initialized`, `{side}.rim.*` (as pneumatic), `{side}.patch.count`, `{side}.patch.{k}.{kappa_prime|alpha_prime}` for `k < kMaxPatches3d = 8`. Width fixed by tier; NaN for inactive slots. (The planar backend reserves `(n_rays+1)/2` slots; the 3D grid labels connected components, which are few — overflow above 8 is a counted diagnostic, §10.3.)

### 5.3 Config schema `spatial` / `validate_supported`

As §4.1. Wire: `config['spatial'] = {'mode': 'planar'|'3d', 'root': …, 'steer': {...}|None, 'bootstrap': {...}, 'imu': {...}, 'corridor_half_width_m', 'capsize_roll_rad'}`; `config['tire']['pneumatic_3d'] = {tier, n_circ, n_lat, half_angle_deg, lat_half_angle_deg, lateral: {...pinned coefficients...}, knife_edge, front: {...}, rear: {...}}`. C++: `nativecfg::SpatialConfig`, `nativecfg::Pneumatic3dConfig`.

### 5.4 Observation v2 (`SensorObservation`, schema field `schema = 2`)

```python
@dataclass(frozen=True)
class SensorObservationV2:           # schema 1 `SensorObservation` stays untouched for planar
    schema: int                      # 2
    time_s: float; source_time_s: float; valid: bool
    specific_force_body_mps2: tuple[float, float, float]   # IMU frame
    angular_rate_body_rad_s: tuple[float, float, float]    # IMU frame, right-handed (replaces pitch_rate_up_rad_s; note schema 1 used −gyro[1])
    front_wheel_rad_s: float; rear_wheel_rad_s: float; crank_rad_s: float
    motor_torque_nm: float; human_torque_nm: float
```
Noise tape: 6 IMU channels (accel std, gyro std, bias per axis) + the existing encoder/torque channels. Planar mode keeps schema 1 untouched; the research CLI selects the schema from `spatial`.

### 5.5 Telemetry 3D fields (schema-2 channel tree additions)

| Field | Unit / frame | Source |
|---|---|---|
| `root.quat_wxyz.{0..3}` | – | `qpos` of `root_free` (free root only) |
| `root.roll_rad`, `root.pitch_rad`, `root.yaw_rad` | right-handed ZYX Euler of the frame quaternion in the world frame (x fwd, y left, z up): positive pitch = nose-down, positive roll = lean right, positive yaw = nose left | derived |
| `root.lateral_offset_m` | world y of frame origin | `xpos` |
| `root.angular_rate_body.{0,1,2}` | rad/s, frame body | `qvel` |
| `steer.angle_rad`, `steer.rate_rad_s`, `steer.torque_nm`, `steer.controller_active` | – | joint + actuator |
| `tires.{side}.patches.{k}.{slip_angle_rad|camber_rad|lateral_force_n}` | world / patch | tyre writer |
| `imu.specific_force.{0,1,2}`, `imu.angular_rate.{0,1,2}` | `site_imu` frame, **truth** (noise is added on the research side); equals `frame_*` with the default mount | `sensor_imu_*` sensordata |
| `bootstrap.hold_active` | 0/1 | §8.2 (always 0 during the scored episode) |
| `truth.roll_rad`, `truth.yaw_rad`, `truth.heading_rad`, `truth.lateral_offset_m`, `truth.{side}_lateral_slip_mps` | – | `WheelieTruth` extension |

Existing `pitch_up_rad` (and `WheelieTruth.pitch_up_rad = −pitch`) keeps its definition and sign. `recording/catalog.py` declares every new field with type, unit, frame, source. `qpos.*`/`qvel.*` stay raw; with free roots `qpos` has 7 entries per free joint and `qvel` 6 (bike + rider pelvis → `nq = nv + 2`).

## 6. Terrain

### 6.1 Generator (Python, `src/bike_sim/terrain/generator3d.py`)

Reuses the longitudinal spec of `generator.py` (grade knots, roughness sections, bump/root features, surface sections, lead-in/out) and adds the lateral dimension. `(spec, seed)` fully determines the grid; one `np.random.default_rng(seed)`.

- Grid: `x ∈ [x0, x0 + L]` at `dx = 0.005 m`; `y ∈ [−W, W]`, `W = corridor_half_width_m = 1.5`, `dy = 0.010 m`. 300 m → 60 001 × 301 doubles (≈145 MB in memory while generating; stored as float32 after validating that float32 rounding keeps slopes under 0.1 % error — else float64).
- Base: `z0(x, y) = h(x) + y · tan(β(x))`, where `β(x)` is the side-slope profile: 0–3 banked sections, `|β| ≤ 15°`, smoothstep transitions over 2–5 m.
- 2D roughness: Gaussian random field with correlation lengths `(ℓx, ℓy)` from the roughness section (ℓy = 0.5–1.0 ℓx), amplitude as today (5–30 mm), generated by FFT filtering.
- Features (1–4 per track as today, each with a lateral placement law):
  - **rock**: superellipsoid bump, height 40–150 mm, footprint 0.15–0.5 m, lateral centre offset `y_c ∈ [−0.12, 0.12] m` so the tyre meets it on a shoulder; crest radius floor `MIN_CONTACT_RADIUS_M = 0.45` kept along x.
  - **rut**: longitudinal groove, depth 20–60 mm, width 60–120 mm, lateral centre `y_c ∈ [−0.2, 0.2] m`, length 2–8 m, wall slope ≤ 45°.
  - **root**: as today, extruded with a slight lateral skew (±10°).
  - **bump**, **edge**, **drop**: as today, extruded; edge/drop remain opt-in.
- Surface zones: as today, now rectangles in (x, y) with a `surface_id` table.
- Output: `.terrain3d` binary (§6.2) + a planar `RoadProfile` along `y = 0` for tools that still want one + the render hfield.

Planar track files (`*.toml` with a 1D spec) are accepted and extruded laterally flat; this is the regression-bridge path.

### 6.2 `.terrain3d` binary

Little-endian: magic `"BKT3"`, `u32 version = 1`, `f64 x0, y0, dx, dy`, `u64 nx, ny`, `u32 dtype (0 = f64, 1 = f32)`, `u32 n_surfaces`, surface table (`u16 id`, `f64 x_lo, x_hi, y_lo, y_hi`, `u16 surface_spec_index`), surface spec table (mu_peak, mu_slide, stribeck, slip_stiffness_per_load, tread_loss_crr — as `SurfaceSpec`), then `z[ny][nx]` row-major. sha256 of the file is part of the equilibrium cache fingerprint and the episode provenance.

### 6.3 Native `Grid3D`

Bicubic Catmull-Rom on the 4×4 neighbourhood; `normal = normalize(−∂z/∂x, −∂z/∂y, 1)` from the analytic derivative of the same spline (C¹ surface → continuous normals; load jumps come only from genuine features). Ray cast: 2D DDA over cells along the ray's (x, y) projection; in each cell, bracket `f(t) = ray_z(t) − z(x(t), y(t))` and refine by 8 bisection steps (deterministic count). Border: clamped.

## 7. Chassis, rider, stages

### 7.1 Stage 1 — constrained root

`root = "constrained"`: x, y, z, pitch. Roll and yaw do not exist kinematically — an honest constraint, like today's planar root, not a force. Steering removed (as physical planar). Purpose: validate terrain, tyre, IMU, telemetry and the native topology generalisation without instability. On a laterally flat track this model is the regression bridge (ADR 0002 Layer C). It stays a permanent configuration (regression fixture and a "no-balance" research mode).

### 7.2 Stage 2 — free root

`root = "free"`: freejoint + `steer_joint` + `steer_torque` actuator. Balance by the rider's steering torque (§8.3). New episode outcomes: `crash:capsize` (`|roll| > capsize_roll_rad`), `crash:off_corridor` (`|y| > corridor_half_width_m − 0.3`). The existing low-speed `balance_monitor` semantics stay (speed floor → `balance_lost`).

### 7.3 Rider

`articulated_planar` tree unchanged in its joints (all hinges about body-Y); only the pelvis root follows the frame root variant. Lateral forces pass through the connect/pin equality constraints (already 3D point constraints). Consequence for native: `rider/support_geometry.cpp`, `attachment_wrench.cpp`, `spindle_controller.cpp` must stop requiring planar ancestors and compute full 3D point Jacobians (they already use `mj_jac`; the planar reductions `relative_planar_jacobian` are replaced by 3D equivalents selected by topology, with the planar path left byte-identical).

### 7.4 Stage 3 — lateral rider (out of plan)

Hip lateral shift / torso roll / knee-out would need new rider joints, new strength data and a new controller. Documented as a follow-up; nothing in this plan precludes it (the steer controller lives in `rider/`, the attachments are already 3D).

## 8. Native topology generalisation, bootstrap, balance

### 8.1 Topology descriptor

`native/src/spatial_topology.hpp` (new) declares `topology::Spatial { enum Root {planar3, constrained4, free6}; int root_qposadr, root_dofadr; int steer_jnt, steer_act; bool has_lateral; }` computed once at entry and passed to writers. Rules replacing the planar invariants:

| Today | 3D rule |
|---|---|
| `nq == nv` ("scalar planar coordinates only") | scalar joints addressed via `jnt_qposadr`/`jnt_dofadr`; `nq == nv + n_free_joints` (2 with the rider pelvis) |
| hinge axis `{0,1,0}` in world | hinge axis `{0,1,0}` **in the joint's body frame** (unchanged check — MuJoCo `jnt_axis` is body-local; the world direction is read from `xaxis`) |
| wheel spin = world-Y angular velocity | `ω_spin = (ω_wheel − ω_hub) · axle_world` |
| "attachment wrench leaves the planar model" | 3D wrench; planar path kept byte-identical behind `Spatial::planar3` |
| resistance `[1] == 0` | aerodynamic drag along the body-x velocity; rolling terms via tyre |
| terrain "must preserve the planar Y axis" | planar backends keep the check; `pneumatic_3d` uses `Grid3D` |
| "spring accounting requires scalar planar joints" | spring accounting over scalar joints only, root excluded by address |
| named `root_x/root_z/root_pitch` | `Spatial` provides position/pitch accessors for each root kind |

Every planar `test_native_*.py` must remain bitwise after this refactor — that is the acceptance test of the refactor itself.

### 8.2 Bootstrap (equilibrium) in C++

`native/src/runtime/equilibrium.{hpp,cpp}` ports `solve_physical_equilibrium` (damped relaxation, `qvel` zeroed every 20 ms, brakes on, convergence `max|qacc| ≤ 0.05`) and `refine_equilibrium` (least squares over scalar `qpos` except `root_x`, residual `M·qacc/√diag M`, Accelerate `DGELSD` as already used in `rider/`). The time budget is `bootstrap.settle_time_s` (default 80 s), not `max(40000, 20/dt)`.

**Bootstrap hold.** A standing free-root bicycle is not an equilibrium. During settle only, a `BootstrapHoldWriter` applies a critically damped virtual spring on frame roll, yaw and lateral position (`hold_*` gains). It is a force from the world, and therefore: it is active only while `t < 0` (bootstrap time), it is forced to exactly zero at `t = 0` when the balance controller is armed, the telemetry channel `bootstrap.hold_active` must read 0 for every scored step, and the energy ledger books its work separately in bootstrap. The interview originally wrote this as an `eq_active` weld; a MuJoCo weld constrains all six DOFs and would prevent suspension sag and pitch settling, so the hold is a writer with the same bootstrap-only status as `initial_front_brake = 1.0` (flagged in the ledger as a deviation needing confirmation). **Release condition:** the hold wrench at `t = 0⁻` must be below 1 N / 1 N·m, otherwise the bootstrap fails with a diagnostic — releasing a loaded hold would be a step disturbance scored in the episode. In practice this requires a laterally flat start zone: the generator guarantees a flat lead-in (§6.1) and the extrusion path is flat everywhere.

Equilibrium cache (`sim/ride/equilibrium_cache.py`): fingerprint gains `spatial`, `.terrain3d` sha256, `pneumatic_3d` tier/coefficients, `imu` mount; stores the 3D tyre state wire.

### 8.3 Balance controller

`native/src/rider/steer_balance.{hpp,cpp}`, part of the rider (telemetry under `steer.*`). Per physics control tick (`control_period_s = 0.005 s`, 200 Hz — not the 10 ms research policy period):

```
e_roll  = roll_rad (right-handed about +x forward: positive = leaning right)
u = s(v) · ( roll_nm_rad·e_roll + roll_rate_nms_rad·roll_rate
           + heading_nm_rad·(yaw − yaw_ref) + lateral_nm_m·(y − y_ref) ) − steer_rate_nms_rad·steer_rate
u → delay buffer (reaction_delay_s, FIFO in control ticks) → clamp(±torque_cap) → ctrl[steer_torque]
```
`s(v)` is the piecewise-linear schedule. Sign convention: the steer hinge axis points up the head tube, so positive steer angle/torque turns the bar left; recovering from a right lean (positive roll) means steering right (negative steer). The implementation applies the sign so that positive gains always steer **into the fall**; the unit test pins "positive roll → negative steer torque". `yaw_ref = 0`, `y_ref = 0` in the straight corridor. The controller is "off" for the Meijaard benchmark. At runtime it is part of the rider: `RideControl.rider_enabled = false` disables it together with the rider's posture/effort intent (`native/src/rider/intent.cpp:350-356`); no separate `steer_enabled` is added (planner decision, see ledger). Telemetry under `steer.*` (§5.5).

### 8.4 IMU

Sensors `sensor_imu_accel`/`sensor_imu_gyro` on `site_imu` give exact specific force and angular rate in the mount frame after `mj_forward` (acc-stage sensors). Native copies them into the sample (`imu.*` truth channels) next to the untouched `frame_*` channels from `site_BB`; the research layer applies the noise tape, bias, latency and dropout exactly as today, now on six channels. `axis_misalignment_rad` is applied as a small rotation to the truth before noise. Observation v2 reports the raw right-handed body rates; the planar schema-1 sign flip (`−gyro[1]`) is not carried over — a policy that wants "pitch rate, nose-up positive" computes it.

## 9. Benchmarks and tests (ADR 0002 instantiated)

The steady-cornering gate (§9.2) and its test-only `yaw_rate_ref` controller input were added by the planner as a second external check of the lateral force path; they were not part of the interview and need the user's confirmation.

### 9.1 Meijaard 2007 fixture

`examples/benchmarks/meijaard2007.toml` (27 parameters from the paper's Table 1) → `mode = "benchmark_whipple"`. Test `tests/reference/test_spatial3d_meijaard.py`: for v ∈ {2, 4, 6, 8, 10} m/s, start upright at speed, apply a 0.5° roll perturbation, record roll and steer for 3 s at dt 0.25 ms (small amplitude; abort if |roll| > 5°), identify the dominant modes with a matrix-pencil fit; compare with the pinned Table 2 values (3 % / 0.05 s⁻¹ floor). Sweep v in 0.05 m/s steps around the expected weave (≈4.29) and capsize (≈6.02) speeds to locate the sign changes; gate ±0.2 m/s. `knife_edge = true`, `controller = "off"`, no rider tree, no drivetrain.

### 9.2 Steady cornering

`tests/reference/test_spatial3d_cornering.py`: flat `Grid3D`, free root, controller in a "hold lean" mode exposed only to tests (constant `yaw_ref` rate = v/ρ); after 5 s check lean angle and lateral force sum (ADR 0002 Layer A).

### 9.3 Planar regression bridge

`tests/reference/test_spatial3d_planar_regression.py` on three tracks: `examples/research/rough_uphill_savage.toml`, `examples/research/alpine_rocky.toml`, and the research scenario `flat` (`terrain/research.py`, seed 0), all with `viewer_physics_pneumatic.toml` physics (ADR 0002 Layer C numbers). Runs in `full`; a 2-s version runs in `native`.

### 9.4 Invariants

`test_spatial3d_invariants.py`: mirror symmetry (bitwise), constrained-root lateral zeros (bitwise), thread-count invariance (bitwise for N ∈ {1, 2, 4, 8}), energy residual, `mj_compareFwdInv`.

### 9.5 Tyre rigs

`src/bike_sim/validation/contact_manifold_rigs.py` gains 3D rigs (new module `contact_manifold_rigs_3d.py`): flat/camber sweep (γ ∈ {0, 5, 10, 20}°), slip-angle sweep (α ∈ {0, 1, 2, 4, 8}°), combined slip, shoulder rock, rut, side slope. `src/bike_sim/validation/contact_acceptance.py::contact_matrix_3d`. Flat-road consistency with planar `pneumatic` (2 %).

### 9.6 Self-golden manifests

`tools/golden_manifest.py`: `write(episode) -> manifest.json` with sha256 per channel group per 100-step window + first/last integration state bytes; `check(episode, manifest)`. Stored under `tests/reference/golden_manifests/spatial3d/`. Three manifests: `constrained_flat_10s`, `free_rocky_10s`, `meijaard_v6_3s`.

## 10. The `pneumatic_3d` tyre

### 10.1 Geometry

Torus: ring radius `R_ring = outer_radius − r_s`, section radius `r_s = casing_width / 2` (circular section; tread ignored). Rays originate on the ring centre circle at circumferential angles `θ_i` (n_circ over `±half_angle_deg = 75°` about the wheel-plane projection of world −Z, world-fixed as today) and lateral angles `φ_j` (n_lat over `±lat_half_angle_deg = 70°`), direction `d(θ, φ) = cos φ · r̂(θ) + sin φ · ŷ_wheel`, undeformed length `r_s`. Each ray is cast against `Grid3D`; deflection `δ_ij = max(0, r_s − t_hit)`. Element area `dA_ij = R_ring·dθ · r_s·dφ · cos φ` (torus metric). Fast tier 64 × 7, detailed 256 × 15.

### 10.2 Carcass (per element)

The planar law per ray (`carcass.py:236-291`) with the chord-based pressure term replaced by an areal one: `f_ij = [c_A · p · dA_ij · g(δ) + k_c · δ_ij · (dA_ij / r_s)]` with `g(δ)` the normalised chord function so that the flat-road integral over φ reproduces the planar chord law; Maxwell branch and hysteresis per element with semi-Lagrangian advection along θ only; rim strike when `δ_ij > r_s − (R_ring − R_rim − t_c)`. Element force acts along the ray toward the ring centre. **Consistency gate:** on a flat road at γ = 0 the grid reproduces the planar backend's load–deflection curve and contact length within 2 % at 1.38 and 1.72 bar (this transfers T1–T4 to the 3D model). The existing native kernels (`native/src/pneumatic/carcass.cpp`) are reused through a thin element adapter — not copied.

### 10.3 Brush

Patches = connected components of loaded elements (8-connectivity on the (i, j) grid). Per patch: `F_z`, centroid, normal (load-weighted element normals), longitudinal half-length `a`, camber `γ` (angle between wheel plane and patch normal), slip velocity at the centroid.

- **Fast tier (lumped):** relaxation for `κ'` and `α'` (`σ_x = relaxation_length_mm`, `σ_y` pinned from literature, default 0.8 σ_x); forces from the steady parabolic brush in combined form (Pacejka's brush model: `F = μF_z · (1 − (1 − θ_eq σ_eq)^3)` along the combined slip direction, with `θ` from `C_κ`, `C_α`), plus camber thrust `F_γ = C_γ(F_z, p) · γ` capped at `F_z · tan γ`, plus the aligning couple from the brush. Coefficients `C_κ/F_z` from `SurfaceSpec.slip_stiffness_per_load` (as today), `C_α/F_z` and `C_γ/F_z` from `tyre_lateral_reference.json` scaled by `(F_z/F_ref)^{n_F}·(p/p_ref)^{n_p}`.
- **Detailed tier (discretised):** one 2D bristle per loaded element, advected along θ at the tread speed, deflection clipped to `μ · f_ij` in the combined direction, dashpot `BRISTLE_DAMPING_S`; camber thrust emerges from the lateral velocity distribution across φ; requires dt 0.25 ms (gate as today).

`knife_edge = true`: `C_α, C_γ × 1000`, `σ_x, σ_y / 100`, `μ × 10` (benchmark only; rejected outside `benchmark_whipple`).

### 10.4 Lateral coefficient provenance

`examples/research/tyre_lateral_reference.json`: `{name, source, C_alpha_per_load_per_rad, C_gamma_per_load_per_rad, sigma_y_over_sigma_x, load_exponent, pressure_exponent, valid_load_n, valid_pressure_bar, provenance}`; pinned in Phase 0 from Dressel (2013), Doria et al. (2013), Cole & Khoo (2001). The physical ceiling `C_γ ≤ F_z·tan γ / γ` is asserted at load.

### 10.5 Determinism and threads

Elements are statically partitioned over the worker pool (§11); each thread writes its elements' results into preallocated arrays; all reductions (patch sums, qfrc) run on the calling thread in fixed element order using the existing pairwise-sum kernel. No atomics, no dynamic scheduling, no per-thread accumulators that depend on N. This yields identical bytes for every thread count (tested).

**Mirror symmetry rules** (so that ADR 0002's bitwise mirror test can hold): (1) every lateral evaluation is written in a form symmetric under `φ → −φ` / `y → −y` — Catmull-Rom is evaluated through its symmetric basis polynomials, not a one-sided Horner in `t`; element areas and ray directions are generated from `|φ_j|` and a sign; (2) reductions along φ use the **mirrored-pair tree**: first `a_j + a_{n_lat−1−j}` (commutative, hence mirror-invariant), then the pairs from the outside inward, the centre element (odd `n_lat`) last; (3) patch order for the final reduction is canonical: sort by `(θ_min, |y_centroid|, sign(y_centroid))` and sum mirror pairs first. **Pre-registered fallback:** if, despite these rules, a reduction cannot be made exactly mirror-invariant, the mirror test is bitwise for terrain, geometry and element layers and `≤ 1e-12` relative for patch sums and `qfrc`; the audit must name the offending reduction.

## 11. Performance, threads, GPU

### 11.1 Targets

Fast tier: interactive viewer mean achieved RTF ≥ 1.0 at 1× on `alpine_rocky_3d`; headless RTF ≥ 2.0. Detailed tier: measured, not gated. dt study in Phase 4: fast tier is expected to stay at 1.25 ms; if the steer/tyre lateral stiffness demands 0.5 ms, the targets apply at that dt.

### 11.2 Worker pool (`native/src/runtime/worker_pool.{hpp,cpp}`)

`N − 1` persistent worker threads + calling thread; work items are `(begin, end)` element ranges computed once per tier; spin-barrier synchronisation (no mutex/condvar on the tick-path; RTSan-clean); `--threads N` CLI flag and `BIKE_NATIVE_THREADS` env; default `min(8, hardware_concurrency/2)`. Used by the tyre writer first; other candidates (per-wheel parallelism, rider support contact math) are measured, not assumed. MuJoCo's own `mjThreadPool` (island solver) is measured in Phase 7 and enabled only if it gains and stays deterministic.

### 11.3 Optional later track: in-process batched episodes

N environments × threads for sweeps. Requires thread-safe runtime instances and zero shared mutable state; conflicts with RTSan on allocation paths. Scheduled only after the Phase 7 gate, as a separate plan.

### 11.4 GPU feasibility (Phase 8, ADR 0003)

Metal only (M4 Max). Three hypotheses with pre-registered kill criteria: (a) per-step offload of ray casting + carcass — expected loss to dispatch/sync latency (kill if GPU step ≥ 0.5 × CPU step at ≤ 10⁴ rays); (b) batched-episode physics on GPU (MJX/Warp style) — requires reimplementing MuJoCo + tyre; kill unless ≥ 20× sweep throughput at ≤ 2 agent-weeks is credible; (c) offline uses (terrain generation, calibration sweeps, telemetry post-processing). Deliverable: memo + microbenchmarks + ADR 0003 go/no-go. Timebox 2 agent-days.

## 12. Frontends

- **Viewer:** `3d` preset becomes a filtered chase camera (azimuth follows yaw with a 0.5 s EMA; elevation −15°, distance 3.2 m); `2d` preset stays world-fixed side view, lookat follows x and y. HUD gains roll, steer angle, steer torque, lateral offset, capsize margin. Replica-mjData frames carry the free-root quaternion unchanged (full `mjSTATE_INTEGRATION`).
- **Viewer key:** `V` toggles the rider (and with it the balance controller) through the existing control path; `B` stays unbound as today.
- **Research/batch:** `bike-research --spatial 3d` (or from the physics TOML), `--terrain3d path`, `--threads N`; sweeps unchanged (process-level).
- **Checked replay:** unchanged mechanism; quaternion normalisation is deterministic, 1e-9 check holds.
- **CLI `bike-ride`:** same flags.

## 13. Documentation repairs (Phase 0)

`CONTEXT.md`: C++17 → C++23; "~100-channel contract" → "schema-2 channel tree (~920 flattened columns)"; add 3D terms. `docs/RIDE.md`: add the pneumatic section the code cites as §3.1 and the literature list cited as §11.2 (from `physics/tyre.py:245-273` and the pneumatic docstrings), then the 3D capability sentences. `docs/agents/native-engineering.md`: fix the two dangling links to non-existent spec/plan files (point to this spec and ADR 0002 where appropriate).

## 14. Risks

| Risk | Mitigation |
|---|---|
| Native topology refactor breaks planar bytes | Refactor behind `Spatial::planar3` paths; every planar `test_native_*.py` bitwise at every task; refactor is its own phase with no new physics |
| Balance controller cannot hold slow climbs (< 2 m/s) | Speed schedule + lateral gain; episode end `balance_lost` is a legitimate outcome; human-delay sweeps are research, not gates |
| Lateral tyre literature is sparse/inconsistent | One coefficient set with explicit provenance and validity ranges; physical ceiling asserted; bands not point values |
| Detailed tier settle (≈60 s simulated) too slow in 3D | Settle budget is a config field; cache; fast tier for interactive work |
| Thread-count invariance constrains algorithm choice | Design rule §10.5 from day one; test for N ∈ {1,2,4,8} in `native` profile |
| Grid memory (145 MB f64) in viewer | f32 storage after validation; render hfield at 2 cm |
| Meijaard identification noise | Small perturbation, 3 s window, matrix pencil; tolerance floor 0.05 s⁻¹ |
