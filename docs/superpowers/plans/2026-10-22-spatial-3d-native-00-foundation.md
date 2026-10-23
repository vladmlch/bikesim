# Пространственная C++-симуляция — 00. Основание: frozen headers, контракты, harness, config (W-фаза)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Все задачи этого документа, кроме S00, — **one-shot write**: исполнитель читает перечисленные файлы, пишет свои файлы целиком и коммитит. Запрещены configure/build/compile/syntax-only/ctest/запуск/генерация fixtures прогоном. Вся проверка — `2026-10-22-spatial-3d-native-09-verification.md` (V00, V01).

**Goal:** Дать всем последующим W-задачам одну базу: зафиксированный SHA, полный набор frozen public headers, machine-readable реестры контрактов/ссылок, C++-тестовый контур и native spatial config, против которых пишется всё остальное без компиляции.

**Architecture:** W00 — единственный автор новых публичных headers (declarations, константы, layouts, private storage). W01–W05 пишут JSON-реестры, ADR, harness, `config.cpp` и тесты. Поля, требующие выполнения, записываются `null` + `pending_verification`.

**Tech Stack:** C++23, CMake ≥3.27, CTest, существующий `bt` harness (`native/tests/support/migration_test.hpp`: `bt::run`, `bt::require`, `bt::invalid`, `bt::fixture`), `bike::io::Value`, root build с `BIKE_BUILD_PYTHON_BINDINGS=OFF`.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`; master; `2026-10-22-spatial-3d-native-contracts.md` (C0–C10).

## Global Constraints

- `spatial="planar"` остаётся режимом по умолчанию; planar canonical bytes, legacy wire/key sets, replay identity и platform goldens не меняются.
- Новые пути в `Create` — результаты W-задач; их наличие в плане не означает наличия в checkout.
- ADR 0002/0003 заняты; W02 пишет `docs/adr/0004-...` со статусом `proposed`; V01 проверяет свободный номер по checkout и при коллизии переименовывает.
- Bitwise comparison относится к одному execution artifact/provider.
- Missing fixture / `null` identity / нулевое число проверок — fail-closed в тесте; W пишет такие тесты, V их наполняет.
- Существующие `CoreStatus` (`native/src/writers/writer_types.hpp:55`: `ok, invalid_input, pending_actuation, invalid_geometry, engine_failure`) не расширяется в W00; подробные причины — в `pneumatic3d::ReasonCode3d` и аналогичных cold reason codes.

## Файловая структура и владельцы

| Путь | Назначение | Владелец |
|---|---|---|
| `migration/spatial3d/baseline.json` | Выбранный committed base, WIP dispositions | S00 (интегратор) |
| `migration/spatial3d/contract-deviations.md` | Журнал расхождений W-кода с frozen headers | W00 создаёт; любая W дописывает |
| `migration/spatial3d/upstream-gates.json` | G0–G6 с полями статуса/evidence (`pending_verification`) | W01 |
| `migration/spatial3d/contracts.json` | Типы/схемы/владельцы/consumers | W02 |
| `migration/spatial3d/references.json` | Источники, таблицы, единицы, uncertainty | W03 |
| `tests/fixtures/spatial3d/` | Data-only эталоны, аналитические случаи, manifests | Владельцы подсистем |
| `native/tests/spatial/` | C++-тесты и `support/` | Владельцы подсистем |
| `native/src/spatial/*.hpp`, `terrain/*3d.hpp`, `pneumatic3d/*.hpp`, … | Frozen headers | W00 |

---

## S00 — Зафиксировать базу (единственная задача до W; read-only git)

**Dependencies:** нет. **Create:** `migration/spatial3d/baseline.json`, `docs/superpowers/audits/2026-10-22-spatial-3d-base.md`. **Разрешённые действия:** только чтение git/файлов и создание ветки. Сборка, тесты, приложения не запускаются.

- [ ] Выполнить `git --no-optional-locks rev-parse --verify impl/python-free-cpp^{commit}`, `git --no-optional-locks worktree list --porcelain`, `git --no-optional-locks status --porcelain=v1 --untracked-files=all`. Отдельно учесть ignored dependencies/fixture payloads.
- [ ] Сверить PFC, lane/b и task/t20 по конкретным файлам. Сохранить исходное содержимое tracked/untracked работ (patch-файлы с SHA-256) до любого переноса; для каждой правки записать `integrate | already_equivalent | retain_separate` и основание.
- [ ] Выбрать чистый committed checkpoint PFC; создать `impl/spatial-3d-native` от него. Не удалять существующие каталоги/ветки.
- [ ] Записать в `baseline.json`: `base_sha` (40 hex), `parent_branch`, `tree_status`, `timestamp_utc`, `source_export_sha256`, `dependency_identities` (пути lock-файлов + их SHA-256), `wip_patches[]` (`path, sha256, owner, disposition, rationale`), `missing_binary_assets[]` с `blocks: ["V0x"]`.
- [ ] Проверить отсутствие вымышленных ancestry claims: пять JSON-приложений старого аудита отсутствуют в repomix.
- [ ] Коммит только документации основания: `docs(spatial): S00 fix base sha and preserve WIP`.

**Acceptance (проверяется W04-тестом в V01):** `base_sha.size()==40`, только `[0-9a-f]`, каждая WIP-правка имеет disposition.

---

## W00 — Freeze всех новых публичных headers

**Dependencies:** S00. **Create (production headers):**
`native/src/spatial/{types.hpp,config.hpp,topology.hpp,frames.hpp,bootstrap_state.hpp,brake_state.hpp,outcomes.hpp}`,
`native/src/terrain/{grid3d.hpp,asset3d.hpp,interpolation3d.hpp,raycast3d.hpp,spec3d.hpp,generator3d.hpp,extrusion3d.hpp,source3d.hpp,render3d.hpp}`,
`native/src/pneumatic3d/{geometry.hpp,carcass.hpp,patches.hpp,brush.hpp,bristles.hpp,state.hpp,model.hpp}`, `native/src/writers/pneumatic_3d_tire.hpp`,
`native/src/initialization/{spatial_equilibrium.hpp,bootstrap_hold.hpp}`, `native/src/rider/{spatial_support.hpp,steer_balance.hpp}`, `native/src/model/benchmark_whipple.hpp`,
`native/src/runtime/{spatial_sensors.hpp,spatial_sensor_pipeline.hpp,spatial_truth.hpp,spatial_checkpoint.hpp,spatial_performance.hpp,spatial_worker_pool.hpp}`,
`native/include/bike/policy_v2.h`, `native/src/replay/spatial_recipe.hpp`, `native/src/golden/spatial_registry.hpp`, `native/src/viz/spatial.hpp`.
**Create (test-support headers):** `native/tests/spatial/support/{terrain_fixture.hpp,tyre_fixture.hpp,startup_fixture.hpp,meijaard_constraints.hpp,meijaard_reference.hpp,benchmark_linalg.hpp,qualification_registry.hpp,spatial_fixture.hpp}`.
**Create (docs):** `migration/spatial3d/contract-deviations.md` (пустая таблица `date | W-task | header | deviation | proposed C10 change`).
**Modify (additive declarations only):** `native/src/recording/catalog.hpp` (+`CatalogVersion`, overloads), `native/src/viewer/camera.hpp` (+`update_spatial`), `native/src/writers/writer_types.hpp` (append-only: `struct SpatialWrenchTelemetry` поля из C4).

**Правила написания:** каждый header — `#pragma once`, полные includes, только declarations/inline-trivial accessors/constexpr константы/POD structs. Никаких реализаций функций с логикой. Private storage классов задан здесь явно: его реализуют соответствующие W. Все namespaces и имена ниже **обязательны**; всё, что не названо (вспомогательные private helpers), на усмотрение W-владельца `.cpp`, но не попадает в header.

### W00.1 `spatial/types.hpp`

```cpp
namespace bike::spatial {
using Vec3 = std::array<double,3>;
using Quat = std::array<double,4>;                 // wxyz, world_from_body
enum class Mode : std::uint8_t { planar, spatial3d };
enum class RootKind : std::uint8_t { planar3, constrained4, free6 };
enum class TyreTier : std::uint8_t { fast, detailed };
struct BodyPose {
    Vec3 position_world_m{}; Quat quat_world_from_body_wxyz{1.,0.,0.,0.};
    Vec3 linear_velocity_world_mps{}; Vec3 angular_velocity_world_rad_s{};
};
inline constexpr std::uint32_t kSpatialContract = 1;
inline constexpr std::uint32_t kSpatialRuntimeSchema = 2;
inline constexpr std::uint32_t kSpatialBootstrapSchema = 2;
inline constexpr std::uint32_t kSpatialWriterSchema = 3;
std::string_view to_string(Mode) noexcept;      std::optional<Mode> parse_mode(std::string_view) noexcept;      // "planar" | "3d"
std::string_view to_string(RootKind) noexcept;  std::optional<RootKind> parse_root_kind(std::string_view) noexcept; // "planar"|"constrained"|"free"
std::string_view to_string(TyreTier) noexcept;  std::optional<TyreTier> parse_tyre_tier(std::string_view) noexcept; // "fast"|"detailed"
}
```

### W00.2 `spatial/config.hpp` (C1, C7.0)

```cpp
namespace bike::spatial {
enum class SteerController : std::uint8_t { off, roll_heading };
struct SteerConfig { double range_deg = 60.; double damping_nms_rad = .5;
    std::optional<double> torque_cap_nm;            // nullopt == "strength" (из профиля)
    double reaction_delay_s = .1; SteerController controller = SteerController::roll_heading;
    std::string profile_id; std::string profile_sha256; };
struct BootstrapConfig { double settle_time_s = 80.; double release_force_limit_n = 1.; double release_moment_limit_nm = 1.;
    double hold_stiffness_n_per_m = 2000.; double hold_damping_ns_per_m = 200.;
    double hold_rot_stiffness_nm_per_rad = 400.; double hold_rot_damping_nms_per_rad = 40.; };
struct ImuMountConfig { std::variant<std::string, Vec3> mount_pos;   // "site_BB" | координаты body-local
    Vec3 mount_rpy_rad{}; Vec3 axis_misalignment_rad{}; std::string calibration_revision; };
enum class TerrainRequestKind : std::uint8_t { none, grid_asset, genspec, profile_extrusion };
struct TerrainRequest3d { TerrainRequestKind kind = TerrainRequestKind::none; std::filesystem::path path;
    double corridor_half_width_m = 1.5; std::uint64_t max_asset_bytes = 1ull << 30; };
struct Pneumatic3dConfig { TyreTier tier = TyreTier::fast; std::string lateral_coefficients_id; std::string lateral_coefficients_sha256; };
struct Config { Mode mode = Mode::planar; RootKind root = RootKind::planar3; double corridor_half_width_m = 1.5;
    double capsize_roll_rad = std::numbers::pi / 3.; SteerConfig steer; BootstrapConfig bootstrap; ImuMountConfig imu;
    TerrainRequest3d terrain; Pneumatic3dConfig tyre; };
struct CapabilityReport { bool implemented = false; bool qualification_pending = true; std::vector<std::string> rejections; };
// TOML/JSON table `[spatial_3d]` + top-level `spatial` string → typed Config. Бросает std::invalid_argument с полным field path.
Config parse_spatial_config(const bike::io::Value& physics_table);
void validate_spatial_config(const Config&, const SimulationPhysicsConfig&);   // кросс-поля: planar+pneumatic_3d, constrained4+controller…
bike::io::Value spatial_config_to_wire(const Config&);                          // exact keys C7.0 `config.spatial`
Config spatial_config_from_wire(const bike::io::Value&);                        // строгий exact-key reader
CapabilityReport check_spatial_capability(const Config&, const SimulationPhysicsConfig&);
std::string spatial_config_fingerprint_hex(const Config&);                      // SHA-256 canonical wire
}
```

### W00.3 `spatial/topology.hpp`, `spatial/frames.hpp` (C2)

```cpp
namespace bike::spatial {
struct ScalarJointBinding { int joint_id = -1; int qpos_adr = -1; int dof_adr = -1; };
struct RootBinding { int body_id = -1; RootKind kind = RootKind::planar3;
    int free_joint_id = -1; int free_qpos_adr = -1; int free_dof_adr = -1;      // free6
    std::array<ScalarJointBinding,4> scalar{};                                   // constrained4: x,y,z,pitch; planar3: x,z,pitch (+[1] unused)
};
struct WheelBinding { int wheel_body_id = -1; int hub_body_id = -1; ScalarJointBinding spin; Vec3 axle_axis_hub{0.,1.,0.}; };
struct SteerBinding { ScalarJointBinding joint; int actuator_id = -1; Vec3 axis_body{}; };
struct Topology { RootBinding root; std::vector<RootBinding> rider_roots; std::array<WheelBinding,2> wheels; // [0]=front,[1]=rear
    SteerBinding steer; int imu_site_id = -1; int nq = 0, nv = 0; std::string model_sha256_hex; std::string fingerprint_hex; };
[[nodiscard]] Topology bind_topology(const mjModel&, const Config&);            // cold; throws std::invalid_argument
[[nodiscard]] std::string topology_fingerprint_hex(const Topology&);
[[nodiscard]] BodyPose body_pose(const mjModel&, const mjData&, const RootBinding&) noexcept;
struct EulerZyx { double roll_rad = 0., pitch_rad = 0., yaw_rad = 0.; bool valid = false; };
Vec3 rotate_body_to_world(const Quat&, const Vec3&) noexcept;  Vec3 rotate_world_to_body(const Quat&, const Vec3&) noexcept;
Quat quat_multiply(const Quat&, const Quat&) noexcept; Quat quat_conjugate(const Quat&) noexcept; Quat quat_normalized(const Quat&) noexcept;
double geodesic_angle_rad(const Quat&, const Quat&) noexcept;   // инвариантен к q/−q
EulerZyx euler_zyx(const Quat&, double singularity_margin_rad = 1e-6) noexcept;
Vec3 point_velocity_world(const BodyPose&, const Vec3& point_world_m) noexcept;
double axis_rate(const Vec3& omega_child_world, const Vec3& omega_parent_world, const Vec3& axis_world) noexcept;
}
```

### W00.4 `spatial/bootstrap_state.hpp`, `brake_state.hpp`, `outcomes.hpp` (C5, C7.0)

```cpp
namespace bike::spatial {
enum class BootstrapPhase : std::uint8_t { settling, release_check, armed, scored };
struct HoldState { std::uint32_t schema = 1; bool active = false; Vec3 reference_position_world_m{}; Quat reference_quat_wxyz{1.,0.,0.,0.};
    Vec3 last_force_world_n{}; Vec3 last_couple_world_nm{}; double stored_energy_j = 0., work_j = 0.; BootstrapPhase phase = BootstrapPhase::settling; std::uint8_t valid_mask = 0; };
struct SteerState { std::uint32_t schema = 1; SteerController controller = SteerController::off; std::int64_t next_tick = 0;
    std::vector<double> delay_fifo_nm; std::size_t fifo_head = 0, fifo_count = 0; std::string profile_sha256;
    double last_command_nm = 0., last_applied_nm = 0.; std::int64_t saturated_ticks = 0; double work_j = 0.; std::uint8_t valid_mask = 0; };
enum class SpatialReason : std::uint8_t { none, capsize, off_corridor, balance_lost, terrain_invalid, terrain_outside, numerical_failure, requested_completion };
struct Outcome { SpatialReason reason = SpatialReason::none; bool terminal = false; bool valid = true; std::int64_t tick = -1; double time_s = 0.; Vec3 position_world_m{}; std::string diagnostic; };
struct OutcomeState { std::uint32_t schema = 1; Outcome latched; std::int64_t capsize_dwell_ticks = 0, corridor_dwell_ticks = 0, balance_dwell_ticks = 0; std::uint8_t valid_mask = 0; };
HoldState canonical_inactive_hold() noexcept; SteerState canonical_inactive_steer(); OutcomeState canonical_unlatched_outcome() noexcept;
bike::io::Value encode_hold(const HoldState&);  HoldState decode_hold(const bike::io::Value&);     // строгие exact-key; throw std::invalid_argument
bike::io::Value encode_steer(const SteerState&); SteerState decode_steer(const bike::io::Value&);
bike::io::Value encode_outcome(const OutcomeState&); OutcomeState decode_outcome(const bike::io::Value&);
std::string_view to_string(BootstrapPhase) noexcept; std::optional<BootstrapPhase> parse_phase(std::string_view) noexcept;
std::string_view to_string(SpatialReason) noexcept; // "crash:capsize", "crash:off_corridor", "balance_lost", "invalid:terrain", "invalid:outside", "invalid:numerical", "completed", "none"

struct BrakeWheelLatch { bool engaged = false; double latch_angle_rad = 0.; double unwrapped_angle_rad = 0.; double stored_energy_j = 0.; double previous_demand = 0.; };
struct BrakeState { std::uint32_t schema = 1; std::array<BrakeWheelLatch,2> wheel{}; std::int64_t interval_index = -1; std::uint8_t valid_mask = 0; };
bike::io::Value encode_brake(const BrakeState&); BrakeState decode_brake(const bike::io::Value&); BrakeState canonical_released_brake() noexcept;

struct SpatialOutcomeInput { std::int64_t tick = 0; double time_s = 0.; BodyPose root; EulerZyx attitude; bool engine_finite = true; bool contact_query_valid = true;
    bool body_ground_contact = false; double capsize_roll_rad = std::numbers::pi/3.; double corridor_half_width_m = 1.5; double bike_half_width_m = .4;
    double lateral_position_m = 0.; bool inside_terrain_domain = true; bool requested_completion = false; double speed_mps = 0.; double balance_speed_floor_mps = .5; std::int64_t balance_dwell_limit_ticks = 200; };
Outcome evaluate_outcome(const SpatialOutcomeInput&, OutcomeState&) noexcept;
}
```

### W00.5 `terrain/grid3d.hpp`, `asset3d.hpp`, `interpolation3d.hpp`, `raycast3d.hpp` (C3, C3.1)

```cpp
namespace bike::terrain {
enum class QueryStatus3d : std::uint8_t { ok, miss, outside, invalid_input, budget_exhausted, origin_below_surface };
struct GridSample3d { QueryStatus3d status = QueryStatus3d::invalid_input; double height_m = 0.; std::array<double,3> normal_world{}; std::uint32_t surface_id = 0; };
struct RayHit3d { QueryStatus3d status = QueryStatus3d::invalid_input; double distance_m = 0.; std::array<double,3> point_world_m{}, normal_world{}; std::uint32_t surface_id = 0; };
struct SurfaceSpec3d { std::uint32_t id = 0; std::string name; double mu_peak = 0., mu_slide = 0., slip_stiffness_per_load = 0., stribeck_speed_mps = 0.; std::string description; };
struct Grid3dDescriptor { std::uint64_t nx = 0, ny = 0; double x0_m = 0., y0_m = 0., dx_m = 0., dy_m = 0.; std::uint32_t dtype = 0; }; // dtype 0=f64, 1=f32
struct Grid3dMetadata { std::string generator_version; bike::io::Value resolved_spec; std::string seed_decimal; std::string units = "m"; double height_datum_m = 0.; bike::io::Value provenance; };
struct RayBudget { std::uint32_t max_cells = 4096; std::uint32_t max_subdivisions = 64; std::uint32_t stack_capacity = 128; double root_interval_m = 1e-6; double residual_m = 1e-8; };
class Grid3d {
public:
    static std::expected<Grid3d,std::string> create(Grid3dDescriptor, std::vector<double> elevation_row_major_iy_nx_plus_ix,
        std::vector<std::uint32_t> material_ids_cells, std::vector<SurfaceSpec3d>, Grid3dMetadata, RayBudget = {});
    std::uint64_t nx() const noexcept; std::uint64_t ny() const noexcept; const Grid3dDescriptor& descriptor() const noexcept;
    std::span<const double> elevation() const noexcept; std::span<const std::uint32_t> materials() const noexcept;
    std::span<const SurfaceSpec3d> surfaces() const noexcept; const Grid3dMetadata& metadata() const noexcept; const RayBudget& budget() const noexcept;
    double x_max_m() const noexcept; double y_max_m() const noexcept;
    GridSample3d sample(double x_m, double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double,3> origin_m, std::array<double,3> unit_direction, double max_distance_m) const noexcept;
    const std::string& sha256_hex() const noexcept;       // от canonical bytes write_terrain3d, вычислен в create
private:
    Grid3dDescriptor desc_; std::vector<double> elevation_; std::vector<std::uint32_t> materials_; std::vector<SurfaceSpec3d> surfaces_;
    std::vector<std::uint32_t> surface_index_by_id_;      // id -> index, -1 как 0xFFFFFFFF
    Grid3dMetadata meta_; RayBudget budget_; std::string sha256_hex_;
};
inline constexpr std::array<char,4> kTerrain3dMagic{'B','K','T','3'}; inline constexpr std::uint32_t kTerrain3dVersion = 1; inline constexpr std::uint32_t kTerrain3dHeaderBytes = 112;
struct Terrain3dLoadLimits { std::uint64_t max_total_bytes = 1ull << 30; std::uint64_t max_nodes = 100'000'000; };
std::expected<Grid3d,std::string> read_terrain3d(std::span<const std::byte>, Terrain3dLoadLimits = {});
std::vector<std::byte> write_terrain3d(const Grid3d&);
// interpolation3d.hpp
struct Catmull { static double value(double p0,double p1,double p2,double p3,double t) noexcept; static double derivative(double p0,double p1,double p2,double p3,double t) noexcept; };
struct BicubicPatch { std::array<double,16> c{}; };                                   // c[j*4+i] * u^i * v^j
BicubicPatch bicubic_patch(const Grid3d&, std::uint64_t ix, std::uint64_t iy) noexcept;  // ghost: p[-1]=2p[0]-p[1], p[n]=2p[n-1]-p[n-2]
struct SurfaceEval { double z = 0., dz_du = 0., dz_dv = 0.; };
SurfaceEval evaluate(const BicubicPatch&, double u, double v) noexcept;
// raycast3d.hpp
struct Poly6 { std::array<double,7> a{}; };                                                // a[k]·s^k, s∈[0,1] внутри cell
Poly6 line_minus_surface(const BicubicPatch&, std::array<double,2> uv0, std::array<double,2> duv, double z0, double dz) noexcept;
struct RootSearchResult { QueryStatus3d status = QueryStatus3d::miss; double s = 0.; std::uint32_t subdivisions = 0; };
RootSearchResult earliest_root_bernstein(const Poly6&, const RayBudget&) noexcept;    // deterministic bisection, left first; tangency via stationary points
RayHit3d ray_cast_grid(const Grid3d&, std::array<double,3> origin_m, std::array<double,3> unit_direction, double max_distance_m) noexcept;
}
```

### W00.6 `terrain/spec3d.hpp`, `generator3d.hpp`, `extrusion3d.hpp`, `source3d.hpp`, `render3d.hpp` (C3.2)

```cpp
namespace bike::terrain {
struct Bank3dSpec { double start_x_m = 0., ramp_m = 0., angle_deg = 0.; };                 // |angle|<=15, quintic ramp
struct Roughness3dSpec { double sigma_m = 0., ell_m = 0.; };
struct Feature3dSpec { std::string kind;  /* "rock"|"rut"|"root" */ double x_m = 0., y_m = 0., amplitude_m = 0., radius_x_m = 0., radius_y_m = 0., yaw_rad = 0.; };
struct MaterialZone3dSpec { std::uint32_t surface_id = 0; double x0_m = 0., x1_m = 0., y0_m = 0., y1_m = 0.; };   // half-open справа/сверху, последняя граница включена
struct Track3dSpec { bike::io::Value base_track_spec; std::string seed_decimal; double dx_m = .005, dy_m = .010, half_width_m = 1.5;
    std::vector<Bank3dSpec> banks; Roughness3dSpec roughness; std::vector<Feature3dSpec> features; std::vector<SurfaceSpec3d> surfaces; std::vector<MaterialZone3dSpec> zones;
    double lead_in_m = 4., lead_out_m = 5.; std::string generator_version = "grid3d-gen/1"; };
Track3dSpec read_track3d_spec(const bike::io::Value& toml_or_json); bike::io::Value write_track3d_spec(const Track3dSpec&);
double quintic_smoothstep(double u) noexcept;                                           // 6u^5-15u^4+10u^3, clamp [0,1]
Grid3d generate_grid3d(const Track3dSpec&);                                              // cold; strips с halo; throws std::invalid_argument
enum class Terrain3dKind : std::uint8_t { grid, profile_extrusion };
class ProfileExtrusion3d {
public:
    static std::expected<ProfileExtrusion3d,std::string> create(std::vector<double> knots_x_m, std::vector<double> knots_z_m,
        std::vector<std::uint32_t> segment_surface_ids, std::vector<SurfaceSpec3d>, double corridor_half_width_m, bike::io::Value source_track_toml);
    GridSample3d sample(double x_m, double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double,3> origin_m, std::array<double,3> unit_direction, double max_distance_m) const noexcept;
    double corridor_half_width_m() const noexcept; std::span<const double> knots_x_m() const noexcept; std::span<const double> knots_z_m() const noexcept;
    const std::string& fingerprint_hex() const noexcept;   // SHA-256(knots+surfaces+corridor+source bytes)
private:
    std::vector<double> kx_, kz_; std::vector<std::uint32_t> seg_surface_; std::vector<SurfaceSpec3d> surfaces_; double half_width_ = 0.; bike::io::Value source_; std::string fingerprint_hex_;
};
struct Terrain3dQueryView { Terrain3dKind kind; const void* context;
    GridSample3d (*sample_fn)(const void*,double,double) noexcept; RayHit3d (*ray_fn)(const void*,std::array<double,3>,std::array<double,3>,double) noexcept;
    GridSample3d sample(double x_m,double y_m) const noexcept; RayHit3d ray_cast(std::array<double,3>,std::array<double,3>,double) const noexcept; };
class Terrain3dSource {
public:
    explicit Terrain3dSource(Grid3d); explicit Terrain3dSource(ProfileExtrusion3d);
    Terrain3dSource(Terrain3dSource&&) = delete; Terrain3dSource& operator=(Terrain3dSource&&) = delete;   // не перемещать после выдачи view
    Terrain3dKind kind() const noexcept; Terrain3dQueryView query_view() const noexcept;
    const Grid3d* grid() const noexcept; const ProfileExtrusion3d* extrusion() const noexcept;    // nullptr для другого kind
    const std::string& fingerprint_hex() const noexcept; double x_min_m() const noexcept; double x_max_m() const noexcept; double half_width_m() const noexcept;
private: std::variant<Grid3d,ProfileExtrusion3d> source_;
};
struct RenderResolution { std::uint32_t nrow = 0, ncol = 0; };
struct RenderGrid3d { std::uint32_t nrow = 0, ncol = 0; std::vector<float> normalized_height; double x0_m = 0., y0_m = 0., x1_m = 0., y1_m = 0., z_min_m = 0., z_max_m = 0.; std::string physical_asset_sha256_hex; RenderResolution resolution; };
RenderGrid3d make_render_grid(const Terrain3dSource&, RenderResolution);              // area downsample; constant height → normalized 0, z_min==z_max
}
```

### W00.7 `pneumatic3d/*.hpp`, `writers/pneumatic_3d_tire.hpp` (C4, C4.1)

```cpp
namespace bike::pneumatic3d {
using ::CoreStatus;   // существующий глобальный enum из writers/writer_types.hpp; детальная причина — ReasonCode3d
enum class ReasonCode3d : std::uint8_t { none, terrain_query_failed, terrain_budget_exhausted, origin_below_surface, nonfinite_geometry, degenerate_down_projection,
    patch_overflow, invalid_tier, invalid_spec, state_identity_mismatch, duplicate_advance, writer_not_bound, engine_contact_enabled };
struct TorusDims { std::uint32_t n_theta_fan = 64, n_phi = 7, n_theta_material = 256; };     // fast; detailed = {256,15,1024}
TorusDims tier_dims(bike::spatial::TyreTier) noexcept;
struct TorusGrid3d { TorusDims dims; double R_m = 0., r_m = 0.;  double theta_min_rad = -75.*std::numbers::pi/180., theta_max_rad = 75.*std::numbers::pi/180., phi_min_rad = -70.*std::numbers::pi/180., phi_max_rad = 70.*std::numbers::pi/180.;
    std::vector<double> theta_fan, phi, w_theta, w_phi;       // quadrature nodes/weights
    std::vector<std::array<double,3>> local_position_fan;     // size n_theta_fan*n_phi, index = it*n_phi+ip, fan frame
    std::vector<double> geometric_area_m2;                     // r*(R+r cosφ)*dθ*dφ на элемент
    std::size_t element_count() const noexcept; std::size_t material_cell_count() const noexcept; };
TorusGrid3d make_torus_grid(TorusDims, double outer_radius_m, double casing_width_m);   // R=outer−r, r=width/2; throws
struct WheelKinematics3d { bike::spatial::BodyPose hub; bike::spatial::Vec3 axle_world{0.,1.,0.}; double relative_spin_rad_s = 0.; double wheel_phase_rad = 0.; };
struct FanFrame { bike::spatial::Vec3 axle, down, forward; bool valid = false; };
FanFrame fan_frame(const WheelKinematics3d&) noexcept;                                    // down = proj(world −Z) на wheel plane; forward = down×axle
struct ContactElement3d { std::uint32_t material_cell = 0; bike::terrain::QueryStatus3d ray_status = bike::terrain::QueryStatus3d::invalid_input;
    bike::spatial::Vec3 point_world_m{}, normal_world{}, longitudinal_world{}, lateral_world{}; double geometric_area_m2 = 0., projected_area_m2 = 0., penetration_m = 0.;
    std::uint32_t surface_id = 0; bool valid = false; ReasonCode3d reason = ReasonCode3d::none; };
CoreStatus query_geometry(const TorusGrid3d&, const WheelKinematics3d&, const bike::terrain::Terrain3dQueryView&, std::span<ContactElement3d>) noexcept;
struct CarcassSpec3d { double pressure_pa = 0.; double k_area_n_per_m3 = 0.; double maxwell_k_n_per_m3 = 0., maxwell_tau_s = 0.; double rim_clearance_m = 0., rim_stiffness_n_per_m3 = 0.; double initial_volume_m3 = 0.; bool constant_pressure = true; std::string provenance_status; /* synthetic|measured|calibrated|inferred|provisional */ };
struct NormalElement3d { std::uint32_t element = 0; double normal_load_n = 0., pressure_force_n = 0., elastic_force_n = 0., rate_force_n = 0., rim_force_n = 0.; double stored_energy_j = 0., dissipated_work_j = 0.; bool active = false; };
CoreStatus update_carcass(const CarcassSpec3d&, std::span<const ContactElement3d>, double dt_s, bool advance, CarcassState3d&, std::span<NormalElement3d>) noexcept;
struct Patch3d { std::uint64_t id = 0; std::span<const std::uint32_t> elements; bike::spatial::Vec3 centroid_world_m{}, normal_world{}, longitudinal_world{}, lateral_world{};
    double normal_load_n = 0., area_m2 = 0.; std::uint32_t surface_id = 0; bike::spatial::Vec3 force_world_n{}, couple_world_nm{}; bool valid = false; };
CoreStatus assign_patches(std::span<const ContactElement3d>, std::span<const NormalElement3d>, bool advance, PatchState3d&, std::span<Patch3d>, std::size_t& count) noexcept;
struct BrushSpec3d { double kx_n_per_m3 = 0., ky_n_per_m3 = 0.; double relaxation_x_m = 0., relaxation_y_m = 0.; double mu_x_scale = 1., mu_y_scale = 1.;
    double camber_coefficient_per_rad = 0.; double pneumatic_trail_m = 0.; double r_eff_m = 0.; std::string provenance_status; };
struct PatchWrench { bike::spatial::Vec3 point_world_m{}, force_world_n{}, couple_world_nm{}; double normal_load_n = 0., fx_n = 0., fy_n = 0., mz_nm = 0.;
    double slip_ratio = 0., slip_angle_rad = 0., camber_rad = 0.; std::uint32_t surface_id = 0; std::uint64_t patch_id = 0; bool valid = false; double loss_j = 0.; };
CoreStatus update_fast_brush(const BrushSpec3d&, std::span<const Patch3d>, std::span<const bike::spatial::Vec3> point_velocity_world_mps, double dt_s, bool advance, BrushState3d&, std::span<PatchWrench>) noexcept;
CoreStatus update_detailed_brush(const BrushSpec3d&, std::span<const ContactElement3d>, std::span<const NormalElement3d>, const WheelKinematics3d&, double dt_s, bool advance, BrushState3d&, std::span<PatchWrench> element_wrench_world) noexcept;
// state.hpp — полностью W00, W21–W27 не меняют
inline constexpr std::uint32_t kPneumatic3dStateVersion = 1;
struct CarcassState3d { std::vector<double> deflection_m, maxwell_q_n_per_m2, previous_penetration_m; std::vector<std::uint8_t> active; double material_phase_rad = 0.; double stored_energy_j = 0., dissipated_j = 0.; };
struct PatchState3d { std::vector<std::uint64_t> ids; std::vector<std::uint32_t> element_to_patch_slot; std::vector<std::uint8_t> slot_used; std::uint64_t next_id = 1; std::uint32_t capacity = 0; };
struct BrushState3d { std::vector<double> ux_m, uy_m; std::vector<double> camber_state; std::vector<double> bristle_ux_m, bristle_uy_m, bristle_energy_j; double elastic_energy_j = 0., sliding_loss_j = 0., convective_release_j = 0.; };
struct TyreStateIdentity3d { std::uint32_t schema = kPneumatic3dStateVersion; bike::spatial::TyreTier tier = {}; TorusDims dims; std::uint32_t patch_capacity = 0;
    std::string topology_fingerprint_hex, config_fingerprint_hex, terrain_sha256_hex, calibration_sha256_hex; };
struct TyreState3d { TyreStateIdentity3d identity; std::array<CarcassState3d,2> carcass; std::array<PatchState3d,2> patches; std::array<BrushState3d,2> brush; std::int64_t accepted_interval = -1; };
std::vector<std::byte> encode_tyre_state(const TyreState3d&);
std::expected<TyreState3d,std::string> decode_tyre_state(std::span<const std::byte>, const TyreStateIdentity3d&);
bike::io::Value tyre_identity_to_wire(const TyreStateIdentity3d&); TyreStateIdentity3d tyre_identity_from_wire(const bike::io::Value&);
std::string base64_canonical(std::span<const std::byte>); std::expected<std::vector<std::byte>,std::string> base64_decode_strict(std::string_view);
// model.hpp — kernels без MuJoCo
struct Pneumatic3dSpec { bike::spatial::TyreTier tier = {}; std::array<double,2> outer_radius_m{}, casing_width_m{}; std::array<CarcassSpec3d,2> carcass; std::array<BrushSpec3d,2> brush; std::uint32_t patch_capacity = 0; /* 0 → доказанный максимум */ };
class Pneumatic3dModel {
public:
    Pneumatic3dModel(Pneumatic3dSpec, bike::terrain::Terrain3dQueryView, TyreStateIdentity3d);   // cold: sizing всех buffers
    CoreStatus evaluate(const std::array<WheelKinematics3d,2>&, std::span<const bike::spatial::Vec3> patch_point_velocity_world_mps, double dt_s, bool advance) noexcept;
    std::span<const PatchWrench> wrenches(std::size_t wheel) const noexcept; std::span<const Patch3d> patches(std::size_t wheel) const noexcept;
    const TyreState3d& committed_state() const noexcept; CoreStatus restore(const TyreState3d&) noexcept; ReasonCode3d last_reason() const noexcept;
    double stored_energy_j() const noexcept; double dissipated_j() const noexcept; const Pneumatic3dSpec& spec() const noexcept;
private:
    Pneumatic3dSpec spec_; bike::terrain::Terrain3dQueryView view_; std::array<TorusGrid3d,2> grid_;
    TyreState3d committed_, staging_; std::array<std::vector<ContactElement3d>,2> elements_; std::array<std::vector<NormalElement3d>,2> normals_;
    std::array<std::vector<Patch3d>,2> patches_; std::array<std::vector<PatchWrench>,2> wrench_; std::array<std::vector<std::uint32_t>,2> patch_element_index_;
    std::array<std::size_t,2> patch_count_{}; ReasonCode3d reason_ = ReasonCode3d::none; std::int64_t last_advanced_interval_ = -1;
};
class Pneumatic3dTireWriter {   // writers/pneumatic_3d_tire.hpp
public:
    Pneumatic3dTireWriter(const mjModel&, const bike::spatial::Topology&, Pneumatic3dSpec, const bike::terrain::Terrain3dSource&, TyreStateIdentity3d);   // cold; throws
    CoreStatus try_compute_into(const mjData*, double dt_s, bool advance, std::span<double> qfrc) noexcept;
    const TyreState3d& state() const noexcept; std::vector<std::byte> state_bytes() const; CoreStatus restore(const TyreState3d&) noexcept;
    std::span<const PatchWrench> wrenches(std::size_t wheel) const noexcept; double axle_moment_nm(std::size_t wheel) const noexcept; ReasonCode3d last_reason() const noexcept;
    const Pneumatic3dModel& model() const noexcept;
private:
    const mjModel* model_ = nullptr; bike::spatial::Topology topology_; Pneumatic3dModel kernels_;
    std::vector<double> jacp_, jacr_;  /* 3*nv каждый, scratch */ std::vector<bike::spatial::Vec3> point_velocity_; std::array<double,2> axle_moment_{};
};
}
```

`CoreStatus` остаётся глобальным enum (`writers/writer_types.hpp`); `CarcassState3d/PatchState3d/BrushState3d` объявлены в `state.hpp` и включаются в `carcass.hpp/patches.hpp/brush.hpp` до использования.

### W00.8 `initialization/*.hpp`, `rider/*.hpp`, `model/benchmark_whipple.hpp` (C5)

```cpp
namespace bike::initialization {
struct SpatialEquilibriumReport { double max_linear_accel_mps2 = 0., max_angular_accel_radps2 = 0.; double weight_residual_fraction = 0.; double pitch_moment_residual_nm = 0., roll_moment_residual_nm = 0.;
    int contacts_valid = 0, contacts_total = 0; int evaluations = 0; std::vector<std::string> errors; std::vector<int> worst_dofs; };
// settle_context(SetupContext&, const EquilibriumOptions&) существующая; W29 добавляет в EquilibriumResult поле std::optional<SpatialEquilibriumReport> spatial;
double spatial_refine(SetupContext&, int max_evaluations, double acceleration_tolerance);   // возвращает достигнутый max accel
void spatial_initial_pose(SetupContext&, const bike::spatial::Topology&, const bike::terrain::Terrain3dQueryView&, double heading_rad, double initial_speed_mps);
class BootstrapHold {
public:
    BootstrapHold(const bike::spatial::BootstrapConfig&, const bike::spatial::RootBinding&, const mjModel&);   // cold
    CoreStatus try_compute_into(const mjModel&, const mjData&, bike::spatial::BootstrapPhase, std::span<double> qfrc) noexcept;   // phase>=armed → точный ноль, без изменений state
    const bike::spatial::HoldState& state() const noexcept; CoreStatus restore(const bike::spatial::HoldState&) noexcept;
    bool release_check(const SpatialEquilibriumReport&, double force_limit_n, double moment_limit_nm) const noexcept;
private: bike::spatial::BootstrapConfig cfg_; bike::spatial::RootBinding root_; bike::spatial::HoldState state_; std::vector<double> jacp_, jacr_;
};
}
namespace bike::rider {
struct ShoulderProfile { std::uint32_t schema = 1; double swivel_range_rad = 0., swivel_stiffness_nm_per_rad = 0., swivel_damping_nms_per_rad = 0., swivel_strength_nm = 0.; std::string provenance_status = "provisional"; std::string sha256_hex; };
ShoulderProfile read_shoulder_profile(const bike::io::Value&);
struct SpatialSupportFrame { bike::spatial::Vec3 normal_world, tangent_x_world, tangent_y_world; bool valid = false; };
SpatialSupportFrame support_frame(const bike::spatial::BodyPose& body, const bike::spatial::Vec3& normal_body) noexcept;
struct GripRigResult { double left_gap_m = 0., right_gap_m = 0.; int grip_jacobian_rank = 0; double steering_mobility = 0.; double internal_power_residual_w = 0.; };
struct SteerInput { std::int64_t control_tick = 0; double roll_rad = 0., roll_rate_rad_s = 0., heading_error_rad = 0., lateral_error_m = 0., steer_angle_rad = 0., steer_rate_rad_s = 0., speed_signed_mps = 0.; bool enabled = true; };
struct SteerProfile { std::uint32_t schema = 1; std::vector<double> speed_nodes_mps; std::vector<std::array<double,6>> gains; /* K per node: roll, roll_rate, heading, lateral, steer, steer_rate */ double reference_torque_nm = 0.; double cap_nm = 40.; double delay_s = .1; std::string sha256_hex; std::string provenance_status = "provisional"; };
SteerProfile read_steer_profile(const bike::io::Value&); bike::io::Value write_steer_profile(const SteerProfile&);
class SteerBalance {
public:
    SteerBalance(SteerProfile, double control_period_s);   // cold: FIFO = round(delay/period), запись фактической задержки
    double tick(const SteerInput&) noexcept; const bike::spatial::SteerState& state() const noexcept; CoreStatus restore(const bike::spatial::SteerState&) noexcept; void reset() noexcept;
    double resolved_delay_s() const noexcept; std::size_t delay_ticks() const noexcept;
private: SteerProfile profile_; double period_s_; bike::spatial::SteerState state_;
};
}
namespace bike::model {
struct WhippleParameters { double w_m = 1.02, c_m = .08, lambda_rad = std::numbers::pi/10., g_mps2 = 9.81, v_mps = 0.;
    double rR_m = .3, mR_kg = 2., IRxx = .0603, IRyy = .12;   double xB_m = .3, zB_m = -.9, mB_kg = 85., IBxx = 9.2, IBxz = 2.4, IByy = 11., IBzz = 2.8;
    double xH_m = .9, zH_m = -.7, mH_kg = 4., IHxx = .05892, IHxz = -.00756, IHyy = .06, IHzz = .00708;   double rF_m = .35, mF_kg = 3., IFxx = .1405, IFyy = .28; };   // Meijaard 2007 Table 1, z-down в статье
BuildRequest make_whipple_request(const WhippleParameters&);   // z-up конверсия внутри
}
```

### W00.9 `runtime/spatial_*.hpp`, `include/bike/policy_v2.h` (C6, C7)

`runtime/spatial_sensor_pipeline.hpp` воспроизводит дословно блок V2 из `05-observation-recording.md` §«Контракты V2» (`ImuTruth`, `rotate_vector`, `SensorObservationV2`, `NoiseTapeV2`, `SensorConfigV2`, `SensorStateV2`, `SensorPipelineV2`). Дополнительно:

```cpp
namespace runtime::spatial {
struct ImuMountBinding { int site_id = -1; int body_id = -1; int accel_sensor_id = -1, gyro_sensor_id = -1; int accel_adr = -1, gyro_adr = -1; Matrix3 sensor_from_nominal{1,0,0,0,1,0,0,0,1}; std::string calibration_revision; };
ImuMountBinding bind_imu_mount(const mjModel&, const bike::spatial::Config&, const bike::spatial::Topology&);   // cold; throws
ImuTruth read_imu_truth(const mjModel&, const mjData&, const ImuMountBinding&, double time_s) noexcept;         // после constraint solve, до endpoint forward
bool is_rotation_matrix(const Matrix3&, double tol = 1e-9) noexcept;
double integrate_abs_slip(double accumulated_m, double slip_mps, double dt_s);   // throws std::invalid_argument
struct SpatialTruth { bike::spatial::BodyPose root; bike::spatial::EulerZyx attitude; double lateral_displacement_m = 0.; double steer_angle_rad = 0., steer_rate_rad_s = 0., steer_torque_nm = 0., steer_work_j = 0.;
    std::array<bool,2> contact_valid{}; std::array<double,2> lateral_slip_mps{}, normal_load_n{}, slip_path_m{}; std::array<Vector3,2> support_normal_world{}; bool front_unloaded = false, front_lift = false, flight = false; };
bike::io::Value spatial_truth_to_wire(const SpatialTruth&);
inline constexpr std::uint32_t kSpatialCheckpointSchema = 1;
struct SpatialCheckpoint { std::uint32_t schema = kSpatialCheckpointSchema; bike::io::Value bootstrap2; bike::io::Value research; bike::io::Value sensor_v2; bike::io::Value policy; bike::io::Value identity; };
bike::io::Value encode_checkpoint(const SpatialCheckpoint&); SpatialCheckpoint decode_checkpoint(const bike::io::Value&);   // strict exact keys
struct CheckpointStage;   // opaque, определён в .cpp: validated-but-uncommitted owners
std::unique_ptr<CheckpointStage> stage_checkpoint(const SpatialCheckpoint&, const struct RuntimeIdentity&);  // throws до любого изменения
void commit_checkpoint(CheckpointStage&&, class NativeRideRuntime&);                                           // noexcept-семантика после staging; документировать
bool rtf_threshold_met(double simulated_s, double wall_s, double target);     // throws std::invalid_argument
struct SpatialPerformanceProfile { std::string id = "spatial-fast-v1"; bike::spatial::RootKind root; bike::spatial::TyreTier tier; std::uint64_t grid_nx = 0, grid_ny = 0; double dt_s = 0.;
    std::string sensor_mode, recording_mode; unsigned processes = 1, tyre_threads = 1, engine_threads = 1; std::string source_sha, config_sha256, calibration_sha256, provider_identity; };
struct RegionTimings { double terrain_rays_s = 0., tyre_elements_s = 0., engine_step_s = 0., rider_steer_s = 0., accounting_s = 0., sensors_s = 0., recording_s = 0., ui_s = 0.; bool sensors_pending = true, recording_pending = true, ui_pending = true; };
struct IndexRange { std::size_t begin = 0, end = 0; };
std::vector<IndexRange> partition_ranges(std::size_t count, unsigned workers);   // throws при workers==0
using ElementWork = void (*)(void* context, std::size_t begin, std::size_t end) noexcept;
class WorkerPool { public: explicit WorkerPool(unsigned threads); ~WorkerPool(); WorkerPool(const WorkerPool&) = delete; WorkerPool& operator=(const WorkerPool&) = delete;
    unsigned worker_count() const noexcept; void run(std::span<const IndexRange>, ElementWork, void* context); void stop();
private: struct Impl; std::unique_ptr<Impl> impl_; };
}
```

`policy_v2.h` (C ABI, `extern "C"`): `BIKE_POLICY_V2_ABI_VERSION 2U`, `BIKE_SENSOR_V2_SCHEMA_VERSION 2U`, `BIKE_SENSOR_V2_VALUE_COUNT 11U`; `struct BikeObservationV2 { uint32_t struct_size; uint32_t sensor_schema_version; uint32_t valid; uint32_t valid_mask; double time_s, source_time_s; double values[11]; }`; `struct BikePolicyV2 { uint32_t struct_size, abi_version, capabilities; void* (*create)(const char* config_json, BikeErrorV1*); int (*reset)(void*, const BikeObservationV2*, BikeErrorV1*); int (*act)(void*, const BikeObservationV2*, BikeDemandV1*, BikeErrorV1*); void (*destroy)(void*); }`; экспорт `const BikePolicyV2* bike_policy_v2(void);`. `policy_v1.h` включается явно и не меняется.

### W00.10 `recording/catalog.hpp` (+), `replay/spatial_recipe.hpp`, `golden/spatial_registry.hpp`, `viz/spatial.hpp`, `viewer/camera.hpp` (+)

```cpp
namespace bike::recording { enum class CatalogVersion : std::uint32_t { planar1 = 1, spatial2 = 2 };
  Catalog build_catalog(const mjModel&, CatalogVersion); const Catalog& schema_catalog(CatalogVersion); std::span<const TableDescriptor> all_tables(CatalogVersion); }   // старые overloads == planar1
namespace bike::replay { inline constexpr std::uint32_t kSpatialReplayContract = 4;
  struct SpatialRecipe4 { std::uint32_t contract = kSpatialReplayContract; bike::terrain::Terrain3dKind terrain_kind; std::string terrain_sha256_hex; bike::io::Value config_spatial; bike::io::Value checkpoint_initial, checkpoint_final; std::string execution_identity_sha256; };
  SpatialRecipe4 read_spatial_recipe(const bike::io::Value&); bike::io::Value write_spatial_recipe(const SpatialRecipe4&); }
namespace bike::golden { inline constexpr std::uint32_t kSpatialManifestVersion = 1; void validate_spatial_manifest(const bike::io::Value&); }   // throws std::invalid_argument
namespace bike::viz { Report spatial_attitude_plot(const Channels&); Report spatial_contact_plot(const Channels&); Report spatial_wheel_path_plot(const Channels&); Report spatial_imu_plot(const Channels&); Report spatial_energy_plot(const Channels&); }
// viewer/camera.hpp (+): void update_spatial(mjvCamera&, const std::array<double,3>& position_world_m, std::optional<double> heading_rad);
```

### W00.11 Test-support headers (`native/tests/spatial/support/`)

```cpp
namespace spatial_tests {
// spatial_fixture.hpp — bounded reader под BIKE_SPATIAL_FIXTURE_ROOT (генерируется W04)
std::filesystem::path fixture(std::string_view relative);                  // containment/traversal checks; throws std::invalid_argument
std::string text(const std::filesystem::path&); std::vector<std::byte> bytes(const std::filesystem::path&); bike::io::Value json(const std::filesystem::path&);
std::string sha256_hex(std::span<const std::byte>);
// terrain_fixture.hpp
bike::terrain::Grid3d plane_grid(double a, double b, double c, std::uint64_t nx, std::uint64_t ny, double dx, double dy);   // z = a x + b y + c
bike::terrain::Grid3d polynomial_grid(std::span<const double> coeff_xy, std::uint64_t nx, std::uint64_t ny, double dx, double dy);
bike::terrain::Grid3d asymmetric_4x4();   // разные x/y значения для transpose detection
bike::terrain::Grid3d bump_grid(double height_m, double sigma_m, std::uint64_t nx, std::uint64_t ny, double dx, double dy);
std::vector<double> polynomial_ray_roots_highprecision(const bike::terrain::Poly6&);   // независимый oracle (long double Durand–Kerner)
// tyre_fixture.hpp
bike::pneumatic3d::Pneumatic3dSpec synthetic_spec(bike::spatial::TyreTier);           // provenance_status="synthetic"
bike::pneumatic3d::WheelKinematics3d wheel_at(const bike::spatial::Vec3& hub, double yaw_rad, double camber_rad, double speed_mps, double spin_rad_s);
double full_torus_area(const bike::pneumatic3d::TorusGrid3d&); double full_torus_volume(const bike::pneumatic3d::TorusGrid3d&);
double potential_energy_fd(const bike::pneumatic3d::Pneumatic3dModel&, const std::array<bike::pneumatic3d::WheelKinematics3d,2>&, const bike::spatial::Vec3& displacement, double h);
// startup_fixture.hpp
struct RigModel { mjModel* model = nullptr; mjData* data = nullptr; bike::spatial::Topology topology; ~RigModel(); };
RigModel constrained_rig(const bike::spatial::Config&); RigModel free_rig(const bike::spatial::Config&, bool articulated_rider); RigModel mixed_joint_rig();
std::vector<std::byte> complete_state_bytes(const RigModel&);
// meijaard_constraints.hpp / meijaard_reference.hpp / benchmark_linalg.hpp
struct RollingConstraints { std::vector<double> A; std::vector<double> Adot_v; int rows = 6; };
RollingConstraints knife_edge_constraints(const mjModel&, const mjData&, const bike::spatial::Topology&);
struct WhippleMatrices { std::array<double,4> M, C1, K0, K2; };   WhippleMatrices reference_matrices(const bike::model::WhippleParameters&);   // Appendix A
std::vector<std::complex<double>> quadratic_eigenvalues(const WhippleMatrices&, double v, double g);   // det(λ²M+λvC1+gK0+v²K2)
struct Kkt { std::vector<double> vdot, lambda; double relative_residual = 0.; int rank = 0; };
Kkt solve_kkt(std::span<const double> M, std::span<const double> bias, const RollingConstraints&, int nv);
// qualification_registry.hpp
void validate_qualification(const bike::io::Value&);   // root qualification_schema=1, milestones M0–M6, upstream_gates G0–G6
}
```

- [ ] Написать все перечисленные headers и test-support headers согласно блокам W00.1–W00.11, с полными includes (`<array>`, `<cstdint>`, `<expected>`, `<filesystem>`, `<optional>`, `<span>`, `<string>`, `<variant>`, `<vector>`, `<numbers>`, `<complex>`, `<memory>`, `mujoco/mujoco.h` где нужен `mjModel`), forward declarations существующих типов (`SimulationPhysicsConfig`, `SetupContext`, `NativeRideRuntime`, `Catalog`, `Channels`, `Report`) через их реальные headers.
- [ ] Создать `migration/spatial3d/contract-deviations.md`.
- [ ] Коммит `feat(spatial): W00 freeze spatial public headers` с трейлером `Verification: pending V00`.

**Deferred to V00:** компиляция каждого header в пустой TU; разбор `contract-deviations.md`.

---

## W01 — Реестр upstream gates и planar-baseline manifest (← S01)

**Dependencies:** S00, W00. **Create:** `migration/spatial3d/upstream-gates.json`, `tests/fixtures/spatial3d/planar-baseline/manifest.json`, `native/tests/spatial/test_baseline.cpp` (регистрация — W04).

- [ ] `upstream-gates.json`: массив `gates[]` для `G0-source … G6-qualification` с полями `id, required_contract, pfc_tasks[], consumers[] (V-задачи), status:"unverified_in_this_planning_run", evidence:[], owner:null, failing_case:null, next_step:null`.
- [ ] `planar-baseline/manifest.json`: `manifest_version=1`, `base_sha:null`, `platforms:{"macos-arm64-accelerate":{...},"linux-aarch64-openblas":{...}}`, у каждой `traces[]` для `flat`, `rough_uphill_savage.toml`, `alpine_rocky.toml` с полями `config_canonical_sha256, model_sha256, state_t0_path, committed_steps_path, reset_path, probe_path, payload_sha256, provider, compiler, command, exit_code` — все `null`, `status:"pending_verification"`.
- [ ] `test_baseline.cpp`: читает manifest через `spatial_tests::json`; для каждой platform/trace требует non-null payload_sha256 и совпадение provider между reference/candidate; загружает два state-массива (`read_f64`) и сравнивает `std::bit_cast<std::uint64_t>` поэлементно (код из прежней S01 сохраняется дословно); `bt::require(reference_state.size()==candidate_state.size(), ...)`. Тест обязан падать на `null`.
- [ ] Коммит `test(spatial): W01 upstream gates registry and planar baseline manifest`, `Verification: pending V01`.

**Deferred to V01:** фактический захват planar traces на базе S00 обеими платформами; заполнение manifest; статусы gates по producer evidence.

## W02 — Contracts registry, ADR и ownership (← S02)

**Dependencies:** W00. **Create:** `migration/spatial3d/contracts.json`, `docs/adr/0004-spatial-3d-native-correctness.md` (status `proposed`). **Modify:** `migration/interface-contracts.md` — ссылка на пространственное расширение.

- [ ] `contracts.json`: `schema_version=1`, `contracts[]` по C1–C9: `id, header_path, namespace, types[], functions[], producer (W-task), consumers[] (W-tasks), versions {spatial_contract:1, runtime_schema:2, bootstrap_schema:2, writer_schema:3, terrain:"BKT3/1", tyre_state:1, checkpoint:1, sensor:2, policy_abi:2, catalog:2, container:2, replay:4}, frames[], units[], owner`. Один owner на contract; schema names distinct — проверяется W04 тестом.
- [ ] ADR-0004: контекст (нет принятого spatial ADR; 0002/0003 заняты), решение: native-only spatial; новая физика проверяется unit/invariant/independent-reference/convergence gates; planar comparisons по ADR-0003; без Python twin/xfail stubs; W/V-разделение исполнения. Статус `proposed`, без вымышленного approval.
- [ ] Коммит `docs(spatial): W02 contracts registry and ADR-0004 proposal`, `Verification: pending V01`.

## W03 — Реестр физических эталонов (← S03)

**Dependencies:** W00. **Create:** `migration/spatial3d/references.json`, `tests/fixtures/spatial3d/references/meijaard2007.json`, `tests/fixtures/spatial3d/references/tyre-lateral.json`, `docs/reference/spatial3d-provenance.md`, `native/tests/spatial/test_references.cpp`.

- [ ] `meijaard2007.json`: DOI `10.1098/rspa.2007.1857`, Table 1 параметры (значения из `WhippleParameters` W00.8 с единицами и исходной z-down конвенцией), Table 2 eigenvalues: v=2 weave `2.68234517512745 ± 1.68066296590675i`; v=6 weave `−1.52644486584142 ± 5.87673060598709i`, capsize `−0.00406690076970`; boundaries weave `4.29238253634111`, capsize `6.02426201538837` m/s; `source_location` для каждого числа.
- [ ] `tyre-lateral.json`: строки измерений с `tyre, dimensions, pressure_pa, normal_load_n, speed_mps, surface, slip_angle_deg, slip_angle_rad, camber_deg, cornering_stiffness_n_per_rad, fit_convention, uncertainty, units_original, source_doi, source_location, applicability` (TU Delft/Zenodo city/cargo как cross-check: `applicability:"cross_check_only"`); `payload_sha256:null` для внешних датасетов.
- [ ] `references.json`: индекс обоих файлов с лицензиями и `status: "data_entered_pending_review"`.
- [ ] `test_references.cpp`: для каждой строки `std::isfinite(normal_load_n) && >0`, `|radians − degrees·π/180| < 1e-14`, `cornering_stiffness_n_per_rad > 0`, non-empty `source_doi`/`source_location`; для Meijaard — что `WhippleParameters{}` совпадает с JSON по каждому полю (1e-12).
- [ ] Коммит `docs(spatial): W03 physical reference registry`, `Verification: pending V01`.

## W04 — C++-тестовый контур (← S04)

**Dependencies:** W00. **Create:** `native/tests/spatial/CMakeLists.txt`, `native/tests/spatial/fixture_paths.hpp.in`, `native/tests/spatial/support/spatial_fixture.cpp`, `native/tests/spatial/test_harness.cpp`, `native/tests/spatial/test_contracts_registry.cpp`. **Modify:** root `CMakeLists.txt` (`add_subdirectory(native/tests/spatial)` под `BUILD_TESTING`).

- [ ] `CMakeLists.txt`: `function(bike_add_spatial_test slug)` — `cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES;LABELS" ${ARGN})`; пустой `SOURCES` → `message(FATAL_ERROR)`; `add_executable(bike_spatial_${slug} ${ARG_SOURCES})`; `target_link_libraries(... PRIVATE bike_spatial_support bike_migration_test bike_io ${ARG_LIBRARIES})`; `bike_apply_native_policy(bike_spatial_${slug})`; `add_test(NAME spatial.${slug} COMMAND bike_spatial_${slug})`; `set_tests_properties(... LABELS "spatial;${ARG_LABELS}")`. `add_library(bike_spatial_support STATIC support/spatial_fixture.cpp)` + `configure_file(fixture_paths.hpp.in ...)` с `BIKE_SPATIAL_FIXTURE_ROOT="${CMAKE_SOURCE_DIR}/tests/fixtures/spatial3d"`. Регистрации: `bike_add_spatial_test(harness SOURCES test_harness.cpp)`, `baseline`, `references`, `contracts_registry`.
- [ ] `spatial_fixture.cpp`: реализация W00.11 `fixture/text/bytes/json/sha256_hex` с запретом пустых/абсолютных/`..` путей и post-canonical containment.
- [ ] `test_harness.cpp`: положительный control (`bit_cast(0.) != bit_cast(-0.)`, `bt::invalid` на `std::invalid_argument`), негативный child-process control: запуск собственного executable с аргументом `--zero-checks`, который выполняет `bt::run([]{})` и обязан вернуть ненулевой exit (используется `std::system` с argv через существующий native process helper, если он есть; иначе `std::system`).
- [ ] `test_contracts_registry.cpp`: читает `contracts.json`, требует один owner на contract, уникальность `header_path`, distinct schema names/версии по таблице C7, non-empty consumers; читает `baseline.json` и проверяет форму `base_sha` (40 lowercase hex) и disposition каждой WIP-правки.
- [ ] Коммит `test(spatial): W04 spatial CTest harness and registry checks`, `Verification: pending V01`.

## W05 — Native spatial config и capability (← S05)

**Dependencies:** W00. **Create:** `native/src/spatial/config.cpp`, `native/src/spatial/types.cpp`, `native/tests/spatial/test_config.cpp`, fixtures `tests/fixtures/spatial3d/config/{valid_free.toml,valid_constrained.toml,planar_explicit.toml,neg_spatial_table_without_mode.toml,neg_3d_planar_tyre.toml,neg_planar_pneumatic3d.toml,neg_constrained_controller.toml,neg_nan_grid.toml,neg_unknown_key.toml,neg_unsupported_schema.toml,neg_rider_drive_combo.toml}`. **Modify:** `native/src/configuration/{model_config.hpp,load.cpp,schema.cpp,validate.cpp}` (поле `std::optional<bike::spatial::Config> spatial` в `SimulationPhysicsConfig`; strict parse `[spatial_3d]`), `native/src/runtime/{capabilities.hpp,capabilities.cpp,project_config.cpp}` (row `spatial`, dispatch runtime1 vs runtime2), `native/src/writer_config.cpp` (writer schema3 ветка: обязательные `schema,spatial`, exact `spatial` keys из C7.0), root `CMakeLists.txt` (добавить `native/src/spatial/{types,config}.cpp` в `bike_configuration` или новую `bike_spatial` STATIC с link к `bike_io`).

- [ ] `types.cpp`: `to_string/parse_*` для трёх enum по строкам W00.1.
- [ ] `config.cpp`: `parse_spatial_config` — exact-key проверка `spatial_3d.{root,corridor_half_width_m,capsize_roll_rad,steer.*,bootstrap.*,imu.*,terrain.*,tires.pneumatic_3d.*,lateral_coefficients}`; ошибки `std::invalid_argument("spatial_3d.steer.range_deg: must be in (0,90)")` с полным путём; defaults из C1. `validate_spatial_config`: planar+`pneumatic_3d` → reject; 3D + planar tyre backend → reject; constrained4 + controller≠off → reject; NaN/Inf/≤0 grid steps → reject; rider/drive комбинации по существующей таблице capabilities. `spatial_config_to_wire/from_wire`: ровно `spatial_contract,mode,root,terrain,steer,bootstrap,imu,profile`. `check_spatial_capability`: `implemented=false`, `qualification_pending=true`, `rejections=["spatial runtime not connected: <row>"]` для любого 3D-запроса — реальные rows открывают W25/W31/W36 правками `capabilities.cpp`, которые перечислены в тех задачах.
- [ ] `project_config.cpp`: отсутствие `spatial` → прежние байты и `runtime_schema=1`; присутствие → `runtime_schema=2`, обязательный root key `spatial`, writer `schema=3`. `bootstrap2` формирует W17.
- [ ] `test_config.cpp`: каждый negative fixture → `bt::invalid` с проверкой подстроки field path в `what()`; два valid → `!report.implemented && report.qualification_pending && !report.rejections.empty()`; `planar_explicit.toml` и отсутствие `spatial` дают побайтно равный canonical wire (`legacy_wire_before == legacy_wire_after`); roundtrip `from_wire(to_wire(cfg))` равен `cfg` по всем полям; каждый существующий `examples/**/*.toml` без `spatial` парсится с прежней схемой.
- [ ] Коммит `feat(spatial): W05 native spatial config and capability gate`, `Verification: pending V01`.

## Передача

W06–W55 читают W00 как единственный источник declarations; S-номера сохранены в `task-index.json` как `source_sections`. Все checkbox статусы остаются открытыми до фактического выполнения; `source_written` фиксируется коммитом, `verified` — только в V.
