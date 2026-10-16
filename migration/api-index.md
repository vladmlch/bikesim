# Produced API and executable contracts

Each section is owned by the named task. Reused native types are defined by the source authority in interface-contracts.md; new records use that document and the source dataclass field inventory. There are no independent duplicate implementations in consumers.

## T00

Freeze the actual checkout and a complete removal ledger

```text
The machine-readable ledger has one row per tracked first-party Python file:
{path, sha256, line_count, owner_task, action, replacement_paths,
 verification_ids, deletion_gate, status}. Status begins "planned".
The test map uses collected pytest node IDs, not just filenames.
```

## T01

Capture reference behavior before deleting any oracle

```text
Temporary capture entrypoint (deleted by T63):
uv run --frozen --group native python tools/migration/capture_reference.py   --output tests/fixtures/migration --manifest migration/oracle-manifest.json
Each fixture records input hashes, baseline commit, provider/tool versions,
comparison class, expected outcome and SHA-256 for every binary payload.
```

## T02

Introduce a standalone contract-test harness and binary fixtures

```cpp
namespace bt {
int run(const std::function<void()>& test);
void require(bool condition, std::string_view message);
std::filesystem::path fixture(std::string_view relative);
std::string text(const std::filesystem::path& path);
std::vector<double> read_f64(const std::filesystem::path& path);
void golden_text(std::string_view relative, std::string_view actual);
void golden_f64(std::string_view relative, std::span<const double> actual);
template<class F> void invalid(F&& action); // requires std::invalid_argument
}
// golden_f64 compares uint64_t object representations; signed zero is significant.
// The root is supplied by a CMake compile definition, never the current directory.
```

## T03

Build the application without finding Python or wheel directories

```bash
cmake -S . -B build/native-release -DCMAKE_BUILD_TYPE=Release   -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=OFF   -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake --build build/native-release
ctest --test-dir build/native-release --output-on-failure
# During migration only, ON adds adapters and oracle tests; OFF is the default.
```

## T04

Preserve numerical providers and fatal-engine containment

```cpp
namespace engine {
struct Identity { std::string mujoco_version, library_sha256, numeric_provider; };
Identity identity();
void verify_identity(const Identity& required);
}
// Existing ErrorBuffer, EngineFailure and try_* contracts remain authoritative.
// Add native callback provenance; do not add any Python exception dependency.
```

## T05

Create strict lossless data and JSON boundaries

```cpp
namespace bike::io {
struct Value; // owned recursive tagged value; exact number lexemes, bytes and containers
Value parse_json(std::string_view text);
std::string canonical_json(const Value& value);
std::string sha256(std::span<const std::byte> bytes);
}
// Runtime WireValue/WireObject remain the existing hot-path types.
// Value is a cold-path input/output AST, not a replacement for typed physics.
```

## T06

Introduce filesystem, subprocess and provenance primitives

```cpp
namespace bike::platform {
struct ProcessResult { int exit_code; std::string out, err; };
ProcessResult run(const std::vector<std::string>& argv, const std::filesystem::path& cwd);
void write_exclusive(const std::filesystem::path& path, std::span<const std::byte> data);
std::filesystem::path executable_path();
}
// No shell interpolation; stdout and stderr are drained concurrently.
```

## T07

Separate typed writer construction from nanobind parsing

```cpp
namespace nativecfg {
struct WriterConfig; // optional typed sections for the current nine writer families
WriterConfig decode(const bike::io::Value& value);
}
// Add, without changing the owned Stepper lifetime:
// Stepper(const std::string& mjb_path, const nativecfg::WriterConfig& config);
// Adapter-only nanobind constructors delegate to this constructor.
```

## T08

Remove binding types from rider and drivetrain state wires

```cpp
namespace rider {
SpindleConfig decode_spindle_config(const bike::io::Value& value);
}
namespace drivetrain {
DriveSnapshot decode_drive_snapshot(const bike::io::Value& value);
bike::io::Value encode_drive_snapshot(const DriveSnapshot& state);
}
// SpindleConfig already lives in rider/spindle_controller.hpp.
// DriveSnapshot already lives in writers/drivetrain.hpp; reuse it.
// Binding-only decoder declarations must not enter the pure core include graph.
```

## T09

Extract owned ride runtime and bootstrap into a pure C++ API

```cpp
namespace runtime {
struct ModelImage { std::vector<std::byte> mjb; std::string sha256; };
struct RuntimeBootstrap {
  ModelImage model;
  RuntimeConfig config;
  BootstrapState state;
};
RuntimeBootstrap read_bootstrap(const std::filesystem::path& directory);
// Replace binding-shaped constructor/method inputs in NativeRideRuntime:
// explicit NativeRideRuntime(RuntimeBootstrap bootstrap);
// AdvanceResult advance(int64_t target_step, const RideControl& control,
//   double front, double rear, std::optional<double> wall_budget_s);
}
// Existing RuntimeSnapshot, SampleBatch and owned episode types are retained.
```

## T10

Make the research runtime consume only native owned records

```cpp
namespace runtime {
struct ResearchBootstrap {
  RuntimeBootstrap ride;
  ResearchConfig config;
  NoiseTape tape;
  SensorState startup;
};
ResearchBootstrap read_research_bootstrap(const std::filesystem::path& directory);
std::unique_ptr<NativeResearchRuntime> make_research(ResearchBootstrap bootstrap);
}
// Existing begin_control/advance_control/acknowledge_control semantics are retained.
```

## T11

Separate native samples and column pieces from their Python boxers

```cpp
namespace runtime {
struct Piece { /* move-only; member_slots point into the piece's own
  children map — pieces are moved or emplaced, never copied */ };
struct EncodePlanColumn { /* recursive leaf/member schema */ };
struct EncodePlanMask { /* presence/validity mask columns */ };
struct EncodePlan { /* fixed-table metadata + channel routing indexes */ };
struct DerivedEventPlan { /* tracked (kind, path) fields + kinds */ };
EncodePlan parse_encode_plan(const bike::io::Value& plan);
DerivedEventPlan parse_derived_event_plan(const bike::io::Value& plan);
struct EncodedTable { std::int64_t rows; std::map<std::string, Piece> columns; };
EncodedTable encode_table_native(const OwnedNativeEpisode& episode,
  const EncodePlan& plan, std::int64_t start, std::int64_t limit);
struct AppliedCommandRow { std::int64_t application_id; double time_s;
  std::int64_t step; Wire control;
  std::optional<Wire> front_brake_demand, rear_brake_demand,
    rollback_brake_demand; };  // absent keys stay absent, not null
struct DerivedEventRow { std::int64_t interval_id;
  std::string derived_kind; WireObject changes; };
std::vector<AppliedCommandRow> applied_command_rows_native(
  const OwnedNativeEpisode& episode, std::int64_t start, std::int64_t limit);
std::vector<DerivedEventRow> derived_event_rows_native(
  const OwnedNativeEpisode& episode, const DerivedEventPlan& plan,
  std::int64_t start, std::int64_t limit);
struct RuntimeIdentity { std::string mujoco_version, mujoco_library_path,
  native_source_sha256, build_context_json; };
RuntimeIdentity runtime_identity();   // mj_versionString + dladdr +
}                                     // compiled Track-B identity
// Headers: runtime/encode.hpp + runtime/provenance.hpp (pure C++ —
// parse/emit/identity read bike::io::Value and owned records only).
// The nanobind plan materializer, Piece/row boxing and module
// registration stay in runtime/encode_binding.{hpp,cpp} on the optional
// extension; samples_binding.cpp delegates through encode_binding.hpp.
```

## T12

Port configuration and parameter schemas without changing defaults

```cpp
namespace bike::configuration {
struct SimulationPhysicsConfig; // full field list: migration/dataclass-contracts.md
struct RiderSpecs;
SimulationPhysicsConfig default_physics();
void validate(const SimulationPhysicsConfig& config);
bike::io::Value schema();
}
// Reuse native typed subrecords when they are semantically identical;
// input-facing defaults are not borrowed from zero-initialized wire structs.
```

## T13

Port TOML/JSON loading, merge precedence and asset resolution

```cpp
namespace bike::configuration {
SimulationPhysicsConfig load_physics(const std::filesystem::path& file);
SimulationPhysicsConfig resolve_physics(const bike::io::Value& overrides,
  const SimulationPhysicsConfig& base);
std::string write_physics_toml(const SimulationPhysicsConfig& config);
}
// Relative referenced assets resolve against the declaring configuration file.
```

## T14

Port geometry and Horst-link kinematics

```cpp
namespace bike::kinematics {
double brentq(const std::function<double(double)>& f, double a, double b,
              double xtol = 1e-12);
class HorstLinkageSolver; // public operations mirror solver.py's named solves
}
// Geometry source units remain mm; conversions to engine meters stay explicit.
```

## T15

Port mass properties, suspension setup and rider geometry

```cpp
namespace bike::model {
std::array<double,9> ring_inertia(double mass_kg, double inner_m,
                                double outer_m, double width_m);
std::array<double,9> parallel_axis(double mass_kg, std::array<double,3> offset);
}
// Row-major full 3x3 tensors, not just diagonal moments. Cylinder axis is Y.
// RiderSpecs/SeatedPose/LegChain/ArticulatedPose retain the source field contracts
// documented in migration/dataclass-contracts.md.
```

## T16

Generate and compile physical MJCF entirely in C++

```cpp
namespace bike::model {
struct BuildRequest; // resolved bike, rider, masses, mode, terrain and physics
std::string generate_mjcf(const BuildRequest& request);
struct ModelDeleter { void operator()(mjModel*) const noexcept; };
using ModelOwner = std::unique_ptr<mjModel, ModelDeleter>;
ModelOwner compile_mjcf(std::string_view xml, const std::filesystem::path& asset_root);
}
// Reuse engine containment for compilation; report XML diagnostics with context.
```

## T17

Port standard, stand, playground and legacy MJCF exports

```cpp
namespace bike::model {
void export_mjcf(const BuildRequest& request, const std::filesystem::path& file);
void export_geometry_json(const BuildRequest& request, const std::filesystem::path& file);
void export_playground_models(const std::filesystem::path& directory);
}
```

## T18

Resolve model names into typed native writer/runtime configuration

```cpp
namespace bike::model {
int required_id(const mjModel& model, mjtObj kind, std::string_view name);
}
namespace runtime {
RuntimeConfig project_runtime(const bike::configuration::SimulationPhysicsConfig& config,
  const mjModel& model);
}
// Full writer projection includes resolved tire material/terrain and rider geometry;
// input-facing config and model-specific IDs must not become a single mutable object.
```

## T19

Reproduce seeded random streams and distribution transforms

```cpp
namespace bike::random {
class SeedSequence {                    // bit_generator.pyx, pool_size fixed 4
public:
  explicit SeedSequence(std::uint64_t entropy);
  explicit SeedSequence(std::string_view entropy);          // int()/0x grammar
  SeedSequence(std::string_view entropy,
               std::span<const std::uint32_t> spawn_key);
  const std::vector<std::uint32_t>& spawn_key() const noexcept;
  std::uint32_t n_children_spawned() const noexcept;
  const std::array<std::uint32_t, 4>& pool() const noexcept;
  std::vector<std::uint32_t> generate_state_u32(std::size_t n_words) const;
  std::vector<std::uint64_t> generate_state_u64(std::size_t n_words) const;
  std::vector<SeedSequence> spawn(std::uint32_t n_children);
};
class NumpyRng {                        // Generator(PCG64) seeded by SeedSequence
public:
  explicit NumpyRng(std::uint64_t seed);
  NumpyRng(std::string_view nonnegative_decimal_entropy,
           std::span<const std::uint32_t> spawn_key);
  explicit NumpyRng(const SeedSequence& sequence);
  ~NumpyRng();
  NumpyRng(NumpyRng&&) noexcept;
  NumpyRng& operator=(NumpyRng&&) noexcept;
  NumpyRng(const NumpyRng&) = delete;
  NumpyRng& operator=(const NumpyRng&) = delete;
  std::uint64_t raw64();
  std::uint32_t raw32();                // buffered low-half-first lane
  double uniform01();                   // Generator.random scalar
  double normal01();                    // Ziggurat standard normal
  void uniform01(std::span<double> out);
  void normal01(std::span<double> out);
  double uniform(double low, double high);
  void uniform(double low, double high, std::span<double> out);
  double normal(double loc, double scale);
  void normal(double loc, double scale, std::span<double> out);
  std::int64_t integers(std::int64_t low, std::int64_t high,
                        bool endpoint = false);
  std::int64_t integers(std::int64_t high);
  void integers(std::int64_t low, std::int64_t high,
                std::span<std::int64_t> out, bool endpoint = false);
  std::int64_t choice(std::int64_t population);
  std::int64_t choice(std::span<const double> probabilities);
  void choice(std::span<const double> probabilities,
              std::span<std::int64_t> out);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
struct SensorNoiseConfig { double acceleration_std_mps2 = .05,
  gyro_std_rad_s = .002, encoder_std_rad_s = .01, torque_std_nm = .05;
  void validate() const; };             // sensor_noise.py's only config reads
struct NoiseTape { std::vector<std::array<double, 9>> noise;
  std::vector<double> dropout_uniform; void validate() const; };
class SensorNoiseSource {               // SeedSequence(seed).spawn(2), one parent
public:
  explicit SensorNoiseSource(SensorNoiseConfig config, std::uint64_t seed = 0);
  std::pair<std::array<double, 9>, double> draw();
};
NoiseTape build_noise_tape(const SensorNoiseConfig& config,
                           std::uint64_t seed, std::int64_t count);
struct RiderRandomSpec { /* five (low, high) pairs, Python defaults */
  void validate() const; };
struct RiderKeyframe { double time_s; rider::RiderPosture posture;
  std::optional<double> human_torque_nm; };
struct RiderProgram { std::vector<RiderKeyframe> keyframes;
  double reaction_delay_s = 0.; };
struct SampledRider { configuration::RiderSpecs rider; RiderProgram program; };
SampledRider sample_rider(const RiderRandomSpec& spec, std::uint64_t seed);
SampledRider sample_rider(const RiderRandomSpec& spec,
                          const SeedSequence& seed);
}
// Header: native/src/random/numpy_compat.hpp; TUs: numpy_compat.cpp (PCG64
// core incl. the buffered next_uint32 lane), seed_sequence.cpp (entropy
// coercion + INIT_A/MULT_A mixing + spawn bookkeeping), distributions.cpp
// (Ziggurat tables + Lemire 32/64 + kahan_sum), noise_tape.cpp
// (sensor_noise.py + rider_random.py seams). No Python/nanobind/numpy
// dependency; every draw order and bit pattern pinned by random/* fixtures.
```

## T20

Port terrain primitives, profiles, surfaces and heightfields

```cpp
namespace bike::terrain {
struct TrackSpec;
TrackSpec flat(double length_m);
std::vector<std::array<double,2>> profile(const TrackSpec& track, double resolution_m);
}
// Existing biketyre::SignedProfile and native tyre/distance.cpp are reused
// wherever they implement the same nearest-profile contract.
```

## T21

Port all track generators, presets and track-file round trips

```cpp
namespace bike::terrain {
TrackSpec load_track(const std::filesystem::path& file);
std::string dump_track(const TrackSpec& track);
TrackSpec generate(const bike::io::Value& specification, std::uint64_t seed);
std::vector<std::string> available_presets();
}
```

## T22

Port physical settling without a reference RideSimulation

```cpp
namespace bike::initialization {
struct EquilibriumOptions { std::optional<std::int64_t> max_steps; double tolerance = 0.05; };
struct EquilibriumResult { std::int64_t steps; double residual_qacc; bool cache_hit; };
class SetupOwner; // complete PImpl interface in migration/interface-contracts.md
std::unique_ptr<SetupOwner> make_setup(const std::filesystem::path& request_toml);
EquilibriumResult settle(SetupOwner& setup, const EquilibriumOptions& options);
}
// SetupOwner owns a model/Stepper and reuses the same physical writer path.
// A NativeRideRuntime is constructed only after a completed t=0 bootstrap exists.
```

## T23

Port bounded least squares, equilibrium refinement and sag fitting

```cpp
namespace bike::numerics {
struct LeastSquaresResult { std::vector<double> x; double cost; int evaluations; bool converged; };
LeastSquaresResult bounded_least_squares(
  const std::function<std::vector<double>(std::span<const double>)>& residual,
  std::span<const double> initial, std::span<const double> lower,
  std::span<const double> upper, int max_evaluations);
}
// Solver-specific tolerances/scaling/finite-difference modes live in an explicit
// options overload; initialize them from the exact calls in the Python sources.
```

## T24

Replace Python equilibrium cache/state files and retain old readers

```cpp
namespace bike::initialization {
struct StateFile;
StateFile read_state_file(const std::filesystem::path& file);
void write_state_file(const std::filesystem::path& file, const StateFile& state);
std::string fingerprint(const StateFile& state);
}
// Version any new writer format explicitly; retain read-only import of the
// archived NPZ/NPY variants actually used by equilibrium_cache.py.
```

## T25

Construct standalone native rides from user configuration

```cpp
namespace bike::app {
struct PreparedSimulation { runtime::RuntimeBootstrap bootstrap; bike::io::Value metadata; };
PreparedSimulation prepare(const std::filesystem::path& request_toml);
std::unique_ptr<runtime::NativeRideRuntime> make_ride(PreparedSimulation prepared);
}
// Request files carry explicit physics profile, track, rider, initial conditions
// and output-independent options. They never reference a Python callable.
// No create_ride alias: callers use prepare() followed by make_ride().
```

## T26

Close parity for already-ported kernels and retire their Python mirrors

```text
The reuse map records {python_symbol, native_symbol, source_locations,
 test_ids, comparison_class, remaining_branches}. A symbol is deletable only
 when every reachable branch and named input domain has native coverage.
```

## T27

Implement remaining drivetrain capabilities

```cpp
namespace drivetrain {
struct Capability { bool elastic_chain; bool motor_clutch; bool reflected_rotor; };
Capability implemented_capabilities();
}
// Extend existing DriveConfig/DrivetrainWriter; do not create a parallel drive engine.
```

## T28

Port constrained allocation, linear feasibility and calibration solvers

```cpp
namespace bike::rider {
struct Allocation { std::vector<double> solution; bool feasible; double violation; };
struct AllocationProblem; // explicit matrices, bounds and differentiable extras
AllocationProblem allocation_problem_from_json(const bike::io::Value& value);
Allocation allocate(const AllocationProblem& problem);
}
// Port the exact nearest-feasible-point objective and source residual check;
// optimizer success alone never determines feasibility.
```

## T29

Port remaining rider dynamics and coupled task controllers

```cpp
namespace bike::rider {
struct ModeSearchResult { bool complete; std::size_t modes_tried; Allocation allocation; };
// Controller-specific typed interfaces mirror rider_dynamics.py,
// rider_response_allocation.py and rider_control.py and consume the same
// owned model context as existing SpindleController, never a foreign mjData*. 
}
```

## T30

Implement flat/weld/spring rider supports and missing physical modes

```cpp
namespace runtime {
struct CapabilityReport { bool supported; std::vector<std::string> reasons; };
CapabilityReport capabilities(const bike::configuration::SimulationPhysicsConfig& config,
                              const bike::configuration::RiderSpecs& rider);
}
// Successful parsing is distinct from supported execution; each mode has a
// tested writer/controller/topology combination before capabilities returns true.
```

## T31

Port legacy tire backends and nonphysical ride integration

```cpp
namespace bike::legacy {
class RideSimulation; // owned model/data and explicit control/step/snapshot/reset
std::unique_ptr<RideSimulation> make_ride(const std::filesystem::path& request);
}
// Do not relabel the old legacy pneumatic model as the newer native pneumatic
// carcass/brush backend; they are separate source-supported behaviors.
```

## T32

Port remaining legacy rider/control/measurement behavior

```cpp
namespace bike::numerics {
std::vector<double> legacy_ride_filter(std::span<const double> samples,
                                     double sample_rate_hz);
}
// The exact order/cutoff/padding settings come from sim/ride/metrics.py.
// Low sample counts use the same source behavior, not an invented filter fallback.
```

## T33

Port suspension playground and stand controllers

```cpp
namespace bike::legacy {
struct Playground;
std::unique_ptr<Playground> make_playground();
void step(Playground& playground);
void reset(Playground& playground);
}
// The GUI adapter is T48; this task delivers the testable non-windowed model/controller.
```

## T34

Construct research environments, programs and sensor tapes natively

```cpp
namespace bike::research {
std::unique_ptr<runtime::NativeResearchRuntime> make_environment(
  const std::filesystem::path& request_toml);
}
// Existing runtime::ResearchConfig, NoiseTape, SensorState, programs and sensor
// kernels remain the runtime contract; construction no longer takes ResearchEnvironment.
```

## T35

Replace Python motor policies with built-ins and a versioned native ABI

```cpp
// Stable plugin entrypoint; only C-compatible scalars/arrays and explicit sizes.
// The host owns observation/control/error memory; no STL object crosses this ABI.
extern "C" const BikePolicyV1* bike_policy_v1(void);
// BikePolicyV1 defines version/struct_size plus create, reset, step and destroy.
// create/reset/act return a status and writes bounded error text; no exception
// crosses the ABI. Observation fields come only from SensorObservation.
namespace bike::research {
std::unique_ptr<Policy> load_policy(std::string_view specification);
}
```

## T36

Port policy sessions, pacing boundaries and lifecycle coordination

```cpp
namespace bike::research {
class PolicySession {
public:
  PolicySession(std::unique_ptr<runtime::NativeResearchRuntime> environment,
                std::unique_ptr<Policy> policy);
  ~PolicySession();
  PolicySession(const PolicySession&) = delete;
  PolicySession& operator=(const PolicySession&) = delete;
  void begin_advance();
  std::optional<runtime::ResearchTransition> advance_pending(
    std::optional<double> wall_budget_s, std::optional<std::int64_t> target_step);
  void request_stop();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
// Constructor takes an owned environment and Policy; pending command ownership
// survives pauses/yields and is cleared only at its documented boundary.
```

## T37

Port research metrics, validity and experiment evidence

```cpp
namespace bike::research {
bike::io::Value summarize(const runtime::OwnedResearchEpisode& episode);
}
// Reuse runtime/research_metrics.cpp and status.cpp where equivalent;
// this API assembles complete frontend/recording metadata without Python views.
```

## T38

Port the full recording catalog and schema descriptors

```cpp
namespace bike::recording {
struct Catalog;
Catalog build_catalog(const mjModel& model);
bike::io::Value describe_catalog(const Catalog& catalog);
}
// Container schema=1; research replay contract=3. These are separate version axes.
// Canonical schema descriptors include units, frame, phase, optionality and children.
```

## T39

Port source encoding plans and ingest native column pieces

```cpp
namespace bike::recording {
runtime::EncodePlan compile_plan(const Catalog& catalog, std::string_view table);
void validate_pieces(const runtime::EncodedTable& table, const Catalog& catalog,
                     std::string_view table_name);
}
// The existing Piece is the source of typed values, offsets and validity.
// DuckDB chunks are bounded publication-time batches, not retained duplicate histories.
```

## T40

Port RAM-only run/episode capture and retained evidence

```cpp
namespace bike::recording {
class RunCapture; // owns metadata, consecutive episodes and immutable evidence
// append/seal do not write files; finalize is supplied by T42.
struct RunMetadata { std::string run_uuid, started_utc; double wall_origin_s; };
}
// Native generation and run episode_id are different identifiers.
```

## T41

Write and validate DuckDB recordings natively

```cpp
namespace bike::recording {
void write_database(const std::filesystem::path& file, const RunCapture& capture);
void validate_database(const std::filesystem::path& file);
}
// Database write/validation use the pinned native DuckDB library, no external
// executable, Python module, network extension download or automatic spill design change.
```

## T42

Preserve exclusive final publication and retry semantics

```cpp
namespace bike::recording {
struct Publication { std::filesystem::path directory; std::string report_status; };
Publication finalize(RunCapture& capture, const std::filesystem::path& output_root);
}
// Successful repeated finalize returns its original path. Failed publication
// keeps the capture retryable and chooses a fresh exclusive destination.
```

## T43

Port read-only recording validation and analyst operations

```cpp
namespace bike::recording {
class Recording;
std::unique_ptr<Recording> open_recording(const std::filesystem::path& directory);
bike::io::Value overview(const Recording& recording);
bike::io::Value query(const Recording& recording, std::string_view sql,
                     std::uint64_t row_limit);
}
```

## T44

Port checked replay and versioned legacy compatibility

```cpp
namespace bike::replay {
struct ReplayResult { bool matched; bool physically_valid; std::string reason; };
ReplayResult check(const std::filesystem::path& recording, std::int64_t episode);
}
// Playback of stored states and checked re-simulation are separate operations.
// A historical Python execution identity is not silently relabeled as native.
```

## T45

Port summary reports and explicit data exports

```cpp
namespace bike::recording {
std::string render_summary(const Recording& recording);
void export_table(const Recording& recording, std::int64_t episode,
                  std::string_view table, std::string_view format,
                  const std::filesystem::path& output,
                  std::optional<std::uint64_t> limit = std::nullopt);
}
// Source-supported analyst formats are exactly CSV and JSONL.
// Explicit legacy replay-bundle export is a separate native/runtime operation.
```

## T46

Deliver the standalone bike-recording analyst command

```text
int bike_recording_main(std::span<const std::string_view> arguments);
// Keep all source parser subcommands/options, selectors and tagged-JSON outputs.
// Output paths and table/query values are passed as arguments, never shell fragments.
```

## T47

Replace all remaining Python CLI entrypoints with installed native commands

```cpp
namespace bike::cli {
int dispatch(std::string_view executable, std::span<const std::string_view> arguments);
}
// Installed names: bike-sim, bike-playground, bike-export, bike-ride,
// bike-settle, bike-research, bike-batch, bike-replay, bike-recording.
// bike-batch is wired to T51; do not advertise it as implemented before T51.
```

## T48

Implement a native render owner with independent simulation state

```cpp
namespace runtime {
struct RenderFrame { RuntimeSnapshot state; PresentationState presentation; };
// Add NativeRideRuntime::render_frame() const returning this owned record.
}
namespace bike::viewer {
class RenderOwner {
 public:
  explicit RenderOwner(const runtime::ModelImage& model);
  void apply(const runtime::RenderFrame& frame);
  void draw(int framebuffer_width, int framebuffer_height);
  ~RenderOwner();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
// GLFW lifetime/event/window work belongs to the main thread. The initial
// PresentationState is captured by native initialization, not a Python amend().
```

## T49

Port viewer controls, HUD, cameras and cooperative pacing

```cpp
namespace bike::viewer {
struct PaceDecision { std::uint64_t physics_steps; bool request_frame; };
class Pacer {
 public:
  explicit Pacer(double dt);
  ~Pacer();
  Pacer(const Pacer&) = delete;
  Pacer& operator=(const Pacer&) = delete;
  void set_scale(unsigned scale); // exactly 1, 2, 4 or 8
  PaceDecision poll(std::chrono::steady_clock::time_point now);
  void completed(std::uint64_t actual_steps);
  void reset(std::chrono::steady_clock::time_point now);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
// Runtime advances remain bounded by the existing interval/stop/yield contract.
```

## T50

Replace Matplotlib and Python Plotly construction with native artifact generation

```cpp
namespace bike::viz {
struct Series { std::string name; std::vector<double> x, y; };
struct Plot { std::string title, x_label, y_label; std::vector<Series> series; };
std::string to_svg(const Plot& plot);
std::string to_html(const Plot& plot);
void write_png(const Plot& plot, const std::filesystem::path& path,
               unsigned width, unsigned height);
}
// Plot is the common primitive; task-owned geometry/dyno/ride adapters retain
// domain panels, labels, units, range conventions and marker meanings.
```

## T51

Port batch jobs and research sweeps with deterministic process ownership

```cpp
namespace bike::batch {
struct Job { std::string id; std::vector<std::string> argv; };
struct Result { std::string id; int exit_code; bike::io::Value summary; };
std::vector<Result> execute(std::span<const Job> jobs, unsigned workers);
}
// Results are returned in input job order; process completion order is not identity.
```

## T52

Port every numerical, unit and property assertion out of pytest

```text
Each test-port-map record has:
{source_path, source_symbol, collected_node_ids, owner_task, native_test_id,
 native_source, assertions, fixtures, comparison_class, disposition, evidence}.
Allowed dispositions: ported, duplicate-with-proof, obsolete-Python-boundary-with-replacement.
A filename deletion or one smoke test cannot stand in for every parameterized case.
```

## T53

Port integration, frontend, replay and failure-path test suites

```cpp
namespace bike::tests {
struct FaultPoint { std::string name; std::uint64_t occurrence; };
// Fault injection is compiled into test-only collaborators, not enabled by
// an undocumented production environment variable or user-controlled metadata.
}
// Test executables register stable CTest IDs recorded in test-port-map.json.
```

## T54

Port validation rigs, calibration and physical release evidence

```cpp
namespace bike::validation {
struct Verdict { bool accepted; bike::io::Value evidence; };
Verdict run_case(std::string_view case_id, const std::filesystem::path& configuration);
}
// Names and requirements come from the supplied validation modules and datasets;
// the dispatch registry lists every retained validation operation.
```

## T55

Port performance, diagnostic and native-benchmark developer tools

```text
struct BenchmarkSummary {
  std::uint64_t requested_steps, completed_steps;
  double simulated_seconds, wall_seconds;
  bool physically_valid;
};
// All report arithmetic uses actual completed steps. State interchange preserves
// tools/proto_native_bench/state_format.py section ordering/checksums/dimensions.
```

## T56

Replace Python native-check orchestration and inline interpreter scripts

```bash
bike-native-checks --kind frontends --build build/native-release
bike-native-checks --kind tidy --build build/native-release
bike-native-checks --kind analyzer --build build/native-release
bike-native-checks --kind cppcheck --build build/native-release
bike-native-checks --kind odr --build build/native-release
bike-native-checks --kind scripts --build build/native-release
bike-native-checks --kind headers --build build/native-release
bike-native-checks --kind diagnostic-controls --build build/native-release
bike-native-checks --kind context --build build/native-release
```

## T57

Deliver a native test runner, sanitizers and coverage provenance

```bash
bike-test-runner --profile quick --build build/native-release
bike-test-runner --profile native --build build/native-release
bike-test-runner --profile full --build build/native-release
// Native and full preserve the existing shared native preflight before tests.
// Every report names the exact executable, build context and fixture manifest.
```

## T58

Qualify Linux headless native support without changing macOS arithmetic silently

```text
The supported release matrix initially requires:
- macOS on the architecture/provider captured by T01: existing strict parity.
- Linux x86_64 headless: native replacement for current offline wheel bundles.
No Windows or Linux GUI support is claimed without its own completed gates.
```

## T59

Replace Python wheel installers with offline native distributions

```bash
bike-package-verify --root dist/bike-sim --manifest dist/bike-sim/manifest.json
bike-install --bundle dist/bike-sim --prefix install-test
// Runtime bundles contain native binaries, locked shared libraries, model/config
// assets, licenses, help and offline report resources; no wheels or interpreter.
```

## T60

Update active documentation and agent contracts to the native workflow

```text
migration/command-map.json maps every active old command to:
{old_command, native_command, behavior_notes, verification_id}.
Historical research logs are retained as evidence and marked historical;
active setup/run/test instructions contain no Python/uv/mjpython requirements.
```

## T61

Run end-to-end release qualification and close capability parity

```text
A capability row contains:
{mode, source_entrypoint, config_fixture, native_entrypoint, parity_tests,
 platform_results, recording_results, replay_results, status}.
Allowed final status is passed, or explicit-user-approved-breaking-change.
Agent discretion is not user approval.
```

## T62

Enforce executable and dependency-level absence of Python

```bash
bike-python-audit --root . --mode source
bike-python-audit --root install-test --mode package
bike-python-audit --root . --mode documentation
// source: first-party executable Python files/invocations/imports/build edges.
// package: binaries, dynamic dependencies, shebangs, bytecode, wheels and launch trace.
// documentation: active command blocks only; historical prose is not banned.
```

## T63

Delete Python implementations, adapters and packaging only after parity gates

```text
Final deletion transaction:
1. verify every removal row has passed replacement/compatibility evidence;
2. delete exactly the reviewed first-party paths and binding-only C++ adapters;
3. remove Python build/package/dependency discovery and transition fixtures tools;
4. rebuild, test, package and audit from a new clean native-only worktree.
The immutable reference tag and frozen data fixtures remain available as history/data.
```
