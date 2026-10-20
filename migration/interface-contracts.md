# Native interface contracts and implementation seams

This document defines proposed interfaces, while the source excerpts define existing behavior. New paths in the plans are deliberate additions, not claims about files already present. Include a type's owning header; do not copy a second local declaration into another subsystem. Changes to a shared signature require the integration owner to update the producer, all consumers, tests and this document in one reviewed change.

## 1. Reused types and their owners

| Type | Existing source authority | Migration owner |
|---|---|---|
| `Stepper` | `native/src/stepper.hpp` | T07/T09: add typed config/model-image construction; preserve lifetime |
| writer configs and validation | `native/src/config_types.hpp`, `native/src/config.hpp` | T07 |
| `runtime::Wire`, `WireObject`, `RideControl` | `native/src/runtime/control.hpp` | T08/T09; keep insertion order and `rider::IntentControl` alias |
| `rider::RiderPosture`, `IntentControl`, `SpindleConfig` | `native/src/rider/rider_posture.hpp`, `intent.hpp`, `spindle_controller.hpp` | T08/T29/T30 |
| `runtime::RuntimeConfig`, `BootstrapState` | `native/src/runtime/config.hpp`, `bootstrap.hpp` | T09: replace binding members with typed existing records |
| `runtime::RuntimeSnapshot`, `AdvanceResult`, `NativeRideRuntime` | `native/src/runtime/runtime_binding.hpp` | T09: move core to `runtime.hpp` |
| `runtime::PresentationState` | `native/src/runtime/presentation.hpp` | T11/T48 |
| `runtime::ResearchConfig`, `ResearchTransition`, `OwnedResearchEpisode` | `native/src/runtime/research.hpp` | T10 |
| `runtime::SensorObservation`, `NoiseTape`, `SensorState` | `native/src/runtime/sensors.hpp` | T10/T34 |
| `runtime::OwnedNativeEpisode` and sample records | `native/src/runtime/samples.hpp` | T11 |
| `runtime::Piece`, `EncodePlan` | `native/src/runtime/encode.hpp`, `encode.cpp` | T11: move complete EncodePlan to a pure header |
| input dataclasses/defaults/validators | `migration/dataclass-contracts.md` plus referenced source | T12 and the ledger's domain owner |

`runtime` is the existing global namespace. Do not invent `bike::runtime`. New orchestration namespaces use `bike::app`, `bike::configuration`, `bike::model`, `bike::initialization`, `bike::recording`, `bike::viewer`, `bike::research` and similar focused names. Existing `rider`, `drivetrain` and `biketyre` namespaces remain where currently used.

## 2. Owned cold-path values (T05)

Use this shape, with declarations in `native/src/io/value.hpp`. `Number` is a validated JSON number lexeme, not a string value. Encoding tagged recording types is separate from raw JSON syntax. The source `json_codec.py` remains the tag authority.

```cpp
namespace bike::io {
struct Number { std::string lexeme; };
struct Bytes { std::vector<std::byte> value; };
struct Tagged {
  std::string kind;
  std::vector<std::pair<std::string, std::string>> fields;
};
struct Value {
  using Array = std::vector<Value>;
  using Object = std::vector<std::pair<std::string, Value>>;
  std::variant<std::monostate, bool, Number, std::string,
               Array, Object, Bytes, Tagged> data;
};
Value parse_json(std::string_view source);
std::string canonical_json(const Value& value);
std::string recording_json(const Value& value);
Value parse_recording_json(std::string_view source);
std::string sha256(std::span<const std::byte> bytes);
}
```

Missing object members are absent entries; explicit null is `monostate`. Boolean never travels through the numeric alternative. An object is an insertion-ordered vector with duplicate-key rejection during parsing. `canonical_json` is deterministic for fixture/provenance use; external CLI/recording formatting follows the captured source codec and is not silently reformatted by the hash writer. Preserve integer versus floating spelling where it carries source type. Preserve negative zero. Native typed extraction checks the target domain/range before conversion.

## 3. Startup requests and SetupOwner (T22/T25)

`request.toml` is an internal fixture/application assembly format, not a new public config format. Version 1 accepts these exact top-level keys; paths resolve relative to the request file. Additional metadata belongs to captured fixtures, not unrecognized executable fields.

```toml
request_schema = 1
physics_file = "../config/viewer_physics_welded.toml"
track_file = "../tracks/flat.toml"
model_mode = "ride"
rider_variant = "articulated_planar"
speed_kph = 0.0
seed_decimal = "1234"
use_equilibrium_cache = false
# Optional initial_state_file is absent when not requested.
# Optional overrides is a table using the existing physics field schema.
```

`model_mode` uses the source builder mode names captured in T01; the example is a ride. `physics_file` and `track_file` are required for file-driven fixture requests. `rider_variant`, `speed_kph`, and cache/initial-state choices retain source validation and are set explicitly in golden requests. A decimal seed string supports the source integer domain without limiting it to a JavaScript/double number or silently narrowing it to uint64. The default request writer emits every effective choice; no hidden environment override exists.

```cpp
namespace bike::initialization {
class SetupOwner {
 public:
  ~SetupOwner();
  SetupOwner(const SetupOwner&) = delete;
  SetupOwner& operator=(const SetupOwner&) = delete;
  runtime::RuntimeBootstrap seal_bootstrap();
  bike::io::Value metadata() const;
 private:
  struct Impl;
  explicit SetupOwner(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
  friend std::unique_ptr<SetupOwner> make_setup(const std::filesystem::path&);
  friend EquilibriumResult settle(SetupOwner&, const EquilibriumOptions&);
};
std::unique_ptr<SetupOwner> make_setup(const std::filesystem::path& request_toml);
}
namespace runtime {
struct ModelImage { std::vector<std::byte> mjb; std::string sha256; };
struct RuntimeBootstrap {
  ModelImage model;
  RuntimeConfig config;
  BootstrapState state;
};
}
namespace bike::app {
struct PreparedSimulation {
  runtime::RuntimeBootstrap bootstrap;
  bike::io::Value metadata;
};
PreparedSimulation prepare(const std::filesystem::path& request_toml);
std::unique_ptr<runtime::NativeRideRuntime> make_ride(PreparedSimulation prepared);
}
```

Declare `EquilibriumOptions` and `EquilibriumResult` from T22 before `SetupOwner`. Implementation details stay in `setup_owner.cpp`; compile the out-of-line destructor where `Impl` is complete. `seal_bootstrap` fails if setup is incomplete/invalid and can transfer immutable completed state only once. No arbitrary mutator is exposed on the live runtime. `make_setup` may initially handle frozen flat-track requests while T21 completes other generators; T25 cannot finish until every required source startup mode is integrated.

`RuntimeBootstrap::state` includes the captured initial presentation and all mutable model/writer/controller state. A valid model byte stream is not enough to validate compatibility: include the schema, engine/provider identity, model hash and dimensions/IDs. Add `Stepper(std::span<const std::byte>, const nativecfg::WriterConfig&)` alongside the path constructor; both use one native model loader and owned storage.

## 4. New records with opaque implementation

Where a type is forward-declared in a task, define a complete owning class in its named header, with an out-of-line destructor, deleted copy operations unless a tested immutable copy is required, and explicit move behavior. Do not return an incomplete type to callers that must construct/destroy it without its owning header.

`BuildRequest` (T16) is a complete owner of resolved configuration, geometry, rider, mass, mode and terrain, built by the configuration/model assembly path. Its fields come from the source schemas; it is not an arbitrary JSON dictionary passed into hot writers. `TrackSpec` (T20) transcribes the source terrain schema and all profile/surface/preset fields in the source contract excerpts. `Catalog` (T38) is an immutable descriptor owner transcribing catalog.py's tables/fields/coordinate/alias definitions. `StateFile` (T24) owns source version/metadata plus typed dtype/shape/endian array payloads, not views into an NPZ reader buffer.

The source-to-native symbol cards name the exact source declarations for these types. These are prescribed source transcriptions, not opportunities to invent shortened schemas. Add them to the public header and their constructor/domain tests before consuming them in another task. `Recording`, `RunCapture`, legacy `RideSimulation` and `Playground` likewise require complete public owners and out-of-line resource cleanup; the source lifecycle methods remain the API authority.

## 5. Allocation/solver seams (T23/T28)

These definitions keep solver callers from inventing incompatible matrix formats. Arrays are row-major and dimension-checked; matrices own values.

```cpp
namespace bike::rider {
struct Matrix {
  std::size_t rows = 0, cols = 0;
  std::vector<double> values;
};
struct ExtraConstraint {
  std::function<std::vector<double>(std::span<const double>)> evaluate;
  std::function<Matrix(std::span<const double>)> jacobian;
  std::vector<double> lower, upper;
};
struct AllocationProblem {
  std::vector<double> target;
  Matrix equality, inequality;
  std::vector<double> equality_rhs, inequality_rhs, lower, upper;
  std::vector<ExtraConstraint> extra;
  std::optional<std::vector<double>> initial;
};
struct Allocation {
  std::vector<double> solution;
  bool feasible = false;
  double violation = std::numeric_limits<double>::infinity();
};
AllocationProblem allocation_problem_from_json(const bike::io::Value& value);
Allocation allocate(const AllocationProblem& problem);
}
```

The JSON fixture loader supports named native constraint fixtures, never arbitrary executable expressions. An absent analytic Jacobian is represented by an empty callable and invokes the source-compatible finite-difference path. Equality/inequality convention is exactly `Aeq*x=beq`, `G*x<=h`. Bounds may be infinite; coefficients and target must be finite. A failed linear feasibility solve reports unknown/failure separately from proven infeasible. Solver-specific finite-difference/scaling/tolerance options are explicit internal records initialized from the source call sites, not generic defaults guessed by each consumer.

`LeastSquaresResult` from T23 records x, cost, evaluation count and convergence. Add a typed options overload carrying method, finite-difference scheme, x-scale policy, bounds handling, tolerances and evaluation limits, with values taken from each captured source call. The short overload in T23 is only the simple solver-control test surface. Integration callers use the explicit options overload.

## 6. Native policy ABI (T35)

Put this C-compatible contract in `native/include/bike/policy_v1.h`. Use `extern "C"` guards in C++ consumers and export the single `bike_policy_v1` symbol. Loaders require version 1, sufficient struct sizes and all required callbacks; reject unknown required capability flags. Native plugins are trusted native code, not a sandbox. No exception, STL container, allocator ownership or engine pointer crosses this ABI.

```c
#include <stddef.h>
#include <stdint.h>

typedef struct BikeErrorV1 {
  char* data;
  size_t capacity;
  size_t written;
} BikeErrorV1;

typedef struct BikeObservationV1 {
  uint32_t struct_size;
  uint8_t valid;
  double time_s;
  double source_time_s;
  double values[9];
} BikeObservationV1;

typedef struct BikeDemandV1 {
  uint8_t present;
  double torque_nm;
} BikeDemandV1;

typedef struct BikeControlV1 {
  uint32_t struct_size;
  uint32_t present_mask;
  double motor_torque_nm;
  double motor_limit_nm;
  double human_torque_nm;
  double crank_target_rate_rad_s;
  double torso_lean_rad;
  double pelvis_pitch_rad;
  double pelvis_offset_m[2];
  uint8_t use_saddle;
  uint8_t rider_enabled;
} BikeControlV1;

typedef struct BikePolicyV1 {
  uint32_t abi_version;
  uint32_t struct_size;
  uint64_t required_capabilities;
  int (*create)(const char* config_json, size_t config_size,
                void** instance, BikeErrorV1* error);
  int (*reset)(void* instance, const char* seed_decimal, size_t seed_size,
               BikeErrorV1* error);
  int (*act)(void* instance, const BikeObservationV1* observation,
             const BikeDemandV1* demand, BikeControlV1* control,
             BikeErrorV1* error);
  void (*destroy)(void* instance);
} BikePolicyV1;

const BikePolicyV1* bike_policy_v1(void);
```

`values` follows the existing `runtime::SensorObservation`: acceleration x/y/z, gyro, front/rear/crank encoder, motor/human torque. `present_mask` bits 0..3 mark the four scalar optional control values; bit 4 marks posture; bit 5 marks the optional pelvis offset and is invalid unless bit 4 is set. Unknown bits fail. All uint8 booleans must be 0 or 1. Host initialization sets `rider_enabled=1`, `use_saddle=1`, zero scalar fields and an empty presence mask. An absent motor request is not numeric zero. Validate the resulting command through the same core `RideControl`/posture domain checks before acceptance.

The host owns all input/output/error buffers for the duration of a callback. A callback may not retain those pointers. `create` returns its own opaque instance; only the same module destroys it. Status zero means success; nonzero means failure with bounded error text. `destroy` is nonthrowing and cannot report recoverable failure. Host code checks `written<=capacity` before using diagnostic text; no unchecked C string scan is required. A crashing/malicious in-process plugin cannot be made safe by this ABI and is outside the trust model.

C++ orchestration uses:

```cpp
namespace bike::research {
class Policy {
 public:
  virtual ~Policy() = default;
  virtual void reset(std::string_view seed_decimal) = 0;
  virtual runtime::RideControl act(const runtime::SensorObservation& observation,
                                  std::optional<double> demand_nm) = 0;
};
std::unique_ptr<Policy> load_policy(std::string_view specification);
}
```

Accept captured source built-in names and known factory aliases through a static native registry. The `builtin:zero` spelling in T35 is an additional explicit native spelling, not permission to drop old built-in CLI names. User native libraries use an explicit library path and recorded ABI/config identity. Unknown Python module strings produce an actionable error and never invoke Python. Preserve traction-control state/reset/composition and the source information boundary in tests.

## 7. Render and publication seams

`runtime::RenderFrame` contains owned `RuntimeSnapshot` and `PresentationState`. `NativeRideRuntime::render_frame() const` performs no stepping, force/sensor work or acknowledgment mutation. Native initialization provides the source's nonzero step-zero presentation; there is no Python `_initial_frame` amendment. `RenderOwner` constructs its own model/data from `ModelImage` and applies the six declared mutable arrays before the source-compatible kinematic refresh.

`RunCapture` owns launch metadata, consecutive episode identities, immutable native/legacy evidence and publication status. `finalize` is its only publishing operation. Snapshot/seal/append operations never create recording files. Use explicit fault-injectable filesystem/database/report collaborators in tests, not production environment variables that permit users to bypass validations.

The analyst `Recording` owner validates the manifest and hashes before exposing read-only queries. It does not link to a policy loader or initialize a runtime. Output/export failure must not mutate the recording. Legacy export and checked replay retain their separate execution identities.

## 8. Concrete test harness bootstrap (T02; strengthened by T05)

The first T02 test harness can be implemented directly from this header skeleton. The source tree supplies the root through CMake. T05 then adds fixture-manifest hash/provenance validation using the shared JSON/SHA implementation before domain fixtures are accepted. That ordering avoids a circular T02/T05 dependency. Test payloads are bounded; large trajectories are chunked into declared fixture pieces.

```cpp
#pragma once
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bt {
inline void require(bool condition, std::string_view message) {
  if (!condition) throw std::runtime_error(std::string(message));
}
inline int run(const std::function<void()>& test) {
  try { test(); return 0; }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
  catch (...) { std::cerr << "non-standard test exception\n"; return 1; }
}
template<class F> void invalid(F&& action) {
  bool rejected = false;
  try { std::forward<F>(action)(); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "expected std::invalid_argument");
}
inline std::filesystem::path fixture(std::string_view relative) {
  const std::filesystem::path p{relative};
  require(!p.empty() && !p.is_absolute(), "fixture must be relative");
  for (const auto& part : p) require(part != "..", "fixture path escape");
  const auto root = std::filesystem::weakly_canonical(BIKE_FIXTURE_ROOT);
  const auto full = std::filesystem::weakly_canonical(root / p);
  const auto rel = full.lexically_relative(root);
  require(!rel.empty() && !rel.is_absolute(), "invalid fixture path");
  for (const auto& part : rel) require(part != "..", "fixture symlink escape");
  return full;
}
inline std::string text(const std::filesystem::path& p) {
  const auto size = std::filesystem::file_size(p);
  require(size <= 512ULL*1024ULL*1024ULL, "fixture payload exceeds 512 MiB");
  std::ifstream f(p, std::ios::binary);
  require(static_cast<bool>(f), "cannot open fixture");
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  require(!f.bad(), "fixture read failed");
  require(s.size() == size, "fixture changed while reading");
  return s;
}
inline std::vector<double> read_f64(const std::filesystem::path& p) {
  const auto bytes = text(p);
  require(bytes.size() % 8 == 0, "truncated little-endian binary64 fixture");
  std::vector<double> out;
  out.reserve(bytes.size()/8);
  for (std::size_t off=0; off<bytes.size(); off+=8) {
    std::uint64_t bits=0;
    for (unsigned k=0; k<8; ++k)
      bits |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes[off+k])) << (8*k);
    out.push_back(std::bit_cast<double>(bits));
  }
  return out;
}
inline void golden_text(std::string_view name, std::string_view actual) {
  require(text(fixture(name)) == actual, std::string("text mismatch: ")+std::string(name));
}
inline void golden_f64(std::string_view name, std::span<const double> actual) {
  const auto expected = read_f64(fixture(name));
  require(expected.size() == actual.size(), "binary64 shape/length mismatch");
  for (std::size_t i=0; i<actual.size(); ++i)
    require(std::bit_cast<std::uint64_t>(expected[i]) == std::bit_cast<std::uint64_t>(actual[i]),
            std::string("binary64 mismatch at index ")+std::to_string(i));
}
}
```

Move nontrivial definitions to `migration_test.cpp` as the harness grows. Keep diagnostics bounded and add source/test IDs, first mismatch, expected/actual hex bits and shape to production test reports. `invalid` intentionally accepts only the required exception type; other exceptions fail rather than falsely satisfy validation tests. Add separate helpers for source contracts requiring different typed errors.

Each task supplies a concrete regression body for its named test_tNN.cpp. Define main to return bt::run with that body in its lambda and include the actual owning headers listed by the task. Create all T02 functions before other tasks consume them. T00/T01 are acquisition/manifest tasks and do not pretend to have native feature binaries before T02/T03.

CMake registration for a concrete task uses:

```cmake
add_executable(migration_T15 test_t15.cpp)
target_link_libraries(migration_T15 PRIVATE bike_model bike_test_support)
target_compile_features(migration_T15 PRIVATE cxx_std_23)
add_test(NAME migration_T15 COMMAND migration_T15)
set_tests_properties(migration_T15 PROPERTIES LABELS "migration;unit;model")
```

Apply the shared compiler policy to every test, including this one. Set `RUNTIME_OUTPUT_DIRECTORY` consistently so commands referencing `build/native-release/bin/` are true. The root CMake project source/binary directories, not a nested native subproject's local directories, supply these paths. They reach the test sources through a configure-generated header, never a quoted compile definition: the native check-context writer embeds target properties verbatim, so a quote inside one is a configure error. `ctest --no-tests=error` is required for focused selections, or an equivalent explicit discovery-count check in the native runner.

## 9. Fixture records and comparison metadata

A fixture manifest entry is not complete until the baseline capture fills real hashes/identity and an owner has reviewed it. The JSON schema requires these fields; there are no synthetic success values:

```json
{
  "type": "object",
  "required": ["id", "input_files", "baseline_revision", "toolchain", "execution_identity", "comparison", "expected_validity", "payloads"],
  "properties": {
    "id": {"type": "string", "minLength": 1},
    "input_files": {"type": "array", "minItems": 1},
    "baseline_revision": {"type": "string", "minLength": 1},
    "toolchain": {"type": "object"},
    "execution_identity": {"type": "object"},
    "comparison": {"enum": ["B", "N", "S", "Q", "P"]},
    "expected_validity": {"type": "object"},
    "payloads": {"type": "array", "minItems": 1}
  }
}
```

Each payload includes a relative path, actual SHA-256, byte count and optional dtype/shape/order/endian metadata. Class N payloads additionally include the exact tolerance/residual rule and authority. Missing fields/hash mismatches fail fixture validation. Models/recordings can contain inherently varying UUID/timestamp fields; the comparator excludes only the explicitly enumerated volatile fields, never simulation state or validity labels.

## 10. Resource and error contracts

All resource owners have deterministic cleanup on constructor failure and normal destruction. Database appender/connection, model/data, render context/window and plugin instance/library each have one owner. Do not throw from destructors. Finalization errors are surfaced through explicit close/finalize results, preserving the primary execution error. A poisoned runtime never resumes stepping.

Public config/domain errors map to `std::invalid_argument` or a documented derived domain type and stable CLI nonzero exits. Engine errors retain the source fatal/poisoned distinction. IO errors remain distinguishable from invalid physical reference outcomes. No task catches all failures and turns them into a zero-filled result or a physically valid run.

## 11. Implemented recording/application seams — 2026-10-17

T38–T51 now have source implementations. The declarations in their owning headers
are authoritative for current signatures; earlier skeletons in this document
remain design context. The implementation has not been configured, compiled,
linked or executed. Its status is `implemented_unverified`.

| Boundary | Owning header / implementation contract |
|---|---|
| Typed column values | `runtime/encoding_values.hpp` owns move-only `Piece` and `EncodedTable`, with standard-library dependencies only. `runtime/encode.hpp` owns the projector's complete `EncodePlan`. The analyst never includes that engine-dependent projector header. |
| Catalog and storage | `recording/catalog.hpp`, `encoding.hpp`, `database.hpp`, `reader.hpp` own the schema, typed piece validation, writer and immutable reader. `bike_recording_reader` links only IO, platform and DuckDB; runtime-dependent plan construction stays in `bike_recording`. |
| Capture and publication | `recording/capture.hpp`, `contracts.hpp`, `publish.hpp` own capture, episode evidence and the explicit `finalize` operation. Sealing an episode does not publish it. Finalization keeps captured asset bytes separate from input-file identity observed at publication. |
| Async telemetry | `runtime/async_close.hpp` owns the data-only close worker. The runtime owner observes completions in order and retains the only acknowledgment authority. Worker or publication failures are explicit; a lost worker cannot silently produce a successful recording. |
| Replay | `replay/session.hpp` owns `ReplaySession`; `playback.hpp` owns stored-state inspection. `report()` requires completed checked verification; partial or failed progress uses `diagnostics()`. Native checked replay verifies native execution identities and does not forge historical extension identities. |
| Physical metadata | `research/evidence.hpp` exposes the shared compiled-configuration metadata builder, implemented in `research/metadata.cpp`. `cli/physical_capture.hpp` combines it with prepared startup state, captured model/profile bytes and initial-state assets. Recorded profile identity is based on the compiled terrain sample payload. |
| Rendering | `viewer/render_owner.hpp` owns independent model/data. `viewer/render_fields.hpp` centralizes the six mutable model arrays. `viewer/session.hpp` supplies lifecycle, presentation and HUD callbacks; applying a frame never advances the physical runtime. |
| CLI | `cli/arguments.hpp` owns parser contracts; `cli/application.hpp` owns typed request assembly and dispatch. `platform/argv.hpp` is the single checked process-entry argument boundary. `bike-recording` retains its independent analyst target. |
| Batch | `batch/plan.hpp` declares ordered `parse_spec_toml` / `expand_spec`, owned jobs/results and bounded execution. Integer lexemes retain their full signed decimal domain before jobs reach the CLI parser. `platform/process.hpp` owns each child process, its streams and cancellation/reaping. |
| Plotting | `viz/plot.hpp` owns series and report values. PNG has a native raster writer and optional explicit Cairo. Autonomous HTML reports declare `native-html-v1` unless an actual local Plotly resource is supplied. |

Exact file and public-symbol mappings, authored regression sources, compatibility
decisions and outstanding acceptance gates are recorded in
`recording-application-implementation.json` and
`recording-application-review.md`. No source-deletion authorization is implied.
