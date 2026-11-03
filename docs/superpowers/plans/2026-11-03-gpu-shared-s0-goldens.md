# GPU Shared Physics Stage 0 — Goldens, Digest and Recording Schema v2 Implementation Plan

> **Imported into main3 (2026-11-03), not started.** Written in the colab-gpu
> worktree against `research/colab-gpu` `b25120af`, before main3 merged
> passive-joint-damping and colab-gpu, ported kinetic-damping and removed the
> battery energy model (`54ee44f8`). Re-verify every file:line fact against
> main3 before executing. Known differences:
> - `tools/fp32_probe/artifacts/common/model.xml` (with `captured_bootstrap.json`
>   and `hfield_data.bin`) is tracked in main3 (`70f72783`).
> - `docs/superpowers/audits/` does not exist in main3 (`072f0002`); research
>   records live in `docs/research/`.
> - Recording channels `battery_energy_j`, `battery_empty`, `energy_limited`,
>   `electrical_power_w` and energy terms `electrical_work_j`,
>   `electrical_residual_j` no longer exist.
> - Open question Q23 from the planning session (record every per-step model
>   mutation: brake joint friction, motor tendon limit, solver warmstart) was
>   never answered.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Freeze bit-exact 30 s CPU golden episodes (`flat`, `rough_uphill_extreme`) behind an end-to-end digest tool and a `slow` CTest node, and add recording schema v2 (`controls` table, `manifest.precision`) without changing a single physics or forces bit.

**Architecture:**
- A new engine-free `bike-recording-digest` tool (in `bike_recording_reader`) hashes the per-step rows of one episode: `physics`, `forces` and, from schema 2, `controls`. It never hashes run identity, wall-clock or metadata. It is built first, and it records the schema-1 goldens before any recorder change.
- Schema v2 adds a catalog-declared `controls` table and the `precision` lane in the manifest. It is filled from one sampling point in `PhysicalStep::advance_body`, between `apply_forces` and the integrator.
- The digest of the v2 `physics,forces` rows must equal the schema-1 golden (gate B). Then the final three-table goldens are captured, and the CTest node `golden_episode_digest` replays them.

**Tech Stack:** C++23, MuJoCo 3.12 (CPU, f64), DuckDB C API (recording containers), CMake/CTest (`bt::` test harness, `bike_add_reference_test`), CMake `-P` script driver, `bash tools/run_tests.sh`.

**Spec:** `docs/superpowers/plans/2026-11-03-gpu-shared-physics-master.md` — Stage 0. The decisions it implements are D2, D16, D18 and D22, plus the Global Constraints and risk R5. Read it together with this plan.

## Global Constraints

- Python only via `uv` (`uv run ...`). GPU Python uses `.venv-metal` (`uv run --python .venv-metal/bin/python ...` or the existing `tools/colab_gpu` invocation pattern).
- Read `docs/agents/native-engineering.md` before any native change.
- The completion gate per stage is `bash tools/run_tests.sh native`, exit 0, run **once** at the end of the stage. The inner loop uses targeted CTest nodes.
- CPU bit identity (D2) is checked with the end-to-end digest from Stage 0 after **every** task that touches `native/src/writers/`, `native/src/tyre/`, `native/src/contact/`, `native/src/drivetrain/` or `native/src/physics_core/`.
- No golden fixture under `tests/fixtures/golden/` is re-captured or edited.
- Never forge sealed recording containers. New fields come from the recorder, never from post-editing a `run.duckdb`.
- `physics_core/` headers:
  - must compile as MSL, CUDA and host C++;
  - no `std::` containers, no allocation, no exceptions, no virtuals, no recursion;
  - fixed-size arrays sized by `constexpr` capacities;
  - address-space macros from `gpu/common.hpp` (`BIKE_HD`, `BIKE_THREAD`, `BIKE_DEVICE`, `BIKE_CONSTANT`).
- `-ffp-contract=off` stays on for CPU. The core must not depend on FMA contraction anywhere; explicit fma is a toolkit primitive.
- T63 (Python removal): `tools/colab_gpu/**` keeps its recorded exemption (the old plan's Task 1 edit to `2026-10-20-cpp-linux-completion.md`).

Stage-local rules derived from the constraints above:

- All new fixtures go under **new** directories, `tests/fixtures/episode-digest/` and `tests/fixtures/recording-schema1/`. Nothing under `tests/fixtures/golden/` is created, edited or re-captured. This keeps the "no golden fixture under `tests/fixtures/golden/`" rule unambiguous.
- `mujoco` API names with an opening parenthesis — for example `mj_step(`, `mj_setConst(` or `mj_forward(` — must not appear in any new `native/src/**/*.cpp` text, comments included. The contracts sweep rejects them outside `engine_call`. Write `engine::step` / `set_const` in prose instead.
- Spans over engine buffers come from `std::views::counted`, `buffer_copy`, `buffer(...)` or `model_access::readonly_buffer(...)`. Never build a pointer+size `std::span` (clang `-Wunsafe-buffer-usage`).

---

## File structure

| File | Status | Responsibility |
|---|---|---|
| `native/src/recording/digest.hpp` | new | Digest API: `DigestTable`, request/result types, `digest_episode`, `format_digest`, `parse_digest`, `compare_digests` |
| `native/src/recording/digest.cpp` | new | Byte format, per-table streaming, block hashes, text codec, comparison |
| `native/src/cli/recording_digest.hpp` / `.cpp` | new | `recording_digest_command(args, out, err)`: argument parsing, `--expect`, `--against`, exit codes |
| `native/src/cli/recording_digest_main.cpp` | new | `bike-recording-digest` entry point |
| `native/cmake/RecordingApplication.cmake` | modify | Add `digest.cpp` to `bike_recording_reader`; new lib `bike_recording_digest_cli`; exe `bike-recording-digest` in `bin/`, not installed |
| `native/tests/reference/test_recording_digest.cpp` | new | Digest contract test |
| `native/tests/reference/run_golden_episode_digest.cmake` | new | `cmake -P` driver: records the golden rides and captures (`CAPTURE_DIR`) or verifies (`EXPECT_DIR`) their digests |
| `native/tests/reference/GoldenEpisodeDigest.cmake` | new | Registers the Stage 0 reference tests and the `golden_episode_digest` node |
| `native/tests/reference/CMakeLists.txt` | modify | `include(GoldenEpisodeDigest.cmake)` |
| `tests/fixtures/episode-digest/darwin-arm64/*.schema1.digest` | new | Schema-1 goldens (`physics,forces`) captured before any recorder change |
| `tests/fixtures/episode-digest/darwin-arm64/*.digest` | new | Final schema-2 goldens (`physics,forces,controls`) |
| `tests/fixtures/recording-schema1/flat_5ms/` and `flat_5ms.digest` | new | A real sealed schema-1 container plus its digest: the R5 v1 read regression |
| `native/src/recording/catalog_data.inc` | modify | `"schema": 2`; new top-level `"schema_2_tables"` holding `controls` |
| `native/src/recording/catalog.{hpp,cpp}` | modify | `schema_version = 2`; kind `controls`; `tables_for_schema(int)`; `catalog_rows(int)` |
| `native/src/recording/schema.cpp` | modify | `catalog_rows(int version)` |
| `native/src/recording/database.hpp`, `database_validate.cpp` | modify | Version-aware validation; `container_schema_version`; controls rules |
| `native/src/recording/reader.{hpp,cpp}` | modify | Manifest v1/v2; `schema_version()` from the container; `precision()` |
| `native/src/recording/publish.cpp` | modify | Manifest `precision` |
| `native/src/recording/physical_source.cpp` | modify | Emit the `controls` rows |
| `native/src/replay/input.cpp` | modify | `container_schema_version` from the reader instead of the literal 1 |
| `native/src/runtime/control.hpp` | modify | `runtime::StepControls` |
| `native/src/runtime/samples.hpp` | modify | `PhysicalSampleData::controls` |
| `native/src/runtime/step.{hpp,cpp}` | modify | `RawStep::controls`, `SolvedInputs::controls`, `PhysicalStep::sample_controls`, the sampling point |
| `native/src/runtime/accounting.cpp` | modify | Move controls into the published sample; restore it on failure |
| `native/src/drivetrain/transmission.{hpp,cpp}` | modify | `applied_upper_bound()`, `applied_ratio()` |
| `native/tests/reference/test_recording_schema_v2.cpp` | new | Schema v2 storage, validation, manifest and v1 read contract |
| `native/tests/reference/test_recording_controls_capture.cpp` | new | Runtime sampling contract on the welded fixture |
| Existing tests pinning schema 1 (listed in Task 3) | modify | Expectation-only edits |
| `docs/RECORDING.md`, `docs/TESTING.md`, master plan | modify | Schema 2 documentation, the D2 command |

---

## Verified facts (read 2026-11-03 at `b25120af`, working tree as in `git status`)

Recording container and schema 1:
- F1. `schema_version = 1` and `replay_contract_version = 3` are at `native/src/recording/catalog.hpp:16-17`. `all_tables`, `research_tables` and `catalog_rows` are declared at `catalog.hpp:87-88,102`.
- F2. The catalog document embeds `"schema": 1` at `native/src/recording/catalog_data.inc:6`, split into 6 raw-literal parts (`:4`). The last part ends at `:10088-10093` with `"description": "transitions", "source": "research owned history" } ] } )CATALOG",`. `descriptors()` (`catalog.cpp:56-69`) rejects a document whose `"schema"` differs from `schema_version`. It reads only `"tables"` and `"research_tables"`.
- F3. `TableSpec::validate` accepts only the kinds `physics|forces|channels` (`catalog.cpp:126-127`). `FieldSpec::validate` accepts free-form non-empty units, phases from {interval_start, interval, interval_end, control_period, episode, n/a} and measurements from {measured, forwarded, solved, derived, declared, diagnostic} (`catalog.cpp:89-105`).
- F4. `all_tables()` = physical + research (`catalog.cpp:191`). `table_spec` searches `all_tables()` (`:192`). `schema_catalog()` holds the physical tables only (`:189`).
- F5. `catalog_rows()` iterates `all_tables()` (`schema.cpp:37-49`). `create_streams` iterates `all_tables()` (`:64-73`). `create_views` uses `schema_catalog().table_order()` and skips `forces` (`:74-87`).
- F6. `write_database` inserts `recording_meta(schema_version, …)` at `database.cpp:106` and the `field_catalog` from `catalog_rows()` at `:109`. Before appending, the physical batch sink rejects only the four research table names (`:58-65`).
- F7. `validate_container_schema` (`database_validate.cpp:113-140`):
  - it requires table set == metadata + `all_tables()`;
  - it checks `recording_meta.schema_version == schema_version` (`:119-122`);
  - its errors name "schema 1".
  - `validate_declared_catalog` compares against `catalog_rows()` (`:141-152`).
  - The orphan-interval check covers `forces` and `channel_*` (`:160`).
  - `validate_episode_row` is at `:167-213`, with channel coverage at `:177` and the forces width at `:190`.
  - `validate_database` is at `:225-233`.
- F8. `read_manifest` requires the exact keys {schema_version, sealed, run_id, database, file_sha256, row_counts, report_status, report_error} (`reader.cpp:36`) and `schema_version == 1` (`:38`). `Recording::schema_version()` returns the literal `1` (`reader.hpp:35`).
- F9. `publish.cpp:66-68` writes exactly those eight manifest keys.
- F10. `NativePhysicalSource::tables` (`physical_source.cpp:32-78`):
  - per window, it emits `commands_applied` (ordinary runs only) and then every spec of `catalog_.table_order()`;
  - the empty-episode branch is at `:35-37`;
  - its generic fallback is `encode_rows(spec, rows, episode_->generation)` (`:72`).
  - `encode_rows` (`encoding.cpp:289-327`) fills `<key>_present` for optional fields. It null-fills absent optional fields.
- F11. `NativePhysicalSource` is the only `PhysicalSource` implementation (`physical_source.cpp:16`; grep over `native/src`, `native/tests`).
- F12. `replay/input.cpp:65` and `:105` hard-code `container_schema_version` = 1.

Runtime sampling point:
- F13. `PhysicalStep::advance_body` (`step.cpp:1548-1657`):
  - `t`, `q`, `v` prestep copies are at `:1569-1571`;
  - `apply_forces(true, true, …)` is at `:1592`;
  - the existing golden observation copies `d->ctrl` right after it (`golden->ctrl = buffer_copy(d->ctrl, m->nu)`, `:1596`);
  - `prepare_attachment_raw(d)` is at `:1601-1603`;
  - `mass_observations`/`com_observations` are at `:1614-1615`;
  - the warning counters are read at `:1622-1626`;
  - the integrator is `engine::step_skip_fresh` / `engine::step_skip` / `engine::step` (`:1633-1641`);
  - `solve_tail({…, .warning_counts = warning_counts})` is at `:1644-1656`.
- F14. `apply_forces_body` (`step.cpp:1189-1470`):
  - zero-fills `ctrl` (`:1232`);
  - `brake_.apply` writes `dof_frictionloss` (`:1235`);
  - `drive().prepare` is at `:1255`;
  - `drive().commit(tick)` is at `:1358`;
  - the rider writes `ctrl` at `:1416`;
  - cruise writes `ctrl[drive_ctrl_adr_]` at `:1461-1462`.
- F15. `Transmission::commit(TransmissionUpdate&)` (`transmission.cpp:410-430`):
  - writes `wrap_prm` at the `coefficients_` indices;
  - calls `engine::set_const` only when a coefficient changed (`:424`);
  - writes `tendon_range[2*tendon_ .. +1]` (`:426-429`).
- F16. The post-step settlement commit `Transmission::commit(SolvedTransmission&)` is memory-only (`transmission.cpp:561-571`, `transmission.hpp:165-167`). `DrivetrainWriter::settle()` runs post-step (`step.cpp:1701`). Its commit swaps `state_` and touches no model row (`writers/drivetrain.cpp:1364-1390`).
  - Consequence: `Transmission::ratio()` (`transmission.hpp:192`, `state_.ratio`) may differ from the live `wrap_prm` after a settle.
  - The recorded chain ratio must therefore be read from `wrap_prm`.
- F17. The ideal hub transmission is built only when `simplified_ && effort_`. It is geometric only for `"geometric_ideal_mid_drive"`; its tendon is `ideal_mid_drive_freehub` or `geometric_mid_drive_freehub` (`writers/drivetrain.cpp:182-189`).
  - For a non-geometric hub, `coefficients_` holds exactly one wrap index (`transmission.cpp:131`).
  - `DrivetrainWriter::transmission_storage()` returns `ideal_hub_.get()` (`writers/drivetrain.hpp:367-368`).
  - `Stepper::drive()` / `has_drive()` / `model()` / `data()` are at `stepper.hpp:177-178,195,200`.
- F18. `Transmission::state_into` reads `range` and `coefficients` back from the live model rows (`transmission.cpp:580-601`).
- F19. Per-step value types:
  - `RawStep` is at `step.hpp:76-106`;
  - `SolvedInputs` is at `step.hpp:431-458`, with its staged members ending `std::array<int,3> warning_counts{}` at `:447`;
  - `PhysicalSampleData` is at `samples.hpp:58-69`; `samples.hpp:16` includes `control.hpp`.
  - `buffer_copy(const double*, mjtSize)` is in the anonymous namespace of `step.cpp:128-131`.
  - Accounting moves the raw fields into the published sample (`accounting.cpp:738-750`). It moves them back on failure (`:656-670`).
- F20. `samples_binding.cpp:286` (the other `PhysicalSampleData` producer) is nanobind-only (`native/CMakeLists.txt:154`, inside `nanobind_add_module`). The root build refuses Python bindings.

Ride and goldens:
- F21. `bike-ride` (`cli/ride.cpp`):
  - steps = `ceil(duration/dt)` (`:131-133`); 30/0.00125 = 24000.0 exactly, checked with `python3`;
  - terminal reasons include `duration_reached` and `wall_clock_cap` (`:153-154`), where `max_wall_clock_s = 300.` (`legacy/controls.hpp:134`);
  - recordings use `recording_retention = "full"` (`:162`);
  - the headless loop advances in chunks with a 0.02 s wall budget (`:178`);
  - it prints `[bike-ride] recording: <dir>` (`:229`).
- F22. Strict headless runs terminate at the `energy.constraint_work` baseline at t = 0.005 s (`docs/RIDE.md:102-105`). Package smoke therefore uses `--check-policy diagnostic` and qualifies exit 2 only as a `duration_reached` run (`native/tests/package/package_smoke.cmake:59-84`).
- F23. `examples/research/viewer_physics_distributed.toml` has:
  - `drive_mode = "articulated_effort"` (`:5`), `timestep_s = 0.00125` (`:6`);
  - `equilibrium_cache_enabled = true` (`:13`);
  - `transmission_model = "ideal_mid_drive"` (`:36`), `motor_clutch = false` (`:37`).
  - The cache directory resolves to `context.cache_directory`, then `BIKE_SIM_EQUILIBRIUM_CACHE_DIR`, then the temp dir (`initialization/equilibrium_cache.cpp:224-226`).
- F24. `flat` is a built-in preset (`terrain/presets.cpp:10,14`).
- F25. `.gitignore:14-15` ignores `output/*` except `*.md`.

Tests and build:
- F26. `bike_add_reference_test` (`native/tests/reference/CMakeLists.txt:19-37`) names targets `reference.<name>` and labels them `reference;<labels>`. `BIKE_GOLDEN_PLATFORM` = lowercase `${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}` (`:6`). `Phase3Ports.cmake` is included at `:45`.
- F27. `bike_recording_reader` is engine-free: `catalog/coordinates/duckdb/schema/encoding/database_validate/reader/analysis/query/report/export` (`RecordingApplication.cmake:30-47`). `bike_recording` links `bike_app` (`:49-64`). The `bike-recording` exe pattern is at `:205-223`, `bike-research-sweep` at `:224-230`.
- F28. `reference.native_platform_linkage` runs `--help` on every `build/bin` artifact. Its engine link-table checks skip binaries without `libmujoco` (`test_native_platform_linkage.cpp:82-88,109,137`). Package smoke checks only the installed entry points (`package_smoke.cmake:41-46`). The digest tool is not installed.
- F29. Expectations pinning schema 1 or the emitted table set:
  - `migration/test_t43.cpp:19`;
  - `migration/test_t38.cpp:22,25` (vs the 15-table `tests/fixtures/migration/recording/schema.json`);
  - `reference/test_native_platform_replay.cpp:71`;
  - `reference/test_recording_platform_compatibility.cpp:144,358` (`:419-421` reads a fixture JSON and stays unchanged);
  - `reference/test_replay_recording_v3.cpp:22,33`;
  - `reference/test_native_encode_columns.cpp:2398-2402` (label `slow`);
  - `reference/test_native_recording_retention.cpp:326-331` (label `slow`).
- F30. Test support (`native/tests/reference/recording_test_support.hpp`):
  - `sample_row` (`:79`), `make_evidence` (`:303`), `with_forces` (`:366`), `synthetic_capture` (`:432`);
  - `sealed` (`:464`), `mutate_database` (`:470`), `welded_runtime(retention, decimation, strict)` (`:509`), `advance` (`:558`), `repo_file` (`:564`), `throws<Ex>` (`:608`);
  - `run_metadata()` has a fixed UUID and start time (`:257-270`).
  - `RuntimeRigAccess::inspect(NativeRideRuntime&, std::function<void(const Stepper&, const PhysicalStep&)>)` is in `bike_validation` (`validation/runtime_access.hpp:17`, `cmake/NativeValidation.cmake:3,10`).
- F31. `io::tagged_double` maps `Number` lexemes and the `float64` sentinels `nan/+inf/-inf` to doubles (`io/value.hpp:427-444`). `io::Sha256::{update,finish}` is at `io/hash.hpp:17-20`; `io::sha256(span)` returns hex (`:34`). `Cursor::next_batch` is at `recording/duckdb.hpp:27`.
- F32. The sweep kinds are `frontends tidy analyzer cppcheck odr scripts headers diagnostic-controls context` plus `contracts` (`tools/run_native_checks.sh:32-38`). The presets set `NATIVE_SANITIZE/NATIVE_RTSAN/NATIVE_COVERAGE` (`CMakePresets.json:36-38`, options at `BikeCompilerPolicy.cmake:52-54`).
- F33. The existing committed sealed containers under `tests/fixtures/migration/recording/{good,value_cases,…}` (≈10 MB `run.duckdb`) are not opened by any C++ test (grep `native/tests`). They are Python-era evidence whose `field_catalog` may predate the current catalog.

## Ordering argument: where `controls` is sampled

The `controls` row of interval *k* is sampled in `PhysicalStep::advance_body`. The point is after `apply_forces(true, true, …)` returns (F13, `step.cpp:1592`), right next to the existing `golden->ctrl` copy (`:1596`), and before the integrator call (`:1633-1641`).

1. Every controller write the integrator consumes happens inside `apply_forces_body` (F14):
   - the `ctrl` zero-fill;
   - the brake `dof_frictionloss` bounds;
   - the drivetrain prepare and commit, which writes the hub `wrap_prm`, calls `set_const` when needed and writes `tendon_range` (F15), plus the motor `ctrl`;
   - the rider actuator `ctrl`;
   - the cruise `ctrl`.
2. Between that return and the integrator, only these run:
   - `prepare_attachment_raw` (const `mjData*` read);
   - `mass_observations` / `com_observations`, which write only derived subtree quantities;
   - the `last_force_sample_` / `components` copies;
   - the warning-counter reads;
   - `acc_inputs_unchanged`, a read.
   None of them writes `ctrl`, `wrap_prm` or `tendon_range`.
3. The integrator does not write `ctrl`, `wrap_prm` or `tendon_range`. After the step, `DrivetrainWriter::settle()` commits memory-only state (F16). The next interval's `apply_forces` re-derives everything. So the live values after the step still equal what the integrator read. Task 4's runtime test checks this independently.
4. Values recorded:
   - `ctrl = d->ctrl[0..nu)`;
   - `freehub_upper_rad = m->tendon_range[2*tendon_+1]` of the ideal hub;
   - `chain_ratio = m->wrap_prm[coefficients_[0]]` for a non-geometric hub.
   These are the model rows — never `Transmission::ratio()`, which can drift from the model after a settle (F16).
5. The Stage 2 replay contract that follows from this:
   - write `wrap_prm`, and call `set_const` only if it changed, exactly like F15;
   - write `tendon_range[…+1]`;
   - write `ctrl`;
   - then integrate.

Two independent checks support this placement, as native-engineering.md §Establish the contract requires:
- the source reading above;
- Task 4's runtime test, which compares each sample bitwise with the live model after the step through `Transmission::state()`. `state()` reads the model rows independently of the new accessors (F18).

The existing `GoldenForceObservation::ctrl` copy at the same point is corroborating precedent, not a third proof.

Not recorded by v2: the brake `dof_frictionloss`, `qacc_warmstart`, `act`, the clutch/freewheel tendons, and the `set_const` side effects. These are master-plan R4 candidates, listed under Open risks.

---

## Task 1: `bike-recording-digest` tool

**Files:**
- Create: `native/src/recording/digest.hpp`, `native/src/recording/digest.cpp`
- Create: `native/src/cli/recording_digest.hpp`, `native/src/cli/recording_digest.cpp`, `native/src/cli/recording_digest_main.cpp`
- Create: `native/tests/reference/test_recording_digest.cpp`, `native/tests/reference/GoldenEpisodeDigest.cmake`
- Modify: `native/cmake/RecordingApplication.cmake`, `native/tests/reference/CMakeLists.txt`

**Interfaces:**
- Consumes: `bike::recording::Recording` (`stream`, `rows`, `episode_ids`, `schema_version`), `io::Sha256`, `io::tagged_double`, `io::parse_recording_json`.
- Produces:
  - `bike::recording::{DigestTable, DigestRequest, DigestBlock, EpisodeDigest, DigestComparison}`;
  - `digest_table_name`, `parse_digest_tables`, `default_digest_tables`, `digest_episode`, `format_digest`, `parse_digest`, `compare_digests`;
  - `bike::cli::recording_digest_command`;
  - executable `build/native-release/bin/bike-recording-digest`;
  - CTest `reference.recording_digest`.

**Byte format (pinned by the test):**
- The whole-episode sha256 covers:
  - the ASCII magic `bike-recording-digest/1\n`;
  - then, per requested table in order, the ASCII header `table <name>\n` followed by that table's records.
- Records are little-endian. `i64`/`u64` are 8 bytes; `f64` is the IEEE-754 bit pattern, with NaN canonicalized by the reader to the quiet NaN.
  - `physics`: i64 interval_id, f64 time_s, f64 interval_end_s, f64 dt_s, u64 n + n×f64 qpos, u64 n + n×f64 qvel.
  - `forces`, ordered by `interval_id, component`: i64 interval_id, u64 len + bytes component, u64 n + n×f64 qfrc, f64 power_w.
- A block hash is the sha256 of the records alone (no magic, no header) of one table for intervals `[first + b·block_steps, first + (b+1)·block_steps)`.

- [ ] **Step 1: Write the failing test**

Create `native/tests/reference/test_recording_digest.cpp`:

```cpp
// GPU shared physics Stage 0 — bike-recording-digest contract
// (docs/superpowers/plans/2026-11-03-gpu-shared-s0-goldens.md, Task 1).
// The D2/D18 end-to-end digest hashes only per-step physical rows: it is
// blind to run identity and wall clock, its byte format is pinned here,
// and a mismatch names the first differing block/interval and table.
#include "recording_test_support.hpp"
#include "cli/recording_digest.hpp"
#include "io/hash.hpp"
#include "recording/digest.hpp"
#include "recording/publish.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace recording_test;

// Little-endian twin of the digest byte format, written independently of
// recording/digest.cpp so a silent format change fails this test.
struct Bytes {
    std::vector<std::byte> bytes;
    void u64(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) { bytes.push_back(static_cast<std::byte>(v & 0xffU)); v >>= 8U; }
    }
    void i64(std::int64_t v) { u64(std::bit_cast<std::uint64_t>(v)); }
    void f64(double v) { u64(std::bit_cast<std::uint64_t>(v)); }
    void f64s(const std::vector<double>& values) { u64(values.size()); for (const double v : values) f64(v); }
    void raw(std::string_view text) { for (const char c : text) bytes.push_back(static_cast<std::byte>(c)); }
    void append(const Bytes& other) { bytes.insert(bytes.end(), other.bytes.begin(), other.bytes.end()); }
};

constexpr double dt = 1. / 8;  // exact binary clock: interval_end_s == next time_s

std::vector<SamplePtr> forced_rows(std::size_t count, std::optional<std::size_t> nudged = std::nullopt) {
    std::vector<SamplePtr> out;
    for (std::size_t i = 0; i < count; ++i) {
        std::vector<double> qpos(8, 0.), qvel(8, 0.);
        qpos.front() = .25 * static_cast<double>(i);
        if (nudged && *nudged == i) qpos.front() = std::nextafter(qpos.front(), 1e9);
        qvel.front() = -.5 * static_cast<double>(i);
        const auto row = sample_row(static_cast<std::int64_t>(i), static_cast<double>(i) * dt, dt, std::move(qpos), std::move(qvel));
        out.push_back(with_forces(*row));
    }
    return out;
}

rec::RunMetadata other_run() {
    auto metadata = run_metadata();
    metadata.run_uuid = "00000000-0000-4000-8000-0000000000ff";
    metadata.started_utc = "2026-06-30T12:34:56+00:00";
    metadata.wall_origin_s = 99.25;
    return metadata;
}

std::filesystem::path publish(const std::filesystem::path& root, std::vector<SamplePtr> rows, rec::RunMetadata metadata) {
    auto capture = std::make_unique<rec::RunCapture>(std::move(metadata), true);
    capture->add_episode(make_evidence(0, 1, std::move(rows)));
    return rec::finalize(*capture, root).directory;
}

rec::DigestRequest request(std::vector<rec::DigestTable> tables, std::int64_t block_steps = 400) {
    rec::DigestRequest out;
    out.tables = std::move(tables);
    out.block_steps = block_steps;
    return out;
}

void test_identity_and_wall_clock_are_excluded() {
    bt::TemporaryDirectory dir("digest-identity");
    const auto left = publish(dir.path() / "left", forced_rows(4), run_metadata());
    const auto right = publish(dir.path() / "right", forced_rows(4), other_run());
    const auto a = rec::digest_episode(*sealed(left), {});
    const auto b = rec::digest_episode(*sealed(right), {});
    bt::require(a.tables == rec::default_digest_tables(a.schema_version), "an empty request digests the schema's default tables");
    bt::require(a.sha256 == b.sha256 && a.blocks == b.blocks && rec::compare_digests(a, b).matches,
        "run uuid, start time, wall origin and directory name never enter the digest");
    bt::require(a.intervals == 4 && a.first_interval == 0 && a.outcome == "synthetic" && a.episode == 0,
        "the digest records the episode extent and outcome reason");
}

void test_byte_format_is_pinned() {
    bt::TemporaryDirectory dir("digest-format");
    std::vector<double> qpos(8, 0.), qvel(8, 0.);
    qpos.front() = 1.5;
    qvel.back() = -0.;
    const auto path = publish(dir.path() / "run", {sample_row(0, 0., dt, qpos, qvel)}, run_metadata());
    const auto digest = rec::digest_episode(*sealed(path), request({rec::DigestTable::physics}));
    Bytes record;
    record.i64(0); record.f64(0.); record.f64(dt); record.f64(dt); record.f64s(qpos); record.f64s(qvel);
    Bytes whole;
    whole.raw("bike-recording-digest/1\n"); whole.raw("table physics\n"); whole.append(record);
    bt::require(digest.sha256 == bike::io::sha256(whole.bytes),
        "the episode hash is sha256 over magic, table header and little-endian records (signed zero kept)");
    bt::require(digest.blocks.size() == 1 && digest.blocks.front().sha256 == bike::io::sha256(record.bytes),
        "a block hash covers only its own records");
}

void test_mismatch_names_first_block_and_table() {
    bt::TemporaryDirectory dir("digest-mismatch");
    const auto left = publish(dir.path() / "left", forced_rows(6), run_metadata());
    const auto right = publish(dir.path() / "right", forced_rows(6, 4), run_metadata());
    const auto tables = std::vector{rec::DigestTable::physics, rec::DigestTable::forces};
    const auto blocks = rec::compare_digests(rec::digest_episode(*sealed(left), request(tables, 2)),
                                             rec::digest_episode(*sealed(right), request(tables, 2)));
    std::string text;
    for (const auto& line : blocks.differences) text += line + '\n';
    bt::require(!blocks.matches && text.find("first differing block 2 (intervals [4, 6)) in physics") != std::string::npos &&
        text.find("forces") == std::string::npos, "a one-ulp qpos change names block 2 of physics only: " + text);
    const auto steps = rec::compare_digests(rec::digest_episode(*sealed(left), request(tables, 1)),
                                            rec::digest_episode(*sealed(right), request(tables, 1)));
    text.clear();
    for (const auto& line : steps.differences) text += line + '\n';
    bt::require(text.find("first differing interval 4 in physics") != std::string::npos,
        "per-interval blocks name the exact first differing step: " + text);
}

void test_text_codec_round_trips_and_rejects() {
    bt::TemporaryDirectory dir("digest-codec");
    const auto path = publish(dir.path() / "run", forced_rows(3), run_metadata());
    const auto digest = rec::digest_episode(*sealed(path), request({rec::DigestTable::physics, rec::DigestTable::forces}, 2));
    const auto text = rec::format_digest(digest);
    const auto back = rec::parse_digest(text);
    bt::require(rec::format_digest(back) == text && rec::compare_digests(digest, back).matches, "format/parse round trip");
    bt::invalid([] { (void)rec::parse_digest("bike-recording-digest 2\n"); });
    auto bad_hex = text;
    bad_hex.replace(bad_hex.find("sha256 ") + 7, 1, "G");
    bt::invalid([&] { (void)rec::parse_digest(bad_hex); });
    bt::invalid([&] { (void)rec::parse_digest(text.substr(0, text.size() - 1)); });
    bt::require(rec::parse_digest_tables("forces,physics") == std::vector{rec::DigestTable::forces, rec::DigestTable::physics},
        "table order is the caller's order");
    for (const auto* bad : {"", "physics,,forces", "physics,physics", "bogus"})
        bt::invalid([&] { (void)rec::parse_digest_tables(bad); });
}

int run_cli(std::initializer_list<std::string_view> args, std::string& out, std::string& err) {
    std::ostringstream o, e;
    const std::vector<std::string_view> arguments(args);
    const int rc = bike::cli::recording_digest_command(arguments, o, e);
    out = o.str(); err = e.str();
    return rc;
}

void test_cli_exit_codes() {
    bt::TemporaryDirectory dir("digest-cli");
    const auto left = publish(dir.path() / "left", forced_rows(6), run_metadata()).string();
    const auto right = publish(dir.path() / "right", forced_rows(6, 4), run_metadata()).string();
    const auto stored = (dir.path() / "left.digest").string();
    std::string out, err;
    bt::require(run_cli({left}, out, err) == 0 && out.starts_with("bike-recording-digest 1\n"), "print mode exits 0: " + err);
    { std::ofstream file(stored); file << out; }
    bt::require(run_cli({left, "--expect", stored}, out, err) == 0 && out.starts_with("match "), "a matching --expect exits 0: " + err);
    bt::require(run_cli({right, "--expect", stored}, out, err) == 1 && err.find("first differing block 0") != std::string::npos,
        "a mismatching --expect exits 1 and names the block: " + err);
    bt::require(run_cli({right, "--against", left}, out, err) == 1 && err.find("first differing interval 4 in physics") != std::string::npos,
        "--against names the first differing interval and table: " + err);
    bt::require(run_cli({"--help"}, out, err) == 0 && out.starts_with("usage: bike-recording-digest"), "--help exits 0");
    bt::require(run_cli({left, "--tables", "physics,bogus"}, out, err) == 2, "an unknown table is a usage error");
    bt::require(run_cli({left, "--expect", stored, "--against", left}, out, err) == 2, "--expect and --against are exclusive");
    bt::require(run_cli({}, out, err) == 2, "RECORDING is required");
    bt::require(run_cli({(dir.path() / "missing").string()}, out, err) == 2, "an unreadable recording is an input error");
}
} // namespace

int main() {
    return bt::run([] {
        test_identity_and_wall_clock_are_excluded();
        test_byte_format_is_pinned();
        test_mismatch_names_first_block_and_table();
        test_text_codec_round_trips_and_rejects();
        test_cli_exit_codes();
    });
}
```

`run_cli` takes `std::initializer_list<std::string_view>`. The paths are therefore kept in named `std::string` locals (`left`, `right`, `stored`) so that the views never dangle.

Create `native/tests/reference/GoldenEpisodeDigest.cmake`:

```cmake
# GPU shared physics Stage 0 (docs/superpowers/plans/2026-11-03-gpu-shared-s0-goldens.md):
# the bike-recording-digest contract, the schema-2 recording contracts and
# the D2/D18 golden episode digest gate.
bike_add_reference_test(recording_digest ENGINE
  SOURCES test_recording_digest.cpp
  LIBRARIES bike_recording bike_recording_digest_cli
  LABELS recording golden)
```

In `native/tests/reference/CMakeLists.txt`, add one line after `include("${CMAKE_CURRENT_SOURCE_DIR}/Phase3Ports.cmake")` (line 45):

```cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/GoldenEpisodeDigest.cmake")
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cmake --preset release
cmake --build build/native-release --target reference.recording_digest
```

Expected: the configure fails with an error naming the unknown link target `bike_recording_digest_cli`, or the build fails with `fatal error: 'recording/digest.hpp' file not found`. Either is the expected red.

- [ ] **Step 3: Write the digest library**

Create `native/src/recording/digest.hpp`:

```cpp
// recording/digest.hpp — the D2/D18 end-to-end episode digest (GPU shared
// physics Stage 0). Hashes the per-step physical rows of one episode and
// nothing else: run identity, wall clock, metadata, events, assets and
// manifest hashes are excluded so two runs of the same physics agree.
#pragma once
#include "reader.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace bike::recording {
enum class DigestTable { physics, forces };
[[nodiscard]] std::string_view digest_table_name(DigestTable table);
// Comma-separated names, caller order kept; empty, unknown and duplicate
// names are rejected.
[[nodiscard]] std::vector<DigestTable> parse_digest_tables(std::string_view list);
[[nodiscard]] std::vector<DigestTable> default_digest_tables(std::int64_t container_schema);
struct DigestRequest {
    std::vector<DigestTable> tables;    // empty = default_digest_tables(schema)
    std::optional<std::int64_t> episode; // empty = the only episode
    std::int64_t block_steps = 400;
};
struct DigestBlock {
    std::int64_t index = 0;
    DigestTable table = DigestTable::physics;
    std::string sha256;
    friend bool operator==(const DigestBlock&, const DigestBlock&) = default;
};
struct EpisodeDigest {
    std::int64_t schema_version = 0, episode = 0, block_steps = 0, first_interval = 0, intervals = 0;
    std::vector<DigestTable> tables;
    std::string outcome, sha256;
    std::vector<DigestBlock> blocks;  // ordered by index, then by `tables` order
};
[[nodiscard]] EpisodeDigest digest_episode(const Recording& recording, const DigestRequest& request);
[[nodiscard]] std::string format_digest(const EpisodeDigest& digest);
[[nodiscard]] EpisodeDigest parse_digest(std::string_view text);
struct DigestComparison {
    bool matches = false;
    std::vector<std::string> differences;
};
// `schema_version` is deliberately not compared: gate B checks a schema-2
// recording's physics,forces rows against the schema-1 golden.
[[nodiscard]] DigestComparison compare_digests(const EpisodeDigest& expected, const EpisodeDigest& actual);
} // namespace bike::recording
```

Create `native/src/recording/digest.cpp`:

```cpp
#include "digest.hpp"
#include "value_util.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <span>
#include <utility>

namespace bike::recording {
namespace {
using io::Value;
using Object = Value::Object;

constexpr std::string_view magic = "bike-recording-digest/1\n";
constexpr std::array<std::pair<DigestTable, std::string_view>, 2> table_names{{
    {DigestTable::physics, "physics"}, {DigestTable::forces, "forces"}}};

std::string to_hex(const std::array<std::uint8_t, 32>& digest) {
    static constexpr std::string_view digits = "0123456789abcdef";
    std::string out;
    out.reserve(64);
    for (const auto byte : digest) {
        const auto value = static_cast<std::size_t>(byte);
        out.push_back(digits[value >> 4U]);
        out.push_back(digits[value & 0x0fU]);
    }
    return out;
}
void update_text(io::Sha256& sha, std::string_view text) noexcept { sha.update(std::as_bytes(std::span<const char>(text))); }

// Feeds the whole-episode hash and the open block's hash with one byte
// stream, so a block hash is exactly the records it covers.
class Stream {
public:
    explicit Stream(io::Sha256* whole) noexcept : whole_(whole) {}
    void u8(std::uint8_t v) noexcept { const std::array<std::byte, 1> b{static_cast<std::byte>(v)}; update(b); }
    void u64(std::uint64_t v) noexcept {
        std::array<std::byte, 8> bytes{};
        for (auto& b : bytes) { b = static_cast<std::byte>(v & 0xffU); v >>= 8U; }
        update(bytes);
    }
    void i64(std::int64_t v) noexcept { u64(std::bit_cast<std::uint64_t>(v)); }
    void f64(double v) noexcept { u64(std::bit_cast<std::uint64_t>(v)); }
    void text(std::string_view v) noexcept { u64(v.size()); update(std::as_bytes(std::span<const char>(v))); }
    void f64s(const Value::Array& values, std::string_view field) {
        u64(values.size());
        for (const auto& v : values) f64(io::tagged_double(v, field));
    }
    [[nodiscard]] std::string finish_block() noexcept {
        auto hex = to_hex(block_.finish());
        block_ = io::Sha256{};
        return hex;
    }
private:
    void update(std::span<const std::byte> bytes) noexcept { whole_->update(bytes); block_.update(bytes); }
    io::Sha256* whole_;
    io::Sha256 block_;
};

const Value& cell(const Object& row, std::string_view name) {
    if (const auto* v = io::find_member(row, name)) return *v;
    throw RecordingError("digest query lacks column " + std::string(name));
}
std::int64_t integer_cell(const Object& row, std::string_view name) { return io::as_int64(cell(row, name), name); }
double real_cell(const Object& row, std::string_view name) { return io::tagged_double(cell(row, name), name); }

std::string_view table_query(DigestTable table) {
    switch (table) {
    case DigestTable::physics:
        return "SELECT interval_id,time_s,interval_end_s,dt_s,qpos,qvel FROM physics WHERE episode_id=? ORDER BY interval_id";
    case DigestTable::forces:
        return "SELECT interval_id,component,qfrc,power_w FROM forces WHERE episode_id=? ORDER BY interval_id,component";
    }
    throw std::invalid_argument("unknown digest table");
}
void record(DigestTable table, Stream& stream, const Object& row) {
    stream.i64(integer_cell(row, "interval_id"));
    switch (table) {
    case DigestTable::physics:
        for (const std::string_view name : {"time_s", "interval_end_s", "dt_s"}) stream.f64(real_cell(row, name));
        for (const std::string_view name : {"qpos", "qvel"}) stream.f64s(io::as_array(cell(row, name), name), name);
        return;
    case DigestTable::forces:
        stream.text(io::as_string(cell(row, "component"), "component"));
        stream.f64s(io::as_array(cell(row, "qfrc"), "qfrc"), "qfrc");
        stream.f64(real_cell(row, "power_w"));
        return;
    }
    throw std::invalid_argument("unknown digest table");
}
void walk(const Recording& recording, DigestTable table, const EpisodeDigest& extent, io::Sha256& whole, std::vector<DigestBlock>& blocks) {
    update_text(whole, "table ");
    update_text(whole, digest_table_name(table));
    update_text(whole, "\n");
    Stream stream(&whole);
    std::optional<std::int64_t> open;
    const auto flush = [&] { if (open) blocks.push_back({.index = *open, .table = table, .sha256 = stream.finish_block()}); };
    const std::array params{value::integer(extent.episode)};
    auto cursor = recording.stream(table_query(table), params);
    for (;;) {
        const auto batch = cursor.next_batch(4096);
        if (batch.empty()) break;
        for (const auto& row : batch) {
            const auto interval = integer_cell(row, "interval_id");
            if (interval < extent.first_interval) throw RecordingError(std::string(digest_table_name(table)) + " row precedes the physics extent");
            const auto index = (interval - extent.first_interval) / extent.block_steps;
            if (open != index) { flush(); open = index; }
            record(table, stream, row);
        }
    }
    flush();
}

std::int64_t parse_integer(std::string_view text, std::string_view what) {
    const std::string owned(text);
    std::size_t used = 0;
    const std::int64_t value = std::stoll(owned, &used, 10);
    if (used != owned.size()) throw RecordingError(std::string(what) + " must be an integer");
    return value;
}
bool is_hex64(std::string_view text) {
    return text.size() == 64 && std::ranges::all_of(text, [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}
class Lines {
public:
    explicit Lines(std::string_view text) : rest_(text) {}
    [[nodiscard]] bool done() const noexcept { return rest_.empty(); }
    std::string_view value(std::string_view key) {
        const auto end = rest_.find('\n');
        if (end == std::string_view::npos) throw RecordingError("digest text must end every line with a newline");
        const auto line = rest_.substr(0, end);
        rest_.remove_prefix(end + 1);
        if (line.size() <= key.size() || !line.starts_with(key) || line[key.size()] != ' ')
            throw RecordingError("digest line '" + std::string(key) + "' expected, got '" + std::string(line) + "'");
        return line.substr(key.size() + 1);
    }
private:
    std::string_view rest_;
};
std::string join_tables(const std::vector<DigestTable>& tables) {
    std::string out;
    for (const auto table : tables) { if (!out.empty()) out += ','; out += digest_table_name(table); }
    return out;
}
} // namespace

std::string_view digest_table_name(DigestTable table) {
    for (const auto& [id, name] : table_names) if (id == table) return name;
    throw std::invalid_argument("unknown digest table");
}
std::vector<DigestTable> parse_digest_tables(std::string_view list) {
    std::vector<DigestTable> out;
    for (;;) {
        const auto comma = list.find(',');
        const auto name = list.substr(0, comma);
        const auto found = std::ranges::find_if(table_names, [&](const auto& entry) { return entry.second == name; });
        if (found == table_names.end()) throw RecordingError("unknown digest table '" + std::string(name) + "'");
        if (std::ranges::find(out, found->first) != out.end()) throw RecordingError("duplicate digest table '" + std::string(name) + "'");
        out.push_back(found->first);
        if (comma == std::string_view::npos) return out;
        list.remove_prefix(comma + 1);
    }
}
std::vector<DigestTable> default_digest_tables(std::int64_t container_schema) {
    if (container_schema == 1) return {DigestTable::physics, DigestTable::forces};
    throw RecordingError("no digest defaults for container schema " + std::to_string(container_schema));
}

EpisodeDigest digest_episode(const Recording& recording, const DigestRequest& request) {
    if (request.block_steps <= 0) throw RecordingError("block_steps must be positive");
    EpisodeDigest out;
    out.schema_version = recording.schema_version();
    out.tables = request.tables.empty() ? default_digest_tables(out.schema_version) : request.tables;
    out.block_steps = request.block_steps;
    const auto ids = recording.episode_ids();
    if (request.episode) {
        if (std::ranges::find(ids, *request.episode) == ids.end()) throw RecordingError("no episode " + std::to_string(*request.episode));
        out.episode = *request.episode;
    } else if (ids.size() == 1) {
        out.episode = ids.front();
    } else {
        throw RecordingError("recording has " + std::to_string(ids.size()) + " episodes; pass --episode");
    }
    const std::array params{value::integer(out.episode)};
    const auto extent = recording.rows("SELECT count(*) AS n, coalesce(min(interval_id), 0) AS first_interval FROM physics WHERE episode_id=?", params);
    out.intervals = integer_cell(extent.at(0), "n");
    out.first_interval = integer_cell(extent.at(0), "first_interval");
    const auto summary = recording.rows("SELECT summary_json FROM episodes WHERE episode_id=?", params);
    const auto parsed = io::parse_recording_json(io::as_string(cell(summary.at(0), "summary_json"), "summary_json"));
    out.outcome = "-";
    if (const auto* outcome = io::find_member(io::as_object(parsed, "summary"), "outcome"))
        if (const auto* reason = io::find_member(io::as_object(*outcome, "outcome"), "reason")) out.outcome = io::as_string(*reason, "outcome.reason");
    io::Sha256 whole;
    update_text(whole, magic);
    for (const auto table : out.tables) walk(recording, table, out, whole, out.blocks);
    out.sha256 = to_hex(whole.finish());
    std::ranges::stable_sort(out.blocks, {}, &DigestBlock::index);
    return out;
}

std::string format_digest(const EpisodeDigest& d) {
    std::string out = "bike-recording-digest 1\n";
    out += "schema " + std::to_string(d.schema_version) + "\n";
    out += "episode " + std::to_string(d.episode) + "\n";
    out += "tables " + join_tables(d.tables) + "\n";
    out += "block_steps " + std::to_string(d.block_steps) + "\n";
    out += "first_interval " + std::to_string(d.first_interval) + "\n";
    out += "intervals " + std::to_string(d.intervals) + "\n";
    out += "outcome " + d.outcome + "\n";
    out += "sha256 " + d.sha256 + "\n";
    for (const auto& b : d.blocks) out += "block " + std::to_string(b.index) + " " + std::string(digest_table_name(b.table)) + " " + b.sha256 + "\n";
    return out;
}

EpisodeDigest parse_digest(std::string_view text) {
    Lines lines(text);
    if (lines.value("bike-recording-digest") != "1") throw RecordingError("unsupported digest format version");
    EpisodeDigest out;
    out.schema_version = parse_integer(lines.value("schema"), "schema");
    out.episode = parse_integer(lines.value("episode"), "episode");
    out.tables = parse_digest_tables(lines.value("tables"));
    out.block_steps = parse_integer(lines.value("block_steps"), "block_steps");
    out.first_interval = parse_integer(lines.value("first_interval"), "first_interval");
    out.intervals = parse_integer(lines.value("intervals"), "intervals");
    out.outcome = std::string(lines.value("outcome"));
    out.sha256 = std::string(lines.value("sha256"));
    if (!is_hex64(out.sha256) || out.block_steps <= 0) throw RecordingError("malformed digest header");
    while (!lines.done()) {
        const auto rest = lines.value("block");
        const auto first = rest.find(' ');
        const auto second = first == std::string_view::npos ? first : rest.find(' ', first + 1);
        if (second == std::string_view::npos) throw RecordingError("malformed digest block line");
        DigestBlock block;
        block.index = parse_integer(rest.substr(0, first), "block index");
        const auto table = parse_digest_tables(rest.substr(first + 1, second - first - 1));
        block.table = table.front();
        block.sha256 = std::string(rest.substr(second + 1));
        if (table.size() != 1 || !is_hex64(block.sha256)) throw RecordingError("malformed digest block line");
        out.blocks.push_back(std::move(block));
    }
    return out;
}

DigestComparison compare_digests(const EpisodeDigest& expected, const EpisodeDigest& actual) {
    DigestComparison out;
    const auto differ = [&](std::string_view what, const std::string& want, const std::string& got) {
        if (want != got) out.differences.push_back(std::string(what) + ": expected " + want + ", got " + got);
    };
    differ("tables", join_tables(expected.tables), join_tables(actual.tables));
    differ("block_steps", std::to_string(expected.block_steps), std::to_string(actual.block_steps));
    differ("episode", std::to_string(expected.episode), std::to_string(actual.episode));
    differ("first_interval", std::to_string(expected.first_interval), std::to_string(actual.first_interval));
    differ("intervals", std::to_string(expected.intervals), std::to_string(actual.intervals));
    differ("outcome", expected.outcome, actual.outcome);
    differ("sha256", expected.sha256, actual.sha256);
    if (expected.sha256 != actual.sha256 && expected.tables == actual.tables && expected.block_steps == actual.block_steps) {
        const auto size = std::max(expected.blocks.size(), actual.blocks.size());
        for (std::size_t i = 0; i < size; ++i) {
            const bool same = i < expected.blocks.size() && i < actual.blocks.size() && expected.blocks.at(i) == actual.blocks.at(i);
            if (same) continue;
            const auto& block = i < expected.blocks.size() ? expected.blocks.at(i) : actual.blocks.at(i);
            const auto start = expected.first_interval + block.index * expected.block_steps;
            std::string tables;
            for (std::size_t j = i; j < size; ++j) {
                const auto& probe = j < expected.blocks.size() ? expected.blocks.at(j) : actual.blocks.at(j);
                if (probe.index != block.index) break;
                const bool equal_probe = j < expected.blocks.size() && j < actual.blocks.size() && expected.blocks.at(j) == actual.blocks.at(j);
                if (!equal_probe) { if (!tables.empty()) tables += ','; tables += digest_table_name(probe.table); }
            }
            out.differences.push_back(expected.block_steps == 1
                ? "first differing interval " + std::to_string(start) + " in " + tables
                : "first differing block " + std::to_string(block.index) + " (intervals [" + std::to_string(start) + ", " +
                  std::to_string(start + expected.block_steps) + ")) in " + tables);
            break;
        }
    }
    out.matches = out.differences.empty();
    return out;
}
} // namespace bike::recording
```

Stop condition: the block walk aligns blocks by vector position. That is valid only when both digests have the same block sequence up to the first difference. If the extents differ (an `intervals` difference is reported), the reported block is the first position where the two sequences disagree. That is still the first differing block, so no extra handling is required. Do not "improve" this into a keyed merge without a failing test.

Create `native/src/cli/recording_digest.hpp`:

```cpp
#pragma once
#include <iosfwd>
#include <span>
#include <string_view>
namespace bike::cli {
// bike-recording-digest: exit 0 printed/match, 1 mismatch, 2 usage or input error.
int recording_digest_command(std::span<const std::string_view> arguments, std::ostream& out, std::ostream& error);
} // namespace bike::cli
```

Create `native/src/cli/recording_digest.cpp`:

```cpp
#include "recording_digest.hpp"
#include "../recording/digest.hpp"
#include "../recording/reader.hpp"
#include <filesystem>
#include <fstream>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace bike::cli {
namespace {
constexpr std::string_view usage =
    "usage: bike-recording-digest RECORDING [--tables physics,forces[,controls]] [--episode N]\n"
    "                             [--block-steps N] [--expect DIGEST_FILE | --against RECORDING]\n\n"
    "Prints a sha256 over the per-step physics, forces and (schema 2) controls rows of one\n"
    "episode; run identity, wall clock and metadata are excluded. --expect compares with a stored\n"
    "digest (block resolution, tables/block size taken from the file unless given); --against\n"
    "digests both recordings per interval and names the first differing interval and table.\n"
    "Exit 0 printed or match, 1 mismatch, 2 usage or input error.\n";

std::int64_t parse_integer(const std::string& text, std::string_view option) {
    std::size_t used = 0;
    const std::int64_t value = std::stoll(text, &used, 10);
    if (used != text.size()) throw std::invalid_argument(std::string(option) + " must be an integer");
    return value;
}
std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::invalid_argument("cannot read digest file " + path);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}
int report(const recording::DigestComparison& comparison, std::ostream& error) {
    for (const auto& line : comparison.differences) error << "bike-recording-digest: " << line << '\n';
    error << "bike-recording-digest: inspect with bike-recording compare GOLDEN ACTUAL --left-episode 0 "
             "--right-episode 0 --mode exact --atol 0 --rtol 0\n";
    return 1;
}
} // namespace

int recording_digest_command(std::span<const std::string_view> arguments, std::ostream& out, std::ostream& error) {
    try {
        std::optional<std::string> recording_path, tables, expect, against, episode, block_steps;
        for (std::size_t i = 0; i < arguments.size(); ++i) {
            const auto argument = arguments[i];
            if (argument == "-h" || argument == "--help") { out << usage; return 0; }
            const auto value = [&](std::optional<std::string>& slot) {
                if (slot) throw std::invalid_argument(std::string(argument) + " given twice");
                if (i + 1 >= arguments.size()) throw std::invalid_argument(std::string(argument) + " requires a value");
                slot = std::string(arguments[++i]);
            };
            if (argument == "--tables") value(tables);
            else if (argument == "--expect") value(expect);
            else if (argument == "--against") value(against);
            else if (argument == "--episode") value(episode);
            else if (argument == "--block-steps") value(block_steps);
            else if (argument.starts_with("-")) throw std::invalid_argument("unknown option " + std::string(argument));
            else if (recording_path) throw std::invalid_argument("exactly one RECORDING is accepted");
            else recording_path = std::string(argument);
        }
        if (!recording_path) throw std::invalid_argument("RECORDING is required");
        if (expect && against) throw std::invalid_argument("--expect and --against are exclusive");
        recording::DigestRequest request;
        if (tables) request.tables = recording::parse_digest_tables(*tables);
        if (episode) request.episode = parse_integer(*episode, "--episode");
        if (block_steps) request.block_steps = parse_integer(*block_steps, "--block-steps");
        const recording::Recording actual{std::filesystem::path(*recording_path)};
        if (against) {
            if (block_steps) throw std::invalid_argument("--against compares per interval; --block-steps does not apply");
            request.block_steps = 1;
            const recording::Recording golden{std::filesystem::path(*against)};
            const auto comparison = recording::compare_digests(recording::digest_episode(golden, request), recording::digest_episode(actual, request));
            if (!comparison.matches) return report(comparison, error);
            out << "match per interval against " << *against << '\n';
            return 0;
        }
        if (expect) {
            const auto expected = recording::parse_digest(read_file(*expect));
            if (!tables) request.tables = expected.tables;
            if (!block_steps) request.block_steps = expected.block_steps;
            const auto comparison = recording::compare_digests(expected, recording::digest_episode(actual, request));
            if (!comparison.matches) return report(comparison, error);
            out << "match " << expected.sha256 << '\n';
            return 0;
        }
        out << recording::format_digest(recording::digest_episode(actual, request));
        return 0;
    } catch (const std::exception& failure) {
        error << "bike-recording-digest: " << failure.what() << '\n';
        return 2;
    }
}
} // namespace bike::cli
```

Create `native/src/cli/recording_digest_main.cpp`:

```cpp
#include "recording_digest.hpp"
#include "../platform/argv.hpp"
#include <iostream>
#include <string_view>
#include <vector>
// NOLINTNEXTLINE(bugprone-exception-escape) entry point lets failures surface as the reference does
int main(int argc, char** argv) {
    std::vector<std::string_view> arguments;
    const auto raw = bike::platform::main_arguments(argc, argv);
    for (std::size_t i = 1; i < raw.size(); ++i) arguments.emplace_back(raw[i]);
    return bike::cli::recording_digest_command(arguments, std::cout, std::cerr);
}
```

In `native/cmake/RecordingApplication.cmake`:
- add `"${native_dir}/src/recording/digest.cpp"` to the `bike_recording_reader` source list, after the `export.cpp` line (`:41`);
- insert after the `bike-research-sweep` block (after `:230`, before `if(BUILD_TESTING)`):

```cmake
  # GPU shared physics Stage 0: the D2/D18 end-to-end digest. Built into
  # bin/ for golden_episode_digest and the D2 command; deliberately not
  # installed — a verification tool, not a shipped entry point. Engine-free:
  # it links only the analyst reader chain.
  add_library(bike_recording_digest_cli STATIC "${native_dir}/src/cli/recording_digest.cpp")
  bike_startup_target_policy(bike_recording_digest_cli "${native_dir}")
  target_link_libraries(bike_recording_digest_cli PUBLIC bike_recording_reader)
  add_executable(bike-recording-digest "${native_dir}/src/cli/recording_digest_main.cpp")
  bike_startup_target_policy(bike-recording-digest "${native_dir}")
  target_link_libraries(bike-recording-digest PRIVATE bike_recording_digest_cli)
  set_target_properties(bike-recording-digest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cmake --preset release
cmake --build build/native-release --target reference.recording_digest bike-recording-digest
ctest --test-dir build/native-release -R '^reference\.recording_digest$' --no-tests=error --output-on-failure
build/native-release/bin/bike-recording-digest --help; echo "exit=$?"
```

Expected:
- `100% tests passed, 0 tests failed out of 1`;
- the usage text starting `usage: bike-recording-digest`, followed by `exit=0`.

Stop conditions:
- `test_byte_format_is_pinned` fails only on the `-0.` element: DuckDB or the reader does not preserve signed zero in `DOUBLE[]`. Stop and report this. It contradicts `docs/RECORDING.md` §Data model, and it decides whether the digest can claim bit identity for `-0.0`.
- Do not weaken the test.

- [ ] **Step 5: Run the sweeps that cover the new translation units**

```bash
build/native-release/bin/bike-native-checks --kind headers --build build/native-release
build/native-release/bin/bike-native-checks --kind tidy --build build/native-release
build/native-release/bin/bike-native-checks --kind frontends --build build/native-release
build/native-release/bin/bike-native-checks --kind contracts --build build/native-release --repo-root "$PWD"
```

Expected: each exits 0.

Stop condition: a finding in a file this task created must be fixed in the code. Do not add a suppression unless it carries the checker, origin, reason, reproducer and removal condition (native-engineering.md §Review). A finding in a file this task did not touch is preexisting: record it and continue. Verify it against the documented baseline in `docs/TESTING.md` and AGENTS.md §Verification speed.

- [ ] **Step 6: Commit**

```bash
git add native/src/recording/digest.hpp native/src/recording/digest.cpp \
  native/src/cli/recording_digest.hpp native/src/cli/recording_digest.cpp native/src/cli/recording_digest_main.cpp \
  native/cmake/RecordingApplication.cmake native/tests/reference/test_recording_digest.cpp \
  native/tests/reference/GoldenEpisodeDigest.cmake native/tests/reference/CMakeLists.txt
git commit -m "$(cat <<'EOF'
feat(s0.1): add bike-recording-digest for the D2 gate

Stage 0 needs a bit-exact end-to-end check before any recorder or physics
change. The digest hashes only per-step physics and forces rows, so two
runs of the same physics agree regardless of run identity or wall clock,
and a mismatch names the first differing block or interval and table.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Schema-1 goldens (before any recorder change)

**Files:**
- Create: `native/tests/reference/run_golden_episode_digest.cmake`
- Create: `tests/fixtures/episode-digest/darwin-arm64/flat.schema1.digest`, `tests/fixtures/episode-digest/darwin-arm64/rough_uphill_extreme.schema1.digest`
- Create: `tests/fixtures/recording-schema1/flat_5ms/{manifest.json,run.duckdb,summary.md}` and `tests/fixtures/recording-schema1/flat_5ms.digest`
- Generated, not committed: `output/gpu_golden/schema1-{a,b,warm}/…`

**Interfaces:**
- Consumes: `bike-ride`, `bike-recording-digest` (Task 1).
- Produces:
  - the driver, with `-DRIDE -DDIGEST -DSOURCE_DIR -DSCRATCH -DTRACKS` plus exactly one of `-DCAPTURE_DIR` or `-DEXPECT_DIR`, and optional `-DTABLES -DSUFFIX -DCACHE_DIR -DKEEP`;
  - the schema-1 golden digests;
  - the real schema-1 container for R5.

The gate must be decided before any recorder code changes. So this task runs at the Task 1 commit, and nothing under `native/src/recording/` or `native/src/runtime/` may differ from `HEAD` while it runs.

- [ ] **Step 1: Confirm the tree is the pre-change recorder**

```bash
git status --short native/src
git log -1 --format=%s
```

Expected:
- the first command lists only ` M native/src/viewer/render_owner.cpp`, a preexisting change outside the recorder and the physics path;
- the second command prints `feat(s0.1): add bike-recording-digest for the D2 gate`.

Stop condition: any other `native/src` modification is listed. Stop and ask. The goldens must come from the committed recorder.

- [ ] **Step 2: Write the driver**

Create `native/tests/reference/run_golden_episode_digest.cmake`:

```cmake
# Records the GPU shared physics Stage 0 golden episodes and captures or
# verifies their bike-recording-digest (D2/D18;
# docs/superpowers/plans/2026-11-03-gpu-shared-s0-goldens.md).
#
# Every ride starts from an empty equilibrium cache unless CACHE_DIR names
# one: a cache hit skips the settle (initialization/equilibrium.cpp), so a
# warm cache is a different initial state, not the same golden.
# --check-policy diagnostic: strict headless rides stop at the documented
# energy.constraint_work baseline at t=0.005 s (docs/RIDE.md); exit 2 is
# accepted because the digest itself pins outcome and extent.
cmake_minimum_required(VERSION 3.27)
foreach(_name RIDE DIGEST SOURCE_DIR SCRATCH TRACKS)
  if(NOT DEFINED ${_name} OR "${${_name}}" STREQUAL "")
    message(FATAL_ERROR "run_golden_episode_digest requires ${_name}")
  endif()
endforeach()
if(DEFINED CAPTURE_DIR AND DEFINED EXPECT_DIR)
  message(FATAL_ERROR "pass exactly one of CAPTURE_DIR or EXPECT_DIR")
endif()
if(NOT DEFINED CAPTURE_DIR AND NOT DEFINED EXPECT_DIR)
  message(FATAL_ERROR "pass exactly one of CAPTURE_DIR or EXPECT_DIR")
endif()
if(NOT DEFINED SUFFIX)
  set(SUFFIX ".digest")
endif()
set(_digest_args "")
if(DEFINED TABLES AND NOT TABLES STREQUAL "")
  set(_digest_args --tables "${TABLES}")
endif()
string(REPLACE "," ";" _tracks "${TRACKS}")
foreach(_track IN LISTS _tracks)
  if(_track STREQUAL "flat")
    set(_track_arg flat)
  elseif(EXISTS "${SOURCE_DIR}/examples/research/${_track}.toml")
    set(_track_arg "${SOURCE_DIR}/examples/research/${_track}.toml")
  else()
    message(FATAL_ERROR "unknown golden track ${_track}")
  endif()
  set(_root "${SCRATCH}/${_track}")
  if(IS_SYMLINK "${_root}")
    message(FATAL_ERROR "Refusing a symlinked golden scratch directory ${_root}")
  endif()
  file(REMOVE_RECURSE "${_root}")
  file(MAKE_DIRECTORY "${_root}/recording")
  if(DEFINED CACHE_DIR)
    set(_cache "${CACHE_DIR}")
  else()
    set(_cache "${_root}/equilibrium-cache")
    file(MAKE_DIRECTORY "${_cache}")
  endif()
  string(TIMESTAMP _start "%s" UTC)
  execute_process(COMMAND "${CMAKE_COMMAND}" -E env "BIKE_SIM_EQUILIBRIUM_CACHE_DIR=${_cache}"
      "${RIDE}" --physics-config "${SOURCE_DIR}/examples/research/viewer_physics_distributed.toml"
      --track "${_track_arg}" --headless --duration 30 --check-policy diagnostic
      --record --out "${_root}/recording" --no-plots
    WORKING_DIRECTORY "${SOURCE_DIR}" RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  string(TIMESTAMP _end "%s" UTC)
  math(EXPR _wall "${_end} - ${_start}")
  file(WRITE "${_root}/ride.log" "exit=${_rc}\nwall_s=${_wall}\n${_out}${_err}")
  if(NOT _rc EQUAL 0 AND NOT _rc EQUAL 2)
    message(FATAL_ERROR "${_track}: bike-ride failed (${_rc}); log ${_root}/ride.log")
  endif()
  file(GLOB _manifests LIST_DIRECTORIES false "${_root}/recording/*/manifest.json")
  list(LENGTH _manifests _count)
  if(NOT _count EQUAL 1)
    message(FATAL_ERROR "${_track}: expected exactly one sealed recording under ${_root}/recording, found ${_count}")
  endif()
  get_filename_component(_recording "${_manifests}" DIRECTORY)
  if(DEFINED CAPTURE_DIR)
    file(MAKE_DIRECTORY "${CAPTURE_DIR}")
    execute_process(COMMAND "${DIGEST}" "${_recording}" ${_digest_args}
      OUTPUT_FILE "${CAPTURE_DIR}/${_track}${SUFFIX}" RESULT_VARIABLE _rc ERROR_VARIABLE _err)
    if(NOT _rc EQUAL 0)
      message(FATAL_ERROR "${_track}: digest capture failed (${_rc}): ${_err}")
    endif()
    message(STATUS "${_track}: captured ${CAPTURE_DIR}/${_track}${SUFFIX} from ${_recording} (bike-ride exit ${_rc}, wall ${_wall} s)")
  else()
    execute_process(COMMAND "${DIGEST}" "${_recording}" ${_digest_args} --expect "${EXPECT_DIR}/${_track}${SUFFIX}"
      RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    if(NOT _rc EQUAL 0)
      message(FATAL_ERROR "${_track}: digest differs from ${EXPECT_DIR}/${_track}${SUFFIX} (${_rc}); recording kept at ${_recording}\n${_out}${_err}")
    endif()
    message(STATUS "${_track}: ${_out} (wall ${_wall} s)")
    if(NOT KEEP)
      file(REMOVE_RECURSE "${_root}")
    endif()
  endif()
endforeach()
```

- [ ] **Step 3: Build and record run a (cold cache)**

```bash
cmake --build build/native-release --target bike-ride bike-recording-digest
cmake -DRIDE="$PWD/build/native-release/bin/bike-ride" -DDIGEST="$PWD/build/native-release/bin/bike-recording-digest" \
  -DSOURCE_DIR="$PWD" -DSCRATCH="$PWD/output/gpu_golden/schema1-a" -DTRACKS=flat,rough_uphill_extreme \
  -DTABLES=physics,forces -DSUFFIX=.schema1.digest -DCAPTURE_DIR="$PWD/output/gpu_golden/schema1-a" \
  -P native/tests/reference/run_golden_episode_digest.cmake
grep -E '^(schema|tables|intervals|outcome) ' output/gpu_golden/schema1-a/*.schema1.digest
head -2 output/gpu_golden/schema1-a/*/ride.log
```

Expected:
- two `-- <track>: captured …` lines;
- for each track: `schema 1`, `tables physics,forces`, `intervals 24000`, `outcome duration_reached`;
- each `ride.log` starts with `exit=0` or `exit=2` and a `wall_s=` value.

Stop conditions:
- `outcome wall_clock_cap` on either track: the ride hit the 300 s wall cap (F21). Its extent then depends on host load, so it cannot be a golden. Stop and report the measured `wall_s`. The fix (a larger `max_wall_clock_s` for golden rides) is a user decision.
- Any other `outcome`, or `intervals` ≠ 24000: stop and report `ride.log`. The 30 s episode the master plan's D18 requires was not produced.
- `bike-ride` fails with an argument error: compare the flags against `native/src/cli/argument_contracts.inc` (`bike-ride`). Do not guess replacement flags.

- [ ] **Step 4: Record run b and prove determinism**

```bash
cmake -DRIDE="$PWD/build/native-release/bin/bike-ride" -DDIGEST="$PWD/build/native-release/bin/bike-recording-digest" \
  -DSOURCE_DIR="$PWD" -DSCRATCH="$PWD/output/gpu_golden/schema1-b" -DTRACKS=flat,rough_uphill_extreme \
  -DTABLES=physics,forces -DSUFFIX=.schema1.digest -DCAPTURE_DIR="$PWD/output/gpu_golden/schema1-b" \
  -P native/tests/reference/run_golden_episode_digest.cmake
diff output/gpu_golden/schema1-a/flat.schema1.digest output/gpu_golden/schema1-b/flat.schema1.digest && echo flat-identical
diff output/gpu_golden/schema1-a/rough_uphill_extreme.schema1.digest output/gpu_golden/schema1-b/rough_uphill_extreme.schema1.digest && echo rough-identical
build/native-release/bin/bike-recording-digest output/gpu_golden/schema1-b/flat/recording/*/ --tables physics,forces \
  --against output/gpu_golden/schema1-a/flat/recording/*/; echo "exit=$?"
```

Expected: `flat-identical`, `rough-identical`, `match per interval against …`, `exit=0`.

Stop condition: any difference means the CPU ride is not run-to-run deterministic, so D2 cannot be checked by a digest. Stop and report:
- the `--against` output, which gives the first differing interval and table;
- the `wall_s` of both runs.

The likely suspects are the wall-budgeted advance chunks (F21, `ride.cpp:178`) and threading. Do not commit a fixture.

- [ ] **Step 5: Warm-cache check (informational, decides the cache rule)**

```bash
cmake -DRIDE="$PWD/build/native-release/bin/bike-ride" -DDIGEST="$PWD/build/native-release/bin/bike-recording-digest" \
  -DSOURCE_DIR="$PWD" -DSCRATCH="$PWD/output/gpu_golden/schema1-warm" -DTRACKS=flat \
  -DCACHE_DIR="$PWD/output/gpu_golden/schema1-a/flat/equilibrium-cache" \
  -DTABLES=physics,forces -DSUFFIX=.schema1.digest -DCAPTURE_DIR="$PWD/output/gpu_golden/schema1-warm" \
  -P native/tests/reference/run_golden_episode_digest.cmake
ls output/gpu_golden/schema1-a/flat/equilibrium-cache
diff output/gpu_golden/schema1-a/flat.schema1.digest output/gpu_golden/schema1-warm/flat.schema1.digest && echo warm-identical
```

Expected: the cache directory lists at least one file. The last line is either `warm-identical` or a diff. Record which one in the task report. The gate always uses a cold cache (the driver default), so both outcomes are acceptable. A diff confirms that the cold-cache rule is load-bearing. Mention it in Task 5's TESTING.md text.

Stop condition: the cache directory is empty, so run a never wrote a cache entry. Report this. The warm comparison then proves nothing; skip it and say so.

- [ ] **Step 6: Commit the schema-1 golden digests**

```bash
mkdir -p tests/fixtures/episode-digest/darwin-arm64
cp output/gpu_golden/schema1-a/flat.schema1.digest output/gpu_golden/schema1-a/rough_uphill_extreme.schema1.digest \
  tests/fixtures/episode-digest/darwin-arm64/
wc -l tests/fixtures/episode-digest/darwin-arm64/*.schema1.digest
```

Expected: each file has 129 lines: 9 header lines plus 60 blocks × 2 tables.

- [ ] **Step 7: Record the real schema-1 container fixture (R5)**

```bash
rm -rf output/gpu_golden/schema1-fixture && mkdir -p output/gpu_golden/schema1-fixture/cache
env BIKE_SIM_EQUILIBRIUM_CACHE_DIR="$PWD/output/gpu_golden/schema1-fixture/cache" \
  build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_distributed.toml \
  --track flat --headless --duration 0.005 --check-policy diagnostic --record \
  --out output/gpu_golden/schema1-fixture/recording --no-plots; echo "exit=$?"
ls output/gpu_golden/schema1-fixture/recording/*/
du -sk output/gpu_golden/schema1-fixture/recording/*/
```

Expected:
- `exit=0` or `exit=2`;
- the published directory lists `manifest.json`, `run.duckdb` and possibly `summary.md`;
- `du` prints a size in KiB.

Stop condition: the published directory is larger than 16384 KiB. Stop and ask before committing it; the largest committed precedent is ≈10 MB, F33. One alternative to offer: first check whether `tests/fixtures/migration/recording/good` opens with the native reader (`build/native-release/bin/bike-recording overview tests/fixtures/migration/recording/good`). If it opens, it can serve as the v1 fixture instead.

```bash
mkdir -p tests/fixtures/recording-schema1/flat_5ms
cp -p output/gpu_golden/schema1-fixture/recording/*/* tests/fixtures/recording-schema1/flat_5ms/
build/native-release/bin/bike-recording-digest tests/fixtures/recording-schema1/flat_5ms > tests/fixtures/recording-schema1/flat_5ms.digest
grep -E '^(schema|tables|intervals) ' tests/fixtures/recording-schema1/flat_5ms.digest
```

Expected: `schema 1`, `tables physics,forces`, `intervals 4`.

- [ ] **Step 8: Commit**

```bash
git add native/tests/reference/run_golden_episode_digest.cmake \
  tests/fixtures/episode-digest/darwin-arm64/flat.schema1.digest \
  tests/fixtures/episode-digest/darwin-arm64/rough_uphill_extreme.schema1.digest \
  tests/fixtures/recording-schema1/flat_5ms tests/fixtures/recording-schema1/flat_5ms.digest
git commit -m "$(cat <<'EOF'
test(s0.2): capture schema-1 golden episode digests

D18 requires the 30 s flat and rough_uphill_extreme episodes to be frozen
before the first recorder change, so gate B can prove schema v2 leaves
physics and forces bit-identical. Two cold-cache runs produced identical
digests. The 5 ms schema-1 container keeps the v1 read path testable after
the writer moves to schema 2 (R5).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Recording schema v2 storage (`controls`, `precision`)

**Files:**
- Modify:
  - `native/src/recording/catalog_data.inc`, `catalog.hpp`, `catalog.cpp`, `schema.cpp`;
  - `database.hpp`, `database_validate.cpp`, `reader.hpp`, `reader.cpp`, `publish.cpp`, `physical_source.cpp`, `digest.hpp`, `digest.cpp`;
  - `native/src/replay/input.cpp`;
  - `native/src/runtime/control.hpp`, `native/src/runtime/samples.hpp`.
- Create: `native/tests/reference/test_recording_schema_v2.cpp`
- Modify (expectation-only): `native/tests/migration/test_t38.cpp`, `native/tests/migration/test_t43.cpp`, `native/tests/reference/test_native_platform_replay.cpp`, `native/tests/reference/test_recording_platform_compatibility.cpp`, `native/tests/reference/test_replay_recording_v3.cpp`, `native/tests/reference/test_native_encode_columns.cpp`, `native/tests/reference/test_native_recording_retention.cpp`
- Modify: `native/tests/reference/GoldenEpisodeDigest.cmake`

**Interfaces:**
- Consumes: Task 1 digest API, the Task 2 fixture `tests/fixtures/recording-schema1/flat_5ms`.
- Produces:
  - `runtime::StepControls`, `PhysicalSampleData::controls`;
  - `recording::schema_version == 2`, `tables_for_schema(int)`, `catalog_rows(int)`, `container_schema_version(const Connection&)`;
  - `Recording::schema_version()` read from the container, `Recording::precision()`;
  - `DigestTable::controls`, `default_digest_tables(2)`.

- [ ] **Step 1: Write the failing test**

Create `native/tests/reference/test_recording_schema_v2.cpp`:

```cpp
// GPU shared physics Stage 0 — recording schema 2 (D16/D22, R5;
// docs/superpowers/plans/2026-11-03-gpu-shared-s0-goldens.md, Task 3).
// The writer publishes schema 2 with a catalog-declared controls table and
// the manifest precision lane; validation enforces controls coverage and
// width; the real schema-1 container recorded before the change still
// opens and digests identically.
#include "recording_test_support.hpp"
#include "recording/digest.hpp"
#include "recording/publish.hpp"
#include <bit>
#include <cmath>
#include <fstream>
#include <limits>

namespace {
using namespace recording_test;
namespace detail = rec::detail;

constexpr double dt = 1. / 8;

SamplePtr controlled(std::int64_t id, std::vector<double> ctrl, std::optional<double> bound, std::optional<double> ratio) {
    auto sample = std::make_shared<runtime::PhysicalSampleData>(*sample_row(id, static_cast<double>(id) * dt, dt));
    sample->controls = runtime::StepControls{.ctrl = std::move(ctrl), .freehub_upper_rad = bound, .chain_ratio = ratio};
    return SamplePtr(std::move(sample));
}
std::vector<SamplePtr> controlled_rows(double ctrl_one = -2.) {
    return {controlled(0, {.5, -0.}, 1.25, 22. / 51.),
            controlled(1, {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}, .75, std::nullopt),
            controlled(2, {1e-300, ctrl_one}, std::nullopt, std::nullopt)};
}
std::filesystem::path publish(const std::filesystem::path& root, std::vector<SamplePtr> rows) {
    auto capture = capture_rows(std::move(rows));
    return rec::finalize(*capture, root).directory;
}
bool same_bits(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }
// recording_test::member takes an Object; manifest members are read from a Value.
const Value& field_of(const Value& object, std::string_view key) { return detail::field(object, key); }
std::filesystem::path schema1_fixture() { return repo_file("tests/fixtures/recording-schema1/flat_5ms"); }
void write_manifest(const std::filesystem::path& dir, const Value& manifest) {
    std::ofstream out(dir / "manifest.json", std::ios::trunc);
    out << io::recording_json(manifest) << '\n';
}
std::filesystem::path copy_dir(const std::filesystem::path& from, const std::filesystem::path& to) {
    std::filesystem::copy(from, to, std::filesystem::copy_options::recursive);
    return to;
}

void test_writer_publishes_schema_2_controls() {
    bt::TemporaryDirectory dir("schema2-controls");
    const auto path = publish(dir.path() / "run", controlled_rows());
    const auto reader = sealed(path);
    bt::require(reader->schema_version() == 2 && reader->precision() == std::optional<std::string>("f64"),
        "the native writer publishes container schema 2 on the f64 lane");
    const auto rows = reader->rows("SELECT interval_id,ctrl,freehub_upper_rad,freehub_upper_rad_present,chain_ratio,chain_ratio_present "
                                   "FROM controls ORDER BY interval_id");
    bt::require(rows.size() == 3, "one controls row per physical interval");
    const auto ctrl = [&](std::size_t r, std::size_t k) { return io::tagged_double(io::as_array(*member(rows.at(r), "ctrl"), "ctrl").at(k), "ctrl"); };
    bt::require(same_bits(ctrl(0, 0), .5) && same_bits(ctrl(0, 1), -0.) && std::isnan(ctrl(1, 0)) &&
        same_bits(ctrl(1, 1), std::numeric_limits<double>::infinity()) && same_bits(ctrl(2, 0), 1e-300) && same_bits(ctrl(2, 1), -2.),
        "ctrl round-trips binary64, signed zero and nonfinite values");
    bt::require(as_bool(*member(rows.at(0), "freehub_upper_rad_present")) && same_bits(as_real(*member(rows.at(0), "freehub_upper_rad")), 1.25) &&
        as_bool(*member(rows.at(0), "chain_ratio_present")) && same_bits(as_real(*member(rows.at(0), "chain_ratio")), 22. / 51.),
        "hub bound and chain ratio are stored exactly when present");
    bt::require(as_bool(*member(rows.at(1), "freehub_upper_rad_present")) && !as_bool(*member(rows.at(1), "chain_ratio_present")) &&
        !as_bool(*member(rows.at(2), "freehub_upper_rad_present")) && !as_bool(*member(rows.at(2), "chain_ratio_present")),
        "absent hub values are absent, not zero");
    bt::require(detail::find(field_of(reader->manifest(), "row_counts"), "controls") != nullptr, "the manifest counts the controls table");
}

void test_validation_rejects_inconsistent_controls() {
    const std::array<std::pair<std::string_view, std::string_view>, 3> cases{{
        {"DELETE FROM controls WHERE interval_id=2", "controls: coverage differs from physical prefix"},
        {"UPDATE controls SET ctrl=[1.0] WHERE interval_id=1", "controls: actuator width differs"},
        {"UPDATE controls SET freehub_upper_rad=NULL, freehub_upper_rad_present=false WHERE interval_id=0",
         "controls: chain ratio without freehub bound"}}};
    for (const auto& [sql, message] : cases) {
        bt::TemporaryDirectory dir("schema2-invalid");
        const auto path = publish(dir.path() / "run", controlled_rows());
        mutate_database(path, sql);
        throws<std::invalid_argument>([&] { (void)sealed(path); }, message, std::string("sealed validation refuses: ") + std::string(sql));
    }
}

void test_manifest_lanes_are_versioned() {
    bt::TemporaryDirectory dir("schema2-manifest");
    const auto path = publish(dir.path() / "run", controlled_rows());
    auto manifest = io::parse_recording_json(bt::text(path / "manifest.json"));
    auto& object = std::get<Value::Object>(manifest.data);
    (void)detail::pop(object, "precision");
    write_manifest(path, manifest);
    bt::invalid([&] { (void)rec::read_manifest(path); });
    detail::set(object, "precision", text("f16"));
    write_manifest(path, manifest);
    bt::invalid([&] { (void)rec::read_manifest(path); });
    const auto v1 = copy_dir(schema1_fixture(), dir.path() / "v1");
    auto old = io::parse_recording_json(bt::text(v1 / "manifest.json"));
    detail::set(std::get<Value::Object>(old.data), "precision", text("f64"));
    write_manifest(v1, old);
    bt::invalid([&] { (void)rec::read_manifest(v1); });
}

void test_schema_1_container_still_reads_and_digests() {
    const auto reader = sealed(schema1_fixture());
    bt::require(reader->schema_version() == 1 && !reader->precision() && !reader->episode_ids().empty(),
        "the pre-change schema-1 container opens read-only with no precision lane");
    bt::require(detail::find(field_of(reader->manifest(), "row_counts"), "controls") == nullptr, "schema 1 has no controls table");
    const auto expected = rec::parse_digest(bt::text(repo_file("tests/fixtures/recording-schema1/flat_5ms.digest")));
    const auto actual = rec::digest_episode(*reader, {});
    bt::require(rec::compare_digests(expected, actual).matches && actual.tables == rec::default_digest_tables(1),
        "schema-2 code reads the schema-1 rows bit-identically (R5)");
}

void test_digest_covers_controls_on_schema_2() {
    bt::TemporaryDirectory dir("schema2-digest");
    const auto left = sealed(publish(dir.path() / "left", controlled_rows()));
    const auto right = sealed(publish(dir.path() / "right", controlled_rows(std::nextafter(-2., 0.))));
    rec::DigestRequest per_interval;
    per_interval.block_steps = 1;
    const auto a = rec::digest_episode(*left, per_interval);
    const auto b = rec::digest_episode(*right, per_interval);
    bt::require(a.tables == std::vector{rec::DigestTable::physics, rec::DigestTable::forces, rec::DigestTable::controls},
        "schema 2 digests physics, forces and controls by default");
    std::string text;
    for (const auto& line : rec::compare_digests(a, b).differences) text += line + '\n';
    bt::require(text.find("first differing interval 2 in controls") != std::string::npos, "a one-ulp ctrl change is named: " + text);
    rec::DigestRequest physical;
    physical.tables = {rec::DigestTable::physics, rec::DigestTable::forces};
    bt::require(rec::compare_digests(rec::digest_episode(*left, physical), rec::digest_episode(*right, physical)).matches,
        "controls never leak into the physics,forces digest that gate B compares");
    bt::invalid([] { (void)rec::digest_episode(*sealed(schema1_fixture()), [] { rec::DigestRequest r; r.tables = {rec::DigestTable::controls}; return r; }()); });
}
} // namespace

int main() {
    return bt::run([] {
        test_writer_publishes_schema_2_controls();
        test_validation_rejects_inconsistent_controls();
        test_manifest_lanes_are_versioned();
        test_schema_1_container_still_reads_and_digests();
        test_digest_covers_controls_on_schema_2();
    });
}
```

Append to `native/tests/reference/GoldenEpisodeDigest.cmake`:

```cmake
bike_add_reference_test(recording_schema_v2 ENGINE
  SOURCES test_recording_schema_v2.cpp
  LIBRARIES bike_recording
  LABELS recording)
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cmake --preset release
cmake --build build/native-release --target reference.recording_schema_v2
```

Expected: the build fails with `no member named 'controls' in 'runtime::PhysicalSampleData'` (or `no type named 'StepControls'`). That is the red.

- [ ] **Step 3: Add the controls value type**

In `native/src/runtime/control.hpp`, insert immediately before the file's last line `} // namespace runtime`:

```cpp
// Recording schema 2 (D16): the controller outputs one interval's
// integrator consumed — sampled in PhysicalStep::advance_body after
// apply_forces and before engine::step. Hub values are the live model
// rows (tendon_range upper bound, wrap_prm driver coefficient) and are
// absent when the plant has no ideal hub (chain_ratio also for the
// geometric hub, whose coefficients are not a scalar ratio).
struct StepControls {
    std::vector<double> ctrl;
    std::optional<double> freehub_upper_rad, chain_ratio;
};

```

In `native/src/runtime/samples.hpp`, `PhysicalSampleData` (`:58-69`), add after `std::shared_ptr<const ConstraintSnapshot> constraints;`:

```cpp
    StepControls controls;  // recording schema 2 `controls` row
```

- [ ] **Step 4: Declare the controls table in the catalog**

In `native/src/recording/catalog_data.inc` line 6, change `"schema": 1,` to `"schema": 2,`.

In the same file, replace the document tail:

```
      "description": "transitions",
      "source": "research owned history"
    }
  ]
}
)CATALOG",
```

with:

```
      "description": "transitions",
      "source": "research owned history"
    }
  ],
  "schema_2_tables": [
    {
      "claims": [],
      "exempt_keys": [],
      "name": "controls",
      "kind": "controls",
      "keys": [
        {
          "frame": "n/a",
          "path": [
            "episode_id"
          ],
          "aliases": [],
          "shape": null,
          "axes": null,
          "except_keys": [],
          "children": [],
          "key": "episode_id",
          "dtype": "bigint",
          "unit": "n/a",
          "phase": "n/a",
          "optional": false,
          "measurement": "declared",
          "source": "docs/superpowers/specs/2026-10-12-run-recording-design.md::R01",
          "description": "episode identity within the owning run"
        },
        {
          "frame": "n/a",
          "path": [
            "interval_id"
          ],
          "aliases": [],
          "shape": null,
          "axes": null,
          "except_keys": [],
          "children": [],
          "key": "interval_id",
          "dtype": "bigint",
          "unit": "n/a",
          "phase": "interval",
          "optional": false,
          "measurement": "declared",
          "source": "native/src/runtime/step.cpp::PhysicalStep::sample_controls",
          "description": "accounted physics interval identifier; joins physics on (episode_id, interval_id)"
        }
      ],
      "fields": [
        {
          "frame": "model actuator order (mjModel actuator index)",
          "path": [
            "ctrl"
          ],
          "aliases": [],
          "shape": null,
          "axes": null,
          "except_keys": [],
          "children": [],
          "key": "ctrl",
          "dtype": "double[]",
          "unit": "per_actuator",
          "phase": "interval",
          "optional": false,
          "measurement": "measured",
          "source": "native/src/runtime/step.cpp::PhysicalStep::sample_controls",
          "description": "actuator controls the integrator consumed for this interval, sampled after apply_forces and before engine::step"
        },
        {
          "frame": "ideal hub freehub tendon",
          "path": [
            "freehub_upper_rad"
          ],
          "aliases": [],
          "shape": null,
          "axes": null,
          "except_keys": [],
          "children": [],
          "key": "freehub_upper_rad",
          "dtype": "double",
          "unit": "rad",
          "phase": "interval",
          "optional": true,
          "measurement": "measured",
          "source": "native/src/drivetrain/transmission.cpp::Transmission::applied_upper_bound",
          "description": "live tendon_range upper bound of the ideal hub freehub tendon the integrator consumed; absent without an ideal hub"
        },
        {
          "frame": "ideal hub freehub tendon",
          "path": [
            "chain_ratio"
          ],
          "aliases": [],
          "shape": null,
          "axes": null,
          "except_keys": [],
          "children": [],
          "key": "chain_ratio",
          "dtype": "double",
          "unit": "1",
          "phase": "interval",
          "optional": true,
          "measurement": "measured",
          "source": "native/src/drivetrain/transmission.cpp::Transmission::applied_ratio",
          "description": "live wrap_prm driver coefficient (applied chain ratio) of a non-geometric ideal hub; absent otherwise"
        }
      ],
      "description": "per-interval controller outputs the integrator consumed (recording schema 2)",
      "source": "native/src/runtime/step.cpp::PhysicalStep::sample_controls"
    }
  ]
}
)CATALOG",
```

Stop condition: after the edit, the last raw-literal part must stay below 65536 bytes (`-Woverlength-strings`, `catalog_data.inc:2-3`). Check it with:

```bash
awk 'NR>=9140' native/src/recording/catalog_data.inc | wc -c
```

Expected: a number below 65536 (≈34000). If line 9140 is no longer the start of the last part, find the last `R"CATALOG(` line first: `grep -n 'R"CATALOG(' native/src/recording/catalog_data.inc | tail -1`.

In `native/src/recording/catalog.hpp`:
- change `inline constexpr int schema_version = 1;` to:

```cpp
// Container schema 2 (D16/D22) adds the controls table and the manifest
// precision lane; schema 1 containers stay readable (R5).
inline constexpr int schema_version = 2;
```

- after `[[nodiscard]] const std::vector<TableSpec>& all_tables();` add:

```cpp
// The stream tables of one container schema: 1 = physical + research,
// 2 = schema 1 + controls. all_tables() is tables_for_schema(schema_version).
[[nodiscard]] const std::vector<TableSpec>& tables_for_schema(int version);
```

- change `[[nodiscard]] std::vector<io::Value::Array> catalog_rows();` to `[[nodiscard]] std::vector<io::Value::Array> catalog_rows(int version = schema_version);`.

In `native/src/recording/catalog.cpp`:
- `struct Descriptors { std::vector<TableSpec> physical, research; };` becomes `struct Descriptors { std::vector<TableSpec> physical, research, schema2; };`.
- In `descriptors()`, after the `research_tables` loop, add:

```cpp
        for (const auto& table : io::as_array(value::require(document, "schema_2_tables"), "schema_2_tables")) result.schema2.push_back(read_table(table));
```

- In `TableSpec::validate` (`:127`), change `kind != "physics" && kind != "forces" && kind != "channels"` to `kind != "physics" && kind != "forces" && kind != "channels" && kind != "controls"`.
- Replace the `all_tables()` definition (`:191`) with:

```cpp
const std::vector<TableSpec>& tables_for_schema(int version) {
    static const auto v1 = [] { auto result = descriptors().physical; result.insert(result.end(), descriptors().research.begin(), descriptors().research.end()); return result; }();
    static const auto v2 = [] { auto result = v1; result.insert(result.end(), descriptors().schema2.begin(), descriptors().schema2.end()); return result; }();
    if (version == 1) return v1;
    if (version == 2) return v2;
    throw std::invalid_argument("unsupported recording container schema " + std::to_string(version));
}
const std::vector<TableSpec>& all_tables() { return tables_for_schema(schema_version); }
```

In `native/src/recording/schema.cpp:37-39`, change `std::vector<io::Value::Array> catalog_rows() {` / `for (const auto& spec : all_tables()) {` to:

```cpp
std::vector<io::Value::Array> catalog_rows(int version) {
    std::vector<io::Value::Array> rows;
    for (const auto& spec : tables_for_schema(version)) {
```

`create_streams` keeps `all_tables()`, so the writer creates the schema-2 table set. `create_views` stays unchanged: `controls` is not in `schema_catalog()`. Its rows join on `(episode_id, interval_id)` through SQL.

- [ ] **Step 5: Make validation version-aware**

In `native/src/recording/database.hpp`, add after `validate_database(const Connection&)`:

```cpp
// recording_meta.schema_version of an open container; 1 or schema_version,
// anything else rejects.
[[nodiscard]] int container_schema_version(const Connection& connection);
```

In `native/src/recording/database_validate.cpp`, replace `validate_container_schema` (`:113-140`) with:

```cpp
struct ContainerSchema { std::set<std::string> tables; int version = 0; };
ContainerSchema validate_container_schema(const Connection& c) {
    const std::set<std::string> metadata{"recording_meta", "runs", "episodes", "events", "field_catalog", "coordinate_catalog", "alias_catalog", "assets", "integration_states", "metrics"};
    std::set<std::string> actual;
    for (const auto& row : c.query("SELECT table_name FROM information_schema.tables WHERE table_schema='main' AND table_type='BASE TABLE'")) actual.insert(io::as_string(value::require(row, "table_name"), "table_name"));
    if (!actual.contains("recording_meta")) throw std::invalid_argument("recording table set lacks recording_meta");
    const int version = container_schema_version(c);
    const auto label = "schema " + std::to_string(version);
    auto expected = metadata; for (const auto& spec : tables_for_schema(version)) expected.insert(spec.name);
    if (actual != expected) throw std::invalid_argument("recording table set differs from container " + label);
    const auto meta = c.query("SELECT run_id,codec FROM recording_meta");
    const auto runs = c.query("SELECT run_id FROM runs");
    if (meta.size() != 1 || runs.size() != 1 ||
        io::recording_json(value::require(meta.front(), "run_id")) != io::recording_json(value::require(runs.front(), "run_id")) ||
        io::as_string(value::require(meta.front(), "codec"), "codec") != "bike_recording.json.v1") throw std::invalid_argument("invalid recording identity/schema");
    for (const auto& spec : tables_for_schema(version)) {
        const auto columns = c.query("DESCRIBE " + quote(spec.name)); const auto types = column_types(spec);
        if (columns.size() != types.size()) throw std::invalid_argument(spec.name + ": columns differ from " + label);
        for (std::size_t i = 0; i < columns.size(); ++i) if (io::as_string(value::require(columns[i], "column_name"), "column_name") != types[i].first ||
            normalize(io::as_string(value::require(columns[i], "column_type"), "column_type")) != normalize(types[i].second)) throw std::invalid_argument(spec.name + ": column order/types differ from " + label);
        presence(c, spec);
        std::vector<std::string> keys; keys.reserve(spec.keys.size());
        for (const auto& f : spec.keys) keys.push_back(f.key);
        check_keys(c, spec.name, keys);
    }
    const std::map<std::string, std::vector<std::string>> meta_keys{{"runs", {"run_id"}}, {"episodes", {"episode_id"}}, {"events", {"episode_id", "event_id"}},
        {"field_catalog", {"table_name", "field_name"}}, {"coordinate_catalog", {"episode_id", "kind", "coordinate_index"}},
        {"alias_catalog", {"episode_id", "alias"}}, {"assets", {"episode_id", "name"}}, {"integration_states", {"episode_id", "phase"}}, {"metrics", {"episode_id", "name"}}};
    for (const auto& [table, keys] : meta_keys) check_keys(c, table, keys);
    return {.tables = std::move(actual), .version = version};
}
```

Further changes in the same file:
- `validate_declared_catalog(Connection const& c)` becomes `validate_declared_catalog(Connection const& c, int version)`, and its first line becomes `auto declared = catalog_rows(version);`.
- `validate_relationships_and_assets(const Connection& c)` becomes `validate_relationships_and_assets(const Connection& c, int version)`. Its loop `for (const auto& spec : all_tables())` becomes `for (const auto& spec : tables_for_schema(version))`. In that loop, the condition `spec.name == "forces" || spec.name.starts_with("channel_")` becomes `spec.name == "forces" || spec.name == "controls" || spec.name.starts_with("channel_")`.
- Add above `validate_episode_row`:

```cpp
// Schema 2 (D16): exactly one controls row per physical interval, one
// actuator width per episode, and a chain ratio only with its hub bound.
void validate_controls(const Connection& c, std::span<const Value> params, std::int64_t physical_rows) {
    if (table_count(c, "controls", params) != physical_rows) throw std::invalid_argument("controls: coverage differs from physical prefix");
    require_zero(c, "SELECT count(*) FROM controls WHERE episode_id=? AND ctrl IS NULL", "controls: missing actuator vector", params);
    if (count(c, "SELECT count(DISTINCT len(ctrl)) FROM controls WHERE episode_id=?", params) > 1) throw std::invalid_argument("controls: actuator width differs");
    require_zero(c, "SELECT count(*) FROM controls WHERE episode_id=? AND chain_ratio_present AND NOT freehub_upper_rad_present", "controls: chain ratio without freehub bound", params);
}
```

- `void validate_episode_row(const Connection& c, const Object& row)` becomes `void validate_episode_row(const Connection& c, const Object& row, int container_version)`. Its `all_tables()` channel loop (`:177`) becomes `tables_for_schema(container_version)`. Directly after the forces width `require_zero` (`:190`), insert:

```cpp
    if (container_version >= 2) validate_controls(c, params, physical_rows);
```

- `validate_episode_rows(const Connection& c)` becomes `validate_episode_rows(const Connection& c, int container_version)`, and its call becomes `validate_episode_row(c, row, container_version);`.
- Replace the body of `validate_database` (`:225-233`) and add `container_schema_version` above it, outside the anonymous namespace:

```cpp
int container_schema_version(const Connection& c) {
    const auto meta = c.query("SELECT schema_version FROM recording_meta");
    if (meta.size() != 1) throw std::invalid_argument("invalid recording identity/schema");
    const auto version = io::as_int64(value::require(meta.front(), "schema_version"), "schema_version");
    if (version != 1 && version != schema_version) throw std::invalid_argument("unsupported recording container schema " + std::to_string(version));
    return static_cast<int>(version);
}
io::Value::Object validate_database(const Connection& c) {
    const auto schema = validate_container_schema(c);
    validate_declared_catalog(c, schema.version);
    validate_relationships_and_assets(c, schema.version);
    validate_episode_rows(c, schema.version);
    Object counts;
    for (const auto& table : schema.tables) counts.emplace_back(table, value::integer(count(c, "SELECT count(*) FROM " + quote(table))));
    return counts;
}
```

Stop condition: `container_schema_version` is used inside the anonymous namespace before its definition. It is declared in `database.hpp`, which this file includes at `:1`, so this compiles. If a frontend reports an undeclared identifier, the `database.hpp` edit is missing.

- [ ] **Step 6: Versioned manifest and reader**

In `native/src/recording/reader.cpp`, `read_manifest` (`:30-55`), replace the lines from `const auto& object = io::as_object(manifest, "manifest");` through the `"unsupported recording manifest/container schema");` throw that follows the `database != "run.duckdb"` check with:

```cpp
    const auto& object = io::as_object(manifest, "manifest");
    const auto* declared = find(object, "schema_version");
    const auto version = declared ? count(*declared) : 0;
    if (version != 1 && version != schema_version) throw RecordingError("unsupported recording manifest/container schema");
    // Schema 2 (D22) names the build's precision lane; schema 1 never did.
    std::set<std::string> keys{"schema_version", "sealed", "run_id", "database", "file_sha256", "row_counts", "report_status", "report_error"};
    if (version >= 2) keys.insert("precision");
    exact_keys(object, keys, "unsupported recording manifest/container schema");
    if (version >= 2) {
        const auto lane = string(field(object, "precision"));
        if (lane != "f64" && lane != "f32" && lane != "gpu-f32") throw RecordingError("invalid recording precision lane");
    }
    if (!io::as_bool(field(object, "sealed"), "manifest.sealed") || string(field(object, "database")) != "run.duckdb")
        throw RecordingError("unsupported recording manifest/container schema");
```

In `Recording::Recording` (`:57-80`), directly after `const auto counts = validate_database(database_->connection());`, add:

```cpp
    schema_version_ = container_schema_version(database_->connection());
    if (schema_version_ != count(field(manifest_, "schema_version"))) throw RecordingError("manifest and database container schema disagree");
```

Add after the constructor:

```cpp
std::optional<std::string> Recording::precision() const {
    if (const auto* lane = find(manifest_, "precision")) return string(*lane);
    return std::nullopt;
}
```

`reader.cpp` must see `schema_version`. It includes `database.hpp`, which includes the catalog header; check with `grep -n 'include' native/src/recording/database.hpp`. If `catalog.hpp` is not reachable, add `#include "catalog.hpp"` to `reader.cpp`.

In `native/src/recording/reader.hpp`:
- replace `[[nodiscard]] std::int64_t schema_version() const noexcept { return 1; }` with:

```cpp
    // Container schema of the opened recording (1 or 2), checked equal
    // between manifest.json and recording_meta.
    [[nodiscard]] std::int64_t schema_version() const noexcept { return schema_version_; }
    // Schema-2 precision lane ("f64", "f32", "gpu-f32"); empty for schema 1.
    [[nodiscard]] std::optional<std::string> precision() const;
```

- add `std::int64_t schema_version_ = 0;` after `std::unique_ptr<Database> database_;`.

In `native/src/recording/publish.cpp`:
- add `#include <mujoco/mujoco.h>` and `#include <type_traits>` to the includes;
- add to the anonymous namespace, after `name_base`:

```cpp
// D22: precision is a build choice; the native CPU recorder writes the
// lane its mjtNum was compiled for.
constexpr std::string_view recording_precision() noexcept { return std::is_same_v<mjtNum, double> ? "f64" : "f32"; }
```

- extend the manifest object (`:66-68`) with one more member after `{"report_error", report_error}`: `, {"precision", value::text(recording_precision())}`.

In `native/src/replay/input.cpp`:
- at `:65`, replace `out.container_schema_version = 1;` with `out.container_schema_version = static_cast<int>(reader.schema_version());`;
- at `:105`, replace `{"container_schema_version", integer(1)}` with `{"container_schema_version", integer(reader.schema_version())}`.

- [ ] **Step 7: Emit the controls rows**

In `native/src/recording/physical_source.cpp`, add to the file's anonymous namespace (above `class NativePhysicalSource`):

```cpp
// Schema 2 controls row of one retained interval (D16). Encoded through the
// generic reviewed adapter: the table is outside the catalog projector's
// table_order, so the optimized plans and their golden fixtures are untouched.
Object controls_row(const runtime::PhysicalSampleData& sample) {
    Object row{{"interval_id", value::integer(sample.interval_id)}, {"ctrl", value::doubles(sample.controls.ctrl)}};
    if (sample.controls.freehub_upper_rad) row.emplace_back("freehub_upper_rad", value::real(*sample.controls.freehub_upper_rad));
    if (sample.controls.chain_ratio) row.emplace_back("chain_ratio", value::real(*sample.controls.chain_ratio));
    return row;
}
```

In `tables()`:
- the empty-episode branch (`:35-37`) becomes:

```cpp
        if (episode_->samples.empty()) {
            for (const auto& spec : specs) sink(spec.name, encode_rows(spec, {}, episode_->generation));
            sink("controls", encode_rows(table_spec("controls"), {}, episode_->generation));
            return;
        }
```

- directly before `first += count;` (`:76`), insert:

```cpp
            std::vector<Object> controls;
            controls.reserve(count);
            for (std::size_t n = first; n < first+count; ++n) controls.push_back(controls_row(*episode_->samples[n]));
            sink("controls", encode_rows(table_spec("controls"), controls, episode_->generation));
```

The third `encode_rows` argument mirrors the existing calls at `:36` and `:72` (`episode_->generation`). `append_pieces` assigns the owning `episode_id`, as it does for every physical table (`database.cpp:64`).

Stop condition: if `physical_source.cpp` has no `Object` alias or anonymous namespace, use `io::Value::Object` and wrap the helper in `namespace { … }`. Do not alter the existing emission order.

- [ ] **Step 8: Digest the controls table**

In `native/src/recording/digest.hpp`, `enum class DigestTable { physics, forces };` becomes `enum class DigestTable { physics, forces, controls };`.

In `native/src/recording/digest.cpp`:
- `table_names` becomes a 3-element array with `{DigestTable::controls, "controls"}` appended;
- `table_query` gets a new case:

```cpp
    case DigestTable::controls:
        return "SELECT interval_id,ctrl,freehub_upper_rad,freehub_upper_rad_present,chain_ratio,chain_ratio_present "
               "FROM controls WHERE episode_id=? ORDER BY interval_id";
```

- `record` gets a new case (byte format: u64 n + n×f64 ctrl; then u8 present + f64 value, or +0.0 bits when absent, for the bound and then the ratio):

```cpp
    case DigestTable::controls:
        stream.f64s(io::as_array(cell(row, "ctrl"), "ctrl"), "ctrl");
        for (const std::string_view name : {"freehub_upper_rad", "chain_ratio"}) {
            const bool present = io::as_bool(cell(row, std::string(name) + "_present"), name);
            stream.u8(static_cast<std::uint8_t>(present ? 1 : 0));
            stream.f64(present ? real_cell(row, name) : 0.);
        }
        return;
```

- `default_digest_tables` becomes:

```cpp
std::vector<DigestTable> default_digest_tables(std::int64_t container_schema) {
    if (container_schema == 1) return {DigestTable::physics, DigestTable::forces};
    if (container_schema == 2) return {DigestTable::physics, DigestTable::forces, DigestTable::controls};
    throw RecordingError("no digest defaults for container schema " + std::to_string(container_schema));
}
```

- in `digest_episode`, after `out.tables = …;`, add:

```cpp
    if (out.schema_version < 2 && std::ranges::find(out.tables, DigestTable::controls) != out.tables.end())
        throw RecordingError("a schema-1 recording has no controls table");
```

- [ ] **Step 9: Update the expectations that pin schema 1**

Each edit below changes only an expected value. None changes a fixture.
- `native/tests/migration/test_t38.cpp:22`: `rec::all_tables().size()` becomes `rec::tables_for_schema(1).size()`. At `:25`, `rec::all_tables()[i]` becomes `rec::tables_for_schema(1)[i]`. The source fixture `recording/schema.json` is the schema-1 table set, and the 497-descriptor count at `:36` is the schema-1 catalog.
- `native/tests/migration/test_t43.cpp:19`: `reader.schema_version() == 1` becomes `reader.schema_version() == 2`.
- `native/tests/reference/test_native_platform_replay.cpp:71`: `recording->schema_version() == 1` becomes `recording->schema_version() == 2`.
- `native/tests/reference/test_recording_platform_compatibility.cpp:144` and `:358`: `recording->schema_version() == 1` becomes `recording->schema_version() == 2`.
- `native/tests/reference/test_replay_recording_v3.cpp:22`: `*data.container_schema_version == 1` becomes `== 2`, and the message at `:24` becomes `"container input carries schema-2/contract-3 for episode 0"`. At `:33`, `rr::count(field(result, "container_schema_version")) == 1` becomes `== 2`.
- `native/tests/reference/test_native_encode_columns.cpp:2398-2400` and `native/tests/reference/test_native_recording_retention.cpp:329-331`: after the loop that fills `expected` from `table_order()`, add

```cpp
    // Schema 2 (D16): controls is emitted through the generic adapter,
    // outside the catalog projector's table order.
    expected.insert("controls");
```

Stop condition: do not edit `test_recording_platform_compatibility.cpp:419-421`. That reads the committed `tests/fixtures/platform-recordings/darwin-arm64.json`, a fixture.

- [ ] **Step 10: Run the targeted tests**

```bash
cmake --build build/native-release --target reference.recording_schema_v2 reference.recording_digest bike-recording-digest
ctest --test-dir build/native-release -R '^reference\.(recording_schema_v2|recording_digest)$' --no-tests=error --output-on-failure
ctest --test-dir build/native-release -L 'recording|replay|platform' -LE 'slow|audit|package' -j8 --no-tests=error --output-on-failure
ctest --test-dir build/native-release -R '^migration_T(3[89]|4[0-9]|5[01])$' -j8 --no-tests=error --output-on-failure
```

Expected: every command ends with `100% tests passed`.

Stop conditions:
- Another test fails on a schema-1 literal, the table set, `row_counts` keys or `field_catalog` counts. Apply the same expectation-only change, list it in the task report and continue. If the failure is anything else (encoded bytes, physics values, a golden fixture compare), stop and report it. Schema 2 must not change any existing row.
- `test_writer_publishes_schema_2_controls` fails on `-0.` inside `ctrl`: stop. This is the same signed-zero question as Task 1 Step 4.

- [ ] **Step 11: Run the slow tests whose expectations changed**

```bash
cmake --build build/native-release --target reference.native_encode_columns reference.native_recording_retention reference.recording_platform_compat
ctest --test-dir build/native-release -R '^reference\.(native_encode_columns|native_recording_retention|recording_platform_compat)$' --no-tests=error --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 3`.

- [ ] **Step 12: Sweeps over the changed translation units**

```bash
build/native-release/bin/bike-native-checks --kind headers --build build/native-release
build/native-release/bin/bike-native-checks --kind tidy --build build/native-release
build/native-release/bin/bike-native-checks --kind frontends --build build/native-release
build/native-release/bin/bike-native-checks --kind contracts --build build/native-release --repo-root "$PWD"
```

Expected: each exits 0. The stop condition is the same as Task 1 Step 5.

- [ ] **Step 13: Commit**

```bash
git add native/src/recording native/src/replay/input.cpp native/src/runtime/control.hpp native/src/runtime/samples.hpp \
  native/tests/reference/test_recording_schema_v2.cpp native/tests/reference/GoldenEpisodeDigest.cmake \
  native/tests/migration/test_t38.cpp native/tests/migration/test_t43.cpp \
  native/tests/reference/test_native_platform_replay.cpp native/tests/reference/test_recording_platform_compatibility.cpp \
  native/tests/reference/test_replay_recording_v3.cpp native/tests/reference/test_native_encode_columns.cpp \
  native/tests/reference/test_native_recording_retention.cpp
git commit -m "$(cat <<'EOF'
feat(s0.3): recording schema 2 with controls and precision

D16 needs the controller outputs the integrator consumed recorded per
step, and D22 needs the precision lane in the manifest, so later GPU
stages can replay controls instead of porting controllers. The controls
table stays outside the catalog projector, so existing encoded bytes are
unchanged; schema-1 containers keep validating and reading (R5).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Runtime capture of the integrator controls, plus gate B

**Files:**
- Modify: `native/src/drivetrain/transmission.hpp`, `native/src/drivetrain/transmission.cpp`, `native/src/runtime/step.hpp`, `native/src/runtime/step.cpp`, `native/src/runtime/accounting.cpp`, `docs/RECORDING.md`
- Create: `native/tests/reference/test_recording_controls_capture.cpp`
- Modify: `native/tests/reference/GoldenEpisodeDigest.cmake`

**Interfaces:**
- Consumes: `runtime::StepControls` (Task 3), `RuntimeRigAccess::inspect`, `recording_test::welded_runtime`.
- Produces: `drivetrain::Transmission::applied_upper_bound()`, `applied_ratio()`, `PhysicalStep::sample_controls`, the filled `PhysicalSampleData::controls`, and CTest `reference.recording_controls_capture`.

- [ ] **Step 1: Write the failing test**

Create `native/tests/reference/test_recording_controls_capture.cpp`:

```cpp
// GPU shared physics Stage 0 — the recorded controls are what the
// integrator consumed (D16; docs/superpowers/plans/2026-11-03-gpu-shared-s0-goldens.md,
// "Ordering argument"). Each retained sample's controls must equal, bit
// for bit, the live ctrl and the live hub model rows after its step —
// read through Transmission::state(), which reads the model independently
// of the recorder's accessors.
#include "recording_test_support.hpp"
#include "model_access.hpp"
#include "validation/runtime_access.hpp"
#include <bit>
#include <optional>
#include <vector>

namespace {
using namespace recording_test;

struct Live {
    std::vector<double> ctrl;
    std::optional<double> bound, ratio;
};
bool same_bits(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }
bool same_bits(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() && std::ranges::equal(a, b, [](double x, double y) { return same_bits(x, y); });
}
bool same_bits(const std::optional<double>& a, const std::optional<double>& b) {
    return a.has_value() == b.has_value() && (!a || same_bits(*a, *b));
}

void test_samples_record_the_integrator_controls() {
    const auto rig = welded_runtime("full");
    constexpr std::int64_t steps = 40;
    std::vector<Live> live;
    for (std::int64_t k = 1; k <= steps; ++k) {
        advance(*rig->runtime, k);
        bike::validation::RuntimeRigAccess::inspect(*rig->runtime, [&](const auto& stepper, const auto&) {
            const mjModel* m = stepper.model();
            const mjData* d = stepper.data();
            const auto ctrl = model_access::readonly_buffer(d->ctrl, m->nu);
            Live row{.ctrl = {ctrl.begin(), ctrl.end()}, .bound = std::nullopt, .ratio = std::nullopt};
            if (stepper.has_drive()) {
                if (const auto* hub = stepper.drive().transmission_storage()) {
                    const auto state = hub->state();
                    row.bound = state.range[1];
                    if (state.coefficients.size() == 1) row.ratio = state.coefficients.front();
                }
            }
            live.push_back(std::move(row));
        });
    }
    const auto episode = rig->runtime->seal_episode();
    bt::require(episode.samples.size() == static_cast<std::size_t>(steps), "full retention keeps every interval");
    bt::require(!live.front().ctrl.empty(), "the welded fixture has actuators");
    bt::require(live.front().bound.has_value() && live.front().ratio.has_value(),
        "the welded fixture runs an ideal (non-geometric) hub, so both hub values are recorded");
    for (std::size_t i = 0; i < live.size(); ++i) {
        const auto& sample = *episode.samples.at(i);
        bt::require(sample.interval_id == static_cast<std::int64_t>(i), "samples are in interval order");
        bt::require(same_bits(sample.controls.ctrl, live.at(i).ctrl), "interval " + std::to_string(i) + ": ctrl is the integrator's ctrl");
        bt::require(same_bits(sample.controls.freehub_upper_rad, live.at(i).bound), "interval " + std::to_string(i) + ": freehub bound is the live tendon_range");
        bt::require(same_bits(sample.controls.chain_ratio, live.at(i).ratio), "interval " + std::to_string(i) + ": chain ratio is the live wrap_prm");
    }
    bt::require(std::ranges::any_of(live, [&](const Live& row) { return !same_bits(row.ctrl, live.front().ctrl); }),
        "ctrl varies over the window, so the comparison is not vacuous");
}
} // namespace

int main() { return bt::run(test_samples_record_the_integrator_controls); }
```

Append to `native/tests/reference/GoldenEpisodeDigest.cmake`:

```cmake
bike_add_reference_test(recording_controls_capture ENGINE
  SOURCES test_recording_controls_capture.cpp
  LIBRARIES bike_recording bike_validation
  LABELS recording runtime)
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cmake --preset release
cmake --build build/native-release --target reference.recording_controls_capture
ctest --test-dir build/native-release -R '^reference\.recording_controls_capture$' --no-tests=error --output-on-failure
```

Expected: the build succeeds and the test fails with `interval 0: ctrl is the integrator's ctrl`, because the samples carry an empty `ctrl`.

Stop conditions:
- It instead fails at `the welded fixture runs an ideal (non-geometric) hub…`: the welded fixture has no ideal hub, so this test cannot cover the hub values. Stop and report this. The replacement fixture choice (for example a `geometric`/`ideal` bootstrap capture) is the caller's decision.
- It fails at `the welded fixture has actuators`: report `nu = 0`. Then no fixture-level evidence for `ctrl` exists.
- If `ctrl varies…` is the only failing assertion after Step 4, report it. The bitwise equalities still hold, but they are weak evidence. Do not delete the assertion without the caller's agreement.

- [ ] **Step 3: Add the transmission accessors**

In `native/src/drivetrain/transmission.hpp`, after `[[nodiscard]] double ratio() const noexcept { return state_.ratio; }` (`:192`), add:

```cpp
        // Recording schema 2 (D16): the live model rows the integrator
        // consumes — never state_ mirrors, which a memory-only settlement
        // commit can move ahead of the model (commit(SolvedTransmission&)).
        // tendon_range upper bound of this transmission's tendon:
        [[nodiscard]] double applied_upper_bound() const;
        // wrap_prm driver coefficient (the applied chain ratio) of the
        // single-coefficient ideal hub; empty for the geometric model,
        // whose several coefficients are not a scalar ratio:
        [[nodiscard]] std::optional<double> applied_ratio() const;
```

Add `#include <optional>` to `transmission.hpp` if it is not already included (`grep -n '<optional>' native/src/drivetrain/transmission.hpp`).

In `native/src/drivetrain/transmission.cpp`, after `Transmission::state_into` (ends `:601`), add:

```cpp
    double Transmission::applied_upper_bound() const {
        const auto range = buffer(model_->tendon_range, 2 * model_->ntendon)
                .subspan(2 * static_cast<std::size_t>(tendon_), 2);
        return range[1];
    }

    std::optional<double> Transmission::applied_ratio() const {
        if (geometric_ || coefficients_.size() != 1)
            return std::nullopt;
        return buffer(model_->wrap_prm, model_->nwrap)[static_cast<std::size_t>(coefficients_.front())];
    }
```

- [ ] **Step 4: Sample at the ordering point and carry the value to the sample**

In `native/src/runtime/step.hpp`:
- in `struct RawStep` (`:76-106`), add after `bool full = false;`:

```cpp
    // Recording schema 2 (D16) controls the integrator consumed.
    StepControls controls;
```

- in `struct SolvedInputs`, add directly after `std::array<int, 3> warning_counts{};` (`:447`):

```cpp
        StepControls controls{};
```

- in the private section, add directly after `[[nodiscard]] RawStep publish_raw(SolvedInputs in);` (`:465`):

```cpp
    // The controller outputs the integrator consumes — called between
    // apply_forces and engine::step (see the plan's ordering argument).
    [[nodiscard]] StepControls sample_controls(const mjModel *m, const mjData *d) const;
```

In `native/src/runtime/step.cpp`:
- in `advance_body`, directly after the closing brace of the `if (golden != nullptr) { … }` block that follows `apply_forces` (`:1593-1598`), insert:

```cpp
        // Recording schema 2 (D16): apply_forces has written every ctrl,
        // hub wrap_prm/tendon_range and brake bound this interval uses, and
        // nothing between here and the integrator writes them — sample now.
        StepControls controls = sample_controls(m, d);
```

- in the `solve_tail({…})` call (`:1644-1656`), replace `.warning_counts = warning_counts});` with:

```cpp
                           .warning_counts = warning_counts,
                           .controls = std::move(controls)});
```

- in `publish_raw`, directly after `raw.full = in.full;` (`:2050`), insert `raw.controls = std::move(in.controls);`;
- add after `RawStep PhysicalStep::publish_raw(SolvedInputs in) { … }` ends (`:2064`):

```cpp
StepControls PhysicalStep::sample_controls(const mjModel *m, const mjData *d) const {
        StepControls out;
        out.ctrl = buffer_copy(d->ctrl, m->nu);
        if (stepper_->has_drive()) {
            if (const auto *hub = stepper_->drive().transmission_storage()) {
                out.freehub_upper_rad = hub->applied_upper_bound();
                out.chain_ratio = hub->applied_ratio();
            }
        }
        return out;
}
```

In `native/src/runtime/accounting.cpp`:
- in the `restore` lambda (`:656-670`), after `raw.components = std::move(sample.forces);`, add `raw.controls = std::move(sample.controls);`;
- in the published-row assembly (`:738-750`), after `out.forces = std::move(raw.components);`, add `out.controls = std::move(raw.controls);`.

Stop condition: `-Wmissing-field-initializers` or `-Wmissing-designated-field-initializers` fires on another designated `SolvedInputs`/`RawStep` initializer that this task did not edit. Grep for it with `grep -rn 'SolvedInputs{\|RawStep{' native/src native/tests`. Add `.controls = {}` there. Do not disable the warning.

- [ ] **Step 5: Run the test to verify it passes**

```bash
cmake --build build/native-release --target reference.recording_controls_capture reference.recording_schema_v2
ctest --test-dir build/native-release -R '^reference\.(recording_controls_capture|recording_schema_v2|recording_digest)$' --no-tests=error --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 3`.

Stop condition: a bitwise mismatch means something writes `ctrl`, `tendon_range` or `wrap_prm` after the sampling point. The ordering argument is then wrong. Stop and report the interval and field. Do not move the sampling point without re-deriving the argument from source.

- [ ] **Step 6: Gate B — schema 2 leaves physics and forces bit-identical (D2)**

```bash
cmake --build build/native-release --target bike-ride bike-recording-digest
cmake -DRIDE="$PWD/build/native-release/bin/bike-ride" -DDIGEST="$PWD/build/native-release/bin/bike-recording-digest" \
  -DSOURCE_DIR="$PWD" -DSCRATCH="$PWD/output/gpu_golden/gate-b" -DTRACKS=flat,rough_uphill_extreme \
  -DTABLES=physics,forces -DSUFFIX=.schema1.digest -DKEEP=ON \
  -DEXPECT_DIR="$PWD/tests/fixtures/episode-digest/darwin-arm64" \
  -P native/tests/reference/run_golden_episode_digest.cmake
build/native-release/bin/bike-recording-digest output/gpu_golden/gate-b/flat/recording/*/ | grep -E '^(schema|tables) '
```

Expected:
- `-- flat: match <sha> (wall … s)` and `-- rough_uphill_extreme: match <sha> (wall … s)`;
- then `schema 2` and `tables physics,forces,controls`.

Stop condition: a mismatch is a D2 violation. Schema 2 changed physics or forces. Stop. Do not re-capture anything. Run the per-interval comparison against the Task 2 recording, which is still on disk under `output/gpu_golden/schema1-a`:

```bash
build/native-release/bin/bike-recording-digest output/gpu_golden/gate-b/flat/recording/*/ --tables physics,forces \
  --against output/gpu_golden/schema1-a/flat/recording/*/
```

Report the first differing interval and table.

- [ ] **Step 7: Document schema 2**

In `docs/RECORDING.md` §Data model and type semantics, replace the paragraph that starts `Container schema version is **1**. Research replay contract version is **3**;` (`:133-139`) with:

```markdown
Container schema version is **2**; schema-1 containers still validate and
read (`Recording::schema_version()` reports which). Research replay contract
version is **3**; these version numbers serve different purposes. Core tables
are `runs`, `episodes`, `events`, `field_catalog`, `coordinate_catalog`,
`alias_catalog`, `assets`, `integration_states` and `metrics`, with
`recording_meta` for container identity/codec. The finite physical streams are
`physics`, `forces`, the reviewed `channel_*` tables and, from schema 2,
`controls`. Research uses `commands_requested`, `commands_applied`,
`observations` and `transitions`.

### Schema 2: `controls` and the precision lane

`controls` has one row per physical interval, keyed like `physics` by
`(episode_id, interval_id)`:

| Column | Type | Meaning |
|---|---|---|
| `ctrl` | `DOUBLE[]` (`nu`) | actuator controls the integrator consumed, in model actuator order |
| `freehub_upper_rad` (+`_present`) | `DOUBLE` | live `tendon_range` upper bound of the ideal-hub freehub tendon; absent without an ideal hub |
| `chain_ratio` (+`_present`) | `DOUBLE` | live `wrap_prm` driver coefficient (applied chain ratio) of a non-geometric ideal hub; absent otherwise |

The row is sampled in `PhysicalStep::advance_body` after `apply_forces` and
before the integrator. This is the point after which no controller writes
`ctrl`, `wrap_prm` or `tendon_range` for the interval. The hub values are the
model rows, not the transmission's logical state, which a post-step
settlement can move ahead of the model. To replay one interval, write
`wrap_prm` (and re-run `set_const` only if it changed), the `tendon_range`
upper bound and `ctrl`, then step. Not recorded: brake `dof_frictionloss`
bounds, `qacc_warmstart`, `act`, and the clutch/freewheel tendons.

`manifest.json` of a schema-2 container has a ninth key, `precision`: `f64`,
`f32` or `gpu-f32`. It names the build lane that produced the recording;
schema-1 manifests have no such key. `telemetry` does not join `controls`;
query it with `bike-recording sql`.
```

- [ ] **Step 8: Sweeps over the changed translation units**

```bash
build/native-release/bin/bike-native-checks --kind headers --build build/native-release
build/native-release/bin/bike-native-checks --kind tidy --build build/native-release
build/native-release/bin/bike-native-checks --kind frontends --build build/native-release
build/native-release/bin/bike-native-checks --kind contracts --build build/native-release --repo-root "$PWD"
```

Expected: each exits 0. The stop condition is the same as Task 1 Step 5. `contracts` specifically rejects any `mj_…(` text in the edited `.cpp` files. The new comments use `engine::step` and `set_const` for this reason.

- [ ] **Step 9: Commit**

```bash
git add native/src/drivetrain/transmission.hpp native/src/drivetrain/transmission.cpp \
  native/src/runtime/step.hpp native/src/runtime/step.cpp native/src/runtime/accounting.cpp \
  native/tests/reference/test_recording_controls_capture.cpp native/tests/reference/GoldenEpisodeDigest.cmake docs/RECORDING.md
git commit -m "$(cat <<'EOF'
feat(s0.4): record the controls the integrator consumed

The controls row is sampled after apply_forces and before the integrator,
the last point where controllers write ctrl and the hub tendon rows, and
the hub values come from the model, because a post-step settlement can
move the transmission's logical state ahead of it. Gate B confirms the
schema-2 physics and forces digests equal the schema-1 goldens (D2).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Final schema-2 goldens and the `golden_episode_digest` CTest node

**Files:**
- Create: `tests/fixtures/episode-digest/darwin-arm64/flat.digest`, `tests/fixtures/episode-digest/darwin-arm64/rough_uphill_extreme.digest`
- Modify: `native/tests/reference/GoldenEpisodeDigest.cmake`, `docs/TESTING.md`, `docs/superpowers/plans/2026-11-03-gpu-shared-physics-master.md`
- Generated, not committed: `output/gpu_golden/<track>/recording/<published>/`

**Interfaces:**
- Consumes: the Task 2 driver, the Task 4 recorder.
- Produces: the three-table golden digests, the golden recordings in `output/gpu_golden/`, CTest `golden_episode_digest` (labels `reference;golden;slow`), and the documented D2 command.

**Registration mechanism:** `golden_episode_digest` is registered only if both of these hold:
1. `tests/fixtures/episode-digest/${BIKE_GOLDEN_PLATFORM}/{flat,rough_uphill_extreme}.digest` exist;
2. the build is `CMAKE_BUILD_TYPE=Release` without `NATIVE_SANITIZE`, `NATIVE_RTSAN` or `NATIVE_COVERAGE`.

Otherwise configure prints a `STATUS` line naming the reason, and the node does not exist.

Why this mechanism:
- The fixtures are bit patterns of one platform's f64 Release build. Linux aarch64 has no fixtures yet; the other presets compile different code or instrumentation. On those, the node could only fail or be skipped.
- A configure-time decision keeps `ctest -L slow --no-tests=error` honest on every lane.
- The `STATUS` line makes the absence visible instead of a silent pass.
- It also follows the precedent of platform-gated golden fixtures (`BIKE_GOLDEN_PLATFORM`, F26).

- [ ] **Step 1: Write the failing check (the node does not exist yet)**

```bash
ctest --test-dir build/native-release -R '^golden_episode_digest$' --no-tests=error -N
```

Expected: `No tests were found!!!` and a nonzero exit. This is the red.

- [ ] **Step 2: Record the final goldens into `output/gpu_golden/`**

```bash
cmake --build build/native-release --target bike-ride bike-recording-digest
cmake -DRIDE="$PWD/build/native-release/bin/bike-ride" -DDIGEST="$PWD/build/native-release/bin/bike-recording-digest" \
  -DSOURCE_DIR="$PWD" -DSCRATCH="$PWD/output/gpu_golden" -DTRACKS=flat,rough_uphill_extreme \
  -DCAPTURE_DIR="$PWD/output/gpu_golden" -P native/tests/reference/run_golden_episode_digest.cmake
grep -E '^(schema|tables|intervals|outcome) ' output/gpu_golden/flat.digest output/gpu_golden/rough_uphill_extreme.digest
ls -d output/gpu_golden/flat/recording/*/ output/gpu_golden/rough_uphill_extreme/recording/*/
```

Expected:
- for each track: `schema 2`, `tables physics,forces,controls`, `intervals 24000`, `outcome duration_reached`;
- two published recording directories.

- [ ] **Step 3: Cross-check the final goldens against the schema-1 goldens**

```bash
for t in flat rough_uphill_extreme; do
  build/native-release/bin/bike-recording-digest output/gpu_golden/$t/recording/*/ --tables physics,forces \
    --expect tests/fixtures/episode-digest/darwin-arm64/$t.schema1.digest
done
```

Expected: `match <sha>` twice.

Stop condition: a mismatch here after Task 4 Step 6 matched means the host is non-deterministic across sessions (Task 2 tested runs back to back). Stop and report both `ride.log` `wall_s` values.

- [ ] **Step 4: Commit-stage the fixtures and register the node**

```bash
cp output/gpu_golden/flat.digest output/gpu_golden/rough_uphill_extreme.digest tests/fixtures/episode-digest/darwin-arm64/
wc -l tests/fixtures/episode-digest/darwin-arm64/flat.digest tests/fixtures/episode-digest/darwin-arm64/rough_uphill_extreme.digest
```

Expected: 189 lines each: 9 header lines plus 60 blocks × 3 tables.

Append to `native/tests/reference/GoldenEpisodeDigest.cmake`:

```cmake
# D2/D18 end-to-end digest gate. Registered only where the recorded
# fixtures exist and the build is the plain f64 Release lane they were
# captured on; elsewhere configure says why, so the gate is never a
# silent pass. The fixtures are bit patterns of one platform's build.
set(_digest_fixture_dir "${CMAKE_SOURCE_DIR}/tests/fixtures/episode-digest/${BIKE_GOLDEN_PLATFORM}")
set(_digest_missing "")
foreach(_track flat rough_uphill_extreme)
  if(NOT EXISTS "${_digest_fixture_dir}/${_track}.digest")
    list(APPEND _digest_missing "${_track}.digest")
  endif()
endforeach()
if(_digest_missing)
  message(STATUS "golden_episode_digest: not registered: no ${BIKE_GOLDEN_PLATFORM} fixtures (${_digest_missing}) under ${_digest_fixture_dir}")
elseif(NOT CMAKE_BUILD_TYPE STREQUAL "Release" OR NATIVE_SANITIZE OR NATIVE_RTSAN OR NATIVE_COVERAGE)
  message(STATUS "golden_episode_digest: not registered: fixtures were captured on the plain Release lane "
    "(build type ${CMAKE_BUILD_TYPE}, sanitize=${NATIVE_SANITIZE}, rtsan=${NATIVE_RTSAN}, coverage=${NATIVE_COVERAGE})")
else()
  add_test(NAME golden_episode_digest
    COMMAND "${CMAKE_COMMAND}"
      "-DRIDE=$<TARGET_FILE:bike-ride>"
      "-DDIGEST=$<TARGET_FILE:bike-recording-digest>"
      "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}"
      "-DSCRATCH=${CMAKE_CURRENT_BINARY_DIR}/golden_episode_digest"
      "-DTRACKS=flat,rough_uphill_extreme"
      "-DEXPECT_DIR=${_digest_fixture_dir}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/run_golden_episode_digest.cmake")
  set_tests_properties(golden_episode_digest PROPERTIES
    LABELS "reference;golden;slow" TIMEOUT 3600 RUN_SERIAL TRUE)
endif()
```

- [ ] **Step 5: Run the node to verify it passes**

```bash
cmake --preset release
cmake --build build/native-release --target bike-ride bike-recording-digest
ctest --test-dir build/native-release -R '^golden_episode_digest$' --no-tests=error --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 1`. The output shows `flat: match …` and `rough_uphill_extreme: match …`.

Stop condition: if the configure output contains `golden_episode_digest: not registered`, the platform string is not `darwin-arm64`. Print it with `grep BIKE_GOLDEN_PLATFORM build/native-release/native/tests/reference/support/bike_golden_paths.hpp`. Report it; do not rename fixtures to match.

- [ ] **Step 6: Document the D2 command**

Append to `docs/TESTING.md`:

```markdown
## Golden episode digest (GPU shared physics D2/D18)

`golden_episode_digest` (labels `reference;golden;slow`) records 30 s
`bike-ride` episodes on `flat` and `examples/research/rough_uphill_extreme.toml`
with `viewer_physics_distributed.toml`, `--check-policy diagnostic` and an
empty equilibrium cache. It then compares their `bike-recording-digest`
(physics, forces, controls) with
`tests/fixtures/episode-digest/<platform>/<track>.digest`. A cold cache is
required: a cache hit skips the settle and starts from a different state.
The node exists only where fixtures exist and on the plain Release preset;
configure prints why otherwise. The D2 command, run after every task touching
`native/src/{writers,tyre,contact,drivetrain,physics_core}/`:

    cmake --build build/native-release --target bike-ride bike-recording-digest && \
      ctest --test-dir build/native-release -R '^golden_episode_digest$' --no-tests=error --output-on-failure

A mismatch keeps the recording under
`build/native-release/native/tests/reference/golden_episode_digest/<track>/`.
To name the first differing interval and table, run
`bike-recording-digest <that recording> --against output/gpu_golden/<track>/recording/<published>/`.
The `*.schema1.digest` files are the pre-schema-2 goldens (`physics,forces`)
that proved schema 2 left both tables bit-identical.
```

In `docs/superpowers/plans/2026-11-03-gpu-shared-physics-master.md`, directly under the Global Constraints bullet that begins `- CPU bit identity (D2) is checked with the end-to-end digest from Stage 0`, add:

```markdown
  - The D2 command (Stage 0): `cmake --build build/native-release --target bike-ride bike-recording-digest && ctest --test-dir build/native-release -R '^golden_episode_digest$' --no-tests=error --output-on-failure` (see `docs/TESTING.md` §Golden episode digest).
```

- [ ] **Step 7: Commit**

```bash
git add tests/fixtures/episode-digest/darwin-arm64/flat.digest tests/fixtures/episode-digest/darwin-arm64/rough_uphill_extreme.digest \
  native/tests/reference/GoldenEpisodeDigest.cmake docs/TESTING.md docs/superpowers/plans/2026-11-03-gpu-shared-physics-master.md
git commit -m "$(cat <<'EOF'
test(s0.5): add golden_episode_digest D2 gate

Every later stage must prove CPU output stays bit-identical (D2) against
the D18 goldens. The node records both 30 s episodes from a cold cache and
checks the physics, forces and controls digests. It registers only where
darwin-arm64 Release fixtures exist, so other lanes report its absence
instead of failing or passing silently.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Stage gate

**Files:** none changed, unless a gate failure is fixed. Any fix gets its own commit with the same message conventions.

- [ ] **Step 1: The slow nodes this stage touched**

```bash
ctest --test-dir build/native-release \
  -R '^(golden_episode_digest|reference\.(native_encode_columns|native_recording_retention|recording_platform_compat))$' \
  --no-tests=error --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 4`.

- [ ] **Step 2: The stage completion gate (once)**

```bash
bash tools/run_tests.sh native; echo "exit=$?"
```

Expected: the last line is `exit=0`.

Stop conditions:
- Compare each failure against the documented baseline (AGENTS.md §Verification speed). Use stash+rerun at `HEAD~5` only for a failure the baseline does not explain. Use `git stash push -u -m "s0-gate-<date>"` and capture the SHA — never a bare `git stash`.
- A failure in a file this stage changed is this stage's defect. Fix it, commit, rerun the gate once.
- Do not tick this box until `exit=0`.

- [ ] **Step 3: Report**

Report:
- the commands run and their exit codes;
- the two golden `sha256` lines (`grep '^sha256' tests/fixtures/episode-digest/darwin-arm64/*.digest`);
- the `wall_s` of each golden ride;
- the Task 2 Step 5 warm-cache result;
- every expectation-only test edit made under Task 3 Step 10;
- every unresolved item from §Open risks.

---

## Open risks and stop conditions (collected)

1. **Run-to-run nondeterminism** (Task 2 Step 4, Task 5 Step 3). The headless loop advances in wall-budgeted chunks (`ride.cpp:178`). If physics depends on chunk boundaries, or on threading, the digests differ between runs. Stop; D2 cannot be a digest until this is fixed.
2. **Wall-clock cap** (Task 2 Step 3). If a 30 s distributed-tyre ride takes more than 300 s of wall time, it ends `wall_clock_cap` with a load-dependent extent. Stop; raising the cap for goldens is a user decision.
3. **Cold vs warm equilibrium cache** (Task 2 Step 5). The gate always runs cold. A warm-cache digest difference is expected to be possible (the settle is skipped) and is documented, not fixed.
4. **Exit code 2 under `diagnostic`.** The driver accepts exit 0 or 2. Outcome and extent are pinned by the digest, not by the exit code.
5. **Signed zero / NaN payload.** The digest relies on the reader keeping `-0.0` (asserted in Task 1 and Task 3) and canonicalizes NaN payloads to the quiet NaN. A recorded NaN payload change is therefore invisible to the digest. This is accepted: the reader has the same limit.
6. **Gate B mismatch** (Task 4 Step 6) is a D2 violation. Stop. Never re-capture.
7. **What v2 still does not record** (master-plan R4 candidates for Stage 2's closure test):
   - brake `dof_frictionloss` (`step.cpp:1235`);
   - `qacc_warmstart`;
   - `act` (if `na > 0`);
   - `set_const` side effects beyond `wrap_prm`;
   - the clutch/freewheel tendons (`motor_clutch = true` configs);
   - `eq_active` (mutated only in `legacy/controls.cpp:169-239`);
   - `xfrc_applied`.
   The geometric hub records the bound but not a ratio.
8. **Welded fixture coverage** (Task 4 Step 2). If the welded bootstrap has no ideal hub, or `nu = 0`, or a constant `ctrl` over 40 steps, the runtime test is weaker or cannot run. Stop and ask.
9. **Fixture size** (Task 2 Step 7). The new v1 container may be several MB (bundled model asset). Stop above 16 MiB. The alternative is reusing `tests/fixtures/migration/recording/good`, if the native reader accepts it.
10. **Other tests pinning schema 1** (Task 3 Step 10). Only expectation-only edits are allowed. Anything that changes encoded bytes or physics is a stop.
11. **Profile choice.** `docs/agents/native-engineering.md` §Verify asks for both `native` and `full` on a native physics/realtime change. The master plan's Global Constraints set `native` as the stage gate. This stage changes only recording code and a const sampling copy in the step path, with physics proven unchanged by gate B. Whether `bash tools/run_tests.sh full` must also run is the caller's decision. Ask before skipping it, or run it after Task 6 Step 2 if asked.
12. **Per-step allocation.** `sample_controls` copies `nu` doubles each step, as `advance_body` already does for `qpos`/`qvel`. `advance_body` is not a marked warm-core entry (TESTING.md §R1), so no allocation contract applies. An RTF measurement is out of scope.
13. **Preexisting gate reds.** Compare them with the baseline before treating them as this stage's defects.
14. **Python recording reader.** `src/bike_sim/recording` still accepts only schema 1. It is not in the native gate and is removed at T63. Any Python consumer of new recordings breaks until then.
15. **Analyst surfaces.** `telemetry`, `bike-recording window` and `compare` do not include `controls.*`. Use `bike-recording sql`. Extending them is not Stage 0 scope.
16. **Linux aarch64.** No digest fixtures. The node is not registered there, and configure says so. Capturing them is a later step on that host.

## Facts not verified (no build, no run was allowed while planning)

- The model's `nu`/`na` for the distributed and welded configs. Whether the welded bootstrap fixture runs an ideal non-geometric hub (Task 4 asserts it).
- The wall time of a 30 s distributed-tyre recorded ride, and whether it stays under the 300 s cap.
- Recording sizes: the 30 s goldens, and the 5 ms v1 fixture against the 16 MiB stop.
- Whether two cold-cache runs are bit-identical, and whether warm and cold cache digests differ.
- The `bike-ride` exit code for the 30 s diagnostic rides (0 vs 2), and whether `outcome.reason` is `duration_reached` for both tracks.
- That DuckDB keeps `-0.0` inside `DOUBLE[]` and in `DOUBLE` columns through the reader. RECORDING.md claims it; the plan asserts it.
- That `ctrl` varies within 40 welded steps.
- That GCC/clang-tidy/cppcheck accept the new code as written. Examples: `const auto* found` from `std::ranges::find_if` over `std::array`; `std::stoll` into `std::int64_t` on Linux; std::string_view `operator[]` under `-Wunsafe-buffer-usage`.
- That no other test counts `field_catalog` rows, manifest keys or the stream table set beyond those listed in F29.
- That `catalog_data.inc` line 9140 starts the last raw-literal part (Task 3 Step 4 re-derives it).
- That `reader.cpp` sees `schema_version` through its current includes (Task 3 Step 6 says how to check).
- That `docs/TESTING.md` documents a baseline failure list for the native gate.

## Self-review

- Spec coverage:
  - D18 (two 30 s goldens before the first change): Task 2.
  - D2 (bit identity, digest after every relevant task): gate B in Task 4, the `golden_episode_digest` node and the D2 command in Task 5.
  - D16 (`ctrl`, freehub upper bound, chain ratio; v1 still reads; RECORDING.md): Tasks 3–4.
  - D22 (manifest `precision`): Task 3.
  - R5 (both versions round-trip; validation accepts both): `test_schema_1_container_still_reads_and_digests`, `test_manifest_lanes_are_versioned`.
  - The caller's items A (tool, sha, exclusions, `--tables`, `--expect`, nonzero exit, first differing step/table, goldens, determinism), B (table, ordering argument, version bumps, v1 validates, manifest, docs, gate) and C (v2 re-record, fixtures, CTest with justified registration, D2 command, final gate) map to Tasks 1–2, 3–4 and 5–6.
- Placeholder scan: every code step contains complete code. The commands use the real preset, targets and paths. The `<track>`/`<published>` tokens appear only inside documentation text, where they name directory patterns.
- Type consistency:
  - `DigestTable`, `DigestRequest{tables, episode, block_steps}`, `EpisodeDigest`, `DigestBlock`, `compare_digests` and `default_digest_tables(std::int64_t)` are used with the same names in Tasks 1, 3 and 5.
  - `StepControls{ctrl, freehub_upper_rad, chain_ratio}` is the same in `control.hpp`, the catalog keys, `controls_row`, the digest record and both tests.
  - `tables_for_schema(int)` / `catalog_rows(int)` / `container_schema_version(const Connection&)` match between their declarations and their uses.
