# Recording, publication, analysis and replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T38 through T46 with source-backed parity and independently reviewed test cycles.

**Architecture:** Preserve the existing native execution path and replace only this subsystem's Python-owned construction, behavior or orchestration. Downstream consumers use the declared owned interfaces; Python reference code stays isolated until final removal.

**Tech Stack:** C++23, CMake >=3.24, pinned native MuJoCo 3.12.0/DuckDB 1.5.6, existing numeric kernels, native IO/GUI/test tools.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`. Also read `migration/interface-contracts.md`, `migration/api-index.md`, and the assigned source cards.

## Global Constraints

- Preserve all required first-party behavior; no Python fallback, embedded interpreter, subprocess interpreter or mandatory Python build/test tool in the final product.
- Preserve existing C++ kernels and the single authoritative physical stepping path; boundary extraction is not permission to change equations or operation order.
- Use C++23 and CMake >=3.24. Retain strict warnings, stack protection, numerical flags, hardening and native safety checks.
- Initially retain MuJoCo 3.12.0 with the verified private TLS error ABI and DuckDB 1.5.6; lock actual native dependency hashes before use.
- Maintain named schema/config/CLI/recording/replay contracts and every supported mode, or obtain explicit user approval for a documented compatibility break.
- Keep existing same-provider bitwise guarantees; no global epsilon, golden regeneration to hide failures, or silent cross-provider equivalence claims.
- Recording remains opt-in, RAM-retained during execution and final-only publication; no streaming redesign, silent history loss or automatic recording from --out.
- Every Python deletion requires a ledger owner, native replacement or approved retirement, assertion/capability coverage and passing evidence; final removal is T63.
- Changes to shared headers/CMake/registries go through one integration owner. Run full native gates serially per machine and record the exact build identity.
- Source examples and API snippets in this plan are proposed implementation contracts, not a claim that code was compiled or tests passed during planning.

---

The named execution sub-skills refer to the downstream agent's environment. When unavailable, follow the same task-by-task red/green/independent-review protocol; no capability is assumed in this handoff. All statuses start planned. The code examples are contracts/regressions to implement and validate, not patches already applied to the application.

## Package map

| Task | Deliverable | Dependencies |
|---|---|---|
| T38 | Port the full recording catalog and schema descriptors | T05, T18 |
| T39 | Port source encoding plans and ingest native column pieces | T38, T11 |
| T40 | Port RAM-only run/episode capture and retained evidence | T37, T39 |
| T41 | Write and validate DuckDB recordings natively | T40, T06 |
| T42 | Preserve exclusive final publication and retry semantics | T41, T06 |
| T43 | Port read-only recording validation and analyst operations | T41, T42 |
| T44 | Port checked replay and versioned legacy compatibility | T43, T34, T36, T24 |
| T45 | Port summary reports and explicit data exports | T43, T11 |
| T46 | Deliver the standalone bike-recording analyst command | T45 |

## Task T38: Port the full recording catalog and schema descriptors

**Dependencies:** T05, T18

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/resources/recording/catalog-v1.json` |
| Create | `native/src/recording/catalog.hpp` |
| Create | `native/src/recording/catalog.cpp` |
| Create | `native/src/recording/coordinates.cpp` |
| Create | `native/src/recording/schema.cpp` |
| Create | `native/tests/migration/test_t38.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/catalog.py:1-1699`
- `src/bike_sim/recording/contracts.py:1-253`
- `src/bike_sim/recording/schema.py:1-214`
- `src/bike_sim/sim/ride/telemetry_v2.py:1-73`

Declaration-level card: `migration/cards/T38.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T05` / `migration/api-index.md#t05`: Create strict lossless data and JSON boundaries.
- `T18` / `migration/api-index.md#t18`: Resolve model names into typed native writer/runtime configuration.

**Produces:**

```cpp
namespace bike::recording {
struct Catalog;
Catalog build_catalog(const mjModel& model);
bike::io::Value describe_catalog(const Catalog& catalog);
}
// Container schema=1; research replay contract=3. These are separate version axes.
// Canonical schema descriptors include units, frame, phase, optionality and children.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t38.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "recording/catalog.hpp"
#include "model/builder.hpp"
#include "io/json.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    auto model = bike::model::compile_mjcf(
      bt::text(bt::fixture("model/welded.xml")), bt::fixture("model/assets"));
    const auto catalog = bike::recording::build_catalog(*model);
    bt::golden_text("recording/catalog_welded.json",
      bike::io::canonical_json(bike::recording::describe_catalog(catalog)));
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T38
ctest --test-dir build/native-release -R '^migration_T38$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T38` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T38` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Transcribe all FieldSpec/TableSpec/AliasSpec descriptors from the 1,699-line catalog.py into a single native-consumed canonical descriptor source. Preserve every canonical key and exact SQL type/units/frame/phase/source/measurement definition.
  - [ ] 2. Port dynamic coordinate construction for scalar/free/ball joints, qpos versus DOF offsets and mixed units. Do not assume nq==nv outside the explicit planar solver.
  - [ ] 3. Port metadata/research table schemas, parent presence/validity masks, telemetry and legacy_telemetry views. Component-force rows must not multiply base telemetry interval rows.
  - [ ] 4. Reject unknown source fields rather than infer units or silently omit evidence. Alias catalogs are projections, not duplicated retained data.
  - [ ] 5. Ship the descriptor as a versioned installed resource or compiled native asset; no Python build-time schema generation may remain in the final tree.

### Mandatory additional regression cases

- [ ] Exact field/alias/coordinate inventory and column type comparison against the reference.
- [ ] Optional nested structs and arrays, free/ball coordinate metadata.
- [ ] Zero duplicated canonical keys and no unclassified units.
- [ ] New native sample channels cannot bypass schema review.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T38$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/resources/recording/catalog-v1.json native/src/recording/catalog.hpp native/src/recording/catalog.cpp native/src/recording/coordinates.cpp native/src/recording/schema.cpp native/tests/migration/test_t38.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T38 port the full recording catalog and schema descriptors"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T39: Port source encoding plans and ingest native column pieces

**Dependencies:** T38, T11

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/recording/encoding.hpp` |
| Create | `native/src/recording/encoding.cpp` |
| Create | `native/src/recording/plan.cpp` |
| Create | `native/src/recording/duckdb_chunks.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/runtime/encode.hpp:1-86`
- `src/bike_sim/recording/encoding.py:1-425`
- `src/bike_sim/recording/native_encode.py:1-286`

Declaration-level card: `migration/cards/T39.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T38` / `migration/api-index.md#t38`: Port the full recording catalog and schema descriptors.
- `T11` / `migration/api-index.md#t11`: Separate native samples and column pieces from their Python boxers.

**Produces:**

```cpp
namespace bike::recording {
runtime::EncodePlan compile_plan(const Catalog& catalog, std::string_view table);
void validate_pieces(const runtime::EncodedTable& table, const Catalog& catalog,
                     std::string_view table_name);
}
// The existing Piece is the source of typed values, offsets and validity.
// DuckDB chunks are bounded publication-time batches, not retained duplicate histories.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R migration_T39 --output-on-failure --no-tests=error
uv run --frozen --group native python -m pytest tests/reference/test_native_encode_columns.py tests/reference/test_recording_values.py tests/reference/test_recording_storage.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T39
ctest --test-dir build/native-release -R '^migration_T39$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T39` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T39` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port native_encode.py plan compilation, piece slicing/fragmentation and schema.py type construction. Reuse encode_table_native rather than rebuild rows into generic maps.
  - [ ] 2. Convert flat/nested Piece buffers to DuckDB data chunks through the native C API, including structs, lists, maps, offsets, validity and parent masks. This removes the PyArrow runtime requirement.
  - [ ] 3. Keep a native slow/reference encoder for source kinds without an optimized plan. Unsupported plan variants must fall back to this C++ path or fail explicitly; never disappear silently.
  - [ ] 4. Preserve booleans, integer widths, strings, NaN/infinities, negative zero, absent/null, empty containers and binary64 fidelity. Compare logical cells and raw float bits, not nondeterministic DB file bytes.
  - [ ] 5. Validate dimensions/offset arithmetic before access and ensure bounded chunks split identically at every cut position.

### Mandatory additional regression cases

- [ ] All native encoder differential cases and old Python non-native-source encoder cases.
- [ ] Chunk widths 0/1/boundary/full and nested null parents.
- [ ] Moves do not invalidate Piece member pointers.
- [ ] No PyArrow/Arrow-Python bridge is involved in ingestion.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T39$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/recording/encoding.hpp native/src/recording/encoding.cpp native/src/recording/plan.cpp native/src/recording/duckdb_chunks.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T39 port source encoding plans and ingest native column pieces"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T40: Port RAM-only run/episode capture and retained evidence

**Dependencies:** T37, T39

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/recording/contracts.hpp` |
| Create | `native/src/recording/capture.hpp` |
| Create | `native/src/recording/capture.cpp` |
| Create | `native/src/recording/physical_source.cpp` |
| Create | `native/src/recording/research_source.cpp` |
| Create | `native/src/recording/events.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/assets.py:1-89`
- `src/bike_sim/recording/capture.py:1-221`
- `src/bike_sim/recording/native.py:1-370`
- `src/bike_sim/recording/research.py:1-191`
- `src/bike_sim/sim/research/recording_lifecycle.py:1-199`
- `src/bike_sim/sim/ride/physical_recorder.py:1-166`
- `src/bike_sim/sim/ride/recorder.py:1-505`

Declaration-level card: `migration/cards/T40.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T37` / `migration/api-index.md#t37`: Port research metrics, validity and experiment evidence.
- `T39` / `migration/api-index.md#t39`: Port source encoding plans and ingest native column pieces.

**Produces:**

```cpp
namespace bike::recording {
class RunCapture; // owns metadata, consecutive episodes and immutable evidence
// append/seal do not write files; finalize is supplied by T42.
struct RunMetadata { std::string run_uuid, started_utc; double wall_origin_s; };
}
// Native generation and run episode_id are different identifiers.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R migration_T40 --output-on-failure --no-tests=error
uv run --frozen --group native python -m pytest tests/reference/test_recording_capture.py tests/reference/test_native_recording_retention.py tests/reference/test_acceptance_interval_capture.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T40
ctest --test-dir build/native-release -R '^migration_T40$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T40` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T40` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port RunCapture/EpisodeJournal and owned physical/research evidence. --record is the only opt-in flag; --out alone never enables retention/publication.
  - [ ] 2. Keep all accounted intervals and reset-separated episodes in RAM until launch finalization. Off mode retains only required live state and metrics, not optional growing histories.
  - [ ] 3. Retain measurement freshness/source metadata when expensive checks are decimated. Full recording is every accounted interval, not every measurement interval.
  - [ ] 4. Port ordinary versus research event-kind differences, operator receipt/processed-step clocks, requested/applied/control-change streams and derived-transition reconstruction.
  - [ ] 5. Sealing preserves all owned evidence after runtime reset/close. A terminal viewer frame seals in RAM and holds; it does not publish before window close.
  - [ ] 6. Do not introduce streaming or background database writes as a migration shortcut. Memory exhaustion remains a documented limitation of the current contract.

### Mandatory additional regression cases

- [ ] Recording on/off numerical passivity for fixed step inputs and nonbinding wall caps.
- [ ] Two resets produce consecutive episode IDs with independent zero-time rows.
- [ ] Off mode creates no output root or optional telemetry files.
- [ ] Failed research window retains its physical suffix without a fabricated transition.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T40$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/recording/contracts.hpp native/src/recording/capture.hpp native/src/recording/capture.cpp native/src/recording/physical_source.cpp native/src/recording/research_source.cpp native/src/recording/events.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T40 port ram-only run/episode capture and retained evidence"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T41: Write and validate DuckDB recordings natively

**Dependencies:** T40, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/recording/database.hpp` |
| Create | `native/src/recording/database.cpp` |
| Create | `native/src/recording/database_validate.cpp` |
| Create | `native/src/recording/assets.cpp` |
| Create | `native/tests/migration/test_t41.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/storage.py:1-448`

Declaration-level card: `migration/cards/T41.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T40` / `migration/api-index.md#t40`: Port RAM-only run/episode capture and retained evidence.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```cpp
namespace bike::recording {
void write_database(const std::filesystem::path& file, const RunCapture& capture);
void validate_database(const std::filesystem::path& file);
}
// Database write/validation use the pinned native DuckDB library, no external
// executable, Python module, network extension download or automatic spill design change.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t41.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "recording/database.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    bike::recording::validate_database(bt::fixture("recording/good/run.duckdb"));
    bt::invalid([] { bike::recording::validate_database(bt::fixture("recording/bad_presence/run.duckdb")); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T41
ctest --test-dir build/native-release -R '^migration_T41$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T41` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T41` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port storage.py metadata/catalog/assets/metrics/streams and per-episode integrity checks exactly. Use one transaction for creation, chunk ingestion, relational/presence validation and commit.
  - [ ] 2. Use RAII for database/config/connection/query/appender/data-chunk handles. Check create/append/flush/destroy/commit errors and retain diagnostics even when cleanup also fails.
  - [ ] 3. After successful commit, CHECKPOINT and close every handle before hashing. WAL or a live connection is not a finalized payload.
  - [ ] 4. Validate primary/composite keys, episode extents, observation/request/transition references, integration-state widths, required stream presence and all parent/child masks.
  - [ ] 5. Make zero-episode and terminal-prefix cases explicit and match old behavior. Use reference recordings as reader/writer compatibility fixtures.

### Mandatory additional regression cases

- [ ] Old writer -> new reader; new writer -> old reader during migration.
- [ ] Injected append/flush/commit/checkpoint failures produce no successful publication.
- [ ] Exact cell types and binary64 values across all tables.
- [ ] Foreign key-like relationships and missing/duplicate interval rows reject.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T41$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/recording/database.hpp native/src/recording/database.cpp native/src/recording/database_validate.cpp native/src/recording/assets.cpp native/tests/migration/test_t41.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T41 write and validate duckdb recordings natively"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T42: Preserve exclusive final publication and retry semantics

**Dependencies:** T41, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/recording/publish.hpp` |
| Create | `native/src/recording/publish.cpp` |
| Create | `native/src/recording/finalize.cpp` |
| Create | `native/src/recording/interrupt.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/interrupt.py:1-64`
- `src/bike_sim/recording/storage.py:1-448`
- `src/bike_sim/sim/ride/async_close.py:1-289`

Declaration-level card: `migration/cards/T42.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T41` / `migration/api-index.md#t41`: Write and validate DuckDB recordings natively.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```cpp
namespace bike::recording {
struct Publication { std::filesystem::path directory; std::string report_status; };
Publication finalize(RunCapture& capture, const std::filesystem::path& output_root);
}
// Successful repeated finalize returns its original path. Failed publication
// keeps the capture retryable and chooses a fresh exclusive destination.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R migration_T42 --output-on-failure --no-tests=error
uv run --frozen --group native python -m pytest tests/reference/test_recording_publication.py tests/reference/test_research_recording_lifecycle.py tests/test_async_close.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T42
ctest --test-dir build/native-release -R '^migration_T42$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T42` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T42` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port the exclusive staging/destination/hard-link publication protocol in storage.write_recording. Hash the closed DB and successful report; publish manifest.json last as the sole sealing marker.
  - [ ] 2. Never adopt or overwrite an existing directory, including a pre-existing empty one. Validate opened regular files, reject symlink payloads and use descriptor-based/no-follow operations to avoid check/use races.
  - [ ] 3. A report failure may publish a valid DB with report_status=failed and explicit error. A DB/payload/manifest failure leaves an unsealed destination and the original RAM capture intact.
  - [ ] 4. Retry must not advance the simulator or discard sealed episodes. A second successful finalize returns the exact original path.
  - [ ] 5. Repeated SIGINT defers to the owning finalizer; signal handlers do not allocate, log, query DuckDB or perform filesystem publication. Preserve the primary simulation/policy exception plus secondary publication errors.
  - [ ] 6. Test actual crash points in subprocesses. The contract is atomic recognizable publication, not guaranteed recovery of the entire RAM-only launch after SIGKILL/power loss.

### Mandatory additional regression cases

- [ ] Failure injected at every write/link/hash/commit/checkpoint/report/manifest stage.
- [ ] Two concurrent finalizers collide safely without shared ownership.
- [ ] Repeated SIGINT, existing/symlink destination and explicit retry.
- [ ] Published manifest hashes match closed payloads; no sealed partial DB.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T42$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/recording/publish.hpp native/src/recording/publish.cpp native/src/recording/finalize.cpp native/src/recording/interrupt.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T42 preserve exclusive final publication and retry semantics"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T43: Port read-only recording validation and analyst operations

**Dependencies:** T41, T42

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/recording/reader.hpp` |
| Create | `native/src/recording/reader.cpp` |
| Create | `native/src/recording/analysis.cpp` |
| Create | `native/src/recording/query.cpp` |
| Create | `native/tests/migration/test_t43.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/__init__.py:1-12`
- `src/bike_sim/recording/analysis.py:1-378`
- `src/bike_sim/recording/reader.py:1-170`

Declaration-level card: `migration/cards/T43.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T41` / `migration/api-index.md#t41`: Write and validate DuckDB recordings natively.
- `T42` / `migration/api-index.md#t42`: Preserve exclusive final publication and retry semantics.

**Produces:**

```cpp
namespace bike::recording {
class Recording;
std::unique_ptr<Recording> open_recording(const std::filesystem::path& directory);
bike::io::Value overview(const Recording& recording);
bike::io::Value query(const Recording& recording, std::string_view sql,
                     std::uint64_t row_limit);
}
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t43.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "recording/reader.hpp"
#include "io/json.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    auto recording = bike::recording::open_recording(bt::fixture("recording/good"));
    bt::golden_text("recording/good/overview.json",
      bike::io::canonical_json(bike::recording::overview(*recording)));
    bt::invalid([] { bike::recording::open_recording(bt::fixture("recording/no_manifest")); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T43
ctest --test-dir build/native-release -R '^migration_T43$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T43` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T43` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port reader.py manifest/schema/hash/presence checks and analysis.py overview/fields/events/window/compare/statistics/selection semantics.
  - [ ] 2. Read-only operations must not construct a simulator or load a recorded policy. Disable automatic extension acquisition and avoid requiring graphics libraries.
  - [ ] 3. Preserve numeric/structural comparison distinction, episode selection, canonical/alias field resolution, row limits, time windows, mismatch counts and tagged JSON.
  - [ ] 4. Quote identifiers and bind values rather than interpolate untrusted field names as SQL. Check reader-only query handling, resource limits and existing CLI query semantics explicitly.
  - [ ] 5. Retain invalid-run facts in analysis results; opening or comparing a diagnostic recording never certifies it physically valid.

### Mandatory additional regression cases

- [ ] Missing manifest, wrong hashes, symlink payloads, unknown schema and wrong presence masks.
- [ ] Read-only mutation attempts and invalid field selectors.
- [ ] Zero/negative/oversized limits, mismatched episodes and NaN/signed-zero comparison.
- [ ] Bounded output even for large recordings.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T43$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/recording/reader.hpp native/src/recording/reader.cpp native/src/recording/analysis.cpp native/src/recording/query.cpp native/tests/migration/test_t43.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T43 port read-only recording validation and analyst operations"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T44: Port checked replay and versioned legacy compatibility

**Dependencies:** T43, T34, T36, T24

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/replay/input.hpp` |
| Create | `native/src/replay/input.cpp` |
| Create | `native/src/replay/session.hpp` |
| Create | `native/src/replay/session.cpp` |
| Create | `native/src/replay/legacy.cpp` |
| Create | `native/src/replay/playback.cpp` |
| Create | `native/tests/migration/test_t44.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/replay_input.py:1-231`
- `src/bike_sim/sim/playback.py:1-94`
- `src/bike_sim/sim/research/playback.py:1-112`
- `src/bike_sim/sim/research/replay.py:1-319`
- `src/bike_sim/sim/research/replay_session.py:1-250`

Declaration-level card: `migration/cards/T44.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T43` / `migration/api-index.md#t43`: Port read-only recording validation and analyst operations.
- `T34` / `migration/api-index.md#t34`: Construct research environments, programs and sensor tapes natively.
- `T36` / `migration/api-index.md#t36`: Port policy sessions, pacing boundaries and lifecycle coordination.
- `T24` / `migration/api-index.md#t24`: Replace Python equilibrium cache/state files and retain old readers.

**Produces:**

```cpp
namespace bike::replay {
struct ReplayResult { bool matched; bool physically_valid; std::string reason; };
ReplayResult check(const std::filesystem::path& recording, std::int64_t episode);
}
// Playback of stored states and checked re-simulation are separate operations.
// A historical Python execution identity is not silently relabeled as native.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t44.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "replay/session.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const auto good = bike::replay::check(bt::fixture("replay/native_v3"), 0);
    bt::require(good.matched, "recorded native execution reproduced");
    bt::invalid([] { bike::replay::check(bt::fixture("replay/tampered"), 0); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T44
ctest --test-dir build/native-release -R '^migration_T44$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T44` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T44` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port LegacyReplayInput/DuckDBReplayInput, recipe/asset/runtime validation, integration-state reconstruction, command scheduling and ReplaySession pacing/acknowledgment.
  - [ ] 2. Preserve container schema 1 versus replay contracts 1/2/3, legacy backend identity, execution contract and separate observation/transition comparisons. Read all supported legacy formats.
  - [ ] 3. For historical Python-backed recordings, use the corresponding ported legacy/native arithmetic only after dedicated compatibility fixtures certify that execution profile. Do not rewrite manifests or claim checked equivalence from mere successful playback.
  - [ ] 4. When exact historical execution cannot be certified, return an explicit incompatible-execution result for checked replay while retaining validated stored-state playback/export. Full Python deletion remains blocked for a required checked-replay profile until certified or the user explicitly approves that compatibility break.
  - [ ] 5. Never execute recorded module:factory strings. Inputs are recorded applied/requested controls and immutable assets. The same stop/yield/close boundaries apply as live research.
  - [ ] 6. Report first mismatch field/step/max error with the recorded validity state. Do not relax tolerances to make a long diverging replay green.

### Mandatory additional regression cases

- [ ] Legacy v1, recorded-backend v2 and DuckDB v3, including tampered identities.
- [ ] No policy import; no extra command on viewer close.
- [ ] Mismatched initial state, missing assets, wrong step extent and delayed control.
- [ ] Playback quaternion/loop/speed behavior and checked replay remain distinct.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T44$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/replay/input.hpp native/src/replay/input.cpp native/src/replay/session.hpp native/src/replay/session.cpp native/src/replay/legacy.cpp native/src/replay/playback.cpp native/tests/migration/test_t44.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T44 port checked replay and versioned legacy compatibility"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T45: Port summary reports and explicit data exports

**Dependencies:** T43, T11

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/recording/report.hpp` |
| Create | `native/src/recording/report.cpp` |
| Create | `native/src/recording/export.hpp` |
| Create | `native/src/recording/export.cpp` |
| Create | `native/resources/reports/summary.md.in` |
| Create | `native/tests/migration/test_t45.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/cli/recording.py:1-132`
- `src/bike_sim/native/recording.py:1-229`
- `src/bike_sim/recording/report.py:1-155`

Declaration-level card: `migration/cards/T45.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T43` / `migration/api-index.md#t43`: Port read-only recording validation and analyst operations.
- `T11` / `migration/api-index.md#t11`: Separate native samples and column pieces from their Python boxers.

**Produces:**

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

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t45.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "recording/report.hpp"
#include "recording/reader.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    auto r = bike::recording::open_recording(bt::fixture("recording/good"));
    bt::golden_text("recording/good/summary.md", bike::recording::render_summary(*r));
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T45
ctest --test-dir build/native-release -R '^migration_T45$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T45` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T45` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port report.py query/formatting/summary content and native/recording.py explicit legacy export metadata/streams.
  - [ ] 2. Keep summary generation independent from the running simulator and use only validated recording evidence. Distinguish invalid/diagnostic outcomes plainly.
  - [ ] 3. Implement CSV and JSONL exactly as cli/recording.py: exclusive output creation, required episode, selected table, default limit=None, fetch batches of 4096, tagged JSON for structured/null/nonfinite CSV cells and whole JSONL rows. Do not introduce Parquet or Arrow IPC as a supposed existing requirement.
  - [ ] 4. Port fixed legacy file naming, manifest and stream semantics. Exports are explicit commands, not new automatic output on every run.
  - [ ] 5. Test report exceptions so T42 can publish a DB with report_status=failed as specified.

### Mandatory additional regression cases

- [ ] Summary text headings/facts and invalid prefix descriptions.
- [ ] CSV quoting/newlines/UTF-8 and JSONL tagged values retain integer precision/null/nonfinite behavior.
- [ ] Explicit legacy bundle loads in old reader during transition.
- [ ] Export path errors never mutate the source recording.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T45$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/recording/report.hpp native/src/recording/report.cpp native/src/recording/export.hpp native/src/recording/export.cpp native/resources/reports/summary.md.in native/tests/migration/test_t45.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T45 port summary reports and explicit data exports"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T46: Deliver the standalone bike-recording analyst command

**Dependencies:** T45

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/cli/recording.cpp` |
| Create | `native/src/cli/recording_main.cpp` |
| Create | `native/tests/cli/recording.cmake` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/cli/recording.py:1-132`

Declaration-level card: `migration/cards/T46.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T45` / `migration/api-index.md#t45`: Port summary reports and explicit data exports.

**Produces:**

```text
int bike_recording_main(std::span<const std::string_view> arguments);
// Keep all source parser subcommands/options, selectors and tagged-JSON outputs.
// Output paths and table/query values are passed as arguments, never shell fragments.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
build/native-release/bin/bike-recording --help
build/native-release/bin/bike-recording overview tests/fixtures/migration/recording/good --json
ctest --test-dir build/native-release -R migration_T46 --output-on-failure --no-tests=error
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T46
ctest --test-dir build/native-release -R '^migration_T46$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T46` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T46` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port cli/recording.py parser/dispatch exactly from the captured T01 contract: overview/fields/events/window/sql/compare/export and the actual source spelling of each subcommand.
  - [ ] 2. Keep bounded row limits, episode selectors, output format choices, machine-readable JSON tagging, stdout/stderr separation and exit behavior.
  - [ ] 3. Use the native reader/catalog/report modules. Verify the command works without MuJoCo initialization, Python, a display or recorded plugins.
  - [ ] 4. Install help/man/examples alongside the binary and maintain script-compatible command names.
  - [ ] 5. Run old CLI expected-output tests against the new executable through a subprocess harness, then preserve their cases as native/CMake tests.

### Mandatory additional regression cases

- [ ] Every captured parser flag/default and invalid combination.
- [ ] Read-only analyst operations do not load simulator/policy code.
- [ ] Invalid manifest/query/field and export failures use stable nonzero exits.
- [ ] Existing analyst CLI/report tests have mapped native replacements.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T46$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/cli/recording.cpp native/src/cli/recording_main.cpp native/tests/cli/recording.cmake migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T46 deliver the standalone bike-recording analyst command"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
