# Baseline, native build and shared infrastructure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T00 through T06 with source-backed parity and independently reviewed test cycles.

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
| T00 | Freeze the actual checkout and a complete removal ledger | None |
| T01 | Capture reference behavior before deleting any oracle | T00 |
| T02 | Introduce a standalone contract-test harness and binary fixtures | T00, T01 |
| T03 | Build the application without finding Python or wheel directories | T00, T02 |
| T04 | Preserve numerical providers and fatal-engine containment | T03 |
| T05 | Create strict lossless data and JSON boundaries | T02, T03 |
| T06 | Introduce filesystem, subprocess and provenance primitives | T03, T05 |

## Task T00: Freeze the actual checkout and a complete removal ledger

**Dependencies:** None; start here.

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `migration/source-manifest.json` |
| Create | `migration/python-removal.json` |
| Create | `migration/test-port-map.json` |
| Create | `migration/baseline.md` |

Read-only source authority (do not delete during the port):

- `AGENTS.md:1-135`
- `pyproject.toml:1-51`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.


**Produces:**

```text
The machine-readable ledger has one row per tracked first-party Python file:
{path, sha256, line_count, owner_task, action, replacement_paths,
 verification_ids, deletion_gate, status}. Status begins "planned".
The test map uses collected pytest node IDs, not just filenames.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Run and retain the baseline/acquisition commands.** Record individual exit codes; the reference can have failures that must be classified rather than silently erased.

```bash
git rev-parse HEAD
git status --porcelain=v1
git ls-files '*.py' '*.pyi' '*.ipynb' 'uv.lock' 'native/*' > migration/tracked-inputs.txt
# Compare this output to the supplied source-manifest.json before any source edits.
test -s migration/tracked-inputs.txt
```

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Use the supplied exhaustive inventory as a starting point, not as a claim that the Repomix snapshot is the complete checkout. Compare every tracked source hash, record added/deleted files, and assign added Python files before proceeding.
  - [ ] 2. Recover the original uv.lock, excluded binary fixtures, matching model assets, and referenced native-safety spec/master from the real checkout or its history. A missing artifact blocks only the tasks consuming it; record exact missing paths and obtain them before those gates.
  - [ ] 3. Create an immutable reference Git tag and a separate reference worktree. Record compiler, SDK, architecture, BLAS, MuJoCo, Python/NumPy/SciPy, DuckDB/PyArrow, seed and relevant environment variables. Do not use the Repomix generation date as a commit identity.
  - [ ] 4. Keep the uploaded source archive read-only. The delivered inventory covers exactly the 1,075 extracted text files; binary content and Git history were not restored.

### Mandatory additional regression cases

- [ ] All 459 supplied Python files have exactly one accountable owner; all newly discovered files also have owners.
- [ ] The duplicate-looking project/tests/reference/test_constraint_mode_search.py is treated as an independent test until content and collection prove otherwise.
- [ ] A dirty working tree is recorded without overwriting or discarding user changes.
- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- migration/source-manifest.json migration/python-removal.json migration/test-port-map.json migration/baseline.md
git commit -m "feat(cpp): T00 freeze the actual checkout and a complete removal ledger"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T01: Capture reference behavior before deleting any oracle

**Dependencies:** T00

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `migration/oracle-manifest.json` |
| Create | `migration/cli-contracts.json` |
| Create | `migration/config-contracts.json` |
| Create named fixture files from recipe (data) | `tests/fixtures/migration/` |
| Create | `tools/migration/capture_reference.py` |

Read-only source authority (do not delete during the port):

- `tests/reference/conftest.py:1-180`
- `tools/run_tests.sh:1-194`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T00` / `migration/api-index.md#t00`: Freeze the actual checkout and a complete removal ledger.

**Produces:**

```text
Temporary capture entrypoint (deleted by T63):
uv run --frozen --group native python tools/migration/capture_reference.py   --output tests/fixtures/migration --manifest migration/oracle-manifest.json
Each fixture records input hashes, baseline commit, provider/tool versions,
comparison class, expected outcome and SHA-256 for every binary payload.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Run and retain the baseline/acquisition commands.** Record individual exit codes; the reference can have failures that must be classified rather than silently erased.

```bash
uv run --frozen --group native python -m pytest --collect-only -q > migration/pytest-collection.txt
bash tools/run_tests.sh quick > migration/baseline-quick.log 2>&1
bash tools/run_tests.sh native > migration/baseline-native.log 2>&1
bash tools/run_tests.sh full > migration/baseline-full.log 2>&1
```

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Run the three baseline commands separately and retain each exit code; do not let one failure suppress later evidence. Run native and full serially. tools/run_tests.sh invokes the same native preflight for BOTH native and full profiles; preserve all nine sweep classes and direct CTest/contract checks.
  - [ ] 2. Capture all nine CLI help surfaces plus parser defaults, positive invocations, invalid combinations and exit codes. Use parser calls for coverage, not regex-extracted option names alone.
  - [ ] 3. Capture resolved configuration, named topology, MJCF, loaded model arrays, complete t=0 bootstrap, mutable model arrays, one-step inputs/outputs, retained intervals, sealed episodes and replay identities. Store raw binary64 bytes plus metadata rather than rounded JSON arrays for numerical oracles.
  - [ ] 4. Use the *_py oracle kernels pinned in tests/reference/conftest.py when freezing the existing bitwise comparisons; also capture separate production-Numba end-to-end cases. Never mix their identities.
  - [ ] 5. Seed fixture groups for welded, distributed, pneumatic, elastic-chain, alternative supports, legacy ride/playground, research delay/noise, reset, invalid input, engine poison, publication failures and replay versions 1/2/3. The owning later task must add its concrete input cases to this same capture tool before porting its source.
  - [ ] 6. Capture old DuckDB recordings and old NPZ/state bundles while the original writer is available. Archive external module:factory policies actually used by users; record their interface and supported replacements without executing untrusted plugins.

### Mandatory additional regression cases

- [ ] A deliberately changed force bit is detected by the fixture comparator.
- [ ] Invalid strict-reference prefixes remain invalid with their original first-failure identity.
- [ ] Fixture generation never labels a diagnostic run physically valid.
- [ ] No performance gate is derived solely from the documented four-step/0.005-second prefix.
- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- migration/oracle-manifest.json migration/cli-contracts.json migration/config-contracts.json tools/migration/capture_reference.py
git commit -m "feat(cpp): T01 capture reference behavior before deleting any oracle"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T02: Introduce a standalone contract-test harness and binary fixtures

**Dependencies:** T00, T01

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Modify | `native/CMakeLists.txt` |
| Create | `native/tests/support/migration_test.hpp` |
| Create | `native/tests/support/migration_test.cpp` |
| Create | `native/tests/migration/CMakeLists.txt` |
| Create | `native/tests/migration/test_t02.cpp` |

Read-only source authority (do not delete during the port):

- `native/tests/test_distance.cpp:1-234`
- `native/tests/test_station.cpp:1-269`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T00` / `migration/api-index.md#t00`: Freeze the actual checkout and a complete removal ledger.
- `T01` / `migration/api-index.md#t01`: Capture reference behavior before deleting any oracle.

**Produces:**

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

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t02.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
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
    const double minus_zero = -0.0;
    bt::require(std::bit_cast<std::uint64_t>(minus_zero) !=
                std::bit_cast<std::uint64_t>(0.0), "signed-zero comparator");
    bt::invalid([] { throw std::invalid_argument("control"); });
    bt::require(bt::text(bt::fixture("harness/utf8.txt")) == "bike\n", "fixture root");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
uv run --frozen --group native cmake -S native -B native/build
uv run --frozen --group native cmake --build native/build --target migration_T02
uv run --frozen --group native ctest --test-dir native/build -R '^migration_T02$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T02` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T02` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Follow the existing standalone require()/TestCase style in native/tests/test_distance.cpp; add no Python-powered test framework or code generator.
  - [ ] 2. Implement explicit little-endian binary64 decoding, length checks, path containment and bounded test payload reads. T05 adds strict JSON manifest/provenance/SHA-256 validation to this same harness before domain goldens are accepted; do not create a second parser or a circular T02/T05 dependency.
  - [ ] 3. Register one executable/CTest name migration_TNN per task. Each task owns native/tests/migration/test_tNN.cpp and can be run alone. Keep existing native_contract_tests, native_distance_tests and native_station_tests intact.
  - [ ] 4. Implement labeled bitwise, bounded-numeric and structural comparisons. There is no global epsilon and no auto-update-goldens flag in the normal test runner.
  - [ ] 5. Before T03, register the harness in native/CMakeLists.txt and use the pinned legacy native build commands below. Compute the fixture root from native/tests/migration/../../.., not a nested PROJECT_SOURCE_DIR which is native in the old build. T03 switches subsequent packages to the root standalone build.

### Mandatory additional regression cases

- [ ] Pass and deliberate failure controls for equality, sign of zero, NaN bit payloads, different shapes and corrupt files.
- [ ] Tests find fixtures from an unrelated working directory.
- [ ] Nonzero test count is mandatory and Release tests do not depend on assert/NDEBUG.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
uv run --frozen --group native cmake --build native/build
uv run --frozen --group native ctest --test-dir native/build -R '^migration_T02$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/CMakeLists.txt native/tests/support/migration_test.hpp native/tests/support/migration_test.cpp native/tests/migration/CMakeLists.txt native/tests/migration/test_t02.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T02 introduce a standalone contract-test harness and binary fixtures"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T03: Build the application without finding Python or wheel directories

**Dependencies:** T00, T02

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `CMakeLists.txt` |
| Modify | `native/CMakeLists.txt` |
| Create | `native/cmake/Dependencies.cmake` |
| Create | `native/cmake/CompilerPolicy.cmake` |
| Create | `native/cmake/Provenance.cmake` |
| Create | `native/CMakePresets.json` |
| Create | `native/dependencies.lock.json` |

Read-only source authority (do not delete during the port):

- `native/CMakeLists.txt:1-978`
- `pyproject.toml:1-51`
- `src/bike_sim/native/__init__.py:1-7`

Declaration-level card: `migration/cards/T03.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T00` / `migration/api-index.md#t00`: Freeze the actual checkout and a complete removal ledger.
- `T02` / `migration/api-index.md#t02`: Introduce a standalone contract-test harness and binary fixtures.

**Produces:**

```bash
cmake -S . -B build/native-release -DCMAKE_BUILD_TYPE=Release   -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=OFF   -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake --build build/native-release
ctest --test-dir build/native-release --output-on-failure
# During migration only, ON adds adapters and oracle tests; OFF is the default.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
cmake -S . -B build/no-python -DBIKE_BUILD_PYTHON_BINDINGS=OFF   -DBIKE_BUILD_GUI=OFF -DCMAKE_DISABLE_FIND_PACKAGE_Python=ON   -DCMAKE_DISABLE_FIND_PACKAGE_Python3=ON -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake --build build/no-python --target native_distance_tests native_station_tests
ctest --test-dir build/no-python -R 'native_(distance|station)_tests' --output-on-failure --no-tests=error
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake -S . -B build/native-release -DCMAKE_BUILD_TYPE=Release -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=OFF -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake --build build/native-release --target migration_T03
ctest --test-dir build/native-release -R '^migration_T03$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T03` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T03` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Keep CMake >=3.24 and C++23. Move the current macOS SDK probing behind APPLE and retain its explicit-SDK failure checks. Do not silently pick a different SDK or compiler.
  - [ ] 2. Resolve MuJoCo headers and shared library from explicit native dependency roots/imported targets. Pin the snapshot-required 3.12.0 binary and its private TLS error ABI by version and hash; never query import mujoco or nanobind when bindings are OFF.
  - [ ] 3. Pin DuckDB 1.5.6 to match the input snapshot for initial compatibility. Choose and lock exact verified archives/commits for toml++, RapidJSON, GLFW, plotting support and native math support at this task; write actual SHA-256 and licenses into dependencies.lock.json. Missing versions are acquisition failures, not permission to substitute latest.
  - [ ] 4. Extract strict compile flags, stack protection, libc++ hardening, no-fast-math and disabled FP contraction into shared target policies applied to every first-party target, including new CLIs/tools/tests. Isolate third-party warning policy.
  - [ ] 5. Allow only explicit dependency acquisition into .deps before configuration. Release/offline configure, build and install must make no network calls and execute no dependency code-generation Python.
  - [ ] 6. At this stage only the already Python-free object/engine/distance/station targets need build with bindings OFF. As T07-T11 land, grow bike_core into the complete native runtime rather than trying to compile coupled files prematurely.
  - [ ] 7. The qualified Python-free source workflow compiles first-party source against pinned, hash-verified native dependency SDKs. It does not claim that every upstream dependency can itself be regenerated without Python. Use verified native SDK artifacts for dependencies such as Cairo when their upstream build tooling needs Python; if an upstream source build is included in the qualified workflow, it must have an independently tested native-only route. SDK acquisition provenance and licenses are mandatory, and no Python executable or package may enter the product or application build closure.

### Mandatory additional regression cases

- [ ] OFF configure logs and compile commands contain no Python include/lib/wheel path.
- [ ] An invalid MuJoCo version or missing ABI fails before building an application.
- [ ] The existing optional extension still builds against the exact same core objects for differential tests.
- [ ] Relocation/RPATH configuration does not embed a developer .venv path.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T03$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- CMakeLists.txt native/CMakeLists.txt native/cmake/Dependencies.cmake native/cmake/CompilerPolicy.cmake native/cmake/Provenance.cmake native/CMakePresets.json native/dependencies.lock.json migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T03 build the application without finding python or wheel directories"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T04: Preserve numerical providers and fatal-engine containment

**Dependencies:** T03

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Modify | `native/src/engine_call.cpp` |
| Modify | `native/src/engine_call.hpp` |
| Modify | `native/src/engine_abi_312.hpp` |
| Modify | `native/src/cblas_abi.hpp` |
| Modify | `native/src/numeric_numpy.hpp` |
| Modify | `native/src/numeric_sincos.hpp` |
| Create | `native/src/engine_identity.hpp` |
| Create | `native/tests/migration/test_t04.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/cblas_abi.hpp:1-127`
- `native/src/engine_abi_312.hpp:1-30`
- `native/src/engine_call.cpp:1-670`
- `native/src/numeric_numpy.hpp:1-277`
- `native/src/numeric_sum.hpp:1-82`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T03` / `migration/api-index.md#t03`: Build the application without finding Python or wheel directories.

**Produces:**

```cpp
namespace engine {
struct Identity { std::string mujoco_version, library_sha256, numeric_provider; };
Identity identity();
void verify_identity(const Identity& required);
}
// Existing ErrorBuffer, EngineFailure and try_* contracts remain authoritative.
// Add native callback provenance; do not add any Python exception dependency.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t04.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
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
    const auto id = engine::identity();
    bt::require(id.mujoco_version == "3.12.0", "pinned engine");
    auto bad = id; bad.mujoco_version = "0.0.0";
    bt::invalid([&] { engine::verify_identity(bad); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T04
ctest --test-dir build/native-release -R '^migration_T04$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T04` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T04` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Preserve the existing guarded engine-call mechanism and its process/thread-local handler restoration. The private MuJoCo ABI is a retained risk with a compatibility probe, not something to remove casually while deleting bindings.
  - [ ] 2. Split vendor/Python callback recognition from the numerical stepper. A standalone runtime accepts the documented native timer or no timer and rejects foreign callbacks; optional adapters can still recognize current Python vendor timers until T63.
  - [ ] 3. Keep the same Accelerate reduction/BLAS paths and numeric_numpy/numeric_sum/numeric_sincos/writers/pyfloat semantics on the reference platform. A filename containing pyfloat does not imply a Python runtime dependency.
  - [ ] 4. Move Python-result boxing entirely outside engine error/RTSan regions. Preserve last committed snapshots after engine failure, poisoned-runtime rejection, reset behavior and complete error-buffer diagnostics.
  - [ ] 5. Run all fatal/foreign-callback/UB controls as child processes. Never replace a handler with an unguarded C++ throw through arbitrary C frames or introduce longjmp across live C++ destructors.

### Mandatory additional regression cases

- [ ] Handler restoration after success/failure; nested call restrictions; independent contexts on different threads.
- [ ] Deliberate engine failure leaves committed-prefix snapshots readable and cannot be resumed as if successful.
- [ ] Same-provider existing bitwise tests remain bitwise, with no wider tolerance.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T04$' --no-tests=error --output-on-failure
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
git add -- native/src/engine_call.cpp native/src/engine_call.hpp native/src/engine_abi_312.hpp native/src/cblas_abi.hpp native/src/numeric_numpy.hpp native/src/numeric_sincos.hpp native/src/engine_identity.hpp native/tests/migration/test_t04.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T04 preserve numerical providers and fatal-engine containment"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T05: Create strict lossless data and JSON boundaries

**Dependencies:** T02, T03

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/io/value.hpp` |
| Create | `native/src/io/json.hpp` |
| Create | `native/src/io/json.cpp` |
| Create | `native/src/io/hash.hpp` |
| Create | `native/src/io/hash.cpp` |
| Create | `native/tests/migration/test_t05.cpp` |
| Create | `native/tests/support/migration_test.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/json_codec.py:1-126`
- `tests/reference/test_recording_values.py:1-121`

Declaration-level card: `migration/cards/T05.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T02` / `migration/api-index.md#t02`: Introduce a standalone contract-test harness and binary fixtures.
- `T03` / `migration/api-index.md#t03`: Build the application without finding Python or wheel directories.

**Produces:**

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

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t05.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
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
    using namespace bike::io;
    bt::invalid([] { parse_json(R"({"x":1,"x":2})"); });
    bt::invalid([] { parse_json("NaN"); });
    bt::require(canonical_json(parse_json("9007199254740993")) ==
                "9007199254740993", "integer precision");
    bt::require(canonical_json(parse_json("-0.0")) == "-0.0", "negative zero");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T05
ctest --test-dir build/native-release -R '^migration_T05$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T05` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T05` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Use a SAX parser with number lexemes retained (RapidJSON numbers-as-strings is the proposed implementation) and detect duplicate object keys explicitly. Avoid routing integers through binary64 or silently converting out-of-range integers to doubles.
  - [ ] 2. Represent absence separately from an explicit null member. Preserve object insertion order where the historical codec exposes it; canonical hashing uses a separately specified deterministic key order.
  - [ ] 3. Port bike_recording.json.v1 exactly: $bike_recording escape objects, float64 nan/+inf/-inf tags, bytes/base64, decimal, date/time/datetime, timedelta and literal reserved-key objects. Numeric physics domains still reject nonfinite inputs unless a named field permits a sentinel.
  - [ ] 4. Give syntax errors source/offset and semantic errors a complete API/field path. Bound parser recursion and allocation sizes with explicit diagnostics; document these as input resource limits rather than silently truncating.
  - [ ] 5. Keep SHA-256 as an integrity/content-identity primitive, not a claim of authenticity. Use known digest vectors and binary fixtures.
  - [ ] 6. Extend T02 fixture loading with the shared strict JSON/SHA-256 manifest checks: validate recorded size/shape/dtype/order/provider/hash before allocating or comparing domain payloads. Corrupt/missing/mismatched fixtures fail; no auto-regeneration or skip-to-pass path.

### Mandatory additional regression cases

- [ ] All tests/reference/test_recording_values.py cases including false/zero/empty/null distinctions.
- [ ] Overflowing finite-double fields, duplicate keys, invalid tag keys, malformed base64 and escaped reserved keys.
- [ ] UTF-8 and locale-independent float round trips; exact large integers in metadata, SQL-domain overflow rejected.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T05$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/io/value.hpp native/src/io/json.hpp native/src/io/json.cpp native/src/io/hash.hpp native/src/io/hash.cpp native/tests/migration/test_t05.cpp native/tests/support/migration_test.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T05 create strict lossless data and json boundaries"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T06: Introduce filesystem, subprocess and provenance primitives

**Dependencies:** T03, T05

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/platform/files.hpp` |
| Create | `native/src/platform/files.cpp` |
| Create | `native/src/platform/process.hpp` |
| Create | `native/src/platform/process.cpp` |
| Create | `native/src/platform/signals.hpp` |
| Create | `native/src/platform/signals.cpp` |
| Create | `native/src/platform/provenance.hpp` |
| Create | `native/src/platform/provenance.cpp` |
| Create | `native/tests/migration/test_t06.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/recording/interrupt.py:1-64`
- `src/bike_sim/recording/storage.py:1-448`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T03` / `migration/api-index.md#t03`: Build the application without finding Python or wheel directories.
- `T05` / `migration/api-index.md#t05`: Create strict lossless data and JSON boundaries.

**Produces:**

```cpp
namespace bike::platform {
struct ProcessResult { int exit_code; std::string out, err; };
ProcessResult run(const std::vector<std::string>& argv, const std::filesystem::path& cwd);
void write_exclusive(const std::filesystem::path& path, std::span<const std::byte> data);
std::filesystem::path executable_path();
}
// No shell interpolation; stdout and stderr are drained concurrently.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t06.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "platform/process.hpp"
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
    const auto p = bike::platform::run({"/usr/bin/printf", "%s", "a b;not-a-command"}, "/");
    bt::require(p.exit_code == 0 && p.out == "a b;not-a-command", "argv is not a shell");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T06
ctest --test-dir build/native-release -R '^migration_T06$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T06` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T06` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Implement native RAII handles, exclusive creation, no-follow regular-file validation, hash/read-after-close and directory synchronization behind platform-specific files. Path checks must validate the opened object, not only a pre-open stat.
  - [ ] 2. Use posix_spawn/exec-style argument vectors on macOS/Linux. Drain both output pipes, impose explicit output bounds for tool output, and kill/reap the entire process group on cancellation. Preserve exit status and signal information.
  - [ ] 3. Signal handlers may only store sig_atomic_t state or notify through an async-signal-safe descriptor. All database/filesystem finalization happens later on the owning thread.
  - [ ] 4. Build identity includes the actual executable/library hashes, compiler/flags, SDK/platform, dependency lock hash and source manifest hash. Separate numerical execution identity from adapter or packaging identity so deleting bindings alone need not invalidate physical compatibility.
  - [ ] 5. Do not replace monotonic clocks with wall time. Keep UTC metadata and monotonic receipt/measurement origins separate.

### Mandatory additional regression cases

- [ ] Spaces, Unicode and shell metacharacters in paths; symlinks; existing empty destination; child failure; timeout; SIGINT twice.
- [ ] Subprocess stderr flood cannot deadlock stdout collection.
- [ ] Moving an installed tree preserves resource lookup through executable-relative paths.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T06$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/platform/files.hpp native/src/platform/files.cpp native/src/platform/process.hpp native/src/platform/process.cpp native/src/platform/signals.hpp native/src/platform/signals.cpp native/src/platform/provenance.hpp native/src/platform/provenance.cpp native/tests/migration/test_t06.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T06 introduce filesystem, subprocess and provenance primitives"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
