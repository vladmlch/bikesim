# Release qualification and final Python removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T61 through T63 with source-backed parity and independently reviewed test cycles.

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
| T61 | Run end-to-end release qualification and close capability parity | T53, T54, T55, T58, T59, T60 |
| T62 | Enforce executable and dependency-level absence of Python | T57, T59, T61 |
| T63 | Delete Python implementations, adapters and packaging only after parity gates | T61, T62 |

## Task T61: Run end-to-end release qualification and close capability parity

**Dependencies:** T53, T54, T55, T58, T59, T60

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `migration/release-evidence.json` |
| Create | `migration/capability-matrix.json` |
| Create | `migration/compatibility-approvals.json` |
| Create | `docs/releases/python-free-qualification.md` |

Read-only source authority (do not delete during the port):

- `AGENTS.md:1-135`
- `tools/run_tests.sh:1-194`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T53` / `migration/api-index.md#t53`: Port integration, frontend, replay and failure-path test suites.
- `T54` / `migration/api-index.md#t54`: Port validation rigs, calibration and physical release evidence.
- `T55` / `migration/api-index.md#t55`: Port performance, diagnostic and native-benchmark developer tools.
- `T58` / `migration/api-index.md#t58`: Qualify Linux headless native support without changing macOS arithmetic silently.
- `T59` / `migration/api-index.md#t59`: Replace Python wheel installers with offline native distributions.
- `T60` / `migration/api-index.md#t60`: Update active documentation and agent contracts to the native workflow.

**Produces:**

```text
A capability row contains:
{mode, source_entrypoint, config_fixture, native_entrypoint, parity_tests,
 platform_results, recording_results, replay_results, status}.
Allowed final status is passed, or explicit-user-approved-breaking-change.
Agent discretion is not user approval.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
build/native-release/bin/bike-test-runner --profile full --build build/native-release
build/native-release/bin/bike-test-ledger --manifest migration/test-port-map.json --require-complete
build/native-release/bin/bike-package-verify --root install-test --no-python
ctest --test-dir build/native-release -L qualification --output-on-failure --no-tests=error
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T61
ctest --test-dir build/native-release -R '^migration_T61$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T61` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T61` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Execute the complete mode/configuration matrix: current native physical subset, pneumatic/distributed/reference tires, elastic chain/clutch/rotor, alternative supports, legacy/nonphysical ride, playground/export/settle, research built-ins/native policy, batch, plotting and analyst/replay tools.
  - [ ] 2. For each required mode compare input resolution, topology, initialization, one-step state/forces, event clocks, valid trajectory windows, failures and artifacts. Cover no-record versus record, reset, terminal/close, explicit state/cache and historical recording profiles.
  - [ ] 3. Compare startup/runtime/finalization performance against the frozen baseline on the same supported machine. Report actual data and regressions; a faster invalid trajectory cannot pass. Do not invent performance targets absent from the source/user requirement.
  - [ ] 4. Require zero unowned Python files, zero unmapped test assertions, complete dependency/CLI maps, no unresolved required capability and all package/platform gates. An unsupported custom Python policy needs its native migration or a written user-approved break.
  - [ ] 5. Run full preflight/tests in a serialized release worktree and retain exact command exits/artifacts. Distinguish existing baseline failures from introduced failures; neither becomes an unqualified green claim.
  - [ ] 6. Have a separate reviewer check evidence against the immutable baseline and fixtures. The reviewer must not regenerate or weaken expected outputs to approve a failing change.

### Mandatory additional regression cases

- [ ] Fresh installed end-to-end run without a prebuilt Python-produced model/cache.
- [ ] All feature rows and test nodes closed with auditable evidence.
- [ ] No native-only path secretly depends on transition adapter/tool/oracle.
- [ ] Physical validity, parity and performance independently reported.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T61$' --no-tests=error --output-on-failure
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
git add -- migration/release-evidence.json migration/capability-matrix.json migration/compatibility-approvals.json docs/releases/python-free-qualification.md migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T61 run end-to-end release qualification and close capability parity"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T62: Enforce executable and dependency-level absence of Python

**Dependencies:** T57, T59, T61

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/tools/python_audit_main.cpp` |
| Create | `migration/no-python-policy.json` |
| Create | `native/tests/package/no_python.cmake` |
| Create | `native/tests/package/audit_negative_controls.cmake` |

Read-only source authority (do not delete during the port):

- `native/CMakeLists.txt:1-978`
- `pyproject.toml:1-51`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T57` / `migration/api-index.md#t57`: Deliver a native test runner, sanitizers and coverage provenance.
- `T59` / `migration/api-index.md#t59`: Replace Python wheel installers with offline native distributions.
- `T61` / `migration/api-index.md#t61`: Run end-to-end release qualification and close capability parity.

**Produces:**

```bash
bike-python-audit --root . --mode source
bike-python-audit --root install-test --mode package
bike-python-audit --root . --mode documentation
// source: first-party executable Python files/invocations/imports/build edges.
// package: binaries, dynamic dependencies, shebangs, bytecode, wheels and launch trace.
// documentation: active command blocks only; historical prose is not banned.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
build/native-release/bin/bike-python-audit --root install-test --mode package
ctest --test-dir build/native-release -R 'no_python|audit_negative' --output-on-failure --no-tests=error
# Run the source mode before T63 as a measured failure; after T63 it must pass.
build/native-release/bin/bike-python-audit --root . --mode source
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T62
ctest --test-dir build/native-release -R '^migration_T62$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T62` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T62` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Enable release enforcement of the bike-python-audit utility implemented in T56. Its source mode becomes mandatory after T63; package mode is mandatory already at T59. There is no forward dependency on a not-yet-built audit executable.
  - [ ] 2. Scan tracked first-party executable files, shebangs, CMake/shell/CI command invocations, Python includes/link targets, dlopen candidates, libpython dependencies, .py/.pyi/.pyc/.pyo/.ipynb, wheels and virtual environments. Document exclusions for immutable archives/historical prose/third-party source that is not built or shipped.
  - [ ] 3. Inspect dynamic dependency graphs with native platform tools (otool/dyld info or readelf/ldd under safe trusted-binary conditions) and compare the allowlist/SBOM. Refuse nanobind/CPython links even if no .py source is present.
  - [ ] 4. Run package smoke tests in an environment genuinely lacking Python and with subprocess execution traced/allowlisted where supported. A PATH trick alone cannot detect an absolute interpreter path; combine filesystem image audit and execution evidence.
  - [ ] 5. Add negative controls: a hidden Python shebang, python3 command substitution, embedded interpreter link, bytecode/wheel, absolute interpreter path and runtime subprocess fallback each fail the proper mode.
  - [ ] 6. The audit must distinguish a Python-free runtime from a Python-free development build and a cleaned first-party repository. Report each claim separately, and never delete useful historical text merely to make grep empty.

### Mandatory additional regression cases

- [ ] All injected interpreter paths and linked libpython detected.
- [ ] Historical mention of Python and a C++ file named pyfloat.hpp are not false failures.
- [ ] Source build/test/check/package workflow and installed runtime independently Python-free.
- [ ] Audit failure lists exact path/edge and remediation owner.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T62$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/tools/python_audit_main.cpp migration/no-python-policy.json native/tests/package/no_python.cmake native/tests/package/audit_negative_controls.cmake migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T62 enforce executable and dependency-level absence of python"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T63: Delete Python implementations, adapters and packaging only after parity gates

**Dependencies:** T61, T62

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `migration/python-removal.json` |
| Create | `migration/test-port-map.json` |
| Create | `migration/final-closure.json` |
| Modify | `native/CMakeLists.txt` |
| Modify | `tools/run_tests.sh` |
| Modify | `README.md` |

Read-only source authority (do not delete during the port):

- `native/CMakeLists.txt:1-978`
- `pyproject.toml:1-51`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T61` / `migration/api-index.md#t61`: Run end-to-end release qualification and close capability parity.
- `T62` / `migration/api-index.md#t62`: Enforce executable and dependency-level absence of Python.

**Produces:**

```text
Final deletion transaction:
1. verify every removal row has passed replacement/compatibility evidence;
2. delete exactly the reviewed first-party paths and binding-only C++ adapters;
3. remove Python build/package/dependency discovery and transition fixtures tools;
4. rebuild, test, package and audit from a new clean native-only worktree.
The immutable reference tag and frozen data fixtures remain available as history/data.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
cmake -S . -B build/final-clean -DBIKE_BUILD_GUI=OFF -DBIKE_DEPENDENCY_ROOT="$PWD/.deps"
cmake --build build/final-clean
ctest --test-dir build/final-clean --output-on-failure --no-tests=error
build/final-clean/bin/bike-python-audit --root . --mode source
build/final-clean/bin/bike-test-runner --profile full --build build/final-clean
cmake --install build/final-clean --prefix install-final
build/final-clean/bin/bike-package-verify --root install-final --no-python
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T63
ctest --test-dir build/native-release -R '^migration_T63$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T63` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T63` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Delete all 459 supplied first-party Python files plus any newly discovered/generated first-party Python introduced during transition, including tools/migration/capture_reference.py, conftest.py and the independent project test. Follow the reviewed ledger, not a blind find-delete over vendor/history directories.
  - [ ] 2. Delete Python-only binding translation units after the typed cores have been extracted. Remove nanobind declarations/includes/build targets and BIKE_BUILD_PYTHON_BINDINGS entirely. Preserve numeric compatibility helpers such as writers/pyfloat.hpp unless equivalent C++ code replaces them with proven parity.
  - [ ] 3. Remove Python-only pyproject/uv lock/groups/wheel installer metadata and virtualenv bootstrap paths from active build/install surfaces; retain unrelated declarative config/assets. Capture the old dependency identity in the immutable baseline before deletion.
  - [ ] 4. Require every removal row to name replacement files, native verification IDs, commit and passing artifacts. Stop deletion when any required feature, historical checked-replay profile or policy replacement lacks approved closure.
  - [ ] 5. Perform clean configure/build/test/install/execute on supported macOS and Linux matrix, not an incremental tree that still contains old extension caches. Run no-Python negative controls and dynamic package dependency audit again.
  - [ ] 6. Write final-closure.json with source revision, package hashes, dependency lock, test/fixture manifests, capability results, platform evidence and explicit compatibility approvals. Release claims must match these artifacts; retain rollback tag/artifacts without shipping Python.

### Mandatory additional regression cases

- [ ] Zero remaining first-party executable Python or nanobind/CPython linkage.
- [ ] All nine installed entrypoints and all native developer workflows work in clean environments.
- [ ] No deleted test/capability lacks an evidence-backed disposition.
- [ ] Immutable reference/golden artifacts are retained without becoming runtime dependencies.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T63$' --no-tests=error --output-on-failure
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
git add -- migration/python-removal.json migration/test-port-map.json migration/final-closure.json native/CMakeLists.txt tools/run_tests.sh README.md
git commit -m "feat(cpp): T63 delete python implementations, adapters and packaging only after parity gates"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
