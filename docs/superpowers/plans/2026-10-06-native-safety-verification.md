# Trustworthy Native Verification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:
> executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make native test and analysis success prove complete analysis of the selected binary and sources.

**Architecture:** CMake emits source and dependency context. A Python controller validates selection and runs tools;
thin shell entry points preserve existing CMake targets. A common test loader and profile preflight unify normal,
sanitizer, RTSan, and coverage builds.

**Tech Stack:** uv, Python, pytest, CMake/CTest, clang-tidy, CSA, cppcheck, GCC, LLVM coverage.

**Spec:** [Native safety specification](../specs/2026-10-06-native-safety.md), S1–S2.

## Global Constraints

- Use `uv` for every Python command.
- Retain Python >=3.13, CMake >=3.24, and nanobind >=3.1.0; native platform scope remains Apple arm64/Accelerate.
- Preserve valid-input numerical parity, including operation order, intentional saturation, and documented legacy
  behavior.
- Reject malformed inputs consistently in Python and C++; update both implementations in the same task.
- Compile first-party native code as ISO C++23 with extensions disabled.
- Use the existing MuJoCo 3.12.0 dependency; fail configuration if the error-interception ABI is unavailable.
- Keep alpha analyzer findings report-only; report analyzer execution failures separately from findings.
- Run `native` for native code or shell tooling changes; also run `full` for physics or realtime changes.
- Preserve existing AGENTS.md behavioral-stability requirements.
- Keep allocation, Python boxing, logging, and exception construction outside marked realtime regions.
- Make each task independently reviewable; record exact commands, selected binary, tool health, and test results.

---

## File map

Create `tools/native_checks.py` for manifest/context validation and subprocess orchestration;
`tests/reference/test_native_check_tools.py` for mocked tool controls; `tests/reference/native_loader.py` for extension
selection; `tests/reference/test_native_loader.py` for import-path controls; `native/tests/test_contracts.cpp` for
direct C++ cases; `tools/check_native_contracts.py` for structural policy checks. Modify existing shell wrappers, CMake,
`native/.clang-tidy`, `docs/TESTING.md`, and native test imports. Keep ordinary tool findings distinct from
execution-health failures.

Focused commands, after writing the corresponding tests:

```bash
uv run python -m pytest tests/reference/test_native_check_tools.py -q
uv run python -m pytest tests/reference/test_native_loader.py -q
uv run python -m pytest tests/reference/test_native_contract_checks.py -q
uv run cmake --build native/build
uv run ctest --test-dir native/build --output-on-failure
bash tools/run_tests.sh native
```

Each newly introduced negative control fails before its implementation for the named defect; after implementation every
focused test and mandatory sweep passes. Launcher changes also require `bash tools/run_tests.sh full` to prove native
preflight occurs there.

### V1: Complete source selection and fail-closed tool execution

**Files:** Modify `native/CMakeLists.txt`, `tools/check_native_tidy.sh`, `tools/check_analyzer.sh`,
`tools/check_cppcheck.sh`, `tools/check_native_frontends.sh`, `tools/check_odr.sh`; create `tools/native_checks.py`;
test `tests/reference/test_native_check_tools.py`.

**Interfaces:** `load_entries(build: Path, expected: set[Path]) -> list[dict]`;
`run_sweep(kind: str, build: Path) -> int`; command
`uv run python tools/native_checks.py --kind tidy --build native/build`. CMake emits `native_sources.json` and
`native_check_context.json`. JSON sources contain canonical first-party source paths, target names, and expected compile
contexts; context contains compiler, SDK, include/library paths, Python executable, and tool overrides. Expected sources
come from actual CMake targets, including contract/bench targets when added. A TU identity includes source and
compilation context: reject duplicate identical entries, but analyze every intentionally distinct target/context rather
than dropping a legitimate second compilation of a shared source. Report unique source count and TU count separately.

- [x] Write selection tests with compilation-database entries using both `command` and `arguments`, relative filenames
  resolved against entry `directory`, duplicate identical entries, legitimate shared-source target variants, empty
  selection, missing expected files, and unexpected first-party files. Create temporary fixture files, not references to
  the checkout's current absolute basename.

```python
def test_empty_selection_fails(tmp_path):
    (tmp_path / "compile_commands.json").write_text("[]")
    source = tmp_path / "first_party.cpp"
    source.write_text("int main() { return 0; }\n")
    with pytest.raises(ValueError, match="missing.*first_party"):
        load_entries(tmp_path, {source.resolve()})
```

- [x] Add executable mock tools that return 127 (missing), 2 with error-only output, 139 with crash text, and 0 with a
  warning. Assert all stable-sweep cases fail. Assert alpha warning returns report-only success, but alpha crash fails
  health. Add positive control: one selected TU, tool exit0, no diagnostics succeeds and reports one TU. Run
  `uv run pytest tests/reference/test_native_check_tools.py -q`; expect failures exposing current wrappers before
  implementation.
- [x] Generate source/context manifests during CMake configuration. Make `load_entries` reject missing/duplicate sets
  before spawning tools. Validate every required include/library path; use argument vectors and `subprocess.run`,
  preserving complete logs and return codes. Capture concurrency results individually rather than using an aggregate
  `xargs` result masked by `true`.

```python
completed = subprocess.run(argv, text=True, capture_output=True, check=False)
health = "ok" if completed.returncode == 0 else "failed"
record = {"tool": argv[0], "source": str(source),
          "returncode": completed.returncode, "health": health,
          "stdout": completed.stdout, "stderr": completed.stderr}
if health != "ok":
    failures.append(record)
```

- [x] Route wrappers through the controller using `exec uv run python`; keep their existing build-directory argument and
  CMake target names. Accept `CLANG_TIDY`, `ANALYZER_CLANG`, `CPPCHECK`, and `GXX` overrides, resolving each executable
  before execution. Read SDK/includes from the generated context and remove silent nonexistent-path fallback. Detect
  error diagnostics even if a faulty tool returns0.
- [x] Run focused tests, `bash tools/run_tests.sh native`, and
  `uv run cmake --build native/build --target check_frontends`. Confirm absent build directory fails before selection
  and normal logs show the exact expected set. Update TESTING.md with controller result schema and alpha health
  semantics.
- [x] Review/commit this task only when all negative controls fail, the positive control passes, and the actual selected
  TU set matches CMake. Save a summary JSON under the build directory alongside logs.

### V2: Portable analysis, ISO C++23, and scoped suppression

**Files:** Modify `native/.clang-tidy`, `native/CMakeLists.txt`, `tools/native_checks.py`, `tools/check_analyzer.sh`,
`docs/TESTING.md`; test `tests/reference/test_native_check_tools.py`.

**Interfaces:** V1's manifests/controller; add `--kind headers`, `--kind diagnostic-controls`, and `--kind context` to
`native_checks.py`. Create `native/analysis_suppressions.json` with entries `checker`, `origin`, `message`,
`tool_version`, `reason`, `reproducer`, `remove_when`; an empty initial list is valid. Suppression matching is
diagnostic-specific and reports counts.

- [x] Write a relocated-checkout test that creates the same bad first-party header under two differently named temporary
  roots. Run real installed clang-tidy with integer-division diagnostic enabled; both roots must report it. Add compiler
  controls for discarded nodiscard results, bitwise-instead-of-logical, extra semicolons, signed bounds, and first-party
  use-after-move. Verify actual coverage rather than requiring redundant flag names.

```python
@pytest.mark.parametrize("root_name", ["original-root", "renamed-root"])
def test_header_filter_is_portable(tmp_path, root_name):
    root = tmp_path / root_name
    result = run_header_control(root)
    assert result.returncode != 0
    assert "integer-division" in result.stdout + result.stderr
```

Define `run_header_control(root: Path) -> subprocess.CompletedProcess[str]` in the test: create `native/src/control.hpp`
with `inline double f(int a,int b){return a/b;}`, a TU including it, and a compile DB; invoke the same controller with a
test context enabling that checker.

- [x] Run the controls and capture current relocation failure. Set `CMAKE_CXX_EXTENSIONS OFF` on every first-party
  target and match strict frontend compiler mode. Use a checkout-independent header filter such as
  `(^|/)(native/(src|tests)|tools/proto_native_bench)/`; verify main-file and included-header diagnostics separately.
- [x] Enable first-party `clang-analyzer-cplusplus.Move` in tidy and CSA. Reproduce the documented nanobind false
  positive before creating any suppression. Apply only diagnostic-specific third-party-origin suppression through the
  controller; first-party controls must still fail. If a tool cannot distinguish origin safely, fix the analysis
  invocation instead of globally disabling Move.
- [x] Default libc++ hardening to EXTENSIVE in all configurations that support it. Add a CMake cache option for measured
  FAST comparison; shipped presets use EXTENSIVE. Probe installed support at configure time, keep non-libc++ behavior
  explicit, and correct the ABI/layout comment. Compare compiler mode, `sizeof` representative containers, assertions,
  and native timing in Debug/Release/empty build type.

```cmake
set(CMAKE_CXX_EXTENSIONS OFF)
set(BIKE_LIBCPP_HARDENING "EXTENSIVE" CACHE STRING "libc++ hardening mode")
set_property(CACHE BIKE_LIBCPP_HARDENING PROPERTY STRINGS EXTENSIVE FAST)
```

- [x] Compile each first-party header alone using a generated TU and the target's compile flags. Run
  `uv run python tools/native_checks.py --kind headers --build native/build`, focused diagnostic tests, and the native
  profile. Record hardening timing rather than promising zero cost.
- [x] Respect explicit CMAKE_OSX_SYSROOT/compiler cache values. Resolve a compatible installed SDK only when none is
  supplied, then validate it during configuration and write the actual selection into context. Test a supplied valid
  SDK, a missing SDK, and the current default toolchain; unavailable/incompatible SDKs fail with a diagnostic rather
  than being overwritten. Keep Apple arm64/Accelerate scope explicit; this task does not promise a Linux port.
- [x] Review/commit when relocation controls, Move positive/false-positive controls, standalone headers, and ISO
  configuration checks pass. Every remaining suppression must have the fields defined above and zero unexplained
  first-party matches.

### V3: One selected-artifact loader and real native/full preflight

**Files:** Create `tests/reference/native_loader.py`, `tests/reference/test_native_loader.py`,
`native/tests/test_contracts.cpp`; modify `tests/reference/test_native_*.py` (the concrete native test files are
enumerated by `rg --files tests/reference | rg '/test_native_'`), `tools/run_tests.sh`, `native/CMakeLists.txt`,
`docs/TESTING.md`.

**Interfaces:** `selected_build(environ: Mapping[str,str]) -> Path`; `load_native() -> ModuleType`;
`NATIVE_TEST_BUILD_PATH` is an absolute directory, legacy `NATIVE_TEST_BUILD_DIR` remains `''|asan|rtsan|coverage`.
Different simultaneous selections fail. Native CMake registers `native_contract_tests`; test executable accepts a case
name or runs all registered cases. C++ check helper `require(bool condition, std::string_view message)` throws on
failure and remains active with NDEBUG.

- [x] Test the loader in isolated child Python processes: normal path, custom path, every legacy selector, conflicting
  selectors, preimported wrong module, missing extension, and ambiguous multiple extensions. Use `uv run python`
  children. Add a control specifically covering `test_native_rider_forces.py` import order.

```python
def test_conflicting_build_selectors(tmp_path):
    env = {"NATIVE_TEST_BUILD_PATH": str(tmp_path),
           "NATIVE_TEST_BUILD_DIR": "asan"}
    with pytest.raises(ValueError, match="conflicting"):
        selected_build(env)
```

- [x] Run `uv run pytest tests/reference/test_native_loader.py -q` before implementation; expect the missing loader
  failure. Implement selector resolution, artifact cardinality, and import provenance. Replace every native test's
  manual path edits and skip-based imports with `bike_native = load_native()`.

```python
build = selected_build(os.environ)
module = importlib.import_module("bike_native")
actual = Path(module.__file__).resolve().parent
if actual != build.resolve():
    raise RuntimeError(f"native artifact mismatch: selected={build}, imported={actual}")
return module
```

Before this snippet, insert the verified selected directory, reject a previously imported wrong module, and resolve
exactly one platform extension suffix. Exclude support shared libraries using `importlib.machinery.EXTENSION_SUFFIXES`
and the `bike_native` module stem.

- [x] Add the Release-safe C++ contract executable with a registry and a deliberately failing `require` control run by
  the Python harness; the control must exit nonzero in Debug and Release. Include the executable sources in V1's
  manifest and applicable sanitizer settings. Run CTest from native/full preflight.
- [x] Reuse production core object/static-library targets for direct C++ contracts where their interfaces are
  Python-free; keep binding/Stepper nanobind constructors outside the pure contract executable. F1's config split makes
  writer constructor cases independent of Python. Ensure static core objects are PIC for module linkage and all targets
  inherit the same warning/language/hardening options. If a source is deliberately compiled twice with different
  contexts, list and analyze both in the manifest.
- [x] Factor launcher preflight into one function that configures/builds/sweeps/CTest in the selected directory. Call it
  for `native` and `full`; run pytest only once per chosen profile. Print selected extension and collected native test
  count. For ASan, launch `uv run env DYLD_INSERT_LIBRARIES=<resolved-runtime> ASAN_OPTIONS=<options> python -m pytest`;
  pass variables as argument-vector values, not interpolated executable shell text. Resolve runtime from selected
  compiler; verify it inside the child. Keep Linux preload spelling platform-specific.
- [x] Run normal native/full profiles and selected sanitizer smoke tests. In docs, replace recipes that preload `uv`
  itself or silently load the regular extension. Review/commit only after wrong-build/missing-runtime controls fail and
  rider tests consistently load the requested extension.

### V4: Coverage provenance and enforceable contract checks

**Files:** Modify `tools/native_coverage.sh`, `tools/run_tests.sh`, `native/CMakeLists.txt`, `docs/TESTING.md`; create
`tools/check_native_contracts.py`, `tests/reference/test_native_contract_checks.py`; extend
`tests/reference/test_native_loader.py`.

**Interfaces:** `uv run python tools/check_native_contracts.py --build native/build` is CMake target
`check_native_contracts`. Checks consume V1's manifests and V3's loader provenance. Coverage writes
`coverage_provenance.json` containing selected build, imported extension, profile count, object path, tool versions, and
report paths.

- [x] Add fixture tests for custom coverage directory, zero/two extension matches, no profiles, wrong imported binary,
  missing native test count, and missing contract cases. Use fake LLVM tools for orchestration tests and a real one-file
  coverage control for report correctness.

```python
def test_coverage_requires_one_extension(tmp_path):
    for suffix in ("one.so", "two.so"):
        (tmp_path / f"bike_native.{suffix}").write_bytes(b"control")
    result = run_coverage_control(tmp_path)
    assert result.returncode != 0
    assert "exactly one" in result.stderr
```

Define `run_coverage_control(build: Path)` to invoke the wrapper with `BUILD_DIR` set to the fixture directory, fake
tool paths, and a fixture provenance manifest. Do not execute a fake extension.

- [x] Run focused tests and observe existing unchecked glob/custom-path failures. Make the wrapper pass the absolute
  selected build to tests and obtain the report object from loader provenance; report with that one resolved object. Use
  unique profile directories per run, merge only that run's profiles, and require a nonempty report. Preserve logs when
  a command fails.
- [x] Implement contract checks using structured context and narrow source policy checks: ISO language mode, expected
  hardening, no global Move disable, portable header filter, selected-artifact assertion in native tests, declared
  engine-call wrappers, and manifest membership for benchmark/contracts. Avoid a regex pretending to prove exception
  safety; test behavior in CTest instead.

```python
if context["cxx_extensions"]:
    failures.append("first-party build enables C++ extensions")
if selected != expected:
    failures.append("source manifest differs from selected translation units")
if provenance["imported_extension"] != provenance["report_object"]:
    failures.append("coverage report does not use tested extension")
```

- [x] Wire the target into native/full gates and document which checks are structural versus behavioral. Run focused
  tests, native profile, and custom-directory coverage. Report line/branch deltas for new failure paths without
  inventing a percentage floor.
- [x] Review/commit when coverage provenance is identical end-to-end, all malformed-context controls fail, and new
  contract checks identify actual installed rules. Enable later-task structural checks when their corresponding code
  lands, so this task remains independently green.
