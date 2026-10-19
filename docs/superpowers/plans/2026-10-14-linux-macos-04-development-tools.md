# 04 - Development checks, sanitizers, and coverage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Make the existing strict native development gate run truthfully on both operating systems.

**Architecture:** Version build context, retain target-derived translation-unit selection, and probe toolchain capabilities explicitly. Keep negative controls and runtime instrumentation attached to the selected build instead of inferring platform support from filenames.

**Tech Stack:** Python/uv, NumPy, MuJoCo 3.12.0, nanobind, CMake, C++23, Accelerate on macOS, the selected NumPy OpenBLAS provider on Linux, pytest, CTest, LLVM/GCC analysis tools.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`. Read the source audit and master plan as well as this file.

## Global Constraints

- Add Linux support; retain macOS support and existing macOS behavior.
- Keep Python >=3.13, CMake >=3.27, and strict ISO C++23 without compiler language extensions.
- Keep MuJoCo exactly 3.12.0 for native execution until its private error ABI is deliberately migrated in a separate change.
- Preserve the macOS Accelerate backend, SDK override/fallback behavior, mjpython launcher, and current application commands.
- Preserve same-environment native/Python bitwise tests, exception safety, source/artifact identity, and fail-closed verification.
- Do not promise bitwise-identical results between different operating systems, CPUs, BLAS implementations, or dependency builds.
- Do not change physics, fixed integration periods, controller behavior, recording opt-in/final-only semantics, or replay tolerances for this port.
- Use uv for project Python environments. Never silently regenerate the production lock or upgrade dependencies during a build.
- No implicit native compilation on application import; no silent native-to-Python fallback; no required GUI for headless execution.
- Keep quick/native/full profile meanings; missing required analysis tools remain errors, not successful skips.
- Treat the supplied Repomix export as a source snapshot, not a complete Git checkout or a runnable distribution.
- Native builds are the initial delivery target. Cross-compilation, universal binaries, Windows, and musl are outside this change.

---

## File map and prerequisites

T13 is integrated with T10 so the new context writer and reader land together. T14 follows T13. T15-T17 consume the T09 policy and T13 context. Most changes belong in existing tools/native_checks.py and its existing fixture/test file; do not fork a parallel Linux check system.

New focused helpers: tools/native_runtime.py owns instrumented subprocess environments, while tools/native_toolchain.py owns executable discovery/version/capability reporting. CMake records decisions in native_check_context.json. Existing tools/run_tests.sh remains the authoritative quick/native/full dispatcher.

### T13: Version build context without weakening source and toolchain validation

**Files:** Modify `native/CMakeLists.txt`, `tools/native_checks.py`, `tools/check_native_contracts.py`, and `tests/reference/test_native_check_tools.py`. Create `native/cmake/BikeCheckContext.cmake` for context generation after target creation. Update existing fixtures in `tests/reference/test_native_check_tools.py` rather than removing their malformed-context cases.

**Interfaces:** Keep `load_context(build: Path) -> dict` and `load_entries(build: Path, expected: set[Path]) -> list[dict]`. Add `validate_platform_context(context: dict) -> dict`. Produce native_check_context schema_version=2, retaining compiler/include/library/required-file/source-root/tool/target fields and adding `platform`, `sysroot`, `stdlib`, `numerics`, `policy`, and `capabilities`. native_sources.json remains schema_version=1 unless its actual structure changes.

**Reason and boundaries:** The current loader rejects Linux because sdk must be a real directory. A fake SDK path would also make SDK regression controls meaningless. Platform metadata must augment, not replace, the existing fail-closed compiler, source membership, path, and flag checks.

- [ ] **Step 1 - Add the regression and a valid control.**

Place these cases in the existing test module, which already imports the tool and creates complete valid contexts:

```python
import pytest
from tools.native_checks import CheckError, validate_platform_context

def linux_platform():
    return {"schema_version": 2,
            "platform": {"system": "Linux", "host_system": "Linux",
                         "machine": "x86_64", "pointer_bits": 64},
            "sdk": None, "sdk_requested": None,
            "sdk_source": "not-applicable", "sysroot": None}

def test_linux_has_no_fake_apple_sdk():
    context = linux_platform()
    assert validate_platform_context(context)["sdk"] is None

@pytest.mark.parametrize("field,value", [
    ("sdk", "/"), ("sdk_requested", "macosx"), ("sdk_source", "default"),
])
def test_linux_rejects_apple_sdk_fields(field, value):
    context = linux_platform()
    context[field] = value
    with pytest.raises(CheckError, match="SDK"):
        validate_platform_context(context)

def test_cross_target_context_is_rejected():
    context = linux_platform()
    context["platform"]["host_system"] = "Darwin"
    with pytest.raises(CheckError, match="cross"):
        validate_platform_context(context)
```

Extend the existing write_fixture helper with keyword `system="Darwin"` and version selection, defaulting to its current behavior so old tests retain meaning. Add complete Linux contexts to load_context/load_entries cases: real fake executable files, headers/libraries, source manifest and compile commands. Repeat stale compiler, missing library, duplicate TU, missing TU, wrong target, wrong selected flags, relative path, malformed command and absent compile database tests for Linux. Do not limit the new tests to the small platform subvalidator.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_check_tools.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Separate platform validation from filesystem/target validation. The initial branch is:

```python
def validate_platform_context(context):
    platform = context.get("platform")
    if not isinstance(platform, dict):
        raise CheckError("native context has no platform record")
    system = platform.get("system")
    if system not in {"Darwin", "Linux"}:
        raise CheckError("unsupported native context platform")
    if platform.get("host_system") != system:
        raise CheckError("cross-compilation context is unsupported")
    if not isinstance(platform.get("machine"), str) or not platform["machine"]:
        raise CheckError("native context has no target machine")
    if type(platform.get("pointer_bits")) is not int or platform["pointer_bits"] != 64:
        raise CheckError("native context requires the qualified 64-bit ABI")
    if system == "Linux":
        if (context.get("sdk") is not None
                or context.get("sdk_requested") is not None
                or context.get("sdk_source") != "not-applicable"):
            raise CheckError("Linux context must not contain an Apple SDK")
    return context
```

Retain the current full Darwin SDK path/request/source validation immediately after this branch. Linux `sysroot` is null or a separately validated absolute directory corresponding to an explicitly selected native sysroot; do not translate it to sdk. No cross sysroot may bypass the host/target check.

Version handling: schema 2 is the new writer. Schema 1 can be read only as a legacy Darwin record after its existing SDK and compiler checks and a verified Darwin compiler target; never infer Linux from a missing sdk or silently rewrite a legacy file on disk. Tests for fake legacy compilers must explicitly model a Darwin target. An unsupported schema/version type fails with an actionable reconfigure command.

`stdlib` is `{name: "libc++"|"libstdc++", version: string, hardening: {mode: string, effective: bool}}`. Keep the old libcpp_hardening object for the existing libc++ controls; require consistency with stdlib. libstdc++ uses mode `ASSERTIONS` and `_GLIBCXX_ASSERTIONS`; legacy libc++ modes stay EXTENSIVE/FAST. `policy` records required flags, accepted optional warnings and rejected optional warnings with reasons per configured compiler. `numerics` is T06's validated build-only record, including absolute library paths. `capabilities` records requested/supported/enabled states and measured evidence for ASan, RTSan, coverage and each frontend tool. These build-only paths do not enter T12 persisted identity.

Generate the context after all first-party targets exist. Preserve target contexts for bike_native, bike_native_contract_core, bike_native_engine, all contract/distance/station executables and native_bench; add bike_native_numerics and its new test executables. Expand every effective generator expression per configuration before comparing with compile_commands.json. An interface alias is not an extra compiled TU. Preserve compile-command multiplicity for a source intentionally compiled under different target settings.

Update check_native_contracts.py to validate these platform-aware expectations while retaining strict C++23/no-GNU-extensions, Werror, no-fast-math, FP contraction, actual hardening and first-party source checks. Native source fingerprinting already includes native/cmake; confirm new .cmake/.in files affect both compiled and Python fingerprints identically.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_check_tools.py -q
uv run --frozen --group native python tools/native_checks.py --kind context --build native/build/release
uv run --frozen --group native python tools/native_checks.py --kind headers --build native/build/release
uv run --frozen --group native python tools/check_native_contracts.py --build native/build/release
```

Run new generation and loading together on macOS and Linux; test Debug and Release contexts separately. Corrupt one compiler path, one library hash/path, one target compile option and one source manifest entry in throwaway build copies and require nonzero failure. Check source-set count against actual compile commands. No validation success is allowed with an empty TU set.

- [ ] **Step 5 - Review and commit.**

```sh
git add native/CMakeLists.txt native/cmake/BikeCheckContext.cmake tools/native_checks.py tools/check_native_contracts.py tests/reference/test_native_check_tools.py
git commit -m "feat: validate platform-aware native build contexts"
```

**Review gate:** Linux uses a truthful no-Apple-SDK context; every former compiler/source/library/flag negative control remains active. The context writer and reader are compatible in the same integration commit.

### T14: Discover analysis tools and preserve real negative controls on both systems

**Files:** Create `tools/native_toolchain.py` and `tests/portability/test_native_toolchain.py`. Modify `tools/native_checks.py`, `native/cmake/BikeCheckContext.cmake`, `native/CMakeLists.txt`, `native/.clangd`, `tests/reference/test_native_check_tools.py`, and the existing `tools/check_native_frontends.sh`, `tools/check_native_tidy.sh`, `tools/check_analyzer.sh`, `tools/check_cppcheck.sh`, `tools/check_odr.sh` wrappers only if their forwarding behavior requires it.

**Interfaces:** Produce `resolve_tool(name: str, override: str | None, candidates: list[str]) -> Path` and `inspect_tool(path: Path) -> dict`. Preserve CLANG_TIDY, ANALYZER_CLANG, CPPCHECK and GXX environment overrides. Resolve each tool by explicit override, compatible configured-compiler sibling, then PATH; Darwin may additionally use its existing Homebrew/Xcode discovery. Invalid explicit overrides fail rather than falling through.

**Reason and boundaries:** The old defaults assume one Homebrew prefix and g++-16. Compiler-target metadata and availability, not branding or path prefix, determine which frontend arguments and hardening controls are meaningful. Missing required tools must remain errors in native/full.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from pathlib import Path
import pytest
from tools.native_toolchain import resolve_tool

def executable(path: Path):
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(0o755)
    return path

def test_explicit_tool_wins(tmp_path):
    requested = executable(tmp_path / "chosen-clang-tidy")
    fallback = executable(tmp_path / "other-clang-tidy")
    assert resolve_tool("clang-tidy", str(requested), [str(fallback)]) == requested

def test_broken_override_is_not_hidden(tmp_path):
    fallback = executable(tmp_path / "clang-tidy")
    with pytest.raises(ValueError, match="override"):
        resolve_tool("clang-tidy", str(tmp_path / "missing"), [str(fallback)])
```

Add argv tests for a path containing spaces, compiler-launcher preservation, Linux --sysroot pairs, Darwin -isysroot pairs, libstdc++ assertion definitions and platform-appropriate standard library selection. Negative controls must deliberately trigger a warning, a header self-sufficiency error, an invalid source context and a runtime hardening assertion. A control that fails because the executable cannot launch is not evidence that its intended diagnostic works.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_native_toolchain.py tests/reference/test_native_check_tools.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Implement executable resolution with no shell interpretation:

```python
import os
import shutil
from pathlib import Path

def resolve_tool(name, override, candidates):
    choices = [override] if override is not None else candidates
    for choice in choices:
        located = shutil.which(choice) if not Path(choice).is_absolute() else choice
        if located and Path(located).is_file() and os.access(located, os.X_OK):
            return Path(located).absolute()
    if override is not None:
        raise ValueError(f"invalid {name} override: {override}")
    raise ValueError(f"required tool {name} was not found; checked {choices!r}")
```

Preserve symlink executable basenames where the compiler driver uses them to select C++ behavior. Record both requested and resolved location/version. `inspect_tool` executes --version and compiler-target/resource queries where supported, records return codes/stdout/stderr, and rejects empty or unrelated tools. Do not claim any clang-tidy major can safely analyze any compiler's compile database: qualify the chosen combination with the real header/diagnostic suite and record it.

Refactor `_gcc_context_args` to retain paired and joined --sysroot values in addition to include/define/undefine and relevant FP/PIC/thread flags. Do not forward Apple's SDK to a native Linux GCC. For a secondary frontend, rebuild only explicitly compiler-specific policy flags; preserve include roots, definitions, language version, architecture/ABI and target ownership. A sweep must not silently analyze a different configuration from the one compiled.

Keep the existing GCC warning set where supported. Probe optional warning names using a compile control with unknown-warning treated as failure; record unavailable warnings. Treat C++23, Werror, pedantic diagnostics, FP and hardening requirements as mandatory. Do not remove all GCC warnings because one flag differs.

Split `run_context_checks` into common controls, Darwin SDK controls and standard-library-specific hardening controls. Common controls include explicit compiler/Python path, stale dependencies, wrong selected artifact, source-list changes, duplicate TUs and target-context drift. Darwin retains explicit/default/missing SDK and fallback-link-failure controls unchanged in intent. Linux adds no-Apple-tool invocation, rejected CMAKE_OSX_SYSROOT and native sysroot-preservation controls. Do not fabricate `/` as an SDK.

libc++ retains the existing EXTENSIVE/FAST assertion, object-size and timing controls. libstdc++ gets a separate `_GLIBCXX_ASSERTIONS` positive/negative control using a subprocess vector bounds violation: control compilation must succeed, the hardened executable must terminate nonzero, and the unhardened control must not be falsely described as a reliable behavior guarantee. Verify `_GLIBCXX_DEBUG` is absent to avoid ABI substitution. Use symbol/macro or safe hardening self-test evidence where an unchecked undefined-behavior control would be unreliable. Control outcomes distinguish compile failure, runtime assertion and tool failure.

Update native/.clangd to use the selected compilation database and a configurable local compiler query-driver setup; do not bake a developer's Homebrew or Linux absolute compiler path into shared config. Document the editor command in T26. Keep suppression entries narrowly matched with existing reason/reproducer/remove_when metadata; no new global suppressions for Linux.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_native_toolchain.py tests/reference/test_native_check_tools.py -q
for target in frontends tidy analyzer cppcheck odr scripts; do
  uv run --frozen --group native cmake --build native/build/release --target "check_$target" || exit "$?"
done
for kind in headers diagnostic-controls context; do
  uv run --frozen --group native python tools/native_checks.py --kind "$kind" --build native/build/release || exit "$?"
done
```

Run once with valid tool overrides and once with a deliberately nonexistent required override. The latter must fail before any empty sweep is labeled passing. Retain per-tool version, command, TU count, return code and unsuppressed diagnostic evidence. Re-run macOS SDK/hardening controls on a real Mac, not only mocked Linux tests.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/native_toolchain.py tools/native_checks.py tools/check_native_frontends.sh tools/check_native_tidy.sh tools/check_analyzer.sh tools/check_cppcheck.sh tools/check_odr.sh native/cmake/BikeCheckContext.cmake native/CMakeLists.txt native/.clangd tests/portability/test_native_toolchain.py tests/reference/test_native_check_tools.py
git commit -m "feat: discover native analysis tools across Linux and macOS"
```

**Review gate:** All required analysis lanes run against the real selected configuration with nonzero source counts and functioning negative controls. Unsupported tools do not become successful skips.

### T15: Load compiler-matched sanitizer runtimes and preserve instrumentation failures

**Files:** Create `tools/native_runtime.py`, `tests/portability/test_native_runtime.py`, and `native/tests/sanitizer_control.cpp`. Modify `tools/run_tests.sh`, `native/CMakeLists.txt`, `native/cmake/BikeCompilerPolicy.cmake` (T09), `native/cmake/BikeCheckContext.cmake`, and existing sanitizer tests in `tests/reference/test_native_check_tools.py`.

**Interfaces:** Produce `runtime_environment(context: dict, environ: dict[str,str]) -> dict[str,str]` returning the complete subprocess environment; CLI `python tools/native_runtime.py run --build PATH -- COMMAND [ARGUMENTS]` runs an argv array, propagates exit status and does not print secrets. Context capabilities record sanitizer compiler ID, target, resolved runtime, requested/enabled flags and probe evidence. Preserve NATIVE_SANITIZE and the current NATIVE_TEST_SANITIZER_RUNTIME evidence variable.

**Reason and boundaries:** The shell runner already distinguishes DYLD_INSERT_LIBRARIES and LD_PRELOAD. Its fragile part is guessing GCC versus Clang from a filename. Instrumented Python extensions, CTest binaries and contract scripts must all use the same configured runtime and remain fatal on actual sanitizer findings.

- [ ] **Step 1 - Add the regression and a valid control.**

```python
from tools.native_runtime import runtime_environment

def test_linux_asan_uses_recorded_compiler_not_filename(tmp_path):
    runtime = tmp_path / "libasan.so"
    runtime.write_bytes(b"test-fixture")
    context = {"platform": {"system": "Linux", "machine": "x86_64"},
               "compiler": {"id": "GNU", "path": "/custom/c++"},
               "capabilities": {"asan": {"enabled": True,
                   "supported": True, "runtime": str(runtime)},
                   "rtsan": {"enabled": False}}}
    result = runtime_environment(context, {"LD_PRELOAD": "/existing/preload.so"})
    assert result["LD_PRELOAD"] == f"{runtime}:/existing/preload.so"
    assert result["NATIVE_TEST_SANITIZER_RUNTIME"] == str(runtime)
    assert result["PYTHONMALLOC"] == "malloc"
```

Add Darwin insertion, no-sanitizer untouched environment, missing runtime, conflicting modes, paths containing spaces and pre-existing ASAN_OPTIONS tests. Keep an instrumented positive program with no error and a separate intentional use-after-free control. Require the negative control's stderr to identify AddressSanitizer and the intended error; a dynamic-loader error is not a passing negative control.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_native_runtime.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Use configured compiler ID/target, not basename. Clang's runtime discovery uses its own supported resource/runtime lookup; GCC asks its own driver for libasan.so. For every returned filename require a real file, correct architecture and a successful instrumented load/run control. A compiler returning the unresolved filename string is a failure. Linux must never preload an Apple dylib. Record the actual runtime selected by the configured compiler.

CMake checks NATIVE_SANITIZE and NATIVE_RTSAN mutual exclusion at configure time as well as the runner. For Clang retain the existing supported address/undefined/local-bounds/float-cast-overflow flags and shared-runtime behavior. For GCC use the supported ASan/UBSan subset with explicit per-capability controls; do not pass Clang-only -shared-libasan or -fsanitize=local-bounds. Preserve -fno-omit-frame-pointer and fatal-error behavior. Require all first-party objects, including the numerical adapter/core/engine targets, to receive the appropriate instrumentation. Do not apply first-party warning/instrumentation policy blindly to vendored nanobind source.

`runtime_environment` starts with a copy of the provided environment. It prepends the selected runtime to DYLD_INSERT_LIBRARIES on Darwin or LD_PRELOAD on Linux, preserving prior entries. It rejects both sanitizers enabled. Keep the existing project ASan settings `detect_leaks=0:detect_stack_use_after_return=1` unless a separately reviewed test changes their rationale; this port must not claim leak coverage that this configuration disables. Set PYTHONMALLOC=malloc for ASan and preserve unrelated environment keys.

The run CLI loads the validated T13 context, computes the environment, and executes the passed command with subprocess.run(check=False). It prints only selected sanitizer/tool evidence, not a full environment dump. Replace shell compiler-name heuristics with calls to this helper while preserving native_uv and all existing preflight/pytest/CTest ordering. Every invocation remains in the frozen native group.

A deliberately failing control can be compiled in a throwaway control directory, never linked into the application:

```cpp
#include <cstddef>
#include <memory>
int main(int argc, char**) {
    auto values = std::make_unique<int[]>(4);
    values[0] = 7;
    volatile int* escaped = values.get();
    values.reset();
    return escaped[static_cast<std::size_t>(argc - 1)];
}
```

Compile without optimization that can erase the intended memory access, run with the matching runtime and require the intended sanitizer finding. Treat this as an expected-failure subprocess test with explicit stderr validation, not a CTest WILL_FAIL flag that would accept any crash. Keep a valid allocation/free control beside it. Test Python `load_native_extension` in a fresh instrumented process and CTest under the same environment. Avoid a second competing ASan runtime inherited from the host.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native cmake -S native -B native/build/asan -DCMAKE_BUILD_TYPE=Debug -DNATIVE_SANITIZE=ON
uv run --frozen --group native cmake --build native/build/asan --parallel
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/asan" bash tools/run_tests.sh native
uv run --frozen --group native python -m pytest tests/portability/test_native_runtime.py -q
```

Run Linux Clang, Linux GCC and the previously supported macOS sanitizer lane. Save the clean positive control, intentional finding, imported extension path, compiler/runtime identity and all actual native findings. A missing runtime or new sanitizer report is a failed gate. Do not run the uninstrumented loader test from T10 against an ASan extension without this helper.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/native_runtime.py tools/run_tests.sh native/CMakeLists.txt native/cmake native/tests/sanitizer_control.cpp tests/portability/test_native_runtime.py tests/reference/test_native_check_tools.py
git commit -m "feat: select and verify native sanitizer runtimes by toolchain"
```

**Review gate:** Instrumentation is active, the intended negative control is detected, clean native tests remain clean, and both platform preload paths retain existing behavior. GCC is not treated as Clang by filename.

### T16: Qualify RTSan explicitly and keep realtime contracts intact

**Files:** Modify `native/src/rtsan.hpp`, `native/CMakeLists.txt`, `native/cmake/BikeCompilerPolicy.cmake`, `native/cmake/BikeCheckContext.cmake`, `tools/native_runtime.py`, and `tests/portability/test_native_runtime.py`. Add `native/tests/rtsan_control.cpp` and a subprocess control test in `tests/reference/test_native_check_tools.py`. Keep existing BIKE_NONBLOCKING call sites and native/tests/test_contracts.cpp realtime cases.

**Interfaces:** Consume NATIVE_RTSAN and T15 runtime execution. Produce capability record `{requested: bool, supported: bool, enabled: bool, runtime: absolute path|null, evidence: object}`. Requested-but-unsupported is a configure error; unrequested unsupported is explicitly unavailable, not tested. BIKE_NONBLOCKING remains the one annotation macro.

**Reason and boundaries:** AppleClang or a generic clang executable does not prove realtime sanitizer instrumentation/runtime exists. Removing the annotation or reporting a requested unsupported lane as passed would hide allocation/locking regressions.

- [ ] **Step 1 - Add the regression and a valid control.**

Add a compiler-probe fixture that accepts C++23 but rejects -fsanitize=realtime; require `-DNATIVE_RTSAN=ON` to fail with a message naming that flag and the selected compiler. Add a valid nonblocking arithmetic control and this intentional violation:

```cpp
#include <cstdlib>
#include "rtsan.hpp"
BIKE_NONBLOCKING void forbidden_allocation() {
    void* volatile pointer = std::malloc(32);
    std::free(pointer);
}
int main() {
    forbidden_allocation();
    return 0;
}
```

The subprocess test requires the selected RTSan diagnostic and nonzero status; unrelated linker failures, missing libraries or signals do not count. Compile the probe using the same annotation support and runtime flags as production.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_native_runtime.py tests/reference/test_native_check_tools.py -k "rtsan or realtime" -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

CMake first compile-probes the flag and annotation together, then link/run-probes the configured dynamic runtime and violation control. Preserve the __has_feature guard in rtsan.hpp so normal GCC builds do not see unsupported attributes. When RTSan is explicitly enabled, require that BIKE_NONBLOCKING expands to the functioning annotation rather than an empty macro. Add a preprocessor/error control to prove this.

Resolve the runtime through the selected Clang toolchain using T15, not a universal Homebrew path or host uname alone. Keep ASan and RTSan in separate build trees, and reject contradictory toggles in both configure and runner. Do not replace realtime checks with allocation counters; retain both because they cover different contracts.

Run existing `test_rtsan_invalid_status_control` and all marked try_* kernels through the native contract suite. Keep exception construction, configuration, Python boxing and diagnostic rendering outside realtime regions. A Linux report caused by a lazy BLAS allocation must be solved by the existing explicit preparation/warmup boundary or by a verified nonallocating call path, not by removing BIKE_NONBLOCKING. Prove all lazy initialization completes before entering the guarded hot loop and that errors still return the established status/result type.

The capability matrix distinguishes: required qualified Clang RTSan lane, compiler without RTSan (ordinary build supported, requested RTSan fails), and missing runtime installation (setup failure). A green GCC lane does not replace the required qualified Clang lane. Preserve the working macOS RTSan toolchain from baseline.

Add this requested-mode compile control in BikeCompilerPolicy.cmake; the real runtime/violation probe described above is an additional required gate, not replaced by this compile result:

```cmake
if(NATIVE_RTSAN)
  if(NATIVE_SANITIZE)
    message(FATAL_ERROR "ASan/UBSan and RTSan require separate build trees")
  endif()
  include(CMakePushCheckState)
  include(CheckCXXSourceCompiles)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_FLAGS "-fsanitize=realtime")
  set(CMAKE_REQUIRED_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/src")
  check_cxx_source_compiles([=[
    #include "rtsan.hpp"
    #if !defined(__has_feature)
    #error realtime sanitizer feature query is unavailable
    #elif !__has_feature(realtime_sanitizer)
    #error realtime sanitizer instrumentation is unavailable
    #endif
    BIKE_NONBLOCKING void control() {}
    int main() { control(); }
  ]=] BIKE_RTSAN_INSTRUMENTATION_LINKS)
  cmake_pop_check_state()
  if(NOT BIKE_RTSAN_INSTRUMENTATION_LINKS)
    message(FATAL_ERROR "Requested RTSan compiler/annotation/link control failed")
  endif()
endif()
```

Keep the probe variables tied to the selected compiler/build cache, and reject reuse after a toolchain change. An executable that links but fails its violation control does not set supported=true.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native cmake -S native -B native/build/rtsan -DCMAKE_BUILD_TYPE=Debug -DNATIVE_RTSAN=ON
uv run --frozen --group native cmake --build native/build/rtsan --parallel
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/rtsan" bash tools/run_tests.sh native
```

Run the positive/negative control and normal native tests on each qualified RTSan host. On a compiler known to lack the feature, explicitly request it in a separate build directory and require failure. Document the concrete toolchain needed on Linux and macOS; do not generalize support from compiler major alone.

- [ ] **Step 5 - Review and commit.**

```sh
git add native/src/rtsan.hpp native/CMakeLists.txt native/cmake native/tests/rtsan_control.cpp tools/native_runtime.py tests/portability/test_native_runtime.py tests/reference/test_native_check_tools.py
git commit -m "test: qualify realtime sanitizer support per native toolchain"
```

**Review gate:** Realtime annotations are demonstrably active in qualified builds, requested unsupported builds fail, and no hot-path safety contract was removed to make Linux pass.

### T17: Generate coverage with matching LLVM tools and selected-artifact provenance

**Files:** Modify `tools/native_coverage.sh`, `tools/native_coverage.py`, `tools/native_toolchain.py`, `native/CMakeLists.txt`, `native/cmake/BikeCompilerPolicy.cmake`, and existing coverage tests in `tests/reference/test_native_check_tools.py`. Create `tests/portability/test_native_coverage_tools.py` for discovery/capability tests.

**Interfaces:** Preserve `bash tools/native_coverage.sh [build_dir]`, LLVM_PROFDATA and LLVM_COV overrides, per-run directories and test_native_provenance.json. Add `resolve_coverage_tools(compiler: Path, overrides: dict[str,str]) -> dict` to native_toolchain.py, returning actual tool paths/versions and a successful format compatibility probe.

**Reason and boundaries:** The existing script already has a non-xcrun fallback, but a random PATH llvm-cov/llvm-profdata can disagree with the compiler. Coverage is a qualified LLVM lane; this port does not need a second GCC gcov reporting system.

- [ ] **Step 1 - Add the regression and a valid control.**

Add tests that an explicit missing LLVM_COV override fails, a Linux tool search never invokes xcrun, and compiler-sibling tools precede unrelated PATH tools. Use a fake merge tool that exits nonzero and require the wrapper to preserve status and log path. Keep the current negative cases for no profraw files, zero native tests, wrong imported extension, wrong build, duplicate artifacts and malformed provenance.

Use an actual minimal instrumented program to establish profile format compatibility:

```cpp
int main(int argc, char**) {
    return argc > 1 ? 1 : 0;
}
```

Compile with the selected LLVM coverage flags, run with a fresh LLVM_PROFILE_FILE, merge with the chosen llvm-profdata and export with chosen llvm-cov. Only the complete chain proves tool compatibility.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_native_coverage_tools.py tests/reference/test_native_check_tools.py -k coverage -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

CMake rejects NATIVE_COVERAGE=ON with a compiler that cannot pass the LLVM instrumentation/format probe; it does not silently produce an uninstrumented build. Keep -fprofile-instr-generate and -fcoverage-mapping on the intended first-party targets and link steps. Do not add GNU gcov flags to the same build.

Resolve LLVM_PROFDATA/LLVM_COV in this order: valid explicit override, selected LLVM compiler sibling, qualified PATH candidate; Darwin may use xcrun as a platform-only fallback. Record tool versions and successful format-probe output. Different branding/major strings alone are not a proof of incompatibility or compatibility; the actual minimal compile/run/merge/export control is authoritative.

Configure coverage in its own build directory with a known configuration and selected compiler. Keep one fresh run directory, process-unique profiles, separate stdout/stderr/command logs and exact extension identity. Preserve the native test reporter's nonzero count and imported-extension checks. Include new platform adapter TUs in source coverage, but exclude third-party/generated code using the existing documented policy rather than gaming percentages. If CTest and Python write profiles concurrently, use collision-resistant `%p`/module patterns appropriate to the selected LLVM runtime and verify all produced profiles merge.

The shell script delegates discovery to the helper and still delegates tests to run_tests.sh native. No stale profiles from earlier runs are merged. Preserve existing coverage threshold/baseline behavior; introducing a new arbitrary coverage percentage is outside this port.

Use this concrete private format probe from resolve_coverage_tools after candidate-path validation; capture command stdout/stderr in the surrounding diagnostic wrapper on failure:

```python
import json
import os
from pathlib import Path
import subprocess
import tempfile

def _coverage_format_probe(compiler: Path, profdata: Path, cov: Path) -> dict:
    with tempfile.TemporaryDirectory(prefix="bike-coverage-") as folder:
        root = Path(folder)
        source = root / "control.cpp"
        binary = root / "control"
        merged = root / "merged.profdata"
        source.write_text("int main(int argc, char**) { return argc > 1 ? 1 : 0; }\n")
        subprocess.run([str(compiler), "-std=c++23", "-fprofile-instr-generate",
                        "-fcoverage-mapping", str(source), "-o", str(binary)],
                       check=True, capture_output=True, text=True, timeout=60)
        env = dict(os.environ, LLVM_PROFILE_FILE=str(root / "control-%p.profraw"))
        subprocess.run([str(binary)], env=env, check=True,
                       capture_output=True, text=True, timeout=15)
        raw = sorted(root.glob("*.profraw"))
        if not raw:
            raise RuntimeError("coverage control produced no raw profile")
        subprocess.run([str(profdata), "merge", "-sparse",
                        *map(str, raw), "-o", str(merged)],
                       check=True, capture_output=True, text=True, timeout=30)
        exported = subprocess.run([str(cov), "export", str(binary),
                                   f"-instr-profile={merged}"],
                                  check=True, capture_output=True, text=True,
                                  timeout=30)
        data = json.loads(exported.stdout)
        if data.get("type") != "llvm.coverage.json.export" or not data.get("data"):
            raise RuntimeError("coverage export did not contain valid control data")
        return {"format_probe_passed": True, "export_version": data.get("version")}
```

Retain this probe evidence with the actual compiler/tool versions. Let subprocess or JSON failures make the requested coverage check fail with the exact command; do not return an empty successful report.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
LLVM_COV="$(command -v llvm-cov)" LLVM_PROFDATA="$(command -v llvm-profdata)" bash tools/native_coverage.sh "$(pwd)/native/build/coverage"
uv run --frozen --group native python -m pytest tests/portability/test_native_coverage_tools.py tests/reference/test_native_check_tools.py -k coverage -q
```

Use explicit tool paths only after T14/T17 doctor has qualified them; the shown PATH query is not itself qualification. Run on Linux LLVM and the retained macOS coverage toolchain. Require a valid exported report, nonzero native test count, selected extension identity, new backend source visibility and retained logs. Deliberately mismatched tools must fail with the merge/export diagnostic, not produce an empty successful report.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/native_coverage.sh tools/native_coverage.py tools/native_toolchain.py native/CMakeLists.txt native/cmake/BikeCompilerPolicy.cmake tests/portability/test_native_coverage_tools.py tests/reference/test_native_check_tools.py
git commit -m "feat: qualify cross-platform LLVM native coverage"
```

**Review gate:** Coverage measures the selected instrumented artifact and real executed tests using compatible tools on both platforms; unsupported configurations are explicit failures when requested.
