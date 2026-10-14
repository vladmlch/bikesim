# 02 - Native platform and numerical ABI boundaries Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Compile the same native runtime on Linux and macOS without replacing the macOS backend.

**Architecture:** Split platform detection, dependency discovery, numerical ABI adapters and compiler policy into focused modules. Integrate them into the existing target graph while preserving source manifests and private MuJoCo error handling.

**Tech Stack:** Python/uv, NumPy, MuJoCo 3.12.0, nanobind, CMake, C++23, Accelerate on macOS, the selected NumPy OpenBLAS provider on Linux, pytest, CTest, LLVM/GCC analysis tools.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`. Read the source audit and master plan as well as this file.

## Global Constraints

- Add Linux support; retain macOS support and existing macOS behavior.
- Keep Python >=3.13, CMake >=3.24, and strict ISO C++23 without compiler language extensions.
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

## File map and order

Create native/cmake/BikePlatform.cmake, BikeAppleSDK.cmake, BikeDependencies.cmake, BikeCompilerPolicy.cmake and BikeLinkage.cmake. Move code from the monolithic native/CMakeLists.txt only where these files own a specific responsibility. Create tools/native_dependencies.py and focused platform adapter files; do not reorganize physics modules.

T04 and T05 can proceed in separate worktrees after T02. T06 depends on T05's JSON probe. T07/T08 consume T06. T09 can run alongside ABI work. T10 is the single-owner integration point for native/CMakeLists.txt and must merge all preceding work. Until T10, platform-module/adapter test harnesses are the task-local gates; the full Linux extension is not claimed to work.

### T04: Isolate pre-project Apple SDK discovery and preserve every macOS control

**Files:** Modify `native/CMakeLists.txt:1-156`; create `native/cmake/BikePlatform.cmake`, `native/cmake/BikeAppleSDK.cmake`, `tests/portability/test_cmake_platform.py`.

**Interfaces:** BikePlatform.cmake sets `BIKE_TARGET_SYSTEM` to Darwin or Linux for native builds. BikeAppleSDK.cmake retains `_native_sdk_path`, `_native_sdk_requested`, `_native_sdk_source` for the existing Apple consumer contract; Linux sets empty internal values and emits null/not-applicable in JSON later.

**Reason and boundaries:** The reproduced failure happens before project(). Adding if(APPLE) only around final link flags cannot fix it. Existing explicit/default/fallback SDK behavior is valuable and must remain intact.

- [ ] **Step 1 - Add the regression and a valid control.**

Create a minimal temporary CMake project that includes BikePlatform.cmake before `project(platform_probe CXX)`. Put executable xcrun and xcode-select scripts at the front of PATH that record invocation and exit 91. On Linux require successful configure and no marker files:

```python
import os
import subprocess
import sys
from pathlib import Path
import pytest

@pytest.mark.skipif(sys.platform != "linux", reason="Linux negative Apple-tool control")
def test_linux_configure_never_invokes_apple_tools(tmp_path):
    root = Path.cwd()
    fake = tmp_path / "bin"
    fake.mkdir()
    marker = tmp_path / "apple-invoked"
    for name in ("xcrun", "xcode-select"):
        script = fake / name
        script.write_text(f'#!/bin/sh\ntouch "{marker}"\nexit 91\n')
        script.chmod(0o755)
    source = tmp_path / "probe"
    source.mkdir()
    module = root / "native/cmake/BikePlatform.cmake"
    (source / "CMakeLists.txt").write_text(
        'cmake_minimum_required(VERSION 3.24)\n'
        f'include("{module}")\nproject(platform_probe CXX)\n')
    result = subprocess.run(["cmake", "-S", str(source), "-B", str(tmp_path / "build")],
                            env={**os.environ, "PATH": str(fake) + os.pathsep + os.environ["PATH"]},
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not marker.exists()
```

The OS-specific skip is legitimate because it tests a Linux-only branch; macOS has its own mandatory positive SDK controls, not an empty substitute suite.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_cmake_platform.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Use an explicit native-build dispatch before project():

```cmake
if(DEFINED CMAKE_SYSTEM_NAME AND
   NOT CMAKE_SYSTEM_NAME STREQUAL CMAKE_HOST_SYSTEM_NAME)
  message(FATAL_ERROR "Cross-compilation is not supported by this native-build workflow")
endif()
set(BIKE_TARGET_SYSTEM "${CMAKE_HOST_SYSTEM_NAME}")
if(BIKE_TARGET_SYSTEM STREQUAL "Darwin")
  include("${CMAKE_CURRENT_LIST_DIR}/BikeAppleSDK.cmake")
elseif(BIKE_TARGET_SYSTEM STREQUAL "Linux")
  if(DEFINED CMAKE_OSX_SYSROOT AND NOT CMAKE_OSX_SYSROOT STREQUAL "")
    message(FATAL_ERROR "CMAKE_OSX_SYSROOT is a macOS option; use a Linux-native toolchain")
  endif()
  set(_native_sdk_path "")
  set(_native_sdk_requested "")
  set(_native_sdk_source "not-applicable")
else()
  message(FATAL_ERROR "Unsupported native-build host: ${BIKE_TARGET_SYSTEM}")
endif()
```

Move the old lines 4-121 into BikeAppleSDK.cmake without dropping compiler launcher, compiler arg1, CXX_FLAGS, architecture list, SDK name resolution, symlink normalization, explicit-request retention, duplicate candidate removal, or the compile-and-link Accelerate probe. Keep the Availability.h try_compile as an Apple-only post-project function invoked from CMakeLists.txt. Recheck CMAKE_CROSSCOMPILING after project() and reject it clearly.

Do not use `/` as a fake Linux SDK, do not set CMAKE_OSX_SYSROOT on Linux, and do not overwrite an explicitly supplied compiler/sysroot cache value. Keep invalid Apple requests fatal rather than falling back silently.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_cmake_platform.py -q
```


Run the Linux fake-tool control plus negative tests for a Linux Apple option and explicit foreign CMAKE_SYSTEM_NAME. On macOS rerun current context controls for explicit SDK path, SDK name, missing path, incompatible compiler/SDK, default SDK and fallback from a link-incompatible default. Record the requested cache value separately from the resolved path and compare pre/post behavior.

- [ ] **Step 5 - Review and commit.**

```sh
git add native/CMakeLists.txt native/cmake/BikePlatform.cmake native/cmake/BikeAppleSDK.cmake tests/portability/test_cmake_platform.py
git commit -m "build: isolate Apple SDK setup from Linux configuration"
```

**Review gate:** Linux module configure invokes no Apple tool. macOS keeps explicit-request and fallback semantics; a mere successful default configure is insufficient.

### T05: Discover the selected Python dependencies and MuJoCo shared library

**Files:** Create `tools/native_dependencies.py`, `native/cmake/BikeDependencies.cmake`, `tests/portability/test_native_dependencies.py`; modify the dependency section of `native/CMakeLists.txt:158-215`.

**Interfaces:** Produce probe_dependencies(python: Path) -> dict and validate_dependencies(value: dict) -> dict as specified. BikeDependencies.cmake creates `Bike::MuJoCo` with IMPORTED_LOCATION and INTERFACE_INCLUDE_DIRECTORIES, and exposes `_native_mujoco_library` for manifests and existing consumers.

**Reason and boundaries:** The interpreter used for path discovery must be the one used by nanobind/Python linkage. Version-matched filenames alone do not prove private error ABI compatibility.

- [ ] **Step 1 - Add the regression and a valid control.**

Use package-shaped temporary directories to test pure library selection, which does not import third-party packages:

```python
import pytest
from tools.native_dependencies import select_mujoco_library

def test_linux_library_selection_is_not_a_dylib_glob(tmp_path):
    library = tmp_path / "libmujoco.so.3.12.0"
    library.write_bytes(b"fixture")
    (tmp_path / "libmujoco.3.12.0.dylib").write_bytes(b"wrong platform")
    assert select_mujoco_library(tmp_path, "Linux", "3.12.0") == library.resolve()

def test_missing_exact_engine_is_fatal(tmp_path):
    (tmp_path / "libmujoco.so.3.15.0").write_bytes(b"wrong version")
    with pytest.raises(ValueError, match="3.12.0"):
        select_mujoco_library(tmp_path, "Linux", "3.12.0")
```

Add Darwin, symlink-to-same-file, conflicting-real-files, nonexistent directory and path-with-spaces controls. A mock filename test verifies selection logic only; actual ABI testing is separate.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_native_dependencies.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Implement exact, platform-specific candidates with realpath de-duplication:

```python
from pathlib import Path

def select_mujoco_library(directory: Path, system: str, version: str) -> Path:
    if version != "3.12.0":
        raise ValueError(f"native requires MuJoCo 3.12.0, found {version}")
    names = {"Darwin": [f"libmujoco.{version}.dylib"],
             "Linux": [f"libmujoco.so.{version}"]}
    if system not in names:
        raise ValueError(f"unsupported native platform: {system}")
    found = {p.resolve() for name in names[system]
             if (p := directory / name).is_file()}
    if len(found) != 1:
        raise ValueError(f"expected one MuJoCo {version} library in {directory}; found {sorted(map(str, found))}")
    return found.pop()
```

Confirm exact installed wheel filenames during the real probe before extending this small candidate table. Never add unrestricted system search. Run the probe via Python_EXECUTABLE; use sysconfig for SOABI and Python include data, importlib for package roots, and nanobind's own `--cmake_dir`/`--include_dir` queries. Check every execute_process result and include stderr in a failure message. Return JSON only on stdout.

Create a GLOBAL imported SHARED Bike::MuJoCo target. Reuse the original CheckCXXSourceCompiles source for all three private/raw-data symbols, but set CMAKE_REQUIRED_LIBRARIES to the imported target and use cmake_push_check_state(RESET)/cmake_pop_check_state() so probe settings do not contaminate other checks. Compile/link against engine_abi_312.hpp and the selected include directory. Add a public header/runtime version control. Do not replace the TLS wrapper with mju_user_error or a process-global callback.

Validate dependency JSON types, absolute paths, real file existence and the Python executable before consumption. Use CMake string(JSON) or a generated data-only CMake file with strict escaping, never eval arbitrary text returned by a package.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
mkdir -p output/portability
PYTHON="$(uv run --frozen --group native --no-sync python -c 'import sys; print(sys.executable)')"
uv run --frozen --group native --no-sync python tools/native_dependencies.py --python "$PYTHON" --output output/portability/dependencies.json
```

Invoke the CLI with the actual resolved interpreter rather than relying on VIRTUAL_ENV being set in the caller shell. The normal driver supplies it. On both hosts, inspect the exact library path and run a compile/link probe. In an isolated venv with a non-3.12.0 MuJoCo installation, require a named dependency error before application startup. Retain error-ABI negative-control tests using a deliberately incomplete stub library; never test an unsafe guessed function signature in the controlling process.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/native_dependencies.py native/cmake/BikeDependencies.cmake native/CMakeLists.txt tests/portability/test_native_dependencies.py
git commit -m "build: resolve MuJoCo from the selected Python environment"
```

**Review gate:** One interpreter, one exact engine image, explicit process failures, and all existing error ABI symbols verified on Linux and macOS.

### T06: Qualify the actual NumPy BLAS/LAPACK provider and integer ABI

**Files:** Extend `tools/native_dependencies.py`; create `tests/portability/test_numerics_discovery.py`, `native/cmake/BikeNumerics.cmake`, `docs/development/numerical-profiles.md`.

**Interfaces:** Produce the exact `numerics` record in dependency schema 1; add `discover_numpy_provider() -> dict`, executed inside the selected interpreter child. Consume `BIKE_NUMERICS_BACKEND`; output provider paths, exported symbol map and separate BLAS/LAPACK integer widths.

**Reason and boundaries:** A generic find_package(BLAS) would not ensure the library is the one NumPy uses. Linux wheels may prefix/suffix symbols, and a wrong integer ABI is memory corruption, not a harmless rounding error.

- [ ] **Step 1 - Add the regression and a valid control.**

Test normalized provider selection as data before executing foreign functions:

```python
import pytest
from tools.native_dependencies import validate_numerics

def test_linux_rejects_unqualified_integer_width():
    with pytest.raises(ValueError, match="integer ABI"):
        validate_numerics({"backend": "NUMPY_OPENBLAS",
                           "blas_integer_bits": None,
                           "lapack_integer_bits": 64})

def test_mixed_numpy_and_native_libraries_are_not_accepted(tmp_path):
    from tools.native_dependencies import require_same_library
    a, b = tmp_path / "numpy-blas.so", tmp_path / "system-blas.so"
    a.write_bytes(b"one")
    b.write_bytes(b"two")
    with pytest.raises(ValueError, match="NumPy provider"):
        require_same_library(a, b)
```

Add a valid ILP64 metadata fixture, a Darwin 32/64 split fixture, duplicate providers, missing symbols, paths outside installed distribution records, unsafe generated identifiers and symlink aliases to the same file.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_numerics_discovery.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

In a fresh selected-interpreter subprocess, import NumPy only, exercise one dot/lstsq to force provider loading, and obtain threadpoolctl.threadpool_info(). On Linux, cross-check the returned provider paths against the installed NumPy distribution records and platform dependency inspection, not against any OpenBLAS already loaded by SciPy. Darwin system Accelerate is not a NumPy wheel file: match the actual system framework/loaded provider and selected SDK evidence instead of requiring a wheel RECORD entry. Record NumPy show_config/show_runtime output for review. On Darwin retain Accelerate and qualify both legacy CBLAS and NEWLAPACK ILP64; on Linux accept the selected wheel's verified OpenBLAS ILP64 profile initially.

Separate metadata validation from unsafe symbol invocation:

```python
import re
from pathlib import Path

SYMBOL = re.compile(r"[A-Za-z_][A-Za-z0-9_$]*")

def require_same_library(numpy_path: Path, native_path: Path) -> None:
    if numpy_path.resolve(strict=True) != native_path.resolve(strict=True):
        raise ValueError("native numerical library differs from the NumPy provider")

def checked_symbol(value: str) -> str:
    if not isinstance(value, str) or SYMBOL.fullmatch(value) is None:
        raise ValueError("invalid exported numerical symbol identifier")
    return value
```

Derive integer width from the provider's build configuration and documented symbol/header ABI, not from process pointer width or whether a function address exists. Inspect exported symbols with platform tools; reject unsupported ambiguous cases. Expected symbol families may include cblas_ddot or scipy_cblas_ddot64_, but these are candidates to verify, not constants to assume. Confirm ddot/dgemv/dgemm/dgelsd independently and store the exact accepted names.

Generate a compile-only type check followed by a disposable subprocess compile/link/run check using the qualified signatures. A small dot, rectangular GEMV/GEMM, rank-deficient DGELSD and workspace query provide valid controls; a failed/corrupt child is a hard provider failure. Import SciPy after NumPy and repeat provider identity checks to detect symbol interposition. Include BLAS thread settings and OpenBLAS core/configuration in the report without changing them. An unsupported distribution/system NumPy gets a concrete instruction to install the qualified locked wheel, not a silent fallback.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_numerics_discovery.py -q
```


Run discovery tests, then the real provider probe on each target with NumPy alone and NumPy+SciPy imported in both orders. Record actual provider-file SHA-256 for file-backed Linux libraries, names, ABI widths, NumPy version/wheel identity, OpenBLAS configuration and thread count. For a system Accelerate image in the dyld cache, use T12 apple-system identity with a genuine macOS/provider/SDK build identifier and null file hash; never hash an SDK stub or fabricate a missing digest. Require repeated probes to produce the same stable identity excluding absolute installation roots. A provider mismatch fails before building a physics target.

- [ ] **Step 5 - Review and commit.**

```sh
git add tools/native_dependencies.py native/cmake/BikeNumerics.cmake tests/portability/test_numerics_discovery.py docs/development/numerical-profiles.md
git commit -m "build: qualify the NumPy numerical provider ABI"
```

**Review gate:** Actual provider and integer widths are proven; no system-BLAS substitution, guessed function call, or unrecorded dispatch/threading assumption.

### T07: Introduce typed dual-platform CBLAS adapters without changing callers

**Files:** Modify `native/src/cblas_abi.hpp` and `native/src/binding.cpp`; create `native/src/platform/blas_accelerate.cpp`, `native/src/platform/blas_numpy_openblas.cpp`, `native/src/platform/numerics_symbols.hpp.in`, `native/tests/test_platform_blas.cpp`, `tests/reference/test_native_platform_blas.py`; extend BikeNumerics.cmake.

**Interfaces:** Retain `blas::Order`, `blas::Transpose`, `double blas::ddot(int,const double*,int,const double*,int) noexcept` and current dgemv/dgemm signatures. Produce `bike_native_numerics` static target and alias `Bike::Numerics`. Backend integer widths are private, not leaked to callers.

**Reason and boundaries:** Current hand-written declarations pin 32-bit CBLAS globally. Linux ILP64 needs safe widening and separately verified symbol spelling, while macOS must continue using its exact legacy entry points.

- [ ] **Step 1 - Add the regression and a valid control.**

Add direct C++ tests for dot=11 from [1,2] and [3,4], row/column-major GEMV/GEMM, transpose and strides. Require zero-size behavior to match existing callers. Expose a debug binding `_debug_platform_blas(a,b,operation)` returning owning arrays for Python oracle tests:

```python
import numpy as np
from _bits import assert_bitwise_equal

def test_blas_rectangular_matmul_matches_selected_numpy(native_module):
    rng = np.random.default_rng(6021)
    a = rng.normal(size=(3, 17))
    b = rng.normal(size=(17, 5))
    actual = native_module._debug_platform_blas(a, b, "matmul")
    assert_bitwise_equal(actual, a @ b)
```

Parameterize dimensions 0,1,2,3,7,16,17,64,127,128,129 and C/F contiguous/transposed cases actually supported by the binding. Reject unsupported overlapping or noncontiguous buffer shapes explicitly rather than misrepresenting strides.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_platform_blas.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Move foreign declarations from the common header into exactly one backend implementation. Keep common typed enums/signatures and mjtNum/double checks in cblas_abi.hpp. Darwin backend copies the current foreign signatures exactly. Linux declares qualified ILP64 symbols using int64_t and generated validated assembler aliases, including the platform's correct leading-underscore convention; do not reuse the Mach-O convention on ELF.

The safe widening direction is explicit:

```cpp
namespace blas {
double ddot(int n, const double* x, int incx,
            const double* y, int incy) noexcept {
    return backend_ddot(static_cast<std::int64_t>(n), x,
                        static_cast<std::int64_t>(incx), y,
                        static_cast<std::int64_t>(incy));
}
}
```

Here `backend_ddot` is the Linux foreign declaration defined in that translation unit with the exact generated symbol name and verified 64-bit signature. Define dgemv/dgemm analogously with every integer argument widened; enum ABI values stay 101/102/111/112 and get compile-time assertions. No reinterpret_cast of integer storage. The backend C API cannot throw; preserve noexcept wrappers and put shape validation at the existing owning call boundaries.

Compile only the selected backend into bike_native_numerics. Apply first-party policy directly to it; link the resolved numerical shared object/framework through its PUBLIC usage requirements so Python-free tests receive it too. Keep release performance by checking compiler output or LTO feasibility only after correctness; do not add dynamic discovery in ddot calls. The debug hook must invoke the same wrappers used by real writers, not a second reference implementation.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_platform_blas.py -q
```


Build and run the standalone adapter harness on both platforms before whole-project integration. After T10 run the Python bitwise matrix plus existing test_native_brake_resistance, test_native_drivetrain, test_native_rider_forces and distributed/pneumatic suites. Exercise import order NumPy-first/native-first in separate processes. Inspect the linked dependency table to confirm the selected library. Any small-matrix NumPy dispatch mismatch goes to T11, not a tolerance change.

- [ ] **Step 5 - Review and commit.**

```sh
git add native/src/cblas_abi.hpp native/src/binding.cpp native/src/platform native/tests/test_platform_blas.cpp native/cmake/BikeNumerics.cmake tests/reference/test_native_platform_blas.py
git commit -m "feat: add NumPy-matched Linux CBLAS backend"
```

**Review gate:** All existing wrapper signatures remain compatible, every foreign integer argument has the right width, macOS symbols are unchanged, and actual callers use the tested adapter.

### T08: Isolate DGELSD and preserve least-squares workspace semantics

**Files:** Modify `native/src/rider/least_squares.cpp:49-75,202-268`, `least_squares.hpp`; create `native/src/platform/lapack.hpp`, `lapack_accelerate.cpp`, `lapack_numpy_openblas.cpp`, `native/tests/test_platform_lapack.cpp`; extend `tests/reference/test_native_attachment_wrench.py`.

**Interfaces:** Keep rider::lapack_int=int64_t and LeastSquaresWorkspace public API. Add `numeric_backend::dgelsd` with all existing 14 pointer arguments using int64_t integer storage. Both backend translation units implement exactly that signature.

**Reason and boundaries:** The solver deliberately mirrors NumPy buffer ordering, lwork query, rank, residuals and FP-flag handling. Replacing it with a convenient LAPACKE driver or QR solver would violate the existing oracle contract.

- [ ] **Step 1 - Add the regression and a valid control.**

Retain all current least-squares tests and add a backend-specific multiple-RHS/strided-input control:

```python
import numpy as np
from _bits import assert_bitwise_equal

def test_linux_and_mac_lstsq_use_numpy_workspace_contract(native_module):
    rng = np.random.default_rng(808)
    a = rng.normal(size=(9, 4))
    b = rng.normal(size=(9, 3))
    expected = np.linalg.lstsq(a, b, rcond=1e-12)
    actual = native_module._rider_least_squares(a, b, 1e-12)
    for got, want in zip(actual, expected):
        assert_bitwise_equal(np.asarray(got), np.asarray(want))
```

Confirm the existing binding's named argument/default signature in attachment_binding.cpp before using a positional rcond. Add empty dimensions, rank-deficient, underdetermined, singular-value cutoff, nonfinite behavior and prior FE_INVALID controls using the existing test helpers.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_attachment_wrench.py -k least_squares -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Declare the common driver without changing its workspace contract:

```cpp
#pragma once
#include <cstdint>
namespace numeric_backend {
using integer = std::int64_t;
void dgelsd(const integer* m, const integer* n, const integer* nrhs,
            double* a, const integer* lda, double* b, const integer* ldb,
            double* s, const double* rcond, integer* rank, double* work,
            const integer* lwork, integer* iwork, integer* info) noexcept;
}
```

Move `_dgelsd$NEWLAPACK$ILP64` and its static signature assertions into lapack_accelerate.cpp. Linux binds the exact T06-qualified ILP64 DGELSD export. The common function forwards to one exact declaration; no LP64 pointer casts. Add static_assert for all widths and binary64.

Replace only the two foreign calls in LeastSquaresWorkspace with numeric_backend::dgelsd. Preserve m/n/nrhs validation, LDA/LDB maxima, row/column-major conversion, query_lwork=-1, NumPy's actual returned workspace size, query-cache keys and empty-workspace safety. Do not round workspace upward merely for convenience: blocking decisions can change floating-point results. Verify the selected provider's iwork-query behavior against pinned NumPy source and tests; if that provider follows a different documented query rule, implement its exact NumPy caller rule in the backend, not a guessed buffer size.

Preserve info-to-LeastSquaresError, rank reporting, singular values, empty/full-rank residual shape, nonfinite flow and FE_INVALID restoration. Fatal/corruption tests run in subprocesses with a timeout. Workspace reuse remains instance-owned; no shared global scratch or hidden BLAS thread mutation.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_attachment_wrench.py -k least_squares -q
```


Run complete test_native_attachment_wrench.py, not only the new test, on both platforms. Add standalone C++ query/solve/valid-retry tests to CTest. Run ASan/UBSan after T15. Confirm existing attachment/equality recovery fixtures retain full-rank and residual policy; solver portability must not change caller acceptance criteria.

- [ ] **Step 5 - Review and commit.**

```sh
git add native/src/rider/least_squares.cpp native/src/rider/least_squares.hpp native/src/platform/lapack* native/tests/test_platform_lapack.cpp tests/reference/test_native_attachment_wrench.py
git commit -m "feat: add typed platform DGELSD backends"
```

**Review gate:** Correct ABI and workspace sizes; exact local NumPy outputs; unchanged exceptions/rank/residual semantics; no alternative mathematical solver or broad tolerance.

### T09: Make compiler, architecture and standard-library policies explicit

**Files:** Modify `native/CMakeLists.txt:129-156,218-263,318-456,490-554`; create `native/cmake/BikeCompilerPolicy.cmake`, `tests/portability/test_compiler_policy.py`.

**Interfaces:** Produce `bike_apply_native_policy(target)` applying direct compile/link options and definitions. Record selected required flags, supported optional warning flags, omitted warnings with reasons, compiler family, target architecture, standard-library identity and hardening in the build context.

**Reason and boundaries:** CXX_STANDARD=23 alone is insufficient for std::format and selected hardening; broad Clang-only warnings and -mbranch-protection currently break GCC/x86. Existing effective policy cannot disappear behind unevaluated transitive target properties.

- [ ] **Step 1 - Add the regression and a valid control.**

Use tiny generated CMake fixtures to compile a std::format/std::span/std::bit_cast program and inspect compile_commands.json. Linux x86 commands must not contain `-mbranch-protection=standard`, `-headerpad_max_install_names`, `-framework`, or `-isysroot`. Every first-party command must retain strict C++23 and -ffp-contract=off. Add an intentionally unsupported warning name and require a recorded omission, but deliberately remove a required numeric flag and require configuration to fail.

```python
def test_linux_x86_policy_has_no_arm_or_apple_flags(policy_fixture):
    commands, report = policy_fixture(system="Linux", machine="x86_64")
    joined = " ".join(commands)
    assert "-ffp-contract=off" in joined
    assert "-mbranch-protection=standard" not in joined
    assert "-headerpad_max_install_names" not in joined
    assert report["cxx_standard"] == 23
```

Define policy_fixture in this test module to configure the tiny project with the actual selected compiler; a mocked JSON fixture is not sufficient for compiler support claims.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_compiler_policy.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Factor required policy from compiler-family warning lists:

```cmake
include(CheckCXXCompilerFlag)
function(bike_require_cxx_flag flag result_var)
  check_cxx_compiler_flag("${flag}" ${result_var})
  if(NOT ${result_var})
    message(FATAL_ERROR "Required native compiler flag is unsupported: ${flag}")
  endif()
endfunction()
bike_require_cxx_flag("-ffp-contract=off" BIKE_HAS_FP_CONTRACT_OFF)
```

Include CheckCXXSourceCompiles to exercise the actual C++23 library features used by native_bench and writers. For Clang probe warnings under `-Werror=unknown-warning-option`; for GCC use the positive warning spelling, because GCC may tolerate unknown negative warnings until another diagnostic occurs. Keep the existing Clang/AppleClang list where supported and define GCC equivalents rather than passing all Clang options to GCC. Record omissions; do not remove -Werror globally.

Gate `-mbranch-protection=standard` by aarch64 target plus successful compile/link probe. Keep Apple linker header padding only on Darwin. Keep Release's final -O2 override and verify effective ordering after nanobind. Reject fast-math/reassociation flags in the final compile database. Probe stack protection, automatic-variable initialization and strict-flex-arrays; required safety policy fails when unavailable or receives a documented, reviewed equivalent, not a silent skip.

Detect libc++ using a normal standard header and _LIBCPP_VERSION; detect libstdc++ using __GLIBCXX__. Preserve EXTENSIVE/FAST libc++ mode checks on both OSes. On libstdc++ use _GLIBCXX_ASSERTIONS as the non-ABI-changing checked mode; never introduce _GLIBCXX_DEBUG into production module ABI. On glibc, enable _FORTIFY_SOURCE=3 only for compatible optimized configurations, record Debug/unoptimized behavior explicitly, and avoid sanitizer macro redefinition. Sanitizer-specific flags are finalized in T15-T17. All first-party targets receive direct policy through the function, including new numerics and direct tests.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native python -m pytest tests/portability/test_compiler_policy.py -q
```


Compile the fixture and whole project after T10 under Linux Clang, Linux GCC and existing macOS AppleClang/LLVM. Check actual compile_commands and linker invocations. Re-run diagnostic positive/negative controls and contract allocation/exception tests; stack-frame budgets and hardening assertion controls must remain meaningful in Release and Debug.

- [ ] **Step 5 - Review and commit.**

```sh
git add native/CMakeLists.txt native/cmake/BikeCompilerPolicy.cmake tests/portability/test_compiler_policy.py
git commit -m "build: make native compiler policy platform-aware"
```

**Review gate:** Equivalent strict policy is measurable in actual target commands. Unknown warnings are not build failures; unsupported required safety/numerics features are not hidden.

### T10: Integrate portable linkage, RPATH and the complete native target graph

**Files:** Modify `native/CMakeLists.txt:470-977`; create `native/cmake/BikeLinkage.cmake`, `tests/reference/test_native_platform_linkage.py`; update target registration for native/tests/test_platform_blas.cpp and test_platform_lapack.cpp.

**Interfaces:** Consume Bike::MuJoCo, Bike::Numerics and bike_apply_native_policy. Produce all existing targets unchanged in name, plus `native_platform_blas_tests` and `native_platform_lapack_tests`. Expose the same build artifact locations to application/test loaders.

**Reason and boundaries:** A module-only link fix misses contract tests, distance/station tests and benchmarks. ELF dependencies and Mach-O install names need different handling; source/ODR manifests must still cover every first-party translation unit.

- [ ] **Step 1 - Add the regression and a valid control.**

Add a no-environment-rescue subprocess smoke test using the selected artifact:

```python
import os
import subprocess
import sys
from pathlib import Path

def test_native_import_without_library_path_rescue(native_module, tmp_path):
    build = Path(native_module.__file__).resolve().parent
    env = dict(os.environ)
    for key in ("LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH", "DYLD_FALLBACK_LIBRARY_PATH"):
        env.pop(key, None)
    code = ('import sys; sys.path.insert(0, sys.argv[1]); '
            'import bike_native; print(bike_native.runtime_identity())')
    result = subprocess.run([sys.executable, "-c", code, str(build)],
                            cwd=tmp_path, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
```

Run this control only for a regular non-sanitized build; sanitizer preload requirements have separate tests. Add direct execution controls for every CTest executable, not just Python import.

- [ ] **Step 2 - Run the smallest test before implementation.**

```sh
uv run --frozen --group native python -m pytest tests/reference/test_native_platform_linkage.py -q
```

Expected: the new behavior fails for the stated reason, not because an unrelated package, fixture, or compiler is missing. Record the failure. If the current checkout already satisfies it, retain the test and document that result instead of manufacturing a failure.

- [ ] **Step 3 - Implement the bounded change.**

Create linkage helpers that distinguish platform semantics:

```cmake
function(bike_link_engine target)
  target_link_libraries(${target} PRIVATE Bike::MuJoCo ${CMAKE_DL_LIBS})
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "${MJ_DIR}")
    set_property(TARGET ${target} PROPERTY BUILD_RPATH_USE_ORIGIN TRUE)
  endif()
endfunction()
```

Add resolved NumPy provider directories to numerical consumers' BUILD_RPATH on Linux. External environment directories may remain absolute for developer builds; do not claim BUILD_RPATH_USE_ORIGIN makes arbitrary virtual environments relocatable. A relocated source bundle rebuilds locally. Do not set LD_LIBRARY_PATH in package code.

On Darwin preserve the existing MuJoCo framework-install-name workaround for all affected final binaries, but obtain install_name_tool using find_program and inspect `otool -L`/the configured library's install name before rewriting. Make the post-build action deterministic/idempotent, preserve header padding, and do not run it on Linux. Do not copy a guessed framework directory into the wheel.

Link engine/core/numerics according to their dependency graph. Keep bike_native_contract_core as a single object-library compilation; do not also list its sources directly in bike_native or tests. Add CMAKE_DL_LIBS where dladdr is used; use find_package(Threads REQUIRED) and Threads::Threads for actual threading requirements. Keep nanobind/Python headers SYSTEM and first-party warnings PRIVATE. Shared/static objects used by the extension remain PIC.

Register the two direct adapter tests with CTest and the target-derived source manifest. Ensure the native_bench source under tools/proto_native_bench remains selected. Remove every fixed dylib/framework reference outside Darwin modules. Update dependency path lists to actual MuJoCo/numerics libraries. The source fingerprint already includes native/cmake .cmake/.in and native/src templates; keep that behavior and include any new generated-header templates, never generated build output. Emit context schema 2 together with T13; if integrating this task first, keep both generator and consumer change in one integration commit so the main branch is not permanently broken.

- [ ] **Step 4 - Verify the deliverable and negative paths.**

```sh
uv run --frozen --group native cmake -S native -B native/build/release -DCMAKE_BUILD_TYPE=Release
uv run --frozen --group native cmake --build native/build/release --parallel
uv run --frozen --group native ctest --test-dir native/build/release --output-on-failure
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python -m pytest tests/reference/test_native_platform_linkage.py tests/reference/test_native_platform_blas.py tests/reference/test_native_attachment_wrench.py -q
```

On Linux inspect readelf -d/ldd for each final binary; on macOS inspect otool -L. Confirm no source-machine path beyond the selected current environment, one engine image, correct numerics image, and no accidental Python dependency for intended Python-free C++ tests. Repeat a second incremental build and a clean Debug build. Run source manifest/ODR checks after T13/T14.

- [ ] **Step 5 - Review and commit.**

```sh
git add native/CMakeLists.txt native/cmake/BikeLinkage.cmake tests/reference/test_native_platform_linkage.py
git commit -m "build: link all native targets on Linux and macOS"
```

**Review gate:** All final targets configure/link/run on each required host; no unresolved private symbols, duplicate TU ownership, Apple-only Linux command, or environment-variable workaround.
