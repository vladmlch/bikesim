# Linux and macOS portability design

Date: 2026-10-14. Status: proposed implementation contract, not an implemented feature.

## 1. User requirement and interpretation

The project must be runnable, compilable, and developable on Linux **as well as macOS**. macOS support must not be removed. "Runs" includes the Python reference path, native physical/research path, interactive viewers, headless execution, recording, analysis, and replay within their existing capability limits. "Developable" includes editable Python installation, native Debug/Release builds, selected-artifact imports, unit/contract/oracle tests, static analysis, sanitizer builds, coverage, and documented dependency installation.

This is an additive portability change. It is not a physics rewrite, a transition to a different engine, a replacement of Accelerate on macOS, or authorization to weaken validation.

## 2. Current source evidence

The companion audit records exact source locations. The main issues are: unconditional Apple SDK configuration; fixed MuJoCo dylib paths and install-name rewriting; Apple CBLAS and ILP64 LAPACK linkage; compiler/architecture-specific flags; SDK-shaped analysis manifests; NumPy/CPython numerical assumptions; incomplete native offline delivery; and insufficient environment-specific provenance. The actual Linux configure failure was reproduced from this snapshot before any source modification.

The snapshot contains 1,072 extracted text files, including 159 under native/, 283 under src/, 154 under tests/, and 34 under tools/. It does not contain uv.lock, .python-version, .github workflows, Git history, or binary artifacts. Absence here does not prove absence in the real repository. The nested project/ subtree has two historical files, not a second application root.

## 3. Supported-platform contract

### Release-blocking initial targets

| Target | Required scope | Notes |
|---|---|---|
| Existing macOS Apple Silicon environment | All existing supported commands, native checks, real viewers, offline source/development route | Recover the actual OS/SDK/compiler/Python/NumPy baseline rather than replacing it with a convenient runner. |
| Ubuntu 24.04 x86_64, glibc | Python + native Debug/Release + full development gate + headless + X11/GLFW viewer | Chosen initial Linux baseline, not a claim that every Linux distribution is equivalent. |
| Linux x86_64 with both Clang and GCC | Strict C++23 build and native contract/oracle tests on both configured compiler families | Pin concrete passing toolchain versions in the implementation's environment ledger. |
| CPython 3.13 and the actual current macOS interpreter | Dependency and numerical compatibility | Source comments refer to CPython 3.14.3; do not silently abandon either the declared 3.13 floor or the real baseline. |

Linux aarch64 and macOS x86_64 are additional validation lanes. Preserve pre-existing support on either if the baseline proves it existed. Otherwise label them unqualified until the same gates pass. Keep the code architecture-neutral rather than hard-coding x86_64 as the meaning of Linux. Wayland via XWayland is a separate manual acceptance row; native Wayland, EGL, OSMesa, GPU acceleration, musl, Windows, and cross-compilation are not substitutes for the required desktop/headless targets.

Do not infer a whole-application glibc or macOS deployment floor from one wheel. The intersection of Python, NumPy, SciPy, MuJoCo, Numba, DuckDB, Arrow, and native toolchain requirements determines the supported environment.

## 4. Architecture decisions

### D1. Resolve platform before Apple SDK work

Move existing Apple pre-project SDK logic unchanged into native/cmake/BikeAppleSDK.cmake. Dispatch using CMAKE_HOST_SYSTEM_NAME before project(), because compiler selection happens inside project() and APPLE is not a safe substitute at that point. Explicitly reject cross-compilation for this iteration; after project() verify host/target agreement. Linux performs no xcrun, xcode-select, Accelerate-header, install_name_tool, or Apple SDK probe. A Linux caller supplying CMAKE_OSX_SYSROOT receives a precise invalid-option diagnostic.

### D2. One selected interpreter and one MuJoCo image

Resolve Python once through CMake, then ask that interpreter for MuJoCo and nanobind locations. Create Bike::MuJoCo as an imported target referring to the exact selected wheel library. Keep the 3.12.0 version check and the compile/link control for _mjPRIVATE_setTlsLogHandler, _mjPRIVATE_getGlobalLogHandler, and mj_makeRawData. Neither a system libmujoco nor a newer public-ABI-compatible release is an acceptable fallback.

### D3. Preserve numerical identity, not merely API compatibility

Add BIKE_NUMERICS_BACKEND=AUTO|ACCELERATE|NUMPY_OPENBLAS. AUTO selects the existing Accelerate ABI on Darwin and the BLAS/LAPACK shared object actually used by the selected NumPy on Linux. Linux must not silently link a distribution OpenBLAS while NumPy uses a different bundled OpenBLAS. A matching vendor name alone is insufficient: library path, content identity, integer widths, exported symbol names, threading implementation, and CPU dispatch matter.

Keep the current public CBLAS wrapper names and public rider::lapack_int=int64_t. Put foreign ABI declarations and conversions into small platform files. Accelerate keeps its legacy 32-bit CBLAS and 64-bit NEWLAPACK DGELSD. Linux initially qualifies the NumPy wheel's verified ILP64 OpenBLAS profile. Reject unknown, mixed, or LP64 profiles with installation guidance rather than reinterpreting int64_t buffers as int32_t. An LP64 implementation is a separate explicitly tested backend, not an automatic fallback.

BLAS discovery and symbol qualification happen during configuration/doctor checks, never in the physics hot path. Bind a concrete shared object with normal linker/imported-target semantics. Do not use process-global dlsym or mutate BLAS thread settings after worker startup. Do not introduce Python callbacks into C++ numerical kernels.

### D4. Preserve the oracle and separate portability from reproducibility

Native must continue to match the local, pinned reference arithmetic at existing exact test points. Keep tests/reference/_bits.py and the _pin_oracle_kernels fixture unchanged in meaning. Production Python Numba behavior and pinned pure-Python oracle behavior are separate test lanes. Scalar libm, NumPy array ufuncs, BLAS reductions, CPython math.hypot, signed zero, NaN payloads, and LAPACK workspace choices need different probes.

Do not change every assertion to allclose, add broad platform skips, adjust energy tolerance, replace the Python oracle, or disable SIMD globally to conceal failures. Where a Linux numerical lowering differs, identify the exact upstream kernel/call shape and implement the matching lowering behind the platform boundary. Identical local BLAS libraries are necessary but do not prove NumPy dispatches the same operation for every array shape.

Cross-platform checked replay may legitimately reject a numerical identity mismatch. Cross-platform data inspection and non-simulating playback should remain usable when the serialized data/assets support them. They must never be labeled a successful checked physical replay.

### D5. Direct target policy and portable dependency graph

Use bike_apply_native_policy(target) to apply direct first-party flags/definitions and preserve their visibility in the existing manifest. Build policy probes distinguish Clang, AppleClang, GCC, libc++, libstdc++, Darwin, Linux, and target architecture. Required C++23, no-fast-math, disabled implicit FP contraction, and effective selected hardening are errors if unsupported. Optional warning names can be probed, but every omission is recorded. No unconditional -mbranch-protection on x86 and no Apple linker switches on ELF.

Keep source selection target-derived. Bike::Numerics and Bike::MuJoCo must reach the extension, contract tests, distance/station tests, and native benchmark according to their real symbol dependencies. Link CMAKE_DL_LIBS where dladdr is used and Threads::Threads where thread linkage is required. Preserve single ownership of object-library translation units and avoid duplicate ODR-sweep inputs.

### D6. Build-only and persisted identities have separate versioning

Bump native_check_context.json to schema_version=2 for platform-aware development checks. Linux has sdk=null, sdk_requested=null, sdk_source="not-applicable"; an explicit non-Apple sysroot is recorded separately. Add platform, standard-library/hardening, numerics, resolved tool capabilities, and effective policy information. Accept old schema 1 as a validated legacy Darwin context only; do not treat it as a Linux manifest.

Persisted execution identities gain identity_schema=2 while runtime_schema remains 1 unless runtime behavior actually changes. Legacy identities without identity_schema continue to validate under the old exact field set. New native build_context contains the existing four fields plus structured platform and numerics records. New Python identities also use identity_schema=2, with build_context containing exactly platform and numerics; their existing native-only hash fields remain null. Legacy Python identities retain null build_context. Legacy data stays readable, but a v2 checked replay rejects missing environment identity before stepping rather than silently projecting away new fields. Existing recording container version numbers and data tables do not change merely because build metadata grows. Replay must provide a field-level incompatibility explanation before stepping.

### D7. Keep viewer and process behavior explicit

Retain ensure_macos_mjpython() and all existing calls. Linux does not re-exec mjpython. Pure headless execution must not initialize a GLFW window or force an offscreen rendering backend. GUI availability is checked at the interactive boundary without making environment-variable presence the sole test of availability.

Guard Apple's performance-core sysctl query. Continue to use the existing affinity-aware CPU fallback and explicit spawn for batch workers. Audit AsyncCloseRunner's explicit fork separately: a NumPy-only child can still inherit BLAS locks from a threaded parent. Qualify a module-safe spawn/forkserver transport before changing it; retain behavior until equivalence and shutdown tests pass, and do not ship Linux async execution with a demonstrated deadlock.

### D8. Deliver a source-first native development installation

Keep the existing setuptools Python package for this port. Add an explicit bootstrap/build/doctor route that operates from the real checkout or included source tree. Do not claim a py3-none-any wheel contains bike_native. Native-capable offline bundles include source, the correct dependency/development wheels, build instructions, platform manifest, and a native build performed on the target. Python-only bundles remain honestly labeled and cannot claim ordinary native recording support.

Full/slim modes are preserved. Slim validates the exact scientific environment needed for the native profile, not just very broad package minimums. Do not copy a virtual environment or a macOS binary to Linux. Build directories must not leak into source bundles. Relocated source bundles rebuild their extension and refresh build manifests locally; distribution of relocatable native wheels is a separate packaging project.

## 5. Shared interfaces

- tools/native_dependencies.py: probe_dependencies(python: Path) -> dict; validate_dependencies(value: dict) -> dict; CLI --python PATH --output PATH. Probe runs the selected interpreter in a subprocess and returns versioned JSON.
- Dependency JSON schema_version=1: platform {system,machine,libc}; python {executable,version,soabi}; mujoco {version,include_dir,library_path}; nanobind {cmake_dir,include_dir}; numerics {backend,numpy_version,library_paths,blas_integer_bits,lapack_integer_bits,symbols,threading,provider_config}. All filesystem paths are absolute and all required libraries are verified. A Darwin system Accelerate image is verified by real framework/SDK/loaded-provider evidence, not a NumPy wheel RECORD entry; SDK stubs and dyld-cache images must not be represented as hashable provider files. T12 defines the separate file and apple-system identity variants.
- Numerics symbols keys: ddot, dgemv, dgemm, dgelsd; each has a nonempty exported identifier and separately validated ABI width. Configured filenames/symbol identifiers are validated before generating CMake or C++ content.
- native/src/cblas_abi.hpp retains blas::Order, blas::Transpose, blas::ddot, blas::dgemv, blas::dgemm and their existing signatures.
- native/src/platform/lapack.hpp declares numeric_backend::dgelsd with the same pointer argument sequence now declared in rider/least_squares.cpp, using rider-independent int64_t backend integers. Shared LeastSquaresWorkspace owns buffers and exceptions.
- tools/dev.py CLI: bootstrap [--python VERSION] [--offline --wheelhouse PATH], configure --build PATH --config Debug|Release [--compiler PATH], build --build PATH, doctor [--build PATH] [--numerics], test --build PATH --profile quick|native|full. Every subprocess uses argument arrays, propagates status, and never inserts shell-evaluated flags.
- doctor output schema_version=1 with status=pass|fail and individually named checks containing status, evidence, and remediation; missing native binary is a failure only when a native build check was requested.
- Platform-specific metadata is diagnostic unless explicitly part of the numerical identity. Hostname, timestamps, temporary roots, and absolute build paths must not make otherwise equivalent persisted identities differ.

## 6. Acceptance and non-regression

Release requires successful fresh-source bootstrap, Debug/Release compile, CTest, direct native oracle collection with nonzero item count, native profile, full-profile baseline comparison, selected-loader checks, native error-ABI subprocess controls, sanitizer controls, coverage generation, Linux and macOS GUI acceptance, headless no-display acceptance, batch execution, opt-in recording/finalization, analysis, same-environment checked replay, cross-environment mismatch diagnostics, and an offline source-bundle install with network blocked.

A historical macOS full-profile failure is not a waiver for a new Linux failure. Record exact test IDs, failure signatures, source commit, dependencies, and first-divergence evidence. Never normalize every invalid-controller termination into success. Performance comparisons use warm JIT/BLAS conditions and the same hardware before/after; no cross-machine absolute timing assertion.

## 7. Execution boundaries

The ordered work packages in the master plan implement this contract. Implementation agents may make mechanical fixes required by a failing portability test, but must not broaden scope into the adjacent spatial-3D plans or unrelated historical physics work. Preserve user changes. Each review identifies which tests ran, which could not run on that agent's host, and the exact remaining release gates. A platform is not qualified by another platform's green build.
