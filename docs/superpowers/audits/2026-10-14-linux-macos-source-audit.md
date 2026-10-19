# Linux/macOS source audit

## Scope and provenance

Input: repomix-output2.xml.zip. SHA-256: cd44e1ad291c887e8cf91303b153ccc9c016284b341b6db4efbdb8f059e4fecc.

The archive contains repomix-output.xml (12,453,251 bytes) and a macOS metadata entry. The bundled repomix-unmixer script restored 1,072 unique, path-validated text files. Every restored text body was compared with its packed text body: 1,072 matches. Original Git executable bits, symlinks, binaries, ignored files, and history are not established by this extraction. No implementation source was edited. See evidence/snapshot-audit.json and evidence/source-file-manifest.json.

## Reproduced blocker

Host used for the configure control: Linux x86_64, Python 3.13.5.

```sh
cmake -S /mnt/data/unmixed-project/native -B /mnt/data/audit-linux-build
```

Exit status: 1. Output:

```text
-- xcrun's default macOS SDK was unavailable:
CMake Error at CMakeLists.txt:119 (message):
  No installed macOS SDK is compatible with compiler '/usr/bin/c++':
-- Configuring incomplete, errors occurred!
```

This is a real configure result, not a successful build or test result. It occurs before dependency discovery. No complete dependency install, native compile, simulation suite, numerical comparison, or GUI acceptance was run. No macOS machine was available for this planning session.

## Findings and task mapping

Line references below are to the extracted snapshot. Match by symbol/section after the implementation starts; line numbers will move.

| ID | Source evidence | Consequence | Tasks |
|---|---|---|---|
| A01 | native/CMakeLists.txt:1-156, especially 4-120 | xcrun, xcode-select, Accelerate and Availability headers required before/after project() on every host | T04 |
| A02 | native/CMakeLists.txt:158-207, 528, 693, 779 | MuJoCo is pinned to 3.12.0 but its library is always named as a dylib; private error ABI must survive Linux linking | T05, T10 |
| A03 | native/CMakeLists.txt:559-566, 592-599, 623-630, 664-669, 699-710 | Framework linkage and install_name_tool applied to executables and extension, not just one target | T10 |
| A04 | native/src/cblas_abi.hpp; native/src/rider/least_squares.cpp:49-75, 202-268; least_squares.hpp:41-44 | Legacy 32-bit CBLAS and Mach-O _dgelsd$NEWLAPACK$ILP64 alias are explicit Apple ABIs | T06-T08 |
| A05 | native/src/numeric_numpy.hpp:1-43; numeric_sincos.hpp:1-31; numeric_norm.hpp:1-27; numeric_sum.hpp | NumPy 2.5.2 ARM/NEON behavior, separate transcendental calls, CPython 3.14.3 compensated norm, and reduction order are contractual | T11 |
| A06 | native/CMakeLists.txt:318-400, 423-456 | Many Clang warnings, aarch64 branch protection, Mach-O linker options, compiler-specific sanitizer/coverage flags | T09, T15-T17 |
| A07 | tools/native_checks.py:156-240, 1343-1733; native/CMakeLists.txt:899-940 | Context validator and control suite assume a real Apple SDK and default Homebrew tool paths | T13-T14 |
| A08 | native/CMakeLists.txt:470-529, 759-898; tools/check_native_contracts.py | Object/core/engine/executable target membership and effective compile options must remain visible to source and ODR checks | T09-T10, T13-T14 |
| A09 | tools/run_tests.sh:23-25, 84-174 | Linux preload branch already exists; keep it and make compiler-runtime selection robust rather than rewriting from zero | T15 |
| A10 | src/bike_sim/native/artifact.py:1-145; tests/reference/native_loader.py | Selected build path and extension suffix logic are already platform-aware; installed wheel default paths still assume checkout layout | T12, T23 |
| A11 | src/bike_sim/native/artifact.py:147-253; native/src/runtime/provenance.cpp; native/CMakeLists.txt:942-977 | Persisted build context has an exact four-field schema and lacks BLAS/NumPy/platform identity; source fingerprints already include native/cmake | T12 |
| A12 | src/bike_sim/sim/playground.py:314-345; cli/ride.py:812-813; cli/replay.py:49-58; ride/viewer.py; research/viewer.py | mjpython guard already returns on Linux; preserve macOS re-exec and test both branches | T18-T19 |
| A13 | src/bike_sim/cli/batch.py:173-183,252; sim/ride/async_close.py:190-211 | Apple sysctl is queried unconditionally but has fallback; batch uses spawn; AsyncCloseRunner explicitly forks | T20 |
| A14 | tools/make_offline_bundle.sh:1-121; packaging/install_full.sh; packaging/install_slim.sh | Linux/cp313 wheel download does not build a native extension; full/slim semantics and compiler prerequisites are incomplete | T24-T25 |
| A15 | tools/install_offline.py:56-114; tools/check_offline_sources.py | Another slim installation route hashes/compares Python sources but hard-codes Linux x86_64/cp313 and mentions absent build helper scripts | T24-T25 |
| A16 | pyproject.toml:1-51; AGENTS.md:1-13 | Python >=3.13; core MuJoCo lower bound differs from native ==3.12.0; native NumPy is unpinned; developer reference sources use one person's /Users path | T01-T03, T26 |
| A17 | docs/RIDE.md:7-81; docs/TESTING.md | Historical macOS full-profile and energy.constraint_work failures exist; a diagnostic prefix is not throughput or blanket Linux acceptance | T01, T27-T30 |
| A18 | tests/reference/conftest.py::_pin_oracle_kernels; tests/reference/_bits.py; test_native_attachment_wrench.py:312-326 | Existing local oracle contract is exact, and Numba is deliberately bypassed in native oracle tests | T11, T27 |
| A19 | native/src/runtime/provenance.cpp:3-18; native/src/stepper.cpp:9,207-214 | dladdr is already used; Linux needs a verified declaration/link context, including CMAKE_DL_LIBS where applicable | T10, T12 |
| A20 | docs/agents/native-engineering.md:5-6 | Linked 2026-10-06 safety spec/master plan are absent from this export; recover before conflicting safety changes | T01 |

## Existing behavior to keep

Application imports do not build a native module and reject stale/conflicting artifacts. tests/reference/native_loader.py uses Python's actual extension suffixes, not a literal .dylib. Native and full already share verification preflight. MuJoCo errors already have a private-ABI wrapper rather than a process-global unsafe fallback. Viewer setup already has a Darwin guard. Batch execution already uses spawn. These are assets to preserve, not missing features to invent.

## Facts, proposals, and unresolved evidence

**Reproduced:** extraction integrity and unconditional SDK configure failure on Linux.

**Source-supported:** all blockers in the table, current declared dependency constraints, current loader and recording/replay schemas, and the documented historical macOS acceptance/failure statements. Historical claims have not been independently rerun here.

**Design proposals:** imported dependency targets, NumPy-matched Linux backend, versioned context/provenance, source-first offline native delivery, initial Ubuntu baseline, and the work/task division.

**Implementation-time evidence required:** real Git revision/dirty tree and excluded files; exact baseline interpreter/NumPy/compiler/SDK; complete wheel availability and dependency resolution; Linux wheel export names and integer widths; same-host bitwise numerical parity; all runtime/GUI/analysis/sanitizer/replay gates. The task list supplies the measurements and stop conditions, not fabricated results.

## Primary external references checked during planning

These references validate mechanisms, not this repository's completion. URLs are included so the handoff remains self-contained. Consult version-specific source when implementing a pinned ABI.

- E01: MuJoCo Python documentation, https://mujoco.readthedocs.io/en/stable/python.html . Describes the macOS mjpython requirement and separate rendering/context APIs. Do not treat an offscreen context as an interactive GLFW window.
- E02: MuJoCo 3.12.0 release files, https://pypi.org/project/mujoco/3.12.0/ . CPython 3.13 Linux x86_64/aarch64 and macOS wheels are listed; existence does not verify the private symbols this project uses.
- E03: NumPy BLAS/LAPACK build documentation, https://numpy.org/doc/stable/building/blas_lapack.html . Backend and integer-interface selection are explicit build concerns; generic BLAS compatibility does not establish identical results.
- E04: NumPy SIMD documentation, https://numpy.org/doc/stable/reference/simd/index.html . CPU dispatch is a separate numerical variable from BLAS choice.
- E05: NumPy 2.5.2 source, https://github.com/numpy/numpy/blob/v2.5.2/numpy/linalg/umath_linalg.cpp . Use the actual DGELSD call, workspace and buffer behavior as the reference.
- E06: NumPy 2.5.2 release files, https://pypi.org/project/numpy/2.5.2/ . Different wheel tags/OS floors exist even for one version; choose the actual installed wheel, not an assumed Accelerate/OpenBLAS mapping.
- E07: CMake FindBLAS, https://cmake.org/cmake/help/latest/module/FindBLAS.html . BLAS::BLAS and BLA_SIZEOF_INTEGER exist; the generic Fortran BLAS finder alone does not prove a CBLAS or NumPy-specific symbol contract.
- E08: CMake BUILD_RPATH_USE_ORIGIN, https://cmake.org/cmake/help/latest/prop_tgt/BUILD_RPATH_USE_ORIGIN.html . Relative build-tree RPATH is supported where $ORIGIN exists; it does not automatically relocate external virtual-environment dependencies.
- E09: uv synchronization, https://docs.astral.sh/uv/concepts/projects/sync/ . Locking, frozen execution, optional groups and exact synchronization must be handled consistently by all project wrappers.
- E10: Clang ASan, https://clang.llvm.org/docs/AddressSanitizer.html . Instrumented extension loading needs the matching runtime; do not guess from the executable basename alone.
- E11: Clang RTSan, https://clang.llvm.org/docs/RealtimeSanitizer.html . Probe selected compiler/runtime support; do not infer it from AppleClang/Clang version branding.
- E12: Python multiprocessing, https://docs.python.org/3/library/multiprocessing.html . Start methods and safety constraints differ; explicit fork with pre-existing threads requires scrutiny.

- E13: GitHub-hosted runner reference, https://docs.github.com/en/actions/reference/runners/github-hosted-runners . The proposed Ubuntu 24.04 and macOS 15 runner lanes are based on documented labels as checked on 2026-10-14; assert the actual architecture at job start and re-verify availability before enabling the workflow.
- E14: actions/checkout v7.0.1 release, https://github.com/actions/checkout/releases/tag/v7.0.1 . The example workflow pins reviewed commit 3d3c42e5aac5ba805825da76410c181273ba90b1; retain a recovered project action policy when it is stricter, and re-verify the immutable ref during implementation.

Full dependency and toolchain support was not independently qualified from these pages. The plan deliberately makes qualification a measured task.
