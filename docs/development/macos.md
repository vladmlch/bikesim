# macOS development guide

## Spatial product — pending V08

The source-authored spatial application uses the existing `@loader_path`
install rpaths, copied-library load-name relocation and native dependency
owner. Examples, schema2 catalog, ABI2 header and spatial libraries travel with
the normal product; no source checkout or Python import is required by the
installed run/report/replay paths. Install/package smokes invoke the same
native `spatial/installed_product.cmake` gate using its built fixture helper.
It keeps logs from a fresh directory outside the checkout and checks both
successful inspection and named rejection of incompatible checked replay.

**Verification is pending V08.** No compiler, executable, installation or GUI
was run for W45–W49. The real-display GUI lane must run the viewer on its main
thread and retain C/1/2/R interaction evidence. The Linux DISPLAY predicate is
not a macOS display detector; a headless macOS session is not GUI evidence.
See [spatial workflows](../spatial-3d.md) and the
[evidence ledger](../superpowers/audits/2026-10-22-spatial-3d-application-evidence.md).

Active instructions for developing bike-sim on macOS (Apple Silicon). The
sibling [Linux guide](linux.md) covers the Linux toolchain; shared failure
diagnosis lives in [troubleshooting.md](troubleshooting.md).

Historical binding lane (measured 2026-10-14/17; not qualification of the Phase 3 root presets):

| Component | Qualified value |
|---|---|
| Host | macOS 27.0.1, **arm64** (Darwin 27) |
| Interpreter | CPython **3.14.3**, uv-managed (`cpython-3.14.3-macos-aarch64-none`), project venv `.venv` |
| Compiler | AppleClang **21.0.0** (`/usr/bin/c++`); analysis lanes use the keg-only Homebrew LLVM toolchain |
| SDK | `MacOSX.sdk` under Xcode 26.6 (default `xcrun` resolution; explicit `CMAKE_OSX_SYSROOT` also qualified) |
| Build tools | CMake 4.4.3, uv 0.12.9 |
| Selected build | `native/build` (artifact `bike_native.cpython-314-darwin.so`); `native/build/release` is the application default |
| Numerics | `ACCELERATE` — system Accelerate.framework, legacy 32-bit CBLAS and 64-bit `NEWLAPACK$ILP64` LAPACK (`dgelsd$NEWLAPACK$ILP64`) |
| Gate evidence | All static-analysis sweeps and headers/diagnostic/context controls green; CTest 10/10; native+golden pytest ~3,203 items (see limits below) |

Recorded limits, not qualifications: macOS **Intel** has no measured lane
and stays unqualified. Real desktop viewer sessions from this wave are
pending on the acceptance matrix
([viewer-platform-acceptance.md](viewer-platform-acceptance.md)); the dated
GUI numbers below in this repository's documents are historical. The
measured `native` gate run recorded `3202 passed, 1 failed` — the retained
failure
`tests/reference/test_native_replay_session.py::test_native_replay_rejects_a_missing_selected_extension`
(diagnostic-text drift in the install-modes workstream) is evidence, not a
waiver. Linux aarch64 is a separately qualified lane, not evidence for
macOS.

## Prerequisites

- The root graph requires no Python. The transition oracle pins **CPython 3.13**
  (`requires-python = ">=3.13,<3.14"`); the 3.14 interpreter in the historical
  baseline above does not satisfy the current oracle contract.
- uv for the project environment; Git; Xcode with the macOS SDK (Xcode
  26.6 measured) or the Command Line Tools; CMake 3.27+ (4.4.3 measured).
- **The OS-default compiler is a candidate, not a promise.** Configure
  probes strict ISO C++23, libc++ hardening (EXTENSIVE by default), the
  Accelerate ABI, the MuJoCo 3.12.0 private ABI, and the selected SDK's
  compatibility with the chosen compiler. An unsupported probe fails
  configure rather than degrading to a "close enough" toolchain.
- Analysis/sanitizer lanes need a full LLVM toolchain, not only AppleClang:
  clang-tidy, the clang analyzer and the RTSan runtime live in the keg-only
  Homebrew LLVM install (`brew install llvm`). Discover it per machine with
  `brew --prefix llvm` — `/opt/homebrew/opt/llvm` on Apple Silicon,
  `/usr/local/opt/llvm` on Intel — never a single hard-coded prefix.

## Transition oracle environment

```sh
uv sync --frozen --group native
```

This creates/uses the checkout's `.venv` with the locked dependency set.
An installer or developer may select another environment by exporting an
absolute **`UV_PROJECT_ENVIRONMENT`** before the first uv command and
keeping it for every step; a relative value is rejected.
`tools/dev.py bootstrap` is the explicit equivalent for
`uv sync --locked --group native --group dev` and accepts `--python`; it
never relocks or upgrades dependencies. The native group pins
`mujoco==3.12.0` and `numpy==2.5.2`; the loaded MuJoCo image must expose
the project's private ABI symbols and the numerical backend is the system
Accelerate provider ([numerical-profiles.md](numerical-profiles.md),
[dependency-matrix.md](dependency-matrix.md)).

## Configure and build with root presets

Acquire the pinned archives and extracted trees described in
[installation](../installation.md), then use the standalone graph:

```sh
cmake --workflow --preset release
cmake --workflow --preset debug
bash tools/run_native_checks.sh build/native-release
```

`release` keeps `build/native-release`; the other preset names use
`build/<preset>`. Ordinary presets use the selected Xcode compiler and SDK;
`rtsan` explicitly selects `/opt/homebrew/opt/llvm/bin/clang++`.
The root graph uses `.deps` and the measured numerics record without
finding an interpreter or loading the extension. SDK selection and actual
compiler/stdlib capability probes remain mandatory. Pass an explicit SDK
with `cmake --preset release -DCMAKE_OSX_SYSROOT=<SDK>` when required.

## Configure diagnostics

Configure now owns the former doctor's native checks: dependency archive
and selected-file hashes, MuJoCo private ABI, compiler/stdlib support,
numerics record and instrumentation controls. Inspect
`build/<preset>/native_check_context.json` and `bike_policy_report.json`
(`build/native-release` for release). A successful configure is not a CTest,
performance, package or GUI acceptance result. The interpreter-specific
checks below apply only to the transition binding graph, which remains
until the Phase 4 deletion gate.

## Transition Python entry points

The application selects `native/build/release` by default, or the absolute
directory in **`BIKE_NATIVE_BUILD_PATH`**:

```sh
export BIKE_NATIVE_BUILD_PATH="$(pwd)/native/build/release"
uv run --frozen --group native bike-ride --backend python --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --headless --duration 1 --no-plots
uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --headless --duration 1 --no-plots
```

`--backend python` is the reference path; `--backend native` requires the
selected artifact and never falls back to Python. Headless runs need no
window and initialize no GLFW context. Strict headless physical runs
terminate at the documented `energy.constraint_work` baseline violation —
a retained diagnostic prefix, not a crash ([RIDE.md](../RIDE.md)).

**Interactive viewers require `mjpython`** from the same environment as
MuJoCo — Cocoa's `NSApplication` event loop must run on the main OS thread.
Every viewer path calls `ensure_macos_mjpython()`
(`src/bike_sim/sim/playground.py`), which re-execs the original argv under
the resolved `mjpython` once, marks the process with `MUJOCO_MJPYTHON=1`,
and restores the environment when the lookup or exec fails. For
**uv-managed interpreters the re-exec needs
`DYLD_FALLBACK_LIBRARY_PATH=<python base_prefix>/lib`**: `mjpython` dlopens
the interpreter by `@rpath`, and a venv keeps `libpython3.x.dylib` in
`base_prefix/lib`, outside dyld's default fallback path — the helper
prepends it automatically only when that dylib exists. Without it the
re-exec dies with `Library not loaded`.

## Tests and analysis

```sh
bash tools/run_tests.sh quick
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/release" bash tools/run_tests.sh native
```

Profiles: `quick` (Python tests minus `slow` minus direct native files;
keeps `test_native_check_tools.py`), `native` (configure/build the selected
directory, all sweeps and controls, CTest, then native+golden pytest —
~3,200 items) and `full` (adds slow physical/realtime episodes; the
documented 24-failure baseline set still applies —
[TESTING.md](../TESTING.md)). `NATIVE_TEST_BUILD_PATH` selects the absolute
build directory; set `BIKE_NATIVE_BUILD_PATH` to the same directory for the
full suite — the conftest fails when the two selectors disagree, and child
`uv run` processes inherit the environment. The legacy
`NATIVE_TEST_BUILD_DIR` values `''`, `asan`, `rtsan`, `coverage` remain
accepted.

The `native`/`full` preflight runs the sweeps — also standalone CMake
targets: `check_frontends` (second-frontend syntax+analyzer), `check_tidy`
(clang-tidy), `check_analyzer` (clang `--analyze`; alpha findings print but
never block), `check_cppcheck`, `check_odr`, `check_scripts` (shellcheck),
plus `headers`, `diagnostic-controls`, `context` and
`check_native_contracts`. Each failed sweep names its CMake target, e.g.
`cmake --build <selected-build> --target check_tidy`. **A missing required
analysis tool is an error, not a skip.** Tools resolve through
`tools/native_toolchain.py` — explicit `CLANG_TIDY`, `ANALYZER_CLANG`,
`CPPCHECK`, `GXX`, `LLVM_PROFDATA`, `LLVM_COV` overrides first, then the
configured compiler's toolchain siblings, qualified PATH candidates, and —
on macOS only — `xcrun --find` inside the selected Xcode toolchain. The
qualified tidy/analyzer set is the Homebrew LLVM toolchain discovered via
`brew --prefix llvm` (AppleClang lacks several checkers); the GCC frontend
for `check_frontends`/`check_odr` was the Homebrew `g++-16` on the measured
host.

## Sanitizers and coverage

ASan/UBSan lane:

```sh
uv run --frozen --group native --no-sync cmake -S native -B native/build/asan -DNATIVE_SANITIZE=ON
uv run --frozen --group native cmake --build native/build/asan
NATIVE_TEST_BUILD_DIR=asan bash tools/run_tests.sh native
```

The sanitizer set is `address,undefined,local-bounds,float-cast-overflow`
with `-fno-sanitize-recover=all`. Every instrumented consumer runs through
`tools/native_runtime.py run --build <build> -- <argv>`, which prepends the
recorded compiler-matched runtime to `DYLD_INSERT_LIBRARIES` at exec time —
setting it in the caller's shell does not survive the `uv run` chain. A
stale or unresolvable runtime fails closed; the import then verifies the
runtime is actually mapped in the process's dyld image list.

RTSan (`-fsanitize=realtime`) verifies the marked warm core never
allocates or locks. AppleClang rejects the flag — configure a separately
qualified LLVM toolchain (the measured lane used Homebrew LLVM 23.1.2;
LLVM >= 20 is the floor):

```sh
LLVM_PREFIX="$(brew --prefix llvm)"
uv run --frozen --group native --no-sync cmake -S native -B native/build/rtsan \
  -DCMAKE_CXX_COMPILER="$LLVM_PREFIX/bin/clang++" -DNATIVE_RTSAN=ON
uv run --frozen --group native cmake --build native/build/rtsan
NATIVE_TEST_BUILD_DIR=rtsan bash tools/run_tests.sh native
```

Coverage (`NATIVE_COVERAGE=ON`) requires an LLVM clang-family compiler and
a `llvm-profdata`/`llvm-cov` pair qualified against it by a real
instrumented control — `LLVM_PROFDATA`/`LLVM_COV` overrides, the configured
compiler's siblings, PATH, then the macOS-only `xcrun --find` fallback:

```sh
tools/native_coverage.sh native/build/coverage
```

Artifacts land in `<build>/native_coverage_runs/run.*` with a
`coverage_provenance.json`; mismatched tools fail with the tool's own
diagnostics. See [TESTING.md](../TESTING.md) for sweep internals and dated
measurement snapshots.

## Editor setup

`native/.clangd` points clangd at `native/build`'s
`compile_commands.json` — CMake writes one per configured build directory
(`CMAKE_EXPORT_COMPILE_COMMANDS=ON`). Select a different build per editor
session with `--compile-commands-dir=<build dir>` (for example
`native/build/asan` or `native/build/release`), and let clangd resolve the
recorded compiler's headers with `--query-driver=<the configured compiler
in that build>` — e.g. `/usr/bin/c++` or the Homebrew LLVM `clang++`, never
a fixed absolute path shared across machines.

## Offline installation

Native-capable offline delivery uses a **schema-1 source bundle**: the
extracted root carries `manifest.json`, `project/` (the source tree),
`wheelhouse/`, `requirements.txt`, `offline-tools.json`,
`dependency-closure.json` and `assembly-report.json`. `tools/offline_bundle.py`
assembles platform-matched bundles online; installation runs entirely from
validated local payloads — no network, no warm-cache dependency.

Entry points: `packaging/install_full.sh` and `packaging/install_slim.sh`
inside an extracted bundle (both delegate to `tools/install_offline.py`).
Their flags are still being qualified by the offline-install task — run
`python3 tools/install_offline.py --help` inside the bundle for the
authoritative surface rather than trusting a copied command line. A native
install creates the source project's environment with
`UV_PROJECT_ENVIRONMENT` pinned to the new venv, syncs the frozen native+dev
closure offline, then configures/builds/doctors locally. Slim bundles
verify every declared external provider at the exact pinned version on the
selected interpreter before and after environment creation; an unqualified
external stack fails with the full bundle as remediation. GUI capability in
an offline install still requires the macOS desktop session and `mjpython`;
headless native installation never needs a window.

Legacy manifest-less bundles remain Python-only and are labeled as such —
a filename never grants native capability. See
[install-modes.md](install-modes.md) for the three installation modes.

## Failure diagnosis

`doctor --build <dir> --numerics` is the first evidence command for any
environment or artifact failure; the symptom → evidence → bounded
remediation table (SDK selection, missing MuJoCo private symbol, provider
mismatch, missing artifact, wrong SOABI, stale source stamp, unresolved
dyld image, `mjpython`/`DYLD_FALLBACK_LIBRARY_PATH`, sanitizer and coverage
tool lanes, strict invalid-controller stop, replay identity, slim
externals) lives in [troubleshooting.md](troubleshooting.md).
## Phase 3 instrumentation and packaging

```sh
cmake --workflow --preset asan
cmake --workflow --preset coverage
ctest --test-dir build/native-release -R '^package_smoke$' --no-tests=error --output-on-failure
```

On Linux, prefix each command with `bash tools/linux/container.sh exec`.
Coverage runs the native tests, merges only fresh profiles and writes
`build/coverage/coverage.lcov` plus `coverage-summary.txt`. ASan and RTSan
executables link their own runtime; no interpreter preload is used by the
root graph. RTSan fails configure if the selected compiler lacks the actual
flag/annotation/runtime contract.

Ordinary presets exclude `slow`, `audit` and `package` labels. Those remain
explicit gates: package smoke is selected above; slow tests and the final
completeness/documentation audits belong to the release checklist. Source
implementation and a recorded command do not establish a passing gate.
