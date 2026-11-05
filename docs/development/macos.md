# macOS development guide

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
and stays unqualified. The dated
GUI numbers below in this repository's documents are historical. The
measured `native` gate run recorded `3202 passed, 1 failed` — the retained
failure
`native_replay_session/native_replay_rejects_a_missing_selected_extension`
(diagnostic-text drift in the install-modes workstream) is evidence, not a
waiver. Linux aarch64 is a separately qualified lane, not evidence for
macOS.

## Prerequisites

- The root graph requires no interpreter. The transition oracle's
  **CPython 3.13** pin, the `bike_native` extension, the uv `--group native`
  dependency group and the tools/dev.py bootstrap were all removed at T63;
  the 3.14 interpreter in the historical baseline above is a recorded
  measurement, not a requirement.
- Git; Xcode with the macOS SDK (Xcode
  26.6 measured) or the Command Line Tools; CMake 3.27+ (4.4.3 measured).
- **The OS-default compiler is a candidate, not a promise.** Configure
  probes strict ISO C++23, libc++ hardening (EXTENSIVE by default), the
  Accelerate ABI, the MuJoCo 3.12.0 private ABI, and the selected SDK's
  compatibility with the chosen compiler. An unsupported probe fails
  configure rather than degrading to a "close enough" toolchain.
- Analysis/sanitizer lanes need a full LLVM toolchain, not only AppleClang:
  clang-tidy and the RTSan runtime live in the keg-only
  Homebrew LLVM install (`brew install llvm`). Discover it per machine with
  `brew --prefix llvm` — `/opt/homebrew/opt/llvm` on Apple Silicon,
  `/usr/local/opt/llvm` on Intel — never a single hard-coded prefix.

## Dependency acquisition

The standalone graph consumes the pinned lock at
`native/dependencies.lock.json`. Populate `.deps` with the checked-in
acquisition helper, which verifies every archive and member hash before
placing files:

```sh
bash tools/linux/acquire_deps.sh native/dependencies.lock.json .deps darwin-arm64
```

For local development, `mise run deps` (`tools/link_deps.sh`) runs that
same acquisition once into a shared copy,
`~/.cache/bike-deps/<platform>-<lock hash>` (override with
`BIKE_DEPS_CACHE`), and makes `<checkout>/.deps` a link to it. Every
checkout and worktree then shares one copy. A real `.deps` already in the
checkout is adopted when it equals a fresh acquisition, so an existing build
does not recompile. Offline bundles and the Linux container keep using
`acquire_deps.sh` directly.

Configure points at the result with `-DBIKE_DEPENDENCY_ROOT=<checkout>/.deps`
(the root presets already default there). The loaded MuJoCo image must
expose the project's private ABI symbols and the numerical backend is the
system
Accelerate provider ([numerical-profiles.md](../agents/native-numerical-profiles.md). The transition oracle's
`uv sync --frozen --group native` environment, `UV_PROJECT_ENVIRONMENT`
override and `tools/dev.py bootstrap` helper were removed at T63; there
is nothing to sync.

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
performance, package or GUI acceptance result.

## Running the native executables

Build products land in `build/<preset>/bin` (`build/native-release/bin`
for the release preset). The CLI family — `bike-ride`, `bike-sim`,
`bike-replay`, `bike-research`, `bike-batch`, `bike-settle`, `bike-export`,
`bike-recording` — plus the lane executables
`bike-native-ride`, `bike-native-checks`, `bike-validate` and
`bike-measure-realtime` run directly; there is no interpreter, extension
artifact or environment to select. The retired `BIKE_NATIVE_BUILD_PATH`
selector and the `--backend` switch survive only as fail-closed CLI
surface (`--backend` accepts `native` and rejects anything else); the
interpreter backend was removed at T63.

```sh
build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --headless --duration 1 --no-plots
```

Headless runs need no
window and initialize no GLFW context. Strict headless physical runs
stop at the first model violation; the 1 s ride above has none and exits 0 (measured on darwin-arm64, 2026-11-04) ([RIDE.md](../RIDE.md)).

**Interactive viewers run on the main OS thread** — Cocoa's
`NSApplication` event loop requirement is a native constraint, not an
interpreter one. The `mjpython` re-exec machinery and its
`DYLD_FALLBACK_LIBRARY_PATH` workaround died with the interpreter at T63;
the native viewer binary simply must launch from the process's main
thread, which the produced executables already do.

## Tests and analysis

```sh
bash tools/run_tests.sh quick
bash tools/run_tests.sh native
```

Profiles: `quick` (one `cmake --workflow --preset <preset>` run whose test
step excludes the `slow|audit|package` labels), `native` (configure +
build, every produced static-analysis sweep and the structural contracts
check via `tools/run_native_checks.sh`, then CTest without
`slow|audit|package`) and `full` (`native` plus the explicit `slow`,
`audit` and `package` label gates). `bash tools/run_tests.sh native asan`
runs the same stages on `build/asan`. `NATIVE_TEST_BUILD_PATH` selects the
absolute build directory for the produced checks; the retired
`BIKE_NATIVE_BUILD_PATH` selector was removed at T63.

The `native`/`full` sweep stage runs the produced checks — also standalone
CMake targets where defined: `check_frontends` (second-frontend
syntax+analyzer), `check_tidy`
(clang-tidy, which also runs the clang static analyzer), `check_cppcheck`,
`check_odr`, `check_scripts` (shellcheck),
plus `headers`, `diagnostic-controls`, `context` and the contracts sweep.
Each failed sweep names its kind; rerun it standalone with
`build/native-release/bin/bike-native-checks --kind <kind> --build build/native-release`.
**A missing required
analysis tool is an error, not a skip.** Tools resolve through the
configure-recorded check context — explicit `CLANG_TIDY`, `CPPCHECK`, `GXX`,
`LLVM_PROFDATA`, `LLVM_COV` overrides first, then the
configured compiler's toolchain siblings, qualified PATH candidates, and —
on macOS only — `xcrun --find` inside the selected Xcode toolchain. The
qualified clang-tidy is the Homebrew LLVM toolchain discovered via
`brew --prefix llvm` (AppleClang lacks several checkers); the GCC frontend
for `check_frontends`/`check_odr` was the Homebrew `g++-16` on the measured
host.

## Sanitizers and coverage

ASan/UBSan lane:

```sh
cmake --workflow --preset asan
bash tools/run_tests.sh native asan
```

The sanitizer set is `address,undefined,local-bounds,float-cast-overflow`
with `-fno-sanitize-recover=all`. Instrumented executables link their own
compiler-matched runtime — the preset's sanitizer link flags replaced the
interpreter preload launcher; an unresolvable runtime fails closed at
configure.

RTSan (`-fsanitize=realtime`) verifies the marked warm core never
allocates or locks. AppleClang rejects the flag — the `rtsan` preset
explicitly selects `/opt/homebrew/opt/llvm/bin/clang++`, a separately
qualified LLVM toolchain (the measured lane used Homebrew LLVM 23.1.2;
LLVM >= 20 is the floor):

```sh
cmake --workflow --preset rtsan
bash tools/run_tests.sh native rtsan
```

Coverage requires an LLVM clang-family compiler and
a `llvm-profdata`/`llvm-cov` pair qualified against it by a real
instrumented control — `LLVM_PROFDATA`/`LLVM_COV` overrides, the configured
compiler's siblings, PATH, then the macOS-only `xcrun --find` fallback:

```sh
cmake --workflow --preset coverage-report
```

The coverage preset builds the instrumented tree, runs the native tests,
merges only fresh profiles and writes `build/coverage/coverage.lcov` plus
`coverage-summary.txt`; mismatched tools fail with the tool's own
diagnostics. See [TESTING.md](../TESTING.md) for sweep internals and dated
measurement snapshots.

## Editor setup

`native/.clangd` points clangd at the default build tree's
`compile_commands.json` — CMake writes one per configured build directory
(`CMAKE_EXPORT_COMPILE_COMMANDS=ON`). Select a different build per editor
session with `--compile-commands-dir=<build dir>` (for example
`build/native-release` or `build/asan`), and let clangd resolve the
recorded compiler's headers with `--query-driver=<the configured compiler
in that build>` — e.g. `/usr/bin/c++` or the Homebrew LLVM `clang++`, never
a fixed absolute path shared across machines.

## Offline rehearsal and packages

Native delivery is the CPack source/binary TGZ pair produced by the
`package` lane (`bash tools/ci_presets.sh release package`); the package
audit and `package_smoke` checks verify the archives contain no
interpreter or binding surface. The `offline` rehearsal mode is the
Linux lane's job: it unpacks the source package in a fresh container with
`--network none`, extracts dependencies with `tools/linux/extract_deps.sh`,
then runs `cmake --workflow --preset release`.

The interpreter-era schema-1 bundles (wheelhouse, `requirements.txt`,
the offline_bundle assembler and the packaging install_full/install_slim
entry points delegating to tools/install_offline.py) were removed at
T63 — a filename never grants capability. See
[install-modes.md](install-modes.md) for the delivery modes.

## Failure diagnosis

Configure diagnostics and the selected build's
`native_check_context.json` are the first evidence for any
environment or artifact failure; the symptom → evidence → bounded
remediation table (SDK selection, missing MuJoCo private symbol, provider
mismatch, missing artifact, unresolved
dyld image, sanitizer and coverage
tool lanes, strict invalid-controller stop, replay identity) lives in
[troubleshooting.md](troubleshooting.md).
## Phase 3 instrumentation and packaging

```sh
cmake --workflow --preset asan
cmake --workflow --preset coverage
ctest --test-dir build/native-release -R '^package_smoke$' --no-tests=error --output-on-failure
```

On Linux, prefix each command with `bash tools/linux/container.sh exec`
(deferred lane — the container guard exits 3 on this tree).
Coverage runs the native tests, merges only fresh profiles and writes
`build/coverage/coverage.lcov` plus `coverage-summary.txt`. ASan and RTSan
executables link their own runtime; no interpreter preload is used by the
root graph. RTSan fails configure if the selected compiler lacks the actual
flag/annotation/runtime contract.

Ordinary presets exclude `slow`, `audit` and `package` labels. Those remain
explicit gates: package smoke is selected above; slow tests and the final
completeness/documentation audits belong to the release checklist. Source
implementation and a recorded command do not establish a passing gate.
