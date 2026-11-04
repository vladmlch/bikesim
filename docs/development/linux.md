# Linux development guide

Active instructions for developing bike-sim on Linux. The sibling
[macOS guide](macos.md) covers the Darwin toolchain; shared failure
diagnosis lives in [troubleshooting.md](troubleshooting.md).

**Qualification status:** post-T63 Linux qualification is **deferred** —
no lane of the Python-free root graph has executed on this tree, so every
"qualified" claim below is pre-removal historical evidence, not a current
qualification.
Commands in `~~~` fences are the recorded/deferred lane contract, not a
run book for this checkout (the container driver currently exits 3 under
the plan-change guard; Docker is unavailable on the Darwin-only host).

Historical qualified lane (measured 2026-10-17/18, before the completion changes below):

| Component | Qualified value |
|---|---|
| Host | Ubuntu 24.04.5 LTS container, **aarch64** (`bike-linux`, Docker linux/arm64) |
| Interpreter | CPython **3.13.16**, uv-managed, venv `/opt/bike-venv` |
| Compilers | Clang **18.1.3** (`/usr/bin/clang++-18`, qualified lane) and GCC **13.3.0** (`g++-13`, both lanes build and pass CTest) |
| Build tools | CMake 3.28.3, uv 0.13.1 |
| Selected build | `native/build-linux-clang` (artifact `bike_native.cpython-313-aarch64-linux-gnu.so`); `native/build-linux-gcc` for the GCC lane |
| Numerics | `NUMPY_OPENBLAS` — the ILP64 OpenBLAS bundled inside the pinned NumPy 2.5.2 wheel (BLAS 64-bit / LAPACK 64-bit) |
| Gate evidence | All static-analysis sweeps and headers/diagnostic/context controls green on both compiler lanes; CTest 10/10; native+golden pytest ~3,221 items (see limits below) |

Recorded limits, not qualifications: **Ubuntu 24.04 x86_64** has no real
host evidence (wheel availability was cross-probed only). Decision D10 in
the C++ + Linux completion plan makes Linux **aarch64** the required lane;
x86_64 is optional, not qualified and not advertised. Desktop GUI sessions on X11/XWayland are pending real acceptance.
The measured Linux pytest run reported `3221 passed, 3 failed`; the three
retained failures
(`native_platform_recording/publication_permission_denied_then_retry`,
`native_runtime_commands/engine_failure_keeps_committed_prefix`,
`native_runtime_ownership/fatal_engine_error_preserves_completed_prefix_and_can_reset`)
have since been root-caused and fixed under Task 3.11 of the C++ + Linux
completion plan — the original logs stay verbatim in
`output/portability/logs/` as history. No retained failures remain in this
list. macOS arm64 is a separately qualified lane, not evidence for Linux.

## Prerequisites

- **No interpreter is required or used.** The transition oracle's
  CPython 3.13 stack (the `bike_native` extension, the uv `--group native`
  dependency group, `UV_PROJECT_ENVIRONMENT` selection and the
  tools/dev.py bootstrap) was removed at T63; the qualification table
  above records that stack as history only.
- Git; a C++ toolchain, CMake 3.27+, pkg-config and Ninja as initial
  candidates (Debian-style: `build-essential`/`clang`, `cmake`,
  `ninja-build`, `pkg-config`, plus GLFW/OpenGL development packages for
  the GUI lane), and the analysis tools used by the sweep lanes
  (`clang-tidy`, `cppcheck`, `shellcheck`, LLVM
  `llvm-profdata`/`llvm-cov` for coverage).
- **The distribution compiler is a candidate, not a promise.** Configure
  runs real capability probes — strict ISO C++23, standard library
  hardening, sanitizer flags, MuJoCo 3.12.0 private ABI, and the NumPy
  BLAS/LAPACK provider ABI. An unsupported probe fails configure; nothing
  degrades to a "close enough" toolchain. Clang 18.1.3 and GCC 13.3.0 are
  the measured compilers.
- Apple-only machinery does not exist here: `xcrun`/`xcode-select` are never
  invoked, and passing **`CMAKE_OSX_SYSROOT` on Linux is invalid** —
  configure rejects it with `CMAKE_OSX_SYSROOT is a macOS option; use a
  Linux-native toolchain` before dependency discovery. Likewise no
  `DYLD_*`, Accelerate, or `mjpython` re-exec applies.

### Container lane (C++ + Linux completion)

The committed recipe is in `tools/linux/Dockerfile`; the driver is
`tools/linux/container.sh`. It targets Docker linux/arm64 and pins Ubuntu
24.04 by image digest, uv 0.13.1 by archive SHA-256, and managed CPython
3.13.16. Compiler packages come from Ubuntu's signed package repositories;
their actual versions and the built image digest still belong in the
qualification evidence.

The recipe was implemented with source inspection only on 2026-10-21.
The earlier measured lane above does not qualify this new image.

The lane currently carries a plan-change guard: Docker was stopped for the
Darwin-arm64-only migration wave, so `container.sh` exits 3 until the
Linux wave resumes and the guard block is removed. The commands below are
the lane contract for that wave, not a run book for this checkout.

Run from a real Git checkout when executing the lane's validation:

~~~sh
tools/linux/container.sh image
tools/linux/container.sh sync
tools/linux/container.sh configure root-clang -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=OFF
tools/linux/container.sh configure root-gcc -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/g++-13 -DBIKE_BUILD_GUI=OFF
tools/linux/container.sh build root-clang
tools/linux/container.sh ctest root-clang -j8
~~~

Each checkout has its own container, named `bike-dev-<checkout-name>`.
`BIKE_LINUX_CONTAINER` overrides the name; an existing container belonging
to a different checkout is rejected. `BIKE_LINUX_IMAGE` overrides the image
tag. Sources are copied into `/src`; root-graph build trees stay in
`/build`.
The host `.venv`, `.deps` and generated build directories are excluded even
if accidentally tracked. `/src/.deps` and `/src/build` stay intact on sync.
Archive staging and rsync preserve unchanged mtimes for incremental builds.
An unstaged deletion of a tracked file must be staged before sync.

Available commands: image, up, sync, configure, build, failures,
ctest, exec, pull, gate, down. The driver retains the failing Ninja exit code
through tee. The failures command prints the failed-command count and
returns an error for an absent log; ctest rejects an empty test selection.
`pull PATH...` accepts files or whole directories, including
`pull tests/fixtures/golden/linux-aarch64`. Each path lands at that exact
checkout-relative location on initial and repeated pulls. Directory pulls
merge recursively: matching files are overwritten, unrelated destination
entries remain, and hidden files, spaces and executable modes are preserved.
Source trees must contain only regular files/directories outside the driver's
environment/build exclusions. Symlink parents, copied symlink/special entries,
destination symlink counterparts and file/directory conflicts are rejected.
All operands are staged and validated before publication; failed transfers
leave existing content intact, and publication errors restore backups. Use
nonoverlapping paths (exact duplicates are accepted). Pulls reject filesystem-
equivalent operands and ancestor aliases, including case/Unicode-normalization
aliases on the host volume even for absent destinations. Environment/build
exclusions also reject all ASCII case variants on every host; other non-ASCII
names remain supported. Keep source/destination trees unchanged during the
pull, and allow disk space for the staged merge
and original backups. This is not a crash-atomic snapshot; if rollback itself
fails, the driver reports and retains its backup directory for recovery.
No automatic cleanup removes old containers.
After changing the Dockerfile, rebuilding the image does not replace an
existing container. Use an explicit `down`, then `sync` to create
and provision a replacement; `down` removes that container's build trees.

The binding graph is gone (T63): `configure NAME` configures the root
graph at `/src` directly, and the produced checkers are the C++
`bike-native-checks` binary plus `tools/run_native_checks.sh` — there is
no interpreter-side checker.

#### Root graph lane

The root graph uses the same `BikePlatform`,
`BikeCompilerPolicy`, schema-2 dependency-lock selection and check-context
emitter on every lane.

The image command stages only the Dockerfile, acquisition helper and
dependency lock. Its explicit acquisition stage verifies the selected
Linux archives and extracted files before placing them in `/opt/bike-deps`.
The final image supplies `/src/.deps` independently of the host checkout.
Configuration itself performs no downloads and never substitutes a system
OpenBLAS. The locked provider archive and its `libgfortran` companion
are verified and installed together.

The numerics record `native/numerics.lock.json` is consumed record-only:
configure binds the measured per-platform entries and never probes a live
provider.

When those prerequisites are ready and execution is authorized:

~~~sh
tools/linux/container.sh image
tools/linux/container.sh sync
tools/linux/container.sh configure root-clang -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=OFF
tools/linux/container.sh build root-clang
tools/linux/container.sh ctest root-clang -j8
tools/linux/container.sh exec /build/root-clang/bin/bike-native-checks --kind context --build /build/root-clang
tools/linux/container.sh exec tools/run_native_checks.sh /build/root-clang
~~~

`install_smoke` builds the full root graph, installs it, moves the prefix,
inspects the binaries and libraries, and runs all 11 installed commands
with `--help` and scrubbed loader environment variables. It checks
`$ORIGIN/../lib` on Linux and `@loader_path/../lib` on macOS. Its checks do
not qualify simulation behavior or desktop rendering. Focused T03 builds
retain optional executable installs.

For the GUI lane, the complete locked GLFW tree builds with X11 enabled
and Wayland disabled:

~~~sh
tools/linux/container.sh configure root-gui -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=ON
tools/linux/container.sh build root-gui
tools/linux/container.sh exec tools/linux/gui_smoke.sh /build/root-gui
~~~

The smoke uses software GL and Xvfb, requests one simulated second, and
limits wall time to 120 seconds plus ten seconds for forced cleanup. It
propagates diagnostic exits and timeouts. 
`bike-ride --close-on-finish` requires `--duration` and an interactive
physical or research run; without the flag the terminal window still
stays open as before.

The final Linux gate is `tools/linux/container.sh gate native`. It runs
the existing `tools/run_tests.sh native` profile against the produced
checker tree at `/src/build/native-release`.
Fresh trees use Clang 18. An existing cache for a different compiler is
rejected without deleting it; move that build directory aside and
configure the correct lane. `sync` preserves the root gate's build tree.

## Dependency acquisition

The standalone graph consumes the pinned lock at
`native/dependencies.lock.json`. Populate `.deps` with the checked-in
acquisition helper, which verifies every archive and member hash before
placing files:

```sh
bash tools/linux/acquire_deps.sh native/dependencies.lock.json .deps linux-aarch64
```

For offline runs, `tools/linux/extract_deps.sh` consumes already-downloaded
packages with the same verification and no network fallback. Configure
points at the result with `-DBIKE_DEPENDENCY_ROOT=<checkout>/.deps` (the
root presets already default there). The loaded MuJoCo image must expose
the project's private ABI symbols, and the locked provider archive
supplies the qualified `NUMPY_OPENBLAS` numerics backend — an arbitrary
distro OpenBLAS is not a substitute
([numerical-profiles.md](../agents/native-numerical-profiles.md). The transition oracle's
`uv sync --frozen --group native` environment and `UV_PROJECT_ENVIRONMENT`
override were removed at T63; there is nothing to sync.

## Configure and build with root presets

The standalone graph uses `.deps` and the measured numerics record. Its
configure/build/test route invokes no interpreter. Every Linux
qualification command runs inside the checkout's aarch64 container; that
lane is deferred on this tree (container guard exits 3), so the recorded
contract is:

~~~sh
bash tools/linux/container.sh sync
bash tools/linux/container.sh exec cmake --workflow --preset release
bash tools/linux/container.sh exec cmake --workflow --preset debug
bash tools/linux/container.sh exec bash tools/run_native_checks.sh build/native-release
~~~

`release` selects `/usr/bin/clang++-18` and keeps
`build/native-release`; `debug` selects `/usr/bin/g++` and uses
`build/debug`. GUI, ASan, RTSan and coverage use their own preset-named build
directories. An explicit `-DCMAKE_CXX_COMPILER=...` is a toolchain override,
which must be independently qualified; existing caches are not migrated
between compilers.

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
`bike-playground`, `bike-recording` — plus the lane executables
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
display and initialize no GLFW window. Strict headless physical runs
stop at the first model violation; the 1 s ride above has none and exits 0 on darwin-arm64 (2026-11-04; linux-aarch64 not re-run since the start-up
`energy.constraint_work` fix) ([RIDE.md](../RIDE.md)).

Interactive viewers require a real display (X11, or XWayland inside a
Wayland session) plus the system GLFW/OpenGL stack — those are OS packages,
not bundle dependencies. The `mjpython` re-exec never applied on Linux and
is gone with the interpreter; there is no Cocoa main-thread rule. Linux
GUI acceptance remains pending on the desktop matrix linked above — do not
report a container run as viewer qualification.

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
runs the same stages on `build/asan`. Inside the container lane
`NATIVE_TEST_BUILD_PATH` points the produced checks at the selected build
directory; on the host the preset selects it. The retired
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
configured compiler's toolchain siblings and qualified PATH candidates
(Linux never invokes `xcrun`). Clang 18's packaged `clang-tidy`/`clang++`
siblings were the qualified set; an override that names a different
toolchain family fails rather than silently mixing.

## Sanitizers and coverage

ASan/UBSan lane (Clang and GCC were both qualified on aarch64 in the
pre-removal lane; the post-T63 ASan run is deferred):

```sh
cmake --workflow --preset asan
bash tools/run_tests.sh native asan
```

The sanitizer set is `address,undefined,local-bounds,float-cast-overflow`
with `-fno-sanitize-recover=all` on Clang lanes and `address,undefined` on
GCC. Instrumented executables link their own compiler-matched runtime —
the preset's sanitizer link flags replaced the interpreter preload
launcher; an unresolvable runtime fails closed at configure.

**RTSan (`-fsanitize=realtime`) requires a separately qualified LLVM
toolchain.** Clang 18 lacks the `[[clang::nonblocking]]` semantics and
runtime support the marked warm core needs — configure fails closed with a
capability diagnostic rather than silently building an uninstrumented lane.
Provision a qualifying LLVM release (the qualified macOS lane used upstream
LLVM 23.1.2; LLVM >= 20 is the floor) and point `configure --compiler` at
its `clang++`. Do not mark the RTSan lane skipped-as-passed when the
distribution toolchain lacks it — the lane is pending until a qualified
toolchain runs it.

Coverage requires an LLVM clang-family compiler and
a `llvm-profdata`/`llvm-cov` pair qualified against it by a real
instrumented control — resolution order is `LLVM_PROFDATA`/`LLVM_COV`
overrides, the configured compiler's siblings, then PATH. The
`xcrun --find` fallback is Darwin-only and never runs on Linux:

```sh
cmake --workflow --preset coverage-report
```

The coverage preset builds the instrumented tree, runs the native tests,
merges only fresh profiles and writes `build/coverage/coverage.lcov` plus
`coverage-summary.txt`; mismatched tools fail with the tool's own
diagnostics. See [TESTING.md](../TESTING.md) for sweep internals — its
sanitizer/coverage examples retain their dated macOS paths.

## Editor setup

`native/.clangd` points clangd at the default build tree's
`compile_commands.json` — CMake writes one per configured build directory
(`CMAKE_EXPORT_COMPILE_COMMANDS=ON`). Select a different build per editor
session with `--compile-commands-dir=<build dir>` (for example
`build/native-release` or a container `/build/<name>` tree), and let
clangd resolve the recorded compiler's
headers with `--query-driver=<the configured compiler in that build>` —
e.g. `/usr/bin/clang++-18` or `/usr/bin/g++-13` here, never a fixed
absolute path shared across machines.

## Offline rehearsal and packages

Native delivery is the CPack source/binary TGZ pair produced by the
`package` lane (`bash tools/ci_presets.sh release package`). The
`offline` mode rehearses the no-network rebuild: it unpacks the source
package in a fresh container with `--network none`, extracts dependencies
with `tools/linux/extract_deps.sh native/dependencies.lock.json`, then
runs `cmake --workflow --preset release` — validated local payloads only,
no warm-cache dependency.

The interpreter-era schema-1 bundles (wheelhouse, `requirements.txt`,
the offline_bundle assembler and the packaging install_full/install_slim
entry points delegating to tools/install_offline.py) were removed at
T63 — a filename never grants capability. See
[install-modes.md](install-modes.md) for the delivery modes.

## Failure diagnosis

Configure diagnostics and the selected build's
`native_check_context.json` are the first evidence for any
environment or artifact failure; the symptom → evidence → bounded
remediation table (Apple SDK option on Linux, missing MuJoCo private
symbol, provider/ABI mismatch, missing artifact, unresolved DSO,
missing display, unsupported sanitizer, coverage
tool mismatch, strict invalid-controller stop, cross-environment replay
rejection) lives in
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
