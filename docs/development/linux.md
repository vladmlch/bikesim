# Linux development guide

Active instructions for developing bike-sim on Linux. The sibling
[macOS guide](macos.md) covers the Darwin toolchain; shared failure
diagnosis lives in [troubleshooting.md](troubleshooting.md).

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
x86_64 is optional, not qualified and not advertised. Desktop GUI sessions on X11/XWayland are pending real
acceptance ([viewer-platform-acceptance.md](viewer-platform-acceptance.md)).
The measured Linux pytest run reported `3221 passed, 3 failed`; the three
retained failures
(`tests/reference/test_native_platform_recording.py::test_publication_permission_denied_then_retry`,
`tests/reference/test_native_runtime_commands.py::test_engine_failure_keeps_committed_prefix`,
`tests/reference/test_native_runtime_ownership.py::test_fatal_engine_error_preserves_completed_prefix_and_can_reset`)
have since been root-caused and fixed under Task 3.11 of the C++ + Linux
completion plan — the original logs stay verbatim in
`output/portability/logs/` as history. No retained failures remain in this
list. macOS arm64 is a separately qualified lane, not evidence for Linux.

## Prerequisites

- **CPython 3.13 is required** (`requires-python = ">=3.13,<3.14"`), and
  the qualified Linux interpreter is CPython **3.13** specifically —
  CPython 3.13.16 measured. Ubuntu 24.04's unqualified `python3` is not the
  requirement; provision an exact interpreter with `uv python install 3.13`
  or select an already installed one via `tools/dev.py bootstrap --python`.
- uv for the project environment; Git; a C++ toolchain, CMake 3.27+,
  pkg-config and Ninja as initial candidates (Debian-style:
  `build-essential`/`clang`, `cmake`, `ninja-build`, `pkg-config`,
  plus GLFW/OpenGL development packages for the GUI lane).
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

## Container lane (C++ + Linux completion)

The committed recipe is in `tools/linux/Dockerfile`; the driver is
`tools/linux/container.sh`. It targets Docker linux/arm64 and pins Ubuntu
24.04 by image digest, uv 0.13.1 by archive SHA-256, and managed CPython
3.13.16. Compiler packages come from Ubuntu's signed package repositories;
their actual versions and the built image digest still belong in the
qualification evidence.

The recipe was implemented with source inspection only on 2026-10-21.
The earlier measured lane above does not qualify this new image.

Run from a real Git checkout when executing the phase's validation:

~~~sh
tools/linux/container.sh image
tools/linux/container.sh sync
tools/linux/container.sh venv
# Before configuring, regenerate both platforms' eight-symbol numerics
# records as specified in completion-plan Task 1.10.
tools/linux/container.sh configure native-clang native -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
tools/linux/container.sh configure native-gcc native -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
tools/linux/container.sh build native-clang
tools/linux/container.sh ctest native-clang -j8
~~~

Each checkout has its own container, named `bike-dev-<checkout-name>`.
`BIKE_LINUX_CONTAINER` overrides the name; an existing container belonging
to a different checkout is rejected. `BIKE_LINUX_IMAGE` overrides the image
tag. Sources are copied into `/src`; binding build trees stay in `/build`.
The host `.venv`, `.deps` and generated build directories are excluded even
if accidentally tracked. `/src/.deps` and `/src/build` stay intact on sync.
Archive staging and rsync preserve unchanged mtimes for incremental builds.
An unstaged deletion of a tracked file must be staged before sync.

Available commands: image, up, sync, venv, configure, build, failures,
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
existing container. Use an explicit `down`, then `sync` and `venv` to create
and provision a replacement; `down` removes that container's build trees.

During Phase 1, Linux configures the binding graph only. The root graph
and its C++ checker have their qualification gate in Phase 2. Until that
gate passes, use the Python checker invocations listed in Task 1.19 for
Linux binding sweeps.

### Python-free root graph: Phase 2 source implementation

The 2026-10-21 Phase 2 changes use the same `BikePlatform`,
`BikeCompilerPolicy`, schema-2 dependency-lock selection and check-context
emitter as the binding graph. This session did not configure, build or run
either graph. The historical counts above do not qualify these changes,
and the Repomix input contains no Git objects from which to report a new
`impl/python-free-cpp` commit. See
[`migration/cpp-linux-completion-evidence.md`](../../migration/cpp-linux-completion-evidence.md)
for the source handoff and open checkpoints.

The image command stages only the Dockerfile, acquisition helper and
dependency lock. Its explicit acquisition stage verifies the selected
Linux archives and extracted files before placing them in `/opt/bike-deps`.
The final image supplies `/src/.deps` independently of the host checkout.
Configuration itself performs no downloads and never substitutes a system
OpenBLAS. The selected NumPy wheel's OpenBLAS and `libgfortran` companion
are locked and installed together.

Before configuring, finish Phase 1's real per-platform numerics probes.
The supplied `native/numerics.lock.json` still contains the old Darwin
four-operation record, whereas the validator now requires eight
operations and a measured Linux entry. Source-only changes deliberately
do not replace that generated evidence with guessed values.

When those prerequisites are ready and execution is authorized:

~~~sh
tools/linux/container.sh image
tools/linux/container.sh sync
tools/linux/container.sh venv
tools/linux/container.sh configure root-clang root -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=OFF
tools/linux/container.sh build root-clang
tools/linux/container.sh ctest root-clang -j8
tools/linux/container.sh exec /build/root-clang/bin/bike-native-checks --kind context --build /build/root-clang
tools/linux/container.sh exec tools/run_native_checks.sh /build/root-clang
tools/linux/container.sh exec tools/run_native_checks.sh /build/native-clang
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
tools/linux/container.sh configure root-gui root -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 -DBIKE_BUILD_GUI=ON
tools/linux/container.sh build root-gui
tools/linux/container.sh exec tools/linux/gui_smoke.sh /build/root-gui
~~~

The smoke uses software GL and Xvfb, requests one simulated second, and
limits wall time to 120 seconds plus ten seconds for forced cleanup. It
propagates diagnostic exits and timeouts. It never closes a row in
[`viewer-platform-acceptance.md`](viewer-platform-acceptance.md).
`bike-ride --close-on-finish` requires `--duration` and an interactive
physical or research run; without the flag the terminal window still
stays open as before.

The final Linux gate is `tools/linux/container.sh gate native`. It runs
the existing `tools/run_tests.sh native` profile, with the binding tree at
`/build/native-clang`, the produced checker tree at
`/src/build/native-release`, and the environment at `/opt/bike-venv`.
Fresh trees use Clang 18. An existing cache for a different compiler is
rejected without deleting it; move that build directory aside and
configure the correct lane. `sync` preserves the root gate's build tree.
Checkpoint 2-A, L3, measured failure counts and the push remain pending.

## Transition oracle environment

One editable environment per source tree. On Linux, always select a
**Linux-native** venv before the first uv command and keep it for every
step — a copied or cross-OS `.venv` is not portable (observed failure: a
container `uv run` against a bind-mounted checkout clobbered the host's
`.venv`).

```sh
export UV_PROJECT_ENVIRONMENT=/opt/bike-venv   # absolute Linux-native venv
uv sync --frozen --group native
```

Without the override, wrappers select `<checkout>/.venv`. A relative
`UV_PROJECT_ENVIRONMENT` is rejected. `tools/dev.py bootstrap` is the
explicit equivalent for `uv sync --locked --group native --group dev` and
accepts `--python` to select the interpreter; it never relocks or upgrades
dependencies. The native group pins `mujoco==3.12.0` and `numpy==2.5.2`;
the loaded MuJoCo image must expose the project's private ABI symbols and
the NumPy wheel supplies the qualified `NUMPY_OPENBLAS` provider — an
arbitrary distro OpenBLAS is not a substitute
([numerical-profiles.md](numerical-profiles.md),
[dependency-matrix.md](dependency-matrix.md)).

## Configure and build with root presets

The standalone graph uses `.deps` and the measured numerics record. Its
configure/build/test route invokes no interpreter. Run every Linux
qualification command inside the checkout's aarch64 container:

```sh
bash tools/linux/container.sh sync
bash tools/linux/container.sh exec cmake --workflow --preset release
bash tools/linux/container.sh exec cmake --workflow --preset debug
bash tools/linux/container.sh exec bash tools/run_native_checks.sh build/native-release
```

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
display and initialize no GLFW window. Strict headless physical runs
terminate at the documented `energy.constraint_work` baseline violation —
a retained diagnostic prefix, not a crash ([RIDE.md](../RIDE.md)).

Interactive viewers require a real display (X11, or XWayland inside a
Wayland session) plus the system GLFW/OpenGL stack — those are OS packages,
not uv dependencies. `ensure_macos_mjpython()` returns immediately on
Linux; there is no `mjpython` re-exec and no Cocoa main-thread rule. Linux
GUI acceptance remains pending on the desktop matrix linked above — do not
report a container run as viewer qualification.

## Tests and analysis

```sh
bash tools/run_tests.sh quick
NATIVE_TEST_BUILD_PATH="$(pwd)/native/build/release" bash tools/run_tests.sh native
```

Profiles: `quick` (Python tests minus `slow` minus direct native files;
keeps `test_native_check_tools.py`), `native` (configure/build the selected
directory, all sweeps and controls, CTest, then native+golden pytest —
~3,200 items) and `full` (adds slow physical/realtime episodes).
`NATIVE_TEST_BUILD_PATH` selects the absolute build directory; set
`BIKE_NATIVE_BUILD_PATH` to the same directory for the full suite — the
conftest fails when the two selectors disagree, and child `uv run`
processes inherit the environment. The legacy `NATIVE_TEST_BUILD_DIR`
values `''`, `asan`, `rtsan`, `coverage` remain accepted.

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
configured compiler's toolchain siblings and qualified PATH candidates
(Linux never invokes `xcrun`). Clang 18's packaged `clang-tidy`/`clang++`
siblings were the qualified set; an override that names a different
toolchain family fails rather than silently mixing.

## Sanitizers and coverage

ASan/UBSan lane (Clang and GCC both qualified on aarch64):

```sh
uv run --frozen --group native --no-sync cmake -S native -B native/build/asan -DNATIVE_SANITIZE=ON -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
uv run --frozen --group native cmake --build native/build/asan
NATIVE_TEST_BUILD_DIR=asan bash tools/run_tests.sh native
```

The sanitizer set is `address,undefined,local-bounds,float-cast-overflow`
with `-fno-sanitize-recover=all` on Clang lanes and `address,undefined` on
GCC. Every instrumented consumer runs through
`tools/native_runtime.py run --build <build> -- <argv>`, which prepends the
recorded compiler-matched runtime to `LD_PRELOAD` — never a caller-side
broad `LD_LIBRARY_PATH`. An unresolvable runtime fails closed; the import
then verifies the runtime is actually mapped via `/proc/self/maps`.

**RTSan (`-fsanitize=realtime`) requires a separately qualified LLVM
toolchain.** Clang 18 lacks the `[[clang::nonblocking]]` semantics and
runtime support the marked warm core needs — configure fails closed with a
capability diagnostic rather than silently building an uninstrumented lane.
Provision a qualifying LLVM release (the qualified macOS lane used upstream
LLVM 23.1.2; LLVM >= 20 is the floor) and point `configure --compiler` at
its `clang++`. Do not mark the RTSan lane skipped-as-passed when the
distribution toolchain lacks it — the lane is pending until a qualified
toolchain runs it.

Coverage (`NATIVE_COVERAGE=ON`) requires an LLVM clang-family compiler and
a `llvm-profdata`/`llvm-cov` pair qualified against it by a real
instrumented control — resolution order is `LLVM_PROFDATA`/`LLVM_COV`
overrides, the configured compiler's siblings, then PATH. The
`xcrun --find` fallback is Darwin-only and never runs on Linux:

```sh
tools/native_coverage.sh native/build/coverage
```

Artifacts land in `<build>/native_coverage_runs/run.*` with a
`coverage_provenance.json`; mismatched tools fail with the tool's own
diagnostics. See [TESTING.md](../TESTING.md) for sweep internals — its
sanitizer/coverage examples retain their dated macOS paths.

## Editor setup

`native/.clangd` points clangd at `native/build`'s
`compile_commands.json` — CMake writes one per configured build directory
(`CMAKE_EXPORT_COMPILE_COMMANDS=ON`). Select a different build per editor
session with `--compile-commands-dir=<build dir>` (for example
`native/build-linux-clang`), and let clangd resolve the recorded compiler's
headers with `--query-driver=<the configured compiler in that build>` —
e.g. `/usr/bin/clang++-18` or `/usr/bin/g++-13` here, never a fixed
absolute path shared across machines.

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
external stack fails with the full bundle as remediation.

Legacy manifest-less bundles remain Python-only and are labeled as such —
a filename never grants native capability. See
[install-modes.md](install-modes.md) for the three installation modes.

## Failure diagnosis

`doctor --build <dir> --numerics` is the first evidence command for any
environment or artifact failure; the symptom → evidence → bounded
remediation table (Apple SDK option on Linux, missing MuJoCo private
symbol, provider/ABI mismatch, missing artifact, wrong SOABI, stale source
stamp, unresolved DSO, missing display, unsupported sanitizer, coverage
tool mismatch, strict invalid-controller stop, cross-environment replay
rejection, slim external mismatch) lives in
[troubleshooting.md](troubleshooting.md).
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
