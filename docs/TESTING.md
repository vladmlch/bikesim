# Running tests

> **Current verification contract (2026-10-20, [ADR-0003](adr/0003-native-authoritative-python-removal.md)):**
> a test file's live Python/native comparisons remain bitwise gates until its
> porting task captures the consumed golden values on macOS arm64 and Linux
> aarch64 and lands its C++ test port. Then the C++ test against those per-platform
> goldens becomes the gate. ADR-0002's historical immediate-removal statements
> are superseded by this transition. Native unit/contract tests, in-build
> determinism, physical invariants and reviewed self-goldens remain required.
> **Python was removed at T63**: the `uv run` invocations, `tools/dev.py`,
> interpreter sweeps and binding-graph build directories quoted below are
> historical evidence, not runnable commands; the produced
> `build/<preset>/bin/bike-*` executables and `tools/run_tests.sh`
> profiles are the live surface. Live commands sit in ` ``` ` fences;
> recorded/deferred commands sit in `~~~` fences. The dated execution
> records below are unchanged.

## Track C verification status (2026-10-10)

The Track C source delivery has been applied and executed in this checkout. The
`native` profile is green; the `full` profile fails only on the recorded
platform-baseline set. Real macOS windows were exercised (physical ride 1x/8x,
research viewer, checked replay) and backend-selectable measurements were run —
see [the execution audit](superpowers/audits/2026-10-08-native-visual-runtime.md)
and [docs/RIDE.md](RIDE.md) for numbers and GUI evidence.

Track C regressions cover `test_playback_clock.py`,
`test_physical_view.py`, `test_ride_backend_cli.py`, `test_research_frontend.py`,
`test_frontend_windows.py`, `test_replay_session.py` and
`test_realtime_measurement.py`.

Selected-artifact tests are `test_native_viewer_snapshot.py`,
`test_native_ride_frontend.py`, `test_native_research_frontend.py` and
`test_native_replay_session.py`. The existing native recording test now expects
checked native reconstruction instead of the old Track-C-not-implemented error.
The fixture never builds or skips a missing extension. Keep
`NATIVE_TEST_BUILD_PATH` and `BIKE_NATIVE_BUILD_PATH` set to the same absolute
directory, containing a newly built extension with the current source stamp.

Run the targeted regressions and the complete native/full profiles after native
changes. Separately exercise macOS windows and measure one warm-up plus three
runs per backend with `tools/measure_realtime.py`. A fake window is not a GUI
acceptance check, and a measurement report is not a numerical parity result.

## Platform support

The dated status block above is the 2026-10-10 macOS record. Current
measured development lanes and per-platform setup are documented in
[development/linux.md](development/linux.md),
[development/macos.md](development/macos.md) and
[development/troubleshooting.md](development/troubleshooting.md): macOS
arm64 is the qualified host; Linux aarch64 in the Ubuntu
24.04 container is the deferred lane (its container guard exits 3 on this
tree). The pre-removal lanes above ran CPython-backed builds; post-T63
there is no interpreter environment to select and
`CMAKE_OSX_SYSROOT` is rejected as an Apple-only option on Linux. The sweep
and profile semantics below are platform-independent; tool *resolution*
differs per lane and is described where it matters.

## Upstream verification commands

Run these commands from the repository root:

~~~bash
bash tools/run_tests.sh          # defaults to quick
bash tools/run_tests.sh quick    # one cmake --workflow run; tests exclude slow|package
bash tools/run_tests.sh native   # build + all static-analysis sweeps + CTest (minus slow|package)
bash tools/run_tests.sh full     # native plus the explicit slow and package label gates
```

Use `quick` outside the native scope and as the final fast regression check.
Use `native` for C++ implementation, build and verification-tool work — it
configures/builds the selected preset, runs every produced static-analysis
sweep and the structural contracts check via `tools/run_native_checks.sh`,
then runs CTest without `slow|package`.
Use `full` for complete physical episodes or realtime acceptance — the
`native` stage plus explicit `slow` and `package` label gates
(`--no-tests=error`). The optional second argument selects a configure
preset (`native asan` runs the same stages on `build/asan`); it exports
`NATIVE_TEST_BUILD_PATH` to that build directory. The launcher resolves its
repository root, so it also works when invoked from another directory.

An unknown profile returns exit status 2. For a targeted test node, invoke
CTest against the selected build directly:

~~~bash
ctest --test-dir build/native-release -R 'native_suspension' --output-on-failure
ctest --test-dir build/native-release -N                # list registered tests
```

## Native prerequisite

`native` and `full` configure and build the selected preset before checking.
For a standalone build, configure a preset and build its tree:

~~~bash
cmake --preset release
cmake --build build/native-release -j4
```

The pre-T63 loader contract (a `bike_native` extension selected through
`NATIVE_TEST_BUILD_PATH`/`BIKE_NATIVE_BUILD_PATH`, the Python oracle
comparisons that phrase described) was removed at T63 — the produced build
tree's executables and CTest registrations are the test surface now.

`native_contract_tests` remains registered with CTest; its `require` helper
throws even when `NDEBUG` is defined. To check both Debug and Release,
configure each preset and run the same focused control against each:

~~~bash
ctest --test-dir build/debug -R native_contract_tests --output-on-failure
ctest --test-dir build/native-release -R native_contract_tests --output-on-failure
```

The contract source is included in the CMake analysis manifest.

## Standalone root build

`cmake -S . -B build/native-release` (or `cmake --preset release`, same
thing) is the Python-free graph — the only graph since T63 removed the
legacy `native/` subproject configure. Its configure reads the committed
numerics record (`BIKE_NUMERICS_SOURCE=RECORD_ONLY`).
The T53-era `test_root_configure.py` record checked two things on that
configure: no interpreter reachable by name on `PATH` is spawned, and none
is recorded in the CMake cache — both remain true and are enforced by the
produced source manifest and context checks. An absolute-path spawn is
still not caught by configure. A cross-compiling
toolchain (`-DCMAKE_SYSTEM_NAME=Windows`) is refused by the root's own include
chain. `BIKE_BUILD_PYTHON_BINDINGS=ON` is rejected outright by the root:
the binding graph and the `cmake -S native` route no longer exist.

## Static analysis and sanitizer builds

The `native` and `full` profiles share the full verification preflight:
configure, build, every CMake sweep, the standalone header/diagnostic/context
controls, CTest, an exact selected-extension import, and the `contracts`
kind before pytest. Each configured sweep is also a standalone CMake target.
The profiles first configure and build the produced check orchestrator into
`build/native-release` (`bike-native-checks`) when its
cache is absent, then always rebuild it incrementally so checks never run on
a stale binary. CMake writes
`<selected-build>/native_sources.json` from the first-party sources attached to
configured targets and `<selected-build>/native_check_context.json` with the
compiler, SDK, Python, dependency paths, target compile contexts, and tool overrides.
The produced `bike-native-checks` orchestrator checks those manifests against
`compile_commands.json` before starting any analyzer. It rejects an empty,
missing, duplicate, unexpected, or malformed first-party selection and missing
include/library dependencies. The output reports both unique source and
translation-unit counts, so a source compiled for distinct targets remains
visible as multiple contexts.

Every run writes `native/build/native_check_summaries/<kind>.json`; complete
stdout and stderr are retained under `native/build/native_check_logs/<kind>/`.
The summary separates tool health from findings. Stable diagnostics block even
when the analyzer exits zero. Alpha findings are marked `report-only`, while an
alpha tool crash or other nonzero exit fails tool health. Tool executables can
be overridden with `CLANG_TIDY`, `ANALYZER_CLANG`, `CPPCHECK`, and `GXX`; each
override is resolved and checked before the sweep runs.

~~~bash
cmake --build build/native-release --target check_frontends   # gcc -fsyntax-only + -fanalyzer (second frontend)
cmake --build build/native-release --target check_tidy        # clang-tidy, checks in native/.clang-tidy
cmake --build build/native-release --target check_analyzer    # clang --analyze, stable+optin block; alpha report-only
cmake --build build/native-release --target check_cppcheck    # cppcheck third engine
cmake --build build/native-release --target check_odr         # g++ -flto -Wodr merge — cross-TU type conflicts
cmake --build build/native-release --target check_scripts     # shellcheck over tools/*.sh
```

The produced orchestrator can also be invoked directly — the check
targets and the preflight resolve it via `$BIKE_NATIVE_CHECKS`, the
selected build's `bin/`, or the produced standalone build
`build/native-release/bin/bike-native-checks`:

~~~bash
build/native-release/bin/bike-native-checks --kind tidy --build build/native-release
```

The Python `tools/native_checks.py` controller was deleted at T63 — the
produced `bike-native-checks` binary is the only orchestrator. The same
configured source manifest is
used by each standalone target and by the native profile; wrappers do not
glob `native/src` or hide process status.

V2 adds three focused controls:

~~~bash
build/native-release/bin/bike-native-checks --kind headers --build build/native-release
build/native-release/bin/bike-native-checks --kind diagnostic-controls --build build/native-release
build/native-release/bin/bike-native-checks --kind context --build build/native-release
```

The `headers` control compiles each first-party header by itself with its
configured target flags. `diagnostic-controls` verifies that the selected
compiler still reports discarded `nodiscard` values, bitwise logical tests,
extra semicolons, signed bounds, and first-party use after move. It also
verifies that libc++ hardening configuration rejects a missing mode macro.
`context` tests explicit, missing, and default SDK selection, then records
container sizes, active bounds assertions, and timings for FAST and EXTENSIVE
libc++ modes in Debug, Release, and the empty build type.

V4 adds the `contracts` kind to native/full preflight (run by the
produced orchestrator; the
`check_native_contracts` CMake target remains for direct reruns). It
checks the CMake
source manifest against the compilation database, strict ISO C++23 and the
configured hardening mode, the `.clang-tidy` Move/header policy, selected-loader
use by direct native tests, and the contract source's manifest and CTest
registration. It asks the contract executable to list and run each registered
case. These are structural and focused behavioral checks; they do not claim to
prove exception safety for every native operation. The engine-call wrapper
rules now run actively (pinned wrappers manifested, fatal-capable calls
confined to `engine_call.cpp`).

The contracts target can be rerun directly for a regular build:

~~~bash
cmake --build build/native-release --target check_native_contracts
```

For ASan or RTSan builds, use `tools/run_tests.sh native`; it resolves and
passes the selected sanitizer runtime to the contract executable's child
processes.

First-party targets use strict ISO C++23. The `.clang-tidy` header filter is
checkout independent and covers `native/src` and `native/tests`.
First-party Move findings remain blocking. The
only current third-party exception is in
`native/analysis_suppressions.json`: it matches the observed Move diagnostic
from nanobind's fixed size `std::array` caster by checker, header path, message,
and analyzer version. The controller prints each suppression count.

`CMAKE_OSX_SYSROOT` and `CMAKE_CXX_COMPILER` cache values supplied by the
caller are retained and checked during configuration. If no SDK is supplied,
CMake probes the `xcrun` default and installed SDK candidates by compiling and
linking against Accelerate. An explicit missing or incompatible SDK fails
configuration. libc++ hardening defaults to EXTENSIVE in every build type;
FAST is an explicit comparison option:

~~~bash
cmake -S . -B build/native-release-fast -DBIKE_LIBCPP_HARDENING=FAST
cmake --build build/native-release-fast -j4
```

The timing control uses five samples of three million indexed reads per build
type and hardening mode. It records representative container sizes, whether
bounds assertions fired, each timing sample, and the median for Debug, Release,
and the empty build type. Read the current compiler, SDK, run ID, and measured
values in `native/build/native_check_summaries/context.json`; these local
measurements are evidence for the selected toolchain, not a performance floor.

`check_analyzer` and `check_tidy` need a full LLVM toolchain, not AppleClang:
on the qualified macOS arm64 host they resolve to the keg-only brew LLVM
(`/opt/homebrew/opt/llvm/bin`, discovered per machine via `brew --prefix
llvm`), and on the qualified Linux aarch64 lane to the distro clang-18
tools. Missing tools are an error, not a skip. The
analyzer's alpha pass prints findings without failing; they are unstable by
upstream contract, so review them when they appear instead of gating on them.
`check_odr` compiles every TU with `-flto` and merges with `-Wodr` — the only
phase that can see a type defined differently across translation units.

The profile's optional preset argument selects the configured build
directory (`native asan` → `build/asan`); `tools/run_tests.sh` exports
`NATIVE_TEST_BUILD_PATH` to that directory for the produced checks. The old
`NATIVE_TEST_BUILD_DIR` selector convention was replaced by the preset
argument.

Sanitizer build:

~~~bash
cmake --workflow --preset asan
bash tools/run_tests.sh native asan
```

The sanitizer set is `address,undefined,local-bounds,float-cast-overflow`
with `-fno-sanitize-recover=all` on clang lanes and the `address,undefined`
core on GCC. Instrumented executables link their own compiler-matched
runtime — the preset's sanitizer link flags resolved
`capabilities.asan.runtime` through the configured driver's own
`-print-file-name` at configure time, and an unresolvable or
wrong-architecture runtime fails configure closed. The transition-era
`tools/native_runtime.py` preload launcher (`DYLD_INSERT_LIBRARIES`/
`LD_PRELOAD` injection plus `PYTHONMALLOC=malloc`/`ASAN_OPTIONS` plumbing)
was deleted with the interpreter at T63.

RTSan (`-fsanitize=realtime`) verifies the marked hot path never allocates or
locks — the mechanical half of the allocation-free tick requirement. It needs
a separately qualified LLVM toolchain: the measured macOS lane uses the brew
clang (Apple clang rejects the flag), and on Linux the Ubuntu 24.04 default
clang-18 lacks the realtime runtime too — the `rtsan` preset explicitly
selects `/opt/homebrew/opt/llvm/bin/clang++` on macOS (LLVM >= 20 is the
floor), and configure fails closed rather than silently skipping the
lane:

~~~bash
cmake --workflow --preset rtsan
bash tools/run_tests.sh native rtsan
```

`Stepper::try_step`/`try_forward` and every writer `try_*`/`try_*_into`
warm-core entry carry `BIKE_NONBLOCKING` (rtsan.hpp); the throwing
`step`/`forward` wrappers reach the marked functions underneath. The full
native suite runs under RTSan with zero reports — mj_step/mj_forward are
verified allocation-free. Marked functions that start allocating abort the
test with a report.

Coverage:

~~~bash
cmake --workflow --preset coverage-report
```

The preset configures the instrumented `coverage` build, runs the native
tests, merges only fresh `.profraw` profiles and writes
`build/coverage/coverage.lcov` plus `coverage-summary.txt`.
`coverage_provenance.json` records the selected build, reported objects,
profile count, LLVM tool versions and the tool-resolution/format-probe
evidence. The transition-era `tools/native_coverage.sh` wrapper and
`tools/native_coverage.py` finalizer were deleted at T63 — the preset is
the coverage entry point.

The `llvm-profdata`/`llvm-cov` pair is qualified by a real
instrumented control — compile with `-fprofile-instr-generate
-fcoverage-mapping`, run under a fresh `LLVM_PROFILE_FILE`, merge, export —
against the compiler recorded in the build context. Resolution order is:
explicit `LLVM_PROFDATA`/`LLVM_COV` overrides (an invalid override is an
error, not a fallback), a sibling beside the configured compiler, a PATH
candidate, and — on macOS only — `xcrun --find` for the Xcode toolchain.
Linux never invokes `xcrun`. The coverage configure rejects a
non-clang compiler, a compiler that fails the instrumentation flag probe,
and a tool pair that cannot merge/export the control profile; mismatched
tools fail with the command diagnostics rather than an empty report.

V4 verification snapshot (2026-10-06): the regular native profile passed with
782 tests and 15 warnings; `full --collect-only` collected 1,253 tests without
executing them. The ASan native preflight and contract target also passed. The
custom build at `native/build/v4-custom-coverage`, run `run.wazeIy`, merged 17
profiles from 690 native test items. Its imported extension, report object, and
object path were identical. Apple LLVM 21.0.0 reported 91.63% line and 78.00%
branch coverage for that run. The real one-file control reported 3/4 lines and
1/2 branches covered, exercising both a covered and an uncovered path. These
are observed counts, not minimum coverage thresholds. Engine-call wrapper
checks remain pending on E1. The slow `full` profile was not run.

The launcher sets `PYTHONDEVMODE=1` everywhere and `MallocScribble`,
`MallocPreScribble`, `MallocGuardEdges` on quick/native (not `full`, which
measures realtime). It resolves the selected ASan or RTSan runtime from the
selected compiler and passes it to CTest and pytest after `uv` launches them.

`tests/reference/test_native_state_fuzz.py` mutates genuine state snapshots
through Hypothesis and asserts the restore parsers only ever raise the
contracted error types and never corrupt live state on rejection.

## Timing and current acceptance

Prior measurements were approximately 2 seconds for `-m 'not slow'`, 6 seconds
for the six-module P2 focused run, and 478 seconds for the full suite on commit
`a216f36`. These are measurements from prior runs, not runtime promises.

The quick and P2 focused checks were green in those runs. The full run on
`a216f36` had 520 passes and 24 failures; passing quick or native checks does not
establish full physical or realtime acceptance. Full episodes are explicit
because they take minutes and realtime tests measure machine speed. Keep those
tests serial and evaluate their failures separately when full acceptance is
needed.

Native P3 scalar drive policies use unchanged Python policies as bytewise
oracles, with a real compiled scalar MuJoCo model for configuration projection.
The focused check is:

~~~bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drive_policies.py tests/reference/test_native_cruise.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
~~~

The policy test asserts that `bike_native` came from `native/build`; a missing
extension or API fails visibly. Coverage includes sequential coasting/mash/slew,
shifting/landing/slip filters, all four profile modes and the eMTB ramp, motor
curve/power/speed ceilings, lag and gates, electrical losses/energy caps, battery
depletion, elastic freehub overrun, configuration ownership and atomic replay.
Every mutable policy field is checked after each mutating oracle call.

`DrivePolicies.state()` returns exactly these nested sections and keys:

| Section | Keys |
| --- | --- |
| `pedaling` | `coasting`, `target_phase_rad`, `target_rate_rad_s`, `deceleration_rad_s2`, `_effort`, `_cadence_ema` |
| `shifting` | `rear_teeth`, `from_teeth`, `cooldown_s`, `cut_remaining_s`, `shift_count`, `direction`, `cadence_ema`, `required_ema` |
| `assist` | `torque`, `pedaling`, `last_gain` |
| `battery` | `initial_energy_j`, `energy_j`, `drawn_energy_j` |
| `hub` | `boundary`, `energy_j`, `torque_nm` |

Optional values retain `None`. State dictionaries own their values, and complete
restoration validates all policies before changing any. Finite snapshots can
contain signed rates, signed zero, an arbitrary finite hub boundary and shifter
EMAs, and positive torques/energy beyond configured ceilings. Tooth counts are
integers in `[3, 2147483647]`; shift counts are integers in `[0, 2147483647]`
on this platform. A successful shift at the count maximum raises `OverflowError`
without changing policy state. The diagnostic
battery draw mirrors `Battery.draw` directly; the battery `enabled` flag is for
the drivetrain writer to decide whether to reserve/debit energy.

Bitwise curve interpolation depends on this platform's NumPy build. Its scalar
interpolation uses a fused multiply/add: for the segment `(0,80)` to `(43,71)`
at 10 rpm, separate multiply/add yields `0x1.37a0be82fa0bep+6`, while NumPy and
explicit FMA yield `0x1.37a0be82fa0bfp+6`. The port uses explicit `std::fma` only
in interpolation; `-ffp-contract=off` stays active for Python scalar equations.
A different NumPy build or platform requires repeating the bytewise gate.
These scalar checks do not establish whole-episode drivetrain equivalence.

### P3 model-owned drivetrain

Task 2 follows the scalar-policy gate: first chain geometry and the typed
ideal/geometric transmission per-call gate, then the writer, complete snapshots,
and solved energy accounting. `_drive_core` is a private per-call test adapter
on the existing Stepper owner; it reuses `Transmission` and the writer snapshot
conversion. It does not introduce a simulation owner or a step loop.

~~~bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drivetrain.py tests/reference/test_native_drive_policies.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build/asan -j4
~~~

Run the same two test modules against `native/build/asan` with
`NATIVE_TEST_BUILD_DIR=asan`; pass `DYLD_INSERT_LIBRARIES`, `ASAN_OPTIONS`,
and `PYTHONMALLOC` after `uv` starts via `uv run env ... python`. The shared
loader selects and verifies the extension path. The policy and drivetrain
tests accept exactly the regular or explicitly selected sanitizer build;
neither permits an arbitrary extension fallback.

All three transmission models (`elastic_chain`, `ideal_mid_drive`,
`geometric_ideal_mid_drive`) support the four construction-time drive modes.
Effort modes require `mid_drive`; articulated effort excludes `human_crank`.
Passive modes permit no human or motor actuator. Ideal effort modes support
an optional legacy crank clutch or an inertial rotor freewheel, exclusively.
Elastic mode needs cassette/rear-wheel sibling bodies; geometric mode needs
one fixed-tendon coefficient per scalar coordinate, and planar sprocket frames.
Automatic shifting needs an ideal transmission. Rotor/clutch passive modes
and unknown modes/models fail explicitly. Projection copies initial config
gearing; it does not serialize the shifter's changed rear sprocket as reset gear.

`drive_state()` has exactly these top-level fields:
`policies`, `shift_time_s`, `last_time_s`, `reference`, `psi`, `angles`, `last`,
`probe_last`, `pending_actuation`, `ideal_hub`, `clutch`, and `freewheel`.
`policies` uses the five scalar schemas above; its `hub` is `None` for ideal
models. Angles are `None` before initialization, otherwise one crank angle for
ideal models or crank/cassette angles for elastic models. Optional clocks,
reference, psi, pending actuation, probe diagnostics, and absent transmissions
retain `None`. Pending actuation contains `requested`, `omega`, `dt`, `enabled`.

Each transmission snapshot contains `ratio`, `rear_teeth`, `boundary`,
`prepared`, `diagnostics`, `shift_pending`, `shift_parameter_work_j`,
`shift_constraint_work_j`, `last_tension_n`, `range` (two model tendon limits),
and `coefficients` (one driver coefficient for ideal, nv coefficients for
geometric). Geometric `prepared` contains `phi`, `time`, `jacobian` and `qpos`;
the two vectors are nv wide. Every field is owning. Restoration parses and
validates all sections, enum/bool/integer/numeric types, vector widths and
positive ratios before changing policies or model coefficients/limits.
Detached `mjData` handles model constants and endpoint geometry. No derived
solver arrays or Python engine addresses are restored across the FFI.

Coverage compares every force channel, control, diagnostic and mutable state
with unchanged Python algorithms, including forward multi-turn angles,
shifts, probes, reset/restart, injected state and both effort modes. Genuine
8–10-step solves cover dense/sparse Jacobians, unrelated joint-limit rows,
wheel/rotor overrun, both clutch topologies, actual motor delivery below its
request, disabled/depleted batteries, and energy ceilings. `set_inputs` accepts
finite nu/nv arrays and snapshots both before either write, including crossed
aliases. Probe diagnostics are separate while live policy/transmission state
is preserved; force/control writes follow Python behavior.

These gates establish per-call and short solved-interval parity on the pinned
MuJoCo/libm/NumPy/Accelerate platform. They do not establish long episode
acceptance, an allocation-free P4 tick, viewer integration, or a speedup.
After task reviews the controller runs the quick profile once; the previously
measured eight-minute full suite is not repeated in this increment.

### P3 finite rider support geometry and grip laws

~~~bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_rider_contact_math.py tests/reference/test_native_tire.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
~~~

The contact math test asserts the exact regular extension directory or the
explicit `NATIVE_TEST_BUILD_DIR=asan` selection. Finite box faces, edges,
corners, medial ties, footprint boundaries, sole height/projection, and grip
energy/release use the unchanged Python implementations as raw float64 byte
oracles. It retains unreachable targets as a native `UnreachableSoleTarget`
subclass of `ValueError`, and retains target nonconvergence as `RuntimeError`.
The 32-iteration height solve and 18-iteration goal projection are unchanged.
Diagnostic vectors and dictionaries own their storage; array inputs accept
numeric dtype and layout conversions. Native sole diagnostics require finite
scalars, nonnegative pad half length, positive pad radius, and a proper planar
box. Compression and shear may be signed.

Matrix/vector and dot operations call the same Accelerate operation shapes as
NumPy on this platform. Both occurrences of the grip velocity dot product
retain Python's expression order. The tire writer and downstream rider writer use the
same typed `contactlaw::normal_contact` and `contactlaw::brush_step`; the tire
calculation and finite/passivity guards are preserved by literal extraction.

CPython 3.13 and 3.14 (identical) `math.hypot` uses exact products, compensated accumulation, and
a square-root differential correction. The typed port follows that algorithm
for both two-coordinate contact distances and three-coordinate distances.
System two-argument `hypot` differs by one ULP for the corner delta
`(-0x1.999999999999ap-4, -0x1.47ae147ae1475p-6)` (system result ends in `32`,
Python in `31`); C++ three-argument `hypot` also loses the subnormal norm of
`(5e-324, 5e-324, 5e-324)`. The suite retains these regressions and tests
three-coordinate norms across subnormal/normal/large ranges. Explicit FMA
computes the product error only; `-ffp-contract=off` remains active.

The private `_rider_hypot3` diagnostic observes the typed compensated helper.
`_rider_validate_support_model(stepper, geom_ids)` uses the existing Stepper's
owned compiled model and current data. Real tiny models cover planar hinge and
slide topology, unrelated nonplanar bodies, invalid joint topology, non-box
supports, improper support frames, and out-of-range geom IDs. These are local
per-call gates; whole-episode contact and P4 runtime acceptance remain separate.

### T2a equality reactions and DGELSD least squares

~~~bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_attachment_wrench.py -q
~~~

`Stepper::rider_equality_qfrc(eq_id)` and the `_rider_equality_rows`,
`_rider_equality_reaction`, `_rider_equalities_qfrc_at` hooks read the
Stepper-owned EFC arena on every call: `efc_type`/`efc_id` masks are
re-derived, equality rows are grouped by id with a stable ascending order,
and a missing equality is a zero measurement — force `zeros(3)`, residual
`0.`, qfrc `zeros(nv)`. `equality_qfrc` zero-fills its output before
`mj_mulJacTVec`, whose `nefc == 0` early return never touches the result.
Row membership is never cached; contact/limit/friction rows interleaving or
colliding on `efc_id` cannot shadow an equality because the type half of
the mask selects first. Outputs are capsule-owned copies — writeable,
unshared storage on every call.

`_rider_least_squares(a, b, rcond)` drives Accelerate ILP64
`_dgelsd$NEWLAPACK$ILP64` — the same undefined import numpy's
`_umath_linalg` carries — with numpy's buffer contract (LDA = max(1, m),
LDB = max(1, m, n), one LWORK = -1 query, the queried optimum passed
verbatim) and numpy's output contract: `x` is `(n,)` for a 1-D `b` and
`(n, k)` for 2-D, `resids` is empty unless `rank == n` and `m > n`, `s` is
`min(m, n)` singular values, and `rank` is DGELSD's count. Nonfinite inputs
flow into DGELSD unscreened; a nonzero `info` or a pre-set FE_INVALID flag
raises `LeastSquaresError` (a ValueError subclass) with numpy's
"SVD did not converge in Linear Least Squares" message. Shape violations
raise ValueError before any copy; a row-count mismatch raises
"Incompatible dimensions" like the oracle. A `(m, 0)` target still runs
the SVD (rank/s populated) and emits the empty `x`/`resids`. Non-float64
inputs convert to float64 before solving — numpy would dispatch f32 to
`sgelsd`, so the bitwise oracle there is `lstsq` on the widened inputs.

Two parity subtleties the tests pin: numpy's gufunc binary was compiled
with FP contraction, so its tail-row `res += el*el` fused multiply-adds —
the port reproduces that rounding with an explicit `std::fma` even under
`-ffp-contract=off`; and `np.linalg.norm`'s 1-D fast path is `x.dot(x)`
through Accelerate `ddot`, which the residual reader calls directly rather
than re-associating.

### T2b solved attachment wrench measurements

~~~bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_attachment_wrench.py -q
~~~

`native/src/rider/attachment_wrench.{hpp,cpp}` is the typed, Python-free
port of `bike_sim/sim/ride/attachment_wrench.py`; the binding face lives
in `native/src/rider/attachment_binding.cpp` (the existing
`bind_rider_attachment` seam) as

- `stepper.rider_equality_qfrc(eq_id)` (T2a, unchanged),
- `stepper.rider_relative_planar_jacobian(body_a, body_b, point,
  rotational)`,
- `stepper.rider_prepare_attachment(eq_id, body_rider, body_bike, point,
  normal, kind, rotational, half_patch_m=0., pull_direction=None)`,
- `stepper.rider_attachment_raw(...)`,
- `stepper.rider_attachment_raw_from_geometry(geometry,
  validate_wrench=True)`,
- `stepper.rider_attachment_sample(...)`,
- `bike_native.rider_recover_wrench(relative_jacobian, qfrc)` and
- `bike_native.rider_decompose_wrench(wrench, normal, kind, rotational,
  half_patch_m=0., gap_m=0., pull_direction=None)`.

The Stepper's own `(m_, d_)` is the only model/data pair ever read;
`set_state` copies no derived engine buffers, so a caller that only
mirrors `qpos` needs `stepper.forward()` for `cdof`/`xpos` before any
jacobian read — `mj_jac` consumes `cdof`, which `mj_kinematics` alone
does not refresh (verified on the pinned 3.12.0 dylib). Every returned
array is capsule-owned storage; geometry/raw/sample arrive as dicts with
the dataclasses' exact field names and compare bitwise against the
oracle, including soft-CONNECT per-body compiled anchors
(`_attachment_points` reads `eq_type[eq_id]` only on the nonrotational
path — a bogus `eq_id` then indexes like numpy: negatives wrap,
out-of-range is IndexError).

The least-squares solves reuse the T2a `_dgelsd$NEWLAPACK$ILP64`
workspace at the oracle's `rcond=1e-12`. `recover_wrench` additionally
screens nonfinite input (`'nonfinite wrench input'`) and requires full
column rank of the transposed matrix
(`'rank-deficient attachment Jacobian'`); reconstruction checks use
numpy 2.5's asymmetric isclose on the second operand as the relative
reference — `(|a-b| <= atol + rtol*|b| AND isfinite(b)) OR (a == b)` —
so `allclose(Jᵀw, q, rtol=1e-8, atol=1e-8)`, the Newton-third-law gate
(`rtol=1e-4`, `atol=1e-4*scale`) and the `1e-6*scale` absolute
out-of-plane bound behave bitwise-identically, including `inf == inf`
pass-throughs and NaN rejections. Error strings, types and evaluation
order match the oracle (`'attachment body carries no degrees of
freedom'`, `'attachment bodies share kinematic support'`,
`'in-plane attachment force is not observable'`,
`'rank-deficient attachment Jacobian'`,
`'attachment wrench does not explain generalized force'`,
`'attachment wrenches fail Newton third law'`,
`'attachment wrench leaves the planar model'`, the normal/pull
validation messages), and `validate_wrench=False` defers every spatial
check exactly like the oracle's fast path. `equality_rows` membership is
re-derived from the current `efc_type`/`efc_id` arena on every call, so
contact/limit rows shifting the arena cannot produce a stale gap: a
missing equality reports `gap_m == 0.`.

### T3b model-owned rider contact writer

~~~bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_rider_contacts.py -q
~~~

`native/src/writers/rider_contacts.{hpp,cpp}` is the model-owned port of
`RiderContactApplier` (`src/bike_sim/sim/ride/rider_contacts.py`); the
Stepper method face lives in `native/src/rider/rider_contact_binding.cpp`
(the `bind_rider_contacts` seam) as

- `stepper.rider_contacts_reset()`,
- `stepper.rider_contacts_restart_clock()`,
- `stepper.rider_contacts_initialize_settled_state()`,
- `stepper.rider_contacts_release_all()`,
- `stepper.rider_contacts_set_enabled(name, enabled)`,
- `stepper.rider_contacts_qfrc(dt, advance=True, detailed=True)`,
- `stepper.rider_contacts_stored_energy()`,
- `stepper.rider_contacts_diagnostics(probe=False)`,
- `stepper.rider_contacts_state()` and
- `stepper.set_rider_contacts_state(state)`,

plus the T3b-2 settle/sample face

- `stepper.rider_contacts_prepare_attachment_raw()` — returns
  `(geometry dict by name, errors tuple)` captured at the incoming pose;
- `stepper.rider_contacts_settle(interval_state=None, raw=False,
  prepared=None)` — the `settle_welds` result dict, or the
  `(dict, samples, errors)` triple when `raw=True`;
- `stepper.rider_contacts_attachment_samples(interval_state=None,
  raw=False)` — `(samples dict, errors tuple)`.

The writer is owned by `Stepper` like the other writers: constructed when
the config carries a `rider_contacts` section, reset by
`Stepper::reset()`, and reachable only through the methods above — a
Stepper built without the section answers the shared `logic_error`. The
typed config parse sits in `native/src/config.hpp`
(`rider_contacts_from_dict`) behind the schema-2 section reader; the
`RiderContactsConfig` struct and its domain validation live in
`native/src/rider/contact_config.hpp` (T3a).

Ported in this wave: the two-pad unilateral supports (compiled box pads
on the named support geoms, `contactlaw::normal_contact`/`brush_step`,
tangent transport with face-switch loss, paired-reaction generalized
force through `jrel` — the rider-vs-bike relative point Jacobian — and
per-pad patch diagnostics), the releasable spring grip plus its cohesive
pair overload, the connect-grip diagnostics path reading live equality
residuals, welded/pinned/spindle support diagnostics against the settled
latch, capture/release through `set_enabled` (grip capture gated on
distance and approach speed, release loss booked into
`pending_release_loss_j`), probe publication for `advance=False`
evaluations, and the serializable state container with atomic staged
validation before commit. The T3b-2 face adds `settle_welds`' solved-
equality latch (`_settled_welds` — support/grip reaction dict plus the
crank-torque scalar the next `compute_qfrc` reads), interval-start
`prepare_attachment_raw` geometry capture, and `attachment_samples`'
scalar/raw measurements with the optional `(qpos, qvel)` interval-state
swap. Interval measurement validates widths and finiteness before any
buffer is touched, then restores the live `qpos`/`qvel` and refreshes
`mj_kinematics`/`mj_comPos` on every exit path — success, collected
errors, or a propagating failure. Measurement errors arrive as the
oracle's `'<attachment>:unobservable_attachment_wrench'` strings, never
as fabricated zero samples.

Numerical parity follows the oracle's expression order: dot products and
`np.linalg.norm` route through CBLAS `ddot`, `jrel @ qvel`/`R @ v`/
`jrel.T @ f` through `dgemv`, `math.hypot` through `rider::hypot3`,
`x ** y` through `pyfloat::pow`, `np.cross` through literal component
products, and `min`/`max` through `std::min`/`std::max` — under the
module-wide `-ffp-contract=off`. `mj_kinematics`/`mj_comPos` refreshes in
the capture/release paths go through `engine::invoke` so a fatal MuJoCo
error poisons the owning Stepper; `mj_jac` and `mj_name2id` are pure
arithmetic/table reads and are invoked directly.
`rider_contacts_stored_energy` stays callable after a fatal engine error
poisons the mutation paths (same stance as the other read-only component
reads).

Verification ran `cmake --build native/build --target check_frontends`
and `bash tools/run_tests.sh native` to green: all static-analysis
sweeps, CTest 4/4, the structural contract checks, and the full
native+golden pytest sweep (2128 items) passed. The reference file's 63
tests compare qfrc, diagnostics trees, state snapshots, settle results
and attachment measurements/errors bitwise against the live Python oracle
on tiny MuJoCo models, across the flat/weld/pin/spindle × spring/connect
attachment matrix — including latch-fed next-step qfrc, the raw
interval-start-geometry path, interval-state pose restoration (success
and collected-error exits), unobservable-attachment error collection, and
settled snapshot round-trip/ownership/atomicity.

## V3 verification record

Verification was run from the V3 tree based on `1ccdfcb28244dd14af6e584f958a40d3dbc04918`.
The normal native profile passed all preflight gates: 22 selected translation
units, 27 headers across 54 header TUs, six diagnostic controls, all SDK and
hardening controls, and CTest 1/1. Pytest collected 690 native extension
items and completed 758 passed with 15 runtime warnings. The selected artifact
was `native/build/bike_native.cpython-314-darwin.so`.

The selected ASan smoke used `native/build/asan`, passed the same preflight,
verified the selected dylib in the Python process's dyld image list, and passed
the wrong-preload control, one Stepper extension test, and the deliberate C++
`require` failure control: 6 passed, 752 deselected. The `require` control also
passed against separately configured Debug and Release builds.

The current `full` profile completed with 1,205 passed, 24 failed, and 48
warnings in 611.29 seconds. To establish whether the failures predated V3, the
24 failing node IDs were rerun from an isolated archive of commit `1ccdfcb` in
`/private/tmp/cpp-port-p2-baseline-1ccdfcb`. That tree used the shared locked
Python environment with `PYTHONPATH` pointing at its own `src` and
`native/build`; `bike_sim`, `bike_native`, and all 21 source-manifest entries
resolved inside the baseline directory. The selected baseline run produced
24 failed and 15 warnings in 579.75 seconds. Its `.pytest_cache/lastfailed`
set exactly matched the current full run's set. These are baseline failures;
the full profile is not green.

Representative baseline traces were: the flat-launch motor threshold observed
0.5875 against a required 0.8; two coupled-rider tests raised `TOMLDecodeError`
for a duplicate key at line 104; the scalar/batch period comparison differed
by about `8.72e-5 J`; the planar-arm model lacked
`rider_ankle_front`/`rider_ankle_rear`; and a weld-equilibrium run stopped at
40,000 steps with residual `0.197922`. Rider-weld load checks measured about
632 N against a 785 N rider weight.

The 13 failing node IDs, identical in the baseline and current full run, are:

- `closed_form_rider/flat_launch_delivers_crank_torque` — resolved-in-refresh: measured PASS at `fe4feaa` on 2026-10-23 (the 2026-10-23 baseline refresh record (removed with the migration ledger)); retained for the historical record.
- `joint_strength/stepped_effort_respects_strength_and_the_power_budget` — resolved-in-refresh: measured PASS at `fe4feaa` on 2026-10-23 (the 2026-10-23 baseline refresh record (removed with the migration ledger)); retained for the historical record.
- `realistic_pedelec_acceptance/flat_reaches_25_kmh_within_10_s`
- `realistic_pedelec_acceptance/motor_follows_the_rider_through_a_shift_without_an_extra_cut`
- `realistic_pedelec_acceptance/fifteen_percent_climb_holds_12_kmh`
- `realistic_pedelec_acceptance/savage_is_ridden_to_the_end`
- `realtime_gate/realtime_factor_for_full_twenty_seconds/rough_uphill_savage.toml` — superseded-in-place (F3, 2026-10-25): the >=1x RTF contract now measures the native Release `bike-measure-realtime` artifact via CTest nodes `reference.realtime_gate.rough_uphill_savage`/`.rough_uphill_extreme` in the root graph (`build/native-release`, container `root-clang`). This Python node still measures the T63-deleted oracle accounted path (~0.62-0.69 on these tracks) and stays baseline-red until T63 removes it; it is superseded, not fixed.
- `realtime_gate/realtime_factor_for_full_twenty_seconds/rough_uphill_extreme.toml` — same F3 supersession as the savage row above; the contract successor is `reference.realtime_gate.rough_uphill_extreme`.
- `seated_pedaling_cycle/strict_flat_cycle_delivers_and_stays_within_budgets/20.0` — resolved-in-refresh: measured PASS on the lane/b F1 fix round on 2026-10-25 (`task-F1-report.md`), strict on both darwin-arm64 and linux-aarch64; retained for the historical record.
- `seated_pedaling_cycle/strict_flat_cycle_delivers_and_stays_within_budgets/40.0` — resolved-in-refresh: measured PASS on the lane/b F1 fix round on 2026-10-25 (`task-F1-report.md`), strict on both darwin-arm64 and linux-aarch64; retained for the historical record.
- `seated_pedaling_cycle/strict_flat_cycle_delivers_and_stays_within_budgets/60.0` — resolved-in-refresh: measured PASS on the lane/b F1 fix round on 2026-10-25 (`task-F1-report.md`), strict on both darwin-arm64 and linux-aarch64; retained for the historical record.
- `seated_pedaling_cycle/free_coast_is_an_intent_not_a_crank_lock` — resolved-in-refresh: measured PASS on the lane/b F1 fix round on 2026-10-25 (`task-F1-report.md`), strict on both darwin-arm64 and linux-aarch64; retained for the historical record.
- `seated_posture_program/pulse_redistributes_load_through_inertia_only` — resolved-in-refresh: measured PASS on the lane/b F1 fix round on 2026-10-25 (`task-F1-report.md`); strict on darwin-arm64, linux-aarch64 runs the monitor in diagnostic mode with a pinned failure signature (D16 disposition in the F1 section); retained for the historical record.

2026-10-24 addition (batch 3.12b lane/a triage): two further nodes of
`test_native_pneumatic_tire.py` were verified red on the unmodified
baseline (stash + rerun at `e843f0d`), so they join the documented
baseline set rather than blocking the port batch:

- `native_pneumatic_tire/pneumatic_golden_episode_event_rich` — episode capture exhausts the 64,000-step equilibrium budget (`front_wheel_spin` residual ≈ 0.081; same savage-plant defect family as `test_savage_is_ridden_to_the_end`).
- `native_pneumatic_tire/pneumatic_probe_and_clock` — the recorded probe interval diverges bitwise from the live simulation at step 1 on both platforms.

2026-10-23 refresh (the 2026-10-23 baseline refresh record (removed with the migration ledger), measured at
`fe4feaa`): the two annotated nodes now pass; the other 11 nodes remain
failing with deterministic signatures, so the live residual is 11 nodes.

2026-10-25 lane/b F1 fix round (`task-F1-report.md`): the five
seated-pedalling/posture nodes annotated above now pass — the four
pedaling-cycle nodes strict on both darwin-arm64 and linux-aarch64, the
posture pulse node strict on darwin-arm64 and diagnostic with a pinned
`energy.constraint_work` signature on linux-aarch64 (D16 disposition in
the F1 section). The live residual of the original 13 is 6 nodes.
The list above is kept as the historical baseline record.

## E1 recoverable MuJoCo errors

The native dependency group in `pyproject.toml` pins MuJoCo 3.12.0 and
nanobind >=3.1.0. Native launchers use `uv run --frozen --group native`; CMake
checks the imported MuJoCo version and compile/links the private TLS handler
ABI before building the extension. The exact upstream 3.12.0 tag is commit
`13827e9ee56f097f57acf69ae52b078f9839682d`.

`native/src/engine_call.cpp` owns the pinned private ABI. A thread-local,
fixed-storage frame catches fatal `mj_loadModelBuffer`, private raw-data
allocation, `mj_forward`, `mj_step`, `mj_resetData`, and `mj_setConst` calls
without unwinding through C frames, then restores the previous handler.
Nonfatal messages reach the outer frame's forwarding destination, including
native/stock/native nesting without warning recursion.
The extension exports the actual `mujoco.FatalError` class; a fatal runtime
error poisons its Stepper until a guarded `reset` or valid `set_state` succeeds.
Recovery also resets owned drivetrain pending work/clocks and tire/cruise
state before clearing poison. Read-only state views remain accessible while
poisoned. Native model loading reads a local MJB into owned bytes before
entering the error frame; resource-provider paths and plugin-bearing models
are unsupported. The plugin count is checked against the pinned 3.12.0 MJB
header before `mj_loadModelBuffer`, since sensor plugins can execute during
load. A value-initialized, owned `mjData` is passed to the pinned private
`mj_makeRawData`, so partial raw-data allocations can be released after a
fatal error. Reset then runs through a separate guarded call. Invalid
history/timestep combinations are rejected before data allocation. Constructor
resources are held by local owners until initial forward succeeds.

One pinned-library cleanup limit remains: `mj_loadModelBuffer` can allocate an
internal `mjModel` and then call its fatal handler before returning any pointer
to the caller. A model with an invalid equality type reproduces a recoverable
`mujoco.FatalError` at `mj_validateReferences`, but the native owner cannot
free MuJoCo's unreturned partial model. The same applies to a late allocation
failure inside that loader. The user chose to document this upstream limit,
keep the S5 cleanup requirement and overall gate open, and continue independent
plan tasks. Process survival does not prove leak-free loading.

MuJoCo's stock `MjData` installs a C-level `mjcb_time = GetTime`, so that slot
is supported. Its module origin is checked and its lazy clock epoch is warmed
outside marked realtime regions when the callback pointer changes. Stock Python
time callback errors retain their Python exception during unmarked
construction; an installed `ctypes.CFuncPtr` timer is rejected before any
native constructor engine call. An installed Python timer callback is rejected before runtime
`step`/`forward` with `ValueError`. Direct C++ callers of `try_step`/`try_forward`
must call `refresh_time_callback_policy` outside their realtime loop after
callback changes. Other installed `mjcb_*` and legacy allocator/log callbacks
are rejected before an owner mutation or native engine call; they have no validated cleanup-safe
boundary for this owner. Concurrent owners use separate Stepper instances; do
not mutate process-global callbacks concurrently with a native engine call.

The closed solver enum is validated in the fixed-storage realtime status path.
An invalid solver returns a fatal status before entering MuJoCo's formatter:
the latter calls `snprintf`, which takes a libc lock under RTSan. The general
interceptor still handles other fatal engine errors. Arbitrary future MuJoCo
fatal formatting inside a marked realtime call is not claimed RTSan-safe;
adding such a path requires a separate precheck or a changed engine contract.

| Checker | Narrow origin and reason | Reproducer | Remove when |
|---|---|---|---|
| `-Wreserved-identifier` | Two declarations in `engine_abi_312.hpp` use MuJoCo's exact exported private names. | CMake `BIKE_MUJOCO_ERROR_ABI_312` probe | A public per-thread error API replaces these symbols. |
| `cert-err52-cpp`, `modernize-avoid-setjmp-longjmp`, array-to-pointer decay | The two jump calls in `engine_call.cpp` cross only trivial C frames; C++ unwinding through MuJoCo is undefined. | `test_native_engine_errors.py` fatal subprocess cases | MuJoCo provides a recoverable public error API. |
| `cppcoreguidelines-avoid-non-const-global-variables`, `misc-const-correctness` | One thread-local frame pointer and its mutable jump buffer are required for nested handlers. | `native_contract_tests` nested, concurrent, and handler-restoration cases | A public scoped handler removes the local frame. |
| cppcheck `danglingLifetime` | `engine_call.cpp` points at a stack frame only while `invoke` runs; both normal and jump exits restore its predecessor before return. | `native_contract_tests` nested/concurrent/handler-restoration cases | A public scoped handler removes the frame or cppcheck follows both exits. |
| `bugprone-bitwise-pointer-cast` | `stepper.cpp` passes the stock timer's function address to Apple's `dladdr`; the pinned Apple arm64 ABI has equal-sized function and object pointers and C++ offers no portable `dladdr` argument conversion. | `test_stock_timer_installed_after_native_construction_is_allowed` and the ctypes constructor rejection control | MuJoCo exposes stock timer identity or a portable timer callback query. |
| `cppcoreguidelines-pro-type-vararg`, compiler format-buffer warning | One `test_contracts.cpp` call uses MuJoCo's variadic error API with a NUL-terminated literal. | `native_contract_tests` fatal-status case | A typed test error entry point replaces the variadic call. |
| `cppcoreguidelines-owning-memory`, `cppcoreguidelines-no-malloc` | The isolated late-allocation contract implements MuJoCo's raw `void*` allocator/free callback pair; ownership is counted and checked after cleanup. | `test_actual_engine_allocation_error_is_caught_in_c_contract[--late-data-allocation-failure]` | A typed allocator injection point replaces the C callbacks. |

E1 focused controls live in `tests/reference/test_native_engine_errors.py`. The
selected ASan/UBSan and RTSan builds run them with the matching runtime loaded
in the Python process. The master plan's separate E2-E4 atomicity gates and the
24 pre-existing full-suite failures above remain open.

E1 verification snapshot on the final local code (2026-10-06):
`bash tools/run_tests.sh native` passed 816 tests with 15 existing runtime
warnings. Selected `native/build/asan` and `native/build/rtsan` profiles each
passed 30 engine-error tests, CTest, artifact/runtime provenance, and mandatory
sweeps. `full -q --tb=no` completed with exactly the same 24 failing node IDs
listed above and no new IDs; its output is retained at
`/private/tmp/e1-full-after-review.log`. The overall `full` gate remains red.
During RTSan verification, the context
checker was corrected to retain the C++ driver symlink and to allow the
default-SDK control to choose its own compiler; the launcher now reads CMake's
`UNINITIALIZED` compiler cache entry as well as `FILEPATH` and `STRING`.

## F1 strict wire readers and native config schema 2

`tools.native_config.project` now emits schema 2. The fork damper carries an
exact `legacy_behavior` boolean; Python `Charger3Damper` retains that flag.
Schema 2 requires it. Schema 1 remains supported and infers the historical
fork branch only when the already resolved `hbo_start_mm == 160.0`. Migration
keeps every resolved click, coefficient, velocity knee, travel and HBO value;
it never reconstructs a damper using defaults. This also preserves the
ambiguous modern 180 mm fork, whose resolved force law is identical. A schema
1 fork rejects schema 2's new metadata key.

Config types and numeric predicates are available independently of Python in
`native/src/config_types.hpp` and `native/src/validation.hpp`. The nanobind
readers use exact keys, full public field paths, exact builtin booleans and
Python/NumPy real and integral scalar classification. Numeric fields and
numeric sequences reject booleans before conversion. Lists, tuples and
promised ordered sequence interfaces remain supported; unordered sets and
bare iterators are rejected. Genuine allocation failures retain their normal
memory exception. Resolved Python projection values are checked before
builtin conversion, including attributes changed after construction.

The existing disabled-writer convention is retained: an empty root dict
turns off all writers; an absent optional writer section disables that
writer; a present section must satisfy its complete schema. Profile-less
custom assist modes and optional sparse diagnostic restore fields remain
valid. Tire row sentinels are checked by their existing named-field domain;
wire validation checks their original numeric types before passing them on.

`tests/reference/test_native_input_contracts.py` covers recursive config and
runtime dictionary paths, malformed keys/types, populated pending/prepared
states, projection mutation, and schema migration. Existing drivetrain,
policy, suspension and state controls retain numerical bitwise expectations.
The shared owning ndarray helpers keep a `unique_ptr` until capsule creation
succeeds, then transfer ownership to that capsule.

| Checker | Narrow origin and reason | Reproducer | Remove when |
|---|---|---|---|
| `cppcoreguidelines-avoid-c-arrays`, `modernize-avoid-c-arrays` | The two `make_unique<T[]>` factories in `binding_arrays.hpp` allocate the runtime-sized primitive storage required by NumPy. `vector<bool>` does not provide contiguous bool storage. | Native suspension, resistance dtype/layout, drive-state arrays and selected ASan input controls | Nanobind supplies an owning contiguous primitive-array factory with the same lifetime and failure guarantees. |
| `cppcoreguidelines-owning-memory` | The two capsule deleters in `binding_arrays.hpp` release the exact array transferred from the preceding `unique_ptr`. | The same owning ndarray controls and selected ASan run | The owning ndarray factory replaces the raw `void*` capsule callback contract. |
| `bugprone-easily-swappable-parameters` | FFI lambdas in `binding.cpp`, `drive_binding.cpp`, and `bind_drive_policies` have the fixed Python argument order. Raw handles are needed to check original types before conversion; every argument has a named field-path reader. | Policy/drivetrain numerical parity plus invalid scalar/array/control input tests | A binding API can expose these Python signatures through distinct typed argument wrappers while preserving strict type checks. |

F1's final normal verification used Python 3.14.3, MuJoCo 3.12.0,
nanobind 3.1.0, NumPy 2.5.2 and Apple clang 21.0.0 on arm64. Commands used
`UV_CACHE_DIR=/private/tmp/e1-uv` and the locked native dependency group.
The focused input/policy/drivetrain/writer/state suite passed 529 tests with
six existing runtime warnings. A bounded paired audit of 46 real, integer,
boolean and sequence cases had no mismatches; separate Unicode and huge
integer controls passed. Standalone Python-free config headers compiled with
`clang++ -std=c++23 -pedantic-errors -fsyntax-only`.

`bash tools/run_tests.sh native` passed 912 tests with 15 existing warnings.
All sweeps selected 23 translation units with healthy execution; 33 headers
across 99 header TUs, six diagnostic controls, SDK/hardening controls and
CTest 1/1 passed. The imported extension was the selected
`native/build/bike_native.cpython-314-darwin.so`. The final log is
`/private/tmp/f1-native-final.log`.

`bash tools/run_tests.sh full -q --tb=no` completed with 1,359 passed and
exactly the same 24 failing IDs as the independently reproduced baseline,
with 48 existing runtime warnings. The extra quiet flag suppresses pytest's
numeric summary; the complete progress lines contain 1,383 items, and the
24 final `FAILED` IDs were compared directly. No new or missing failing IDs
were found. The log and comparison are `/private/tmp/f1-full-final.log` and
`/private/tmp/f1-full-baseline-comparison.json`. **The full gate remains red**;
the 24 baseline failures above remain open.

The selected ASan/UBSan profile passed 529 affected tests, with 383
deselected and six existing warnings in 57.03 seconds. Its mandatory sweeps,
33/99 header controls, diagnostic/context controls and CTest passed. The
test process imported `native/build/asan/bike_native.cpython-314-darwin.so`;
the shared loader required the selected Apple clang 21 ASan runtime to be
present in the process's dyld image list on every import. The final log is
`/private/tmp/f1-asan-final.log`. Reproduce with:

~~~bash
UV_CACHE_DIR=/private/tmp/e1-uv NATIVE_TEST_BUILD_DIR=asan bash tools/run_tests.sh native \
  -k 'native_input_contracts or native_cruise or native_rider_forces or native_tire or native_drive_policies or native_drivetrain or native_suspension or native_state_restore or native_state_fuzz'
~~~

## F2 physical numeric domains and topology

Python's shared scalar/integer/boolean predicates and the Python-free
`native/src/config_validation.hpp` enforce the S4 configuration domains in
constructors and the Python evaluation paths that consume mutable attributes.
`native/src/model_topology.hpp` checks scalar joint kind/axis and joint-actuator
targets. The native classes validate their typed configurations as well as
the wire readers' original input types. Field errors retain the public
section/field path; derived nonfinite values raise `OverflowError` before
force, control, energy or geometry publication.

| Area | Validation and retained behavior | Main controls |
|---|---|---|
| Air | Positive geometry/gamma/atmosphere; nonnegative pressure/token volume; integer counts; positive evaluated chambers; finite pressure/force/calibration/curve energy | Suspension scalar matrix, mutable properties/setters, pressure/energy overflow, calibration retry |
| Damper/coil/end stops | Positive click denominators/knees/stroke; ordered nonnegative damping; exact flags; coil geometry; finite raw stop force before unilateral clipping | Constructor and mutation matrices, activated zero HBO denominator, unloading stop overflow, modern/legacy goldens |
| Brake/rider | Nonnegative physical parameters and finite offsets; scalar/type checks before normalization; finite raw force | Rider matrix, original bool/array rejection at brake bindings, derived-force controls |
| Resistance | Nonnegative coefficients, positive taper, finite planar vectors, physical resolved body IDs; finite mapped point and forces | Body-ID controls, rolling/drag overflow, bitwise resistance goldens |
| Tire/surface | Positive material/brush coefficients, valid loads/provenance, surface bounds, nonoverlapping intervals; mutable material/map revalidation; finite accumulated energy/loss/force | Field/relationship matrices, Unicode blank provenance, mutated tire/map controls, two-wheel loss overflow, tire goldens |
| Pedaling/shifting/gearing | Positive required denominators, ordered thresholds/timings, unique integer cassette, enabled-gear membership, closed slip modes; finite filter/slew/coast/landing results | Paired policies, 0-D cassette rejection, zero-disable and historical NumPy controls |
| Assist/battery/freehub | Positive lag/slew/taper/full-gain denominators, nonnegative physical limits/losses, ordered curves/profile bounds, positive hub stiffness; finite demand/filter/cap/energy results | Battery cap numerator overflow, mutated profile/assist controls, crank conversion and freehub overflow |
| Topology | Cruise slide about local +X; physical wheel/drivetrain spin hinges about local +Y; brake/human/motor actuators target the corresponding joint; generic ideal one-way coupling retains scalar hinge/slide coordinates | Wrong kind/axis/target models plus valid plain/clutch/rotor, generic +Z-hinge/+X-slide preload and solved bitwise controls |

Integer click/token clipping remains intentional normalization. Air pressure
setters/calibration retain their 10 psi floor after domain/derived-value checks.
Declared unreachable air travel is permitted when the evaluated chamber is
usable. Legacy HBO may start beyond travel and keeps its negative-denominator
clamped branch; an activated zero denominator is rejected. Zero coast/filter
and rider-effort slew values retain their existing disabled branches; assist
lag/slew/taper denominators remain positive. Zero physical damping, torque,
power and energy limits are retained where their existing branches permit them.
The cruise integral clamp bound can be infinite at the valid extreme support
control; its actual integral update and published torque must remain finite.
The native tire backend remains `compliant_2d`; configured and track surface
modes are retained.

New checks do not replace operands before arithmetic that previously used a
NumPy scalar/array. Validation is separate from the original scalar conversion
boundaries. A bounded audit loads exact `957e4f0` Python source in isolated
modules and compares dtype, shape and bytes with the current implementation:
34 controls across builtin float, `np.float32`, `np.float64`, damper HBO/stations,
air pressure/calibration/stiffness, assist ceiling/taper, pedaling coast,
shifting landing, rider preload/path, end stops, surface scalar/array laws and
surface-map branch selection
have no mismatch. The audit script/log are
`/private/tmp/f2-numpy-final-audit.py` and `/private/tmp/f2-numpy-final-audit.log`.
The captured historical NumPy regressions retain F1's expected force values;
existing numerical goldens were not rewritten.

`test_native_numeric_domains.py` and `test_native_numeric_completion.py` hold
the paired matrices, mutation/overflow/topology controls and explicit valid
normalization controls. The standalone typed-validator probe uses the selected
build's compiler and SDK and includes only Python-free config/validation
headers. A bounded constructor audit of 60 string/huge-real/bool inputs across
20 constructors reported no unexpected error class after the fixes; log
`/private/tmp/f2-constructor-audit.log`. Review findings were reproduced before
their fixes, with separate logs under `/private/tmp/f2-*-red.log`.

| Checker | Narrow origin and reason | Reproducer | Remove when |
|---|---|---|---|
| `bugprone-easily-swappable-parameters` | `PedalingPolicy::update` and the brake binding lambdas retain the fixed positional Python signatures; every raw binding argument has its own field-path reader. Internal topology helpers separate same-type arguments without suppression. | Pedaling sequences, brake demand/bitwise controls and original bool/array rejection cases | The public signatures can use distinct argument types without breaking compatibility. |
| `-Wunsafe-buffer-usage`, `cppcoreguidelines-pro-bounds-pointer-arithmetic`, `cppcoreguidelines-pro-bounds-array-to-pointer-decay` | `native/src/gpu/{common,suspension_kernel,drivetrain_resistance_kernel,rider_kernel,tire_kernel,batch_step}.hpp`, `cuda_kernels.cu`: the same FP32 kernel source compiles as host C++, CUDA and Metal MSL; device buffers are address-space-qualified raw pointers (`BIKE_THREAD`/`BIKE_DEVICE`) and MSL has no `std::span`. Host-only code (`model_assets.hpp`, runners, tests) carries no suppression. | `bike_test_gpu_*` CTest nodes; Metal runtime shader compile in `bike_test_gpu_batch_runners` | The kernels are split into a host span-based front and a device-only body, or MSL gains a span-equivalent the headers can use. |
| `cppcoreguidelines-avoid-c-arrays`, `modernize-avoid-c-arrays` | Same files: POD structs (`GpuModelIndices`, `GpuRiderParams`, …) are bit-copied into Metal/CUDA buffers and mirrored by Python ctypes (`tools/colab_gpu/build_cuda_writers.py`); `std::array` is unavailable in MSL. | `test_gpu_batch_step` sizeof checks; `static_assert(sizeof(GpuModelIndices) == 228)` | Same as above. |
| `bugprone-easily-swappable-parameters` | Same files: positional float signatures are the shared CUDA/MSL kernel ABI; call sites are covered by per-kernel unit tests with distinct values per argument. | `bike_test_gpu_{suspension,tire,drivetrain_resistance,rider,batch_step}` | Kernel entry points take parameter structs. |
| `-Wunsafe-buffer-usage-in-container` | `native/src/gpu/model_assets.hpp` `detail::model_span(ptr, count)` and its copies `model_span` in `native/tests/gpu/test_gpu_model_{geometry,assets}.cpp`: each is the file's single raw `mjModel` array → `std::span` boundary — MuJoCo owns the arrays and sizes them by the adjacent model counts (`hfield_data`, `hfield_size`, `hfield_ncol`, `geom_type`, `geom_pos`, `jnt_qposadr`, `jnt_dofadr`, `jnt_limited`, `jnt_range`, `geom_size`). Scoped `#pragma clang diagnostic` like `native/src/platform/argv.hpp`; the rest of each host file carries no suppression. | `bash .work/gpu-compliance/compile_tu.sh native/tests/gpu/test_gpu_model_assets.cpp native/src/gpu/model_assets.hpp`; `bash .work/gpu-compliance/compile_tu.sh native/tests/gpu/test_gpu_model_{geometry,assets}.cpp native/tests/gpu/test_gpu_model_{geometry,assets}.cpp`; `bike_test_gpu_model_{assets,geometry}` CTest nodes | The `mjModel` arrays arrive as spans or behind a checked accessor upstream. |
| `bugprone-suspicious-include` | `native/src/gpu/cuda_kernels_host.cpp` `#include`s `gpu/cuda_kernels.cu`: the `.cu` stays nvcc's direct input (Colab), and this TU is its single host C++ compile, the `bike_gpu_kernels` library. The CPU batch runner and the GPU tests declare the `extern "C"` entry points from `cuda_kernels_abi.hpp` and link that library; the GPU tests take `tyre/station.cpp` and `drivetrain/freehub.cpp` from `bike_core` and `rider/spindle_math.cpp` from `bike_rider_math`. The ODR sweep relocatable-links every selected TU into one image, so a second textual compile of any of these would be a duplicate definition. Scoped `NOLINTNEXTLINE` on the single include line only. | `check_odr`, `check_tidy`; the `bike_test_gpu_*` CTest nodes | Each first-party definition has exactly one compile in the check context. |
| `-Wunsafe-buffer-usage-in-container` | `native/src/gpu/cpu_batch_runner.cpp` `boundary_span(ptr, count)`: the file's single raw pointer → `std::span` boundary — the `bike_cpu_run_batch{,_ex,_sweep}` C ABI hands in caller-owned (pointer, count) ranges (`d_configs`, `d_hf_heights`, `init_qpos`, `init_qvel`, `out_*` sized by `nworld` or the caller's sample count) and MuJoCo exposes `mjModel`/`mjData` arrays (`qpos`, `qvel`, `xpos`, `qfrc_applied`, `ctrl`, `tendon_range`) sized by the adjacent model counts. Scoped `#pragma clang diagnostic` like `native/src/platform/argv.hpp` and `model_assets.hpp`'s `detail::model_span`; the rest of the file carries no suppression. | `bash .work/gpu-compliance/compile_tu.sh native/src/gpu/cpu_batch_runner.cpp native/src/gpu/cpu_batch_runner.cpp`; `bike_test_gpu_batch_runners` CTest node | The batch ABI buffers and the `mjModel`/`mjData` arrays arrive as spans or behind checked accessors upstream. |
| `-Wunsafe-buffer-usage-in-container` | `native/src/gpu/metal_batch_runner.mm` `boundary_span(ptr, count)`: the file's single raw pointer → `std::span` boundary — the `bike_metal_{init,launch_*,benchmark_writers_only,run_hybrid_*}` C ABI hands in caller-owned (pointer, count) ranges (`d_configs`, `d_hf_heights`, `init_qpos`, `init_qvel`, `d_states`, `d_qpos`, `d_qvel`, `d_xpos`, `d_qfrc_applied`, `d_ctrl`, `d_tendon_range`, `out_*` sized by `nworld`, the idx dims or the caller's sample count), `[MTLBuffer contents]` exposes each `MTLResourceStorageModeShared` buffer as a raw `void*` over its allocated byte length (`b_st`, `b_qpos`, `b_qvel`, `b_xpos`, `b_qfrc`, `b_ctrl`, `b_tendon`, `b_hf`), and MuJoCo exposes `mjModel`/`mjData` arrays (`qpos`, `qvel`, `xpos`, `qfrc_applied`, `ctrl`, `tendon_range`) sized by the adjacent model counts. Scoped `#pragma clang diagnostic` like `native/src/platform/argv.hpp`, `model_assets.hpp`'s `detail::model_span` and `cpu_batch_runner.cpp`'s `boundary_span`; the rest of the file carries no suppression. | `bash .work/gpu-compliance/compile_tu.sh native/src/gpu/metal_batch_runner.mm native/src/gpu/metal_batch_runner.mm`; `bike_test_gpu_batch_runners` CTest node | The Metal ABI buffers, `[MTLBuffer contents]` and the `mjModel`/`mjData` arrays arrive as spans or behind checked accessors upstream. |

F2's scope follows the validation plan's F2 lane. State/control shapes,
relational restore and clock/interval/live-timestep requirements remain in
F3/F5. Allocation/boxing failure atomicity, complete transactional boundaries,
view/private-hook ownership and realtime allocation/performance work remain
in their E/C/R tasks. These requirements are not marked complete by F2.
E1's upstream model-loader cleanup limitation and the master final full gate
remain open under the documented user decision.

F2 task verification is complete on the frozen implementation; the master
full gate remains red on the 24 documented baseline failures. The focused command is:

~~~bash
UV_CACHE_DIR=/private/tmp/e1-uv uv run --frozen --group native python -m pytest \
  tests/reference/test_native_numeric_completion.py \
  tests/reference/test_native_numeric_domains.py \
  tests/reference/test_native_input_contracts.py \
  tests/reference/test_native_drive_policies.py \
  tests/reference/test_native_drivetrain.py \
  tests/reference/test_native_cruise.py \
  tests/reference/test_native_suspension.py \
  tests/reference/test_native_brake_resistance.py \
  tests/reference/test_native_rider_forces.py \
  tests/reference/test_native_tire.py \
  tests/reference/test_energy_ledger.py -q --tb=short
~~~

Result: 1,297 passed, eight warnings in 48.13 seconds; log
`/private/tmp/f2-focused-preload-final.log`. Six warnings are the existing
reference-monitor warnings; two are intentional overflowing NumPy operations
in the bearing and force-curve rejection probes.

Final normal verification uses Python 3.14.3, MuJoCo 3.12.0, nanobind 3.1.0,
NumPy 2.5.2, CMake 4.4.3 and Apple clang 21.0.0 on arm64, with the explicitly
selected Xcode SDK. The locked native group and
`UV_CACHE_DIR=/private/tmp/e1-uv` are used throughout.

Final `bash tools/run_tests.sh native` passed 1,654 tests with 17 warnings in
122.26 seconds. All mandatory sweeps selected 23 translation units and
reported healthy execution; headers 35/105, diagnostic/context controls,
shellcheck and CTest passed. Alpha analyzer findings remained report-only.
The test process imported the selected
`native/build/bike_native.cpython-314-darwin.so`; log
`/private/tmp/f2-native-preload-final.log`. The 17 warnings include the two deliberate
NumPy overflow probes and 15 existing native/golden reference warnings.

The first full run found 25 failures: the 24 baseline IDs and one new generic
preload case. Its log/comparison are retained in
`/private/tmp/f2-full-preload-regression.log` and
`/private/tmp/f2-full-preload-regression-comparison.json`. The cause was an
axis restriction applied to the generic scalar one-way coupling; the original
slide-joint cache test now passes unchanged, with explicit +Z-hinge and
+X-slide controls retaining solver preload. Physical wheel-axis validation
stays at the owning drivetrain/brake lookup.

Final `bash tools/run_tests.sh full --tb=short` completed with 2,101 passed,
24 failed and 50 warnings in 705.74 seconds. Its failed-ID set exactly matches
the 24 independently reproduced baseline IDs above, with no new or missing
ID. The generic preload regression is absent. The final log/comparison are
`/private/tmp/f2-full-final.log` and
`/private/tmp/f2-full-baseline-comparison.json`. **The full gate remains red**
on the documented baseline failures.

Selected ASan/UBSan passed 1,297 affected tests, with 378 deselected and eight
warnings in 103.43 seconds. Mandatory sweeps, 35/105 header controls,
diagnostic/context controls, CTest and selected-artifact checks passed.
The process imported `native/build/asan/bike_native.cpython-314-darwin.so`;
the shared loader required the selected Apple clang 21 ASan runtime to be
mapped in that Python process on every import. A separate explicit import
proof also confirmed the extension and mapped runtime, recorded at
`/private/tmp/f2-asan-runtime-proof.log`. The selected-build provenance is
`native/build/asan/native_test_provenance.json`; the final run log is
`/private/tmp/f2-asan-final.log`. Reproduce with:

~~~bash
UV_CACHE_DIR=/private/tmp/e1-uv NATIVE_TEST_BUILD_DIR=asan bash tools/run_tests.sh native \
  tests/reference/test_energy_ledger.py \
  -k 'native_numeric_domains or native_numeric_completion or native_input_contracts or native_cruise or native_rider_forces or native_tire or native_drive_policies or native_drivetrain or native_suspension or native_brake_resistance or energy_ledger'
~~~

F2 is ready for root review/integration. No child commit was made. The user
requested a stop before the next task, so no F3/F4/F5/E/C/R task follows this
handoff. E1 and the master final full gate remain open.

## R1 warm-core allocation contract and realtime measurement

R1 removes the per-tick C++ allocations earlier phases sized but still
performed. Each marked warm-core entry point is `noexcept`, returns
`CoreStatus` instead of throwing, and carries `BIKE_NONBLOCKING`
(`native/src/rtsan.hpp`, `[[clang::nonblocking]]` under RTSan-capable
Clang, empty otherwise):

| Marked entry | Status surface |
|---|---|
| `SuspensionWriter::try_components_into` | `ok`, `invalid_input` |
| `BrakeWriter::try_compute_into` | `ok`, `invalid_input` |
| `ResistanceWriter::try_components_into` | `ok`, `invalid_input` |
| `RiderForcesWriter::try_compute_into` | `ok`, `invalid_input` |
| `TireWriter::try_compute_into` | `ok`, `invalid_input` |
| `drivetrain::DrivetrainWriter::try_components_into` | `ok`, `invalid_input`, `invalid_geometry`, `pending_actuation` |
| `Stepper::try_step` / `Stepper::try_forward` | engine `ErrorBuffer` (fixed storage) |
| `DrivetrainWriter::stage_settle` + `commit(PreparedSettlement&)` | the settlement pair the binding interleaves; the commit is `noexcept`, the stage can still reject |

The throwing convenience surfaces (`components()`, `qfrc()`, `torques()`,
the boxed `Stepper` methods) route through the same `*_into` writers over
construction-sized member scratch (`Stepper::suspension_views_`,
`resistance_views_`, `tire_out_`, `rider_out_`); their only allocation is
the returned boxed object. Writer scratch is sized at construction:
`ProfileQuery` owns its candidate/keep-mask/near/normal/dot vectors, the
tire writer owns output/Jacobian/staging buffers, and the drivetrain owns
two alternating tick banks plus two settlement banks whose candidate
slots engage once.

### Allocation counters

`native/tests/allocation_faults.{hpp,cpp}` overrides the throwing global
`new`/`delete` forms in the contract executable only — never in a shipped
target. Two thread-local counters record every scalar, array, aligned,
sized, and sized-aligned call; allocation attempts count even when they
throw, and delete calls count including null releases.

**Counter scope boundary:** the counters see C++ `operator new`/`delete`
only. MuJoCo's internal `mju_malloc`/`mju_free` calls — in particular the
scratch-model copies inside `mj_copyModel` — are C allocations outside
counter scope. RTSan, by contrast, intercepts the malloc family, so a
path can show zero on the matrix yet still trip a realtime check.

### Contract cases

`native_contract_tests` registers three R1 cases (all also listed in
`tests/reference/test_native_loader.py`):

- `warm_core_zero_allocation` — a suspension(physical)+tire(track) rig on
  a bump-terrain model runs four warm ticks through every marked entry
  plus the advance/settle pair, then one tick over a changing contact
  (frame moved across the bump edge inside scratch extents). Zero
  allocation/deallocation activity is required.
- `rtsan_invalid_status_control` — every declared `CoreStatus` control
  flow (invalid_input gates on every writer, the drivetrain's
  pending_actuation, double-advance and invalid_geometry paths) must
  return through the status value with zero allocation/deallocation.
  Assertion text is built lazily so it cannot contaminate the measured
  window. This is the always-on twin of the RTSan build's coverage.
- `warm_allocation_matrix` — the matrix below.

### Allocation matrix

`warm_allocation_matrix` measures each writer/topology combination across
construction, warmup, core rows, operation rows, and the boxed row. Core
rows assert zero allocation **and** zero deallocation;
construction/warmup/op/boxed rows are recorded, not asserted. Observed
counts (Apple clang 21, arm64, `native/build`):

| Row | allocs (frees) | Claim |
|---|---|---|
| drive/*/core_advance, core_probe, core_settle | 0 (0) | asserted zero |
| drive/geometric_ideal_mid_drive/core_advance_coefficient_delta | 0 (0) | recorded — see limitations |
| drive/elastic_chain op_reset / op_restore | 0 / 1 (1) | recorded — out of claim |
| drive/ideal_mid_drive op_reset / op_restore | 1 / 4 (5) | recorded — out of claim |
| drive/geometric_ideal_mid_drive op_reset / op_restore | 7 / 10 (13) | recorded — out of claim |
| drive/*/boxed | 0 (0–2 freed) | recorded — boxed surface |
| suspension_legacy core / boxed | 0 / 8 (8) | core asserted, boxed recorded |
| suspension_physical core / boxed | 0 / 9 (9) | core asserted, boxed recorded |
| brake core / boxed | 0 / 0 | core asserted, boxed recorded |
| resistance core / boxed | 0 / 5 (5) | core asserted, boxed recorded |
| rider core / boxed | 0 / 1 (1) | core asserted, boxed recorded |
| tire_configured, tire_track core / boxed | 0 / 1 (1) | core asserted, boxed recorded |
| engine/forward | 0 (0) | asserted zero |
| */construction | nonzero | recorded — scratch and first-engagement sizing live here |
| drive/ideal_mid_drive, drive/geometric_ideal_mid_drive warmup | 1 (0) | recorded — see multipliers limitation |
| all other */warmup | 0 (0) | recorded — the committed-slot storage below never engages inside marked code |

Warmup rows are unasserted, but after the lazy-engagement fixes below
every warmup is C++-quiet except one: `Transmission::multipliers_` grows
once from its `njmax` construction hint to the observed `nefc` inside
`stage_solved_into` (`native/src/drivetrain/transmission.cpp`,
"assign() grows once to the observed extent"). That grow lives in
`stage_settle` — an unmarked stage entry, not a `BIKE_NONBLOCKING`
function — so the RTSan sweep is silent on it; the asserted
`core_settle` row still measures zero.

Two warmup cycles per fixture are required: the drivetrain alternates its
tick and settlement banks, so each bank's candidate slot engages once
before steady state.

### Lazy-engagement fixes (RTSan-found)

RTSan flagged what the C++ counters could not: a `malloc` inside the
marked `TireWriter::try_compute_into` on its first call — the committed
`snapshots_`/`diagnostics_` optionals were intentionally disengaged until
the first advancing compute (the observable `tire_snapshots() == {}`
contract), so first engagement constructed the patch vector and surface
string inside the marked path. The same first-commit shape existed in the
drivetrain:

- `TireWriter` keeps the committed slots engaged for life behind a
  `committed_` flag; the accessors project `nullopt`/value from that
  flag. `patches` reserves `kMaxSnapshotPatches` and `diagnostics.surface`
  reserves the longest configured material name at construction, so the
  marked commit is assign-into-capacity only (`writers/tire.hpp:129-152`,
  `writers/tire.cpp:315-336`, `:355-372`).
- `Transmission::commit` swaps the old live `state_` into the staged bank
  slot; the first swap poisoned the slot with a capacity-0
  `coefficients`, which the next marked `make_update_into` reallocated.
  The constructor now reserves `state_.coefficients` to the topology's
  coefficient width, so both sides of the swap always hold capacity
  (`drivetrain/transmission.cpp:112-118`).
- Settlement banks were seeded with an engaged `prepared` mirror the
  first `stage_solved_into` assign destroyed (two frees per bank); the
  seed now drops `s.state.prepared` to the post-commit disengaged shape
  (`writers/drivetrain.cpp:442-467`).
- `DriveTelemetry` open-label lanes (`coasting_reason`,
  `assist_mode`) are the only telemetry strings a marked tick can grow;
  `reset()` calls `ensure_open_capacity` on the staged snapshot and every
  tick bank (`writers/drivetrain.cpp:361-373`, `:385-395`).

### Measured limitations and deviations

- **Coefficient-delta drivetrain ticks are outside the marked claim.** A
  tick whose staged transmission coefficients change rehearse-commits on
  a scratch model (`Transmission::evaluate_candidate` →
  `refresh_scratch_model` (`mj_copyModel`) + `engine::set_const`), which
  performs MuJoCo-internal allocation the counters cannot see — and which
  RTSan would flag inside a marked region. For the **geometric**
  transmission the staged coefficients *are* the geometry Jacobian, so
  any tick after qpos moves produces a delta: geometric advance ticks are
  not realtime-safe. Ideal and elastic transmissions rehearse only when a
  shift or external model mutation actually changes the wrap row. The
  `core_advance_coefficient_delta` matrix row exercises this path and
  proves the C++ surface itself allocates nothing (0/0); the exemption is
  documented at `native/src/writers/drivetrain.hpp:321-325`.
- **`op_reset`/`op_restore` size their own buffers** (bank reseed and
  snapshot capture) and are recorded, not claimed.
- **`boxed` rows** are the intentional serialization copies — the Python
  boxing surface. Tire/suspension boxed paths also free a moved-from
  temporary, visible as small `freed` counts.
- **`warm_core_zero_allocation` measures the declared warm tick**, not
  construction, reset, restore, or state capture.
- **`multipliers_` grows once per transmission** from its `njmax`
  construction hint to the observed `nefc` (models with an unlimited
  constraint arena report `njmax <= 0`, so the hint starts empty). The
  grow happens inside `stage_settle` — unmarked by design — and shows as
  the `warmup = 1` rows above.
- **`stage_settle` is deliberately unmarked**: the settlement pair
  (`stage_settle` + `commit(PreparedSettlement&)`) is part of the tick
  cycle the binding interleaves, but only the commit is `noexcept`; the
  stage may throw on policy/pending checks. Its first-engagement costs
  are warmup-absorbed and the asserted `core_settle` row is zero.
- **The first `probe_last` publication can allocate** when an open-label
  telemetry string exceeds the small-string buffer: `live_.probe_last`
  must serialize `None` until the first probe commit (wire parity), so it
  cannot be pre-engaged, and the engaging copy-construct allocates per
  over-long lane. `coasting_reason` is bounded by its closed label set
  (≤ 9 chars); `assist_mode` comes from the config — over-long mode names
  are the only trigger. All shipped/test configurations are unaffected.
- **`warmup` rows are measured, not asserted** — first-engagement sizing
  that cannot move to construction (like `multipliers_`'s data-dependent
  `nefc` extent) lands here by design.

### RTSan scope

RTSan (`-fsanitize=realtime`, brew clang only) builds the contract
executable and the extension with the same instrumentation: every
`BIKE_NONBLOCKING` entry runs under `[[clang::nonblocking]]`, so the same
31 contract cases (including the three R1 cases) verify the marked paths
against malloc-family interception, and the Python suite exercises
`step`/`forward`. As of the lazy-engagement fixes above, the RTSan
contract run reports **zero** unsafe-library-call findings across all 31
cases — including the `warm_allocation_matrix` warmup ticks, the probe
commits, and the tire first-commit path that previously tripped malloc
interception. Excluded by design: coefficient-delta drivetrain ticks
(above), unmarked stage entries (`stage_settle`, `stage_components`,
`stage_prepare` — only the `try_*` wrappers and noexcept commits are
marked), MuJoCo fatal-error formatting paths that take libc locks (the
closed solver enum is prevalidated ahead of `snprintf` for this reason —
see E1), and any callback/FFI surface outside the marked functions.

~~~bash
cmake -S native -B native/build/rtsan \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ -DNATIVE_RTSAN=ON
uv run cmake --build native/build/rtsan -j4
NATIVE_TEST_BUILD_DIR=rtsan bash tools/run_tests.sh native
~~~

**macOS launcher caveat (resolved):** `DYLD_INSERT_LIBRARIES` does not
survive a `uv run` process chain if set in the caller's environment — the
runtime lands in the `uv` launcher, not the spawned interpreter.
`tools/run_tests.sh` therefore never exports it: every instrumented leg
runs through `tools/native_runtime.py run --build <build> -- <argv>`,
which injects the recorded runtime into the child's environment at exec
time, so the variable reaches the process that actually loads the images
and `load_native()`'s `verify_sanitizer_runtime()` sees the runtime
mapped. Invoking the venv interpreter directly still works the same way:

~~~bash
DYLD_INSERT_LIBRARIES=/opt/homebrew/opt/llvm/lib/clang/23/lib/darwin/libclang_rt.rtsan_osx_dynamic.dylib \
NATIVE_TEST_BUILD_DIR=rtsan \
NATIVE_TEST_SANITIZER_RUNTIME=/opt/homebrew/opt/llvm/lib/clang/23/lib/darwin/libclang_rt.rtsan_osx_dynamic.dylib \
PYTHONPATH=tests/reference .venv/bin/python -m pytest tests/reference/test_native_tire.py -x -q
~~~

### Realtime measurement report

`tools/measure_realtime.py` has two entry points. The backend-selectable
`main()` runs the whole-boundary measurement: a fresh physical driver per
warm-up/repetition, wall clock around all `advance` calls plus the final
flush, setup and export clocked separately, and per-run JSON plus a
schema-1 `report.json` (`median_real_time_factor`, comparability blocks,
prefix/validity/error evidence, execution and artifact provenance —
imported extension path, compiled source stamp, native build context).
The profile's timestep is used unless `--dt` is given explicitly.

~~~bash
uv run --frozen --group native python tools/measure_realtime.py \
  --backend python --physics-config examples/research/viewer_physics_welded.toml \
  --track examples/research/rough_uphill_savage.toml \
  --duration 30 --warmup 1 --runs 3 --out output/rtf/python

BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" \
uv run --frozen --group native python tools/measure_realtime.py \
  --backend native --physics-config examples/research/viewer_physics_welded.toml \
  --track examples/research/rough_uphill_savage.toml \
  --duration 30 --warmup 1 --runs 3 \
  --compare output/rtf/python/report.json \
  --out output/rtf/native
~~~

Arguments: `--backend python|native` (native uses the selected artifact,
never a fallback), `--physics-config`, `--track`, `--duration`,
`--dt`, `--warmup`, `--runs`, `--record-decimation/--decimate`,
`--seed` (generated-road only), `--out` (new/empty dir), `--compare`
(earlier `report.json` for a same-prefix check). Exit code is `0` only
when every measured run is `measured_ok`; `1` means retained evidence of a
failed/invalid run; `2` is a usage/data error. Comparisons accept only
identical settings **and** identically classified measured prefixes —
failed runs are never `same_valid_prefix`, and a short natural terminal
prefix stays a flagged limitation rather than being hidden.

The historical accounted Python-only path (`measure_realtime()` /
`realtime.json`, `step_ms_*` percentiles, `--baseline` deltas with a
`comparability` block) is unchanged; it is invoked from tests, not through
the CLI above.

### Realtime gate (re-aimed at the native backend, F3 2026-10-25)

The >=1x RTF release contract (`realtime.json`, v2.05 plan: mean factor >=1
on both tracks on the dev machine) is carried by the shipped backend, not
the T63-deleted Python oracle. Two CTest nodes in the root graph —

```bash
ctest --test-dir build/native-release -R '^reference\.realtime_gate' --output-on-failure
```

(deferred Linux container lane, recorded contract only:)

~~~sh
tools/linux/container.sh ctest root-clang -R '^reference\.realtime_gate'
~~~

— spawn the produced Release `bike-measure-realtime` artifact on
`rough_uphill_savage.toml` and `rough_uphill_extreme.toml`
(`viewer_physics_welded.toml`, 20 s at dt=1.25 ms = 16,000 steps,
`--record-decimation 80`, `--warmup 1 --runs 3 --benchmark`). `--benchmark`
is the Python gate's `reference_monitor.strict=False` equivalent: the
monitor still records the documented `energy.constraint_work` baseline
violation as `prefix.first_failure` instead of stopping physics at step 4,
so the measured prefix reaches `duration_reached`. Each node asserts the
schema-1 report: exit 0 (every run `measured_ok`), `duration_reached`,
16,000/16,000 steps, `simulated_time_s >= 20 - dt`, `build_type=Release`
provenance, and `median_real_time_factor >= 1` — the contract bar,
unchanged. The threshold was not weakened: the shipped native backend
clears it by roughly an order of magnitude (darwin-arm64 medians
11.52/11.62 savage/extreme, linux-aarch64 9.10/9.09 in the pinned dev
container, both measured under ordinary dev-machine load), so the
gate still fails loudly on a real step-loop regression while no longer
pricing interpreter overhead the roadmap deliberately declined to fund
("Python-backend — oracle, not a product"). The Python pytest nodes
measuring the oracle path remain baseline-red above until T63 removes
them — superseded, not fixed. The labels are `reference;t53;runtime;slow`
so `ctest -L slow` carries them and `-LE slow` sweeps skip the measurement.

## F1 welded-plant contract re-scope (seated pedalling)

Task F1 (lane/b, `2026-10-20-cpp-linux-completion`) re-scoped five
baseline-red seated-pedalling/posture contract nodes. The approved ruling:
weld/equality closures exchange elastic spring work during sustained
pedalling (measured `energy.constraint_work` ≈1.8% steady vs the 1%
numerical bound), chronic foot/saddle cone violations were measured against
a unilateral contact cone that welded interfaces never had, and delivered
muscle power exceeded the 450 W *command* cap inside a period because
intra-period velocity drift is invisible to the tick-time clip.

### Closure channel

Constraint-power rows whose name starts with `equality:` are weld/closure
spring work and are charged to `closure_signed_j` / `closure_absolute_j`
(elastic channel) in `step_work`/`runtime_step_work`. All other rows remain
numerical constraint work and keep the existing `constraint_work_ok`
round-off bound. First-appearance ordering is preserved within each class,
mirrored between `src/bike_sim/physics/energy_ledger.py`,
`src/bike_sim/sim/ride/period_buffer.py` and
`native/src/runtime/accounting.cpp`.

### Welded attachment envelope

`AttachmentBudget` gained optional bidirectional normal bounds and flat
shear capacities. They apply only to equality-attached interfaces —
foot attachment `weld`/`spindle`, saddle `weld`/`pin` — never to flat-pad
contacts, which keep the legacy unilateral cone (`foot_min_normal_n=20`,
`mu=0.9`/`0.6`). Weld bounds may be negative (tension retention); weld shear
capacity stacks on the compressive friction-cone contribution.
`None`/`0` weld fields preserve legacy behaviour bitwise. The welded
example (`examples/research/viewer_physics_welded.toml`) declares:

| Field | Value | Derivation (≈1.25× measured extremum, rounded to capacity) |
| --- | --- | --- |
| `weld_foot_min_normal_n` | -1050 | worst foot tension −820 N |
| `weld_foot_max_normal_n` | 3000 | worst foot compression 2383 N |
| `weld_foot_shear_n` | 2250 | worst foot shear 1803 N |
| `weld_saddle_min_normal_n` | -2250 | worst saddle tension −1800 N (onset) |
| `weld_saddle_max_normal_n` | 1300 | worst saddle compression 1054 N |
| `weld_saddle_shear_n` | 1300 | worst saddle shear 1024 N |

The envelope is derived from measured extrema on the provisional strength
sheet — the seated-rider audit
(`docs/superpowers/audits/2026-10-04-seated-rider-closed-form.md`, status
NOT ACCEPTED) records the curves as recalled/`verified=false` with no
internet verification — so these are defect-signal bounds on this plant,
not rated hardware specifications.

### Delivered-power monitor budgets

`active_positive_power_limit_w` (450 W) and `joint_power_limit_w` remain
*command/plant* caps applied by the spindle clip; nothing about solved
truth is clipped. New monitor-side budgets —
`delivered_positive_power_budget_w` (750 W) and
`delivered_joint_power_budget_w` (375 W) — bound what diagnostics may
*declare delivered*, derived at ≈1.25× the measured extrema (596 W total,
289 W per joint). Effective properties
(`delivered_positive_budget_w`, `delivered_joint_budget_w`) fall back to
the capability caps when unset, so un-declared configs keep the legacy
contract bitwise. `rider_effort` and period diagnostics verdict on the
effective delivered budgets.

### Sustained crank delivery ceiling

`test_strict_flat_cycle_delivers_and_stays_within_budgets` asserts demand
tracking only up to the plant's measured sustained delivery ceiling
(`delivery_ceiling_nm = 26` in the test, with derivation comment). Under
the provisional strength sheet plus the 25 Nm weld-servo hold limit the
measured late-run mean crank torque plateaus at 17.8/23.1/25.7 Nm for
20/40/60 Nm demands — requests above the ceiling are intents, the same
model-delivery gap as the power re-band.

### Linux-aarch64 pulse disposition (D16)

On linux-aarch64 `test_pulse_redistributes_load_through_inertia_only` trips
the strict monitor's `energy.constraint_work` verdict at 0.005 s: the
non-equality constraint rows integrate a sustained ~3–8x larger absolute
residual than darwin-arm64 (measured 5.9% → 2.3% vs 0.9% → 0.7% of source
positive work across the first ten closes of the settle transient). Weld and
equality work is charged to the closure channel and never enters this
verdict on either platform, so the residual is solver-internal
joint/tendon-limit bookkeeping — a platform-dependent numerical-work rate,
not a physical contract change. The episode therefore runs the reference
monitor in `--check-policy diagnostic` on linux only; every posture, grip
and delivered-power assertion still applies, and the test asserts the
recorded first failure stays exactly `('energy.constraint_work',)`, so any
new or different verdict still fails. The identical `constraint_work_ok`
verdict keeps full strict linux coverage through the four
`test_seated_pedaling_cycle.py` welded-plant nodes (all measured green
strict on linux-aarch64), which is the named D14 coverage for dropping the
strict record here.

### Parity caveat

`native_runtime_accounting/strict_rejection_publishes_the_whole_period_before_raising`
compares interval structure and violation evidence only. Payload leaves
(sensors, weld forces, rider torques, bookkeeping values) are not compared
because the Python and native backends drift on this weld-heavy plant from
interval 0 — native fires `rider_strength.rider_torso_hinge` at t=0.2425
where Python does not, and `frame_specific_force_body_mps2` diverges on
unmodified HEAD. That is a pre-existing parity gap owned by the ADR-0003
golden gate, not an F1 regression.

Focused checks:

~~~bash
uv run pytest tests/reference/test_attachment_budget.py \
  tests/reference/test_articulated_config.py \
  tests/reference/test_period_buffer.py -q
BIKE_NATIVE_BUILD_PATH=$PWD/native/build uv run pytest \
  tests/reference/test_native_runtime_accounting.py -q
~~~
