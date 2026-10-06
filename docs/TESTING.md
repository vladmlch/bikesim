# Running tests

Run these commands from the repository root:

```bash
bash tools/run_tests.sh          # defaults to quick
bash tools/run_tests.sh quick    # excludes tests marked slow
bash tools/run_tests.sh native   # native extension and golden episode checks
bash tools/run_tests.sh full     # includes slow physics and realtime episodes
```

Use `quick` for ordinary edits and the final fast regression check. Use `native`
for work on the C++ implementation and its Python bridge. Use `full` explicitly
when evaluating complete physical episodes or realtime acceptance. The launcher
resolves its repository root, so it also works when invoked from another directory.

All profiles run serially with `-q --durations=10`, forward additional arguments
to pytest, and return pytest's exit status. For example:

```bash
bash tools/run_tests.sh native --collect-only
bash tools/run_tests.sh quick -k config
bash tools/run_tests.sh full --collect-only
bash tools/run_tests.sh --help
```

An unknown profile returns exit status 2. For a targeted file or test node, invoke
pytest directly; the native profile always selects all native and golden files:

```bash
uv run python -m pytest tests/reference/test_native_suspension.py -q --durations=10
uv run python -m pytest tests/reference/test_native_suspension.py::test_suspension_components_bitwise -q
uv run python -m pytest tests/reference/test_native_cruise.py -q
```

## Native prerequisite

Build the extension in an already configured `native/build` directory before
running native checks:

```bash
uv run cmake --build native/build -j4
```

The cruise module uses a tiny local model and compares each controller call and
PI state against the unchanged Python controller by raw float64 bytes. It does
not run physical episodes. `Stepper.set_state` restores only mjData; cruise PI
state survives it. Use `cruise_state` and `set_cruise_state` to replay controller
state explicitly, and `cruise_reset` to clear its integral, torque and engaged
flag while preserving the target and assist scale. Cruise computes a scalar
torque; runtime actuator writes and the force loop are later port increments.

The native profile prepends the absolute `native/build` directory to `PYTHONPATH`,
preserving an existing value. It imports `bike_native` and prints the imported
module's path before starting pytest. If import fails, the launcher exits with
the import command's failure status and pytest does not start. This makes a
missing or broken extension visible before native tests can be skipped.

For a direct native test invocation, set the same import path:

```bash
PYTHONPATH="$PWD/native/build${PYTHONPATH:+:$PYTHONPATH}" uv run python -m pytest tests/reference/test_native_suspension.py -q
```

## Static analysis and sanitizer builds

The `native` profile runs the full verification chain: build, then every
sweep, then tests — a green run cannot rest on a stale `.so`. Each sweep is
also a standalone CMake target. CMake writes `native/build/native_sources.json`
from the first-party sources attached to configured targets and
`native/build/native_check_context.json` with the compiler, SDK, Python,
dependency paths, target compile contexts, and configured tool overrides.
`tools/native_checks.py` checks those manifests against
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

```bash
uv run cmake --build native/build --target check_frontends   # gcc -fsyntax-only + -fanalyzer (second frontend)
uv run cmake --build native/build --target check_tidy        # clang-tidy, checks in native/.clang-tidy
uv run cmake --build native/build --target check_analyzer    # clang --analyze, stable+optin block; alpha report-only
uv run cmake --build native/build --target check_cppcheck    # cppcheck third engine
uv run cmake --build native/build --target check_odr         # g++ -flto -Wodr merge — cross-TU type conflicts
uv run cmake --build native/build --target check_scripts     # shellcheck over tools/*.sh
```

The controller can also be invoked directly, for example
`uv run python tools/native_checks.py --kind tidy --build native/build`.
The same configured source manifest is used by each standalone target and by
the native profile; wrappers do not glob `native/src` or hide process status.

V2 adds three focused controls:

```bash
uv run python tools/native_checks.py --kind headers --build native/build
uv run python tools/native_checks.py --kind diagnostic-controls --build native/build
uv run python tools/native_checks.py --kind context --build native/build
```

The `headers` control compiles each first-party header by itself with its
configured target flags. `diagnostic-controls` verifies that the selected
compiler still reports discarded `nodiscard` values, bitwise logical tests,
extra semicolons, signed bounds, and first-party use after move. It also
verifies that libc++ hardening configuration rejects a missing mode macro.
`context` tests explicit, missing, and default SDK selection, then records
container sizes, active bounds assertions, and timings for FAST and EXTENSIVE
libc++ modes in Debug, Release, and the empty build type.

First-party targets use strict ISO C++23. The `.clang-tidy` header filter is
checkout independent and covers `native/src`, `native/tests`, and
`tools/proto_native_bench`. First-party Move findings remain blocking. The
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

```bash
uv run cmake -S native -B native/build-fast -DBIKE_LIBCPP_HARDENING=FAST
```

The timing control uses five samples of three million indexed reads per build
type and hardening mode. It records representative container sizes, whether
bounds assertions fired, each timing sample, and the median for Debug, Release,
and the empty build type. Read the current compiler, SDK, run ID, and measured
values in `native/build/native_check_summaries/context.json`; these local
measurements are evidence for the selected toolchain, not a performance floor.

`check_analyzer` and `check_tidy` use the keg-only brew LLVM
(`/opt/homebrew/opt/llvm/bin`) — Apple clang lacks several checkers. The
analyzer's alpha pass prints findings without failing; they are unstable by
upstream contract, so review them when they appear instead of gating on them.
`check_odr` compiles every TU with `-flto` and merges with `-Wodr` — the only
phase that can see a type defined differently across translation units.

`NATIVE_TEST_BUILD_DIR` selects the extension subdirectory the native test
files import from `native/build/`: empty (default build), `asan`, `coverage`,
or `rtsan`. There is no fallback — the imported module's path is asserted.

Sanitizer build:

```bash
cmake -S native -B native/build/asan -DNATIVE_SANITIZE=ON
uv run cmake --build native/build/asan -j4
```

The sanitizer set is `address,undefined,local-bounds,float-cast-overflow`
with `-fno-sanitize-recover=all`. Run the native suite against it with the
toolchain ASan dylib injected — invoke `.venv/bin/python` directly: `uv run`
drops `DYLD_INSERT_LIBRARIES` before exec and the interceptors fail to install.

```bash
ASAN_LIB="$(clang -print-file-name=libclang_rt.asan_osx_dynamic.dylib)"
NATIVE_TEST_BUILD_DIR=asan ASAN_OPTIONS=detect_leaks=0:detect_stack_use_after_return=1 \
  DYLD_INSERT_LIBRARIES="$ASAN_LIB" PYTHONMALLOC=malloc \
  .venv/bin/python -m pytest tests/reference/test_native_*.py -q
```

RTSan (`-fsanitize=realtime`) verifies the marked hot path never allocates or
locks — the mechanical half of the allocation-free tick requirement. It needs
the brew clang (Apple clang rejects the flag):

```bash
cmake -S native -B native/build/rtsan \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ -DNATIVE_RTSAN=ON
uv run cmake --build native/build/rtsan -j4
RTSAN_LIB="$(/opt/homebrew/opt/llvm/bin/clang -print-file-name=libclang_rt.rtsan_osx_dynamic.dylib)"
NATIVE_TEST_BUILD_DIR=rtsan DYLD_INSERT_LIBRARIES="$RTSAN_LIB" \
  .venv/bin/python -m pytest tests/reference/test_native_*.py -q
```

`Stepper::step`/`forward` carry `BIKE_NONBLOCKING` (rtsan.hpp); the full
native suite runs under RTSan with zero reports — mj_step/mj_forward are
verified allocation-free. Marked functions that start allocating abort the
test with a report.

Coverage:

```bash
cmake -S native -B native/build/coverage -DNATIVE_COVERAGE=ON
tools/native_coverage.sh            # builds, tests, prints llvm-cov summary
```

The script merges profraw into `native/build/coverage/coverage.profdata`;
`xcrun llvm-cov show` on the .so with that profdata gives per-line detail.
Xcode's llvm-cov is used because it matches the producing Apple clang.

The launcher sets `PYTHONDEVMODE=1` everywhere and `MallocScribble`,
`MallocPreScribble`, `MallocGuardEdges` on quick/native (not `full`, which
measures realtime). With `NATIVE_TEST_BUILD_DIR=asan` it also exports
`PYTHONMALLOC=malloc` and the documented `ASAN_OPTIONS` additions, so the
`native` profile alone is a valid asan run.

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

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drive_policies.py tests/reference/test_native_cruise.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
```

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

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drivetrain.py tests/reference/test_native_drive_policies.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build/asan -j4
```

Run the same two test modules against `native/build/asan`, with
`NATIVE_TEST_BUILD_DIR=asan`, `ASAN_OPTIONS=detect_leaks=0`, and the toolchain
ASan dylib in `DYLD_INSERT_LIBRARIES` passed through `uv run env ... python`.
Prepend that build directory before importing and assert the extension path.
The policy and drivetrain tests accept exactly the regular or explicitly
selected sanitizer build; neither permits an arbitrary extension fallback.

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

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_rider_contact_math.py tests/reference/test_native_tire.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
```

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

CPython 3.14's `math.hypot` uses exact products, compensated accumulation, and
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
