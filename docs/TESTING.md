# Running tests

Run these commands from the repository root:

```bash
bash tools/run_tests.sh          # defaults to quick
bash tools/run_tests.sh quick    # excludes tests marked slow
bash tools/run_tests.sh native   # native extension and golden episode checks
bash tools/run_tests.sh full     # includes slow physics and realtime episodes
```

Use `quick` for ordinary Python edits and the final fast regression check. It
excludes direct native extension tests but keeps native-check controller tests.
Use `native` for C++ implementation, Python bridge, and verification-tool work.
Use `full` for complete physical episodes or realtime acceptance. Both `native`
and `full` configure/build the selected native directory, run every configured
sweep plus the V2 header/diagnostic/context controls and V4 structural contract
checks, run CTest, and then invoke pytest once. The launcher fails if pytest
selection leaves zero direct native extension test items. It resolves its
repository root, so it also works when
invoked from another directory.

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

`native` and `full` configure and build the selected directory before checking
or importing the extension. For a standalone native test command, first build
the desired directory:

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

Set `NATIVE_TEST_BUILD_PATH` to an absolute build directory. The legacy
`NATIVE_TEST_BUILD_DIR` selector still accepts `''`, `asan`, `rtsan`, and
`coverage`; simultaneous selectors must resolve to the same directory. The
shared `tests/reference/native_loader.py` inserts that directory, requires one
`bike_native` extension artifact there, and verifies the imported module's
resolved path. Missing/ambiguous artifacts and an earlier import from another
build fail immediately. Native/full print the selected extension path and the
number of direct native extension test items collected after pytest
deselection. Tool-controller and loader test items do not satisfy this gate.

`native_contract_tests` is registered with CTest and links a PIC Python-free
drivetrain object shared with `bike_native`. Its `require` helper throws even
when `NDEBUG` is defined. The loader suite invokes its deliberate failure mode
against the selected build and requires a nonzero exit. To check both Debug and
Release, build each directory and run the same focused control against each:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/v3-debug" uv run python -m pytest tests/reference/test_native_loader.py::test_native_contract_require_control_fails_in_selected_build -q
NATIVE_TEST_BUILD_PATH="$PWD/native/build/v3-release" uv run python -m pytest tests/reference/test_native_loader.py::test_native_contract_require_control_fails_in_selected_build -q
```

The contract source is included in the CMake analysis manifest.

For a direct native test invocation, select the build in the same way:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" uv run python -m pytest tests/reference/test_native_suspension.py -q
```

## Static analysis and sanitizer builds

The `native` and `full` profiles share the full verification preflight:
configure, build, every CMake sweep, the standalone header/diagnostic/context
controls, CTest, and an exact selected-extension import before pytest. Each
configured sweep is also a standalone CMake target. CMake writes
`<selected-build>/native_sources.json` from the first-party sources attached to
configured targets and `<selected-build>/native_check_context.json` with the
compiler, SDK, Python, dependency paths, target compile contexts, and tool overrides.
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

V4 adds `check_native_contracts` to native/full preflight. It checks the CMake
source manifest against the compilation database, strict ISO C++23 and the
configured hardening mode, the `.clang-tidy` Move/header policy, selected-loader
use by direct native tests, and the contract source's manifest and CTest
registration. It asks the contract executable to list and run each registered
case. These are structural and focused behavioral checks; they do not claim to
prove exception safety for every native operation. Engine-call wrapper checks
and benchmark membership remain marked pending until their planned declarations
and `native_bench` target exist.

After the selected extension has been loaded and recorded, the target can be
rerun directly for a regular build:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" \
NATIVE_TEST_PROVENANCE_PATH="$PWD/native/build/native_test_provenance.json" \
PYTHONPATH="$PWD/tests/reference" \
  uv run python -c 'from native_loader import load_native; load_native()'
uv run cmake --build native/build --target check_native_contracts
```

For ASan or RTSan builds, use `tools/run_tests.sh native`; it resolves and
passes the selected sanitizer runtime to the contract executable's child
processes.

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

`NATIVE_TEST_BUILD_PATH` may select any absolute configured build directory.
`NATIVE_TEST_BUILD_DIR` remains a compatibility selector for `''`, `asan`,
`coverage`, or `rtsan` under `native/build`. The shared loader checks artifact
cardinality and import provenance in both profiles.

Sanitizer build:

```bash
cmake -S native -B native/build/asan -DNATIVE_SANITIZE=ON
uv run cmake --build native/build/asan -j4
NATIVE_TEST_BUILD_DIR=asan bash tools/run_tests.sh native
```

The sanitizer set is `address,undefined,local-bounds,float-cast-overflow`
with `-fno-sanitize-recover=all`. The launcher reads the compiler from the
selected build's cache, resolves its ASan runtime, then injects it into CTest
and the Python process after `uv` starts. It also sets `PYTHONMALLOC=malloc`
and the documented ASan options in those child processes. Before importing the
extension, the loader checks the selected runtime's actual dyld image list (or
`/proc/self/maps` on Linux); an environment variable alone does not count as
runtime evidence.

RTSan (`-fsanitize=realtime`) verifies the marked hot path never allocates or
locks — the mechanical half of the allocation-free tick requirement. It needs
the brew clang (Apple clang rejects the flag):

```bash
cmake -S native -B native/build/rtsan \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ -DNATIVE_RTSAN=ON
uv run cmake --build native/build/rtsan -j4
NATIVE_TEST_BUILD_DIR=rtsan bash tools/run_tests.sh native
```

`Stepper::step`/`forward` carry `BIKE_NONBLOCKING` (rtsan.hpp); the full
native suite runs under RTSan with zero reports — mj_step/mj_forward are
verified allocation-free. Marked functions that start allocating abort the
test with a report.

Coverage:

```bash
tools/native_coverage.sh
tools/native_coverage.sh /tmp/native-coverage-build
```

The wrapper configures coverage for the selected build, runs the native profile
against that absolute build, and writes artifacts under a unique
`<build>/native_coverage_runs/run.*` directory. It merges only that run's
nonempty `.profraw` files and reports against the exact extension recorded by
the loader. `coverage_provenance.json` records the selected build, imported and
reported object, collected native test count, profile count, LLVM tool versions,
merged profile, and text/JSON report paths. Logs remain in the run directory
when configuration, tests, or reporting fails. On macOS the wrapper resolves
the matching Xcode tools through `xcrun`; set `LLVM_PROFDATA` and `LLVM_COV` to
executable paths to select another compatible pair.

Coverage finalization is also available directly for an existing run:

```bash
uv run python tools/native_coverage.py \
  --build native/build/coverage \
  --run-dir native/build/coverage/native_coverage_runs/run.ID \
  --llvm-profdata "$(xcrun --find llvm-profdata)" \
  --llvm-cov "$(xcrun --find llvm-cov)"
```

V4 verification snapshot (2026-10-06): the regular native profile passed with
782 tests and 15 warnings; `full --collect-only` collected 1,253 tests without
executing them. The ASan native preflight and contract target also passed. The
custom build at `native/build/v4-custom-coverage`, run `run.wazeIy`, merged 17
profiles from 690 native test items. Its imported extension, report object, and
object path were identical. Apple LLVM 21.0.0 reported 91.63% line and 78.00%
branch coverage for that run. The real one-file control reported 3/4 lines and
1/2 branches covered, exercising both a covered and an uncovered path. These
are observed counts, not minimum coverage thresholds. Engine-call wrapper
checks remain pending on E1; benchmark manifest membership remains pending
until R2 adds the `native_bench` target. The slow `full` profile was not run.

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

The 24 failing node IDs, identical in the baseline and current full run, are:

- `tests/reference/test_closed_form_rider.py::test_flat_launch_delivers_crank_torque`
- `tests/reference/test_coupled_rider_task.py::test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step`
- `tests/reference/test_coupled_rider_task.py::test_tissue_damping_does_not_eat_the_muscle_budget`
- `tests/reference/test_joint_strength.py::test_stepped_effort_respects_strength_and_the_power_budget`
- `tests/reference/test_period_buffer.py::test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms`
- `tests/reference/test_period_buffer.py::test_runtime_batch_matches_preserved_scalar_step_and_flush`
- `tests/reference/test_planar_arms.py::test_compiled_topology_has_two_arms_and_two_grip_sites`
- `tests/reference/test_realistic_pedelec_acceptance.py::test_flat_reaches_25_kmh_within_10_s`
- `tests/reference/test_realistic_pedelec_acceptance.py::test_motor_follows_the_rider_through_a_shift_without_an_extra_cut`
- `tests/reference/test_realistic_pedelec_acceptance.py::test_fifteen_percent_climb_holds_12_kmh`
- `tests/reference/test_realistic_pedelec_acceptance.py::test_savage_is_ridden_to_the_end`
- `tests/reference/test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds[rough_uphill_savage.toml]`
- `tests/reference/test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds[rough_uphill_extreme.toml]`
- `tests/reference/test_rider_allocation.py::test_healthy_step_allocates_a_feasible_command`
- `tests/reference/test_rider_allocation.py::test_allocated_wrenches_close_the_rider_dynamics`
- `tests/reference/test_rider_allocation.py::test_infeasible_intent_keeps_the_physical_bounds`
- `tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[20.0]`
- `tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[40.0]`
- `tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[60.0]`
- `tests/reference/test_seated_pedaling_cycle.py::test_free_coast_is_an_intent_not_a_crank_lock`
- `tests/reference/test_seated_posture_program.py::test_pulse_redistributes_load_through_inertia_only`
- `tests/test_rider_welds.py::test_held_rider_weight_is_carried_by_the_connects`
- `tests/test_rider_welds.py::test_contact_diagnostics_carry_the_solved_weight`
- `tests/test_rider_welds.py::test_steep_grade_saddle_shear_exceeds_friction`

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
failure inside that loader. This S5 cleanup requirement remains open pending
an owned loader API or an approved scope change; process survival does not
prove leak-free loading.

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
