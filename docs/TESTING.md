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
