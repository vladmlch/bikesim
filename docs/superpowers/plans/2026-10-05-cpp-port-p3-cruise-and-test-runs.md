# C++ Port P3: Cruise Controller and Focused Test Runs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Port the independently testable cruise controller from the P3 backlog and provide explicit, fast test commands for everyday native work.

**Architecture:** Continue the accepted ADR 0001 architecture and the P2 Stepper/config bridge. Cruise owns its PI state inside C++; tests compare sequential calls against the Python controller without running long physical episodes. A small shell entry point selects existing pytest suites and requires an importable C++ extension for native checks.

**Tech Stack:** C++23, nanobind, MuJoCo 3.12, Apple clang primary, GCC frontend sweep; Python >=3.13 through uv, pytest, numpy.

**Spec:** `docs/adr/0001-native-port-mujoco-core.md`; P3 deferred surface in `docs/superpowers/plans/2026-10-05-cpp-port-p2-force-writers.md`; user instructions to optimize test runs and continue execution.

## Global Constraints

- Python commands always run through `uv run`.
- Preserve all existing Python runtime behavior in `src/bike_sim/`; Python remains the numerical oracle.
- Native uses C++23, the existing warnings-as-errors and `-ffp-contract=off` flags, the same MuJoCo dylib, and the existing GCC frontend sweep.
- Per-call numerical comparisons use raw float64 bytes, including signed zero. No tolerance loosening for this scalar PI port.
- Cruise receives contact booleans, not Python contact objects; no Python callbacks or retained Python objects in the native controller.
- Config schema remains 1; `cruise` is an optional section. Missing sections disable the corresponding native controller, as for P2 writers.
- `Stepper.set_state` restores mjData only; controller state has a separate explicit restore API and survives mjData restores.
- Iteration runs the changed module; after each task run its relevant suite. Run `not slow` once after the final code changes. Do not repeat the eight-minute full suite: the prior full run on a216f36 produced 520 passes and 24 failures, and the user requested optimized test runs. Full physics and realtime checks remain available explicitly.
- Reviewers use recorded test evidence and rerun only a focused check needed to resolve a concrete doubt.

## File Responsibilities

`tools/run_tests.sh` selects quick/native/full checks and forwards pytest arguments. `docs/TESTING.md` records these modes and build prerequisites. `native/src/writers/cruise.{hpp,cpp}` implements PI state and scalar operations. The existing config, Stepper, binding and config projection expose that controller. `tests/reference/test_native_cruise.py` exercises numerical behavior, state continuity and boundary failures.

### Task 1: Explicit quick, native and full test entry points

**Files:**
- Create: `tools/run_tests.sh`
- Create: `docs/TESTING.md`

**Interfaces:**
- Consumes: pytest's existing `slow` marker, test files in `tests/reference/test_native_*.py` and `tests/reference/test_golden_episode*.py`, native extension in `native/build`.
- Produces: `bash tools/run_tests.sh [quick|native|full] [pytest arguments...]`, with `quick` as the no-argument default. Help succeeds; unknown profiles return exit 2. Native requires a successfully imported `bike_native` before pytest starts. Every test command adds `-q --durations=10` and preserves pytest's exit code.

- [x] **Step 1: Implement the small launcher**

Use `set -euo pipefail`, resolve the repository root relative to `${BASH_SOURCE[0]}`, and `cd` there. Keep the paths quoted. Select mode with a `case` and reject unknown modes. Use an array for pytest arguments and `exec uv run python -m pytest` for the final command. For native mode, prepend the absolute `native/build` path to `PYTHONPATH` preserving any existing value; use `uv run python -c 'import bike_native; print(bike_native.__file__)'` as the preflight, then select the native/golden files via bash arrays. Modes:

```bash
# quick
exec uv run python -m pytest -q --durations=10 -m 'not slow' "$@"
# native (after the required import preflight)
exec uv run python -m pytest -q --durations=10 \
  tests/reference/test_native_*.py tests/reference/test_golden_episode*.py "$@"
# full
exec uv run python -m pytest -q --durations=10 "$@"
```

Do not add xdist, default cache-based test selection, or change pytest's own defaults. The realtime tests measure machine speed and must have a serial mode.

- [x] **Step 2: Write usage documentation**

Document the three commands, targeted node/file invocation with `uv run python -m pytest ...`, when to use each profile, native build prerequisite `uv run cmake --build native/build -j4`, import failure behavior, and why full realtime episodes are explicit. Distinguish the known full-suite failures from checks that are green. Record prior timings as measurements rather than promises: approximately 2 s for `not slow`, 6 s for the six-module P2 focused run, and 478 s for the full suite on a216f36.

- [x] **Step 3: Verify launcher behavior cheaply**

Run `bash -n tools/run_tests.sh`, help, and an invalid profile (expect exit 2). Use `bash tools/run_tests.sh native --collect-only` to verify native import, selected files, and forwarding. Use `bash tools/run_tests.sh full --collect-only` to verify that full includes slow tests without running them. Record actual selected counts. Do not add tests that merely reproduce the shell case logic.

- [x] **Step 4: Self-review and commit**

Review quoting, native import failure propagation, default selector, user arguments and exit codes. Commit only `tools/run_tests.sh` and `docs/TESTING.md` with message `chore(test): add focused native and quick test profiles`.

### Task 2: Native stateful cruise controller with bitwise oracle

**Files:**
- Create: `native/src/writers/cruise.hpp`
- Create: `native/src/writers/cruise.cpp`
- Create: `tests/reference/test_native_cruise.py`
- Modify: `native/src/config.hpp`
- Modify: `native/src/stepper.hpp`
- Modify: `native/src/stepper.cpp`
- Modify: `native/src/binding.cpp`
- Modify: `native/CMakeLists.txt`
- Modify: `tools/native_config.py`
- Modify: `docs/TESTING.md` (new focused command)

**Interfaces:**
- Consumes: `src/bike_sim/sim/ride/cruise.py::CruiseController` as the unchanged scalar oracle; current mjData `qvel[root_x]` and `mjModel.opt.timestep`.
- Config: `{'schema': 1, 'cruise': {'target_speed_kmh': float, 'kp_nm_per_mps': float, 'ki_nm_per_mps_s': float, 'torque_ceiling_nm': float}}`.
- Produces: `Stepper.cruise_compute(rear_in_contact: bool, traction_limited: bool = False, controller_grounded: bool | None = None) -> float`; `cruise_reset() -> None`; `cruise_set_target_speed(value_kmh: float) -> None`; `cruise_set_assist_compensation(support_factor: float) -> float`; `cruise_state() -> dict`; `set_cruise_state(state: dict) -> None`.
- State dict has exactly five numeric/bool fields: `target_speed_mps`, `integral_mps_s`, `torque_nm`, `engaged`, `gain_scale`. Store target in m/s to avoid extra km/h roundtrip rounding. Restoration validates the complete candidate before mutation. All doubles must be finite; target must be within `[15.0 / 3.6, 45.0 / 3.6]`; gain scale must be positive. Torque and integral may be arbitrary finite recorded values: do not silently clamp restored state.
- Config construction validates finite positive gains and ceiling, finite target within `[15,45]`, and resolves a scalar slide/hinge `root_x` joint. Public target/support setters reject nonfinite values before mutation. Negative finite support remains accepted and maps to scale 1, matching Python. Reject an assist value whose resulting scale or scaled integral gain is zero (division by zero would otherwise occur); rejection preserves state.
- `project(env)` adds cruise only when `getattr(sim, 'cruise', None)` is present. Emit the four configuration attributes, not mutable PI state; existing duck-typed fixture sims without cruise remain supported.

- [x] **Step 1: Add failing sequential bitwise tests**

Create a tiny local MuJoCo model with one `root_x` slide joint, inertia, timestep 0.0005 and no simulation loop. Save it as MJB under `tmp_path`; construct Python `CruiseController(model)` and native Stepper with only cruise config. This avoids research-environment setup and slow marks. Import `bike_native` using the existing native test convention; compare numeric state/output using `assert_bitwise_equal` from `_bits.py`.

Core oracle loop:

```python
from types import SimpleNamespace

for speed, grounded, limited, override in cases:
    data.qvel[root_adr] = speed
    native.set_state(data.qpos, data.qvel, data.act,
                     data.qacc_warmstart, data.time)
    expected = oracle.compute(model, data,
        SimpleNamespace(rear_in_contact=grounded), limited,
        controller_grounded=override)
    actual = native.cruise_compute(grounded, limited, override)
    assert_bitwise_equal(np.array([actual]), np.array([expected]))
    state = native.cruise_state()
    assert_bitwise_equal(np.array([state['target_speed_mps'],
        state['integral_mps_s'], state['torque_nm'], state['gain_scale']]),
        np.array([oracle.target_speed_mps, oracle.integral_mps_s,
                  oracle.torque_nm, oracle.gain_scale]))
    assert state['engaged'] == oracle.engaged
```

Use sequences containing accelerating/decelerating speed, both signs of saturation, interior integration, airborne transitions, traction-limited integration freeze, and both grounded override directions. Seed restored integral values to exercise unwinding from positive/negative saturation and exact saturation boundaries. Include signed-zero input/output where Python produces it. Add target changes at 15,25,45 km/h, compensation at negative,zero,positive support, reset preserving target and gain scale, snapshot/restore replay, and mjData restore preserving controller state. Capture Python outputs before the native calls; never compute expected values by reusing native results.

Add boundary tests for missing config (every API raises RuntimeError), missing required keys, bad schema, invalid gain/target/ceiling, model missing `root_x`, wrong joint type, nonfinite setter input, invalid state fields and support underflow. Check old state remains unchanged on rejected setter/restore. Add a projection check using the existing small seated fixture (`test_native_suspension.py` construction) or a duck-typed env that feeds `project`; also confirm absent cruise leaves existing projection valid.

- [x] **Step 2: Run RED**

Run `uv run python -m pytest tests/reference/test_native_cruise.py -q` against the current built extension. Expect missing `cruise_compute`/state API failures. Record the exact result in the task report.

- [x] **Step 3: Implement scalar native controller and bridges**

Implement a small typed `CruiseConfig` and `CruiseState`, and an owned `CruiseWriter`. Port Python's operation order literally: choose engaged gate; error; early airborne return; scaled kp/ki; proportional and demand; the `abs(demand) >= ceiling` plus same-sign saturation test; clamped integral update when unsaturated and not traction limited; final clamped demand. Clamp must preserve Python's `max(-limit, min(limit, value))` signed-zero behavior; avoid `std::clamp` if its equal-value tie behavior differs. Do not add an automatic cruise actuator write: that belongs to the later runtime orchestration task.

Expose a std::optional<bool> override at the FFI edge (include `nanobind/stl/optional.h`). Return owning state dict values. Parse config once and keep no Python references. Controller compute takes `const mjData*` and the stored timestep/root dof, never calls Python. Update the existing NativeConfig optional sections and constructor cleanup path, Stepper forward declarations/owned member, bindings and CMake source list. Document that mjData restore does not reset PI state.

- [x] **Step 4: Run GREEN and native safety checks**

Build with `uv run cmake --build native/build -j4`; run the focused cruise module. Run the existing config-related suspension/rider modules once alongside cruise to check projection compatibility. Run `uv run cmake --build native/build --target check_frontends` to verify the new TU under GCC as well as clang. Build the existing ASan configuration with `uv run cmake --build native/build/asan -j4` and run the focused cruise module with the ASan dylib injected using the established Xcode clang 21 runtime. Force the ASan extension via a Python preimport path/assertion so the test's own normal-build sys.path insertion cannot substitute it. Record commands, results, and any diagnostics. Do not rerun unrelated eight-minute physical episodes.

- [x] **Step 5: Self-review and commit**

Review Python operation ordering, validation before mutation, optional override semantics, no callback ownership, config compatibility and signed-zero comparisons. Commit the Task 2 files with message `feat(native): port stateful cruise controller with bitwise oracle`.

## Completion and Remaining Roadmap

After task review, run `bash tools/run_tests.sh quick` once on final code and use the recorded focused/sanitizer evidence. Whole-change review covers this plan's commits, not the already-reviewed P2 range. Preserve the worktree and branch; no merge/push is authorized. The known full-suite failures prevent claiming the entire project is green.

This is the first independently testable P3 increment, not completion of P3/P4. Remaining ordered work: drivetrain state/capture and component ports; rider contact state and model mutation; rider controller allocation and held commands; native apply-forces/step-loop; intent/telemetry/recorder seam; viewer integration and standalone batch. Each must retain the ADR's layer-1/2/3 gates. The current viewer only exercises native MuJoCo stepping; full native force/controller execution remains pending those increments.
