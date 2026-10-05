# Native drivetrain P3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the physical drivetrain's policies, force components, transmission constraints, and solved-actuation energy accounting into independently callable C++ code.

**Architecture:** Typed, Python-independent policy classes own scalar state. A model-owned `DrivetrainWriter` composes them with MuJoCo transmission helpers; nanobind converts plain dictionaries only at the public boundary. This increment supplies the drivetrain required by the accepted P3 port; full intent scheduling and viewer integration remain P4 work.

**Tech Stack:** C++23, nanobind, MuJoCo 3.12.0, NumPy/Accelerate numerical oracle, uv/pytest, clang and GCC, ASan/UBSan.

**Spec:** `docs/adr/0001-native-port-mujoco-core.md`, P3 drivetrain. Existing Python physical drivetrain is the behavioral reference, especially `src/bike_sim/sim/ride/drivetrain_forces.py`.

## Global Constraints

- C++23; retain the strict warning flags and `-ffp-contract=off` already applied to the native target.
- No Python callbacks, retained Python config objects, or Python computations in the typed policy or writer core.
- Config contains resolved numeric runtime fields; preserve schema 1 and optional sections, including construction without a drive section.
- Preserve Python expression order, signed zeros, sentinels, and update order. Use `writers/pyfloat.hpp` for Python exponentiation and matching Accelerate operations for NumPy dot products. Compare floating results bytewise on the supported platform; never relax a mismatch without diagnosis and a recorded decision.
- Python commands use uv. Use `UV_CACHE_DIR=/tmp/cpp-port-p3-uv` in this environment.
- Tests use unchanged Python algorithms as independent oracles and real compiled MuJoCo models. Prove native code is imported from `native/build` or the sanitizer build. Do not replace Python implementations or silently skip expected native behavior.
- Run focused tests while iterating, the affected native tests after each task, and one quick profile after all tasks. Do not repeat the known eight-minute full suite in this increment; record this acceptance limitation.
- Preserve the existing dirty `CLAUDE.md` and unrelated untracked files. Make scoped commits. No merge or push.

---

### Task 1: Scalar drive policies and projection

**Files:**
- Create: `native/src/drivetrain/policy_config.hpp` — typed resolved configuration and state/result structs, no Python includes.
- Create: `native/src/drivetrain/pedaling.hpp`, `pedaling.cpp` — `PedalingPolicy`, `PedalingState`, and `human_crank_torque`.
- Create: `native/src/drivetrain/shifting.hpp`, `shifting.cpp` — `CadenceShifter` and its diagnostics.
- Create: `native/src/drivetrain/motor.hpp`, `motor.cpp` — assist controller, battery, electrical-power/energy-cap helpers.
- Create: `native/src/drivetrain/freehub.hpp`, `freehub.cpp` — elastic freehub ratchet.
- Create: `native/src/drivetrain/policy_binding.hpp`, `policy_binding.cpp` — config parsing, state conversion, diagnostic FFI class and binding registration.
- Modify: `native/src/binding.cpp`, `native/CMakeLists.txt` — register/build the new implementation.
- Modify: `tools/native_config.py` — `project_drive_policies(drive)` helper, independently callable; leave `project()` drive emission to Task 2.
- Test: `tests/reference/test_native_drive_policies.py`.
- Modify: `docs/TESTING.md` — policy conformance coverage and focused command.

**Interfaces:**

`project_drive_policies(drive)` consumes a `DrivetrainForceApplier` and emits a plain dictionary with:

- `gearing`: `front_teeth`, `rear_teeth`, `chain_pitch_m`.
- `pedaling`: the eight fields actually read by `PedalingPolicy` (`enabled`, `coast_above_rpm`, `resume_below_rpm`, `stop_time_s`, `coast_cadence_tau_s`, `mash_cadence_rpm`, `mash_torque_nm`, `effort_slew_nm_s`).
- `shifting`: all `ShiftingConfig` fields, including the cassette list and signed/magnitude slip mode.
- `assist`: resolved `gain`, `max_torque`, `max_power`, `tau`, `slew`, `engage_torque_nm`, `gate_min_crank_rad_s`, `cutoff_mps`, `taper_width_mps`, optional torque-curve rows, and `mode`; optional profile contains resolved four mode gains and `emtb_full_gain_at_nm`. Serialize runtime `drive.assist` values after profile overrides, not authored synthetic defaults.
- `battery`: all `BatteryConfig` fields; `hub_stiffness_nm_rad`, `hub_damping_nm_s` from `drive.config.freehub_k_nm_rad` and `drive.config.freehub_c_nms_rad`.

The public diagnostic class is `bike_native.DrivePolicies(config: dict)`. It owns typed policies and implements:

```python
policies.reset()
policies.pedaling_update(phase_rad, rate_rad_s, required_cadence_rpm,
                         effort_nm, dt, enabled=True, braking=False)  # dict of PedalingState fields
policies.shifting_update(cadence_rpm, required_cadence_rpm, dt,
                         pedaling=True, braking=False, rear_in_contact=True,
                         rear_slip_mps=None)  # bool
policies.shifting_diagnostics()  # shifter fields plus gear_ratio and torque_factor
policies.assist_ceiling(shaft_rpm, speed_mps)  # (ceiling, taper)
policies.assist_step(human_nm, cadence_rpm, speed_mps, braking, dt,
                    torque_request_nm=None, shaft_rpm=None)  # float
policies.battery_draw(requested_power_w, dt)  # float
policies.electrical_power(torque, omega, enabled)  # float, config losses
policies.energy_limit(request, omega, budget_w)  # float, config losses
policies.freehub_torque(phi_c, phi_w, omega_c, omega_w)  # float
policies.state()  # dict, every mutable field, None preserved
policies.set_state(state)  # complete validated atomic restoration
bike_native.human_crank_torque(mean_nm, phase_rad, ripple=.35)
```

Typed classes mirror the unchanged Python method signatures with doubles, bools, optional<double>, and typed result/state structs. The writer in Task 2 consumes those same classes directly, not the FFI diagnostic wrapper. `policy_binding.hpp` exports `DrivePolicyConfig parse_drive_policy_config(const nanobind::dict&)` and `void bind_drive_policies(nanobind::module_&)`; model core only includes typed headers.

State mirrors `vars()` of each policy excluding config/gearing/profile: pedaling's coasting/phase/rate/deceleration/effort/cadence EMA, shifter's rear/from teeth, cooldown/cut/count/direction/two EMAs, assist's torque/pedaling/last_gain, battery's initial/current/drawn energy, hub's boundary/energy/torque. Document exact dict keys in the tests/doc. Reject missing keys, nonfinite numbers, invalid bools, invalid tooth/count integers, negative energy/timers, and mismatched optional shifter EMAs before changing any member. Preserve arbitrary finite values that are legitimate Python snapshots; do not invent narrow physical limits.

- [ ] **Step 1: Write independent failing oracle tests.** Construct a Python applier solely to project configuration and use the unchanged scalar objects/functions as oracle. Include a minimal concrete battery test:

```python
def test_battery_draw_matches_python(config):
    from bike_sim.physics.battery import Battery
    native = bike_native.DrivePolicies(config)
    python = Battery(config['battery']['energy_j'])
    for power, dt in [(5., .002), (1000., .01), (1e9, .1), (0., .01)]:
        assert_bitwise_equal(np.asarray(native.battery_draw(power, dt)),
                             np.asarray(python.draw(power, dt)))
```

Add sequences for coasting hysteresis, mash/slew/EMA, signed rollback cadence and braking; shifter up/down, landing/cooldown/cut/airborne/positive and negative slip in both modes; assist all four profile modes, eMTB ramp, exact curve knots/interior/outside, separate crank and shaft rpm, power/speed taper, gate, brake reset, request ceiling, lag/slew; battery enabled/disabled, a=0/a>0, negative shaft speed, nonzero idle remainder; elastic hub engagement/overrun. Compare every state field after every call, not only returns. Test copied configs and state survive original Python object's deletion and original dict mutation. Test atomic invalid restoration, missing/nonfinite config, reset, state roundtrip, and crank torque phases/ripple.

- [ ] **Step 2: Run RED and retain the expected missing-API output.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drive_policies.py -q
```

Expected failures identify `bike_native.DrivePolicies`/`human_crank_torque` absent, not collection skips or fixture errors.

- [ ] **Step 3: Port the literal reference equations into the typed classes.** Source files are `physics/pedaling.py`, `shifting.py`, `motor.py`, `motor_profile.py`, `battery.py`, `freehub.py`. For example the lag expression must be:

```cpp
double candidate = torque - std::expm1(-dt / config.tau) * (target - torque);
candidate = std::max(torque - config.slew * dt,
                     std::min(torque + config.slew * dt, candidate));
```

Curve interpolation follows NumPy's scalar interpolation arithmetic and endpoint behavior; diagnose any raw-bit mismatch. Validate complete candidates before restoring state. Keep each responsibility in its named file; config parsing and dict conversion stay at the FFI boundary.

- [ ] **Step 4: Build and run GREEN, affected native checks, and compiler sweep.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drive_policies.py tests/reference/test_native_cruise.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
```

Report exact counts, time, warnings, red/green commands, and public typed interfaces for Task 2. Stop and escalate a numerical/platform ambiguity instead of altering oracle tolerance.

- [ ] **Step 5: Self-review and commit only the Task 1 files.**

```bash
git add native/src/drivetrain native/src/binding.cpp native/CMakeLists.txt tools/native_config.py tests/reference/test_native_drive_policies.py docs/TESTING.md
git commit -m "feat(native): port scalar drivetrain policies with bitwise oracles"
```

### Task 2: Model-owned drivetrain and solved transmission accounting

**Files:**
- Create: `native/src/drivetrain/chain.hpp`, `chain.cpp` — chain geometry/gradient, tension and planar geometry/Jacobian helpers.
- Create: `native/src/drivetrain/transmission.hpp`, `transmission.cpp` — ideal and geometric fixed-tendon ratchets, optional crank clutch/motor rotor freewheel, solved force extraction.
- Create: `native/src/writers/drivetrain.hpp`, `drivetrain.cpp` — model-owned applier and force/diagnostic assembly.
- Create: `native/src/drivetrain/drive_binding.hpp`, `drive_binding.cpp` — ride inputs, writer state, diagnostics and output conversion; keep existing main binding manageable.
- Modify: `native/src/config.hpp`, `stepper.hpp`, `stepper.cpp`, `binding.cpp`, `native/CMakeLists.txt` — optional typed drive config, writer ownership, registration.
- Modify: `tools/native_config.py` — `project_drive(drive)` reuses `project_drive_policies`; `project()` emits optional `drive` only when `sim.physical.drive` exists.
- Test: `tests/reference/test_native_drivetrain.py`.
- Modify: `docs/TESTING.md` — stage ordering, supported configurations and acceptance limits.

**Interfaces:**

The drive section contains Task 1 policy config fields plus `drive_mode`, `transmission_model`, `human_torque_nm`, `torque_ripple`, `crank_phase_rad`, `chain_k_n_m`, `chain_c_ns_m`, `bearing_c_nms_rad`, and `motor_clutch`. Runtime model topology determines actuator IDs and optional rotor, with authored rotor inertia serialized/validated as needed. `drive_mode` is the applier's construction-time mode, not a per-call switch. Names and supported topologies are those resolved by the Python applier. No live model, actuator, or policy object crosses the bridge.

`DrivetrainWriter` consumes `mjModel*`, `mjData*`, typed config, and Task 1 policies. It exposes typed methods underlying:

```python
stepper.drive_reset()
stepper.drive_restart_clock()
stepper.drive_prepare_pedaling(control, dt, braking=False, active=True,
    advance=True, rear_in_contact=True, rear_slip_mps=None, effort_ceiling_nm=None)
stepper.drive_components(control, dt, speed_mps, braking=False, active=True,
    advance=True, sensed_human_torque_nm=0.,
    pedaling_state=None, rear_in_contact=True, rear_slip_mps=None)
# dict of owning arrays, exactly Python compute() component names
stepper.drive_settle_actuation()  # owning f64[nv] solved transmission force
stepper.drive_diagnostics(probe=False)  # owning dict of live or probe diagnostics
stepper.drive_stored_energy()  # same dict as Python stored_energy()
stepper.drive_state()  # every mutable writer/policy/transmission field
stepper.set_drive_state(state)  # atomic, complete restoration
stepper.set_inputs(ctrl, qfrc_applied)  # copies finite width nu/nv arrays atomically
```

`control` accepts the same torque/request/enable keys the drive reads from `RideControl`: `motor_torque_nm`, `motor_limit_nm`, `human_torque_nm`, `rider_enabled`; absent values use Python defaults. Optional `pedaling_state` has exactly Task 1 result fields. `drive_mode` values match Python: `crank_effort`, `articulated_effort`, `coast`, `ideal_speed_control`; wrong modes and models fail explicitly. Calls without configured writer fail clearly. `set_inputs` snapshots potentially aliased native views before writes and validates both inputs before either mutation.

Preserve optional policy times/angles/reference/pending-actuation sentinel; model tendon range and coefficient mutations; ideal/geometric boundaries; geometric prepared phi/J/q/time, last tension, shift flags and cumulative work; every policy and diagnostic field. State restoration must recreate these without reading derived MuJoCo arrays from Python. It must validate dimensions, finite/integer/bool/enum fields and positive ratios before mutation. State schemas are documented with tests. Setup and FFI conversion may allocate; the typed writer reuses model-size scratch buffers and contains no Python calls. Do not claim a full allocation-free P4 tick or speedup in this increment.

- [ ] **Step 1: Write failing layer-one and solved-interval tests.** Use compiled small scalar planar models with frame/crank/wheels/pedals; add cassette for elastic-chain mode, fixed tendon for ideal mode, and one coefficient per scalar dof for geometric mode. Use the real builder's compiled model helper where practical without invoking slow equilibrium relaxation. Save MJB, load a separate native owner, initialize identical state and clock. Examples for force staging:

```python
expected = python_drive.compute_components(python_model, python_data, dt,
    speed_mps=speed_mps, braking=braking, active=True, control=control)
actual = native.drive_components(control_dict, dt, speed_mps, braking)
assert set(actual) == set(expected)
for name in expected:
    assert_bitwise_equal(np.asarray(actual[name]), np.asarray(expected[name]))
assert_bitwise_equal(native.ctrl, python_data.ctrl)
```

For solved accounting, run genuine identical `mj_step` and `Stepper.step()` intervals after applying identical force arrays and controls with `set_inputs`; compare returned solved transmission vectors, energy debit, motor diagnostics, geometric defect/work, and state at each of 8–12 short steps before trajectory divergence. Never copy fabricated `efc_*` arrays or Python `_address` into native engine memory. Select actual solved tendon rows using MuJoCo API (`mj_mulJacTVec`), covering dense and sparse Jacobian models and a case with unrelated constraint rows.

Exercise all three transmission models, rotor/legacy clutch, both effort drive modes and both passive modes (including legal absence of human actuator), forward multi-turn angles, slack/tension, wheel overrun, shifting ratio continuity, inactive and `advance=False` force probes, duplicate-time and pending-settlement guards, injected/restored valid states, restart-clock versus full reset, disabled/depleted batteries and energy ceiling. Compare every force channel and diagnostic, not just torque. Verify optional config projection and no-drive Stepper compatibility. Test nonfinite/invalid inputs and atomic restoration, including aliasing in `set_inputs`.

- [ ] **Step 2: Run RED against the missing writer APIs.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drivetrain.py -q
```

Retain expected failures separately from topology/fixture mistakes.

- [ ] **Step 3: Implement geometry, transmissions, and model-owned writer in that dependency order.** Port `physics/chain.py`, `physics/transmission_constraint.py`, `sim/ride/ideal_freehub.py`, `sim/ride/geometric_freehub.py`, and `sim/ride/drivetrain_forces.py` literally. Preserve this ordering:

```text
prepare ratchets -> prepare pedaling/shift -> chain + hub + bearings
-> human sensor -> assist -> energy cap -> write actuator requests
-> genuine MuJoCo solve -> extract solved tendon reactions
-> read actual actuator_force -> validate request bound -> meter/debit battery
```

The solved battery debit uses the saved incoming shaft rate and actual nonnegative actuator force, not the requested moment or outgoing shaft rate. Only `advance=True` live calls mutate policy/time/pending state; `advance=False` uses copies, emits probe diagnostics, and preserves the live policy/transmission state while retaining Python's force/control writes. Do not shift/touch ratchets during probes. Model mutations use owned detached `mjData` for `mj_setConst`; geometric endpoint kinematics use another owned scratch data. Chain Jacobians and dot products match NumPy/Accelerate operation shapes; do not add a duplicate sprocket torque on top of `-T*J`.

- [ ] **Step 4: Run focused GREEN and compiler checks; sanitize the new code once.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_drivetrain.py tests/reference/test_native_drive_policies.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build/asan -j4
```

Run these two test files against the actual sanitizer extension by prepending `native/build/asan` inside Python and asserting its module path. Use `uv run env ASAN_OPTIONS=detect_leaks=0 DYLD_INSERT_LIBRARIES=/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/lib/clang/21/lib/darwin/libclang_rt.asan_osx_dynamic.dylib python ...` so the sanitizer is injected into Python after uv. Record exact test evidence and numerical/platform limitations. After task review and any fixes the controller runs `bash tools/run_tests.sh quick` once, followed by the final increment review.

- [ ] **Step 5: Self-review and commit scoped changes.**

```bash
git add native/src/drivetrain native/src/writers/drivetrain.hpp native/src/writers/drivetrain.cpp native/src/config.hpp native/src/stepper.hpp native/src/stepper.cpp native/src/binding.cpp native/CMakeLists.txt tools/native_config.py tests/reference/test_native_drivetrain.py docs/TESTING.md
git commit -m "feat(native): port physical drivetrain and solved energy accounting"
```

## Completion evidence

Both task reviews and one final increment review must resolve correctness findings. Report policy/writer and sanitizer counts plus the quick result; explicitly retain full-suite/P4 viewer limitations. Collect every ledger ruling and its cost before removing this plan's scratch directory. Keep the existing branch for the remaining P3/P4 work.
