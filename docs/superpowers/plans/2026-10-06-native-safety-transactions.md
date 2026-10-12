# Engine Errors and Atomic State Changes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:
> executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the Python process on fatal engine errors and prevent partial drivetrain updates on rejected input or
allocation failure.

**Architecture:** A pinned C-compatible engine call frame returns fixed-buffer status to the C++ owner. Pure staging
produces candidate Transmission and drivetrain state; Python bindings allocate results before committing candidates.
Settlement uses the same transaction boundary for energy, telemetry, and pending state.

**Tech Stack:** MuJoCo 3.12.0, C++23, nanobind, Python/pytest through uv, CTest, allocation-failure injection,
ASan/UBSan.

**Spec:** [Native safety specification](../specs/2026-10-06-native-safety.md), S5–S6.

## Global Constraints

- Use `uv` for every Python command.
- Retain Python >=3.13, CMake >=3.24, and nanobind >=3.1.0; native platform scope remains Apple arm64/Accelerate.
- Preserve valid-input numerical parity, including operation order, intentional saturation, and documented legacy
  behavior.
- Reject malformed inputs consistently in Python and C++; update both implementations in the same task.
- Compile first-party native code as ISO C++23 with extensions disabled.
- Use the existing MuJoCo 3.12.0 dependency; fail configuration if the error-interception ABI is unavailable.
- Keep alpha analyzer findings report-only; report analyzer execution failures separately from findings.
- Run `native` for native code or shell tooling changes; also run `full` for physics or realtime changes.
- Preserve existing AGENTS.md behavioral-stability requirements.
- Keep allocation, Python boxing, logging, and exception construction outside marked realtime regions.
- Make each task independently reviewable; record exact commands, selected binary, tool health, and test results.

---

## File map and transaction vocabulary

Create `native/src/engine_call.hpp/.cpp` and a narrowly scoped `engine_abi_312.hpp`; change Stepper and engine-using
writers. Add `tests/reference/test_native_engine_errors.py` and `test_native_atomicity.py`; extend Release-safe C++
contract cases. Stage objects own candidates; commit publishes only already-valid state. Fixed vectors/model
coefficients are copied at commit into preallocated storage; strings/maps are swapped. `noexcept` applies to memory-only
commit helpers, not to unguarded engine recomputation. Engine failure poisons the owner even if staging had not
published logical state.

Focused commands after writing regressions:

```bash
uv run python -m pytest tests/reference/test_native_engine_errors.py -q
uv run python -m pytest tests/reference/test_native_atomicity.py -q
uv run cmake --build native/build --target native_contract_tests
uv run ctest --test-dir native/build --output-on-failure
bash tools/run_tests.sh native
bash tools/run_tests.sh full
```

Expected before fixes: fatal child exit1, rejected shift/prepare changes state, injected settlement failure duplicates
debit. Expected after fixes: child survives with FatalError and every rejected/allocation-failed operation preserves its
declared transaction state. Execute selected ASan/UBSan and RTSan builds through V3's corrected launcher.

### E1: Pinned thread-local fatal-error interception

**Files:** Create engine-call/ABI files; modify `native/CMakeLists.txt`, `native/src/stepper.hpp/.cpp`, `binding.cpp`,
engine call sites in `drivetrain/transmission.cpp`, `chain.cpp`, writer implementations and
`drivetrain/drive_binding.cpp`; modify `pyproject.toml`, `uv.lock`, and native launcher dependency selection; create
engine-error tests; extend `native/tests/test_contracts.cpp`; update TESTING.md.

**Interfaces:**

```cpp
namespace engine {
struct ErrorBuffer { std::array<char, 2048> message{}; };
using Operation = void (*)(void *) noexcept;
[[nodiscard]] bool invoke(Operation operation, void *context, ErrorBuffer &error) noexcept;
}
```

`EngineFailure : std::runtime_error` is constructed only outside marked regions.
`Stepper::try_step(engine::ErrorBuffer&) noexcept -> bool` and
`Stepper::try_forward(engine::ErrorBuffer&) noexcept -> bool` carry realtime status; existing `step()`/`forward()`
translate status outside those regions. Stepper tracks `healthy|poisoned`; constructor never publishes a poisoned
object. `reset`/`set_state` may recover only after a successful guarded engine reset and validated state application.

- [x] Build a subprocess regression from inline sphere/plane contact MJCF. Save MJB after setting solver99, invoke
  native constructor, and print a post-exception marker. Compare to stock `mujoco.mj_forward`; native must raise the
  same FatalError class and print the marker. Add missing/empty/garbage MJB controls, runtime failure after successful
  construction via isolated C++ test mutation, makeData/load/setConst coverage, repeated calls, concurrent separate
  owners, and imports in both orders.

```python
def test_native_fatal_preserves_process(fatal_child_script):
    child = subprocess.run(["uv", "run", "python", str(fatal_child_script)],
                           text=True, capture_output=True, check=False)
    assert child.returncode == 0, child.stderr
    assert "caught FatalError; process alive" in child.stdout
```

The fixture creates the model and child script in tmp_path. The child catches `mujoco.FatalError`, asserts
`bike_native.FatalError is mujoco.FatalError`, and prints the marker. Its valid solver control completes forward without
raising.

- [x] Run the fatal subprocess before implementation; expect native exit1. Inspect installed 3.12.0 headers/library
  exports and pinned official interceptor. Add configure-time version and compile/link checks for the private TLS
  log-handler functions and exact callback types. Put all private declarations in `engine_abi_312.hpp`; unsupported ABI
  fails loudly rather than installing a global callback. Compile engine_call.cpp once in a reusable PIC static library
  linked by the module, C++ contracts, and benchmark.
- [x] Add native dependency group `native = ["mujoco==3.12.0", "nanobind>=3.1.0"]` under pyproject.toml's
  dependency-groups and update uv.lock. Native preflight resolves this group from the lock before configuration, and
  subsequent `uv run` commands preserve that selected group. Test an installed unsupported MuJoCo version is rejected
  before building/loading native code. Retain Python-only package version scope and existing Python/CMake floors.
- [x] Implement one fixed-storage jump frame per thread, with a linked prior-frame pointer and saved prior TLS handler.
  The nonreturning fatal handler copies bounded message bytes and jumps to the active frame; nonfatal logs delegate to
  the prior handler. Restore prior frame/handler on both paths. Keep setjmp bookkeeping in a trivial C-compatible frame
  with rules for locals modified after setjmp; no vector/string/unique_ptr/Python objects between jump site and engine
  operation. Never longjmp across a user callback's C++/Python cleanup.

For callback compatibility, add nested stock/native invocation and Python callback controls before exposing the bridge.
Stock callback trampolines must catch their own failures before returning to the native engine frame. Reject unsupported
installed callback trampolines before entering native engine execution, with a descriptive Python error and no mutation;
document which callback slots/versions were validated. A global callback throwing through engine frames is not an
acceptable fallback.

- [x] Replace every fatal-capable engine call found by
  `rg 'mj_(loadModel|makeData|forward|step|resetData|setConst)' native/src` with a bridge operation or an
  already-guarded owning call. Operation contexts are trivial pointers/results; allocate ownership and exceptions
  outside invoke. Ensure partial constructor resources are cleaned by RAII established before the operation, outside the
  jump frame.
- [x] Register nanobind exception translation into the actual imported `mujoco.FatalError` class. Move allocating
  translation outside BIKE_NONBLOCKING. On runtime failure set poisoned and require explicit successful
  reset/reinitialization before mutating APIs; allow read-only diagnostic access. Test poison on failure, rejected
  retries, recovery, nested TLS restoration, allocation-free status return, and resource cleanup.
- [x] Run engine tests, CTest, native/full, ASan/UBSan error probes, and RTSan `try_*` probes. Review/commit when each
  call family survives a fatal error without invalid unwinding, prior stock behavior remains intact, and all advertised
  callback paths have a demonstrated cleanup-safe boundary. Record unsupported callbacks as an explicit interface
  limitation, not silent behavior.

### E2: Stage Transmission ratio and model updates

**Files:** Modify `native/src/drivetrain/transmission.hpp/.cpp`, `chain.hpp/.cpp`, `writers/drivetrain.cpp`,
`src/bike_sim/sim/ride/ideal_freehub.py`, `geometric_freehub.py`; test `test_native_atomicity.py`, native drivetrain
tests and C++ contracts.

**Interfaces:** Add `TransmissionUpdate` containing `GearingConfig gear`, `TransmissionSnapshot state`,
`PreparedTransmission prepared`, `std::vector<double> coefficients`, `std::array<double,2> range`, and
`bool prepared_valid`. `stage_ratio(mjData*, double) -> TransmissionUpdate`;
`stage_prepare(mjData*) -> TransmissionUpdate`; `commit(TransmissionUpdate&)` applies prevalidated model changes through
E1 then memory-only state swaps. Stage calculations use owning scratch model/data; they never mutate the live model.

- [x] Add a valid parsed geometric cassette `(24,1000)` case that makes the next gear geometry invalid. Capture ratio,
  rear teeth, prepared geometry, boundary, range, coefficients, model constants and generation before/after; all remain
  unchanged on geometry error. Repeating the attempt gives the same exception and state. Include valid ideal/geometric
  shift, reset/prepare/restore, clutch/freewheel, and probe controls.

```python
def test_failed_geometric_shift_is_atomic(tmp_path):
    from bike_sim.physics.physical_config import ShiftingConfig
    model, data, _, native = drive_pair(tmp_path, kind="geometric_ideal_mid_drive",
                                        shifting=ShiftingConfig(enabled=True, cassette=(24, 1000),
                                                                cadence_smoothing_tau_s=0.0,
                                                                target_cadence_min_rpm=100.0,
                                                                target_cadence_max_rpm=1000.0))
    data.qvel[model.joint("crank_spin").dofadr[0]] = 4.0
    data.qvel[model.joint("rear_wheel_spin").dofadr[0]] = 1.0
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, data.time)
    native.forward()
    before = freeze(native.drive_state())
    with pytest.raises(ValueError):
        native.drive_prepare_pedaling({}, 0.1)
    assert freeze(native.drive_state()) == before
```

Direct C++ contract tests additionally call stage_ratio with `front_teeth/1000.0`. The Python test uses the audited
parsed-config reproduction directly, without a new public ratio hook.

- [x] Run failing shift test and record current ratio change. Refactor geometry/linearization to accept candidate
  gearing and scratch model/data explicitly. Compute all coefficients/ranges/diagnostics and validate finiteness before
  model writes; run candidate mj_setConst on scratch via E1.

```cpp
TransmissionUpdate Transmission::stage_ratio(mjData *data, double ratio) {
    auto update = make_update_from_current();
    update.gear.rear_teeth = validated_rear_teeth(ratio);
    update.state.ratio = ratio;
    evaluate_candidate(data, update);
    return update;
}
```

Define private `make_update_from_current() -> TransmissionUpdate`, `validated_rear_teeth(double) -> int`, and
`evaluate_candidate(mjData*, TransmissionUpdate&) -> void` with the existing gear conversion/geometry arithmetic.
Candidate scratch allocation occurs before live mutation; do not change rounding or tooth-resolution behavior.

- [x] Implement guarded model commit, then no-throw logical/storage commit. Preallocate destination extents, copy
  coefficients/range, recompute constants through E1, then swap logical state and copy prepared values into persistent
  prepared-storage buffers without changing capacity or identity. If recomputation unexpectedly fatals, poison owning
  Stepper and do not present a successful logical transaction. Mirror staging in Python ideal/geometric constraints
  using scratch model/data; include reset/restore as well as set_ratio.
- [x] Inject allocation failure at each stage allocation using E4's harness once available; before that use direct
  invalid geometry and restore validation cases. Run native/full and ASan geometry tests. Review/commit when valid shift
  numerical parity and rejected shift state/model equality both hold.

### E3: Atomic prepare, compute, probe, and Python boxing

**Files:** Modify `native/src/writers/drivetrain.hpp/.cpp`, `drivetrain/drive_binding.cpp`, policy implementations,
`src/bike_sim/sim/ride/drivetrain_forces.py`; test `test_native_atomicity.py`, existing probe/clock/state tests.

**Interfaces:** Add `PreparedTick` with candidate `DriveSnapshot`, `PedalingSnapshot`, `ShiftingSnapshot`,
`AssistSnapshot`, `ForceComponents`, ctrl vector, and optional E2 `TransmissionUpdate` values for hub/clutch/freewheel.
Define
`PrepareInputs { RideControl control; double dt; bool braking, active, advance, contact; std::optional<double> slip, ceiling; }`
and
`TickInputs { RideControl control; double dt, speed, sensed; bool braking, active, advance, contact; std::optional<PedalingState> pedaling; std::optional<double> slip; }`.
Methods are `stage_prepare(const PrepareInputs&) -> PreparedTick`,
`stage_components(const TickInputs&) -> PreparedTick`, and `commit(PreparedTick&) -> void`. FFI boxes from candidate
components/diagnostics before calling commit.

- [x] Test negative effort ceiling, invalid late control, nonfinite sensed torque, duplicate timestamp, unsettled
  pending actuation, failed shifting geometry, and probe calls. Capture policy snapshots, ctrl, live/probe diagnostics,
  timestamps, pending state and model coefficients. Check Python as well as native. Pending rejection happens before any
  pedaling/assist advance.

```python
def test_rejected_prepare_keeps_policy_state(tmp_path):
    _, _, _, native = drive_pair(tmp_path)
    before = freeze(native.drive_state())
    with pytest.raises(ValueError):
        native.drive_prepare_pedaling({}, 0.01, effort_ceiling_nm=-1.0)
    assert freeze(native.drive_state()) == before
```

The empty control dictionary is the existing declared default and remains valid; all supplied optional control fields
still undergo exact type/key validation.

- [x] Run regression cases and record current mutation. Perform all runtime/pending/time checks first. Create candidate
  policies from current snapshots, use E2 staged model updates, and build diagnostics/components before committing.
  Restore/probe paths use candidate state rather than mutate-then-copy rollback. Add candidate control writes instead of
  writing live data.ctrl while computing.

```cpp
auto tick = owner.drive().stage_components(inputs);
nb::dict result = box_components(tick.components);
owner.drive().commit(tick);
return result;
```

`box_components(const ForceComponents&) -> nb::dict` reuses F4's RAII arrays. Boxing success precedes commit; no
implicit new ndarray/string conversion follows commit.

- [x] Ensure direct C++ components API retains its current return type and internally stages/commits without boxing.
  Probe returns a candidate result and commits only the declared probe telemetry after successful allocation; all
  physical/policy/clock state stays unchanged. Mirror Python staged policies, ctrl/model updates and result dict
  creation; only assignment of complete candidates occurs after success.
- [x] Document and test successful prepare→compute with supplied pedaling state versus compute's internal prepare path.
  They produce the same one-tick policy advance. Preserve the existing standalone successful-prepare semantics; rejected
  prepares/compute calls remain atomic. Add a control that repeating a rejected prepare cannot accumulate effort,
  without treating repeated successful prepares as an already specified idempotent API.
- [x] Add failing-result-boxing controls using isolated CPython allocation injection; no production fault-injection
  hook. Use E4's C++ harness to enumerate core allocation positions. Run native/full, ASan, and valid drive goldens.
  Review/commit when failure retries equal one fresh successful call, probe state contract holds, and no stage leaks
  live mutations.

### E4: Exactly-once settlement and exhaustive allocation-failure tests

**Files:** Modify `native/src/writers/drivetrain.hpp/.cpp`, `drivetrain/motor.hpp/.cpp`, `drivetrain/drive_binding.cpp`,
`src/bike_sim/sim/ride/drivetrain_forces.py`, `src/bike_sim/physics/battery.py`; create
`native/tests/allocation_faults.hpp/.cpp`; extend C++ contracts and atomicity tests.

**Interfaces:** `PreparedSettlement` owns candidate `BatterySnapshot`, `Diagnostics`, pending-state removal, and force
output. `stage_settle() -> PreparedSettlement`; `commit(PreparedSettlement&) noexcept -> void` is memory-only.
`allocation_faults::arm(std::size_t successful_allocations)` and `disarm() noexcept` affect only the test executable's
thread-local overridden allocation operators, including aligned forms.

- [x] Reproduce sparse `last={}` restore followed by settlement allocation failure after battery draw. Assert unchanged
  battery/pending/telemetry on failure, then exactly one debit on retry. Include complete diagnostics (normal
  zero-allocation settle), ideal/geometric/elastic, enabled/disabled assist, and clutch/rotor cases.

```cpp
for (std::size_t fail_at = 0; fail_at < allocation_count; ++fail_at) {
    fixture.restore_initial();
    allocation_faults::arm(fail_at);
    bool failed = false;
    try { fixture.perform_operation(); }
    catch (const std::bad_alloc &) { failed = true; }
    allocation_faults::disarm();
    if (failed) fixture.require_unchanged_then_retry_matches_success();
}
```

Build snapshots/assertion output before arming; disarm on every unexpected exception through a test-only guard.
`allocation_count` is measured on one successful operation after fixture initialization. Repeat enumeration for E2/E3
reset/restore/prepare/advance/probe/settle operations, not just the first allocation.

- [x] Run the regression and capture current duplicate debit. Add a pure candidate battery debit calculation with the
  same arithmetic as `Battery::draw`; populate all diagnostic entries and output storage on the candidate before commit.
  Remove pending state only alongside successful battery/telemetry publication.

```cpp
auto settlement = owner.drive().stage_settle();
auto output = owned(settlement.force);
owner.drive().commit(settlement);
return output;
```

Use F4's array helper; return an already-owned Python object. C++ convenience settlement stages and commits before
returning its persistent output span.

- [x] Mirror candidate battery/result publication in Python. Test repeated settlement with no pending work follows the
  documented existing behavior and cannot debit energy again. Add budget/initial-energy relational checks from F3 to
  restore.
- [x] Run CTest allocation enumeration in Release and ASan/UBSan, atomicity tests, native/full. Check the allocation
  injector does not link into the production module and no test expectation/allocation runs while it is armed.
- [x] Review/commit when all allocation-position failures preserve state and successful retries match clean execution;
  publish a compact operation/path/positions table. Engine fatal cases remain the separate poison contract tested in E1.
