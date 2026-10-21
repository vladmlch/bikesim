# Review of native startup and physics/research commits

Date: 2026-10-17.

Reviewed commits: `9c940c9617c55c7b2e3deb800b697b89563fa17d`
and `9d9e49c0414c2dc03f712f55fb38efdbd8ef5925`. The review covers the
combined tree at the second commit relative to the parent of the first.

## Re-verification 2026-10-19

Each finding was re-checked against the current working tree (which now
compiles: ctest 57/57 registered tests pass, and all nine sweeps that
emit summary reports — analyzer, context, contracts, cppcheck,
diagnostic-controls, frontends, headers, odr, tidy — report
`status: passed`; the shellcheck `check_scripts` target also ran).

- **F1 — FIXED.** `initialization/capture.cpp:191-196` now captures tire
  state only behind `stepper.has_tire()` and emits the empty names/row
  pair for `native_reference`; `runtime/runtime.cpp:509-511` restores
  only when a writer exists or the document carries tire names. The
  bootstrap path no longer calls an absent writer.
- **F2 — STILL OPEN (P1).** `runtime/step.cpp:1454-1462` still writes
  `0.` to `rear_drive` for `drive_mode == "ideal_speed_control"`, which
  `runtime/capabilities.cpp:20-21` still admits. The oracle computes the
  cruise torque at `src/bike_sim/sim/ride/physical_runtime.py:389-394`.
  The stale comments claiming the branch was unreachable/"dead by
  construction" were corrected on 2026-10-19 to mark the open finding.
- **F3 — STILL OPEN (P2).** `runtime/step.cpp:1300` still gates
  resistance behind `active && stepper_->has_tire()`; the oracle applies
  resistance from contact snapshots when the tire writer is absent
  (`physical_runtime.py:385-387`). The stale comment claiming the source
  couples resistance only to a tire writer was corrected on 2026-10-19.
- **F4 — FIXED.** `research/policy.cpp:69-70` routes the decoded control
  through `runtime::control_validate_for()` and translates the failure
  to `PolicyFailure`; `runtime/control.cpp:27-34` now enforces the
  finite-nonnegative scalar and posture domains the oracle's
  `RideControl.__post_init__` requires. `migration_T35` passes.
- **F5 — STILL OPEN (P2).** `runtime/step.cpp:1426-1429` still passes
  `active && !initializing_` as `pedal_recovery`; the oracle passes
  `cfg.seated_climb.enabled` (`physical_runtime.py:367`). The divergence
  is now marked with an `OPEN FINDING F5` comment at the call site.

Remaining acceptance gaps are unchanged: runtime qualification,
per-source oracle equivalence and wave acceptance reviews are still
open. The original review below is kept as history.

## Result and evidence boundary

**Changes required.** Five source-supported findings remain open. Two
prevent an advertised physical route from working, and three change
physics or validation behavior. All five concern integration introduced by
`9d9e49c0`, including consumers delivered by the startup wave.

This review inspected source and diffs only. No tests, configuration,
compilation, simulation, plugin loading, project Python imports, reference
generation, or numerical experiments were executed. Existing test sources
are used to identify intended contracts, never as passing evidence. None of
the findings below is labelled runtime-reproduced.

The review followed `docs/agents/native-engineering.md` and checked model
and configuration ownership, startup/finalization/bootstrap consumers,
build registration, contact/drive/controller dispatch, state/cache
boundaries, research construction, policy ownership and lifecycle. Python
counterparts were inspected for the behavioral discrepancies below. The
existing numerical-provider and executable-qualification gates in both
wave reports remain open; this review does not establish full parity for
every declaration in the migration inventories.

## F1 — P1: native-reference tires cannot cross the bootstrap boundary

Locations:

- `native/src/runtime/capabilities.cpp:36`: the new capability gate admits
  `tires.backend == "native_reference"`.
- `native/src/runtime/project_config.cpp`, `project_writer_config`: that
  backend intentionally omits the applied tire writer.
- `native/src/initialization/capture.cpp:191`: bootstrap capture calls
  `stepper.tire_state()` unconditionally.
- `native/src/runtime/runtime.cpp:506`: bootstrap restoration calls
  `stepper_->set_tire_state(...)` unconditionally.

**Trigger and consequence:** prepare a valid physical native-reference
configuration and attempt `seal_bootstrap()` / `app::prepare()`. Even if
equilibrium succeeds, capture calls an absent writer and throws. Guarding
capture alone leaves a second failure when the runtime restores the
bootstrap, including on reset. Thus the newly admitted backend cannot
complete the advertised native startup path.

**Independent contract check:** `Stepper::tire_state()` and
`Stepper::set_tire_state()` delegate to `tire_writer()`;
`native/src/stepper.cpp:361` explicitly throws when all three applied
writers are absent. The default names returned by `tire_state_names()` do
not provide a writer or a usable state row. The low-level native contact
tests bypass this startup/restore boundary.

**Required correction:** define and validate the bootstrap representation
for an absent applied tire writer and handle it consistently in capture,
construction and reset. Until then, keep this route rejected at startup.

**Future regression, not run:** native-reference prepare → seal → construct
→ reset, with a compliant-tire startup/reset control. A low-level contact
query alone does not cover this failure.

## F2 — P1: newly admitted ideal speed control always commands zero torque

Location: `native/src/runtime/step.cpp:1449`.

**Trigger and consequence:** select physical
`drive_mode == "ideal_speed_control"` and step with a positive target speed
and no braking. The capability gate and runtime decoder now accept the
mode, but its active branch still writes `0.` to the `rear_drive` actuator.
It never calls the cruise controller. The old comment claiming this branch
is unreachable is no longer true. The bike therefore does not accelerate
under the requested speed controller.

**Independent contract check:**
`src/bike_sim/sim/ride/physical_runtime.py:389` calls
`sim.cruise.compute(...)`, uses rear grounded state and only suppresses a
positive torque while braking. Native startup projects a cruise controller
and the model retains `rear_drive` for this mode, so zero torque is not an
absence-of-actuator fallback.

**Required correction:** connect cruise evaluation and braking behavior to
the physical force assembly, including its state and accounting; otherwise
reject the mode before setup.

**Future regression, not run:** positive speed error with grounded rear
wheel must produce the source controller torque; braking and zero-error
controls must follow their respective suppression/response contracts.

## F3 — P2: native-reference contact drops resistance forces

Locations: `native/src/runtime/step.cpp:1295` and the force assembly after
`refresh_contacts()` at line 1439.

**Trigger and consequence:** step a physical native-reference model with
nonzero aerodynamic or rolling resistance. The only resistance-writer
call is inside `active && stepper_->has_tire()`. The native-contact route
refreshes its solved snapshots later, but never evaluates resistance from
them. Aerodynamic and rolling losses are omitted, changing acceleration,
coasting behavior and work accounting. This remains a separate physics
defect after F1 is fixed and also affects direct `PhysicalStep` consumers.

**Independent contract check:**
`src/bike_sim/sim/ride/physical_runtime.py:385` explicitly evaluates
`self.resistance.compute_components(m, d, snapshots)` when the applied tire
writer is absent, after forwarding and obtaining native contacts. The
comment claiming the source couples resistance only to an applied tire
writer contradicts that branch.

**Required correction:** add resistance evaluation using the native solved
contact snapshots at the matching lifecycle point and retain its component
work in the common accumulator.

**Future regression, not run:** native-reference coasting with nonzero
resistance versus a zero-resistance control; compare named force components
and accumulated work to the Python route.

## F4 — P2: policy ABI accepts invalid numeric commands

Locations: `native/src/research/policy.cpp:45` and
`native/src/runtime/control.cpp:20`.

**Trigger and consequence:** a plugin returns a present motor torque or
motor limit containing NaN, infinity, or a negative value, with otherwise
valid ABI fields. `policy_control_from_abi()` copies the payload and calls
`control_validate_for()`. That function checks mode and rider applicability
only; it does not validate numeric domains. `validate_motor_policy_control()`
adds ownership checks but still no scalar validation. The malformed command
therefore returns from `PluginPolicy::act()` successfully and can be
accepted into the delayed research command queue before a later consumer
rejects it. This breaks rejection and error attribution at the policy
boundary. Present posture payloads have the same missing numeric validation
in the public ABI decoder.

**Independent contract checks:**

- `src/bike_sim/sim/ride/control.py`, `RideControl.__post_init__`, requires
  finite nonnegative optional torque/rate values; the native wire decoder
  in `native/src/runtime/programs.cpp:70` also enforces those domains.
- `native/tests/migration/test_t35.cpp:88` explicitly expects NaN torque
  to cause `PolicyFailure`; `policy_fixture.cpp` supplies the corresponding
  `bad_number` callback. The current decoder has no check that can satisfy
  those assertions. These tests were inspected, not run.

**Required correction:** validate all present scalar and posture fields at
the ABI boundary, preserving absent-payload semantics and translating
domain errors to `PolicyFailure`. Apply the same value validation before a
typed policy command is staged or accepted into the environment.

**Future regression, not run:** present NaN/infinite/negative torque,
limit and rate, malformed posture, valid explicit zero and absent optional
payloads; verify rejection occurs before request/queue mutation.

## F5 — P2: pedal recovery ignores the configuration switch

Location: `native/src/runtime/step.cpp:1419`.

**Trigger and consequence:** use a non-spindle articulated rider with flat
pedals and `seated_climb.enabled == false`. On an active post-initialization
control tick the new call passes `active && !initializing_` as
`pedal_recovery`. The non-spindle controller can therefore enter its
release/escape/raise/return state machine even though the source feature is
disabled. In the affected foot-contact states it changes sole targets,
stance blending and delivered effort.

**Independent contract check:**
`src/bike_sim/sim/ride/physical_runtime.py:366` passes
`pedal_recovery=self.cfg.seated_climb.enabled`. The native
`LegacyController::compute()` gates `observe_recovery()` on the received
recovery flag, non-steady operation and non-welded pedals, confirming this
argument affects mutable controller state rather than diagnostics alone.

**Required correction:** preserve the configured recovery switch when
dispatching the controller, retaining any required initialization/probe
guards.

**Future regression, not run:** identical flat-pedal states below the pedal
with recovery disabled/enabled, plus a non-triggering foot-position control.
Compare recovery state and target/effort outputs to Python.

## Handoff

No production code or tests were changed in this review. These findings
remain open and the waves cannot be accepted as correct from their current
static evidence. After corrections, the documented focused regressions and
required native/full gates still need execution in an authorized session.
No acceptance/deletion status was advanced here.

## Source review 2026-10-21 — completion-plan Task 1.05

The newer supplied source contains the F2/F3/F5 corrections described by
the completion plan (which attributes them to 89cfcb1):

- F2: PhysicalStep calls cruise.compute() in the active ideal-speed
  branch, retains the positive-torque brake suppression, and writes the
  result to the drive actuator.
- F3: the engine-contact route adds resistance after refresh_contacts()
  when no tire writer exists.
- F5: the rider compute call receives config_.intent.enabled, preserving
  the seated_climb.enabled switch.

The matching branches in src/bike_sim/sim/ride/physical_runtime.py and the
native_reference_resistance_and_cruise / seated_climb_recovery_switch
cases in native/tests/migration/test_t23.cpp were inspected as source.
No tests were executed: the current user explicitly prohibited compilation
and execution. These findings remain verification-open; the 73 removal
rows and four task records have not been promoted to implemented_tested.
The Task 1.05 gate still requires migration_T23/T26/T29/T30/T32 on both
graphs. This section does not retroactively change the older observations.
