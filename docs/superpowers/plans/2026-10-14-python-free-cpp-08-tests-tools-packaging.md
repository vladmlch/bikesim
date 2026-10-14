# Test migration, validation, tooling and distributions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T52 through T60 with source-backed parity and independently reviewed test cycles.

**Architecture:** Preserve the existing native execution path and replace only this subsystem's Python-owned construction, behavior or orchestration. Downstream consumers use the declared owned interfaces; Python reference code stays isolated until final removal.

**Tech Stack:** C++23, CMake >=3.24, pinned native MuJoCo 3.12.0/DuckDB 1.5.6, existing numeric kernels, native IO/GUI/test tools.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`. Also read `migration/interface-contracts.md`, `migration/api-index.md`, and the assigned source cards.

## Global Constraints

- Preserve all required first-party behavior; no Python fallback, embedded interpreter, subprocess interpreter or mandatory Python build/test tool in the final product.
- Preserve existing C++ kernels and the single authoritative physical stepping path; boundary extraction is not permission to change equations or operation order.
- Use C++23 and CMake >=3.24. Retain strict warnings, stack protection, numerical flags, hardening and native safety checks.
- Initially retain MuJoCo 3.12.0 with the verified private TLS error ABI and DuckDB 1.5.6; lock actual native dependency hashes before use.
- Maintain named schema/config/CLI/recording/replay contracts and every supported mode, or obtain explicit user approval for a documented compatibility break.
- Keep existing same-provider bitwise guarantees; no global epsilon, golden regeneration to hide failures, or silent cross-provider equivalence claims.
- Recording remains opt-in, RAM-retained during execution and final-only publication; no streaming redesign, silent history loss or automatic recording from --out.
- Every Python deletion requires a ledger owner, native replacement or approved retirement, assertion/capability coverage and passing evidence; final removal is T63.
- Changes to shared headers/CMake/registries go through one integration owner. Run full native gates serially per machine and record the exact build identity.
- Source examples and API snippets in this plan are proposed implementation contracts, not a claim that code was compiled or tests passed during planning.

---

The named execution sub-skills refer to the downstream agent's environment. When unavailable, follow the same task-by-task red/green/independent-review protocol; no capability is assumed in this handoff. All statuses start planned. The code examples are contracts/regressions to implement and validate, not patches already applied to the application.

## Package map

| Task | Deliverable | Dependencies |
|---|---|---|
| T52 | Port every numerical, unit and property assertion out of pytest | T26, T27, T29, T30, T31, T32, T33, T37, T45, T51 |
| T53 | Port integration, frontend, replay and failure-path test suites | T46, T49, T51, T52 |
| T54 | Port validation rigs, calibration and physical release evidence | T29, T30, T31, T37, T50, T52 |
| T55 | Port performance, diagnostic and native-benchmark developer tools | T25, T37, T44, T54, T06 |
| T56 | Replace Python native-check orchestration and inline interpreter scripts | T03, T06, T02 |
| T57 | Deliver a native test runner, sanitizers and coverage provenance | T52, T53, T56, T55 |
| T58 | Qualify Linux headless native support without changing macOS arithmetic silently | T04, T25, T54, T57 |
| T59 | Replace Python wheel installers with offline native distributions | T47, T50, T51, T55, T56, T57, T58 |
| T60 | Update active documentation and agent contracts to the native workflow | T47, T54, T55, T56, T57, T59 |

## Task T52: Port every numerical, unit and property assertion out of pytest

**Dependencies:** T26, T27, T29, T30, T31, T32, T33, T37, T45, T51

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `migration/test-port-map.json` |
| Create | `native/tests/support/property_cases.hpp` |
| Create | `native/tests/integration/test_rider_welds.cpp` |
| Create | `native/tests/reference/project_test_constraint_mode_search.cpp` |
| Create | `native/tests/reference/test_acceptance_interval_capture.cpp` |
| Create | `native/tests/reference/test_alpine_track.cpp` |
| Create | `native/tests/reference/test_articulated_config.cpp` |
| Create | `native/tests/reference/test_assist_controller.cpp` |
| Create | `native/tests/reference/test_attachment_budget.cpp` |
| Create | `native/tests/reference/test_attachment_wrench.cpp` |
| Create | `native/tests/reference/test_balance_monitor.cpp` |
| Create | `native/tests/reference/test_cadence_shifter.cpp` |
| Create | `native/tests/reference/test_closed_form_rider.cpp` |
| Create | `native/tests/reference/test_contact_safety.cpp` |
| Create | `native/tests/reference/test_control_clock.cpp` |
| Create | `native/tests/reference/test_coupled_rider_task.cpp` |
| Create | `native/tests/reference/test_crank_split.cpp` |
| Create | `native/tests/reference/test_distributed_tire_reference.cpp` |
| Create | `native/tests/reference/test_divergence_horizon.cpp` |
| Create | `native/tests/reference/test_drivetrain_policy_clone.cpp` |
| Create | `native/tests/reference/test_energy_ledger.cpp` |
| Create | `native/tests/reference/test_episode_compare.cpp` |
| Create | `native/tests/reference/test_frozen_force_arrays.cpp` |
| Create | `native/tests/reference/test_golden_episode.cpp` |
| Create | `native/tests/reference/test_golden_episode_v2.cpp` |
| Create | `native/tests/reference/test_hud_columns.cpp` |
| Create | `native/tests/reference/test_joint_strength.cpp` |
| Create | `native/tests/reference/test_lean_limit.cpp` |
| Create | `native/tests/reference/test_leg_loop.cpp` |
| Create | `native/tests/reference/test_monitor_strictness.cpp` |
| Create | `native/tests/reference/test_motor_freewheel_topology.cpp` |
| Create | `native/tests/reference/test_motor_profile.cpp` |
| Create | `native/tests/reference/test_native_attachment_wrench.cpp` |
| Create | `native/tests/reference/test_native_benchmark.cpp` |
| Create | `native/tests/reference/test_native_brake_resistance.cpp` |
| Create | `native/tests/reference/test_native_clock_contracts.cpp` |
| Create | `native/tests/reference/test_native_cruise.cpp` |
| Create | `native/tests/reference/test_native_distributed_stands.cpp` |
| Create | `native/tests/reference/test_native_distributed_tire.cpp` |
| Create | `native/tests/reference/test_native_drive_policies.cpp` |
| Create | `native/tests/reference/test_native_drivetrain.cpp` |
| Create | `native/tests/reference/test_native_encode_columns.cpp` |
| Create | `native/tests/reference/test_native_numeric_completion.cpp` |
| Create | `native/tests/reference/test_native_numeric_domains.cpp` |
| Create | `native/tests/reference/test_native_pneumatic_brush.cpp` |
| Create | `native/tests/reference/test_native_pneumatic_carcass.cpp` |
| Create | `native/tests/reference/test_native_pneumatic_geometry.cpp` |
| Create | `native/tests/reference/test_native_pneumatic_model.cpp` |
| Create | `native/tests/reference/test_native_pneumatic_tire.cpp` |
| Create | `native/tests/reference/test_native_pneumatic_wire.cpp` |
| Create | `native/tests/reference/test_native_rider_contact_math.cpp` |
| Create | `native/tests/reference/test_native_rider_contacts.cpp` |
| Create | `native/tests/reference/test_native_rider_forces.cpp` |
| Create | `native/tests/reference/test_native_rider_intent.cpp` |
| Create | `native/tests/reference/test_native_spindle_controller.cpp` |
| Create | `native/tests/reference/test_native_static_brake.cpp` |
| Create | `native/tests/reference/test_native_stepper.cpp` |
| Create | `native/tests/reference/test_native_suspension.cpp` |
| Create | `native/tests/reference/test_native_tire.cpp` |
| Create | `native/tests/reference/test_numba_kernels.cpp` |
| Create | `native/tests/reference/test_pedal_waveform.cpp` |
| Create | `native/tests/reference/test_pedaling_policy.cpp` |
| Create | `native/tests/reference/test_period_buffer.cpp` |
| Create | `native/tests/reference/test_pinned_topology.cpp` |
| Create | `native/tests/reference/test_planar_arms.cpp` |
| Create | `native/tests/reference/test_playback_clock.cpp` |
| Create | `native/tests/reference/test_pneumatic_tire_config.cpp` |
| Create | `native/tests/reference/test_pneumatic_tire_forces.cpp` |
| Create | `native/tests/reference/test_realistic_pedelec_acceptance.cpp` |
| Create | `native/tests/reference/test_recorder_columns.cpp` |
| Create | `native/tests/reference/test_reference_monitor.cpp` |
| Create | `native/tests/reference/test_ride_backend_cli.cpp` |
| Create | `native/tests/reference/test_ride_plots_html_markers.cpp` |
| Create | `native/tests/reference/test_rider_allocation.cpp` |
| Create | `native/tests/reference/test_rider_dynamics.cpp` |
| Create | `native/tests/reference/test_rider_information_boundary.cpp` |
| Create | `native/tests/reference/test_savage_track_surface.cpp` |
| Create | `native/tests/reference/test_seated_climb_surge.cpp` |
| Create | `native/tests/reference/test_seated_pedaling_cycle.cpp` |
| Create | `native/tests/reference/test_seated_posture_program.cpp` |
| Create | `native/tests/reference/test_seated_topology.cpp` |
| Create | `native/tests/reference/test_sensor_noise.cpp` |
| Create | `native/tests/reference/test_system_momentum_drive.cpp` |
| Create | `native/tests/reference/test_terrain_placement_regression.cpp` |
| Create | `native/tests/reference/test_tyre_road_profile.cpp` |
| Create | `native/tests/reference/test_welded_profile_config.cpp` |

Read-only source authority (do not delete during the port):

- `project/tests/reference/test_constraint_mode_search.py:1-170`
- `tests/reference/_bits.py:1-17`
- `tests/reference/_encode_oracle.py:1-223`
- `tests/reference/_legacy_sensor_oracle.py:1-157`
- `tests/reference/_physical_hud_oracle.py:1-633`
- `tests/reference/conftest.py:1-180`
- `tests/reference/test_acceptance_interval_capture.py:1-19`
- `tests/reference/test_alpine_track.py:1-373`
- `tests/reference/test_articulated_config.py:1-51`
- `tests/reference/test_assist_controller.py:1-197`
- `tests/reference/test_attachment_budget.py:1-79`
- `tests/reference/test_attachment_wrench.py:1-199`
- `tests/reference/test_balance_monitor.py:1-118`
- `tests/reference/test_cadence_shifter.py:1-83`
- `tests/reference/test_closed_form_rider.py:1-123`
- `tests/reference/test_contact_safety.py:1-119`
- `tests/reference/test_control_clock.py:1-84`
- `tests/reference/test_coupled_rider_task.py:1-96`
- `tests/reference/test_crank_split.py:1-60`
- `tests/reference/test_distributed_tire_reference.py:1-137`
- `tests/reference/test_divergence_horizon.py:1-39`
- `tests/reference/test_drivetrain_policy_clone.py:1-39`
- `tests/reference/test_energy_ledger.py:1-315`
- `tests/reference/test_episode_compare.py:1-51`
- `tests/reference/test_frozen_force_arrays.py:1-28`
- `tests/reference/test_golden_episode.py:1-37`
- `tests/reference/test_golden_episode_v2.py:1-112`
- `tests/reference/test_hud_columns.py:1-46`
- `tests/reference/test_joint_strength.py:1-290`
- `tests/reference/test_lean_limit.py:1-38`
- `tests/reference/test_leg_loop.py:1-99`
- `tests/reference/test_monitor_strictness.py:1-278`
- `tests/reference/test_motor_freewheel_topology.py:1-203`
- `tests/reference/test_motor_profile.py:1-90`
- `tests/reference/test_native_attachment_wrench.py:1-1183`
- `tests/reference/test_native_benchmark.py:1-1016`
- `tests/reference/test_native_brake_resistance.py:1-233`
- `tests/reference/test_native_clock_contracts.py:1-370`
- `tests/reference/test_native_cruise.py:1-336`
- `tests/reference/test_native_distributed_stands.py:1-159`
- `tests/reference/test_native_distributed_tire.py:1-410`
- `tests/reference/test_native_drive_policies.py:1-510`
- `tests/reference/test_native_drivetrain.py:1-560`
- `tests/reference/test_native_encode_columns.py:1-1088`
- `tests/reference/test_native_numeric_completion.py:1-959`
- `tests/reference/test_native_numeric_domains.py:1-229`
- `tests/reference/test_native_pneumatic_brush.py:1-893`
- `tests/reference/test_native_pneumatic_carcass.py:1-868`
- `tests/reference/test_native_pneumatic_geometry.py:1-338`
- `tests/reference/test_native_pneumatic_model.py:1-1058`
- `tests/reference/test_native_pneumatic_tire.py:1-965`
- `tests/reference/test_native_pneumatic_wire.py:1-378`
- `tests/reference/test_native_rider_contact_math.py:1-244`
- `tests/reference/test_native_rider_contacts.py:1-1256`
- `tests/reference/test_native_rider_forces.py:1-345`
- `tests/reference/test_native_rider_intent.py:1-617`
- `tests/reference/test_native_spindle_controller.py:1-585`
- `tests/reference/test_native_static_brake.py:1-158`
- `tests/reference/test_native_stepper.py:1-22`
- `tests/reference/test_native_suspension.py:1-116`
- `tests/reference/test_native_tire.py:1-396`
- `tests/reference/test_numba_kernels.py:1-178`
- `tests/reference/test_pedal_waveform.py:1-110`
- `tests/reference/test_pedaling_policy.py:1-89`
- `tests/reference/test_period_buffer.py:1-397`
- `tests/reference/test_pinned_topology.py:1-84`
- `tests/reference/test_planar_arms.py:1-266`
- `tests/reference/test_playback_clock.py:1-70`
- `tests/reference/test_pneumatic_tire_config.py:1-158`
- `tests/reference/test_pneumatic_tire_forces.py:1-485`
- `tests/reference/test_realistic_pedelec_acceptance.py:1-196`
- `tests/reference/test_recorder_columns.py:1-95`
- `tests/reference/test_reference_monitor.py:1-36`
- `tests/reference/test_ride_backend_cli.py:1-152`
- `tests/reference/test_ride_plots_html_markers.py:1-70`
- `tests/reference/test_rider_allocation.py:1-640`
- `tests/reference/test_rider_dynamics.py:1-228`
- `tests/reference/test_rider_information_boundary.py:1-165`
- `tests/reference/test_savage_track_surface.py:1-11`
- `tests/reference/test_seated_climb_surge.py:1-44`
- `tests/reference/test_seated_pedaling_cycle.py:1-229`
- `tests/reference/test_seated_posture_program.py:1-276`
- `tests/reference/test_seated_topology.py:1-175`
- `tests/reference/test_sensor_noise.py:1-117`
- `tests/reference/test_system_momentum_drive.py:1-25`
- `tests/reference/test_terrain_placement_regression.py:1-39`
- `tests/reference/test_tyre_road_profile.py:1-72`
- `tests/reference/test_welded_profile_config.py:1-48`
- `tests/test_rider_welds.py:1-88`

Declaration-level card: `migration/cards/T52.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T26` / `migration/api-index.md#t26`: Close parity for already-ported kernels and retire their Python mirrors.
- `T27` / `migration/api-index.md#t27`: Implement remaining drivetrain capabilities.
- `T29` / `migration/api-index.md#t29`: Port remaining rider dynamics and coupled task controllers.
- `T30` / `migration/api-index.md#t30`: Implement flat/weld/spring rider supports and missing physical modes.
- `T31` / `migration/api-index.md#t31`: Port legacy tire backends and nonphysical ride integration.
- `T32` / `migration/api-index.md#t32`: Port remaining legacy rider/control/measurement behavior.
- `T33` / `migration/api-index.md#t33`: Port suspension playground and stand controllers.
- `T37` / `migration/api-index.md#t37`: Port research metrics, validity and experiment evidence.
- `T45` / `migration/api-index.md#t45`: Port summary reports and explicit data exports.
- `T51` / `migration/api-index.md#t51`: Port batch jobs and research sweeps with deterministic process ownership.

**Produces:**

```text
Each test-port-map record has:
{source_path, source_symbol, collected_node_ids, owner_task, native_test_id,
 native_source, assertions, fixtures, comparison_class, disposition, evidence}.
Allowed dispositions: ported, duplicate-with-proof, obsolete-Python-boundary-with-replacement.
A filename deletion or one smoke test cannot stand in for every parameterized case.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
cmake --build build/native-release --target bike_unit_tests bike_property_tests
ctest --test-dir build/native-release -L 'unit|property|reference' --output-on-failure --no-tests=error
build/native-release/bin/bike-test-ledger --manifest migration/test-port-map.json --require-complete
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T52
ctest --test-dir build/native-release -R '^migration_T52$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T52` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T52` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Use the supplied AST symbol map and T01 collected nodes. Split task execution into one test-file-sized reviewed change per commit, with the mapped numerical owner reviewing the replacement. Preserve each distinct assertion, parameter combination, malformed input and boundary condition.
  - [ ] 2. Convert pytest fixtures to owned C++ fixture builders, pytest.raises to typed exception assertions and NumPy bit comparisons to T02 binary fixtures. Never use assert for Release tests.
  - [ ] 3. For Numba tests, retain the behavior/precision cases as C++ tests and retire only interpreter/JIT-specific checks with an explicit replacement reason. Differential tests compare current native output to captured reference artifacts, not one native implementation to itself.
  - [ ] 4. Port randomized/property cases with fixed seeds, logged reproductions and bounded workloads. Do not change source generation distributions until the legacy cases are captured. Add independent invariants such as conservation, monotonicity and idempotence where source tests already express them.
  - [ ] 5. Create the native bike-test-ledger validator used by this task. It checks schema, path ownership, unique collected IDs, executable registration and evidence. It fails on unassigned IDs, missing replacements or unsupported retirement reasons.
  - [ ] 6. Keep the separate project/tests/reference/test_constraint_mode_search.py accounted for even when pytest default collection omits it. Its assertions need native coverage or a content-level duplication proof.
  - [ ] 7. The exact native test files in this package are generated from each assigned source test path, not a directory-level instruction. Use the qualified source_symbol/native_test_id pairs in test-port-map.json; T01 fills actual collection/parameter nodes before any source test retires.

### Mandatory additional regression cases

- [ ] All collected nodes have explicit dispositions; no count-only substitute for assertion coverage.
- [ ] Deliberately changed kernel output makes its ported test fail.
- [ ] Fixture corruption fails before comparisons; all tests pass under Release NDEBUG.
- [ ] No surviving pytest, unittest, Hypothesis, NumPy or Numba requirement in native tests.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T52$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- migration/test-port-map.json native/tests/support/property_cases.hpp native/tests/integration/test_rider_welds.cpp native/tests/reference/project_test_constraint_mode_search.cpp native/tests/reference/test_acceptance_interval_capture.cpp native/tests/reference/test_alpine_track.cpp native/tests/reference/test_articulated_config.cpp native/tests/reference/test_assist_controller.cpp native/tests/reference/test_attachment_budget.cpp native/tests/reference/test_attachment_wrench.cpp native/tests/reference/test_balance_monitor.cpp native/tests/reference/test_cadence_shifter.cpp native/tests/reference/test_closed_form_rider.cpp native/tests/reference/test_contact_safety.cpp native/tests/reference/test_control_clock.cpp native/tests/reference/test_coupled_rider_task.cpp native/tests/reference/test_crank_split.cpp native/tests/reference/test_distributed_tire_reference.cpp native/tests/reference/test_divergence_horizon.cpp native/tests/reference/test_drivetrain_policy_clone.cpp native/tests/reference/test_energy_ledger.cpp native/tests/reference/test_episode_compare.cpp native/tests/reference/test_frozen_force_arrays.cpp native/tests/reference/test_golden_episode.cpp native/tests/reference/test_golden_episode_v2.cpp native/tests/reference/test_hud_columns.cpp native/tests/reference/test_joint_strength.cpp native/tests/reference/test_lean_limit.cpp native/tests/reference/test_leg_loop.cpp native/tests/reference/test_monitor_strictness.cpp native/tests/reference/test_motor_freewheel_topology.cpp native/tests/reference/test_motor_profile.cpp native/tests/reference/test_native_attachment_wrench.cpp native/tests/reference/test_native_benchmark.cpp native/tests/reference/test_native_brake_resistance.cpp native/tests/reference/test_native_clock_contracts.cpp native/tests/reference/test_native_cruise.cpp native/tests/reference/test_native_distributed_stands.cpp native/tests/reference/test_native_distributed_tire.cpp native/tests/reference/test_native_drive_policies.cpp native/tests/reference/test_native_drivetrain.cpp native/tests/reference/test_native_encode_columns.cpp native/tests/reference/test_native_numeric_completion.cpp native/tests/reference/test_native_numeric_domains.cpp native/tests/reference/test_native_pneumatic_brush.cpp native/tests/reference/test_native_pneumatic_carcass.cpp native/tests/reference/test_native_pneumatic_geometry.cpp native/tests/reference/test_native_pneumatic_model.cpp native/tests/reference/test_native_pneumatic_tire.cpp native/tests/reference/test_native_pneumatic_wire.cpp native/tests/reference/test_native_rider_contact_math.cpp native/tests/reference/test_native_rider_contacts.cpp native/tests/reference/test_native_rider_forces.cpp native/tests/reference/test_native_rider_intent.cpp native/tests/reference/test_native_spindle_controller.cpp native/tests/reference/test_native_static_brake.cpp native/tests/reference/test_native_stepper.cpp native/tests/reference/test_native_suspension.cpp native/tests/reference/test_native_tire.cpp native/tests/reference/test_numba_kernels.cpp native/tests/reference/test_pedal_waveform.cpp native/tests/reference/test_pedaling_policy.cpp native/tests/reference/test_period_buffer.cpp native/tests/reference/test_pinned_topology.cpp native/tests/reference/test_planar_arms.cpp native/tests/reference/test_playback_clock.cpp native/tests/reference/test_pneumatic_tire_config.cpp native/tests/reference/test_pneumatic_tire_forces.cpp native/tests/reference/test_realistic_pedelec_acceptance.cpp native/tests/reference/test_recorder_columns.cpp native/tests/reference/test_reference_monitor.cpp native/tests/reference/test_ride_backend_cli.cpp native/tests/reference/test_ride_plots_html_markers.cpp native/tests/reference/test_rider_allocation.cpp native/tests/reference/test_rider_dynamics.cpp native/tests/reference/test_rider_information_boundary.cpp native/tests/reference/test_savage_track_surface.cpp native/tests/reference/test_seated_climb_surge.cpp native/tests/reference/test_seated_pedaling_cycle.cpp native/tests/reference/test_seated_posture_program.cpp native/tests/reference/test_seated_topology.cpp native/tests/reference/test_sensor_noise.cpp native/tests/reference/test_system_momentum_drive.cpp native/tests/reference/test_terrain_placement_regression.cpp native/tests/reference/test_tyre_road_profile.cpp native/tests/reference/test_welded_profile_config.cpp migration/python-removal.json
git commit -m "feat(cpp): T52 port every numerical, unit and property assertion out of pytest"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T53: Port integration, frontend, replay and failure-path test suites

**Dependencies:** T46, T49, T51, T52

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/tests/support/process_fixture.cpp` |
| Create | `migration/test-port-map.json` |
| Create | `native/tests/integration/test_async_close.cpp` |
| Create | `native/tests/integration/test_batch.cpp` |
| Create | `native/tests/integration/test_replay_playback.cpp` |
| Create | `native/tests/reference/test_equilibrium_state_file.cpp` |
| Create | `native/tests/reference/test_frontend_windows.cpp` |
| Create | `native/tests/reference/test_measure_realtime_report.cpp` |
| Create | `native/tests/reference/test_native_atomicity.cpp` |
| Create | `native/tests/reference/test_native_check_tools.cpp` |
| Create | `native/tests/reference/test_native_contract_checks.cpp` |
| Create | `native/tests/reference/test_native_engine_errors.cpp` |
| Create | `native/tests/reference/test_native_input_contracts.cpp` |
| Create | `native/tests/reference/test_native_loader.cpp` |
| Create | `native/tests/reference/test_native_recording_lifecycle.cpp` |
| Create | `native/tests/reference/test_native_recording_retention.cpp` |
| Create | `native/tests/reference/test_native_replay_recording_v3.cpp` |
| Create | `native/tests/reference/test_native_replay_session.cpp` |
| Create | `native/tests/reference/test_native_research_capture.cpp` |
| Create | `native/tests/reference/test_native_research_environment.cpp` |
| Create | `native/tests/reference/test_native_research_failures.cpp` |
| Create | `native/tests/reference/test_native_research_frontend.cpp` |
| Create | `native/tests/reference/test_native_research_policy.cpp` |
| Create | `native/tests/reference/test_native_research_programs.cpp` |
| Create | `native/tests/reference/test_native_research_recording.cpp` |
| Create | `native/tests/reference/test_native_research_sensors.cpp` |
| Create | `native/tests/reference/test_native_ride_frontend.cpp` |
| Create | `native/tests/reference/test_native_runtime_accounting.cpp` |
| Create | `native/tests/reference/test_native_runtime_bootstrap.cpp` |
| Create | `native/tests/reference/test_native_runtime_commands.cpp` |
| Create | `native/tests/reference/test_native_runtime_config.cpp` |
| Create | `native/tests/reference/test_native_runtime_ownership.cpp` |
| Create | `native/tests/reference/test_native_runtime_performance_contract.cpp` |
| Create | `native/tests/reference/test_native_runtime_rollout.cpp` |
| Create | `native/tests/reference/test_native_runtime_step.cpp` |
| Create | `native/tests/reference/test_native_state_bundle.cpp` |
| Create | `native/tests/reference/test_native_state_fuzz.cpp` |
| Create | `native/tests/reference/test_native_state_restore.cpp` |
| Create | `native/tests/reference/test_native_view_contracts.cpp` |
| Create | `native/tests/reference/test_native_viewer_snapshot.cpp` |
| Create | `native/tests/reference/test_pedelec_replay_contract.cpp` |
| Create | `native/tests/reference/test_physical_view.cpp` |
| Create | `native/tests/reference/test_pinned_measurement.cpp` |
| Create | `native/tests/reference/test_pneumatic_physical_runtime.cpp` |
| Create | `native/tests/reference/test_realtime_deferred_checks.cpp` |
| Create | `native/tests/reference/test_realtime_gate.cpp` |
| Create | `native/tests/reference/test_realtime_measurement.cpp` |
| Create | `native/tests/reference/test_recording_analysis.cpp` |
| Create | `native/tests/reference/test_recording_analyst_cli.cpp` |
| Create | `native/tests/reference/test_recording_capture.cpp` |
| Create | `native/tests/reference/test_recording_catalog.cpp` |
| Create | `native/tests/reference/test_recording_cli.cpp` |
| Create | `native/tests/reference/test_recording_publication.cpp` |
| Create | `native/tests/reference/test_recording_report.cpp` |
| Create | `native/tests/reference/test_recording_storage.cpp` |
| Create | `native/tests/reference/test_recording_values.cpp` |
| Create | `native/tests/reference/test_replay_manifest_v2.cpp` |
| Create | `native/tests/reference/test_replay_recording_v3.cpp` |
| Create | `native/tests/reference/test_replay_session.cpp` |
| Create | `native/tests/reference/test_research_capture.cpp` |
| Create | `native/tests/reference/test_research_frontend.cpp` |
| Create | `native/tests/reference/test_research_recording_lifecycle.cpp` |
| Create | `native/tests/reference/test_research_sweep.cpp` |
| Create | `native/tests/reference/test_runtime_single_path.cpp` |

Read-only source authority (do not delete during the port):

- `tests/reference/_native_runtime_support.py:1-78`
- `tests/reference/_recording_support.py:1-668`
- `tests/reference/test_equilibrium_state_file.py:1-140`
- `tests/reference/test_frontend_windows.py:1-291`
- `tests/reference/test_measure_realtime_report.py:1-131`
- `tests/reference/test_native_atomicity.py:1-533`
- `tests/reference/test_native_check_tools.py:1-1042`
- `tests/reference/test_native_contract_checks.py:1-755`
- `tests/reference/test_native_engine_errors.py:1-867`
- `tests/reference/test_native_input_contracts.py:1-445`
- `tests/reference/test_native_loader.py:1-543`
- `tests/reference/test_native_recording_lifecycle.py:1-967`
- `tests/reference/test_native_recording_retention.py:1-515`
- `tests/reference/test_native_replay_recording_v3.py:1-29`
- `tests/reference/test_native_replay_session.py:1-49`
- `tests/reference/test_native_research_capture.py:1-62`
- `tests/reference/test_native_research_environment.py:1-351`
- `tests/reference/test_native_research_failures.py:1-69`
- `tests/reference/test_native_research_frontend.py:1-25`
- `tests/reference/test_native_research_policy.py:1-170`
- `tests/reference/test_native_research_programs.py:1-50`
- `tests/reference/test_native_research_recording.py:1-142`
- `tests/reference/test_native_research_sensors.py:1-154`
- `tests/reference/test_native_ride_frontend.py:1-78`
- `tests/reference/test_native_runtime_accounting.py:1-207`
- `tests/reference/test_native_runtime_bootstrap.py:1-211`
- `tests/reference/test_native_runtime_commands.py:1-248`
- `tests/reference/test_native_runtime_config.py:1-217`
- `tests/reference/test_native_runtime_ownership.py:1-281`
- `tests/reference/test_native_runtime_performance_contract.py:1-317`
- `tests/reference/test_native_runtime_rollout.py:1-208`
- `tests/reference/test_native_runtime_step.py:1-225`
- `tests/reference/test_native_state_bundle.py:1-519`
- `tests/reference/test_native_state_fuzz.py:1-116`
- `tests/reference/test_native_state_restore.py:1-421`
- `tests/reference/test_native_view_contracts.py:1-187`
- `tests/reference/test_native_viewer_snapshot.py:1-58`
- `tests/reference/test_pedelec_replay_contract.py:1-44`
- `tests/reference/test_physical_view.py:1-79`
- `tests/reference/test_pinned_measurement.py:1-85`
- `tests/reference/test_pneumatic_physical_runtime.py:1-111`
- `tests/reference/test_realtime_deferred_checks.py:1-90`
- `tests/reference/test_realtime_gate.py:1-17`
- `tests/reference/test_realtime_measurement.py:1-164`
- `tests/reference/test_recording_analysis.py:1-301`
- `tests/reference/test_recording_analyst_cli.py:1-124`
- `tests/reference/test_recording_capture.py:1-338`
- `tests/reference/test_recording_catalog.py:1-327`
- `tests/reference/test_recording_cli.py:1-126`
- `tests/reference/test_recording_publication.py:1-71`
- `tests/reference/test_recording_report.py:1-19`
- `tests/reference/test_recording_storage.py:1-209`
- `tests/reference/test_recording_values.py:1-121`
- `tests/reference/test_replay_manifest_v2.py:1-201`
- `tests/reference/test_replay_recording_v3.py:1-113`
- `tests/reference/test_replay_session.py:1-140`
- `tests/reference/test_research_capture.py:1-160`
- `tests/reference/test_research_frontend.py:1-30`
- `tests/reference/test_research_recording_lifecycle.py:1-167`
- `tests/reference/test_research_sweep.py:1-30`
- `tests/reference/test_runtime_single_path.py:1-167`
- `tests/test_async_close.py:1-164`
- `tests/test_batch.py:1-96`
- `tests/test_replay_playback.py:1-137`

Declaration-level card: `migration/cards/T53.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T46` / `migration/api-index.md#t46`: Deliver the standalone bike-recording analyst command.
- `T49` / `migration/api-index.md#t49`: Port viewer controls, HUD, cameras and cooperative pacing.
- `T51` / `migration/api-index.md#t51`: Port batch jobs and research sweeps with deterministic process ownership.
- `T52` / `migration/api-index.md#t52`: Port every numerical, unit and property assertion out of pytest.

**Produces:**

```cpp
namespace bike::tests {
struct FaultPoint { std::string name; std::uint64_t occurrence; };
// Fault injection is compiled into test-only collaborators, not enabled by
// an undocumented production environment variable or user-controlled metadata.
}
// Test executables register stable CTest IDs recorded in test-port-map.json.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -L 'integration|frontend|recording|replay' --output-on-failure --no-tests=error
ctest --test-dir build/native-release -R 'engine_error|atomicity|publication|close|reset' --output-on-failure --no-tests=error
build/native-release/bin/bike-test-ledger --manifest migration/test-port-map.json --require-complete
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T53
ctest --test-dir build/native-release -R '^migration_T53$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T53` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T53` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port all Python runtime ownership, view contract, bootstrap/state fuzz, loader/provenance, failure atomicity, input-domain, CLI, capture/publication and replay tests to the native API/process seam.
  - [ ] 2. Retain the exact stop/yield/commit/reset/poisoned-state boundaries. Validate committed prefixes and failed suffixes after every injected failure; do not assume a thrown exception means no partial state change.
  - [ ] 3. Replace Python monkeypatching with narrow dependency injection for clock, filesystem, publisher, policy and engine test seams. Production physics still uses one execution path; a test seam must not become a second runtime.
  - [ ] 4. Exercise process-level crash/signals and actual closed DuckDB artifacts, not only mocked calls. Reader tests never import policies. Replay tests preserve execution identity and first-mismatch diagnostics.
  - [ ] 5. Port async close, playback and batch tests from tests/ plus all relevant tests/reference modules. UI-independent lifecycle tests run headlessly; genuine display tests remain separately labeled with supported-machine evidence.
  - [ ] 6. Validate the retirement of Python-boundary-only checks individually: replace dtype/array alias checks with equivalent native ownership/span/domain assertions and preserve core semantic failures.
  - [ ] 7. The exact native test files in this package are generated from each assigned source test path, not a directory-level instruction. Use the qualified source_symbol/native_test_id pairs in test-port-map.json; T01 fills actual collection/parameter nodes before any source test retires.

### Mandatory additional regression cases

- [ ] Engine fatal errors, nested error scopes, use-after-poison and immutable-prefix snapshots.
- [ ] Invalid strides become native span/shape/size ownership checks rather than disappearing.
- [ ] Repeated finalize/reset/close and policy exceptions leave deterministic state.
- [ ] A test run selecting zero cases fails the release gate.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T53$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/tests/support/process_fixture.cpp migration/test-port-map.json native/tests/integration/test_async_close.cpp native/tests/integration/test_batch.cpp native/tests/integration/test_replay_playback.cpp native/tests/reference/test_equilibrium_state_file.cpp native/tests/reference/test_frontend_windows.cpp native/tests/reference/test_measure_realtime_report.cpp native/tests/reference/test_native_atomicity.cpp native/tests/reference/test_native_check_tools.cpp native/tests/reference/test_native_contract_checks.cpp native/tests/reference/test_native_engine_errors.cpp native/tests/reference/test_native_input_contracts.cpp native/tests/reference/test_native_loader.cpp native/tests/reference/test_native_recording_lifecycle.cpp native/tests/reference/test_native_recording_retention.cpp native/tests/reference/test_native_replay_recording_v3.cpp native/tests/reference/test_native_replay_session.cpp native/tests/reference/test_native_research_capture.cpp native/tests/reference/test_native_research_environment.cpp native/tests/reference/test_native_research_failures.cpp native/tests/reference/test_native_research_frontend.cpp native/tests/reference/test_native_research_policy.cpp native/tests/reference/test_native_research_programs.cpp native/tests/reference/test_native_research_recording.cpp native/tests/reference/test_native_research_sensors.cpp native/tests/reference/test_native_ride_frontend.cpp native/tests/reference/test_native_runtime_accounting.cpp native/tests/reference/test_native_runtime_bootstrap.cpp native/tests/reference/test_native_runtime_commands.cpp native/tests/reference/test_native_runtime_config.cpp native/tests/reference/test_native_runtime_ownership.cpp native/tests/reference/test_native_runtime_performance_contract.cpp native/tests/reference/test_native_runtime_rollout.cpp native/tests/reference/test_native_runtime_step.cpp native/tests/reference/test_native_state_bundle.cpp native/tests/reference/test_native_state_fuzz.cpp native/tests/reference/test_native_state_restore.cpp native/tests/reference/test_native_view_contracts.cpp native/tests/reference/test_native_viewer_snapshot.cpp native/tests/reference/test_pedelec_replay_contract.cpp native/tests/reference/test_physical_view.cpp native/tests/reference/test_pinned_measurement.cpp native/tests/reference/test_pneumatic_physical_runtime.cpp native/tests/reference/test_realtime_deferred_checks.cpp native/tests/reference/test_realtime_gate.cpp native/tests/reference/test_realtime_measurement.cpp native/tests/reference/test_recording_analysis.cpp native/tests/reference/test_recording_analyst_cli.cpp native/tests/reference/test_recording_capture.cpp native/tests/reference/test_recording_catalog.cpp native/tests/reference/test_recording_cli.cpp native/tests/reference/test_recording_publication.cpp native/tests/reference/test_recording_report.cpp native/tests/reference/test_recording_storage.cpp native/tests/reference/test_recording_values.cpp native/tests/reference/test_replay_manifest_v2.cpp native/tests/reference/test_replay_recording_v3.cpp native/tests/reference/test_replay_session.cpp native/tests/reference/test_research_capture.cpp native/tests/reference/test_research_frontend.cpp native/tests/reference/test_research_recording_lifecycle.cpp native/tests/reference/test_research_sweep.cpp native/tests/reference/test_runtime_single_path.cpp migration/python-removal.json
git commit -m "feat(cpp): T53 port integration, frontend, replay and failure-path test suites"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T54: Port validation rigs, calibration and physical release evidence

**Dependencies:** T29, T30, T31, T37, T50, T52

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/validation/rigs.cpp` |
| Create | `native/src/validation/tire.cpp` |
| Create | `native/src/validation/rider.cpp` |
| Create | `native/src/validation/drivetrain.cpp` |
| Create | `native/src/validation/acceptance.cpp` |
| Create | `native/src/validation/evidence.cpp` |
| Create | `native/src/tools/validate_main.cpp` |
| Create | `native/src/validation/acceptance.hpp` |
| Create | `native/tests/migration/test_t54.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/physics/tire_calibration.py:1-84`
- `src/bike_sim/physics/tire_curve.py:1-92`
- `src/bike_sim/validation/__init__.py:1-1`
- `src/bike_sim/validation/antiwheelie_bench.py:1-133`
- `src/bike_sim/validation/antiwheelie_reports.py:1-70`
- `src/bike_sim/validation/benchmarks.py:1-240`
- `src/bike_sim/validation/climbing_benchmark.py:1-166`
- `src/bike_sim/validation/climbing_evidence.py:1-111`
- `src/bike_sim/validation/contact_acceptance.py:1-82`
- `src/bike_sim/validation/contact_manifold_rigs.py:1-173`
- `src/bike_sim/validation/contact_resolution.py:1-95`
- `src/bike_sim/validation/datasets.py:1-237`
- `src/bike_sim/validation/drive_suspension_rig.py:1-163`
- `src/bike_sim/validation/environment.py:1-82`
- `src/bike_sim/validation/experiment_acceptance.py:1-44`
- `src/bike_sim/validation/fidelity_sweep.py:1-409`
- `src/bike_sim/validation/grade_feasibility.py:1-20`
- `src/bike_sim/validation/load_transfer.py:1-71`
- `src/bike_sim/validation/plant_prefix.py:1-43`
- `src/bike_sim/validation/plant_release.py:1-28`
- `src/bike_sim/validation/plant_torque_rig.py:1-40`
- `src/bike_sim/validation/reference_release.py:1-289`
- `src/bike_sim/validation/ride_cases.py:1-239`
- `src/bike_sim/validation/rider_cases.py:1-54`
- `src/bike_sim/validation/rider_factor_matrix.py:1-40`
- `src/bike_sim/validation/rider_replay.py:1-295`
- `src/bike_sim/validation/rider_work.py:1-10`
- `src/bike_sim/validation/rigs.py:1-194`
- `src/bike_sim/validation/system_momentum.py:1-128`
- `src/bike_sim/validation/tire_characterization.py:1-42`

Declaration-level card: `migration/cards/T54.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T29` / `migration/api-index.md#t29`: Port remaining rider dynamics and coupled task controllers.
- `T30` / `migration/api-index.md#t30`: Implement flat/weld/spring rider supports and missing physical modes.
- `T31` / `migration/api-index.md#t31`: Port legacy tire backends and nonphysical ride integration.
- `T37` / `migration/api-index.md#t37`: Port research metrics, validity and experiment evidence.
- `T50` / `migration/api-index.md#t50`: Replace Matplotlib and Python Plotly construction with native artifact generation.
- `T52` / `migration/api-index.md#t52`: Port every numerical, unit and property assertion out of pytest.

**Produces:**

```cpp
namespace bike::validation {
struct Verdict { bool accepted; bike::io::Value evidence; };
Verdict run_case(std::string_view case_id, const std::filesystem::path& configuration);
}
// Names and requirements come from the supplied validation modules and datasets;
// the dispatch registry lists every retained validation operation.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t54.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "validation/acceptance.hpp"
#include "io/json.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const auto result = bike::validation::run_case(
      "tire-characterization", bt::fixture("validation/tire/config.toml"));
    bt::golden_text("validation/tire/verdict.json",
      bike::io::canonical_json(result.evidence));
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T54
ctest --test-dir build/native-release -R '^migration_T54$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T54` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T54` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port all 28 validation modules, not just the two easiest rigs: contact manifolds/resolution/safety, tire characterization, drivetrain torque/suspension, antiwheelie, system momentum, climbing, load transfer, rider factors/work/replay and plant/reference release checks.
  - [ ] 2. Preserve scenario parameters, units, validity thresholds, diagnostic invalid-prefix treatment, benchmark datasets and report fields. A new C++ runner must not turn historical failed evidence into accepted evidence.
  - [ ] 3. Port tire_calibration.py and supporting curve/NNLS/sag fitting paths with T23/T28 numerical interfaces. Keep synthetic profiles identified as synthetic; a software rewrite adds no new empirical calibration claim.
  - [ ] 4. Expose a native bike-validate CLI with a registry generated from an explicit C++ table, not a Python module scan. Supply discover/list/run commands for developer use without changing public nine-command behavior.
  - [ ] 5. Keep short CI gates distinct from long qualification scenarios. Reproduce approved thresholds from source; duration changes require a recorded reason and cannot hide unstable tails.
  - [ ] 6. Output structured evidence and offline reports using T05/T50. Include input hashes, solver/provider identity, case parameters, actual steps and first failure.

### Mandatory additional regression cases

- [ ] Every source validation callable is mapped to a native case or support function.
- [ ] Known invalid control fixtures remain rejected.
- [ ] Tire/contact/drive/rider conservation and dimensional checks.
- [ ] Reports distinguish model validity, numerical parity and runtime performance.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T54$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/validation/rigs.cpp native/src/validation/tire.cpp native/src/validation/rider.cpp native/src/validation/drivetrain.cpp native/src/validation/acceptance.cpp native/src/validation/evidence.cpp native/src/tools/validate_main.cpp native/src/validation/acceptance.hpp native/tests/migration/test_t54.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T54 port validation rigs, calibration and physical release evidence"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T55: Port performance, diagnostic and native-benchmark developer tools

**Dependencies:** T25, T37, T44, T54, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/tools/golden_episode_main.cpp` |
| Create | `native/src/tools/episode_compare_main.cpp` |
| Create | `native/src/tools/divergence_horizon_main.cpp` |
| Create | `native/src/tools/measure_realtime_main.cpp` |
| Create | `native/src/tools/step_profile_main.cpp` |
| Create | `native/src/tools/diag_main.cpp` |
| Create | `native/src/tools/finalize_bench_main.cpp` |
| Create | `native/src/tools/benchmark_main.cpp` |
| Create | `native/src/tools/state_format.hpp` |
| Create | `native/src/tools/state_format.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/realtime_measurement.py:1-196`
- `tools/diag_run.py:1-35`
- `tools/divergence_horizon.py:1-164`
- `tools/episode_compare.py:1-125`
- `tools/golden_episode.py:1-342`
- `tools/measure_realtime.py:1-343`
- `tools/proto_native_bench/bench.py:1-631`
- `tools/proto_native_bench/dump_model.py:1-113`
- `tools/proto_native_bench/state_format.py:1-577`
- `tools/recording_finalize_bench.py:1-201`
- `tools/step_profile.py:1-63`

Declaration-level card: `migration/cards/T55.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T25` / `migration/api-index.md#t25`: Construct standalone native rides from user configuration.
- `T37` / `migration/api-index.md#t37`: Port research metrics, validity and experiment evidence.
- `T44` / `migration/api-index.md#t44`: Port checked replay and versioned legacy compatibility.
- `T54` / `migration/api-index.md#t54`: Port validation rigs, calibration and physical release evidence.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```text
struct BenchmarkSummary {
  std::uint64_t requested_steps, completed_steps;
  double simulated_seconds, wall_seconds;
  bool physically_valid;
};
// All report arithmetic uses actual completed steps. State interchange preserves
// tools/proto_native_bench/state_format.py section ordering/checksums/dimensions.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
build/native-release/bin/bike-golden-episode --help
build/native-release/bin/bike-episode-compare --help
build/native-release/bin/bike-measure-realtime --help
ctest --test-dir build/native-release -R migration_T55 --output-on-failure --no-tests=error
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T55
ctest --test-dir build/native-release -R '^migration_T55$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T55` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T55` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port tools/golden_episode.py, episode_compare.py, divergence_horizon.py, measure_realtime.py, step_profile.py, diag_run.py and recording_finalize_bench.py; preserve each captured developer CLI and report schema or document an explicit native executable-name mapping.
  - [ ] 2. Port tools/proto_native_bench/bench.py, dump_model.py and state_format.py. Reuse existing C++ benchmark kernels and preserve magic/version, endian rules, CRC64, section sizes, alignments and malformed-input rejection. Generate model/state assets natively via T25.
  - [ ] 3. Measure cold startup, model generation, equilibrium, warmed stepping, allocations/retained memory, no-record/record runs, sensor/policy workloads, encoding and finalization separately. Include actual valid simulated duration and validity state.
  - [ ] 4. Use fixture-based correctness/identity gates before comparing timings. Keep CPU/SDK/compiler/provider/config provenance and pin the same workload; no performance claim from an invalid truncated run or a four-step diagnostic prefix.
  - [ ] 5. Use monotonic timing, explicit warmups and repeated measurements; report distribution/variance without setting speculative speed targets. The release envelope is approved from T01 baseline on supported hardware.
  - [ ] 6. Port native_coverage.py report orchestration to T57 rather than bundling it into physics benchmarks. Remove Python dependencies from every developer command after its native replacement passes.

### Mandatory additional regression cases

- [ ] Binary state format round trips with old writer/reader during transition.
- [ ] Corrupt CRC, duplicate/truncated sections, wrong dimensions and incompatible engine identity.
- [ ] Zero valid steps/divergence yields an honest invalid or inconclusive performance result.
- [ ] Finalize benchmark accounts separately for recording size and summary/report work.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T55$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/tools/golden_episode_main.cpp native/src/tools/episode_compare_main.cpp native/src/tools/divergence_horizon_main.cpp native/src/tools/measure_realtime_main.cpp native/src/tools/step_profile_main.cpp native/src/tools/diag_main.cpp native/src/tools/finalize_bench_main.cpp native/src/tools/benchmark_main.cpp native/src/tools/state_format.hpp native/src/tools/state_format.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T55 port performance, diagnostic and native-benchmark developer tools"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T56: Replace Python native-check orchestration and inline interpreter scripts

**Dependencies:** T03, T06, T02

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/tools/checks_main.cpp` |
| Create | `native/src/tools/check_context.cpp` |
| Create | `native/src/tools/check_frontends.cpp` |
| Create | `native/src/tools/check_contracts.cpp` |
| Create | `native/src/tools/check_reports.cpp` |
| Create | `native/cmake/Checks.cmake` |
| Create | `tools/native_common.sh` |
| Create | `tools/run_native_checks.sh` |
| Create | `native/src/tools/python_audit_main.cpp` |
| Create | `migration/no-python-policy.json` |

Read-only source authority (do not delete during the port):

- `native/CMakeLists.txt:1-978`
- `src/bike_sim/native/contracts.py:1-272`
- `tools/check_native_contracts.py:1-513`
- `tools/native_checks.py:1-1944`
- `tools/native_config.py:1-15`
- `tools/native_schema.py:1-12`

Declaration-level card: `migration/cards/T56.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T03` / `migration/api-index.md#t03`: Build the application without finding Python or wheel directories.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.
- `T02` / `migration/api-index.md#t02`: Introduce a standalone contract-test harness and binary fixtures.

**Produces:**

```bash
bike-native-checks --kind frontends --build build/native-release
bike-native-checks --kind tidy --build build/native-release
bike-native-checks --kind analyzer --build build/native-release
bike-native-checks --kind cppcheck --build build/native-release
bike-native-checks --kind odr --build build/native-release
bike-native-checks --kind scripts --build build/native-release
bike-native-checks --kind headers --build build/native-release
bike-native-checks --kind diagnostic-controls --build build/native-release
bike-native-checks --kind context --build build/native-release
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
cmake --build build/native-release --target check_frontends check_tidy check_analyzer check_cppcheck check_odr check_scripts
build/native-release/bin/bike-native-checks --kind headers --build build/native-release
build/native-release/bin/bike-native-checks --kind diagnostic-controls --build build/native-release
build/native-release/bin/bike-native-checks --kind context --build build/native-release
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T56
ctest --test-dir build/native-release -R '^migration_T56$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T56` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T56` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port tools/native_checks.py, check_native_contracts.py, native_config.py and native_schema.py, preserving all nine sweep classes, diagnostic classification, baseline comparison, command provenance, generated contexts and report behavior.
  - [ ] 2. Retain analyzer execution-health failures even when a diagnostic family is report-only. Do not silently drop optional alpha analysis, suppress warnings, weaken hardening or replace checks with an empty success command.
  - [ ] 3. Parse compile_commands.json/native_check_context.json natively with T05. Resolve explicit tools/SDK/build paths, reject stale/mismatched build provenance and test quoting with paths containing spaces.
  - [ ] 4. Convert first-party inline Python in shell scripts to CMake or the native helper. Enumerate every executable Python invocation in migration/dependency-edges.json before edits, including command substitutions and heredocs.
  - [ ] 5. Keep shell scripts as transparent launchers where they only select profiles/environment. Declarative CMake/shell is retained; the goal is no Python, not gratuitously translating all configuration languages.
  - [ ] 6. Extend dependency rules to reject nanobind/Python/NumPy includes from pure-core libraries and prevent IO/GUI/database dependencies from leaking into hot kernels. Preserve existing contract-rule semantics and add specific native-only rules.
  - [ ] 7. Implement bike-python-audit modes source/package/documentation and negative controls here using the exact specification in T62. This provides the utility needed by T59/T60; T62 later enforces it over release artifacts. During transition source mode is expected to report remaining owned Python paths, not pass.

### Mandatory additional regression cases

- [ ] All existing test_native_check_tools.py and test_native_contract_checks.py cases translated.
- [ ] Missing/crashed analyzers, empty compile DB, stale context, malformed diagnostics and failed negative controls.
- [ ] Check commands invoke the intended built binary/library, not a stale extension.
- [ ] No first-party check requires Python or silently reaches a wheel directory.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T56$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/tools/checks_main.cpp native/src/tools/check_context.cpp native/src/tools/check_frontends.cpp native/src/tools/check_contracts.cpp native/src/tools/check_reports.cpp native/cmake/Checks.cmake tools/native_common.sh tools/run_native_checks.sh native/src/tools/python_audit_main.cpp migration/no-python-policy.json migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T56 replace python native-check orchestration and inline interpreter scripts"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T57: Deliver a native test runner, sanitizers and coverage provenance

**Dependencies:** T52, T53, T56, T55

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/tools/test_runner_main.cpp` |
| Create | `native/src/tools/coverage_main.cpp` |
| Create | `native/cmake/Sanitizers.cmake` |
| Create | `native/cmake/Coverage.cmake` |
| Modify | `tools/run_tests.sh` |
| Create | `native/tests/support/provenance.cpp` |

Read-only source authority (do not delete during the port):

- `conftest.py:1-5`
- `tests/reference/conftest.py:1-180`
- `tests/reference/native_loader.py:1-244`
- `tests/reference/native_safety_helpers.py:1-28`
- `tests/reference/native_test_reporter.py:1-32`
- `tools/native_coverage.py:1-253`
- `tools/run_tests.sh:1-194`

Declaration-level card: `migration/cards/T57.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T52` / `migration/api-index.md#t52`: Port every numerical, unit and property assertion out of pytest.
- `T53` / `migration/api-index.md#t53`: Port integration, frontend, replay and failure-path test suites.
- `T56` / `migration/api-index.md#t56`: Replace Python native-check orchestration and inline interpreter scripts.
- `T55` / `migration/api-index.md#t55`: Port performance, diagnostic and native-benchmark developer tools.

**Produces:**

```bash
bike-test-runner --profile quick --build build/native-release
bike-test-runner --profile native --build build/native-release
bike-test-runner --profile full --build build/native-release
// Native and full preserve the existing shared native preflight before tests.
// Every report names the exact executable, build context and fixture manifest.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
cmake -S . -B build/native-asan -DBIKE_ENABLE_ASAN=ON -DBIKE_BUILD_GUI=OFF
cmake --build build/native-asan
ctest --test-dir build/native-asan --output-on-failure --no-tests=error
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T57
ctest --test-dir build/native-release -R '^migration_T57$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T57` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T57` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Replace pytest selection/report plugins, native_loader.py, native_test_reporter.py and tools/native_coverage.py with native/CMake runner behavior and a native provenance report. Preserve quick/native/full names and captured selection meaning.
  - [ ] 2. Both existing native and full profiles already call preflight_native. Preserve that property when replacing the launcher, including all nine sweeps, direct CTest and contract checks. Run shared preflight once per profile invocation and serialize expensive gates on a machine.
  - [ ] 3. Provide ASan/UBSan builds and a separately qualified thread-sanitizer profile where supported by the compiler/dependencies. Do not combine incompatible sanitizers or require a falsely green platform-wide TSan result from unsupported third-party binaries.
  - [ ] 4. Use llvm-profdata/llvm-cov native tools with explicit executable lists and input profile validation. Reports must identify missing profiles, zero tests and stale binaries as failures, not successful zero coverage.
  - [ ] 5. Create a clean Python-free CI image/environment and run configure/build/tests there. Preserve a separate transition job for reference capture until T63; it is not part of the final native distribution.
  - [ ] 6. Retain check reports, test manifest, fixture hashes, sanitizer results and build identity as release evidence. Do not use coverage percentage alone to certify numerical parity.

### Mandatory additional regression cases

- [ ] Profile selection and empty-selection negative controls.
- [ ] Stale binary/fixture/context mismatch detected before claiming a pass.
- [ ] ASan/UBSan on ownership, failure/unwind, arrays, database/GUI resource teardown.
- [ ] Native tests/check/coverage complete with no Python executable present.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T57$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/tools/test_runner_main.cpp native/src/tools/coverage_main.cpp native/cmake/Sanitizers.cmake native/cmake/Coverage.cmake tools/run_tests.sh native/tests/support/provenance.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T57 deliver a native test runner, sanitizers and coverage provenance"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T58: Qualify Linux headless native support without changing macOS arithmetic silently

**Dependencies:** T04, T25, T54, T57

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/cmake/PlatformLinux.cmake` |
| Create | `native/cmake/PlatformApple.cmake` |
| Create | `native/src/numeric/provider_linux.cpp` |
| Create | `packaging/platform-matrix.json` |
| Create named fixture files from recipe (data) | `native/tests/platform/` |

Read-only source authority (do not delete during the port):

- `native/CMakeLists.txt:1-978`
- `pyproject.toml:1-51`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T04` / `migration/api-index.md#t04`: Preserve numerical providers and fatal-engine containment.
- `T25` / `migration/api-index.md#t25`: Construct standalone native rides from user configuration.
- `T54` / `migration/api-index.md#t54`: Port validation rigs, calibration and physical release evidence.
- `T57` / `migration/api-index.md#t57`: Deliver a native test runner, sanitizers and coverage provenance.

**Produces:**

```text
The supported release matrix initially requires:
- macOS on the architecture/provider captured by T01: existing strict parity.
- Linux x86_64 headless: native replacement for current offline wheel bundles.
No Windows or Linux GUI support is claimed without its own completed gates.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
cmake -S . -B build/linux-release -DBIKE_BUILD_GUI=OFF -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_NUMERIC_PROVIDER=linux
cmake --build build/linux-release
ctest --test-dir build/linux-release --output-on-failure --no-tests=error
build/linux-release/bin/bike-validate list
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T58
ctest --test-dir build/native-release -R '^migration_T58$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T58` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T58` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Separate Apple frameworks/SDK/install-name handling from Linux ELF/RPATH/shared-library discovery. Link a pinned native MuJoCo 3.12.0 library with the required TLS ABI verified on Linux, not a copied macOS dylib.
  - [ ] 2. Choose an explicit locked CBLAS/LAPACK/provider implementation for Linux after enumerating the Accelerate calls actually used. Preserve operation order where feasible; never claim cross-provider bitwise equality by default.
  - [ ] 3. Run the full numerical/physical/recording/replay suite with separate provider identity. Existing bitwise macOS fixtures remain unchanged; Linux-specific tolerances require per-field justification and approved physical/long-replay acceptance.
  - [ ] 4. Do not merge a failing platform by widening global epsilons. If a required replay execution profile is macOS-provider-specific, report incompatibility on Linux rather than falsify metadata; full cross-platform compatibility is a separate explicit gate.
  - [ ] 5. Test no-display and offline execution, dynamic loader resolution, CPU baseline flags and absence of host-wheel/RPATH leakage. Record actual minimum libc/OS/compiler requirements from produced binaries.
  - [ ] 6. The current packaging advertises Linux x86_64 wheel bundles, while the current native build is macOS-specific. Close this mismatch before declaring the Python-free replacement covers the prior install surface.

### Mandatory additional regression cases

- [ ] Native fresh-start ride/research, legacy modes and reader/export on Linux.
- [ ] Explicit unsupported provider/ABI failure before stepping.
- [ ] No .dylib/-framework/xcrun requirement on Linux.
- [ ] Platform matrix contains evidence paths rather than assumed portability.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T58$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/cmake/PlatformLinux.cmake native/cmake/PlatformApple.cmake native/src/numeric/provider_linux.cpp packaging/platform-matrix.json migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T58 qualify linux headless native support without changing macos arithmetic silently"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T59: Replace Python wheel installers with offline native distributions

**Dependencies:** T47, T50, T51, T55, T56, T57, T58

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `packaging/CMakeLists.txt` |
| Create | `packaging/manifest.schema.json` |
| Create | `packaging/install.sh` |
| Create | `native/src/tools/package_verify_main.cpp` |
| Create | `native/src/tools/install_main.cpp` |
| Create | `docs/installation.md` |

Read-only source authority (do not delete during the port):

- `tools/check_offline_sources.py:1-32`
- `tools/install_offline.py:1-141`

Declaration-level card: `migration/cards/T59.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T47` / `migration/api-index.md#t47`: Replace all remaining Python CLI entrypoints with installed native commands.
- `T50` / `migration/api-index.md#t50`: Replace Matplotlib and Python Plotly construction with native artifact generation.
- `T51` / `migration/api-index.md#t51`: Port batch jobs and research sweeps with deterministic process ownership.
- `T55` / `migration/api-index.md#t55`: Port performance, diagnostic and native-benchmark developer tools.
- `T56` / `migration/api-index.md#t56`: Replace Python native-check orchestration and inline interpreter scripts.
- `T57` / `migration/api-index.md#t57`: Deliver a native test runner, sanitizers and coverage provenance.
- `T58` / `migration/api-index.md#t58`: Qualify Linux headless native support without changing macOS arithmetic silently.

**Produces:**

```bash
bike-package-verify --root dist/bike-sim --manifest dist/bike-sim/manifest.json
bike-install --bundle dist/bike-sim --prefix install-test
// Runtime bundles contain native binaries, locked shared libraries, model/config
// assets, licenses, help and offline report resources; no wheels or interpreter.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
cmake --install build/native-release --prefix install-test
install-test/bin/bike-recording --help
install-test/bin/bike-ride --help
build/native-release/bin/bike-package-verify --root install-test --no-python
ctest --test-dir build/native-release -L package --output-on-failure --no-tests=error
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T59
ctest --test-dir build/native-release -R '^migration_T59$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T59` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T59` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Replace tools/install_offline.py and check_offline_sources.py and all wheel/uv-oriented packaging scripts with native artifact manifests/install verification. Preserve source/full/slim product intent only where captured; write an explicit old-to-new bundle mapping.
  - [ ] 2. Bundle required native shared libraries with correct macOS install names or Linux RPATH and licenses, plus configs/tracks/MJCF assets and offline plotting resources. Package no Python interpreter, libpython, .pyc/.pyo, wheels, NumPy/SciPy/PyArrow/nanobind or user Python plugins.
  - [ ] 3. Verify SHA-256, regular files, destination exclusivity/permissions and target platform before installing. Handle spaces, read-only/corrupt bundles and interrupted installs without destroying an existing installation.
  - [ ] 4. Build source distributions using only the pinned native toolchain/dependencies and no network. Audit transitive build tools: an upstream project that requires Python generation must supply licensed checked-in/generated artifacts or a documented native build path.
  - [ ] 5. Smoke-test every installed command from outside the source tree on clean supported systems, with Python absent and network disabled. Include a fresh-start simulation, one record/finalize/open/replay cycle, a batch and representative plot.
  - [ ] 6. Install version/license/third-party notices and produce an SBOM/dependency manifest. macOS signing/notarization is a release-owner operation with real credentials, not a fabricated agent-produced attestation.

### Mandatory additional regression cases

- [ ] No accidental imports/path dependencies on the build machine.
- [ ] Offline help plus actual run/report/recording behavior, not help-only smoke.
- [ ] Missing dependency/model resource, wrong architecture, checksum failure and upgrade collision.
- [ ] Full/slim replacement capabilities and Python-free source build proven separately.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T59$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- packaging/CMakeLists.txt packaging/manifest.schema.json packaging/install.sh native/src/tools/package_verify_main.cpp native/src/tools/install_main.cpp docs/installation.md migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T59 replace python wheel installers with offline native distributions"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T60: Update active documentation and agent contracts to the native workflow

**Dependencies:** T47, T54, T55, T56, T57, T59

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Modify | `README.md` |
| Modify | `AGENTS.md` |
| Create | `docs/native-workflow.md` |
| Create | `docs/python-to-cpp-compatibility.md` |
| Create | `migration/cli-contracts.json` |
| Create | `migration/command-map.json` |

Read-only source authority (do not delete during the port):

- `AGENTS.md:1-135`
- `README.md:1-17`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T47` / `migration/api-index.md#t47`: Replace all remaining Python CLI entrypoints with installed native commands.
- `T54` / `migration/api-index.md#t54`: Port validation rigs, calibration and physical release evidence.
- `T55` / `migration/api-index.md#t55`: Port performance, diagnostic and native-benchmark developer tools.
- `T56` / `migration/api-index.md#t56`: Replace Python native-check orchestration and inline interpreter scripts.
- `T57` / `migration/api-index.md#t57`: Deliver a native test runner, sanitizers and coverage provenance.
- `T59` / `migration/api-index.md#t59`: Replace Python wheel installers with offline native distributions.

**Produces:**

```text
migration/command-map.json maps every active old command to:
{old_command, native_command, behavior_notes, verification_id}.
Historical research logs are retained as evidence and marked historical;
active setup/run/test instructions contain no Python/uv/mjpython requirements.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
build/native-release/bin/bike-native-checks --kind scripts --build build/native-release
build/native-release/bin/bike-python-audit --root . --mode documentation
ctest --test-dir build/native-release -L documented-command --output-on-failure --no-tests=error
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T60
ctest --test-dir build/native-release -R '^migration_T60$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T60` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T60` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Update actual active README/AGENTS/design/run/install documents, shell examples and troubleshooting paths. Preserve physics safety constraints, one execution path, numerical rules and required check gates.
  - [ ] 2. Explain native policy ABI migration, old Python module:factory rejection, legacy recording execution-identity rules, config/output compatibility and supported platforms. Do not describe unsupported modes as removed without explicit user approval.
  - [ ] 3. Publish command substitutions for all nine entrypoints and native developer tools, expected environment/dependency roots and single-machine gate serialization.
  - [ ] 4. Mark historical Python results as historical reference evidence rather than editing them to pretend they were native. Documentation may mention Python historically; the executable audit must not mistake prose for a runtime dependency.
  - [ ] 5. Run every active documented command in the relevant clean/native package test environment. Use fixtures for data-dependent examples and record the output expected by the docs.
  - [ ] 6. Avoid claiming achieved speedups, zero allocations, cross-platform bitwise replay or model validity without the matching evidence from T54/T55/T58.

### Mandatory additional regression cases

- [ ] Active commands point to installed executable names and valid argument spellings.
- [ ] No broken local links or stale setup instructions requiring uv/mjpython.
- [ ] Historical evidence provenance survives.
- [ ] Compatibility changes and extension migration are explicit.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T60$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- README.md AGENTS.md docs/native-workflow.md docs/python-to-cpp-compatibility.md migration/cli-contracts.json migration/command-map.json migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T60 update active documentation and agent contracts to the native workflow"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
