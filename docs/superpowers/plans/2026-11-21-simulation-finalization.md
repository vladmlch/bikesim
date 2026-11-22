# Unified Flat Simulation Finalization & Merge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (inline execution) or superpowers:subagent-driven-development (if user explicitly requests subagents) to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete all remaining non-GPU, non-3D simulation workstreams (Track Surface Stage A + `pneumatic` `crr`/energy closure, bans & fork-rebound audits, Implicit-FSM artifacts & physics-critical boundary fixes, required profile fields, 200 Nm static-brake ceiling, 30/30 % default suspension sag, and CPU `BGTRAJ01` ghost replay), run one consolidated golden/pose refresh and one full `bash tools/run_tests.sh full` gate on `darwin-arm64`, fast-forward `main`, and push `origin/main`.

**Architecture:** Executes sequentially in `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18` (branch `feat/surface-stage-a-2026-11-18`, where the verified SSD backup and `darwin-arm64` Mach-O `build/native-release` are already prepared) across 9 focused tasks grouped into 6 waves. Tasks 1–7 land self-contained commits verified by targeted CTest binaries and incremental TU sweeps without premature golden churn; Task 8 refreshes `poses/*.npz` and `darwin-arm64` goldens once and runs `bash tools/run_tests.sh full`; Task 9 verifies path disjointness against `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean`, fast-forwards `main`, and pushes `origin/main`.

**Tech Stack:** C++23, AppleClang / LLVM (`Darwin.cmake` `release` preset), MuJoCo 3.12, Accelerate/CBLAS, CMake + Ninja + CTest + `bike-native-checks`.

**Spec:**
- `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18/docs/superpowers/specs/2026-11-21-simulation-finalization-design.md`
- `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18/docs/superpowers/specs/2026-11-18-surface-stage-a-design.md`

## Global Constraints

- **Working tree:** `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18` (`WT`). Never rewrite existing commits `37946646..c98d3bfd`.
- **Out of scope:** GPU/FP32/Metal (`native/src/gpu/`, `tools/colab_gpu/`, `feat/gpu-*`, `feat/sp1-*`), 3D (`3d-new`, `spatial-3d`), rejected experiments (`exp/leg-jitter-2026-11-08`, `wip/pedaling-rhythm`), and Linux container runs (`tools/linux/container.sh` is disabled by `PLAN-CHANGE GUARD`).
- **Realtime safety (`BIKE_NONBLOCKING`):** Zero heap allocations inside `DistributedTireWriter::try_compute_into`, `PneumaticTireWriter::try_compute_into`, and `DiscretisedBrush::step_heterogeneous`.
- **Single golden regeneration pass:** Do NOT re-record broad `darwin-arm64` goldens or `savage_pneumatic.npz` until Task 8 (after all physics/pose/brake/profile changes in Tasks 1–7 are complete and acceptance rides pass).
- **Acceptance invariants (`simulation-decisions.md` #34):** Never relax acceptance limits (`-6 N` rider support, `1750 N` saddle compression, `300 N` hand force, `0.05` energy residual) to make a test pass; if an acceptance scenario fails under 30/30 % sag or the 200 Nm static-brake cap, stop and report exact measurements to the user.
- **No destructive operations:** Never run `rm -rf`, `git reset --hard`, `git clean`, or `git stash` in any checkout.

---

### Task 1: Commit Verified `DistributedTireWriter` & Stage A Task 1–2 Minors (Wave 1A)

**Files:**
- Modify (already in working tree): `native/src/writers/distributed_tire.hpp`
- Modify (already in working tree): `native/src/writers/distributed_tire.cpp`
- Modify (already in working tree): `native/src/terrain/alpine.cpp`
- Modify (already in working tree): `native/src/runtime/project_config.cpp`
- Modify (already in working tree): `native/src/writers/tire_common.hpp`
- Modify (already in working tree): `native/src/writers/tire_common.cpp`
- Test (already in working tree): `native/tests/reference/test_native_distributed_tire.cpp`
- Test (already in working tree): `native/tests/reference/test_native_input_contracts.cpp`
- Test (already in working tree): `native/tests/terrain/test_generators.cpp`
- Test (already in working tree): `native/tests/terrain/test_surface_eval.cpp`
- Modify (already in working tree): `tests/fixtures/golden/darwin-arm64/native_distributed_tire/distributed_qfrc_bitwise.bin`
- Modify (already in working tree): `tests/fixtures/golden/darwin-arm64/native_distributed_tire/env_request/initial_state.json`
- Modify (already in working tree): `tests/fixtures/golden/darwin-arm64/native_numeric_completion/tire_channel_parity_across_backends.bin`

**Interfaces:**
- Consumes: `tirecommon::eval_surface`, `tirecommon::sample_mu`, `tirecommon::load_sensitivity_factor` from `a0bb59e1`.
- Produces: Committed `DistributedTireWriter` with `sqrt(r_k)` energy-invariant shear scaling, `crr_delta` rolling moment, finite-output overflow guards, and all 5 closed Task 1–2 review minors.

- [ ] **Step 1: Build and run targeted tests for `DistributedTireWriter` and Stage A minors**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target \
  terrain_generators terrain_surface_eval \
  reference.native_input_contracts \
  reference.native_distributed_tire \
  reference.distributed_tire_reference \
  reference.native_numeric_completion
ctest --test-dir "$WT/build/native-release" -R "^(terrain_generators|terrain_surface_eval|reference\.native_input_contracts|reference\.native_distributed_tire|reference\.distributed_tire_reference|reference\.native_numeric_completion)$" --output-on-failure
```
Expected: 6/6 tests PASS.

- [ ] **Step 2: Run incremental static checks (`cppcheck`, `frontends`, `tidy`) on the 10 touched C++ files**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target bike-native-checks
LIST="$(mktemp)"
git -C "$WT" diff --name-only | awk -v wt="$WT" '/^native\/.*\.(cpp|hpp)$/ {print wt "/" $0}' > "$LIST"
for kind in cppcheck frontends tidy; do
  "$WT/build/native-release/bin/bike-native-checks" --kind "$kind" --build "$WT/build/native-release" --only-sources "$LIST"
done
rm -f "$LIST"
```
Expected: `cppcheck`, `frontends`, and `tidy` exit 0 with 0 diagnostics.

- [ ] **Step 3: Commit `DistributedTireWriter` and the 5 closed Task 1–2 minors**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
git -C "$WT" add \
  native/src/terrain/alpine.cpp \
  native/src/runtime/project_config.cpp \
  native/src/writers/tire_common.hpp \
  native/src/writers/tire_common.cpp \
  native/tests/terrain/test_generators.cpp \
  native/tests/terrain/test_surface_eval.cpp \
  native/tests/reference/test_native_input_contracts.cpp
git -C "$WT" commit -m "$(cat <<'EOF'
fix(terrain,writers): close Stage A Task 1-2 review minors and high-load asymptote

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
git -C "$WT" add \
  native/src/writers/distributed_tire.hpp \
  native/src/writers/distributed_tire.cpp \
  native/tests/reference/test_native_distributed_tire.cpp \
  tests/fixtures/golden/darwin-arm64/native_distributed_tire \
  tests/fixtures/golden/darwin-arm64/native_numeric_completion
git -C "$WT" commit -m "$(cat <<'EOF'
feat(distributed-tire): add surface shear scaling, load sensitivity and overflow-safe crr_delta moment

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```
Expected: `git -C "$WT" status --porcelain` is empty.

---

### Task 2: `Pneumatic` Radial Energy Closure, Ray-Resolved Surface Physics & `crr = 0.0` (Wave 1B)

**Files:**
- Modify: `native/src/pneumatic/carcass.hpp`
- Modify: `native/src/pneumatic/carcass.cpp`
- Modify: `native/src/pneumatic/brush.hpp`
- Modify: `native/src/pneumatic/brush.cpp`
- Modify: `native/src/pneumatic/model.hpp`
- Modify: `native/src/pneumatic/model.cpp`
- Modify: `native/src/writers/pneumatic_tire.hpp`
- Modify: `native/src/writers/pneumatic_tire.cpp`
- Modify: `examples/research/viewer_physics_pneumatic.toml`
- Modify: `tests/fixtures/config/viewer_physics_pneumatic.toml`
- Modify: `tests/fixtures/config/resolved/pneumatic.json`
- Create: `native/tests/reference/test_pneumatic_energy_balance.cpp`
- Modify: `native/tests/reference/Phase3Ports.cmake`
- Modify: `native/tests/reference/test_pneumatic_tire_forces.cpp`
- Modify: `native/tests/reference/test_pneumatic_tire_oracle.cpp`
- Modify: `native/tests/reference/test_native_pneumatic_model.cpp`

**Interfaces:**
- Produces in `namespace biketyre`:
  ```cpp
  [[nodiscard]] double casing_chord_integral_m2(double deflection_m, double casing_radius_m) noexcept;
  [[nodiscard]] double ray_elastic_force_n(double deflection_m,
                                           const nativecfg::PneumaticTyreSpec &tyre,
                                           double ds_m) noexcept;
  [[nodiscard]] double ray_elastic_energy_j(double deflection_m,
                                            const nativecfg::PneumaticTyreSpec &tyre,
                                            double ds_m) noexcept;
  [[nodiscard]] double ray_maxwell_energy_j(double deflection_m, double elastic_prev_n,
                                            double maxwell_force_n,
                                            const nativecfg::PneumaticTyreSpec &tyre,
                                            double ds_m) noexcept;
  ```
- Produces `DiscretisedBrush::step_heterogeneous(...)` (`BIKE_NONBLOCKING`), `TyrePatchOutput::shear_energy_j`, `WheelOutputs::carcass_dissipated_power_w`, `WheelOutputs::rolling_crr`, world-Jacobian `m_roll` in `PneumaticTireWriter::evaluate_wheels`, and `[resistance] crr = 0.0` in `viewer_physics_pneumatic.toml`.

- [ ] **Step 1: Write `native/tests/reference/test_pneumatic_energy_balance.cpp` and register `reference.pneumatic_energy_balance` in `Phase3Ports.cmake`**

Include all 6 non-trivial tests incorporating the preflight rulings:
1. `test_air_chord_and_ray_elastic_quadrature()` — singularity-free quadrature $u = u_{\text{cap}} t^2$ across $d \in [0, \rho + 6\text{ mm}]$ and $C^1$ slope across $d = \rho$;
2. `test_maxwell_zero_carcass_stiffness_cap_inverse()` — verifies `carcass_stiffness_n_mm2 == 0.0` at saturation $d \ge \rho$ returns finite nonnegative `ray_maxwell_energy_j` (no `0/0` NaN);
3. `test_maxwell_hold_and_closed_cycle_passivity()` — compares independent `um_pre - um_post` directly against `evaluate_carcass`'s production `dissipated_power_w * dt` (`< 1e-14` J) and checks closed load-hold-unload-airborne passivity;
4. `test_single_rim_hysteresis_and_stored_energy_bitwise()` — verifies rim strike hysteresis is counted once (not 2×) and `PneumaticTireWriter::stored_energy(d)` equals `elastic_energy_j()` bitwise;
5. `test_pneumatic_world_rolling_moment_no_caliper_reaction()` — compares solved `qacc[root_pitch]` with `crr_delta > 0` vs `crr_delta == 0` on concentric free-spinning wheel (`|Δqacc[root_pitch]| <= 1e-12`) and checks rollback passivity;
6. `test_pneumatic_boundary_crossing_smoothness_and_stored_energy()` — advances wheel across `x = 5.0` from `x = 4.55` to `x = 5.45` (`> 3500` steps at `dt = 0.00025`, asserting full footprint before `4.75` and after `5.25`) for `blend_m = 0.0` and `0.4`;
7. `test_pneumatic_60m_mixed_surface_ride_energy_gate()` — runs 60 m mixed-surface (`hardpack` / `scree` / `wet`, `blend_m = 0.4`) normal-rider ride on `viewer_physics_pneumatic.toml` asserting `!first_failure`, `qpos[0] >= 60.0`, zero `model_violations`, `dissipated_power_w >= 0`, `brush_loss_j >= 0`, and `|residual_j| / energy_scale_j <= 0.05`.

- [ ] **Step 2: Verify `reference.pneumatic_energy_balance` fails to compile / fails RED before implementation**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target reference.pneumatic_energy_balance
```
Expected: FAIL (missing `casing_chord_integral_m2`, `ray_elastic_energy_j`, `ray_maxwell_energy_j`).

- [ ] **Step 3: Implement `Pneumatic` radial energy closure, `DiscretisedBrush::step_heterogeneous`, world `m_roll`, and `[resistance] crr = 0.0`**

1. Implement `casing_chord_integral_m2`, `ray_elastic_force_n`, `ray_elastic_energy_j`, and `ray_maxwell_energy_j` in `native/src/pneumatic/carcass.{hpp,cpp}` per `2026-11-18-surface-stage-a-design.md` §4.1–§4.2, handling `carcass_stiffness_n_mm2 == 0.0` at $\xi_e \ge f_e(\rho)$ by setting $\delta_{\text{eq}} = \rho$ without dividing by $k_c \cdot ds = 0$.
2. Remove the duplicate rim-hysteresis term in `carcass.cpp` / `pneumatic_tire.cpp` so rim-strike hysteresis is counted once, and wire `carcass_dissipated_power_w` into `WheelOutputs` and `PneumaticTireWriter`.
3. Implement `DiscretisedBrush::step_heterogeneous` in `native/src/pneumatic/brush.{hpp,cpp}` (`BIKE_NONBLOCKING`), ray-level `tirecommon::eval_surface` + `sample_mu` in `pneumatic/model.{hpp,cpp}`, and world-Jacobian `m_roll` in `writers/pneumatic_tire.{hpp,cpp}` per `01-mujoco-surface-stage-a-completion.md` Task 4.
4. Set `[resistance]\ncrr = 0.0` in `examples/research/viewer_physics_pneumatic.toml` and `tests/fixtures/config/viewer_physics_pneumatic.toml`, and update `"crr": 0.0` in `tests/fixtures/config/resolved/pneumatic.json`.
5. Update non-golden unit assertions in `test_pneumatic_tire_forces.cpp`, `test_pneumatic_tire_oracle.cpp`, and `test_native_pneumatic_model.cpp` for the closed radial energy and world rolling moment.

- [ ] **Step 4: Build and run `reference.pneumatic_energy_balance` and pneumatic unit tests**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target \
  reference.pneumatic_energy_balance \
  reference.pneumatic_tire_forces \
  reference.test_schema
"$WT/build/native-release/native/tests/reference/reference.pneumatic_energy_balance"
"$WT/build/native-release/native/tests/reference/reference.pneumatic_tire_forces"
```
Expected: 100 % PASS (including the 60 m mixed-surface pneumatic ride gate with `|residual_j| / energy_scale_j <= 0.05`).

- [ ] **Step 5: Run incremental `cppcheck`/`frontends`/`tidy` on touched pneumatic TUs and commit**

Run `bike-native-checks` (`cppcheck`, `frontends`, `tidy`) on the modified files, verify 0 diagnostics, and commit as `feat(pneumatic): close radial energy ledger, ray-resolved surface physics and crr double-count`.

---

### Task 3: Record Implicit-FSM Audit Artifacts, Re-Verify Bans 36–41 (Item 6) & Diagnose Fork Rebound on 10 % Stop (Item 19) (Wave 2)

**Files:**
- Create: `docs/superpowers/audits/2026-11-21-implicit-fsm/{fsm-map.md,R0.md,R1.md,R2.md,Rfinal.md,rfinal_data.json,journal.md}`
- Create: `docs/research/2026-11-21-fork-rebound-stop-diagnosis.md`
- Modify: `docs/research/2026-11-05-physics-bans-check.md`
- Modify: `docs/agents/simulation-decisions.md`

**Interfaces:**
- Consumes: Completed audit outputs in `/Users/vladislav.molchanov/Desktop/Projects_downloaded/mujoco-implicit-fsm-audit-swe2/` and live C++ physics runtime in `WT`.
- Produces: Committed Implicit-FSM audit record, verified Bans 36–41 report, and measured diagnosis of fork rebound after 100 % braking from ~24 km/h on a 10 % grade.

- [ ] **Step 1: Copy verified Implicit-FSM audit summary artifacts into `docs/superpowers/audits/2026-11-21-implicit-fsm/`**

Copy `fsm-map.md`, `R0.md`, `R1.md`, `R2.md`, `Rfinal.md`, `rfinal_data.json`, and `journal.md` from `/Users/vladislav.molchanov/Desktop/Projects_downloaded/mujoco-implicit-fsm-audit-swe2/` into `$WT/docs/superpowers/audits/2026-11-21-implicit-fsm/` and verify `bike-native-checks --kind docs` (if applicable) passes. (Check `native/src/tools/check_docs.cpp` first so no Python-audit or link check fails on audit markdown!)

- [ ] **Step 2: Audit Bans 36–41 across `native/src/` and update `2026-11-05-physics-bans-check.md` + `simulation-decisions.md` (#6, #36–41)**

Verify directly in source:
- Ban 36 (no hidden stabilizing/anti-flip forces): inspect `native/src/model/` and `native/src/writers/`;
- Ban 37 (no `armature` on joints): grep `armature` in `native/src/model/`;
- Ban 38 (no Pacejka magic formula): grep `pacejka|magic_formula|atan.*atan` in `native/src/`;
- Ban 39 (no viscous tire damping across all 3 backends `compliant_2d`, `distributed_2d_reference`, `pneumatic`): verify `contact/laws.hpp`, `tyre/station.cpp`, `pneumatic/carcass.cpp`, and `validate.cpp`;
- Ban 40 (non-deformable road in 2D): verify `terrain/` and `contact/`;
- Ban 41 (no automatic motor anti-wheelie/traction cut in core runtime): verify `drivetrain/motor.cpp` (only standalone opt-in research example plugin `examples/policies/traction_control.cpp` exists, never active in normal rides).
Record findings and close Item 6 in `docs/agents/simulation-decisions.md`.

- [ ] **Step 3: Reproduce and measure Item 19 (fork rebound after 100 % braking from ~24 km/h on 10 % climb)**

Write a temporary C++ diagnostic probe (or run `bike-ride` with recording + DuckDB/JSON inspection into `/tmp` or `.scratch`, never leaving untracked files in `$WT`) on `viewer_physics_welded.toml` on a smooth 10 % grade: accelerate/start at ~6.67 m/s (24 km/h), apply `front_brake = 1.0, rear_brake = 1.0` to full stop, and extract 1.25 ms time series of:
- `fork_travel_mm`, `fork_velocity_mps`, `F_air_n`, `F_damper_compression_n`, `F_damper_rebound_n` (`c_reb = 909.4 N·s/m` at `fork_rebound = 9`),
- `root_pitch`, `root_pitch_vel`, `front_support_n` (duration `< 50 N` and minimum normal load).
Check `native/src/writers/suspension.cpp` / `damper` sign and velocity convention on extension (`v_fork < 0` vs `> 0`) to confirm whether rebound damping opposes extension with full `c_reb = 909.4 N·s/m` (critical damping ratio $\zeta_{\text{reb}}$ vs unsprung+front-triangle effective mass) or has a code bug.
- If a genuine code bug is found: STOP and ask the user before changing damper physics.
- If rebound damping acts with the exact intended sign and magnitude and the 53–75 ms unload is physical dive-energy release on a 10 % incline: write `docs/research/2026-11-21-fork-rebound-stop-diagnosis.md` with exact numbers and update Item 19 in `docs/agents/simulation-decisions.md`.

- [ ] **Step 4: Run `bike-native-checks --kind contracts` / doc checks and commit Wave 2**

Commit audit artifacts and research updates cleanly.

---

### Task 4: Fix Physics-Critical Implicit-FSM Audit Violations (`P01b`, `P04b`, `P07a/b`, `C2/C3`) (Wave 3A)

**Files:**
- Modify: `native/src/configuration/validate.cpp:1330-1333`
- Modify: `native/src/writer_config.hpp:970-985, 1075-1115`
- Modify: `native/src/drivetrain/shifting.hpp`
- Modify: `native/src/drivetrain/pedaling.cpp:20-45`
- Modify: `native/src/drivetrain/motor.cpp:28-36`
- Modify: `native/src/drivetrain/state_codec.cpp`
- Modify: `native/src/writers/drivetrain.hpp`
- Modify: `native/src/writers/drivetrain.cpp`
- Modify: `native/src/rider/launch_plan.hpp`
- Modify: `native/src/rider/launch_plan.cpp`
- Modify: `native/src/runtime/step.hpp`
- Modify: `native/src/runtime/step.cpp`
- Test: `native/tests/config/test_schema.cpp`
- Test: `native/tests/reference/test_native_drivetrain.cpp`
- Test: `native/tests/reference/test_native_drive_policies.cpp`
- Test: `native/tests/reference/test_native_runtime_step.cpp`

**Interfaces:**
- Produces:
  1. `P01b`: `reposition_on_stall == true` rejected unless `pedal_attachment == "spindle"` AND `motor_clutch == true` in `configuration::validate` and `writer_config::decode_drive` (`reposition_on_stall` requires `motor_clutch = true`).
  2. `P04b`: `constexpr bool drivetrain::shift_cut_active(double cut_remaining_s) noexcept { return cut_remaining_s > 1e-12; }` (single threshold everywhere across `drivetrain`, `step.cpp`, and `launch_plan`).
  3. `P07a`: RAII guard in `PhysicalStep::probe_step_inputs` (or non-advancing `apply_forces`) saving/restoring `model->dof_frictionloss[front_dof]`, `model->dof_frictionloss[rear_dof]`, and `stepper_->posvel_fresh()` so non-advancing probes leave `mjModel` and `Stepper` freshness bitwise untouched.
  4. `P07b`: `double PhysicalStep::peek_rollback_brake_demand(double speed_mps, const RideControl *control) const noexcept` evaluating rollback hysteresis without mutating `rollback_hold_`, wired into non-advancing `apply_forces` (`resolved == nullptr`) so `probe_step_inputs` matches `advance_physics` under active rollback hold.
  5. `C2`/`C3`: Cross-field relational validation in `PedalingPolicy::set_state` + `state_codec.cpp` (`!stall_elapsed_s => crank_rad == 0 && wheel_rad == 0`, `goal_rad.has_value() <=> (reason == 1 || reason == 2)`, `goal_rad.has_value() => cooldown_s == 0 && !coasting && effort == 0`, `coasting => effort == 0`) and `AssistController::set_state` + `state_codec.cpp` (`release_hold_left_s > 0 => hold_power_w > 0 && hold_torque_nm > 0`, `(hold_power_w > 0) == (hold_torque_nm > 0)`).

- [ ] **Step 1: Write failing unit tests for `P01b`, `P04b`, `P07a`, `P07b`, `C2`, and `C3`**

Add targeted regressions in:
- `test_schema.cpp` & `test_native_drivetrain.cpp` (`reposition_on_stall = true` with `motor_clutch = false` throws `std::invalid_argument` at both `configuration::validate` and `DrivetrainWriter` wire decode);
- `test_native_drive_policies.cpp` (`cut_remaining_s = 0.5e-12` agrees across policy/writer/step via `shift_cut_active`; incoherent `PedalingSnapshot` and `AssistSnapshot` throw `std::invalid_argument` on `set_state` and wire restore while leaving prior state intact);
- `test_native_runtime_step.cpp` (`probe_step_inputs` preserves pre-existing `model->dof_frictionloss` and `posvel` cache state after return; when `rollback_hold_` is engaged on a backward slope, `probe_step_inputs` reports the exact rollback brake demand and `brake_bounds` that `advance_physics` applies, without mutating `rollback_hold_`).

- [ ] **Step 2: Build and run the 4 test targets to confirm RED**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target \
  config.test_schema \
  reference.native_drivetrain \
  reference.native_drive_policies \
  reference.native_runtime_step
ctest --test-dir "$WT/build/native-release" -R "^(config\.test_schema|reference\.native_drivetrain|reference\.native_drive_policies|reference\.native_runtime_step)$" --output-on-failure
```
Expected: FAIL on the newly added assertions.

- [ ] **Step 3: Implement `P01b`, `P04b`, `P07a`, `P07b`, `C2`, and `C3`**

Before enforcing `C2`/`C3`, inspect `PedalingPolicy::update` (`pedaling.cpp:147-253`) and `AssistController::step` (`motor.cpp:50-120`) line-by-line to verify whether any reachable live state during coasting or release-hold transiently has `coasting && effort > 0` (e.g. slew-down during coasting!) or `release_hold_left_s > 0` at zero torque!
- **Critical check:** Remember `simulation-decisions.md` #9/#12 (`pedaling.cpp:72-78`): when `coasting` engages, does `state_.effort` slew down via `effort_slew_nm_s` over multiple steps while `coasting == true`, or is `effort` zeroed immediately? Check `pedaling.cpp`! Only enforce relational predicates in `set_state` that are **true inductive invariants of `PedalingPolicy::update` and `AssistController::step`** (never reject a state that normal stepping can produce and checkpoint!). Adjust the exact predicate and test if `coasting` slews `effort` down gradually.

- [ ] **Step 4: Re-run targeted tests + state-restore/fuzz/reposition suites and commit**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target \
  config.test_schema \
  reference.native_drivetrain \
  reference.native_drive_policies \
  reference.native_runtime_step \
  reference.native_state_restore \
  reference.native_state_fuzz \
  reference.crank_reposition
ctest --test-dir "$WT/build/native-release" -R "^(config\.test_schema|reference\.native_drivetrain|reference\.native_drive_policies|reference\.native_runtime_step|reference\.native_state_restore|reference\.native_state_fuzz|reference\.crank_reposition)$" --output-on-failure
```
Expected: 7/7 PASS, 0 static-check diagnostics on modified TUs. Commit as `fix(drivetrain,runtime): close P01b, P04b, P07a/b and set_state invariants from FSM audit`.

---

### Task 5: Require Explicit Profile Keys in `SeatedClimbConfig` (Item 12 / Reverify Queue Item 1) (Wave 3B)

**Files:**
- Modify: `native/src/rider/intent.hpp:38-66`
- Modify: `native/src/configuration/model_config.hpp:202-241`
- Modify: `native/src/configuration/validate.cpp:825-875, 1380-1395`
- Modify: `examples/research/{viewer_physics_welded.toml,viewer_physics_distributed.toml,viewer_physics_pneumatic.toml,rider_type_careful.toml,rider_type_aggressive.toml}`
- Modify: `tests/fixtures/config/{viewer_physics_welded.toml,viewer_physics_distributed.toml,viewer_physics_pneumatic.toml,variant_alt_support.toml,variant_grip_weld_coercion.toml}`
- Modify: `tests/fixtures/golden/darwin-arm64/{control_clock/env_request/physics.toml,native_distributed_tire/env_request/physics.toml,pneumatic_tire/env_fast/physics.toml}`
- Modify: `tests/fixtures/golden/linux-aarch64/{control_clock/env_request/physics.toml,native_distributed_tire/env_request/physics.toml,pneumatic_tire/env_fast/physics.toml}`
- Modify: `tests/fixtures/golden/episode/{pinned_savage_40/physics.toml,pinned_savage_40_f2/physics.toml}`
- Test: `native/tests/config/test_schema.cpp`

**Interfaces:**
- Promotes all remaining `SeatedClimbConfig` fields (`enabled`, `target_crank_power_w`, `lean_gain`, `max_backward_lean_rad`, `surge_power_w`, `surge_grade`, `surge_recovery_rate`, `front_load_share_target`, `lean_trim_gain_rad_s`, `lean_trim_limit_rad`) to `required = true` in `kSeatedClimbConfigFields`, with `-1.` unset sentinels in `rider::IntentConfig` and explicit initialization in `default_physics()` (and any C++ test helpers constructing `IntentConfig` directly).

- [ ] **Step 1: Add failing schema test in `test_schema.cpp` asserting omitting any `[seated_climb]` scalar key in TOML fails validation**

- [ ] **Step 2: Promote `SC_REAL` → `SC_REAL_REQ` (+ `enabled` `.required = true`), set `-1.` sentinels in `IntentConfig`, populate `default_physics()` and all `.toml` `[seated_climb]` tables (`lean_gain = 1.0`, `max_backward_lean_rad = 0.10`, etc. matching each file's prior effective values so resolved JSONs do not drift unintentionally)**

- [ ] **Step 3: Build and run config/loader/input-contract tests and commit**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target \
  config.test_schema \
  reference.native_loader \
  reference.native_input_contracts \
  reference.welded_profile_config
ctest --test-dir "$WT/build/native-release" -R "^(config\.test_schema|reference\.native_loader|reference\.native_input_contracts|reference\.welded_profile_config)$" --output-on-failure
```
Expected: 4/4 PASS. Commit as `feat(config): require all seated_climb fields explicitly in physics profiles`.

---

### Task 6: Cap Combined Static Brake + Hold Bite at `brake_ceiling_nm * demand` (Item 11) (Wave 4A)

**Files:**
- Modify: `native/src/runtime/static_brake.hpp`
- Modify: `native/src/runtime/static_brake.cpp`
- Modify: `native/src/runtime/step.cpp:1320-1340`
- Modify: `docs/agents/simulation-decisions.md`
- Test: `native/tests/reference/test_native_runtime_step.cpp` (and/or dedicated `StaticBrake` unit test)

**Interfaces:**
- Updates `StaticBrake`:
  1. In `hold_qfrc(...)`: `const double cap = std::min(hold_.torque_nm, ceiling_nm_) * u;` (both for the `reseat` saturation threshold and the `std::clamp` output bound).
  2. Adds overload or parameter `bool StaticBrake::apply(mjModel *model, double front_demand, double rear_demand, double front_hold_nm = 0., double rear_hold_nm = 0.) const`:
     ```cpp
     const double front_loss = std::max(0., ceiling_nm_ * std::clamp(front, 0., 1.) - std::abs(front_hold_nm));
     const double rear_loss  = std::max(0., ceiling_nm_ * std::clamp(rear,  0., 1.) - std::abs(rear_hold_nm));
     ```
  3. In `PhysicalStep::apply_forces` (`step.cpp:1323-1336`): evaluate `brake_hold = brake_.hold_qfrc(...)` FIRST, then pass `brake_hold[brake_.front_dof()]` and `brake_hold[brake_.rear_dof()]` into `brake_.apply(...)` and invalidate `stepper_->note_posvel_stale()` when `dof_frictionloss` changed (still protected by Task 4's `P07a` RAII restore guard when `advance == false`).

- [ ] **Step 1: Write failing test proving total wheel holding torque (`|tau_hold + tau_frictionloss|`) never exceeds `ceiling_nm * demand` (200 Nm at `u = 1.0`, not 400 Nm), while still preventing creep below 200 Nm and sliding smoothly above 200 Nm**

- [ ] **Step 2: Run targeted test to confirm RED (currently holds up to 400 Nm)**

- [ ] **Step 3: Implement the combined budget split in `static_brake.{hpp,cpp}` and `step.cpp`, update Item 11 in `docs/agents/simulation-decisions.md`**

- [ ] **Step 4: Run targeted brake, step, settle, and launch tests to verify GREEN and commit**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target \
  reference.native_runtime_step \
  reference.launch_preparation \
  reference.crank_reposition
ctest --test-dir "$WT/build/native-release" -R "^(reference\.native_runtime_step|reference\.launch_preparation|reference\.crank_reposition)$" --output-on-failure
```
Expected: PASS. Commit as `feat(physics): cap combined static brake friction and hold bite at brake_ceiling_nm (decision 11)`.

---

### Task 7: Apply 30/30 % Default Suspension Sag Across All Three Backends (Item 9) & Verify Acceptance Rides (Wave 4B)

**Files:**
- Modify: `native/src/geometry/specs.hpp:65-106`
- Modify: `native/src/configuration/suspension_spec.hpp:118-125`
- Modify: `native/src/model/builder.cpp:20-28`
- Modify: `native/src/initialization/request.cpp:116-124`
- Modify: `native/src/cli/configuration.cpp:114-122`
- Modify: `native/tests/kinematics/test_horst_link.cpp:172-185`
- Modify: `native/tests/model/test_inertia.cpp` (if default tuning snapshot asserts `85.2` / `114600.0`)
- Modify: `docs/agents/simulation-decisions.md`
- Modify: `docs/research/2026-11-17-default-sag-calibration.md`

**Interfaces:**
- Updates `geometry::SuspensionSpecs` / `geometry::BikeSpecs` / `configuration::SuspensionInputs` base defaults to the calibrated `welded` (`compliant_2d`) 30/30 % sag constants:
  - `fork_initial_psi = 97.0` (was `85.2`),
  - `shock_stiffness = 81550.0` (was `114600.0`).
- Adds helper `geometry::sync_backend_default_suspension(geometry::BikeSpecs &bike, std::string_view tire_backend) noexcept`: when `bike.fork_initial_psi == 97.0 && bike.shock_stiffness == 81550.0` (i.e. caller kept base `BikeSpecs` defaults), sets:
  - `tire_backend == "distributed_2d_reference"` → `fork_initial_psi = 90.8`, `shock_stiffness = 84372.0`;
  - `tire_backend == "pneumatic"` → `fork_initial_psi = 113.8`, `shock_stiffness = 73557.0`;
  - otherwise (`compliant_2d`) → keeps `97.0` / `81550.0`.
  Called in `model/builder.cpp`, `initialization/request.cpp`, and `cli/configuration.cpp` right alongside the `pneumatic` wheel-radius sync.

- [ ] **Step 1: Update `test_horst_link.cpp` (and add a 3-backend equilibrium sag check verifying 30.0 % ± 0.5 mm fork & shock-stroke sag out-of-the-box on `welded`, `distributed`, and `pneumatic`) and confirm RED**

- [ ] **Step 2: Implement 30/30 % defaults and backend sync, update `test_inertia.cpp` / `resolved/*.json` if needed, and verify unit tests GREEN**

- [ ] **Step 3: Run the physics acceptance & launch suites BEFORE golden regeneration to prove 30/30 % sag + 200 Nm brake satisfy every acceptance threshold**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --build "$WT/build/native-release" --target \
  kinematics.test_horst_link \
  model.test_inertia \
  reference.pneumatic_energy_balance \
  reference.realistic_pedelec_acceptance \
  reference.rider_launch_rides_careful \
  reference.rider_launch_rides_normal \
  reference.rider_launch_rides_aggressive
ctest --test-dir "$WT/build/native-release" -R "^(kinematics\.test_horst_link|model\.test_inertia|reference\.pneumatic_energy_balance|reference\.realistic_pedelec_acceptance|reference\.rider_launch_rides_(careful|normal|aggressive))$" --output-on-failure
```
Expected: ALL PASS. (If any launch or acceptance threshold fails, STOP immediately per `simulation-decisions.md` #34 and present exact metrics to the user.)

- [ ] **Step 4: Update Item 9 status in `docs/agents/simulation-decisions.md` and commit**

Commit as `feat(suspension): apply calibrated 30/30 % default sag across welded, distributed and pneumatic backends`.

---

### Task 8: Port Pure-C++ `BGTRAJ01` Ghost Replay & Viewer Ghost Mode Without GPU Dependencies (Wave 5)

**Files:**
- Modify: `native/cmake/RecordingApplication.cmake`
- Create: `native/src/replay/trajectory.hpp`
- Create: `native/src/replay/trajectory.cpp`
- Create: `native/src/replay/golden_overlay.hpp`
- Create: `native/src/replay/golden_overlay.cpp`
- Modify: `native/src/replay/playback.hpp`
- Modify: `native/src/replay/playback.cpp`
- Modify: `native/src/cli/argument_contracts.inc`
- Modify: `native/src/cli/replay.cpp`
- Modify: `native/src/viewer/decorations.hpp`
- Modify: `native/src/viewer/render_owner.hpp`
- Modify: `native/src/viewer/render_owner.cpp`
- Modify: `native/src/viewer/ride.cpp`
- Modify: `native/src/viewer/session.hpp`
- Modify: `native/tests/replay/CMakeLists.txt`
- Create: `native/tests/replay/test_trajectory.cpp`
- Create: `native/tests/replay/test_golden_overlay.cpp`
- Modify: `native/tests/viewer/test_render.cpp`

**Interfaces:**
- Ports commits `a9e8bc1f` and `a886f91b` (`7aab8c38..a886f91b` excluding `tools/colab_gpu/**`) cleanly onto `feat/surface-stage-a-2026-11-18`, verifying zero references to `physics_core` or `native/src/gpu/`.

- [ ] **Step 1: Apply the C++-only diff `git diff 7aab8c38..a886f91b -- native/` in `$WT` and verify zero GPU/Python files touched**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
git -C "$WT" diff 7aab8c38..a886f91b -- native/ | git -C "$WT" apply --index
git -C "$WT" status -sb
```
Expected: Clean apply of the 18 `native/` files only (`0` files under `tools/colab_gpu/`).

- [ ] **Step 2: Build and run `replay` and `viewer` test suites + static checks on ported files**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
cmake --preset release -S "$WT"
cmake --build "$WT/build/native-release" --target \
  replay.test_trajectory \
  replay.test_golden_overlay \
  viewer.test_render \
  reference.replay_session \
  reference.native_replay_session
ctest --test-dir "$WT/build/native-release" -R "^(replay\.|viewer\.|reference\.(native_)?replay)" --output-on-failure
```
Expected: 100 % PASS; `cppcheck`/`frontends`/`tidy` on ported files exit 0.

- [ ] **Step 3: Commit `BGTRAJ01` ghost replay**

Commit as `feat(replay,viewer): port BGTRAJ01 external trajectory and viewer ghost mode`.

---

### Task 9: Consolidated Pose/Golden Regeneration, Full Qualification Gate (`run_tests.sh full`), Fast-Forward `main` & Push `origin/main` (Wave 6)

**Files:**
- Modify: `tests/fixtures/config/poses/**` (including `savage_pneumatic.npz`)
- Modify: `tests/fixtures/golden/darwin-arm64/**` (and any affected `tests/fixtures/config/resolved/*.json` / `tests/fixtures/golden/episode/**`)
- Modify: `docs/TESTING.md` / `docs/agents/simulation-decisions.md` (record `linux-aarch64` pending golden refresh list if expanded by 30/30 % sag + 200 Nm brake)

**Interfaces:**
- Consumes: All completed code commits from Tasks 1–8.
- Produces: Green `bash tools/run_tests.sh full` on a clean `HEAD`, fast-forwarded `main`, and pushed `origin/main`.

- [ ] **Step 1: Re-settle stale pinned poses (`savage_pneumatic.npz`, etc.) and run ordinary + slow CTest once to enumerate exact golden fixtures affected by Tasks 2–7**

Run `bike-settle` for `savage_pneumatic.npz` (and any other pinned pose flagged by `reference.equilibrium_state_file`), inspect failing golden tests, and verify every failure is an expected consequence of Tasks 2, 5, 6, or 7 (never a regression in invariants or violations).

- [ ] **Step 2: Re-record affected `darwin-arm64` golden fixtures, audit `git diff --stat`, and commit on a clean working tree**

Re-record only the genuinely affected golden suites (`BIKE_RECORD_GOLDENS=1` / suite-specific record commands used in `docs/TESTING.md`), verify `git status --porcelain` is clean after committing `test(golden): refresh darwin-arm64 fixtures and pinned poses for Stage A, 30/30 sag and 200 Nm brake`.

- [ ] **Step 3: Run the complete Level 3 Extended gate (`bash tools/run_tests.sh full`)**

Run:
```bash
WT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18"
test -z "$(git -C "$WT" status --porcelain)"
bash "$WT/tools/run_tests.sh" full
test -z "$(git -C "$WT" status --porcelain)"
```
Expected: `run_tests.sh full` exits `0` (`configure`, `build`, all 10 `bike-native-checks` sweeps, ordinary CTest, `slow` CTest, and `package` CTest all green), and `git status --porcelain` remains empty.

- [ ] **Step 4: Verify zero path overlap with root dirty/untracked files, fast-forward `main`, and push `origin/main`**

Run:
```bash
(
  set -euo pipefail
  ROOT="/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean"
  WT="$ROOT/.worktrees/surface-stage-a-2026-11-18"
  VERIFIED_SHA="$(git -C "$WT" rev-parse HEAD)"

  test "$(git -C "$ROOT" branch --show-current)" = "main"
  git -C "$ROOT" merge-base --is-ancestor main "$VERIFIED_SHA"

  OVERLAP="$(comm -12 \
    <(git -C "$ROOT" diff --name-only "main..$VERIFIED_SHA" | sort -u) \
    <(git -C "$ROOT" status --porcelain -uall | cut -c4- | sed 's/.* -> //' | sort -u))"
  if [[ -n "$OVERLAP" ]]; then
    printf 'OVERLAP DETECTED:\n%s\n' "$OVERLAP" >&2
    exit 1
  fi

  git -C "$ROOT" merge --ff-only "$VERIFIED_SHA"
  git -C "$ROOT" push origin main
  test "$(git -C "$ROOT" rev-parse main)" = "$VERIFIED_SHA"
  test "$(git -C "$ROOT" rev-parse origin/main)" = "$VERIFIED_SHA"
)
```
Expected: `main` and `origin/main` advance cleanly via `--ff-only` to `$VERIFIED_SHA` with zero disturbance to root untracked files.
