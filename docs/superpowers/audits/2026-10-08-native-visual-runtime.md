# Native visual runtime — execution audit

Evidence ledger for [the master plan](../plans/2026-10-08-native-visual-runtime.md) and
[the design spec](../specs/2026-10-08-native-visual-runtime-design.md). This work implements
the 2026-10-08 native visual-runtime plan; it does not continue the unfinished
2026-10-06 native-safety ledger.

## Execution baseline

| Field | Value |
|---|---|
| Checkout | `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2` |
| Branch | `impl/cpp-port-p2` |
| Baseline HEAD | `7a12394ed22e0657dbb02b29f23f76d5f8308dd5` |
| Selected test artifact | `native/build/release` (`NATIVE_TEST_BUILD_PATH`) |

Baseline `git status` at A0 start: no tracked modifications. Untracked user-owned
content preserved untouched: `.claude/skills/*`, `docs/agents/`,
`docs/reference/simulation-audit-2026-10-07.md`, the six `2026-10-06-native-safety-*`
plan docs, `docs/superpowers/specs/2026-10-06-native-safety.md`, `output/`.

Task-owned additions at A0: the four `2026-10-08-native-visual-runtime*` plan docs,
`docs/superpowers/specs/2026-10-08-native-visual-runtime-design.md` (copied from the
reviewed package at `/private/tmp/native-visual-runtime-plan-2026-10-08/` per user
instruction), this audit, `tests/reference/_native_runtime_support.py`,
`tests/reference/conftest.py`.

## Baseline verification

Environment: macOS arm64, Python 3.14.3, NumPy 2.5.2 (**Accelerate** BLAS/LAPACK —
source of the `DLASCL illegal parameter` warnings emitted during the run),
SciPy 1.17.0, MuJoCo 3.12.0, nanobind per locked `native` group.

| Item | Result |
|---|---|
| Release configure/build | passed — `native/build/release/bike_native.cpython-314-darwin.so` |
| Static sweeps | `check_frontends`, `check_tidy`, `check_analyzer`, `check_cppcheck`, `check_odr`, `check_scripts`, `check_headers`, diagnostic controls (6/6), context SDK controls — all `status: passed`, `errors: []` |
| CTest | 4/4 passed |
| `check_native_contracts` | all controls passed (manifest, loader provenance, ISO mode, analysis policy, artifact assertions, contract manifest membership, ctest cases, engine wrappers, benchmark manifest) |
| `run_tests.sh full` (baseline) | **26 failed, 2579 passed** in 730.64 s; exit 0; 2032 native extension items collected; log `/tmp/a0-baseline-full.log` |

### Baseline failure classification (A0)

Of the 26 failures, **24 exactly match the documented baseline set** in
`docs/TESTING.md` (verified there against an isolated `1ccdfcb` archive —
pre-existing on this platform, unrelated to this work). The remaining **2 are
Release-artifact-specific**:

- `test_native_drivetrain.py::test_random_multiturn_staging[5-ideal_mid_drive]`
- `test_native_drivetrain.py::test_random_multiturn_staging[5-geometric_ideal_mid_drive]`

Both are 1-ULP bitwise mismatches (`1.1775466189556463` vs `...465`) between the
native extension and the Python drivetrain oracle. They **pass against the
unoptimized default `native/build`** (`CMAKE_BUILD_TYPE` empty) and fail only
under `-O3` Release — FP contraction/codegen difference in the C++ drivetrain
kernels. Verified by rerunning the pair against each artifact
(`NATIVE_TEST_BUILD_PATH=.../native/build`: 2 passed; `.../native/build/release`:
2 failed). **Consequence for A2–A4:** verify commands all select the Release
artifact, so new bitwise-parity assertions must be written knowing Release
codegen can differ from Python by last-ULP amounts; the documented baseline used
the unoptimized artifact and never observed this class.

The 24 documented baseline failures (unchanged on this machine, all in Python
physics or test/model skew, none affected by A0 files):

- Platform physics divergence (Accelerate numerics; first-interval
  `energy.constraint_work` violations at t=0.005; saddle weld reactions ~632–645 N
  vs expected ~785 N; equilibrium non-convergence at 40 000 steps in one case):
  `test_closed_form_rider::test_flat_launch_delivers_crank_torque`,
  `test_joint_strength::test_stepped_effort_respects_strength_and_the_power_budget`,
  `test_period_buffer::test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms`,
  `test_period_buffer::test_runtime_batch_matches_preserved_scalar_step_and_flush`,
  `test_realistic_pedelec_acceptance` (4),
  `test_rider_allocation` (3),
  `test_seated_pedaling_cycle` (4),
  `test_seated_posture_program::test_pulse_redistributes_load_through_inertia_only`,
  `test_rider_welds` (3).
- Deterministic test/code skew at HEAD (fails on any platform):
  `test_coupled_rider_task` (2 — test injects `joint_passive_damping_nms_rad`,
  which `viewer_physics_welded.toml` already defines → TOMLDecodeError),
  `test_planar_arms::test_compiled_topology_has_two_arms_and_two_grip_sites`
  (`reference_joint_names()` includes `rider_ankle_front/rear` not compiled by
  the welded profile).
- Performance environment: `test_realtime_gate` (2 — Python backend reaches only
  ~0.46–0.50× real-time at dt=0.00125 on this machine; motivates this port).

**Conftest independence:** the two period_buffer/rider_allocation failures and
all `tests/test_rider_welds.py` failures reproduce with
`tests/reference/conftest.py` removed; the fixture only sets
`BIKE_NATIVE_BUILD_PATH`, which nothing at this HEAD consumes (the loader
selector is `NATIVE_TEST_BUILD_PATH`). `_native_runtime_support.py` is imported
by nothing until A1 tests. A0 changes are inert.

**Gate-relevant baseline failures:** `test_joint_strength` appears in the A2
verify list; `test_period_buffer` (2 IDs) appears in the A4 verify list; all 26
recur in `full` at A4/B3/C4. Per the master plan these are recorded baseline
failures — final acceptance stays red until resolved or explicitly reported.

**Helper smoke test:** `make_python_ride()` + `advance_python(sim, 40)` produced
40 samples in ~0.2 s (equilibrium cache warm). The first interval reports
`energy.constraint_work` in diagnostic mode — consistent with the baseline
violation class; parity tests must use diagnostic (`strict=False`) mode on both
sides, as the plan prescribes.

## Evidence ledger

| Task | Source revision | Red test/error | Green command/result | Selected artifact | Inspection result | Remaining limitations |
|---|---|---|---|---|---|---|
| A0 | `7a12394` + A0 files | baseline `full`: 26 failed / 2579 passed (24 documented + 2 Release-FP) | Release build + all sweeps + CTest + contract checks green; helper smoke ok | `native/build/release` | sweeps `errors: []` | 26 recorded baseline failures; Release-vs-Python 1-ULP FP divergence risk for bitwise parity gates |
| A1 | A0 + `bike_sim.native` package + `native/src/runtime/` | `test_native_runtime_config.py` failed on `import bike_sim.native` (module absent) | 43/43 `test_native_runtime_{config,bootstrap}`; 129/129 A1 set incl. `native_loader`/`state_restore` | `native/build/release` + `native/build` | IDEA lint: no new findings on touched files; `check_frontends` green after GCC-only fixes | `NativeRideRuntime` exposes only snapshot/reset/close — stepping is A3; controller/intent state is staged wire data, not yet consumed |
| A2 | `3855be1` + fixes `e63423a`, `dcf4d47`, `a55ffb7` | review: UB on missing-joint lookup, missing nq/nv gate, vacuous activation/damping and `front_load_share=None` coverage; Release `test_spindle_compute[crank_q=pi/2-*]` ~4e-12 rel fails | scratch Release build: 68/68 spindle+intent, 5/5 static-brake, 12/12 compute parity incl. π/2 cases; Debug 68/68; drivetrain staging 24/24 Release | scratch Release at `dcf4d47`/`a55ffb7` (in-tree link blocked by in-flight A3 `step.cpp` at the time) | independent re-reviews clean; root cause of both Release-ULP classes was `__sincos_stret` fusion — fixed via `numeric::sin/cos` | in-tree `run_tests.sh native` deferred until `step.cpp` compiles (sweeps haven't seen fix TUs together); 2 `requested_filled`/compute-path throw sites unexecutable on pinned plant (deferred minor); baseline failures now 24 (2 drivetrain Release-ULP eliminated) |
| A3 | `a55ffb7` + `runtime/{step,control}`, `NativeRideRuntime.advance`, `bike_sim.native.runtime` adapter | Release `SIGTRAP` on first `advance`; 1-ULP rider/brake lanes (NumPy FMA-contracted interp + matmul rounding order); full-suite schema drift (`rider_forces.paths[].name`, `rollback_*` pedaling keys) | `bash tools/run_tests.sh native`: build + all six sweeps + CTest 4/4 + contract checks green; pytest 2263 passed / 0 failed; focused A3 set 67/67 (incl. 18 `runtime_{step,commands}`); custom 30-step integration-state parity: 0 mismatched lanes | `native/build/release` + all five variant `.so`s | `nm -u`: no `__sincos_stret` in any variant; no new trig sites in the diff; `rollback_*` confirmed optional-with-defaults matching `PedalingConfig` dataclass (True/.25/0./1.) | 21 failures in `tests/reference` outside native scope verified identical on `a55ffb7` (rider allocation/QP `DLASCL` noise — pre-existing); `NativeRideDriver` period accounting intentionally empty until A4 |

### A1 implementation notes

- **Package boundary.** `tools/native_config.py` and `tools/native_schema.py`
  moved verbatim into `bike_sim/native/{config,schema}.py`; both tools modules
  are now compatibility shims re-exporting the full surface (including the
  `validate_config`/`plain`/`sequence` and leading-underscore names the
  original imported from the schema module). 34 initially-failing consumers
  verified green after the shims were completed.
- **Capability matrix** (`setup.py::validate_supported`): field-path rejections
  for every design section-1 predicate; construction-time rejections tested via
  `object.__setattr__` bypass so `validate_supported` itself is exercised.
- **Bootstrap** (`capture_bootstrap` → `RuntimeBootstrap`): writes
  `model.mjb`, SHA-256 digest, 13 model dims, 292-wide `mjSTATE_INTEGRATION`
  vector (finite-checked), name-keyed mutable model coefficients
  (`dof_frictionloss` per joint+dof, `site_pos` per site), and owner-side
  `state_dict()` exporters added on every state owner (drivetrain, tire,
  rider contacts, rider control, intent policy/program, pedal recovery,
  terrain queries, filters, monitor, clock; `PhysicalRuntime`/`RideSimulation`
  gained small public readers so the exporter touches no private fields).
- **Native side** (`native/src/runtime/`): `NativeRideRuntime` owns
  `Stepper` + decoded `BootstrapState`; construction order is config →
  digest → Stepper → decode → ordered restore (`reset_data` → mutable model
  → `mj_setConst` → integration vector → writer snapshots → counters).
  `mj_setConst` rewrites `qpos`, so the integration vector restores strictly
  after it (the one ordering constraint, found by test).
- **State decode reuses existing parsers**: `parse_drive_snapshot` and
  `parse_rider_contacts_state` are the same readers the `set_*_state`
  bindings use — no second wire format.
- **GCC frontend fixes** (Clang-clean but GCC-flagged): redundant `std::move`
  returns, a useless `nb::object` cast, and `size_t → iter_difference_t`
  sign-conversion in `views::counted`.
- **Restore correctness evidence**: constructor snapshot equals the captured
  integration vector exactly (292 doubles, bitwise); `reset()` re-applies the
  bootstrap and bumps generation 1 → 2; `close()` makes further calls raise
  `NativeRideRuntime is closed`.
