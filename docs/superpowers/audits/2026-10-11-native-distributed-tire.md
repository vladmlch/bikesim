# Native distributed tyre — acceptance audit

Evidence ledger for [Plan B](../plans/2026-10-11-distributed-tire-b-native-port.md)
(native C++ port of the portable `distributed_2d_reference` reference tyre).
The oracle throughout is Plan A's Python `DistributedTireForceApplier`
(`src/bike_sim/sim/ride/distributed_tire_forces.py`), itself audited in
[2026-10-11-distributed-reference-rewrite.md](2026-10-11-distributed-reference-rewrite.md).
Numeric contract: `docs/adr/0001-native-port-mujoco-core.md` §"Численный
контракт P2" — `-ffp-contract=off`, CBLAS for `@`/dot, `numpy_pairwise_sum`
for `np.sum`, `biketyre::np_interp` for `np.interp`.

Branch `feat/native-distributed-tire`, platform macOS arm64, Apple clang /
Accelerate CBLAS (the same BLAS provider NumPy binds on this machine),
MuJoCo 3.12.0, Python 3.14.3, NumPy 2.5.2.

## Commit range

The port occupies `70fc5fc^..HEAD` — nine commits, `70fc5fc` (Task 1, the
first port commit) through this audit commit, on top of Plan-A baseline
`97a35e1`:

| Commit | Task | Content |
|---|---|---|
| `70fc5fc` | 1 | `tire.distributed` config section (Python projection + C++ parse/validate); backend acceptance; writer stub |
| `19b8fd3` | 2 | `tyre/distance` `SignedProfile` port + `_debug_signed_distance` oracle binding; `native_distance_tests` |
| `8cb439d` | 3 | `tyre/station` scalar port; `writers/tire_common` extraction; `native_station_tests` |
| `0475ee8` | 4 | `DistributedTireWriter` skeleton: construction, 2N×4 state wire, `stored_energy`, debug factory |
| `87fee49` | 5 | `accumulate_validated` tick + `Stepper` variant dispatch — the bitwise gate; RTSan green |
| `9b4475a` | 6 | Probe/clock/multi-contact wire tests; stale compliant-only comments fixed |
| `4092a11` | 7 | 1 s channel-wise-bitwise trajectory; real `wheel_stand` stands; ASan/RTSan runs; two 1-ulp fixes |
| `bd2b174` | 8 | Distributed performance contract; `measure_realtime.py --benchmark`; RTF evidence |
| this | 9 | `docs/RIDE.md` capability sentence + this audit |

## Per-component parity table

Every arithmetic component of the port measured against the Python oracle.
"Bitwise" = `assert_bitwise_equal` (raw bytes, signed zeros and NaN payloads
observable) on the recorded corpus. No component carries a residual ulp
bound — the two 1-ulp defects found were bugs, fixed, and re-verified
bitwise (see the last table rows).

| Oracle op / component | Native lowering | Parity result | Evidence corpus |
|---|---|---|---|
| `np.interp` (`SignedProfile.height`) | `biketyre::np_interp` | bitwise, 0 ULP | 400-pt gate + 9000-pt extended audit (3 corpora incl. vertex hits, corners, below-surface) |
| `searchsorted` right/left | `std::upper_bound`/`lower_bound` | bitwise (`segment`, `nonsmooth` exact) | same |
| `np.dot`/`np.linalg.norm` 1-D and 2-vectors | `cblas_ddot` / `sqrt(ddot(v,v))` | bitwise, 0 ULP | same |
| `np.sum(axis=1)` / `add.reduce` on 2 terms | seeded sequential fold `(-0.0 + a) + b` (signed-zero preserved) | bitwise, 0 ULP | same |
| `np.min`/`flatnonzero`, tie scan `squared <= best + 2e-15*max(1,best)` | ordered scans, first-tie wins | bitwise (`segment` exact) | same |
| `np.clip` | sequential-comparison `clip01` | bitwise | same |
| broadcast `vector/length`, normals `-ez/len, ex/len` | elementwise divide (never reciprocal-multiply) | bitwise, 0 ULP | same |
| `SignedProfile` output fields `distance/normal/point` | — | bitwise, 0 ULP | 400-pt gate + 9000-pt audit |
| `station_angles` `2*pi*arange(n)/n`, `full(n, 2*pi/n)` | `(two_pi*i)/c`, scalar `two_pi/c` | bitwise (hexfloat `==` on oracle constants) | `test_station.cpp` 7 cases |
| `station_density`/`station_response` knot fold, `max(0.,…)` | sequential fold, `std::max` (CPython `max` semantics incl. signed-zero ties); `zip` shortest-stop | bitwise | oracle-constant `==` cases |
| `local @ R.T` (batched (N,3)@(3,3)) | **one whole-block `cblas_dgemm(RowMajor,NoTrans,Trans)`** — NumPy's batched matmul is a single gemm libcall; a per-row `dgemv` loop diverged 607/1536 elements | bitwise, 0 ULP | 1536-element comparison + 40-state gate |
| `(3,nv) jac @ qvel` | `cblas_dgemv(RowMajor,NoTrans)` | bitwise | 40-state gate + 1 s trajectory |
| `jac_station.T @ force` (`result +=`) | `cblas_dgemv(RowMajor,CblasTrans)` into `tmp_` then elementwise `+=` (oracle materializes the product) | bitwise | same |
| `np.cross` station Jacobian | unrolled product-then-subtract component order; `-ffp-contract=off` keeps mul-mul-sub unfused | bitwise | same |
| `np.sum` wheel reductions (loads, energies) | `numeric::numpy_pairwise_sum` | bitwise | same |
| `np.cos(np.radians(distinct_normal_deg))` | `std::cos(x * (M_PI/180))` — verified numpy's `radians` lowers to the same multiply, per `numeric_sincos.hpp` conventions | bitwise | same |
| `xi**2` (brush shear/energy, stored energy) | `numeric::pyfloat::pow` | bitwise | same |
| `max(0., x)` clamps (response, losses, radial power) | `std::max` | bitwise | same |
| `stored_energy` (radial + shear, `(side,station)` ascending) | sequential `+=` in oracle order | bitwise (`float.hex` equal) | 5 poses/configs incl. displaced, rolled, injected-shear |
| state wire names `f'{side}.s{index:03d}.{leaf}'` | `std::format("{}.s{:03d}.{}")` | byte-equal vs `oracle.state_names()` | round-trip test |
| full tick (`accumulate_validated`) | spec §4.3 order, lowering map `distributed_tire.cpp:553-570` | **bitwise** — qfrc, evolved state row, snapshots, diagnostics vs manifest AND live oracle | 40 alpine_rocky golden states |
| whole-episode trajectory | Stepper dispatch through `TireWriterView` | **channel-wise bitwise** (float/ndarray leaves by `tobytes()`, ints exact, str/bool/None type+value) | 1 s `environment_pair`, decimation 20 |
| `compiled_com` `np.sum(masses[:,None]*xipos, axis=0)` | **was** per-column `numpy_pairwise_sum` — **1-ULP BUG** caught by the trajectory gate (`truth.com_x_m/com_z_m`); axis=0 reduction of a C-order (n,3) product is a sequential per-column fold in numpy (verified on pinned NumPy 2.5.2, 24 000 trials, nbody 2..1000) → sequential `sum +=`; denominator stays `numpy_pairwise_sum` | bitwise after fix (`4092a11`) | failing state re-verified vs oracle |
| `research_metrics.load()` builtin `sum()` | **was** naive `+=` — **1-ULP BUG** caught by the same gate (`truth.rear_load_n`); CPython 3.12+ `sum` is Neumaier-compensated → `numeric::PythonSum` | bitwise after fix (`4092a11`) | same |

Signed-zero footnote (carried from the Plan-B ledger): the `-0.0` seed in
the rate fold is signbit-only — it cannot propagate through the subsequent
arithmetic, so no functional residual exists; recorded for completeness.

`TireWriter` (`compliant_2d`) regression: `test_native_tire.py` 36/36 green
at every task; the Task-3 `tire_common` extraction moved `surface_mu`/
`surface_at`/`TirePatch`/`TireSnapshot`/`TireDiagnostics` byte-identical
(`brush_step` repacks the shared `contactlaw::brush_step` header inline —
zero added arithmetic) and the compliant golden stayed byte-identical.

## State schema

Canonical wire: `2 * station_count * 4` columns named
`{side}.s{index:03d}.{leaf}` in side-major, station-major, leaf-major order
(`leaf ∈ {xi, tangent.0, tangent.1, tangent.2}`) — byte-equal to
`applier.state_names()`. `state()` writes NaN for inactive stations (`xi`)
and unset tangents; `set_state` is the documented strictest reading of the
oracle: unknown-name → shape → missing → decode ordering, duplicates
rejected (oracle's dict dedup keeps last), partially nonfinite tangent and
non-NaN nonfinite `xi` (±inf) rejected, staged-transactional commit, clock
restart. Round-trip byte-equal including a real `tangent.{0..2}` restore;
golden-cache codec encodes invalid tangents as zeros while `state_dict`
uses NaN — both mirrored.

Diagnostics wire (`tires.{front,rear}` channel): compliant schema unchanged
(key-disjoint — asserted by `test_tire_channel_parity_across_backends`);
distributed adds `supports_multiple_contacts=True`,
`station_count`, `active_station_count`, `nonsmooth_station_count`,
`max_patch_count`, `outside_material_deflection_range`,
`calibration_status='experimental_unvalidated'`, `capability_status`, plus
the shared `penetration_m`/energies/loss fields — key order as the oracle's
diagnostics dict.

## Stands — native vs Python

`tests/reference/test_native_distributed_stands.py` builds the real
`contact_manifold_rigs.wheel_stand` rig (hfield terrain,
collision-disabled wheel geoms; no gating needed — the Stepper constructs
on the bare stand model with a tire-only config) and runs
`_run_stand`'s loop with `st.tire_qfrc(dt)` as the force source and an
independent native energy ledger (`kinetic_energy`, `tire_stored_energy`,
`tire_brush_loss_step_j`, `tire_radial_dissipation_power_w`,
`contact_count`).

| Rig | Metric (bound) | Native measured | Python oracle (same cells) |
|---|---|---|---|
| flat (dt .000625, 128 stations, 600 N, .05 s) | `force_relative_error` ≤ .02 | **0.0** | same |
| | `energy_residual_ratio` ≤ .02 | **0.0** | same |
| | `native_wheel_contact_count` == 0 | **0** | 0 |
| step (dt .000625, dx .005, 600 N, .4 s) | `energy_residual_ratio` ≤ .05 | **0.00122765** | same |
| | `max_patch_count` ≥ 2 | **10** | 10 |
| | `native_wheel_contact_count` == 0 | **0** | 0 |
| | `dissipation_j` | 7.98888855 | 7.98888855 |
| | `final_speed_mps` | −0.73854012 | −0.73854012 |
| | `energy_residual_j` | 0.02329773 | 0.02329773 |

The Python oracle prints the same values to the last digit — effectively
bitwise stand behaviour over 640 intervals.

## Sanitizer runs

| Build | Configuration | Run | Outcome |
|---|---|---|---|
| `native/build/asan` | `NATIVE_SANITIZE=ON`; `-fsanitize=address,undefined,local-bounds,float-cast-overflow -fno-sanitize-recover=all`; documented runtime injection (`DYLD_INSERT_LIBRARIES=libclang_rt.asan_osx_dynamic.dylib`, `ASAN_OPTIONS=detect_leaks=0:detect_stack_use_after_return=1`, `PYTHONMALLOC=malloc`), loader dyld-image check verified | distributed suite 10/10 (incl. `test_distributed_qfrc_bitwise`, the 1 s trajectory, both stands), `test_native_research_environment.py` 21/21, `ctest` 6/6 | **zero sanitizer reports** |
| `native/build/rtsan` | `NATIVE_RTSAN=ON`, brew clang, `-fsanitize=realtime`; `libclang_rt.rtsan_osx_dynamic.dylib` verified present in `native_contract_tests` via `otool -L` | Task 5: full native profile **2408 tests** under the RTSan runtime; Task 7: rebuilt at `4092a11` state, distributed module 10/10 | **zero RealtimeSanitizer reports** — the marked `try_compute_into`/`BIKE_NONBLOCKING` warm path never allocates or locks |

ASan-only build note: `NATIVE_SANITIZE` appends
`-Wframe-larger-than=24576` — instrumentation inflates every alloca by
≥32-byte redzones (`accumulate_validated` measures 9152 bytes
instrumented); the production 8192-byte budget is unchanged for
non-sanitizer builds (clang last-wins verified).

## Performance contract and RTF

`alpine_rocky` + `viewer_physics_distributed.toml`, `station_count=128`
(default — the 96/64 fallback was not needed), dt .00125 s, decimation 80,
30 s ⇒ 24 000 steps, `measure_realtime.py --duration 30 --warmup 1 --runs 3`.
Evidence under `output/native-distributed-rtf/` (untracked).

| Measurement | Result | Gate | Verdict |
|---|---|---|---|
| native headless median RTF (`--benchmark`) | **6.6225** (samples 6.7339, 6.7404, 6.6084, 6.6225; all `duration_reached` 24000/24000) | contract floor `NATIVE_UNPACED_MEDIAN_RTF_MIN = 1.0` | PASS — 6.6× margin |
| python headless median RTF (`--benchmark`) | **0.2565** (samples 0.2561, 0.2320, 0.2589, 0.2565; `duration_reached`) | — | native ≈ **25.8×** faster |
| interactive HUD, real `mujoco.viewer` at `--time-scale 1` | mean `achieved_rtf` ≈ **1.00** sustained ~47 s; one transient dip 0.81 (~0.5 s during a balance-lost episode) + 1.09 catch-up, then 1.00 | spec gate ≥ 0.95 at requested 1× | **PASS** |
| native strict control (no `--benchmark`) | 4/24000 steps, `invalid_controller` | — | documented platform baseline (below) |

`--benchmark` was required, not optional: the platform
`energy.constraint_work` violation at t=0.005 s truncates every strict run
at step 4 for both backends, so only benchmark mode yields a steady-state
sample. Every benchmark run reports `valid_prefix=False` (latched
`numerically_valid`), so `headless_rtf_threshold_met` stays `None` — the
machinery's "never a valid-ride claim" contract, pinned by the distributed
contract cases in `test_native_runtime_performance_contract.py`.
`compare_runs(native, python)` → `comparable=True`,
`equally_classified_diagnostic_prefix`, `short_prefix=False`.

## Verification gates

| Gate | Result |
|---|---|
| `test_distributed_qfrc_bitwise` (slow) | 40/40 golden states bitwise — qfrc vs manifest and vs live oracle, evolved state row, snapshots, diagnostics; `resistance_components` bitwise |
| `test_distributed_trajectory_matches_python_channel_wise_bitwise` | 1 s episode channel-wise bitwise incl. truth channels; both backends hit the t=0.005 `energy.constraint_work` monitor identically step-for-step |
| Probe/clock (`test_distributed_probe_and_clock`) | probe fold bitwise vs `apply_forces(advance=False)`; committed state/clock untouched; clock `ValueError` once-per-increasing-timestamp; atomic `set_tire_state` rejection |
| Wire semantics | `supports_multiple_contacts is True`, `backend='distributed_2d_reference'` on the committed channel; compliant wire disjoint from distributed keys |
| Contract cases (`test_contracts.cpp`) | 36/36 incl. `warm_distributed_zero_allocation`, `distributed_status_control`, `distributed_probe_isolation`, `distributed_committed_snapshot_borrow`; CTest 6/6 |
| `bash tools/run_tests.sh native` (Task-9 final run) | **2416 passed, 0 failed** (332.86 s): build, `check_frontends` (gcc 16.2.0, 60 TUs), `check_tidy` none, `check_analyzer` report-only alpha, `check_cppcheck` none, `check_odr` none, `check_scripts`, headers/diagnostic-controls/context SDK, CTest 6/6, `check_native_contracts` 9/9 |
| `bash tools/run_tests.sh full` (Task-9 final run) | **24 failed, 3117 passed** (1071.77 s) — failed node IDs diff-verified **byte-identical** to the documented platform-baseline set (`docs/TESTING.md:717-742`), zero new / zero missing; none tyre-related |

## Residuals and follow-ups

- **Material uncalibrated — `experimental_unvalidated`.** The shipped
  density table is the synthetic `HingeDensity()` default
  (`calibration_status` is validated to equal `experimental_unvalidated` and
  is published on the wire). Fitting/calibration against a real tyre
  dataset is out of scope for this port — the backend is experimental and
  its physical answers are not validated material data.
- **Traction divergence vs `compliant_2d`** (spike audit
  [2026-10-11-multi-contact-tire-spike.md](2026-10-11-multi-contact-tire-spike.md)
  §5–§6, carried into Plan A's risk list): on identical safe-track geometry
  the distributed rear tyre slips ~2.74× harder at peak and ~6.5× in
  cumulative slip distance; distance reached 0.49×; mechanism suspected to
  be per-station brush saturation under distributed normal load. Front load
  share agrees within ~4 %. Consequence stands: **the multi-contact tyre's
  traction answers are not interchangeable with `compliant_2d` evidence.**
  Parity claims here are oracle-vs-port only, not backend-vs-backend.
- **`fidelity_sweep` is dead code on this branch** (all backends):
  `interval_row` (`validation/fidelity_sweep.py:167`) reads
  `sim.physical.sample` after every `sim.step`, but the PeriodBuffer
  runtime publishes `sample` only on period close → `AttributeError` on
  step 1. Additionally its hardcoded default profile
  `examples/research/plant_reference_open_loop.toml` does not exist on this
  branch and `motor_ramp_grade` raises `MOTOR_ONLY_OPEN_LOOP_UNSUPPORTED`
  unconditionally. Pre-existing platform defect recorded in the Plan-A
  audit; fix is outside this plan.
- **`energy.constraint_work` platform baseline**: first-interval violation
  at t=0.005 s fires identically on both backends and predates this branch;
  it truncates strict headless runs at step 4 (`invalid_controller`),
  forces `--benchmark` for RTF evidence, and holds
  `headless_rtf_threshold_met=None`. The `full` profile's 24-node failure
  set is the documented baseline (`docs/TESTING.md:717-742`).
- `set_state` strictness exceeds the oracle (rejects duplicate columns,
  partial-nonfinite tangents, ±inf `xi`) — a documented CONTRACT
  (`distributed_tire.hpp`); every input the oracle meaningfully restores is
  accepted identically.
- RTSan evidence at the final tree is by rerun of the unchanged marked
  path (`git diff 87fee49..HEAD` empty for the writer at the Task-7
  rebuild); the marked tick has been RTSan-clean since Task 5.
- The zero-allocation claim is writer-scoped, and stays so after the
  review-fix commit: `try_compute_into` is proven allocation-free by
  `warm_distributed_zero_allocation` plus the RTSan run; the enclosing
  `apply_forces` path is not a marked region and keeps its pre-existing
  non-tyre allocations (accumulator totals, rider envelopes). The
  tyre-specific per-step heap traffic the review found there is gone:
  `apply_forces_body` borrows committed snapshot storage for the
  resistance input instead of materializing `snapshots()` (~4 heap
  allocs + ~72 KB memcpy per active step at `station_count=128`),
  `refresh_contacts` publishes into durable `snapshots_` slots that
  engage once with the writer's bound reserved (disengaged mirrors
  the oracle's `self.snapshots = {}`) rather than a fresh
  `WheelSnapshots` pair per refresh, and the patch-load/working
  staging is sized once to the
  writer's per-wheel patch bound. The borrow/assign primitives are
  counter-proven by `distributed_committed_snapshot_borrow`;
  `apply_forces` itself cannot be instrumented because `step.cpp` is
  not linked into `native_contract_tests`.
