# Portable distributed-tyre reference rewrite — before/after audit

Plan: `docs/superpowers/plans/2026-10-11-distributed-tire-a-portable-reference.md` (Task 7).
Spec: `docs/superpowers/specs/2026-10-11-native-distributed-tire-design.md` (§2 diagnostics, §4.1 state, §4.2 scalar formulas, §4.3 loop order, §6 freeze).
Evidence root: `output/dist-ref/{before,after}/` — untracked scratch, not committed.

## Commit range

`c794383..75609a6` — five commits, Python-only (`git diff --stat`: 11 files, +749/−144; `native/` untouched; `uv.lock`/`pyproject.toml` unchanged across the range).

| Commit | Task | Content |
|---|---|---|
| `388d464` | 2 | `station_density`/`station_response` scalar formulas (bitwise-equal to the vectorised pair); `station_angles` ceiling 512 via `DistributedTireConfig` |
| `c8646f8` | 3 | `_StationState(xi, tangent)`; canonical wire `state_names`/`state_dict`/`set_state` |
| `34263bb` | 4 | `compute_qfrc` rewritten to the spec §4.3 per-station scalar loop; `stored_energy` sequential fold; diagnostics trimmed to the 15 scalar keys of spec §2 |
| `7f1f7c3` | 5 | Four-array distributed state wire (`sides, xi, tangent, tangent_valid`) in `equilibrium_cache`, `initial_state.project_stations`; golden capture via `state_dict()` |
| `75609a6` | 6 | `alpine_rocky` preset (`ROCKY_PRESET_SEED=0`), frozen `examples/research/alpine_rocky.toml`, `examples/research/viewer_physics_distributed.toml` |

## What changed (vs the spec)

- **State trim (§4.1):** per-station state is `_StationState(xi, tangent)`; `_BrushState` no longer lives in the distributed applier's `states`.
- **Scalar loop (§4.2, §4.3):** `compute_qfrc` iterates stations in ascending `sid` order with scalar `station_response`/`station_density`; BLAS/numpy reductions only where the spec marks them (`jac @ qvel`, `jac.T @ force`, `np.sum` over `loads`/`energies`, etc.); equivalence audit in the Task-4 report.
- **Diagnostics trim (§2):** exactly `penetration_m, normal_load_n, multi_support, supports_multiple_contacts, capability_status, calibration_status, station_count, active_station_count, nonsmooth_station_count, max_patch_count, outside_material_load_range, outside_material_deflection_range, radial_energy_j, shear_energy_j, brush_loss_j`; `active_station_ids`, `road_segment_ids`, `road_projection_m` dropped.
- **Freeze (§6):** pinned step-rig trace test; pinned `alpine_rocky` preset + frozen track + distributed physics profile for the port's golden/performance gates.

## Evidence provenance

The original Task-1 capture was destroyed by the Task-6 smoke cleanup (task-06 report: "All smoke dirs deleted (`output/dist-ref/` removed)"). Before-evidence was therefore **recaptured at the identical commit** `c794383` in a detached worktree (`.worktrees/dist-ref-before`, removed after the run), with the plan's Task-1 Step-4 commands verbatim and outputs written to `output/dist-ref/before/`. Environment parity: CPython 3.14.3 both sides, MuJoCo 3.12.0, identical `uv.lock` (unchanged across the range). The recaptured signature reproduces the Task-1 report's recorded values exactly — contact matrix 24/24 accepted; fidelity 49 series = 42 `numerical_failure` + 7 `initialization_failure`, `passed=false`/`incomplete_evidence`.

The fidelity workaround file was deleted with the same cleanup and was reconstructed identically: `git show 0408101:examples/research/plant_reference_open_loop.toml` verbatim at `output/dist-ref/plant_reference_open_loop.toml`, and `output/dist-ref/plant_reference_open_loop_compat.toml` = that file minus the four `[drive.assist]` keys this branch's `AssistConfig` rejects (`stop_delay`, `spin_rpm`, `stall_timeout_s`, `boost_s`) and their comment lines. After-evidence used the same commands at `75609a6` in the main checkout.

Commands (both sides, plan Task-1 Step 4 + the Task-1 workaround flag):

```bash
uv run --frozen python - <<'EOF'   # contact_matrix, written to before/ or after/
import json
from bike_sim.validation.contact_acceptance import contact_matrix
rows = contact_matrix(dts=(.00125,.000625), station_counts=(128,256),
                      flat_loads=(300.,600.,900.), step_dxs=(.01,.005))
json.dump(rows, open('output/dist-ref/<side>/contact_matrix.json','w'), indent=1, default=float)
EOF
uv run --frozen python -m bike_sim.validation.fidelity_sweep \
  --backend distributed_2d_reference --stations --jobs 3 \
  --physics-config output/dist-ref/plant_reference_open_loop_compat.toml \
  --output output/dist-ref/<side>/fidelity
```

## Stand matrix before/after (`contact_matrix.json`, 24 rows)

| Measure | Before (`c794383`) | After (`75609a6`) |
|---|---|---|
| Accepted | **24 / 24** | **24 / 24** |
| Per-row `checks` bounds | — | all identical (0 diffs) |
| `accepted` flags | — | all identical (plan assertion holds) |
| Physical metrics | — | **bitwise identical (0.0 drift)** — `normal_force_n`, `normal_load_n`, `normal_impulse_ns`, `vertical_impulse_ns`, `total_vertical_support_n`, `deflection_m`, `expected_normal_n`, `tangent_fixture_reaction_n`, `force_relative_error`, `normal_relative_error`, `vertical_relative_error`, `speed_mps`, `final_speed_mps`, `max_patch_count`, `multi_support_fraction`, `station_count`, `terrain_dx_m`, `angle_deg` |
| Max relative drift (plan formula) | — | **1.776e-3**, confined to `max_energy_residual_j` |

Cells over the 1e-9 relative target — exactly three, all `max_energy_residual_j` on `flat` rows:

| case | dt | stations | load N | before J | after J | rel | abs |
|---|---|---|---|---|---|---|---|
| flat | .00125 | 128 | 900 | 3.28626015289e-14 | 3.10862446895e-14 | 1.78e-3 | 1.78e-15 |
| flat | .00125 | 256 | 600 | 2.97539770600e-14 | 3.01980662698e-14 | 4.44e-4 | 4.44e-16 |
| flat | .000625 | 256 | 600 | 2.97539770600e-14 | 3.01980662698e-14 | 4.44e-4 | 4.44e-16 |

Other energy-bookkeeping metrics moved below 1e-9: `energy_residual_j` max rel 6.16e-14 (abs 1.78e-15 J), `energy_residual_ratio` 6.14e-14 (abs 9.2e-17), `dissipation_j` 2.16e-16 (abs 1.78e-15 J).

Cause: the documented reduction-order change in `stored_energy`/diagnostics (sequential per-station fold replacing `np.dot`/`np.sum` pairwise reductions — Task-4 equivalence audit, spec §4.3). The metric is `|work − stored|` on O(10–100 J) quantities, so a 1-ulp reorder of the stored fold shifts the near-zero residual by ~1e-15 J; the values it moves are themselves ~3e-14 J, i.e. machine-noise floor. **No physical output, bound check, or acceptance flag moved.**

## Fidelity sweep before/after (`fidelity/`)

| Measure | Before (`c794383`) | After (`75609a6`) |
|---|---|---|
| Series | 49 | 49 |
| `numerical_failure` | 42 | 42 |
| `initialization_failure` | 7 | 7 |
| Per-series termination match (case/dt/dx/stations/axis) | — | **0 mismatches** |
| Series producing `metrics` | 0 | 0 |
| Comparisons | 63, all `incomplete_evidence` | 63, all `incomplete_evidence` |
| `passed` / `failed_criteria` | `false` / `['incomplete_evidence']` | `false` / `['incomplete_evidence']` |
| `axes_passed` | all `false` | all `false` |
| `source_unchanged_during_run` | `true` | `true` |
| `source_sha256` | `5ae893924d75f10e…` | `bd63f93ef324f239…` (differs — src changed, expected) |
| `worker_errors` | none | none |

Identical root cause on both sides, e.g. `static_sag_dt0.0003125_dx0.005_n256.json.gz`: `termination=numerical_failure`, 0 rows, `AttributeError: 'NoneType' object has no attribute 'channels'` at `fidelity_sweep.py:167` (`s = sim.physical.sample`). Equilibrium itself converges on both sides (static_sag anchor: 4800 steps, `cache_hit=false`; target qacc residual 0.02925156102103655 → 0.02925156102097003, a ~6.7e-17 reorder-level move). `motor_ramp_grade` fails identically at anchor construction: `ValueError: motor-only open-loop replay is unsupported by the pedelec gate…` (`MOTOR_ONLY_OPEN_LOOP_UNSUPPORTED`, `fidelity_sweep.py:133-134`), so its 7 series report `initialization_failure` ("shared candidate initialization failed").

**The sweep is broken identically before and after — no regression is attributable to the rewrite.** Per the Task-1 ledger ruling this is recorded as a pre-existing platform finding, not as evidence quality:

- `interval_row` (`fidelity_sweep.py:167`) reads `sim.physical.sample` after every `sim.step`, but the PeriodBuffer runtime publishes `sample` only on period close (`physical_runtime.py:804`, `self.sample = published[-1]` inside the close block; initialised `None` at :104 and re-cleared at :404). With `control_period_s = .005` = 4 × `dt .00125`, the first step of every period leaves `sample=None` → `AttributeError` on step 1. This kills the sweep for **all** backends on this branch.
- The hardcoded default profile `examples/research/plant_reference_open_loop.toml` (`fidelity_sweep.py:137`) does not exist on this branch (added by `0408101` on `origin/antiwheelie-refocus`, never merged here; schema-incompatible as noted above).
- `motor_ramp_grade` raises `MOTOR_ONLY_OPEN_LOOP_UNSUPPORTED` unconditionally (`fidelity_sweep.py:133-134`).

Fixing the sweep is out of scope for this plan; the load-bearing before/after evidence is the contact-matrix diff plus the pinned step trace below.

## Step-rig pinned trace

`test_step_rig_matches_the_pinned_trace_to_reordering_tolerance` (tolerance `rtol = atol = 1e-9`), re-measured at `75609a6`:

- Max relative drift vs the pinned `BEFORE_*` trace: **7.734e-13** (both normal and vertical channels) — within the 1e-9 reordering tolerance.
- Max absolute deviation: 4.748e-10 N.
- Patch sequence identical `[9,8,8,8,8,8,8,8,9,9]`; `max_patch_count` 9 = `BEFORE_MAX_PATCHES` ≥ 2.
- `energy_residual_ratio` 6.70e-4 ≤ .05; `native_wheel_contact_count` 0.
- Task 4 measured the same **7.73e-13** at `34263bb`; the value is stable through Tasks 5–6 (state wire and preset work did not move it). Reference suite: **8 passed**.

## Residuals

- Three `max_energy_residual_j` cells above the 1e-9 relative target (table above): ≤1.78e-15 J absolute on ~3e-14 J noise-floor values; cause = the spec-§4.3-mandated sequential reduction order in `stored_energy` (diagnostics-only channel; no force, patch, bound, or flag moved).
- Anchor equilibrium residual moved ~6.7e-17 absolute between the two fidelity runs — same reorder class.
- Carried from task reports: Task-4 parked minors (signed-zero edge in the rate fold — footnote for the Plan-B ulp table; docstring nits; `np.float64` accumulator leakage); Task-5 wire-encoding note (cache JSON encodes invalid tangents as zeros while `state_dict()`/golden rows use NaN — the C++ wire reader must mirror both); Task-6 smoke deviation (prescribed smoke exits 2 on the backend-independent `energy.constraint_work` gate, spike M4/M6; `supports_multiple_contacts` lives in `intervals.jsonl`, not `summary.json`).

## Verdict

The portable rewrite introduces **no physics drift**: every physically meaningful stand metric is bitwise identical before/after, all 24 acceptance flags and bound checks are unchanged, and the pinned step trace holds at 7.7e-13 relative drift against a 1e-9 tolerance. The only movement is machine-noise-floor energy bookkeeping (≤1.8e-15 J) produced by the spec-mandated sequential reduction order, plus the identically-broken `fidelity_sweep`, which is a pre-existing platform defect on this branch — documented above for fixing outside this plan, and confirmed unchanged by the rewrite.
