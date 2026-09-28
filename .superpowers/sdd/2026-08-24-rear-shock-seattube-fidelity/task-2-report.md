# Task 2: Constrained Hardpoint Fitter — Report

**Status: DONE_WITH_CONCERNS** (fit meets every tolerance with margin; the concern is that
Nelder-Mead terminated on the evaluation budget rather than on its tolerance — see Concerns.)

## What I implemented

- `tools/fit_hardpoints.py` — offline constrained least-squares fit of the eight suspension
  hardpoints to the Task 1 photo targets, under two hard equalities (stroke at full travel
  = 65.0 mm, shock eye-to-eye = 205.0 mm) plus the rigid `CHAINSTAY_SHIFT_MM = 21.0`
  forward translation of the rear group.
- `docs/reference/fitted_hardpoints.json` — the generated constants (`points_mm`,
  `kinematics`, `residuals_mm`, `chainstay_shift_mm`, `shock_eye_to_eye_mm`).
- `tests/test_fitted_hardpoints.py` — verbatim from the brief; validates the committed
  constants, never re-runs the optimiser.

`scipy` is imported only by this tool and was **not** added to `pyproject.toml`
(`grep -c scipy pyproject.toml` → 0). Everything ran through `uv` per AGENTS.md.

### Rulings applied

**R6 — no trajectory inside the objective.** The objective calls
`_stroke_at_full_travel()`, which does
`solver.solve_state_from_wheel_travel(180.0)["shock_stroke"]` — one root-find on top of the
101 the `HorstLinkageSolver` constructor unavoidably spends on its travel/stroke lookup
table. `evaluate()` is retained, builds the full 41-point trajectory, and is called exactly
once after convergence to produce the reported kinematics. Measured effect: the R6 path is
~43 ms/eval versus ~70 ms for `evaluate()` — a ~39% saving, which is what made an 8000-eval
budget fit in ~6 minutes instead of ~9.5.

**R2 (amended) — budget.** Started at 2000, then raised to the 8000 ceiling (see
Iteration budget below). Never approached the 15-minute limit.

### One deviation from the brief, with evidence

The brief's `_seed()` applies `lift=10.8` to P2 in Z and then sets `goal = seed`, so the
residual objective is pulled to the *lifted* P2. But the brief's own fidelity test
(Step 1) measures P2 against the **unlifted** photo Z with an 8.0 mm tolerance. That is
self-contradictory: it bakes in a 10.8 mm error against an 8.0 mm limit. Verified directly:

```
brief seed (lift=10.8) max_stroke: 68.277   <- matches the controller's quoted 68.322
my seed (no lift)      max_stroke: 66.345
If goal==seed as the brief writes it, the fit is pulled to P2_z = 2.5517
but the fidelity test measures against photo P2_z = -8.2483
=> baked-in P2 error of 10.8 mm vs an 8.0 mm tolerance
```

(The 68.277 vs 68.322 gap is a rounding-level difference in how P6 is projected; it confirms
the controller's known-good number was computed *with* the lift.)

I resolved it in favour of the stated fidelity requirement ("fitted pivots within 8 mm of
photo targets") by splitting the two concepts that the brief conflated:

- `_photo_goal()` — what the fit is **scored against**: raw photo positions, X-shifted by
  21.0 mm for P2 only. This is exactly what the test measures.
- `_seed()` — where the search **starts**: the goal positions, with P6 projected onto the
  205 mm eye-to-eye line so the optimiser begins on the constraint manifold.

The lift is dropped entirely. It was never a hard constraint, and removing it starts the
search *closer* to target (66.345 vs 68.277 mm stroke). Resulting P2 residual: **0.18 mm**.

### One deliberate improvement

Nelder-Mead's default initial simplex perturbs each coordinate by 5% of its own magnitude,
which across hardpoints spanning -400 mm to +400 mm means steps from 0.4 mm to 20 mm — wildly
non-uniform for what is a single-length-scale geometry problem. I pass an explicit
`initial_simplex` with a uniform `SIMPLEX_STEP_MM = 1.5` step. Same cost per evaluation,
much better conditioning.

## Measurements required by R2

| Quantity | Value |
|---|---|
| Per-evaluation time (R6 objective path) | **42.7–46.8 ms** (43 ms typical) |
| `evaluate()` full-trajectory path, for comparison | ~70 ms |
| Budget attempt 1 | 2000 evals, **85.8 s** wall, cost 25.490 |
| Budget attempt 2 (final) | **8000 evals**, **374.5 s** wall (6 min 15 s total incl. startup), cost 21.034 |
| Dimensionality | 16 (8 points × X,Z) |

**Why I raised 2000 → 8000.** At 2000 the run terminated on `maxfev` with cost 25.490. The
tell that it was genuinely unconverged: **P12 has zero kinematic influence** — it feeds only
`r_2_12`, `gamma_12`, `l_dropout_height` and `l_ss_tube`, none of which reach stroke, travel
or leverage ratio — so at a true optimum its residual must be exactly 0. It was 0.4863 mm.
At 8000 it fell to 0.2204 mm and cost dropped to 21.034, confirming real progress. I stopped
at 8000 because that is the ruling's ceiling.

## Converged result

```
max_stroke              65.000068 mm     (target 65.0 ± 0.01)  ✓
initial_leverage_ratio  3.364457         (band 2.9 – 3.4)      ✓
final_leverage_ratio    2.404184         (band 2.1 – 2.6)      ✓
progressivity_pct       28.541706        (band 15 – 32)        ✓
max_link_error          0.0              (< 1e-6)              ✓
shock_eye_to_eye        204.999952 mm    (205.0 ± 0.5)         ✓
leverage ratio monotonically decreasing  ✓
```

### Per-point residuals (mm) — tolerance 8.0, measurement noise floor ±6.0

| Point | Residual | | Point | Residual |
|---|---|---|---|---|
| P0 | 0.4572 | | P5 | 0.2503 |
| P2 | 0.1762 | | P6 | **3.3895** ← largest |
| P3 | 0.4122 | | P7 | 2.4620 |
| P4 | 1.7212 | | P12 | 0.2204 |

Every pivot lands inside the ±6 mm measurement noise floor, i.e. well inside the 8 mm
fidelity tolerance.

**Why P6/P7 carry the largest residuals, and why that is correct.** The photo implies an
eye-to-eye of `|P7-P6| = 199.18 mm`, but the spec mandates 205.0 mm. That 5.82 mm
discrepancy is physically real and must be absorbed somewhere. The fit splits it along the
shock axis between the two eyelets (3.39 + 2.46 ≈ 5.85), rather than dumping it all on one
end as the seed projection did (P6 alone at 5.82). That is the right resolution, and both
ends stay inside the noise floor.

Independent cross-checks:
- `BB → P1` = 447.5012 mm — the spec chainstay, from `compute_rear_axle`, not fitted.
- Photo-implied chainstay `|rear_axle|` = 468.35 mm → implied shift 20.85 mm, consistent
  with `CHAINSTAY_SHIFT_MM = 21.0`.
- The JSON is self-consistent: recomputing the kinematics from the **rounded** committed
  points reproduces the stored `kinematics` block to all 6 decimals, so the 1e-6 mm
  rounding is harmless and the tests validate what is actually on disk.

The output JSON was never hand-edited.

## TDD evidence

### RED

```
$ uv run pytest tests/test_fitted_hardpoints.py -v
E       AssertionError: run: uv run --with scipy --with pillow python -m tools.fit_hardpoints
E       assert False
E        +  where False = exists()
E        +    where exists = PosixPath('.../docs/reference/fitted_hardpoints.json').exists
============================== 5 errors in 0.06s ===============================
```

Expected: the module-scoped `fitted` fixture asserts the generated JSON exists, and
`tools/fit_hardpoints.py` had not been written yet, so all 5 tests error at fixture setup.

### GREEN

```
$ uv run pytest tests/test_fitted_hardpoints.py -v
tests/test_fitted_hardpoints.py::test_binding_equality_stroke_is_exactly_65 PASSED [ 20%]
tests/test_fitted_hardpoints.py::test_four_bar_closes PASSED             [ 40%]
tests/test_fitted_hardpoints.py::test_leverage_curve_is_progressive_and_sane PASSED [ 60%]
tests/test_fitted_hardpoints.py::test_pivots_stay_within_fidelity_tolerance_of_the_photo PASSED [ 80%]
tests/test_fitted_hardpoints.py::test_shock_eye_to_eye_matches_spec PASSED [100%]
============================== 5 passed in 0.22s ===============================
```

### Full suite (run once before committing)

```
$ uv run pytest -q
55 passed in 4.54s
```

No pre-existing test regressed.

## Files changed

| File | Status |
|---|---|
| `tools/fit_hardpoints.py` | new |
| `tests/test_fitted_hardpoints.py` | new (verbatim from brief) |
| `docs/reference/fitted_hardpoints.json` | new (generated) |

No runtime module was modified; `pyproject.toml` is untouched.

## Self-review findings

1. **Removed an unused parameter.** `fit()` initially took `max_evaluations`, but nothing
   ever passed it (I stepped the budget by editing the constant). Removed — YAGNI, and it
   brings the signature closer to the brief's `fit(targets)` spec. Re-smoke-tested
   afterwards at a 30-eval budget, confirming the module still runs end-to-end and that the
   committed JSON was byte-identical (`diff -q` → IDENTICAL).
2. **`_photo_goal` / `_seed` near-duplication is intentional.** They differ only by the P6
   projection, but they answer different questions (what we score against vs. where we
   start). Collapsing them is precisely the conflation that produced the brief's P2 bug, so
   the split stays, with docstrings explaining why.
3. **Added `shock_eye_to_eye_mm` to the payload.** A hard constraint whose satisfaction is
   otherwise invisible in the output. All three brief-mandated keys are present.
4. **`INFEASIBLE_COST` as a large finite value, not `inf`**, so Nelder-Mead's shrink steps
   stay well defined when a candidate leaves the four-bar's feasible manifold.
5. **Do the tests verify real behaviour?** Yes — they reconstruct a solver from the
   committed constants and re-derive stroke, link closure, leverage curve and monotonicity
   from scratch. They are not assertions about the optimiser's self-reported numbers; note
   that `kinematics` in the JSON is *reported* data while the tests independently recompute
   it, and I checked the two agree.
6. **Naming/verbatim values.** `CHAINSTAY_SHIFT_MM = 21.0`, `TARGET_STROKE_MM = 65.0`,
   `TARGET_E2E_MM = 205.0`, penalty weights `4.0e4` — all as specified. Neither penalty
   weight needed adjusting: stroke landed within 6.8e-5 mm and no residual came near 8 mm.

## Concerns

1. **The optimiser terminated on budget, not on tolerance** (`success=False`,
   "Maximum number of function evaluations has been exceeded"). Nelder-Mead in 16
   dimensions wants ~1600–3200 evaluations *per restart* and got 8000 total. The residual
   fit could still be shaved: P12's 0.2204 mm residual is provably pure slack, since P12
   has no kinematic influence at all. **This does not threaten any requirement** — the hard
   equalities are met to 6.8e-5 mm (stroke) and 4.8e-5 mm (eye-to-eye), and every pivot is
   inside the ±6 mm noise floor, so further iterations would only polish numbers that are
   already far better than the ±6 mm the photograph can actually resolve. I flag it because
   the run is not reproducibly *converged*, only reproducibly *good enough*. If a later task
   ever needs tighter pivot fidelity, the fix is a polish stage (feed `result.x` back in as
   a fresh simplex, or drop P12 from the optimisation vector since it is a passenger,
   reducing the search to 14 dimensions) rather than a larger single budget.
2. **P12 wastes 2 of 16 search dimensions.** It has zero effect on the objective's
   constraint terms and its residual is minimised trivially at its authored seed. Holding
   it fixed and emitting it unchanged would make the search meaningfully cheaper. I kept it
   free because the brief's `FREE` tuple and output contract both list it, and I did not
   want to alter the produced interface unilaterally.
3. **`initial_leverage_ratio = 3.364` sits near the 3.4 upper bound** of the sanity band
   (the seed was 3.281). It passes, and the progressivity and final LR are comfortably
   mid-band, but downstream tasks tightening this band should know the margin is ~0.036.

---

# Fix round 2 — report

**Status: DONE**

Picked this up after two prior fix-round attempts were killed by infra (API 529), not by
anything wrong with their work. The checkout had an uncommitted, never-run rewrite of
`tools/fit_hardpoints.py` (127 insertions / 53 deletions) already sitting in the tree. I
read it in full against the prescription in `task-2-brief.md`'s fix-round instructions and
found it already correctly implemented both required fixes. I did not need to rewrite
anything in `tools/fit_hardpoints.py` — verification only. What I did:

## What I verified was already correct in the pending rewrite

**Important #1 (P6 collinearity).** `FREE` dropped `P6`, leaving `("P0","P2","P3","P4",
"P5","P7","P12")` — 7 points × 2 (X,Z) = 14 dims, down from 16. A new `_derive_p6(p4, p7)`
computes `P6 = P7 - 205*unit(P7-P4)` — bit-for-bit the convention documented at
`linkage_solver.py:239-245` — and `_unpack()` appends it to every candidate before the
solver ever sees it. The eye-to-eye penalty term was deleted from the objective (`cost()`
now scores only the stroke-excursion equality). `SCORED` (separate from `FREE`) still
includes `P6` so its reported residual is against the *realised, derived* pivot, not a
free parameter — exactly what the brief's R9 discussion implies downstream consumers read.

**Important #2 (per-evaluation cost).** A tools-local `_FitSolver(HorstLinkageSolver)`
overrides `solve_state_from_wheel_travel` to short-circuit during `__init__` only (a
`self._skip_table` flag flips true for the constructor's superclass call, then false
immediately after), so the 101-point Brent lookup table is built with the returned data
discarded and is never consulted by the fit loop (`solve_state_from_shock_stroke` is
overridden to raise `NotImplementedError`, since the objective never calls it). A new
`_stroke_excursion()` calls `solve_state_from_wheel_travel` at 0 and at 180 mm directly on
a `_FitSolver` instance and subtracts, replacing the old `_stroke_at_full_travel()` that
implicitly assumed rest stroke was zero — the exact assumption Important #1 broke. The
full-fidelity `evaluate()` (unmodified `HorstLinkageSolver`, full 41-point trajectory) is
retained and called exactly once, after convergence, for the reported kinematics — never
inside the objective.

The optimiser loop was rewritten to restart from the incumbent with a fresh initial
simplex (`MAX_RESTARTS = 25`, `MAX_EVALUATIONS_PER_RESTART = 20000`) until
`scipy.optimize.minimize` itself reports `success=True`, rather than the fix-round-1
partial's still-present fixed budget. This runs to convergence rather than terminating on
an evaluation cap.

## What I changed myself

1. **Minor #4 test changes** (not present in the pending `tools/fit_hardpoints.py`
   rewrite, since that only touched the tool): edited `tests/test_fitted_hardpoints.py`:
   - Added `test_shock_stroke_is_zero_at_zero_travel`, asserting
     `solve_state_from_wheel_travel(0.0)["shock_stroke"] == pytest.approx(0.0, abs=1e-6)`
     — the invariant Important #1 actually violated.
   - Rewrote `test_shock_eye_to_eye_matches_spec` to assert against the *realised* eyelet
     (`solve_state_from_wheel_travel(0.0)["shock_length"]`, which equals `|P7-P6|` at rest
     once P6 is collinear) rather than `np.linalg.norm(p7 - p6)` on the stored constant, and
     tightened `abs=0.5` to `abs=1e-6` — since P6 is now derived collinear at exactly 205 mm
     by construction, the realised value closes to machine precision (measured:
     `205.0000002282594`, i.e. 2.3e-7 mm off, an artifact of the JSON's 6-decimal rounding).
2. Ran the fitter, regenerated `docs/reference/fitted_hardpoints.json`, ran both test files
   plus the full suite, and committed.

## Converged result (this run)

```
$ time uv run --with scipy --with pillow python -m tools.fit_hardpoints
  restart 0: cost 19.928851248  evals 14043  success=True
  total evaluations 14043  wall 10.3s  (0.730 ms/eval)  success=True
Wrote docs/reference/fitted_hardpoints.json
{
  "max_stroke": 65.000040368,
  "stroke_at_zero_travel": 0.0,
  "stroke_excursion": 65.000040368,
  "initial_leverage_ratio": 3.349454568,
  "final_leverage_ratio": 2.411762626,
  "progressivity_pct": 27.995362339,
  "max_link_error": 0.0
}
{
  "P0": 0.5855, "P2": 0.1945, "P3": 0.7225, "P4": 0.7306,
  "P5": 0.6629, "P6": 3.0852, "P7": 2.9213, "P12": 0.0132
}
uv run ... 10.41s user 0.20s system 94% cpu 11.233 total
```

`optimiser` block in the committed JSON: `{"method": "Nelder-Mead", "dimensions": 14,
"evaluations": 14043, "converged": true, "wall_seconds": 10.3}` — **success=True on the
first restart**, no budget cap hit, well under the 15-minute wall-time ceiling.

Per-evaluation time: **0.730 ms** (down from the fix-round-1 partial's target of ~100x
cheaper than the original ~43 ms/eval — measured here at ~59x, still comfortably in the
regime that lets the search run to convergence rather than to a budget). Budget used:
14043 of a possible 20000-per-restart cap (1 restart, 25 available). Total wall time:
10.3 s reported / 11.2 s including process startup — three orders of magnitude under the
~15-minute ceiling.

**stroke at zero travel: `0.0`** (rounded; exact value `5.68e-14`, i.e. floating-point
zero). **Realised eye-to-eye** (`shock_length` at rest, reconstructed from the committed,
rounded JSON points): `205.0000002282594` mm.

### Per-point residuals (mm) — tolerance 8.0, noise floor ±6.0

| Point | Residual | | Point | Residual |
|---|---|---|---|---|
| P0 | 0.5855 | | P5 | 0.6629 |
| P2 | 0.1945 | | P6 | **3.0852** ← largest |
| P3 | 0.7225 | | P7 | 2.9213 |
| P4 | 0.7306 | | P12 | 0.0132 |

All comfortably inside the 8 mm tolerance and the ±6 mm noise floor. (P6's residual
compares the *derived* pivot to its photo target — this is the honest number the collinear
fix implies; it did not get worse than the previous, physically-broken fit's 3.3895 mm.)

## Test evidence

```
$ uv run pytest tests/test_fitted_hardpoints.py -v
tests/test_fitted_hardpoints.py::test_binding_equality_stroke_is_exactly_65 PASSED [ 16%]
tests/test_fitted_hardpoints.py::test_shock_stroke_is_zero_at_zero_travel PASSED   [ 33%]
tests/test_fitted_hardpoints.py::test_four_bar_closes PASSED                      [ 50%]
tests/test_fitted_hardpoints.py::test_leverage_curve_is_progressive_and_sane PASSED [ 66%]
tests/test_fitted_hardpoints.py::test_pivots_stay_within_fidelity_tolerance_of_the_photo PASSED [ 83%]
tests/test_fitted_hardpoints.py::test_shock_eye_to_eye_matches_spec PASSED         [100%]
============================== 6 passed in 0.27s ===============================

$ uv run pytest tests/test_kinematics.py -v
tests/test_kinematics.py::TestBicycleGeometry::test_fixed_frame_points PASSED
tests/test_kinematics.py::TestBicycleGeometry::test_frame_angles PASSED
tests/test_kinematics.py::TestBicycleGeometry::test_steering_geometry_and_offset PASSED
tests/test_kinematics.py::TestBicycleGeometry::test_wheelbase_and_bb_drop PASSED
tests/test_kinematics.py::TestLinkageKinematics::test_leverage_ratio_progressive PASSED
tests/test_kinematics.py::TestLinkageKinematics::test_rigid_length_invariance PASSED
tests/test_kinematics.py::TestLinkageKinematics::test_shock_stroke_and_travel PASSED
tests/test_kinematics.py::TestLinkageKinematics::test_transmission_angle_safe_range PASSED
tests/test_kinematics.py::TestMuJoCoExport::test_json_export_structure PASSED
tests/test_kinematics.py::TestMuJoCoExport::test_mujoco_xml_validity PASSED
tests/test_kinematics.py::TestConfigAndInverseSolver::test_bike_config_composition PASSED
tests/test_kinematics.py::TestConfigAndInverseSolver::test_shock_stroke_inverse_solver PASSED
============================== 12 passed in 0.50s ==============================

$ uv run pytest -q
56 passed in 4.93s
```

No pre-existing test regressed; `pyproject.toml` untouched (`grep -c scipy pyproject.toml`
→ 0).

## Self-review (fresh eyes)

1. **Completeness.** Both Important fixes verified present and correct; Minor #4 applied.
   R9's downstream `st0["P6"]` consumers (`export_mujoco.py:119,2319`) now read the
   collinear-derived value, not a free parameter — the fix actually reaches them, it's not
   just a fitter-local nicety.
2. **Naming.** `_derive_p6`, `_stroke_excursion`, `_FitSolver`, `FREE`/`SCORED` split are
   self-explanatory and each carries a docstring explaining *why*, not just *what* —
   important given this is the second implementer to touch this file.
3. **YAGNI check on the restart loop.** `MAX_RESTARTS = 25` is new machinery beyond the
   original single `minimize()` call. It's justified: it directly implements the brief's
   "let the optimiser converge on tolerance rather than terminating on budget," and it cost
   nothing here (converged on restart 0). I did not add anything beyond what R9/R10/R11
   already settled, and left all R11-deferred minors untouched (`test_four_bar_closes`,
   the duplicated `p12_0` literal, the `except` tuple, `_brentq_pure`'s silent
   `maxiter` return, `max_evaluations` as a module constant — none touched).
4. **Do the tests verify real behaviour?** Yes for the new ones: both reconstruct a fresh
   `HorstLinkageSolver` from the committed JSON and call the real kinematics
   (`solve_state_from_wheel_travel(0.0)`) rather than re-deriving from `points_mm` in
   Python — so they'd catch a solver that silently used a different P6 than the one
   stored, not just a JSON arithmetic check.
5. **Nothing hand-edited.** `docs/reference/fitted_hardpoints.json` is fully
   tool-generated; verified via `git diff` before commit that only the fitter wrote it.

## Commit

`05221c3` — `fix(geometry): derive P6 collinear, drop redundant root-finds in hardpoint fit`
(`tools/fit_hardpoints.py`, `tests/test_fitted_hardpoints.py`,
`docs/reference/fitted_hardpoints.json`).

## Concerns

None blocking. Two notes for later tasks, in case they matter:

1. `initial_leverage_ratio = 3.349` (this run) is comfortably inside the 2.9–3.4 band,
   with more margin to the upper bound than fix-round-1's 3.364 had — no action needed,
   noted only because a prior report flagged the margin as tight.
2. P12 still occupies 2 of 14 search dimensions despite having zero kinematic influence
   (confirmed again: residual 0.0132 mm, effectively at its seed). This was flagged as a
   possible future optimisation in the original report and remains true, but is out of
   scope for this fix round (R11 keeps `max_evaluations`/dimensionality choices out of
   scope) and does not affect correctness.
