# G5.M - bounded mechanical-mode and guarded grip search

## Scope and status

Completed one corrective G5 subtask from
`docs/superpowers/plans/2026-10-03-v2-02-gaps.md`, not the whole G5 phase.
The latest user request authorizes exactly this continuation from the archived
pause. B1/B2/B3, R7/R8, D and W have not been started by this change.

The supplied ZIP has no `.git`. No historical commit or independent reviewer
approval is invented. The delivered patch is relative to the supplied archive,
not to its historical `3a17085` or `72afbe9c` source labels. All existing G5
corrections and review material are preserved.

## Reproduced defects and corrections

### 1. Duplicate pending modes misreported exact exhaustion

`search_constraint_modes` queued the same mode from both the failed iterate
and neighboring-mode enumeration. If the last distinct mode consumed the
candidate budget, duplicate queue entries remained and `complete` incorrectly
became false. Real MuJoCo tendon (two modes) and friction (three modes) fixtures
reproduced this independently of the optimization objective.

The pending queue now stores distinct keys and promotes an already queued
failed-iterate mode without duplication. Visited modes, deterministic neighbor
order, first-feasible early return, and best-residual result/payload/response
pairing are retained. Non-positive or non-integer candidate budgets fail before
dynamics is evaluated. The default remains 256 distinct candidates.

Exhaustion is NOT proof of physical impossibility: an optimizer may fail to find
a feasible point inside an examined mode. Truncation remains explicit and an
infeasible candidate never becomes a certified feasible command.

### 2. Guarded press branch wrote to a read-only SciPy bound

The unchanged full runtime oracle exposed
`ValueError: assignment destination is read-only` during initialization.
`allocation_grip_constraints` wrote through `LinearConstraint.lb[:]`, which
is a read-only broadcast view with this supplied runtime stack.

The guarded press row is now constructed as a new `LinearConstraint`, retaining
its matrix, upper bound, keep-feasible flags and existing 1e-5 N interior guard.
There is no force-limit change. The pull-circle guard and 300 N pull limit stay
unchanged; press magnitude remains subject to the separate joint envelope.
A real optimizer regression also verifies remembered pull-to-press fallback.

## Validation

Environment: Python 3.13.5, NumPy 2.3.5, SciPy 1.17.0, MuJoCo 3.12.0,
pytest 9.0.2, Linux x86_64. This follows the supplied Python 3.13 slim bundle;
it is not a Python 3.14/macOS/viewer qualification. System scientific packages
were reused and remaining packages loaded from `wheelhouse` through offline
`uv`; no dependency was downloaded or added to the project.

| Check | Result |
| --- | --- |
| Original reference non-slow baseline | 196 passed, 1 failed, 26 deselected |
| Mode-search RED | 7 failed, 9 passed: two exact-exhaustion regressions and five budget-validation cases |
| Guarded-grip RED | 1 failed, 1 passed, 16 deselected: read-only lower bound |
| Focused GREEN: mode search + dynamics + allocation | 84 passed, 4 deselected in 1.57 s |
| New regression cases included in GREEN | 19 passed |
| Complete reference non-slow run after correction | 215 passed, 1 failed, 26 deselected in 2.49 s |
| Preserved scalar/batched runtime oracle | 1 passed, 3 warnings in 50.22s |

The one reference-unit failure is identical before and after these edits:
`test_measurement_uses_v2_dt_configures_diagnostics_and_counts_flush` calls
`git rev-parse HEAD` through `tools.measure_realtime`, but the provided ZIP has
no Git checkout. The test was neither skipped nor weakened, and no fake Git
history was supplied to make it green.

The 19 new cases cover six actual tendon/friction transitions (predicted
acceleration and constraint multipliers checked against a fresh `mj_forward`),
exact exhaustion versus real budget truncation, minimum-residual fallback,
result/payload alignment, first-feasible stopping, invalid budgets, guarded
press/pull bounds, and remembered pull-to-press fallback.

The preserved scalar/batched/decimated oracle passed on 2003 intervals per
path, including a three-interval final flush. It compares positions,
velocities, per-interval violations, positive muscle work, absolute
constraint work and the first recorded failure. This is diagnostic-mode
equivalence evidence, NOT a strict valid-plant or realtime qualification.

## Files and evidence

Production changes are limited to
`src/bike_sim/sim/ride/rider_dynamics.py` and
`src/bike_sim/sim/ride/rider_response_allocation.py`.
New regressions: `tests/reference/test_constraint_mode_search.py`.
No track, profile, physical parameter, energy budget, tolerance or timestep was
changed. The new queue does not use previous solved reactions.

Complete RED/GREEN/baseline/runtime logs and source hashes are retained at
`.superpowers/sdd/2026-10-03-v2-seated-plant/evidence/g5-mode-search/`.
`manifest.json` records package versions, input ZIP hash, source hashes and
individual evidence hashes. An initial nine-thread BLAS attempt was interrupted
at 235.21 s (exit 2); its log is retained separately and is not a completed
result. The final full oracle uses `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`
with identical physics, inputs and assertions. Diagnostic energy rejection at
0.005 s remains visible, not waived. Review here is a focused implementation self-review,
not the historical independent whole-G review.

## Reproduction from the source checkout

After preparing the supplied offline environment as described in the bundle's
`OFFLINE_README.md`, run from the bundle root:

```sh
cd project
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export UV_OFFLINE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
uv run --no-project --python ../.venv/bin/python -m pytest \
  tests/reference/test_constraint_mode_search.py \
  tests/reference/test_rider_dynamics.py \
  tests/reference/test_rider_allocation.py -m 'not slow' -q
uv run --no-project --python ../.venv/bin/python -m pytest \
  tests/reference/test_period_buffer.py::test_runtime_batch_matches_preserved_scalar_step_and_flush -v
```

`wheelhouse/bike_sim-0.3.0-py3-none-any.whl` is intentionally unchanged: the plan
excludes wheel parity/rebuilding. Use the source path above to run this change.
The session used the same source override with the local wheel packages in a
separate target directory; automatic third-party pytest plugins were disabled.

## Remaining work

The parent G5/whole-G source review and strict physical/performance gates remain
open. In particular, this subtask does not certify full crank/cadence/coast/R6
trajectories, the 1% work budget on a valid assembled plant, 20 s track runs or
viewer realtime. Continue from the existing G5 findings and the preserved
`.superpowers` ledger; do not treat this subtask's completion as permission to
mark the whole phase green or silently begin B/R.
