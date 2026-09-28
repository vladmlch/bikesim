# Task C2 implementation report

Base: `37f292d` on `pedals`. Commit: `ec648a3` (`fix: separate time-based grounded filter from physical loads`).

## Change

- Added `GroundedFilter(hold_s)` with finite/nonnegative validation, an idempotent timestamp update, backward/non-finite timestamp rejection, and full reset.
- `TerrainContactQuery` now derives two separate controller booleans from current `terrain` patch normal loads above `CONTACT_LOAD_THRESHOLD_N` (1 N). Each boolean is held for 5 ms by a timestamp filter. `catch_plane` patches cannot refresh the hold.
- The physical native-reference cruise path explicitly passes `rear_controller_grounded` to `CruiseController`. Its legacy call keeps the existing `rear_in_contact` gate. Physical pedal/drivetrain modes are currently rejected by `RideSimulation`, so there is no active physical drivetrain gate to change.
- Existing bridged `front_load_n`/`rear_load_n`, raw `front_support_n`/`rear_support_n`, legacy `front_in_contact`/`rear_in_contact`, pneumatic output, rolling resistance, contact forces, HUD, recorder, and crash detection retain their prior paths. `TerrainContactQuery.reset()` clears both legacy bridges and the new filters.

Changed source/tests: `src/bike_sim/sim/ride/contact_filter.py`, `src/bike_sim/sim/ride/contacts.py`, `src/bike_sim/sim/ride/cruise.py`, `src/bike_sim/sim/ride_sim.py`, `tests/test_contact_filter_time.py`. No dependency or lockfile changes.

## Test-first and verification transcript

All Python commands used `uv` with the existing lockfile and `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928`.

1. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_filter_time.py -q` after writing the exact required timestamp test and initial behavioral tests: exit 2 during collection, `ModuleNotFoundError: No module named 'bike_sim.sim.ride.contact_filter'` (expected red).
2. Same focused command after the filter and contact-query implementation, with the physical cruise integration test present: exit 1, `10 passed, 1 failed`; `test_physical_native_cruise_uses_controller_grounded_instead_of_held_load` showed `sim.cruise.engaged == True` with `rear_controller_grounded=False` (expected red for missing wiring).
3. Same focused command after physical cruise wiring: exit 0, `11 passed in 1.65s`.
4. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q`: exit 1, `896 passed, 4 errors in 284.54s`. All four errors were setup errors in `tests/test_render_comparison.py`, caused by `mujoco.cgl.cgl.CGLError: invalid CoreGraphics connection` while creating `mujoco.Renderer`; no test assertions failed.
5. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_filter_time.py -q -k requires_more_than_one_newton`: exit 1, `1 failed, 11 deselected`. A native `terrain` row carrying 0.5 N had `road_loaded_contact=True` and `rear_controller_grounded=True`, proving the missing 1 N threshold (expected red).
6. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_contact_filter_time.py -q` after applying the threshold: exit 0, `12 passed in 2.95s`. The tests cover the exact timestamp example; non-finite/negative hold and time, backward time, reset, 0.5/0.25/0.125 ms step sizes, catch-plane exclusion, raw load preservation, 1 N gate, and physical cruise routing.
7. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q` after the threshold change: exit 1, `897 passed, 4 errors in 292.04s`. The same four render fixture setup errors had the same CoreGraphics connection cause; no assertion failures.
8. `git diff --check`: exit 0, no whitespace errors.
9. `git diff --cached --check`: exit 0. The staged name list contained exactly the five C2 files above.
10. `git commit -m "fix: separate time-based grounded filter from physical loads"`: exit 0, commit `ec648a3`, 5 files changed, 228 insertions, 9 deletions.

The full suite cannot be reported as passing in this headless CoreGraphics environment. The 897 non-render tests passed. The four renderer tests were not skipped or counted as passing.

## Scope and remaining concern

The existing recorder/summary still derive airtime from legacy bridged `front_in_contact`/`rear_in_contact`; C2 did not redirect telemetry, per the task's compatibility ruling. The new controller filter does not affect that metric. No material C2 assertion failure remains in the completed test runs.

The user-owned untracked plans/specs and `.claude/skills/` were left unstaged. The report is under the repository's ignored `.superpowers/` directory and is intentionally outside the C2 commit.
