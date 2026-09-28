# Task A2 report

Status: implemented, awaiting task review.

## Changes

- Added `ForceAccumulator` with named, copied force vectors; shape/finite and duplicate-name validation; detached totals and read-only component views.
- Added `SuspensionForceApplier.compute_qfrc(model, data)` to calculate the fork/shock generalized-force vector without mutating MuJoCo input. The legacy `apply` path retains its assigning behavior through this calculation.
- The physical step copies, validates, and freezes explicit `external_qfrc` before clearing MuJoCo input, including when the caller passes `data.qfrc_applied` itself. It clears retained `qfrc_applied` and `xfrc_applied` before `mj_forward`, then clears the per-step accumulator, collects suspension/rider/rolling/leg contributions by name, installs the sum, copies `(time, qpos, qvel, components)` into `last_force_snapshot`, and calls `mj_step` once. Pitch-assist disabling occurs before collection. Pneumatic tyre forces are body wrenches in `xfrc_applied`, so they remain on that separate MuJoCo channel.
- The legacy step is unchanged for callers that omit `external_qfrc`; explicit external generalized force is rejected in legacy mode rather than silently discarded.

## TDD evidence

1. Added the exact two-writer accumulator example. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_force_accumulator.py -q` returned exit 2 with `ModuleNotFoundError: No module named 'bike_sim.sim.ride.force_accumulator'`.
2. Implemented the accumulator; the same command returned `1 passed in 0.23s`.
3. Added copy/validation and physical integration cases before changing the ride path. The same command returned `4 passed, 3 failed`; all three failures were the missing `external_qfrc` keyword on `RideSimulation.step`.
4. Implemented the physical step and legacy adapter. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_force_accumulator.py tests/test_ride_controllers.py -q` returned `48 passed in 4.58s`.
5. Self-review found that an explicit external argument could alias MuJoCo's retained array. Added a failing case: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_force_accumulator.py::test_explicit_external_input_can_alias_mujoco_applied_force_array -q` returned `1 failed` with actual external component `0.0` versus expected `11.0`. Copied/validated the input before clearing and made the copy read-only for the step. The focused command from step 4 then returned `49 passed in 5.00s` after the final change.

## Full suite

Before the final aliasing correction, `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q` returned `614 passed, 4 errors in 210.28s`. All four errors are setup failures in `tests/test_render_comparison.py`, each due to MuJoCo `CGLError: invalid CoreGraphics connection`; these match the baseline environment failure. The aliasing correction was verified by the focused `49 passed` run; a second full run was not performed.

`git diff --check` passed before commit.
