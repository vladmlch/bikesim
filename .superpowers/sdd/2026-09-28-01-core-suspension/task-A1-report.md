# Task A1 report

Status: implemented and committed on the `pedals` checkout, based on `70623815b98788018bcdbd8eef347d778f9bb3f3`.
Commit: `0da36d0` (`feat: separate legacy and physical simulation modes`).

## Changes

- Added frozen `SimulationPhysicsConfig` with validated mode, drive source, timestep and pitch assistance. The implicit config remains legacy with ideal speed control.
- Added keyword-only `physics_config` to `RideSimulation` and `generate_mujoco_xml`. The configured timestep reaches ride-mode MJCF; omitted config preserves the existing ride timestep and legacy behaviour.
- Split orchestration into `_step_legacy` and `_step_physical`. Physical coast clears rear and crank drive controls each step and does not call `PitchStabilizer.apply`. The stabilizer's new `disable` method clears the retained root-pitch generalized force.
- Added a revision label to the simulation (`legacy-v1` or `physical-v1`). Until drivetrain tasks land, `crank_effort` and `articulated_effort` fail at construction with a named `NotImplementedError`.
- Physical mode refreshes MuJoCo kinematics before contact and pneumatic-tyre force reads. Its `contacts` field describes the state used by the most recent step's force writers and remains a pre-step snapshot until the next step. A2 owns the synchronized force/state sample.
- Explicit legacy `drive_mode="coast"` skips cruise torque; explicit `pitch_assist=False` clears the pitch moment. Existing legacy `motor`/`pedal`/`pedelec` calls remain on the historical path.

## TDD and verification

- Red: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_physics_config.py -q` failed during collection with missing `bike_sim.physics.model_config`.
- Red: explicit legacy coast test failed because `rear_drive` was 150 N.m instead of zero.
- Red: physical pneumatic geometry test failed because a wheel raised 1 m above the road still carried 112.6 N from stale MuJoCo kinematics.
- Green: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_physics_config.py tests/test_ride_model.py tests/test_ride_pneumatic.py -q` — 46 passed.
- Full suite before the final kinematics change: 604 passed, 4 setup errors, all in `tests/test_render_comparison.py` with `mujoco.cgl.cgl.CGLError: invalid CoreGraphics connection`. This matches the supplied baseline of 595 passed and the same 4 setup errors; the 9 extra passes were new A1 tests at that point. The final kinematics change was verified by the focused 46-test run above.
- `git diff --check` passed.

## Follow-up / limits

- The existing sphere and pneumatic tyre models, suspension and rider mass paths are still the legacy physical components; A2 and later tasks replace their composition and bookkeeping. Passing synthetic tests is not experimental validation.
- No GUI render validation was possible in the sandbox because of the CoreGraphics setup failure.

## Review fix 1: detailed pneumatic timestep

Commit: `b69c674` (`fix: reject conflicting detailed tyre timestep in physical mode`).

The detailed pneumatic tyre constructor previously overwrote `model.opt.timestep` with
`0.00025` after `RideSimulation` had compiled an explicit physical timestep. The physical
constructor now rejects a conflicting detailed-tier timestep before model compilation,
with the required `0.00025` value in the error. A matching explicit value is accepted.
The existing legacy tier override is unchanged.

Red command:

```text
UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_physics_config.py::test_physical_detailed_tyre_rejects_incompatible_explicit_timestep -q
F                                                                        [100%]
E       Failed: DID NOT RAISE ValueError
1 failed in 1.79s
```

Green command after the fix:

```text
UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_physics_config.py tests/test_ride_model.py tests/test_ride_pneumatic.py -q
..............................................
..                         [100%]
48 passed in 39.86s
```

`git diff --check` passed. The original full-suite result and CoreGraphics baseline
remain recorded above; no full-suite rerun was requested for this small fix.
