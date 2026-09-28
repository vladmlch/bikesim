# Task A4 report

## Change

- `SuperDeluxeDamper(legacy_behavior=False)` uses a continuous Firm compression curve with `15000 * 0.03 = 450 N` at the knee and applies HBO after selecting the Open or Firm base force.
- `legacy_behavior=True` calls `_compute_legacy_damping_force`, whose previous calculation body was moved unchanged. Golden force values cover the old Open and Firm behavior, including the Firm jump and its missing HBO.
- Integration wiring is recorded in the review fix below.

## TDD evidence

- Red: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_damper_physics.py -q` → 10 failed, 65 passed. Firm HBO produced the same force at 20 and 65 mm; the Firm knee differed by 30.000166 N across a 2e-8 m/s velocity step; `legacy_behavior` did not yet exist.
- Green: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_damper_physics.py tests/test_ride_controllers.py -q` → 116 passed in 4.01 s. The new cases cover HBO, knee continuity, both velocity signs at eight velocities and four strokes in both modes, and legacy golden forces.
- Full suite: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q` → 697 passed, 4 errors in 213.68 s. All four errors are fixture setup failures in `tests/test_render_comparison.py` from MuJoCo `CGLError: invalid CoreGraphics connection`, matching the known baseline failure class. No other failures appeared.

## Review fix: legacy factory propagation

- `BikeSuspensionSystem` now accepts `legacy_behavior` and forwards it to its shock damper. Its standalone default remains corrected.
- `RideSimulation` selects the shock law from `physics_config.physics_mode`. The legacy playground and CLI `--sag` controller explicitly request the old law.
- Red: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_damper_physics.py::test_firm_knee_is_exactly_continuous_at_450_n tests/test_damper_physics.py::test_hbo_adds_the_same_force_in_open_and_firm tests/test_physics_config.py::test_implicit_legacy_and_physical_have_distinct_revisions tests/test_playground.py::test_telemetry_metrics_and_forces tests/test_ride_cli.py::test_sag_fitting_keeps_legacy_damper_law -q` → 3 failed, 2 passed. All three legacy entry points returned the corrected 450 N at the 0.03 m/s Firm knee, rather than the old 480 N.
- Green: the same command → 5 passed in 1.53 s.
- Focused integration: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_damper_physics.py tests/test_ride_controllers.py tests/test_physics_config.py tests/test_ride_invariants.py tests/test_ride_cli.py tests/test_playground.py -q` → 183 passed in 30.07 s. This includes the new exact 450 N knee, equal Open/Firm HBO increment, and legacy versus physical `RideSimulation` assertions.
- The earlier full suite remains the last full run for this task; its only errors were the four known CoreGraphics setup failures.
