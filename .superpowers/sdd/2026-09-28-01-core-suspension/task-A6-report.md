# A6 implementation report

Commit: `d896d57` (`fix: reflect shock mass correctly and solve requested sag`).

## Scope

- Corrected wheel-to-shock reflected mass using kinetic energy, with finite and positive input validation.
- Added a bounded, positive log-space two-parameter sag fitter and a physical equilibrium evaluator. The evaluator fixes base bike specifications, track, rider, field, tyre, start position and physics config, and rebuilds the physical suspension for each pressure/rate candidate. `RideSimulation` supplies travel measured by `solve_static_equilibrium` during construction.
- Replaced the old analytical rear damping assertion with an energy-based expectation, retaining the 3% tolerance.
- Kept legacy CLI sag dispatch unchanged; F4 owns the physical CLI route.

## Verification

- Red: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_sag_physics.py -q` failed at collection because `reflected_shock_mass` did not exist.
- Green: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_sag_physics.py tests/test_mass_distribution.py tests/test_ride_equilibrium.py -q` passed: 57 tests.
- Full suite: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q` exited 1 with 781 passed and four known setup errors in `tests/test_render_comparison.py`. Each renderer error is `mujoco.cgl.cgl.CGLError: invalid CoreGraphics connection`, matching the pre-A6 baseline. No test assertion failed.
- `git diff --check` passed.

## Concern

- Physical CLI hookup and its final achieved-sag check remain F4's task; A6 exposes `build_equilibrium_evaluator` and `fit_sag` for that route.
