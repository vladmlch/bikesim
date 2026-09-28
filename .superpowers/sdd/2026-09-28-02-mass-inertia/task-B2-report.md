# Task B2 report — explicit physical wheel tensors

Status: implemented on `pedals` from `9aa8cb3` in commit `af53548`. Only B2 source and tests were committed. The profile remains **synthetic**, without measured wheel mass distributions or experimental validation.

## Design and changed files

- `src/bike_sim/physics/inertia.py`: validates annular-cylinder and parallel-axis formulas, composes wheel components about a common CoM, and writes one full body-frame `<inertial>`. The immutable front/rear component tuples are the shared physical profile.
- `src/bike_sim/mujoco/steering_fork.py`, `src/bike_sim/mujoco/drivetrain.py`: physical wheel visual/contact geoms have zero mass; each wheel body owns one inertial. Rear cassette and core stay on `rear_wheel` until D1. Legacy geom masses remain in the legacy branch.
- `src/bike_sim/mujoco/rear_linkage.py`, `src/bike_sim/mujoco/builder.py`, `src/bike_sim/physics/component_masses.py`: pass the same `BikeMassSpecs` budget through the rear builder and retain all 15 registry IDs while registering wheel inertials as the sole mass owners. Other components still use their B1 geom allocations.
- `src/bike_sim/physics/mass.py`: corrected the stale default compiled-total docstring from 24.35 kg to the physical 24.40 kg budget, and identified `compute_wheel_rotational_inertia` and its fractions as legacy analytic approximations.
- `tests/test_wheel_inertia_contract.py`, `tests/test_compiled_mass_contract.py`: check formula validation, body-frame compiled tensors (recovering axes from `body_iquat`), mass ownership, stand acceleration and energy, geometry independence, legacy behavior, and B1 overrides.

No hardpoint, axle position, wheel visual dimensions, tyre contact rule, or non-wheel mass allocation was changed.

## Synthetic profile and numerical result

All parameters are SI units. Each tuple has `(mass_fraction, inner_radius_m, outer_radius_m, width_m, offset_m)`; all offsets are `(0, 0, 0)` in wheel-body coordinates. The wheel axis is Y.

| Wheel | Ring component | Core component | Compiled body mass | Body-frame tensor diagonal (kg m²) |
| --- | --- | --- | ---: | --- |
| Front | `(0.75, 0.320, 0.372, 0.060, (0,0,0))` | `(0.25, 0, 0.045, 0.110, (0,0,0))` | 2.400000000 kg | `(0.10980155, 0.21731310, 0.10980155)` |
| Rear | `(0.75, 0.300, 0.352, 0.064, (0,0,0))` | `(0.25, 0, 0.045, 0.148, (0,0,0))` | 2.800000000 kg | `(0.114648508333, 0.22530795, 0.114648508333)` |

The fixed-axis front-wheel stand has kinetic energy `1.7385048 J` at `4 rad/s`; applying `2.173131 N m` produces `10 rad/s²`. These are model checks, not experimental validation. The current compiled CoM is a state-dependent MuJoCo quantity; this task does not equate it with the unloaded analytic CoM.

All 15 `BikeMassSpecs.component_provenance` entries remain `mass=synthetic, distribution=synthetic`. The compiled physical total is `24.400000000 kg`. The per-field compiled contributions (found by reducing one budget to `0.0001 kg` and taking the compiled-total difference) are:

| Component | Declared budget (kg) | Compiled contribution (kg) |
| --- | ---: | ---: |
| motor | 2.900000 | 2.900000 |
| battery | 4.300000 | 4.300000 |
| frame_structure | 3.200000 | 3.200000 |
| saddle_post | 0.900000 | 0.900000 |
| crank_pedals | 0.850000 | 0.850000 |
| steer_assembly | 1.100000 | 1.100000 |
| stanchions | 0.850000 | 0.850000 |
| fork_lowers | 1.500000 | 1.500000 |
| chainstay | 1.250000 | 1.250000 |
| seatstay | 1.050000 | 1.050000 |
| rocker | 0.400000 | 0.400000 |
| shock_yoke | 0.300000 | 0.300000 |
| shock_damper | 0.600000 | 0.600000 |
| front_wheel | 2.400000 | 2.400000 |
| rear_wheel | 2.800000 | 2.800000 |

## TDD and verification evidence

1. Added `tests/test_wheel_inertia_contract.py` first. `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_wheel_inertia_contract.py -q` exited 2 at collection with `ModuleNotFoundError: No module named 'bike_sim.physics.inertia'` (the required absent-API red).
2. Added only the formula/inertial module, then reran the same command before changing generated wheels. It exited 1 with `5 failed, 15 passed in 0.48s`: both generated wheels lacked `<inertial>`, both decorative-rim changes altered compiled inertia, and the selected-wheel energy test lacked the inertial. This confirmed the generated-wheel red.
3. Connected the generated wheels and B1 registry. The new contract suite reported `20 passed in 0.59s`.
4. The first combined focused run reported `4 failed, 85 passed in 2.36s`, all from B1's geom-only assertions for front/rear wheel overrides. Updated those assertions to check the wheel inertial mass owners. The final required focused command, `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_wheel_inertia_contract.py tests/test_compiled_mass_contract.py tests/test_mujoco_export.py -q`, exited 0: `89 passed in 2.27s`.
5. Ran one full suite with `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q`: `866 passed, 4 errors in 193.27s`. The four errors were setup errors in `tests/test_render_comparison.py`, each caused by `mujoco.cgl.cgl.CGLError: invalid CoreGraphics connection` while creating `mujoco.Renderer`. There were zero assertion failures. No rendered-image behavior was validated in this sandbox.
6. `git diff --check` exited 0 before commit. The existing `uv.lock` and dependencies were not changed.

## Concerns and follow-up boundary

- The 75/25 ring/core split, dimensions, zero offsets, and both wheel mass budgets are synthetic. A measured profile should replace these tuples as a unit.
- D1 should separate rear cassette mass/inertia from the rear core without changing the 2.80 kg total or silently reusing these synthetic core values as measurements.
- The four CoreGraphics setup errors prevent validation of the image comparison tests in this environment. They do not indicate a B2 assertion failure.

Commit: `af53548` (`fix: assign physical wheel inertia independent of visual cylinders`).

## Review fix round 1

The compiled-wheel test now derives component masses, the common CoM, and the full expected body-frame tensor from the same `FRONT_WHEEL_PROFILE` / `REAR_WHEEL_PROFILE` tuples and the corresponding `BikeMassSpecs` budget. It checks MuJoCo's compiled `body_mass` and `body_ipos` per wheel, and keeps the `body_iquat` rotation when comparing the compiled tensor. The expectation evaluates the specified formulas in the test rather than calling the production wheel composition helper. This catches a wheel budget or CoM being lost when MJCF is compiled.

Focused acceptance command (exact):

```text
UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_wheel_inertia_contract.py tests/test_compiled_mass_contract.py tests/test_mujoco_export.py -q
........................................................................ [ 80%]
.................                                                        [100%]
89 passed in 3.57s
```

Exit code: 0. `git diff --check` also exited 0. The full suite was not rerun in this review fix round, per task instruction; its previous 866-pass/four-CoreGraphics-error result is recorded above. The separately reviewed `physics_revision` distinction is already covered by A1 and required no B2 change.

Review fix commit: `1fcd925` (`test: check compiled wheel mass and center of mass`).
