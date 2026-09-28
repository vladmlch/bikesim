# Task B1 report — unified component mass budget and compiled CoM

## Result

Implemented physical-mode mass normalization for all 15 `BikeMassSpecs` component fields. Each MJCF builder registers its own mass-bearing geoms in a Python registry. The assembly checks that the registry has every expected component and covers each bike geom exactly once; rider and zero-mass debug/contact geoms are excluded. `assign_component_mass` validates finite positive budgets and nonnegative finite relative weights, then writes 17-significant-digit masses. The legacy MJCF path retains its authored absolute geom masses and pinned shock inertial.

`RideSimulation` now accepts keyword-only `mass_specs` and passes it alongside `physics_config` to its single model build. Physical-mode ride telemetry records compiled total mass and world CoM from `model.body_mass` and current `data.xipos`, with kinematics refreshed before reading. The `site_CG` marker follows that compiled CoM after equilibrium and each physical step. The unloaded analytic CoM remains available as `compute_unloaded_analytic_system_cg` (and its old `compute_static_system_cg` name). Static load in the physical sphere ride is calculated from the actual front and rear MuJoCo contact positions after sag and stored in equilibrium and telemetry; pneumatic rides report NaN for this contact-based diagnostic because they have no native wheel contacts.

B2's wheel inertial geometry and visual-only wheel geoms were not changed. The physical shock body omits its old fixed inertial so MuJoCo derives mass and inertia from its normalized geoms; the legacy fixed inertial remains.

## Verification

- Before changes: `uv run --locked pytest tests/test_geometry.py tests/test_linkage_kinematics.py -q` → 13 passed. Existing axle/hardpoint/linkage geometry tests were retained.
- Red: `uv run --locked pytest tests/test_compiled_mass_contract.py -q` → missing component-mass module at collection. A later rider-specific regression was observed failing: physical lumped rider compiled as 24.4 kg instead of 104.4 kg before exclusion from the frame group.
- Final focused: `uv run --locked pytest tests/test_compiled_mass_contract.py tests/test_geometry.py tests/test_linkage_kinematics.py -q` → 61 passed.
- Final full suite: `uv run --locked pytest -q` → 829 passed, 4 setup errors, 0 assertion failures. The four errors are all `tests/test_render_comparison.py` failing at `mujoco.Renderer` setup with `CGLError: invalid CoreGraphics connection`, matching the known sandbox renderer baseline.
- `git diff --check` → clean before commit.

The new tests exercise every actual mass field, group isolation, rider exclusion, debug geoms, invalid weights, exact total mass, 1 m root translation of compiled CoM with unchanged compiled mass/inertia, live ride telemetry, and contact-position-dependent static load. The physical mass distribution remains synthetic, pending component measurements. GUI rendering was not validated in this environment.

## Review fix round 1

The review found that a physical motor budget of `0.001` kg was formatted as `0.00` before component registration. The motor geom now retains a fixed positive authored relative weight in physical mode; the registry applies the requested budget with 17 significant digits. The audit also found the same budget-dependent weighting risk for the battery, saddle/post split, and stanchions. Their physical relative weights now stay fixed as their component budgets change. Legacy formatting and absolute masses are unchanged.

`BikeMassSpecs.component_provenance` is the machine-readable source map, keyed by the same 15 stable IDs as `component_masses`. Each entry labels both `mass` and `distribution` as `synthetic`; no measured component-mass dataset was supplied. F4 can carry this map into resolved release metadata.

New regressions cover a `0.001` kg motor budget, all 15 components at `0.0001` kg with unchanged relative geom proportions and requested compiled total mass, and the complete synthetic provenance map. Before the fix, the targeted red command reported `2 failed in 1.51s`: the motor registration raised `ValueError: component 'motor' has no mass-bearing geoms`, and `BikeMassSpecs` lacked `component_provenance`. The broader tiny-budget red command reported `2 failed, 13 passed in 1.26s` for battery and saddle/post weighting.

Focused verification only, as requested:

```text
$ UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_compiled_mass_contract.py tests/test_geometry.py tests/test_linkage_kinematics.py -q
........................................................................ [ 92%]
......                                                                   [100%]
78 passed in 2.02s
```

`git diff --check` produced no output. The full suite was not rerun for this amendment.
