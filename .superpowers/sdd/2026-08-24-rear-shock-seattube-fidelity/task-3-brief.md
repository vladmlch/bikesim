### Task 3: Adopt the Fitted Hardpoints

Moves the fitted constants into the two places the whole codebase reads geometry from, regenerates `coordinates.json`, and hardens the kinematics tests with explicit invariant assertions.

**Files:**
- Modify: `bike_geometry.py:269-295` (`get_fixed_frame_points`)
- Modify: `linkage_solver.py:227-248` (solver seed defaults)
- Modify: `tests/test_kinematics.py:138-197` (`test_fixed_frame_points`), `:419-458` (`test_leverage_ratio_progressive`)
- Modify: `coordinates.json` (regenerated)
- Test: `tests/test_kinematics.py` (new `TestPublishedGeometryInvariants`)

**Interfaces:**
- Consumes: `docs/reference/fitted_hardpoints.json` from Task 2
- Produces: updated `get_fixed_frame_points` returning fitted `P0`, `P5`, `P7`; updated `HorstLinkageSolver` defaults for `p2_0`, `p3_0`, `p4_0`, `p6_0`, `p12_0`

- [ ] **Step 1: Write the failing invariant test**

Append to `tests/test_kinematics.py`:

```python
class TestPublishedGeometryInvariants(unittest.TestCase):
    """
    Locks the published geometry table. These values are user-fixed: a future
    hardpoint refit must never move them silently.
    """

    def setUp(self) -> None:
        self.specs = BikeSpecs()
        self.solver = HorstLinkageSolver(self.specs)

    def test_frame_table_is_unchanged(self) -> None:
        self.assertAlmostEqual(self.specs.reach, 480.0, places=6)
        self.assertAlmostEqual(self.specs.stack, 646.0, places=6)
        self.assertAlmostEqual(self.specs.head_angle_deg, 64.0, places=6)
        self.assertAlmostEqual(self.specs.effective_seat_angle_deg, 77.0, places=6)
        self.assertAlmostEqual(self.specs.bb_drop, 22.5, places=6)
        self.assertAlmostEqual(self.specs.wheelbase, 1280.55, places=6)

    def test_wheel_and_shock_hardware_is_unchanged(self) -> None:
        self.assertAlmostEqual(self.specs.front_wheel_radius, 372.0, places=6)
        self.assertAlmostEqual(self.specs.rear_wheel_radius, 352.0, places=6)
        self.assertAlmostEqual(self.specs.shock_eye_to_eye, 205.0, places=6)
        self.assertAlmostEqual(self.specs.shock_stroke, 65.0, places=6)
        self.assertAlmostEqual(self.specs.fork_travel, 180.0, places=6)
        self.assertAlmostEqual(self.specs.rear_wheel_travel, 180.0, places=6)

    def test_chainstay_length_is_447_5(self) -> None:
        st0 = self.solver.solve_state_from_wheel_travel(0.0)
        chainstay = float(np.linalg.norm(st0["P1"] - np.zeros(3)))
        self.assertAlmostEqual(chainstay, 447.5, delta=0.1)

    def test_full_travel_consumes_exactly_the_65mm_stroke(self) -> None:
        traj = self.solver.solve_trajectory(n_points=101, max_travel=180.0)
        self.assertAlmostEqual(traj["max_stroke"], 65.0, delta=0.01)

    def test_shock_eye_to_eye_is_205_at_full_extension(self) -> None:
        st0 = self.solver.solve_state_from_wheel_travel(0.0)
        e2e = float(np.linalg.norm(st0["P7"] - st0["P6"]))
        self.assertAlmostEqual(e2e, 205.0, delta=0.5)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_kinematics.py::TestPublishedGeometryInvariants -v`
Expected: `test_chainstay_length_is_447_5` FAILS (current 447.50 passes) and `test_full_travel_consumes_exactly_the_65mm_stroke` FAILS — current `max_stroke` is 65.0464, outside the 0.01 tolerance.

- [ ] **Step 3: Transfer the fitted constants**

Read `docs/reference/fitted_hardpoints.json`, then edit `bike_geometry.py` `get_fixed_frame_points` — replace the `P0`, `P5`, `P7` literals (and their descriptive aliases `main_pivot`, `rocker_frame_pivot`, `shock_upper_mount`) with the fitted values, keeping full precision:

```python
    points = {
        "P0": np.array([<fitted P0 X>, 0.0, <fitted P0 Z>], dtype=float),
        "P5": np.array([<fitted P5 X>, 0.0, <fitted P5 Z>], dtype=float),
        "P7": np.array([<fitted P7 X>, 0.0, <fitted P7 Z>], dtype=float),
        ...
        "main_pivot": np.array([<fitted P0 X>, 0.0, <fitted P0 Z>], dtype=float),
        "rocker_frame_pivot": np.array([<fitted P5 X>, 0.0, <fitted P5 Z>], dtype=float),
        "shock_upper_mount": np.array([<fitted P7 X>, 0.0, <fitted P7 Z>], dtype=float),
```

Then edit `linkage_solver.py:230-248` — replace the `p2_0`, `p3_0`, `p4_0`, `p6_0`, `p12_0` fallback literals with the fitted values. Keep the existing comment above `p6_0` warning that full precision is required for the 1e-6 m MJCF rounding of the `site_P7` loop closure.

- [ ] **Step 4: Run the invariant tests to verify they pass**

Run: `uv run pytest tests/test_kinematics.py::TestPublishedGeometryInvariants -v`
Expected: 5 passed

- [ ] **Step 5: Regenerate coordinates.json and read the new curve**

Run:
```bash
uv run python main.py --export-json
uv run python -c "
from bike_geometry import BikeSpecs
from linkage_solver import HorstLinkageSolver
t = HorstLinkageSolver(BikeSpecs()).solve_trajectory(n_points=101, max_travel=180.0)
print('LR', round(t['initial_leverage_ratio'],4), '->', round(t['final_leverage_ratio'],4))
print('progressivity', round(t['progressivity_pct'],3))
print('max_stroke', round(t['max_stroke'],4))
"
```
Note the three printed numbers — the next step pins them.

- [ ] **Step 6: Update the existing kinematics expectations**

In `tests/test_kinematics.py`, update `test_leverage_ratio_progressive` (line 419) and `test_fixed_frame_points` (line 138) so their hardcoded expectations match the values printed in Step 5. Pin the leverage figures as a characterization assertion with a ±0.02 window, e.g.:

```python
        self.assertAlmostEqual(traj["initial_leverage_ratio"], <printed LR init>, delta=0.02)
        self.assertAlmostEqual(traj["final_leverage_ratio"], <printed LR final>, delta=0.02)
        self.assertAlmostEqual(traj["progressivity_pct"], <printed progressivity>, delta=0.10)
```

- [ ] **Step 7: Run the whole kinematics suite**

Run: `uv run pytest tests/test_kinematics.py -v`
Expected: all pass. If `test_json_export_structure` fails on a stale value, regenerate with Step 5 first.

- [ ] **Step 8: Commit**

```bash
git add bike_geometry.py linkage_solver.py coordinates.json tests/test_kinematics.py
git commit -m "feat(geometry): adopt photo-fitted hardpoints, lock published geometry table"
```

---

