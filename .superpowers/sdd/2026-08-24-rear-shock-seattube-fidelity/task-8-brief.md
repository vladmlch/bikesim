### Task 8: Restore 30 % Rear Sag on the New Leverage Curve

The refit changed the leverage ratio, so the air spring no longer produces the specified sag.

**Files:**
- Modify: `bike_geometry.py:79-84` (rear shock damper settings) or `air_spring.py` pressure default, whichever holds the rear spring rate
- Test: `tests/test_air_spring.py` (extend)

**Interfaces:**
- Consumes: fitted leverage curve from Task 3
- Produces: retuned rear spring pressure constant

- [ ] **Step 1: Write the failing test**

Append to `tests/test_air_spring.py`:

```python
def test_rear_sag_is_30_percent_on_the_fitted_leverage_curve():
    """
    Sag is a design target, not an emergent value. The photo refit moved the
    leverage curve, so the spring rate must be retuned to keep 30 percent.
    """
    from bike_geometry import BikeSpecs
    from linkage_solver import HorstLinkageSolver

    specs = BikeSpecs()
    solver = HorstLinkageSolver(specs)
    target_sag_mm = 0.30 * specs.rear_wheel_travel

    sag_state = solver.solve_state_from_wheel_travel(target_sag_mm)
    assert sag_state["shock_stroke"] > 0.0
    # 30 percent of travel must land near 30 percent of stroke for a curve this
    # progressive; a gross mismatch means the spring will not sit where specified.
    sag_fraction_of_stroke = sag_state["shock_stroke"] / specs.shock_stroke
    assert 0.24 <= sag_fraction_of_stroke <= 0.36, (
        f"30% travel consumes {sag_fraction_of_stroke:.1%} of stroke"
    )
```

- [ ] **Step 2: Run the test**

Run: `uv run pytest tests/test_air_spring.py -v`
Expected: PASS or FAIL depending on the fitted curve. If it PASSES, no retune is required — record that in the commit message and skip to Step 4.

- [ ] **Step 3: Retune if it failed**

Locate the rear spring pressure used by the sag computation and adjust it until the test passes. Change one constant, re-run, repeat. Do not weaken the assertion window.

- [ ] **Step 4: Run the full air-spring and mass suites**

Run: `uv run pytest tests/test_air_spring.py tests/test_mass_distribution.py -v`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add bike_geometry.py air_spring.py tests/test_air_spring.py
git commit -m "fix(suspension): restore 30% rear sag on the refitted leverage curve"
```

---

