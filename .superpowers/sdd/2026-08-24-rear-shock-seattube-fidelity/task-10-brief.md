### Task 10: Full Suite and Documentation

**Files:**
- Modify: `README.md` (geometry and kinematics tables)

**Interfaces:**
- Consumes: everything above
- Produces: documentation consistent with the shipped numbers

- [ ] **Step 1: Run the entire test suite**

Run: `uv run --with pillow pytest tests/ -v`
Expected: all pass. Fix any failure before continuing — do not proceed on a red suite.

- [ ] **Step 2: Refresh the README numbers**

Update every geometry and kinematics table in `README.md` to the values now in `coordinates.json`: hardpoint coordinates, initial/final leverage ratio, progressivity, transmission angles, link lengths. Add a short subsection recording that hardpoints are photo-derived, naming `tools/photo_reference.py` and `docs/reference/bulls_reference_points.json`, and stating plainly that front-triangle tube shapes are authored styling rather than measurements.

- [ ] **Step 3: Verify the README claims against the artifacts**

Run:
```bash
uv run python -c "
import json
d = json.load(open('coordinates.json'))
print(json.dumps(d['geometry_summary'], indent=2))
"
```
Cross-check each printed figure against what you wrote.

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "docs: refresh geometry and kinematics tables for photo-derived frame"
```

---

## Self-Review

**Spec coverage:**

| Spec section | Task |
|---|---|
| §2 invariants | Task 3 (`TestPublishedGeometryInvariants`) |
| §3 measurement method | Task 1 |
| §4 reference extraction + ambiguity | Tasks 1, 2 |
| §5 constrained refit + chainstay shift | Tasks 2, 3 |
| §6 seat tube / casting | Task 4 |
| §6 trunnion, damper, piggyback | Task 5 |
| §6 down tube | Task 6 |
| §6 debug livery | Task 7 |
| §7 verification, sag, render, README | Tasks 3, 5, 8, 9, 10 |
| §8.1 styling labelled in code | Tasks 4, 6, 10 |

No spec requirement is unassigned.

**Known execution risks:**

1. Task 2 Step 4 may need penalty reweighting; the exact adjustment and its direction are given inline.
2. Task 8 may be a no-op if sag already lands in the window — Step 2 says so explicitly rather than forcing a change.
3. Task 3 Steps 3 and 6 deliberately carry values that do not exist until Task 2's fitter has run. Both steps name the exact file and command to read them from; they are data dependencies, not unfilled blanks.

**Verified against the codebase while writing this plan:** the MJCF builder is `generate_mujoco_xml` at `export_mujoco.py:65` (not a `build_*` name); `run_playground.py` calls it from `load_model` at line 300 (not from the viewer setup at 1296); the JSON regeneration flag is `main.py --export-json`. Task 7's marker guard must skip `site_P3_ss`, `site_P3_rocker`, `site_P6_yoke` and `site_P7_shock`, which `<equality><connect>` depends on.
