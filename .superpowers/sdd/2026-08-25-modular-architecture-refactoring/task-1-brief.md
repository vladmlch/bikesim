### Task 1: Phase 0 — Output Directory Isolation & Golden Baseline Snapshots

**Files:**
- Create: `output/.gitignore`
- Create: `output/plots/.gitkeep`
- Create: `output/models/.gitkeep`
- Create: `tests/golden/.gitkeep` (temporary directory for baseline snapshot)
- Modify: `.gitignore`

**Interfaces:**
- Consumes: Existing root `bike_model.xml`, `bike_playground.xml`, `coordinates.json`, root `.png` plots
- Produces: `output/` clean hierarchy, `tests/golden/` snapshot XMLs and JSONs for equivalence testing

- [ ] **Step 1: Capture golden reference XML and JSON files**

Run a capture script to save current `bike_model.xml`, `bike_playground.xml`, `bike_playground_dynamic.xml`, `bike_playground_stand.xml`, and `coordinates.json` into `tests/golden/`.

```bash
mkdir -p tests/golden output/plots output/models
cp bike_model.xml tests/golden/baseline_bike_model.xml
cp bike_playground.xml tests/golden/baseline_bike_playground.xml
cp bike_playground_dynamic.xml tests/golden/baseline_bike_playground_dynamic.xml
cp bike_playground_stand.xml tests/golden/baseline_bike_playground_stand.xml
cp coordinates.json tests/golden/baseline_coordinates.json
```

- [ ] **Step 2: Move existing loose PNG files and XMLs to `output/`**

Move generated plots (`leverage_ratio.png`, `axle_path.png`, `linkage_geometry.png`, `damper_dyno_curves.png`, `suspension_compressed_comparison.png`, `shock_stroke.png`, `test_plot.png`, `crop_*.png`, `bb_*.png`, `headtube_*.png`, `rocker_*.png`, `seat_*.png`, `shock_*.png`, `fork_dropout.png`, `rear_axle_area.png`) to `output/plots/`.
Move root XMLs and log files (`MUJOCO_LOG.TXT`) to `output/models/` or remove generated temporary logs.

- [ ] **Step 3: Configure `output/.gitignore` and root `.gitignore`**

Create `output/.gitignore`:
```gitignore
*
!.gitignore
!plots/
!models/
!plots/.gitkeep
!models/.gitkeep
```

Update root `.gitignore` to ignore `.work/`, `output/plots/*.png`, `output/models/*.xml`.

- [ ] **Step 4: Verify test suite runs cleanly on baseline**

Run: `uv run pytest`
Expected: 95 passed in ~12s.

- [ ] **Step 5: Commit Phase 0**

```bash
git add output/ tests/golden/ .gitignore
git commit -m "refactor(phase0): isolate outputs and capture golden baseline snapshots"
```

---

