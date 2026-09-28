### Task 6: Battery-Box Down Tube

Replaces the thin down-tube capsule with the measured battery-box envelope.

**Files:**
- Modify: `export_mujoco.py:706-717` (`geom_downtube`)
- Test: `tests/test_frame_visuals.py` (extend)

**Interfaces:**
- Consumes: `BB`, `P_HT_bot` from the existing hardpoint block
- Produces: `geom_downtube` changes `type` from `capsule` to `box`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_frame_visuals.py`:

```python
def test_down_tube_is_a_battery_box_not_a_thin_tube(root):
    downtube = next(g for g in root.iter("geom") if g.get("name") == "geom_downtube")
    assert downtube.get("type") == "box"
    half_extents = [float(v) for v in downtube.get("size").split()]
    # Measured envelope is ~116 mm across at Z = 255 mm, so the largest cross
    # section half-extent must be well above the old 28 mm capsule radius.
    assert max(half_extents) >= 0.055
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py::test_down_tube_is_a_battery_box_not_a_thin_tube -v`
Expected: FAIL — `assert 'capsule' == 'box'`

- [ ] **Step 3: Replace the down tube geom**

Replace the `geom_downtube` block at `export_mujoco.py:706-717`:

```python
    # Down tube is a battery box on this e-bike, not a tube. Envelope measured
    # from the reference silhouette: X 150..266 mm at Z = 255, X 208..305 at Z = 323.
    # AUTHORED STYLING - the exact casting shape is not recoverable from the photo.
    downtube_mid = (BB + P_HT_bot) / 2.0
    downtube_vec = P_HT_bot - BB
    downtube_angle = float(np.arctan2(downtube_vec[2], downtube_vec[0]))
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_downtube",
            "type": "box",
            "pos": _format_vec(downtube_mid),
            "size": f"{float(np.linalg.norm(downtube_vec)) / 2.0:.6f} 0.038 0.058",
            "euler": f"0 {-downtube_angle:.6f} 0",
            "mass": "0.60",
            "material": "mat_frame",
        },
    )
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: 7 passed

- [ ] **Step 5: Commit**

```bash
git add export_mujoco.py tests/test_frame_visuals.py
git commit -m "feat(visual): battery-box down tube from measured envelope"
```

---

