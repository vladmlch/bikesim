### Task 5: Trunnion Mount and Damper Proportions

Removes the floating shock bracket, derives the damper proportions from spec instead of literals, and pulls the piggyback back into the frame plane.

**Files:**
- Modify: `export_mujoco.py:848-860` (`geom_shock_tab`)
- Modify: `export_mujoco.py:1765-1776` (damper length literals)
- Modify: `export_mujoco.py:1891-1919` (piggyback offsets)
- Test: `tests/test_frame_visuals.py` (extend)

**Interfaces:**
- Consumes: `specs.shock_eye_to_eye`, `specs.shock_stroke`
- Produces: geom `geom_frame_junction`; removes `geom_shock_tab`; `body_can_len` and `trunnion_overhang` become derived values

- [ ] **Step 1: Write the failing test**

Append to `tests/test_frame_visuals.py`:

```python
BOTTOM_OUT_CLEARANCE_MM = 2.0


def test_floating_shock_bracket_is_replaced_by_frame_material(root):
    names = _geom_names(root)
    assert "geom_shock_tab" not in names, "the thin floating bracket must be gone"
    assert "geom_frame_junction" in names


def test_air_can_clears_the_lower_eyelet_at_bottom_out():
    """
    At full bottom-out the eye-to-eye collapses to e2e - stroke. The can is
    measured back from P7, so it must stay shorter than that by the clearance.
    """
    from bike_geometry import BikeSpecs

    specs = BikeSpecs()
    collapsed = specs.shock_eye_to_eye - specs.shock_stroke
    root_el = ET.fromstring(generate_mujoco_xml(mode="stand"))
    can = next(g for g in root_el.iter("geom") if g.get("name") == "geom_shock_body")
    coords = [float(v) for v in can.get("fromto").split()]
    can_len_mm = 1000.0 * (
        (coords[3] - coords[0]) ** 2 + (coords[4] - coords[1]) ** 2 + (coords[5] - coords[2]) ** 2
    ) ** 0.5
    assert can_len_mm <= collapsed - BOTTOM_OUT_CLEARANCE_MM + 1e-6
    assert can_len_mm == pytest.approx(138.0, abs=0.5)


def test_piggyback_sits_in_the_frame_plane(root):
    piggy = next(g for g in root.iter("geom") if g.get("name") == "geom_shock_piggyback")
    coords = [float(v) for v in piggy.get("fromto").split()]
    lateral = max(abs(coords[1]), abs(coords[4]))
    assert lateral <= 0.012, f"piggyback is {lateral*1000:.0f} mm off-plane; expected <= 12 mm"
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: FAIL — `geom_shock_tab` still present, can length is 133 mm, piggyback lateral offset is 0.038 m.

- [ ] **Step 3: Replace the shock tab with frame material**

In `export_mujoco.py`, delete the `geom_shock_tab` block (lines 848-860) and substitute:

```python
    # Frame casting at the top-tube / seat-tube / down-tube junction. The trunnion
    # at P7 bolts into this material - the real bike has no standoff bracket.
    p_junction_top = P10 + (P8 - P10) * 0.52
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_frame_junction",
            "type": "capsule",
            "fromto": _format_fromto(p_junction_top, P7),
            "size": "0.042",
            "mass": "0.30",
            "material": "mat_frame",
        },
    )
```

- [ ] **Step 4: Derive the damper proportions from spec**

Replace the literals at `export_mujoco.py:1768-1776`:

```python
    shaft_len = 0.145  # m (145 mm shaft; tip hides inside the trunnion overhang at bottom-out)
    shaft_tip_rel_p6 = u_shock * shaft_len
    # Air can length is DERIVED, not chosen: at full bottom-out the eye-to-eye
    # collapses to (e2e - stroke), so a can measured back from P7 must stay
    # shorter than that or it drives through the lower eyelet. 2 mm clearance.
    bottom_out_clearance_m = 0.002
    body_can_len = (specs.shock_eye_to_eye - specs.shock_stroke) / 1000.0 - bottom_out_clearance_m
    body_can_end_rel_p7 = shock_slide_axis * body_can_len
    # Trunnion overhang measured from the reference photo: ~12 mm of damper body
    # continues past the mounting bolt axis, carrying the reservoir cap.
    trunnion_overhang = 0.014
    trunnion_cap_rel_p7 = -shock_slide_axis * trunnion_overhang
    trunnion_half_width = 0.027  # m (54 mm across the two trunnion bosses)
```

- [ ] **Step 5: Move the piggyback into the frame plane**

Replace `export_mujoco.py:1892-1893` and the bridge offsets at `:1906-1907`:

```python
    # Reservoir rides on the damper body in the frame plane. The perpendicular
    # direction is the shock axis rotated 90 degrees in XZ, pointing up-rearward.
    perp = np.array([-shock_slide_axis[2], 0.0, shock_slide_axis[0]])
    piggy_offset = perp * 0.034 + np.array([0.0, 0.008, 0.0])
    piggy_start = (shock_slide_axis * 0.018) + piggy_offset
    piggy_end = (shock_slide_axis * 0.108) + piggy_offset
```

and

```python
    piggy_bridge_1 = shock_slide_axis * 0.030
    piggy_bridge_2 = (shock_slide_axis * 0.030) + piggy_offset
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: 6 passed

- [ ] **Step 7: Commit**

```bash
git add export_mujoco.py tests/test_frame_visuals.py
git commit -m "feat(visual): trunnion into frame, spec-derived damper proportions"
```

---

