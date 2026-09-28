### Task 7: Debug Livery Toggle

Puts the pivot markers and diagnostic materials behind a flag so hero renders show the bike, not the debug rig.

**Files:**
- Modify: `export_mujoco.py:934-945` (frame markers), `:1652-1665` (rocker markers), `:1758-1759` (yoke markers)
- Modify: `export_mujoco.py:65-72` (`generate_mujoco_xml` signature)
- Modify: `run_playground.py:300-307` (the `load_model` call into `generate_mujoco_xml`)
- Test: `tests/test_frame_visuals.py` (extend)

**Interfaces:**
- Consumes: nothing new
- Produces: MJCF builder gains keyword argument `debug_markers: bool = False`; every `marker_*` geom and every diagnostic `site` is emitted only when it is `True`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_frame_visuals.py`:

```python
def test_markers_are_absent_by_default():
    root_el = ET.fromstring(generate_mujoco_xml(mode="stand"))
    markers = [g.get("name") for g in root_el.iter("geom")
               if (g.get("name") or "").startswith("marker_")]
    assert markers == [], f"hero render must have no debug markers, found {markers}"


def test_markers_return_when_requested():
    root_el = ET.fromstring(generate_mujoco_xml(mode="stand", debug_markers=True))
    markers = {g.get("name") for g in root_el.iter("geom")
               if (g.get("name") or "").startswith("marker_")}
    assert "marker_p5_l" in markers
    assert "marker_p7" in markers


def test_equality_constraint_sites_survive_marker_removal():
    """site_P3_ss / site_P3_rocker / site_P7_shock drive loop closure - never optional."""
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="stand"))
    site_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
                  for i in range(model.nsite)}
    for required in ("site_P3_ss", "site_P3_rocker", "site_P7_shock", "site_P6_yoke"):
        assert required in site_names
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: FAIL — markers present by default; `debug_markers` is an unexpected keyword.

- [ ] **Step 3: Thread the flag through the builder**

Add `debug_markers: bool = False` to the MJCF builder signature and to any wrapper that forwards to it. Guard every `marker_*` geom with it, for example at `export_mujoco.py:934`:

```python
    # Visual Pivot Markers (debug only - matching engineering sketch)
    if debug_markers:
        ET.SubElement(frame, "geom", {"name": "marker_bb", ...})
        ...
```

Apply the same guard to the rocker markers (`marker_p3_roc_l/r`, `marker_p4_l/r`) and the yoke markers (`marker_p6_l/r`). **Do not guard** `site_P3_ss`, `site_P3_rocker`, `site_P6_yoke` or `site_P7_shock` — the `<equality><connect>` elements reference them and the model will not compile without them.

- [ ] **Step 4: Expose it at the playground call site**

In `run_playground.py`, add `self.debug_markers = False` beside the other display flags in `__init__` (near `self.show_telemetry` at line 288), then forward it in `load_model` at line 300:

```python
        xml_str = generate_mujoco_xml(
            specs=self.specs,
            solver=self.solver,
            mode=mode,
            include_rider=self.include_rider,
            bump_scale=self.bump_scale,
            pit_scale=self.pit_scale,
            debug_markers=self.debug_markers,
        )
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py tests/test_playground.py -v`
Expected: all pass

- [ ] **Step 6: Commit**

```bash
git add export_mujoco.py run_playground.py tests/test_frame_visuals.py
git commit -m "feat(visual): gate debug markers behind a flag"
```

---

