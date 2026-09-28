### Task 4: Continuous Seat Tube and Frame Casting

Removes the split-strut cage and replaces the linkage-region structure with a continuous seat tube dying into a casting, as the reference silhouette shows.

**Files:**
- Modify: `export_mujoco.py:755-822` (split seat tube block)
- Test: `tests/test_frame_visuals.py` (create)

**Interfaces:**
- Consumes: fitted `P5`, `P10` from Task 3
- Produces: geoms named `geom_seattube`, `geom_frame_yoke`; removes `geom_seattube_upper`, `geom_seattube_strut_l_top`, `geom_seattube_strut_l_bot`, `geom_seattube_strut_r_top`, `geom_seattube_strut_r_bot`

- [ ] **Step 1: Write the failing test**

Create `tests/test_frame_visuals.py`:

```python
"""Structural assertions on the generated MJCF frame visuals."""

import xml.etree.ElementTree as ET

import mujoco
import pytest

from export_mujoco import generate_mujoco_xml

SEAT_TUBE_TERMINATION_Z_M = 0.258


@pytest.fixture(scope="module")
def root():
    return ET.fromstring(generate_mujoco_xml(mode="stand"))


def _geom_names(root):
    return {g.get("name") for g in root.iter("geom")}


def test_split_seat_tube_cage_is_gone(root):
    names = _geom_names(root)
    for removed in (
        "geom_seattube_strut_l_top", "geom_seattube_strut_l_bot",
        "geom_seattube_strut_r_top", "geom_seattube_strut_r_bot",
        "geom_seattube_upper",
    ):
        assert removed not in names, f"{removed} should have been removed"


def test_seat_tube_is_continuous_and_terminates_at_the_casting(root):
    names = _geom_names(root)
    assert "geom_seattube" in names
    assert "geom_frame_yoke" in names

    seattube = next(g for g in root.iter("geom") if g.get("name") == "geom_seattube")
    coords = [float(v) for v in seattube.get("fromto").split()]
    lower_z = min(coords[2], coords[5])
    assert lower_z == pytest.approx(SEAT_TUBE_TERMINATION_Z_M, abs=0.015)


def test_model_still_compiles(root):
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="stand"))
    assert model.ngeom > 0
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: FAIL — `geom_seattube_upper` still present, `geom_seattube` and `geom_frame_yoke` missing.

The builder is `generate_mujoco_xml(specs=None, solver=None, mode="standard", mass_specs=None, include_rider=True, bump_scale=1.0, pit_scale=1.0) -> str` at `export_mujoco.py:65`. It returns the MJCF string directly, so the tests parse its output with `ET.fromstring` and need no temporary files.

- [ ] **Step 3: Replace the split seat tube block**

In `export_mujoco.py`, delete lines 755-822 (the `# Split Seat Tube (Shock Tunnel Architecture)` block through `geom_seattube_strut_r_bot`) and substitute:

```python
    # Seat Tube (continuous) + Frame Casting
    #
    # Reference measurement: the real seat tube runs straight from the collar and
    # dies into the frame casting at Z ~= 258 mm, where the silhouette width steps
    # from 29 mm to 68 mm. It never reaches the BB, which is why no split-strut
    # cage is needed to clear the rocker. Shapes below the tube are AUTHORED
    # STYLING - the white-on-white front triangle is not measurable from the photo.
    seattube_end = np.array([-0.030, 0.0, 0.258])
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_seattube",
            "type": "capsule",
            "fromto": _format_fromto(P10, seattube_end),
            "size": "0.020",
            "mass": "0.45",
            "material": "mat_frame",
        },
    )
    # Motor / shock-tunnel casting: carries the rocker pivot P5 and spans forward
    # of the linkage. Sized from the measured silhouette (X -35..+33 at Z = 255).
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_frame_yoke",
            "type": "box",
            "pos": f"{(seattube_end[0] + P5[0]) / 2.0:.6f} 0 {(seattube_end[2] + 0.0) / 2.0:.6f}",
            "size": "0.075 0.036 0.130",
            "mass": "0.55",
            "material": "mat_frame",
        },
    )
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add export_mujoco.py tests/test_frame_visuals.py
git commit -m "feat(visual): continuous seat tube into frame casting, drop strut cage"
```

---

