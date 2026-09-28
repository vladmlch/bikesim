### Task 9: Side-by-Side Render Comparison

Makes similarity judgeable on one frame instead of from memory.

**Files:**
- Create: `tools/render_comparison.py`
- Create: `docs/reference/comparison.png` (generated)

**Interfaces:**
- Consumes: the MJCF builder from Tasks 4-7, `docs/reference/bulls_sonic_evo_side.jpg`
- Produces: `render_comparison(output_path: str | Path) -> Path`

- [ ] **Step 1: Write the tool**

Create `tools/render_comparison.py`:

```python
"""
Renders the simulated bike from the reference viewpoint and pastes it beside
the reference photograph, so fidelity is judged on a single frame.

Run with:  uv run --with pillow python -m tools.render_comparison
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from export_mujoco import generate_mujoco_xml

REPO = Path(__file__).resolve().parent.parent
PHOTO = REPO / "docs" / "reference" / "bulls_sonic_evo_side.jpg"
OUTPUT = REPO / "docs" / "reference" / "comparison.png"

WIDTH, HEIGHT = 1400, 1050


def render_comparison(output_path: str | Path = OUTPUT) -> Path:
    """Renders the model side-on and writes a side-by-side PNG against the photo."""
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="stand"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth = 90.0      # side-on, drive side, matching the product shot
    camera.elevation = 0.0
    camera.distance = 2.4
    camera.lookat[:] = np.array([0.15, 0.0, 0.30])

    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        renderer.update_scene(data, camera=camera)
        rendered = Image.fromarray(renderer.render())

    reference = Image.open(PHOTO).convert("RGB").resize((WIDTH, HEIGHT))
    canvas = Image.new("RGB", (WIDTH * 2, HEIGHT), "white")
    canvas.paste(reference, (0, 0))
    canvas.paste(rendered, (WIDTH, 0))

    output = Path(output_path)
    canvas.save(output)
    return output


if __name__ == "__main__":
    print(f"Wrote {render_comparison()}")
```

- [ ] **Step 2: Render and inspect**

Run:
```bash
uv run --with pillow python -m tools.render_comparison
```
Expected: `docs/reference/comparison.png` written. Open it and check the seat tube runs unbroken into the casting, the trunnion sits in frame material, and no debug markers appear.

- [ ] **Step 3: Commit**

```bash
git add tools/render_comparison.py docs/reference/comparison.png
git commit -m "feat(tooling): side-by-side render comparison against reference photo"
```

---

