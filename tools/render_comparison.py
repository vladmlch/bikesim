"""
Renders the simulated bike inside the reference photograph's own coordinate frame,
so frame fidelity is judged by measurement instead of memory.

Two artefacts are produced:

* ``docs/reference/comparison.png`` - photograph left, render right, captioned.
* ``docs/reference/overlay.png``    - render composited over the photograph at 50 %
  opacity in the *same* pixel frame, which is what actually answers "does the rear
  shock and seat tube look like the original".

Alignment is quantitative, not eyeballed. ``tools/photo_reference.py`` calibrated the
photograph (mm per pixel, bottom-bracket origin, image +x -> bike +X, image +y ->
bike -Z) and wrote the result to ``docs/reference/bulls_reference_points.json``.
This tool re-uses that calibration:

* the camera is switched to **orthographic** (``model.vis.global_.orthographic``), so
  the projection is an exact affine map with no perspective foreshortening - a product
  side-shot is effectively orthographic anyway;
* the orthographic height ``fovy`` is pinned to ``HEIGHT * mm_per_px``, so one rendered
  pixel spans exactly the same distance as one photograph pixel;
* the camera ``lookat`` is the least-squares translation registering the model's bottom
  bracket, front axle and rear axle onto their measured photo pixels.

The three residuals are asserted below ``RESIDUAL_TOLERANCE_PX`` and printed, because an
unaligned side-by-side would turn fidelity back into a matter of opinion.

Run with:  uv run --with pillow python -m tools.render_comparison
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Tuple

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from bike_sim.mujoco.builder import generate_mujoco_xml

REPO = Path(__file__).resolve().parent.parent
PHOTO = REPO / "docs" / "reference" / "bulls_sonic_evo_side.jpg"
REFERENCE_JSON = REPO / "docs" / "reference" / "bulls_reference_points.json"
COMPARISON_OUTPUT = REPO / "docs" / "reference" / "comparison.png"
OVERLAY_OUTPUT = REPO / "docs" / "reference" / "overlay.png"

WIDTH, HEIGHT = 1400, 1050

# Registration anchors: the only three points measured in BOTH frames independently
# of the linkage fit (bottom bracket plus the two axles).
ANCHOR_SITES: Dict[str, str] = {
    "bb": "site_BB",
    "rear_axle": "site_PRA",
    "front_axle": "site_PFA",
}
RESIDUAL_TOLERANCE_PX = 10.0

# Scenery that is not part of the bike and must stay out of the hero render.
NON_BIKE_GEOM_PREFIXES: Tuple[str, ...] = ("floor", "geom_stand")

SEPARATOR_PX = 4
CAPTION_PX = 46
CAPTION_BG = (24, 24, 26)
CAPTION_FG = (238, 238, 238)
RENDER_BACKGROUND = (255, 255, 255)
OVERLAY_ALPHA = 0.5

_FONT_CANDIDATES = (
    "/System/Library/Fonts/Supplemental/Arial.ttf",
    "/System/Library/Fonts/Helvetica.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


@dataclass(frozen=True)
class PhotoFrame:
    """The photograph's pixel-to-millimetre calibration, as produced by photo_reference."""

    origin_px: Tuple[float, float]
    mm_per_px: float

    @property
    def metres_per_px(self) -> float:
        return self.mm_per_px / 1000.0

    def px_of_mm(self, x_mm: float, z_mm: float) -> Tuple[float, float]:
        """Maps a bike-frame (X, Z) in mm to a photo pixel coordinate."""
        return (
            self.origin_px[0] + x_mm / self.mm_per_px,
            self.origin_px[1] - z_mm / self.mm_per_px,
        )


def load_photo_frame(reference_json: str | Path = REFERENCE_JSON) -> Tuple[PhotoFrame, Dict[str, Tuple[float, float]]]:
    """
    Loads the photograph calibration and the anchor pixel coordinates.

    Returns:
        (frame, anchors) where anchors maps each of ANCHOR_SITES' keys to its
        measured (px_x, px_y) position in the photograph.
    """
    payload = json.loads(Path(reference_json).read_text())
    cal = payload["calibration"]
    frame = PhotoFrame(origin_px=tuple(cal["origin_px"]), mm_per_px=float(cal["mm_per_px"]))
    anchors = {
        name: frame.px_of_mm(*payload["points_mm"][name])
        for name in ANCHOR_SITES
    }
    return frame, anchors


def build_stand_model(frame: PhotoFrame) -> Tuple[mujoco.MjModel, mujoco.MjData]:
    """
    Builds the stand-mode model at rest (zero travel) with a photo-matched ortho camera.

    Raises:
        RuntimeError: If the MJCF still carries debug marker geoms, which must never
            appear in a hero render.
    """
    xml = generate_mujoco_xml(mode="stand")
    if "marker_" in xml:
        raise RuntimeError(
            "Debug marker geoms are present in the stand-mode MJCF; refusing to render "
            "the comparison with the debug livery. Check generate_mujoco_xml(debug_markers=...)."
        )

    model = mujoco.MjModel.from_xml_string(xml)
    # Orthographic projection with a vertical extent of exactly HEIGHT photo pixels,
    # so rendered pixels and photograph pixels share one scale.
    model.vis.global_.orthographic = 1
    model.vis.global_.fovy = HEIGHT * frame.metres_per_px

    # Hide the ground plane and the test stand. Masking them out after the fact is not
    # enough: the stand clamp grips the seat tube and would punch a hole through exactly
    # the region this comparison exists to inspect. MuJoCo drops fully transparent geoms
    # from the scene, so they stop occluding as well as stop drawing.
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name.startswith(NON_BIKE_GEOM_PREFIXES):
            model.geom_matid[geom_id] = -1
            model.geom_rgba[geom_id] = (0.0, 0.0, 0.0, 0.0)

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def anchor_world_positions(model: mujoco.MjModel, data: mujoco.MjData) -> Dict[str, np.ndarray]:
    """Returns the world-frame (x, y, z) position of each registration anchor site."""
    positions: Dict[str, np.ndarray] = {}
    for name, site in ANCHOR_SITES.items():
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        if site_id < 0:
            raise RuntimeError(f"Anchor site {site!r} is missing from the model")
        positions[name] = np.array(data.site_xpos[site_id], dtype=float)
    return positions


def fit_camera(
    frame: PhotoFrame,
    anchors_px: Dict[str, Tuple[float, float]],
    world: Dict[str, np.ndarray],
) -> Tuple[mujoco.MjvCamera, Dict[str, float]]:
    """
    Registers the model onto the photograph and reports the per-anchor residuals.

    Scale is fixed by the photograph's calibration, so only the camera translation is
    free; solving it in least squares spreads the (real) frame mismatch over all three
    anchors instead of pinning one and letting the others drift.

    Returns:
        (camera, residuals_px) with one Euclidean pixel residual per anchor.
    """
    scale = frame.metres_per_px

    # For an orthographic camera at azimuth 90 / elevation 0, image right is +X and
    # image down is -Z, matching the photograph's axes:
    #     px_x = WIDTH/2  + (x - lookat_x) / scale
    #     px_y = HEIGHT/2 - (z - lookat_z) / scale
    lookat_x = float(np.mean([world[k][0] - (anchors_px[k][0] - WIDTH / 2.0) * scale for k in world]))
    lookat_z = float(np.mean([world[k][2] + (anchors_px[k][1] - HEIGHT / 2.0) * scale for k in world]))

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth = 90.0  # drive side, front wheel to image right
    camera.elevation = 0.0
    camera.distance = 6.0  # only affects clipping under an orthographic projection
    camera.lookat[:] = np.array([lookat_x, 0.0, lookat_z])

    residuals = {}
    for name, pos in world.items():
        px, py = project(pos, camera, frame)
        target = anchors_px[name]
        residuals[name] = float(np.hypot(px - target[0], py - target[1]))
    return camera, residuals


def project(point: Iterable[float], camera: mujoco.MjvCamera, frame: PhotoFrame) -> Tuple[float, float]:
    """
    Projects a world point into rendered-image pixels under the orthographic camera.

    Verified against actual renders: marker spheres injected into the scene land within
    ~1 px of the value returned here.
    """
    p = np.asarray(list(point), dtype=float)
    scale = frame.metres_per_px
    px = WIDTH / 2.0 + (p[0] - camera.lookat[0]) / scale
    py = HEIGHT / 2.0 - (p[2] - camera.lookat[2]) / scale
    return float(px), float(py)


def render_bike(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: mujoco.MjvCamera,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Renders the model and returns (rgb, mask) where mask selects bike pixels only.

    The mask comes from a segmentation pass, so the ground plane and the test stand are
    dropped exactly rather than by colour keying.
    """
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render().copy()

        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera=camera)
        seg = renderer.render().copy()
        renderer.disable_segmentation_rendering()

    geom_ids = seg[:, :, 0]
    obj_types = seg[:, :, 1]
    mask = (geom_ids >= 0) & (obj_types == int(mujoco.mjtObj.mjOBJ_GEOM))
    for geom_id in np.unique(geom_ids[mask]):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
        if name.startswith(NON_BIKE_GEOM_PREFIXES):
            mask &= geom_ids != geom_id
    return rgb, mask


def alignment_residuals() -> Dict[str, float]:
    """Returns the per-anchor registration residual in pixels, without rendering."""
    frame, anchors_px = load_photo_frame()
    model, data = build_stand_model(frame)
    _, residuals = fit_camera(frame, anchors_px, anchor_world_positions(model, data))
    return residuals


def _load_font(size: int) -> "ImageFont.FreeTypeFont | ImageFont.ImageFont":
    for candidate in _FONT_CANDIDATES:
        if Path(candidate).exists():
            try:
                return ImageFont.truetype(candidate, size)
            except OSError:
                continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10.1
        return ImageFont.load_default()


def _flatten(rgb: np.ndarray, mask: np.ndarray) -> Image.Image:
    """Composites the masked bike pixels over a flat background."""
    canvas = np.empty_like(rgb)
    canvas[:, :, :] = np.array(RENDER_BACKGROUND, dtype=rgb.dtype)
    canvas[mask] = rgb[mask]
    return Image.fromarray(canvas)


def compose_side_by_side(photo: Image.Image, render: Image.Image, residuals: Dict[str, float]) -> Image.Image:
    """Builds the captioned photograph-vs-render sheet."""
    total_w = WIDTH * 2 + SEPARATOR_PX
    canvas = Image.new("RGB", (total_w, HEIGHT + CAPTION_PX), CAPTION_BG)
    canvas.paste(photo, (0, 0))
    canvas.paste(render, (WIDTH + SEPARATOR_PX, 0))

    draw = ImageDraw.Draw(canvas)
    font = _load_font(24)
    worst = max(residuals.values())
    draw.text((16, HEIGHT + CAPTION_PX // 2), f"Reference photograph - {PHOTO.name}",
              fill=CAPTION_FG, font=font, anchor="lm")
    draw.text((WIDTH + SEPARATOR_PX + 16, HEIGHT + CAPTION_PX // 2),
              "MuJoCo model - stand mode, 0 mm travel, no debug markers "
              f"(anchor residual <= {worst:.1f} px)",
              fill=CAPTION_FG, font=font, anchor="lm")
    return canvas


def compose_overlay(photo: Image.Image, rgb: np.ndarray, mask: np.ndarray) -> Image.Image:
    """Blends the rendered bike over the photograph in the shared pixel frame."""
    base = np.asarray(photo, dtype=float).copy()
    alpha = mask[:, :, None].astype(float) * OVERLAY_ALPHA
    blended = base * (1.0 - alpha) + rgb.astype(float) * alpha
    return Image.fromarray(np.clip(blended, 0, 255).astype(np.uint8))


def render_comparison(
    output_path: str | Path = COMPARISON_OUTPUT,
    overlay_path: str | Path = OVERLAY_OUTPUT,
) -> Path:
    """
    Renders the bike onto the photograph's frame and writes both comparison artefacts.

    Args:
        output_path: Destination for the side-by-side sheet.
        overlay_path: Destination for the 50 % overlay.

    Returns:
        The path of the side-by-side sheet.

    Raises:
        RuntimeError: If debug markers are present, or if any anchor fails to register
            within RESIDUAL_TOLERANCE_PX.
    """
    frame, anchors_px = load_photo_frame()
    model, data = build_stand_model(frame)
    world = anchor_world_positions(model, data)
    camera, residuals = fit_camera(frame, anchors_px, world)

    bad = {k: v for k, v in residuals.items() if v > RESIDUAL_TOLERANCE_PX}
    if bad:
        detail = ", ".join(f"{k}={v:.2f}px" for k, v in sorted(bad.items()))
        raise RuntimeError(
            f"Render is not registered to the photograph within {RESIDUAL_TOLERANCE_PX} px: {detail}"
        )

    rgb, mask = render_bike(model, data, camera)

    photo = Image.open(PHOTO).convert("RGB")
    if photo.size != (WIDTH, HEIGHT):
        raise RuntimeError(
            f"Reference photograph is {photo.size}, expected ({WIDTH}, {HEIGHT}); the "
            "calibration in bulls_reference_points.json is tied to that pixel grid."
        )

    output = Path(output_path)
    overlay_out = Path(overlay_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    overlay_out.parent.mkdir(parents=True, exist_ok=True)

    compose_side_by_side(photo, _flatten(rgb, mask), residuals).save(output)
    compose_overlay(photo, rgb, mask).save(overlay_out)
    return output


def main() -> None:
    output = render_comparison()
    residuals = alignment_residuals()

    print(f"Wrote {output}")
    print(f"Wrote {OVERLAY_OUTPUT}")
    print("Anchor registration residuals (rendered vs photographed pixel):")
    for name in ANCHOR_SITES:
        print(f"  {name:11s} {residuals[name]:6.2f} px")


if __name__ == "__main__":
    main()
