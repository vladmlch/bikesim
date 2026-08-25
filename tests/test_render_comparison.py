"""Tests for the photo-registered render comparison tool."""

from pathlib import Path

import pytest

# Pillow is deliberately NOT a project dependency; the tool is run with
# `uv run --with pillow`. Skip cleanly rather than fail when it is absent.
pytest.importorskip("PIL")

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from tools.render_comparison import (  # noqa: E402
    ANCHOR_SITES,
    CAPTION_PX,
    HEIGHT,
    RESIDUAL_TOLERANCE_PX,
    SEPARATOR_PX,
    WIDTH,
    anchor_world_positions,
    build_stand_model,
    fit_camera,
    load_photo_frame,
    render_comparison,
)


@pytest.fixture(scope="module")
def artefacts(tmp_path_factory) -> tuple[Path, Path]:
    out = tmp_path_factory.mktemp("render_comparison")
    comparison = out / "comparison.png"
    overlay = out / "overlay.png"
    returned = render_comparison(comparison, overlay)
    assert returned == comparison
    return comparison, overlay


def test_writes_both_artefacts(artefacts):
    comparison, overlay = artefacts
    for path in (comparison, overlay):
        assert path.exists(), f"{path.name} was not written"
        assert path.stat().st_size > 0, f"{path.name} is empty"


def test_artefacts_have_the_expected_dimensions(artefacts):
    comparison, overlay = artefacts
    assert Image.open(comparison).size == (WIDTH * 2 + SEPARATOR_PX, HEIGHT + CAPTION_PX)
    assert Image.open(overlay).size == (WIDTH, HEIGHT)


def test_rendered_half_is_not_blank(artefacts):
    """Guards against a black frame, a white frame, or a render that failed silently."""
    comparison, _ = artefacts
    sheet = np.asarray(Image.open(comparison).convert("RGB"))
    rendered = sheet[:HEIGHT, WIDTH + SEPARATOR_PX:].astype(float)

    assert rendered.std() > 20.0, f"rendered half looks uniform (std={rendered.std():.2f})"
    # A real render is neither mostly-black nor entirely background.
    ink = (rendered.mean(axis=2) < 200).mean()
    assert 0.05 < ink < 0.80, f"implausible ink coverage in the rendered half: {ink:.3f}"


def test_overlay_differs_from_the_bare_photograph(artefacts):
    """The overlay must actually carry the render, not just re-save the photo."""
    from tools.render_comparison import PHOTO

    _, overlay = artefacts
    photo = np.asarray(Image.open(PHOTO).convert("RGB")).astype(float)
    blended = np.asarray(Image.open(overlay).convert("RGB")).astype(float)
    changed = (np.abs(blended - photo).max(axis=2) > 8).mean()
    assert changed > 0.05, f"overlay barely differs from the photograph ({changed:.3f})"


def test_anchors_register_onto_the_photograph():
    """BB and both axles must land within tolerance of their measured photo pixels."""
    frame, anchors_px = load_photo_frame()
    model, data = build_stand_model(frame)
    _, residuals = fit_camera(frame, anchors_px, anchor_world_positions(model, data))

    assert set(residuals) == set(ANCHOR_SITES)
    for name, residual in residuals.items():
        assert residual < RESIDUAL_TOLERANCE_PX, f"{name} residual {residual:.2f} px"


def test_stand_mode_xml_carries_no_debug_markers():
    from bike_sim.mujoco import generate_mujoco_xml

    assert "marker_" not in generate_mujoco_xml(mode="stand")
