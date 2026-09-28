#!/usr/bin/env python3
"""
Pedal-Cycle Visual Check -- the eyeball harness for the articulated legs.

Builds a pedal-mode `RideSimulation` on a lengthened flat track, settles it onto its
cruise target, then steps ~1.5 crank revolutions while rendering the rider's legs in
profile at evenly spaced crank phases. Each phase is saved from both sides -- the
near leg hides the far one in a strict side view -- plus a two-row contact strip.

    uv run --with pillow python -m tools.render_pedals
    uv run --with pillow python -m tools.render_pedals --speed 15 --settle 18

Outputs land in ``output/pedal_check/``:

* ``pedal_XX_az090.png`` -- camera on -Y (the front/right crank-arm side),
* ``pedal_XX_az270.png`` -- camera on +Y (the rear/left crank-arm side),
* ``pedal_strip.png``    -- both rows composited, captioned with crank degrees.

The track is a 300 m copy of ``flat``: the shipped 112 m preset ends under the bike
at ~27 s of 20 km/h riding, which is mid-capture for this script's settle+rev window
(see validate_pedals' sustained-load check, which lengthens the same way).

This tool exists because "the numbers pass" is not "it looks like pedalling": read
the PNGs. Knees must flex forward (knee ahead of the hip-ankle line, shin sweeping
back on the upstroke), feet must stay glued to their pedals, and the foot/platform
pitch should stay near level rather than toe-down ballet or heel-down braking.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from math import degrees, pi
from pathlib import Path
from typing import List, Optional, Tuple

REPO_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(REPO_SRC))

import mujoco
import numpy as np

WIDTH, HEIGHT = 960, 720
REPO = Path(__file__).resolve().parent.parent
DEFAULT_OUTDIR = REPO / "output" / "pedal_check"

# Side-on framing of the leg group: centre the shot midway between the BB and the
# pelvis -- that is the box hip/knee/ankle move through -- and zoom so one leg fills
# the frame. Too tight a crop crops the far pedal at dead centres; 1.35 m covers the
# crank circle plus the thighs.
LOOKAT_UP_M = 0.34
LOOKAT_AHEAD_M = -0.08
CAMERA_DISTANCE_M = 1.35
CAMERA_ELEVATION_DEG = -8.0
AZIMUTHS: Tuple[Tuple[int, str], ...] = ((90, "az090"), (270, "az270"))

# `vis.map.znear` and `zfar` are fractions of `stat.extent`, which on a ride track is
# the full 300 m of terrain: the shipped 0.01 puts the near clip ~3.2 m out, so any
# close leg shot renders literally nothing (the bike is inside the near plane).
# 2e-4 lands the near plane at ~6 cm without touching zfar.
RENDER_ZNEAR_FRACTION = 2.0e-4


def _build_sim(speed_kmh: float, track_length_m: float):
    """Compiles the pedalled sim on a flat track long enough to settle on."""
    from bike_sim.physics.drivetrain import DrivetrainSpecs
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.terrain import get_preset

    track = replace(get_preset("flat"), name="flat_render", length_m=track_length_m)
    return RideSimulation(
        track=track,
        target_speed_kmh=speed_kmh,
        drive_mode="pedal",
        assist="off",
        drivetrain=DrivetrainSpecs(),
        legs="articulated",
    )


def _crank_phase(sim) -> float:
    """Current crank angle, rad, straight off the drivetrain's qpos address."""
    return float(sim.data.qpos[sim.drivetrain.crank_qposadr])


def _camera(sim, azimuth_deg: float) -> mujoco.MjvCamera:
    """Side camera tracking the BB, framing crank to hip."""
    m, d = sim.model, sim.data
    bb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "site_BB")
    anchor = np.array(d.site_xpos[bb]) if bb >= 0 else np.array(d.xpos[1])
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth = azimuth_deg
    camera.elevation = CAMERA_ELEVATION_DEG
    camera.distance = CAMERA_DISTANCE_M
    camera.lookat[:] = anchor + np.array([LOOKAT_AHEAD_M, 0.0, LOOKAT_UP_M])
    return camera


def _step_to_phase(sim, target_rad: float, max_steps: int) -> None:
    """Steps until the crank reaches `target_rad` (phase grows monotonically)."""
    for _ in range(max_steps):
        sim.step()
        if sim.crash is not None:
            return
        if _crank_phase(sim) >= target_rad:
            return


def _weld_separation_m(sim) -> float:
    """Worst current |site_foot - site_pedal| over both legs, metres."""
    m, d = sim.model, sim.data
    worst = 0.0
    for side in ("front", "rear"):
        foot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_foot_{side}")
        pedal = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"site_pedal_{side}")
        if foot >= 0 and pedal >= 0:
            worst = max(worst, float(np.linalg.norm(
                d.site_xpos[foot] - d.site_xpos[pedal])))
    return worst


def _load_font(size: int):
    from PIL import ImageFont
    for candidate in (
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(candidate).exists():
            try:
                return ImageFont.truetype(candidate, size)
            except OSError:
                continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def _compose_strip(rows: List[List[Tuple[Path, str]]], out_path: Path) -> Path:
    """Composites saved PNGs into a captioned grid; rows of (path, caption)."""
    from PIL import Image, ImageDraw

    caption_px, sep_px, bg = 34, 4, (24, 24, 26)
    n_cols = max(len(r) for r in rows)
    canvas = Image.new(
        "RGB",
        (n_cols * WIDTH + (n_cols - 1) * sep_px,
         len(rows) * (HEIGHT + caption_px) + (len(rows) - 1) * sep_px),
        bg,
    )
    draw = ImageDraw.Draw(canvas)
    font = _load_font(22)
    for r, row in enumerate(rows):
        for c, (path, caption) in enumerate(row):
            x = c * (WIDTH + sep_px)
            y = r * (HEIGHT + caption_px + sep_px)
            canvas.paste(Image.open(path).convert("RGB"), (x, y))
            draw.text((x + 12, y + HEIGHT + caption_px // 2), caption,
                      fill=(238, 238, 238), font=font, anchor="lm")
    canvas.save(out_path)
    return out_path


def render_pedal_cycle(
    speed_kmh: float,
    settle_s: float,
    revolutions: float,
    n_phases: int,
    outdir: Path,
    track_length_m: float,
) -> List[Path]:
    """
    Settles a pedalled run, then renders `n_phases` side views over `revolutions`.

    Returns:
        Paths of every file written, strip last.
    """
    from PIL import Image

    sim = _build_sim(speed_kmh, track_length_m)
    assert sim.drivetrain is not None, "pedal mode must build the drivetrain"
    assert sim.leg_drive.active, "articulated legs must be active for this check"
    sim.model.vis.map.znear = RENDER_ZNEAR_FRACTION  # see constant: visual-only fix

    settle_steps = int(settle_s / sim.model.opt.timestep)
    for step in range(settle_steps):
        sim.step()
        if sim.crash is not None:
            raise RuntimeError(
                f"sim crashed during settle at t={sim.data.time:.1f}s: {sim.crash.cause}")
    cmd = sim.drivetrain.command
    print(f"settled: t={sim.data.time:.1f}s v={sim.speed_mps * 3.6:.2f} km/h "
          f"cadence={cmd.cadence_rpm:.1f} rpm x={sim.data.qpos[0]:.1f} m "
          f"(freewheel={'open' if cmd.freewheel else 'closed'})")

    phase0 = _crank_phase(sim)
    delta = revolutions * 2.0 * pi / n_phases
    outdir.mkdir(parents=True, exist_ok=True)
    written: List[Path] = []
    rows: List[List[Tuple[Path, str]]] = [[] for _ in AZIMUTHS]

    with mujoco.Renderer(sim.model, height=HEIGHT, width=WIDTH) as renderer:
        for k in range(n_phases):
            target = phase0 + (k + 1) * delta
            _step_to_phase(sim, target, int(delta / 6.0 / sim.model.opt.timestep) + 4000)
            if sim.crash is not None:
                raise RuntimeError(
                    f"sim crashed at t={sim.data.time:.1f}s during capture: {sim.crash.cause}")
            phase_deg = degrees(_crank_phase(sim)) % 360.0
            sep_mm = _weld_separation_m(sim) * 1000.0
            for row, (azimuth, tag) in zip(rows, AZIMUTHS):
                renderer.update_scene(sim.data, camera=_camera(sim, azimuth))
                path = outdir / f"pedal_{k:02d}_{tag}.png"
                Image.fromarray(renderer.render()).save(path)
                written.append(path)
                row.append((path, f"phase {phase_deg:5.1f} deg  weld {sep_mm:4.2f} mm"))
            print(f"  phase {k}: crank={phase_deg:6.1f} deg  "
                  f"weld sep {sep_mm:.2f} mm  t={sim.data.time:.2f}s")

    strip = _compose_strip(rows, outdir / "pedal_strip.png")
    written.append(strip)
    return written


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--speed", type=float, default=20.0,
                        help="cruise target, km/h (default 20)")
    parser.add_argument("--settle", type=float, default=22.0,
                        help="settle time before capturing, s (default 22)")
    parser.add_argument("--revolutions", type=float, default=1.5,
                        help="crank revolutions to cover (default 1.5)")
    parser.add_argument("--phases", type=int, default=8,
                        help="number of crank phases to capture (default 8)")
    parser.add_argument("--track-length", type=float, default=300.0,
                        help="flat track length, m (default 300; shipped flat is 112 m "
                             "and ends under the bike mid-run)")
    parser.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR,
                        help=f"output directory (default {DEFAULT_OUTDIR})")
    args = parser.parse_args(argv)

    try:
        import PIL  # noqa: F401
    except ImportError:
        sys.exit("Pillow is required: uv run --with pillow python -m tools.render_pedals")

    written = render_pedal_cycle(
        speed_kmh=args.speed,
        settle_s=args.settle,
        revolutions=args.revolutions,
        n_phases=args.phases,
        outdir=args.outdir,
        track_length_m=args.track_length,
    )
    print(f"wrote {len(written)} files under {args.outdir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
