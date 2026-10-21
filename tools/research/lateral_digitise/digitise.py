"""Digitise Dressel & Sadauckas 2020 (Appl. Sci. 10:3156) Fig. 17a/b.

Extracts stiffness-coefficient-vs-pressure curves for every colour family,
then splits same-colour series by connected components + value band.

Two independent passes:
  pass A  raster embedded in the PDF (figs/p18_img498.png, 1725x1022)
          axis calibration: least-squares on OCR-verified tick-label centres
          point statistic : per-column median of mask pixels
  pass B  publisher full-res raster (figs/applsci10_03156_g017.png, 3457x3399)
          axis calibration: spine box edges -> declared tick limits,
          checked against detected label-block centres
          point statistic : per-column mean of component pixels

Series colour/marker key (recovered from the legend, notes/legend_map.md):
  colour = tyre family;  marker = construction variant
  blue   filled square   29x2.3" knobby  on 25 mm   <- primary series
  blue   open   square   29x2.3" BALD    on 25 mm
  red    filled tri      29x2.5" file-tread on 25 mm
  red    open   square   29x2.5" BALD    on 25 mm
  cyan   filled square   29x2.3" knobby  on 22 mm
  magenta filled square  29x2.5" file-tread on 22 mm
  green  filled square   27.5x2.8" knobby on 38 mm  (Fig.6 legend: 38 mm)
  orange filled square   29x3.0" knobby  on 45 mm
  brown  filled square   26x4.0" knobby  on 86 mm
"""

import json
import numpy as np
from PIL import Image
from scipy import ndimage
from pathlib import Path

HERE = Path(__file__).parent
FIGS = HERE / "figs"
OUT = HERE / "points"


def masks_rgb(rgb):
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    m = {
        "blue":    (b > 140) & (b > r + 60) & (b > g + 60),
        "cyan":    (b > 150) & (g > 130) & (r < 120) & (b > g + 20),
        "green":   (g > 110) & (g > r + 50) & (g > b + 40),
        "red":     (r > 170) & (r > g + 60) & (r > b + 60),
        "magenta": (r > 170) & (b > 150) & (r > g + 80) & (b > g + 60),
        "orange":  (r > 200) & (g > 120) & (g < 215) & (b < 100),
        "brown":   (r > 140) & (r < 235) & (g > 55) & (g < 150)
                   & (b < 90) & (r > g + 40),
    }
    m["orange"] = m["orange"] & ~m["brown"]
    return m


def components_of(mask, min_size):
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(lab == i)
        if len(ys) >= min_size:
            out.append((xs, ys))
    return out


def column_profile(xs, ys, x0, x1, stat):
    """per-column representative y for one connected component."""
    order = np.argsort(xs)
    xs, ys = xs[order], ys[order]
    data = {}
    for x in np.unique(xs):
        col = ys[xs == x]
        data[int(x)] = float(np.median(col)) if stat == "median" \
            else float(np.mean(col))
    return data


def panel_curves(img, box, xtick_val_at, ytick_val_at, stat, min_size):
    """Return {colour: [ (comp_id, {pxx->pxy}), ...] } plus calibration."""
    x0, x1, y0, y1 = box
    cal = None
    if xtick_val_at is not None and ytick_val_at is not None:
        (px_lo, v_lo), (px_hi, v_hi) = xtick_val_at
        ax = (v_hi - v_lo) / (px_hi - px_lo)
        bx = v_lo - ax * px_lo
        (py_lo, u_lo), (py_hi, u_hi) = ytick_val_at
        ay = (u_hi - u_lo) / (py_hi - py_lo)
        by = u_lo - ay * py_lo
        cal = (ax, bx, ay, by)
    rgb = np.asarray(img).astype(int)
    res = {}
    for key, mask in masks_rgb(rgb).items():
        comps = components_of(mask[y0:y1 + 1, x0:x1 + 1], min_size)
        ser = []
        for cid, (xs, ys) in enumerate(comps):
            prof = column_profile(xs + x0, ys + y0, x0, x1, stat)
            ser.append((cid, prof))
        if ser:
            res[key] = ser
    return res, cal


def to_data(prof, cal):
    ax, bx, ay, by = cal
    xs = np.array(sorted(prof))
    ys = np.array([prof[x] for x in xs])
    return ax * xs + bx, ay * ys + by


# ============================================================ pass A =======
def pass_a():
    img = Image.open(FIGS / "p18_img498.png")
    # panel a: spine box x[189,801] y[21,631]; ticks: x 0..60 psi;
    #   label centres fitted: rows [628,504,386,265,142,21]<->[0,5,..,25]
    res = {}
    # -- panel a -------------------------------------------------------------
    a_cal_rows = np.array([628, 504, 386, 265, 142, 21], float)
    a_cal_vals = np.array([0, 5, 10, 15, 20, 25], float)
    ay, by = np.polyfit(a_cal_rows, a_cal_vals, 1)     # val = ay*y + by
    ax = 60.0 / (800.5 - 189.5); bx = -ax * 189.5      # spines 0..60 psi
    resA, _ = panel_curves(img, (190, 800, 23, 630), None, None,
                           "median", 40)
    res["17a"] = (resA, (ax, bx, ay, by))
    # -- panel b -------------------------------------------------------------
    b_cal_rows = np.array([628, 526, 424, 324, 221, 119, 17], float)
    b_cal_vals = np.array([0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
    ay2, by2 = np.polyfit(b_cal_rows, b_cal_vals, 1)
    ax2 = 60.0 / (1604.5 - 997.5); bx2 = -ax2 * 997.5
    resB, _ = panel_curves(img, (998, 1604, 21, 629), None, None,
                           "median", 40)
    res["17b"] = (resB, (ax2, bx2, ay2, by2))
    return res


# ============================================================ pass B =======
def pass_b():
    img = Image.open(FIGS / "applsci10_03156_g017.png")
    res = {}
    # panel a spine box: x 371..1592 -> 0..60 psi ; y 1431..212 -> 0..25
    resA, _ = panel_curves(img, (371, 1592, 213, 1431), None, None,
                           "mean", 120)
    ax = 60.0 / (1592 - 371); bx = -ax * 371
    ay = -25.0 / (1431 - 213); by = -ay * 1431.5
    res["17a"] = (resA, (ax, bx, ay, by))
    # panel b spine box: x 1989..3201 -> 0..60 psi ; y 1431..213 -> 0..3.0
    resB, _ = panel_curves(img, (1989, 3201, 213, 1431), None, None,
                           "mean", 120)
    ax2 = 60.0 / (3201 - 1989); bx2 = -ax2 * 1989
    ay2 = -3.0 / (1431 - 213); by2 = -ay2 * 1431.5
    res["17b"] = (resB, (ax2, bx2, ay2, by2))
    return res


def dump(res, tag):
    OUT.mkdir(exist_ok=True)
    meta = {}
    for panel, (series, cal) in res.items():
        meta[panel] = {"calibration val=a*px+b":
                       {"x": [cal[0], cal[1]], "y": [cal[2], cal[3]]},
                       "series": {}}
        for colour, comps in series.items():
            for cid, prof in comps:
                xs, ys = to_data(prof, cal)
                name = f"{panel}_{colour}_c{cid}"
                meta[panel]["series"][name] = {
                    "n": len(xs), "psi_range": [float(xs.min()),
                                              float(xs.max())],
                    "val_range": [float(ys.min()), float(ys.max())]}
                with open(OUT / f"{tag}_{name}.csv", "w") as f:
                    f.write("psi,value_1_per_rad\n")
                    for x, y in zip(xs, ys):
                        f.write(f"{x:.4f},{y:.5f}\n")
    with open(OUT / f"{tag}_meta.json", "w") as f:
        json.dump(meta, f, indent=1)
    return meta


if __name__ == "__main__":
    for tag, fn in (("passA", pass_a), ("passB", pass_b)):
        meta = dump(fn(), tag)
        for panel in ("17a", "17b"):
            print(f"--- {tag} {panel}")
            for name, m in sorted(meta[panel]["series"].items()):
                print(f"  {name:26s} n={m['n']:4d} psi {m['psi_range'][0]:5.1f}"
                      f"-{m['psi_range'][1]:5.1f} val {m['val_range'][0]:5.2f}"
                      f"-{m['val_range'][1]:5.2f}")