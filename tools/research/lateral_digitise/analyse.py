"""Assemble digitised series into named tyre curves, compare pass A/B,
compute stiffness values and pressure exponents.

Series assignment (Fig.17, panel a = cornering, b = camber):
  knobby 29x2.3" 25mm : blue  lower curve (a: ~9.4-12.7, b: ~0.67-1.55)
  bald   29x2.3" 25mm : blue  upper curve
  knobby 29x2.3" 22mm : cyan
  file-tread 29x2.5" 25mm : red  lower curve at >20 psi
  bald   29x2.5" 25mm : red  upper curve
  file-tread 29x2.5" 22mm : magenta
  knobby 27.5x2.8" 38mm : green
  knobby 29x3.0" 45mm  : orange (low-val comps; ~22 comp is 26x4 bleed)
  knobby 26x4.0" 86mm  : brown (+orange_high comp, same physical curve)
"""
import numpy as np, json
from pathlib import Path

OUT = Path(__file__).parent / "points"
PSI2BAR = 0.0689476


def load(tag, name):
    d = np.loadtxt(OUT / f"{tag}_{name}.csv", delimiter=",", skiprows=1)
    return d[:, 0], d[:, 1]


def merge(parts):
    xs = np.concatenate([p[0] for p in parts])
    ys = np.concatenate([p[1] for p in parts])
    o = np.argsort(xs)
    return xs[o], ys[o]


def smooth(xs, ys, win=7):
    """median filter on sorted profile"""
    import numpy as np
    out = np.copy(ys)
    for i in range(len(ys)):
        lo = max(0, i - win // 2); hi = min(len(ys), i + win // 2 + 1)
        out[i] = np.median(ys[lo:hi])
    return out


def at(xs, ys, psi):
    return float(np.interp(psi, xs, ys))


SERIES = {
    "17a": {   # cornering stiffness coefficient
        "knobby_29x2.3_25mm": ["blue_c1", "blue_c2"],
        "bald_29x2.3_25mm":   ["blue_c0"],
        "knobby_29x2.3_22mm": ["cyan_c0"],
        "filetread_29x2.5_25mm": ["red_c1", "red_c2", "red_c3", "red_c4",
                                  "red_c5"],
        "bald_29x2.5_25mm":   ["red_c0"],
        "filetread_29x2.5_22mm": ["magenta_c0", "magenta_c1"],
        "knobby_27.5x2.8_38mm": ["green_c0", "green_c1"],
        "knobby_29x3.0_45mm":   ["orange_c1", "orange_c2"],
        "knobby_26x4.0_86mm":   ["brown_c0", "brown_c1", "orange_c0"],
    },
    "17b": {   # camber stiffness coefficient
        "knobby_29x2.3_25mm": ["blue_c1", "blue_c2", "blue_c3", "blue_c4"],
        "bald_29x2.3_25mm":   ["blue_c0"],
        "knobby_29x2.3_22mm": ["cyan_c0"],
        "filetread_29x2.5_25mm": ["red_c2", "red_c4", "red_c5", "red_c6"],
        "bald_29x2.5_25mm":   ["red_c0", "red_c1", "red_c3"],
        "filetread_29x2.5_22mm": ["magenta_c0"],
        "knobby_27.5x2.8_38mm": ["green_c0", "green_c1", "green_c2"],
        "knobby_29x3.0_45mm":   ["orange_c0", "orange_c1", "orange_c2"],
        "knobby_26x4.0_86mm":   ["brown_c0", "brown_c1"],
    },
}


def build(tag, panel):
    got = {}
    for name, parts in SERIES[panel].items():
        pp = []
        for p in parts:
            try:
                pp.append(load(tag, f"{panel}_{p}"))
            except OSError:
                pass
        if pp:
            xs, ys = merge(pp)
            ys = smooth(xs, ys)
            got[name] = (xs, ys)
    return got


def main():
    tables = {}
    for panel in ("17a", "17b"):
        A = build("passA", panel)
        B = build("passB", panel)
        tables[panel] = {}
        for name in sorted(set(A) & set(B)):
            xa, ya = A[name]; xb, yb = B[name]
            psis = sorted(set(np.round(np.concatenate([xa, xb])).astype(int)))
            tables[panel][name] = {"A": (xa, ya), "B": (xb, yb)}
    # comparison at key pressures
    print(f"{'panel':5s} {'series':24s} {'psi':>4s} {'bar':>5s} "
          f"{'A':>6s} {'B':>6s} {'diff%':>6s}")
    rows = []
    for panel in ("17a", "17b"):
        for name, d in tables[panel].items():
            xa, ya = d["A"]; xb, yb = d["B"]
            lo = max(xa.min(), xb.min()); hi = min(xa.max(), xb.max())
            for psi in range(10, 55, 5):
                if not (lo <= psi <= hi):
                    continue
                va, vb = at(xa, ya, psi), at(xb, yb, psi)
                diff = abs(va - vb) / ((va + vb) / 2) * 100
                rows.append((panel, name, psi, va, vb, diff))
    for r in rows:
        print(f"{r[0]:5s} {r[1]:24s} {r[2]:4d} {r[2]*PSI2BAR:5.2f} "
              f"{r[3]:6.2f} {r[4]:6.2f} {r[5]:6.1f}")
    # summary numbers for the knobby 25mm tyre
    for panel, label in (("17a", "C_alpha/Fz"), ("17b", "C_gamma/Fz")):
        A = tables[panel]["knobby_29x2.3_25mm"]["A"]
        B = tables[panel]["knobby_29x2.3_25mm"]["B"]
        print(f"\n== {label} 29x2.3 knobby 25mm ==")
        for psi, bar in ((10, .69), (15, 1.03), (20, 1.38), (25, 1.72),
                         (30, 2.07), (40, 2.76), (50, 3.45)):
            print(f"  {psi:3d} psi ({bar:4.2f} bar): "
                  f"A={at(*A, psi):5.2f}  B={at(*B, psi):5.2f}")
        # pressure exponent on knobby: fit log(val) = c + np*log(psi)
        for tag, (xs, ys) in (("A", A), ("B", B)):
            for lo, hi in ((10, 50), (20, 50), (10, 25)):
                m = (xs >= lo) & (xs <= hi)
                c = np.polyfit(np.log(xs[m]), np.log(ys[m]), 1)
                print(f"  pass{tag} n_p over {lo}-{hi} psi: {c[0]:+.3f}")


if __name__ == "__main__":
    main()
