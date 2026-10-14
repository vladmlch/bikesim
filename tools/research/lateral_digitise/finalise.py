"""Produce canonical digitised tables (pass mean + spread) for the primary
series and compute derived quantities (exponents, sigma_y, ratios)."""
import numpy as np, json, csv
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
    xs, ys = xs[o], ys[o]
    # median-filter to kill marker-thickness noise
    sm = np.copy(ys)
    for i in range(len(ys)):
        sm[i] = np.median(ys[max(0, i - 3):i + 4])
    return xs, sm


SERIES = {
    "17a": {
        "knobby_29x2.3_25mm": ["blue_c1", "blue_c2"],
        "bald_29x2.3_25mm":   ["blue_c0"],
        "knobby_29x2.3_22mm": ["cyan_c0"],
        "filetread_29x2.5_25mm": ["red_c1", "red_c2", "red_c3",
                                  "red_c4", "red_c5"],
        "bald_29x2.5_25mm":   ["red_c0"],
        "filetread_29x2.5_22mm": ["magenta_c0", "magenta_c1"],
        "knobby_27.5x2.8_38mm": ["green_c0", "green_c1"],
        "knobby_29x3.0_45mm":   ["orange_c1", "orange_c2"],
        "knobby_26x4.0_86mm":   ["brown_c0", "brown_c1", "orange_c0"],
    },
    "17b": {
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


def main():
    psis = np.array([10, 15, 20, 25, 30, 35, 40, 45, 50], float)
    summary = {}
    for panel, unit in (("17a", "C_alpha/Fz"), ("17b", "C_gamma/Fz")):
        summary[panel] = {}
        for name, comps in SERIES[panel].items():
            ser = {}
            for tag in ("passA", "passB"):
                pp = []
                for c in comps:
                    try:
                        pp.append(load(tag, f"{panel}_{c}"))
                    except OSError:
                        pass
                if pp:
                    xs, ys = merge(pp)
                    m = (xs >= 8) & (xs <= 51)
                    ser[tag] = (xs[m], ys[m])
            if not ser:
                continue
            lo = max(s[0].min() for s in ser.values())
            hi = min(s[0].max() for s in ser.values())
            va = np.interp(psis, *ser["passA"]) if "passA" in ser else None
            vb = np.interp(psis, *ser["passB"]) if "passB" in ser else None
            rows = []
            for p, a, b in zip(psis, va if va is not None else [np.nan]*9,
                               vb if vb is not None else [np.nan]*9):
                ok = (ser["passA"][0].min() <= p <= ser["passA"][0].max() if
                      "passA" in ser else False) and \
                     (ser["passB"][0].min() <= p <= ser["passB"][0].max() if
                      "passB" in ser else False)
                if not ok:
                    continue
                mean = (a + b) / 2
                err = abs(a - b) / 2
                rows.append((p, p * PSI2BAR, a, b, mean, err))
            summary[panel][name] = rows
            with open(OUT / f"final_{panel}_{name}.csv", "w",
                      newline="") as f:
                w = csv.writer(f)
                w.writerow(["psi", "bar", "passA", "passB",
                            "mean", "half_spread"])
                for r in rows:
                    w.writerow([f"{r[0]:.0f}", f"{r[1]:.3f}", f"{r[2]:.4f}",
                                f"{r[3]:.4f}", f"{r[4]:.4f}", f"{r[5]:.4f}"])
    with open(OUT / "final_summary.json", "w") as f:
        json.dump({p: {n: [[float(x) for x in r] for r in rows]
                       for n, rows in d.items()}
                   for p, d in summary.items()}, f, indent=1)
    # --- derived quantities -------------------------------------------------
    ka = dict(summary["17a"])["knobby_29x2.3_25mm"]
    kb = dict(summary["17b"])["knobby_29x2.3_25mm"]
    print("knobby 29x2.3'' 25mm  (Fz=418 N)")
    print(" psi  bar   Ca/Fz  pm    |  Cg/Fz  pm")
    am = {r[0]: r for r in ka}; bm = {r[0]: r for r in kb}
    for p in psis:
        if p in am and p in bm:
            print(f"{p:4.0f} {p*PSI2BAR:5.2f}  {am[p][4]:5.2f} ±{am[p][5]:.2f}"
                  f"   |  {bm[p][4]:5.2f} ±{bm[p][5]:.2f}")
    # exponents from pass-mean profiles
    for lbl, data in (("C_alpha", ka), ("C_gamma", kb)):
        p = np.array([r[0] for r in data]); v = np.array([r[4] for r in data])
        for lo, hi in ((10, 50), (20, 50), (10, 25)):
            m = (p >= lo) & (p <= hi)
            c = np.polyfit(np.log(p[m]), np.log(v[m]), 1)
            print(f"{lbl} n_p[{lo}-{hi}psi] = {c[0]:+.3f}")
    # sigma_y from BCDs and Ky (Fig.8a printed value 27710 N/m)
    BCDs, Fz, Ky = 10.488, 418.0, 27710.0
    print(f"\nsigma_y = C_alpha/K_y = {BCDs*Fz:.0f}/{Ky} = "
          f"{BCDs*Fz/Ky:.4f} m  (@25 psi)")
    print(f"sigma_y/sigma_x (sigma_x=60mm authored) = "
          f"{BCDs*Fz/Ky/0.06:.2f}")


if __name__ == "__main__":
    main()
