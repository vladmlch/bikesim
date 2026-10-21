# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///
"""Gate check: diff a fresh stats2_out.json against the pinned reference.

  uv run compare_stats.py [fresh.json] [expected.json]

Defaults: fresh=stats2_out.json (cwd), expected=expected/stats2_out.json
next to this script. Exit 0 iff every numeric field matches within
rounding (abs diff < 5e-4 on quantiles, < 5e-4 on fractions, runs/km
integer-exact in metres as emitted).
"""
import json
import os
import sys


def cmp_num(a, b, path, tol):
    d = abs(a - b)
    if isinstance(a, int) and isinstance(b, int) or path.endswith((".km",)):
        ok = abs(a - b) < 0.5
    else:
        ok = d <= tol * max(1.0, abs(b))
    if not ok:
        print(f"DIFF {path}: {a} vs {b}"); return 1
    return 0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    fresh = sys.argv[1] if len(sys.argv) > 1 else "stats2_out.json"
    ref = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "expected", "stats2_out.json")
    a = json.load(open(fresh)); b = json.load(open(ref))
    bad = 0
    if sorted(a) != sorted(b):
        print("key sets differ:", set(a) ^ set(b)); sys.exit(2)
    for k in a:
        for f in set(a[k]) | set(b[k]):
            va, vb = a[k].get(f), b[k].get(f)
            if isinstance(va, list) or isinstance(vb, list):
                if va is None or vb is None or len(va) != len(vb):
                    print(f"DIFF {k}.{f}: {va} vs {vb}"); bad += 1; continue
                for i, (u, v) in enumerate(zip(va, vb)):
                    bad += cmp_num(u, v, f"{k}.{f}[{i}]", 5e-4)
            else:
                bad += cmp_num(va, vb, f"{k}.{f}", 5e-4)
    print("fields compared OK" if bad == 0 else f"{bad} differing fields")
    sys.exit(0 if bad == 0 else 1)


if __name__ == "__main__":
    main()