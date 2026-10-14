"""Recalculate the Gemini tyre report's "Golden Reference Cases" TC-M1..TC-M3.

WP-V4 (round-2 adjudication). Every number is recomputed with the formulas
*as stated in the Gemini report itself*
(`docs/researches/gemini-взаимодействие пневматической шины….md`, table
"Эталонные численные тест-кейсы … для юнит-тестов `soft_layer`").

Run:  uv run python tools/research/adjudication/gemini_golden_cases.py
No dependencies beyond the standard library.
"""

import math

PASS_TOL_REL = 0.01  # verdict: |recomputed - claimed| <= 1% => "reproduces"


def check(name, claimed, recomputed, unit, tol_rel=PASS_TOL_REL):
    err = recomputed - claimed
    rel = abs(err) / abs(claimed) if claimed else float("inf")
    ok = rel <= tol_rel
    print(f"  {name:<38} claimed {claimed:>12.4f} {unit:<6} "
          f"recomputed {recomputed:>12.4f} {unit:<6} "
          f"diff {err:+10.4f} ({rel * 100:6.3f}%)  ->  {'OK' if ok else 'MISMATCH'}")
    return ok


def tc_m1():
    """TC-M1: pure Bekker before the hard-bottom breaking point.

    Gemini inputs: rectangular plate b = 0.058 m, l = 0.125 m (A = 0.00725 m^2);
    k_c = 0, k0 = 400 kPa, n = 0.80, H_s = 0.080 m, zeta_o = 0.50
    (z_o = 0.040 m); load F_z = 580 N (p = 80.0 kPa).

    Gemini claims:
      z_eff = b * (p/k0)^(1/n) = 0.058 * (80/400)^1.25 = 0.007757 m
      R_c   = F_z * z_eff / (l * (n+1))  = 27.00 N
      C_rr,comp = 0.04655
    """
    print("TC-M1 (pure Bekker, plate before bottom break point)")
    b, l = 0.058, 0.125
    k0 = 400e3        # Pa
    n = 0.80
    Fz = 580.0        # N
    p = Fz / (b * l)  # Pa; Gemini states 80.0 kPa

    z_eff = b * (p / k0) ** (1.0 / n)
    # Compaction work per unit travel: R_c = b * ∫_0^z p(z') dz'
    # with p(z) = k0 (z/b)^n  =>  ∫ = k0 z^(n+1) / (b^n (n+1)).
    # Since p(z_eff) = Fz/(b l) = k0 (z_eff/b)^n this reduces to
    # R_c = Fz * z_eff / (l (n+1))  — the formula Gemini prints.
    Rc = Fz * z_eff / (l * (n + 1.0))
    # Same quantity via the primitive form (cross-check of the algebra):
    Rc_direct = b * k0 * z_eff ** (n + 1.0) / (b ** n * (n + 1.0))
    Crr = Rc / Fz

    check("p (input consistency)", 80e3, p, "Pa")
    check("z_eff", 0.007757, z_eff, "m", tol_rel=0.005)
    check("R_c (own formula Fz*z/(l(n+1)))", 27.00, Rc, "N")
    check("R_c (primitive b∫p dz)", 27.00, Rc_direct, "N")
    check("C_rr,comp", 0.04655, Crr, "-")
    # What R_c would need as z_eff to yield 27.00 N:
    z_needed = 27.00 * l * (n + 1.0) / Fz
    print(f"    note: for R_c = 27.00 N the required z_eff = {z_needed:.5f} m, "
          f"not {z_eff:.6f} m")
    print()


def tc_m2():
    """TC-M2: exponential hardening near the rigid bottom (the 'Alzubaidi'
    / Salman et al. 2021 form).

    Gemini inputs: same plate, H_s = 0.035 m, z_o = 0.014 m, B_hp = 700 kPa,
    c_hp = 5.0; given z_eff = 0.0256 m  (Δz/b = (0.0256-0.014)/0.058 = 0.20).

    Gemini claims:
      Δk = 700 * (e^(5.0*0.20) - 1) = 1202.80 kPa
      k_app = 400 + 1202.80 = 1602.80 kPa
      (z/b)^0.8 = 0.51982 ;  p = 833.17 kPa  (4.01x vs no-bottom)
    """
    print("TC-M2 (hard-bottom hardening branch)")
    b = 0.058
    k0 = 400e3
    n = 0.80
    z_o = 0.014
    z = 0.0256
    B_hp = 700e3
    c_hp = 5.0

    dz_over_b = (z - z_o) / b
    dk = B_hp * (math.exp(c_hp * dz_over_b) - 1.0)
    k_app = k0 + dk
    zn = (z / b) ** n
    p = k_app * zn
    p_homog = k0 * zn
    ratio = p / p_homog

    check("Δz/b", 0.20, dz_over_b, "-")
    check("Δk", 1202.80e3, dk, "Pa", tol_rel=0.002)
    check("k_app", 1602.80e3, k_app, "Pa", tol_rel=0.002)
    check("(z/b)^0.8", 0.51982, zn, "-")
    check("p", 833.17e3, p, "Pa", tol_rel=0.002)
    check("p / p_homogeneous", 4.01, ratio, "-")
    print()


def tc_m3():
    """TC-M3: Janosi surface shear vs basal slab slide.

    Gemini inputs: F_z = 600 N, theta = 30 deg, H_s = 0.050 m, b = 0.058 m,
    l = 0.125 m, rho = 1600 kg/m^3, c = 0, phi_int = 40 deg, phi_bed = 30 deg,
    K_x = 0.020 m, j = 0.040 m.

    Gemini claims:
      F_x,janosi = 600 * tan40 * (1 - e^(-2.0)) = 435.33 N
      F_x,slab   = 600 * tan30 + 0             = 346.41 N
      active     = min = 346.41 N

    NOTE: the slab term is printed by Gemini as F_z * tan(phi_bed).
    Elsewhere in the *same* report the slab cap is given as
    F_z * (cos θ·tanφ_bed − sinθ) — which at θ = φ_bed = 30° is 0.
    We recompute both readings.
    """
    print("TC-M3 (Janosi vs basal slide)")
    Fz = 600.0
    theta = math.radians(30.0)
    phi_int = math.radians(40.0)
    phi_bed = math.radians(30.0)
    Kx = 0.020
    j = 0.040

    # Janosi shear: F = (c A + Fz tanφ) (1 - exp(-|j|/K)); c = 0
    F_janosi = Fz * math.tan(phi_int) * (1.0 - math.exp(-abs(j) / Kx))
    check("j/K_x", 2.0, abs(j) / Kx, "-")
    check("F_x,janosi", 435.33, F_janosi, "N")

    # Slab term as printed in the TC-M3 row: F_z * tan(phi_bed)
    F_slab_printed = Fz * math.tan(phi_bed)
    check("F_x,slab (as printed Fz·tanφbed)", 346.41, F_slab_printed, "N")

    # Slab term in the report's own body formula (slope-resolved):
    #   F_slab = Fz (cosθ tanφ_bed − sinθ)   [+ cohesion term, c=0]
    F_slab_resolved = Fz * (math.cos(theta) * math.tan(phi_bed)
                            - math.sin(theta))
    print(f"    note: report's own body formula gives "
          f"F_slab = {F_slab_resolved:.4f} N "
          f"(cos30·tan30 − sin30 = 0) — internally inconsistent")
    check("active limit min(F_j, F_slab)", 346.41,
          min(F_janosi, F_slab_printed), "N")
    print()


if __name__ == "__main__":
    print("=" * 100)
    print("Gemini tyre report golden cases TC-M1..TC-M3 — recomputation")
    print("=" * 100)
    tc_m1()
    tc_m2()
    tc_m3()
