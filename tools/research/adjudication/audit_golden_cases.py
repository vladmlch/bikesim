"""Recalculate the audit's angle-M golden cases G1..G12.

WP-V4 (round-2 adjudication). Sources:
`.scratch/tire-research/M/findings-memory.md` section (b), tables
"Эталонные случаи (вычислены/воспроизведены в этом запуске)" and the
RoverDevKit / PINNeAPPle conventions they cite:

  plate:    z = (p / k_eff)^(1/n),  k_eff = (k_c/b + k_phi) * 1e3
            (k_c in kN/m^2, k_phi in kN/m^3, result in Pa/m^n)
  work:     R_c = b * k_eff * z^(n+1) / (n+1)
  wheel:    Wong–Reece arc model, RoverDevKit convention
            theta_r = 0, theta_m = (0.4 + 0.2|s|) theta_1,
            j(theta) = R [(theta_1-theta) - (1-s)(sin theta_1 - sin theta)],
            tau = (c + sigma tanφ) (1 - exp(-|j|/K)) sign(j)

Run:  uv run python tools/research/adjudication/audit_golden_cases.py
No dependencies beyond the standard library.
"""

import math

TOL = 0.01  # 1% band for "reproduces"


def check(name, claimed, recomputed, unit, tol=TOL):
    err = recomputed - claimed
    rel = abs(err) / abs(claimed) if claimed else float("inf")
    ok = rel <= tol
    print(f"  {name:<42} claimed {claimed:>12.4f} {unit:<5} "
          f"recomputed {recomputed:>12.4f} {unit:<5} "
          f"diff {err:+10.4f} ({rel * 100:6.3f}%)  ->  {'OK' if ok else 'MISMATCH'}")
    return ok


def k_eff(b, kc_kN_m2, kphi_kN_m3):
    """Bekker effective modulus (kc/b + kphi), SI Pa/m^n."""
    return (kc_kN_m2 / b + kphi_kN_m3) * 1e3


def plate_cases():
    """G1..G6 — plate sinkage z = (p/k_eff)^(1/n)."""
    print("Plate golden cases G1..G6 (z = (p/k_eff)^(1/n))")
    b = 0.06
    cases = [
        # (id, soil kc kN/m2, kphi kN/m3, n, pressure kPa, claimed z mm)
        ("G1 JSC-1A p=10kPa", 1.4, 820.0, 1.0, 10.0, 11.86),
        ("G2 JSC-1A p=50kPa", 1.4, 820.0, 1.0, 50.0, 59.29),
        ("G3 GRC-1 p=50kPa", 0.7, 505.0, 0.8, 50.0, 53.98),
        ("G4 Apollo loose p=50kPa", 0.5, 400.0, 1.0, 50.0, 122.45),
        ("G5 Apollo dense p=50kPa", 2.0, 1200.0, 1.2, 50.0, 69.17),
    ]
    for name, kc, kphi, n, p_kpa, claimed_mm in cases:
        ke = k_eff(b, kc, kphi)
        z = (p_kpa * 1e3 / ke) ** (1.0 / n)
        check(name, claimed_mm, z * 1e3, "mm")

    # G6: design-band plate, single k_eq = 2 MPa/m, n = 1, p = 150 kPa
    z6 = 150e3 / 2e6
    check("G6 k_eq=2MPa/m n=1 p=150kPa", 75.0, z6 * 1e3, "mm")
    print()


def compaction_work():
    """G11 — compaction work Rc = b·k_eff·z^(n+1)/(n+1), JSC-1A, n=1.

    Claimed Rc = 18.66 N for b = 0.06 m, z = 27.2 mm (the G7 sinkage).
    Note: 27.2 mm is rounded; we test both 27.2 and 27.16 mm.
    """
    print("G11 compaction work (Rc = b·k_eff·z^2/2)")
    b = 0.06
    ke = k_eff(b, 1.4, 820.0)
    for z_mm in (27.2, 27.16):
        z = z_mm * 1e-3
        Rc = b * ke * z ** 2 / 2.0
        check(f"Rc z={z_mm}mm", 18.66, Rc, "N")
    print()


def wheel_solver(R, b, W, n, ke, c, phi_deg, K,
                 theta_r_frac, a0, a1, npts=4000):
    """Generic Wong–Reece solver, parameterised by convention.

    theta_r = theta_r_frac * theta_1  (rear contact angle; 0 => patch ends
    at bottom dead centre, negative => contact extends behind the wheel);
    theta_m = (a0 + a1|s|) theta_1    (angle of maximum normal stress);
    rear arc uses Reece's equivalent-angle mapping onto [theta_m, theta_1].
    Returns dict(theta_1, z, F_z, DP, T) for each requested slip via closure.
    """
    phi = math.radians(phi_deg)

    def solve(s):
        def sigma(theta, th1, th_m, theta_r):
            if theta >= th_m:
                z_th = R * (math.cos(theta) - math.cos(th1))
            else:
                th_eq = th1 - (theta - theta_r) * (th1 - th_m) / (th_m - theta_r)
                z_th = R * (math.cos(th_eq) - math.cos(th1))
            return ke * z_th ** n

        def tau(theta, th1, th_m, theta_r):
            j = R * ((th1 - theta) - (1.0 - s)
                     * (math.sin(th1) - math.sin(theta)))
            return (c + sigma(theta, th1, th_m, theta_r) * math.tan(phi)) \
                * (1.0 - math.exp(-abs(j) / K)) * (1.0 if j >= 0 else -1.0)

        def forces(th1):
            theta_r = theta_r_frac * th1
            th_m = (a0 + a1 * abs(s)) * th1
            fz = dp = tq = 0.0
            dth = (th1 - theta_r) / npts
            for i in range(npts):
                th = theta_r + (i + 0.5) * dth
                sg = sigma(th, th1, th_m, theta_r)
                ta = tau(th, th1, th_m, theta_r)
                fz += sg * math.cos(th) + ta * math.sin(th)
                dp += ta * math.cos(th) - sg * math.sin(th)
                tq += ta
            return R * b * dth * fz, R * b * dth * dp, R * R * b * dth * tq

        lo, hi = 1e-6, math.acos(1.0 - 0.9)
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            fz, _, _ = forces(mid)
            if fz > W:
                hi = mid
            else:
                lo = mid
        th1 = 0.5 * (lo + hi)
        fz, dp, tq = forces(th1)
        return th1, R * (1.0 - math.cos(th1)), fz, dp, tq

    return solve


def wong_reece_wheel():
    """G7–G9 — rigid wheel, Wong–Reece, RoverDevKit / PINNeAPPle conventions.

    G7: R=0.10, b=0.06, W=50 N, JSC-1A canonical (n=1.0, k_c=1.4 kN/m^2,
        k_phi=820 kN/m^3, c=1.0 kPa, phi=45 deg, K=0.018 m),
        RDK convention theta_r=0, theta_m=(0.4+0.2|s|)theta_1.
        Claimed: z = 27.16/27.07/27.58 mm, DP = -0.19/6.59/13.70 N,
                 T = 1.75/2.42/3.17 N m  (s = 0.05/0.2/0.5)
    G8: same inputs, PINNeAPPle default convention theta_r=-theta_1/3,
        theta_m=(0.4+0.15|s|)theta_1.
        Claimed: z = 22.71 mm; DP = 4.22/13.12/22.18 N.
    G9: R=0.10, b=0.10, W=40 N, s=0.6, Apollo nominal (c=0.17 kPa, phi=46 deg,
        same n/k_c/k_phi), RDK convention.
        Claimed: z = 17.79 mm, DP = 11.786 N, T = 2.349 N m.
    """
    print("G7 wheel R=0.10 b=0.06 W=50N JSC-1A (Wong-Reece, RDK convention)")
    R, b, W = 0.10, 0.06, 50.0
    solve = wheel_solver(R, b, W, n=1.0, ke=k_eff(b, 1.4, 820.0),
                         c=1.0e3, phi_deg=45.0, K=0.018,
                         theta_r_frac=0.0, a0=0.4, a1=0.2)
    for s, (zc, dpc, tc) in {0.05: (27.16, -0.19, 1.75),
                             0.20: (27.07, 6.59, 2.42),
                             0.50: (27.58, 13.70, 3.17)}.items():
        th1, z, fz, dp, tq = solve(s)
        print(f"  s = {s:<5} theta_1 = {math.degrees(th1):7.3f} deg  "
              f"F_z = {fz:7.3f} N")
        check(f"z   s={s}", zc, z * 1e3, "mm")
        check(f"DP  s={s}", dpc, dp, "N", tol=0.05)
        check(f"T   s={s}", tc, tq, "N·m", tol=0.05)
    print()

    print("G8 same wheel, PINNeAPPle default convention "
          "(theta_r=-th1/3, a1=0.15)")
    solve8 = wheel_solver(R, b, W, n=1.0, ke=k_eff(b, 1.4, 820.0),
                          c=1.0e3, phi_deg=45.0, K=0.018,
                          theta_r_frac=-1.0 / 3.0, a0=0.4, a1=0.15)
    for s, (zc, dpc) in {0.05: (22.71, 4.22),
                         0.20: (22.71, 13.12),
                         0.50: (22.71, 22.18)}.items():
        th1, z, fz, dp, tq = solve8(s)
        check(f"z   s={s}", zc, z * 1e3, "mm")
        check(f"DP  s={s}", dpc, dp, "N", tol=0.05)
    print()

    print("G9 R=0.10 b=0.10 W=40N s=0.6 Apollo nominal (RDK pinned test)")
    solve9 = wheel_solver(0.10, 0.10, 40.0, n=1.0, ke=k_eff(0.10, 1.4, 820.0),
                          c=0.17e3, phi_deg=46.0, K=0.018,
                          theta_r_frac=0.0, a0=0.4, a1=0.2)
    th1, z, fz, dp, tq = solve9(0.6)
    check("z   s=0.6", 17.79, z * 1e3, "mm")
    check("DP  s=0.6", 11.786, dp, "N")
    check("T   s=0.6", 2.349, tq, "N·m")
    print()


def terzaghi_nc():
    """G12 — bearing-capacity factor N_c at phi = 30 deg.

    Two conventions circulate under the same name:
      * Prandtl–Reissner closed form: N_q = e^{pi tanφ} tan^2(45+φ/2),
        N_c = (N_q − 1) cotφ              -> N_c(30°) = 30.14
      * Terzaghi (1943) chart value for general shear:
                                          N_c(30°) ≈ 37.2
    The audit quotes 37.16 ± 1% — i.e. the Terzaghi chart convention.
    Both are "correct"; they must not be mixed inside one formula set.
    """
    print("G12 bearing capacity N_c(30 deg) — convention check")
    phi = math.radians(30.0)
    Nq = math.exp(math.pi * math.tan(phi)) * math.tan(math.radians(45) + phi / 2) ** 2
    Nc_prandtl = (Nq - 1.0) / math.tan(phi)
    print(f"  Prandtl–Reissner: N_q = {Nq:8.3f}   N_c = {Nc_prandtl:8.3f}")
    print(f"  Terzaghi 1943 chart value quoted by audit: N_c = 37.16")
    print(f"  -> audit value matches the Terzaghi convention, "
          f"NOT the Prandtl closed form ({Nc_prandtl:.2f})")
    print()


if __name__ == "__main__":
    print("=" * 100)
    print("Audit angle-M golden cases — recomputation")
    print("=" * 100)
    plate_cases()
    compaction_work()
    wong_reece_wheel()
    terzaghi_nc()