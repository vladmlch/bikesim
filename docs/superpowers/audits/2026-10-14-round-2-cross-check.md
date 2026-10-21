# Cross-check: internal audits vs external deep-research reports (ChatGPT, Gemini)

Date: 2026-10-14. Two read-only passes by independent agents, each reading three files in
full. Purpose: find where the external reports in `docs/researches/` agree with, contradict,
or add to the internal audits before any of it reaches a specification. Verdicts and
resolution paths are in `docs/superpowers/plans/2026-10-14-round-2-integration-master.md` §1;
this file is the evidence. "—" = the document does not address the point. Numbers are as the
reports state them; none were re-verified here.

---

## Part 1 — Tyre × scree (audit `2026-10-14-deep-research-round-2.md` vs ChatGPT tyre report vs Gemini tyre report)

**a) Scree model form**

| | Position | Numbers | Source |
|---|---|---|---|
| Audit | Bimodal: explicit clasts plus Bekker on the matrix only | b/d50 ≈ 0.5–6 fails the thresholds b/d ≥ 4–5 and B/d50 ≥ 30–35; d_split = b_p/5 ≈ 10 mm; rigid if H_s/d50 ≲ 2–3 | angle N |
| ChatGPT | Agrees on hybrid. Conflict: refuses any universal threshold ("do not bake b/d > 5"), wants a measured classifier on B/d90 | B/d table: 2 mm→30, 10 mm→6, 30 mm→2, 100 mm→0.6 | Cerato & Lutenegger scale effects |
| Gemini | Agrees, adds 3 regimes | Continuum when d50 < 12 mm (λ_b > 4.83). Hybrid at 12–40 mm, with H_s,eff = 0.6·H_s. Discrete only when d50 > 40 mm (λ_b < 1.45, H_s := 0). Rigid when H_s ≤ max(1.2·d50, 10 mm); H_min = 12 mm | Gotteland & Benoit 2006, Alzubaidi 2021 |

**b) Pressure–sinkage near the rigid bottom**

| | Position | Numbers | Source |
|---|---|---|---|
| Audit | Hardening branch near the bottom | `k_app = k + B(e^{c·Δz/D} − 1)` for z > z_o, `z_o = max(0, H_s − (1.0–1.5)·b)` | Salman 2021 |
| ChatGPT | Agrees hardening is needed. Conflict on onset: starts at H/B up to ~3 (≈165–180 mm, whole layer) | Multiplicative `p = p_∞(z)·G(z/H_s, H_s/B)`, G ≥ 1, uncalibrated | Cerato & Lutenegger 2006 |
| Gemini | Same exponential form. Conflict on onset: fraction of H_s | `k_app = k0 + B_hp(e^{c_hp(z−z_o)/b} − 1)`, z_o = ζ_o·H_s, ζ_o 0.35–0.55 (nominal 0.45); B_hp 400–1200 kPa; c_hp 3.5–8.0; bottom felt for H_s < 1.8·D | Alzubaidi et al. 2021, Appl. Sci. 11:5499 (possibly = audit's "Salman 2021") |

**c) Parameter bands**

| | k_eq / k_c, k_φ | n | φ | c | Slip–sinkage | Crr |
|---|---|---|---|---|---|---|
| Audit | k_eq 0.5–10 MPa/mⁿ; 0.4–2.4 documented | 0.7–1.3 | repose 33–38°, peak 40–48° (A measured 43–60°) | — | g_s 1.5–2.5, widened to 0.5–10; alt. n_eff = n0 + n1·s, n1 0.64–0.94 | keep 0.02–0.04; 0.018–0.030 measured-adjacent |
| ChatGPT | All null/unpinned; k_c, k_φ only with ≥ 2 plate widths | null | null; repose ≠ φ; nonlinear rockfill envelope | 0 (authored) | z = z_static + z_slip; Ding 3–7× static at s = 0.6, non-transferable | Do not pin. Meyer: 0.01221 / 0.01468 / 0.01832. Knobby +21 ± 15 % over smooth |
| Gemini | k_c 0–2 kN/m^(n+1); k_φ 150–600 kN/m^(n+2) (k_eq ≈ 0.15–0.6 MPa/mⁿ); also k0 250–900 kPa in `p = k0·(z/b)^n` | 0.70–0.90/0.95 (mean 0.8) | θ1 33–38°, φ_int 38–45°, φ_bed 27–34° | 0 | Rejects n0 + n1·s. Uses z = z_eff + z_s, z_s non-bearing; k_s,max 0.015–0.045, k_s,exp 1–2 | drum 0.01265; thin layer 0.025–0.045; deep loose 0.045–0.075 |

Notes: Gemini's k0 band converts to k_eq ≈ 2.4–8.8 MPa/mⁿ (b = 0.058 m, n = 0.8), ~15× its own k_φ band — internally inconsistent. Gemini's objection to coupling slip to F_z applies equally to the audit's `k/(1+g_s·i)` form.

**d) Traction cap**

| | Position | Numbers | Source |
|---|---|---|---|
| Audit | `min(μ_rubber, c/σ + tanφ)`; slope correction tanφ − tanθ; at bedrock cap degrades to μ_layer | Cap falls ~6× at 33° with φ = 38°; m_τ, m_int ∈ [1.0, 1.3]; r_c_knob 0.7–1.0; μ_layer ~0.35; braking 0.25–0.58 g | A, G, K, L, Moreland |
| ChatGPT | `min(τ_local, τ_basal)`, second term calibrated threshold/flag; adds combined-slip D_s(|s_x|). Disagrees on knobs: "a universal multiplier would be unjustified" | No numbers; no tanθ formula | Ding grousers |
| Gemini | `min(F_janosi(φ_eff, K_x), F_slab(φ_bed, H_s, θ))` + flag `scree_slab_slide_active` | γ_knob 1.20–1.35 (nom. 1.25); tanφ_eff 0.80–1.05; μ_rubber–rock 0.55–0.70; knobs cut K_x by 20–30 % (to 15–25 mm). Slab term (cosθ·tanφ_bed − sinθ); at θ = 30°, φ_bed = 31° cap ≈ 0.60·F_z; μ_bed 0.51–0.67 | Pouliquen/Ancey (extrapolated), Leite 2012 |

**e) Layer-slide regime**

| | Position | Numbers |
|---|---|---|
| Audit | `h_stop/d = B·(tanθ₂ − tanθ)/(tanθ − tanθ₁)`; metastable band h_stop…h_start; a 300–1200 N wheel is above threshold | θ₁ 33–38°, θ₂ 45–52°; θ ≥ θ₂ → bare rock |
| ChatGPT | Failure mode transfers, coefficients do not; build from a local tilt test; `basal_layer_failure` off until phase 2 | — |
| Gemini | Same mechanism, different form: `tanθ = tanθ1 + (tanθ2 − tanθ1)·e^{−h_stop/(L_p·d50)}`; slab slide *primary* failure at 25–35° (extrapolated) | θ1 33–38°, θ2 43–48°, L_p 1.5–3.5; ρ_bulk 1500–1750 kg/m³ |

**f) MTB lateral data**

| | Status | C_α/F_z | C_γ/F_z | σ_y/σ_x | Exponents |
|---|---|---|---|---|---|
| Audit | Sadauckas & Dressel 2020, pinned | 9.5–13 | 0.9–1.5 | 2.5 (2–3), σ_y 0.15–0.19 m; spec default 0.8 wrong | n_F −0.55; n_p −0.2 (C_α), −0.5 (C_γ) |
| ChatGPT | Paper exists (29×2.3 at ~1.72 bar, down to 10 psi). Conflict: all values null/unpinned until digitised | null | "below roughly 1 rad⁻¹" | null; no transfer from Dell'Orto | null, "tire-specific" |
| Gemini | Exists (0.69–3.45 bar), pinned | 5.8–8.6 (nom. 7.2 at 1.5 bar, 418 N); slick 8.5–11.5 | 0.55–0.99 (nom. 0.78); slick/file 1.05–1.45 | 1.15–1.65 (nom. 1.35), unpinned; σ_y 0.09–0.18 m | q_Fz 0.85 on absolute C_Fy (≈ −0.15 normalised); p_Cα −0.22; p_Cγ −0.32 |

Neither external report defends the 0.8 default. The audit's C_α band resembles Gemini's *slick* band — possible tyre-selection difference.

**g) Multipass / level B**

| | Position | Numbers |
|---|---|---|
| Audit | Not needed; level A stays; J caveat | +5–10 % rear traction < uncertainty bands |
| ChatGPT | Disagrees: "state persistence cannot wait" | 80–90 % of strain on first pass (VT, non-transferable); 0.17–1.44 s front→rear |
| Gemini | Agrees: no grid memory; keeps per-wheel z_peak with `K_u = K0 + A_u·z_peak` (DLR) | < 1 %, < 0.1 % when F_z,front < 50 N; tracks diverge 30–100 mm |

**h) Determinism**

| | Position |
|---|---|
| Audit | Pinned pow via fdlibm/CORE-MATH (libm 0.5–1.73 ulp apart); -ffp-contract=off; Jacobi/post-pass; Chrono SCM counterexample (unordered_map, atomics, in-place stencils, commit 1a33029); 32-iteration bisection |
| ChatGPT | Agrees on risks (pow/exp/tanh, FMA, hash order, atomics; Box2D, Demmel); softer fix: "test platforms explicitly *or* controlled implementation"; treats Chrono (same commit) as equation oracle, not nondeterministic |
| Gemini | Agrees: CORE-MATH `cr_pow`; libm 0.52–3.0 ULP; adds Padé[4/4] `det_exp_neg`, `-fno-fast-math -fno-associative-math`, MSVC `/fp:strict`; fixed 8-step Newton–bisection; lexicographic (ring_j, ray_i) or pairwise reduction; Chrono "vulnerable" |

**i) Bulldozing lateral force**

| | Formula | Share of F_y |
|---|---|---|
| Audit | `R_b(z) = D₁(c·z + D₂·γ·z²/2)` (Ishigami); F_bulld = Σ R_b·ΔL_i; camber thrust on soft patch → 0 | 35–50 % |
| ChatGPT | F_y = F_y,tire + F_y0(α, γ, z, θ)·D_s(|s_x|); no bulldozing formula | — |
| Gemini | F_y = F_u + F_s; F_u ∝ (1 − s); F_s ∝ ρ·g·K_pγ·z_down²; K_pγ = tan²(45° + φ/2) = 3.8–5.2; z_down = z0 + (b/2)·sin|γ|; c_γ 0.35–0.65. Camber *raises* F_s — conflicts with audit's camber → 0 | 25–45 % at z_down 25–50 mm; small when z < 10 mm |

**j) Controller relevance**

| | Position |
|---|---|
| Audit | No front-wheel sensor on serial e-MTB TC; anti-wheelie by IMU pitch (Bosch US12565285B2); anomaly "spins but does not spin up" |
| ChatGPT | No J content beyond a "controller-validation scenarios" node |
| Gemini | Agrees no front sensor. Bosch CX BDU384Y/386Y (120 Nm, 750 W, 600 %); TC Off/Low/Med/High (fw 1.38). Three anomalies: (1) z_s 40–60 mm sinking at 50–150 mm/s → false +2–3° pitch → false anti-wheelie trigger; (2) flat Janosi curve needs s 0.20–0.35, so "High" TC at s ≈ 0.05 delivers < 40 % of traction; (3) slab slide: no ω̇ spike and a_x < 0 fools the speed observer. Does not cite the patent |

**k) Items only in external reports**

ChatGPT: combined-slip degradation D_s on soft-ground lateral force; plate protocol fitting k_eq(H/B), n(H/B) with 40/70 mm plates, H = 30/50/70/100 mm, ≥ 5 repacks; Crr matrix 1.2/1.6/2.0 bar × virgin/front-track/firm; 0.01832 as firm-gravel regression test; check whether Janosi curve peaks then softens; fixed-slip protocol F_x/F_z = f(s, F_z, p, H, packing); cleat test with lateral offset centre→shoulder; stable-sort by cell ID before writing terrain state.

Gemini: non-bearing slip-sinkage ODE (Barthelmes 2018); quantified F_x,slab with A_bed = (b + H_s)(l + H_s), φ_bed; H_min = 12 mm activation; H_s,eff = 0.6·H_s in hybrid regime; unloading law K_u = K0 + A_u·z_max (compaction raises k_app 40–75 %); knob-driven K_x reduction; pressure-inversion test (2.0→1.3 bar cuts F_rr 8–15 %; ChatGPT: trend "not established"); golden cases TC-M1..M3; test `DropSled_Sadauckas2024_15psi` (48.7 ± 3.5 mm deflection, ε₁ 0.85–0.94, ζ 0.02–0.05, sag 8.9 mm); Magic Formula with E = 0; FastBike twisting torque; k_lat_couple 3.5–8 N/mm; rock-normal projections for M_x, M_z; temperature: F_y −16 %, M_z −50 % between 35 and 70 °C; flags "Hurrell 2025" as unverified.

Arithmetic check: Gemini's TC-M1 golden value R_c = 27.00 N does not follow from its own formula (F_z·z/(l(n+1)) = 580·0.007757/(0.125·1.8) ≈ 20.0 N). TC-M2, TC-M3 recompute correctly.

**l) Contradictions > 2×**

1. k_eq upper bound: audit 10 MPa/mⁿ vs Gemini k_φ ≤ 0.6 (≈ 17×); Gemini's k0 band contradicts its own k_φ band.
2. Multipass gain: +5–10 % vs < 1 % (≥ 5×); ChatGPT disagrees on conclusion, not number.
3. Hardening onset: 1–1.5·b (55–82 mm) vs ChatGPT H/B ~3 (~170 mm); Gemini's ζ_o form gives no hardening for z < 10–16 mm at H_s = 30 mm.
4. Slope-corrected cap at 30°: (tan38° − tan30°) ≈ 0.20 vs Gemini slab cap ≈ 0.60·F_z (3×).
5. Load exponent: n_F −0.55 vs Gemini ≈ −0.15 normalised (≈ 3.7×), basis unclear.
6. Crr deep loose scree: Gemini 0.045–0.075 vs audit 0.018–0.030 vs ChatGPT firm 0.0183.
7. Sinkage solver: 32 bisections vs 8 Newton–bisection (numerical contract, not physics).

Under 2× but notable: C_α/F_z 9.5–13 vs 5.8–8.6 (non-overlapping); C_γ/F_z 0.9–1.5 vs 0.55–0.99; σ_y/σ_x 2.5 vs 1.35; μ_layer 0.35 vs μ_bed 0.51–0.67; rigid threshold H_s/d50 2–3 vs 1.2; slip–sinkage at s = 0.6 1.5–4× vs 3–7×; pinned vs unpinned status of lateral JSON.

---

## Part 2 — Track (audit `2026-10-14-track-deep-research.md` vs ChatGPT track report vs Gemini track report)

**a) Grade distribution**

| | Run length | 8–15 % share | >25 % share (T3) | P90 | Steep runs / 120 m | Sampling |
|---|---|---|---|---|---|---|
| Audit | median 4–8 m; ramp_len (4,22) | 14–26 % | ~35–38 % | |g5| 41–42 % | 2–5 (median 10–12 m, P90 42–73 m) | quantile; long tail P99/P50 3–4.5 |
| ChatGPT | "unresolved, keep authored" | — | — | — | "learn from profiles"; 1/120 m unsupported | no beta/lognormal; OSM bounds as truncation; U3 prior 10–25 % with excursions 25–30 % |
| Gemini | split by surface: loose 6–18 m, bedrock 10–35 m | traverses 8–14 % | S3-up 5–15 %, S4-up 15–25 % | S3-up P90 25 % (P50 18); S2 P90 20; S4 P90 32 | 1–3 | preset-conditioned quantiles; Marion & Wimpey ANP >20 % = 1.6 % |

**b) Scree**

| | Crossing length | scree_ramp_bias | Scree on 26–32 % walls | H_s |
|---|---|---|---|---|
| Audit | median 34/46/64 m, p95 500–580 m; mix 25 % 3–15 m … 15 % >150 m | 0.6–0.7 only as P(grade ≥ 15 % | scree); 3D tie to 20–35° terrain; TSA 46/28/26 %; grade p10–p90 6–44 % | μ_req 0.46–0.48, "almost impossible" | 3–10 cm = unmeasured veneer (incision proxy 4–6 cm); bulk 2–5 m / 10–25 m; untouched 0.2–1.0 m |
| ChatGPT | "no trail-specific distribution", keep authored | remove; replace with P(scree | host_slope, alignment, unit); talus 28–36° | traction = min(rubber–clast, granular shear) | agrees unverified |
| Gemini | pocket 3–10 m, traverse 10–30 m (cones to 75 m) | 0.40–0.50; 0–0.1 on walls >28 %; traverse grade 5–14 % | agrees: absent on >28 %; scree climb limit 18–24 % | Conflict on provenance: WTA handbook "4 in (≈10 cm, 3–15 cm)" said to verify 0.03–0.12 m |

**c) Rocks**

| | H/D | Golombek k → count | Boulder H | Steps 5–30 cm | Step length |
|---|---|---|---|---|---|
| Audit | 0.4–0.5 | k 0.05/0.08–0.16 → 9–44 rocks ≥ 5 cm per 10 m | bg 0.03–0.12; rocky 0.08–0.30; tail 0.4 | ≥ 1 per 10–20 m (S2+); 0.05–0.30; chainring 0.25–0.35 | 0.3–1.0 m |
| ChatGPT | no number; "diameter ≠ protrusion" | q = 1.79 + 0.152/k, k 5–40 % analogue only | decimetre tail | rate to be learned | — |
| Gemini | 0.5 (Golombek & Rapp) and 0.31 ± 0.20 (Chang'E-3, N = 582); η ~ N(0.35, 0.12²) | bg 0.02–0.07, rock garden 0.15–0.35 (field 0.20–0.40); boulders (D ≥ 0.25 m) S2 3–8 / S3 8–18 / S4 18–35 per 120 m; 21–27 /m² for D ≥ 1–1.5 cm | S2 0.05–0.12 / S3 0.08–0.20 / S4 0.12–0.30 | S2 2–5 / S3 5–12 / S4 10–20 per 120 m; heights 0.05–0.14 / 0.10–0.22 / 0.18–0.35; chainring 18–24 cm (S3), 25–38 cm (S4) | — (water-bar chord 0.4–0.8 m) |

**d) Roughness**

| | Form | w | Amplitude | Anisotropy |
|---|---|---|---|---|
| Audit | power law + discrete objects | 1.8–2.0 bg/scree; 2.4–3.0 slabs (1.4–3.0) | `amplitude_m` is a peak (~3σ); σ bg 1.7–3.3 mm (ISO B–C), scree 3–7 mm (C–D), fresh ×2–3 | isotropic field; anisotropy from discrete objects; ℓy/ℓx authored |
| ChatGPT | agrees: multiscale PSD/variogram | — | current amplitudes have no provenance | do not freeze ratio; fit directional variogram |
| Gemini | agrees: ISO 8608 / Matérn ν 0.5–0.8, band 0.02–5 m, cut-off 0.2 cyc/m | 2.0–2.5 | reads current values as σ; bg σ 6–15 mm (C–D), scree σ 12–28 mm (D–E); S3–S4 reach G–H | ℓy/ℓx 0.30–0.60 eroded, 0.75–1.0 scree/slab |

**e) Cross geometry**

| | Tread width | Tread cross-slope | Ruts | Drainage | Switchbacks |
|---|---|---|---|---|---|
| Audit | 0.4–0.9 m (lognormal ~0.6) | 3–8 % (max 10, elements 10–15); landform separate (25°+) | median 25–40 mm, tail 100–250+ | rename knick/drain_dip; knick 2.5–3 m, rolling dip 6–15 m; waterbar +5–20 cm raised; 75–92 % of 30 m windows without drain | 2–8 /km, spacing 130–300 m, landing r 1.5–3 m, ≤ 5 % |
| ChatGPT | 0.6–1.2 m (RMNP mean 0.89) | ~5 % outslope; host_slope separate | keep authored + separate `incision` (Poprad: >0.1 m on 44 %) | ≈ 0 for natural_alpine; built mode only | — |
| Gemini | 0.30–0.60 natural, 0.35–0.90 total; ledges 0.25–0.45 | 2–6 %; 10–18 % on scree traverses and dips; inslope −6…−9 % | S1/S2 20–60, S3/S4 50–130 mm; width 80–180 mm; length 2–12 m | bar 0.10–0.20 m, chord 0.4–0.8 m, 45–60°; rolling dip 1.8–3.0 m, depth 0.10–0.20; 2–5 per 120 m | built R 2.13–2.74 (USFS 1.83–3.66); alpine R 0.6–1.5; spacing 30–80 m |

**f) Friction**

| | Burckhardt verdict | rock | wet | wet smooth | dusty | scree / hardpack |
|---|---|---|---|---|---|---|
| Audit | confirmed (car tyre) | 0.90–1.10 / 0.70–0.90 | 0.55–0.75 / 0.50–0.65 | 0.25–0.40 / 0.15–0.30 | 0.40–0.55 / 0.35–0.45 | keep, flagged |
| ChatGPT | agrees | 0.8–1.2 / 0.7–1.1 (analogue, low confidence) | Persson: −20–30 %, small below ~1 m/s | — | — | scree = min(rubber, granular yield) |
| Gemini | agrees | 0.92–1.08 / 0.78–0.90; Stribeck 0.05–0.15 m/s | basalt 0.56–0.61, limestone 0.50–0.55, granite 0.42–0.52 (peak) | polished 0.35–0.50; lichen 0.25–0.35 / 0.20–0.28 | 0.90–1.05 → 0.45–0.60 | hardpack 0.75–0.85 / 0.58–0.68; scree 0.42–0.55 / 0.35–0.45, φ 34–38.7° |

**g) e-MTB reality**

| | Speed | Grade ceiling | Wheelie | Pulse / rear step | low_grip |
|---|---|---|---|---|---|
| Audit | 3–8 km/h | avg 25–30 %, burst ≤ 40 % | always dynamic; static ≥ 45–48 % | ±100 N (Klug 2019); rear-wheel step is trigger #1 | 20–28 % |
| ChatGPT | 3–8 km/h unvalidated | U3 26–30 % upper edge; U4 push | no lift-event data | — | — |
| Gemini | S2 7.2–12.6, S3 4.3–9.0, S4 2.9–6.1 km/h (authored) | bedrock 35–40, hardpack 26–32, scree 18–24 % | static 45–55 %, threshold "halves" with obstacles | rear-wheel step 10–20 cm; adds front-step pitch (+7–16 % grade) and ratcheting torque bursts | scree limit 18–24 % |

**h) Presets and OSM**

| | U2 | U3 | U4 | OSM |
|---|---|---|---|---|
| Audit | ramp .12–.20, wall .20–.25 | ramp .15–.22, wall .25–.30, steps 5–15 cm | ramp .18–.28, wall .30–.38, boulders 10–28 cm, steps 15–30 cm | 77,969 trails: mode 1 (23 %), ≥ 3 37 %, 4–5 21 % |
| ChatGPT | OSM bounds < 20 / < 25 % | < 25 / < 30 % | < 30 / < 40 % | separate obstacle_class axis |
| Gemini | ramp 14–20 (10–25 m), wall 21–25, width 0.6–0.9, R 2.0–3.5 | ramp 18–26, wall 26–32 (1–2), k 15–25 %, width 0.45–0.70, R 1.2–2.2 | ramp 22–32, wall 32–40 (2–3), k 25–40 %, width 0.30–0.55, R 0.7–1.4 | Uni Graz 2015: > 45,780 segments; e-MTB shifts one class up |

**i) Reference trails and DEM**

| | Trails | DEM notes |
|---|---|---|
| Audit | Scalettapass, Pas de Lona, Pico Veleta, Etna, Pic du Midi | swissALTI3D 0.5 m, BEV 1 m, IGN HD 0.5 m, Slovenia 1 m; stereo > 2000 m ±1–3 m; GPX barometer RMSE ~4 m |
| ChatGPT | refuses to name 3–5 meeting all criteria; Pinnell Mtn (negative benchmark), Slickrock/Amasa Back (non-alpine) | 0.5 m grid may be oversampled; LiDAR 0.3/0.5 m; swissSURFACE3D ≥ 15 pts/m², ±10 cm Z; RGE ALTI updates stopped 2024 |
| Gemini | no overlap: Gornergrat–Riffelberg, Barrhorn/Turtmanntal, Blauberg/Schwarzmilzferner (UAV SfM 1–3 cm), Sellrain, Eglwyseg; per-site numbers uncited | NMAD > 2 m on slopes > 45° (ESSD 2025); swissBEDROCK 10 m as rock/scree mask; Tiris 13 cm orthophoto; σ_grade ≈ 17.7 % at 2 m base; GPX ±3–10 m → false 2–6 m vertical; Savitzky–Golay 15–20 m |

**j) Items only in external reports**

ChatGPT: two-axis presets `uphill_class × obstacle_class`; per-field `provenance / confidence / calibration_state`; separate large-scale `incision` process; `local_surface_tilt`; `natural_alpine` vs `built_or_maintained` modes; scree traction as min(rubber, bed shear); sample 120 m windows from long real tracks; hierarchical P(T) factorisation; grades at 2/5/20 m windows with DEM-error propagation; scree-association study; bench-test matrix and instrumented ride log; generated-vs-measured statistics loop.

Gemini: ramp length conditioned on substrate; two scree types (traverse vs pocket) with own H_s; scree D50 15–35 / 60–90 mm; `rock_slab` primitive 0.8–2.0 m; superellipsoid ε1, ε2 0.3–0.8, b/a ~ U(0.6, 0.9); bimodal lateral rock placement ±U(0.06, 0.35) m + 20 % centre; asymmetric height distribution; Matérn ν; ISO class per STS level; diagonal water bars 45–60°; switchberm inslope; climbing turn vs switchback; hairpin exit as trigger (inside-radius grade ~2×); 35–80 % slope beyond tread edge as corridor limit; μ(P) = μ0(P/P0)^−β, β 0.10–0.18; dμ/dT ≈ −0.0022 /°C; glass transition at 0–5 °C; wet μ by rock type; lichen μ; pressures 0.6–1.4 bar; step heights from chainring/BB clearance (BB 335–345 mm); ratcheting pedalling as trigger.

**k) Contradictions > 2×**

1. >25 % share on T3/U3: 35–38 % vs Gemini 5–15 %.
2. Scree crossing length: median 34–64 m, p95 500–580 vs Gemini 3–10 / 10–30, max 75 m.
3. Counter-slope length: (6,16), median 12–16 vs Gemini 1.8–4.5 m.
4. Rolling dip length: 6–15 m vs Gemini 1.8–3.0 m (matches audit's knick).
5. Drainage frequency: ~0.3–1 per 120 m vs Gemini 2–5 vs ChatGPT ≈ 0.
6. Roughness σ: bg 1.7–3.3 mm vs Gemini 6–15 mm; scree 3–7 vs 12–28 mm (peak-vs-σ underneath).
7. Switchback spacing: 130–300 m vs Gemini 30–80 m.
8. Background n_scree: 0–1 per 120 m vs Gemini 2–4.
9. Rock-garden k: 0.08–0.16 vs Gemini 0.15–0.35.
10. Tread width upper bound: ChatGPT 1.2 m vs Gemini 0.6 m natural.
11. Borderline ~2×: hairpin radius 1.5–3 vs 0.6–1.5 m; rut tail 100–250+ vs ≤ 130 mm; rock_corr 0.15–0.45 vs 0.10–0.22 m.
12. Provenance only: H_s 3–10 cm "unmeasured" (audit, ChatGPT) vs "verified via WTA handbook" (Gemini) — needs a source check.