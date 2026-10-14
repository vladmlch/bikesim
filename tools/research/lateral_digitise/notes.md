# lateral_digitise — WP-V3 working notes

Digitisation of Dressel & Sadauckas 2020, *Applied Sciences* 10(9):3156,
DOI 10.3390/app10093156 («Characterization and Modelling of Various Sized
Mountain Bike Tires and the Effects of Tire Tread Knobs and Inflation
Pressure»), UWM treadmill rig, Fz = 418 N.

## Sources used (figs/)

| file | origin |
|---|---|
| `dressel2020.pdf` | `applsci-10-03156-v2.pdf`, mdpi-res.com, 22 pp. (the copy in `.scratch/tire-research/F/papers/` is a 412-byte Akamai «Access Denied» stub — re-downloaded) |
| `p18_img498.png` / `p18_img499.png` | embedded rasters of Fig. 17, extracted at native res 1725×1022 / 1725×511 (PDF page 19) |
| `p05_img174.png` | embedded raster of Fig. 3, 1725×1016 (PDF page 6) |
| `p11_img290.png` | embedded raster of Fig. 8 bar charts, 1352×410 (PDF page 12) |
| `applsci10_03156_g0NN.png` | publisher full-res figures (mdpi-res.com), used as the independent second raster |
| `fig17c_stitched.png` | panel c spans the img498/img499 cut; stitched at x[194,801] |

## Figure geometry (embedded raster, pixels)

* Fig. 17a box: x[189.5, 800.5] → 0–60 psi, y label centres
  {628, 504, 386, 265, 142, 21} → {0, 5, 10, 15, 20, 25} 1/rad (OLS fit).
* Fig. 17b box: x[997.5, 1604.5] → 0–60 psi, label centres
  {628, 526, 424, 324, 221, 119, 17} → {0, 0.5, …, 3.0} 1/rad.
* Fig. 17c (stitched): labels {4, 66, 132, 200, 268, 336, 404, 470, 538,
  606} → {0.00, −0.10, …, −0.90} m.
* Fig. 3a: box x[202, 824] → −5…+5 deg slip, y {19…620, 11 labels} →
  +1.0…−1.0 step 0.2 normalised force.
* Fig. 3b: box x[1024, 1648] → −25…+25 deg camber, same y scale.

Publisher raster `g017` (3457×3399) panel boxes: a x[371,1592]
y[213,1431]; b x[1989,3201] y[213,1431]; same axis limits (verified by
label-block centres).

## Legend key (read from the legend column, x≈1180–1290 px in img498/499)

Colour = tyre family, marker = construction variant:

| entry | colour | marker |
|---|---|---|
| 29×2.3″ knobby, 25 mm | blue | filled square |
| 29×2.5″ file-tread, 25 mm | red | filled triangle |
| 29×2.5″ BALD, 25 mm | red | open square |
| 29×2.3″ BALD, 25 mm | blue | open square |
| 29×2.3″ knobby, 22 mm | cyan | filled square |
| 29×2.5″ file-tread, 22 mm | magenta | filled square |
| 27.5×2.8″ knobby, 38 mm | green | filled square |
| 29×3.0″ knobby, 45 mm | orange | filled square |
| 26×4.0″ knobby, 86 mm | brown | filled square |

Same-colour series are split by connected components; they differ enough
in value (e.g. knobby ≈ 9–13 vs bald ≈ 17–22 in panel a) that component
labelling alone separates them. Orange/brown overlap for the 26×4″ fat
tyre at ~21–22 1/rad is merged into `knobby_26x4.0_86mm`.

## Authors' printed coefficients (Fig. 3, 29×2.3″ knobby @25 psi)

* BCDs = **10.488** 1/rad (slip / cornering)
* BCDc = **0.909** 1/rad (camber)
* also visible: Cs = 0.96, Cc = 0.34, D = 1.295, Es = 0.19, Ec = −1.04
* Fig. 8a printed bars (static lateral stiffness, N/m):
  27 710 (29×2.3″ @25 psi), 30 859 (27.5×2.8″ @20 psi),
  36 181 (29×3.0″ @20 psi), 58 376 (26×4.0″ @15 psi)

These numbers are *printed on the figures by the authors* — they anchor
the digitisation absolutely (no pixel error possible at 25 psi).

## Pipeline

`digitise.py` — two passes (A: embedded raster, label-centre calibration,
per-column median; B: publisher raster, spine calibration, per-column
mean). Output: `points/pass{A,B}_17{a,b}_<colour>_c<i>.csv`.

`analyse.py` — merges components into named series, prints A-vs-B diff
table and exponents. `finalise.py` — writes
`points/final_17{a,b}_<series>.csv` (psi, bar, passA, passB, mean,
half-spread) + `final_summary.json`, prints derived quantities.

## Known extraction artefacts (secondary series only)

* 29×3.0″ cornering @15 psi: pass A picks up neighbouring 26×4″ marker
  pixels → 35 % local diff; pass B is clean (14.6).
* 29×2.5″ BALD camber @15 psi: red marker split → 21 % local diff.
* knobby camber @20 psi sits at a component-join: A 1.03 / B 1.09 (5.9 %).
Target series (knobby 25 mm) is ≤ 2.5 % everywhere else.

## Extra artefacts added after the main pass

* `points/passA_17c_blue_c*.csv` — Fig. 17c stitched-panel extraction:
  two blue series = 29×2.3″ bald (large |Mz| stiffness, −0.36…−0.82 m)
  and knobby (−0.12…−0.29 m); printed BCDsa = −0.194 confirms the
  knobby attribution.
* `points/dellorto_nF_check.json` — independent load-exponent recomputation
  from Dell'Orto VSD-2024 Tables 2–6 (camber 0° rows): median n_F = −0.38,
  band −0.55…−0.13, one anomalous +0.19 (T04 @500 kPa).
