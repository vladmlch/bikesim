# Uphill Climb and Wheelspin Simulation Design Spec

**Date:** 2026-09-28  
**Topic:** Uphill climbing simulation on loose/hardpack ground with 12-speed adaptive cadence shifting and tyre wheelspin investigation.  
**Branch / Mode:** `ride` mode with `--tyre-model pneumatic`.

---

## 1. Problem Statement & Motivation

Existing ride-mode presets in `bike_sim` either run on flat ground (`flat`), traverse rolling obstacle tracks (`enduro_aggressive`, `road_*`), or descend through drops and kickers. All existing tracks have a net elevation change $\le 0$ (descending or level).

The goal of this feature is to investigate **traction limits and wheelspin (пробуксовка)** during an aggressive e-MTB climb:
1. When riding up a progressively steepening grade ($5\% \to 25\%$), load transfers rearward, increasing rear normal force $F_z$ but demanding proportionally higher tractive force.
2. In realistic e-MTB climbing, a rider naturally shifts down through a multi-speed cassette ($10\text{--}51\text{T}$) to keep cadence in a comfortable band ($65\text{--}85\text{ rpm}$).
3. On low gears ($45\text{T}, 51\text{T}$) with mid-drive assist (`turbo`, 340% support, 85 N·m motor ceiling), torque pulses on the pedal downstroke produce peak wheel forces that exceed the friction ceiling of `hardpack` ground ($\mu_{\text{peak}} = 0.80$).
4. This results in **cadence-synchronized, cyclical wheelspin** on steep slopes.

---

## 2. Physics & Kinematics Architecture

### 2.1 Track Geometry: `climb_steps`

* **Track name:** `climb_steps`
* **Surface:** `hardpack` ($\mu_{\text{peak}} = 0.80, \mu_{\text{slide}} = 0.60, C_\kappa/F_z = 12$)
* **Total length:** 115 m
* **Layout:**
  * $0\text{--}15\text{ m}$: Flat run-up ($0\%$, elevation $0\text{ m}$)
  * $15\text{--}18\text{ m}$: Smooth transition ($0\% \to 5\%$)
  * $18\text{--}33\text{ m}$: $5\%$ grade (rise $+0.75\text{ m}$)
  * $33\text{--}36\text{ m}$: Smooth transition ($5\% \to 10\%$)
  * $36\text{--}51\text{ m}$: $10\%$ grade (rise $+1.50\text{ m}$)
  * $51\text{--}54\text{ m}$: Smooth transition ($10\% \to 15\%$)
  * $54\text{--}69\text{ m}$: $15\%$ grade (rise $+2.25\text{ m}$)
  * $69\text{--}72\text{ m}$: Smooth transition ($15\% \to 20\%$)
  * $72\text{--}87\text{ m}$: $20\%$ grade (rise $+3.00\text{ m}$)
  * $87\text{--}90\text{ m}$: Smooth transition ($20\% \to 25\%$)
  * $90\text{--}105\text{ m}$: $25\%$ grade (rise $+3.75\text{ m}$)
  * $105\text{--}115\text{ m}$: Crest & flat runout ($+11.25\text{ m}$ elevation)

Smooth transitions use cubic blending across 3-meter zones so curvature is continuous, preventing artificial contact normal shocks.

### 2.2 Heightfield Dynamic Vertical Sizing

The default heightfield geometry has fixed vertical limits:
* `datum_z_m = 2.8 m`, `elevation_m = 3.4 m`, envelope $[-2.8, +0.6]\text{ m}$.
* A $+11.25\text{ m}$ climb requires extending `HeightFieldSpec.for_track`:
  * If `extent[1] > FIELD.max_profile_m` or `extent[0] < FIELD.min_profile_m`:
    * `datum_z_m` is sized to accommodate minimum profile plus margin.
    * `elevation_m` is sized to accommodate total vertical span plus margin.
    * `catch_plane` height is positioned safely below the new field floor.
  * For all existing presets fitting $[-2.8, +0.6]\text{ m}$, the default field remains bit-identical with the golden baseline.

### 2.3 12-Speed Cassette and Adaptive Auto-Shifter

* **Chainring:** 32T
* **Cassette (12-speed):** `(10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51)` teeth.
* **Starting Gear:** Cog 16T (Gear 4, ratio $32/16 = 2.00$).
* **Shifter Logic:**
  * Evaluated every step in `PedalDrivetrain`.
  * If $\text{cadence} < 65\text{ rpm}$ and positive drive torque is demanded, and current cog $< 51\text{T}$:
    Trigger downshift to next larger cog.
  * If $\text{cadence} > 85\text{ rpm}$ and current cog $> 10\text{T}$:
    Trigger upshift to next smaller cog.
  * **Shift Torque Cut:** During shifting ($\sim 200\text{ ms}$), pedal/assist torque is attenuated by $70\%$ to model the chain derailment window.
  * **Dynamic Chain Equality Re-datuming:**
    * When gear changes from $R_1$ to $R_2$, MuJoCo's `model.eq_data[chain_eq_id, 1] = 1.0 / R_2`.
    * `_redatum(model, data)` is called immediately so position residual is zero.
    * Crank angular velocity is updated to match wheel velocity through the new ratio:
      $\omega_{\text{crank}} = \omega_{\text{wheel}} / R_2$.
  * **Override:** `--gearing` flag forces a single fixed gear and disables the auto-shifter if requested.

### 2.4 Wheelspin Mechanics (`pneumatic` tyre model)

In `bike_sim.sim.ride.tyre`:
* Tangential brush force:
  $F_t = \text{brush}(\kappa', N_p, \mu(V_s))$
* Wheelspin detection:
  When a contact patch is fully sliding ($|\theta \sigma_x| \ge 1$) and slip ratio $\kappa > 0$, the rear wheel spins faster than ground speed:
  $V_{\text{tread}} = \omega \cdot R_e > V_x$.
* Telemetry records:
  * `rear_slip_ratio` ($\kappa$)
  * `rear_wheelspin_time_s`
  * `rear_dissipated_power_w`
  * `gear_teeth` and `cadence_rpm`

---

## 3. Tooling and Visualization

`tools/analyze_wheelspin.py` will run a headless simulation on `climb_steps` and generate:
1. `output/wheelspin/climb_kinematics.png`:
   * Panel 1: Chassis speed $V_x$ vs rear wheel linear speed $\omega \cdot R_e$ (showing wheelspin bursts).
   * Panel 2: Road profile elevation and grade percentage.
   * Panel 3: Active gear (teeth) and cadence (RPM).
   * Panel 4: Longitudinal slip ratio $\kappa(t)$ with sliding threshold.
   * Panel 5: Drive torque vs maximum traction limit $F_{\text{traction}} = \mu \cdot F_z$.
2. `output/wheelspin/summary.json`:
   * Wheelspin duration per slope step ($5\%, 10\%, 15\%, 20\%, 25\%$).
   * Critical gradient of first wheelspin onset.
   * Total energy dissipated in wheel slip.
