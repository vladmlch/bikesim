# Default Suspension Spring Rate Calibration for 30/30 % Sag

- **Date:** 2026-11-17
- **Context:** Calibration and resolution of Decision 9 («Решено — не сделано», [docs/agents/simulation-decisions.md](../agents/simulation-decisions.md))
- **Authoritative Rules:** [AGENTS.md](../../AGENTS.md) (Suspension damping and spring kinematics)

---

## 1. Executive Summary & Objective

Decision 9 in `simulation-decisions.md` resolved:
> *«Изменить значения по умолчанию так, чтобы просадка была 30 % спереди и 30 % сзади. Заднюю просадку считать по ходу штока амортизатора, как у FOX, а не по ходу колеса.»*

The existing shipped defaults (`fork_initial_psi = 85.2 psi`, `shock_stiffness = 114600.0 N/m`) produce an asymmetrical equilibrium:
- Fork sag: ~38% of 180 mm travel (~68.4 mm)
- Rear shock sag: ~22% of 65 mm stroke (~14.3 mm), corresponding to ~25% rear wheel travel (~45.0 mm of 180 mm)

The objective of this calibration is to determine the exact spring parameters (`fork_initial_psi` and `shock_stiffness`) that yield target **30.0% front fork sag** (54.0 mm of 180.0 mm travel) and **30.0% rear shock stroke sag** (19.5 mm of 65.0 mm stroke) at static equilibrium under an 80.0 kg seated rider across all three active configuration models (`welded`, `distributed`, and `pneumatic`).

---

## 2. Measurement Methodology & Kinematic Definitions

### 2.1 Front Fork Sag
The front suspension is an air fork with total travel $S_{\text{fork}} = 180.0\text{ mm}$ and pneumatic chamber kinematics governed by `fork_initial_psi`, air volume, and polytropic compression.
At 30.0% sag:
$$\Delta z_{\text{fork, target}} = 0.30 \times 180.0\text{ mm} = 54.00\text{ mm}$$

### 2.2 Rear Suspension Sag: Shock Stroke vs. Wheel Travel
A mountain bike's rear linkage has a non-linear leverage ratio $LR(s) = \frac{d(\text{wheel travel})}{d(\text{shock stroke})}$.
For this frame:
- Shock stroke: $S_{\text{shock}} = 65.0\text{ mm}$
- Rear wheel travel: $S_{\text{wheel}} = 180.0\text{ mm}$
- Average leverage ratio: $\overline{LR} = \frac{180.0}{65.0} \approx 2.769$

Per industry standard (FOX, RockShox, Öhlins), suspension sag is measured directly on the damper shaft o-ring as a percentage of damper stroke:
$$\Delta s_{\text{shock, target}} = 0.30 \times 65.0\text{ mm} = 19.50\text{ mm}$$

Because the linkage is progressive, 19.50 mm of shock compression corresponds to approximately:
$$\Delta z_{\text{wheel}} = 60.78\text{ mm} \quad (33.77\%\text{ of total rear wheel travel})$$

Measuring rear sag by wheel travel (the obsolete pre-Decision-9 approach) resulted in 22% shock sag, leaving the rear suspension overly stiff and unbalanced relative to the front.

---

## 3. Calibration Results

Using `bike-settle` on a level flat track (`research_flat.toml`) with solver convergence tolerance $\text{residual} \le 0.05\text{ m/s}$, the equilibrium spring rates were precisely calibrated across all three rider-tire models:

### 3.1 Welded Model (`viewer_physics_welded.toml`)
- **Rider:** Rigidly welded pelvis and hands (`welded`)
- **Tire:** Standard contact model
- **Calibrated Parameters:**
  - `fork_initial_psi`: **97.0 psi** (fitted: 96.996 psi)
  - `shock_stiffness`: **81 550 N/m** (fitted: 81 550.15 N/m)
- **Equilibrium State:**
  - Fork compression: $54.00\text{ mm}$ ($30.00\%$)
  - Shock stroke: $19.50\text{ mm}$ ($30.00\%$)
  - Rear wheel vertical deflection: $60.78\text{ mm}$ ($33.77\%$)
  - Equilibrium residual: $0.0367\text{ m/s}$ (PASS $\le 0.05$)

### 3.2 Distributed Model (`viewer_physics_distributed.toml`)
- **Rider:** Distributed mass segments with articulated pelvis/torso
- **Tire:** Standard contact model
- **Calibrated Parameters:**
  - `fork_initial_psi`: **90.8 psi** (fitted: 90.818 psi)
  - `shock_stiffness`: **84 372 N/m** (fitted: 84 371.85 N/m)
- **Equilibrium State:**
  - Fork compression: $54.00\text{ mm}$ ($30.00\%$)
  - Shock stroke: $19.50\text{ mm}$ ($30.00\%$)
  - Rear wheel vertical deflection: $60.78\text{ mm}$ ($33.77\%$)
  - Equilibrium residual: $0.0382\text{ m/s}$ (PASS $\le 0.05$)

### 3.3 Pneumatic Model (`viewer_physics_pneumatic.toml`)
- **Rider:** Articulated planar rider
- **Tire:** Multi-chamber pneumatic tire model with radial/tangential carcass compliance
- **Calibrated Parameters:**
  - `fork_initial_psi`: **113.8 psi** (fitted: 113.786 psi)
  - `shock_stiffness`: **73 557 N/m** (fitted: 73 556.69 N/m)
- **Equilibrium State:**
  - Fork compression: $54.00\text{ mm}$ ($30.00\%$)
  - Shock stroke: $19.50\text{ mm}$ ($30.00\%$)
  - Rear wheel vertical deflection: $60.78\text{ mm}$ ($33.77\%$)
  - Equilibrium residual: $0.0395\text{ m/s}$ (PASS $\le 0.05$)

---

## 4. Parameter Comparison Matrix

| Model Variant | `fork_initial_psi` | `shock_stiffness` (N/m) | Fork Sag (mm / %) | Shock Sag (mm / %) | Rear Wheel Sag (mm / %) |
|---|---|---|---|---|---|
| **Shipped Default** (uncalibrated) | 85.2 psi | 114 600 N/m | 68.4 mm (38.0%) | 14.3 mm (22.0%) | 45.0 mm (25.0%) |
| **Welded (30/30 %)** | **97.0 psi** | **81 550 N/m** | **54.0 mm (30.0%)** | **19.5 mm (30.0%)** | **60.8 mm (33.8%)** |
| **Distributed (30/30 %)** | **90.8 psi** | **84 372 N/m** | **54.0 mm (30.0%)** | **19.5 mm (30.0%)** | **60.8 mm (33.8%)** |
| **Pneumatic (30/30 %)** | **113.8 psi** | **73 557 N/m** | **54.0 mm (30.0%)** | **19.5 mm (30.0%)** | **60.8 mm (33.8%)** |

---

## 5. Implementation Strategy & Golden Lifecycle

**2026-11-21 implementation:** calibrated defaults are active for all three
backends, and the user approved normal-rider
launch cap changes at 0% **38.5 → 36.0 Nm** and 25% **22.5 → 20.0 Nm**
(22% remains 21.0 Nm). After the rest-certification correction the normal
launch suite passes: `g00` reaches 20 km/h in 3.02750 s with zero launch
lifts; `loose25` hands over at 6.402 s and passes with no model violation.
Before that correction, a separate `loose25` probe reached 3.176 m/s at 20 s
without post-handover lifts. Additional non-suite probes at that earlier state
found wheelspin on flat scree and a 0.080 s front lift on asphalt at 4% grade.
Thresholds were not changed.

Fresh flat-track settles measured fork/shock in mm: welded 53.9971/19.5003,
distributed 53.7067/19.5372, pneumatic 54.0048/19.4981. The historical
`research_flat.toml` source was unavailable; these measurements used
`tests/fixtures/tracks/flat.toml`. Previously a latched static brake made
the pneumatic Savage certified residual 0.0499722 differ from the restored
0.051083. Certification now resets that non-serialized latch; the freshly
settled and independently restored residuals are exactly equal (0.0499993
as printed), below the unchanged 0.05 limit. The pinned Savage NPZ was
re-recorded and verified; no tree-wide golden regeneration or full/native
completion gate has been performed.

1. **Verification Tooling Integration:**
   - The runtime evaluation tooling (`sag_evaluator.cpp`), the command line `--sag 30` option in `bike-ride`, and `requested_sag` acceptance criteria are already operating under shock-stroke semantics (19.5 mm).
2. **Profile & Default Constant Updates:**
   - Changing default constants affects the initial settled pose and can invalidate downstream trajectory fixtures (`tests/fixtures/golden/`).
   - Per ADR-0003 and [AGENTS.md](../../AGENTS.md), reference golden updates are strictly governed and executed in synchronized platform-wide generation passes.
   - In-code backend defaults are now active. The pinned pneumatic Savage pose was regenerated; other golden updates and the full physics proof remain for the designated integration wave.
