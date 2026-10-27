# Spatial tyre: normal finite-volume law

Status: derivation and source written for W20–W21; numerical validation pending V04.
No simulation, test, compiler, build or measurement was executed for this change.

## Geometry and work-conjugate indentation

The tyre is a torus with major radius `R = outer_radius - casing_width/2`
and minor radius `r = casing_width/2`. The stationary contact fan uses the
world downward direction projected into the wheel plane. Wheel spin changes
the material-cell association; it does not rotate this fan.

Each cell uses its midpoint for geometry and its **integrated** area

```
A_i = r * Δθ * (R * Δφ + r * (sin φ_hi - sin φ_lo)).
```

This retains positive cell area and exactly conserves the analytic fan area.
The W20 prose instead specifies midpoint area and an analytic-area tolerance
of `1e-10`; those two requirements conflict at seven lateral nodes. The exact
cell integration is the explicit resolution, recorded in the contract ledger.

The outward casing normal points toward the road at a lower contact. The ray
starts at `p - clearance*n_out`, and proceeds along `n_out`. Reversing these
signs starts underneath the road. With ray indentation `δ_ray`, the normal
indentation is `δ_n = δ_ray * max(0, -n_out·n_road)`. For a locally fixed planar
surface, `-∂δ_n/∂z = n_road.z`; this makes the normal load conjugate to rigid
displacement. Using radial indentation directly with an unprojected normal
force does not satisfy that virtual-work identity away from the fan centre.

## Elastic and pressure potentials

For positive normal indentation `δ_i` and cell area `A_i`,

```
V_d = Σ A_i δ_i
U_c = 1/2 Σ k_area A_i δ_i²
U_r = 1/2 Σ k_rim A_i max(δ_i - clearance_rim, 0)²
```

The pressure potential is `U_p = p V_d` for the constant-pressure fixture.
For an isothermal closed volume,

```
p_eff = p0 V0 / (V0 - V_d)
U_p = -p0 V0 log(1 - V_d/V0).
```

The closed-volume domain requires `V_d < V0`; reaching or crossing zero
remaining volume is invalid geometry, not a pressure clamp. The pressure,
carcass, and rim force densities are the respective derivatives of these
potentials. Normal tensile loads are excluded by the unilateral contact law.
Inactive or invalid elements produce zero load and retain their diagnostic
reason at the geometry layer.

`NormalElement3d::stored_energy_j` contains carcass, rim and Maxwell elastic
energy. The low-level kernel reports their sum. The owning model adds the
reversible pressure potential to its staged `CarcassState3d::stored_energy_j`,
so the model aggregate includes all four contributions. The frozen state has
no distinct pressure-work channel; those contributions cannot be separately
restored or reported without a C10 extension. The independent potential
fixture also includes pressure explicitly.

## Maxwell branch

Let `q` be branch stress, `k_m` branch stiffness per area, `τ` its relaxation
time, `x = dt/τ`, and let the normal indentation rate
`d = (δ - δ_prev)/dt` be constant during the interval. `δ_prev` is the
previous accepted penetration of the same material cell; a cell that was not
in contact has `δ_prev = 0`, so first contact sees `d = δ/dt`.

```
q_ss = k_m τ d
q(t+dt) = q0 exp(-x) + q_ss (1 - exp(-x))
U_m = A q²/(2 k_m)
D_m = A ∫ q(t)²/(k_m τ) dt >= 0.
```

The loss integral is evaluated in closed form with `a = (1-e^{-2x})/2`,
`b = (1-e^{-x})²` and `c = x - (1-e^{-x}) - (1-e^{-x})²/2`:

```
D_m / A = (q0² a + q0 q_ss b + q_ss² c) / k_m          (x >= 0.01)
```

The three `q_ss` terms of that form cancel to `O(x³)` when `dt << τ`, so for
an ordinary small step the subtraction would erase (or invent) the loss. For
`x < 0.01` the implementation therefore integrates in `q0` and the elastic
strain increment `Δ = k_m d dt` (so `q_ss = Δ/x`):

```
q       = q0 e^{-x} + Δ f,            f = (1-e^{-x})/x
D_m / A = (x/k_m) (q0² a/x + q0 Δ f² + Δ² c_s(x))
c_s(x)  = [x - 2(1-e^{-x}) + (1-e^{-2x})/2]/x³
        = 1/3 - x/4 + 7x²/60 - x³/24 + 31x⁴/2520 - x⁵/320 + 127x⁶/181440 - 17x⁷/120960 + O(x⁸)
```

with the truncated series error below `3e-21` on the branch. The stress update
uses `q0 e + q_ss (1-e)`, not `q_ss + (q0-q_ss) e`: after complete relaxation
`e` rounds to zero and the second form would subtract the small steady stress
away. `expm1` supplies `1-e^{-x}` and `1-e^{-2x}` without cancellation. A
nonfinite loss is reported as invalid geometry before any state changes; it is
never clamped to zero. The stress, elastic force, pressure force and rim force
are exposed separately. The normal force used by the writer is the nonnegative
sum.

## Ownership, probes and material history

Input dimensions, indices and coefficient domains are validated before state
mutation. `advance=false` and `dt=0` never alter committed state. The owner
stages both wheels before committing an accepted interval.

The material arrays are indexed by full-circle material-cell IDs carried by
the geometry output. Consequently the core does not rotate these arrays a
second time. The owning model records exact wheel phase. The frozen carcass
function has no grid dimensions or continuous phase input, so fractional-cell
interpolation is not representable at this seam; quantized membership is not
evidence of the W21 material-refinement acceptance. Old material-index mapping
and fractional deposition require the C10 resolution listed in the handoff.

The owning model also accounts for Maxwell memory outside the current fan.
The grid cell area is independent of its longitudinal index; material-cell ID
modulo `n_phi` therefore recovers the correct previous area. For an inactive
cell with incoming stress `q0` and decayed stress `q1`, the owner adds
`A*q1²/(2*k_m)` to storage and `A*(q0²-q1²)/(2*k_m)` to relaxation loss.
Unloaded memory is retained and decays without disappearing from the ledger.
The standalone kernel has no grid argument and its element energy output
continues to describe the current contact quadrature; only the owning model
has the complete material energy inventory.

This reconstruction closes unloaded Maxwell relaxation, but does not prove
work closure when a geometric contact enters or exits within one accepted
interval. Pressure and elastic contact potential change with that topology;
the frozen endpoint force law has no sub-interval contact event or explicit
normal numerical-loss lane. V04 must measure this gap instead of labelling
all disappearance of reversible storage as physical heat. The general
moving-topology force/work and material-refinement acceptances remain open.

## Verification handoff

The written test covers the `10 N / 0.005 J` synthetic cell, constant and
isothermal pressure derivatives (`1e-5` relative), exact constant-rate Maxwell
evolution, halved intervals, invalid indices, inactive contacts and bitwise
probe preservation. Geometry tests cover analytic area/volume, local frames,
yaw invariance and named failure statuses.

V04 still owns numerical execution, refinements, force–potential tests on a
moving complete contact topology, and the `2%` old-planar compatibility matrix
at `{100,300,600,900} N × {1,1.5,2} bar`. The old planar law integrates a casing
chord with independently configured area/stiffness coefficients; matching
these laws is a qualification/calibration question, not a reason to change
the untouched planar implementation or silently fit the synthetic fixture.

| Acceptance boundary | Source state | Qualification gate |
|---|---|---|
| Fixed-topology pressure/elastic derivative | Independent potential fixture written | pending V04 |
| Constant-rate Maxwell primitive | Exact analytic update and nonnegative integral | pending V04 |
| Inactive material Maxwell relaxation | Owning model reconstructs cell area and complete energy | pending V04 |
| Contact entry/exit within an interval | No sub-interval event/work inventory in W00 | open C10; no passivity claim |
| Fractional material advection | State lacks fractional deposition/history basis | open C10; no refinement claim |
