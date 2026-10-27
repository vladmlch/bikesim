# Spatial tyre: tangential law and energy accounting

Status: W23–W24 source and analytic tests written; numerical validation pending
V04. Coefficients in the fixture are synthetic and do not represent measured
MTB tyre data.

## Spring convention and combined grip

The stored displacement `u` is spring extension, the contact material velocity
relative to the fixed road is `v`, and the force on the wheel is `F = -K u`.
With convective decay rate `a = |v_roll|/ell` (`v_roll` is the hub velocity
projected on the patch longitudinal tangent; `ell_x`/`ell_y` are the two
relaxation lengths, so `a_x` and `a_y` differ),

```
du/dt = v - a u
u_trial = (u_old + dt v)/(1 + dt a).
```

The W23 prose uses both `du/dt=-v-a*u` and `F=-K*u`. That pair makes a fresh
tyre accelerate the slip that loads it. The source retains a work-conjugate,
passive sign convention, and the discrepancy is explicit in the ledger.

For each tangent direction `K_x = kx_n_per_m3 * A` and `K_y = ky_n_per_m3 * A`
with `A` the patch (fast) or cell (detailed) area. The return map scales the
trial displacement radially until

```
(Fx/(mu_x Fz))² + (Fy/(mu_y Fz))² <= 1.
```

with `Fx = -K_x u_x`, `Fy = -K_y u_y` and `mu_i` the road coefficient times
`mu_i_scale`. A zero coefficient independently removes that force component. A
zero normal load clears the force. Camber contributes through the same spring
and shares this ellipse; it never adds force outside the combined grip budget.
A trial value, stiffness, denominator or grip capacity that overflows is
`invalid_geometry`, never a silent zero-force contact.

For endpoint force work, the discrete identity is

```
-F_new · v dt = U_new - U_old + D_sliding + D_convective + D_algorithmic
D_sliding = Σ K u_new (u_trial-u_new)
D_convective = dt Σ a K u_new u_trial
D_algorithmic = 1/2 Σ K (u_new-u_old)².
```

Every loss is nonnegative under the radial return map. The two latter terms
are exposed together through the existing `convective_release_j` accumulator;
the frozen state has no independent numerical-loss field. They are not
claimed to be separately measured physical hysteresis. Tests compare work to
the change of elastic storage plus all three terms during stop/start/reverse.

## Camber and trail

The resolved kernel receives axle, spin and camber from wheel kinematics.
The camber source is

```
b_gamma = r_eff * C_gamma * Fz * sin(gamma) / (K_y * ell_y)    (0 when K_y = 0)
v_y,eff = v_y + b_gamma * omega_spin
```

`C_gamma` is `camber_coefficient_per_rad`, normalized by normal load, so
`b_gamma` has units of length and `b_gamma * omega_spin` is a velocity. The
`Fz` factor makes the source invariant when a patch is split into equal
area/load elements (`K_y` and `Fz` both halve). `gamma` is
`asin(axle . patch_normal)` and `omega_spin` is the absolute wheel angular
velocity projected on the axle, conjugate to the wheel-body couple. The
conjugate wheel couple is `b_gamma * Fy * axle`. Its power accompanies the
modified lateral slip in the energy identity.

Pneumatic trail (fast tier only) shifts the **tangential** application point
along the patch longitudinal tangent by `pneumatic_trail_m` with the sign of
the hub longitudinal velocity (zero offset at standstill). The actual velocity
at the shifted point includes `omega × offset`. A compensating couple
`-offset × F_normal` leaves the distributed normal load at its original
centroid. The reported `mz_nm` is the tangential lever-arm moment, not another
couple to add to the engine. Thus the writer applies one lever arm and one
intrinsic couple only. The detailed tier applies no trail offset and no
convective decay (`a = 0`): each cell already moves with the material, so its
own position and residence time carry both effects.

The frozen standalone `update_fast_brush` signature lacks axle, spin, angular
velocity and a material resolver. It is a unit-road synthetic kernel. The
production owning model calls the private resolved per-patch seam with the
actual surface friction coefficient and complete kinematics. Actual material
IDs are never treated as numerical friction coefficients.

## Material-resolved bristles

The detailed tier stores two displacement components and elastic energy per
full-circle material cell. Geometry provides the current material association;
the same signed spin is not applied again as a second array rotation.
Each cell uses its local normal load, actual surface ID and resolved Stribeck
coefficient `mu = mu_slide + (mu_peak - mu_slide) exp(-(v/v_s)²)`, with `v` the
local tangential slip speed. A nonpositive `v_s` is the zero-width limit: peak
at exactly zero slip speed, sliding value otherwise. Local return mapping
occurs before deterministic force reduction. Stored energy and losses are
reduced in material-cell order, so the result does not depend on the fan
enumeration; per-element wrenches keep the caller's order.
Leaving contact releases stored bristle energy once and clears its displacement.

The resolved overload rejects an unresolvable surface instead of assuming a
grip coefficient. The standalone wrapper again represents a synthetic
unit-friction road. A rigid rotation of the whole fixture rotates the force
without altering elastic energy.

There are two unresolved frozen-state limitations: no previous tangent basis
is stored, and no fractional material-coordinate/deposition state exists.
The current components therefore use a co-rotating tangent convention and
quantized cell association. This is insufficient to claim the general
`B_new^T B_old` transport or subcell-advection refinement criteria. Those are
explicit C10 work for V04, not hidden passing qualifications.

## Qualification rows

| Observable | Fast | Detailed | Acceptance / evidence |
|---|---|---|---|
| Combined ellipse | Written synthetic and resolved kernel | Per-material-cell return map | `<= 1+1e-12`; pending V04 |
| Probe state | No committed mutation | No committed mutation | bitwise comparison; pending V04 |
| Endpoint spring work | Implicit decay plus return mapping | Material spring plus return mapping | `1e-6*scale+1e-9 J`; pending V04 |
| Rigid frame rotation | Named world tangent inputs | Material displacement components | energy `1e-12 J`, force `1e-9 N`; pending V04 |
| General changing-frame transport | Previous basis absent | Previous basis absent | C10 required |
| Subcell material advection | Phase supplied by owner | Fractional deposition absent | C10 required |
| Steady fast/detailed curves | No measured values | No measured values | `<=10%`, pending V04 |

No curve comparison, physical calibration, force error, timing or convergence
result has been fabricated by the source-authoring phase.

The owning fast model reconstructs each previous patch area from its committed
`element_to_patch_slot` assignment and the immutable finite-volume grid areas.
It can therefore recover `K = k_area * area` without another serialized field.
A vanished slot releases its whole previous elastic energy once. A shrinking
survivor releases the energy of the removed area; a growing survivor scales
both displacement components by `sqrt(old_area/new_area)` so that the geometric
transfer creates no stored energy. These operations precede the new interval's
return map and enter `convective_release_j`. Their tests remain pending V04;
the previous-basis and fractional-material-transport limitations above still
apply.

## Bounded workspace ownership

The frozen standalone patch and bristle signatures expose no caller workspace.
Their temporary connectivity, overlap and coefficient arrays exceed the native
8 KiB frame policy for the detailed tier. They use `constinit thread_local`
bounded scratch records instead of automatic arrays or warm-path allocation.
The bristle record maps each material cell to its source element (a sentinel
marks an absent cell) and holds one resolved friction lane per element, both
sized at compile time. Every lane the call can read is reset at entry; a
per-thread reentry guard rejects nested use. The kernel runs only on the owner
thread, so worker-thread count cannot change the reduction. No tyre history,
contact identity, force result or accepted interval lives in that scratch.
Histories still belong entirely to the explicit state and its codec. An
owner-supplied workspace is the preferred future C10 signature; these source
changes do not claim an allocation or real-time measurement.
