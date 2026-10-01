# Welded pedal attachment for articulated_planar — design

## Context

In `physics_mode="physical"` with `--rider articulated_planar` the feet are NOT welded:
the foot–pedal interface is a manual unilateral pad model (`RiderContactApplier`,
two compliant pads per sole against `geom_pedal_*`, brush friction, finite
~5 cm footprint) plus `PedalRecovery` for feet that end up below the platform.
On rough tracks this loses contact: pads can only push, friction vanishes when
the foot is unweighted, and the crank can be accelerated by the mid-drive motor,
the other leg, or impulsive freehub engagement faster than the bounded leg PD
can track. (The `weld_foot_*` equalities exist only in the non-physical seated
path, `builder.py` `if crank_joint and not physical`.)

Goal: an opt-in profile where feet are rigidly attached to pedals ("педали —
штыри, ноги — палки"), removing pedal contact physics, while keeping:

- coasting: rider stays seated, legs keep standing on the pedals and carry their
  weight share while the crank decelerates;
- pedaling: when cadence spikes, the lagging foot is dragged by the weld — it
  can never detach (crank↔wheel freewheel unchanged).

## Settled decisions (interview)

| Decision | Outcome |
|---|---|
| Attachment | MuJoCo `weld` equality `rider_foot_{side}` ↔ `pedal_{side}` body. Pedal bodies and `pedal_*_spin` hinges stay (mass/inertia budget unchanged); the platform becomes slaved to the foot (clipless semantics — ankle sets sole/platform pitch). |
| Guarantee | Unbreakable weld. `PedalRecovery` becomes inert in weld mode (stage stays `none`). |
| Force requests | Push-only: `pedaling_force_requests`/`feasible_pedal_force` friction-cone projection unchanged. Pull-up force exists physically as weld constraint reaction but is never commanded; to move pedals backward the rear leg presses down. |
| Coasting | Unchanged: `PedalingPolicy` 0.35 s decel ramp realized by pressing the ascending pedal down (`coasting_torque`). Seated posture and saddle/pedals/grip weight split unchanged (no standing-up transition). |
| Config | `[articulated] pedal_attachment = "flat" \| "weld"`, default `"flat"`; valid whenever `articulated_planar` + `physical`, independent of `drive_mode`. TOML→dataclass mapping is automatic (`resolution.py`). |
| Torque sensor | `delivered_crank_torque_nm` keeps its name and semantics ("torque applied by feet to crank"), but in weld mode it is read from the weld equality reaction (`efc_type==mjCNSTR_EQUALITY`, `efc_id==weld id`) mapped via `mj_mulJacTVec` onto `crank_spin` — same pattern as `shock_joint_limit_qfrc`. |
| Weld stiffness | `solref` at the stability floor: `max(2*timestep_s, closure_time_constant_s)` ≈ 0.0025 s for the fast viewer profile. `solref` is not damping — it is the residual-correction timescale; this is as rigid as a MuJoCo constraint can be. |
| Verification | Headless run on `rough_uphill` + telemetry asserts; plus manual viewer inspection. |

## What changes per component

**Build (`builder.py` / `physical_topology.py`)**
- New `ArticulatedConfig.pedal_attachment` field with validation.
- Emit `weld_foot_front`/`weld_foot_rear` equalities in physical mode — only
  AFTER `build_articulated_rider` (foot bodies do not exist when
  `finish_physical_topology` runs). No `relpose` → datum is the design pose at
  `qpos0` (sole flat on platform), consistent with the non-physical path.
- No contact-exclusion changes: rider geoms are `contype=0`; pads were never
  native contacts.

**Rider contacts (`rider_contacts.py`)**
- Weld mode: `front_pedal`/`rear_pedal` supports skip pad force evaluation.
  Diagnostics keep the same keys (`in_platform`, `normal_load_n`,
  `force_on_rider_n`, `gap_m`) but are filled from the weld reaction so
  downstream telemetry/HUD/validity readers keep working.
- `delivered_crank_torque_nm` computed from weld efc rows (post-step read,
  one-step lag — same cadence as today).
- Saddle and grip supports unchanged (still unilateral pads / compliant grip).
- `set_enabled('front_pedal'|'rear_pedal', False)` rejected or no-op in weld
  mode: welded feet cannot be released.

**Rider control (`rider_control.py`)**
- `_targets`: sole goal = fixed weld anchor on the pedal body (no
  `project_sole_goal`, no footprint/shear/compression machinery for feet).
- `PedalRecovery.observe` skipped; `stage` forced to `none`.
- `swing_clearance_m` unused (foot cannot hover); the rising-pedal unload
  preference in `enabled`/support requests stays — it is "don't brake the
  rising crank", not a contact property.
- `loads[side]` input for `feasible_pedal_force` = weld normal reaction from
  the previous step, clamped at 0 (pull-up reaction is not available friction).

**Init / equilibrium (`physical_equilibrium.py`, `physical_runtime.py`)**
- Legs are posed by `rider_control.initialize` at the actual crank phase before
  relaxation (existing path) → weld residual ≈ 0 at start.
- `data.qpos[pedal_{side}_spin] = -phase` writes must not fight the weld: in
  weld mode the platform pose follows the foot — set `pedal_spin` consistently
  with the posed foot (or let relaxation settle it; verify residual).
- `solve_physical_equilibrium` runs `mj_step` — weld rows are ordinary
  constraints; verify convergence and `settled_crank_phase_rad` sanity.

**Drivetrain (`drivetrain_forces.py`, `physical_runtime.py`)**
- `sensed_human_nm` wiring unchanged; the value now comes from the weld
  reaction (see above). Assist engage/boost semantics preserved.
- Freehub (`ideal_mid_drive_freehub`), assist motor, shifting, mash ramp,
  PedalingPolicy: untouched.

**Config surface**
- `[articulated] pedal_attachment = "weld"` added to
  `viewer_physics_fast.toml` (or a welded copy); all existing profiles keep
  `flat` behavior.

## Semantics contract (weld mode)

- Foot can never leave the pedal: structurally guaranteed by the constraint.
- The rising foot cannot hover: `swing_clearance` is meaningless; leg targets
  stay at the weld anchor for the whole revolution.
- Crank accel sources (motor boost, other leg, freehub engagement kick) drag
  the feet through the weld; legs absorb via joint PD/torque limits and the
  pelvis is pulled if IK saturates — physically correct clipless behavior.
- Weight distribution machinery unchanged; pedal load is now a weld reaction
  rather than pad compression.

## Risks / honest caveats

1. **Closed kinematic loop**: pelvis ↔ both legs ↔ crank ↔ frame is
   over-constrained. If the pose is inconsistent, weld rows fight each other
   and yank the pelvis. Mitigation: IK-consistent init + weld-residual
   telemetry assert (< ~3 mm translation equivalent).
2. **Equilibrium unverified** with active welds — refine may need weld-aware
   handling (pose pedals from feet, not `-phase`).
3. **Dead-center IK saturation**: weld wins over leg torque limits — pelvis
   gets pulled toward the pedal. Watch `saturated_ik` + residual.
4. **Solver cost**: net cheaper (2 equality constraints replace 4 pad contacts
   + brush friction per foot), but equality rows are stiffer than pads —
   confirm no solver instability at dt=1.25 ms.

## Acceptance

- Headless `rough_uphill` run, `pedal_attachment="weld"`:
  zero `recovery_stage != 'none'` events; weld residual below ~3 mm; assist
  engages from weld-sensed torque ≥ `engage_torque_nm`; crank rate → 0 within
  ~0.35 s of coasting entry; run completes or crashes only via existing
  detector reasons (no weld-induced blowup).
- Existing `flat` profiles: bitwise-identical behavior (default unchanged).
- Manual viewer pass: feet stay pinned, sole stays level via ankle.
