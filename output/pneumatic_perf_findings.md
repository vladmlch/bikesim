# Detailed-tier pneumatic equilibrium failure — root cause investigation

Worktree: `.worktrees/pneumatic-perf`
Config: `examples/research/viewer_physics_pneumatic.toml` with
`pneumatic.tier='detailed'`, `timestep_s=0.00025` (forced by gate).

Original failure:

```
RuntimeError: physical equilibrium did not converge after 80000 steps:
residual=0.0714746; [('rear_wheel_spin', 0.0714746),
('rocker_frame_pivot', -0.065370), ('yoke_pivot', 0.056523),
('pedal_rear_spin', -0.049370), ('pedal_front_spin', -0.049370)]
```

## TL;DR

Not a brush bug and not a solver-code defect. The standstill equilibrium of this
bike+rider genuinely requires a **nonzero tangential self-stress between the
axles** (~±30–90 N at the contact patches) to balance the suspension-linkage /
drivetrain / rider-contact moment loop. That shear is stored as bristle
deflection (detailed) / κ′ (lumped) and can only be reached through **physical
tread creep** — the wheel micro-rotates through the regularized MuJoCo
brake-friction constraint (~4 µrad/cycle ≈ 1.5 µm of tread per 20 ms). The
process is monotone but slow: residual decays ≈ 5–7e-4/s once the bodies have
settled. It needs ~55–60 s of simulated time on this branch; the detailed tier
gets exactly 20 s because `max_steps = max(40000, round(20.0/dt))`
(`physical_equilibrium.py:71`) — while the shipped fast tier at `dt=0.00125`
silently gets 50 s via the 40k-step floor and passes (~35 s).

Verified: the same detailed configuration converges at **step 236000
(t = 59.0 s), residual 0.049995** when allowed to run past the cap.

## Residual trajectory (instrumented)

`output/eq_probe.py` re-implements `solve_physical_equilibrium` with per-cycle
logging (`qacc`, `qfrc_smooth`, `qfrc_constraint`, friction-row force, tire
diagnostics). Trace artifacts: `output/eq_trace*.json`.

| phase | time | residual | dominant DOFs |
|-------|------|----------|----------------|
| drop  | 0–0.2 s | 826 → ~50 | rider elbows, then suspension pivots |
| settle bodies | 0.2–5 s | → ~0.09 | fork_travel, pivots, rider joints |
| creep grind | 5–59 s | 0.09 → 0.05 | rear_wheel_spin + rocker/yoke + pedal spins |

From step ~20k onward the decay is **linear ≈ 6-7e-4/s** — no plateau, no
oscillation, no blow-up. (trace: `eq_trace_plain.json` — 236k-step converged run)

## What drives the residual

Late-state force balance (converged state, step 236000):

```
rear_wheel_spin   smooth=+9.243  constr=-9.233  qacc=+0.050
front_wheel_spin  smooth=-10.144 constr=+10.144 qacc=-0.005
yoke_pivot        smooth=+1.015  constr=-1.014  qacc=+0.039
rocker_frame_pivot smooth=+0.200 constr=-0.199  qacc=-0.046
pedal_*_spin      smooth=~0      constr=~0      qacc=-0.034   (inertia coupling)
```

- Tire tangential force maps to wheel torque: Ft_rear=-27.0 N × R≈0.342 m ≈ −9.2 N·m = smooth torque. `pneumatic_tire_forces.py:348-475` (Jᵀ·F through the patch-centroid Jacobian).
- The brake is a **regularized MuJoCo friction constraint** (`dof_frictionloss=200`,
  `static_braking.py:13-36`), not a hard clamp: under ~9–17 N·m held torque the
  constraint solve leaves ~0.01–0.04 N·m net → qacc 0.05–0.08 on a 0.225 kg·m²
  inertia with coupling. Residual per unit torque ≈ 0.004 rad/s²·N·m — the
  regularization give.
- Wheel creep measured per-step (`eq_steps_no_refine_step_log.json`): right after
  each `qvel.fill(0)` the wheel re-accelerates at ~0.097 rad/s², the friction row
  engages within ~10 steps, net rotation **4.3e-6 rad/cycle** — that creep is the
  channel through which bristle shear self-tunes.
- Pedal-spin residuals are inertia-coupled (smooth≈constr≈0; qacc≠0 via
  off-diagonal M through crank/foot welds) — not independently driven.

## Key experiments

1. **Zap** (zero all 86 bristles at step 40000): residual **exploded to 33.7**,
   dominated by rocker/yoke/main_pivot + rider joints. → The stored shear is
   **load-bearing**, not debris: without it the linkage is out of static balance.
   (`eq_zap.log`, `eq_trace_zap.json`)

2. **Biasprobe** (scale both wheels' bristle fields s_f, s_r ∈ [−0.4,1.4] at
   step 40000): residual is a **V-cusp with minimum exactly at s=1.0** — 0.078
   vs ~4.7–5.0 at ±0.2 offsets. → The creep already self-tuned shear to the
   residual-optimal value; **a refiner-owned shear channel cannot beat the
   floor**. (`eq_bias.log`)

3. **Convergence matrix** (all `equilibrium_cache_enabled=false`, fresh settles):

   | run | brush | rays | dt | cap | result | branch (front/rear Ft) |
   |-----|-------|------|----|-----|--------|------------------------|
   | detailed (baseline) | discretised | 256 | 0.00025 | 80k=20s | FAIL 0.0715@20s | compression +52/−51 |
   | lumped256 | lumped | 256 | 0.00025 | 80k=20s | **PASS 0.0495@5.8s** | tension −28/+29 |
   | fast250 | lumped | 64 | 0.00025 | 40k=10s | FAIL 0.0647@10s | compression +44/−43 |
   | dt1250 | discretised | 256 | 0.00125 | 40k=50s | FAIL 0.0531@50s | compression +32/−31 |
   | brush64 | discretised | 64 | 0.00125 | 40k=50s | FAIL 0.0520@50s | compression +27/−26 |
   | shipfast (shipped TOML) | lumped | 64 | 0.00125 | 40k=50s | **PASS 0.0500@35s** | tension −87/+87 |
   | plain240k | discretised | 256 | 0.00025 | 240k=60s | **PASS 0.0500@59.0s** | compression +28/−27 |
   | brake=0 | discretised | 256 | 0.00025 | 40k | FAIL 1.45 (drivetrain rings) | — |
   | denserefine (period 1s) | discretised | 256 | 0.00025 | 120k | 0.065@30s ≈ baseline | compression |

   → **Branch lottery**: the first transient (~800 residual) throws the system
   onto either a *tension* branch (ground pulls wheels apart — Ft front<0, rear>0)
   or a *compression* branch. Tension-branch runs pass even at |Ft|=87 N (the
   torque reaction flows through the drivetrain/foot-weld path); compression-
   branch runs grind slowly (torque reacted mainly by soft brake friction).
   Convergence time is dominated by which branch is landed on + shear magnitude,
   not by brush type per se — lumped-64 at fine dt also failed; discretised-64
   at coarse dt nearly passed.

4. **Brush-only microexperiment** (v_x=0.1, v_s=−0.035 m/s, 80 steps of
   dt=0.00025): discretised reaches 126 N vs lumped 98 N — the discretised brush
   is ~1.28× stiffer per unit slip history (≈ σ/a ratio); both hold shear
   statically after v_s→0. A contributor to winding rate, not the root cause.

5. **Per-step in-cycle data** shows the residual *is* the regularized constraint
   give: smooth +27.37 N·m vs friction row −37.5 N·m (the difference is reacted
   through linkage/foot-weld equality rows), net → qacc 0.097 right after qvel
   zeroing, decaying to ~0.01 within ~20 steps as the constraint catches.

## Brush comparison at ω ≈ 0 (`tyre/brush.py`)

- `relax()` zero-speed branch (`brush.py:65-66`): `κ' -= v_s·dt/σ` — a pure
  integrator of slip speed; no divide-by-zero, no discontinuity.
- Discretised brush (`brush.py:214-264`): κ' filtered with
  `input_sigma = max(0, σ−a)` (≈1 cm — fast); at `v_x=0` the bristle growth input
  is raw `v_s` (`brush.py:225-226`); advection by `wheel_surface_speed = v_x−v_s`
  (`brush.py:229-241`), copy at exactly 0; per-element friction-limit clipping
  and projection (`brush.py:255-261`).
- At standstill both brushes are **history-bound integrators**: shear only
  changes as fast as tread actually slips (~µm/cycle through the soft brake
  constraint). No zero-speed bug found in `brush.py` or `carcass.py` — the
  behavior is physically consistent; the shear they must store is real.

## Why the equilibrium needs shear at all

The bike is a closed static structure: wheel contact forces → swingarm linkage
(main_pivot/rocker/yoke) + telescopic fork → frame → rider welds
(saddle/grips) and feet→pedals→crank→chain→rear wheel. The shock's line of
action and the contact-force line through the patch don't pass through the same
instant center, so the swingarm moment balance needs a *horizontal* contact
component; equal-and-opposite tangential forces at the two axles form a
self-stress. The settle must find (qpos, bristle state) jointly — qpos moves on
body timescales, bristles only by creep.

`equilibrium_refine.py:21-27` literally documents the missing channel: the
optimizer owns `xi` scalars of other contact models but "the pneumatic tyre's
carcass and patch memory has no xi coordinate for the optimizer to own."
(Experiment 2 shows even that wouldn't beat the cusp floor at fixed pose.)

## Minimal unblock

**Immediate, no tracked-source change** — monkeypatch the solver before building
(any driver/bench script in `output/` can do this; `physical_runtime.py:373`
does a *late* `from ... import solve_physical_equilibrium` inside `reset()`, so
patching the module attribute works — this is exactly what `eq_probe.py` does):

```python
import bike_sim.sim.ride.physical_equilibrium as _pe
_orig_solve = _pe.solve_physical_equilibrium
def _long_settle(runtime, **kw):
    kw.setdefault('max_steps', 320000)   # ~80 s at dt=0.00025; converged at 59 s
    return _orig_solve(runtime, **kw)
_pe.solve_physical_equilibrium = _long_settle
```

Plus `equilibrium_cache_enabled = true` in the TOML (`model_config.py:61`
already supports it) so the ~12 min settle is a one-time cost per config
fingerprint.

**Durable fix (solver-side)** — `src/bike_sim/sim/ride/physical_equilibrium.py:71`:

```python
max_steps = max(40000, round(20.0/dt))
```

is dt-inconsistent: it intends "20 s of settling" but the 40k floor grants 50 s
at dt=0.00125 (the only reason shipped `fast` passes). Change the time budget,
e.g. `round(80.0/dt)` for margin, or plumb a `equilibrium_settle_time_s` /
`equilibrium_max_steps` field through `PhysicsConfig` (the solver already takes
`max_steps`/`tolerance` kwargs; only the plumbing is missing —
`physical_runtime.py:394` calls it with no kwargs).

**Not recommended**: refiner shear channel (cusp floor — can't beat 0.078 at the
landed pose), denser refinement (measured: no material speedup, creep-limited),
brake removal (worse — drivetrain rings at 1.4), brush zero-speed model changes
(no defect found; would need native-mirrored physics change for a non-bug).

## Remaining open questions

- The compression-vs-tension branch landing is chaotic (initial transient
  decides). With 320k the compression branch passed at 59 s — margin exists but
  a different pose/track could land higher-shear and exceed even that. A
  residual-floor-aware acceptance (e.g., accept residual < ~0.1 while decaying)
  would be more robust than pure time scaling — heavier semantics change.
- Exact constraint-force attribution: friction rows vs linkage/foot-weld
  equality rows share the hold (friction row −37.5 N·m vs net constraint −27.4);
  the per-row split isn't currently surfaced — MuJoCo efc introspection could
  quantify it if needed.
- Why tension branch (Ft_front<0) reacts through a stiffer path — likely chain
  tension direction changes which constraint rows carry the wheel torque; not
  fully traced.
- `initial_speed_mps` cannot help: the settle zeroes qvel every 20 ms cycle, so
  the tire sees ~zero rolling speed regardless — verified by code path
  (`physical_equilibrium.py:87`), not separately run.
- dt=0.00125 for detailed is **gated impossible** in config
  (`model_config.py:116-125`) — the isolation used a monkeypatched tier; making
  it legitimate would need the gate relaxed (it exists to keep the discretised
  brush's per-step advection sub-cell: `wheel_surface·dt/dx` — at 0.00125 the
  shift per step can exceed one element).

## Update — refiner shear channel implemented and measured (then reverted)

The requested design was implemented and exercised end-to-end before being
rejected on measurement:

- `_PatchBrushState` gained an idempotent scalar `shear_offset_m` (setter shifts
  `detailed.bristle_deflection_m` by `(new−old)·shear_weights`, weights =
  load-proportional shares of `elastic_prev_n` so light elements clip last).
  `equilibrium_refine` collected one coordinate per patch:
  `shear_offset_m` (scale 2 mm) for discretised brushes, `kappa_prime`
  (scale 0.5) for lumped — i.e. the same "own the tangential-shear channel"
  contract `xi` gives every other backend, expressed on the real committed
  state (wire/native contract untouched).

**A/B outcome (same probe, only the coordinate differs):**

| run | coords | result |
|-----|--------|--------|
| lumped256 | none (baseline) | converged 23280 steps — reproduces Oct-13 baseline bitwise |
| lumped256 | kappa_prime | **stalled ~0.10 → FAIL at 64000** (refiner even returned worse-than-entry residuals) |
| detailed | none (baseline) | converged 236000 @ 59 s |
| detailed | shear_offset_m | **slower manifold — still 0.064 at 320000 (80 s) → FAIL** |

Mechanism: κ′/bristle are pure integrators of slip speed (`brush.py:66`). The
least-squares commits a shear value that satisfies the *instantaneous* torque
balance — which removes the wheel-spin residual that powers the creep — while
being slip-inconsistent with the pose. The trajectory is thrown onto the slow
manifold and deadlocks there. Owning the channel doesn't accelerate the creep;
it dismantles it.

**Optimizer battery at a mid-creep state (residual ≈ 0.10) — all fail:**
plain TRF (stalls 0.098), peak-weighted residual (0.0974), LP linearization
(linear model predicts feasibility at ±1 box but actual eval explodes to ~1300 —
the residual is discontinuous at ~50 µm coordinate scales), sequential LP and
damped Newton (trust radius collapses to 0), block-coordinate shear↔pose
(returns to 0.0973), shear-continuation homotopy (residual grows to 78 — the
wheel-pinning self-stress is physically incompatible with the landed pose),
2-D shear grid at fixed pose (min 0.0973 — better than entry only by luck, the
needed ±51 N self-stress is not producible at that pose), Aitken Δ² sequence
extrapolation of the whole creep state (explodes at both 24 k: λ=0.36 off the
manifold, and 80 k: λ=0.95 → ×21 amplification leaves it).

**Conclusion**: the standstill equilibrium is reachable only by integrating the
slip dynamics (~59 s simulated creep on the compression branch). No refiner-side
coordinate optimization or state extrapolation can cross the ridge. The shipped
change is therefore confined to the settling-time budget
(`physical_equilibrium.py`): `max_steps` default `max(40000, round(80/dt))`,
so detailed converges at ~236 k well inside the cap, while every other backend
and the wire/native contract are bitwise-identical to before.
