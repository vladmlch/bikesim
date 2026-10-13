# Pneumatic tyre native perf — findings (2026-10-13, research worktree)

Scope: where per-step time goes in the `pneumatic` native backend and which
optimizations preserve fidelity. Evidence: 4 parallel investigations (cost-map, probe-overhead, jac-parity, bench) +
measured numbers.

## Measured (Release -O3, same track/config, native backend, headless)

rough_uphill_savage, launch-from-rest, record_decimation=1, best-of-3 ×420 steps:

| backend                            | ms/step                              | RTF @ dt=1.25ms |
|------------------------------------|--------------------------------------|-----------------|
| compliant_2d                       | 0.141                                | 8.84x           |
| distributed_2d_reference (128 st.) | 0.175                                | 7.14x           |
| pneumatic tier=fast (64 rays)      | 0.187                                | 6.68x           |
| pneumatic tier=detailed (256 rays) | setup fails (equilibrium, see below) | —               |

**Pneumatic costs +46 µs/step vs compliant (+33%), not the ~20x the
interactive 0.45x RTF suggested.** The interactive number is viewer/pacing/
equilibrium-refine bound (or an unoptimized `native/build`), not tyre cost.
Repro: `output/bench_tire_backends.py` + `.log` in this worktree.

## Per-step cost map of the pneumatic backend (n=64)

Per advancing step (`PneumaticTireWriter::accumulate_validated` →
`evaluate_wheels`, `native/src/writers/pneumatic_tire.cpp:828,873`):

- 2 × `PneumaticTyreModel::evaluate` (one per wheel), each doing
  `intersect` (geometry.cpp:126-285) + `evaluate_carcass` (carcass.cpp:362-543)
    + patch/brush loop (model.cpp:277-408).
- +2 × extra `intersect_at` at the SAME pose in the writer tail (pneumatic_tire.cpp:970-971) — for radial energy +
  penetration only.
  => 4 intersect passes/step total.
- ~1080 `atan2`/step — the dominant transcendental vector. Root cause:
  `phi_upper_bound` (geometry.cpp:68-87) recomputes `atan2` on every binary
  -search probe (~n·log2 (win) + win per intersect) purely to avoid a
  per-call buffer (documented geometry.hpp:25-32). A construction-sized
  phi scratch fixes it bitwise-identically (~20x fewer atan2).
- ~128 `np_tanh` LUT + 2n `np_interp_unit_grid` advection (carcass.cpp:445-501).
- 2 × `assign()` backups ≈ 5-7 KB memcpy/step (rollback atomicity,
  pneumatic_tire.cpp:843-844). Real but ~µs-class — not worth chasing.
- ~2-4 `mj_jac`/step (patch count ≈1 per wheel; bound is 32 but realistic
  contacts are 1 contiguous arc, multi-arc only on pothole/bridging terrain).
- Warm path verified allocation-free (construction-sized scratch, capacity
  reuse). `backup_`/`probe_` not prewarmed for detailed brushes — cold-path
  only.

Measured-delta sanity: model eval ≈ 46 µs/step of the ~55-75 µs static
estimate — consistent; the remaining ~140 µs baseline (all backends) is
engine forwards ×2-3, rider/drivetrain/suspension writers, accounting flush,
diagnostics/wire serialization.

## Probe evaluations — dead hypothesis

`probe_compute_into` (full clone+re-evaluate of both tyre models) fires ONLY
via `probe_step_inputs` — which has **zero production callers** (tests and
the pre-first-commit `refresh_contacts` fallback only; step.cpp:1899).
Per advancing step in every real run mode: **0 probe evaluates**.
Interactive snapshot ()/headless/research loops never probe the tyre model.

## Jacobian shift-trick — technically feasible, negligible gain

- Oracle+C++ lockstep change is precedented (distributed rewrite audit
  `docs/superpowers/audits/2026-10-11-distributed-reference-rewrite.md`,
  commit `34263bb`) and needs **zero artifact regen** — pneumatic goldens
  are captured at test time (`test_native_pneumatic_tire.py::_golden`),
  none committed.
- BUT: ~1 patch/wheel in practice; `mj_jac` ≈ 3·nv memset + ~10-DOF chain
  walk — the shift loop costs about the same. Verdict: skip; keep the
  per-centroid `mj_jac` (it is already the bitwise-correct fast path).
- docstring `pneumatic_tire_forces.py:5-7` already records the deliberate
  rejection of J (center)+ω×arm on rounding grounds.

## Ranked opportunities (fidelity-preserving)

1. **Memoize the phi row in `phi_upper_bound`** — construction-sized scratch;
   identical results (pure function of the profile window); cuts the biggest
   transcendental volume ~20x. ~tens of µs/step.
2. **Deduplicate the writer's second `intersect_at`** — reuse the model's
   last hits for radial-energy/penetration (same pose, deterministic →
   bitwise-identical values) or fold the energy computation into the model's
   own pass. Removes 2 of 4 intersect passes (~25% of model work).
3. **Detailed-tier setup failure** (blocking, not perf):
   `physical_equilibrium` does not converge in 80k steps at dt=0.00025 —
   residual dominated by wheel/pedal spin DOFs; consistent with discretised
   brush stick-slip holding a standstill limit. Investigate the equilibrium
   solver/brush interaction before any detailed-tier benchmarking.
4. Concave-terrain regime: `_brute_force` fallback (geometry.cpp:247-270)
   swaps n·log (win) atan2 for n× (win−1) divisions — profile kerb/pothole
   tracks separately before optimizing for it.
5. Bigger fish are outside the tyre: engine forward passes, rider/drivetrain
   writers, interval accounting — that's the ~75% of step time all backends
   share. Only relevant if overall step cost is the goal.

## Non-goals checked

- Probe-path optimization: pointless (no production caller).
- Backup assign elimination: ~µs, not worth contract risk.
- Patch-count-based branching (keep per-centroid mj_jac ≤1 patch): already
  the dominant case.
