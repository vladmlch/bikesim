# Spatial 3D — Phase 8 (Track H): GPU Feasibility Memo (Metal) Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** none — starts on day 1 in parallel with Phase 0. **Timebox: 2 agent-days.** When the timebox is spent, write the memo with whatever was measured and the verdicts the pre-registered criteria give; do not extend.

**Goal:** A written, measured answer to "should any part of this simulator run on the GPU?", on the only target machine (Apple M4 Max, 16 CPU / 40 GPU cores, 64 GB, Metal 4; no CUDA), delivered as `docs/superpowers/audits/2026-10-14-gpu-feasibility.md` plus microbenchmarks under `tools/gpu_feasibility/`, and closed by `docs/adr/0003-gpu-offload.md` with a go/no-go per hypothesis. Nothing from this phase enters the tick-path.

**Architecture:** Three hypotheses with **kill criteria written before measuring** (spec §11.4). Microbenchmarks are standalone programs (Metal via Swift or C++ with `metal-cpp`; whichever is quicker to stand up) that reproduce the data shapes of the 3D tyre: `R` rays per step against a `nx × ny` float32 grid with bicubic sampling, and per-element carcass arithmetic — not the real writer.

**Tech Stack:** Metal (metal-cpp or Swift), `xcrun metal` for shader compilation, Python for plotting; no repository build changes.

**Spec:** §11.4. **ADR:** produces 0003.

## Global Constraints

- All master-plan constraints that apply to documentation; this phase changes no tracked source outside `tools/gpu_feasibility/` and `docs/`.
- Pre-registration: the kill criteria below are copied verbatim into the memo before the first measurement; results are reported against them unchanged.
- Report medians over ≥ 200 iterations with warm-up; report dispatch + sync latency separately from kernel time.

## Pre-registered hypotheses and kill criteria

| # | Hypothesis | Measurement | Kill if |
|---|---|---|---|
| H1 | Per-step offload of ray casting + element carcass to Metal beats the CPU for the fast tier (R ≈ 900 rays) or the detailed tier (R ≈ 7 700). | GPU time per step (dispatch + kernel + readback of ≤ 8 patch aggregates) vs CPU single-thread and CPU 8-thread estimates from P3/P7 (or an analytic 100 ns/ray if P3 is not yet merged), for R ∈ {900, 7 700, 10⁴, 10⁵}. | GPU step ≥ 0.5 × CPU-8-thread step at R ≤ 10⁴. (Expected outcome: killed — dispatch/sync latency of tens of µs vs a ≤ 100 µs CPU budget.) |
| H2 | Batched-episode physics on the GPU (MJX/mujoco_warp style) would give the research sweeps a large throughput gain. | Desk study + one measurement: MJX on Metal (JAX Metal plugin) stepping the planar MJCF with MuJoCo contacts only, batch ∈ {64, 1 024}, steps/s; list what would have to be reimplemented (analytic tyres, rider controller, recorder, energy ledger, equality introspection) and whether MJX supports the model's features (equality welds, implicitfast, hfield). | Projected sweep throughput gain < 20× **or** reimplementation effort > 2 agent-weeks **or** a required feature unsupported. (Expected: killed on effort/features; recorded as a possible separate product, not a track.) |
| H3 | GPU helps offline work: terrain generation (FFT roughness, feature stamping on 60k × 300 grids), tyre calibration sweeps (thousands of independent rig evaluations), telemetry post-processing. | Terrain generation time CPU (NumPy) vs Metal/`mlx`; calibration sweep as embarrassingly parallel rig runs — compare process-level CPU vs a GPU batch of the lumped brush equations. | Speed-up < 3× on the end-to-end task, or added dependency weight judged disproportionate (document). |

---

### Task 1: Scaffold and pre-registration

- [ ] **Step 1:** Create `tools/gpu_feasibility/README.md` with the table above verbatim and the machine facts (`sysctl`, `system_profiler` output).
- [ ] **Step 2:** Create `docs/superpowers/audits/2026-10-14-gpu-feasibility.md` skeleton with the criteria section filled and empty result tables.
- [ ] **Step 3:** Commit `docs: pre-register GPU feasibility hypotheses`.

### Task 2: H1 microbenchmark

- [ ] **Step 1:** `tools/gpu_feasibility/h1_raycast/`: Metal compute kernel: per ray, DDA over a float32 grid with bicubic sampling and 8 bisection steps (same algorithm as P2's `Grid3D::ray_cast`), writing per-ray `t`; a second kernel for the per-element carcass arithmetic (the planar law, vectorised); a reduction to ≤ 8 patch aggregates on GPU; readback. Host loop with `MTLSharedEvent` or blit sync; measure dispatch+sync and kernel separately.
- [ ] **Step 2:** CPU baseline in the same program (scalar C++), single thread; 8-thread estimate = single/6 (document the assumption) unless P3/P7 numbers exist.
- [ ] **Step 3:** Fill the table for R ∈ {900, 7 700, 10⁴, 10⁵}; plot; apply the criterion. Commit `bench: GPU per-step ray-cast/carcass offload measurement (H1)`.

### Task 3: H2 desk study and one measurement

- [ ] **Step 1:** Feature matrix: the model's MJCF features (freejoint, equality connect/weld, hfield, implicitfast, sensors, actuators) vs MJX support on Metal; list the native units that would need GPU reimplementation with their line counts (spec §3 facts table: writers 10 541, runtime 10 681, rider 8 836, pneumatic 3 226).
- [ ] **Step 2:** If the JAX Metal plugin installs in a throwaway `uv` environment within 1 hour: step the welded planar MJCF (MuJoCo contacts only) with MJX at batch 64 and 1 024; record steps/s; otherwise record the failure and use published MJX numbers with the caveat.
- [ ] **Step 3:** Apply the criterion. Commit `study: batched GPU physics feasibility (H2)`.

### Task 4: H3 measurements

- [ ] **Step 1:** Terrain: time `build_track3d` (NumPy, from P2 if merged; otherwise a synthetic 60k × 300 FFT roughness + 50 feature stamps) vs an `mlx` (Apple's array framework) implementation of the same operations.
- [ ] **Step 2:** Calibration sweep: 10⁵ evaluations of the lumped 2D brush (P3 formulas) across `(α, κ, γ, F_z, p)`: NumPy vectorised vs `mlx`; report end-to-end including conversion.
- [ ] **Step 3:** Apply the criterion. Commit `bench: offline GPU uses (H3)`.

### Task 5: Memo and ADR 0003

- [ ] **Step 1:** Complete the memo: results vs criteria, verdict per hypothesis, what would change the verdict (e.g., a CUDA machine, a 10× larger ray count, a batched-product decision).
- [ ] **Step 2:** `docs/adr/0003-gpu-offload.md`: status accepted; decision = the verdicts; consequences (expected: no GPU on the tick-path; optional `mlx` for offline tooling only if H3 passed; H2 recorded as a separate potential product).
- [ ] **Step 3:** Commit `docs: GPU feasibility memo and ADR 0003`. Merge (docs + tools only); tag `spatial-3d-p8`.
