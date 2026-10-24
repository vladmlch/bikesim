# Spatial 3D GPU feasibility — W55 skeleton

Authored: 2026-10-24. **Execution: not_run. Decisions: pending-measurement.**

This is the pre-measurement record for the optional
[W55/V11 plan](../plans/2026-10-22-spatial-3d-native-08-gpu-research.md).
No GPU host was probed, SDK installed, kernel written, benchmark run or backend
selected during W55. The threshold helper and test are source-authored only.

## 1. Questions and scope

| Hypothesis | Question | Candidate scope |
|---|---|---|
| H1 | Can offloaded rays/elements reduce the wall time of one sequential interactive ride? | A section consumed by the next CPU MuJoCo tick |
| H2 | Can a separately implemented batch workload beat the best measured CPU process batch? | Many independent, physically equivalent episodes |
| H3 | Can a self-contained offline task justify GPU support? | Generation, calibration or analysis from input to final artifact |

CPU is the reference and default backend. Metal evidence establishes nothing
about a Linux device. CUDA support is not promised. A positive result authorizes
consideration of a separate design, not a product integration.

## 2. CPU identity and workload size

| Required field | Value |
|---|---|
| Source/compiler/provider/dependency identity | pending V09 |
| Physical config, dt, root, tyre tier, sensor/recording modes | pending V09 |
| Terrain dimensions, physical resolution and material table | pending V09 |
| Rays/elements/material slots per step | pending V09 |
| Transfer bytes per tick, residency and readback boundary | pending V09 |
| Full valid ride duration and demand trace identity | pending V09 |
| Serial measurements | pending V09 |
| Tyre-pool decision and measured best CPU thread count | pending V09 |
| Best process batch and complete offline task reference | pending V11 |

Use the [CPU performance ledger](2026-10-22-spatial-3d-performance.md).
An assumed `CPU8≈CPU1/6` or `100 ns/ray` is not measurement evidence.

## 3. GPU host and toolchain

| Required field | Value |
|---|---|
| OS, architecture, hardware | pending V11 |
| Device identity and memory | pending V11 |
| Driver/runtime version | pending V11 |
| SDK/compiler/API versions | pending V11 |
| Float64/storage/synchronization support relevant to the workload | pending V11 |
| Availability decision | pending-measurement |

If no suitable GPU host is available, V11 records `unsupported-host`. Do not
translate absence of a host into a measured performance no-go or a passing
qualification result.

## 4. Official support sources

| Platform/API | Official source URL | Version | Publication/access date | Supported claim |
|---|---|---|---|---|
| Candidate API | pending | pending | pending | pending |
| Numerical and synchronization limits | pending | pending | pending | pending |

Populate sources when a real host/API is selected in V11. No invented API
support matrix or stale hardware assumption is supplied by this skeleton.

## 5. Criteria fixed before measurement

| Hypothesis | Measured boundary | Go-for-separate-design requires | No-go condition |
|---|---|---|---|
| H1 | Dispatch + upload + kernel + reduction + readback + sync; also complete ride | GPU section ≤0.5× best qualified CPU section **and** whole ride time ≤0.90× CPU; unchanged accuracy and determinism | Any condition fails |
| H2 | Valid complete episodes/s with the same physical model | ≥20× best measured CPU process batch **and** separately accepted implementation scope | Physical equivalence fails, <20×, or engineering budget exceeds the accepted scope |
| H3 | Complete task input→artifact, including setup and I/O | CPU task time ≥3× GPU task time and exact artifact checks pass | <3×, correctness failure or unjustified dependency cost |

The source helpers are in
[`tools/spatial_gpu/decision.hpp`](../../../tools/spatial_gpu/decision.hpp).
H1 uses `interactive_goal_met`; `latency_goal_met` alone covers only its section
criterion. H2's throughput helper does not decide implementation scope. No
criterion is adjusted after seeing the result.

## 6. Measurements to acquire

Warm up before steady-state measurements and retain at least 200 microtest
repetitions. Report distributions and the complete valid workload, not a chosen
best run. The sequential CPU consumer must receive the actual readback before
the measured GPU section ends.

| Region | Serial CPU | Best qualified CPU | GPU | Evidence/identity |
|---|---|---|---|---|
| Setup/allocation/compilation | pending | pending | pending | pending |
| Dispatch/queue | pending | pending | pending | pending |
| Upload/transfer | pending | pending | pending | pending |
| Ray/element kernel | pending | pending | pending | pending |
| Reduction | pending | pending | pending | pending |
| Readback | pending | pending | pending | pending |
| Synchronization | pending | pending | pending | pending |
| End-to-end section | pending | pending | pending | pending |
| Complete valid ride, including final flush | pending | pending | pending | pending |
| Complete batch, valid episodes/s | pending | pending | pending | pending |
| Complete offline input→artifact task | pending | pending | pending | pending |

Setup and warmup are separately identified. No omitted dispatch/readback/sync
cost may be relabelled as end-to-end time. Accuracy and exact-state checks use
the pre-existing CPU/physical contracts and the same identities.

## 7. CPU comparisons and correctness

| Comparison | Status |
|---|---|
| Same workload versus measured serial CPU | pending V11 |
| Same workload versus measured, qualified CPU pool or documented not-selected decision | pending V11 |
| Accuracy/physical invariants at unchanged dt and tyre law | pending V11 |
| Determinism and complete state across sequential steps | pending V11 |
| H2 same physics and valid-episode counts | pending V11 |
| H3 exact final-artifact checks | pending V11 |

Changing precision, fixtures, quality tier or tolerances defines a different
experiment and cannot satisfy these criteria. A shader-only timing cannot
qualify the interactive path.

## 8. Decisions and engineering budget

| Hypothesis | Decision | Measured reason | Separate design/cost assessment |
|---|---|---|---|
| H1 | pending-measurement | pending V11 | pending |
| H2 | pending-measurement | pending V11 | pending |
| H3 | pending-measurement | pending V11 | pending |

Budget: at most **two engineering days** to acquire V11 evidence. A positive
decision requires an explicit implementation estimate and separate design and
qualification plan. No GPU ADR is accepted here; ADR-0003 is occupied and must
not be overwritten. A later accepted decision chooses an available ADR number.

## 9. Isolation and handoff

Prototype isolation: **confirmed: `tools/spatial_gpu` only** for this source
delivery. The native product has no GPU kernel, GPU SDK, tool link, optional
build dependency or changed CPU/numerical default. The root option is only a
pointer to the standalone directory. Source helper files and this memo are not
GPU execution evidence.

The standalone target reuses the real `bt` harness and directly compiles its
existing cold JSON/hash dependencies. It neither links product targets nor
replaces the harness with unchecked assertions. Its only external include tree
is already acquired, locked RapidJSON; no SDK is needed to verify decision
arithmetic. The [README](../../../tools/spatial_gpu/README.md) documents later
V11 commands and prototype/evidence locations. Those commands have not run.

V10 spatial qualification is independent of W55/V11 completion.
