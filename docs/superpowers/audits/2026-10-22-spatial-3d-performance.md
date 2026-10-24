# Spatial 3D performance — W50–W53 source handoff

Authored: 2026-10-24. **Execution: not_run. Verification: pending V09.**

This is the pre-measurement ledger, not a benchmark result. The source delivery
contains a native measurement driver/report, persistent tyre worker pool,
invariance/lifecycle tests and an isolated engine-pool probe. No configure,
compiler, application, CTest, sanitizer, timer workload or GUI was executed.
The [qualification ledger](2026-10-22-spatial-3d-qualification.md) remains pending.

## Profile and measured candidate identity

| Field | Declared source input / measured value |
|---|---|
| Profile | `spatial-fast-v1` |
| Model | Free root, articulated effort rider, fast pneumatic_3d tyre |
| Configuration | `examples/spatial/performance_fast.toml` |
| Terrain input | `examples/spatial/performance_flat.toml`: 200 m flat, ±6 m wide, 0.5 m grid spacing |
| Demand | W36 F `examples/research/spatial_flat.toml`; final demand held to 30 s |
| Initial speed / position / seed | 3 m/s; x=3 m; 73421 |
| dt / controller interval | 0.00125 s / 0.005 s |
| Sensor mode | Native mounted truth/IMU; research delivery/noise pipeline absent |
| Recording / processes | Retention off, synchronous accounting, one process |
| Tyre / engine threads | Serial default 1 / 1; explicit experiments below |
| Source Git SHA / dirty payload | `null`, pending actual checkout identity |
| Native source/executable/config/terrain/calibration digests | `null`, pending actual build and prepared model |
| MuJoCo / provider / compiler / build flags | `null`, pending consumed execution identity |
| Hardware / OS / thermal/load policy | `null`, pending per-platform measurement |

The CLI resolves the source profile and records actual root/tier, terrain grid
dimensions, dt, thread counts and source/config/calibration/provider identities.
The driver uses owned native startup and W33 outcomes/exact completion, with no
planar finish position. Profile names alone do not authorize comparison of
different workloads. The source inputs are provisional until physical V06/V07
qualification; this document does not claim that the 30 s episode succeeds.

## Measurement contract

At least one complete strict warmup and three complete measured runs share one
identity. Each run includes all advance calls and final flush in its wall
interval. Setup and export are reported separately; warmups are retained but
excluded from the median. Early fall, invalid state, changed identity,
non-strict policy and measurement failure remain diagnostic. A single run,
missing instrumentation or insufficient duration cannot set `target_met`.

The 30 s window is checked against complete ticks × dt after checking actual
clock consistency. The observed simulated time is used unrounded in RTF;
`sim/wall >= 2.0` is inclusive without tolerance on that threshold. Cooperative
chunk latency is not per-step latency. No report closes physical, replay,
installed-product or real-display qualification on the strength of timing.

Region scopes are exclusive and owner-local. Worker dispatch/barrier cost is
inside the terrain/tyre region; workers do not update shared timing counters.
The clock-read estimate is a separately labelled component of instrumentation
overhead, never subtracted from wall time. `--no-region-timing` supplies a
collector-disabled control with the same physics; its observer-mode identity
differs and `target_met` remains null. V09 must measure the full incremental
collector cost, including bookkeeping, and document the residual disabled
scope branches. `instrumentation_total_overhead_s` remains null before that
experiment.

## Runs — all pending V09

| Platform | Configuration | Warmups | Measured runs | Median RTF | Validity / identity | Evidence |
|---|---|---|---|---|---|---|
| macOS arm64 | tyre1 / engine1 | pending | pending | null | pending | none |
| Linux aarch64 | tyre1 / engine1 | pending | pending | null | pending | none |
| Each platform | tyre2 / engine1 | pending | pending | null | pending | none |
| Each platform | tyre4 / engine1 | pending | pending | null | pending | none |
| Each platform | tyre8 / engine1 | pending | pending | null | pending | none |
| Each platform | Collector disabled control | pending | pending | null | pending | none |
| Each platform, real display | 1× visible viewer, recording off/on | pending | pending | null | pending | none |

No run rows or wall times have been fabricated. Save actual commands, source,
executable/input identities, exit codes and raw per-run results in V09/V10.
Record invalid outcomes alongside the valid sample set.

## Regions — all pending V09

| Region | Source measurement boundary | Seconds |
|---|---|---|
| Terrain/rays | Complete geometry query plus pool dispatch/barrier | null |
| Tyre elements | Carcass preparation/element work, owner patches/brush/reduction | null |
| Engine step | Actual step/forward work through the owned engine boundary | null |
| Rider/steer | Intent, steering, rider/controller work | null |
| Accounting | Interval close, capture/publication and final flush | null |
| Sensors | Raw reads observed; full delivery pipeline pending | null |
| Recording | Absent from headless source fixture; pending | null |
| UI | Real-display pacing/report path authored; pending V10 | null |
| Clock-read cost estimate | Cold calibration × observed read count | null |
| Full instrumentation overhead | Matched enabled/disabled full-run experiment | null |

The region sum is not asserted to cover all outer-loop instructions. Headless
pending flags are explicit for absent sensor delivery, recording and UI.
GUI summary records actual completed presentations, debt and frame intervals;
its final RTF also includes seal/flush/publication/runtime close. Pauses,
resets, changed scale, hidden/missing framebuffer, early close and non-strict
episodes are ineligible. The final publication timing is emitted on stdout,
after recording publication, not retroactively inserted into stored evidence.

## Decision — tyre pool

**decision: pending V09**. One thread remains the default. Source availability
does not imply pool selection.

The baseline is the same valid complete serial workload, built with the same
compiler/provider/flags. Selection requires exact N=1/2/4/8 state/force/sensor/
contact/checkpoint invariance and successful supported sanitizer controls.
It also requires a measured full-run gain after dispatch/barrier overhead:
either the candidate closes the serial deficit to headless RTF 2.0, or, for an
already qualified serial baseline, adds at least 20% throughput margin
(`candidate_median_rtf >= 1.20 * serial_median_rtf`). A candidate below the
physical/RTF gate is not selected simply because a component becomes faster.
Otherwise V09 records `not_selected` and removes unused product code as required
by the plan. Do not select a thread count from a single best run.

The pool owns N−1 persistent threads, preassigned contiguous ranges and fixed
statuses. Caller executes range 0. Readiness and generation barriers are
explicit; serial mode starts no worker. Owner alone computes shared pressure,
advances material, assigns patches, reduces sums and commits. Canonical index is
`((wheel * n_theta_fan) + longitudinal_index) * n_phi + lateral_index`.

W52 tests compare full W42 continuations. They first prove that complete runtime
configs differ only in `tyre_threads`, then normalize only that field's derived
`runtime_config_sha256` in comparison copies. Production checkpoint bytes,
restore validation and all causal state remain intact. Flat extrusion is an
unconditional fixture and currently encounters the prior missing surface-view
contract; C10/V05 must resolve it. It is not skipped or replaced by a flat grid.

Linux `linux-tsan` is a separate compiler/runtime lane with context provenance.
The intentional shared-cell race is an opt-in negative control; V09 must see
the actual data-race diagnostic and configured exit 66. Darwin TSan is rejected
explicitly. RTSan remains a separate gate; no hot `BIKE_NONBLOCKING` annotation
is justified by source review.

## Engine pool

**decision: pending V09**. No production engine-pool hook is added.

The test inspected the actual pinned MuJoCo 3.12.0 header, whose SHA-256 matches
the lock: `c7378e471cb951f4ff4bd4801139be41495a3df82c5ae643c4134ded850fb7ef`.
It declares `mjVERSION_HEADER 3012000` and
`MJAPI void mju_threadpool(mjData* d, int nthread);`.
The exact lock/source references are recorded in `engine_pool_probe.cpp`.
The parameter is the number of background workers; the caller participates.
`mju_threadpool(d, 0)` removes the pool, and the existing `mjData` owner tears it
down on data destruction. This is header/source evidence, not execution evidence.

The probe compares tyre1/engine1, tyre4/engine1, tyre1/engine4 and tyre4/engine4.
All use one serial-settled W36 F bootstrap. Each combination gets one warmup and
three complete 30 s measured runs in rotated order. Separate diagnostic runs
compare full state at 0/5/10/15/20/25/30 s, outside scored timing. Final flush is
included; cold construction and state serialization are separate.

Selection requires the candidate complete-run wall time to be at most 90% of
the matched CPU baseline **and** exact complete-state equality. Both tyre and
engine configurations must have valid workloads. An invalid or non-invariant
CPU baseline is a blocked decision, not a positive engine result. JSON evidence
only recommends V09 review; production hook/selection is a later decision.

## Verification handoff

Source-authored targets: `spatial.performance_report`, `spatial.pacer_report`,
`spatial.worker_ranges`, `spatial.worker_pool`, `spatial.thread_invariance` and
`spatial.engine_pool_probe`. The latter two are `slow`; the engine probe is also
`perf`. None has been compiled or executed for this delivery.

See [usage and future commands](../../perf.md),
[C10 deviations](../../../migration/spatial3d/contract-deviations.md), and the
[W50–W55 source report](2026-10-24-spatial-w50-w55-source.md). V09/V10 must resolve
predecessor physical gates and candidate identity before promoting any result.
