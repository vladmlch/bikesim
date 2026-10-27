# Performance log

## spatial-fast-v1 — targets, pending V09

The native spatial fast profile targets realtime factors **2.0 headless** and
**1.0 viewer** on the plan's qualified platform lanes. These are acceptance
targets, not measurements. **There are no spatial performance measurements
before V09.** W50–W55 add measurement, workers, engine probes and qualification
tools as source only. No compiler, application or benchmark ran for this delivery.
The existing historical rows below must
not be presented as spatial results.

Qualification must preserve fixed terrain/calibration hashes, root/tyre tier,
dt, sensor schema, seed, execution identity, recording state and validity.
Report startup separately from scored intervals, exclude crashed/model-invalid
jobs from speed aggregates, and retain their outcomes. Process worker count
changes scheduling; canonical job keys/outcomes remain stable while elapsed
wall time is measured separately. The scenario definitions live in
[spatial-3d.md](spatial-3d.md). V07 physics and V08 delivery acceptance precede
any claim that a diagnostic throughput value represents qualified physics.

### Native spatial measurement protocol

The [performance audit](superpowers/audits/2026-10-22-spatial-3d-performance.md)
records the pending measurements and predeclared CPU-pool decisions. Final
release evidence belongs to the
[qualification ledger](superpowers/audits/2026-10-22-spatial-3d-qualification.md),
whose candidate identity and all milestones/gates remain pending.

The following are **future V09 commands, not executed results**:

```sh
bike-measure-realtime --profile spatial-fast-v1 --threads 1 \
  --duration 30 --warmup 1 --runs 3 --out output/spatial-serial
bike-measure-realtime --profile spatial-fast-v1 --threads 4 \
  --duration 30 --warmup 1 --runs 3 --out output/spatial-tyre4
bike-measure-realtime --profile spatial-fast-v1 --threads 1 \
  --duration 30 --warmup 1 --runs 3 --no-region-timing \
  --out output/spatial-observer-control
```

`spatial-fast-v1` selects the free-root fast tyre configuration in
`examples/spatial/performance_fast.toml`, the flat genspec beside it, seed
73421, start x=3 m and the W36 F demand trace in
`examples/research/spatial_flat.toml`. Setup/settle uses the native owned
startup path. The cold driver also accepts a complete owned `SetupRequest`
directly, without constructing a CLI or research session. Overridden physical
inputs remain explicit in the canonical profile/configuration/terrain/demand
identities; a matching profile name alone is not qualification.

Spatial termination uses W33 outcomes, corridor/domain checks, a complete-run
wall watchdog and the requested physical step. It does not use planar finish
position or export `terrain_vertices.npy`. The wall interval includes every
advance call and the final flush; setup and export are separate fields.
Warmups are retained and excluded from the median. An early fall, invalid
model, identity change, non-strict `--benchmark` or failed run remains
diagnostic and is excluded from the valid-episode median. Incompatible
warmup/run identities are rejected. Fewer than one valid warmup or three
complete measured runs of at least 30 simulated seconds produces
`target_met: null`. Threshold comparison uses the unrounded simulated/wall
ratio. `full_acceptance_verified` remains false even if the timing target is met.

`regions` reports exclusive owner-side wall time for terrain/rays, tyre work,
engine step, rider/steer and accounting. Dispatch/barriers are included in the
responsible region. Raw mounted IMU reads can be timed, but the absent research
sensor pipeline, container recorder and UI retain their pending flags. Region
totals are not asserted to cover every instruction of the outer wall interval.
`chunk_latency` is seconds per cooperative advance call, not per-step p95.

`instrumentation_overhead_s` is explicitly the estimated clock-read cost,
included in full wall time and never subtracted from RTF. The total observer
overhead is pending until V09 compares complete collector-enabled and
`--no-region-timing` runs with otherwise identical identity. The latter still
contains disabled scope branches, reports `instrumentation_available: false`
(its region seconds are zero placeholders, not measurements) and cannot close
the instrumented profile gate. Its raw full-run timing is an overhead control.
The regular `--compare` operation rejects either a thread-count or observer-mode
change as the same-run protocol; those deliberate experiments are adjudicated
in the performance audit after proving the only input difference.

### Threads, invariance and engine probe

`--threads N` selects **tyre threads** in the range 1–64, default 1. N=1 creates
no background tyre workers. Batch processes and internal engine threads are
independent parameters. Workers write assigned geometry/normal slots; pressure,
material history, patches, brush, sums and commit remain on the owner in the
same order. No hot-path `BIKE_NONBLOCKING` annotation or race-free claim is made.

V09 must execute the N=1/2/4/8 full-state, force, sensor, contact and checkpoint
tests and Linux TSan positive/negative controls. The `linux-tsan` preset is an
isolated build lane; Darwin TSan remains explicitly unsupported. The negative
control is opt-in and intentionally absent from ordinary CTest:

```sh
TSAN_OPTIONS=halt_on_error=1:exitcode=66 \
  build/linux-tsan/native/tests/spatial/bike_spatial_worker_pool --tsan-race-control
```

Accept that control only with a real TSan data-race diagnostic **and** its
configured exit code 66; a nonzero exit by itself is not evidence. The default
CTest invocation exercises safe callbacks and lifecycle contracts.

`spatial.engine_pool_probe` is labelled `perf;slow` and uses the locked MuJoCo
3.12.0 API `mju_threadpool(mjData*, int)`. It measures four tyre/engine
combinations and compares complete checkpoint states. There is no product
engine-pool hook. Pool selection, overhead, same-build invariance and both
platform results all remain pending V09. The existing extrusion material-view
contract conflict intentionally blocks that full invariance case until its
upstream C10/V fix; no passing skip replaces it.

### Viewer evidence

Spatial physical rides emit `viewer_performance` in their final stdout summary.
It reports achieved RTF, debt, actual successfully presented frames, frame rate
and inter-presentation lateness. Setup and post-terminal idle are excluded;
seal/flush, preview finalization, recording publication and runtime close are
reported separately and included in a second complete-time RTF. Plot export is
separate. This final report is emitted after publication, so stored episode
summaries are not retroactively rewritten with a fabricated publication time.

Pause, time-scale changes, reset, hidden/missing framebuffer, premature close,
non-strict policy or an incomplete/invalid episode make the 1× timing claim
ineligible. Rendering smoke and pacing arithmetic tests do not replace real
display acceptance on both platform lanes in V10.

### Historical planar/Python measurements

Standard measurement: `tools/measure_realtime.py --no-strict --duration 10 --runs 3 --warmup 1`,
track `rough_uphill_savage`, physics `viewer_physics_welded.toml` (dt 1.25 ms), this Mac.
`per-step` is from `tools/step_profile.py` (1600 loop steps, construction excluded).

Note: the dated rows below were measured on the python-only bikesim_new tree
that this branch ports from; numbers on this branch may differ.

| Date | Commit | Change | median RTF | per-step ms |
|---|---|---|---|---|
| 2026-10-11 | dfe4da1 | python-only baseline | 0.513 | 2.101 |
| 2026-10-11 | 8591bb1 | validate specs at construction, not per call | 0.586 | 1.949 |
| 2026-10-11 | dacc33b | clone drivetrain policies instead of deepcopy per step | 0.572 | 1.931 |
| 2026-10-11 | 70a7a42 | freeze force arrays once per step; cache contact vectors; drop scalar numpy overhead | 0.674 | 1.786 |
| 2026-10-11 | b614c5d | energy_state + mass_observations only on full samples (decimation-gated telemetry) | 0.729 | 1.725 |
| 2026-10-11 | 34129dc | skip apply_forces entry mj_forward in running loop (3.25->2.25 calls/step) | 0.710 | 1.70 |
| 2026-10-11 | dacf00d | numba njit(cache) on directional_capacity/_brush_step/_normal_contact/_box_pad_contact | 0.774 | 1.54 |
| 2026-10-12 | e3b1d9e | impl/cpp-port-p2 post-port measurement (this branch); note: this repo's measure_realtime.py has no --no-strict flag and every run aborts at step 4 (reason=invalid_controller, pre-existing `energy.constraint_work` reference violation at t=0.005s — same on base d52ed38), so median RTF is a 4-step prefix figure; per-step is the honest 1600-step number | 0.576 | 1.609 |
