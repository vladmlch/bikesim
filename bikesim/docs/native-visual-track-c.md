# Native visual/runtime frontends: Track C source delivery

Date: **2026-10-10**.

## Status and delivery boundary

Track C1-C4 implementation, regression test sources and usage documentation are
included. **No project code, import, compiler, build, test, benchmark, analyzer or
viewer was run for this delivery.** This is a source implementation, not a
verified release or an accepted real-time result. Static inspection cannot
establish compilation, numerical parity, GUI behavior or throughput.

The input is the supplied Track B source overlay. Its historical audit already
leaves A4/B executable verification pending. It omits `.git`, root packaging and
lock files, upstream verification tools and most older tests. Those omissions
are not filled with invented scaffolding or test results. Apply this delivery
to the corresponding complete checkout and retain the project's pinned
NumPy/MuJoCo environment and native numerical flags.

The source archive contains that supplied tree with the changes below. The
accompanying unified patch is against the exact supplied tree, with paths
`original/...` and `bikesim/...`; it is intended for `patch -p1` from the project
root in a later authorized session. Neither a native binary nor a new dependency
archive is included. A Track B binary is stale for the added interfaces.

## C1: Completed-step pacing and owned presentation

`src/bike_sim/sim/playback.py` defines the shared 1/2/4/8x clock. Debt is consumed
by actual completed physics steps, not by a requested target. Budget yields
retain remaining debt; pause, scale changes and resets rebase it. F6/F7/F8 do not
reuse existing brake, camera, telemetry or marker keys. The existing legacy
`RealTimePacer` is retained.

`PhysicalViewState` and `FrameSnapshot` carry owning values for the entire
historical physical HUD/preview schema, the latest accounted sample, full
integration state, outcome and validity. Native `PresentationState` caches
endpoint and writer/controller diagnostics in C++ at committed boundaries;
wire/Python dictionaries are constructed at snapshot requests, not used as the
physical runtime's state. Accounting-published saturation values are refreshed
without another force evaluation. The settled Python t=0 presentation is copied
at bootstrap so initial drawing does not insert an extra engine solve.

`RenderReplica` owns a different model/data pair. Applying a complete frame
validates its state and all six mutable model fields before copying, then calls
kinematic/geometry updates only. The fields are `dof_frictionloss`, `site_pos`,
`tendon_stiffness`, `tendon_damping`, `tendon_lengthspring`, `tendon_range`.
Rendering never runs `mj_step`, `mj_forward`, physical accounting or sensors.
Camera/marker changes affect the replica only. The frame remains retained
through viewer synchronization and independently of later reset/close.

The controller/model/timestep recipes are not changed. The supplied welded
profile remains at 0.00125 s. Requested playback scale is separate from measured
RTF. Compute slices use cooperative 8 ms budgets between physical steps; sync
requests are limited to 60 Hz. A single solve, external policy call, export or
GUI operation is not preemptible by this budget. No additional C++ region is
claimed to be allocation-free or real-time-safe.

## C2: Backend routes and frontend lifecycle

`bike-ride` now selects `--backend python|native`, defaulting to Python, for
physical and research paths. `bike-research` exposes the same selection but
remains headless. Explicit `--time-scale` is rejected for headless, legacy and
preview routes, including explicit 1x; absent means 1x for supported viewers.
The macOS viewer re-execution guard precedes expensive setup.

`sim/backend.py` preflights supported configuration and the selected native
artifact/source identity before equilibrium construction or output creation.
There is no fallback. Existing support remains articulated spindle/pin/connect,
track-material analytic compliant tires and the supported ideal mid-drive;
this delivery does not add another topology or numerical model.

`PhysicalRideDriver` selects the sole authoritative backend. Native finish
position is checked inside committed-step advancement, so a large target cannot
skip the track endpoint. Native recording retains decimated owned intervals as
well as columns after delivery queues are acknowledged. Frontends export full
accounted telemetry/intervals, terrain and summary, including committed prefixes
when advancement or final flush reports failure. Reset exports the outgoing
generation before starting a new recorder. Temporary bootstrap resources and
owners have explicit cleanup paths.

Research continues an external control transaction across wall-budget and
absolute-target yields. `PolicySession.begin_advance()` invokes the external
Python policy once; `advance_pending()` resumes that same command. Optional
`target_step` is implemented in the Python environment, native adapter and C++
runtime without flushing/publishing a new transition on a pacing yield. The
old no-target synchronous APIs remain. Live pause/brake/stop/reset are applied
at boundaries; window close finishes only its active window and saves it.
`close(discard_pending=True)` is available for error/replay cleanup without
simulating hidden work; default close still rejects an active window.

## C3: One checked replay session

Headless replay and `bike-replay --viewer` share `ReplaySession`. Schema 1 uses
Python; schema 2 uses its recorded backend. Manifest/source/runtime/artifact
identity and native capabilities are checked before reconstructing the plant.
The recorded policy path is never imported. Bundled parameter files replace
original external recipe paths.

The same session checks initial observation and integration state, requested
commands/times, transitions, observations, applied command events, terminal
outcome, model status, event/episode metrics and final integration state with
the existing 1e-9 numerical tolerances. It also checks source/runtime identity
again before publishing success. A recorded `not_finished` prefix is checked
as a prefix, without falsely labeling its episode terminal.

Wall and target yields resume one recorded command. The last control boundary
is not promoted to verified before final outcome/metrics/state checks pass.
On mismatch the viewer holds the previous verified frame; on success it holds
the final verified frame. Restart constructs a fresh checked environment and
rejects a changed manifest. No successful report exists for an incomplete
session. Viewer return codes distinguish pass (0), mismatch/error (1), and
unverified early close (2).

## C4: Opt-in measurement and explicit evidence

`tools/measure_realtime.py` and `sim/realtime_measurement.py` implement an opt-in
measurement route. They never configure or build an extension. The effective
profile timestep is retained unless `--dt` is explicitly supplied. Warm-up and
measurement counts default to one and three; the native Release headless
threshold field requires at least those counts and valid matching prefixes.

The measured outer wall interval includes all advance calls, loop/boundary
overhead and final accounting flush. Setup and export are separate. Percentiles
are labeled as whole-call chunk latency, with actual completed-step counts;
there is no inferred per-physics-step p95. Reports include source/runtime/native
identity, effective settings, controller periods, recording decimation,
strictness, outcome, prefix length, numerical/model validity and failures.
Comparison rejects different settings or classified prefixes. Matching invalid
prefixes remain diagnostic; shorter terminal prefixes are explicitly flagged.
`full_acceptance_verified` remains false: headless throughput alone is not the
complete Track C acceptance contract.

**This delivery has no measured RTF, timing, successful replay log or test count.**
Usage and later authorized commands are in `docs/RIDE.md` and `docs/TESTING.md`.

## Regression sources supplied, not executed

The added sources cover playback debt/scale/reset; the complete old HUD and
preview schema; owning state/model fields and kinematic-only rendering; CLI
selection/preflight; physical exports and native parity; one policy call per
partial external window; replay round-trip/tamper/restart/last-verified/early
close; and whole-loop measurement/comparability. Mock windows exercise lifecycle
and pacing but cannot substitute for a real macOS viewer.

`tests/reference/_physical_hud_oracle.py` preserves the supplied pre-C1 HUD
implementation for schema/format comparisons, rather than comparing the new
adapter to itself. Native regression fixtures use an explicitly selected
existing extension and do not build one. The historical native recording test
expectation was updated from the Track B replay rejection to the new checked
same-backend reconstruction path.

## Supplied reference sources used

Only the user's attached source archives were used for these reference checks;
no installed third-party implementation or online replacement was substituted.
Paths below are relative to the `untitled folder` root inside each reference
archive. The archives were read, not imported or built, and are not copied into
the delivery.

### NumPy

`numpy/_core/src/npymath/npy_math_internal.h.src`, lines 444-449, defines the
radian-to-degree factor and multiplication order. The C++ presentation helper
uses `x * (180.0 / pi)`, rather than changing it to `(x * 180.0) / pi`. This is a
specific operation-order reference, not a claim that unrelated transcendental
results are bitwise identical. Runtime parity remains untested.

File SHA-256:
`231d9baedb4be31d644898c78997bfe72b2093bc5ef1c827f4aa9fd553045640`.

### MuJoCo

- `include/mujoco/mjtype.h`: full `mjSTATE_INTEGRATION` membership, including user
  fields and warm start. SHA-256:
  `f20d165d2713a4432535850eea6ed08f8a7b471fc4c4d673f31d77ddca758886`.
- `include/mujoco/mujoco.h`: state transfer, model copying and geometry-only
  APIs. SHA-256:
  `422f3a88e4780f6a188803f58ab19595f22ca3c56cf7d9573d0f082d163fa9f2`.
- `src/engine/engine_support.c`: full-state layout and get/set behavior, including
  the leading time field. SHA-256:
  `2696a5847e10870cfb341085ee8e7bbab2a3bdccf8ed6fe1e4306dd86a555438`.
- `python/mujoco/structs.cc`: Python model copy wrapper behavior. SHA-256:
  `cb930f3dc4b93768b98f81b16793e9519f9031839e40a147f175d3ea590a47af`.

### Input archive identity

| Input | SHA-256 |
| --- | --- |
| `bikesim_track_B_source(1).zip` | `df7e615a4858a4544806be69e5e9be0beda1d4a327f71ef47142849c3b49426b` |
| `numpy.zip` | `478e5f5a6983d5fcf2f01c1157a6fbac2bf2a4273cdbe8d895a855ef36ab7312` |
| `mujoco.zip` | `57ad67969fc4a649be553dc7d4cb82169fa13301ee1f343fd0417fd217e59979` |

## Acceptance gates deliberately left open

| Gate | Delivery status |
| --- | --- |
| Fresh native Release configure/build/import | **NOT RUN** |
| Python/full/native correctness profiles and new regression sources | **NOT RUN** |
| Numerical state/accounting/HUD/recording parity | **NOT RUN** |
| Python/native replay round-trips and tamper cases | **NOT RUN** |
| ASan/UBSan and supported RTSan paths | **NOT RUN** |
| C++ static analyzer / IDE inspection | **NOT RUN** |
| One warm-up + three same-setting measured runs per backend; native median RTF >= 1 | **NOT RUN** |
| Real macOS viewer at 1x with mean achieved RTF >= 0.95 | **NOT RUN** |
| Real macOS elevated-scale controls, pause/reset/close and visual parity | **NOT RUN** |

The operations performed were source/archive inspection, source/document/test
editing, declaration/call-site and error-lifecycle review, reference hashing,
and delivery diff/whitespace/archive-content inspection. These are not
substitutes for the open executable gates above. No generated binary, timing
report or successful-test artifact accompanies this source delivery.
