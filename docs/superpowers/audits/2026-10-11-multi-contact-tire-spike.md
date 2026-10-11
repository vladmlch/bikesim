# Multi-contact tyre spike — `distributed_2d_reference` measurements

Phase 0 of `docs/superpowers/plans/2026-10-11-multi-contact-tire.md`. Measurement
session on the Python backend only; no production files changed. Artifacts
(profiles, JSON, telemetry, scripts) live under `output/mct-spike/` and are not
committed.

## 1. Setup

| Item | Value |
|---|---|
| Commit | `71f3aeec279c5b7b3fb006ae96314e2aa182b987` (branch `feat/native-distributed-tire`) |
| Platform | macOS 27.0.1 arm64, Python 3.14.3, MuJoCo 3.12.0, NumPy 2.5.2 |
| Profile | `output/mct-spike/physics_distributed.toml` = `examples/research/viewer_physics_welded.toml` with **one-line diff**: `[tires] backend = "compliant_2d"` → `"distributed_2d_reference"`. `surface_mode = "track"`, `mu = 1.1` both wheels. No `[tires.distributed]` table → synthetic `HingeDensity()` defaults (`knots_m=(0,.004,.012)`, `stiffness_n_m=(300000,0,0)`, `damping_ns_m=2000`), `station_count=128` |
| 64/256 variants | same profile + `[tires.distributed] station_count = 64` / `= 256` appended |
| Track presets | `alpine_scree` (seeded preset, seed=41, 120 m), rocky track = `--scenario alpine --gen-spec examples/research/alpine_rocky_genspec.toml --seed 0` (120 m, `single_contact_safe = false`) |

Load check (Task 0.1): the distributed profile loads, the plant builds, and
`summary.json → resolved_config.physics.tires.backend` resolves to
`distributed_2d_reference` with `station_count = 128` and the synthetic density
table above. The flat smoke run (`--duration 0.2`) ended at 0.160 s with reason
`numerical_quality` — see methodology note M2.

### Methodology notes (deviations from the brief, all recorded, none blocking)

- **M1 — asset paths.** The profile references `rider_joint_envelope_anatomical.json`
  / `rider_strength_reference.json` relative to the profile directory; both files
  were copied into `output/mct-spike/` (content unchanged). First run without
  them failed with `No such file or directory: .../rider_joint_envelope_anatomical.json`.
- **M2 — platform `energy.constraint_work` baseline violation.** Every run on
  this platform logs `invalid reference step at 0.005000000: ('energy.constraint_work',)`
  (first_model_violation interval_id 3, t=0.00375 s, every 5 ms control interval
  thereafter — 4000 counts per 20 s run). In strict headless mode this truncates
  all runs at step 4 (0.005 s) exactly as `docs/RIDE.md` documents. It is
  identical across backends and is therefore a constant offset, not a tyre
  signal; it does make `model_valid=false` for every run below.
- **M3 — throughput fallback.** The prescribed `tools/measure_realtime.py`
  strict runs produced only the 4-step prefix (median "RTF" 0.499 compliant /
  0.291 distributed over 0.005 s — recorded in `rtf-compliant/`, `rtf-distributed/`,
  meaningless as throughput). The interactive `bike-ride --time-scale 1` HUD
  fallback is not observable from an automated session, so the same
  `measure_run`/`build_physical_driver` whole-boundary machinery was driven with
  `strict=False` (`output/mct-spike/measure_nonstrict.py`): violations are still
  recorded (and are what sets `valid_prefix=False`), they simply do not abort
  the run. Every measured run below completed the full 8000 steps /
  10 s simulated with outcome `duration_reached`. The strict flag is explicit in
  each run's `comparability` record.
- **M4 — `--energy-tolerance 0.5` for the ride runs.** The prescribed Task 0.4
  commands truncated both backends at ~0.13–0.15 s / ~2 m (still on the flat
  rollout, before any rock) with reason `numerical_quality`:
  `max_energy_residual_ratio` crossed the default 0.05 gate at 0.050–0.051 —
  the same bookkeeping offset as M2, not a rock-induced blowup. Runs repeated
  with `--energy-tolerance 0.5` so the actual residual is measured instead of
  the run dying at the gate. Both sets of outcomes are reported.
- **M5 — extra measurements beyond the brief** (marked "extra" below):
  stand matrix also run at `station_count=64`; a 55 s distributed rocky run to
  push past the first scree band; `episode-0000/summary.json` is actually
  `<out>/summary.json` (no episode subdirectory in this revision);
  `resolved_config.tires` is nested under `resolved_config.physics`.
- **M6 — exit codes.** `bike-research` exits 2 whenever the outcome is
  truncated/invalid, including successful `duration` runs flagged
  `model_valid=false`. Exit 2 below means "not a clean-valid run", not
  "the run failed".

## 2. Stand acceptance matrix (Task 0.2)

`contact_matrix(dts=(0.00125, 0.000625), station_counts=(128, 256), flat_loads=(300, 600, 900), step_dxs=(0.01, 0.005))`
→ **24 / 24 accepted**. Extra: `station_counts=(64,)` same grid → **12 / 12 accepted**.

| Case | dt (s) | stations | Result | Key metrics |
|---|---|---|---|---|
| flat ×3 loads | 0.00125, 0.000625 | 64, 128, 256 | all pass | `force_relative_error` = 0.000000, `energy_residual_ratio` = 0.000000 at every load/dt/count |
| step ×2 dx | 0.00125 | 64 | pass | `max_patch_count` = 6, `multi_support_fraction` ≈ 0.26, `energy_residual_ratio` ≤ 0.0073 |
| step ×2 dx | 0.00125 | 128 | pass | `max_patch_count` = 10, fraction ≈ 0.32, residual ≤ 0.0027 |
| step ×2 dx | 0.00125 | 256 | pass | `max_patch_count` = 21, fraction ≈ 0.35, residual ≤ 0.0033 |
| step ×2 dx | 0.000625 | 64/128/256 | pass | same patch counts; residual ≤ 0.0035 (halved dt halves residual) |
| incline 15° | both dts | 64, 128, 256 | pass | `normal_relative_error` = `vertical_relative_error` = 0.000000 |

`native_wheel_contact_count` = 0 everywhere — the hfield geom never touches the
wheel natively; all force comes through the distributed model.

### Road traces, dt = 0.00125, n = 128, dx = 0.005 (Task 0.2 step 2)

| Case | energy_residual_ratio | max_patch_count | multi_support_fraction | native_wheel_contact_count |
|---|---|---|---|---|
| two_support | 0.0023 | 11 | 0.4625 | 0 |
| recontact | 0.0062 | 9 | 0.3719 | 0 |
| bump | 0.0018 | 11 | 0.0000 | 0 |
| edge_unload | 0.0059 | 11 | 0.2812 | 0 |

## 3. Throughput (Task 0.3)

Non-strict whole-boundary measurement (M3), `alpine_scree`, 10 s × (1 warmup +
3 runs), dt = 0.00125 s (8000 steps), decimation 80, `--backend python`. All runs
completed the full duration; `valid_prefix=false` only via the M2 violation.

| Backend / station count | Median RTF | Runs (RTF) | Wall per 10 s sim | ≈ per-step cost |
|---|---|---|---|---|
| `compliant_2d` (welded) | **0.579** | 0.592 / 0.576 / 0.579 (warmup 0.604) | ~17.2 s | ~2.15 ms |
| distributed n = 64 | **0.415** | 0.415 / 0.414 / 0.415 (warmup 0.407) | ~24.1 s | ~3.01 ms |
| distributed n = 128 (default) | **0.327** | 0.322 / 0.327 / 0.333 (warmup 0.334) | ~30.6 s | ~3.83 ms |
| distributed n = 256 | **0.240** | 0.242 / 0.231 / 0.240 (warmup 0.243) | ~42.0 s | ~5.25 ms |

- Ratio distributed@128 / compliant = **0.565** (~1.77× slower).
- Tyre marginal cost ≈ +0.86 ms (n=64), +1.68 ms (n=128), +3.10 ms (n=256)
  per step over the compliant baseline — close to linear in station count
  (~12 µs per station-step across both wheels).
- Cheapest **stand-passing** station count: **64** (12/12 stands, RTF 0.415);
  the declared default 128 passes at RTF 0.327 and resolves ~2× more patches
  on step geometry (10 vs 6 stand patches; up to 17 rear patches in the ride —
  see §4).

## 4. Rocky alpine track (Task 0.4 step 1)

`--scenario alpine --gen-spec examples/research/alpine_rocky_genspec.toml --seed 0`,
`--diagnostic-model-limits`. Track features along the distance axis:
17.5 cm pothole @11.4 m, scree band 14.8–28.3 m (2.7 cm amp), 11.9 cm pothole
@35.0 m, 10.4 cm bump @38.2 m, 4.2 cm square-edge ledge @42.5 m, scree
45.8–53.2 m (2.2 cm), 16.9 cm pothole @55.1 m, rock garden 57.7–64.6 m
(7.1 cm amp), 7.2 cm ledge @68.1 m.

### Prescribed commands (default `--energy-tolerance` = 0.05)

| Backend | Outcome | Sim time | Distance | Note |
|---|---|---|---|---|
| compliant | `numerical_quality` | 0.130 s | 2.01 m | residual crossed 0.050 during flat rollout |
| distributed | `numerical_quality` | 0.150 s | 2.01 m | same gate, residual 0.0506 |

Both truncated before reaching any rock — a marginal crossing of the default
gate during rollout (M4), not a terrain event.

### Diagnostic reruns, `--energy-tolerance 0.5`, 20 s

| Metric | compliant_2d | distributed_2d_reference (n=128) |
|---|---|---|
| Outcome / terminal reason | `duration` (20.0 s) | `duration` (20.0 s) |
| Distance reached | **55.27 m** | **29.54 m** |
| `model_valid` | false | false |
| `numerically_valid` (≤0.5 gate) | true | true |
| First invalidity reason | `energy.constraint_work` @3.75 ms (M2, both) | `energy.constraint_work` @3.75 ms |
| Tyre-scope violations | **`front:multi_support` 13 + `rear:multi_support` 63 control intervals**; `material_load_range` 2 front + 73 rear | **none** (multi-contact is in-scope) |
| Multi-contact evidence | 1 patch max per wheel; `supports_multiple` = 0 | max active patches **13 front / 14 rear**; `supports_multiple` = 1 |
| `multi_support` fraction (0.1 s samples) | 0.0 % front / 1.0 % rear | 1.5 % front / 4.0 % rear |
| Max normalized energy residual | 0.1747 | 0.1620 |
| Rear slip distance / peak rear slip | 5.25 m / 1.62 m/s | 28.69 m / 3.36 m/s |
| Mean front load share (telemetry-derived *) | 0.290 | 0.284 |
| Wall time (≈) | ~90 s | ~70 s |

\* Mean front load share here is a telemetry-derived mean over sampled
contact intervals; the episode `front_load_fraction_mean` summary field
(0.262 compliant / 0.277 distributed) uses a different aggregation — both
put the front/rear ratio within ~4 % of each other.

Compliant `multi_support` violations fire where expected — sampled true at
x ≈ 21.2 m (inside scree band 14.8–28.3) and x ≈ 46.7 m (inside scree
45.8–53.2). The distributed tyre's sampled multi-support patches cluster at
x ≈ 11.4–27.7 m: the 17.5 cm pothole edge and the first scree band.

### Extra: distributed 55 s run (n=128, same track/flags)

Reached **47.11 m** — crossed both potholes (17.5 cm, 11.9 cm), the 10.4 cm
bump, the 4.2 cm ledge and ~32 m of scree; still short of the formal
`rock_garden` obstacle at 57.7–64.6 m (progress ~0.6–1.5 m/s — the tyre is
traction-limited, not terminated). Rear `multi_support` in **118 / 550**
sampled intervals (21 %), front 9; max patches **13 front / 17 rear**; residual
still ≤ 0.162; still zero tyre-scope violations; one short wheelie (0.016 s),
two_wheels contact state 97.5 % of the run.

## 5. Safe-track agreement — `alpine_scree`, 20 s (Task 0.4 step 2)

Same flags as §4 (`--track-file examples/research/alpine_scree.toml`).

| Metric | compliant_2d | distributed_2d_reference (n=128) | Ratio D/C |
|---|---|---|---|
| Distance reached (20 s) | 58.02 m | 28.63 m | **0.49** |
| Mean front load share (telemetry-derived, as §4 *) | 0.256 | 0.266 | 1.04 |
| Peak rear slip | 1.239 m/s | 3.394 m/s | **2.74** |
| Rear slip distance | 4.46 m | 28.87 m | **6.5** |
| Max energy residual ratio | 0.1747 | 0.1617 | 0.93 |
| Tyre-scope violations | `rear:material_load_range` ×11 | none | — |
| Wheelie episodes | 2 | 0 | — |

Agreement is **poor on traction outputs**: the distributed rear tyre slips
~2.7× harder at peak and ~6.5× in cumulative slip distance, halving progress on
identical geometry/mu (1.1)/controller. Front load share agrees within 4 %.
The energy residual is comparable (and is dominated by the M2 platform offset
in both). Both tyres see the same terrain; this is a tangential-model
difference, not a track difference.

## 6. Observations

1. **The model works.** 36/36 stands pass at all tested counts (64–256);
   energy residuals stay ≤ 0.73 % on the stands and are dt-consistent (halving
   dt halves residual). Up to 17 simultaneous patches on the real track with
   zero native contacts — genuine multi-contact, no silent fallback.
2. **It is in-scope where compliant is not.** On the rocky track compliant
   accumulates 76 multi_support + 75 material_load_range interval violations;
   distributed accumulates none. The claim "rides where compliant_2d is flagged
   out of scope" is supported for every feature up to the 55 s run's 47 m mark
   (17.5 cm potholes, 10 cm bump, ledges, scree). The named `rock_garden`
   obstacle at 57.7 m was not reached in-run — the limit is traction-driven
   progress, not stability or termination.
3. **Traction disagreement is the main risk item.** On safe geometry the
   distributed rear tyre spins roughly twice its ground speed during climbs
   (28.9 m slip over 28.6 m). Same `mu`, same `_brush_step` law per station —
   the difference comes from normal load spread across patches/stations. Until
   this is understood, the multi-contact tyre's traction answers are not
   interchangeable with `compliant_2d` evidence. This also explains the slower
   ride progress (0.49× distance), fewer wheelies (front stays planted when the
   rear spins), and why the bike never reached the rock garden.
4. **Throughput is adequate for batch, not for interactive.** Python RTF 0.33–0.42
   at stand-passing counts ≥ 0.2 → headless research runs are practical
   (~30 s wall per 10 s sim at n=128); interactive 1× viewing is not (3× too
   slow even at n=64). Cost scales ~linearly with stations: ~12 µs/station-step.
5. **Platform caveats are constant, backend-independent offsets.** The step-4
   `energy.constraint_work` violation and the ~0.05 rollout residual floor hit
   compliant and distributed identically; they gate `model_valid`/`numerically_valid`
   on every run of either backend. Separating platform bookkeeping from tyre
   signal needs the M3/M4 non-strict/tolerant path or a platform fix.
6. **Exit code 2 is not a crash** — `bike-research` uses it for any
   truncated/not-fully-valid outcome, including successful `duration` runs.

## 7. Recommendation

Outcome criteria from the plan (Phase 1 table):

| Row | Criterion | Evidence |
|---|---|---|
| A | stands fail at dt=0.00125 for all counts | **not met** — 36/36 pass at 64/128/256 |
| A | rocky track still ends invalid on distributed | **not met** — full-duration rides, zero tyre-scope violations |
| A | safe-track agreement is poor | **partially met** — rear slip 2.7×/6.5×, distance 0.49× (load share agrees) |
| B | stands pass; rocky rides; Python distributed RTF ≥ 0.2 at cheapest passing count | **met** — cheapest passing n=64 @ RTF 0.415; n=128 @ 0.327 |
| C | (decision taken) | — |

**Matched row: B** (all three B criteria hold), with row A's safe-track trigger
also firing on traction outputs — that disagreement goes into the port's risk
list, it does not reverse the taken C decision.

For Plan A (`2026-10-11-distributed-tire-a-portable-reference.md`):

- **`station_count` default: keep 128.** It is the declared synthetic material
  default, passes all stands with ~1.7× the patch resolution of 64 (10–17
  patches vs 6 on steps), and RTF 0.327 is still comfortably ≥ 0.2. n=64
  (0.415) is a documented throughput fallback; n=256 (0.240) buys stand margin
  the terrain evidence does not need.
- **Port risk list (additions):** (a) traction/slip divergence vs `compliant_2d`
  on identical geometry — mechanism unknown, suspect per-station brush
  saturation under distributed normal load; (b) progress on rocky terrain is
  traction-limited (~0.6–1.5 m/s) so ride-based parity evidence needs ≥ 60 s
  runs or a fixed-seed track excerpt; (c) platform `energy.constraint_work`
  step-4 violation is backend-independent — the native port's parity gate
  should treat it as a shared baseline, not a regression.
