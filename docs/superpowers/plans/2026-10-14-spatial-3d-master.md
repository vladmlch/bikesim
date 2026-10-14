# Spatial (3D) Ride Model — Master Implementation Plan

> **For agentic workers:** This master plan is the index and the dependency graph. Execute the phase plans (`2026-10-14-spatial-3d-p0…p8-*.md`) task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. Phases are agent-neutral — any agent runtime may take a phase. **Prerequisites:** `feat/native-pneumatic-tire` merged into `main` (the 3D tyre reuses its C++ kernels); `docs/agents/native-engineering.md` read; `docs/adr/0002-spatial-3d-correctness-method.md` accepted by the user.

**Goal:** A `spatial = "3d"` physical mode — free 6-DOF chassis, steering, rider balance by steering torque, a new C++ `pneumatic_3d` tyre with lateral forces and camber, laterally varying terrain, a 6-axis IMU observation — that passes the Meijaard 2007 benchmark, the steady-cornering check and the planar regression bridge, runs ≥ 1× interactive and ≥ 2× headless on the fast tier, with bytes independent of thread count, while every planar test stays bitwise green.

**Architecture:** One `spatial` switch in the physics config selects the root kind in the builder and a `topology::Spatial` descriptor in native; planar code paths are preserved byte-identical behind that descriptor. New native units: `terrain/grid3d`, `writers/pneumatic_3d_tire` (+ `pneumatic/element_adapter`), `rider/steer_balance`, `runtime/equilibrium`, `runtime/bootstrap_hold`, `runtime/worker_pool`, IMU sample fields. Python gains `terrain/generator3d`, observation schema 2, catalog fields, viewer camera/HUD. Correctness per ADR 0002.

**Tech Stack:** C++23 (Apple clang primary, GCC sweep), nanobind, CMake, MuJoCo 3.12.0, Accelerate CBLAS/LAPACK; Python ≥ 3.13 (venv 3.14) + pytest; `uv run --frozen --group native …`; `bash tools/run_tests.sh {quick|native|full}`.

**Spec:** `docs/superpowers/specs/2026-10-14-spatial-3d-design.md`. **ADR:** `docs/adr/0002-spatial-3d-correctness-method.md`.

## Global Constraints (apply to every phase plan)

- **Base and branches.** Base: `main` after the merge of `feat/native-pneumatic-tire` (and `feat/alpine-scree-track` if it lands first — it touches `terrain/generator.py`). Each phase on its own branch `feat/spatial-3d-p<N>-<slug>`, merged to `main` at its gate. Parallel tracks rebase on `main`, never on each other.
- **Planar mode is sacred.** `spatial = "planar"` is the default; no existing TOML, golden test or planar byte changes. Every `tests/reference/test_native_*.py` stays green at every task of every phase. A shared-code refactor is accepted only with the planar bitwise gates passing; if a refactor cannot be made byte-preserving, it is split behind `topology::Spatial::planar3`.
- **No Python twin for 3D physics.** Correctness is ADR 0002's four layers. Do not add `*_py` twins or `_pin_oracle_kernels` entries for 3D kernels. Do add: unit/property tests, invariants, benchmark fixtures, self-golden manifests.
- **No forces from the world during a scored episode.** The bootstrap hold (spec §8.2) exists only while bootstrap time `t < 0`; tests assert `bootstrap.hold_active = 0` and zero hold work on every scored step, and a hold wrench below 1 N / 1 N·m at `t = 0⁻`.
- **Determinism.** Bytes identical run-to-run and for every thread count. No atomics on floats, no dynamic scheduling, fixed-order reductions (`numeric::numpy_pairwise_sum` or an equivalent fixed tree). `-ffp-contract=off` stays.
- **Native discipline.** Warm cores `noexcept BIKE_NONBLOCKING`, allocation-free, scratch sized at construction from validated config; RTSan (`NATIVE_RTSAN=ON`) and ASan builds pass; two compilers; every `check_<name>` sweep green; a missing sweep tool fails the run — install, never skip.
- **Verification profile per task:** Python-only tasks `quick`; any native or bindings/config-projection task `native`; physics/realtime tasks `full` at the phase gate. Warm up Numba kernels before any timing.
- **Units and frames.** SI; ISO 8855 (x forward, y left, z up) for derived channels; MuJoCo convention for raw `qpos/qvel`; every new telemetry field declared in `recording/catalog.py` with unit and frame.
- **Interface contracts** (spec §5) are frozen at the end of Phase 0. A change needs a row in the "Contract changes" table below and a note in the affected phase plans before implementation.
- **Commits.** One commit per task, message `<type>: <summary>` as in the repository history; use whatever co-author trailer your agent runtime prescribes (plans are agent-neutral). Never commit generated `output/` artifacts except the small sha256 manifests under `tests/reference/golden_manifests/`.
- **Documentation.** `docs/RIDE.md` capability sentences, `CONTEXT.md` terms and `docs/TESTING.md` profiles are updated in the phase that introduces the capability; an audit file `docs/superpowers/audits/<date>-spatial-3d-p<N>.md` closes each phase with evidence (tables, RTF, residuals).

---

## Phases, tracks, dependencies

| Phase | Track | Plan | Depends on | Est. agent-days | Gate (merge criterion) |
|---|---|---|---|---|---|
| **P0** Foundation | — | `…-p0-foundation.md` | — | 3 | ADR 0002 accepted; 5 contracts (headers + stub tests) merged; literature pins (`meijaard2007.toml`, `tyre_lateral_reference.json`) with provenance; `golden_manifest.py`; doc repairs; `native`/`quick` green |
| **P1** Native topology generalisation + constrained root | A | `…-p1-native-topology-4dof.md` | P0 | 6–8 | All planar `test_native_*` bitwise; `spatial="3d", root="constrained"` builds and runs with planar backends disabled; **planar regression bridge passes** (ADR 0002 Layer C) with a laterally flat extruded track and a temporary `pneumatic_3d` stub = flat-road planar law; lateral-zero bitwise test |
| **P2** 3D terrain | B | `…-p2-terrain-3d.md` | P0; P1 Task 4 (builder `spatial` argument) for its Task 3 | 4 | `generator3d` seeded reproducibility; `.terrain3d` round-trip; `Grid3D` sample/ray-cast unit + property tests; render hfield; planar extrusion byte-equals the 1D profile along y = 0 |
| **P3** `pneumatic_3d` tyre | C | `…-p3-pneumatic-3d-tyre.md` | P0 (contracts 5.1, 5.2); P1 and P2 for integration tests (Tasks 5–8) | 9–12 | Flat-road consistency 2 %; T1–T4; lateral/camber rigs inside literature bands and ≤ F_z·tan γ; state wire round-trip; RTSan; thread-partition hooks in place (single-thread) |
| **P4** Free root, steering, balance, C++ equilibrium | D | `…-p4-free-root-steer-balance.md` | P1, P3 | 8–10 | **Meijaard gate** (3 % / ±0.2 m/s); steady cornering (1° / 2 %); bootstrap hold zero in scored steps; equilibrium cache 3D fingerprint; `crash:capsize`/`off_corridor`; mirror-symmetry bitwise |
| **P5** IMU v2, research env, truth/metrics 3D | E | `…-p5-imu-research-3d.md` | P1 (P4 for free-root runs) | 4–5 | Observation schema 2 end-to-end with noise tape; `WheelieTruth` 3D fields; metrics; `bike-research` 3D episode on `alpine_rocky_3d` with the passthrough policy; planar research byte-identical |
| **P6** Frontends | F | `…-p6-frontends.md` | P4, P5 | 3 | Viewer chase camera/HUD at 1× on fast tier; checked replay 1e-9 on a free-root episode; CLI flags; `docs/RIDE.md` |
| **P7** Multithreading + performance gate | G | `…-p7-multithreading-perf.md` | P3 (P4 for the full model) | 5–6 | Thread-count invariance bitwise N ∈ {1,2,4,8}; RTSan-clean pool; **viewer ≥ 1×, headless ≥ 2× fast tier**; `mjThreadPool` measured; perf contract tests |
| **P8** GPU feasibility memo | H | `…-p8-gpu-feasibility.md` | — (day 1) | 2 | Memo + microbenchmarks + ADR 0003 go/no-go with pre-registered kill criteria |

**Total ≈ 44–53 agent-days (interview estimate ≈ 45–55).** Critical path: P0 → P1 → P4 → P6 (≈ 20–24 days). With three parallel agents: wall ≈ 3–4 weeks.

### Parallel schedule (suggested)

```
week 1 : P0 ───────┐        P8 (independent, day 1-2)
week 1-2:          ├─ P1 (A) ──────────────┐
                   ├─ P2 (B) ──┐           │
                   └─ P3 (C) ──┴──(needs P2 for integration)──┐
week 3  :                                  P4 (D) ◄───────────┘      P5 (E, after P1; free-root tests after P4)
week 4  :                                  P6 (F) ◄── P4, P5          P7 (G) ◄── P3 (full model after P4)
```

### Phase handoffs (what each phase must leave for the next)

- P0 → all: headers `native/src/terrain/grid3d.hpp`, `native/src/writers/pneumatic_3d_tire.hpp` (output/state structs), `nativecfg::SpatialConfig`/`Pneumatic3dConfig`, `SensorObservation` v2 dataclass, catalog field declarations; stub tests that fail with `NotImplemented` markers; pinned references.
- P1 → P4/P5: `topology::Spatial`, constrained root in builder + native, regression bridge test harness, extruded-track path.
- P2 → P3/P4/P5: `.terrain3d` writer/reader, `Grid3D`, the track files `examples/research/alpine_rocky_3d.toml` / `rough_uphill_savage_3d.toml` with `Track3dSpec` resolution in `terrain/research.py` (seeded, hash manifest), render hfield.
- P3 → P4/P7: `Pneumatic3dTireWriter` with element partition hooks, state wire, rigs.
- P4 → P5/P6/P7: free root, steer, balance controller, C++ equilibrium, capsize/off-corridor outcomes, Meijaard + cornering fixtures.
- P5 → P6: observation v2, truth/metrics, research CLI 3D.

## Contract changes

| Date | Contract | Change | Reason | Affected plans |
|---|---|---|---|---|
| 2026-10-14 (pre-freeze) | 5.1 | `Grid3D::from_profile_extrusion(profile, half_width)` added to the contract | regression-bridge path needs a grid before P2 | P0 Task 3, P1 Task 5, P2 |

## Open items carried into execution (not decisions — facts to measure)

- dt for the fast tier in 3D (expected 1.25 ms; measured in P4 Task "dt study").
- Default thread count and whether `mjThreadPool` helps (P7).
- Exact Meijaard Table 2 values and lateral coefficient bands are pinned from the sources in P0 (never typed from memory).
- Whether `float32` grid storage keeps slopes within 0.1 % (P2); fallback `float64`.

## Out of scope (recorded so nobody re-asks)

Anti-wheelie policy itself; C++ policy plug-in; legacy stack; lateral rider DOFs; curved trails/path following (optional later track); in-process batched episodes (optional later track after P7); standalone CLI; Linux/CUDA (no such machine; GPU memo is Metal only).
