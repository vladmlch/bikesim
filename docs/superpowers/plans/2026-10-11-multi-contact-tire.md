# Multi-Contact Tyre: Spike, Decision Gate and Port Outline

> **For agentic workers:** Phase 0 is executable as written (superpowers:executing-plans; it is a spike — nothing built in it is kept except the audit report). Phases 1–2 are a decision gate and a scoped outline: they MUST go through superpowers:brainstorming → spec → superpowers:writing-plans before any native code is written.

**Goal:** Decide, on measured evidence, how the project gets a tyre that can rest on several road facets at once (a real tyre wrapping a rock), so that alpine tracks can carry realistic rocks instead of the single-contact-safe geometry the `compliant_2d` model demands.

**Architecture:** The project already has a multi-contact tyre — `distributed_2d_reference` (128 circumferential stations, each a hinge-density radial spring with its own brush; `supports_multiple_contacts=True`; energy-audited) — but only on the Python backend and with `calibration_status = 'experimental_unvalidated'` enforced in code. Native has `compliant_2d` only (`docs/RIDE.md`, "Native capability remains the A/B subset … does not add … distributed-tire"). The question is therefore not "design a tyre" but "is the existing one good and fast enough, and is a native port worth its cost".

**Tech Stack:** Python 3.12 / numpy / MuJoCo 3.12.0 (Phase 0); C++20 + nanobind under `native/` (Phase 2, if chosen).

**Spec:** none yet. This plan covers the spike and the decision gate. The port (Phase 2) requires its own spec; the outline below is the input to that brainstorm, not a substitute for it.

## Global Constraints

- All Phase 0 artefacts are throwaway except `docs/superpowers/audits/<date>-multi-contact-tire-spike.md`; no production file changes in Phase 0 beyond a copied physics profile under `output/` (not committed).
- Use `uv run --frozen …` for every command; native commands use `--group native`.
- `AGENTS.md` requires reading `docs/agents/native-engineering.md` before native changes. **That file does not exist in the repository** (checked 2026-10-11). Before Phase 2 starts, ask the user where the native engineering rules live; do not infer them.
- Physics changes (Phase 2) run the `full` test profile; Phase 0 changes nothing, so no profile gate applies to it.
- `DistributedWheelMaterial.calibration_status` cannot be set to anything but `'experimental_unvalidated'` (`physics/distributed_tire.py:41`). No phase of this plan "validates" the tyre; a validated status is a separate measurement project with a fitting dataset (`physics/plant_profile.py:build_candidate` already demands radial records).

---

## Facts gathered (2026-10-11)

| Fact | Where |
|---|---|
| `distributed_2d_reference` is selectable through `TireBackendConfig.backend`; `surface_mode='track'` is allowed with it | `physics/physical_config.py:51-56` |
| The physical runtime instantiates `DistributedTireForceApplier` for that backend | `sim/ride/physical_runtime.py:86-88` |
| The research environment accepts it (`cfg.tires.backend in ('compliant_2d','distributed_2d_reference')`) | `sim/research/environment.py:110` |
| The physical session labels it "experimental distributed tire; mesh/time/station gates required" | `sim/ride/physical_session.py:171` |
| `multi_support` is reported but with `supports_multiple_contacts=True`, so validity is not revoked for it | `sim/ride/distributed_tire_forces.py:141`, `sim/research/validity.py:14` |
| Stand rigs exist: flat load, step, two-support, edge-unload, recontact, bump, incline, with bounds (force error ≤ 2 %, energy residual ratio ≤ 2–5 %, `max_patch_count ≥ 2` on the step) | `validation/contact_manifold_rigs.py`, `validation/contact_acceptance.py:contact_matrix` |
| A dt × road-step × station-count fidelity sweep exists for whole-bike cases, with `--backend distributed_2d_reference --stations` | `validation/fidelity_sweep.py:393-402` |
| Throughput tool: `tools/measure_realtime.py --backend python --physics-config … --track … --duration … --out …` | `docs/RIDE.md`, "Throughput evidence" |
| Native `compliant_2d` port size: `native/src/tyre/profile.{cpp,hpp}` 792 lines, `native/src/writers/tire.{cpp,hpp}` 1169 lines; Python distributed tyre: 155 + 153 + 80 lines | `wc -l` |
| Native config projection has no `tire.distributed` section; the writer rejects anything but `compliant_2d` | `bike_sim/native/config.py:34`, `native/src/writers/tire.cpp` |
| The multi-contact-ready alpine spec will be `examples/research/alpine_rocky_genspec.toml` (from the alpine-track plan) | `docs/superpowers/plans/2026-10-11-alpine-scree-track.md`, Task 7 |

---

## Phase 0 — Spike: measure `distributed_2d_reference` on the Python backend

Everything below runs after the alpine-track plan has landed (it uses `alpine_scree` and `alpine_rocky_genspec.toml`). Expected effort: one working session. Output: one audit file with numbers; recommendation at the end.

### Task 0.1: Profile copy with the distributed tyre

- [ ] **Step 1: Copy the welded profile and switch the backend**

```bash
mkdir -p output/mct-spike
cp examples/research/viewer_physics_welded.toml output/mct-spike/physics_distributed.toml
```

Edit `output/mct-spike/physics_distributed.toml`: in `[tires]` change `backend = "compliant_2d"` to `backend = "distributed_2d_reference"`. Leave `surface_mode = "track"` and both `mu = 1.1`. Do not add a `[tires.distributed]` table: the synthetic `HingeDensity()` defaults (`knots_m=(0, .004, .012)`, `stiffness_n_m=(300000, 0, 0)`, `damping_ns_m=2000`) are the declared experimental material.

- [ ] **Step 2: Confirm it loads and the plant builds**

```bash
uv run --frozen bike-research --physics-config output/mct-spike/physics_distributed.toml --scenario flat --duration 0.2 --diagnostic-model-limits --out output/mct-spike/flat --overwrite
```

Expected: exit 0, an `episode-0000/summary.json` whose `resolved_config.tires.backend` is `distributed_2d_reference`. If the config loader rejects the backend, record the exact error in the audit and stop: Phase 0 then begins with fixing the loader, which is a production change needing its own approval.

### Task 0.2: Stand acceptance matrix (does the model behave?)

- [ ] **Step 1: Run the existing matrix at the research timestep and the default station count**

```bash
uv run --frozen python - <<'EOF'
import json
from bike_sim.validation.contact_acceptance import contact_matrix
rows = contact_matrix(dts=(.00125, .000625), station_counts=(128, 256), flat_loads=(300., 600., 900.), step_dxs=(.01, .005))
json.dump(rows, open('output/mct-spike/contact_matrix.json', 'w'), indent=1, default=float)
print(sum(r['accepted'] for r in rows), 'of', len(rows), 'accepted')
for r in rows:
    if not r['accepted']:
        print(r['case'], r['dt_s'], r['station_count'], {k: v for k, v in r['checks'].items() if not v})
EOF
```

Record: accepted/total, which (case, dt, stations) fail and on which bound. The step case must show `max_patch_count ≥ 2` — that is the multi-contact behaviour we are buying.

- [ ] **Step 2: Two-support and recontact traces at dt = 0.00125**

```bash
uv run --frozen python - <<'EOF'
from bike_sim.validation.contact_manifold_rigs import distributed_road_trace
for case in ('two_support', 'recontact', 'bump', 'edge_unload'):
    rows, m = distributed_road_trace(.00125, 128, .005, case=case)
    print(case, {k: round(float(v), 4) for k, v in m.items() if k in ('energy_residual_ratio', 'max_patch_count', 'multi_support_fraction', 'native_wheel_contact_count')})
EOF
```

Record all four lines. `native_wheel_contact_count` must be 0 (the hfield geom never touches the wheel natively).

### Task 0.3: Throughput — the number the decision hinges on

- [ ] **Step 1: Python `compliant_2d` baseline and Python distributed, same track, same duration**

```bash
uv run --frozen python tools/measure_realtime.py --backend python --physics-config examples/research/viewer_physics_welded.toml --track alpine_scree --duration 10 --warmup 1 --runs 3 --out output/mct-spike/rtf-compliant
uv run --frozen python tools/measure_realtime.py --backend python --physics-config output/mct-spike/physics_distributed.toml --track alpine_scree --duration 10 --warmup 1 --runs 3 --compare output/mct-spike/rtf-compliant/report.json --out output/mct-spike/rtf-distributed
```

If `--track alpine_scree` is not accepted by the tool (it defaults to a file path), pass `--track examples/research/alpine_scree.toml`. Record median RTF of both and the ratio. Caveat from `docs/RIDE.md`: strict headless runs on this platform end at step 4 on the `energy.constraint_work` baseline violation, so if both reports show a 0.005 s prefix, re-run with the interactive path instead:

```bash
uv run --frozen bike-ride --backend python --physics-config output/mct-spike/physics_distributed.toml --track alpine_scree --time-scale 1
```

and read the HUD's achieved RTF after 20 s of simulated time (the HUD shows requested vs measured RTF). Do the same with the welded profile for the baseline. Record both.

- [ ] **Step 2: Station-count sensitivity (cost vs fidelity)**

Repeat Step 1's distributed measurement with `[tires.distributed] station_count = 64` and `= 256` added to the profile copy. Record RTF for 64/128/256 and, from Task 0.2 Step 1, which station counts passed the stands. This gives the cheapest station count that still passes.

### Task 0.4: The actual use case — realistic rocks

- [ ] **Step 1: Generate a rocky alpine track and ride it on both tyres**

```bash
uv run --frozen bike-research --physics-config examples/research/viewer_physics_welded.toml --scenario alpine --gen-spec examples/research/alpine_rocky_genspec.toml --seed 0 --duration 20 --diagnostic-model-limits --out output/mct-spike/rocky-compliant --overwrite
uv run --frozen bike-research --physics-config output/mct-spike/physics_distributed.toml --scenario alpine --gen-spec examples/research/alpine_rocky_genspec.toml --seed 0 --duration 20 --diagnostic-model-limits --out output/mct-spike/rocky-distributed --overwrite
```

From each `episode-0000/summary.json` record: outcome/terminal reason, `valid` flag and the first invalidity reason, total `multi_support` interval count (compliant) vs `max_patch_count`/`multi_support_fraction` (distributed), maximum normalized energy residual, distance reached, and wall time. The claim under test: the distributed tyre rides over 6–12 cm rocks and ledges with bounded energy residual where `compliant_2d` is flagged out of scope.

- [ ] **Step 2: Same two runs on `alpine_scree` (safe geometry) — do the tyres agree where both are in scope?**

Record distance reached, mean front load share and peak rear slip from telemetry for both backends. Large disagreement on safe geometry means the tyres are not interchangeable and the multi-contact tyre's answers to the traction question would not be comparable to the existing `compliant_2d` evidence.

### Task 0.5: Audit report

- [ ] **Step 1: Write `docs/superpowers/audits/<date>-multi-contact-tire-spike.md`**

Sections, each with the numbers from Tasks 0.1–0.4 (tables, not prose): Setup (profile diff, commit hash, platform), Stand matrix, Throughput (table: backend × station count → median RTF), Rocky track outcomes, Safe-track agreement, Observations, Recommendation (one of the Phase 1 outcomes below, with the criteria that selected it).

- [ ] **Step 2: Commit the audit only**

```bash
git add docs/superpowers/audits/*multi-contact-tire-spike.md
git commit -m "$(cat <<'EOF'
docs: multi-contact tyre spike — distributed_2d_reference measurements

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

`output/mct-spike/` is not committed.

---

## Phase 1 — Decision gate (criteria fixed before the numbers exist)

| Outcome | Chosen when | Consequence |
|---|---|---|
| **A. Stop at the spike** | Stand matrix fails at dt = 0.00125 for all station counts, or the rocky track still ends invalid on the distributed tyre, or safe-track agreement is poor | The distributed tyre is not ready to carry research conclusions; alpine tracks stay single-contact-safe. A new brainstorm on the tyre model itself (not a port) would be the next step |
| **B. Python-only, headless** | Stands pass; rocky track rides; Python distributed RTF ≥ 0.2× at the cheapest passing station count | Multi-contact research runs headless on `--backend python` in batches; `alpine_rocky_genspec.toml` gets a pinned preset and frozen TOML (a bounded follow-up to the alpine plan); no native work. Interactive 1× viewing of rocky tracks is not available |
| **C. Native port** | Stands pass; rocky track rides; the user wants interactive 1× or batch throughput that Python cannot give (RTF < 0.2×, or wall time per episode judged unacceptable) | Phase 2 |

The user makes the choice; this plan only states the criteria so the spike measures the right things.

---

## Phase 2 — Native port (only after Outcome C)

The brainstorm and spec for this phase are done: `docs/superpowers/specs/2026-10-11-native-distributed-tire-design.md`, with two implementation plans — `2026-10-11-distributed-tire-a-portable-reference.md` (Python reference made bitwise-portable, state wire, frozen `alpine_rocky` + distributed profile, before/after audit) and `2026-10-11-distributed-tire-b-native-port.md` (C++ writer, config, bitwise per-state gate, stands, sanitizers, RTF gate). The outline below is kept as the summary that fed that spec.

**Scope to port (Python → C++):**
1. `terrain/profile_distance.py::SignedProfile` (80 lines) — signed distance from a point to the piecewise-linear road, with the same winding/segment conventions as `ProfileQuery`.
2. `physics/distributed_tire.py` (153 lines) — `station_angles`, `normal_station_response`, `density_response`, `HingeDensity` evaluation; closed-form energy per station.
3. `sim/ride/distributed_tire_forces.py` (155 lines) — per-station geometry (`center + local @ rotation.T`), penetration set, brush state per `(side, station)` reusing the existing native brush step, material-point Jacobian wrench, snapshot/diagnostics, `stored_energy`, loss accounting. The native writer already has brush, friction and surface-map resolution for `compliant_2d`; the port adds a second contact-geometry path and a station loop, not a new friction model.

**Config projection:** `bike_sim/native/config.py` / `schema.py` gain a `tire.distributed` section (`station_count`, `density{knots_m, stiffness_n_m, damping_ns_m}`, per-wheel material domains). `native/src/writers/tire.cpp` accepts `backend == distributed_2d_reference`. `docs/RIDE.md` capability sentence is updated.

**Parity and tests (the expensive part):**
- Python↔native parity at the existing tolerances (the repo upgraded determinism asserts to bitwise and cross-backend state asserts to bitwise — see commits `6ae40a1`, `f3e2d90`); the spec must decide whether the distributed path promises bitwise or `1e-9` parity, since a 128-station reduction order differs between numpy and a C++ loop.
- Stand rigs re-run against native (`contact_matrix` on the native stepper), golden episode for a rocky track, sanitizer (ASan, RTSan) and coverage builds, `check_*` sweeps — profile `native`, then `full`.
- Performance contract test (`tests/reference/test_native_runtime_performance_contract.py`) extended with the distributed path: the whole point of the port is RTF.

**Risks to put in the spec:**
- Per-step cost is O(stations × window segments) × 2 wheels; the spike's station-count sensitivity decides the default.
- `experimental_unvalidated` stays. Porting does not calibrate; the spec must say the native tyre is a faster copy of an unvalidated model, and the HUD/summary wording from `physical_session.py:171` carries over.
- Equilibrium cache (`sim/ride/equilibrium_cache.py:194`) and initial state (`sim/ride/initial_state.py:92`) already branch on the backend; the native bootstrap must mirror whichever branch Python takes.

**Size (for planning, not a promise):** C++ roughly 1.5–2× the `compliant_2d` tyre port by line count (two geometry paths, station loop, extra config), plus tests. It is a multi-week project with the project's native rigor, not a follow-up task to the alpine track.

---

## Relationship to the alpine track plan

- The track plan ships `single_contact_safe = true` defaults and `alpine_rocky_genspec.toml` as the multi-contact-ready opt-in. Nothing in it waits on this plan.
- Outcome B or C turns `alpine_rocky_genspec.toml` into a pinned preset (`alpine_rocky`) with a frozen TOML — a bounded change: register in `PRESETS`/`SEEDED_PRESETS`, dump the file, add the T4-style equality test. Not before the tyre decision: pinning a track no supported tyre can ride honestly would be a fake reference.
