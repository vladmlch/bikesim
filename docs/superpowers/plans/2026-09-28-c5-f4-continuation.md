# C5-F4 Continuation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Continue the existing physics-correctness plans from C5 through F4 without altering legacy physics silently.

**Architecture:** Preserve the existing force accumulator, contact snapshots and suspension implementation. Add explicit planar tire, drivetrain and independent-rider components, with immutable SI configurations. Physical telemetry owns pre-step samples and per-substep work; validation reports failures and unavailable gates, not fabricated passes.

**Tech Stack:** Python >=3.12, NumPy, SciPy, MuJoCo and pytest. No dependency or uv.lock changes.

**Spec:** `docs/superpowers/specs/2026-09-28-physics-correctness.md`; task-level implementation plans `2026-09-28-03-tire-contact.md` through `2026-09-28-06-validation-release.md` remain authoritative.

## Global Constraints

- Original specification base: 70623815b98788018bcdbd8eef347d778f9bb3f3.
- Continuation source base: 9a641f6fa1a9e39fb6e95d7ce27d1aa14e1128c6. C3/C4 are already committed there.
- Python >=3.12; preserve the existing uv.lock and declared dependencies.
- SI internally: m, s, kg, N, N*m, rad. Preserve explicit legacy external conversions.
- Planar X-Z only; no lateral traction or roll-balance claim.
- No physical pitch stabilization, manual weight-transfer force or post-step qpos/qvel correction.
- Legacy and physical revisions and numerical references remain separate.
- Unmeasured parameters are synthetic. Synthetic tests do not constitute experimental calibration.
- Acceptance tolerances are criteria, not results until executed.
- Read only relevant source ranges. Changes must apply to the verified source revision.
- Full repository and locked MuJoCo execution are unavailable in this runtime; isolated Python checks must not be reported as those gates.

---

## Task C5-C6: contact and external resistance

**Files:** `terrain/contact_profile.py`, `sim/ride/tire_forces.py`, `physics/external_resistance.py`, shared numeric validation and physical configuration; narrow builder/runtime integration patches.

**Interfaces:** `closest_profile_contact`, `ProfileQuery.contact`, `TireForceApplier.compute_qfrc`, `rolling_moment`, `drag_force`.

- [x] Write geometry/passivity tests in `tests/test_profile_contact.py` and `tests/test_external_resistance.py`.
- [x] Run `PYTHONPATH=src python -m pytest tests/test_profile_contact.py tests/test_external_resistance.py -q`; observe missing modules.
- [x] Implement local segment search with true distance minima (discard false endpoint supports), unilateral contact and odd rolling resistance.
- [x] Run the same command: 23 passed in the isolated staging tree.
- [ ] Add one force mapper and a compiled-heightfield adapter; disable native wheel-road forces only for compliant contact.
- [ ] Execute virtual-work, native-double-counting and dynamic refinement tests in MuJoCo.

## Task D1-D6: physical drivetrain

**Files:** `physics/chain.py`, `physics/freehub.py`, `physics/pedaling.py`, `physics/motor.py`, `physics/battery.py`, `sim/ride/drivetrain_forces.py`, static brake adapter, physical topology and runtime integration.

**Interfaces:** chain extension/Jacobian, passive freehub state, assist request controller, battery budget limiter, dynamic crank/cassette and solver-friction brakes.

- [ ] Add failing pure-law and energy tests: `tests/test_physical_drivetrain_laws.py`.
- [ ] Run `PYTHONPATH=src python -m pytest tests/test_physical_drivetrain_laws.py -q`.
- [ ] Implement each specified law with finite-input validation and atomic state changes.
- [ ] Run the same tests and inspect load reversal, zero-speed losses, cadence jumps and brake cancellation.
- [ ] Transfer existing mass, rather than duplicating it, into independent crank/pedal/cassette bodies.
- [ ] Integrate named contributions, actual actuator work and brake constraint rows.
- [ ] Execute full topology, virtual-work, freehub/chain dt-refinement and incline-hold gates in MuJoCo.

## Task E1-E4: independent rider

**Files:** `physics/rider_segments.py`, `mujoco/articulated_rider.py`, `sim/ride/rider_contacts.py`, `sim/ride/rider_control.py`, and narrow rider-resolution/runtime patches.

**Interfaces:** fixed anatomical masses; world-rooted pelvis; shared-point reaction pairs; contact release; bounded internal joint torques with no second human crank source.

- [ ] Write anatomy, common-point reaction, stance, inverse-kinematics and joint saturation tests.
- [ ] Run `PYTHONPATH=src python -m pytest tests/test_articulated_components.py -q` before implementation.
- [ ] Implement mass decomposition, topology, finite platform contacts and internal control.
- [ ] Rerun the isolated tests; separately run compiled mass/topology and released-rider momentum tests in MuJoCo.

## Task F1-F4: recording, validation and configuration

**Files:** physical telemetry/energy integration, `validation/benchmarks.py`, `validation/datasets.py`, `tools/validate_physics.py`, physical CLI/session adapter, HUD/plot adapter and documentation.

**Interfaces:** synchronized samples; full-body momentum; explicit benchmark registry; strict experiment splits; canonical config/terrain hash; schema 2 with schema-1 legacy preserved.

- [ ] Add failing dataset, hash, CLI precedence and validation-result tests.
- [ ] Run `PYTHONPATH=src python -m pytest tests/test_physical_release.py -q`.
- [ ] Implement strict SI schemas and whole-experiment holdout, fail-closed benchmark reporting, runtime version metadata and config resolution.
- [ ] Run the isolated tests, then the exact locked commands in the original F4 plan in a complete checkout.
- [ ] Review every changed file for force duplication, state timing, mass ownership, unsupported fallbacks and false validation claims.
- [ ] Record findings and remaining release gates in `verification/REVIEW.md`; do not check off gates without execution.

## Recorded execution result

Status: incomplete; REQUEST CHANGES. This continuation produced isolated modules
and standalone adapters, not a finished C5-F4 implementation. The final isolated
suite has 273 passing tests. See `verification/REVIEW.md` for blockers and the exact
boundary of verification. No unexecuted engine or release gate is marked complete.
