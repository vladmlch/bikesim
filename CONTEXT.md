# CONTEXT

Ubiquitous language. Terms that resolved into stable shared vocabulary; use exactly these. Started 2026-10-04 during the cpp-port decision effort (`.scratch/cpp-port/`).

## Simulation runtime

- **physics step** — one `mj_step` call; dt = 0.5 ms of simulated time.
- **control tick** — the slower cadence on which the rider controller recomputes (e.g. 200 Hz); spans many physics steps.
- **intent-tick** — 10 ms boundary at which Python hands commands into the runtime ("intent-programs": scripted control intentions that play out inside the step loop).
- **tick-path** — everything executed per physics step or per control tick: step loop, force writers, controller, telemetry, recorder, intent-programs. The port boundary runs along it.
- **force writers** — modules that compute and inject `qfrc_applied`/`xfrc_applied` each step (tyre, motor, rider, attachments).
- **telemetry / channels** — the recorder's ~100-channel contract of per-step outputs; full contract must survive any port.

## Model construction

- **MJCF-builder** — Python code generating the MuJoCo model XML (mujoco/, geometry/, kinematics/). Runs once per model, never on the tick-path; stays Python.
- **equality welds / virtual chain** — MuJoCo equality constraints and the generated tendon/actuator chains used to weld or drive bodies.

## Port effort (decision in `docs/adr/0001-native-port-mujoco-core.md`)

- **native core** — the C++17 library that owns the tick-path, `mjModel`, `mjData`, and all native buffers.
- **frontends** — the two entry points to the same native core: nanobind-extension (interactive/viewer, seam-checkable from Python tests) and standalone CLI (headless batch, own threads).
- **bespoke** — the parked fourth option: replace MuJoCo with own nv=26 constraint/contact pipeline (Pinocchio-based). Activated only by trigger: post-port RTF insufficient AND engine dominates the step.
- **golden episode** — frozen Python-produced artifact (raw state + all recorder channels) serving as correctness oracle; Python is oracle via artifacts, not live comparison.
- **replica-mjData** — the viewer's local copy of simulation state for rendering (snapshot per frame; core keeps the authoritative `mjData`).
- **RTF** — real-time factor, simulated seconds per wall second; 1.0 = real time at dt=0.5 ms means step must cost ≤0.5 ms.
