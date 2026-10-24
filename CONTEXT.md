# CONTEXT

Ubiquitous language. Terms that resolved into stable shared vocabulary; use exactly these. Started 2026-10-04 during the cpp-port decision effort (`.scratch/cpp-port/`).

## Simulation runtime

- **spatial mode** — explicit native 3D physics extension; the default planar request remains separate. Source implementation is not qualification; W45–W49 application verification is pending V08.
- **spatial root** — `constrained` control root or `free` six-degree root; qpos quaternion coordinates and qvel tangent coordinates have independent widths.
- **accepted spatial pose** — committed world position/quaternion used by the render replica, reports and schema2 channels. Render smoothing never writes back to physical state.
- **sensor schema2 / noise2** — eleven ordered spatial measurement components with source/delivery clocks, validity mask and ABI2; sensor-frame IMU axes are not world-frame truth.
- **recipe4** — spatial checked research replay contract inside container2; inspection alone never grants checked status.
- **spatial-fast-v1** — source-defined fast profile; realtime-factor targets 2.0/1.0 have no measurement until V09. See [spatial workflows](docs/spatial-3d.md).

- **physics step** — one `mj_step` call; dt = 0.5 ms of simulated time.
- **control tick** — the slower cadence on which the rider controller recomputes (e.g. 200 Hz); spans many physics steps.
- **intent-tick** — 10 ms boundary at which Python hands commands into the runtime ("intent-programs": scripted control intentions that play out inside the step loop).
- **tick-path** — everything executed per physics step or per control tick: step loop, force writers, controller, telemetry, recorder, intent-programs. The port boundary runs along it.
- **force writers** — modules that compute and inject `qfrc_applied`/`xfrc_applied` each step (tyre, motor, rider, attachments).
- **telemetry / channels** — the recorder's ~100-channel contract of per-step outputs; full contract must survive any port.

## Model construction

- **MJCF-builder** — model construction that generates MuJoCo XML once per model, outside the tick-path. The Python implementation remains an oracle during migration; its native replacement becomes authoritative under ADR-0003 and Python is removed at T63.
- **equality welds / virtual chain** — MuJoCo equality constraints and the generated tendon/actuator chains used to weld or drive bodies.

## Port effort (decision in `docs/adr/0001-native-port-mujoco-core.md`)

- **native core** — the C++23 library that owns the tick-path, `mjModel`, `mjData`, and all native buffers.
- **frontends** — the two entry points to the same native core: nanobind-extension (interactive/viewer, seam-checkable from Python tests) and standalone CLI (headless batch, own threads).
- **bespoke** — the parked fourth option: replace MuJoCo with own nv=26 constraint/contact pipeline (Pinocchio-based). Activated only by trigger: post-port RTF insufficient AND engine dominates the step.
- **golden episode** — a frozen artifact (raw state + all recorder channels) serving as a correctness oracle. Live Python comparisons remain gates until each test file's values are captured on both platforms and its C++ port lands; then per-platform goldens become the gate ([ADR-0003](docs/adr/0003-native-authoritative-python-removal.md)).
- **replica-mjData** — the viewer's local copy of simulation state for rendering (snapshot per frame; core keeps the authoritative `mjData`).
- **RTF** — real-time factor, simulated seconds per wall second; 1.0 = real time at dt=0.5 ms means step must cost ≤0.5 ms.

## Performance effort (map in `.scratch/perf-roadmap/`)

- **requested scale / achieved RTF** — what the viewer was asked to run at (1×, 2×, 4×, 8×) versus the measured simulated-time/wall-time ratio it sustained. Physics is scale-invariant; only the achieved number is evidence.
- **lever** — one independent way to make the same simulation faster on the same hardware (threading, SIMD, render stack, GPU, ...). A lever changes speed, never fidelity.
- **kill criterion** — a condition fixed before research starts under which a lever is dropped from the roadmap; stated per lever so the verdict is not post-hoc.
- **headroom** — achieved RTF above 1.0 at the interactive gate; the margin that absorbs a dearer tyre model or a slower frame.
- **benchmark mode** — a measurement run whose energy-residual cutoff is raised solely so the episode reaches steady state; marked as such in provenance, never a valid ride.
- **baseline configuration** — the measured "today" column of the ledger: native backend, welded profile, distributed tyre at its default station count on `alpine_rocky`.
- **order-changing optimization** — any speed-up that alters floating-point operation order (parallel sums, FMA, SIMD lanes, different BLAS). Under [ADR-0003](docs/adr/0003-native-authoritative-python-removal.md), preserve valid-input binary64 operation order and live-oracle bitwise gates until each affected file's two-platform golden capture and C++ test port land. Afterwards, numerical behavior changes still require review, in-build determinism and updates to the affected native self-goldens with before/after comparison evidence. `-ffp-contract=off` stays; changing it requires a separate decision.
