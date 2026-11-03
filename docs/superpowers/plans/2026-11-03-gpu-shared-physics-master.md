# GPU Shared Physics Core — Master Plan

> **Imported into main3 (2026-11-03), not started.** Written in the colab-gpu
> worktree against `research/colab-gpu` `b25120af`, before main3 merged
> passive-joint-damping and colab-gpu, ported kinetic-damping and removed the
> battery energy model (`54ee44f8`). Re-verify every file:line fact against
> main3 before executing. Known differences:
> - `tools/fp32_probe/artifacts/common/model.xml` (with `captured_bootstrap.json`
>   and `hfield_data.bin`) is tracked in main3 (`70f72783`).
> - `docs/superpowers/audits/` does not exist in main3 (`072f0002`); research
>   records live in `docs/research/`.
> - Recording channels `battery_energy_j`, `battery_empty`, `energy_limited`,
>   `electrical_power_w` and energy terms `electrical_work_j`,
>   `electrical_residual_j` no longer exist.
> - Open question Q23 from the planning session (record every per-step model
>   mutation: brake joint friction, motor tendon limit, solver warmstart) was
>   never answered.

> **For agentic workers:** this is the roadmap. It is not executed directly.
> Each stage has, or will get, its own implementation plan (`docs/superpowers/plans/2026-11-03-gpu-shared-*.md`).
> Execute those with superpowers:subagent-driven-development (recommended) or superpowers:executing-plans.
> A stage's plan is written only once the stages before it have landed, because it depends on what they teach (the toolkit shape, the kinematics adapter).

**Goal:** the GPU path (warp-metal + mujoco_warp) computes the bike's per-step force laws with **the same source code** as the CPU runtime.
Its behaviour is proved against CPU golden recordings and made visible in the native viewer.

**Architecture:**
- The per-step force laws (tyre, suspension, resistance, rider joint envelope, freehub ratchet) move out of `native/src/writers/` into allocation-free, exception-free, header-only templates in `native/src/physics_core/`.
- Each template is parameterised on a scalar type and a *toolkit* of primitive operations (dot, gemv, sums, pow, sin/cos).
  - The CPU instantiates them with `double` and the existing NumPy/BLAS-exact primitives, so CPU output stays bit-identical.
  - Metal instantiates them with `float` and plain loops, compiled in safe math mode.
- Stateful controllers (rider spindle controller, pedaling policy, motor/assist, intent, brakes) stay CPU-only in stage 1. Their outputs are recorded explicitly (recording schema v2) and replayed into the GPU world.
- A windowed GPU runner, a comparison report and a viewer ghost overlay measure and show the divergence.
- A CPU-FP32 lane separates precision error from solver error and from bugs.

**Tech Stack:** C++23 (CPU), MSL via `metal_batch_runner.mm`, CUDA (`cuda_kernels.cu`, Colab only), MuJoCo 3.12.0 native, MuJoCo 3.17.0 + mujoco_warp 3.11.0 + warp-metal 1.17.0 in `.venv-metal`, DuckDB recordings, uv for Python.

**Supersedes:** `docs/superpowers/plans/2026-11-02-gpu-golden-replay.md`. The mapping to the new stages is in §Old plan mapping.

---

## Decisions (D1–D22, agreed with the user 2026-11-03)

| # | Decision |
|---|---|
| D1 | The shared code is extracted **from the CPU code**. The CPU is the reference. Each GPU `*_kernel.hpp` copy is deleted once its block has migrated. |
| D2 | CPU output stays **bit-identical** through the refactor. No golden fixture is re-captured. Any changed bit means changed physics, and that is a bug in the refactor. |
| D3 | Scope is what `examples/research/viewer_physics_distributed.toml` exercises. Other tyre backends (`compliant_2d`, `pneumatic`) stay CPU-only and untouched. |
| D4 | Migrate one block at a time, tyre first. For each block: extract the core, check CPU bits are identical, switch the GPU to the core, then delete the old GPU copy. The project is green after every block. |
| D5 | The cores are templates over the scalar type: CPU uses `double`, Metal and the FP32 lane use `float`. |
| D6 | The cores must also compile under CUDA. CUDA is verified once, in Colab, at the end (Stage 8). |
| D7 | The GPU comparison against CPU golden recordings follows the unification; it is no longer a tyre-parameter fix. Old plan Tasks 7–8 are dropped. |
| D8 | Staged scope. **Stage 1** shares only the per-step laws. Controllers stay CPU-only, and their outputs are fed to the GPU from the recording. Whether controllers are ported later (full port vs CPU-in-the-loop hybrid) is a separate user decision after Stage 7, informed by measured cost. Until then, a *free-running* GPU world does **not** behave like the CPU. |
| D9 | The core is one formula with platform primitives supplied by a toolkit type parameter:<br>• `CpuExactToolkit` wraps `blas::ddot/dgemv/dgemm`, `numeric::numpy_sum`, `PythonSum`, `pyfloat::pow`, `numeric::np_sin/np_cos`, plus the explicit fma in `np.interp`.<br>• `GpuToolkit` uses plain loops in `float`. |
| D10 | Kinematics enter the core as data: wheel centre, wheel rotation and the 3×nv translational Jacobian of the wheel centre.<br>• The CPU fills these from `mj_jac`.<br>• The GPU computes the exact planar Jacobian from the dof chain (root → fork or swingarm pivots → wheel).<br>• The per-station cross product (`distributed_tire.cpp:673-681`) is shared.<br>• A test checks the GPU Jacobian against `mj_jac` at several poses.<br>• The GPU "resultant at centroid" approximation is deleted. |
| D11 | The terrain polyline (`tyre/profile.cpp`) is built once on the host by the existing CPU code and uploaded to the GPU. The closest-point query is shared. |
| D12 | GPU-only features are deleted: tyre lateral ribs (`num_lateral_ribs`) and the fork top-out stop. |
| D13 | Cores never throw. They return a status code.<br>• CPU wrappers map the code to the exact exception thrown today.<br>• On the GPU, the code goes into a per-world status buffer and that world stops. The runner reports the world, step and reason. |
| D14 | Diagnostics (energy, loss and passivity ledgers, snapshots) stay in the CPU wrappers. The GPU computes forces and the state the next step needs (bristles, ratchet boundary). |
| D15 | GPU throughput will drop: 128 rotating stations, exact distance and an exact Jacobian. The µs/step figures in the 2026-10-31 audit become stale and are re-measured at the end. |
| D16 | Recording schema **v2** adds per-step controller outputs: `ctrl[nu]`, the freehub tendon upper bound and the chain ratio. v1 recordings still read. `docs/RECORDING.md` is updated. |
| D17 | In Stage 1 the GPU computes tyre, suspension, rolling and aero resistance, joint envelope and freehub ratchet with the shared cores.<br>• `ctrl` (rider, crank, motor, brakes) and the chain ratio come from the recording.<br>• The recorded freehub bound is checked against the GPU-computed bound. |
| D18 | Before the first code change, record 30 s CPU episodes on `flat` and `rough_uphill_extreme`. Every migrated block must reproduce them bit-for-bit (end-to-end digest). They double as the comparison goldens. |
| D19 | Metal compiles the cores in **safe math mode**, not with the current `options:nil` fast-math default (`metal_batch_runner.mm:878`). A test requires the Metal output to be bitwise equal to the host-`float` instantiation. If Metal cannot disable FMA contraction, the tolerance is 1 ulp. That fact is established first. |
| D20 | The shared code lives in `native/src/physics_core/`, header-only, one topic per file. CPU wrappers stay in `native/src/writers/`. The MSL stitcher includes `physics_core/` instead of the deleted `gpu/*_kernel.hpp`. |
| D21 | After Stage 1, a CPU-FP32 lane: port the `research/fp32-physics` mjtNum cascade (22 commits, 63 files, prebuilt `third-party/mujoco-312-install-f32`) onto the current code. It builds in a separate dir and is not a gate build. The report shows three pairs: CPU64↔CPU32 (precision), CPU32↔GPU (solver plus bugs) and CPU64↔GPU (total). The CPU64↔CPU32 band is the bug threshold. |
| D22 | Precision is a **build** choice, not a runtime switch: presets `release` (f64) and `release-f32`. `manifest.json` records the lane (`f64` / `f32` / `gpu-f32`). Replay and compare label the lanes and refuse a pair whose lanes do not match the requested comparison. |

**Out of scope:** porting controllers (D8), 3D/spatial tyre, `compliant_2d` and `pneumatic` on GPU, GPU speed optimisation, CUDA beyond a compile-and-smoke check.

---

## Global Constraints

- Python only via `uv` (`uv run ...`). GPU Python uses `.venv-metal` (`uv run --python .venv-metal/bin/python ...` or the existing `tools/colab_gpu` invocation pattern).
- Read `docs/agents/native-engineering.md` before any native change.
- The completion gate per stage is `bash tools/run_tests.sh native`, exit 0, run **once** at the end of the stage.
  - Stages that change native physics (1, 4, 5, 6, 7) also need `bash tools/run_tests.sh full`, per `docs/agents/native-engineering.md:47-49` ("until `full` contains native preflight, a native physics/realtime change requires both `native` and `full`").
  - Check that condition from the launcher code at the start of each stage.
  - The inner loop uses targeted CTest nodes.
- Strict headless rides stop at t = 0.005 s on the `energy.constraint_work` baseline (`docs/RIDE.md:102-105`). Golden and comparison rides therefore use `--check-policy diagnostic` and accept exit 2 only as `duration_reached` (precedent: `native/tests/package/package_smoke.cmake:59-84`).
- CPU bit identity (D2) is checked with the end-to-end digest from Stage 0 after **every** task that touches `native/src/writers/`, `native/src/tyre/`, `native/src/contact/`, `native/src/drivetrain/` or `native/src/physics_core/`.
- No golden fixture under `tests/fixtures/golden/` is re-captured or edited.
- Never forge sealed recording containers. New fields come from the recorder, never from post-editing a `run.duckdb`.
- `physics_core/` headers:
  - must compile as MSL, CUDA and host C++;
  - no `std::` containers, no allocation, no exceptions, no virtuals, no recursion;
  - fixed-size arrays sized by `constexpr` capacities;
  - address-space macros from `gpu/common.hpp` (`BIKE_HD`, `BIKE_THREAD`, `BIKE_DEVICE`, `BIKE_CONSTANT`).
- `-ffp-contract=off` stays on for CPU. The core must not depend on FMA contraction anywhere; explicit fma is a toolkit primitive.
- T63 (Python removal): `tools/colab_gpu/**` keeps its recorded exemption (the old plan's Task 1 edit to `2026-10-20-cpp-linux-completion.md`).

---

## Stages

Each stage ends green: `bash tools/run_tests.sh native` exits 0, and the end-to-end digest matches D18.

| Stage | Plan file | Delivers | Depends on |
|---|---|---|---|
| **0** Goldens and recording v2 | `2026-11-03-gpu-shared-s0-goldens.md` | Recording schema v2 (`controls` table and `manifest.precision`), 30 s goldens on two tracks, `bike-recording-digest` tool, CTest node `golden_episode_digest` (label `slow`), RECORDING.md | — |
| **1** Toolkit and tyre core | `2026-11-03-gpu-shared-s1-tyre.md` | Metal safe-math fact and test; `physics_core/{toolkit,kinematics,terrain,tyre}.hpp`; `writers/distributed_tire.cpp` calls the core (bits identical); GPU tyre uses the core with 128 rotating stations and the exact planar Jacobian; `gpu/tire_kernel.hpp` deleted; Metal = host-float bit test | 0 |
| **2** GPU golden runner and report | written after 1 | Model bundle export (old Task 2), windowed GPU runner (old Task 5) with recorded `ctrl` / chain ratio and **recorded force injection for blocks not yet migrated**, comparison report (old Task 6) | 1 |
| **3** Viewer ghost | written after 2 | Old Tasks 9–10: BGTRAJ01 trajectory, `bike-replay GOLDEN --view --trajectory DIR`, ghost, Δ HUD, tyre arrows. **First visual check by the user.** | 2 |
| **4** Suspension core | written after 3 | `physics_core/suspension.hpp`; the GPU stops injecting `suspension` | 2 |
| **5** Resistance core | written after 4 | `physics_core/resistance.hpp` (rolling via the D10 Jacobian, aero); injection of `resistance` removed | 4 |
| **6** Envelope and freehub cores | written after 5 | `physics_core/{joint_envelope,freehub}.hpp`; GPU freehub bound checked against the recorded bound | 5 |
| **7** FP32 lane | written after 6 | D21/D22: `release-f32` preset, cascade port, f32 goldens, three-pair report | 6 |
| **8** CUDA smoke | written after 7 | The cores compile under nvcc in Colab; one-world smoke run vs Metal | 7 |
| — | — | **Decision point (user):** port controllers (full port) or CPU-in-the-loop hybrid, with Stage 7 numbers and measured costs | 7 |

**Why the runner (2) and viewer (3) come before the other cores.** The runner injects the recorded force component of every block that has not migrated yet. As soon as the tyre is shared, the GPU world runs with a CPU-identical tyre law and the CPU's own suspension, resistance and envelope forces. Each later stage flips one component from *injected* to *computed*, and the report shows that flip's effect in isolation. This also gives the user the visual check early, which was the original goal.

---

## Verified facts this roadmap relies on (2026-11-03)

- **The GPU writers are an independent re-port, sharing no code.** `native/src/gpu/*.hpp` is 3 099 lines and includes nothing from `writers/`; the reverse is also true. The active CPU physics is about 19 k lines.
- **The tyre law differs on both sides, not just its parameters** (`tire_kernel.hpp` vs `distributed_tire.cpp`):
  - penetration (vertical gap vs Euclidean polyline distance);
  - station placement (fixed world angles vs a ring rotated by `xmat`);
  - bristle update and sign convention;
  - friction (linear blend vs Stribeck surface map);
  - projection (resultant at centroid vs per-station `mj_jac`);
  - lateral ribs and tangent transport, each present on one side only.
- **CPU tyre primitives** exist for NumPy bit-parity: `cblas_abi.hpp` (Accelerate `ddot/dgemv/dgemm`), `numeric_sum.hpp`, `numeric_sincos.hpp`, `pyfloat.hpp`, and the explicit fma in `tyre/profile.hpp`. None of them compiles under MSL.
- **CPU tyre bitwise tests:**
  - CTest `native_distributed_tire`, `distributed_tire_reference`, `native_distributed_stands` (`native/tests/reference/Phase3Ports.cmake:116-129`);
  - `native_distance_tests` and `native_station_tests` (`native/CMakeLists.txt:431-447`);
  - fixtures under `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/native_distributed_tire/`.
- **The rider under this config** is `articulated_planar` with spindle pedals.
  - `RiderForcesWriter` and the legacy controller are inactive.
  - `RiderContactWriter` writes qfrc = 0 (all supports linked).
  - The live force law is `SpindleController::envelope_forces` (about 50 lines).
  - The live controller is `SpindleController::compute`. It reads weld reactions (`efc_*`), `qfrc_bias` and `mul_m`, and runs after `refresh_forward`. Control period 5 ms = 4 steps of 1.25 ms (`model_config.hpp:49`, TOML `timestep_s = 0.00125`).
- **Drivetrain:** `transmission_model = "ideal_mid_drive"`. `Transmission` mutates `tendon_range[1]` and `wrap_prm`, and calls `set_const` and `mj_copyModel`. Pedaling policy, motor, assist, battery and shifting are stateful controllers. Brakes write `ctrl`.
- **Recording:** schema v1 (`recording/catalog.hpp:16`) stores per-step `physics(time, qpos, qvel)` and `forces(component, qfrc, power_w)`, but no `ctrl` and no model mutations.
- **Metal library compile:** `newLibraryWithSource:... options:nil` (`metal_batch_runner.mm:878`). That is fast math by default.
- **FP32 experiment** (`research/fp32-physics`, merge-base `4667639c`, 2026-10-14):
  - The whole native runtime builds with `-DmjUSESINGLE` (`BIKE_FP32`).
  - Drift vs f64 is about 1/7 of the smallest designed initial-condition perturbation.
  - Systematic effects: freehub tendon-limit engagement +47%, and an `ideal_transmission` activation flip at step 0.
  - The current branch changed the same writer files by +6.9k/−0.7k lines since the fork, so the port is real work.

## Risks and gates

| Risk | Gate | Stop condition |
|---|---|---|
| R1. The core cannot reproduce CPU bits: BLAS order, NumPy sums or libm `pow`. | D9 toolkit; end-to-end digest plus per-unit bitwise tests after each task | A digest mismatch that a toolkit primitive cannot explain. Stop and report the first differing step and component. |
| R2. Metal cannot be made IEEE-safe enough to match host-float. | Stage 1 Task 1 measures it before any core work. | Neither bitwise nor 1-ulp equality is achievable. Report, then pick a tolerance with the user. |
| R3. The planar Jacobian disagrees with `mj_jac`. | Stage 1 Jacobian test, ≤ 1e-12 in double on host | Mismatch. Stop: the dof-chain assumption is wrong. |
| R4. `ctrl` + chain ratio + recorded injection do not close the CPU step. This was old R1, now addressed at the root by D16. | Stage 2 closure test on the CPU: replaying the recorded controls and the injected components with MuJoCo 3.12 FP64 must reproduce the next state to ≤ 1e-9. | Residual above the bound. Stop: something mutated is still unrecorded. List the candidates (`eq_active`, `set_const` side effects). |
| R5. Recording v2 breaks v1 readers or sealed-container validation. | Stage 0: the reader round-trips both versions; `database_validate.cpp` accepts both. | — |

## Old plan mapping (`2026-11-02-gpu-golden-replay.md`)

| Old task | Fate |
|---|---|
| 1 Golden recordings + Python reader | Stage 0 (goldens), with schema v2 added; the Python reader moves to Stage 2 |
| 2 `bike-gpu-export` bundle | Stage 2, unchanged |
| 3 Closure gate | Stage 2 closure test (R4); now expected to pass by construction |
| 4 Host golden writer step | Replaced: the GPU step calls the cores plus injection of non-migrated components |
| 5 Metal golden kernel and runner | Stage 2, adapted (controls from recording) |
| 6 Comparison report | Stage 2; three-pair extension in Stage 7 |
| 7 Baseline before tyre fix | Dropped (D7) |
| 8 Tyre parameter mapping | Dropped: one law, parameters from the same config |
| 9–10 Trajectory input and viewer overlay | Stage 3, unchanged |
| 11 Docs and final gate | Spread across stages; each stage updates its own docs |
