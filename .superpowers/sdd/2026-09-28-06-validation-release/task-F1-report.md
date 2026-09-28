# Task F1 report — synchronized physical telemetry

Status: implemented on branch `pedals`, based on A2 commit `a60df5e`.

## Delivered

- `ForceSample` owns immutable copies of pre-step `time_s`, `qpos`, `qvel`, and every named generalized force. `component_powers` uses the velocity from that same sample. The A2 tuple interface, `last_force_snapshot`, is retained; `last_force_sample` exposes the new API.
- `EnergyLedger.residual` uses the sign convention `E - E0 - active_work - external_work + loss`. `mechanical_energy_terms` derives kinetic energy from the compiled mass matrix and gravity from compiled body masses and inertial positions; externally modelled elastic energy is an explicit input.
- `system_momentum` refreshes kinematics and sums linear and angular momentum over every physical body. Angular momentum uses each inertial-frame tensor plus the orbital term about the common compiled CoM.
- `RideRecorder` preserves legacy schema version 1. Physical schema version 2 stores the force interval start and duration, all pre-step qpos/qvel entries, whole-system momentum, kinetic and gravitational energy, and per-component synchronized power and integrated work. Work integration occurs on every `record` callback, even when rows are decimated. Component columns expand as writers appear.
- The physical CSV leaves elastic energy, total mechanical energy, and energy residual as NaN for now. Current A2 suspension force combines spring and damper, and future C/D/E writers have not supplied the rest of storage and loss channels; asserting a closed SYS-04 balance at this stage would be false.

## Verification

- Red: the first F1 test collection failed because the new modules did not exist. Additional tests exposed the missing recorder schema, the absence of pre-step state columns, and mutable NumPy write flags before implementation.
- Focused: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest tests/test_physics_telemetry.py tests/test_ride_telemetry.py tests/test_force_accumulator.py -q` — **34 passed**.
- Full: `UV_CACHE_DIR=/private/tmp/uv-cache-physics-20260928 uv run --locked pytest -q` — **622 passed, 4 errors**, exit 1. The four errors are the known `tests/test_render_comparison.py` fixture failure, `mujoco.cgl.cgl.CGLError: invalid CoreGraphics connection`; no new test failures appeared. Earlier baseline had the same four renderer setup errors.
- `git diff --check` passed.

## Follow-up integration

- C/D/E writers should register their generalized force channels with A2 and provide elastic storage, motor input, dissipations, and any separately signed world/contact work. Once every term is available, use the ledger to populate the currently unavailable energy columns and run SYS-04 convergence checks.
- The current recorder contract requires `record` once after every physical substep; decimation controls row storage, not callbacks. No runtime renderer test is available in this CoreGraphics session.
