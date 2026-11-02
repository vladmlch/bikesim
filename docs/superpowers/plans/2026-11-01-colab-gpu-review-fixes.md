# Colab/Metal GPU Physics & Runner Review Fixes Implementation Plan

**Goal:** Fix all 6 physical blockers, all 7 Metal runner correctness/safety defects, all 3 benchmark methodology flaws, and all test/CMake integration gaps identified and verified in the `1063e479` technical review.

**Global Constraints:**
1. **Cross-Target Header Portability (`BIKE_THREAD`):** All functions in `native/src/gpu/*.hpp` must use `BIKE_THREAD` (defined in `common.hpp` as `thread` under `__METAL_VERSION__` and empty in C++/CUDA) on pointer and reference parameters/locals instead of fragile whitespace-dependent string replacement.
2. **Capacity & Bounds Safety:** `kMaxNq = 32`, `kMaxNv = 32`, `kMaxNu = 16`, `kMaxNbody = 32`, default `idx.nbody = 28`. Host runners must validate model dimensions against these bounds before stepping.
3. **Terrain Binary Loading:** Every runner and sweep script loading `tools/fp32_probe/artifacts/common/model.xml` must load `hfield_data.bin` (if present in the same directory and `hfield_data` has matching element count `48002`) into `m->hfield_data` before extracting the 1D heightfield or stepping MuJoCo.
4. **Full Suspension Kinematic-Chain Jacobian ($J$ and $J^T$):** Front wheel contact velocity and force projection must include `fork_travel_dof` (`dof 6`) along the world-frame fork slide unit vector $\vec{e}_{\text{fork}} = \text{normalize}(\text{xpos}[\text{fork\_lower}] - \text{xpos}[\text{steer}])$. Rear wheel contact velocity and force projection must include `main_pivot_dof` (`dof 8`, anchor $\text{xpos}[\text{chainstay}]$) and `horst_pivot_dof` (`dof 9`, anchor $\text{xpos}[\text{seatstay}]$).
5. **Rider 1-to-1 Joint/Actuator Pairing & `+=` Accumulation:** The 6 rider slots map 1-to-1 to paired `(joint, actuator)` in `model.xml`:
   - Slot 0 (`hip_front`): `qpos=22, dof=22, ctrl=8` (`rider_hip_front` / `act_rider_hip_front`)
   - Slot 1 (`knee_front`): `qpos=23, dof=23, ctrl=9` (`rider_knee_front` / `act_rider_knee_front`)
   - Slot 2 (`hip_rear`): `qpos=24, dof=24, ctrl=10` (`rider_hip_rear` / `act_rider_hip_rear`)
   - Slot 3 (`waist_pitch`): `qpos=17, dof=17, ctrl=3` (`rider_torso_hinge` / `act_rider_torso_hinge`)
   - Slot 4 (`shoulder_pitch`): `qpos=20, dof=20, ctrl=6` (`rider_shoulder_right` / `act_rider_shoulder_right`)
   - Slot 5 (`elbow`): `qpos=21, dof=21, ctrl=7` (`rider_elbow_right` / `act_rider_elbow_right`, `sign_flip=1`)
   `gpu_write_rider_qfrc_and_ctrl` must accumulate via `qfrc_world[dof] += out.envelope_torque_nm[j]`.
6. **Unified Drivetrain Gear Ratio (`22 / 51`) & Single Torque Path:** `GpuDrivetrainParams` defaults to `chainring_teeth = 22.0f`, `cog_teeth = 51.0f` (`ratio = 22.0f / 51.0f = 0.43137255f`), matching `<fixed name="ideal_mid_drive_freehub">` in `model.xml`. When the MuJoCo tendon constraint is active (`tendon_range_batch != nullptr && idx.freehub_tendon >= 0`), `rear_drive_torque_nm` is not added to `qfrc_world[idx.rear_wheel_spin_dof]` (the tendon constraint transmits the torque), and `tendon_range_batch[w * 2 + 1]` is synced into `tendon_range` in all 3 scenarios. In standalone pawl mode (`tendon_range_batch == nullptr || idx.freehub_tendon < 0`), pawl reaction torque `-pawl_torque_nm * ratio` is subtracted from `crank_torque_nm`.

---

## Wave 1: Foundation Header (`native/src/gpu/common.hpp`)

### Task 1: Update `native/src/gpu/common.hpp` with MSL address-space macros, model indices, and suspension $J / J^T$ helpers
- **Files:** `native/src/gpu/common.hpp`
- **Requirements:**
  1. Define `BIKE_THREAD`, `BIKE_DEVICE`, `BIKE_CONSTANT` (`thread`, `device`, `constant` under `__METAL_VERSION__`, empty otherwise).
  2. Set `kMaxNq = 32`, `kMaxNv = 32`, `kMaxNu = 16`, `kMaxNbody = 32`.
  3. Update `GpuModelIndices` defaults:
     - `nbody = 28`
     - Add `steer_body = 5`, `fork_lower_body = 6`, `chainstay_body = 8`, `seatstay_body = 9`
     - Set `rider_qpos[6] = {22, 23, 24, 17, 20, 21}`, `rider_dof[6] = {22, 23, 24, 17, 20, 21}`, `rider_ctrl[6] = {8, 9, 10, 3, 6, 7}`
  4. Provide `gpu_fork_axis_world(const GpuModelIndices &idx, const float *xpos, float root_pitch_rad)` returning `GpuVec2` unit vector of `fork_travel` in world `(x, z)` from `xpos[fork_lower_body] - xpos[steer_body]` (fallback `(-0.4345159f, 0.90066416f)` rotated by `root_pitch_rad` if `steer`/`fork_lower` positions coincide).
  5. Provide `project_front_wheel_force_to_qfrc` and `project_rear_wheel_force_to_qfrc` alongside `project_planar_force_to_root`.

---

## Wave 2: Parallel Physics Kernels & Unit Tests (4 Disjoint Tasks)

### Task 2A: Distributed Tire Kernel Full $J / J^T$ Suspension Coupling & Native Station Test
- **Files:** `native/src/gpu/tire_kernel.hpp`, `native/tests/gpu/test_gpu_tire.cpp`

### Task 2B: Rider Kernel Paired Mapping, `+=` Accumulation & Native Spindle Math Test
- **Files:** `native/src/gpu/rider_kernel.hpp`, `native/tests/gpu/test_gpu_rider.cpp`

### Task 2C: Drivetrain & Resistance Kernel Unified Gear Ratio (`22/51`), Single Torque Path & Native Freehub Test
- **Files:** `native/src/gpu/drivetrain_resistance_kernel.hpp`, `native/tests/gpu/test_gpu_drivetrain_resistance.cpp`

### Task 2D: Suspension Kernel `BIKE_THREAD` Qualification & Accumulation Test
- **Files:** `native/src/gpu/suspension_kernel.hpp`, `native/tests/gpu/test_gpu_suspension.cpp`

---

## Wave 3: Parallel Batch Step & Host Runners (3 Disjoint Tasks)

### Task 3A: Fused Batch Step (`batch_step.hpp`, `cuda_kernels.cu`, `test_gpu_batch_step.cpp`)
- **Files:** `native/src/gpu/batch_step.hpp`, `native/src/gpu/cuda_kernels.cu`, `native/tests/gpu/test_gpu_batch_step.cpp`

### Task 3B: CPU Batch Runner (`cpu_batch_runner.cpp`)
- **Files:** `native/src/gpu/cpu_batch_runner.cpp`

### Task 3C: Metal Batch Runner (`metal_batch_runner.mm`)
- **Files:** `native/src/gpu/metal_batch_runner.mm`

---

## Wave 4: Parallel Python Tooling & CMake/CTest Integration (4 Disjoint Tasks)

### Task 4A: `build_cuda_writers.py` Paired Rider Resolution & Body Indices
- **Files:** `tools/colab_gpu/build_cuda_writers.py`

### Task 4B: `cpu_batch_sweep.py` Heightfield Binary Loader & Gear Ratio
- **Files:** `tools/colab_gpu/cpu_batch_sweep.py`

### Task 4C: `mjwarp_batch_sweep.py`, `mjwarp_metal_sweep.py`, `metal_hybrid_sweep.py` Fixes
- **Files:** `tools/colab_gpu/mjwarp_batch_sweep.py`, `tools/colab_gpu/mjwarp_metal_sweep.py`, `tools/colab_gpu/metal_hybrid_sweep.py`

### Task 4D: CMake/CTest Registration of `native/tests/gpu/*.cpp`
- **Files:** `native/CMakeLists.txt`
