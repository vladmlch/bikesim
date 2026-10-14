# Spatial 3D — Phase 7 (Track G): Intra-step Multithreading and the Performance Gate Implementation Plan

> **For agentic workers:** Execute task-by-task with your runtime's plan-execution skill; steps use checkbox (`- [ ]`) syntax. **Prerequisites:** Phase 3 merged (the writer's `n_parts` partition hooks and the single-thread byte-equality test); Phase 4 merged for the full-model performance gate (Tasks 4–6). Read `docs/perf.md`, `docs/TESTING.md` (RTSan section), `tests/reference/test_native_runtime_performance_contract.py`, `native/src/rtsan.hpp`.

**Goal:** A persistent worker pool with spin-barrier synchronisation running the `pneumatic_3d` element pipeline across threads with **bytes identical for every thread count** (N ∈ {1, 2, 4, 8}), RTSan-clean on the tick-path, selectable by `--threads`/`BIKE_NATIVE_THREADS`; the performance gate met — **interactive mean achieved RTF ≥ 1.0 at 1× and headless RTF ≥ 2.0 on the fast tier** on `alpine_rocky_3d` with the free-root model; MuJoCo's `mjThreadPool` measured and enabled only if it gains and stays deterministic; the performance contract tests extended.

**Architecture:** `runtime/worker_pool.{hpp,cpp}`: `N − 1` threads parked on a spin-wait (with `std::this_thread::yield()` after a bounded spin count to avoid burning cores when idle between steps — the yield is outside the hot window and is RTSan-permitted; verify with the RTSan build), a generation counter, static `(begin, end)` ranges computed once per tier, and a `run(ranges, fn)` that executes range 0 on the calling thread. No mutex/condvar on the tick-path; a mutex is allowed only in `start()/stop()`. Reductions stay on the calling thread in fixed order (P3 design). Other candidate parallel regions (per-wheel split, rider support contact math) are measured first; only those with a measured gain are parallelised, under the same determinism test.

**Tech Stack:** C++23 `std::jthread`, `std::atomic` for the generation counter (integers only), RTSan/ASan/TSan builds; `native`/`full` profiles; `native_bench` for RTF.

**Spec:** §10.5, §11.1–11.2. **ADR:** 0002 Layer B (thread-count invariance), D.

## Global Constraints

- All master-plan constraints. Branch `feat/spatial-3d-p7-threads`.
- **Thread-count invariance is a hard gate**: `test_spatial3d_invariants.py::test_bytes_independent_of_thread_count` compares full telemetry and integration state for N ∈ {1, 2, 4, 8} on two tracks — bitwise. No atomics on floats, no dynamic scheduling, no per-thread accumulators whose combination order depends on N.
- RTSan build must pass with the pool active; TSan build (`NATIVE_TSAN=ON`, add the CMake option) must be clean on the pool.
- Planar backends do not use the pool (planar bytes unchanged); the pool is constructed only for `spatial = "3d"`.
- Default thread count `min(8, hardware_concurrency/2)`; `--threads 1` must bypass the pool entirely (no threads created).

---

## File Structure

| File | Responsibility |
|---|---|
| Create `native/src/runtime/worker_pool.{hpp,cpp}` | Pool, spin barrier, static ranges |
| Modify `native/src/writers/pneumatic_3d_tire.cpp` | Call `pool.run(ranges, compute_elements)` when a pool is present |
| Modify `native/src/stepper.{hpp,cpp}`, `binding.cpp`, `src/bike_sim/native/runtime.py`, `cli/*.py` | Thread count plumbing (`--threads`, env) |
| Modify `native/CMakeLists.txt` | `NATIVE_TSAN` option; pool sources in sweeps |
| Modify `native/src/rtsan.hpp` | Annotations for the pool's hot path |
| Modify `tests/reference/test_spatial3d_invariants.py` | Thread-count invariance |
| Modify `tests/reference/test_native_runtime_performance_contract.py` | 3D fast-tier contracts (interactive ≥ 1.0, headless ≥ 2.0) |
| Create `native/tests/test_worker_pool.cpp` | Pool unit tests (start/stop, ranges, generation counter, exceptions never cross) |
| Modify `tools/proto_native_bench/*` or `native_bench` driver | `--threads` and 3D config support |
| Modify `docs/perf.md`, `docs/TESTING.md` | Threads, TSan, new contracts |
| Create `docs/superpowers/audits/2026-10-14-spatial-3d-p7.md` | Scaling tables |

---

### Task 1: Worker pool

- [ ] **Step 1 (failing tests, C++):** `run` executes every range exactly once (coverage bitmap); range 0 on the caller thread; 1 000 consecutive `run` calls with N = 8 complete (no deadlock) under TSan; `stop()` joins; a pool of 1 performs no thread creation.
- [ ] **Step 2:** Implement; bounded spin then yield; generation counter `std::atomic<std::uint64_t>` with acquire/release.
- [ ] **Step 3:** RTSan build: `run` on the tick-path is `BIKE_NONBLOCKING`-annotated and passes. Commit `feat(native): deterministic worker pool with spin barrier`.

### Task 2: Tyre writer on the pool; thread-count invariance gate

- [ ] **Step 1 (failing test):** `test_bytes_independent_of_thread_count`: `alpine_rocky_3d` and `rough_uphill_savage_3d`, free root, fast tier, 5 s, N ∈ {1, 2, 4, 8} → identical telemetry bytes and integration state; detailed tier 1 s likewise.
- [ ] **Step 2:** Wire `compute_elements` through the pool; ranges from `n_parts = N` (or more parts than threads if load-balancing helps — still static).
- [ ] **Step 3:** Commit `feat(native): pneumatic_3d elements on the worker pool, bytes invariant to thread count`.

### Task 3: Profiling and candidate regions

- [ ] **Step 1:** Profile a 10-s free-root fast-tier run (Instruments or `perf`-equivalent on macOS: `xctrace`): table of µs/step by region (MuJoCo step, tyre elements, tyre aggregation/brush, rider support/attachment, drivetrain, recorder).
- [ ] **Step 2:** For each region > 10 % of the step, decide: parallelisable with fixed-order reduction? If yes, implement behind the same invariance test; if no, document why.
- [ ] **Step 3:** Commit `perf(native): parallelise measured hot regions (see audit)`.

### Task 4: `mjThreadPool` measurement

- [ ] **Step 1:** Enable MuJoCo's thread pool (`mju_threadpool(mjData*, int nthread)` — the only thread API in the pinned 3.12.0 header, `mujoco.h:1629`) behind a flag; measure µs/step and check determinism (bytes for 1 vs 4 MuJoCo threads). The model has few constraint islands; expect no gain.
- [ ] **Step 2:** Record the result; keep the flag only if the gain is ≥ 10 % **and** bytes are identical; otherwise remove the code path and note the finding.
- [ ] **Step 3:** Commit `perf(native): mjThreadPool evaluated (<result>)`.

### Task 5: Performance gate and contracts

- [ ] **Step 1:** `native_bench --benchmark` headless, free root, fast tier, `alpine_rocky_3d`, 30 s, default threads: **median RTF ≥ 2.0** (gate). Interactive `bike-ride` 1× for 30 s: **mean achieved RTF ≥ 1.0** (gate; use the existing achieved-RTF reporting). Detailed tier: measured, not gated.
- [ ] **Step 2:** Extend `test_native_runtime_performance_contract.py` with the two 3D contracts (marked `slow`, same structure as the existing ones; the existing planar visual contract stays at its 0.95 constant — the 3D interactive gate is a separate ≥ 1.0 constant).
- [ ] **Step 3:** If a gate fails: reduce `n_lat` in the fast tier (7 → 5) only if P3's consistency gate (2 %) and lateral bands still hold — re-run those; otherwise the plan's budget gets a perf follow-up, not a loosened gate.
- [ ] **Step 4:** Commit `test: spatial 3d performance contracts`.

### Task 6: Docs, audit, gate

- [ ] **Step 1:** `docs/perf.md`: 3D section with the scaling table (N = 1, 2, 4, 8 µs/step), region profile, thread-pool verdict; `docs/TESTING.md`: TSan build, `--threads` in tests.
- [ ] **Step 2:** `native`/`full` green, RTSan/ASan/TSan green. Merge; tag `spatial-3d-p7`.
