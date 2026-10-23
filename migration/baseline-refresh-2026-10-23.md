# Baseline refresh — 2026-10-23

Re-measured the documented 13-node macOS failure residual
(`docs/TESTING.md` ~lines 803–817) at commit `fe4feaa`
(`fe4feaa560df68bce0e15b5b0d8d76eced29a062`, detached worktree
`.worktrees/pfc-baseline`). No tracked files were modified anywhere; this
report is intentionally untracked.

Host: macOS arm64 (Darwin), AppleClang 21.0.0.21000334, CPython 3.13.12
(uv), MuJoCo 3.12.0, NumPy 2.5.2. Selected build: `native/build`
(artifact `bike_native.cpython-313-darwin.so`).

## Commands run (verbatim)

```sh
# worktree + build prep
git -C .worktrees/python-free-cpp worktree add .worktrees/pfc-baseline fe4feaa --detach
ditto .worktrees/python-free-cpp/.deps .worktrees/pfc-baseline/.deps   # gitignored dep SDKs, lock-verified at configure
cd .worktrees/pfc-baseline
uv run --frozen --group native cmake -S native -B native/build
uv run --frozen --group native cmake --build native/build

# pass 1 — all 13 documented nodes in one process (numba warmup + verdicts)
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH=$PWD/native/build \
  uv run --frozen --group native python -m pytest -q --durations=0 \
  "tests/reference/test_closed_form_rider.py::test_flat_launch_delivers_crank_torque" \
  "tests/reference/test_joint_strength.py::test_stepped_effort_respects_strength_and_the_power_budget" \
  "tests/reference/test_realistic_pedelec_acceptance.py::test_flat_reaches_25_kmh_within_10_s" \
  "tests/reference/test_realistic_pedelec_acceptance.py::test_motor_follows_the_rider_through_a_shift_without_an_extra_cut" \
  "tests/reference/test_realistic_pedelec_acceptance.py::test_fifteen_percent_climb_holds_12_kmh" \
  "tests/reference/test_realistic_pedelec_acceptance.py::test_savage_is_ridden_to_the_end" \
  "tests/reference/test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds[rough_uphill_savage.toml]" \
  "tests/reference/test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds[rough_uphill_extreme.toml]" \
  "tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[20.0]" \
  "tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[40.0]" \
  "tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[60.0]" \
  "tests/reference/test_seated_pedaling_cycle.py::test_free_coast_is_an_intent_not_a_crank_lock" \
  "tests/reference/test_seated_posture_program.py::test_pulse_redistributes_load_through_inertia_only"
# → 2 passed, 11 failed in 731.58s

# pass 2 — each of the 11 failures rerun in its own pytest process
# (warm on-disk numba cache) via a bash loop:
#   uv run --frozen --group native python -m pytest -q --tb=short <node>
# All 11: rc=1, signatures identical to pass 1 → deterministic, zero flakes.

# Linux cross-check (Task 3.11 nodes), container bike-dev-pfc-baseline,
# image bike-dev-linux:24.04 (Ubuntu 24.04 aarch64, clang++-18,
# CPython 3.13.16 at /opt/bike-venv):
bash tools/linux/container.sh up
bash tools/linux/container.sh sync
bash tools/linux/container.sh venv
bash tools/linux/container.sh configure native-clang native \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
bash tools/linux/container.sh build native-clang bike_native
bash tools/linux/container.sh exec bash -c 'PYTHONPATH=tests/reference \
  NATIVE_TEST_BUILD_PATH=/build/native-clang \
  uv run --frozen --group native python -m pytest -q \
  tests/reference/test_native_platform_recording.py::test_publication_permission_denied_then_retry \
  tests/reference/test_native_runtime_commands.py::test_engine_failure_keeps_committed_prefix \
  tests/reference/test_native_runtime_ownership.py::test_fatal_engine_error_preserves_completed_prefix_and_can_reset'
# → 3 passed in 20.97s
```

## Per-node verdicts — macOS arm64 @ fe4feaa

| # | Node | Verdict | Duration (pass 1 / rerun) | Failure signature |
|---|------|---------|---------------------------|-------------------|
| 1 | `test_closed_form_rider.py::test_flat_launch_delivers_crank_torque` | **PASS** | 43.14 s | — |
| 2 | `test_joint_strength.py::test_stepped_effort_respects_strength_and_the_power_budget` | **PASS** | 14.30 s | — |
| 3 | `test_realistic_pedelec_acceptance.py::test_flat_reaches_25_kmh_within_10_s` | FAIL | 42.12 s / 43.99 s | `assert mean(60<=cadence<=115 over steady pedaling) >= 0.8` → `0.7628458498023716` (line 149) |
| 4 | `test_realistic_pedelec_acceptance.py::test_motor_follows_the_rider_through_a_shift_without_an_extra_cut` | FAIL | 43.22 s / 43.55 s | `assert motor/human >= 2.5` → `(t=1.17, human=19.547, motor=6.037)` ≈ 0.309 (line 169) |
| 5 | `test_realistic_pedelec_acceptance.py::test_fifteen_percent_climb_holds_12_kmh` | FAIL | 122.61 s / 94.60 s | `assert not _violations_after(rows)` → steps 401+: `('saddle.friction', 'rider_power.positive')` from t≈0.501 (line 184) |
| 6 | `test_realistic_pedelec_acceptance.py::test_savage_is_ridden_to_the_end` | FAIL | 168.82 s / 139.99 s | `RuntimeError: physical equilibrium did not converge after 64000 steps: residual=0.094603` (`front_wheel_spin` dominant) — `physical_equilibrium.py:141` |
| 7 | `test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds[rough_uphill_savage.toml]` | FAIL | 169.17 s / 143.60 s | same equilibrium non-convergence: `RuntimeError … 64000 steps: residual=0.094603` |
| 8 | `test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds[rough_uphill_extreme.toml]` | FAIL | 45.51 s / 33.19 s | `assert report['factor'] >= 1.` → 0.6176 (rerun 0.6239); sim reached 16000 steps / 20.0 s, wall ≈32 s |
| 9 | `test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[20.0]` | FAIL | 12.50 s / 1.22 s | `InvalidReferenceRun: invalid reference step at 0.005: ('energy.constraint_work',)` |
| 10 | `test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[40.0]` | FAIL | 0.39 s / 1.11 s | same `energy.constraint_work` at t=0.005 |
| 11 | `test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[60.0]` | FAIL | 0.37 s / 1.29 s | same `energy.constraint_work` at t=0.005 |
| 12 | `test_seated_pedaling_cycle.py::test_free_coast_is_an_intent_not_a_crank_lock` | FAIL | 0.38 s / 1.14 s | same `energy.constraint_work` at t=0.005 |
| 13 | `test_seated_posture_program.py::test_pulse_redistributes_load_through_inertia_only` | FAIL | 62.79 s / 1.01 s | `InvalidReferenceRun: invalid reference step at 0.0005: ('saddle.friction', 'foot_rear.normal', 'foot_rear.friction')` |

All failure signatures are byte-identical between the batch run and the
process-isolated reruns → **zero flaky nodes**. The `extreme` realtime
factor is wall-clock-derived (0.6176 vs 0.6239 across runs) but fails
deterministically (~0.62 ≪ 1.0).

### Delta vs the documented 13-node residual

- **Now green (2):** `test_flat_launch_delivers_crank_torque`,
  `test_stepped_effort_respects_strength_and_the_power_budget`
  (the documented baseline signature for the former was "0.5875 vs
  required 0.8" — no longer observed).
- **Still red (11):** nodes 3–13 above. Notable signature drift vs the
  old record: the weld/equilibrium residual is now `0.094603` after
  **64,000** steps (was `0.197922` after 40,000); `energy.constraint_work`
  strict-mode `InvalidReferenceRun` now dominates the seated-cycle family.
- **Flaky (0).**

## Linux aarch64 cross-check — Task 3.11 nodes @ fe4feaa

Container `bike-dev-pfc-baseline` (fresh, created from
`bike-dev-linux:24.04`); selected build `/build/native-clang`
(`bike_native.cpython-313-aarch64-linux-gnu.so`, extension subgraph only —
124 ninja edges; `BIKE_MUJOCO_ERROR_ABI_312` probe passed at configure;
numerics record `linux-aarch64` matched live measurement).

| Node | Verdict | Duration |
|------|---------|----------|
| `test_native_platform_recording.py::test_publication_permission_denied_then_retry` | PASS | (shared run) |
| `test_native_runtime_commands.py::test_engine_failure_keeps_committed_prefix` | PASS | (shared run) |
| `test_native_runtime_ownership.py::test_fatal_engine_error_preserves_completed_prefix_and_can_reset` | PASS | (shared run) |

`3 passed in 20.97s` — all three remain fixed at `fe4feaa`.

## Concerns

- The task's suggested `-k` command scoped only
  `test_native_platform_recording.py`; the three Task-3.11 nodes live in
  three different files, so the literal `-k` selects just 1 of 3. The
  three explicit node IDs were run instead (strict superset of intent).
- Docker daemon was down at session start; started Docker Desktop, then
  provisioned a fresh per-worktree container. `container.sh` names it
  after the worktree basename (`bike-dev-pfc-baseline`), so the existing
  `bike-dev-python-free-cpp` container was not reused.
- `.deps` was copied from the sibling worktree (`ditto`) rather than
  re-downloaded via `tools/linux/acquire_deps.sh`; the lock re-verified
  all archives/files during configure (gitignored content, no tracked
  impact).
- Only the `bike_native` extension target was built in the container,
  not all 447 targets — sufficient for the three binding-level nodes but
  not a full-lane gate.
- Pass-1 durations for several nodes include one-time numba `njit`
  compile cost (cold disk cache); process-isolated reruns with a warm
  cache show the true per-node cost (e.g. `test_pulse_…` 62.79 s →
  1.01 s — it dies on the first strict step once compile is amortized).
- Both realtime-gate and seated-cycle nodes spend their runtime inside
  Python-oracle physics (`native_module` is not used by these tests);
  failures here are reference-side, not binding regressions.
- Worktree `.worktrees/pfc-baseline` retained per repo rule D19.
