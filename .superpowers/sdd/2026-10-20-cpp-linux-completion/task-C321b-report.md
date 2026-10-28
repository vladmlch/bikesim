# Task C3.21b report — numeric domains + Numba kernel contracts (T52)

Worktree `.worktrees/pfc-lane-d`, branch `lane/d`, base `dfe8db96`
(integration tip). Batch scope: two pytest files — 18 port-map
records / 206 collected nodes
(`tests/reference/test_native_numeric_domains.py` 10 records / 198
nodes, `tests/reference/test_numba_kernels.py` 8 records / 8 nodes).

## SHAs

- `90b311a4` — ledger(3.21b): qualify both files — 14 port, 4 drop,
  both-platform evidence (this commit's ledger payload).
- No new C++ this batch: both ports pre-exist from Phase 3
  (`b21c63ec`/`ef66e96a`) and are re-qualified here —
  `native/tests/reference/test_native_numeric_domains.cpp` registered
  `Phase3Ports.cmake:65` (`t52 numerics`, `bike_app`),
  `native/tests/reference/test_native_kernel_contracts.cpp` at
  `Phase3Ports.cmake:70` (`t52 numerics`, `bike_app`).

## Dispositions

| File | Records | Disposition | Native coverage |
|---|---|---|---|
| `test_native_numeric_domains.py` | 10 | port ×10 | `reference.native_numeric_domains::<symbol>` ×10 — shared-domain rejection matrix (168-case parameterization: air-spring/damper/coil/end-stop fields × invalid values), damper field validator, air-pressure + coil + policy-derived overflow, HBO zero-denominator, mutated policy configs, cruise/brake topology rejections |
| `test_numba_kernels.py` | 8 | port ×4, drop ×4 | `reference.native_kernel_contracts::{test_directional_capacity_error_paths_match,test_brush_step_error_paths_match,test_normal_contact_error_paths_match,test_box_pad_contact_inside_and_tie_cases}`; the four `test_*_matches_python` Hypothesis equivalences → drop (D14) |

Totals: **14 port / 4 drop / 0 golden**; 206/206 collected nodes
mapped; zero `planned` records in the batch.

## Port fidelity notes

- `test_suspension_domain_is_shared`: the C++ iterates the same
  Cartesian product as the Python `CASES` table (air_spring 6×5 +
  2×4, fork/shock damper 5×5 + 6×4 each, coil 4×5 + 1×4, end_stops
  2×4 = 168 rows) and drives the `SuspensionWriter` construction —
  the native seam the oracle's `load_native().Stepper` reaches; the
  oracle-side `python_suspension` leg is T63-dead, so the native
  contract is the gate.
- `test_mutated_policy_configs_are_rejected` /
  `test_policy_derived_overflow_is_shared`: C++ projects
  `cfg::AssistConfig{profile="bosch_cx_gen4",mode="turbo"}` where the
  oracle's `applier()` leaves `profile=None`. Immaterial: all 11
  mutations are scalar domain checks (`tau=0`, `slew=0`, `gain=-1`,
  `taper_width_mps=0`, …) and all four overflow probes use `1e308`
  inputs that overflow profile-independently — verified green on both
  platforms.
- `test_activated_zero_hbo_denominator_rejected`: C++ mirrors the
  legacy/non-legacy split verbatim — non-legacy rejects at writer
  construction, legacy rejects at `components()` with
  `qpos[fork_qposadr]=.181`, `qvel[fork_dofadr]=.7`.
- Topology records carry the oracle's MuJoCo XML verbatim (joint
  kinds/axes, actuator targets `front_wheel_spin`/`rear_wheel_spin`,
  `torque_ceiling_nm=200`, `taper_radps=1`; cruise uses the oracle's
  `CONFIG` 25/180/150/150).
- Kernel error-path ports replay the oracle's argument tuples exactly
  (`1e200` overflow pairs, the `Fn==0`/`mu==0` airborne ordering
  probe); `test_box_pad_contact_inside_and_tie_cases` asserts
  analytic point/normal/gap/footprint for the same 6 centers × 2
  radii — `contract` class, not the twin equivalence.

## Drop triage (per-function, plan 3.21b note / D14)

All four `drop` records are `@given(max_examples=1000)` Hypothesis
tests comparing a numba-JIT kernel against its `*_py` twin
(`directional_capacity`, `_brush_step`, `_normal_contact`,
`_box_pad_contact`). `tests/reference/conftest.py::_pin_oracle_kernels`
is scoped to `test_native_*` files (early return at line 167), so
these tests exercise the **real njit path** — a Python-vs-Python-JIT
equivalence whose subject disappears entirely at qualified Phase-4
removal (T63). Native kernel coverage is named per record:
`reference.native_kernel_contracts` (error-path contracts,
energy/friction and physical boundaries, deterministic inside/tie
cases) plus migration `kernel_properties`/`test_t23`/`test_t30`.
The evidence explicitly does not claim the analytic controls
reproduce the 1000-example comparisons.

## Verification

- Oracle suite (both files):
  `uv run --frozen --group native python -m pytest -q` —
  **206 passed** darwin-arm64 (14.59 s) and **206 passed**
  linux-aarch64 (16.88 s, container `bike-dev-pfc-lane-d`,
  `NATIVE_TEST_BUILD_PATH=/build/native-clang`; the extension was
  rebuilt first — sources had drifted since its 2026-10-26 build).
  `pytest --collect-only` reproduced all 206 node IDs, including the
  168 `[locationN-value]` suspension-domain expansions.
- Darwin (`build/native-release`, AppleClang, Release; targets
  re-linked before the run):
  `ctest -R 'reference\.(native_numeric_domains|native_kernel_contracts)'
  --output-on-failure` → **2/2 PASS** — `native_numeric_domains`
  1.07 s, `native_kernel_contracts` 0.60 s.
- Linux aarch64 (`bike-dev-pfc-lane-d`, clang++-18,
  `/build/root-clang` Release): same selector → **2/2 PASS** —
  `native_numeric_domains` 0.04 s, `native_kernel_contracts` 0.01 s.
- Direct binary runs: **PASS 406 checks**
  (`native_numeric_domains`) and **PASS 180 checks**
  (`native_kernel_contracts`), identical counts on both platforms.
- No `run_tests.sh` per lane restriction; no pytest files edited; no
  golden fixtures captured (zero `golden` records — every port is
  `contract`-class).

## Ledger

- `migration/test-port-map.json`: all 18 records updated in place —
  measured `collected_node_ids` (206 nodes; `[locationN-value]` and
  section/key parameterizations), `assertions_capture` dated
  2026-10-28 ("oracle green darwin-arm64 + linux-aarch64"),
  `native_test_id` `reference.<target>::<symbol>` (`null` on the 4
  drops), honest `comparison_class` (`contract` ×14, `python-only`
  ×4), dated lane/d `evidence` replacing the "source disposition
  only"/"T01 assigns" placeholders; `line`/`end_line` verified
  current (no drift). Dispositions were pre-assigned by the Phase-3
  source review — ledger counters unchanged (port 1063, drop 125,
  golden 351, planned 855).
- `migration/python-removal.json`: both rows → `replacement_landed`.
  `test_native_numeric_domains.py` already carried live
  `sha256`/`line_count` (229, `bf192eff…`);
  `test_numba_kernels.py` refreshed to live (181 lines, `ace705a8…`,
  file grew 3 lines since the T01 snapshot). `verification_ids` =
  `reference.native_numeric_domains` / `reference.native_kernel_contracts`
  (real CTest targets); `phase3_source_implementation` blocks removed
  (landed schema); `snapshot_*` untouched. `replacement_landed`
  117 → 119.
- `native/tests/reference/Phase3Ports.cmake`: unchanged — both
  registrations pre-exist (`t52 numerics`; neither file is
  `slow`-marked in the plan, none carries `golden`).
- Canonical serialization: `indent=2`, `ensure_ascii`, trailing LF;
  `git diff` confined to the batch rows (no `source_symbol`/
  `source_path` churn).

## Counters

- Records resolved: 18 (14 port, 4 drop, 0 golden, 0 planned).
- Collected nodes covered: 206/206.
- New native test targets: 0 — two pre-existing targets re-qualified
  (`reference.native_numeric_domains` 406 checks,
  `reference.native_kernel_contracts` 180 checks).
- Golden fixtures added: none.
- Ledger rows: 18 port-map rows rewritten; 2 removal rows
  `replacement_landed` (total 117 → 119).

## Remaining / caveats

- The 4 `test_*_matches_python` records stay `drop` — their subject
  (JIT-vs-twin equivalence) is Python-internal; native kernel
  assurance rests on `reference.native_kernel_contracts` +
  `kernel_properties`/`test_t23`/`test_t30` as recorded.
- Deletion of both pytest files stays gated at T63; the oracle suite
  remains green on both platforms and is the qualification gate
  until then.
- Numba warmup caveat (AGENTS.md): the oracle run's 14–17 s includes
  njit work on warm caches; no performance claim is made from these
  timings.
