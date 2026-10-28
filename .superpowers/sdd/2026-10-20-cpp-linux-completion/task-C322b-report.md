# Task C3.22b report — recorder/HUD columns, period buffer, frozen arrays, plot markers (T52)

Worktree `.worktrees/pfc-lane-d`, branch `lane/d`, base `7e683b75`
(integration tip). Batch scope: five pytest files — 22 port-map
records / 24 collected nodes
(`tests/reference/test_recorder_columns.py` 4,
`test_hud_columns.py` 2, `test_period_buffer.py` 11,
`test_frozen_force_arrays.py` 2,
`test_ride_plots_html_markers.py` 3).

## SHAs

- `152aa0de` — lane/a port of all five files (merged `9c08ff17`);
  verified in place on lane/d at `7e683b75`.
- `e181666b` / `a323585d` — post-merge F1 oracle edits the port already
  carries (delivered-budget config fields; darwin parity-floor
  recapture 5e-11/5e-10/5e-9).
- This commit — ledger qualification (22 records), the new
  `test_equality_rows_charge_the_closure_channel_not_the_bound` port,
  `python-removal.json` fingerprint refresh, evidence section.

## Dispositions

| File | Records | Disposition | Native coverage |
|---|---|---|---|
| `test_recorder_columns.py` | 4 | port ×4 | `reference.recorder_columns::<symbol>` ×4 |
| `test_hud_columns.py` | 2 | port ×1, drop ×1 | `reference.hud_columns::test_arm_lean_limit_and_balance_columns` (3-param loop); `test_arm_highlight_uses_published_directional_capacity` → drop — `HudColumn` is `{legend, text}` only (`native/src/viewer/hud.hpp:6`), no styled-segment surface for the `orange1` check; the torque data it colored is asserted in the ported arm column text (`sh  +90`) |
| `test_period_buffer.py` | 11 | port ×9, golden ×2 | `reference.period_buffer::<symbol>` ×11; golden nodes `test_non_aligned_flush…` + `test_runtime_batch…` replay two-platform fixtures under `tests/fixtures/golden/<platform>/tests_reference_test_period_buffer.py/` |
| `test_frozen_force_arrays.py` | 2 | port ×2 | `reference.frozen_force_arrays::<symbol>` ×2 |
| `test_ride_plots_html_markers.py` | 3 | port ×3 | `reference.ride_plots_html_markers::<symbol>` ×3 |

Totals: **19 port / 2 golden / 1 drop**; 24/24 collected nodes mapped;
zero `planned` records in the batch.

## Post-merge drift closed on lane/d

- `e181666b` (F1 re-scope, post-merge) appended
  `test_equality_rows_charge_the_closure_channel_not_the_bound`
  (file line 383) without a ledger row — the file ran 11 live
  functions against 10 records. Ported on lane/d: the oracle's
  `split_constraint_powers` partition is natively `raw_work`'s
  `equality:` lift inside `account_energy`, so the test drives it
  end-to-end through `strictness_support::runtime_for_raws` +
  `neutral_raw` (the energy-ledger port's fixture for the same raw
  path) and repeats the verbatim `runtime::step_work` call at the
  oracle's `dt=.1` inputs. Expected values written as identical
  binary64 expressions (`.75*.1`, `3.*.1`) — the `.075`/`.3` literals
  differ from the computed products by 1 ulp.
- The same F1 commits' other edits are already carried: `093633c4`
  grew the `AttachmentBudget` designated initializers; the darwin
  golden JSON manifests hold `a323585d`'s recaptured floors.

## Verification

- Oracle suite, all five files: **24 passed** darwin-arm64 (53.13 s,
  `.venv/bin/python -m pytest`) and **24 passed** linux-aarch64
  (64.62 s, `bike-dev-pfc-lane-d`, `uv` venv cpython-3.13.16);
  `--collect-only` reproduced all 24 node IDs including the three HUD
  `[stateN-…]` parameterizations.
- Focused CTest
  `reference.(recorder_columns|hud_columns|period_buffer|frozen_force_arrays|ride_plots_html_markers)`:
  **5/5 PASS** darwin-arm64 (`build/native-release`, Release; 5.53 s)
  and **5/5 PASS** linux-aarch64 (`/src/build/native-release`,
  clang, Release; 3.29 s). Check counts identical across platforms:
  297 / 44 / **160268** / 8 / 18 (`period_buffer` = 160265 lane/a
  + 3 new checks).
- No `run_tests.sh` per lane restriction; no pytest files edited (no
  new capture sites needed).

## Ledger

- `migration/test-port-map.json`: 2393 → **2394** records; headers
  `static_test_functions` 2393→2394, `top_level_test_functions`
  2377→2378 (= 2394 − `class_test_methods` 16) per the `bd0f966f`
  precedent. Dispositions `port` 1062→1063, `drop` 125, `golden` 351,
  `planned` 855. All 22 batch records carry measured
  `collected_node_ids` (24 nodes), dated `assertions_capture`
  (2026-10-28), `native_test_id` `reference.<target>::<fn>` (null on
  the drop), honest `comparison_class`, and lane/d verification
  `evidence` appended after the lane/a entries; `line`/`end_line`
  refreshed against the post-`e181666b` source.
- `migration/python-removal.json`: `test_period_buffer.py` `sha256`/
  `line_count` refreshed to live (505 lines, `2ba3b176…`); other four
  rows already matched; `snapshot_*` untouched; `replacement_landed`
  117 rows.
- `native/tests/reference/Phase3Ports.cmake`: unchanged — five
  registrations (`t52 recording`, `slow` where the source marks,
  `golden` on `period_buffer`) pre-exist from the lane/a merge.
- Canonical serialization verified: `indent=2`, `ensure_ascii`,
  trailing LF; `git diff` confined to batch rows.

## Scope limits

- `test_arm_highlight_uses_published_directional_capacity` stays
  `drop` (justified above); the three remaining `drop`s in the file
  family are out of batch scope.
- The two `golden` records re-verified against existing two-platform
  fixtures — no recapture on lane/d (payloads unchanged since
  `a323585d`).
