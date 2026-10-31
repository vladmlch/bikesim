# Task numcomp — `reference.native_numeric_completion` red at HEAD (fixture recapture, darwin-arm64)

**Lane**: `.worktrees/pfc-lane-h`, branch `lane/h3`, integration tip
`eb63319e` (post-4.01-ledger-reconciliation). Darwin-only lane; no
container work. All verification on `build/native-release` (231-test
produced graph) plus the live-oracle pytest nodes under
`native/build`'s `bike_native.cpython-313-darwin.so`.

## Target failure

`ctest --test-dir build/native-release -R '^reference.native_numeric_completion$'`
failed at HEAD with

```
FAIL config.rider_controller.leg_track_damping_nms_rad: missing required key
```

emitted inside `test_tire_channel_parity_across_backends` (the file's
last golden node; the C++ port's final `wire_close` consumption never
ran).

## Root cause (verified)

**Stale golden inputs, not a merge artifact — red since `96fe3d86`
itself.**

`96fe3d86` `feat(native): complete weld-press servo + coast_hold_nm wire`
(2026-10-26 10:56) made `weld_servo_gain`, `weld_servo_limit_nm` and
`leg_track_damping_nms_rad` *required* keys on the closed
rider-controller wire (`state_codec.cpp::kConfigRequiredKeys`,
`spindle_wire.cpp` mirror) and activated the weld-press servo /
leg-follow torque in native replay. The commit touched nine files — no
fixture recapture.

`test_tire_channel_parity_across_backends` materializes the oracle's
recorded `config.json`/`state.json` from golden f64-input entries next
to the committed `model.mjb`, then `runtime::read_bootstrap` decodes
them. The recorded `config.json` (last captured `cdd7bec0` 2026-10-26
08:03, on a parallel lane that does not contain `96fe3d86`) predates
the three keys → strict decode fails on the first missing key
(alphabetically `leg_track_damping_nms_rad`). Verified byte-level:
extracted `compliant.file::config.json` /
`distributed.file::config.json` payloads from the committed
`test_tire_channel_parity_across_backends.bin` — `rider_controller`
carries 70 keys, missing exactly those three, no extras.

`git show 96fe3d86~1:…/test_tire_channel_parity_across_backends.bin`
shows the same staleness before the commit — i.e. `96fe3d86` broke the
ctest on its own tree; the later merges merely carried the red forward.
The "pre-existing, not merge-caused" claim in the 4.01 audit is
confirmed (it was additionally an explicitly *documented* skip:
`migration/cpp-linux-completion-evidence.md` records "Explicitly NOT
recaptured — the node's live-oracle contract is red … recorded
f64_close tol 1e-9" for this node at that vintage; the live parity
that blocked recapture then is green at HEAD — below).

## FT-3 discipline applied

1. **Field addition is a reviewed change** — `96fe3d86` is a landed
   feature commit in the F1 welded-plant series (weld servo +
   `coast_hold_nm` channel), documented in commit body and in the
   recapture ledger.
2. **Live-oracle parity green at HEAD** —
   `uv run --frozen --group native python -m pytest
   tests/reference/test_native_numeric_completion.py::test_tire_channel_parity_across_backends`
   under `PYTHONPATH=tests/reference`,
   `NATIVE_TEST_BUILD_PATH=$PWD/native/build`: **1 passed in 240.21 s**
   (the test's `assert_tree_close(native_wire, oracle_wire)` is the
   live parity check). The blocker that justified the original skip is
   gone.
3. **Re-capture** — same command + `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden`
   → node pair rewritten: recorded `config.json` now carries
   `leg_track_damping_nms_rad=0.0`, `weld_servo_gain=0.05`,
   `weld_servo_limit_nm=25.0` (the committed bootstrap values); 574
   entries (was 553): +21 all under `distributed.front.patches[7]` —
   the distributed front wheel gains an eighth contact patch under
   post-servo physics; no labels removed.
4. **Corrupted-fixture negative check** — flipped
   `compliant.front.normal_load_n` leaf +1000.0 →
   `FAIL … outside recorded tolerance`; restored → green. The fixture
   is load-bearing.

## Same-class collateral fixed (stale-after-`96fe3d86` goldens)

A `ctest -L golden` sweep (37 tests) surfaced three more reds; two were
the same defect, one a documented parity gap:

- `reference.native_drive_policies` — `test_pedaling_sequences` and
  `test_pedaling_parameter_branches[raw|filtered|mash]`
  (`$.target_rate_rad_s` 4.0 vs ≈4.043): goldens captured `2edbd7cb`
  (Oct 26 06:06, pre-`96fe3d86`). Whole-file live run 102/102 green
  (file-wide green per `_golden.py`'s commit rule) → whole-file
  recapture; only the four `.bin` payloads changed (`.json` index
  identical → same labels/tolerances, values only). ctest green.
- `reference.period_buffer` —
  `test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms`
  (`expected/qpos` el. 546, outside recorded tolerance): golden
  re-floored post-servo (`a323585d`) but the recorded trajectory
  drifted again through later merges. Live node green at HEAD →
  recapture changed `.bin` only (floor intact). ctest green.
- `reference.native_pneumatic_tire` —
  `test_pneumatic_observations_declare_multi_contact`
  (`front.normal_load_n` ≈5e-8 N residual): **still red — documented
  parity gap, not a stale fixture.** The node records the *extension's
  own* snapshot; a verified recapture reproduced byte-identical values
  while the C++ twin's replay produces its own settle residual — the
  `seal_bootstrap`-vs-`capture_bootstrap` construction-path family
  already documented at ~1.9e-5 N (`ledger` §7, evidence "Reds").
  Live sanity contract passes. Left red deliberately — the fix is a
  parity/physics task (per stop rule), tracked by the documented
  exception rows.

## Secondary: `test_oracle_parity_with_xfrc_writer` fixture provenance

The 4.01 report's untracked pairs at
`tests/fixtures/golden/darwin-arm64/tests_reference_test_pneumatic_tire_forces.py/`
were gone from this worktree — `reference.pneumatic_tire_oracle` was
red at HEAD (`FAIL fixture is not a regular file`), matching the
ledger's "on a clean checkout it would fail" caveat.

Reproduced the capture under provenance that was missing before:
`tests/reference/test_pneumatic_tire_forces.py` whole file **17/17
green**, then `BIKE_GOLDEN_CAPTURE` on
`test_oracle_parity_with_xfrc_writer[fast]` + `[detailed]` → 4 files,
bitwise kind, correct node ids, **64/68 entries — exactly matching the
ledger's description of the lost out-of-band capture** (64/68). So the
lost files are independently reproduced, not merely trusted. Committed
here. Corrupted-fixture check: flipped `step.*` leaf in the `fast`
`.bin` → ctest `Failed`; recaptured → `Passed`.
`linux-aarch64` pairs still owed outright (deferred container lane —
see Debts).

## Verification summary

| Gate | Before | After |
|---|---|---|
| `reference.native_numeric_completion` | FAIL (`leg_track_damping_nms_rad: missing required key`) | **PASS** 0.52 s |
| `reference.pneumatic_tire_oracle` | FAIL (`fixture is not a regular file`) | **PASS** 0.03 s |
| `reference.native_drive_policies` | FAIL (`target_rate_rad_s` drift) | **PASS** 0.38 s |
| `reference.period_buffer` | FAIL (`expected/qpos` outside tolerance) | **PASS** 2.32 s |
| `ctest -L golden` (37 tests, all built) | 5 real reds + missing binaries | **35/37** |
| `reference.native_pneumatic_tire` | FAIL | FAIL — documented parity gap (out of scope) |
| `reference.golden_episode_tool` | FAIL | FAIL — FT-5 owned debt (out of scope) |

`reference.golden_episode_tool` (`attachment_samples.foot_front.normal_n`
39.5 vs 46.0 N at step 0) replays the frozen `pinned_savage_40`
`episode.npz` captured Oct 25 — the same servo-drift vintage, but its
re-freeze is owned by documented fix task **FT-5** ("freeze/equilibrium
non-convergence … pinned_savage_40", honest-red-inventory §Fix tasks);
the committed `physics.toml` freeze path is documented non-convergent,
so it is not a mechanical recapture. Left for FT-5.

## Commands (evidence)

```
cmake --build build/native-release --target reference.native_numeric_completion -j8
ctest --test-dir build/native-release -R '^reference.native_numeric_completion$' --output-on-failure
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH=$PWD/native/build \
  uv run --frozen --group native python -m pytest \
  tests/reference/test_native_numeric_completion.py::test_tire_channel_parity_across_backends -q   # 1 passed (240 s)
# same + BIKE_GOLDEN_CAPTURE=tests/fixtures/golden → recapture
ctest --test-dir build/native-release -L golden --output-on-failure   # 35/37
```

## Debts (documented, not regressions)

- `linux-aarch64` `test_tire_channel_parity_across_backends` pair still
  carries the pre-`96fe3d86` config → will fail on the container lane
  the same way; needs the identical recapture on linux-aarch64.
- `linux-aarch64` `tests_reference_test_pneumatic_tire_forces.py/` —
  no directory at all; both `xfrc_writer` pairs owed (ledger §8.1).
- `test_pneumatic_observations_declare_multi_contact` C++ residual —
  documented construction-path parity family.
- `reference.golden_episode_tool` — FT-5.

## Files changed

```
M tests/fixtures/golden/darwin-arm64/tests_reference_test_native_numeric_completion.py/test_tire_channel_parity_across_backends.{json,bin}
M tests/fixtures/golden/darwin-arm64/tests_reference_test_native_drive_policies.py/test_pedaling_{sequences,parameter_branches_{raw,filtered,mash}_}.bin
M tests/fixtures/golden/darwin-arm64/tests_reference_test_period_buffer.py/test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms.bin
?? tests/fixtures/golden/darwin-arm64/tests_reference_test_pneumatic_tire_forces.py/ (4 files)
```
