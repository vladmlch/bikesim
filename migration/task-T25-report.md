# T25 — contract_sha256 worktree-portability fix (lane/d)

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/pfc-lane-d`
Branch: `lane/d` · Date: 2026-10-25

## Defect

`PhysicalInitialState`'s `contract_sha256` hashed the *resolved
worktree-absolute* `joint_envelope_path`/`joint_strength_path` spellings.
A captured document therefore validated only inside the checkout that
produced it: moving a worktree (the whole point of lane isolation)
invalidated every stored initial-state artifact even though nothing
about the physics or the external asset files had changed.

## Fix (mirrored Python ↔ C++, per the repair sketch)

- **Contract binds asset content, not location.** Both
  `src/bike_sim/sim/ride/initial_state.py::_contract` and
  `native/src/initialization/initial_state.cpp::contract` remove
  `joint_envelope_path`/`joint_strength_path` from the hashed physics
  object and append symmetric `envelope_sha256`/`strength_sha256`
  digests computed over the resolved file bytes (native
  `platform::file_sha256`, `native/src/platform/files.cpp`). Canonical
  JSON production is unchanged (Python-compatible escaping + `bio::sha256`
  vs `json.dumps(sort_keys=True, separators=(',', ':'))` + `hashlib.sha256`).
- **Schema version 2.** Documents emit `schema_version: 2`; readers
  require exactly 2 — v1 fails closed as stale, v3 as unknown (pinned in
  `native/tests/migration/test_t24.cpp`).
- **Platform-keyed siblings.** Compiled-model arrays and equilibrium
  values are platform-sensitive, so artifacts live under
  `initial_state/<system>-<machine>/`. `platform::key()`
  (`native/src/platform/provenance.cpp`, baked
  `CMAKE_SYSTEM_NAME-CMAKE_SYSTEM_PROCESSOR` lowercased — identical to
  the test graphs' `BIKE_GOLDEN_PLATFORM` string) lets
  `initialization/request.cpp` and `research/configuration.cpp` prefer
  `<parent>/<platform-key>/<filename>` when it is a regular file, else
  retain/fail closed on the shared spelling. The obsolete shared
  `initial_state/welded_flat.json` is removed.
- **Platform-keyed bootstrap pin.** The request pipeline's restored t=0
  integration vector is platform output too. `test_t25.cpp` resolves
  `bootstrap/native_start_welded/<platform-key>/integration.f64` via
  `platform::key()` and fails closed when the platform pin is absent —
  it never substitutes a foreign platform vector. The shared
  `integration.f64` is retained for the T09/T18-era consumers.
- **`--dump-integration-state FILE`** added to `bike-native-ride`
  (`native/src/app/native_ride_main.cpp`): runs after `make_ride`,
  writes the restored t=0 integration vector as little-endian `.f64`
  with exclusive create — the reproducible producer for the keyed pins.
- **Probe docs are input artifacts.** `probe_welded_0005.json` feeds
  `reference.period_buffer`'s pinned input; the wire family under
  `tests/fixtures/golden/<platform>/period_buffer/` was frozen against
  the captured pose, and settle drifted ~2.6e-3 (`qpos[0]`) under
  intervening physics commits. The documents were re-pinned
  (schema 2 + new contract + current `source_sha256`) **onto the frozen
  state values** rather than recaptured — the contract binds model/
  config identity, not stored values. `_t52_capture.py` documents the
  coupling (pose recapture requires re-freezing the wire family).
- **Docs/tools.** `docs/native-startup.md` records the sibling
  convention and the dump flag; `tools/migration/_t25_capture.py`
  (welded recapture), `_t52_capture.py` (probe + warning),
  `capture_reference.py` (platform-keyed emission), and
  `tests/reference/test_native_pneumatic_tire.py` + golden physics
  TOMLs (bundle-relative asset paths) updated.

## Commits

| commit | contents |
|---|---|
| `59b3c4e4` | contract fix (both implementations), schema 2, `platform::key()`, sibling preference, docs, darwin artifacts, manifest |
| `46c74e41` | linux-aarch64 initial-state captures, platform-keyed integration pins, `--dump-integration-state`, T24/T25 updates |
| `fcaf7682` | probe doc re-pin on frozen pose, linux/probe manifest entries, `_t52_capture.py` input-artifact warning |

## Artifacts (sha256 / bytes)

| path | sha256 | bytes | contract_sha256 |
|---|---|---|---|
| `initial_state/darwin-arm64/welded_flat.json` | `bd5972a2…` | 23545 | `dc9ce816…` |
| `initial_state/linux-aarch64/welded_flat.json` | `2abb3255…` | 23571 | `639f49c5…` |
| `initial_state/darwin-arm64/probe_welded_0005.json` | `6b97511f…` | 23384 | `dc9ce816…` |
| `initial_state/linux-aarch64/probe_welded_0005.json` | `38d39ea0…` | 23364 | `639f49c5…` |
| `bootstrap/native_start_welded/darwin-arm64/integration.f64` | `b7e2d0ae…` | 2336 | — |
| `bootstrap/native_start_welded/linux-aarch64/integration.f64` | `641bfea6…` | 2336 | — |

`migration/oracle-manifest.json`: linux `welded_flat` entry added,
both probe entries added (group `research`), full-manifest drift check
clean. Platform contracts differ (`dc9ce816` vs `639f49c5`) because the
contract pins compiled-model arrays — expected and desired.

## Verification

**Darwin arm64**
- `migration_T24` 1.86 s · `migration_T25` 10.16 s — pass (cold equilibrium cache `BIKE_SIM_EQUILIBRIUM_CACHE_DIR=/tmp/t25-cold*`).
- `reference.period_buffer` 3.62 s · `reference.native_pneumatic_tire` 0.71 s — pass, cold cache.
- `migration_T22` 45 checks · `migration_T23` 349 checks — pass (the first sweep red was a stale pre-commit binary; rebuilt = green).
- `--dump-integration-state` reproduces `darwin-arm64/integration.f64` byte-for-byte.
- **Cross-path proof:** temp worktree `/private/tmp/t25-pathproof` at `59b3c4e4` (`.deps` linked) — release build, `migration_T25` pass; the document validates from a different checkout path — the defect scenario itself. Worktree removed after.

**Linux aarch64** (`bike-dev-pfc-lane-d`, `/build/root-clang`, clang++-18)
- `migration_T24` 3.22 s · `migration_T25` 27.63 s · `migration_T22` 9.66 s · `migration_T23` 1.41 s — pass (cold cache `/tmp/t25-cold-linux`).
- `reference.period_buffer` 3.61 s · `reference.native_pneumatic_tire` 0.74 s — pass.
- `--dump-integration-state` reproduces `linux-aarch64/integration.f64` byte-for-byte.
- Fail-closed evidence: pre-recapture the reader refused the absent linux artifact (`platform::open_regular … welded_flat.json: No such file or directory`) — no foreign-platform substitution.

**Gate: `bash tools/run_tests.sh native` (Darwin)** — exit 8 at the
CTest phase (pytest phase not reached). Build + **all static sweeps
green**: frontends (g++-16, none), clang-tidy (333 TUs), alpha analyzer
(report-only), cppcheck (none), gcc `-flto -Wodr` (none), shellcheck
(clean), headers (8950 header TUs, none), diagnostic-controls (6/6),
context SDK controls (all passed). CTest **60/62** — the only reds are
pre-existing, input-determined failures untouched by this task:

- `migration_T09` — `integration vector resolves bitwise`:
  `bootstrap/native_start_welded/state.arrays/a000.f64` (the oracle
  capture embedded in `state.json`) carries nonzero leg-control values
  at indices 87–90; the `integration.f64` pin is the native-shaped
  vector (those slots zeroed). 4-element mismatch, deterministic from
  inputs unchanged since `fe4feaa5`/`e3c541b9` — predates T25.
  Identical signature on linux-aarch64.
- `migration_T29` — `config.rider_controller.weld_servo_gain: unknown
  key`: test and `native/src/configuration/` untouched since
  `fe4feaa5`/`ef66e96a` — a config-key contract red predating T25.

(`migration_T56`'s shellcheck-version string failure from the targeted
sweep did not reproduce in the gate — `check_scripts` passed clean.)

`git diff HEAD~3` confirms none of the T09/T29 inputs, tests, decode,
or config paths were modified by this change set; both fail on their
inputs regardless of T25.

## Native-phase pytest (supplementary, run outside the gate)

Same selection the gate's pytest phase owns
(`tests/reference/test_native_*.py` + `test_golden_episode*.py`,
`-p native_test_reporter`, `NATIVE_TEST_BUILD_PATH=$PWD/native/build`,
`uv`-only env): **75 failed / 3164 passed / 1 skipped in 5809 s**
(1 h 37 m — host shared with three other lanes' header sweeps).

Classification — none of the 75 is attributable to T25:

- `test_native_pneumatic_tire.py::test_pneumatic_probe_and_clock` —
  **documented baseline** (TESTING.md 2026-10-24 addition: recorded
  probe interval diverges bitwise from live simulation at step 1).
- Remaining 74 = lane-level **oracle-parity reds** in physics areas
  disjoint from this change set (`git diff HEAD~3` confirms: contract
  hashing, path resolution, request/research readers, provenance key,
  dump flag, fixtures, tools, docs — nothing in step/settle/spindle/
  drivetrain/tire numerics). Signatures range from low-digit drift
  (`test_native_first_step_uses_settled_material_state`,
  `2.00995945` vs `2.00995795` at `qpos[1]`) to substantive port gaps
  (`test_native_spindle_controller`, `rider_hip_front` torque
  `-0.968` vs `-1.541`). Consistent with the 2026-10-24 physics
  landings (F2 brake-hold re-anchor `051497c9`/`a27a8d6c`, lane
  3.12b/3.14a/3.16 batches) — committed with recorded pre-existing
  reds ("verified failing at HEAD" in `0fdabe23`); the `native`
  profile was never run at lane level before this gate, so the red set
  was unmeasured, not introduced.
- **Zero** failures in the T25 surface: no `initial_state`,
  `period_buffer`, `bootstrap`, `welded`, or T24/T25-adjacent node
  appears in the failure list.

## Residual notes

- `probe_welded_0005.json` state values are intentionally frozen at the
  wire-family pin; `_t52_capture.py` explains the coupling. A future
  deliberate wire re-freeze (both platforms) may then recapture poses.
- T09/T29 are open pre-existing reds owned by other fixture families
  (bootstrap capture / config-key contract) — classified, not repaired,
  per scope discipline.
