# Remove `joint_passive_damping_nms_rad` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the passive joint-damping mechanism (`joint_passive_damping_nms_rad` config knob, MJCF `damping` write, controller passive terms, and the `rider_passive_power_w`/`rider_passive_work_step_j`/`passive_damping_nm`/`solved_passive_nm` telemetry it feeds), while keeping decoders tolerant of the old wire keys so historical recordings still load.

**Architecture:** Write-side removal everywhere; read-side demotion to optional/tolerant wherever a decoder can meet artifacts written by older builds. The `rider_passive_damping` **force-component channel is kept** — it mirrors `qfrc_passive` at rider DOFs (engine truth, not the knob) and keeps `human_joint_power_w`, `loss_increment`, `engine_passive` subtraction, and `rider_work_ledger` bitwise-stable for all current configs (damping was already 0.0 in every shipped TOML).

**Tech Stack:** C++20 native port (`native/src`), CMake presets (`cmake --preset` / `cmake --build --preset`), ctest targets under `native/tests/`, TOML/JSON fixtures. No Python remains (python-free-cpp); `tests/` holds fixtures only.

**Spec:** `docs/superpowers/specs/2026-10-03-v2-seated-plant.md` — Q13 decision (2026-10-04): passive joint damping = 0 in the closed-form plant; derivative action only via active muscle commands. Every shipped config already pins `joint_passive_damping_nms_rad = 0.0`.

## Global Constraints

- Docs touched by behavior change only; `docs/researches/`, `docs/superpowers/specs|audits`, `migration/cards`, and report markdown are historical artifacts — do not edit them to erase history.
- `migration/*.json` manifests ARE consumed by native tests (`native/tests/support/migration_test.*`, `native/tests/migration/test_*.cpp`) — sync them when their referenced config fields/test names change.
- JSON fixtures must keep byte-level formatting/key order — line-oriented deletion only, mind trailing commas; never re-serialize through a JSON library.
- `native/src/recording/catalog_data.inc` is a static transcription of `native/resources/recording/catalog-v1.json` (raw-literal split); the two must stay identical in content — edit both.
- Numba/oracle caveats from AGENTS.md do not apply here (no Python kernels remain), but bitwise determinism of recordings still does: emit nothing that changes recorded bytes for configs that were already `0.0`.
- Verification budget (user constraint): no long gates. Per-task verify = configure once + build touched targets + run only affected ctests. The repo's `bash tools/run_tests.sh native` outer gate is deferred to the user, NOT run by this plan.

## Semantic Rulings (bind every task)

- **R1 — MJCF `damping` attribute on rider hinges: drop the `set("damping", ...)` write entirely** (MuJoCo default = 0). Matches spec Q13; every shipped config already resolves to 0.0. Cost if wrong: configs relying on the `nullopt → joint_kd_nms_rad` fallback silently change physics (verified: all 42 configs that set `joint_kd_nms_rad` also set the passive key explicitly; a config that sets neither still defaulted to 15 — this class changes 15→0, accepted per spec).
- **R2 — `rider_passive_damping` force component: KEEP.** It is `qfrc_passive` sampled at rider DOFs in `native/src/runtime/step.cpp` (~lines 1719, 1752, 1898, 2487) — passive-force truth, which can be nonzero from engine fluid/springs independent of the deleted knob. Removing it would alter `engine_passive`, `human_joint_power_w`, `loss_increment`, and `rider_work_ledger` for zero gain.
- **R3 — wire/recorded keys: write-side remove, read-side demote.** Stop emitting `passive_damping_nm`, `solved_passive_nm`, `rider_passive_power_w`, `rider_passive_work_step_j`. Keep the *strings* in decoder allow-lists as optional/accepted-but-ignored so recordings written before this change still decode. Where a key sits in a required-list (`kEffortRequiredKeys`), move it to the optional list, not delete. Where a decode assigns into a removed struct field, drop the assignment but keep the string allowed.
- **R4 — config key `joint_passive_damping_nms_rad`: hard-remove from schema** (FieldDecl table → TOML loader rejects it as unknown). Exception inside R3: if `state_codec`'s spindle-config decoder (`kConfigRequiredKeys`) also serves recording replay, move the key to the tolerated set and drop the assignment; the implementer must run the Verify-then-apply step for that site.

## File Structure

```
native/src/configuration/physical_config.hpp   field, accessor, FieldDecl  (T1)
native/src/configuration/validate.cpp           >=0 scalar check            (T1)
native/src/rider/spindle_controller.hpp         SpindleConfig field/accessor,
                                                JointTerms.passive_damping_nm,
                                                solved_passive_nm           (T1)
native/src/rider/state_codec.cpp                kConfigRequiredKeys, decode,
                                                emit sites, effort required→
                                                optional demotion           (T1)
native/src/model/articulated_rider.cpp          MJCF damping write          (T1)
native/src/rider/spindle_controller.cpp         passive[] terms x2 paths    (T1)
native/src/rider/legacy_controller.cpp          passive[] + effort fields   (T1)
native/src/rider/legacy_allocation.cpp          QP known-forces term        (T1)
native/src/runtime/accounting.cpp               passive_power_w emit, terms
                                                wire emit, solved split     (T1)
native/src/runtime/step.cpp                     Wire(rider_passive_power_w) (T1)
native/src/validation/scenarios.cpp             channel row key             (T1)
native/resources/recording/catalog-v1.json      2 catalog entries           (T1)
native/src/recording/catalog_data.inc           transcription twin          (T1)
60 TOML/JSON config fixtures                    key removal                 (T2)
migration/*.json manifests                      test-id/config sync         (T3)
native/tests/reference/*.cpp (6 files)          assertions                  (T3)
```

Verified anchor sites (grep `joint_passive_damping|passive_damping|passive_power_w` under `native/` to re-locate after edits):

- `physical_config.hpp:389-392` field+comment, `:478-481` `passive_damping_nms_rad()`, `:881-884` FieldDecl entry
- `validate.cpp:771-773`
- `spindle_controller.hpp:81` field, `:107-109` accessor, `:147` `JointTerms::passive_damping_nm` (also `solved_passive_nm` nearby)
- `state_codec.cpp:286` optional-keys list (KEEP string per R3), `:302` decode assign (drop), `:722-724` emit (drop), `:757` `kConfigRequiredKeys` (verify replay service per R4), `:850-851` assign (drop), `:491` `kEffortRequiredKeys` move `rider_passive_power_w`→`kEffortOptionalKeys`, `:534` make decode conditional, `:628-629` emit (drop)
- `articulated_rider.cpp:69` `value->set("damping", ...)` (drop line; the motor `add` stays)
- `spindle_controller.cpp:996-999` `passive[i]` term, `:1065-1073` `passive_power` ddot + assign, `:1087` terms assign, `:1522` solved-path twin
- `legacy_controller.cpp:391-393` term + `effort.passive_power_w`
- `legacy_allocation.cpp:65` `passive[dof] -=` term (the `envelope_forces` vector itself stays — different mechanism)
- `accounting.cpp:66` optional emit, `~:330` `solved_passive_nm = damping` + upstream `passive`/`damping` vectors, `:342-344` `d.passive_power_w`/`passive_work_step_j`, `:358-359` two channel sets
- `step.cpp:784-785` `Wire(d.passive_power_w)` emit
- `scenarios.cpp:65` `rider_passive_power_w` in row selection
- `catalog-v1.json:5492,8049-8056`; `catalog_data.inc:5538,8097-8104` — `rider_passive_power_w` (and `rider_passive_work_step_j` if cataloged; verify with `rg` in both files)

---

### Task 1: Native source removal (config + physics + controllers + telemetry writers + catalog)

**Files:**
- Modify: all `native/src/**` files listed above; `native/resources/recording/catalog-v1.json`; `native/src/recording/catalog_data.inc`
- Test: none yet — test `.cpp` files still reference the removed field; they are fixed in Task 3. Verify = compile of non-test targets only.

**Interfaces:**
- Produces: `ArticulatedConfig`/`SpindleConfig` without `joint_passive_damping_nms_rad` and without `passive_damping_nms_rad()`; `JointTerms` without `passive_damping_nm`/`solved_passive_nm`; `EffortDiagnostics` without `passive_power_w`; decoder allow-lists that still ACCEPT the old strings per R3; catalog without the removed channels.
- Consumes: rulings R1–R4 verbatim.

- [ ] **Step 1: Configure the build once**

```bash
cd <worktree>
grep -n 'name' CMakePresets.json | head -20   # pick the dev preset tools/run_tests.sh uses
cmake --preset <preset>
```

Expected: configure succeeds. The test binaries are NOT built yet.

- [ ] **Step 2: Config layer** — remove the field, accessor, FieldDecl row, validation block, and the `SpindleConfig` mirror. Then verify-then-apply for `state_codec.cpp`:
  - Run `rg -n "decode_spindle|kConfigRequiredKeys|parse_spindle" native/src native/tests | head -30` and check whether the spindle-config decoder is invoked on recorded/replayed state. If yes → keep `"joint_passive_damping_nms_rad"` in the allowed key set but delete the `config.joint_passive_damping_nms_rad = ...` assignment (accept-and-ignore). If no → remove the string from `kConfigRequiredKeys` too.

- [ ] **Step 3: Physics write** — delete the `value->set("damping",g(cfg.passive_damping_nms_rad()));` line in `add_rider_actuators` (`articulated_rider.cpp`). Keep the `actuators.add("motor", ...)` unchanged.

- [ ] **Step 4: Controller terms** — remove `passive` vector computation and every use in `spindle_controller.cpp` (both `finalize_effort` and the solved-effort path near line 1522), `legacy_controller.cpp`, and the `-=` term in `legacy_allocation.cpp`. Remove `passive_damping_nm`/`solved_passive_nm`/`passive_power_w` struct fields and their writers. In `accounting.cpp` remove the `solved_passive_nm = damping` split (keep `solved_force_nm`/`solved_active_nm` = active), the `passive` upstream vector if now unused, `d.passive_power_w`, `d.passive_work_step_j`, and the two `rider_passive_*` channel sets. In `step.cpp` remove the `rider_passive_power_w` wire emit. In `scenarios.cpp` drop the `||key=="rider_passive_power_w"` disjunct.

- [ ] **Step 5: Decoder demotion (R3)** — `state_codec.cpp`: keep `"passive_damping_nm"` and `"solved_passive_nm"` in the `decode_joint_terms_node` optional-keys list but drop the `out.passive_damping_nm = optional(...)` assignment; move `"rider_passive_power_w"` from `kEffortRequiredKeys` to `kEffortOptionalKeys` and wrap its decode in `if (d.contains(...))`; `"rider_passive_work_step_j"` already optional — drop its assignment; drop the `:628-629` emit and the `JointTerms`/`EffortDiagnostics` emit sites in `state_codec`/`accounting`/`step`.

- [ ] **Step 6: Catalog twins** — remove the `rider_passive_power_w` (and `rider_passive_work_step_j`, if present — check with `rg -n "rider_passive" native/resources/recording/catalog-v1.json`) field definition and section reference from `catalog-v1.json`; apply the identical edit inside the matching raw literal of `catalog_data.inc`. Keep JSON valid in both.

- [ ] **Step 7: Compile non-test targets**

```bash
cmake --build --preset <preset> --target bike   # or the lib target name; discover via cmake --build --preset <preset> --target help
```

Expected: library and tools build clean. Test executables may still fail to compile — expected at this stage, do NOT chase them.

- [ ] **Step 8: Commit**

```bash
git add native/src native/resources
git commit -m "refactor(rider): remove passive joint damping mechanism (spec Q13)"
```

---

### Task 2: Fixture sweep — remove `joint_passive_damping_nms_rad` from all config fixtures

**Files:**
- Modify: every TOML/JSON file under `tests/`, `examples/` containing the key — produce the list with `rg -l "joint_passive_damping" -g '*.toml' -g '*.json' tests/ examples/ native/resources/` (≈60 files; `migration/` is docs-as-artifacts — check whether `migration/cli-contracts.json`, `test-port-map.json`, `oracle-manifest.json` are read by `native/tests` before touching; they ARE consumed per `native/tests/support/migration_test.*`, so they are in scope).

**Interfaces:**
- Consumes: nothing from Task 1 besides "the key no longer exists".
- Produces: zero `rg "joint_passive_damping"` hits outside `docs/`, `migration/cards/*.md`, and report/task markdown (historical).

- [ ] **Step 1:** generate the file list; for `.toml` delete the whole `joint_passive_damping_nms_rad = ...` line; for `.json` delete the member line — if the deleted member was last in its object, remove the trailing comma on the previous line; if it was the only member, leave the (now-empty) object only if the schema allows empty objects, otherwise delete the parent member too. Spot-check 3 edited JSON files with `python3 -m json.tool <file>` — wait, no Python dependency: use `cmake -E` is not a JSON parser; use `uv run --with ... python -m json.tool` per AGENTS uv rule, or `plutil -lint` (macOS).

- [ ] **Step 2:** `rg -n "joint_passive_damping" tests/ examples/ native/resources/ migration/*.json` — expected: zero hits.

- [ ] **Step 3: Commit** `git add -A && git commit -m "test(fixtures): drop joint_passive_damping_nms_rad from config fixtures"`.

---

### Task 3: Test code + manifests update; build and run affected ctests

**Files:**
- Modify:
  - `native/tests/reference/test_coupled_rider_task.cpp` — delete `test_passive_damping_defaults_to_the_legacy_gain_coupling` (`:59-70`) and its registration (`:192`), and the `.5` field set at `:48` (verify it was only feeding the damping term)
  - `native/tests/reference/test_native_coupled_rider_task.cpp:72` — same field set
  - `native/tests/reference/test_welded_profile_config.cpp:101-102` — replace the `has_value() && ==0` assertion with an assertion on the resolved model: rider joints carry no `damping` attribute (or whatever the file's existing probe exposes — reuse its access pattern)
  - `native/tests/reference/test_native_spindle_controller.cpp` — fixture lexeme set at `:131` (drop), `test_spindle_activation_and_passive_damping_match_python` (`:562-585`, registration `:929`): the oracle-pinned expectations were computed WITH damping>0 — the damping portion is no longer reproducible; rename/scope the test to activation-only and update expectations, or drop it with a comment referencing spec Q13 (decide by whether remaining assertions still pin oracle arithmetic)
  - `native/tests/reference/test_native_rider_allocation.cpp:1140` — replace `config().passive_damping_nms_rad()` use with the now-correct expectation (no passive term in the QP known-forces)
  - `native/tests/reference/test_joint_strength.cpp:480,521-523` + `test_actuators_are_pure_motors_with_dof_damping` — repurpose to assert rider joints are pure motors AND carry no dof `damping` attribute; update `rider_passive_damping` component expectation only if the test asserts it equals kd·qvel (it must now assert the channel equals raw `qfrc_passive` at those DOFs, i.e. ~0 unless fluid passive exists — keep whatever tolerance the file already uses)
  - `migration/test-port-map.json` — remove/update the mapping for `test_actuators_are_pure_motors_with_dof_damping` if the test is renamed; sync any other manifest that keys on renamed tests (`rg` the test names under `migration/`)
- Test: `cmake --build --preset <preset>` then `ctest --test-dir <build> -R 'rider|spindle|joint_strength|welded|coupled|alloc' --output-on-failure` (discover exact test names via `ctest -N`)

**Interfaces:**
- Consumes: Task 1's removed API surface; Task 2's fixtures.
- Produces: compiling test suite; affected ctests green.

- [ ] **Step 1:** apply the edits above. Any pinned expected-value table that only changes because the damping term vanished must be re-derived from the remaining arithmetic — do not relax tolerances to hide drift.

- [ ] **Step 2:** `cmake --build --preset <preset>` — full test build green.

- [ ] **Step 3:** run the affected ctests (`ctest -R 'rider|spindle|joint_strength|welded|coupled|alloc'`). Expected: green. If a golden episode fixture fails because recorded bytes differ, the byte diff must be shown to consist only of the removed keys — anything else is a real regression to fix in Task 1's code, not here.

- [ ] **Step 4:** `ctest -N` → run the full `native/tests` ctest set in one bounded pass (these are the repo's native tests, minutes-scale, NOT the `run_tests.sh` sweep). Expected: green or an explicit, reproduced list of pre-existing reds to compare against `migration/honest-red-inventory-2026-10-31.md`.

- [ ] **Step 5: Commit** `git commit -m "test(rider): drop passive-damping coverage, assert pure motors without dof damping"`.

---

### Task 4: Final sweep + report

- [ ] `rg -n "joint_passive_damping|passive_damping_nms_rad|passive_damping_nm|solved_passive_nm|rider_passive_power_w|rider_passive_work_step_j|passive_power_w" --glob '!docs/**' --glob '!migration/cards/**' --glob '!*.md'` — expected survivors: only decoder allow-list strings (`"passive_damping_nm"`, `"solved_passive_nm"`, `"rider_passive_power_w"` in optional lists) each with a "legacy recordings" comment, and the kept `rider_passive_damping` channel.
- [ ] `git status` clean; report files touched, rulings applied, ctest results, and the deferred `run_tests.sh native` gate to the user.
