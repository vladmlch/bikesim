# Remaining Python→C++ Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish the Python→C++ migration: close the three open review findings, commit the in-flight T38–T51 integration, port the remaining test/tooling/packaging work packages T52–T60, qualify the release (T61–T62), and only then delete Python (T63).

**Architecture:** Four sequential phases. Phase A commits the verified but uncommitted wave integration, then fixes findings F2/F3/F5 in `native/src/runtime/step.cpp` with regression tests observed through the existing `NativeRideRuntime::probe_step_inputs` boundary. Phase B executes work packages T52–T60 whose per-file decomposition already lives in `migration/cards/T*.md`. Phase C runs qualification (T61, T62) and the gated deletion (T63). No Python file is deleted before T63.

**Tech Stack:** C++20, CMake/CTest (`native/build`), MuJoCo 3.12.0, DuckDB, TOML/JSON config stack, `tools/run_tests.sh` profiles (`quick|native|full`), `bt::*` migration test harness (`native/tests/support/migration_test.hpp`).

**Spec:** `docs/superpowers/plans/2026-10-14-python-free-cpp-master.md` plus subsystem plans 00–09 in the same directory; per-file work cards in `migration/cards/T*.md`; verified current state in `migration/migration-verified-state-2026-10-19.md`; open findings in `migration/native-waves-commit-review.md` (`## Re-verification 2026-10-19`).

## Global Constraints

- Bitwise parity contract: C++ must bitwise-equal the reference Python arithmetic; the Python oracle stays in the tree until T63.
- Verification entry point is `bash tools/run_tests.sh <quick|native|full>`. Iterate red→green on `ctest --test-dir native/build -R '^migration_TNN$' --output-on-failure`; run `bash tools/run_tests.sh native` once as the final gate before reporting done.
- `native` profile requires the standalone produced build: `cmake -S . -B build/native-release -DCMAKE_BUILD_TYPE=Release -DBIKE_BUILD_PYTHON_BINDINGS=OFF -DBIKE_BUILD_GUI=OFF -DBIKE_DEPENDENCY_ROOT=.deps` then `cmake --build build/native-release` (first time only; it is already configured here).
- The 24 pytest failures in `docs/TESTING.md:717-743` are a documented pre-existing baseline — identical set before and after any change; do not "fix" them inside migration tasks and do not let the set grow.
- Commit convention from git log: `feat(cpp): TNN <title>` / `fix(cpp): TNN <title>` / `docs(migration): TNN <note>`, merges labeled `merge: TNN ...`.
- Never commit secrets; do not push; do not touch `docs/superpowers/plans/*` (historical), oracle manifests' data payloads, or `.deps/`.
- Status vocabulary stays honest: `tests green` ≠ runtime-qualified; per-source oracle equivalence is a separate gate from a registered `migration_TNN` pass.
- Every task that touches `native/src/` must respect `docs/agents/native-engineering.md` (allocation discipline, checked numerics, writer contracts).

---

## Phase A — Stabilize and close open findings

### Task 1: Commit the in-flight T38–T51 integration

**Files:**
- Modify: none — this task commits the existing working-tree state.

**Interfaces:**
- Consumes: nothing
- Produces: a clean tree so Tasks 2–4 produce isolated, reviewable diffs.

- [ ] **Step 1: Inspect the diff scope**

```bash
git status --short | wc -l          # ~360 entries after the reporting cleanup
git diff --stat | tail -5
```

Expected: modifications confined to `native/`, `migration/`, `tests/`, `docs/TESTING.md`, plus untracked `migration/native-waves-commit-review.md`, `migration/migration-verified-state-2026-10-19.md`, `tests/fixtures/migration/initial_state/`.

- [ ] **Step 2: Re-run the current evidence gate**

```bash
cmake --build native/build -j8 && ctest --test-dir native/build -j8 --output-on-failure
```

Expected: 57/57 pass (verified 2026-10-19). If anything fails, fix before committing.

- [ ] **Step 3: Commit in two logical pieces**

```bash
git add native/ tests/fixtures/migration/initial_state/ tests/reference/
git commit -m "feat(cpp): T38-T51 recording and application wave integration"
git add migration/ docs/TESTING.md
git commit -m "docs(migration): record verified state, findings re-verification and card statuses"
```

- [ ] **Step 4: Confirm clean tree**

`git status --short` → empty.

### Task 2: F5 — pass `seated_climb.enabled` to the rider controller (smallest fix first)

**Files:**
- Modify: `native/src/runtime/step.cpp` (~line 1423-1432, inside `PhysicalStep::apply_forces_body`)
- Test: `native/tests/migration/test_t32.cpp` (legacy rider/control block owns `rider_control.py`'s recovery path; add the case there) or `test_t29.cpp` if the rider-dynamics owner fits better — the implementer picks the file whose card owns `sim/ride/rider_control.py` behavior.
- Oracle: `src/bike_sim/sim/ride/physical_runtime.py:362-367`

**Interfaces:**
- Consumes: `StepConfig::intent` (`native/src/runtime/step.hpp:58` — `rider::IntentConfig`, projected from the `seated_climb` document at `native/src/runtime/project_config.cpp:461`; `.enabled` ≡ `cfg.seated_climb.enabled`); `PhysicalStep::rider_control_` (`SpindleController::compute(..., bool pedal_recovery)`, `native/src/rider/spindle_controller.cpp:1138`).
- Produces: corrected `pedal_recovery` argument; F5 closes.

- [ ] **Step 1: Write the failing regression**

In the chosen test file, add a case that stages a non-spindle flat-pedal setup with `seated_climb.enabled == false` and asserts recovery never engages. Reuse the request/ride API proven in `test_t25.cpp:124-150`:

```cpp
[[gnu::noinline]] void pedal_recovery_respects_seated_climb_switch() {
    auto request = bike::initialization::read_setup_request(
        bt::fixture("requests/flat_pedal_flat.toml"));   // create in Step 3
    // seated_climb disabled in the fixture; advance past the control tick:
    auto ride = bike::app::make_ride(bike::app::prepare(std::move(request)));
    ride->advance(40, runtime::RideControl{}, 0., 0., std::nullopt);
    auto probe = ride->probe_step_inputs(runtime::RideControl{}, 0., 0.);
    // The legacy controller's pedal_recovery stage is serialized in the
    // diagnostics view (spindle_controller.cpp:1444 out.pedal_recovery):
    // assert every sole-side stage stays "none".
}
```

Create `tests/fixtures/migration/requests/flat_pedal_flat.toml` by copying `requests/welded_flat.toml` and a physics variant `tests/fixtures/migration/config/viewer_physics_flat_pedal.toml` copied from `viewer_physics_welded.toml` with spindle foot connects replaced by flat pedals and `seated_climb.enabled = false`. The exact config keys come from `tests/fixtures/migration/config/schema.json` (search `seated_climb`, `pedal`, `spindle`).

- [ ] **Step 2: Run it — expect failure**

```bash
cmake --build native/build --target migration_T32 && \
ctest --test-dir native/build -R '^migration_T32$' --output-on-failure
```

Expected: stage transitions observed (or, if the test can't reach the stage field cheaply, a marked divergence asserting current behavior — document the assertion chosen; do NOT weaken the check silently).

- [ ] **Step 3: Apply the fix**

In `native/src/runtime/step.cpp`, in the `control_tick` branch of `apply_forces_body`, change the last `compute(...)` argument:

```cpp
                torques = engaged(rider_control_).compute(
                    d, command, advance,
                    advance && !initializing_ ? control_clock_.period_s : dt,
                    !active, stepper_->rider_contacts().state().enabled,
                    config_.intent.enabled);   // F5: oracle passes cfg.seated_climb.enabled
```

(Any initialization/probe guard the runtime still needs belongs in the callee, not in this argument — match the oracle exactly.)

- [ ] **Step 4: Re-run** — same command as Step 2, expect PASS. Also `ctest --test-dir native/build -R 'migration_T(29|30|32|33)' -j4` for neighbors.

- [ ] **Step 5: Update records + commit**

Remove `F5` from `findings_open` in the affected rows of `migration/python-removal.json` (owners T29/T32), flip their `implementation.status` to `implemented_tested` where it was `implemented_tested_findings_open`, drop the F5 note in `migration/cards/T26.md` (F189 line), update `migration/native-waves-commit-review.md` re-verification section, then:

```bash
git commit -m "fix(cpp): F5 honor seated_climb.enabled for pedal recovery dispatch"
```

### Task 3: F2 — `ideal_speed_control` calls the cruise controller

**Files:**
- Modify: `native/src/runtime/step.cpp` (~line 1446-1463)
- Modify: `native/src/runtime/capabilities.cpp:20-21` — verify `ideal_speed_control` requires a `cruise` config; add `reject(...)` if not already enforced.
- Test: `native/tests/migration/test_t30.cpp` (missing physical modes) — or the T26 kernel file if reviewers prefer; test the behavior, not the file.
- Fixture: new `tests/fixtures/migration/requests/ideal_speed_flat.toml` + physics variant.
- Oracle: `src/bike_sim/sim/ride/physical_runtime.py:388-394`, `src/bike_sim/sim/ride/cruise.py:147-185`.

**Interfaces:**
- Consumes: `Stepper::cruise()` → `CruiseWriter&` (`native/src/stepper.hpp:348`); `CruiseWriter::compute(const mjData*, bool rear_in_contact, bool traction_limited, std::optional<bool> controller_grounded)` (`native/src/writers/cruise.hpp:21`); `RuntimeContacts::rear_in_contact()` / `.rear_controller_grounded` (`native/src/runtime/control.hpp:190-199`); `CruiseState` + `set_state` for the suppression path (same idiom as the `torque_nm = 0.` block at step.cpp:1447-1453).
- Produces: `ideal_speed_control` drives the bike; F2 closes.

- [ ] **Step 1: Write the failing regression**

```cpp
[[gnu::noinline]] void ideal_speed_control_commands_cruise_torque() {
    auto request = bike::initialization::read_setup_request(
        bt::fixture("requests/ideal_speed_flat.toml"));
    auto ride = bike::app::make_ride(bike::app::prepare(std::move(request)));
    ride->advance(20, runtime::RideControl{}, 0., 0., std::nullopt);  // settle contacts
    auto probe = ride->probe_step_inputs(runtime::RideControl{}, 0., 0.);
    bt::require(std::ranges::any_of(probe.ctrl, [](double v) { return v > 0.; }),
        "ideal_speed_control must command positive rear torque on flat ground");
}
```

(`probe_step_inputs` returns `PhysicalStep::ProbeResult` with `.ctrl` = the full `d->ctrl` row, `native/src/runtime/step.hpp:202-220`. Preferred check: resolve the `rear_drive` actuator index via `mj_name2id(model, mjOBJ_ACTUATOR, "rear_drive")` if the model is reachable from the test's include set — the `any_of` form is the fallback.) For the suppression branch add a second probe with `front=1., rear=0.` demands and assert the drive actuator stays `0.` — the oracle zeroes positive torque while braking.

- [ ] **Step 2: Fixture** — `requests/ideal_speed_flat.toml` copied from `welded_flat.toml` pointing at a new `config/viewer_physics_ideal_speed.toml` copied from `viewer_physics_welded.toml` with `drive_mode = "ideal_speed_control"` and the cruise target speed set (find the key in `tests/fixtures/migration/config/schema.json` — `cruise`/`target_speed_kmh`; oracle reads `sim.cruise` target from config).

- [ ] **Step 3: Run — expect failure** (`migration_T30`), the ctrl stays `0.` (current buggy branch writes `0.`).

- [ ] **Step 4: Apply the fix** at `native/src/runtime/step.cpp` (replace the `// OPEN FINDING F2` block):

```cpp
        if (active &&
            stepper_->drive().config().drive_mode == "ideal_speed_control") {
            // physical_runtime.py:389-394 — cruise compute gated on the
            // solved contacts, positive torque suppressed while braking.
            double torque = stepper_->cruise().compute(
                d, contacts.rear_in_contact(), /*traction_limited=*/false,
                contacts.rear_controller_grounded);
            if ((front > 0. || rear > 0.) && torque > 0.) {
                torque = 0.;
                CruiseState cs = stepper_->cruise().state();
                cs.torque_nm = 0.;
                stepper_->cruise().set_state(cs);
            }
            wbuf(d->ctrl, m->nu)[static_cast<std::size_t>(
                drive_ctrl_adr_)] = torque;
        }
```

Notes: `contacts` is the `RuntimeContacts` returned by `refresh_contacts` immediately above; `front`/`rear` are the `apply_forces_body` brake-demands parameters; `cruise()` (not `has_cruise()` guard) keeps an absent cruise config a loud failure — enforce that in `capabilities.cpp` by rejecting `ideal_speed_control` when `config.cruise` is absent (check whether `project_config.cpp`/`Stepper` already enforces; add `reject("drive_mode","ideal_speed_control requires a cruise configuration")` if not).

- [ ] **Step 5: Run** — PASS; also `ctest -R 'migration_T(26|30|31)'`.

- [ ] **Step 6: Update records + commit** — same bookkeeping as Task 2 Step 5 (drop F2 from T26/T30 rows, review doc, cards).

```bash
git commit -m "fix(cpp): F2 ideal_speed_control computes cruise torque like the oracle"
```

### Task 4: F3 — `native_reference` applies contact-snapshot resistance

**Files:**
- Modify: `native/src/runtime/step.cpp` (~line 1295-1314, the `else` path when `!stepper_->has_tire()`)
- Test: `native/tests/migration/test_t30.cpp` or `test_native_contacts.cpp` (owns the `native_reference` backend)
- Fixture: physics variant with `tires.backend = "native_reference"` — the native-contact query path already exists (`native/src/runtime/native_contacts.cpp`).
- Oracle: `src/bike_sim/sim/ride/physical_runtime.py:384-386`, `src/bike_sim/sim/ride/physical_resistance.py:20-42`.

**Interfaces:**
- Consumes: `refresh_contacts(...)` fills member `snapshots_[0..1]` with `TireSnapshot` (patches carry `normal_load_n`, `working_surface`, snapshot carries `effective_radius_m`) on the no-tire path (`step.cpp:1997-2006`); `stepper_->resistance().components_into(d, front_input, rear_input, span)` (`native/src/writers/resistance.hpp:61-63`) and the existing `patch_loads_`/`patch_working_` staging + `resistance_views_` used at `step.cpp:1315-1356`.
- Produces: `road_rolling`/`aerodynamic` components added to `accumulator_` on the `native_reference` route; F3 closes.

- [ ] **Step 1: Write the failing regression**

```cpp
[[gnu::noinline]] void native_reference_applies_resistance() {
    auto request = bike::initialization::read_setup_request(
        bt::fixture("requests/native_reference_flat.toml"));
    auto ride = bike::app::make_ride(bike::app::prepare(std::move(request)));
    ride->advance(20, runtime::RideControl{}, 0., 0., std::nullopt);
    auto probe = ride->probe_step_inputs(runtime::RideControl{}, 0., 0.);
    const auto names = std::views::transform(probe.components,
                                             &std::pair<const std::string,
                                                        std::vector<double>>::first);
    bt::require(std::ranges::find(names, "road_rolling") != names.end() &&
                std::ranges::find(names, "aerodynamic") != names.end(),
        "native_reference must emit resistance components like the oracle");
}
```

`probe.components` is the ordered named-component fold (`step.hpp:203-205`); on the buggy code the two resistance names never appear for `native_reference`.

- [ ] **Step 2: Fixture** — `requests/native_reference_flat.toml` + `config/viewer_physics_native_reference.toml` with `[tires] backend = "native_reference"` and a nonzero rolling-resistance config (confirm the resistance config keys in `schema.json`/`config-contracts.json`; `resistance` section of the physics config).

- [ ] **Step 3: Run — expect failure** (no resistance components in `probe.components`).

- [ ] **Step 4: Apply the fix** — in the `else` branch after `refresh_contacts` fills `snapshots_`, mirror the existing staging at step.cpp:1315-1356 but source patches from the member `snapshots_[i]` (native_reference `TireSnapshot`s), `effective_radius_m` taken unconditionally:

```cpp
        if (active && !stepper_->has_tire()) {
            // physical_runtime.py:384-386 — tire-less route evaluates
            // resistance from the solved contact snapshots.
            std::array<TireSideInput, 2> side_input{};
            for (std::size_t i = 0; i < 2; ++i) {
                const TireSnapshot &snap = engaged(snapshots_[i]);
                const std::size_t n = snap.patches.size();
                patch_loads_[i].assign(n, 0.);
                if (n > patch_working_capacity_[i]) {
                    patch_working_[i] = std::make_unique<bool[]>(n);
                    patch_working_capacity_[i] = n;
                }
                const std::span<bool> working{
                    std::views::counted(patch_working_[i].get(),
                                        static_cast<std::ptrdiff_t>(n))};
                for (std::size_t p = 0; p < n; ++p) {
                    patch_loads_[i][p] = snap.patches[p].normal_load_n;
                    working[p] = snap.patches[p].working_surface;
                }
                side_input[i] = {.patch_loads = std::span<const double>(
                                     patch_loads_[i].data(), n),
                                 .patch_working = working,
                                 .effective_radius_m = snap.effective_radius_m};
            }
            const auto res_names = stepper_->resistance().component_names();
            stepper_->resistance().components_into(
                d, side_input[0], side_input[1],
                std::span{resistance_views_}.first(res_names.size()));
            for (std::size_t i = 0; i < res_names.size(); ++i)
                accumulator_.add(res_names[i], resistance_views_[i].values);
        }
```

Placement: immediately after `refresh_contacts` returns inside `apply_forces_body`, before the cruise block — matching the oracle order (`_contacts` → resistance → cruise). Factor the duplicated staging loop into a small private helper if the reviewer prefers; keep the arithmetic identical.

- [ ] **Step 5: Run** — PASS; also `migration_T30_native_contacts` and neighbors.

- [ ] **Step 6: Update records + commit**

```bash
git commit -m "fix(cpp): F3 apply contact-snapshot resistance on the native_reference route"
```

### Task 5: Close the findings ledger and re-verify the wave

**Files:**
- Modify: `migration/native-waves-commit-review.md` (re-verification section → all findings resolved)
- Modify: `migration/python-removal.json`, `migration/cards/T26.md`, `migration/physics-research-implementation.json` (drop remaining `findings_open` markers)
- Test: full migration gate

- [ ] **Step 1: Full ctest**

```bash
ctest --test-dir native/build -j8 --output-on-failure   # expect 57/57
```

- [ ] **Step 2: Full native gate once**

```bash
bash tools/run_tests.sh native
```

Expected: exit 0; 24-failure pytest baseline unchanged. This is the AGENTS.md completion criterion for native-scope changes.

- [ ] **Step 3: Update the tracking documents**

Flip the T26/T29/T30/T32 `implementation.status` values in `python-removal.json` and `physics-research-implementation.json` from `implemented_tested_findings_open` → `implemented_tested`; update `migration/native-waves-commit-review.md` (`## Re-verification 2026-10-19`: all five findings closed with commit hashes); refresh `migration/migration-verified-state-2026-10-19.md` status table.

- [ ] **Step 4: Commit**

```bash
git commit -m "docs(migration): close wave findings after verification"
```

---

## Phase B — Remaining work packages T52–T60

Each package's per-file decomposition is already written: the owner plan section (`docs/superpowers/plans/2026-10-14-python-free-cpp-08-tests-tools-packaging.md`) + `migration/cards/TNN.md` (one red→green card per file) + `migration/python-removal.json` rows. Work them file-by-file: **write the failing native regression → port → green → update the removal row + card annotation → commit**. Dependency order below follows `migration/task-graph.json::dependency_waves`.

### Task 6: T52 — port unit/numerical assertions out of pytest

- Scope: 1009 test-port records (`migration/test-port-map.json`, `owner_task: T52`), 88 Python files (cards `T52.md`), including `tests/reference/test_*.py` unit/property suites.
- Deliverables: `native/tests/reference/*.cpp` (directory does not exist yet — create), CTest registrations, per-file ledger updates.
- Verify per batch: `ctest --test-dir native/build -R 'native_reference_<name>' --output-on-failure`; whole package: `bash tools/run_tests.sh native`.
- Commit cadence: one commit per card section (`test(cpp): T52 port <file>`).

### Task 7: T53 — port integration/frontend/replay/failure-path suites

- Scope: 602 records, 64 Python files (cards `T53.md`).
- Same per-file loop; replay tests reuse the `recording_fixture.hpp`/`recording_reader_test_support.hpp` helpers already in `native/tests/migration/`.
- Gate: `bash tools/run_tests.sh native` green at package end.

### Task 8: T54 — validation rigs, calibration, physical release evidence

- Scope: 30 Python files (`migration/cards/T54.md`), e.g. `src/bike_sim/physics/tire_calibration.py`, `src/bike_sim/validation/*.py`.
- Same loop; physical rigs reuse `app::prepare`/`make_ride` and `tests/fixtures/migration/requests/*`.

### Task 9: T55 — performance, diagnostic and benchmark developer tools

- Scope: 11 Python files (`cards/T55.md`): `sim/realtime_measurement.py`, `tools/diag_run.py`, `tools/measure_realtime.py`, `tools/step_profile.py`, etc.
- Native candidates exist (`native_bench`, `native/src/diag.hpp`, `bike-native-checks`); per-file cards decide reuse-vs-port. Keep Python tools until T63.

### Task 10: T56 — finish replacing Python check orchestration

- Current: `bike-native-checks` already runs headers/diagnostic-controls/context/contracts kinds; remaining Python/bash pieces: `tools/native_checks.py`, `tools/check_native_contracts.py`, `tools/check_*.sh` drivers inside `tools/run_tests.sh`, inline `native_uv python -c ...` snippets (extension path resolution, build-dir selection).
- Deliverables: `migration_T56` ctest script test registered (see plan-08 §T56 contract), `bike-native-checks --kind` covering the remaining sweep kinds or a native driver for build-dir/extension resolution.
- Verify: `bash tools/run_tests.sh native` still exits 0 with zero Python invoked during the sweep stage; `cmake --build build/native-release --target check_<name>` parity for reruns.

### Task 11: T57 — native test runner, sanitizers, coverage provenance

- Scope (card T57): `native/src/tools/test_runner_main.cpp`, `native/src/tools/coverage_main.cpp`, `native/cmake/Sanitizers.cmake`, `native/cmake/Coverage.cmake`, `native/tests/support/provenance.cpp`; replaces `conftest.py`, `tools/native_coverage.py`, pytest orchestration inside `run_tests.sh`.
- Verify: runner executes the ctest-equivalent suite standalone; sanitizer builds (`NATIVE_SANITIZE`, RTSan) still honored by `resolve_native_runtime`; coverage provenance lands in `native_test_provenance.json`.

### Task 12: T58 — Linux headless qualification

- Scope: `native/cmake/PlatformLinux.cmake`, `native/cmake/PlatformApple.cmake`, `native/src/numeric/provider_linux.cpp`, `packaging/platform-matrix.json`, `native/tests/platform/` (card T58).
- Constraint: no silent change to macOS arithmetic — platform matrix records per-platform numeric provider; identical golden hashes required where the platform pair promises them.

### Task 13: T59 — offline native distributions

- Scope: replace `tools/install_offline.py` + wheel flow (card T59): native packaging producing the 9 CLI binaries + assets + policy ABI headers; `bike-python-audit` package mode becomes mandatory here.
- Verify: install into a clean prefix and run each `bike-*` entrypoint's `--help` and a canned ride/settle/replay job from the package only.

### Task 14: T60 — documentation and agent contracts

- Scope (card T60): `README.md`, `AGENTS.md`, `docs/native-workflow.md` (create), `docs/python-to-cpp-compatibility.md` (create), `migration/cli-contracts.json`, `migration/command-map.json`.
- Known stale items to fix here: `AGENTS.md`'s `cmake --build native/build --target check_<name>` advice (headers/diagnostic-controls/context/contracts run via `bike-native-checks --kind`); README still describes Python install as the entry point.

---

## Phase C — Qualification and removal

### Task 15: T61 — end-to-end release qualification

- Deps: T53, T54, T55, T58, T59, T60. Registers `migration_T61` (CTest script test per plan-09 §T61 contract).
- Runs the capability parity matrix (`migration/capability-seed-matrix.json`), command contract (`command-map.json`), recording/replay round-trips and the release evidence suite; closes the remaining `acceptance_gaps` listed in the three wave manifests.
- Gate: `ctest --test-dir build/native-release -R '^migration_T61$' --output-on-failure` + `bash tools/run_tests.sh full`.

### Task 16: T62 — no-Python enforcement

- Deps: T57, T59, T61. `bike-python-audit` source mode becomes mandatory: binary link audit (no libpython — today already true for `bike-sim`; extend to all shipped binaries), package audit, build-graph audit (`BIKE_BUILD_PYTHON_BINDINGS=OFF` default in the standalone graph).
- Verify per plan-09 §T62 contract: source-mode audit is a measured failure pre-T63, passes post-T63.

### Task 17: T63 — delete Python

- Deps: T61, T62. The 459-row `migration/python-removal.json` ledger (plus any first-party files discovered since) is the reviewed removal set: delete `src/bike_sim/`, `tests/`, `tools/*.py`, `conftest.py`, `pyproject.toml` Python sections, nanobind binding sources (`native/src/binding*.cpp/.hpp`, `engine_call.cpp`, `engine_abi_312.hpp`), `.venv`-dependent tooling.
- Verify: clean-tree build from a fresh checkout; `bash tools/run_tests.sh` (native-only by then) green; `bike-python-audit` source mode passes; commit `feat(cpp): T63 delete Python implementations, adapters and packaging`.

---

## Self-review notes

- Coverage: F2/F3/F5 → Tasks 2–4; uncommitted wave → Task 1; bookkeeping → Task 5; T52–T63 → Tasks 6–17 in dependency-wave order (T56 placed inside Phase B per its in_progress state).
- Honesty guardrails: no task may mark a removal row `ready_to_delete` before T61/T62; `implemented_tested` means migration tests green only.
- Risk order: F3 is the subtle one (probe-vs-committed snapshot publication on the no-tire path — verify `snapshots_` is populated for probe calls too before relying on it in the test).
