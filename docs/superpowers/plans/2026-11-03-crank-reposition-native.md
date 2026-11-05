# Crank Reposition (Native) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring back the crank reposition manoeuvre in native C++ on main3. When the rider is stuck in a dead spot on a hard section, the rider briefly backpedals to the nearest horizontal crank position, so the front leg has the best lever, and then presses again. The manoeuvre also runs on demand from the viewer key V.

**Architecture:** The manoeuvre lives in `drivetrain::PedalingPolicy`, as it did in the Python original (`c809f12a^:src/bike_sim/physics/pedaling.py`). A progress-stall detector arms it, and a one-shot request from the viewer triggers it. While it runs, the policy reports `coasting` with a new reason and a backward crank-rate target. The spindle rider controller gets its own reposition branch, because the existing coasting branch servos the pedal press to zero and would cancel any backward torque. The feature is off by default. Every new state or command key is serialized only when engaged, so with the default config every existing golden stays byte-identical. kinetic-damping (`aece48d7`) used the same pattern.

**Tech Stack:** C++23, MuJoCo 3.12, CMake/CTest (`bike_add_reference_test`, `bt::` harness in `native/tests/support/fixture_test.hpp`), `bash tools/run_tests.sh`.

**Spec:** There is no separate spec document. The requirements are the **Requirements** section below. The behavioural reference is the removed Python implementation: `git show c809f12a^:src/bike_sim/physics/pedaling.py` (classes `ProgressStallDetector`, `PedalingPolicy`), `c809f12a^:src/bike_sim/physics/physical_config.py` (`PedalingConfig.reposition_*`), and its tests `c809f12a^:tests/test_pedaling_stall.py`. The removal decision is recorded in main2 at `docs/superpowers/plans/2026-10-04-realistic-pedelec-drive.md` (Q10). This plan reverses that decision at the user's request (2026-11-03).

## Ревизия 2026-11-05 (читать первой; при расхождении с текстом ниже действует ревизия)

Почему пересмотрен план. 2026-11-05 пользователь решил, что педали назад крутятся через муфту в моторе. Муфт
две: втулка и муфта в моторе. Педаль вперёд — зацепление, назад — свободно
(`docs/agents/simulation-decisions.md`, «Решено — не сделано» 1 и 3). План ниже написан 2026-11-03 под жёсткую связь
шатуна и звезды.

Что изменено:

- **Предусловие.** Сначала выполнены `docs/superpowers/plans/2026-11-05-lossless-drivetrain.md` и
  `docs/superpowers/plans/2026-11-05-motor-clutch.md`. Во всех трёх `examples/research/viewer_physics_*.toml` стоит
  `motor_clutch = true`. Проверка: `grep -n '^motor_clutch' examples/research/viewer_physics_*.toml` выводит три
  строки `= true`. Иначе не начинай.
- **R5 теперь выполняется по устройству.** Муфта в моторе свободна назад
  (`native/src/model/physical_topology.cpp`, ограничение `crank_clutch`, диапазон `-1e12 0`). Поэтому назад
  крутится только шатун, а вал мотора и звезда продолжают ехать вперёд. В R5 «The rear hub and the motor drive
  through one-way freewheels» читай как «the motor clutch is free backward».
- **S3 и задача 7, шаг 1** — к проверке колеса добавляется проверка вала мотора: во время манёвра
  `drive_shaft_rad_s` не становится отрицательной, а `crank_clutch_engaged` ложно. Если вал крутится назад вместе с
  шатуном, это S3.
- **Задача 0.** Рабочего дерева `.worktrees/main3` больше нет. Работа идёт в рабочем дереве, где выполнены два
  плана-предусловия. Базой служит последняя зелёная строка `<git-common-dir>/bike-gate/timings.tsv` на этом коде, а
  не «181/183». Список известных падений (`reference.native_research_recording`, `reference.native_numeric_portability`)
  проверь заново: у AGENTS.md нет списка терпимых падений.
- **Задача 2 уже сделана** (`352f51e0`): девять ключей `reposition_*` объявлены, проверяются, проецируются и
  выбрасываются из записей, если равны значениям по умолчанию. Выполни только проверку: `rg -n 'reposition_on_stall'
  native/src/configuration native/src/writer_config.hpp native/src/replay native/src/runtime/project_config.cpp`
  находит все места. Код поведения в `native/src/drivetrain/pedaling.*` не читает их, это начинается с задачи 3.
- **«Нужный» каденс.** Нижняя граница наката и желание педалирования (`native/src/drivetrain/pedaling.cpp:42-43,
  79-81`) считаются от скорости вала мотора через втулку. Во время манёвра шатун крутится назад, а вал вперёд.
  Задача 4 должна держать манёвр вне этих расчётов, иначе накат «догоняет» шатун до скорости вала. Добавь в тест
  задачи 4 проверку: во время манёвра цель скорости шатуна отрицательна независимо от `required_cadence_rpm`.
- **Задача 9.** Набор трасс: `rough_uphill_savage.toml` (трасса пользователя), `rough_uphill_extreme.toml`,
  `alpine_rocky.toml`. Итоговое доказательство — `bash tools/run_tests.sh full` (AGENTS.md, уровень 3). Отдельный
  `native` перед ним не нужен: `full` его включает.
- **D3 без изменений.** Включать ли `reposition_on_stall = true` в профилях, решает пользователь после задачи 9.
- **Глобальное ограничение о неизменных эталонных записях** остаётся: всё новое выключено по умолчанию. Эталонные
  записи после планов-предусловий уже перезаписаны, сравнение идёт с ними.

## Requirements

- **R1 Stall reflex.** With `drive.pedaling.reposition_on_stall = true`, a stall starts the manoeuvre. A stall means the rider commands at least `reposition_min_effort_nm` and, for `reposition_stall_window_s`, neither the crank nor the wheel (in crank-equivalent radians) advances more than `reposition_stall_progress_rad`. Rocking about a dead spot and rolling back both count as no progress.
- **R2 Manual request.** Key **V** in the interactive *physical* viewer requests one manoeuvre. The same machinery runs as for R1. Holding the key does not retrigger.
- **R3 Goal.** The power positions are the two crank-horizontal poses. The manoeuvre backpedals to the nearest power phase *below* the current phase, which is at most π rad back. It does nothing when the crank is within `reposition_noop_rad` of a power phase.
- **R4 Bounded.** The manoeuvre ends when the crank is within `reposition_phase_tolerance_rad` of the goal or after `reposition_timeout_s`. Braking aborts it. A `reposition_cooldown_s` follows each end or abort. Rider effort is zero during the manoeuvre and slews back from zero afterwards.
- **R5 Realism (user, 2026-11-03).** Backpedalling needs only small forces. The rear hub and the motor drive through one-way freewheels, so the crank turns backward freely. During the manoeuvre the requested crank torque stays inside `coasting_brake_limit_nm`. The pedal welds must not pull: weld normal ≥ −`R5_TENSION_TOLERANCE_N` on both pedals. The backward crank motion must not drive the wheel backward.
- **R6 Default off.** The defaults are `reposition_on_stall = false` and no request. Every existing CTest node, including all goldens under `tests/fixtures/golden/**`, passes without being edited.

## Global Constraints

- Captured goldens under `tests/fixtures/golden/**` are never edited (ADR-0003, `docs/research/README-provenance.md`). Hand-maintained config fixtures under `tests/fixtures/config/**` may be updated, as `aece48d7` did.
- Default output is byte-identical: the reposition state and command keys appear on the wire only when engaged.
- No new control-wire field, policy-ABI field (`native/include/bike/policy_v1.h`) or rider-program keyframe. `native/tests/reference/test_pedaling_policy.cpp` keeps asserting that `crank_reposition` is absent from the control wire and from rider programs.
- The `rider_mode` label domain stays `{disabled, pedaling, coasting}`. It is a cataloged label set (`writers/writer_types.hpp:162`). The manoeuvre is reported as `coasting` with the coast reasons `reposition_requested` and `reposition_stall`. Coast reasons are free text in telemetry (`writers/drivetrain.cpp:1053`).
- Realtime: no allocation in `PedalingPolicy::update`, so the new state is POD.
- Only the spindle-weld rider is supported (`articulated.pedal_attachment = "spindle"`). Any other attachment with `reposition_on_stall = true` is a config error.
- Iterate with targeted CTest nodes. The final gate is `bash tools/run_tests.sh native` **and** `full`, because this is a physics path (`docs/agents/native-engineering.md`). Compare failures against the known baseline: `reference.native_research_recording`, `reference.native_numeric_portability` (fast group); `tooling_checks`, `reference.golden_episode_tool`, `reference.native_pneumatic_tire` (slow).
- Never run two gates or plant runs in parallel in one worktree.

## Decisions taken by default (confirm before Task 1)

- **D1** The manual key works only in the *physical* viewer. Research runs use the reflex only, because a research manual request is not part of the recorded command stream and would break checked replay.
- **D2** The defaults match the Python original: min effort 20 N·m, window 1 s, progress 0.5 rad, cooldown 1 s, back rate 3 rad/s, timeout 3 s, tolerance 0.05 rad, no-op band 0.12 rad.
- **D3** The examples are not switched on in this plan. Whether `viewer_physics_*.toml` sets `reposition_on_stall = true` is decided after the acceptance numbers in Task 9.
- **D4** `R5_TENSION_TOLERANCE_N = 20` N (solver noise allowance). The general returning-foot tension in normal pedaling is a separate issue and is not fixed here.

## Stop conditions

- **S1** After Task 2, a node under `tests/fixtures/golden/**` fails only because of the new config keys. Stop and report. Do not edit the golden.
- **S2** Task 1 shows that crank `qpos = 0` is not a horizontal crank. Set `kPowerPhaseOffsetRad` to the measured value, record it in the commit, and continue. If the pedals turn out not to be 180° apart, stop.
- **S3** Task 7 shows that a backward crank rate drives the wheel backward, so the topology is not free to backpedal. Stop and report.
- **S4** Task 8 cannot reach the goal within the timeout with `coasting_brake_limit_nm`, or it violates R5. Stop and report the measured torque and weld loads. Do not raise the limit silently.

## File map

| File | Responsibility |
|---|---|
| `native/src/configuration/physical_config.hpp` | 9 `reposition_*` fields in `PedalingConfig` and their `FieldDecl` rows |
| `native/src/configuration/validate.cpp` | field domains (`post_init(const PedalingConfig&)`), spindle-only cross-check |
| `native/src/replay/rebuild.cpp` | recipes recorded without the keys reconstruct with default values |
| `tests/fixtures/config/schema.json`, `tests/fixtures/config/resolved/*.json`, `tests/fixtures/config/variant_*.resolved.json` | config fixtures (same set `aece48d7` touched) |
| `native/src/drivetrain/stall_detector.hpp` (new) | header-only `ProgressStallDetector` |
| `native/src/config_types.hpp` | `RepositionSnapshot`, `PedalingSnapshot::reposition` (optional) |
| `native/src/drivetrain/pedaling.hpp/.cpp` | coast reasons, manoeuvre state machine, request input |
| `native/src/drivetrain/state_codec.cpp` | `_reposition` key emitted only when engaged |
| `native/src/writers/drivetrain.hpp/.cpp` | `RideControl::crank_reposition`, passes the request into `update` |
| `native/src/rider/spindle_controller.hpp/.cpp`, `native/src/rider/state_codec.cpp` | `RiderCommand::crank_reposition`, reposition branch, codec key only when true |
| `native/src/runtime/step.hpp/.cpp`, `native/src/runtime/runtime.hpp/.cpp` | one-shot request API, command flag from the pedaling reason |
| `native/src/viewer/input.hpp/.cpp`, `native/src/viewer/session.hpp`, `native/src/viewer/ride.cpp`, `native/src/cli/ride.cpp` | key V → hook → runtime request, help text |
| `native/tests/reference/test_pedaling_policy.cpp` | knob tests inverted, policy tests |
| `native/tests/reference/test_crank_reposition.cpp` (new) | plant-level manoeuvre test (R3–R5) |
| `docs/RIDE.md` | V key and the config knobs |

---

### Task 0: Preflight

**Files:** none.

- [ ] **Step 1: Record the starting point**

Run: `git -C .worktrees/main3 log --oneline -1 && git -C .worktrees/main3 status --short | grep -v '^??'`
Expected: clean tree. Write the HEAD hash into the task report.

- [ ] **Step 2: Baseline fast group**

Run: `cmake --build build/native-release && NATIVE_TEST_BUILD_PATH=$PWD/build/native-release ctest --test-dir build/native-release -LE 'slow|package'`
Expected: 181/183. The only failures are `reference.native_research_recording` and `reference.native_numeric_portability`.

### Task 1: Pin the crank phase convention

The Python policy assumed that the power phases are the multiples of π of the crank joint angle. The drivetrain passes the raw `crank_spin` qpos to `PedalingPolicy::update` (`writers/drivetrain.cpp:818`). The rider controller computes `phase = atan2(-z, x)` of the front spindle relative to the crank (`spindle_controller.cpp`, the `const double phase =` line). This task proves the mapping before any code depends on it.

**Files:**
- Modify: `native/tests/model/test_mjcf.cpp` (add one function and call it from the test's run body)

**Interfaces:**
- Produces: the constant `drivetrain::kPowerPhaseOffsetRad` (Task 4), whose value this test establishes.

- [ ] **Step 1: Write the test**

Add next to `physical_profiles_and_topology()`:

```cpp
// Crank qpos 0 is a power position: the two spindles sit level (same
// height) and 180 degrees apart. PedalingPolicy's reposition goal
// (multiples of pi from kPowerPhaseOffsetRad) depends on it.
void crank_zero_is_horizontal() {
    const auto request = physical("viewer_physics_welded.toml");
    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(mj_makeData(model.get()), mj_deleteData);
    const int crank = mj_name2id(model.get(), mjOBJ_JOINT, "crank_spin");
    const int front = mj_name2id(model.get(), mjOBJ_BODY, "pedal_front");
    const int rear = mj_name2id(model.get(), mjOBJ_BODY, "pedal_rear");
    bt::require(crank >= 0 && front >= 0 && rear >= 0, "crank and both pedal bodies exist");
    for (const double angle : {0., std::numbers::pi}) {
        mj_resetData(model.get(), data.get());
        data->qpos[model->jnt_qposadr[crank]] = angle;
        mj_kinematics(model.get(), data.get());
        const double dz = data->xpos[3 * front + 2] - data->xpos[3 * rear + 2];
        const double dx = data->xpos[3 * front + 0] - data->xpos[3 * rear + 0];
        bt::require(std::abs(dz) < 1e-6, "spindles level at a power phase (dz=" + std::to_string(dz) + ")");
        bt::require(std::abs(dx) > .2, "spindles a crank diameter apart along x");
    }
}
```

If `pedal_front`/`pedal_rear` are not body names in the generated model (look with `grep -n '"pedal_' native/src/model/*.cpp`), use the names that `spindle_controller.cpp:454` resolves (`"pedal_" + stem`).

- [ ] **Step 2: Run it**

Run: `cmake --build build/native-release --target model_mjcf && ctest --test-dir build/native-release -R '^model_mjcf$' --output-on-failure`
Expected: PASS → `kPowerPhaseOffsetRad = 0`. FAIL with `dz` ≠ 0 → S2: find the qpos where `dz = 0` (bisect over `[0, π)` in the test) and use that offset.

- [ ] **Step 3: Commit**

```bash
git add native/tests/model/test_mjcf.cpp
git commit -m "test(model): pin crank qpos 0 as a horizontal power position"
```

### Task 2: Reposition config fields

**Files:**
- Modify: `native/src/configuration/physical_config.hpp` (struct at `:231`, `kPedalingConfigFields` at `:718`)
- Modify: `native/src/configuration/validate.cpp` (`post_init(const PedalingConfig&)` at `:562`; the cross-check next to the `equilibrium_relaxation` check, about `:1079`)
- Modify: `native/src/replay/rebuild.cpp` (next to the `equilibrium_relaxation` drop that `aece48d7` added)
- Modify: `tests/fixtures/config/schema.json`, `tests/fixtures/config/resolved/{welded,distributed,pneumatic}.json`, `tests/fixtures/config/variant_{alt_support,elastic_chain,grip_weld_coercion}.resolved.json`
- Test: `native/tests/reference/test_pedaling_policy.cpp`, `native/tests/config/test_schema.cpp`

**Interfaces:**
- Produces: `PedalingConfig::{reposition_on_stall, reposition_min_effort_nm, reposition_stall_window_s, reposition_stall_progress_rad, reposition_cooldown_s, reposition_back_rate_rad_s, reposition_timeout_s, reposition_phase_tolerance_rad, reposition_noop_rad}`.

- [ ] **Step 1: Invert the knob test (red)**

In `test_pedaling_policy.cpp`, replace `test_reposition_knobs_and_mode_are_gone` with:

```cpp
// The reposition knobs are back (2026-11-03); the mode domain is not:
// the manoeuvre reports as coasting with a reposition reason.
void test_reposition_knobs_exist_and_mode_domain_is_unchanged() {
    const auto doc = bike::io::parse_json(R"({"drive":{"pedaling":{"reposition_on_stall":true}}})");
    bool found = false;
    for (const auto& field : bike::configuration::kPedalingConfigFields)
        found = found || field.name == std::string_view("reposition_on_stall");
    bt::require(found, "pedaling schema declares reposition_on_stall");
    bt::require(!drivetrain::pedal_mode("reposition"), "reposition is not a pedaling mode");
    (void)doc;
}
```

Keep the rest of the old function's body that checks the removed `update(reposition=...)` keyword only if it still compiles. Leave `test_ride_control_has_no_reposition_field` and `test_rider_program_rejects_the_removed_reposition_command` unchanged (Global Constraints). Update the call in the run body.

Run: `cmake --build build/native-release --target reference.pedaling_policy && ctest --test-dir build/native-release -R '^reference.pedaling_policy$' --output-on-failure`
Expected: FAIL "pedaling schema declares reposition_on_stall".

- [ ] **Step 2: Add the fields**

In `PedalingConfig` after `rollback_demand`:

```cpp
    // Crank reposition (stall reflex + operator request): backpedal to the
    // nearest lower horizontal crank pose so the leading leg gets its lever.
    // Spindle-weld rider only. Python original: c809f12a^ pedaling.py.
    bool reposition_on_stall = false;
    double reposition_min_effort_nm = 20.;
    double reposition_stall_window_s = 1.;
    double reposition_stall_progress_rad = .5;
    double reposition_cooldown_s = 1.;
    double reposition_back_rate_rad_s = 3.;
    double reposition_timeout_s = 3.;
    double reposition_phase_tolerance_rad = .05;
    double reposition_noop_rad = .12;
```

In `kPedalingConfigFields` after `REAL(PedalingConfig, rollback_demand),`:

```cpp
    BOOL(PedalingConfig, reposition_on_stall),
    REAL(PedalingConfig, reposition_min_effort_nm),
    REAL(PedalingConfig, reposition_stall_window_s),
    REAL(PedalingConfig, reposition_stall_progress_rad),
    REAL(PedalingConfig, reposition_cooldown_s),
    REAL(PedalingConfig, reposition_back_rate_rad_s),
    REAL(PedalingConfig, reposition_timeout_s),
    REAL(PedalingConfig, reposition_phase_tolerance_rad),
    REAL(PedalingConfig, reposition_noop_rad),
```

- [ ] **Step 3: Validate the domains**

At the end of `post_init(const PedalingConfig& config)`:

```cpp
    scalar(config.reposition_min_effort_nm, "reposition minimum effort", 0.);
    scalar(config.reposition_cooldown_s, "reposition cooldown", 0.);
    scalar(config.reposition_noop_rad, "reposition no-op band", 0.);
    scalar(config.reposition_back_rate_rad_s, "reposition back rate", std::nullopt, true);
    scalar(config.reposition_timeout_s, "reposition timeout", std::nullopt, true);
    scalar(config.reposition_phase_tolerance_rad, "reposition phase tolerance", std::nullopt, true);
    scalar(config.reposition_stall_window_s, "reposition stall window", std::nullopt, true);
    scalar(config.reposition_stall_progress_rad, "reposition stall progress", std::nullopt, true);
    if (config.reposition_noop_rad >= .5 * std::numbers::pi)
        fail("reposition no-op band must be below a quarter crank turn");
    if (config.reposition_stall_progress_rad >= std::numbers::pi)
        fail("reposition stall progress must be below half a crank turn");
```

Check the `scalar(value, name, minimum, positive)` signature at the top of `validate.cpp` and match the existing calls in that function. Add `#include <numbers>` if it is missing.

Next to the `equilibrium_relaxation` check:

```cpp
    if (config.drive.pedaling.reposition_on_stall && config.articulated.pedal_attachment != "spindle")
        fail("drive.pedaling.reposition_on_stall requires articulated.pedal_attachment = 'spindle'");
```

- [ ] **Step 4: Replay compatibility**

In `rebuild.cpp`, after the `equilibrium_relaxation` block, add the same pattern for the pedaling section. Recipes recorded before the knobs carry none of them, and their run used the defaults, so a default reconstruction drops them:

```cpp
    // Recipes recorded before the reposition knobs (2026-11-03) carry none of
    // them; such runs used the defaults, so a default reconstruction drops
    // the keys and still verifies. A non-default value never matches.
    if (const auto* rec_drive = find(field(expected, "physics"), "drive")) {
        const auto* rec_pedaling = find(*rec_drive, "pedaling");
        auto* drive = find_mutable(physics, "drive");
        auto* pedaling = drive ? find_mutable(*drive, "pedaling") : nullptr;
        if (rec_pedaling && pedaling && !find(*rec_pedaling, "reposition_on_stall")) {
            const bike::configuration::PedalingConfig defaults{};
            drop_if_default_reposition(*pedaling, defaults);
        }
    }
```

Implement `drop_if_default_reposition` as a file-local function that erases each `reposition_*` key whose value equals the default (bool and real compare). It leaves any non-default key in place, so verification fails closed. Use the accessor names this file already uses. `find_mutable` stands for whatever mutable lookup `rebuild.cpp` uses (the `aece48d7` hunk uses `std::get<io::Value::Object>(physics.data)` with `std::erase_if`), so follow that exact style.

- [ ] **Step 5: Update the config fixtures**

Rebuild, then run: `ctest --test-dir build/native-release -R '^(config_schema|config_resolution)$' --output-on-failure`
These fail on the new keys. Add the 9 keys with default values to `drive.pedaling` in each fixture listed under **Files**, in the field order above. Keep the files' existing formatting (`git show aece48d7 -- tests/fixtures/config` shows the style). Then extend `test_schema.cpp` with a check that `reposition_on_stall = true` together with `pedal_attachment = "flat"` is rejected, using the same `bt::invalid` pattern as the neighbouring checks.

- [ ] **Step 6: Run the fast group (S1 check)**

Run: `ctest --test-dir build/native-release -LE 'slow|package'`
Expected: 181/183 with the same two baseline failures. Any new failure under a golden comparison → S1.

- [ ] **Step 7: Commit**

```bash
git add native/src/configuration native/src/replay/rebuild.cpp tests/fixtures/config native/tests/reference/test_pedaling_policy.cpp native/tests/config/test_schema.cpp
git commit -m "feat(pedaling): restore the reposition config knobs (default off)"
```

### Task 3: Progress stall detector

**Files:**
- Create: `native/src/drivetrain/stall_detector.hpp`
- Test: `native/tests/reference/test_pedaling_policy.cpp`

**Interfaces:**
- Produces: `drivetrain::StallDetectorState` (POD) and `bool drivetrain::stall_update(StallDetectorState&, const StallDetectorLimits&, double crank_rad_s, double wheel_rad_s, double dt, bool loaded) noexcept`.

- [ ] **Step 1: Write the failing tests**, ported from `c809f12a^:tests/test_pedaling_stall.py`:

```cpp
#include "drivetrain/stall_detector.hpp"
// ProgressStallDetector ports (c809f12a^ tests/test_pedaling_stall.py).
void test_stall_detector() {
    using drivetrain::StallDetectorLimits; using drivetrain::StallDetectorState;
    constexpr double dt = .00125;
    const StallDetectorLimits limits{.window_s = 1., .crank_progress_rad = .5, .wheel_progress_rad = .5};
    {   // wheel progress alone prevents a stall
        StallDetectorState s; bool fired = false;
        for (int i = 0; i < 2400; ++i) fired = drivetrain::stall_update(s, limits, 0., 1., dt, true) || fired;
        bt::require(!fired, "a rolling wheel with a parked crank is not a stall");
    }
    {   // rollback is not progress: fires exactly once after the window
        StallDetectorState s; int fired = 0;
        for (int i = 0; i < 880; ++i) fired += drivetrain::stall_update(s, limits, -.8, -2., dt, true) ? 1 : 0;
        bt::require(fired == 1, "backward motion accumulates as no progress");
    }
    {   // dropping the load restarts the window
        StallDetectorState s;
        for (int i = 0; i < 720; ++i) bt::require(!drivetrain::stall_update(s, limits, 0., 0., dt, true), "no early fire");
        (void)drivetrain::stall_update(s, limits, 0., 0., dt, false);
        bool fired = false;
        for (int i = 0; i < 720; ++i) fired = drivetrain::stall_update(s, limits, 0., 0., dt, true) || fired;
        bt::require(!fired, "a stall needs a full loaded window");
    }
}
```

Call `test_stall_detector()` from the run body. Run the node: it fails to compile (header missing).

- [ ] **Step 2: Implement**

```cpp
#pragma once
// Loaded drive that makes no forward progress within a window
// (c809f12a^ pedaling.py ProgressStallDetector). A low-rate test misses the
// real dead-spot stall: the crank rocks about top/bottom dead centre at
// several rad/s while going nowhere. Net forward advance of the crank or of
// the wheel (crank-equivalent) since the last anchor separates a slow grind
// from a stall; rocking and rollback both count as no progress.
#include <optional>

namespace drivetrain {
struct StallDetectorLimits { double window_s, crank_progress_rad, wheel_progress_rad; };
struct StallDetectorState {
    double crank_rad = 0., wheel_rad = 0.;
    std::optional<double> elapsed_s;
    friend bool operator==(const StallDetectorState&, const StallDetectorState&) = default;
};
// True once per stall; the next stall needs a fresh full window.
[[nodiscard]] inline bool stall_update(StallDetectorState& s, const StallDetectorLimits& l,
                                       double crank_rad_s, double wheel_rad_s, double dt, bool loaded) noexcept {
    if (!loaded) { s = {}; return false; }
    if (!s.elapsed_s) { s.elapsed_s = 0.; return false; }
    s.crank_rad += crank_rad_s * dt;
    s.wheel_rad += wheel_rad_s * dt;
    if (s.crank_rad > l.crank_progress_rad || s.wheel_rad > l.wheel_progress_rad) {
        s = {}; s.elapsed_s = 0.; return false;
    }
    *s.elapsed_s += dt;
    if (*s.elapsed_s >= l.window_s) { s = {}; return true; }
    return false;
}
}  // namespace drivetrain
```

- [ ] **Step 3: Run** → PASS. **Step 4: Commit** `feat(pedaling): native progress-stall detector`.

### Task 4: Reposition state machine in PedalingPolicy

**Files:**
- Modify: `native/src/config_types.hpp` (`PedalingSnapshot` at `:55`)
- Modify: `native/src/drivetrain/pedaling.hpp`, `native/src/drivetrain/pedaling.cpp`
- Test: `native/tests/reference/test_pedaling_policy.cpp`

**Interfaces:**
- Consumes: `StallDetectorState`, `stall_update` (Task 3); config fields (Task 2); `kPowerPhaseOffsetRad` value (Task 1).
- Produces: `CoastReason::reposition_requested`, `CoastReason::reposition_stall` (wire names `"reposition_requested"`, `"reposition_stall"`); `PedalingPolicy::update(..., bool braking, bool reposition_request)`; `RepositionSnapshot`; `PedalingSnapshot::reposition` (`std::optional<RepositionSnapshot>`); `bool drivetrain::is_reposition(CoastReason) noexcept`.

- [ ] **Step 1: Write the failing tests** (ports of `test_rocking_dead_spot_stall_triggers_reposition`, `test_static_dead_spot_stall_still_triggers`, `test_slow_forward_grind_is_not_a_stall`, plus goal/no-op/abort/cooldown):

```cpp
namespace {
constexpr double kDt = .00125;
constexpr double kDeadSpot = 81. * std::numbers::pi + .5 * std::numbers::pi;  // phase % pi == pi/2
drivetrain::PedalingConfig reposition_config() {
    drivetrain::PedalingConfig c; c.enabled = true; c.reposition_on_stall = true; return c;
}
}
void test_reposition_stall_reflex() {
    using drivetrain::CoastReason;
    {   // rocking about the dead spot with no net advance fires within the window
        drivetrain::PedalingPolicy p(reposition_config());
        const double amp = .26, peak = 4.6, w = peak / amp;
        double first = -1.;
        for (int i = 0; i < 1600 && first < 0.; ++i) {
            const double t = i * kDt;
            const auto s = p.update(kDeadSpot + amp * std::sin(w * t), amp * w * std::cos(w * t), 0., 20., kDt);
            if (s.reason == CoastReason::reposition_stall) first = t;
        }
        bt::require(first >= 0. && first <= 1.05, "rocking stall triggers the reflex within the window");
    }
    {   // a frozen crank on the dead spot is a stall; the goal is the lower power phase
        drivetrain::PedalingPolicy p(reposition_config());
        std::optional<drivetrain::PedalingState> hit;
        for (int i = 0; i < 1200 && !hit; ++i) {
            const auto s = p.update(kDeadSpot, 0., 0., 20., kDt);
            if (drivetrain::is_reposition(s.reason)) hit = s;
        }
        bt::require(hit.has_value(), "static dead-spot stall triggers");
        bt::require(hit->mode == drivetrain::PedalMode::coasting && hit->effort_nm == 0., "manoeuvre reports coasting, zero effort");
        bt::require(hit->target_rate_rad_s < 0., "manoeuvre backpedals");
        bt::require(p.state().reposition && p.state().reposition->goal_rad &&
                    std::abs(*p.state().reposition->goal_rad - 81. * std::numbers::pi) < 1e-9,
                    "goal is the nearest lower power phase");
    }
    {   // slow forward grind (~1 rad/s) is progress
        drivetrain::PedalingPolicy p(reposition_config());
        for (int i = 0; i < 4000; ++i) {
            const double t = i * kDt;
            bt::require(!drivetrain::is_reposition(p.update(kDeadSpot + t, 1., 9.5, 20., kDt).reason), "no reflex on a grind");
        }
    }
    {   // a crank already near a power phase is a no-op
        drivetrain::PedalingPolicy p(reposition_config());
        const auto s = p.update(81. * std::numbers::pi + .05, 0., 0., 20., kDt, true, false, true);
        bt::require(!drivetrain::is_reposition(s.reason), "request inside the no-op band does nothing");
    }
    {   // braking aborts and starts the cooldown; a held request does not retrigger
        drivetrain::PedalingPolicy p(reposition_config());
        auto s = p.update(kDeadSpot, 0., 0., 20., kDt, true, false, true);
        bt::require(s.reason == CoastReason::reposition_requested, "request starts the manoeuvre");
        s = p.update(kDeadSpot, 0., 0., 20., kDt, true, true, true);
        bt::require(!drivetrain::is_reposition(s.reason), "braking aborts");
        for (int i = 0; i < 1000; ++i)
            bt::require(!drivetrain::is_reposition(p.update(kDeadSpot, 0., 0., 20., kDt, true, false, true).reason),
                        "held request + cooldown never retrigger");
    }
}
```

Call it from the run body. Run: fails to compile (`is_reposition`, the 8th parameter).

- [ ] **Step 2: Snapshot type**

In `config_types.hpp`, before `PedalingSnapshot`:

```cpp
    // Engaged only while the reposition machinery has state worth carrying
    // (reflex enabled, a manoeuvre, or a cooldown). Absent = byte-identical
    // historical snapshot. Cooldown/elapsed in seconds; reason 1=requested,
    // 2=stall (0 = idle).
    struct RepositionSnapshot {
        double crank_rad = 0., wheel_rad = 0.;
        std::optional<double> stall_elapsed_s;
        std::optional<double> goal_rad;
        double elapsed_s = 0., cooldown_s = 0.;
        bool armed = true;
        std::uint8_t reason = 0;
        friend bool operator==(const RepositionSnapshot&, const RepositionSnapshot&) = default;
    };
```

Add `std::optional<RepositionSnapshot> reposition;` as the last member of `PedalingSnapshot`. Include `<cstdint>` if it is missing.

- [ ] **Step 3: Reasons and policy**

In `pedaling.hpp`, append `reposition_requested, reposition_stall` to `CoastReason`. Extend `coast_reason_name` and `coast_reason` with `"reposition_requested"` and `"reposition_stall"`, and add:

```cpp
    [[nodiscard]] constexpr bool is_reposition(CoastReason r) noexcept {
        return r == CoastReason::reposition_requested || r == CoastReason::reposition_stall;
    }
    // Crank qpos of the first power phase (Task 1 pins it; multiples of pi
    // from it are the two horizontal crank poses).
    inline constexpr double kPowerPhaseOffsetRad = 0.;
```

Change the declaration to `update(double phase_rad, double rate_rad_s, double required_cadence_rpm, double effort_nm, double dt, bool enabled = true, bool braking = false, bool reposition_request = false);`.

In `pedaling.cpp` `update`, before `required = std::max(required, 0.);` keep the signed value: `const double wheel_rad_s = required * 2. * std::numbers::pi / 60.;`. After the `if (!enabled) {...}` block, insert the manoeuvre logic. It mirrors `c809f12a^ pedaling.py:116-163`:

```cpp
        const bool tracked = config_.reposition_on_stall || reposition_request ||
                             (state_.reposition && (state_.reposition->goal_rad || state_.reposition->cooldown_s > 0.));
        if (tracked && !state_.reposition) state_.reposition.emplace();
        if (state_.reposition) {
            auto& r = *state_.reposition;
            r.cooldown_s = std::max(0., r.cooldown_s - dt);
            if (braking) { r.goal_rad.reset(); r.reason = 0; r.cooldown_s = config_.reposition_cooldown_s; }
            bool request_edge = false;
            if (reposition_request) { request_edge = r.armed; r.armed = false; } else r.armed = true;
            const bool loaded = config_.reposition_on_stall && !braking && !r.goal_rad && r.cooldown_s <= 0. &&
                                effort >= config_.reposition_min_effort_nm;
            StallDetectorState detector{.crank_rad = r.crank_rad, .wheel_rad = r.wheel_rad, .elapsed_s = r.stall_elapsed_s};
            const bool reflex_edge = stall_update(detector,
                {.window_s = config_.reposition_stall_window_s,
                 .crank_progress_rad = config_.reposition_stall_progress_rad,
                 .wheel_progress_rad = config_.reposition_stall_progress_rad},
                rate, wheel_rad_s, dt, loaded);
            r.crank_rad = detector.crank_rad; r.wheel_rad = detector.wheel_rad; r.stall_elapsed_s = detector.elapsed_s;
            for (const auto& [edge, why] : {std::pair{request_edge, std::uint8_t{1}}, std::pair{reflex_edge, std::uint8_t{2}}}) {
                if (braking || !edge || r.goal_rad || r.cooldown_s > 0.) continue;
                const double rel = phase - kPowerPhaseOffsetRad;
                double turn = std::fmod(rel, std::numbers::pi);
                if (turn < 0.) turn += std::numbers::pi;
                if (std::min(turn, std::numbers::pi - turn) <= config_.reposition_noop_rad) continue;
                r.goal_rad = phase - turn; r.reason = why; r.elapsed_s = 0.;
                state_.effort = 0.; state_.coasting = false;
                break;
            }
            if (r.goal_rad) {
                const double remaining = phase - *r.goal_rad;
                r.elapsed_s += dt;
                if (remaining <= config_.reposition_phase_tolerance_rad || r.elapsed_s > config_.reposition_timeout_s) {
                    r.goal_rad.reset(); r.reason = 0; r.cooldown_s = config_.reposition_cooldown_s;
                } else {
                    constexpr double kApproachTauS = .05;
                    const double target_rate = -std::min(config_.reposition_back_rate_rad_s, remaining / kApproachTauS);
                    return {.mode = PedalMode::coasting,
                            .reason = r.reason == 1 ? CoastReason::reposition_requested : CoastReason::reposition_stall,
                            .effort_nm = 0., .required_cadence_rpm = std::max(required, 0.),
                            .target_phase_rad = phase + target_rate * dt, .target_rate_rad_s = target_rate};
                }
            }
            if (!config_.reposition_on_stall && !r.goal_rad && r.cooldown_s <= 0. && r.armed) state_.reposition.reset();
        }
```

`required` here is still the signed value. Clamp it right after this block with `required = std::max(required, 0.);`, which moves the existing line down. Every later `reset()` in `update` (the pedaling branch calls `reset()` and restores effort/EMA) must also preserve `state_.reposition`, as the Python `latch` did. Change that block to save and restore `state_.reposition` next to `previous` and `ema`. `set_state` gets no new checks beyond `finite_optional(s.reposition->goal_rad, ...)` when engaged.

- [ ] **Step 4: Run** `reference.pedaling_policy` → PASS. Then run the fast group → 181/183 (R6: the reposition state stays disengaged with the defaults).

- [ ] **Step 5: Commit** `feat(pedaling): reposition manoeuvre state machine`.

### Task 5: Serialize the reposition state only when engaged

**Files:**
- Modify: `native/src/drivetrain/state_codec.cpp` (pedaling encode at `:405-414`, decode at `:269-282`)
- Test: `native/tests/io/test_state_codecs.cpp`

**Interfaces:**
- Consumes: `RepositionSnapshot` (Task 4).
- Produces: wire key `drive.pedaling._reposition` (object), present only when `PedalingSnapshot::reposition` is engaged.

- [ ] **Step 1: Failing test.** In `test_state_codecs.cpp`, add a round trip: take the drive state fixture this file already decodes, set `pedaling.reposition = RepositionSnapshot{.goal_rad = 1.0, .elapsed_s = .1, .reason = 2}`, encode, decode, and require equality. Also require that encoding the unmodified fixture state reproduces the fixture bytes exactly (the existing canonical check, if present, covers this; otherwise compare against `bt::text(fixture)`).
- [ ] **Step 2: Implement.** Encode: after `_effort`/`_cadence_ema`, `if (s.pedaling.reposition) p.emplace_back("_reposition", encode_reposition(*s.pedaling.reposition));`. Here `encode_reposition` emits `crank_rad, wheel_rad, stall_elapsed_s (optional), goal_rad (optional), elapsed_s, cooldown_s, armed, reason` with the same `real_value`/`optional_value` helpers. Decode: relax the pedaling `exact(...)` to accept the historical key set plus optional `_reposition`, and decode it back field by field with the same validators (finite reals, nonnegative times, `reason ∈ {0,1,2}`).
- [ ] **Step 3: Run** `ctest -R 'state_codec|native_state_restore|native_drive_policies|native_drivetrain'`, then the fast group → 181/183.
- [ ] **Step 4: Commit** `feat(pedaling): carry reposition state in the drive snapshot when engaged`.

### Task 6: Rider command flag and controller reposition branch

**Files:**
- Modify: `native/src/rider/spindle_controller.hpp` (`RiderCommand` at `:119`)
- Modify: `native/src/rider/spindle_controller.cpp` (the `else` branch after `if (command.mean_crank_torque_nm > 0.)`, about `:1196-1228`)
- Modify: `native/src/rider/state_codec.cpp` (`decode_rider_command` at `:2053` and its encoder)
- Modify: `native/src/runtime/step.cpp` (command build at `:1347-1358`)
- Test: `native/tests/reference/test_native_spindle_controller.cpp`

**Interfaces:**
- Consumes: `drivetrain::is_reposition` (Task 4).
- Produces: `RiderCommand::crank_reposition` (bool, default false); codec key `crank_reposition` emitted only when true.

The current coasting branch runs a press servo: `coast_hold_nm_` integrates `-weld_crank_torque_nm`, so it drives the delivered crank torque to zero, which is what a coasting rider resting their feet should do. During the manoeuvre that servo would cancel the backward drive, so the manoeuvre gets its own branch.

- [ ] **Step 1: Failing test.** In `test_native_spindle_controller.cpp`, using the file's existing controller fixture: issue `RiderCommand{.mean_crank_torque_nm = 0., .crank_target_phase_rad = 0., .crank_target_rate_rad_s = -3., .crank_reposition = true}` at crank rate 0. Require that the requested crank torque is negative and `>= -coasting_brake_limit_nm`. Also require that repeating the same command with a fake weld reading equal to that torque leaves the request unchanged (no press-servo cancellation).
- [ ] **Step 2: Implement.** Add `bool crank_reposition = false;` to `RiderCommand`. In the `else` branch:

```cpp
        } else if (command.crank_reposition) {
            // Reposition backpedal: damp toward the negative crank-rate goal.
            // The coasting press servo is not run (it would zero the very
            // torque the manoeuvre needs); its hold is neither integrated
            // nor applied, and resumes from its held value afterwards.
            const double raw = config_.coasting_brake_d_nm_s_rad *
                               (command.crank_target_rate_rad_s - rate);
            total = std::clamp(raw, -config_.coasting_brake_limit_nm, config_.coasting_brake_limit_nm);
        } else {
```

Codec: in the encoder, emit `crank_reposition: true` only when set. In `decode_rider_command`, accept the historical five keys plus an optional `crank_reposition` boolean, and reject `crank_reposition` together with `mean_crank_torque_nm > 0`.

In `step.cpp`, after `command.crank_target_rate_rad_s = ...`: `command.crank_reposition = drivetrain::is_reposition(pedaling.reason);`.

- [ ] **Step 3: Run** `reference.native_spindle_controller`, `reference.native_coupled_rider_task`, then the fast group → 181/183.
- [ ] **Step 4: Commit** `feat(rider): spindle reposition branch outside the coasting press servo`.

### Task 7: Request plumbing and the backpedal topology check

**Files:**
- Modify: `native/src/writers/drivetrain.hpp` (`RideControl` at `:37`: add `bool crank_reposition = false;`)
- Modify: `native/src/writers/drivetrain.cpp:818` (pass `in.control.crank_reposition` as the 8th argument)
- Modify: `native/src/runtime/step.hpp/.cpp` (member `bool pending_reposition_ = false;`, `void request_crank_reposition() noexcept`; at `:1237` set `.crank_reposition = std::exchange(pending_reposition_, false)` only on an advancing control tick)
- Modify: `native/src/runtime/runtime.hpp/.cpp` (`void request_crank_reposition()` forwarding to the physical step; follow how `NativeRideRuntime` reaches `PhysicalStep` in other forwarding methods, `grep -n 'step_->' native/src/runtime/runtime.cpp`)
- Create: `native/tests/reference/test_crank_reposition.cpp`
- Modify: `native/tests/reference/Phase3Ports.cmake` (register `crank_reposition`, labels `t52 drive rider slow`)

**Interfaces:**
- Consumes: Tasks 4–6.
- Produces: `runtime::NativeRideRuntime::request_crank_reposition()`.

- [ ] **Step 1: Failing plant test (S3 + R3/R5).** Build the environment with the `weld_env` pattern from `native/tests/integration/test_rider_welds.cpp:65-94`: welded example, 30 m flat, `initial_speed_mps = 0`, diagnostic limits. Also set `request.setup.model.physics->drive.crank_phase_rad` to a dead spot (`kPowerPhaseOffsetRad + π/2 + 0.6`), so the goal is 0.6 rad back. Step with released brakes and `human_torque_nm = 0`. At step 40, call `runtime->request_crank_reposition()`. Then for 3 s read through `RuntimeRigAccess::inspect`: crank qpos/qvel (`crank_spin`), rear wheel qvel (`rear_wheel_spin`), and `stepper.rider_contacts().state().settled_welds` entries `front_pedal`/`rear_pedal`. Require:
  - the crank ends within 0.05 rad of the goal, having moved only backward during the manoeuvre;
  - the rear wheel's backward displacement over the manoeuvre is below 0.02 rad (S3);
  - `min(front_pedal.normal_n, rear_pedal.normal_n) >= -20` throughout the manoeuvre (R5/D4);
  - `|weld_crank_torque_nm| <= coasting_brake_limit_nm` throughout.

If the drive config field for the initial crank phase is named differently, find it with `grep -n crank_phase_rad native/src/configuration/physical_config.hpp`.

- [ ] **Step 2: Run** → FAIL (`request_crank_reposition` missing). **Step 3: Implement** the plumbing listed under **Files**. **Step 4: Run** → PASS. If the wheel assertion fails → S3. If the goal is not reached or the tension bound fails → S4, with the measured numbers.
- [ ] **Step 5: Fast group** → 181/183. **Step 6: Commit** `feat(runtime): one-shot crank reposition request`.

### Task 8: Viewer key V (physical mode)

**Files:**
- Modify: `native/src/viewer/input.hpp` (`Action::crank_reposition`, appended last), `native/src/viewer/input.cpp` (`case 'V'` → `crank_reposition` only for `Mode::physical`; add it to the `apply` no-op list; physical help text gains `V crank reposition`)
- Modify: `native/src/viewer/session.hpp` (`std::function<void()> crank_reposition;` in `SessionHooks`), `native/src/viewer/ride.cpp` (call it on the action)
- Modify: `native/src/cli/ride.cpp:192` area (`hooks.crank_reposition=[&]{runtime->request_crank_reposition();(void)journal->emit("crank_reposition",wall(),runtime->snapshot().step);};`)
- Test: the viewer input test (`grep -ln action_for_key native/tests`)

- [ ] **Step 1: Failing test.** Require `action_for_key(Mode::physical, 'V') == Action::crank_reposition`, `action_for_key(Mode::research, 'v') == Action::none` (D1), and that `help_text(Mode::physical)` contains `"V crank reposition"`.
- [ ] **Step 2: Implement**, rebuild both `build/native-release` and `build/gui` (the GUI branch only compiles with `BIKE_BUILD_GUI=ON`), and run the test plus `viewer_render viewer_hud` in both trees.
- [ ] **Step 3: Commit** `feat(viewer): V requests a crank reposition in the physical viewer`.

### Task 9: Acceptance on real tracks, docs, gates

**Files:**
- Modify: `docs/RIDE.md` (Interactive controls: V; a short "Crank reposition" paragraph with the knobs and the D1 scope)

- [ ] **Step 1: Measure.** Write two copies of `examples/research/viewer_physics_welded.toml` into a scratch directory, with the rider JSON files copied next to them (paths resolve relative to the TOML). One copy has `reposition_on_stall = true` under `[drive.pedaling]`. For each of `examples/research/rough_uphill_extreme.toml` and `examples/research/alpine_rocky.toml`, run 60 s headless, `--check-policy diagnostic --record --decimate 1`, with reflex off and on, one run at a time. From each recording (`bike-recording sql`), report distance reached, outcome, number of manoeuvres (rows whose `channel_drive` coast reason is `reposition_stall`), and min weld normal on both pedals during manoeuvres.
- [ ] **Step 2: Report D3.** Present the table to the user. Do not switch the examples on without their decision.
- [ ] **Step 3: Docs.** Document the V key and the knobs in `docs/RIDE.md`.
- [ ] **Step 4: Gates.** Run `bash tools/run_tests.sh native`, then `bash tools/run_tests.sh full` (or, if `full` stops on the documented baseline failures under `set -e`, run its `slow` and `package` label gates directly, as `tools/run_tests.sh` does). Compare against the baseline listed in Global Constraints.
- [ ] **Step 5: Commit** `docs(ride): document crank reposition`.

## Self-review notes

- R1 → Tasks 3–4. R2 → Tasks 7–8. R3 → Tasks 1, 4, 7. R4 → Task 4. R5 → Tasks 6–7. R6 → the fast-group check in every task, plus Task 9 gates.
- Names used across tasks: `reposition_*` config fields (Task 2); `StallDetectorState`/`stall_update` (3); `RepositionSnapshot`, `CoastReason::reposition_{requested,stall}`, `is_reposition`, `kPowerPhaseOffsetRad`, the 8-argument `update` (4); `_reposition` wire key (5); `RiderCommand::crank_reposition` (6); `drivetrain::RideControl::crank_reposition`, `request_crank_reposition` (7); `Action::crank_reposition`, `SessionHooks::crank_reposition` (8).
- Known risks: the exact helper names in `rebuild.cpp` and `drivetrain/state_codec.cpp` (Tasks 2, 5) must follow those files; the steps name the neighbouring code to copy from.
