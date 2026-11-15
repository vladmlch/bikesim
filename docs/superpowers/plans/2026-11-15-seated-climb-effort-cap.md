# Потолок усилия сидячего подъёма по уклону и скорости и закрытие прихватывания заднего тормоза — план

**Цель.** Выполнить пункты 2 и 3 очереди пост-мердж доработок (`q2-probe/report.md`, `q2-q3-q8-triage.md`, «Решено — не сделано» п. 2в и п. 17):
1. **Пункт 2 (Вариант 2 — потолок усилия `[seated_climb]` по уклону и скорости + расширение стартовых узлов 22 % / 25 %):** ограничить крутящий момент шатунов на малых скоростях на подъёме в `RiderIntent::tick` поверх `SeatedClimbPolicy`, линейно поднимая потолок от табличного `climb_effort_cap_grade_nm(grade_ahead)` на `speed_mps <= climb_effort_rise_speed_mps` до `max_crank_torque_nm` при `speed_mps >= climb_effort_full_speed_mps`, и применить откалиброванные таблицы для `normal` (`N4_lin_sc35`), `careful` (`C1_lin_nosurf`) и `aggressive` (`A1_lin_sc35`).
2. **Пункт 3 (прихватывание заднего тормоза при отрыве переднего колеса — закрытие без включения):** оставить `rear_brake_tap_enabled = false` как эмпирически отвергнутый вариант. По замерам `q2-probe` нажатие `RiderCommands::rear_brake` активирует мгновенное отключение мотора по §16 (`motor_braking`, сброс 67.5 → 0 Н·м), вызывая удар переднего колеса о грунт (994–1112 Н), срывы `grip.pull` / потерю седла (`sad_min = 6.1 Н`, `bal = 1`), тогда как откалиброванный Вариант 2 уже даёт `0` падений и `0` отрывов после передачи управления на асфальте и сыпучем грунте.

**Принятые проектные границы.**
- **Схема `[seated_climb]` (`rider::IntentConfig` / `SeatedClimbConfig`):** четыре обязательных поля без скрытых кодовых умолчаний в `IntentConfig` и `decode_intent_config`:
  - `climb_effort_cap_grades` (`std::vector<double>`) — строго возрастающие конечные узлы уклона;
  - `climb_effort_cap_grade_nm` (`std::vector<double>`) — положительные конечные потолки усилия той же длины;
  - `climb_effort_rise_speed_mps` (`double`) — конечная неотрицательная скорость начала линейного подъёма потолка;
  - `climb_effort_full_speed_mps` (`double`) — конечная скорость выхода на `max_crank_torque_nm` (`>= climb_effort_rise_speed_mps`).
- **Общий интерполятор и валидация:** единый helper кусочно-линейной интерполяции и проверки строго монотонных узлов для `[rider_launch]` и `[seated_climb]`.
- **Порядок применения в `RiderIntent::tick`:** потолок вычисляется по задержанным ощущениям `percepts->senses` (`grade_ahead`, `speed_mps`) и применяется как `if (ccap < intent.effort_ceiling_nm) intent.effort_ceiling_nm = ccap;` строго **до** смешивания передачи от `LaunchPlan` (`if (owns)`) и **до** ослабления реакции `FrontLiftReaction` (`ease_factor` в `output()`). При инертной таблице (`climb_effort_cap_grades = [0.0]`, `climb_effort_cap_grade_nm = [1000.0]`) `ccap >= intent.effort_ceiling_nm`, поэтому `effort_ceiling_nm` и `effort_limited` остаются побитово неизменными.
- **Кэш равновесия и перенос состояния:** в `equilibrium_cache.cpp` и `initial_state.cpp` четыре ключа `climb_effort_cap_*` исключаются из отпечатка/контракта наряду с `min_elbow_flexion_deg`, так как статическое равновесие решается при `v = 0` без динамики подъёма; существующие `.npz` и `.initial_state` фикстуры не ломаются.
- **Калиброванные профили (`examples/research/*.toml`):**
  - `viewer_physics_{welded,distributed,pneumatic}.toml` (`N4_lin_sc35`): `[seated_climb]` `climb_effort_cap_grades = [0.0, 0.05, 0.10, 0.15, 0.20, 0.22, 0.25]`, `climb_effort_cap_grade_nm = [45.0, 35.0, 30.0, 30.0, 34.0, 34.0, 35.0]`, `climb_effort_rise_speed_mps = 2.5`, `climb_effort_full_speed_mps = 4.0`; `[rider_launch]` дополняется узлами `0.22, 0.25` (`21.0, 22.5`).
  - `rider_type_careful.toml` (`C1_lin_nosurf`): `[seated_climb]` `climb_effort_cap_grades = [0.0, 0.05, 0.10, 0.15, 0.20, 0.22, 0.25]`, `climb_effort_cap_grade_nm = [38.0, 25.0, 22.0, 25.0, 30.0, 30.0, 32.0]`, `climb_effort_rise_speed_mps = 2.5`, `climb_effort_full_speed_mps = 4.0`; `[rider_launch]` дополняется узлами `0.22, 0.25` (`18.0, 21.0`).
  - `rider_type_aggressive.toml` (`A1_lin_sc35`): `[seated_climb]` `climb_effort_cap_grades = [0.0, 0.05, 0.10, 0.15, 0.20, 0.22, 0.25]`, `climb_effort_cap_grade_nm = [50.0, 35.0, 32.0, 39.5, 35.0, 35.0, 35.0]`, `climb_effort_rise_speed_mps = 2.5`, `climb_effort_full_speed_mps = 4.0`; `[rider_launch]` без изменений.

## Задачи

- [ ] **Задача 1. Общий интерполятор, схема конфигурации, валидация, кодек и отпечаток равновесия.**
  - Выделить общий неаллоцирующий helper кусочно-линейной интерполяции и проверки строго возрастающих конечных узлов (`native/src/rider/table_lookup.hpp`) для переиспользования в `launch_config.cpp` и `intent.cpp`.
  - Добавить 4 обязательных поля `climb_effort_cap_*` в `rider::IntentConfig` (`native/src/rider/intent.hpp`), `kSeatedClimbConfigFields` (`native/src/configuration/model_config.hpp`), `post_init(const SeatedClimbConfig&)` и `default_physics()` (`native/src/configuration/validate.cpp`).
  - Обновить `decode_intent_config` (`native/src/rider/state_codec.cpp`) и проверки конструкторов `SeatedClimbPolicy` / `RiderIntent`.
  - Исключить 4 ключа `climb_effort_cap_*` рядом с `min_elbow_flexion_deg` в `equilibrium_cache.cpp` и `initial_state.cpp`.

- [ ] **Задача 2. Реализация потолка усилия в `RiderIntent::tick`.**
  - Реализовать расчёт `compute_climb_cap_nm` (кусочно-линейный уклон по `percepts->senses.grade_ahead` и линейная рампа по `percepts->senses.speed_mps` на `[rise, full]` к `config_.max_crank_torque_nm`) в `native/src/rider/intent.cpp` строго до `if (owns)` и без изменения `output()` / `effort_limited`.
  - Проверить побитовую неизменность `effort_ceiling_nm` и `effort_limited` при инертной таблице `[0.0] -> [1000.0]`.

- [ ] **Задача 3. Обновление профилей, C++-билдеров, фикстур и золотых входов.**
  - Обновить 5 профилей `examples/research/*.toml` согласно калибровке `N4_lin_sc35`, `C1_lin_nosurf`, `A1_lin_sc35`.
  - Обновить C++-билдеры в `native/src/validation/*` и `native/tests/**`, TOML/JSON-фикстуры в `tests/fixtures/{config,bootstrap,ft2,golden,replay,research}/**` (включая `assets.json` sha256 при обновлении `resolved_config.json`), а также только `input.adapter.*` в `native_rider_intent` (без перезаписи `output.*` / `error.*`).

- [ ] **Задача 4. Тесты и приёмка.**
  - Обновить `test_welded_profile_config.cpp` (таблицы профилей, проверка 5 профилей, фильтр 4 ключей `climb_effort_cap_*` в `[seated_climb]` без пропуска остальной секции, проверки обязательности/валидации).
  - Добавить модульные тесты: кусочно-линейная интерполяция с зажатием концов, линейный подъём по скорости на `[rise, full]`, насыщение на `full_speed` до `max_crank_torque_nm`, побитовая эквивалентность инертной таблицы `1000.0`, ошибки валидации и round-trip через `state_codec` / `schema`.
  - Добавить приёмочные тесты подъёма на 20 с (включая проверку `g15`, `g20`, `g25`, `loose25`, `restart_g25` для профилей).
  - Сборка `build/native-release`, целевой прогон `ctest` с `perl -e 'alarm 2400; exec @ARGV' --` и фиксация коммитов на `queue-2026-11-15`.
