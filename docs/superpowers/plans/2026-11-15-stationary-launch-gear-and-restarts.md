# Переключение вниз на стартовую передачу на месте и рестарты — план

**Цель.** Устранить срыв рестарта после полной остановки посреди поездки (пункт 16 «Решено — не сделано», П9) по решениям пользователя от 2026-11-15 (сессия `1303e344`): восстановить 3 с форы `BalanceMonitor` после каждой полной остановки на тормозах, разрешить прямой выбор стартовой передачи на месте в `LaunchPhase::hold` перед подъёмом без изменения схем `state.json` и каналов записи, и добавить приёмочный тест рестарта после остановки на 5 % и 10 %.

**Решения пользователя (2026-11-15, сессия `1303e344`).**
- **Перезапуск форы `BalanceMonitor`:** полная остановка на тормозах (`motor_braking && |speed_mps| <= standstill_speed_mps`) сбрасывает `started_s`, чтобы после отпускания тормозов снова давались полные `balance_grace_s = 3.0` с (кратковременное притормаживание на ходу по-прежнему только ставит счётчик простоя на паузу).
- **Выбор стартовой передачи на месте в `LaunchPhase::hold`:** вместо моделирования удержания велосипеда руками и прокручивания педалей/колеса на месте запрет переключения стоя для остановки посреди поездки перед подъёмом заменён: при `LaunchPhase::hold`, `grade_ahead > 0` и `rear_teeth < prepare_launch(config, grade_ahead).rear_teeth` симуляция переключается на `prepare_launch(config, grade_ahead).rear_teeth` прямо на месте (прыжком через несколько звёзд).
- **Совместимость схем:** не добавлять поля в `LaunchPlan::State`, `RiderIntent::State`, `BalanceMonitor::State`, `ShiftingSnapshot`, wire-канал `rider_commands`, `BalanceMonitor::observation()` и `native/src/recording/catalog_data.inc`. Все существующие `state.json` и `.bin` эталоны, а также холодные старты с `t = 0` остаются без изменений.
- **Приёмочные тесты рестарта (`test_normal_rider_restart_after_a_stop`):** на уклонах 5 % и 10 % торможение выполняется плавно (линейный вход на `0.5` и удержание до полной остановки), а чистота рестарта оценивается от момента отпускания тормоза (`LaunchPhase::release` .. `handover`). Отскок вилки при 100 % торможении вынесен в отдельный пункт очереди.

## Задачи

- [x] **Задача 1. `BalanceMonitor`: перезапуск 3 с форы после каждой полной остановки на тормозах.** (`4491661a`, `b3b9fefa`, `e0f999f4`)
  - Обновить `BalanceMonitor::update(..., BalanceStop stop = BalanceStop::none)` в `native/src/runtime/control.{hpp,cpp}`: при `!riding && stop == BalanceStop::braked` сбрасывать `started_s`, сохраняя `state().event` и прежнюю паузу при кратковременном притормаживании на ходу (`!riding && BalanceStop::none`).
  - В `PhysicalStep::balance_channel` (`native/src/runtime/step.cpp`) выставлять `BalanceStop::braked` при `brakes.motor_braking && std::abs(speed_mps) <= config_.launch.standstill_speed_mps && (rider_intent_.launch().phase == rider::LaunchPhase::hold || grace_spent)` (сохраняет побитовую совместимость ранних эталонов вроде `research_flat` на `t = 0.01 s`).
  - Добавить модульный тест `test_full_stop_rearms_grace_after_brake_release` в `native/tests/reference/test_balance_monitor.cpp` и проверить `reference.balance_monitor` (пройден) и `reference.native_research_environment` (пройден).

- [x] **Задача 2. Выбор стартовой передачи на месте в `LaunchPhase::hold` (без изменения схем).** (`1e238fa4`, `b3b9fefa`)
  - Добавить `LaunchPlan::launch_teeth(double grade_ahead) const` в `native/src/rider/launch_plan.{hpp,cpp}` через `detail::launch_teeth_unchecked` в `native/src/rider/launch_config.{hpp,cpp}` (без повторной валидации таблицы на каждом тике).
  - В `RiderCommands` (`native/src/rider/intent.{hpp,cpp}`) добавить `std::optional<int> stationary_rear_teeth = std::nullopt;` с проверками `validate()` и заполнением в `commands_of(..., const RiderSenses *delayed_senses)` без аллокаций на тике при `phase == LaunchPhase::hold`, `delayed_senses != nullptr`, `grade_ahead > 0.` и `rear_teeth < launch_teeth(grade_ahead)`.
  - В `RiderShift` и `CadenceShifter::update` (`native/src/drivetrain/shifting.{hpp,cpp}`) добавить `std::optional<int> stationary_target_teeth = std::nullopt;` с проверками, идемпотентностью при `rear_teeth >= target`, ожиданием конца активного среза (`cut_remaining_s > 1e-12 -> deferred`) и прямым переключением на звезду кассеты без отсечения по `landing`/`slip`.
  - Подключить `stationary_target_teeth` в `PhysicalStep::apply_forces_body` (`native/src/runtime/step.cpp`).
  - Проверено набором тестов `cadence_shifter|shifting|rider_brake_channel|launch_plan|launch_intent|rider_senses|drivetrain` (9/9 пройдены).

- [x] **Задача 3. Приёмочный тест рестарта после остановки `test_normal_rider_restart_after_a_stop`.** (`9cbc13ff`, `b3b9fefa`)
  - Добавить `RestartMetrics` и `ride_restart()` в `native/tests/reference/launch_ride_metrics.hpp` с общим помощником извлечения оконных метрик (трек 200 м постоянного уклона, длительность 25 с, плавный нажим обоих тормозов 0 → 0.5 за `10.0..10.5` с, удержание до `12.5` с, затем отпускание, окно оценки `[t_second_release, t_second_handover]`).
  - Добавить `test_normal_rider_restart_after_a_stop()` на 5 % (ожидаемая звезда 33T) и 10 % (39T) в `native/tests/reference/test_rider_launch_rides_normal.cpp`.
  - Замеры (`viewer_physics_welded.toml`, асфальт): 5 % — `12 → 33T`, откат `-0.12 мм`, `0` отрывов/пробуксовок, пик `6.790 м/с` (`PASS`); 10 % — `12 → 39T`, откат `+0.00 мм`, `0` отрывов/пробуксовок, пик `6.296 м/с` (`PASS`).

- [x] **Задача 4. Финальная проверка этапа.**
  - `bash tools/check_change.sh main`: выход `0` за `397 с` (`194/194` тестов CTest, `346/346` единиц `cppcheck` и `frontends` без замечаний, `clang-tidy` без замечаний).
  - Медленные тесты `rider_launch_rides_(normal|careful|aggressive)`: `3/3` пройдены за `361.95 с` (`normal` `128.25 с`, `careful` `115.37 с`, `aggressive` `118.27 с`).
  - Коммиты на ветке `queue-2026-11-15`: `16b933a9` (план/решения), `4491661a` (BalanceMonitor), `1e238fa4` (переключение на месте), `9cbc13ff` (тесты рестарта), `b3b9fefa` (правки ревью), `e0f999f4` (условие удержания/форы в `step.cpp`).
