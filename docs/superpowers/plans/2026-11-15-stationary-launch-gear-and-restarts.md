# Переключение вниз на стартовую передачу на месте и рестарты — план

**Цель.** Устранить срыв рестарта после полной остановки посреди поездки (пункт 16 «Решено — не сделано», П9) по решениям пользователя от 2026-11-15 (сессия `1303e344`): восстановить 3 с форы `BalanceMonitor` после каждой полной остановки на тормозах, разрешить прямой выбор стартовой передачи на месте в `LaunchPhase::hold` перед подъёмом без изменения схем `state.json` и каналов записи, и добавить приёмочный тест рестарта после остановки на 5 % и 10 %.

**Решения пользователя (2026-11-15, сессия `1303e344`).**
- **Перезапуск форы `BalanceMonitor`:** полная остановка на тормозах (`motor_braking && |speed_mps| <= standstill_speed_mps`) сбрасывает `started_s`, чтобы после отпускания тормозов снова давались полные `balance_grace_s = 3.0` с (кратковременное притормаживание на ходу по-прежнему только ставит счётчик простоя на паузу).
- **Выбор стартовой передачи на месте в `LaunchPhase::hold`:** вместо моделирования удержания велосипеда руками и прокручивания педалей/колеса на месте запрет переключения стоя для остановки посреди поездки перед подъёмом заменён: при `LaunchPhase::hold`, `grade_ahead > 0` и `rear_teeth < prepare_launch(config, grade_ahead).rear_teeth` симуляция переключается на `prepare_launch(config, grade_ahead).rear_teeth` прямо на месте (прыжком через несколько звёзд).
- **Совместимость схем:** не добавлять поля в `LaunchPlan::State`, `RiderIntent::State`, `BalanceMonitor::State`, `ShiftingSnapshot`, wire-канал `rider_commands`, `BalanceMonitor::observation()` и `native/src/recording/catalog_data.inc`. Все существующие `state.json` и `.bin` эталоны, а также холодные старты с `t = 0` остаются без изменений.
- **Приёмочные тесты рестарта (`test_normal_rider_restart_after_a_stop`):** на уклонах 5 % и 10 % торможение выполняется плавно (линейный вход на `0.5` и удержание до полной остановки), а чистота рестарта оценивается от момента отпускания тормоза (`LaunchPhase::release` .. `handover`). Отскок вилки при 100 % торможении вынесен в отдельный пункт очереди.

## Задачи

- [ ] **Задача 1. `BalanceMonitor`: перезапуск 3 с форы после каждой полной остановки на тормозах.**
  - Обновить `BalanceMonitor::update(..., bool stopped = false)` в `native/src/runtime/control.{hpp,cpp}`: при `!riding && stopped` сбрасывать `started_s`, сохраняя `state().event` и прежнюю паузу при кратковременном притормаживании на ходу (`!riding && !stopped`).
  - В `PhysicalStep::balance_channel` (`native/src/runtime/step.cpp`) передавать `stopped = brakes.motor_braking && std::abs(speed_mps) <= config_.launch.standstill_speed_mps`.
  - Добавить модульный тест `test_full_stop_rearms_grace_after_brake_release` в `native/tests/reference/test_balance_monitor.cpp` и проверить `mise run test 'balance_monitor'`.

- [ ] **Задача 2. Выбор стартовой передачи на месте в `LaunchPhase::hold` (без изменения схем).**
  - Добавить `LaunchPlan::launch_teeth(double grade_ahead) const` в `native/src/rider/launch_plan.{hpp,cpp}`.
  - В `RiderCommands` (`native/src/rider/intent.{hpp,cpp}`) добавить `std::optional<int> stationary_rear_teeth = std::nullopt;` с проверками `validate()` и заполнением в `commands_of()` при `phase == LaunchPhase::hold`, `delayed.has_value()`, `grade_ahead > 0.` и `rear_teeth < launch_teeth(grade_ahead)`.
  - В `RiderShift` и `CadenceShifter::update` (`native/src/drivetrain/shifting.{hpp,cpp}`) добавить `std::optional<int> stationary_target_teeth = std::nullopt;` с проверками, идемпотентностью при `rear_teeth >= target`, ожиданием конца активного среза (`cut_remaining_s > 1e-12 -> deferred`) и прямым переключением на звезду кассеты без отсечения по `landing`/`slip`.
  - Подключить `stationary_target_teeth` в `PhysicalStep::apply_forces_body` (`native/src/runtime/step.cpp`).
  - Добавить модульные тесты для `CadenceShifter` и `RiderIntent` и проверить `mise run test 'shifting|rider_brake_channel|launch_plan|launch_intent|rider_senses|drivetrain'`.

- [ ] **Задача 3. Приёмочный тест рестарта после остановки `test_normal_rider_restart_after_a_stop`.**
  - Добавить `RestartMetrics` и `ride_restart()` в `native/tests/reference/launch_ride_metrics.hpp` с общим помощником извлечения оконных метрик (трек 200 м постоянного уклона, длительность 25 с, плавный нажим обоих тормозов 0 → 0.5 за `10.0..10.5` с, удержание до `12.5` с, затем отпускание, окно оценки `[t_second_release, t_second_handover]`).
  - Добавить `test_normal_rider_restart_after_a_stop()` на 5 % (ожидаемая звезда 33T) и 10 % (39T) в `native/tests/reference/test_rider_launch_rides_normal.cpp`.
  - Проверить `mise run test 'rider_launch_rides_normal|realistic_pedelec_acceptance'`.

- [ ] **Задача 4. Финальная проверка этапа.**
  - Выполнить `bash tools/check_change.sh main` и прогон медленных тестов `rider_launch_rides_*`.
  - Отметить выполненные флажки в этом плане.
