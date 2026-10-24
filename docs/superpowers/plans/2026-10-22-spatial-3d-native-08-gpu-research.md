# Spatial 3D Native — 08. Необязательное исследование GPU (W-фаза: W55)

**Source handoff 2026-10-24:** W55 — `source_written`, execution — `not_run`.
Standalone helper/test и memo описаны в
[отчёте поставки](../audits/2026-10-24-spatial-w50-w55-source.md).
SDK, GPU и benchmark не запускались; все решения остаются pending V11.

> **Для исполнителей:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development (рекомендуется) или superpowers:executing-plans. W55 — **one-shot write** записки-скелета, decision helper и его теста. Запрещены configure/build/compile/ctest/запуск/установка GPU SDK. Измерения и решение — `09-verification.md` V11 (optional). Незавершённость W55/V11 не блокирует V10.

**Goal:** Зафиксировать вопрос, критерии и инструмент решения о GPU до появления измерений; не выбирать backend и не менять CPU-контракт.

**Architecture:** CPU-симулятор — источник истины. Эксперименты вне `native/` и вне продуктового CMake (`tools/spatial_gpu/`). Положительное решение требует отдельной спецификации и плана квалификации.

**Tech Stack:** C++23 helper + `bt` тест; GPU SDK — только в V11 при наличии host.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`; master; contracts C8; измеренная нагрузка V09 (S50) и решение V09 (S51/S52).

## Общие ограничения

- В W ничего не измеряется; числа появляются только в V11 после реального измерения.
- Не менять dependency lock, backend по умолчанию, dt, закон шины, точность, fixtures.
- Не создавать `docs/adr/0003-gpu-offload.md`: ADR-0003 занят. Свободный номер (0005 или следующий после W02) используется только при принятом архитектурном решении в V11.
- Metal-эксперимент не доказывает Linux; CUDA не обещается.
- `CPU8≈CPU1/6`, `100 ns/ray` — не измерения.

## Исправления старого P8

Старый P8 начинался с «day 1» без известных размеров/времени, предлагал NumPy/MLX/Python и занимал ADR-0003. Новый порядок: измерить serial (V09), при необходимости CPU-пул, затем оценить перенос. Критерий H1 точен: `gpu_end_to_end_s <= cpu_best_qualified_s/2` **и** ускорение заезда ≥10% **и** прежние точность/детерминизм. Dispatch, sync и readback для следующего последовательного шага MuJoCo входят в измерение; локальный shader benchmark без readback не отвечает на вопрос.

---

## W55 — Записка-скелет, decision helper и тест (← S55)

**Dependencies:** нет для W (читает W50/W51 как описание будущих измерений). **Create:** `docs/superpowers/audits/2026-10-22-spatial-3d-gpu-feasibility.md`, `tools/spatial_gpu/README.md`, `tools/spatial_gpu/decision.hpp`, `tools/spatial_gpu/decision_test.cpp`, `tools/spatial_gpu/CMakeLists.txt` (standalone, **не** включается в root graph; `option(BIKE_SPATIAL_GPU_TOOLS OFF)` в root только как ссылка на этот каталог — без include по умолчанию).

- [ ] `decision.hpp`:

```cpp
#pragma once
#include <cmath>
#include <stdexcept>
namespace spatial_gpu {
inline bool latency_goal_met(double cpu_s, double gpu_s, bool accuracy_passed, bool deterministic) {
    if (!std::isfinite(cpu_s) || !std::isfinite(gpu_s) || cpu_s <= 0. || gpu_s <= 0.)
        throw std::invalid_argument("positive finite measured times required");
    return accuracy_passed && deterministic && gpu_s <= cpu_s / 2.;
}
inline bool batch_goal_met(double cpu_episodes_per_s, double gpu_episodes_per_s, bool same_physics) {
    if (!std::isfinite(cpu_episodes_per_s) || !std::isfinite(gpu_episodes_per_s) || cpu_episodes_per_s <= 0. || gpu_episodes_per_s <= 0.)
        throw std::invalid_argument("positive finite throughput required");
    return same_physics && gpu_episodes_per_s >= 20. * cpu_episodes_per_s;
}
inline bool offline_goal_met(double cpu_task_s, double gpu_task_s, bool exact_checks_passed) {
    if (!std::isfinite(cpu_task_s) || !std::isfinite(gpu_task_s) || cpu_task_s <= 0. || gpu_task_s <= 0.)
        throw std::invalid_argument("positive finite task times required");
    return exact_checks_passed && gpu_task_s * 3. <= cpu_task_s;
}
}
```

- [ ] `decision_test.cpp` (включает `native/tests/support/migration_test.hpp`, `bt::run`): `!latency_goal_met(10,2,false,true)`; `latency_goal_met(10,5,true,true)`; `!latency_goal_met(10,5.01,true,true)`; `bt::invalid` на `(0,1,true,true)`; `batch_goal_met(1,20,true)`, `!batch_goal_met(1,19.9,true)`, `!batch_goal_met(1,100,false)`; `offline_goal_met(3,1,true)`, `!offline_goal_met(3,1.01,true)`.
- [ ] `tools/spatial_gpu/CMakeLists.txt`: `add_executable(spatial_gpu_decision_test decision_test.cpp ../../native/tests/support/migration_test.cpp)`; `add_test(NAME spatial.gpu_decision COMMAND spatial_gpu_decision_test)`; не линкуется с продуктом.
- [ ] `gpu-feasibility.md` — минимальная форма записки с пустыми полями: (1) вопрос и объём: интерактивная поездка / CPU batch / автономный инструмент; (2) CPU identity и размеры нагрузки — `pending V09`; (3) GPU host/driver/SDK — `pending V11`; (4) источники матрицы поддержки (version/date/URL) — `pending`; (5) пороги H1/H2/H3 (таблица ниже) — записаны; (6) измерения setup/dispatch/kernel/transfer/sync/полного запуска — `pending`; (7) сравнение с serial и измеренным CPU-пулом — `pending`; (8) решение каждой гипотезы — `pending-measurement`; (9) изолированность прототипа — `confirmed: tools/spatial_gpu only`. Бюджет: до двух инженерных дней на evidence в V11; без GPU host → `unsupported-host`.
- [ ] `README.md`: назначение каталога, запрет на зависимости продукта, как V11 запускает decision test и где класть прототип выбранного API.
- [ ] Коммит `docs(spatial): W55 GPU feasibility skeleton and decision helper`, `Verification: pending V11 (optional)`.

```cpp
bt::require(!spatial_gpu::latency_goal_met(10.,2.,false,true), "speed is not accuracy");
bt::require(spatial_gpu::latency_goal_met(10.,5.,true,true), "inclusive twofold goal");
```

## Гипотезы и заранее заданные критерии (для V11)

| Гипотеза | Измеряется | Go-for-separate-design только если | No-go когда |
|---|---|---|---|
| H1: перенос rays/элементов одного шага | dispatch + upload + kernel + суммирование + readback + sync; весь tick | GPU-участок ≤0.5 лучшего квалифицированного CPU-участка **и** ускорение заезда ≥10%; прежние точность/детерминизм | хотя бы одно условие не выполнено |
| H2: множество независимых эпизодов | валидные эпизоды/с при одинаковой физике | ≥20× лучшего измеренного CPU process batch и отдельно принятый объём реализации | нет физической возможности / <20× / бюджет превышен |
| H3: автономная генерация/калибровка/анализ | полная задача input→artifact | ≥3× и прежние точные проверки | узкое место CPU/I/O / <3× / зависимость не оправдана |

## Что этот документ не обещает

Не выбирает GPU backend, не переносит MuJoCo, не заменяет C++ ядро Python, не обещает скорость. Переход на 3D завершается через V10 независимо от V11.
