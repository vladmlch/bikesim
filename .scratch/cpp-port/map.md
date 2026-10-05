# Map: Переписать bike-sim на C/C++ — стоит ли и в какой форме

Labels: wayfinder:map

## Destination

Decision doc + рекомендация: технический ledger плюсов/минусов переписки `bike-sim` на C/C++ — по формам (stay-Python-post-P / гибрид / полный порт) и, в той же оценке, по движку (MuJoCo vs альтернативы) — сходящийся к рекомендации и эскизу целевой архитектуры (границы модулей, стек замен зависимостей, форма артефакта). Карта закрыта, когда этот документ можно отдать и решение принято.

## Notes

- Домен: симулятор велосипеда с подвеской и райдером на MuJoCo, ~41K LOC Python (`src/bike_sim`): MJCF-генератор (`mujoco/`, `geometry/`, `kinematics/`), физмодели (`physics/` — tyre, motor, battery, allocation), рантайм (`sim/`, 21K LOC — per-step `mj_step` + контроллер райдера + запись), `terrain/`, `viz/` (matplotlib/plotly), `cli/`.
- Главный и единственный взвешенный драйвер: **скорость симуляции** (real-time viewer + throughput batch-исследований). Остальные плюсы/минусы каталогизируются, но решение взвешивается по скорости.
- Baseline — два столбца: текущий код и пост-P-plan (P-план считается достигшим целей как проекция: ≤0.5мс/шаг, 1 QP на тик). См. `docs/superpowers/plans/2026-10-03-v2-01-performance.md`.
- Порога скорости нет: карта измеряет достижимые диапазоны real-time фактора для каждой формы; пороговое решение принимается при синтезе.
- Consumers нативной версии: live ride-viewer и batch research environment. MJCF-билдинг/экспорт может оставаться Python в любой форме.
- Целевая платформа нативной сборки: **macOS arm64 (Apple Silicon M4+)**.
- Формы артефакта в скоупе: Python-extension и standalone.
- Движок в скоупе оценки: MuJoCo и альтернативы.
- Известный профиль (P-план, 2026-10-03, 2400 шагов / 56с стены): `rider_control.compute` 44.5с, из них `allocate_effort` 39.6с (~9604 вызовов scipy SLSQP); `physical_recorder.write_csv` 5.4с; `settle_welds` 3.1с.
- Свежий замер на HEAD (см. «Анатомию времени шага»): 1.8мс/шаг, RTF 0.28 — P-цель ≤0.5мс в текущем коде не достигнута; с commit `835a48c` дефолтный профиль (`viewer_physics_welded`, spindle) вообще не вызывает QP (closed-form контроллер), SLSQP живёт только в legacy-ветках weld/flat-pedal. Доминируют Python-телеметрия (~55%) и force-writers (~30%).
- Стоимость/трудоёмкость переписки, поддержки и около этого — **не оцениваем** (явное ограничение заказчика). См. Out of scope.
- Словарь форм: «stay» = Python + P-plan; «гибрид» = горячий путь нативный, оркестрация/виз Python; «полный» = весь рантайм-путь C++; «bespoke» = полный + собственный physics pipeline (nv=26, скалярно-плоскостная структура — единственный путь существенно ниже ~50мкс engine-слайса, из «Альтернатив MuJoCo»).
- Tracker: local markdown (`.scratch/cpp-port/`); Type/Status/Blocked by — по `.claude/skills/setup-matt-pocock-skills/issue-tracker-local.md`.

## Decisions so far

<!-- one line per resolved ticket: gist + link -->

- [Нативный стек замен: чем заменить scipy/numpy/matplotlib под macOS arm64](issues/02-native-replacement-stack.md): QP → ProxQP + HiGHS (Clarabel для SOC-круга хвата), numpy → Eigen, viz → write-only CSV + существующий Python-viz (implot для живых кривых в native viewer), CLI → fmt/CLI11; mujoco/glfw — те же нативные либы. Единственный пробел — негладкое равенство активации (по дефолту выключено).
- [Анатомия времени шага: что реально устранит нативный код](issues/01-hotpath-anatomy.md): измерено 1.8мс/шаг (RTF 0.28); `mj_step` лишь ~2%, весь движок ~10% — устранимая нативным кодом доля ≈85–90% (доминирует Python-телеметрия ~55% и force-writers ~30%); потолок RTF ≈2–3.5× против ~1× у stay-пост-P, абсолютный пол ~5–6× при срезании контракта телеметрии.
- [Альтернативы MuJoCo: другой движок или своя динамика](issues/03-mujoco-alternatives.md): библиотечная замена движка ничего не покупает — ни один движок не даёт нужной интроспекции (named-equality multipliers, mutable tendon bounds, re-entrant `mj_forward`); жизнеспособны только stay-MuJoCo и собственный nv=26 constraint-слой (формы «bespoke»); Drake/Bullet доминируемы, Newton нежизнеспособен на macOS arm64 (GPU-first).
- [Планка корректности: что считается «та же симуляция»](issues/05-correctness-bar.md): слоистая планка — per-call ~1e-12, детерминированная эквивалентность эпизода до горизонта расхождения (измеряется perturbation-тестом), дальше статистика метрик; Python — оракул через golden-артефакты (сырой state + все каналы рекордера); bespoke — только метрики/статистика, отсутствие траекторной проверки = зафиксированный минус в ledger.
- [Граница порта и форма артефакта: что в native, кто владеет циклом](issues/06-boundary-and-seam.md): граница по такту — весь per-step/per-tick путь в native (цикл, писатели сил, контроллер, телеметрия, рекордер, intent-программы), setup/эпизод-оркестрация в Python; артефакт = нативное ядро-библиотека + nanobind-extension + standalone CLI; шов на intent-тике 10мс (~100–200 кроссингов/с); mjModel/mjData — собственность ядра, Python — zero-copy views + командная очередь, viewer через replica-mjData; стек C++17/CMake/clang/Eigen/nanobind, гибрид→полный — добавление драйвера, не новый порт.
- [C++ microbenchmark: измеренный потолок mj_step на M4](issues/04-native-microbench.md): измерено — `mj_step` ~32µs одинаков в обоих языках; native glue почти бесплатен (heavy 93µs ≈ glue), движок ~65% нативного шага; экстраполяция на реальный объём ≈200–400мкс/шаг → RTF ~1.3–2.5× — подтверждает потолок 2–3.5× измерением; вердикт человека: лучше ожиданий, кейс порта усилен. Прототип: `tools/proto_native_bench/`, ветка `proto/native-microbench`.
- [Синтез ledger + рекомендация](issues/07-ledger-synthesis.md): **Go — нативное ядро C++17 на MuJoCo, оба фронтенда** (extension + standalone); bespoke припаркован с триггером (RTF после порта недостаточен ∧ движок доминирует); decision doc = `docs/adr/0001-native-port-mujoco-core.md`. Маршрут закрыт.

## Not yet specified

_(пусто — весь туман разрешён; маршрут закрыт на «Синтезе ledger»: destination artifact = `docs/adr/0001-native-port-mujoco-core.md`)_

## Out of scope

- Оценка стоимости, трудоёмкости, сроков переписки и последующей поддержки — явное ограничение заказчика.
- Миграционный план, sequencing, поэтапность перехода, риски переходного периода.
- Переписка тестовой инфраструктуры и CI — вопрос владения, не технических плюсов/минусов.
- Размер деплой-бандла как драйвер — не выбран заказчиком; фиксируется в ledger лишь как второстепенный факт, без веса в решении.
