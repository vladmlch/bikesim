# Нативный стек замен: чем заменить scipy/numpy/matplotlib под macOS arm64

Type: research
Status: resolved
Blocked by: —

## Question

Чем заменяется каждая Python-зависимость, критичная для рантайм-пути, в нативной сборке под macOS arm64? Для каждой: кандидаты, зрелость, лицензия, сборка под Apple Silicon, fit под конкретную задачу этого проекта.

- `scipy.optimize` SLSQP → QP-решатель для allocation-задачи (OSQP, qpOASES, ProxQP, Clarabel, свой active-set): размер задачи, warm-start, плотность — что подходит под ~1 QP на тик пост-P.
- `numpy` → Eigen / xtensor / Accelerate.framework / свои массивы.
- `matplotlib`/`plotly` (viz) → implot / matplotlib-cpp / запись данных + внешний просмотр / Python-viz поверх записанного — что честно в каждой форме.
- `rich`/CLI → fmt / CLI11 / просто stdout.
- `glfw` → уже нативный; `mujoco` → тот же C API (`mujoco.h`).
- pytest-эквивалент для native-части — не оценивать как стоимость, но назвать вариант (Catch2/GoogleTest) для полноты стека.

## Context pointers

- Зависимости: `pyproject.toml` (numpy, scipy 1.17 pin, mujoco, matplotlib, plotly, rich, glfw).
- QP-задача: `src/bike_sim/physics/rider_allocation.py` (и вызовы из `sim/ride/rider_control.py`) — размерность, структура ограничений.
- Формы и платформа: `.scratch/cpp-port/map.md` (Notes: macOS arm64, extension+standalone в скоупе).

## Constraints

- Стоимость миграции не оценивать; только техническая пригодность и что теряется/приобретается против Python-стека.

## Answer

### 0. Что реально стоит на рантайм-пути (измерено по коду)

**Задача оптимизации** (`physics/rider_allocation.py:allocate_effort`, вызовы из `sim/ride/rider_control.py:1331` и `sim/ride/rider_response_allocation.py:241`):

- Цель: `½·‖x−t‖²` в безразмерных координатах — **чистая проекция, гессиан = I** (строго выпуклая, диагональная). Никакой нелинейной цели нет.
- Переменные: `z=[qddot_r, τ, f_attach, p, (excitation)]`. В референс-конфиге `viewer_physics_welded` (spindle): `n_z=40` (`n_r=12`, `n_t=9`, `n_f=10`, `n_p=9`; nv=26 всего в модели, замерено). В coupled-режиме (`allocate_response`): `n_z≈19–29` (+1 crank-task, +9 excitation при `activation_tau_s>0`). Максимум ~60.
- Ограничения: `Aeq` 12 строк (inverse dynamics) или 1 (crank task); `G` ~30–70 **плотных** строк (closures ±-пары, конусы трения, эпиграф мощности, доменные строки режима); box-bounds на всех переменных.
- Два нелинейных extra-ограничения: (а) круговой лимит хвата `‖F·x‖₂≤limit` — это **SOC второго порядка**, до 2 штук на solve (ветки pull/press перебираются снаружи); (б) `ActivationLaw.constraint` — векторное **негладкое** piecewise-affine равенство `x_τ·s − delivered(x_u·s)=0` (clip + проекция по мощности), активно только при `activation_tau_s>0` (дефолт `0.` — выключено). Код после solve делает `activation.project()` и перемеряет резидуал — равенство фактически реализуется проекцией.
- Частота: 1 QP на 5 мс тик пост-P (сегодня ~4 SLSQP/шаг — ветвь хвата × поиск режима; `_last_branch`/`_last_solution` — честный warm-start механизм, `x0` уже принимается). Fast-path: проекция цели на box часто сразу feasible → QP вообще не запускается.
- Сертификация: решатель не доверен — `feasible` выставляется по измеренному резидуалу `≤1e-7` по всем семействам; нужна высокая точность, не «почти сошёлся».
- **LP-префильтр**: `rider_response_allocation.linear_region_infeasible` вызывает `scipy.linprog(method='highs')` на том же размере — до нескольких LP на tick в coupled-режиме.

**numpy** на горячем пути — только мелкая плотная линейная алгебра: `nv=26` (`mj_fullM` 26×26, `np.linalg.solve` 26×(1+9+nefc≈22+9+22)), якобианы 3×26/6×26, сборка QP ~40×70, `lstsq/pinv` на (3..6)-рядных картах wrench, `einsum` на батчах per-step якобианов. Самое крупное регулярное — 512-станционные векторы distributed-tire и колонки рекордера (~100 каналов × шаги). Больших массивов, FFT, sparse — нет.

**Остальной scipy** (полный список `from scipy`): `nnls` — только в `fit_density` (калибровка, офлайн); `least_squares` — `sag_fit` (setup) и `equilibrium_refine` (раз в ~3 с, warm path); `brentq` — validation rigs (офлайн); `butter/filtfilt` — metrics (постобработка записи); `gaussian_filter1d` — сборка террейна (setup).

**viz**: matplotlib — только статические PNG через Agg, из записанных CSV/данных (`viz/ride_plots.py`, `leverage_plot`, `dyno_plot` и др.); plotly — один HTML-отчёт `ride.html`, строится **из записанного CSV** (`viz/ride_plots.py:445` `plot_physical_ride_html`), опциональная зависимость. Ничего live-рендерящегося в viz нет; HUD — rich-текст в консоль.

### 1. `scipy.optimize` → QP/LP-решатель

Сначала факт: **общность SLSQP здесь не нужна почти нигде**. Цель — квадрат с единичным гессианом, всё кроме двух пунктов — линейные равенства/неравенства/bounds; круговой лимит хвата — SOC, а не общий nonlinear; единственный по-настоящему «не-QP» кусок — опциональное негладкое равенство активации (по дефолту выключено, и код его и так добивает проекцией). Смена solver API на «QP+SOC» покрывает весь продуктивный путь.

| Кандидат | Лицензия | Метод | arm64 | Fit |
|---|---|---|---|---|
| **ProxQP** (proxsuite, Inria) | BSD-2 | primal-dual augm.-Lagrangian, dense+sparse бэкенды, Eigen-based | Чистый C++17/CMake, arm64 — основная платформа авторов (роботика) | Лучший прямой fit: dense QP n≤60, нативные eq/ineq/box, warm start (`warm_start(x,y,z)`), точность до 1e-9+, задуман под «1 QP на control tick». Теряет: нет SOC — круг хвата закрывать внешней линеаризацией (SQP-итерации, как делает сам SLSQP) или Clarabel на pull-ветках. |
| **HiGHS** | MIT | simplex/interior-point LP + convex QP | Готовые macos-arm64 бинари и brew-бутыль | Ровно тот же движок, что `linprog(method='highs')` — префильтр портится 1:1 с честными сертификатами infeasibility. Бонус: NNLS из `fit_density` = малый QP → решается им же. Как основной QP-решатель уступает ProxQP по warm-start удобству, но запасной вариант уже в стеке. |
| **Clarabel** (Rust ядро + C API `clarabel-c`) | Apache-2.0 | interior point, QP+**SOCP**+exp/power cones | Rust → статическая lib под darwin-arm64, собирается cargo/cmake | Единственный из списка, кто берёт круговой лимит **как есть** (SOC), плюс точность IPM (~1e-8) для сертификации. Теряет: warm start (IPM — холодный старт каждый solve; при n≈40 это десятки мкс, терпимо). Подходит как «точный» решатель для ветвей с кругом. |
| **NLopt SLSQP** | slsqp.c — **BSD** (весь NLopt — LGPL-2.1 только из-за luksan; сборка без luksan = MIT) | тот же SQP Kraft | Чистый C, arm64 норм | «Тот же алгоритм, что в scipy»: переносит семантику nonlinear constraints (активация, круг) и цикл рестартов как есть, без реформулировки. Теряет: тот же класс точности, что у scipy (SQP на линеаризациях), и это мёртвый код 1988 года — как конечная точка, а не мост, спорно. |
| qpOASES | **LGPL-2.1** | dense online active-set | self-contained C++, make osx.mk; arm64 собирается | По методологии — идеален (warm start — его родной режим, n≤60), но LGPL в стеке — минус против BSD/MIT-соседей. |
| OSQP | Apache-2.0 | ADMM первый порядок | чистый C, везде | Ориентирован на крупные разреженные QP; на n=40 выигрыша нет, а до резидуала 1e-7 надо гнать polish+строгие eps — слабый fit по точности. |
| qpDUNES | BSD | multi-stage structured | — | Не fit: блочно-ступенчатой OCP-структуры у задачи нет. |
| Свой dense active-set (Goldfarb–Idnani/Lemke) | — | active-set | — | Правдоподобно (n≤60, гессиан=I — самый простой случай; круг — внешняя линеаризация). Полный контроль warm-start/лицензии, но это повторная реализация проверенного кода; разумно только если ни один готовый не устроит по API. |

**Что приобретается**: решения за ~10–50 мкс на тик вместо ~4 мс scipy-SLSQP (весь путь — плотная алгебра ≤60-мерная, warm start реально используется qpOASES/ProxQP), честные сертификаты LP-infeasibility от HiGHS вместо `status==2` через scipy-обёртку, никакой marshalling-стоимости `LinearConstraint`→dict→f2c.
**Что теряется**: единый `minimize`-интерфейс с `NonlinearConstraint` — равенство активации (при `activation_tau_s>0`) надо либо решать NLopt-SLSQP/своим SQP-слоем, либо реформулировать на уровне модели (решать в excitation-пространстве и применять `delivered()` как проекцию — код уже делает ровно это в `activation.project()`); круг хвата как quadratic constraint напрямую берут только Clarabel/NLopt. `Bounds/LinearConstraint/NonlinearConstraint` — чистые структуры, переносятся тривиально.

### 2. `numpy` → плотная мелкая линейная алгебра

- **Eigen** (MPL2, header-only) — рекомендованная база: `MatrixXd`/fixed-size `Matrix<double,N,1>` для qfrc/jac-блоков, `PartialPivLU`/`LDLT` для `linalg.solve` 26×26, `CompleteOrthogonalDecomposition` для `pinv`/`lstsq`, `.array()`-elementwise для станционных векторов шины. arm64/NEON зрелый; при желании `-framework Accelerate` как BLAS/LAPACK-бэкенд (`EIGEN_USE_BLAS`). Теряется против numpy: broadcasting-синтаксис `np.ix_`/`column_stack`/`vstack` переписывается блок-нотацией Eigen — механически, но не 1:1.
- **xtensor** (BSD-3, header-only, numpy-семантика с broadcasting) — облегчает буквальный перенос идиом `np.*`; минус — линейная алгебра слабее Eigen и ещё одна зависимость (xtl, xsimd). Реалистично как слой совместимости, не как база.
- **Accelerate.framework / vDSP** — vecLib BLAS/LAPACK + vDSP уже в системе, ноль зависимостей; но голый Fortran-ABI LAPACK без контейнерной эргономики. Роль — бэкенд под Eigen, не самостоятельный API.
- **Свои массивы/std::vector** — достаточно только для скалярных каналов рекордера/телеметрии; для `solve/lstsq/pinv` всё равно нужна библиотека.

Собирается под arm64 тривиально во всех вариантах. Gained против numpy: нет boxing/`np.asarray`-копий на каждый вызов, fixed-size типы на стеке, SIMD через NEON.

### 3. viz: matplotlib/plotly → по форме артефакта

- **Write-only + существующий Python-viz поверх записанного** (рекомендовано): рекордер уже пишет CSV → все PNG (`ride_plots`, `leverage`, `dyno`) и `ride.html` (plotly) строятся из CSV пост-факто. Нативная сторона пишет тот же CSV (`std::ofstream`+`fmt` — и сам `write_csv` 5.4 с профиля исчезает), Python-viz остаётся как есть. Потерь нет — это уже фактическая архитектура.
- **implot + Dear ImGui** (MIT): живые графики внутри нативного viewer, если он понадобится; PNG-экспорта нет — статические отчёты всё равно уходят в вариант выше.
- **matplotlib-cpp** (MIT): это мост в CPython — тащит Python+matplotlib в «нативную» сборку; осмыслен только в гибридной форме, для standalone противоречит цели.
- plotly-HTML в нативной форме: воспроизводимо записью JSON-каналов + статический HTML-шаблон с plotly.js — тот же выход без Python-рантайма, по желанию.

### 4. `rich`/CLI и остальное

- **rich** (HUD, стилизованный вывод): `fmt` (MIT) для форматирования + ANSI-коды или **ftxui** (MIT), если HUD хочется полноценным TUI; в native-viewer HUD естественнее рисовать оверлеем ImGui. Теряется soft_wrap/markup-эргономика rich — функционально эквивалентно.
- **argparse/CLI**: **CLI11** (BSD-3) — feature-parity (flags, подкоманды — есть 6 entry points); альтернатива cxxopts (MIT). Или stdout+таблицы вручную — CLI поверхность проекта узкая.
- **glfw**: уже нативный C (zlib-license), пришёл транзитивно через `mujoco.viewer` — в нативной сборке линкуется напрямую, включая готовый render-код MuJoCo (`mjvScene`/`mjrContext`, пример `simulate`). Замены не требует.
- **mujoco**: тот же C API `mujoco.h` — все используемые вызовы (`mj_step`, `mj_jac`, `mj_fullM`, `mj_jacDot`, `mj_mulJacVec`, `mj_isSparse`, поля `data.efc_*`, `model.*`) существуют дословно; Python-биндинги — тонкая обёртка. Gained: прямое владение `mjModel/mjData`, отсутствие per-call numpy↔ctypes маршалинга, потоки для batch-rollout без GIL. Lost: только удобство `model.joint(name)`-стиля доступа по именам (есть `mj_name2id`).
- **pytest-эквивалент**: **Catch2** (BSL-1.0, `TEST_CASE`/`SECTION` ближе всего к pytest-стилю с fixtures-via-sections) или **GoogleTest** (BSD-3, индустриальный стандарт, richer matchers). Оба — CMake-нативные, arm64 из коробки. doctest (MIT) — минималистичный третий вариант.

### 5. Небольшой scipy-хвост (не на тике)

- `least_squares` (`sag_fit`, `equilibrium_refine` — раз в ~3 с): Eigen unsupported `LevenbergMarquardt` (модуль `unsupported/Eigen/NonLinearOptimization`) или Ceres (BSD, тяжёлая зависимость) или ~150 строк trust-region LM; в гибридной форме остаётся Python-кодом.
- `nnls` (`fit_density`, калибровка): NNLS = малый QP `min‖Ax−b‖², x≥0` → тот же ProxQP/HiGHS; или Lawson–Hanson ~100 строк.
- `brentq` (validation rigs): Brent-сходилка ~60 строк.
- `butter/filtfilt` (metrics, постобработка): IIR-фильтр — ~200 строк (bilinear butter + filtfilt-padding) или DSP-либа (KFR MIT, тяжеловата); обычно остаётся Python-постобработкой вместе с viz.
- `gaussian_filter1d` (terrain setup): FIR-свёртка ~40 строк.

### 6. Рекомендованный shortlist

QP-ядро: **ProxQP** (основной dense QP, BSD-2) + **HiGHS** (LP-префильтр 1:1 к `linprog('highs')` + NNLS-как-QP, MIT) + **Clarabel** на pull-ветках хвата, если SOC хочется точным без линеаризации (Apache-2.0; иначе — SQP-цикл поверх ProxQP). Алгебра: **Eigen** (MPL2). Форматирование/CLI: **fmt** + **CLI11**. Viz: **write-only CSV + существующий Python-viz** (полная форма) или **implot** в native viewer (живые кривые). Тесты: **Catch2** (или GoogleTest). mujoco/glfw: те же нативные либы, `mujoco.h` + `simulate`-подобный viewer. Весь shortlist — permissive-лицензии, без LGPL (qpOASES/полный NLopt исключены намеренно).

**Единственный пробел**: негладкое векторное равенство активации (`activation_tau_s>0`) не является QP/SOC — варианты: (а) реформулировать на уровне модели — решать по excitation и применять `delivered()` проекцией, что код уже делает в `activation.project()` после solve; (б) держать SQP-слой (NLopt-SLSQP, BSD-ядро) только для этого режима. Остальной Python-стек имеет прямые нативные ответы без потерь по функции.

## Comments
