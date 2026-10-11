# Alpine scree climb: seeded trail generator and pinned preset — design

Дата: 2026-10-11. Ветка: `feat/alpine-scree-track`. Статус: дизайн согласован в чате, ждёт ревью файла.

## 1. Цель

Правдоподобная высокогорная MTB-тропа длиной 120 м (масштаб `rough_uphill_savage`) для
исследования сцепления, проскальзывания колёс и разгрузки/отрыва переднего колеса под
тягой: ритм крутых рамп и полок-шпилек, сыпуха (низкое трение + мелкая текстура), вросшие
валуны, небольшие каменные поля, водоотводные канавки на утоптанной земле.

Что строим:

- сидированный генератор `generate_alpine_track(AlpineGenSpec, seed)` → обычный `TrackSpec`;
- новое покрытие `scree` в реестре `SURFACES`;
- геометрические предохранители для одноточечной шины `compliant_2d`
  (`terrain/contact_safety.py`), включаемые параметром спека;
- пинованный пресет `alpine_scree` (фиксированный seed) для `bike-ride`, сценарий
  `--scenario alpine` для `bike-research`, замороженный `examples/research/alpine_scree.toml`;
- заготовку `examples/research/alpine_rocky_genspec.toml` под будущую многоточечную шину.

Чего не строим (вне скоупа, см. §9): повороты/серпантин как геометрию (трасса — 1D профиль),
деформируемую сыпуху (дорога жёсткая), импорт реального рельефа (GPX/DEM), многоточечную шину,
изменения в `native/`.

## 2. Решения (из интервью)

| Ветвь | Решение |
|---|---|
| Масштаб | 120 м фиксированно; lead-in 10 м и lead-out 15 м плоские, без препятствий и зон покрытия |
| Тип дороги | Высокогорная тропа: база `hardpack`, зоны `scree`, камни; без корней, ступеней, washboard, G-out |
| Уклон | Ритм тропы: циклы «рампа 15–30 м на 15–25 % → полка 4–8 м на 2–6 % [→ контр-уклон −3…−8 % на 3–6 м с p = 0.3]» заполняют тело; одна рампа — стенка 26–32 % |
| Сыпуха | Новое покрытие `scree` (μ_peak 0.45, μ_slide 0.38, жёсткость проскальзывания 4.5 — авторские, ниже `loose`); секция = `SurfaceSection('scree')` + `RoadRoughness` 10–20 мм / 0.15–0.20 м на том же интервале; смещение к рампам p = 0.7 |
| Камни | Валуны — гладкие `Bump` 5–15 см; каменные поля — `RockGarden` 3–6 см / 0.30–0.45 м; острые ступени `square_edge` только при `single_contact_safe = false` |
| Канавки | Только `BowlPothole` 8–15 см / 1.0–1.5 м; `SlopedPothole` исключён (всегда ломает одноточечную шину) |
| Фон | `RoadRoughness` 5–10 мм / 0.20–0.30 м на свободных участках тела; `wet` зоны с дефолтом 0 |
| Шина | Вариант D: предохранители — параметры спека (`single_contact_safe`), по умолчанию включены; многоточечная шина — отдельный план (`docs/superpowers/plans/2026-10-11-multi-contact-tire.md`) |
| Код | `terrain/alpine.py`, `terrain/placement.py` (общие хелперы из `generator.py`/`road.py`), `terrain/contact_safety.py`; старые генераторы байт-в-байт неизменны |
| Доступ | `bike-ride --track alpine_scree [--seed N]` (без `--length`); `bike-research --scenario alpine [--gen-spec …] --seed N` |
| Native | Без изменений: покрытие и зоны передаются значениями, имя нигде не сверяется |
| Тесты | `tests/reference/test_alpine_track.py`, профиль `quick` |
| Документация | Докстринги + заголовок TOML; устаревшие ссылки на RIDE.md §4.1/§11.2 не трогаем |

## 3. Факты о среде, на которые опирается дизайн

- Трасса — одномерный профиль `z = h(x)`; hfield из 2 рядов; боковое движение отсутствует.
- `GradeProfile` интерполирует уклон линейно между узлами `(station, grade)`, высота C1.
- `HeightFieldSpec.for_track` сам подбирает длину поля (120 м → 130 м, как у savage) и
  вертикальный конверт (тело ~95 м на ~15 % ≈ 13–15 м набора).
- `trackfile.py` сериализует и загружает все используемые типы: `bump`, `pothole/bowl`,
  `rock_garden`, `roughness`, `square_edge`, `surface_sections`, `grade_profile`.
- Нативная шина получает `surface_map` **значениями** (`{name, mu_peak, mu_slide,
  slip_stiffness_per_load, stribeck_speed_mps}` и `sections[]` с теми же словарями);
  список имён не проверяется ни в `bike_sim/native/schema.py`, ни в `native/src`.
- `multi_support` вычисляется чисто геометрически: `ProfileQuery.contact(центр, радиус,
  вершины)` — две значимые опоры (проникновение ≥ 0.1 мм и ≥ 5 % максимального) с нормалями
  > 20° врозь. Research-прогон в strict-режиме останавливается на первом флаге
  (`sim/research/validity.py`); `--diagnostic-model-limits` продолжает его с `valid=false`.
- Радиусы колёс в модели: 0.372 м перед, 0.352 м зад (`terrain/wheelpath.py`, совпадают с
  `geom_size` контактных геомов).
- `bike-ride --seed/--length` сейчас принимаются только для `road_*` через `ROAD_LEVEL_SPECS`.
- Терраин почти не покрыт тестами: из 111 файлов `tests/reference` — только
  `test_savage_track_surface.py`; `TerrainGenSpec` не тестируется.

## 4. Спайк: когда одноточечная шина ломается

Метод (throwaway, `/tmp/alpine_spike`, в репозиторий не входит): препятствие растеризуется на
5 мм на плоском основании; окружность радиуса 0.372/0.352 м катится по огибающей, центр
опущен на проникновение δ ∈ {4, 10, 20} мм (статика ≈ 500 Н / 130 кН/м ≈ 4 мм); production
`ProfileQuery` с порогами `TireBackendConfig()`; 6 seed на ячейку; критерий «чисто» — ноль флагов.

### 4.1 Результаты

| Элемент | Чисто | Грязно |
|---|---|---|
| `RockGarden(amp, corr)` | (0.03, ≥ 0.25), (0.05, ≥ 0.30), (0.07, ≥ 0.6), (0.10, ≥ 0.8) | (0.03, ≤ 0.20), (0.05, ≤ 0.25), (0.07, ≤ 0.45), (0.10, ≤ 0.45) |
| `RoadRoughness` сыпуха | (0.010, любая ≥ 0.10), (0.015–0.020, ≥ 0.15) | (0.015–0.020, ≤ 0.125) |
| `RoadRoughness` фон 5–10 мм / 0.20–0.30 | всё | — |
| `Bump(h, L)` | L ≥ π·√(2hR), R = 0.45: (0.05, 0.67), (0.10, 0.95), (0.15, 1.16) | 0.7×пол при h ≥ 0.10 |
| `BowlPothole(d, L)` | d ≤ L²/(2π²R): (0.07, 0.8), (0.15, 1.2), (0.20, 1.5) | (0.10, 0.8) |
| `SlopedPothole` (edge 0.1) | — | всегда (0.9–8.6 % отсчётов) |

### 4.2 Критерии, воспроизводящие результаты

1. **Гладкие формы** (raised cosine): радиус кривизны гребня/впадины ≥ `min_contact_radius_m`
   = 0.45 м. Формулы `L_min = π·√(2hR)` и `d_max = L²/(2π²R)` предсказали границы точно
   (0.07205 м при L = 0.8: 0.07 чисто, 0.10 грязно).
2. **Гранёные формы** (`RockGarden`/`RoadRoughness`, линейная интерполяция узлов): вогнутый
   перепад уклона между любыми двумя гранями, отстоящими ≤ `contact_window_m` = 0.25 м
   (хорда пятна контакта `2·√(2Rδ)` ≈ 0.24 м при δ = 20 мм), должен быть < 20°.
   Проверено на 46 ячейках прокатки (RockGarden 28, сыпуха 12, фон 6): при (0.25 м, 20°) —
   **ни одного пропуска** опасной геометрии и одно консервативное ложное срабатывание
   ((0.02, 0.15): критерий 21.4°, прокатка чистая). Критерий «только соседние грани» давал
   пропуски ((0.02, 0.125): излом 17.2° < 20°, но флаги есть), поэтому отвергнут.
   Выпуклые гребни не считаются: на гребне одна опора.
3. Вертикальные и линейные грани (`SquareEdge`, `Pothole`, `SlopedPothole`, `Drop`,
   `Kicker`, `TrapezoidBump`) — опасны всегда.

Дефолт `max_kink_deg = 17°` оставляет запас под порог 20°.

## 5. Дизайн

### 5.1 `AlpineGenSpec` (`src/bike_sim/terrain/alpine.py`)

Frozen dataclass в стиле `TerrainGenSpec`: диапазоны `(lo, hi)`, списки из TOML замораживаются
в кортежи, `from_dict` отклоняет неизвестные ключи, `to_dict`/`sha256` для описания трассы.
Единицы: метры, уклон dz/dx.

```
# геометрия
length_m = 120.0           lead_in_m = 10.0        lead_out_m = 15.0
transition_m = 2.0         # линейный переход уклона между отрезками (зазор между узлами)

# ритм уклона
ramp_len_m = [15, 30]      ramp_grade = [0.15, 0.25]
shelf_len_m = [4, 8]       shelf_grade = [0.02, 0.06]
wall_grade = [0.26, 0.32]  n_walls = 1              # 0 или 1
counter_len_m = [3, 6]     counter_grade = [-0.08, -0.03]   counter_prob = 0.3

# покрытие
base_surface = "hardpack"
n_scree = [2, 3]           scree_len_m = [6, 15]    scree_ramp_bias = 0.7
scree_amp_m = [0.010, 0.020]   scree_corr_m = [0.15, 0.20]
n_wet = [0, 0]             wet_len_m = [2, 5]
background_amp_m = [0.005, 0.010]   background_corr_m = [0.20, 0.30]

# камни и канавки
n_boulders = [2, 5]        boulder_height_m = [0.05, 0.15]   boulder_len_m = [0.6, 1.2]
n_rock_gardens = [1, 2]    rock_len_m = [4, 8]   rock_amp_m = [0.03, 0.06]   rock_corr_m = [0.30, 0.45]
n_water_bars = [1, 3]      water_bar_depth_m = [0.08, 0.15]  water_bar_len_m = [1.0, 1.5]
n_sharp_edges = [0, 0]     sharp_edge_height_m = [0.03, 0.08]

# одноточечная шина
single_contact_safe = true
min_contact_radius_m = 0.45    max_kink_deg = 17.0    contact_window_m = 0.25
```

Валидация (`ValueError`): все диапазоны `lo ≤ hi`, неотрицательные (кроме `counter_grade`:
`hi < 0`); счётчики — целые; `counter_prob`, `scree_ramp_bias` ∈ [0, 1]; `n_walls` ∈ {0, 1};
`wall_grade[0] ≥ ramp_grade[1]`; `base_surface ∈ SURFACES`;
`lead_in + lead_out + 2·transition + ramp_len_m[0] ≤ length_m`;
`single_contact_safe = true` при `n_sharp_edges[1] > 0` — ошибка (ступени всегда ломают модель).

### 5.2 Алгоритм `generate_alpine_track(spec, *, seed, name=None)`

Один `np.random.default_rng(seed)`, фиксированный порядок; позиции округляются до 1 см,
размеры до 0.1 мм (как `generator.py`), чтобы `dump_track` → `load_track` совпадал побитово.

1. **Ритм уклона.** Тело `[lead_in, length − lead_out]`, `limit = body_end − transition`,
   `cursor = lead_in + transition`. Цикл, пока `cursor < limit`: разыграть длину и уклон рампы;
   если `limit − cursor < ramp_len_m[0]` — последний отрезок тянется до `limit`, стоп; иначе
   рампа (усекается по `limit`), полка (усекается), с p = `counter_prob` контр-уклон
   (усекается); каждый отрезок сдвигает `cursor` на свою длину + `transition`. Если
   `n_walls = 1`, одна из рамп (равновероятно) получает уклон из `wall_grade`. Узлы:
   `(0,0), (lead_in,0)`, по `(s,g),(e,g)` на отрезок, `(body_end,0), (length,0)`.
   Генератор запоминает интервалы рамп/стенки.
2. **Каменные поля.** Для каждого (`n_rock_gardens`): длина, амплитуда, корреляция →
   `place(…, shrink=True)` в теле с клиренсом 1 м → `_faceted`: розыгрыш seed до прохождения
   критерия 4.2(2) (≤ 200 попыток, затем амплитуда × 0.8 и снова). При
   `single_contact_safe = false` берётся первый seed.
3. **Сыпуха.** Для каждой (`n_scree`): длина, амплитуда, корреляция; с p = `scree_ramp_bias`
   место ищется внутри случайной рампы/стенки, иначе в теле. Создаются
   `SurfaceSection(start, end, 'scree')` и `RoadRoughness` (через `_faceted`) на одном
   интервале; интервал занимает и препятствия (`taken`), и покрытия (`used`).
4. **Валуны** (`Bump`): высота, длина; при `single_contact_safe` длина ≥ `⌈L_min·100⌉/100`.
   Размещение без деления.
5. **Канавки** (`BowlPothole`): длина, глубина; при `single_contact_safe` глубина
   ≤ `0.98·d_max`. Размещение без деления.
6. **Ступени** (`SquareEdge`, только при `n_sharp_edges > 0`): высота, длина 0.2–0.6 м.
7. **Сырые зоны** (`n_wet`): только `used`.
8. **Фоновая текстура**: одна амплитуда и корреляция на трассу; свободные интервалы тела
   (`free_intervals(body, taken, 0)`) длиной ≥ 4 корреляций получают `RoadRoughness`
   (через `_faceted`), длина подрезана от float-шума как в `road.py`.
9. **Сборка**: `TrackSpec(name or f'alpine_{seed}', length, obstacles, surface=base,
   grade_profile, surface_sections (сорт.), description='Alpine scree climb; seed=…;
   spec_sha256=…')` → `validate()`.

Неудачное размещение (нет места) молча пропускает элемент — как в `generator.py`; тест T2
следит, чтобы дефолтный спек не был слишком тесным.

### 5.3 `terrain/contact_safety.py`

Чистые функции без MuJoCo:

- `min_bump_length_m(height_m, min_radius_m=.45)` = `π·√(2hR)`;
- `max_bowl_depth_m(length_m, min_radius_m=.45)` = `L²/(2π²R)`;
- `concave_span_deg(obstacle, window_m=.25)` — критерий 4.2(2) по узлам `_nodes()` c
  примыканием к плоскости на концах;
- `single_contact_hazards(track, *, min_radius_m, max_kink_deg, window_m) -> list[str]` —
  перечень нарушающих препятствий (`RockGarden`/`RoadRoughness` — по перепаду; `Bump` — по
  длине; `BowlPothole`/`GOut` — по глубине; всё остальное — опасно по построению; пусто = чисто).

Используется генератором и тестами; это «быстрая формула», ground truth — прокатка (T3).

### 5.4 `terrain/placement.py`

Перенос без изменения поведения: из `generator.py` — `CLEARANCE_M`, `PLACEMENT_TRIES`,
`_r → rounded`, `_range → check_range`, `_free → is_free`, `_place → place`; из `road.py` —
`_free_intervals → free_intervals`. Старые модули импортируют под прежними именами; тест T6
фиксирует побитовую неизменность их выходов.

### 5.5 Покрытие `scree`

`SurfaceSpec("scree", mu_peak=0.45, mu_slide=0.38, slip_stiffness_per_load=4.5,
stribeck_speed_mps=DEFAULT_STRIBECK_SPEED_MPS, description="alpine scree: loose rock debris
over bedrock")` в `SURFACES`. Авторская отправная точка, не измерение — как и все записи реестра.

### 5.6 Интеграция

| Файл | Изменение |
|---|---|
| `terrain/presets.py` | `"alpine_scree": alpine_scree` в `PRESETS`; `SEEDED_PRESETS = {"alpine_scree": (alpine_scree, ALPINE_PRESET_SEED)}` — пресеты, принимающие seed, но не длину |
| `terrain/__init__.py` | экспорт `AlpineGenSpec`, `generate_alpine_track`, `alpine_scree`, `ALPINE_PRESET_SEED`, `SEEDED_PRESETS`, `single_contact_hazards` |
| `cli/ride.py` | `resolve_track`: ветка `SEEDED_PRESETS` — `--seed` перегенерирует, `--length` → `ValueError`; `track_seed`: дефолтный seed пресета или override; `_list_tracks`: метка `generated` |
| `cli/research.py` | `--scenario alpine`; `--gen-spec` при `alpine` → `AlpineGenSpec.from_dict`; имя трассы `alpine_{seed}`; `--gen-spec` с другими сценариями по-прежнему отклоняется |
| `examples/research/alpine_scree.toml` | `dump_track(alpine_scree())` — материализованная трасса |
| `examples/research/alpine_rocky_genspec.toml` | `AlpineGenSpec` с `single_contact_safe = false`, камнями 6–12 см / 0.15–0.25 м, 1–2 ступенями; без пресета и без замороженной трассы |

Не трогаем: `trackfile.py`, `heightfield.py`, `native/`, `bike_sim/native/config.py`, `docs/RIDE.md`.

### 5.7 Команды после реализации

```bash
bike-ride --list-tracks
bike-ride --track alpine_scree [--seed 7] --preview
bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track alpine_scree --time-scale 1
bike-ride --backend native --physics-config … --track examples/research/alpine_scree.toml
bike-research --backend native --scenario alpine --seed 3 [--gen-spec my_alpine.toml]
bike-research --scenario alpine --gen-spec examples/research/alpine_rocky_genspec.toml --diagnostic-model-limits
```

## 6. Тесты (`tests/reference/test_alpine_track.py`, профиль `quick`)

- **T1 детерминизм/round-trip**: один seed → побитово равный `dump_track`; `track_from_dict`
  → `track_to_dict` равен исходному; разные seed → разные дампы.
- **T2 структура (8 seed)**: `validate()`; длина 120; lead-in/out без препятствий и зон, уклон
  0; ровно `n_walls` отрезков в `wall_grade`, остальные рампы в `ramp_grade`, полки в
  `shelf_grade`, контр-уклоны в `counter_grade`; каждая `scree`-зона совпадает ровно с одной
  `RoadRoughness` с амплитудой из `scree_amp_m`; база `hardpack`; счётчики ≤ верхних границ и
  ≥ 1 поле, ≥ 2 сыпухи, ≥ 1 валун, ≥ 1 канавка; `build_field_data(track)` строится.
- **T3 одноточечность**: `single_contact_hazards == []` (8 seed); прокатка 0.372/0.352 м,
  δ = 10 мм, 5 мм шаг — ноль `multi_support` (3 seed); контр-пример
  (`single_contact_safe=false`, `n_sharp_edges=(2,2)`, `rock_amp_m=(.10,.10)`,
  `rock_corr_m=(.15,.15)`) → hazards непуст **и** прокатка даёт флаги;
  `single_contact_safe=true` + `n_sharp_edges=(1,1)` → `ValueError`; перерисовка:
  `rock_amp_m=(.07,.07)`, `rock_corr_m=(.45,.45)` → hazards пуст; формулы на точках спайка.
- **T4 пресет/файлы**: `get_preset('alpine_scree')` валиден; `dump_track(get_preset(...)) ==
  alpine_scree.toml`; `load_track(...)` — `hardpack`, есть `scree`, 120 м;
  `alpine_rocky_genspec.toml` парсится, `single_contact_safe is False`; `scree` в `SURFACES`
  строго ниже `loose` по трём параметрам.
- **T5 CLI**: `resolve_track('alpine_scree')`, с `seed=7`, с `length_m=150` → `ValueError`;
  `track_seed('alpine_scree', None) == ALPINE_PRESET_SEED`; `research.build_track` для
  `alpine`/`alpine_3`; `--gen-spec` с чужим ключом → `ValueError`; `--gen-spec` при
  `rough_uphill` → отказ; `_list_tracks` печатает `generated`.
- **T6 регрессия**: sha256 дампов `generate_track(TerrainGenSpec(), seed=0..3)`,
  `road_smooth/worn/broken` и `TerrainGenSpec().sha256()` (значения зафиксированы в плане) не
  меняются после переноса хелперов.

Чего нет намеренно: прогонов MuJoCo/native (трасса — геометрия), тестов HUD/replay (формат не
менялся), проверок «реалистичности» дефолтов (их фиксирует этот документ).

## 7. Верификация

`bash tools/run_tests.sh quick` — все изменения Python-only, вне нативного скоупа. Дополнительно
вручную: `bike-ride --track alpine_scree --preview` и один интерактивный прогон на native с
`viewer_physics_welded.toml` (не гейт, а глазная проверка правдоподобия).

## 8. Ошибки и границы

- Невалидный спек → `ValueError` с именем поля при конструировании/`from_dict`.
- Неразмещаемый элемент → пропуск без ошибки (как `generator.py`).
- Критерий 4.2(2) консервативен: ложное срабатывание ведёт к перерисовке seed/усадке амплитуды,
  не к ошибке.
- `--length` для `alpine_scree` → `ValueError` с подсказкой про `--gen-spec`.

## 9. Вне скоупа / дальнейшие шаги

- Многоточечная шина (вариант D, второй план): спайк `distributed_2d_reference` на Python →
  решение о портировании в native. `alpine_rocky_genspec.toml` — её будущая трасса.
- Реальный рельеф (GPX/DEM → `GradeProfile`).
- Замедление райдера в шпильках (поведение контроллера, не трасса).
- Починка устаревших ссылок на RIDE.md §4.1/§11.2 в докстрингах `terrain/`.
