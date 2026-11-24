# Симуляция трасс и покрытий (Этапы Б, В, Г, примитивы мезо-рельефа и восприятие райдера): проектная спецификация

**Дата:** 2026-11-22 (уточнено решениями пользователя 2026-11-23, сессия `eea0aaf4`)  
**Основание:** `docs/agents/simulation-decisions.md` (пп. 4, 12, 22, 23, 24, 27–34, 39, 41, 42), `docs/superpowers/specs/2026-11-18-surface-stage-a-design.md`, пакет исследований `docs/research/2026-11-22-tracks-and-surfaces/00..08-*.md`.

---

## 1. Цели и жёсткие инварианты

1. **Сохранность замороженных трасс и `compliant_2d` (пп. 4, 23, 24):**
   - Существующие трассы (`rough_uphill.toml`, `rough_uphill_extreme.toml`, `rough_uphill_savage.toml`, `alpine_scree`, `alpine_rocky`) не редактируются; их `terrain::identity_value` и `track_sha256` остаются побайтово неизменными.
   - При отсутствии новых опциональных полей (`StepUp`, `spectrum = "iso8608"`, `Obstacle::surface`, `layer_thickness_m > 0`) сериализация трасс и `config.json` для `compliant_2d` побайтово совпадает с Этапом А.
   - Бэкенд `distributed_2d_reference` не перекалибровывается в рамках этой работы; целевым рабочим бэкендом для неровных и деформируемых трасс закрепляется `pneumatic` (решение пользователя 2026-11-23).
2. **Нулевые аллокации на горячем пути (`BIKE_NONBLOCKING`):**
   - Вызовы `try_compute_into` / `probe_compute_into` в `PneumaticTireWriter` и `DistributedTireWriter` не совершают динамических аллокаций (проверяется `allocation_counter` / RTSan).
3. **Запреты №36, №38, №39 и строгий баланс энергии (`residual_ratio <= 0.05`):**
   - Запрещён любой линейный вязкий член `c·ż` или `c·δ̇` в грунте и шинах; вся диссипация деформируемого грунта рождается только из упругопластической петли разгрузки $k_u$ и пластического сдвига.
   - Пробные вызовы с `advance == false` (`PhysicalStep::probe_step_inputs`, внутренний Ньютон `SpindleController`) никогда не мутируют состояние колеи `RutMemory`.
4. **Информационная граница (`test_native_rider_information_boundary`):**
   - Контроллер мотора (`drivetrain/motor.cpp`) и исследовательские политики (`research/*`, `Observation`) не имеют доступа к `RoadProfile`, `SurfaceMap`, `MicroSurfaceSpan`, `RutMemory` и `RiderSenses`.

---

## 2. Новые примитивы трасс и проверка кривизны (Отчёты 01, 02)

1. **Примитив истинного уступа вверх `StepUp` (`type = "step_up"`):**
   - Поля: `double start_m = 0., height_m = 0.060, ramp_m = 0.120;` (`height_m > 0`, `ramp_m > 0`).
   - Длина `length_m() == ramp_m`; сдвиг базового уровня **`datum_shift_m() == +height_m`** (полотно остаётся на новой высоте, без заднего обрыва `SquareEdge`).
   - Профиль на $[0, \text{ramp\_m})$: $C^1$-гладкий косинусный подъём $z(x) = \frac{1}{2}h\,(1 - \cos(\pi x / \text{ramp\_m}))$ ($z(0)=0, z(\text{ramp\_m})=h, z'(0)=z'(\text{ramp\_m})=0$; `0.0` при $x \le 0$).
   - Критерий `single_contact_hazards` для колеса радиуса $R$: вогнутая кривизна у основания $z''(0) = \frac{\pi^2 h}{2\,\text{ramp\_m}^2} \le \frac{1}{R} \iff \text{ramp\_m} \ge \pi\sqrt{\frac{1}{2}h R}$ (иначе репортится hazard).
   - Для `SteppedClimb` (`obstacles.cpp:151-169`) в `single_contact_hazards` вместо безусловного `else`-отказа проверяется максимальная вогнутая кривизна переходной квинтики $\max z'' = \frac{3|\Delta g|}{2\,t_{\text{len}}} \le \frac{1}{R}$.
2. **Спектральная шероховатость `ISO 8608` на `RoadRoughness`:**
   - Опциональные поля `std::string spectrum = "gaussian";` (`"gaussian"` или `"iso8608"`) и `double waviness = 2.0;` (`[1.0, 4.0]`). При `spectrum == "gaussian"` и `waviness == 2.0` поля не пишутся в `obstacle_fields`, сохраняя все существующие `track_sha256`.
   - При `spectrum == "iso8608"` на равномерной сетке узлов с шагом $\Delta = \text{correlation\_length\_m} / 2$ синтезируется сумма косинусных гармоник в полосе пространственных частот $n_k \in [n_{\min}, n_{\max}]$, где $n_{\max} = \frac{1}{2\,\text{correlation\_length\_m}}$ ($\le \frac{1}{0.12\text{ м}}$ для разделения с `crr_delta`), $n_{\min} = \min(\max(1 / \text{section\_length\_m}, 0.2\text{ м}^{-1}), n_{\max})$, со случайными фазами $\phi_k \sim U(0, 2\pi)$ от `NumpyRng(seed)`, амплитудами $\propto n_k^{-w/2}$, косинусным тейпером краёв и аналитической нормировкой ожидаемого СКО $\text{RMS}(z) = \text{amplitude\_m} / 3.0$ (не зависящей от длины секции и устойчивой при нарезке коротких кусков `carve_roughness`).

---

## 3. Этап В: локальные микро-покрытия препятствий (Отчёт 04)

1. **Расширение реестра микро-покрытий (`native/src/terrain/surface.cpp`):**
   - `available_surfaces()` и `get_surface(name)` сохраняют 5 базовых макро-покрытий (`asphalt`, `hardpack`, `loose`, `wet`, `scree`) для совместимости с `SurfaceLabel` и существующими тестами.
   - Добавляются `available_micro_surfaces()` и `get_obstacle_surface(name)` (принимает все 5 макро-покрытий плюс 5 локальных микро-покрытий с `stribeck_speed_mps = 4.5`):
     - `dry_wood`: `mu_peak=0.45, mu_slide=0.35, C_k=11.0, crr_delta=0.0020, k_z=0.08`;
     - `wet_wood`: `mu_peak=0.25, mu_slide=0.18, C_k=9.0, crr_delta=0.0030, k_z=0.12`;
     - `polished_rock`: `mu_peak=0.60, mu_slide=0.45, C_k=14.0, crr_delta=0.0010, k_z=0.06`;
     - `wet_rock`: `mu_peak=0.30, mu_slide=0.20, C_k=12.0, crr_delta=0.0020, k_z=0.08`;
     - `mud_lens`: `mu_peak=0.28, mu_slide=0.20, C_k=5.0, crr_delta=0.0100, k_z=0.15`.
2. **Поля на `Obstacle` и компиляция `MicroSurfaceSpan`:**
   - В `Obstacle` добавляются `std::string surface{};` и `double surface_blend_m = 0.0;` (`>= 0`, конечное; сериализуются в TOML/identity только если `!surface.empty()`).
   - Метод `Obstacle::micro_surface_spans() const` (вызывается **только на фазе подготовки** `project_config.cpp`) генерирует непересекающиеся локальные интервалы `[start, end)`:
     - `Roots`: `[start_m + c - width_m/2, start_m + c + width_m/2]` для каждого корня $c \in \text{bump\_centres\_m}()$ (промежутки между корнями сохраняют базовый `SurfaceMap`);
     - `Pothole`, `SlopedPothole`, `BowlPothole`, `GOut`: донные `60 %` длины `[start_m + 0.2*L, start_m + 0.8*L]`;
     - `RockGarden`: интервалы сегментов, где на обоих концах $z_{\text{node}} > 0$ (каменные гребни);
     - все остальные препятствия с `length_m() > 0`: `[start_m, end_m)`.
   - Валидация требует `end - start >= surface_blend_m` (воротник Эрмита `[-b/2, +b/2]` на левом и правом краю спана).
3. **Вычисление в `tirecommon` (`0` аллокаций):**
   - `nativecfg::SurfaceMap` получает поле `std::vector<MicroSurfaceSpan> micro_spans{};` (отсортированы по `start_m`, не пересекаются; для `compliant_2d` и при пустом списке JSON-проекция `config.json` не меняется).
   - `eval_surface(map, x_m)` проверяет `micro_spans` бинарным поиском: внутри чистого ядра `[s + b/2, e - b/2]` возвращает `sample_from_spec(span.surface)`, в краевых воротниках `[s - b/2, s + b/2]` и `[e - b/2, e + b/2]` смешивает `eval_surface_base(map, x_m)` и `span.surface` полиномом Эрмита $t^2(3-2t)$. `uniform_surface_for_span` возвращает `nullptr`, если `[x_min, x_max]` задевает любой расширенный интервал `[s - b/2, e + b/2]`, и `&span.surface`, если целиком лежит в чистом ядре `[s + b/2, e - b/2]`.

---

## 4. Этап Б: деформируемый тонкий слой (`loose`, `scree`) с 1D-памятью колеи и раздельным энергобалансом (Отчёты 05, 06)

1. **Активация и утверждённые константы слоёв (решение пользователя 2026-11-23):**
   - В `SurfaceSection` (и `nativecfg::SurfaceSection`) добавляется опциональное поле `double layer_thickness_m = 0.0;` (`0.0` = выключено, твёрдый режим Этапа А, побайтово идентично существующим тестам; положительные значения должны лежать в `[0.012, 0.150]` м и разрешены только для деформируемых поверхностей `"loose"` и `"scree"`).
   - Физические константы слоёв (`loose` vs `scree`, `authored`, строго без линейной вязкости `c·ż`):
     - `loose`: $k_0 = 180\text{ кПа}$, $n = 0.80$, $\zeta_o = 0.45$, $\zeta_w = 0.30$, $B_{hp} = 500\text{ кПа}$, $c_{hp} = 4.5$, $\beta_u = 6.0$, $\mu_{\text{basal}} = 0.52$, $\rho_b = 1650\text{ кг/м}^3$, $\alpha_\tau = 1.0$, $\alpha_n = 0.55$;
     - `scree`: $k_0 = 130\text{ кПа}$, $n = 0.85$, $\zeta_o = 0.40$, $\zeta_w = 0.30$, $B_{hp} = 650\text{ кПа}$, $c_{hp} = 5.0$, $\beta_u = 5.0$, $\mu_{\text{basal}} = 0.42$, $\rho_b = 1750\text{ кг/м}^3$, $\alpha_\tau = 1.0$, $\alpha_n = 0.45$.
   - Чтобы при $n \in (0, 1)$ кривая первичного нагружения имела конечную непрерывную производную $C^1$ вплоть до $z = 0$, базовый член Беккера регуляризуется малым масштабом зерна $z_\varepsilon = 10^{-4}\text{ м}$ (`0.1 мм`):
     $$p_{\text{base}}(z) = k_0 \frac{(z + z_\varepsilon)^n - z_\varepsilon^n}{b^n}, \qquad p'_{\text{base}}(z) = k_0 \frac{n\,(z + z_\varepsilon)^{n-1}}{b^n},$$
     что даёт $p_{\text{base}}(0) = 0$, конечный положительный наклон при $z = 0$ и точное аналитическое совпадение $p'_{\text{virgin}}(z) \equiv \frac{dp_{\text{virgin}}}{dz}$ на всём $[0, H_s)$.
2. **1D-сетка `RutMemory` (`native/src/tyre/rut_memory.{hpp,cpp}`):**
   - Совпадает по узлам с `RoadProfile` (`x0_m`, `dx_m = 5 мм`, $N$ узлов, предвыделенные `z_max_m_`, `z_p_m_`, `z_e_m_`, `h_s_m_`, `0` аллокаций на шаге).
   - На каждом шаге `PneumaticTireWriter` (при наличии узлов с $H_s > 0$):
     1. Переднее колесо вычисляется и коммитит осадку (при `advance == true`) до заднего колеса, создавая эффект многопроходности (заднее колесо идёт по уплотнённой ветви разгрузки $k_u = \beta_u p'_{\text{virgin}}(z_{\max})$ с остаточной колеёй $z_p(x)$).
     2. Для активного луча решается совместное скалярное равновесие «луч шины ↔ узел грунта» на $z \in [0, H_s)$ с гладким барьерным упрочнением скального дна ($z_{\max} < H_s$ строго).
     3. На крутом уклоне $\theta(x_i)$ эффективное трение пятна ограничивается сверху базальной устойчивостью слоя $\mu_{\text{eff}} \leftarrow \min(\mu_{\text{eff}}, \mu_{\text{basal,avail}}(\theta, F_N))$.
     4. При `advance == false` (`probe_compute_into`, `PhysicalStep::probe_step_inputs`) расчёт выполняется на `const RutMemory&` без мутации состояния и счётчиков.
3. **Раздельный учёт упругой энергии и пластической диссипации грунта в балансе энергии (решение пользователя 2026-11-23, пп. 23 и 34):**
   - `PneumaticTireWriter::stored_energy(d)` и `PneumaticTireWriter::elastic_energy_j()` возвращают **только собственную упругую энергию шины** (каркас, обод, ветвь Максвелла, щётки `detailed`), сохраняя побайтовое равенство `stored_energy(d) == elastic_energy_j()`; упругая энергия слоя $U_{\text{soil},e} = \sum_g \frac{1}{2} K_{u,g} z_{e,g}^2 \ge 0$ возвращается отдельно через `PneumaticTireWriter::soil_elastic_energy_j()`.
   - В `PhysicalStep` (`native/src/runtime/step.cpp`) карта `elastic_energy_j` (`map<double>`) содержит `"tires"` (чисто шина), а при `tire_writer().has_deformable_layer() == true` дополнительно публикует ключ **`"soil"`** со значением `soil_elastic_energy_j()` (входящим в полную механическую+упругую сумму `total`). При `has_deformable_layer() == false` ключ `"soil"` не эмитится.
   - `PneumaticTireWriter::brush_loss_step_j()` возвращает **только собственные потери щёток/скольжения шины** (пластическая работа грунта не подмешивается в `brush_loss_step_j` и `tires.*.brush_loss_j`); пластическая работа уплотнения колеи $\Delta D_{\text{soil},p} \ge 0$ (16-точечная квадратура Гаусса–Лежандра с разбиением в изломе $z_{\max}$) возвращается через `PneumaticTireWriter::ground_plastic_loss_step_j()` (`0.0` на твёрдых трассах).
   - Строго по образцу `rider_end_stop_loss_step_j` (`native/src/runtime/accounting.cpp`, `native/src/recording/catalog_data.inc`): `RawStep` передаёт `ground_plastic_loss_step_j`, `step.cpp` включает её в `loss_step`, а `account_energy` публикует в канале `energy` обязательное поле **`ground_plastic_loss_step_j`** (`optional: false`, `0.0` на твёрдых трассах; входит в `loss_step_j` и кумулятивный `loss_j`), удерживая невязку `residual_ratio <= 0.05`.
   - `RutMemory` сериализуется разрежённым списком изменённых узлов (`rut.count` + `rut.node.*` по размеру профиля) с атомарной двухфазной валидацией.

---

## 5. Этап Г: восприятие неровного рельефа, строгие кодеки, `surface_rear` и визуализация во вьюере (Отчёт 08)

1. **Хордовое упреждение уклона `road_grade_chord_preview` (решение пользователя 2026-11-23):**
   - В `native/src/runtime/control.{hpp,cpp}` сохраняется существующая `road_grade_preview(road)` и добавляется `road_grade_chord_preview(std::span<const RoadSample> road, double front_wheel_x_m)`: вычисляет секущий уклон $(h(x_f + L) - h(x_f)) / L$ по окну взгляда (`L = road.back().x_m - x_f`; если $L \le 10^{-9}$, возвращает `road_grade_preview(road)`).
   - В `step.cpp:1290` и `step.cpp:2920` вызовы переводятся на `road_grade_chord_preview`. На гладких постоянных уклонах значение совпадает с точным уклоном трассы (`<= 2 ULP`), а на препятствиях подавляет высокочастотные выбросы в `400×`.
2. **Относительный угол подъёма переднего колеса в `Perception` (решение пользователя 2026-11-23):**
   - В `Perception::State` добавляется `std::optional<double> initial_grade_rad{};`, защёлкиваемое на первом доставленном `delayed` сэмпле как `std::atan(delayed->senses.grade_ahead)`.
   - В `Perception::update` (`native/src/rider/perception.cpp`) публикуемый в `Percepts::pitch_rad` угол подъёма переднего колеса вычисляется относительно видимого уклона тропы:
     `pitch_above_trail = pitch - (std::atan(delayed->senses.grade_ahead) - *initial_grade_rad)`.
   - На любой трассе с постоянным уклоном разность `atan(grade_ahead) - *initial_grade_rad == 0.0` побитово тождественна нулю; на ступенчатых трассах (`0 % -> 15–35 %`) снимается ложная вечная защёлка `FrontLiftReaction`. В `Perception::restore` проверяется инвариант достижимости `state.initial_grade_rad.has_value() == (state.tick > delay_ticks_)` и конечность значения.
3. **Канал `surface_rear` и строгие кодеки `ncd::exact` (решение пользователя 2026-11-23):**
   - В конец `rider::RiderSenses` (`senses.hpp`) добавляется `SurfaceLabel surface_rear = SurfaceLabel::hardpack;`, заполняемое в `step.cpp:2914` как `config_.surface_labels.at(wheel_x_m[1])` и валидируемое в `perception.cpp`.
   - Кодеки `decode_rider_senses` / `encode_rider_senses` и `decode_perception_state` / `encode_perception_state` (`senses_codec.cpp`) сохраняют **строгий контракт `ncd::exact`** без тихих fallback-ветвей: и `surface_rear` (в `RiderSenses`, `step.cpp:2093`, `catalog_data.inc`), и `initial_grade_rad` (в `Perception::State`, `null` до первой доставки и `Number` после неё) являются обязательными полями канонического формата. Затронутые фикстуры `darwin-arm64` штатно перезаписываются в Task 5.
   - В `LaunchPlan::ramp` (`launch_plan.cpp:107`) потолок сцепления при старте вычисляется как `std::min(cap(p.senses.surface_rear), cap(p.senses.surface_ahead))`.
   - В `test_native_rider_information_boundary.cpp` проверяется побайтовая инвариантность `Observation` исследовательской среды при замене покрытия трассы впереди велосипеда.
4. **Визуализация покрытий в текстовом HUD и на 3D-полотне вьюера (`bike-ride`, решение пользователя 2026-11-23):**
   - В `native/src/viewer/hud.cpp` (`physical_columns`) добавляется компактная колонка `surf r/f/a` (`surf=asp/asp/scr` по полям `rider_senses.{surface_rear,surface_front,surface_ahead}`).
   - В `native/src/viewer/decorations.hpp` и `render_owner.cpp` добавляется презентационный список цветных сегментов полотна `std::vector<SurfaceBand> surface_bands;` в `Decorations`, отрисовываемый через `mjv_initGeom` + `mjv_connector` в `RenderOwner::draw` поверх дорожной ленты в окне видимости вокруг велосипеда **без модификации `mjModel` и `model_sha256`**. Цветовая палитра различает как 5 макро-покрытий `SurfaceMap` (`asphalt`, `hardpack`, `wet`, `loose`, `scree`), так и 5 локальных микро-покрытий препятствий Этапа В (`dry_wood`, `wet_wood`, `polished_rock`, `wet_rock`, `mud_lens`); подключается в `viewer::run_ride` (`viewer/ride.cpp`).

---

## 6. Калибровочные исследовательские трассы и критерии приёмки (Task 5)

1. Выпускаются две новые исследовательские трассы (`examples/research/rough_uphill_trail_calibrated.toml` и `examples/research/alpine_scree_deformable.toml`), покрывающие `StepUp`, `RoadRoughness(spectrum="iso8608")`, локальные микро-покрытия `wet_wood` / `mud_lens`, деформируемые слои `loose` / `scree` (`layer_thickness_m > 0`) и обновлённое восприятие Этапа Г.
2. Обе трассы обязаны проходить на профиле `viewer_physics_pneumatic.toml` с **полной строгостью приёмки** (решение пользователя 2026-11-23):
   - `outcome == "end_of_track"`, `crashed == false`, `balance_lost == false`;
   - `model_valid == true` (нулевое число нарушений ограничений модели за всю поездку);
   - реакции опоры седла и обеих стоп `min_n >= -6.0 Н`, тяга каждой руки на руле `<= 300.0 Н`;
   - строгий баланс энергии `residual_ratio <= 0.05` и `energy_quality(energy, 0.05).acceptable == true` на всех измеренных интервалах;
   - на `alpine_scree_deformable.toml` подтверждается положительная работа пластического уплотнения `ground_plastic_loss_step_j` и эффект многопроходной колеи `RutMemory`.
3. Трасса `examples/research/rough_uphill_savage.toml` остаётся побайтово неизменной; в отчёт включается контрольный диагностический замер её прохождения (`welded` и `pneumatic`) до и после исправлений восприятия Этапа Г.
