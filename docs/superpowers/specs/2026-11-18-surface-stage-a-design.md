# Этап А симуляции покрытия трассы: твёрдый контакт, гладкие переходы и закрытие энергобаланса многоточечных шин

**Дата:** 2026-11-18  
**Сессия:** Antigravity `992112b6`  
**Статус:** Утверждённая спецификация дизайна (прошла два раунда независимого adversarial-ревью против C++23 кодовой базы)

---

## 1. Контекст, область действия и границы этапа

В рамках четырёхэтапной дорожной карты развития модели покрытия трассы в плоской (2D) постановке (`docs/agents/simulation-decisions.md`, обновлённые п. 4, п. 23, п. 40):
- **Этап А (настоящая спецификация):** расширение модели твёрдого контакта `SurfaceSpec` (микро-надбавка сопротивления качению $\Delta C_{rr}$, податливость продольного сдвига $C_\kappa/F_z$, насыщение сцепления под нагрузкой колеса $\phi_{\text{load}}(F_{z,\text{wheel}})$), гладкие $C^1$-переходы `blend_m` на границах секций `SurfaceMap`, по-лучевой / по-станционный учёт покрытия и закрытие двойного учёта $C_{rr}$ и строгого баланса энергии в `pneumatic`.
- **Последующие этапы (вне объёма Этапа А):** Этап Б — деформируемый и сползающий тонкий слой конечной толщины для рыхлых покрытий (`loose`, `scree`) с 1D-памятью колеи; Этап В — локальные микро-покрытия препятствий; Этап Г — восприятие `surface_rear` / `surface_ahead` райдером и визуализация во вьюере.

### Жёсткие инварианты области действия
1. **Целевые бэкенды шин:** только многоточечные CPU-модели `pneumatic` (`PneumaticTireWriter` / `PneumaticTyreModel`) и `distributed_2d_reference` (`DistributedTireWriter`).
2. **Полная изоляция `compliant_2d`:** одноточечный бэкенд `compliant_2d` (`TireWriter`, `viewer_physics_welded.toml`, `rider_type_{careful,aggressive}.toml`), внешний `ResistanceWriter` и экспериментальное FP32 GPU-ядро (`native/src/gpu/`) **не меняют поведение ни на один бит**. Все ~35 существующих wire/bootstrap JSON-фикстур и все эталоны `compliant_2d` остаются побайтово неизменными.
3. **Сохранность существующих трасс:** приёмочные трассы (включая `rough_uphill_savage` с асфальтом по всей длине, п. 24) не меняются; для `blend_m == 0.0` канонические сериализованные словари и `track_sha256` (`terrain::identity_value`) совпадают со старыми побайтово.
4. **Отсутствие аллокаций на горячем пути (`BIKE_NONBLOCKING`):** функции `try_compute_into` в `PneumaticTireWriter` и `DistributedTireWriter` не совершают динамических аллокаций (контролируется RTSan и тестами контрактов).

---

## 2. Модель данных, wire-контракт и геометрия переходов `SurfaceMap`

### 2.1. Расширение `SurfaceSpec` и реестр 5 покрытий

В структуры `bike::terrain::SurfaceSpec` (`native/src/terrain/types.hpp`) и `nativecfg::SurfaceSpec` (`native/src/config_types.hpp`) добавляются поля со значениями по умолчанию `0.0` (сохраняя агрегатную инициализацию без предупреждений `-Wmissing-field-initializers`):
```cpp
double crr_delta = 0.0;         // микро-надбавка сопротивления качению >= 0.0
double load_sensitivity = 0.0;  // насыщение трения по нагрузке колеса в [0.0, 0.5]
```

Синхронная валидация в `bike::terrain::SurfaceSpec::validate()` (`native/src/terrain/surface.cpp`) и `nativecfg::validate(const SurfaceSpec&)` (`native/src/config_validation.hpp`):
- `std::isfinite(crr_delta) && crr_delta >= 0.0`;
- `std::isfinite(load_sensitivity) && 0.0 <= load_sensitivity && load_sensitivity <= 0.5`.

Параметр `stribeck_speed_mps = 4.5` м/с сохраняется неизменным для всех покрытий (удерживает пиковую силу `lumped_brush` в пределах 10 % от `mu_peak` на скоростях 15–45 км/ч по `docs/research/tyre-calibration.md` и не затрагивает `compliant_2d`).

#### Калибровочная таблица реестра (`native/src/terrain/surface.cpp`)

| Покрытие | `mu_peak` | `mu_slide` | `C_κ/F_z` | `V_str` (м/с) | `crr_delta` | `load_sensitivity` ($k_z$) | Провенанс и физический смысл новых полей |
|---|---:|---:|---:|---:|---:|---:|---|
| `asphalt` | 1.05 | 0.75 | 15.0 | 4.5 | `0.0000` | `0.06` | Базовый уровень барабана (`0.0`); умеренное насыщение резины при перегрузке (`authored`) |
| `hardpack` | 0.80 | 0.60 | 12.0 | 4.5 | `0.0025` | `0.08` | $\Delta C_{rr} \approx +0.0025$ по полевым заездам Meyer et al. для плотного мелкого грунта (`derived`); $k_z$ (`authored`) |
| `wet` | 0.50 | 0.40 | 10.0 | 4.5 | `0.0035` | `0.12` | Мокрая плотная почва: скользкая, но твёрже рыхлого гравия ($C_\kappa/F_z = 10 > 7$, $\Delta C_{rr} < \text{loose}$); проседание $\mu$ при выдавливании водной плёнки (`authored`) |
| `loose` | 0.55 | 0.45 | 7.0 | 4.5 | `0.0060` | `0.12` | $\Delta C_{rr} \approx +0.0061$ крупного гравия Meyer et al. (`derived`, только суб-лучевые потери зерна $< 15$ мм без макро-осадки Этапа Б); огибающая Barton–Leps (`extrapolated`) |
| `scree` | 0.45 | 0.38 | 4.5 | 4.5 | `0.0080` | `0.15` | Микро-перекладка мелкой фракции под лучами до макро-осадки Этапа Б (`authored`); нелинейная огибающая скальной наброски Leps 2025 (`extrapolated`) |

### 2.2. Секции `SurfaceSection` и зоны плавного перехода `TransitionZone`

В `bike::terrain::SurfaceSection` и `nativecfg::SurfaceSection` добавляется поле `double blend_m = 0.0;`. `nativecfg::SurfaceMap` остаётся открытым двухполевым агрегатом `{surface, sections}`.

Правила валидации секций и переходов (синхронно в `bike::terrain::SurfaceMap::validate()` и `nativecfg::validate(const SurfaceMap&)`):
1. Для каждой секции $i$: `std::isfinite(blend_m) && blend_m >= 0.0`, `0.0 <= start_m < end_m`.
2. **Левый край трассы ($x = 0$):**
   - если `sec[0].start_m == 0.0`, секция открывает трассу с начала координат — переход с фоновым покрытием в точке $x = 0$ **не создаётся** ($b_{\text{left},0} = 0$, при $x \le 0$ действует чистое `sec[0].surface`);
   - если `sec[0].start_m > 0.0`, требуется `sec[0].start_m >= 0.5 * sec[0].blend_m` ($b_{\text{left},0} = \text{sec}[0].\text{blend\_m}$), чтобы окно перехода не уходило в $x < 0$.
3. **Стыки соседних секций $(i, i+1)$:**
   - **вплотную примыкающие секции (`sec[i].end_m == sec[i+1].start_m`):** образуют единую границу $x_b = e_i = s_{i+1}$ между `sec[i].surface` и `sec[i+1].surface`; обязательно требуется `sec[i].blend_m == sec[i+1].blend_m` (иначе `std::invalid_argument`), $b_{\text{right},i} = b_{\text{left},i+1} = b_i$;
   - **разделённые зазором фонового покрытия (`sec[i+1].start_m > sec[i].end_m`):** образуют два независимых перехода (`sec[i] -> base` ширины $b_i$ и `base -> sec[i+1]` ширины $b_{i+1}$); требуется `sec[i+1].start_m - sec[i].end_m >= 0.5 * (sec[i].blend_m + sec[i+1].blend_m)`.
4. **Вместимость самой секции $i$:** для каждой секции требуется `sec[i].end_m - sec[i].start_m >= 0.5 * b_{\text{left},i} + 0.5 * b_{\text{right},i}` (где для последней секции $b_{\text{right},N-1} = \text{sec}[N-1].\text{blend\_m}$).

Это гарантирует, что все ненулевые окна переходов $[x_b - \frac{1}{2}b,\; x_b + \frac{1}{2}b]$ попарно не пересекаются и лежат в $[0, \infty)$.

В процедурном генераторе `AlpineGenSpec` (`native/src/terrain/generators.hpp`, `alpine.cpp`) добавляется опциональный параметр `surface_blend_m = 0.0` (дефолт `0.0` сохраняет все зафиксированные `spec_sha256` и существующие пресеты; в новых исследовательских сценариях задаётся `surface_blend_m = 0.4` м).

### 2.3. Wire-схема, парсер трасс и сохранение фикстур

1. **Схемы `native/src/configuration/schema.cpp` и `native/src/writer_config.hpp`:**
   - в схему `surface` добавляются `.optional = "crr_delta load_sensitivity"` (дефолт при декодировании `0.0`);
   - в схему `tire.surface_map.sections` добавляется `.optional = "blend_m"` (дефолт `0.0`);
   - в `native/tests/reference/test_native_input_contracts.cpp` ключи `"crr_delta"`, `"load_sensitivity"`, `"blend_m"` регистрируются в `optional_nested`.
2. **Условная проекция в `native/src/runtime/project_config.cpp`:**
   - для `tires.backend == "compliant_2d"` функция проекции покрытия эмитит ровно прежние 5 ключей (`name`, `mu_peak`, `mu_slide`, `slip_stiffness_per_load`, `stribeck_speed_mps`), а секция — без `blend_m` при `blend_m == 0.0`; тем самым сериализованный `config.json` для `compliant_2d` побайтово идентичен старому;
   - для `pneumatic` и `distributed_2d_reference` ключи `crr_delta` и `load_sensitivity` эмитятся при `!= 0.0` (как в `tire.surface_map`, так и в `tire.pneumatic.surface` для режима `configured`), а `blend_m` — при `blend_m > 0.0`.
3. **Файлы трасс (`native/src/terrain/trackfile.cpp`):**
   - чтение `surface_sections` принимает обязательные `start_m`, `end_m`, `surface` и опциональный `blend_m` (отклоняя любые иные ключи с `TrackFileError`);
   - запись `track_value` эмитит `"blend_m"` только при `s.blend_m != 0.0`, сохраняя `terrain::identity_value(track)` и `track_sha256` всех существующих `.toml` трасс.
4. **Профиль `pneumatic`:**
   - в `examples/research/viewer_physics_pneumatic.toml` и `tests/fixtures/config/viewer_physics_pneumatic.toml` добавляется `[resistance] crr = 0.0` (с синхронным обновлением `"crr": 0.0` в `tests/fixtures/config/resolved/pneumatic.json`), устраняя двойной учёт внешнего качения без поломки `OPT_REAL` схемы `ResistanceConfig`.

### 2.4. Канонический вычислитель `SurfaceSample` и fast-path однородного отрезка

Единый безсостоятельный модуль в `tirecommon` (`native/src/writers/tire_common.hpp/.cpp`) обслуживает `DistributedTireWriter` и `PneumaticTyreModel` без аллокаций:

```cpp
struct SurfaceSample {
    std::string_view name{};
    double mu_peak = 0.0;
    double mu_slide = 0.0;
    double slip_stiffness_per_load = 0.0;
    double stribeck_speed_mps = 4.5;
    double crr_delta = 0.0;
    double load_sensitivity = 0.0;
};
```

- **`eval_surface(const nativecfg::SurfaceMap& map, double x_m) -> SurfaceSample`:**
  - проверяет `std::isfinite(x_m)`;
  - если `x_m` попадает внутрь валидной зоны перехода $[x_b - \frac{1}{2}b,\; x_b + \frac{1}{2}b]$ ($b > 0$) между левым покрытием $L$ и правым $R$: при $L.\text{name} == R.\text{name}$ возвращает $L$ без арифметики (исключая погрешность $\pm 1$ ULP), иначе вычисляет $t = (x_m - (x_b - 0.5 b)) / b \in [0, 1]$ и гладкий полином Эрмита $s(t) = t^2 (3.0 - 2.0 t) \in [0, 1]$ ($s'(0) = s'(1) = 0$), линейно смешивая все 6 численных полей как $(1 - s) L + s R$;
  - вне зон перехода (включая все случаи `blend_m == 0.0`) копирует поля активной секции / базового покрытия **без арифметических операций**;
  - поле `name` всегда ссылается (`std::string_view`) на каноническое имя `surface_at(map, x_m).name` (никаких конкатенаций строк $\implies$ полное соблюдение зарезервированного `surface_capacity` в RTSan и закрытого `rider::SurfaceLabel`).
- **`uniform_surface_for_span(const nativecfg::SurfaceMap& map, double x_min_m, double x_max_m) -> const nativecfg::SurfaceSpec*`:**
  - если весь отрезок $[x_{\min}, x_{\max}]$ лежит строго внутри одной чистой зоны (не задевая ни одно окно $[x_b - \frac{1}{2}b,\; x_b + \frac{1}{2}b]$ и ни одну границу секции), возвращает указатель на соответствующий `SurfaceSpec`; иначе возвращает `nullptr`.

---

## 3. Физические законы контакта и сохранения энергии

### 3.1. Закон трения Штрибека с $C^1$-гладким насыщением по нагрузке колеса

Для сэмпла покрытия $S$, скорости скольжения $v_s$ и **полной нормальной нагрузки колеса** $F_{z,\text{wheel}} \ge 0$ (опорная нагрузка $F_{z,0} = 500.0$ Н, ширина сглаживания включения $\delta_u = 0.1$):

$$\mu_{\text{str}}(S, v_s) = S.\mu_{\text{slide}} + (S.\mu_{\text{peak}} - S.\mu_{\text{slide}})\exp\!\left(-\frac{|v_s|}{S.v_{\text{str}}}\right),$$

$$u = \max\!\left(0.0,\; \frac{F_{z,\text{wheel}}}{F_{z,0}} - 1.0\right), \qquad g(u) = \begin{cases} 0.0, & u \le 0.0, \\ \dfrac{u^2}{u + \delta_u}, & u > 0.0, \end{cases} \qquad \phi_{\text{load}}(F_{z,\text{wheel}}; k_z) = \frac{1.0}{1.0 + k_z\,g(u)},$$

$$\mu_{\text{eff}}(S, v_s, F_{z,\text{wheel}}) = \phi_{\text{load}}(F_{z,\text{wheel}}; S.k_z)\cdot \mu_{\text{str}}(S, v_s).$$

Математические свойства:
1. При $F_{z,\text{wheel}} \le 500.0$ Н или $S.k_z == 0.0$ множитель $\phi_{\text{load}} \equiv 1.0$ без умножения — побитово сохраняет все стендовые и барабанные тесты на нагрузках $418$ Н и $490.5$ Н.
2. В точке $F_{z,\text{wheel}} = 500.0$ Н выполнено $g(0) = 0$, $g'(0) = 0$ — закон $C^1$-гладкий на $[0, \infty)$.
3. Для всех $k_z \in [0, 0.5]$ предельная сила сцепления колеса $\mu_{\text{eff}}(F_z)\cdot F_z$ строго возрастает ($\frac{d}{dF_z}[\mu_{\text{eff}} F_z] > 0$) и при экстремальных ударах ($F_z \to \infty$) монотонно стремится сверху к конечной горизонтальной асимптоте $\mu_{\text{str}} F_{z,0} / k_z$.

### 3.2. `DistributedTireWriter` (`distributed_2d_reference`)

В `native/src/writers/distributed_tire.hpp/.cpp`:
1. **Предвыделенные буферы конструктора (0 аллокаций в `BIKE_NONBLOCKING`):**
   - добавляются `contact_x_.resize(n)` и `slips_.resize(n)` ($n = \text{stations\_}.\text{angles}.\text{size}()$);
   - резервируется `surface_capacity = max(10, ...)` (минимум длина `"configured"`) в `diagnostics_[s].surface` и `diagnostics_stage_[s].surface`.
2. **Двухпроходный расчёт станций на колесе:**
   - **Проход 1:** по активным станциям $j \in [0, \text{count})$ один раз вызывается `profile_->query`, собираются `jac_station_` и скорость станции `velocity`, записываются `loads_[j]`, `energies_[j]`, `depths_[j]`, `normals_[3*j..]`, `contact_x_[j] = geometry.point[0]`, `slips_[j] = dot3(velocity, tangent)`, накапливается `radial_power`. По завершении прохода вычисляется полная нагрузка колеса `normal_total = numeric::numpy_sum(loads_.first(count))`.
   - **Проход 2:** для каждой активной станции `sid = active_ids_[j]`:
     - если `surface_map_` задана: $S_j = \text{eval\_surface}(*\text{surface\_map\_}, \text{contact\_x\_}[j])$, $\mu_j = \min(\text{pcfg.mu},\, \mu_{\text{eff}}(S_j, \text{slips\_}[j], \text{normal\_total}))$, отношение сдвиговой жёсткости $r_k = S_j.\text{slip\_stiffness\_per\_load} / 15.0$ (эталон `asphalt` $= 15.0$); при `!surface_map_`: $\mu_j = \text{pcfg.mu}$, $r_k = 1.0$, $S_j.\text{crr\_delta} = 0.0$;
     - эффективная касательная жёсткость станции $k_{\text{eff},j} = k_{\text{station}} \cdot r_k$, где $k_{\text{station}} = k_t \cdot w_{\text{sid}}$;
     - **энергетически-приведённое состояние $\tilde{\xi} = \xi_{\text{phys}}\sqrt{r_k}$ в `StationState::xi`:**
       - поворот касательной действует на $\tilde{\xi}$ (`transported = old.xi * dot3(*old.tangent, tangent)`, `losses += max(0, 0.5 * k_station * (old.xi^2 - transported^2))`);
       - при $r_k == 1.0$: `xi_in = transported`, `next.xi = xi_new` без `sqrt` (побитовая идентичность на `configured` и `asphalt`);
       - при $r_k \neq 1.0$: $\text{scale} = \sqrt{r_k}$, $\xi_{\text{in}} = \text{transported} / \text{scale}$, после `contactlaw::brush_step(xi_in, slips_[j], v_roll, loads_[j], k_eff_j, mu_j, pcfg.relaxation_length_m, dt)` записывается `next.xi = xi_new * scale` и `shear_energy += 0.5 * k_station * pyfloat::pow(next.xi, 2.0)`;
       - благодаря тождеству $\frac{1}{2}k_{\text{eff},j}\xi_{\text{in}}^2 = \frac{1}{2}k_{\text{station}}\tilde{\xi}^2$ методы `stored_energy(d)`, потери отрыва станции и 4-колоночный формат `state()`/`set_state()` остаются консервативными на любых стыках покрытий и работают в том числе на старте с места ($v_{\text{roll}} = 0$);
     - обобщённая сила станции отображается в `out` через дешёвый пересчёт `jac_station_ = jac_center_ + jr_ × arm` и `dgemv(Trans)`.
3. **Мировой осевой момент микро-качения $\Delta C_{rr}$:**
   - по активным станциям вычисляется $\overline{\Delta C_{rr}} = \text{normal\_total} > 0 ? \frac{\sum_j \text{loads\_}[j]\, S_j.\text{crr\_delta}}{\text{normal\_total}} : 0.0$;
   - если `normal_total > 0.0 && snap.effective_radius_m > 0.0 && crr_bar > 0.0`:
     - по оси вращения колеса `axis = {xmat[9*body+1], xmat[9*body+4], xmat[9*body+7]}` и уже вычисленному `jr_` находится `row = jr_.T @ axis` (в `tmp_`) и мировая угловая скорость $\omega_{\text{world}} = \text{row} \cdot \dot{q}$;
     - момент $M_{\text{roll}} = -\overline{\Delta C_{rr}} \cdot \text{normal\_total} \cdot R_{\text{eff}} \cdot \tanh(\omega_{\text{world}} / \omega_{\text{taper}})$ с общей константой $\omega_{\text{taper}} = 0.2$ рад/с (`kWheelRollingTaperRadps`, согласованной с `ResistanceConfig::rolling_taper_rad_s`);
     - вклад в обобщённые силы: `out[i] += M_roll * row[i]` (реакция момента уходит строго в землю/world, без паразитного анти-вилли момента на вилку/раму);
     - в каждый патч снимка `snap.patches[j]` записывается доля момента `couple_world_nm = {0.0, M_roll * (loads_[j] / normal_total), 0.0}`;
     - диссипация шага: `losses += (-M_roll * omega_world) * dt` ($\ge 0$ безусловно при любых $v_x, \omega_{\text{world}}$).
4. **Поле `diag.surface`:** в C++ структуре `TireDiagnostics` заполняется доминирующим именем покрытия (или `"configured"`), при этом внешний 15-ключевой wire-словарь `wire_distributed_tire_diagnostics` в `runtime/step.cpp` не ломается (расширение каналов телеметрии относится к Этапу Г).

### 3.3. `PneumaticTyreModel` и `PneumaticTireWriter` (`pneumatic`)

#### 3.3.1. Предварительная полная нагрузка колеса и по-лучевое покрытие пятна
1. Перед циклом по пятнам в `PneumaticTyreModel::evaluate_impl` (`native/src/pneumatic/model.cpp`) за один проход по `carcass.patch_count` вычисляется евклидова сумма нормальных нагрузок пятен `wheel_normal_load_n` ($\equiv \text{WheelOutputs::normal\_load\_n}$, а не скалярная сумма лучей `carcass.normal_load_n`).
2. Для каждого пятна `[start_i, stop_i)` проверяется `spec = uniform_surface_for_span(surface_map, x_min, x_max)` по диапазону `hits.road_x_m[start_i..stop_i)`:
   - **Однородный участок (`spec != nullptr`):** по-лучевое усреднение не выполняется; если `wheel_normal_load_n <= 500.0 || spec->load_sensitivity == 0.0`, вызываются базовые `lumped_brush` / `DiscretisedBrush::step` напрямую от `*spec` (100 % побитовое совпадение с однородными эталонами `native_pneumatic_brush`); при активном насыщении `wheel_normal_load_n > 500.0 && spec->load_sensitivity > 0.0` к `*spec` применяется скалярный множитель $\phi_{\text{load}}(F_{z,\text{wheel}}; \text{spec}->k_z)$ на `mu_peak` и `mu_slide`.
   - **Неоднородный участок (`spec == nullptr`, пересечение стыка или `TransitionZone`):**
     - в `fast` (`lumped_brush`): по нагруженным лучам пятна ($f_i > 0$) вычисляется взвешенный по силам $\bar{S} = \frac{1}{W}\sum_i f_i\,\text{eval\_surface}(\text{surface\_map}, x_{\text{start}+i})$, откуда синтезируется эффективный `SurfaceSpec` с учётом $\phi_{\text{load}}(F_{z,\text{wheel}}; \bar{S}.k_z)$;
     - в `detailed` (`DiscretisedBrush`): перегрузка `DiscretisedBrush::step_heterogeneous(...)` принимает сэмплы покрытия лучей пятна (в предвыделенном scratch-буфере модели длины `ring_.n()`). В `bristle_deflection_m_` **всегда хранятся физические метры** $u_i$ (сохраняя размерность адвекции `updated_[i] - filtered_v_s_mps * dt_s` и wire-контракт `bristle_m`). На каждом луче $i$: $c_{px,i} = \frac{S_i.\text{slip\_stiffness\_per\_load} \cdot F_{N,\text{patch}}}{2 a^2}$, $k_i = c_{px,i}\Delta x$, демпфер $k_i \tau_b (-V'_s)$, предел трения $\pm \mu_{\text{eff}}(S_i, |v_s|, F_{z,\text{wheel}}) \cdot f_i$, среднее трение пятна $\frac{\sum_i \mu_i f_i}{\sum_i f_i}$.
   - **Синхронизация `shear_energy_j` (`stored_energy(d) == elastic_energy_j()`):** в `TyrePatchOutput` добавляется поле `double shear_energy_j = 0.0;`, вычисляемое в конце шага пятна внутри `PneumaticTyreModel::evaluate_impl` на тех же $k_i$ и `state.bristle_deflection_m[i]` (на однородном fast-path — в точном порядке `0.5 * k_px * dx_m * numpy_sum(u_i^2)`). `PneumaticTireWriter::shear_energy_j(tyre)` суммирует закоммиченные `patch.shear_energy_j`, сохраняя побитовый инвариант `stored_energy(d) == elastic_energy_j()` даже после сдвига позы в `mj_step`.

#### 3.3.2. Мировой осевой момент качения `(tread_loss_crr_at_pressure + crr_delta)` и разделение мощностей
1. Линейная сила `tread_loss_force_n` удаляется из касательной силы пятна в `PneumaticTyreModel::evaluate_impl`: `tangential_force_n = brush_result.force_n`, `patch_loss_w = slip_loss_w`.
2. Каждое пятно публикует эффективный коэффициент качения `patch.rolling_crr = tread_loss_crr_at_pressure(tyre_) + patch_crr_delta` (где `patch_crr_delta` не масштабируется давлением $(p/1.5)^{-0.3}$), а `WheelOutputs` публикует взвешенный по нагрузке пятен `rolling_crr` и явное поле `carcass_dissipated_power_w` (отдельно от `dissipated_power_w = carcass_dissipated_power_w + brush_loss_w`).
3. В конструкторе `PneumaticTireWriter` предвыделяется буфер `jr_.resize(3 * nv_)` (для `row` переиспользуется `tmp_` размера `nv_`). В `PneumaticTireWriter::evaluate_wheels`:
   - после формирования `snap.effective_radius_m`, если `outputs.normal_load_n > 0.0 && snap.effective_radius_m > 0.0 && outputs.rolling_crr > 0.0`, вычисляется мировой вращательный якобиан тела колеса `mj_jac(m_, d, nullptr, jr_.data(), xpos_wheel, body)`, строка оси вращения `row = jr_.T @ axis` и мировая скорость вращения $\omega_{\text{world}} = \text{row} \cdot \dot{q}$;
   - прикладывается момент $M_{\text{roll}} = -\text{outputs.rolling\_crr} \cdot \text{outputs.normal\_load\_n} \cdot R_{\text{eff}} \cdot \tanh(\omega_{\text{world}} / 0.2)$ через `out[i] += M_roll * row[i]`;
   - в патчи снимка `snap.patches[k]` записывается `couple_world_nm = {0.0, M_roll * (patch.normal_load_n / outputs.normal_load_n), 0.0}` (обеспечивая полноту `snapshot_axis_moment` в `runtime/step.cpp:222-235` и точное равенство `qfrc == sum(J_p^T F_patch + J_r^T couple_patch)`);
   - мощность качения `rolling_power_w = -M_roll * omega_world >= 0` прибавляется в `diag.dissipated_power_w = outputs.dissipated_power_w + rolling_power_w`, `diag.brush_loss_j = (brush_power_w + rolling_power_w) * dt`, `brush_loss += (brush_power_w + rolling_power_w) * dt`, а `radial_power += outputs.carcass_dissipated_power_w` берётся напрямую без вычитания `brush_power_w`.

#### 3.3.3. Полное закрытие радиального энергобаланса `pneumatic` (воздух, обод, ветвь Максвелла)
1. **Однократный гистерезис обода (`native/src/pneumatic/carcass.cpp`):**
   - `out.hysteresis_force_n[i] = carcass_hysteresis` (без дублирования `rim_hysteresis`, который уже входит в `out.rim_force_n[i] = rim_elastic + rim_hysteresis`);
   - в `CarcassBuffers` сохраняется `rim_hysteresis_force_n[i]` (или вычисляется `rim_hysteresis` в `model.cpp:511-517`), чтобы диссипация каркаса учитывала оба гистерезиса: `max((carcass_hysteresis + rim_hysteresis) * material_rate_mps[i], 0.0)`.
2. **Безошибочный потенциал сжатого воздуха $W(\delta)$ в `PneumaticTireWriter::radial_energy_j`:**
   Для каждого луча с прогибом $d = \max(\delta_i, 0.0)$ и радиусом сечения $\rho = W_c / 2$:
   - при $d == 0.0$: $W(d) = 0.0$;
   - при $0 < d < \rho$: пусть $\psi = 4\arcsin(\sqrt{d / (2\rho)}) \in (0, \pi]$ (`psi = 2 * phi`, где `phi = 2 * asin(sqrt(d / (2 * rho)))` из $1 - \cos\phi = d/\rho$); при $\psi < 0.02$ используется ряд Тейлора $W(d) = \frac{1}{2}\rho^2 \frac{\psi^3}{6}\left(1 - \frac{\psi^2}{20} + \frac{\psi^4}{840}\right) > 0$ (исключающий катастрофическую потерю значащих цифр и отрицательные значения при скользящем касании $d \sim 10^{-18}$ м), иначе $W(d) = \frac{1}{2}\rho^2(\psi - \sin\psi)$ (в точке $d = \rho$, $\psi = \pi$, непрерывно и гладко переходит в $\frac{\pi}{2}\rho^2$ с производной $W'(d) = 2\sqrt{d(2\rho - d)}$);
   - при $d \ge \rho$: $W(d) = \frac{\pi}{2}\rho^2 + 2\rho(d - \rho)$.
   Полная упругая энергия воздуха и каркаса луча: $E_e(d) = \big(p_{\text{area}} W(d) + \frac{1}{2}k_c d^2\big)\Delta s$.
3. **Несингулярный потенциал Брэгмана–Лежандра для силовой ветви Максвелла (`rate_stiffening` $\gamma > 0$):**
   Для инициализированного `CarcassState` (`state.initialized() && gamma > 0`):
   - внутренняя релаксированная сила луча $i$: $\xi_{e,i} = \text{state.elastic\_prev\_n}()[i] - \text{state.maxwell\_force\_n}()[i] / \gamma$;
   - обращение $\delta^*(\xi_e) = f_e^{-1}(\max(\xi_e, 0))$ без корневой сингулярности при $\delta \to 0$:
     - при $\xi_e \le 0$: $\delta^* = 0$, $E_e^*(\xi_e) = 0$;
     - при $\xi_e \ge f_e(\rho)$: $\delta^* = \rho + \frac{\xi_e - f_e(\rho)}{k_c\Delta s}$;
     - при $0 < \xi_e < f_e(\rho)$: тригонометрическая замена $\delta^* = \rho(1 - \cos\phi)$, $\phi \in (0, \pi/2)$, решает гладкое строго монотонное уравнение $\big(2 p_{\text{area}}\rho\sin\phi + k_c\rho(1 - \cos\phi)\big)\Delta s = \xi_e$ (старт $\phi_0 = \arcsin(\min(1.0, \xi_e / (2 p_{\text{area}}\rho\Delta s)))$ + 4 фиксированные итерации Ньютона с производной $(2 p_{\text{area}}\cos\phi + k_c\sin\phi)\rho\Delta s > 0$, константное время без аллокаций), затем $E_e^*(\xi_e) = \xi_e\delta^* - E_e(\delta^*)$;
   - запасённая энергия Максвелла луча $U_M(d_i, \xi_{e,i}) = \gamma\max\!\big(0.0,\; E_e(d_i) + E_e^*(\xi_{e,i}) - \xi_{e,i} d_i\big)$ прибавляется в `radial_energy_j` (при этом $\partial U_M / \partial d_i = f_{m,i}$, а `stored_energy(d)` и `elastic_energy_j_` вызывают одну и ту же `radial_energy_j`, сохраняя побитовое `stored_energy(d) == elastic_energy_j()`);
   - диссипативная мощность релаксации и адвекции Максвелла на шаге (включая ветвь затухания в воздухе `advance_unloaded` в `model.cpp:464-475` при $d_i = 0$): вычисляется телескопирующейся дискретной разностью до и после обновления `CarcassState` на одних и тех же прогибах $d_i$ начала шага:
     $$P_{M,\text{loss}} = \frac{1}{\Delta t}\max\!\left(0.0,\; \sum_{i=0}^{n-1} U_M(d_i, \xi_{e,i}^{\text{pre}}) - \sum_{i=0}^{n-1} U_M(d_i, \xi_{e,i}^{\text{post}})\right) \ge 0$$
     (которая при $\Delta t \to 0$ переходит в $\sum_i \frac{\gamma}{\tau_r}(d_i - \delta^*(\xi_{e,i}))(f_e(d_i) - \xi_{e,i}) \ge 0$, а при конечном $\Delta t$ замыкает дискретный баланс энергии с точностью до $10^{-15}$ Дж!) и включается в `carcass_dissipated_power_w` (в том числе в `write_unloaded`).

---

## 4. Импакт на тесты, эталоны и матрица верификации

### 4.1. Классификация существующих тестов и эталонов
1. **Без изменений кода и без перезаписи эталонов:**
   - все тесты и фикстуры `compliant_2d` (`native_tire`, `episode`, `control_clock`, `rider_launch_rides_*`, `seated_pedaling_cycle`, `welded_profile_config`, `acceptance`);
   - `native_pneumatic_geometry`, `native_pneumatic_brush`, `native_pneumatic_wire`;
   - `distributed_tire_reference` и `native_distributed_stands` (`surface_mode = "configured"`).
2. **Точечное обновление тестовых проверок под новый контракт момента качения:**
   - проброс `crr_delta` и `load_sensitivity` в тестовых адаптерах `pneumatic_reference_support.hpp`, `pneumatic_tire_fixture.hpp`, `test_tyre_road_profile.cpp`;
   - учёт `J_r^T patch.couple_world_nm` в проверках полного `qfrc` в `test_pneumatic_tire_forces.cpp` и `test_pneumatic_tire_oracle.cpp`;
   - обновление `test_model_rolling_loss_signs_bitwise` в `test_native_pneumatic_model.cpp` под перенос `tread_loss` в осевой момент.
3. **Осознанная перезапись эталонов (`BIKE_GOLDEN_RECORD=1` на целевой платформе):**
   - `reference.native_pneumatic_carcass`, `reference.native_pneumatic_model`, `reference.pneumatic_tire_oracle`, `reference.native_pneumatic_tire`, `reference.native_distributed_tire`, `reference.native_numeric_completion` (`tire_channel_parity_across_backends` — половина `distributed`).

### 4.2. Новые тесты Этапа А (TDD)
1. **Геометрия и интерполяция `SurfaceMap`:**
   - валидация `SurfaceSpec` (`crr_delta < 0`, `load_sensitivity < 0` и `> 0.5`) и `SurfaceSection` (`blend_m < 0`, перекрытия через зазор, неравные `blend_m` на вплотную примыкающем стыке, `start_m == 0.0` без левого перехода vs `0 < start_m < 0.5 * blend_m`);
   - $C^1$-гладкость (`eval_surface` непрерывна и имеет нулевую численную производную на обоих краях `TransitionZone`), побитовое совпадение с `surface_at` вне зон перехода и корректность `uniform_surface_for_span`;
   - побайтовая неизменность сериализации wire для `compliant_2d` и `track_sha256` при `blend_m == 0.0`.
2. **Физика и энергия `DistributedTireWriter`:**
   - масштабирование касательной жёсткости в $15.0 / 4.5 = 3.33\times$ между `asphalt` и `scree` как при качении, так и при $v_{\text{roll}} == 0$ (старт с места на уклоне);
   - сохранение энергии при проезде станции через `TransitionZone` (`blend_m > 0`) и неизменность 4-колоночного roundtrip `state()`/`set_state()`;
   - насыщение $\mu_{\text{eff}}$ при $F_{z,\text{wheel}} > 500$ Н (двухпроходный учёт полной нагрузки колеса, а не нагрузки одной станции) и строгий рост $\mu_{\text{eff}}(F_z)\cdot F_z$;
   - осевой момент от `crr_delta`: отсутствие паразитного реактивного момента на раму/вилку, безусловная пассивность при $v_x < 0, \omega > 0$ и нулевые аллокации в `try_compute_into`.
3. **Физика и строгий баланс энергии `PneumaticTireWriter`:**
   - неотрицательность и точность интеграла хорды $W(\delta)$ при скользящих прогибах $\delta \in [10^{-18}, 10^{-6}]$ м и согласованность с численным интегралом $\int_0^\delta f_e(u)\,du$;
   - замкнутый цикл `load -> hold -> unload -> airborne decay` для потенциала Брэгмана–Лежандра $U_M(\delta, \xi_e)$ и диссипации $D_M$, плюс однократный учёт гистерезиса обода при `delta > rim_strike_deflection`;
   - побитовое равенство `stored_energy(d) == elastic_energy_j()` на `detailed` tier внутри зоны `blend_m > 0` и плавность $F_x(t)$ при переезде стыка `hardpack -> scree` даже при `blend_m == 0.0` (за счёт взвешивания по лучам пятна);
   - полный 60-метровый заезд обычного райдера на `viewer_physics_pneumatic.toml` (включая трассу со сменой покрытий и `blend_m > 0`) со сходимостью строгого баланса энергии `|residual_j| / energy_scale_j <= 0.05`.
