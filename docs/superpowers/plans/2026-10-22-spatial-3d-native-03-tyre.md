# Spatial 3D Native — 03. Шина и контакт Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Если эти skills недоступны, сохраняйте тот же порядок реализации, независимого ревью и доказательной приёмки.

**Goal:** Подключить пространственную pneumatic-шину с воспроизводимой историей материала, боковыми силами, camber, единым пределом сцепления и проверяемым балансом энергии.

**Architecture:** Новые kernels образуют `bike::pneumatic3d`; writer использует существующий owning stepper и caller-owned force buffers. Контактная геометрия, нормальная реакция, patch lifecycle и tangential law разделены, чтобы проверять каждый физический смысл независимо. Старый `pneumatic` остаётся отдельным planar dispatch.

**Tech Stack:** C++23, существующие MuJoCo/native numerics и writer interfaces, `bike::terrain::Grid3d`/`Terrain3dSource`, CTest и native JSON/IO для cold artifacts.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`, §§5–8; `2026-10-22-spatial-3d-native-contracts.md`; master — `2026-10-22-spatial-3d-native-master.md`.

## Global Constraints

- Это план последующей реализации. Сейчас не выполняются build, tests, simulation, CMake или syntax-only.
- `G0-source` сохраняет исходный planar baseline. Старые kernels не меняют порядок операций ради удобства нового кода. Новый spatial kernel не обязан побитово совпасть с физически иной моделью; различие получает отчёт и критерий.
- `G1-core` — обязательный gate для включения writer в общий runtime. До него разрешены автономные native kernels/fixtures на замороженных contracts.
- G0–G6 обозначают upstream readiness PFC, сейчас unverified; S20–S28 не закрывают их собственными spatial tests. Собственные результаты входят в milestones M* из master.
- Все coordinates — SI, vectors в названных frames. Нормальная нагрузка — скаляр вдоль terrain normal, а не world-Z component. Все силы применяются вместе с соответствующим плечом и couple.
- Force evaluation/probe не мутирует committed state; material state продвигается ровно один раз на accepted physical interval. Нет allocation, исключений и IO в warm core.
- Tire/terrain engine friction не дублирует external tyre writer. Контакты рамы/райдера имеют отдельную collision policy.
- Численные пороги ниже — **предлагаемые критерии приёмки**, а не измеренные свойства текущего снимка. Калибровка и experimental provenance обязательны для заявления о достоверности MTB-физики.

## Общие типы, файлы и тестовая дисциплина

Новые файлы `native/src/pneumatic3d/{geometry,carcass,patches,brush,state,model}.{hpp,cpp}` отвечают каждый за указанную часть. `model` связывает kernels без MuJoCo; `writers/pneumatic_3d_tire.*` связывает модель с engine. Поддерживающий `native/tests/spatial/tyre_fixture.hpp` содержит только аналитические fixtures, native model builder и измерители; он не вызывает Python.

Все перечисленные типы объявляются в этих файлах, а не появляются неявно в тестах:

| Тип | Обязательное содержание |
|---|---|
| `TorusGrid3d` | Неизменяемые размеры, радиусы тора/сечения; отдельно contact fan и full-circle material grid, координаты/веса quadrature, подготовленные local positions; число элементов и ёмкость |
| `WheelKinematics3d` | `spatial::BodyPose`, положение/скорость hub, единичная ось world, угловая скорость hub, relative spin; без предположения world-Y |
| `ContactElement3d` | Устойчивый material-cell index, ray/hit status, точка контакта, normal поверхности, geometric/projected area, проникновение, material ID |
| `NormalElement3d` | Индекс элемента, normal load, поля упругой энергии и диссипативной работы, active mask |
| `Patch3d` | Persistent ID, view индексов элементов, центр давления, orthonormal tangent frame, нагрузка, материал и wrench |
| `PatchWrench` | `point_world_m`, `force_world_n`, `couple_world_nm` вокруг этой точки, `normal_load_n`, tangent components, `surface_id`, valid/identity; семантика C4 |
| `CarcassState3d` | Деформация элементов, Maxwell/hysteresis memory, прежняя поза материала; inactive entries имеют canonical zero |
| `PatchState3d` | Persistent IDs, прежнее overlap mapping, ограниченные free slots, монотонный next-ID |
| `BrushState3d` | Касательные упругие смещения, переносимое camber state, detailed bristles выбранного tier, упругая энергия |
| `TyreState3d` | Версия, topology/config fingerprints, tier/grid sizes, все три состояния, принятый interval index |

`CarcassSpec3d` и `BrushSpec3d` — строго проверяемые resolved коэффициенты с единицами и provenance из S03/S05/S28. Production default нельзя выдумать в unit test. Синтетические коэффициенты tests помечаются `synthetic`, не экспортируются как профиль MTB.

Для каждого test slug создаются `native/tests/spatial/<slug>.cpp`, target `bike_spatial_<slug>`, CTest `spatial.<slug>` и запись в `native/tests/spatial/CMakeLists.txt`; production sources добавляются в корневой `CMakeLists.txt`. Transitional binding graph не становится основным. Использовать `bt::require` из существующего support. Пример будущего запуска:

```bash
cmake --build --preset release --target bike_spatial_tyre_geometry
ctest --test-dir build/native-release --no-tests=error -R '^spatial\.tyre_geometry$' --output-on-failure
```

Build directory берётся из квалифицированного manifest. Эти команды в текущей работе не запускались.

## S20. Тор, контактные элементы и касательные frames

**Depends on:** S06–S08, S10, S13–S15; чистая geometry может разрабатываться раньше model integration.

**Files:** Create `native/src/pneumatic3d/geometry.hpp`, `geometry.cpp`, `native/tests/spatial/tyre_fixture.hpp`, `native/tests/spatial/tyre_geometry.cpp`; Modify root `CMakeLists.txt`, `native/tests/spatial/CMakeLists.txt`.

**Consumes:** `terrain::Terrain3dSource` и bound `Terrain3dQueryView` из S10/C3.2, `spatial::BodyPose`, `TorusGrid3d` и `WheelKinematics3d` по таблице выше. Owner закрепляется cold; view берётся один раз вне element loop, без allocation/refcount в ray loop. Автономный geometry rig не ждёт runtime ownership S11.

**Produces:** `query_geometry(const TorusGrid3d&,const WheelKinematics3d&,const terrain::Terrain3dSource&,std::span<ContactElement3d>) noexcept -> CoreStatus`. Все buffers предоставлены вызывающим кодом; элемент всегда сохраняет собственный material index.

- [ ] Зафиксировать **fan frame**, отделённый от вращающегося материала: axle +Y, down — проекция world−Z на wheel plane, forward=`down×axle`, θ=0 внизу. Параметризация `p=((R+r*cosφ)*sinθ, r*sinφ, -(R+r*cosφ)*cosθ)` относится к этому frame; wheel spin не вращает fan вместе с material. Contact fan занимает θ∈[−75°,75°], φ∈[−70°,70°], не периодичен. Отдельный material grid θ∈[0,2π) периодичен и отображается через signed wheel phase. `R+r=outer_radius`, `2r=casing_width`; вырожденная down-проекция и несовместимый rim/section получают diagnostic.
- [ ] Геометрическая площадь равна `r*(R+r*cosφ)*dθ*dφ`; projected area вычисляется отдельно. Формула `4π²Rr` и объём `2π²Rr²` проверяются **на отдельной full-torus quadrature** с θ,φ по полному2π, а не на active contact fan. Для fan аналитический surface integral равен `r*Δθ*(R*Δφ+r*(sinφ1−sinφ0))`. Старое `R*r*cosφ` не является полной метрикой тора.
- [ ] Лучи выбирают кандидаты контакта; `origin_below_surface`, exhaustion и invalid query сохраняются как невалидная геометрия с причиной. Нельзя давать такому элементу максимальную normal load. Подчасти contact cells интегрируются детерминированно, чтобы включение целого элемента не создавало скачок pressure при первом касании.
- [ ] Получить tangent longitudinal из проекции wheel-forward на tangent plane; lateral = `normal × longitudinal`. При вырожденной проекции использовать объявленный предыдущий valid frame только внутри допуска; серьёзно вырожденное положение завершает физический tick diagnostic. Проверять ориентацию, handedness, continuity и mirror signs.
- [ ] Fixtures: flat, продольный/поперечный slope, yaw 0/37/90°, camber ±5/15/45°, один камень и разделённые пятна. Для каждого сравнить contact point/normal с независимой аналитической поверхностью; полное вращение установки вокруг Z сохраняет wheel-local результат.
- [ ] Будущий `spatial.tyre_geometry`; commit `feat(tyre): add spatial torus contact geometry and frames` с таблицей domain/invalid outcomes.

```cpp
bt::require(std::abs(dot(patch.longitudinal_world,
                        patch.lateral_world)) <= 1e-12,
            "patch tangent axes are orthogonal");
bt::require(std::abs(surface_area - 4.0*pi*pi*R*r) <= 1e-8,
            "torus quadrature has the geometric area");
```

**Acceptance:** Ошибка ортонормированности≤1e−12; расстояние до аналитической плоскости укладывается в допуск S08; ошибка интеграла площади≤1e−8 m² на fixture. Нормировка нагрузки не зависит от числа φ-элементов; geometric/projected areas имеют разные имена в API.

## S21. Нормальная finite-volume law, carcass и давление

**Depends on:** S20; physical coefficient provenance из S03.

**Files:** Create `native/src/pneumatic3d/carcass.hpp`, `carcass.cpp`, `state.hpp` с ранними declarations/versioned lanes, `native/tests/spatial/tyre_carcass.cpp`, `docs/research/spatial-tyre-normal-law.md`; читать, не менять `native/src/pneumatic/carcass.cpp:425–470`.

**Consumes:** `CarcassSpec3d`, contact elements, signed wheel spin, dt и `CarcassState3d`.

**Produces:** `update_carcass(const CarcassSpec3d&,std::span<const ContactElement3d>,double dt_s,bool advance,CarcassState3d&,std::span<NormalElement3d>) noexcept -> CoreStatus`; energy/work lanes с SI units.

- [ ] Записать вывод выбранной spatial law: при постоянном effective pressure потенциал `U_p=p_eff*V_displaced`; при зависимости давления от объёма используется интеграл `U_p=integral(p_eff(V),dV)` с явно выбранной газовой моделью и начальным объёмом. Carcass potential — integral `0.5*k_area*δ_n²*dA_contact`, `k_area` имеет N/m³. Силы и моменты выводятся из изменения этих potentials при rigid displacement, а не из умножения planar force на число lateral rays. Для flat translation интеграл даёт pressure times contact area плюс elastic integral; при вращении учитывается изменяющийся footprint.
- [ ] Построить cold reference quadrature overlap-volume/contact footprint, отдельно от production ray-grid. Production вычисляет аналитические force/torque из той же дискретизации; finite difference potential нужен только oracle. Сравнить виртуальную работу translation/rotation с derivative энергии. Несогласованные area/penetration/force — блокирующая ошибка, даже если vertical load выглядит правдоподобно.
- [ ] Перенести Maxwell primitive со signed material advection по θ. Для линейного ramp использовать `q_new=e^(-dt/τ)*q_old+k*τ*(1-e^(-dt/τ))*δdot`, устойчивую `expm1`, положительные stiffness/τ. Branch storage и losses вычислять до общего clamp; nonadhesion/rim transitions должны иметь явный энергетический учёт.
- [ ] Сохранить отдельные rim-stop, pressure, elastic и rate/hysteresis contributions. Material states следуют материалу при forward/reverse spin; lateral sampling не умножает общее давление. Probe с `advance=false` и `dt=0` оставляет committed bytes прежними.
- [ ] Сохранить исходный flat/no-camber compatibility gate: normal load/deflection относительно старого planar закона ≤2% в зарегистрированном общем pressure/load диапазоне. Spatial law fit и остатки публикуются; если модель не укладывается, задача остаётся открытой либо отдельное reviewed изменение физического требования объясняет причину до принятия. Independent measured-data calibration ≤5% — другой gate и не заменяет прежние2%.
- [ ] Будущий `spatial.tyre_carcass`: compression/release cycle, dt-halving, 2× θ/φ refinement, material rotation/reversal; commit `feat(tyre): add energy-accounted spatial carcass model`.

```cpp
// Синтетический active cell: p=0, A=.01 m², δ=.001 m, k=1e6 N/m³.
bt::require(std::abs(cell.force_n - 10.0) <= 1e-10,
            "foundation stiffness uses area exactly once");
bt::require(std::abs(cell.stored_energy_j - 0.005) <= 1e-12,
            "elastic energy has joule units");
```

**Acceptance:** Относительная ошибка force–potential derivative≤1e−5 вне смены contact topology; изменение reference quadrature при уточнении≤2%. Замкнутый цикл с разгрузкой не создаёт энергию сверх solver tolerance. Старые planar goldens сохраняют прежние байты.

## S22. Connected patches, split/merge и история

**Depends on:** S20–S21.

**Files:** Create `native/src/pneumatic3d/patches.hpp`, `patches.cpp`, `native/tests/spatial/tyre_patches.cpp`; Modify `geometry.hpp` и `state.hpp` по owned-state contract.

**Consumes:** Normal/contact elements с stable material indices и previous `PatchState3d`.

**Produces:** `assign_patches(std::span<const ContactElement3d>,std::span<const NormalElement3d>,bool advance,PatchState3d&,std::span<Patch3d>,std::size_t& count) noexcept -> CoreStatus`; persistent ID не равен индексу текущего output span.

- [ ] Connected components строятся по **непериодической** contact-fan adjacency; два крайних луча fan не соседствуют. Периодичен отдельный full-circle material history grid, чья phase mapping используется при переносе истории. Разные material IDs имеют самостоятельный friction budget; усреднение μ не должно переносить grip с асфальта на грязь. Граница normal discontinuity фиксируется в config.
- [ ] Принадлежность material elements — первичный источник идентичности. Сопоставление old/new components использует deterministic overlap weight по area/load и стабильный tie-break по минимальному element index. При split largest-overlap branch сохраняет ID, остальные получают новые; при merge survivor выбирается тем же правилом.
- [ ] История normal carcass остаётся на material cells. Patch-level relaxation переносится с консервацией соответствующего weighted state/energy; detailed bristles не копируются дважды в обе ветви. Airborne cells проходят объявленный decay/release, не удерживают бесконечный скрытый grip.
- [ ] Вычислить центроид как load-weighted point; суммировать force и moment относительно одного hub origin. Замена нескольких точек одним centroid допустима только вместе с residual couple. Сохранить общий wrench, иначе неровная поверхность меняет axle moment при aggregation.
- [ ] Capacity по умолчанию равна доказанному максимуму компонентов для данного element count; память резервируется cold. Меньший пользовательский limit допустим с preflight bound/явным overflow status. Нельзя оставить первые восемь patches и потерять нагрузку остальных. Failure не меняет старые IDs/state/output publication.
- [ ] Будущий `spatial.tyre_patches`: moving double bump, 1→2→1, material boundary, ring wrap, deliberate overflow, reversed element enumeration oracle; commit `feat(tyre): preserve spatial patch identity and material history`.

```cpp
bt::require(split.total_force_world == before.total_force_world,
            "identity-only split does not change force");
bt::require(std::abs(split.total_stored_j-before.total_stored_j) <= 1e-12,
            "split does not duplicate stored energy");
bt::require(overflow.state_bytes_after == overflow.state_bytes_before,
            "capacity rejection is transactional");
```

**Acceptance:** Fixture с одним переименованием/reindex/split сохраняет точные суммы при фиксированном element order; ошибка общего wrench остальных split/merge cases≤1e−10 с объявленным масштабом. Persistent IDs/next-ID входят в checkpoint. Исчерпание capacity получает отдельную причину и не трактуется как airborne.

## S23. Fast brush: combined slip, camber и passivity

**Depends on:** S21–S22, coefficient contract S03/S05.

**Files:** Create `native/src/pneumatic3d/brush.hpp`, `brush.cpp`, `native/tests/spatial/tyre_brush_fast.cpp`; Create calibration-format documentation в `docs/research/spatial-tyre-tangential-law.md`.

**Consumes:** `Patch3d`, point velocity из S15, wheel-relative spin, `BrushSpec3d`, `BrushState3d`.

**Produces:** `update_fast_brush(const BrushSpec3d&,std::span<const Patch3d>,std::span<const std::array<double,3>> point_velocity_world_mps,double dt_s,bool advance,BrushState3d&,std::span<PatchWrench>) noexcept -> CoreStatus`; `PatchWrench` объявляется в `brush.hpp` по C4, force/couple/point имеют отдельные именованные fields.

- [ ] Определить slip через скорость материала шины относительно неподвижной дороги в patch frame. У нуля скорости интегрируется tangential elastic displacement; большой `max(v,epsilon)` не заменяет статику. В движении convective decay определяется `abs(v_roll)/relaxation_length`; lateral relaxed variable — creepage, его нельзя ещё раз превращать в `tan(alpha)`.
- [ ] Fast law — двухкомпонентный elastic brush с implicit return mapping на friction ellipse. Trial force `f=-K*u_trial`; scaling `s=min(1,1/sqrt((fx/(μxFz))²+(fy/(μyFz))²))`, zero-load отдельно. Возврат меняет elastic state и записывает nonnegative sliding loss, а не только clamp output, оставляя чрезмерную скрытую энергию.
- [ ] Camber вводится signed kinematic source material transport. Для small-angle coefficient `Cγ` предложить `bγ=r_eff*Cγ*sinγ/(Ky*ell_y)`, lateral source `bγ*ωspin`; сопряжённый spin couple `bγ*Fy` включается в wrench/power. Это кандидат reduced law, требующий S28 calibration, а не доказанная универсальная формула. Mirror меняет знак lateral/camber, не normal/longitudinal load.
- [ ] Pneumatic trail задаёт точку действия tangential resultant и соответствующую point velocity; resulting aligning moment учитывается один раз. Mechanical trail остаётся геометрией steer, его нельзя добавить снова как tyre coefficient. После camber и combined slip итог использует тот же ellipse budget.
- [ ] Проверять энергетическое равенство с `ΔU_elastic`, sliding/convective losses и torque power; простой `F·v<=0` отвергает допустимую отдачу упругой энергии. Проверить stop/start/reverse, steady pure slip, combined braking+cornering, camber ±5/15°, zero μ и zero Fz.
- [ ] Будущий `spatial.tyre_brush_fast`; commit `feat(tyre): implement passive spatial fast brush and combined grip` с отдельным перечнем provisional coefficients.

```cpp
const double ellipse = std::pow(out.fx/(mux*fz),2)
                     + std::pow(out.fy/(muy*fz),2);
bt::require(ellipse <= 1.0 + 1e-12,
            "camber and braking share the friction ellipse");
bt::require(out.loss_j >= -1e-12,
            "return mapping does not generate dissipative energy");
```

**Acceptance:** Установившиеся силы конечны при нулевой скорости/reverse; friction inequality≤1+1e−12; mirror error≤1e−10 на синтетических fixtures. Energy closure≤1e−6 относительно масштаба плюс1e−9 J абсолютного допуска на изолированном управляемом цикле. Утверждения о коэффициентах/профиле проверяются в S28.

## S24. Detailed bristles и сопоставление tiers

**Depends on:** S20–S23.

**Files:** Create `native/src/pneumatic3d/bristles.hpp`, `bristles.cpp`, `native/tests/spatial/tyre_brush_detailed.cpp`; Modify `brush.hpp`, `state.hpp`, `docs/research/spatial-tyre-tangential-law.md`.

**Consumes:** Material-cell mapping, pressure/load distribution, tangential velocity каждого cell, тот же `BrushSpec3d` domain и common friction budget.

**Produces:** `update_detailed_brush(const BrushSpec3d&,std::span<const ContactElement3d>,std::span<const NormalElement3d>,const WheelKinematics3d&,double dt_s,bool advance,BrushState3d&,std::span<PatchWrench> element_wrench_world) noexcept -> CoreStatus`.

- [ ] На каждом material element хранить двумерный bristle displacement и elastic energy. Relative slip считать в локальном tangent frame с переносом displacement при вращении frame; простое копирование двух чисел между повернувшимися basis меняет физическое направление памяти.
- [ ] Advection идёт в направлении signed spin по **full-circle material grid**, сохраняет interpolation weights и fixed source order. Непериодический contact fan лишь считывает соответствующую область; его края не wrap. При dt→0 нет продвижения; reverse не обнуляет state без причины. Выход материала из контакта освобождает энергию по явной release/loss модели, исключая перенос в unrelated patch.
- [ ] Использовать local pressure-derived load и local material friction, return mapping каждой bristle; результирующие Fx,Fy,Mz получаются интеграцией. Не добавлять fast-tier aligning/camber correction поверх already-resolved geometry. Если reduced camber parameter остаётся необходим, отдельно документировать место, где отсутствует detailed geometry effect.
- [ ] Сохранить common energy book: per-element storage, sliding, carcass/hysteresis, convective release и rim contributions. Element forces суммировать в фиксированном material order. Patch-only diagnostics не управляют порядком интеграции и не влияют на solver.
- [ ] Сопоставить fast/detailed не требованием побитового equality, а steady curves и transient observables на одинаковой pressure/load/speed постановке. Предлагаемый target ≤10% для Fx/Fy plateau и relaxation length после согласованной калибровки; различия Mz и sharp-roughness response публикуются, а не объявляются шумом.
- [ ] Будущий `spatial.tyre_brush_detailed`: brush conveyor analytic case, forward/reverse, rotating frame, contact exit/reentry, refinement; commit `feat(tyre): add material-resolved spatial brush tier`.

```cpp
bt::require(std::abs(rotated.energy_j - reference.energy_j) <= 1e-12,
            "basis rotation preserves elastic energy");
bt::require(norm(rotated.force_in_reference_frame-reference.force) <= 1e-9,
            "bristle memory rotates as a vector");
```

**Acceptance:** Удвоение разрешения уменьшает гладкую ведущую ошибку хотя бы вдвое; одно вращение basis не увеличивает энергию. Отклонения fast/detailed curves записаны по всей valid domain. Detailed tier служит подробной моделью сравнения; realtime гарантируется отдельно в S53.

## S25. Writer, generalized forces и один владелец контакта

**Depends on:** S11, S13–S17, S20–S24; `G1-core` для runtime integration. S18/S19 потребляют writer позднее и не являются его входной зависимостью.

**Files:** Create `native/src/pneumatic3d/model.hpp`, `model.cpp`, `native/src/writers/pneumatic_3d_tire.hpp`, `pneumatic_3d_tire.cpp`, `native/tests/spatial/tyre_writer.cpp`; Modify `native/src/stepper.{hpp,cpp}`, `runtime/step.{hpp,cpp}`, `model/physical_topology.cpp`, `writers/writer_types.hpp` только append-only fields.

**Consumes:** Validated `spatial::Topology`/`RootBinding`, selected terrain, carcass/brush specs, engine state, caller-owned generalized force span.

**Produces:** `Pneumatic3dTireWriter::try_compute_into(const mjData*,double dt_s,bool advance,std::span<double> qfrc) noexcept -> CoreStatus`, state/probe/diagnostics accessors и `TyreState3d`. Writer class размещён в `bike::pneumatic3d`.

- [ ] Сделать construction cold: bind wheel/hub/body IDs, validate collision masks, size all current/staging/scratch buffers, read-only terrain owner lifetime. MuJoCo tyre hfield contact/friction отключить для spatial external tyre; catch/объектные контакты рамы/райдера остаются по S17. Нельзя отключить все столкновения ради отсутствия double count.
- [ ] Evaluate использует world body poses/velocities и `omega_wheel-omega_hub` на actual axle. Wrench переносится `J_point^T*F+J_rot^T*M`; axle moment равен `dot(axle,cross(point-hub,F)+M)`. Старые world-Y formulas из `runtime/step.cpp:222–234` не переиспользуются для spatial.
- [ ] Встроить writer в existing `TireWriterView`/PhysicalStep dispatch. Probe считает output в staging, advance публикует state ровно один раз; второй advance одного interval возвращает status. Все readers видят один accepted generation. Partial failure любого kernel оставляет committed state без изменений.
- [ ] Добавить explicit source diagnostics: terrain query failure, overflow, nonfinite geometry, invalid tier/config, capacity. `CoreStatus` остаётся компактным hot status; detailed human message строится на cold boundary по сохранённому reason code.
- [ ] Проверить virtual work на произвольных qvel, yaw/roll/steer и rider model: `dot(qfrc,qvel)` равен sum point/couple power. Отдельный тест включает engine collision с заведомо неправильной mask и требует construction rejection; отсутствие engine contact после отключения подтверждается runtime fixture.
- [ ] Будущий `spatial.tyre_writer` плюс planar golden regression required gate; commit `feat(runtime): integrate spatial tyre writer through owned stepper`.

```cpp
bt::require(std::abs(dot(qfrc, qvel)-wrench_power_w) <= 1e-8,
            "generalized force preserves virtual work");
bt::require(state_before_probe == writer.state_bytes(),
            "force probe is observational");
```

**Acceptance:** Масштабированная ошибка virtual work≤1e−9; warm allocations=0; дублирующий engine tyre contact отсутствует. Повторные оценки на том же binary совпадают; telemetry подтверждает применение нового writer в constrained/full spatial. Planar goldens не меняются.

## S26. Статика тормоза, contact anchors и общий grip

**Depends on:** S23–S25; standalone brake/constrained fixture из S14/S17. S18 потребляет результат позднее; координация с владельцем dirty brake fixes PFC в S01/S02.

**Files:** Create `native/src/spatial/brake_state.hpp`, `brake_state.cpp`, `native/tests/spatial/brake_static_3d.cpp`; Modify `native/src/runtime/static_brake.{hpp,cpp}`, `runtime/step.{hpp,cpp}`, `runtime/accounting.{hpp,cpp}` только spatial-selected path.

**Consumes:** Wheel hinge addresses, brake demand/ceiling, terrain tangent/normal frames, accepted `TyreState3d` и mutable model frictionloss state.

**Produces:** `spatial::BrakeState` с front/rear latch validity, unwrapped hinge angle reference, elastic storage, previous demand, interval index; `capture_spatial_brake()`/`restore_spatial_brake(const BrakeState&)` на existing brake owner.

- [ ] Сначала сверить exact baseline `static_brake.cpp` с dirty-снимком. Предоставленный код latch привязывает к wheel angle; не предполагать, что будущий PFC patch с root-X/world-anchor уже прошёл review. Для 3D internal brake фиксирует угол колеса относительно hub, не положение рамы в мире.
- [ ] Назначить один общий cap `|τ_frictionloss+τ_hold|<=ceiling*demand`. Согласованный путь: при engaged hold reserved spring/damper moment уменьшает оставшийся frictionloss budget; при release вся ёмкость снова доступна friction. Sign и saturation правила проверяются на обоих направлениях вращения.
- [ ] Ground stiction принадлежит tangential brush; его anchor связан с material contact и неподвижной world surface. Перенос patch frame не переводит anchor вместе с колесом. На bank/slope constrained wheel brake и ground traction используют одну friction ellipse; новый world spring от root к точке запрещён.
- [ ] Отделить brake elastic storage `0.5*k*(theta-theta_latch)^2`, damping/friction heat и release work. Нельзя заносить всякую отрицательную hold power в loss, затем повторно учитывать освобождённую spring energy. Tire loss не содержит повторно brake-hinge work.
- [ ] Fixtures: поворот всей установки bike+sloped plane вокруг world-Z на0/90°; stop/start/reverse; release без скачка world force; bank с lateral gravity; demand выше/ниже traction limit. Статику проверять на constrained test rig, который блокирует опрокидывание без продольной опоры: slope5° и μ=.8 удерживаются, а tan(slope)>μ вызывает скольжение независимо от brake ceiling. Свободный стоящий велосипед — отдельная задача баланса.
- [ ] Будущий `spatial.brake_static_3d`, restart while latched and release; commit `fix(spatial): separate internal brake bite from ground traction` без скрытой смены planar baseline.

```cpp
bt::require(std::abs(tau_friction+tau_hold) <= ceiling*demand+1e-12,
            "brake components share one torque ceiling");
bt::require(restarted.latch == uninterrupted.latch,
            "checkpoint preserves an engaged brake bite");
```

**Acceptance:** На допустимом5° slope drift≤1 mm/10 s при объявленном dt/config; недопустимый slope не «приклеен» к миру. Energy residual≤0.1% масштаба входной/освобождённой энергии плюс1e−6 J; поворот всей установки меняет только мировое представление. Это критерии будущей проверки, не текущие измерения.

## S27. Версионированный tyre-state codec и atomic restore

**Depends on:** S17, S21–S26; общий state contract S02/S04. S19 интегрирует codec позднее и не является его входной зависимостью.

**Files:** Create `native/src/pneumatic3d/state.cpp`, `native/tests/spatial/tyre_state.cpp`; Modify `native/src/pneumatic3d/state.hpp` из S21, `model.{hpp,cpp}`, `native/src/runtime/bootstrap_io.cpp`, `runtime/step.{hpp,cpp}`, `initialization/capture.cpp`. S21–S25 создают/расширяют declarations; S27 завершает строгий codec и restore.

**Consumes:** Complete `TyreState3d`, matching terrain/config/topology fingerprints и existing bootstrap serialization boundary.

**Produces:** `encode_tyre_state(const TyreState3d&) -> std::vector<std::byte>`; `decode_tyre_state(std::span<const std::byte>,const TyreStateIdentity3d&) -> std::expected<TyreState3d,std::string>`; `TyreStateIdentity3d` содержит schema/tier/grid/capacity и fingerprints, объявляется в `state.hpp`.

- [ ] Перечислить serializable lanes и порядок: carcass branches, deformation/history, material advection phase, patch IDs/mapping/free slots, brush displacement/energy, tier, accepted interval, config/topology/terrain fingerprints. Brake state находится рядом в complete runtime checkpoint, но не дублируется как скрытая tyre lane.
- [ ] Header содержит `pneumatic3d_state=1` и lengths; inactive slots canonical zero и explicit valid masks. Не кодировать NaN как inactive marker. Endianness/scalar width фиксированы; raw C++ padding, pointers, engine addresses и unordered map order не записываются. Bootstrap2 wrapper следует C7.0: ровно `pneumatic3d_state,identity,encoding,payload`, encoding canonical padded base64 без whitespace; IDs не переводятся в JSON double.
- [ ] Decode сначала проверяет version, sizes/overflow, fingerprint, finite numbers, ID uniqueness, valid ranges, consistency stored energy/history. Только полностью validated temporary state можно publish. Отказ не меняет live writer/model/energy counters; unknown future schema не читается по старой ширине.
- [ ] Restore переводит model mutable inputs и accepted generation в согласованное состояние, invalidates stage cache по existing freshness protocol. Не вызывает `reset()` latch или material memory: такой reset относится только к fresh-episode semantic command.
- [ ] Continuation fixture захватывает state mid-corner и при combined braking после split/merge, затем сравнивает 1000 accepted steps с непрерывным потоком на том же binary. Отдельно corrupt byte, tier mismatch, changed terrain material, truncated last section: transactional refusal.
- [ ] Будущий `spatial.tyre_state`, commit `feat(state): serialize complete spatial tyre memory transactionally`; recording owner получает schema specification для S42–S44.

```cpp
bt::require(encode_tyre_state(*decode_tyre_state(bytes, identity)) == bytes,
            "canonical tyre state round-trips exactly");
bt::require(resumed.accepted_states == continuous.accepted_states,
            "same-build restart retains full material history");
```

**Acceptance:** Round-trip/continuation bytes на том же binary совпадают; любой несовместимый/повреждённый payload отклоняется до публикации. Raw render snapshot не принимается как restart state. Cross-platform exact equality отдельно не обещается и не подменяет physics qualification.

## S28. Физическая и калибровочная квалификация шины

**Depends on:** S20–S27; полный runtime gates G1/G2 нужны только для integrated часть matrix.

**Files:** Create `native/tests/spatial/tyre_qualification.cpp`, `native/tests/spatial/data/tyre_profiles.json`, `docs/research/spatial-tyre-qualification.md`; Modify `native/src/configuration/{physical_config.hpp,schema.cpp,validate.cpp}` для final supported profile registry.

**Consumes:** Fast/detailed native tyre, real tyre spec, data provenance, valid operating domain и all isolated fixture results.

**Produces:** Versioned coefficient profile со статусами `measured/calibrated/inferred/provisional` по семействам коэффициентов, canonical digest, независимый qualification report и capability flag; отсутствие измерений оставляет профиль provisional.

- [ ] Сохранить actual tyre identity из snapshot: front Magic Mary 29×2.4 и rear Hans Dampf 27.5×2.4, соответствующие radii/pressure из config. Не заменять их молча 2.8-дюймовой шиной из прежнего обсуждения. У measurement dataset записать load, pressure, casing, surface, speed, temperature и camber/slip domain.
- [ ] Сформировать matrix: normal load 100/300/600/900 N, pressure 1.0/1.5/2.0 bar, speed 0/.1/1/3/8 m/s, κ 0/±.02/±.1/±.3, alpha 0/±1/±3/±6°, camber 0/±5/±15/±30°. Это coverage proposal; outside measured domain пометить exploratory и не включать в calibrated accuracy claim.
- [ ] Fit использует одну часть measured curves, acceptance — withheld loads/pressure/angles. Заявленные targets: Fz-deflection RMS≤5%, lateral/longitudinal force RMS≤10% of peak, aligning moment≤15%, relaxation length≤15% в supported domain. В dataset без Mz/relaxation соответствующий claim не закрывается.
- [ ] Исследование TU Delft/Zenodo city/cargo полезно для benchmark signs и формы, но его 300–500 kPa, ±5° camber и другие tyres не подтверждают MTB при 1–2 bar и 30° camber. Зафиксировать primary links в report: [публикация TU Delft](https://repository.tudelft.nl/record/uuid:8238f5ba-6505-48ac-90cc-4a679be3a670), [исходный dataset Zenodo](https://zenodo.org/records/7866646).
- [ ] Отдельно от fit проверить conservation, passivity, mirror/yaw invariance, μ→0, pressure/load boundary, step-halving, grid refinement, split/merge, start/stop/reverse. «Хороший график» не отменяет невалидный contact query или незаписанную историю состояния.
- [ ] Будущий `spatial.tyre_qualification`; положить resolved inputs/expected metrics/tolerances/provenance в native artifacts и commit `test(spatial): qualify tyre laws and publish coefficient domains`. Не менять gate после просмотра результата без reviewed rationale.

```cpp
bt::require(report.invalid_cases == 0, "all qualified samples remain valid");
bt::require(report.lateral_force_rms_fraction <= 0.10,
            "withheld lateral curves meet the declared profile target");
bt::require(profile.status != "measured" || report.source_measurements > 0,
            "provenance cannot promote invented coefficients");
```

**Acceptance:** Численно квалифицированный provisional profile можно использовать для разработки и stability tests с явной маркировкой. Формулировка «достоверная MTB-шина» требует независимых measured-data criteria; отсутствие данных остаётся ограничением spatial final qualification S54. Самосогласованность симуляции не заменяет экспериментальную проверку. Это отдельно от upstream G6 readiness PFC.

## Handoff

S25 передаёт writer и telemetry владельцу runtime; S27 — complete state schema владельцу recording/replay; S28 — capability и calibration domain владельцу qualification. Startup/balance-фаза не переписывает tyre constitutive law ради того, чтобы контроллер прошёл заезд. Каждый физический defect получает свой reproducible fixture и отдельное reviewed изменение.
