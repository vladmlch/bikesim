# Статическая перепроверка W00–W55 — 24 октября 2026

## Результат и граница проверки

Подготовлен единый патч с исправлениями подтверждённых по исходникам ошибок
W00–W55, соответствующими регрессионными исходниками и уточнениями контрактов.
Проверка охватывает собственные реализации spatial migration, их вызывающий
код, сериализацию, тестовые контуры и интеграцию с приложениями.

**Код проекта не исполнялся.** Не выполнялись configure, компиляция, линковка,
compiler syntax-only, тесты, симуляции, benchmarks, GUI, установка или проектные
скрипты. Добавленные тесты написаны и прочитаны статически. Они не имеют статуса
passed. Выполнялись операции с файлами, чтение и редактирование текста,
разбор JSON как данных, проверка diff и применимости патча.

Обнаруженные ошибки и исправления перечислены ниже. Этот результат не является
доказательством отсутствия других ошибок, физической квалификацией модели или
подтверждением собираемости на целевой платформе.

### Исходный снимок

| Параметр | Значение |
|---|---|
| Архив | `repomix-output.xml(20261024-063218).zip` |
| SHA-256 архива | `f13f0db77151ad732fc18b2702ee56a8bd8a844cea810da4546bbaaf2c659597` |
| SHA-256 XML | `48fa516b08f057ee464bd8f3f7149f28f659f32a4b1acff9ef8b1a23a96f3604` |
| Извлечено файлов | 2581 |
| База патча | Содержимое этого снимка, до настоящих исправлений |
| Концы файлов | Стандартный завершающий LF восстановлен после извлечения у 2559 непустых файлов одинаково в неизменяемой базе и рабочей копии |

Перед извлечением проверены уникальность и безопасность относительных путей.
История исходного Git-репозитория в этот снимок не входит; локальная база diff
не объявляется проверенным upstream commit. Патч имеет обычные пути `a/...` и
`b/...` относительно корня проекта. В него не входят временные файлы распаковки
и рабочие отчёты отдельных проверяющих.

## Как был разделён обзор

Работа шла последовательными узкими пакетами. У каждого пакета были конкретные
production owners и ближайшие тесты; общие CMake, журналы контрактов и итоговый
патч оставались у интегратора. После первичных исправлений отдельно проверялись
стыки и изменения другого автора.

| Задачи | Проверенные границы | Результат |
|---|---|---|
| W00–W05 | Frozen declarations, config/wire, capability dispatch, registry и CMake ownership | Прослежены новые публичные изменения и их потребители; необходимые отклонения внесены в C10 journal. |
| W06–W08 | Grid factory/codec, Catmull interpolation, bounded ray search | Исправлены обход factory invariants и три ошибки ray query. |
| W09–W12 | Genspec, generation, extrusion, query view, render raster | Исправлены slope validation, shallow ray, default view и лимит памяти. |
| W13 | Joint/root bindings, tree ancestry, body pose | Усилены требования к spin/steer joints и расположению колёс. |
| W14–W16 | Model, drivetrain Jacobians, rider support/attachments/shoulders | Исправлены нормализация support frame и распределение spring/damping work. |
| W17–W19 | Bootstrap, reset/publication, causal state, planar bridge и negatives | Исправлена generation при reset; проверены реальные qpos/dof адреса и отрицательные контракты. |
| W20–W24 | Geometry, carcass, patches, fast/detailed brush | Исправлены camber law, energy reductions, contact exit и некорректный material-index cast. |
| W25–W28 | Tyre owner/writer/state, brake, qualification fixtures | Исправлены power pairing, aggregate validation, telemetry и ненагруженный camber fixture. |
| W29–W31 | Equilibrium, hold, startup, optional cache/import | Исправлены detailed contact accounting, nonfinite residuals, corridor guard и cache fallback. |
| W32–W33 | Steering clock/FIFO, outcomes | Отказ нового steering tick больше не скрывается; устранена потенциальная аллокация в noexcept outcome poll. |
| W34–W36 | Meijaard/Whipple reference, ideal corner, mirror, timestep, target rides | Исправлен Euler roll-rate в круговом reference; квалификационные ограничения сохранены явно. |
| W37–W40 | IMU, SensorPipelineV2, policy ABI2, truth, quality/metrics | Исправлены aliasing при загрузке ABI2 и учёт невалидного последнего интервала. |
| W41 | Catalog, nested descriptors, transition producer/encoder | Schema2 приведена к реальному spatial truth; schema1 не изменена. |
| W42–W43 | Accounting continuation, checked replay, reconstruction, publication | Исправлены порядок causal rows, противоречивые buffered data, initial rollback, generation и reconstruction identity. |
| W44 | Golden capture/replay, startup comparison, platform registry | Полное восстановление causal checkpoint заменило смешивание fresh и reference state. |
| W45 | CLI resolver и TOML/asset overrides | Исправлены конфликт planar flags с загруженным spatial config и остаточные generator settings. |
| W46–W49 | Stored playback/viewer, reports, batch, install declarations, examples/docs | Исправлены интерполированная spatial pose, provenance потоков и исчезавшие unavailable интервалы на графиках. |
| W50–W55 | Timing/report eligibility, worker lifecycle/reductions, engine-pool probe, qualification ledger, isolated GPU decision | Исправлен false-positive performance target для несопоставимых повторов; измерения и решения не выдумывались. |

Дополнительный независимый проход охватил runtime spring/quality/reset fixes,
полный golden restore и четыре малых изменения CLI/outcome/topology/performance.
В процессе исправлены также ошибки новых fixtures: недопустимое parser value
для `--scenario` и попытка создать production terrain с нулевым Stribeck speed.

## 1. Terrain и геометрия

### R01. Пустые объекты обходили обязательную валидацию

`Grid3d` и `ProfileExtrusion3d` имели неявный публичный default constructor.
Их быстрые запросы используют размеры, соседние узлы и `front()/back()` как
уже проверенные factory invariants. Пустой объект нарушал эти предпосылки.
Default constructors сделаны private; создание через `create()` и копирование
проверенных объектов сохраняются. У `Terrain3dQueryView` указатели теперь
инициализируются null; обычный default-initialized view безопасно отвечает
`invalid_input`.

Основание: `terrain/grid3d.hpp`, `extrusion3d.hpp`, `source3d.hpp`; исходники
регрессий: `terrain_codec.cpp`, `terrain_extrusion.cpp`.

### R02. Bernstein search мог пропустить две внутренние точки пересечения

Нулевую производную на конце интервала ошибочно считали достаточным основанием
проверять только этот stationary point. Контрпример с точными коэффициентами:

`p(s) = (s-1/4)(s-3/4)(s+3/16) = 9/256 - (13/16)s² + s³`.

Оба конца положительны, `p'(0)=0`, первая точка пересечения — `1/4`. При грубом
локальном tolerance старый путь мог выдать `miss`. Теперь stationary endpoints
отделены от внутреннего корня производной; знаки непосредственно внутри
интервала определяются через ненулевые Bernstein differences. Ограниченный
subdivision сохраняется.

Основание: `terrain/raycast3d.cpp::earliest_root_bernstein`; регрессия:
`terrain_raycast.cpp`.

### R03. Почти вертикальный луч мог получить фиктивный hit за пределами grid

Порог `abs(dx),abs(dy)<1e-15` отбрасывал реальное перемещение XY на длинном луче.
Впоследствии clamping координат превращал выход из конечного домена в hit на
его краю. Быстрый vertical path теперь разрешён только при точном нуле XY;
остальные лучи проходят slab clipping. Начало точно на поверхности также
обрабатывается одинаково для направленного вверх луча и остальных направлений.

Основание: `terrain/raycast3d.cpp::ray_cast_grid`; аналитические finite-domain
и `t=0` controls добавлены в `terrain_raycast.cpp`.

### R04. Генератор проверял узлы, но не реальный уклон между ними

Последовательность высот `0,0,0.08,0.08` при шаге 0.05 м имеет соседний уклон
1.6, однако Catmull derivative достигает 2 внутри ячейки и превышает
`tan(60°)`. Добавлена холодная проверка фактического bicubic gradient через
Bernstein bounds и детерминированное деление. Недоказанный bound отклоняется
после depth 8 или 256 subdivisions на ячейку. Высоты и нормали не зажимаются.
Одна ограниченная heap workspace не перегружает production stack.

Основание: `terrain/generator3d.cpp` и коэффициенты `interpolation3d`;
регрессии: 80 мм edge должен отклоняться, 60 мм control требует корректного
subdivision и остаётся допустимым (`terrain_generator.cpp`).

### R05. General extrusion ray терял пологие пересечения

Абсолютный cutoff детерминанта зависел от длины сегмента и отбрасывал реальный
hit пологого 3D-луча. В ветке `dy!=0` отклоняется точная параллельность.
Специальная planar bridge ветка `dy==0` сохраняет прежний порядок арифметики.
В `terrain_extrusion.cpp` добавлен независимый flat-plane control.

### R06. Ограничение render raster не учитывало второй буфер

Guard учитывал только float raster, хотя одновременно выделяется полный double
buffer. Теперь перед выделением проверяются произведения размеров и суммарные
байты обоих буферов. Основание: `terrain/render3d.cpp`; отрицательный размер
проверяется в `terrain_startup.cpp` до большой аллокации.

### R07. Production tyres не могли использовать extrusion material registry

Extrusion уже хранил проверенные материалы и возвращал их IDs. Query view не
передавал таблицу потребителю; код шин приводил context к `Grid3d` и запрещал
любой extrusion. Добавлены immutable span и lookup по ID, привязанные к тому
же immovable source owner. Fast/detailed tiers используют фактические
коэффициенты, поддерживают sparse unsorted IDs и отклоняют неизвестный активный
ID без material commit. Пустая таблица отвергается при создании модели.

Основание: `terrain/source3d.*`, `extrusion3d.*`, `pneumatic3d/model.cpp`,
`bristles.cpp`, `spatial/config.cpp`. Регрессии в `tyre_state.cpp` используют
реальный extrusion, IDs 7/42 и разные коэффициенты трения для обоих tiers;
сила проверяется по независимому saturation law `Fx=-mu*Fz`.

## 2. Физические силы, энергия и topology

### R08. Неправильные wheel/steer trees проходили binding

Spin body теперь обязан владеть только своим scalar spin joint; аналогичное
условие применяется к steering body. Front hub должен лежать под steering
body, rear hub — вне этой ветви. Промежуточные тела вилки разрешены.
Основание: `spatial/topology.cpp`; отрицательные trees добавлены в
`test_topology.cpp`, валидная topology identity сохраняется.

### R09. Допущенный quaternion порождал frame, отвергаемый собственным codec

`rider/spatial_support.cpp` принимал малую ошибку нормы quaternion, но вращал
векторы ненормализованным значением. Полученный support frame мог нарушить более
строгий orthonormality guard при сериализации. Нормализуется локальная копия
после входного gate; raw integration bytes не меняются. Регрессия в
`test_rider.cpp` включает настоящий attachment encode/decode roundtrip.

### R10. Shoulder spring ошибочно учитывалась как диссипация

Полная passive force rider joints попадала в `rider_passive_damping`, тогда как
stored energy уже содержала плечевую пружину, а engine loss path возвращал её
консервативную работу. Теперь при spatial partition spring force остаётся в
`engine_passive`, используя qpos начала интервала; rider row содержит остаток,
включая демпфирование. Суммарная физическая generalized force сохраняется.

Основание: `runtime/step.cpp::solve_tail` и energy consumers; регрессия
`rider_spring_is_not_material_loss` в `startup_paths.cpp` проверяет `-c*v`,
`-k*(q-q_spring)` и их сумму. Диагностический `solved_passive_nm` по-прежнему
означает полное signed passive effort; уточнён комментарий контроллера.

### R11. Camber source имел неверную размерность и зависел от разбиения

При normalized `C_gamma` в выражении отсутствовал `Fz`. Для подробной модели
деление элемента на две равные части удваивало суммарный camber response.
Исправленная формула:

`b = r_eff*C_gamma*Fz*sin(gamma)/(Ky*ell_y)`.

`b` имеет размер длины, `b*omega` — скорости, `b*Fy` — момента. Кроме того,
spring source теперь использует абсолютную wheel angular velocity вдоль оси:
couple прикладывается через wheel Jacobian, поэтому относительного spin
недостаточно при вращении carrier.

Основание: `pneumatic3d/brush.cpp`, `model.cpp` и writer Jacobian mapping;
регрессии `tyre_brush_fast.cpp` / `tyre_brush_detailed.cpp` проверяют
full/half element invariance и работу couple при нулевом relative spin.
Это исправление закона и сопряжения мощности, без утверждения о калибровке.

### R12. Contact exit и aggregate overflow нарушали material state

Fast standalone wrapper очищал spring displacement при исчезновении normal
load, не учитывая released energy. Теперь release добавляется в cumulative
loss и returned wrench loss. Production owner продолжает отдельно учитывать
исчезновение/изменение площади patch по сохранённой membership.

Brush/carcass validation также проверяет полные reduction и cumulative sums
до первой записи состояния; finite per-element values больше не могут дать
успешный commit с бесконечной aggregate energy. Model и state restore проверяют
суммы по обоим колёсам. Отрицательные controls требуют сохранения прежних bytes.

Основание: `pneumatic3d/{brush,bristles,carcass,model,state_detail}`; тесты
`tyre_brush_fast`, `tyre_brush_detailed`, `tyre_carcass`, `tyre_state`.

### R13. Некорректная геометрия и cold coefficients отклонялись слишком поздно

Добавлены проверки finite torus coordinates/area до float-to-integer material
index conversion; patch reductions проверяются на overflow и вырожденные axes
до ID/membership commit. Отрицательный pneumatic trail теперь отвергается в
constructor, как уже требовал kernel. Writer проверяет reduced axle moment
до material advance. Query invalid input больше не маскируется диагностикой
вырожденной проекции направления.

Основание: `pneumatic3d/{geometry,patches,model}.cpp`,
`writers/pneumatic_3d_tire.cpp`; регрессии в соответствующих tyre test sources.

### R14. Support telemetry и qualification fixture были неточными

`support_n` теперь равен `sum(Fn*normal_z)`: прежняя сумма total force Z включала
вертикальную часть тангенциальной силы. Полный world force сохраняется отдельно.
Cambered-load fixture использует vertical extent `R*cos(gamma)+r` и явно
проверяет, что bisection bracket действительно охватывает заданную нагрузку.
Старый upright bracket мог вообще не касаться поверхности при большом camber.

Основание: `writers/spatial_tire_adapter.cpp`, `tyre_qualification.cpp`.
Для synthetic zero-width Stribeck boundary detailed kernel приведён к fast
семантике `mu_peak` при нулевом slip; production terrain по-прежнему требует
положительный transition speed.

## 3. Startup, controller и исследовательский runtime

### R15. Detailed equilibrium отвергалось из-за штатных inactive slots

Detailed output содержит не только нагруженные элементы. Отчёт ошибочно
требовал valid wrench у каждого fan slot. Теперь `contacts_total=2` означает
два требуемых колеса; `contacts_valid` считает поддержанные колёса, а баланс
сил/моментов суммирует все active wrenches. Nonfinite residuals и моменты
отклоняются явно; worst-DOF sorting имеет корректный порядок и при NaN/Inf.

Основание: `initialization/spatial_equilibrium.*`, `equilibrium.cpp`;
регрессии: `equilibrium.cpp`, `startup_paths.cpp` в spatial tests.

### R16. Готовый grid мог не покрывать scored corridor

Проверка half-width не учитывала смещение Y-domain у программно переданного
terrain owner. Теперь реальный диапазон должен покрывать
`[-corridor_half_width,+corridor_half_width]`. Основание:
`initialization/setup_owner.cpp`; shifted-grid negative в `startup_paths.cpp`.

### R17. Физически плохой optional cache прерывал setup

Структурно корректный cache мог иметь непригодную позу; physical validation
происходила уже за пределами fallback catch. Теперь эта проверка входит в
cache attempt, а при отказе восстанавливается полный fresh engine/model/
material/causal inventory и выполняется обычный fresh path. Явный import
остаётся строгим. Регрессия изменяет согласованно qpos/integration_state и
mutable friction, проверяя fallback и отдельный explicit-import rejection.

Основание: `initialization/equilibrium.cpp`; `startup_paths.cpp`.

### R18. Hold validation и commit складывали силы в разном порядке

Последовательное добавление force/couple к существующему `qfrc` могло переполнить
промежуточное значение, хотя preflight проверял сначала их сумму. Commit в
`initialization/bootstrap_hold.cpp` теперь выполняет ту же grouping операции,
которую проверил guard.

### R19. Новый rejected steering tick мог оставить устаревшую команду

После tick production owner проверяет, что очередь приняла текущий номер,
используя уже валидированный integer control clock. Отказ из-за нечислового
входа/переполнения проходит failure path. Намеренное повторное чтение уже
принятого tick сохраняется: W35 reference helper заранее подаёт круговую цель
и не должен повторно потреблять FIFO. В этом helper также исправлен roll-rate:
world omega X заменён правильным Euler rate из body angular velocity.

Основание: `runtime/step.cpp`, `rider/steer_balance.cpp`,
`tests/spatial/support/ride_qualification.hpp`.

### R20. Outcome poll и reset publication имели нарушения контракта

`evaluate_outcome` возвращает `const Outcome&` на caller-owned latch, устраняя
копирование string внутри `noexcept`. Текущие потребители не переживают owner.
При runtime reset новая generation одновременно публикуется в boundary и
spatial presentation; раньше первый render frame после reset имел разные
поколения. Source assertion проверяет frame до следующего advance.

Основание: `spatial/outcomes.*`, `runtime/runtime.cpp`, `startup_paths.cpp`.

### R21. Плохой sample увеличивал valid prefix; terminal скрывал quality

Численное качество spatial sample теперь определяется до накопления valid
duration/distance. Максимальный residual обновляется и на физически terminal
интервале. Такой исход сохраняет свой physical reason, но плохая quality
устанавливает `numerically_valid=false`, `terminated=false`, `truncated=true`.

Основание: `runtime/research.cpp::consume_sample`. Две регрессии в
`startup_paths.cpp` используют намеренно смещённый accounting baseline и один
реально принимаемый интервал: отдельно requested completion и nonterminal
numerical-quality stop. Это отрицательные controls, не физические эталоны.

### R22. Загрузка policy ABI2 нарушала C++ aliasing rules

Запись через `reinterpret_cast<void**>(&function_pointer)` заменена копированием
представления через `memcpy` после существующей проверки размеров. Основание:
`research/policy_v2_host.cpp`. ABI1 не изменён.

## 4. Recording, checkpoint и replay

### R23. Schema2 transition truth не соответствовала producer

В catalog оставались 16 planar fields, тогда как реальный SpatialTruth producer
выдаёт 21 пространственное поле. Это могло ломать encoder настоящего research
transition. Resource JSON и embedded catalog исправлены вместе. Recursive
field-catalog projection теперь сохраняет nested name/type/classification/
nullable/presence. Schema1 resource и embedded bytes совпадают с базой.

Основание: `recording/schema.cpp`, `catalog_v2_data.inc`,
`resources/recording/catalog-v2.json`; `recording_catalog.cpp` использует
реальный spatial transition producer и encoder.

### R24. Canonical JSON переставлял causal rows после checkpoint restore

Constraint powers и raw attachments сохранялись object fields; canonical
sorting менял порядок reduction и diagnostic selection. Новая запись использует
ordered arrays `{name,value}`. Decoder принимает также старую object форму
в сохранившемся порядке. Уже потерянный исторический порядок восстановить нельзя.
Регрессия использует неассоциативную сумму `1e16, -1e16, 1` и проверяет порядок
attachment failure.

Основание: `runtime/accounting_checkpoint.cpp`, `tests/spatial/checkpoint.cpp`.
Там же до commit отвергаются противоречия constraint velocity/raw qvel,
ownership flags/energy и published sample clock/velocity.

### R25. Первый checked-replay mismatch не имел корректного rollback frame

Verified frame теперь захватывается сразу после проверки initial state.
Generation согласованно проставляется в header, spatial pose и wire при
создании, выдаче, rollback и restart. Основание: `replay/session.cpp`;
регрессии: `checked_replay.cpp`.

### R26. Replay reconstruction меняла проверенную identity

Отключение optional equilibrium cache для реконструкции меняло resolved config
hash и впоследствии конфликтовало с embedded continuation. После проверки
физической конфигурации возвращается исходная metadata identity. Spatial
reconstruction получает полное состояние из embedded checkpoint и не зависит
от наличия внешнего initial-state файла. Metric origin берётся из принятой
initial presentation; initial research state обязан соответствовать t=0/step=0.

Основание: `replay/rebuild.cpp`, `session.cpp`; controls в `checked_replay.cpp`.

### R27. Golden replay смешивал два различных причинных состояния

Старый путь делал reset и накладывал только reference qpos/qvel/act/ctrl/
warmstart, оставляя fresh tyres, brake, steer FIFO и accounting. Теперь полный
reference checkpoint предварительно восстанавливается в отдельном owner.
Все шесть redundant initial arrays побитово сверяются с ним до reset; model/
stepper references и guard берутся после swap.

Фактическое начало trajectory сохраняется в `spatial_checkpoint_initial`.
Fresh startup остаётся отдельным `spatial_startup_candidate`, чтобы восстановление
reference не скрывало startup regression. Reviewed cross-platform comparison
считает собственную coherent local trajectory. Publication generation
согласуется с reference при cold rebase, поэтому повторное сравнение на том же
host не даёт ложной ошибки только из-за числа предыдущих reset. Сравнение
numeric/counter fields не отключается.

Основание: `golden/capture.cpp`; `golden_registry.cpp` содержит malformed
init/material preflight controls, настоящий detailed material/controller
continuation и repeated matching comparison. Target помечен `slow`, поскольку
теперь включает настоящий cold startup.

## 5. CLI, отображение, отчёты и performance

### R28. Загруженный spatial TOML принимал конфликтующие planar flags

Конфликт проверяется после определения фактического режима, включая загруженный
config. При CLI-замене genspec на immutable `.terrain3d` очищаются inherited
`dx_m/dy_m`. Regression parsing вынесен до ожидаемого отказа resolver;
`--scenario` получает допустимое `flat`, а проверяется именно spatial conflict
diagnostic. Основание: `cli/configuration.cpp`, `tests/spatial/cli.cpp`.

### R29. Stored playback показывал root другой временной фазы

Интерполированные qpos сочетались с spatial presentation левого keyframe;
renderer затем возвращал free root в старую позу. Speed/pitch искались по
несуществующим scalar root joints и становились нулевыми. Теперь pose и
heading-relative speed выводятся из сохранённых интерполированных coordinates
через kinematics-only caches и XBODY velocity. Contact/steering HUD остаётся
записанным evidence. Euler singularity очищает optional angles.

Основание: `replay/playback.*`; `viewer_pose.cpp` проверяет free6,
antipodal quaternions, constrained4 с ненулевыми addresses/world offset и
неизменность сохранённых координат. К test target добавлена `bike_replay`.

### R30. Недоступные интервалы исчезали с графика

Invalid samples корректно удалялись из line series, но auto bounds строились
только по оставшимся точкам. Trailing gap и N/A marker оказывались за осью;
all-invalid panel получала произвольный диапазон. Time plots теперь сохраняют
весь принятый временной диапазон. Регрессия проверяет и metadata plot, и наличие
видимой N/A подписи в native SVG. Основание: `viz/spatial.cpp`,
`tests/spatial/reports_batch.cpp`.

### R31. Batch provenance и performance eligibility давали ложные сведения

Batch identity теперь содержит resolved `setup.runtime.tyre_threads`, а не
постоянную 1. Число процессов batch scheduler остаётся отдельным параметром.
Performance target требует `repeated_comparable`: различающиеся terminal
classifications больше не могут совместно дать `target_met=true`.

Основание: `batch/plan.cpp`, `runtime/realtime_report.cpp`; регрессии:
`reports_batch.cpp`, `performance_report.cpp`.

### R32. Документация содержала дубли и устаревший CLI статус

Удалены вторые точные копии spatial terrain разделов в `docs/RIDE.md` и
`docs/TESTING.md`. `docs/spatial-3d.md` описывает уже реализованный `--threads`
с default 1 и диапазоном 1–64 для tyre workers.

## Совместимость и незакрытая квалификация

Точные изменения frozen declarations, numerical law и continuation
representation перечислены в `migration/spatial3d/contract-deviations.md`.
В частности, query-view layout и return type outcome требуют согласованных
пересобранных потребителей при последующей локальной проверке. Decoder новых
checkpoint rows читает старую форму; совместимость старых binaries с новой
записью не утверждается.

Остаются предметом самостоятельной квалификации:

- Физическая калибровка lateral/camber coefficients, passivity/energy closure,
  mirror/rotational covariance и динамическая устойчивость.
- Уже задокументированные C10 ограничения fractional material deposition и
  переноса состояния между меняющимися tangent bases. Этот патч не выдаёт
  новую модель такого переноса за локальное исправление арифметики.
- Конфликт исходного Savage square edge со строгим slope limit. Исходная
  геометрия не сглажена, лимит не ослаблен; сценарий продолжает явно отклоняться.
- Реальная planar bridge parity, Meijaard/cornering/timestep gates,
  same-build/cross-platform continuation и golden evidence.
- Установка, GUI, performance/RTF, worker N-invariance, sanitizer lanes,
  engine-pool выигрыш и GPU go/no-go. Здесь нет результатов этих запусков.

В `task-index.json` остаются 56 строк `source_written`, две V-строки
`blocked_on_W` и десять `pending_verification`. `supported=false` и
`qualification_pending=true` не превращались в claims о готовности.

## Статические проверки доставки

- Проверены diff whitespace и отсутствие conflict markers в изменениях.
- Catalog v2 JSON разобран с отказом на duplicate keys; его 20 embedded
  fragments побайтово собираются в тот же resource. Наибольший fragment —
  24019 байт. Оба catalog v1 файла не изменены.
- Разобраны task/contract registries; имеются уникальные W00–W55 и V00–V11.
  Проверены файловые ссылки CMake spatial test graph и новые link/label changes.
- Патч проверен на неизменяемой базе извлечённого снимка через
  `git apply --check`; применён к отдельной контрольной копии, результат
  сопоставлен побайтово с итоговым деревом. Это проверка текстового патча,
  а не исполнение проекта.

### Применение локально

Из корня репозитория, соответствующего входному снимку:

```bash
git apply --check /path/to/bikesim-w00-w55-static-fixes.patch
git apply /path/to/bikesim-w00-w55-static-fixes.patch
```

Если локальная версия уже отличается, сначала нужно сопоставить конфликтующие
hunks с этими изменениями. Патч не опирается на наличие синтетического локального
Git commit и не требует его импортировать.
