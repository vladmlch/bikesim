# Статический аудит основания нового native 3D-плана

Дата: 22 октября 2026. Назначение: объяснить, какие решения плана опираются на присланные исходники, какие — на исторический аудит, а какие ещё предстоит реализовать и проверить.

**Ничего из проекта не компилировалось и не запускалось.** Не выполнялись CMake configure, compiler probes, syntax-only, тесты, приложения, контейнеры, benchmarks или Git merge simulations. Статический разбор не подтверждает runtime qualification.

Основные документы результата: `../plans/2026-10-22-spatial-3d-native-master.md`, `../specs/2026-10-22-spatial-3d-native-design.md` и фазовые планы. Все номера строк ниже относятся к исходному дереву, извлечённому из пользовательского XML; это не номера строк будущей реализации.

## 1. Идентичность входа и пределы восстановления

| Объект | Проверенная идентичность |
|---|---|
| Пользовательский архив | `repomix-output.xml(20261022-154203).zip` |
| Размер ZIP | 9 217 709 байт |
| SHA-256 ZIP | `5034ba14e39387030faae72ee8dd8b0d014f7eb8a4d1c713c694ea66b85dd9c1` |
| Основной payload | `repomix-output.xml` |
| Размер XML | 43 345 121 байт |
| SHA-256 XML | `e52d5c41d6d97bfe4aea30d50b79232afb15c1bb2804eb8ce36e68dcd00e94b6` |
| Восстановленные записи | 2 280 текстовых файлов с относительной структурой путей |
| Дополнительный ZIP entry | `__MACOSX/._repomix-output.xml`, служебные метаданные архивирования |

XML — снимок выбранного текстового содержимого, а не Git bundle или backup всех worktree. Он не содержит настоящих объектов `.git`, refs/index/reflog и содержимого `.worktrees`, по которым можно было бы независимо восстановить историю веток и пользовательскую незакоммиченную работу.

Сводка Repomix явно исключает binary files, ignored и отфильтрованные пути. Поэтому отсутствие binary fixture, dependency payload или trace в экспорте не доказывает, что этого файла нет в настоящем checkout. И наоборот, текст manifest не доказывает доступность его payload.

В экспортированном дереве отсутствует каталог `docs/superpowers/audits/2026-10-22-branches-worktrees/` с пятью JSON-приложениями старого аудита: `branches.json`, `worktrees.json`, `merge-simulations.json`, `dirty-comparison.json`, `live-overlap.json`. Старый текст ссылается на эти приложения, но их фактические данные в этой сессии не проверены. Даже полный JSON inventory не заменяет сохранённое содержимое dirty/untracked файлов.

## 2. Три класса утверждений

| Класс | Что можно утверждать | Чего он не доказывает |
|---|---|---|
| **Исследованный source fact** | В XML есть конкретный тип, алгоритм, guard, поле, CMake edge или документ | Что код собран, проходит тест или соответствует текущему Git HEAD |
| **Историческое свидетельство** | Аудит сообщает о ветках, dirty overlap и результатах merge на указанное время | Что эти refs и worktree сегодня неизменны; что отсутствующий JSON доступен |
| **Предлагаемое решение** | Новый план задаёт namespace, schema, branch, task, acceptance и будущие проверки | Что соответствующий source path уже создан или capability уже поддержана |

`9c08ff1`, 39 веток, 27 worktree, 138 коммитов впереди `main2`, 23 различающихся общих dirty файла и конфликтные merge counts остаются данными исторического аудита. Они не пересчитывались из XML. Новая ветка `impl/spatial-3d-native` — предложение; она здесь не создавалась.

Исторический аудит полезен для выбора направления интеграции: он сообщает, что PFC уже содержит старые task/lane истории, а `3d` добавляет документацию. Он одновременно предупреждает о незакоммиченной работе и неподтверждённой релизной готовности. Эти две части нельзя разделять, объявляя «всё уже слито и безопасно удалить».

## 3. Проверенные native seams и миграционные ограничения

| Source reference | Наблюдение по исходнику | Следствие для нового плана |
|---|---|---|
| `docs/superpowers/audits/2026-10-22-branches-worktrees.md:3–15` | Зафиксированы timestamp/ограничения аудита; builds/tests не выполнялись | Не считать прошлый Git аудит runtime evidence |
| `docs/superpowers/audits/2026-10-22-branches-worktrees.md:32–45` | В описании истории 3d-ветка содержит 11 документов | Merge этой ветки сам по себе не реализует пространственную физику |
| `docs/superpowers/audits/2026-10-22-branches-worktrees.md:113–140` | Описаны незавершённая квалификация, WIP и условия cleanup | S00 сохраняет необходимые bytes до переноса; cleanup не входит в задачу |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-master.md:53` | Параллельная работа разрешена при готовности интерфейсов зависимостей | Не ждать T63 для самостоятельных terrain/tyre компонентов |
| `docs/superpowers/plans/2026-10-14-python-free-cpp-master.md:84–100` | Ограничены shared headers/CMake, описаны интеграционные lanes | Один интегратор общих контрактов и source lists |
| `docs/adr/0003-native-authoritative-python-removal.md:37–61` | Per-file переход от live Python oracle после двух платформенных captures и C++ port | Не вводить новые Python twins и не удалять все старые gates одним spatial commit |
| `docs/releases/python-free-qualification.md:18–31` | Итоговые required rows остаются незакрытыми | Наличие нативного исходника не равно final-source qualification |
| `docs/releases/python-free-qualification.md:9–16` | Требуются macOS arm64 и Linux aarch64; реальные GUI условия отделены | Xvfb smoke не подменяет display acceptance |
| `migration/cpp-linux-completion-evidence.md:424–510` | Есть привязанный к своему SHA numerics record на двух платформах | Использовать его provenance, не переносить success на весь новый source state |
| `CMakeLists.txt:26–75` | Root graph использует platform modules, C++23 и bindings-OFF default | Новая пространственная production-линия входит в root no-Python graph |
| `CMakeLists.txt:113–171` | Уже зарегистрировано общее C++ core-ядро | Не создавать второй runtime только ради 3D |
| `native/cmake/NativeStartup.cmake:36–134` | Есть библиотеки core, terrain, model, initialization, app/research/recording | Новые файлы подключаются через текущие ownership boundaries |
| `native/cmake/NativeStartup.cmake:47` | В transition graph есть конкретный `.dylib` путь | Проверить затронутый portability edge с PFC owner; это source concern, не результат failed build |
| `native/src/model/builder.hpp:22–42` | Owned `BuildRequest`, `generate_mjcf()` и `compile_mjcf()` существуют | Расширять этот builder, не возвращаться к Python production generation |
| `native/src/configuration/model_config.hpp:45–65` | `SimulationPhysicsConfig` пока не содержит пространственного контракта | `bike::spatial::Config` — будущий согласованный extension |
| `native/src/initialization/setup_owner.hpp:18–35` | Уже есть `SetupRequest` и owned setup boundary | Не создавать параллельный `runtime/equilibrium` |
| `native/src/initialization/setup_owner.cpp:153–233` | Native model/config projection, stepping и settle соединены в setup | Free/constrained startup меняет существующий lifecycle |
| `native/src/initialization/setup_owner.cpp:254–261` | Перед seal bootstrap есть guard | Spatial release/armed checks должны участвовать в этой границе |
| `native/src/runtime/project_config.hpp:30–43` | Projection привязан к fresh model identity | Модель с теми же размерами не является взаимозаменяемой |
| `native/src/runtime/runtime.hpp:50–54` | `RuntimeBootstrap` уже объединяет model/config/state | Расширять его causal state, не создавать дублирующий envelope |
| `native/src/runtime/runtime.hpp:111–164` | Есть owning advance/probe/snapshot/reset/close API | Новые tyre/controller owners обязаны соблюдать существующую lifecycle semantics |
| `native/src/runtime/bootstrap.hpp:74–97` | Инвентаризированы integration/model/tyre/drive/rider/controller состояния | 3D дополняет полный checkpoint, не ограничивается qpos/qvel |
| `native/src/runtime/bootstrap.cpp:249–258` | Runtime decoder проверяет точные keys/version | Новые поля требуют явной version/dispatch схемы |
| `native/src/terrain/types.hpp:152–171` | `HeightFieldSpec` допускает несколько rows | Эта форма данных ещё не доказывает двумерный terrain generator |
| `native/src/terrain/heightfield.cpp:25–33` | Одна исходная высотная строка повторяется по rows | Нужен отдельный native Grid3d asset/query/generator |
| `native/tests/support/migration_test.hpp:22–68` | Реальный `bt` harness, fail zero assertions и строгая fixture-проверка | Переиспользовать harness; spatial root добавить отдельно от migration root |
| `native/tests/reference/CMakeLists.txt:22–43` | Здесь находится фактический reference registrar | Не ссылаться на несуществующий `native/cmake/ReferenceTests.cmake` |
| `tools/run_tests.sh:114–151` | `full` уже выполняет native preflight | Не требовать повторного идентичного full/native sweep без конкретной причины |

В дереве доступны ADR 0001 native port, 0002 drop Python oracle parity и 0003 native authoritative removal. Старый spatial ADR, о котором говорится в аудите, в экспортированном committed-подобном дереве не найден. Номер 0004 в новом плане — кандидат, который S00/S02 проверят в настоящем checkout.

## 4. Почему механическая замена root joint недостаточна

| Source reference | Существующее ограничение | Требуемое действие |
|---|---|---|
| `native/src/model_topology.hpp:10–26` | Есть scalar helpers, нет spatial topology descriptor | Ввести `RootBinding` с отдельными qpos/dof spans и явными body frames |
| `native/src/model/physical_topology.cpp:22–30` | Physical builder удаляет steer joint/sensors | Spatial builder сохраняет steer только для free variant |
| `native/src/model/physical_topology.cpp:95–96` | Collision disabling перечисляет старые tyre backends | Предотвратить двойной wheel-ground contact для pneumatic_3d |
| `native/src/model/physical_topology.cpp:149–153` | Geometric drive требует scalar joints | Переосмыслить transmission/Jacobian, не просто снять guard |
| `native/src/model/articulated_rider.cpp:36–52` | Pelvis roots и руки построены вокруг scalar/planar hinges | Минимальные spatial shoulder DOF требуют отдельного rig и limits |
| `native/src/model/articulated_rider.cpp:81–102` | Существуют positional hand/grip attachments | Не заменять их бездумным orientation weld |
| `native/src/runtime/step.cpp:1015–1030` | Выполняется lookup `root_x`/`root_pitch` | Spatial branch использует model-bound descriptor |
| `native/src/runtime/step.cpp:222–234` | Axle torque ориентирован по world-Y | Использовать actual wheel axis и внутренний реактивный момент |
| `native/src/runtime/step.cpp:1550–1552` | Rollback speed связан с root_x | Скорость вдоль велосипеда вычислять через pose/velocity projection |
| `native/src/initialization/refine.cpp:24–43` | Есть `nq==nv` и scalar interpretation неизвестных | Решать в tangent space размера nv с manifold updates qpos |
| `native/src/initialization/equilibrium.cpp:87` | Settle обращается к `root_z`/`root_pitch` | Явно различать constrained scalar setup и free-root equilibrium |
| `native/src/writers/drivetrain.cpp:104–126` | Scalar spans и `nq==nv` guard | Полный Jacobian и invariant virtual-work tests |
| `native/src/writers/resistance.cpp:100–126` | Поперечные wind/body offsets ограничены нулём | Ввести пространственные силы, точки и torque accounting |
| `native/src/rider/spindle_controller.cpp:667–685` | Используются world pitch и world-Y rotation | Перейти к объявленным body/track frames только в spatial path |
| `native/src/rider/attachment_wrench.cpp:580–609` | Wrench rows ограничены XZ и Y | Полный spatial Jacobian и power-consistent mapping |
| `native/src/runtime/static_brake.cpp:72–115` | Latch привязан к wheel hinge, а не root_x | Не приписывать тормозу несуществующую root_x ошибку |
| `native/src/runtime/step.cpp:2745–2747` | При restore сбрасывается brake latch | Пространственный snapshot обязан восстановить causal brake state |
| `native/src/runtime/step.cpp:2482–2487` | Работа hold учитывается как loss по отрицательной части | Проверить отдельно elastic storage и dissipation; planar fixes вести через PFC |
| `native/src/runtime/capabilities.cpp:63–66` | Source implementation отделена от qualification | Сохранить различие, не выставлять supported по существованию enum |

Следствие: source работу над Grid3d, geometry/brush kernels, topology contracts и независимыми rigs можно начинать рано. Fresh free-root ride, контроллер, запись и viewer закрываются только после своих producer gates. Необходимость изменить общий header не даёт права обходить PFC ownership и существующие planar goldens.

## 5. Research, state и приложение уже нативные, но остаются planar seams

| Source reference | Проверенный факт | Следствие |
|---|---|---|
| `native/src/runtime/sensors.hpp:20–38` | Observation/noise V1 содержит 9 values; dropout отдельно | V2 — 11 values и отдельный поток dropout, V1 layout неизменен |
| `native/src/research/rider_random.cpp:19–38` | Noise/dropout имеют раздельное seed derivation | Новый V2 RNG не сдвигает существующие planar sequences |
| `native/include/bike/policy_v1.h:11–38` | ABI/schema1 закреплены публичным C header | Новый layout требует отдельного policy ABI V2 |
| `native/src/runtime/research.hpp:70–114` | V1 проходит через transitions/history/pending state | Меняется весь typed observation boundary, не один formatter |
| `native/src/recording/catalog.hpp:16–17` | Container schema1 и replay contract3 — разные версии | Не называть все пространственные форматы общим schema2 |
| `native/src/recording/coordinates.cpp:20–44` | Generic builder знает free/ball coordinate widths | Это полезный seam, а не доказательство полной поддержки freejoint |
| `native/src/recording/coordinates.cpp:70–72` | Default aliases требуют root_x/root_pitch | Spatial aliases должны быть topology-aware |
| `native/src/recording/catalog_model.cpp:7–27` | Real mjModel overload использует default aliases | Тестировать настоящий compiled model overload, а не только generic helper |
| `native/src/replay/rebuild.cpp:77–123` | Native reconstruction собирает прежний 1D terrain recipe | Spatial replay использует bundled Grid3d bytes и собственный recipe |
| `native/src/replay/rebuild.cpp:169–192` | Есть terrain и execution identity checks | Не ослаблять checks ради нового режима |
| `native/src/replay/session.cpp:44–63` | Evidence и state имеют разные формулы допуска | Сохранить два правила; новые fields включить явно |
| `native/src/runtime/presentation.cpp:125–130` | Presentation читает scalar roots | Передавать BodyPose, не создавать фиктивные free-root aliases |
| `native/src/viewer/camera.cpp:17–31` | Camera target фиксирует y=0 | Пространственная камера требует реального pose tracking |
| `native/src/viewer/input.cpp:7–30` | R означает reset; C/1/2 переключают camera | Новая документация не должна выдавать R за recording toggle |
| `native/src/viewer/render_owner.cpp:22–109` | Renderer владеет независимой model/data replica | Сохранить thread ownership и запрет UI stepping физики |
| `native/src/runtime/realtime_measurement.cpp:35–112` | Есть native timing advance/boundary/flush | Указывать scope; chunk percentile не выдавать за step percentile |
| `native/cmake/BikeCompilerPolicy.cmake:343–349` | Platform sanitizer ограничения уже документированы | Не обещать несуществующий switch или универсально доступный TSan |

Полный state включает tyre history, brake anchors, held steering torque/clock, control и sensor queues, RNG cursors, metrics, warmstart и mutable model state. Сохранённая engine integration vector не покрывает эти owners автоматически. Restore проверяется продолжением эпизода из ненулевого внутреннего состояния и failure atomicity.

В новом C7 выбраны будущие runtime envelope2, bootstrap2 и writer config3, отдельно от checkpoint1, observation2, catalog/container2 и replay4. Это решения плана, не версии, найденные в исходном runtime: `native/src/runtime/bootstrap.cpp:249–258,342–349` принимает runtime/bootstrap1, а `native/src/writer_config.hpp:1135–1138` — writer1/2. Новые документы требуют отдельных exact-key branches; старые readers и emitters сохраняются.

## 6. Исправленные физические предположения старого 3D-плана

1. **Интерполяция и raycast.** Проверки знаков на двух концах cell недостаточно: внутри bicubic surface возможны два пересечения и tangency. Нужны ближайший корень, vertical ray, start-below, invalid и budget exhaustion; последнее не означает отсутствие контакта.
2. **Граница grid.** Clamped neighbours нельзя одновременно обещать как точный plane-gradient у края. Выбранное extrapolation/one-sided правило фиксируется в аналитическом test; display clamp не становится бесконечной физической дорогой.
3. **Planar extrusion.** Совпадающие узлы не делают bicubic поверхность тождественной старой piecewise-linear. Exact adapter делегирует старой функции; materialized grid сравнивается с отдельным бюджетом ошибки. Исходная planar interpolation: `native/src/pneumatic/geometry.cpp:207–235`.
4. **Normal/camber force.** Площадь поверхности тора и её projected area различны. Нельзя одновременно засчитать radial lateral component и повторно добавить её как camber. Вывод нормировки и virtual work предшествует калибровке.
5. **Friction budget.** Универсальный предел `Fy <= Fz*tan(gamma)` удалён. Slip и camber расходуют общую admissible friction область; знаки, units и область применимости задаются измерениями и симметрией.
6. **Relaxation и энергия.** Переменная, производная которой содержит `-Vsy`, не становится углом без dimensional definition. Диссипативность проверяется вместе с изменением упругой энергии, а не запретом любого положительного мгновенного power.
7. **Patch lifecycle.** Truncate до top-eight теряет нагрузку и момент. Ёмкость выводится/резервируется либо overflow явно отвергается; split/merge сохраняет material state и не создаёт энергию.
8. **Хват.** Старые body-Y arm hinges не гарантируют reachability руля. Нужны минимальный shoulder swivel, limits/strength assumptions и rig на ±5/15/30/60° с gap ≤1 mm, rank и reactions. Compliance не скрывает геометрическую невозможность.
9. **Startup.** Неустойчивое upright равновесие физически допустимо; рулевой controller не обязан удерживать велосипед неподвижно. Hold разрешён только до scored phase, release проверяется отдельно; переходы settling/release/armed/scored явные.
10. **Benchmark.** Meijaard — изолированная идеализированная модель, а не MTB после умножения stiffness на 1000. Linearization и matched model предшествуют time-domain fit; короткий fit плохо различает near-neutral mode.
11. **Круг и lean.** Для кругового теста нужна широкая площадка; использовать измеренные скорость/радиус и COM lean. Нельзя требовать круг внутри прямого коридора ±1.5 m или считать torso Euler angle равным общему lean.
12. **IMU.** Это specific force/rate в sensor frame с mount и lever-arm acceleration; rest и freefall дают разные controls. Euler pitch-rate не заменяет три gyro компоненты.
13. **Детерминизм.** Mirror/rotation invariance в полной engine simulation проверяется обусловленным допуском; same-build thread-count invariance — отдельное битовое требование. Межплатформенные goldens разделены.
14. **Реализм и скорость.** Self-goldens показывают стабильность реализации, не физическую калибровку. RTF относится к конкретным source/hardware/config/tier и не закрывается существованием thread pool.

## 7. Независимые первичные источники

Источники ниже были прочитаны как внешняя проверка физических и API предпосылок. При реализации S03 фиксирует точную версию, страницу/уравнение, полученный payload hash, единицы и условия использования; ссылка на сайт сама по себе не является fixture.

- Meijaard, Papadopoulos, Ruina, Schwab (2007), **Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review**, DOI [10.1098/rspa.2007.1857](https://doi.org/10.1098/rspa.2007.1857), [авторский PDF](https://arendschwab.com/assets/pdf/meijaard2007linearized.pdf). Работа задаёт benchmark из четырёх rigid bodies с идеализированным knife-edge/no-slip контактом. Её результаты нельзя автоматически перенести на compliant articulated MTB.
- MuJoCo, [официальная API reference](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html): `mj_differentiatePos`, `mj_integratePos`, `mj_objectVelocity`, `mj_applyFT`. Документированы manifold qpos/qvel conversion, object-centred rot:lin velocity и применение point wrench. Перед реализацией сверить API с фактически locked engine headers/source; URL stable не фиксирует будущую версию.
- VeTyT bicycle tyre data, [Zenodo record 7866646](https://zenodo.org/records/7866646), DOI [10.5281/zenodo.7866646](https://doi.org/10.5281/zenodo.7866646). Первичный набор описывает lateral force и self-aligning torque при разных нагрузках, давлениях и camber. Указанные авторами лабораторные условия и ограниченная выборка сохраняются в provenance; это не универсальная калибровка агрессивной low-pressure MTB шины.

Внешние источники обосновывают постановку проверки. Они не являются утверждением, что численные коэффициенты уже перенесены, лицензии payloads проверены или новый код успешно сопоставлен с экспериментом.

## 8. Допуск к дальнейшей работе

- **Можно начинать по source contracts:** Grid3d storage/query, геометрию и brush kernels, frame/topology descriptors, independent fixtures и тестовый harness. Интеграционные зависимости остаются обозначенными.
- **Нужны G1/G2:** model builders, реальные force writers, cold startup, cache/reset/probe и free-root stepping. G1/G2 означают конкретный producer evidence на выбранной базе, а не завершение всех PFC задач.
- **Нужны G3/G4:** observation V2, native policies, semantic checkpoint, catalog2 и checked replay4 с исторической совместимостью.
- **Нужны G5/G6:** viewer/display acceptance, package/offline/no-Python closure, платформенные и final-source измерения. T63 не блокирует начало независимой 3D работы, но whole-repository Python deletion нельзя объявлять заочно.
- **Нужен настоящий Git checkout:** выбрать SHA, восстановить необходимые dirty bytes, подтвердить ancestry/merge-base и создать `impl/spatial-3d-native`. Репомикс не позволяет закрыть эти пункты.

Итог статического аудита: архитектурная база для начала параллельной native 3D-разработки есть; готовность пространственного runtime ещё предстоит получить. Новый план делает эту разницу проверяемой через конкретные producer gates, отдельные версии и физические критерии, вместо предположения, что будущая завершённость PFC автоматически делает 3D безопасной.
