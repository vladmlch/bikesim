# Повторная статическая проверка W00–W55

Дата: 24 октября 2026 года. Вход: `repomix-output.xml(20261024-143059).zip`.

Это отчёт о дополнительной проверке и исправлениях относительно указанного
снимка. В нём уже присутствуют результаты более ранней проверки, описанной в
`2026-10-24-spatial-w00-w55-static-review.md`; они не считаются новыми
исправлениями этого прохода. Итоговая поставка — применяемый локально Git patch.

**Проверка исключительно статическая.** Configure, компиляция, линковка,
syntax-only, приложение, симуляции, тесты, benchmarks, sanitizers и проектные
скрипты не запускались. Новые регрессионные исходники не являются результатами
тестов. Проверка файлов патча отдельно описана в конце.

## Основание и реконструкция

Архив SHA-256:

`9b8c3779f4c7dc1635f961742b2a054d33662c4beadf38d7bc4a7993fc3f1a7d`

Вложенный `repomix-output.xml` SHA-256:

`1bc11dc6aa735d6d46e2dfdb95b148abb901156fe849e3c012e2d2fda1e5618e`

Извлечены 2582 уникальных безопасных относительных пути. Проверены границы
файлов и соответствие содержимому XML. Unmixer удалял завершающий LF вместе с
разделителем записи; до фиксации базы LF восстановлен у 2560 непустых файлов.
Этот технический шаг относится к реконструкции базы, а не к патчу исходников.

Снимок не предоставляет исходную Git-историю. Для diff создана локальная
техническая база; она не выдается за upstream commit и не требуется для
применения патча. Патч рассчитан на содержимое именно этого архива, уже
содержащего предыдущие исправления.

Основания требований: master/numbered plans
`docs/superpowers/plans/2026-10-22-spatial-3d-native-*`, contracts C1–C10,
`migration/spatial3d/task-index.json`, исходники производителей и потребителей,
`AGENTS.md` и `docs/agents/native-engineering.md`. Запрет пользователя на
исполнение сохранён, поэтому предусмотренные репозиторием исполнительные
verification gates здесь не закрываются.

## Как разделялась работа

Агентам выдавались последовательные небольшие пакеты с конкретными файлами.
Общими runtime/CMake/journal-файлами управлял интегратор. Отдельные исправления
после интеграции получили независимый второй просмотр.

| Задачи | Проверенная область и результат этого прохода |
|---|---|
| W00–W05 | Frozen declarations, config/wire, capability dispatch, registries и общая CMake-схема; новые отклонения записаны в C10, статусы готовности не повышены. |
| W06–W08 | Grid/asset/interpolation/raycast; исправлена допустимость сеток со схлопнувшимися соседними координатами. |
| W09–W12 | Generator/spec/extrusion/source/render; устранено проникновение бокового рельефа в flat lead zones. |
| W13–W14 | Builder, physical/spatial topology; исправлены выбор режима и обнаружение корней райдера. |
| W15–W16 | Force/drive/rider bindings, support geometry и дорожный preview; исправлены torque contracts и пространственная геометрия. |
| W17 | Runtime lifecycle/probe/continuation; публичный пространственный probe изолирован от live owners. |
| W18–W19 | Planar bridge и отрицательные controls; проверяется реальный inventory вместо неподходящего rider-флага или одного cached snapshot. |
| W20–W22 | Geometry/carcass/patches; исправлены Maxwell cancellation и deterministic merge tie-break. |
| W23–W24 | Fast/detailed brush; закрыты nonfinite overflow, неполная история standalone helper и порядок reductions. |
| W25–W28 | Tyre owner/writer/adapter/state/brake/qualification; проверяется фактическая модель, публикация атомарна, исправлены state invariants и fixture phase. |
| W29–W31 | Equilibrium/hold/import/finalization; закрыты обход физического gate и отдельные нарушения rollback/commit. |
| W32–W33 | Steering profiles/state/outcomes; typed profiles отвергают duplicate keys, прослежена каноническая terminal reason. |
| W34–W36 | Meijaard/corner/mirror/timestep/target rides; all-step limits больше не теряются при decimation, конечная точка distance run сохраняется. |
| W37–W40 | IMU/sensor FIFO/restore, ABI2 и truth; исправлены dropout restore и общий alias задержки. |
| W41–W43 | Recording/assets/checkpoint/replay; запись переносит внешний профиль руля, checkpoint принимает preview любого heading. |
| W44 | Golden registry/comparison; nonfinite числовые доказательства не могут пройти сравнением одинаковых битов. |
| W45–W49 | CLI/viewer/playback/reports/install/examples; исправлены ошибочная planar preview, exit/performance classification, spatial grade и согласованность display pose. |
| W50–W55 | Performance identities/timing, worker controls, engine-pool harness, qualification registry и GPU decision; закрыты ложные acceptance paths. |

Этот перечень описывает просмотренные границы и выбранные исправления. Он не
является доказательством отсутствия любых других дефектов во всех файлах
репозитория.

## 1. Terrain: допустимость сетки и реальные плоские зоны

### Схлопывание координат binary64

`Grid3d::create` проверял finite positive spacing и общий extent, но не каждый
шаг координаты. При `x0=2^53`, `dx=1` первая соседняя точка совпадает с началом,
хотя конечная граница отличается. При `dx=1.25` первый шаг уже различим, но
следующие два узла совпадают. Такая сетка нарушает предположения sampling/DDA.

Cold factory теперь проверяет строгое возрастание вычисленных координат обеих
осей. Проверка O(nx+ny) не создаёт дополнительной allocation и не меняет hot
query. Регрессии в `terrain_codec.cpp` охватывают x/y, оба spacing и допустимый
контроль с spacing=2.

### Catmull stencil нарушал lead-in/out

Нулевые боковые значения в узлах lead zone не обеспечивали нулевой интерполянт
внутри её крайней cell: четырёхточечный stencil захватывал следующий ненулевой
узел. При единственном внешнем значении A в середине cell возникает `-A/16`.

Генератор оставляет две дополнительные physical grid spacings нулевого halo
перед включением и после выключения бокового рельефа. Quintic fade сохраняется.
Проверки написаны для промежуточных точек, невыравненных lead boundaries,
bank/noise и обеих сторон corridor. **Байты вновь генерируемого terrain могут
измениться**; format и query уже сохранённых grid assets прежние.

## 2. Builder, топология, torque contracts и rider geometry

`BuildRequest` мог содержать spatial config только в `physics`, но builder
выбирал ветку по отдельному model-полю и строил planar модель. Теперь выбор
учитывает фактического владельца physics, противоречащие явные объявления
отвергаются, terrain3d без spatial mode не принимается. Два согласованных
явных planar объявления по-прежнему допустимы; spatial fingerprint не
вычисляется для planar config.

Обнаружение scalar rider root больше не зависит от того, оказался ли pitch
joint раньше translation joints в массиве модели. Независимые qpos/dof
addresses сохраняются.

Потребители spindle, drivetrain и steering применяют control как физический
момент. Поэтому spatial bindings теперь требуют unit gear, fixed gain=1,
отсутствие actuator dynamics и bias. Для явного steering cap проверяется
полный допустимый диапазон `[-cap,+cap]`. При отсутствующем cap новый
необъявленный default не вводится.

В пространственном box support нормаль и преобразование точки/зазора теперь
используют одну и ту же нормализованную derived rotation. Допустимый
near-unit quaternion не может дать две разные геометрии одной опоры;
исходные qpos не переписываются.

Регрессионные исходники: `test_model`, `test_topology`, `test_drive`,
`test_rider`. Допуски существующих valid controls не ослаблялись.

## 3. Реальная дорога под райдером и неизменяющий public probe

### Preview зависит от текущего положения и направления

Spatial rider-control использовал legacy X-Z centreline. На `z=0.2*y` при
движении вдоль +Y фактический уклон равен 0.2, а centreline по X даёт ноль.
При развороте знак должен меняться; на `z=0.1*x*y` продольный уклон зависит
от текущего Y, даже если X одинаков.

Spatial preview теперь строится по реальным XY колёс и текущему горизонтальному
heading. Grade вычисляется из physical normal как
`-(nx*heading_x + ny*heading_y)/nz`. Lookahead начинается от ведущего в этом
направлении колеса; включает дробный конечный интервал. Выход необязательного
lookahead за terrain обрывает preview, а недопустимый запрос под самим колесом
отвергается. Перед чтением геометрии обновляются только stale kinematics.

В spatial checkpoint decoder удалено требование строго возрастающего world X:
при движении вдоль Y он постоянен, при движении назад убывает. Порядок samples
задаётся направлением движения. Width/finite/time checks сохранены, поля wire
не меняются. Новый `road_preview.cpp` и case `turned_rider_preview` в
`checkpoint.cpp` описывают аналитические уклоны и восстановление после yaw
90/180 градусов.

### Публичный probe менял физические владельцы

Вызов `NativeRideRuntime::probe_step_inputs` напрямую входил в force staging
живого `PhysicalStep`. Это могло менять controls, applied forces, model
friction, derived/warm-start rows и controller caches. Равенство только
опубликованного snapshot не обнаруживало этих изменений.

Теперь пространственный публичный probe получает отдельный cold physical arena.
В него переносятся текущие model/integration/material/controller/causal/cache
данные. После constructor refresh восстанавливаются точные integration и
нужные engine-derived rows. Проверки model identity, dimensions, state schema,
topology, rider и poison boundary сохраняются. Уже приобретённый steering
profile копируется из владельца, поэтому probe не требует повторного открытия
внешнего файла.

Временный arena не создаёт accounting worker и не опустошает async queue
исходного runtime. Public return shape и внутренняя startup force staging
ветка сохраняются. Это cold API с дополнительными копиями, а не обещание
бесплатного warm-kernel probe.

W18 bridge больше не требует rider-only `has_contact_probe` у модели без
райдера: проверяется реальная finite/nonzero шинная строка размерности nv.
Статические Fz/Fy берутся из возвращённого probe. W19 сравнивает полный live
inventory после валидного/ошибочного probe, restore и reset. Дополнены
free-root+active-steering continuation и отдельный async control без flush.

## 4. Tyre kernels, state и публикация

### Maxwell stress/loss

Прежняя формула потерь при `dt << tau` вычитала величины порядка
`q_ss²*dt`, оставляя результат порядка `q_ss²*dt³/tau²`. Это теряло значащие
цифры; `max(0, NaN)` дополнительно мог скрывать nonfinite. Для малых `dt/tau`
использовано устойчивое аналитическое разложение, для остальных — эквивалентная
формула без вычитания крупных почти равных коэффициентов. Nonfinite результат
доходит до rejection до commit.

Stress вычисляется как сумма затухающего и steady вкладов. При большой
релаксации малый ненулевой steady stress больше не теряется вычитанием q0
из q0. В `tyre_carcass.cpp` ожидаемые пределы заданы независимо от прогона
реализации.

### Patch association и fixed material order

При одинаковом overlap load merge выбирал меньший исторический patch ID,
хотя требование задаёт минимальный overlapping element. После split эти
порядки могут различаться. Tie-break исправлен без изменения wire.

Detailed bristle storage/loss сводятся в порядке material ID через ограниченный
TLS map. Outputs kernel сохраняют input order для правильной patch mapping.
После назначения patch IDs model поворачивает полные wrench records в
material order; этим порядком пользуется последующая world/Jacobian reduction.
Для двух поддерживаемых fan grids материал имеет не более одного циклического
разрыва, поэтому достаточно одной rotation без allocation и общего sort.

### Fast brush: overflow и неполная история

Конечные входы могли переполнить implicit denominator или friction capacity.
Полученная бесконечность давала нулевую displacement, а NaN loss исчезал через
`max`, после чего результат принимался. Проверяются промежуточные stiffness,
camber, denominator, trial displacement, capacity, ellipse и loss до commit.

В standalone helper конечность проверяется по всей state capacity. Кроме
того, ненулевые ux/uy за пределами переданного patch span теперь отклоняются
до output/state writes: у этого API нет старых площадей, чтобы корректно
освободить пропущенную spring energy. Это явно записано в публичном комментарии
и C10. Нулевая spare displacement capacity с конечным camber допустима;
выгруженный контакт можно передать явно с его старой площадью и нулевой
нагрузкой. Production owner пользуется отдельным resolved seam и собственной
release accounting.

### Identity, state invariants и atomic adapter

Tyre writer сравнивает digest **фактической** `mjModel` с моделью, к которой
привязана supplied topology. Одних равных nq/nv и самосогласованных чужих
fingerprints недостаточно. Для этого экспортирован существующий cold MJB hash
helper; новое вычисление не попадает в physics interval.

Adapter собирает snapshots обоих колёс в заранее подготовленную staging пару
и публикует её только целиком. Ошибка второго этапа больше не оставляет
частично заменённую ранее опубликованную пару.

State restore отвергает невырабатываемые producer состояния: inactive current/
previous penetration должны быть +0, active penetration — положительной и
равной previous. Ненулевой Maxwell stress в inactive материале остаётся
допустимым: он релаксирует после выхода из контакта.

Qualification fixture теперь согласует phase каждого колеса с интегралом его
фактического spin, slip kappa и собственного radius, включая остановку и
реверс. Пороги и число физических cases не заменены удобными результатами.

## 5. Equilibrium, import и hold

Explicit equilibrium file проверял в отдельной ветке только qacc. Дальнейшие
contact/load/moment errors могли игнорироваться из-за наличия hold owner,
а восстановленная фаза `armed/scored` уже пропускала release check.
`PhysicalInitialState` также мог переносить устаревший successful report.

Spatial imports проходят проверку фактической геометрии и опоры. Released
phase не является доказательством равновесия. До lifecycle/accounting mutation
повторно проверяются contacts, acceleration, weight и external moments.
Существующие residual/release limits сохранены; fresh free-root hold сохраняет
свой диагностический release path.

Однократный `spatial_refine(...,1,...)` выделен в observational ветку.
Обычный отказ запроса восстанавливает incoming integration, material, model,
controller и hold inventory, затем повторно бросает ошибку. Fatal
`EngineFailure` сохраняет прежнюю poison policy.

При подготовке hold сначала выполняются fallible validation/allocation,
затем публикуются owner state, force contribution и causal record. Это
устраняет конкретные split-owner transitions; не заявляется rollback всей
длительной процедуры settle при произвольном engine/allocator failure.

Начальный rolling spin использует load-weighted normal каждого колеса,
совместимую с его effective radius и фактическим hub motion. Одна normal под
root больше не применяется к двум различным локальным уклонам. Невозможно
обещать no-slip в каждой точке неоднородного footprint одной spin DOF; новая
правка такого утверждения не делает.

Cases в `equilibrium.cpp` и `startup_paths.cpp` описывают поднятый released
import, обычный failed probe с ненулевой скоростью, local rolling и valid
continuation. Изменения независимо просмотрены вторым агентом.

## 6. Steering, sensor restore и qualification harness

`read_steer_profile(Value)` отвергал unknown fields, но native `Value::Object`
мог содержать duplicate keys; reader выбирал первый, а fingerprint игнорировал
другой. Теперь дубли проверяются до разбора и hashing. JSON parser уже имел
такую защиту; закрыта именно типизированная входная граница.

`SensorPipelineV2::import_state` теперь сверяет dropped count с consumed tape
prefix. Гарантированно потерянная attempt не может восстановиться valid в
startup или retained suffix queue. Nondropped attempt по-прежнему может быть
invalid по другой причине. Проверка cold и выполняется до commit, без изменения
RNG или tick/read arithmetic.

Общий request alias `sensor_delay_s` применялся только к V1. Теперь он
устанавливает также V2 latency с тем же приоритетом над nested config.

Target-ride harness раньше проверял torque caps, brake budget, friction ellipse,
hold work и rim penetration только по decimated points. Краткое нарушение
между ними исчезало. Эти максимумы теперь обновляются на каждом физическом
наблюдении, NaN превращается в явный непроходящий максимум, исходные limits
сохранены. Distance-limited run всегда сохраняет точку фактического окончания,
даже если она не попадает на decimation tick.

## 7. Recording, replay, checkpoint и golden evidence

При итоговой сверке с CMake обнаружены семь поздних W50–W54 test targets без
строк в общем W44 fixture manifest: `performance_report`, `pacer_report`,
`worker_ranges`, `worker_pool`, `thread_invariance`, `engine_pool_probe`,
`qualification_registry`. Добавлены только их source-task/classification
записи со статусами pending и null hashes. Вместе с новым `road_preview`
manifest теперь охватывает все 64 `bike_add_spatial_test` targets и два
отдельных CMake smoke tests; неизвестные measurements не заполнялись.

### Внешний steering profile входит в запись

Запись сохраняла путь к custom steering profile, но не его содержимое.
Checked replay зависел от исходного файла и мог перестать работать после
переноса записи или удаления файла.

Активный внешний профиль теперь сохраняется как `spatial/steer_profile.json`.
Читаются ограниченные исходные bytes, проверяются semantic SHA/cap/delay;
сохраняются именно прочитанные bytes, чтобы canonical expansion не превысил
лимит resolver. Контейнер защищает raw SHA, профиль — semantic identity.

Replay требует этот asset только для active external profile, проверяет
semantic hash и подставляет private extracted path. Конфигурационные digests
нормализуют relocation path в semantic profile hash. Built-in/off profiles не
меняют identity. Тестовый сценарий удаляет исходный профиль после finalize и
проверяет replay, missing-asset rejection и semantic tampering.

Старые записи с custom profile без обязательного asset не объявляются
переносимыми: checked replay отказывает явно. Engine-free inspection остаётся
отдельной возможностью. Это указано в C10.

### Nonfinite не является успешным golden match

Same-platform array comparison раньше мог принять одинаковые NaN/Inf bytes
до проверки numeric domain. Теперь оба массива декодируются и проверяются
на конечность до bitwise/tolerance comparison. Scalar numeric comparison также
отвергает nonfinite до equality. Signed-zero и finite tolerance contracts не
ослаблены.

Дополнительно введён typed preflight пространственного capture до преобразования
`Wire` в generic `Value`: прежняя проекция превращает nonfinite double в
обычную строку. Явно типизированные nonfinite значения отклоняются; законные
строки вроде `"nan"` не запрещаются по написанию. Уже сохранённую старую
нетипизированную строку без provenance нельзя надёжно отличить от текстовых
данных; этот предел не скрывается blanket rejection.

## 8. CLI, viewer и presentation

Spatial `--preview` раньше рисовал planar placeholder TrackSpec, пока настоящая
Terrain3D ещё не была загружена. Неподдерживаемый planar route теперь
отвергается явно. Spatial preview rows не выдают выдуманный нулевой grade
или planar obstacle metadata: отсутствующий принятый directional grade
представляется null; planar поведение сохраняется.

Canonical spatial completion сериализуется как `completed`. CLI viewer
performance и realtime gates теперь сравнивают значение через общего enum
serializer. Раннее закрытие viewer с `not_evaluated` status больше не бросает
исключение из bool conversion: такой run не получает eligibility. Явный
`numerically_valid=false` приводит к physical CLI exit 2 наряду с invalid
model; operator stop сохраняет 130.

RenderOwner сглаживал free root велосипеда, но independent free pelvis
райдера оставался в последнем accepted world pose. При движении они могли
визуально разъезжаться. К pelvis применяется тот же rigid display delta;
относительная принятая pose и локальные articulated coordinates сохраняются.
Изменения относятся к render replica: accepted integration state не
переписывается. Новая inspection функция возвращает owned rendered qpos.

Одинокая valid точка между invalid gaps больше не исчезает из HTML line plot:
для такого segment рисуется point, разрывы не соединяются. Изменения проверены
по source paths; реальный GUI/display/installed-package запуск не выполнялся.
Описание HUD исправлено: отображаемые углы задаются в degrees, записываемые
каналы сохраняют radians. Для W48/W49 просмотрены install declarations,
smoke scripts, incompatible-identity fixture, десять документированных argv
и примеры; дополнительные подтверждённые дефекты этих маршрутов не выбраны.
Отсутствующие исторические binary fixtures и pending/null evidence не
заменены вымышленными assets.

## 9. Performance, workers и qualification decisions

Report reader проверяет полный `SpatialPerformanceProfile` тем же domain
validator, что producer. Равенства двух неполных identity объектов больше
недостаточно. Сравнение включает backend/profile/execution, а warmup проходит
те же terminal/comparability проверки, что measured runs.

`instrumentation_available=true` требует наблюдённых положительных clock reads.
Сумма exclusive region times должна быть конечной и укладываться в полное
wall window с действующим float comparison. Средние nonnegative times и
overhead вычисляются без переполнения промежуточной суммы повторов.

В test allocator исправлено переполнение округления aligned allocation для
SIZE_MAX: оно не может превратиться в маленький успешный buffer. Добавлены
throwing/nothrow и ordinary retry controls.

Worker-pool source control учитывает allocation/free counters отдельно на
caller и каждом worker, после прогрева и на 64 поколениях. Negative control
явно делает allocation на каждой lane. Это делает будущую проверку
содержательной, но не является доказательством её прохождения или отсутствия
гонок. Сам production pool не переписывался без подтверждённой ошибки.

Engine-pool benchmark использовал retention=off, но не потреблял pending
delivery queue. Теперь `release_samples` вызывается на каждом control block
и после flush внутри measured window; очередь не растёт на весь заезд.

GPU latency gate больше не делит odd subnormal CPU time пополам с округлением
порога вверх: точное удвоение GPU time не создаёт ложный 2x gain. Для 10% gain
в engine/GPU full-ride controls используется масштабирование степенью двойки,
устраняющее underflow порога. Accuracy/determinism и обычные численные
thresholds остаются обязательными. W54 structural registry не превращён
в проверку существования или достоверности внешних measurement files.

## Совместимость и незакрытые требования

Новые API/comment/identity/numerical решения перечислены отдельно в
`migration/spatial3d/contract-deviations.md`, раздел input 143059. Строгие
проверки могут отвергать malformed models, fabricated restore state и reports,
которые раньше ошибочно принимались. При последующей локальной проверке
потребители headers должны быть согласованно пересобраны.

Остаются реальные, уже объявленные ограничения исходной модели:

- Fractional material deposition, точная material-overlap patch ancestry и
  общий перенос history между меняющимися tangent bases не реализуются этой
  серией локальных исправлений. Frozen state не содержит необходимых данных;
  для полного требования нужен согласованный C10/state-model дизайн и
  самостоятельная физическая qualification.
- Исходный Savage square edge несовместим со строгим terrain slope limit.
  Геометрия не сглажена и предел не ослаблен; acceptance такого сценария
  продолжает требовать решения зафиксированного конфликта требований.
- Provisional lateral/camber/steering profiles не становятся измеренной
  калибровкой от наличия SHA или исходников теста.

Отдельно остаются **невыполненными исполнением** passivity/energy closure,
convergence, planar bridge parity, Meijaard/cornering/mirror/timestep gates,
fresh/cache/import и same-build/cross-platform continuation, реальные goldens,
install/GUI, performance/RTF, allocation/TSan/RTSan, N-invariance,
engine-pool benefit и GPU go/no-go.

Ни одна строка W/V registry, capability flag или platform result не повышена
до accepted/passed. В task index сохранены 56 W-строк `source_written`, две
V-строки `blocked_on_W` и десять `pending_verification`; отдельная S00-строка
остается `in_progress`. Новый `spatial.road_preview` зарегистрирован в CMake
и fixture manifest с `pending_verification` и null evidence; семь пропущенных
поздних targets получили такие же pending записи.
