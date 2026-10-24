# W20–W36: исходники и передача на V04/V06

Дата: 2026-10-23. Все семнадцать W-задач отмечены как **`source_written`**.
Компиляция, численная проверка и квалификация: **`pending_verification`**.

Поставка относится к двум планам:

- [W20–W28: spatial tyre](../plans/2026-10-22-spatial-3d-native-03-tyre.md);
- [W29–W36: startup и баланс](../plans/2026-10-22-spatial-3d-native-04-startup-balance.md).

По прямому указанию заказчика не выполнялись configure, build, компиляция,
syntax-only, CTest, приложения, симуляции, benchmarks и генерация данных
посредством исполнения проекта. Написанные проверки не выдаются за пройденные.
Acceptance-checkboxes исходных планов не закрыты, V-гейты и upstream evidence
не квалифицированы.

## Источник и применение

| Поле | Значение |
|---|---|
| Входной архив | `repomix-output.xml(20261023-195843).zip` |
| SHA-256 архива | `5f9eb94e53c830aca25d07d81e0bcce056040997d9a7c2a09e8232e3ccd48631` |
| SHA-256 repomix XML | `98ac06d6c3e724e1a804c5a7ab47cf7abb2b90914dc8927aa890990a71754dc5` |
| File blocks | 2439 безопасных уникальных путей |
| Исходная Git history | В архиве отсутствует |
| Основа diff | Извлечённый snapshot с восстановленным terminal LF у непустых файлов |
| Формат поставки | Git unified patch с путями `a/…` и `b/…` |

Terminal LF восстановлен одинаково в неизменяемой базе и рабочем дереве до
редактирования: extractor убирает разделяющий последний перевод строки.
Содержимое прочих строк массово не форматировалось. Локальный synthetic Git
baseline используется только для diff; он не представляет upstream commit
заказчика и не переносит вымышленную историю.

Патч применяется из корня дерева, соответствующего этому входному snapshot,
который уже содержит W00–W19:

```sh
git apply --check /path/to/bikesim-w20-w36.patch
git apply /path/to/bikesim-w20-w36.patch
```

Эти команды не собирают и не запускают проект. При изменениях после snapshot
сначала требуется разрешить конфликт контекста; принудительная перезапись
локальных изменений не предусмотрена.

## W20–W28: шина, силы, тормоз и состояние

| Задача | Написанная реализация | Проверки, ожидающие исполнения |
|---|---|---|
| W20 | Torus grid обоих tiers, finite-volume area, ray contact, material-cell mapping, world tangent frames | Геометрия, аналитическая площадь, bank/camber, зеркальность и covariance |
| W21 | Давление, упругий каркас, точное обновление Maxwell, rim law, storage/loss | Нагрузка/прогиб, relaxation, цикл работы, pressure и rim limits |
| W22 | Connected patches, материал/нормаль как границы, deterministic IDs, split/merge, bounded capacity | Компоненты, стабильность IDs, overflow, allocation boundaries |
| W23 | Fast brush, combined slip, friction ellipse, camber и aligning moment, упругая энергия и потери | Sign, ellipse, relaxation, passivity, зеркальность |
| W24 | Detailed material bristles, локальная поверхность, passive return, выход из контакта | Material memory, fast/detailed comparison, refinement и work balance |
| W25 | Реальный `Pneumatic3dModel`, MuJoCo writer, internal TireWriterView adapter, runtime2/writer3 factory, 3×nv mapping | Виртуальная работа, mixed joints, additive forces, masks, probes, planar golden, warm allocations |
| W26 | Axle-relative brake latch, friction cap, staged restore, hinge storage, damping/yield/release accounting | Удержание, отпускание, общий grip, работа/энергия, restore |
| W27 | BKS3 codec, strict base64/структура/identity, fixed-layout no-throw commit | Roundtrip, malformed bytes, mismatch, atomic rejection, continuation |
| W28 | Provisional coefficient registry и source qualification matrix, методика и профильные данные | V06: измеренные force/load curves, ошибки tiers, экспериментальная квалификация |

Новый `bike_pneumatic3d` содержит семь реальных translation units. Контактные
силы преобразуются через `JpᵀF + JrᵀM` для каждого generalized velocity DOF;
ось колеса и относительный spin берутся из пространственной кинематики.
MuJoCo wheel–terrain masks отключены с проверкой обеих сторон collision mask,
чтобы wheel contact не учитывался двумя владельцами.

Writer сначала выполняет probe, проверяет геометрию и force mapping и только
затем фиксирует материал. Adapter заранее проверяет snapshot projection и
публикует результат в выделенные на cold path буферы. Probe не двигает
material interval; repeated/gapped advance по фиксированным dt/time0 отвергается.
Ошибка до commit не меняет committed material. Выходной additive force span
при неуспешном статусе считается scratch и должен быть отброшен вызывающим.

У fast brush исчезнувшие пятна освобождают оставшуюся упругую энергию; при
уменьшении площади учитывается снятая энергия, при увеличении площадь не
создаёт энергию из сохранённой деформации. Вне контакта Maxwell stress остаётся
в material state; владелец grid учитывает его запас и затухание по площади
соответствующего phi-row. Потенциал давления добавлен в aggregate radial
storage. Эти решения устраняют конкретные пропуски учёта, но не заменяют
численную проверку дискретного work balance при входе/выходе из контакта.

## W29–W36: запуск, управление и сценарии

| Задача | Написанная реализация | Проверки, ожидающие исполнения |
|---|---|---|
| W29 | Spatial initial pose, tangent-space refinement через `mj_integratePos`, раздельные linear/angular residuals, report | Constrained/free и articulated cases, quaternions, probe invariance, failure/report |
| W30 | Hold только при settling/release, explicit release checks, armed/scored lifecycle, moving start | Нулевой scored hold, release rejection, loaded-radius wheel rates, restore |
| W31 | Real fresh/cache/import setup, полный spatial StateFile/PhysicalInitialState inventory, preflight restore | Матрица 2 tiers × 2 root types × 2 rider modes, первые forces/observations, 1000-step continuation, corrupt cache |
| W32 | Stateful steer profile, integer ticks, FIFO delay, interpolation/cap, reset/restore, real actuator work | 20 ticks при 200 Hz/0.1 s, signs/caps/disable, restore, recovery через target rides |
| W33 | First-terminal outcome latch, приоритет invalid/physical/completed, accepted-prefix metrics | Corridor/capsize boundaries, dwell/reset, failure priority, repeated poll, valid duration |
| W34 | Отдельная Whipple model, Appendix A reference, rolling constraints, KKT, polynomial eigenvalues | Native/reference eigenvalues, weave/capsize boundaries, residuals, small rollout |
| W35 | Cornering, mirror и timestep source suites, private complete checkpoint/support, arena example | COM force/momentum balance, lean/radius/speed, mirror tolerances, полные dt episodes |
| W36 | NativeRideRuntime target suites F/B/R/G/C, TOML demand traces и pinned source profiles | Длительные поездки, banks, strict rough uphill, split grip/stop/restart, bytewise continuation |

Free root сохраняет независимые nq/nv и manifold perturbations. Equilibrium
probe восстанавливает causal owners перед оценкой кандидата. Hold удерживает
только заданные lateral/roll/yaw координаты; z/pitch/sag остаются свободными.
Переход к scored требует явного release, после него hold не включается.

На границе t=0 сохраняются settled material/contact snapshots, а interval
clocks и scored cumulative losses переустанавливаются согласованно. Moving
start задаёт root velocity, spin по нагруженному радиусу и согласованный
drivetrain, без внешнего разгонного импульса.

StateFile сохраняет integration, mutable model rows и tyre/brake/drive/rider/
hold/steer/outcome owners. Spatial PhysicalInitialState дополнительно переносит
полный integration vector и семь mutable model arrays; до изменения target
проверяет размеры, finite values, identity и согласование t=0 envelope во
временном независимом mjData. После восстановления projection возвращает
точный integration vector, включая warmstart. Planar payload не получает
пространственные поля.

Steer control использует roll rate тела, heading/lateral errors и фактическое
`rider_enabled`. Работу считает accepted physical interval по реальному
steer actuator torque; tick-period estimate не дублирует её. Built-in gain rows
заданы явными десятичными коэффициентами для стабильной content identity.
Custom profile path разрешается относительно исходного config и проверяется
по digest. При отсутствии cap используется выбранная shoulder strength.

Invalid terrain/numerical failure получает typed outcome, а публикация и
research metrics сохраняют последний достоверный интервал. Physical capsize,
off-corridor и balance-lost не подменяются invalid. Requested completion
обрабатывается после более приоритетных событий того же accepted tick.
При неуспешном advance тормоз, steer FIFO и brake friction bounds возвращаются
к состоянию перед вызовом. Аварийная публикация копирует только outcome в
последний committed spatial record, сохраняя согласование engine/tyre/causal
generation. Для дальнейшего advance/probe требуется reset. Отдельный source
regression проверяет этот отказ после начала реального force stage.

## Методики и данные

- [Нормальная law](../../research/spatial-tyre-normal-law.md).
- [Tangential law и ограничения transport](../../research/spatial-tyre-tangential-law.md).
- [Tyre qualification](../../research/spatial-tyre-qualification.md).
- [Meijaard qualification](../../research/spatial-meijaard-qualification.md).
- [Startup, steering и outcomes](../../research/spatial-startup-balance.md).
- [Timestep/convergence](../../research/spatial-step-convergence.md).
- [Integrated rides](../../research/spatial-integrated-rides.md).

В `native/tests/spatial/data` добавлены provisional tyre/steer profiles и
независимый `ride_profile_pin.json`. Content digest идентифицирует авторский
набор коэффициентов; это не experimental measurement hash и не сертификат
качества. Измеренные значения, выбранный квалифицированный dt, RTF и successful
ride evidence не выдумывались. Результаты остаются null/pending.

W34 helpers имеют одного владельца — `bike_spatial_benchmark_support`, который
используют Meijaard и ideal cornering. W35/W36 работают через реальные setup,
runtime, tyres и controller. Круговой reference задаётся приватным тестовым
входом в настоящий steer controller; публичный production path-following API
не добавляется. Test-only checkpoint C содержит полный mjData и causal/model/
accounting/publication inventory того же владельца; он не выдаётся за будущий
production checkpoint W42.

## Открытые ограничения C10 и V-гейтов

Подробные причины и принятые source decisions записаны в
[contract-deviations](../../../migration/spatial3d/contract-deviations.md).
Для дальнейшей проверки особенно существенны следующие ограничения:

1. Production tyre подключена к grid-backed terrain. `profile_extrusion`
   явно отвергается: frozen API не предоставляет immutable material resolver.
   Поэтому extrusion-based V05 bridge пока не может быть квалифицирован.
2. Frozen material layout не содержит fractional deposition и предыдущий
   tangent basis. Cell association дискретна, bristle components co-rotate;
   общий basis transport и subcell refinement остаются C10, даже при наличии
   отдельных rigid covariance tests.
3. Exact integrated finite-volume area, исправленная ray orientation и brush
   sign отличаются от противоречивых фрагментов prose; замороженные signatures
   сохранены, решения явно изложены в ledger.
4. BKS3 структурно валидирует данные, но frozen layout не имеет checksum.
   Произвольный bit flip, оставляющий допустимое finite value, требует внешнего
   integrity layer или новой версии wire.
5. Оригинальный Savage genspec нарушает прежний terrain slope limit. Сценарий R
   сохраняет этот вход и должен сообщить rejection; замены дороги, сглаживания
   или фиктивного completion evidence нет.
6. Tyre coefficients и steer gains остаются provisional. Нельзя объявлять
   физическую достоверность, устойчивость 20-секундных поездок, Meijaard parity,
   отсутствие аллокаций или численные допуски достигнутыми без V04/V06.
7. Полный spatial research sensor packet относится к W37, production replay/
   checkpoint — к W42. Эти публичные области не подменяются scalar packet или
   test-only checkpoint.

Внутреннее bounded thread-local scratch W22/W24 очищается перед каждым
вызовом, не содержит material history и защищено от same-thread reentry.
Allocation/performance acceptance всё равно требует исполнения. Frozen
`noexcept` outcome API со строковым diagnostic также требует C10 для общего
allocation-free polling произвольного восстановленного diagnostic.

## Проверка самой поставки

Проверка ограничена чтением и сопоставлением исходников/API, регистраций CMake,
форматов данных и целостности патча. Сравнены 51 frozen W00 header; изменений
нет. Все 16 новых production translation units зарегистрированы в общем
`NativeStartup.cmake` для обоих графов. Добавлены 19 test targets W20–W36,
включая четыре slow suites W35/W36; регистрация не является результатом сборки.

`task-index.json` сохраняет W20–W36 как `source_written`, V04/V06 — как
`pending_verification`, обновляет исторические blockers V03/V05 и не закрывает
upstream gates. Capability rows показывают реализованный source path с
`qualification_pending=true`, `supported=false`. Применимость patch проверяется
к неизменяемой нормализованной базе; это проверка текстовой поставки, а не
исполнение или верификация физики проекта.
