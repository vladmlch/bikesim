# W06–W19: исходники и передача на V02/V03/V05

Дата: 2026-10-23. Все четырнадцать W-задач имеют статус **`source_written`**.
Исполнение, численная проверка и квалификация: **`pending_verification`**.

Эта поставка выполняет source-authoring часть двух планов:

- [W06–W12: terrain](../plans/2026-10-22-spatial-3d-native-01-terrain.md);
- [W13–W19: topology](../plans/2026-10-22-spatial-3d-native-02-topology.md).

По прямому указанию заказчика не выполнялись configure, build, компиляция,
syntax-only, CTest, приложения, симуляции, benchmarks и генерация fixtures
посредством исполнения проекта. Написанные тесты не выдаются за пройденные.
Acceptance-checkboxes планов не закрыты, upstream gates не квалифицированы.

**Полная сборка/runtime ещё зависят от будущих W-производителей.** Вызовы
реальных frozen API W20/W21/W25/W27/W29 оставлены в исходниках, как требует
план; отсутствующие определения не заменены заглушками. `bike_pneumatic3d`
является будущей реальной CMake-зависимостью. Автоматическая схема writer3 и
активная spatial шина ещё принадлежат W25; свободное равновесие — W29.
W30–W33 подключат активные hold/steer/outcome owners, W37 — research packet.
Следовательно, этот патч не объявляет текущий W06–W19 snapshot самостоятельно
линкуемым и не включает spatial capability.

## Источник и формат патча

| Поле | Значение |
|---|---|
| Входной архив | `repomix-output.xml(20261023-192428).zip` |
| SHA-256 архива | `a25e71a5b41a0ca03b4211e08087c4efa41be3a31676238e58f09b9adb24c902` |
| SHA-256 repomix XML | `1bc12a263395b7d00c93f671dcf896de2e2fe0d191dd7dc91a9ee7b4be6dcc54` |
| Извлечено file blocks | 2382 |
| Исходная Git history | В repomix отсутствует; исходный commit SHA не восстанавливается из synthetic baseline |
| Основа diff | Извлечённое дерево с terminal LF для непустых текстовых файлов |
| Формат | Обычный Git unified patch, пути `a/…` и `b/…`, применение из корня проекта |

Проверены допустимость путей и отсутствие дублирующих file blocks. Terminal LF
восстановлен одинаково в неизменяемой копии входа и рабочем дереве до diff,
согласно прежней W00–W05 поставке. Это предотвращает фиктивные изменения всех
последних строк; прочие исходные байты не форматировались массово.

Для получения diff создан только локальный synthetic Git baseline. Он не
заменяет Git history заказчика, не публикуется как исходный SHA и не является
новым migration commit. Итоговый файл содержит изменения относительно этого
repomix snapshot, уже включающего W00–W05. Замороженные новые W00 declarations
не редактировались; существующие mutable headers расширены только в требуемых
интеграционных границах.

## Состав W06–W12

| Задача | Написано | Проверка после W-фазы |
|---|---|---|
| W06 | Владеющий `Grid3d`, codec BKT3/v1, canonical bytes/digest, валидация dimensions/sections/materials, ограничение ресурсов | V02: roundtrip, повреждения, точные байты, resource limits |
| W07 | Bicubic patches, высота и нормаль, closed-domain boundaries, C1 seams, no-allocation query | V02: плоскость, seams, край/nextafter, материалы |
| W08 | Grid DDA, polynomial/Bernstein root isolation, первый/tangent hit, явное исчерпание бюджета | V02: first/tangent/miss/degenerate и budget criteria |
| W09 | Строгий genspec, deterministic generator, bank/features/roughness/materials, ограниченное владение полноразмерными буферами | V02: независимые per-platform hashes и peak RSS |
| W10 | `ProfileExtrusion3d` с исходными узлами, общий immutable `Terrain3dSource`, independent render grid | V02: legacy height/ray comparator, oblique rays, render separation |
| W11 | Раздельная загрузка физического terrain, lifetime в setup/runtime, заполнение всех hfield rows, collision masks и metadata | V05: настоящий startup с W25/W27 и отсутствие physical query по render mesh |
| W12 | Acceptance/budget test sources, corrupt/analytic/genspec fixtures, два research examples и manifest, terrain audit, секции TESTING/RIDE | V02/V05: результаты, hashes, измеренные бюджеты |

Физика читает canonical query view владельца grid/extrusion. Визуальный hfield
создаётся отдельно для показа и crash geometry. Изменение числа render rows
не подменяет физические heights/normals/material IDs. Границы domain и corridor
проверяются до настройки runtime. Planar request с spatial model/source явно
отвергается до разыменования несогласованного context.

Реальные platform hashes и benchmark-измерения оставлены null/pending.
Отдельный [terrain audit](2026-10-22-spatial-3d-native-terrain.md) содержит
таблицу критериев. В частности, legacy Savage имеет резкие ступени, которые
при шаге 5 мм нарушают предписанный предел наклона. Текущий generator
отвергает этот вход; профиль не сглажен скрытно ради acceptance.

## Состав W13–W19

| Задача | Написано | Проверка после W-фазы |
|---|---|---|
| W13 | Named descriptor для planar3/constrained4/free6, независимые qpos/dof spans, model/topology SHA-256, frames/velocities | V03: реальные mixed joints, имена/оси/identity, covariance |
| W14 | Раздельные ветки модели, free root, steering joint/actuator, IMU, masks, mass/COM/inertia test sources | V03: engine model и независимый прежний native MJCF oracle |
| W15 | Spatial chain geometry и Jacobian по nv, три drivetrain family, axle-relative brake/drive, spatial resistance и source config | V03: virtual work, rigid covariance, прежние force-stage golden |
| W16 | Free pelvis, shoulder swivel/profile, positional grips, spatial support frame/Jacobian/wrench, versioned shoulder state | V03: grip feasibility, rank/nullspace, power, profile/layout rejection |
| W17 | Topology-aware step/snapshot, runtime2/bootstrap2/writer3 family, causal codecs/owners, capture/import/reset seams и future producer dispatch | V03: codec/identity/lifecycle и completed producer linkage |
| W18 | Short/long constrained bridge на настоящем setup/runtime/writer, per-wheel/load/distance/impulse/transitions/persistence/outcomes | V05 после W25/W27: реальные обе траектории и continuation |
| W19 | Полная negative matrix, реальные witnesses state/publication, geometry/carcass covariance, закрытые capability rows | V03/V05: adversarial rejection и measured covariance |

`free6` имеет реальное `nq=7`, `nv=6` для корневого joint. Внутренние scalar
hinge/slide joints сохраняют собственные qpos/dof addresses, независимо от
предшествующих FREE/BALL joints. Неиспользуемые planar адреса пространственного
шага равны -1 и не читаются в presentation/samples. Продольная скорость
проецируется на горизонтальный heading тела; Euler angles являются derived
display values с validity, а не компонентами quaternion.

В rider branch поддержаны spindle pedals / pin saddle / connect grips и
locked ankles. Shoulder profile содержит provisional coefficients и content
digest. Контроллер не владеет steering actuator, и lateral hip/torso controller
не вводится. Тест grip содержит 288 combinations steer/crank/lean/suspension,
но результаты этих комбинаций не получены и достижимость ±60° не заявлена.

Старый planar model/force oracle не перегенерирован. Там, где входной архив не
содержит необходимый native MJCF capture или бинарный bootstrap fixture,
проверка должна завершаться ошибкой отсутствия evidence, а не skip/pass.
Rigid-rider и constrained bridge не квалифицируют articulated/free-root.

## State и публикация

Разрешены только семейства `(runtime, writer, bootstrap)` `(1,1,1)`, `(1,2,1)`
и `(2,3,2)`. Bootstrap2 содержит обязательные exact-key spatial и tyre sections.
Hold/brake/steer/outcome codecs проверяют finite values, schema, quaternion,
FIFO bounds и неизвестные/повторные поля. Состояние шины делегируется реальному
W27 codec и связывается с topology/terrain/calibration identity.

Capture сохраняет исходный immutable spatial model image и отдельные mutable
model rows; digest не пересчитывается по модели, изменённой settle/material
owners. Sealing без настоящего tyre producer выдаёт точную ошибку
`bootstrap.spatial.tire: producer state missing`.

В equilibrium/initial-state import сначала проверяются identity, typed material,
model rows и целевые lanes. Shoulder `{q,qd}` обязан точно совпадать с реальными
qpos/qvel joint addresses. Spatial native equilibrium сохраняет свою captured
velocity; прежний planar equilibrium reset сохраняет нулевую скорость. Outer
drive/controller/anchor overrides PhysicalInitialState проходят preflight до
prepare/reset. Для финального causal commit не вызывается повторный полный
PhysicalStep restore, который мог бы затереть другие уже установленные owners.

Snapshot хранит real model digest, полную integration vector, owned spatial
record и bytes настоящего tyre state. Все данные очередной границы готовятся
до accounting publication и no-throw commit. Generation входит в committed
boundary. Если после успешного physics capture не завершился, прежний snapshot
остаётся согласованным; дальнейший advance/probe требует успешного reset.
Это recovery contract, а не заявление о rollback уже выполненного physics.

Active hold/steer/outcome dynamics, общий spatial TireWriterView и schema3
typed material parser остаются у будущих W25/W26/W30–W33. В W17 сохранены их
frozen typed records и точки привязки; активные состояния не заменяются
выдуманными producer rows. Existing free-root research packet отклоняется
до W37, потому что его scalar geometry schema не представляет quaternion root.

## CMake и тестовые исходники

Оба CMake graph получают единое подключение через `NativeStartup.cmake`:
`bike_spatial`, малый `bike_rider_spatial`, девять terrain3d translation units,
`model/spatial_topology.cpp` и реальные runtime dependencies.

Неизменённый `tyre/profile.cpp` выделен из core object set в один
`bike_tyre_profile STATIC`, общий для terrain и runtime. Это сохраняет единственного
владельца translation unit и не вводит обратную зависимость terrain → core.
Transition graph также получает ранее написанные `spatial/types.cpp` и
`spatial/config.cpp`. Конфигурация CMake не выполнялась.

Добавлены 19 test executables в CMake: 8 terrain, topology, frames, model,
force_mapping, drive, rider, grip, runtime_state, planar_bridge,
planar_bridge_long и topology_negatives. Tests используют настоящий `bt`
harness, существующий startup-runtime interface и реальные producers.
Long/budget/grip targets маркированы `slow`. Наличие регистраций не является
свидетельством успешной сборки этих targets.

Полные матрицы критериев:

- [terrain](2026-10-22-spatial-3d-native-terrain.md);
- [topology/rider/state](2026-10-22-spatial-3d-topology.md);
- [constrained bridge](2026-10-22-spatial-3d-bridge.md).

## Передача на проверку

`task-index.json` отмечает W06–W19 как `source_written`, V02/V03/V05 — как
`pending_verification`. К consumers G1-core добавлен V05, G2-startup — V03;
status/evidence upstream gates сохранены. Все C10 расхождения и дальнейшие
решения записаны в `migration/spatial3d/contract-deviations.md`.

Проверка самой поставки ограничена чтением исходников, сопоставлением API и
зависимостей, проверкой diff/patch и побайтового результата применения к
неизменяемой распакованной базе. Эти операции не исполняют код проекта и не
заменяют V-проверки. Никакие measured errors, peak RSS, elapsed time, engine
identity или успешные коды выхода тестов в audit не добавлены.

Локальное применение из корня копии, соответствующей входному repomix:

```sh
git apply --check /path/to/bikesim-w06-w19.patch
git apply /path/to/bikesim-w06-w19.patch
```

Если исходные файлы после этого snapshot уже менялись, сначала нужно разрешить
конфликт контекста. Патч не зависит от synthetic Git commit и не требует
переноса временной рабочей history.
