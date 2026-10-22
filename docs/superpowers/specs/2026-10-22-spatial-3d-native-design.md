# Пространственная симуляция велосипеда: актуальная native-спецификация

Дата: **22 октября 2026**. Статус: проектное решение для реализации; **не отчёт о выполненной реализации или проверках**.

Основной план: `docs/superpowers/plans/2026-10-22-spatial-3d-native-master.md`. Контракты: `docs/superpowers/plans/2026-10-22-spatial-3d-native-contracts.md`.

## 1. Назначение и приоритет документов

Эта спецификация заменяет архитектурные и организационные положения `2026-10-14-spatial-3d-design.md`, расходящиеся с переходом на C++. Она сохраняет цель пространственной физики и расширяет описание тех мест, где старые решения недостаточны для реализации. Исторические документы не удаляются и не выдаются за текущий статус.

Порядок разрешения противоречий:

1. Текущие явные требования пользователя: сейчас подготовить план, ничего из проекта не компилировать и не запускать.
2. Эта спецификация и её новый master — целевое поведение 3D и порядок работ.
3. `docs/adr/0003-native-authoritative-python-removal.md`, актуальные требования `2026-10-20-cpp-linux-completion.md`, `docs/agents/native-engineering.md` — миграция, численная дисциплина и квалификация.
4. Замороженные контракты производителей из PFC и документы конкретных S-задач.
5. Старый пространственный план — происхождение требований, а не инструкция создавать Python-модули.

Новая спецификация не объявляет выполненными требования PFC и не отменяет критерии её завершения. Исторические счётчики коммитов, worktree и незакоммиченных файлов остаются свидетельствами аудита на его момент времени.

## 2. Конечный результат

Нативная C++-программа из обычных TOML-конфигураций создаёт велосипед, райдера и трассу, подготавливает начальное состояние, выполняет физический пространственный заезд, показывает его в viewer, записывает необходимые состояния и воспроизводит проверяемый replay. Python не участвует ни в новом генераторе, ни в runtime, ни в новых тестах и инструментах.

Основные физические возможности:

- Свободное поступательное и вращательное движение рамы; управляемый угол руля; подвеска, педалирование и motor assist существующего физического режима.
- Шина с продольной и боковой деформацией/сцеплением, camber, combined slip, амортизацией, rolling loss и контактом с поперечно неоднородной поверхностью.
- Райдер остаётся сидящим с существующей продольной артикуляцией. Для работы руления допускается минимальное пространственное изменение плеч/хвата, доказанное кинематическим тестом. Новый контроллер бокового перемещения таза и корпуса в объём работ не входит.
- Балансировка внутренним рулевым моментом с ограничением, задержкой и учётом энергии; никаких поддерживающих внешних сил после начала измеряемого эпизода.
- Шестиосевая IMU в заданном месте и ориентации, сохраняющая влияние roll/yaw, поперечных ускорений и смещения датчика относительно центра тела.
- Native research, native policy boundary, запись, inspection/playback и отдельно checked replay.

Приоритет исследовательских эффектов: корректный IMU-сигнал → изменение нагрузок от бокового рельефа → конкуренция продольного и бокового сцепления → контролируемый устойчивый заезд. Разработка anti-wheelie политики остаётся за пределами этого плана.

### 2.1 Что означает «3D terrain»

Физическое движение — пространственное. Поверхность первой версии — однозначная высота `z = h(x,y)`, то есть heightfield. У неё есть поперечный уклон, колеи, камни и корни, но нет нависающих поверхностей, пещер, движущихся камней и деформируемого грунта. Это осознанная граница модели, а не ограничение камеры.

Основной маршрут — прямой коридор. Криволинейная трасса, path following и общий route planner не добавляются. Для кругового benchmark используется отдельная достаточно большая площадка, не узкий коридор.

### 2.2 Сохранённые режимы

| Режим | Поведение |
|---|---|
| Старый physical planar | Прежние входы, численный порядок и применимые goldens; отсутствие `spatial` означает planar |
| Старый legacy | Сохраняется миграцией; пространственная физика на него не распространяется |
| Spatial constrained | Корень `x,y,z,pitch`, без roll/yaw/steer; стенд для подключения terrain/шины/состояния |
| Spatial free | Freejoint рамы, соответствующая топология райдера, руление; основной результат |
| Benchmark Whipple | Отдельный тестовый режим с явно ограниченным составом и идеализированными контактами |

`constrained` не является доказательством устойчивости свободного велосипеда. Нельзя использовать его результат для заявления, что free-root готов.

## 3. Архитектура, которую расширяем

В предоставленном снимке уже существуют следующие производители:

| Граница | Существующий интерфейс | Действие 3D |
|---|---|---|
| Конфигурация | `bike::configuration::SimulationPhysicsConfig`, `configuration/load.*`, `schema.cpp` | Добавить строго проверяемое пространственное расширение |
| Модель | `bike::model::BuildRequest`, `generate_mjcf()` | Ввести root/steer/rider/terrain variants в существующем builder |
| Terrain | `bike::terrain::TrackSpec`, `HeightFieldSpec`, `profile`, `generators` | Добавить native Grid3d и native generator3d, оставить 1D ветку |
| Начальное состояние | `SetupRequest`, `make_setup`, `settle`, `refine_equilibrium`, `SetupOwner::seal_bootstrap` | Расширить free-root и material state |
| Работающий runtime | `runtime::RuntimeBootstrap`, `NativeRideRuntime`, `PhysicalStep` | Добавить topology-selected writers и полное состояние |
| Research | `NativeResearchRuntime`, `bike::research::prepare_environment` | Добавить observation/policy V2 и spatial truth |
| Запись | `bike::recording`, `bike::replay`, `bike::golden` | Новые поля, assets, versioned state и replay compatibility |
| Приложение | `native/src/cli`, `viewer`, `viz`, `app` | Нативные параметры, камера, графики и запуск |

Эта таблица подтверждает наличие исходных интерфейсов, а не их прошедшую квалификацию. Точный gate для каждого производителя описан в master. Не создаются второй `RuntimeBootstrap`, независимый `runtime/equilibrium` и второй общий replay engine.

Новые пространственные типы живут в `native/src/spatial/`; новые tyre kernels — в `native/src/pneumatic3d/`. Существующие пространства имён `runtime`, `topology`, `nativecfg` не переименовываются массово. Новые типы используют `bike::spatial` и `bike::pneumatic3d`; terrain остаётся в `bike::terrain`.

## 4. Конфигурация и совместимость

Сохраняются ключи `physics_mode = "physical"`, `spatial = "3d"`, `[spatial_3d]`, `tires.backend = "pneumatic_3d"`. `spatial = "planar"` — значение по умолчанию. Новые поля в старые canonical документы и fingerprints не добавляются только ради нового default.

Пример ниже — **целевая форма конфигурации после реализации S05/S31/S45**, не команда для текущего снимка:

```toml
physics_mode = "physical"
spatial = "3d"

[spatial_3d]
root = "free"
corridor_half_width_m = 1.5
capsize_roll_rad = 1.0471975511965976

[spatial_3d.steer]
controller = "roll_heading"
range_deg = 60.0
damping_nms_rad = 0.5
reaction_delay_s = 0.0
torque_cap_nm = "strength"

[spatial_3d.bootstrap]
settle_time_s = 80.0

[spatial_3d.imu]
mount_pos_m = "site_BB"
mount_rpy_rad = [0.0, 0.0, 0.0]
axis_misalignment_rad = [0.0, 0.0, 0.0]

[tires]
backend = "pneumatic_3d"

[tires.pneumatic_3d]
tier = "fast"
lateral_coefficients = "mtb-provisional-v1"
```

Название `mtb-provisional-v1` является **предлагаемым новым профилем**, а не найденной калибровкой. Его коэффициенты, область применимости и численные диапазоны записываются в S03/S28 на основе доступных измерений. До этого профиль не становится default для сертифицированной физики. Сам факт указания названия не позволяет подставить произвольные константы.

Не переносить стартовые gain-числа из старого плана в роль доказанно устойчивого контроллера. Нужны однозначные знаки, ограничения, собственные измерения и область скорости, на которой контроллер принят.

### 4.1 Отказ вместо скрытого fallback

Ошибки должны называть полный ключ и конфликтующие значения. Запрещены сочетания: legacy+3D; 3D+старый planar tyre backend; planar+3D table; constrained+steer; неизвестный coefficient profile; несовместимая spatial state version; asset с неверным digest; schema V1 policy на observation V2 без явного поддержанного адаптера.

Для стартового этапа адаптер V1→V2 **не создаётся**: V1 plugin на spatial получает понятный отказ. Встроенные C++ policies получают V2 branch с прежним смыслом demand/control. Существующий plugin V1 и его байтовая раскладка остаются прежними.

Принятие нового ключа parser'ом не означает доступности функции. Capability меняется на supported только в интеграционной задаче, у которой есть реальные тесты пути от public input до результата.

## 5. Геометрия, координаты и топология

Derived world frame: `+x` вперёд по начальному коридору, `+y` влево, `+z` вверх. Ориентация рамы — quaternion `w,x,y,z`; его смысл, направление преобразования и порядок умножения фиксируются в контракте. Raw MuJoCo qpos/qvel сохраняют собственные conventions.

Для right-handed ZYX Euler положительный pitch означает наклон носом вниз, положительный roll — наклон вправо. Старый `pitch_up_rad` сохраняет противоположный pitch знак в области определения. Euler angles нужны для UI/metrics; интегрировать их вместо quaternion нельзя. У singularity должен быть явный статус недостоверности производных углов, а не скрытый скачок в управляющем сигнале.

`qpos` и `qvel` имеют разные адреса и размеры. Freejoint даёт семь позиционных и шесть скоростных координат; ball joint — четыре и три. Поэтому глобальное равенство `nq = nv + 2` тоже неприемлемо: число free/ball joints зависит от выбранного rider model. Descriptor строится по реальному `mjModel`, именам, joint types и addresses.

Body angular velocity берётся через документированный engine accessor или Jacobian, в явно выбранном frame. Нельзя трактовать любые три компоненты free-root `qvel` как произвольный body/world gyro. Скорость хода для логики райдера/rollback — проекция на выбранную продольную ось; world `vx` для развернутого велосипеда неверен.

Обязательные преобразования для drivetrain и tyre:

```text
wheel_spin_rate = dot(omega_wheel_world - omega_hub_world, axle_world)
wheel_axis_moment = dot(axle_world, cross(contact_world - hub_world, force_world) + couple_world)
point_velocity = J_point(q) * qvel
generalized_force = J_point^T * force_world + J_rotation^T * couple_world
```

Virtual-work test проверяет согласованность силы, момента, точки и Jacobian. Вращение всей flat-установки вокруг world Z не должно менять физику в локальных координатах.

## 6. Terrain и запрос контакта

Начальная production storage — binary64. Переход на float32 допускается позднее как отдельное измеренное изменение; FP32 worktree не является частью этого плана. Размер типового массива 300 m × 3 m при 5 mm × 10 mm: `60001 × 301 × 8 = 144482408` байт только на высоты. В оценке памяти учитываются дополнительные arrays, coefficients, material map, hfield и peak generation memory.

Grid имеет явную версию, units, grid origin/spacing, размеры, порядок элементов, тип высот, surface table и digest. Бинарный parser проверяет переполнение `nx*ny*element_size`, длину файла, finite значения, положительные шаги, допустимые material IDs и разумный resource limit **до выделения памяти**. Никаких cast файла в native struct с платформенным padding.

Предпочтительное sampling сохраняет Catmull–Rom из старой спецификации, но высота и градиент выводятся из одной полиномиальной поверхности. Overshoot на скалах/краях проверяется как часть generator qualification. Нельзя независимо clamp'ить высоту или нормаль и оставлять их несогласованными.

Ray casting не основывается только на знаках значения в концах ячейки: кубическая по каждой оси поверхность может пересечь наклонный луч внутри ячейки несколько раз. Требуются поиск ближайшего допустимого корня, случаи касания и вертикального луча, ограничение итераций и явное `budget_exhausted`. Исчерпание бюджета не приравнивается к airborne. Точные алгоритм и численные критерии находятся в S08.

Out-of-bounds имеет статус; clamped border допустим только для отображения, а не как бесконечное физическое продолжение дороги. Отдельно определены invalid query, start-below-surface и no-intersection. Верхняя граница обхода выводится из размеров grid и проекции нормированного луча, а не из `t_max/min(dx,dy)` без проверки направления.

Для planar bridge используется adapter к исходному профилю, а не обещание совпадения нового spline с любой старой кусочно-линейной поверхностью. Байт-в-байт совпадает делегированный старый запрос; аппроксимация materialized grid получает отдельную оценку ошибки.

Generator3d работает на C++ и использует существующий native RNG слой. Новый 2D генератор имеет свой algorithm version и seed/draw ordering. Он не обязан воспроизводить никогда не существовавший Python generator3d. Новое потребление RNG не изменяет planar stream.

## 7. Шина

### 7.1 Геометрия и нормальная реакция

Модель покрытия — тороидальная аппроксимация без детализации протектора. Кольцо, сечение, circumferential/lateral grid и направление луча задаются в wheel-local frame и затем преобразуются в world. Геометрическая площадь тороидального элемента и площадь, проектируемая на нормаль контакта, различаются. Старую формулу с `R_ring * r_s * cos(phi)` нельзя без вывода назвать полной метрикой тора.

Нормальная сила относится к нормали поверхности. Поперечную проекцию произвольно направленной radial force нельзя одновременно засчитать как normal support и ещё раз добавить как camber thrust. Уравнение normal law в S21 должно иметь согласованные размерности и нормировку, не зависящую от количества лучей.

Повторно используются математические primitive'ы pneumatic kernels, где их контракт подходит. Нельзя просто умножить старую одномерную нагрузку на количество lateral rays. Если общий primitive нельзя извлечь с сохранением planar arithmetic, spatial adapter получает отдельный короткий вариант с объяснённой формулой, а не меняет старую шину.

### 7.2 Сцепление и энергия

У patch есть касательная система координат, продольный и поперечный slip, camber и совместная граница трения. Продольное тяговое усилие, торможение и боковое удержание расходуют **один** friction budget. После добавления camber force итог снова обязан находиться внутри выбранной friction ellipse.

`Fz*tan(camber)` не вводится как универсальный закон максимальной camber force. Такой ограничитель не выведен из используемой модели и в старой записи ошибочно обрабатывает отрицательный угол. Camber term должен менять знак при зеркальном преобразовании; его область определяется измерениями/калибровкой и общим friction budget.

На нулевой скорости нельзя прятать singularity slip ratio одной большой константой. Нужны continuous low-speed dynamics/material state и отдельные тесты stop/start/reverse, удержания на уклоне и отпускания тормоза. В fast и detailed tiers хранимые энергии/потери различаются, но оба обязаны объяснять свой баланс.

Диссипативность проверяется с учётом изменения **упругой** энергии carcass/bristles, а не условием `F dot v <= 0` для любой силы в каждый момент: упругий элемент вправе возвращать накопленную энергию. Camber и aligning couple также участвуют в power balance.

### 7.3 История материала и patch lifecycle

Состояние хранит не только current force, но и material deformation, Maxwell branches, hysteresis, relaxation, bristles, mapping/advection и patch identities. Connected components могут split/merge, исчезнуть, поменять порядок; индекс patch в текущем кадре не является его устойчивой историей.

Стартовая граница patch capacity вычисляется из grid/tier и заранее резервируется. Если выбирается меньший конфигурационный предел, overflow приводит к invalid diagnostic, а не потере пятна или тихому слиянию. Не фиксировать необоснованные восемь пятен только потому, что они помещаются в старую таблицу.

Неактивные state slots имеют zero/canonical значения и valid mask. NaN не используется как универсальная заглушка для checkpoint, который обязуется пройти finite checked replay. Изменение schema explicit, чтобы старые state rows не читались по новой ширине.

### 7.4 Тормоз

Существующий wheel-hinge brake и контактная статика — разные физические элементы. Brake передаёт внутренний момент между колесом и его опорой; patch передаёт силу от дороги. Удержание не реализуется пружиной от рамы к мировой точке.

Для пространственного пути нужно сохранить latch тормоза при snapshot/restore, учесть совместный torque cap frictionloss+hold, разделить elastic storage и потери, не дублировать работу торможения в шине. Потенциальные аналогичные ошибки planar исправляются отдельной задачей PFC с её review/golden procedure, без тихого изменения baseline ради 3D.

## 8. Райдер, руление и начальное состояние

### 8.1 Хват и руление

Существующие hinges рук вокруг body-Y и contact constraints сами по себе не гарантируют хват при рулении. До заявления о диапазоне ±60° проверяются ±5°, ±15°, ±30° и ±60°: остаток позиционного хвата, ранги Jacobian, реакции, отсутствие блокировки рулевого DOF и перенос мощности между руками/рулём/телом.

Выбранное изменение объёма: разрешены минимальные пространственные DOF плеч/локтей/хвата, необходимые для сохранения контакта кисти с рукояткой. Они не превращаются в отдельную задачу балансировки боковыми движениями корпуса. Пространственные соединения ограничены физически разумными степенями свободы и усилиями; бесконечно жёсткий weld, одновременно задающий несовместимые ориентации, не используется как исправление.

### 8.2 Equilibrium и bootstrap

Используется `native/src/initialization/`. Optimize/refine работает в tangent coordinates размера `nv`: increment применяется к `qpos` через корректную операцию на manifold, quaternion нормирован. Нельзя удалять проверку `nq == nv`, оставляя прежнее индексирование неизвестных.

Начальная стадия явно отделена от scored runtime: `settling -> release_check -> armed -> scored`. Для обозначения bootstrap не требуется отрицательное MuJoCo time; статус стадии и отдельное время достаточно однозначны. Hold допускается только при settling/release_check и не захватывается как активная scored force.

Hold удерживает roll/yaw/lateral при свободных sag/pitch/vertical DOF. На выходе проверяются finite состояние, residual, запасы допустимых контактов, отсутствие скрытой остаточной силы и порог hold wrench менее 1 N и 1 N·m. Это планируемый численный порог старой спецификации; его прохождение не проверялось в текущей работе.

Измеряемый эпизод начинается с корректной скорости либо честно заканчивается `balance_lost` на низкой скорости. Не добавляется невидимый world push, который помогает велосипеду проехать первые метры. Cold/fresh startup, cache-hit и explicit initial-state import сравниваются на одной физической постановке.

### 8.3 Контроллер

Рулевой момент — внутренний torque на steer joint; его величина, saturation и энергообмен входят в telemetry. Управление вычисляется по integer control ticks; физический timestep и частота control period либо имеют доказанную рациональную привязку, либо поддерживается существующий interval clock. Float modulo времени не добавляется.

Входы: roll/roll-rate, heading/lateral offset, steer state и speed; коэффициенты и speed schedule — versioned profile. Для тестов разрешены reference heading/yaw rate в benchmark context. Source truth доступна этому контроллеру как модели райдера; анти-wheelie policy получает только разрешённую sensor observation и demand.

Требуются отдельные тесты sign, delay, saturation, disable/reset/restore и отклонений на малой скорости. Правильный знак мгновенного момента не доказывает динамическую устойчивость. Область работы оценивается в S32/S36.

## 9. IMU, observation, policy, truth

V1 остаётся прежним: девять data/noise чисел. V2 содержит одиннадцать: accel XYZ, gyro XYZ, front/rear/crank speed, motor/human torque. Dropout draws хранятся отдельным потоком. Schema1/Schema2 RNG и очереди не смешиваются.

Для V2 используются имена `specific_force_sensor_mps2` и `angular_rate_sensor_rad_s`: это векторы в системе датчика. Configured mount задаёт pose site; misalignment задаёт документированное отклонение осей до bias/noise. Метаданные позволяют восстановить nominal/body transformation, но policy не получает привилегированный body orientation автоматически.

Sensor timestamp описывает **момент физического измерения**, delivery timestamp — его доставку. Native pipeline определяет stage, порядок forces/acceleration/sensors/step, startup sample, latency rounding, dropout, age и valid mask. Недопустимо брать gyro из одной стадии, accelerometer из другой и назвать их синхронной IMU.

При смещении mount тест учитывает tangential и centripetal acceleration. Равенство default IMU и `site_BB` допускается только для совпадающих pose, stage и арифметики. Старый scalar pitch-rate со знаком «нос вверх» не равен автоматически одной новой gyro component в общем 3D движении.

WheelieTruth использует контактную геометрию и нагрузки в объявленных frames. На side slope мировой Z-компонент силы и нормальная нагрузка — разные числа. Front lift, capsize, off-corridor и отсутствие grid contact различаются; wheelie metric не засчитывает лежащий на боку велосипед как успешный подъём переднего колеса.

## 10. State, запись и replay

Independent version axes:

- configuration/spatial contract;
- `.terrain3d` asset;
- topology/model fingerprint;
- tyre material state;
- bootstrap/checkpoint;
- observation schema;
- policy ABI;
- recording database/container;
- replay contract;
- golden manifest and numeric identity.

Увеличение observation до V2 не переименовывает все остальные форматы в «schema 2». В этом плане выбрано конкретное разделение: существующий planar путь сохраняет recording container 1 / catalog 1 / replay contract 3; новый spatial путь получает container 2 / catalog 2 / replay contract 4. `.terrain3d` начинается с собственной версии 1, пространственный checkpoint — с версии 1, observation и policy ABI — с независимых V2. Эти номера не являются общим счётчиком. Полная матрица версий и decoder dispatch находятся в contracts §C7 и S41–S43; reader принимает старую запись по её прежним правилам.

Полный checkpoint включает engine integration state и warmstart, mutable model coefficients, tyre material/patch state, brake latch, steering state и clock, control delay queue, rider/policy state, sensor queue/noise cursor, counters/outcomes/energy. Viewer RenderFrame не является полным checkpoint.

Replay asset set содержит terrain bytes/hash, config, model identity, coefficient profile, initial state, commands, noise/provenance. Reader не должен зависеть от сегодняшней версии внешнего файла с тем же именем.

Inspection, playback и checked replay сохраняют разные обещания. Несовместимый build/provider не превращается автоматически в verified replay. Для старых записей сохраняются два существующих численных правила сравнения: evidence `max(atol, rtol*max(|a|,|e|))`, state `atol + rtol*|e|`, где сейчас оба допуска равны `1e-9`. Новые 3D поля входят в review comparator, а не получают один случайный абсолютный epsilon.

## 11. Корректность и реалистичность

Проверки состоят из независимых слоёв:

1. **Planar preservation:** действующие per-platform regression gates по ADR-0003; остающаяся live Python parity до соответствующего capture/port.
2. **Exact unit contracts:** units/frames/addresses, analytic terrain, force mapping, codec transactionality, invalid input, deterministic state.
3. **Physical invariants:** finite values, нормальная реакция, friction budget, symmetry/covariance, momentum/power/energy, no scored hold.
4. **Independent benchmark:** Whipple/Meijaard для сопоставимой идеализированной постановки, tyre literature только в области исходных измерений.
5. **Numerical convergence:** timestep, ray resolution, grid resolution и solver tolerances; сравнение fast/detailed без предположения, что detailed автоматически точен.
6. **Target validation:** заезды на rough uphill/Alpine, начало с нуля и на скорости, торможение, поворот/наклон, наблюдаемость IMU.
7. **Reproducibility and performance:** state roundtrip, replay, same-build repeats/threads, квалифицированные platform lanes, real display.

### 11.1 Что доказывает Meijaard

Исходная модель — четыре жёстких тела с идеализированным качением, а не полный MTB с деформируемыми шинами, подвеской и артикулированным райдером. Поэтому независимый fixture проверяет знаки, инерцию и динамические связи сопоставимой постановки. Отдельно проверяется связь с общими spatial code paths; автономная реализация только опубликованной матрицы не закрывает весь runtime.

Нельзя объявлять `stiffness *= 1000` математически эквивалентным идеальной nonholonomic constraint. Предел контакта должен иметь отдельное доказательство сходимости или benchmark использует корректный ограниченный adapter. Near-zero mode не квалифицируется коротким fit одной трёхсекундной траектории; linearization и независимая проверка eigenmodes — основной путь S34.

### 11.2 Калибровка low-pressure MTB

Данные city/cargo tyres не объявляются измерениями Magic Mary/Hans Dampf при низком давлении. У каждого coefficient set есть provenance, valid domain и статус: measured/calibrated/inferred/provisional. Если подходящих измерений нет, технически корректный spatial runtime может быть готов, но строка «реалистичная lateral tyre calibration» остаётся незакрытой. Разрешено явно помеченное exploratory использование; запрещено скрыть этот пробел зелёным self-golden.

## 12. Производительность и потоки

Целевые fast-tier показатели сохраняются: headless RTF ≥ 2, viewer sustainable ≥ 1×. Это требования будущего измерения на записанной hardware/configuration, не результаты аудита. Detailed tier измеряется, но не обязан быть realtime.

RTF вычисляется по полному валидному эпизоду с установленной retention/decimation configuration. Проводятся warmup и повторные прогоны, учитываются initialization отдельно, steady step time, p95/p99, worst-frame/lag и memory. Ранний invalid prefix под `--benchmark` не закрывает ни физическую, ни производительную приёмку.

По умолчанию один поток. Parallel elements допускаются, когда профиль показывает полезность: заранее выделенные arrays, фиксированные element IDs, редукция в порядке, не зависящем от количества потоков, один engine owner. Worker не пишет в общий `mjData`, не вызывает непроверенные engine операции и не мутирует общие scratch buffers. Stop/reset/exception/close races входят в testing.

Thread-count invariance обещается для одной executable/compiler/provider/config identity. Она не означает побитового равенства macOS и Linux. MuJoCo engine pool исследуется отдельно: меняющий численный порядок режим получает отдельную numerical identity и не подменяет базовый deterministic profile.

GPU — отдельное ограниченное исследование S55 после знания CPU bottleneck. Оно не блокирует 3D и не требует merge экспериментальной FP32 ветки. Даже полезный GPU prototype должен учитывать transfer/readback, latency, поддержанные engine features и no-Python production boundary.

## 13. Что сознательно не делаем

Anti-wheelie алгоритм; новый legacy 3D режим; lateral torso/hip balance; spline route planner; деформируемый грунт; новый framework для policy plugins; переезд записи на другой storage; переписывание всех старых native modules; смена numerical provider; удаление Python силами spatial-ветки; Windows и новые неподтверждённые platform lanes.

Дополнительная инерционная/аэродинамическая физика не добавляется «заодно». Производственный spatial режим наследует выбранную исходную модель и её явно записанные параметры; benchmark использует свои опубликованные inertial parameters. Изменение существующего упрощения — отдельное физическое решение с собственными regression tests.

## 14. Происхождение решений

Основная кодовая база и старые требования взяты из пользовательского `repomix-output.xml(20261022-154203).zip`. Сводка точных путей и обнаруженных расхождений: `docs/superpowers/audits/2026-10-22-spatial-3d-native-plan-source-audit.md`.

Первичные внешние источники для отдельных границ корректности:

- Meijaard et al., 2007, [авторский PDF](https://arendschwab.com/assets/pdf/meijaard2007linearized.pdf), DOI 10.1098/rspa.2007.1857.
- Dell’Orto et al., [Bicycle Tyre Data — Lateral Characteristics](https://zenodo.org/records/7866646), DOI 10.5281/zenodo.7866646; [публикация в репозитории TU Delft](https://repository.tudelft.nl/record/uuid:8238f5ba-6505-48ac-90cc-4a679be3a670).
- [MuJoCo computation](https://mujoco.readthedocs.io/en/stable/computation/index.html) и [API functions](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html). При реализации используются headers и документация версии из dependency lock проекта, а не молчаливое обновление до latest.

Все численные acceptance thresholds, предложенные в этом пакете, являются критериями проекта. Они не представлены как гарантии из указанных публикаций.
