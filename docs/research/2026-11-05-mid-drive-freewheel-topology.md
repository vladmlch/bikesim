# Муфты свободного хода у мотора в каретке: проверка (2026-11-05)

**Вопрос.** Где у моторов в каретке (Bosch Performance Line CX поколения 4, для сравнения Shimano EP8 и Brose) стоят
муфты свободного хода между шатунами, передней звездой и мотором?

**Вывод.** (в) Схемы различаются, а для Bosch CX поколения 4 первоисточник не найден.
- Патенты Shimano [8] и Brose [10] описывают две муфты в узле каретки. Первая стоит между шатуном и звездой: вперёд
  зацепление, назад свободно. Вторая стоит между мотором и звездой.
- Для Bosch CX поколения 4 документа производителя о расположении муфт нет. Косвенные признаки [3], [14] указывают на
  ту же схему.
- Патент Bosch 2024 года [6] описывает другую схему: шатун жёстко связан со звездой, муфта есть только у мотора.

Схеме из `simulation-decisions.md` («педаль вперёд — зацепление, назад — свободно, мотор может крутить звезду при
стоящих педалях») найденное не противоречит. Для Shimano и Brose её подтверждают патенты, для Bosch CX поколения 4 она
не доказана.

**Что это значит для модели.** Это касается пункта 1 раздела «Решено — не сделано» в `simulation-decisions.md` и раздела
«Мотор» в AGENTS.md.
- Описание в AGENTS.md совпадает со схемой Shimano и Brose. Предлагается не менять формулировку, но не называть её
  устройством Bosch CX: для Bosch оно не подтверждено.
- В настоящих приводах есть ещё вторая муфта, между мотором и звездой (вопрос 4). При `rotor_inertia_kgm2 = 0` она
  сводится к условию «момент мотора ≥ 0», и это в модели уже есть.
- Если ротору дадут инерцию больше 0, решение «ротор жёстко сидит на валу за муфтой в моторе» расходится с
  настоящими приводами. Там вторая муфта не даёт педалям крутить ротор при езде без мотора. В этом случае
  пользователю стоит пересмотреть удаление `motor_freewheel`.
- Предупреждение: 2026-10-03 схема, в которой мотор обгонял шатуны (`motor_clutch = true`), давала замирание шатунов
  в верхней точке (`simulation-decisions.md`, «Пробовали — не сработало», пункт 1). Ногам не во что было упираться.
  Переделка по схеме Shimano и Brose устроена так же: мотор может обогнать шатуны. Значит, ровное педалирование
  нужно проверять отдельно.

## Источники

Пункты 1–11 — документы производителей и патенты. Пункты 12–15 — подсказки (пресса, обзоры, форумы), выводом не
считаются.

| # | Источник | Что в нём сказано (цитата или пересказ) | Доступ |
|---|---|---|---|
| 1 | Bosch eBike Systems, «Originalbetriebsanleitung Drive Unit BDU490P \| BDU450 CX», 0 275 007 XD4 (2023.02), стр. «Deutsch – 1» (указания по безопасности) и «Deutsch – 4». Копия у Simplon: https://www.simplon.com/Downloads/Manuals/Komponenten/Bosch_BES2/BDU4xx_DriveUnit.pdf | «Wenn die Schiebehilfe eingeschaltet ist, drehen sich möglicherweise die Pedale mit.» и «Bei der Nutzung der Schiebehilfe können sich die Pedale mitdrehen.» То есть при помощи при ведении педали могут вращаться. О муфтах в документе ничего нет. | 2026-11-05 |
| 2 | Bosch eBike Systems, «Drive Unit BDU3740 \| BDU3741 \| BDU3760 \| BDU3761 \| BDU3780 \| BDU3781» (Performance Line CX, поколение 4, система smart), 0 275 007 3D1 (14.03.2022). Копия у KTM: https://www.ktm-bikes.at/fileadmin/user_upload/Documents/2023_Bosch/User_Manual_PerformanceLineCX_Bosch_SmartSystem.pdf | Та же фраза, что в [1]: «…können sich die Pedale mitdrehen». О муфтах ничего нет. | 2026-11-05 |
| 3 | Bosch eBike Systems, «Drive Unit Performance Line Speed», 0 275 007 PDS (2016.04), поколение 3, указания по безопасности. Копия у Cannondale: https://www.cannondale.com/-/media/files/manual-uploads/manuals/driveunit_201604_0275007pds_45kmh_perfline_du_weu.pdf | «Wenn die Schiebehilfe eingeschaltet ist, drehen sich bei Fahrrädern mit Rücktrittfunktion die Pedale mit.» То есть в поколении 3 педали при помощи при ведении вращаются только у велосипедов с ножным тормозом. | 2026-11-05 |
| 4 | Bosch eBike, страница Performance Line CX, https://www.bosch-ebike.com/at/produkte/performance-line-cx/ | В таблице для BDU384Y (поколение 5) в строке «Rücktrittbremse» стоит «x». Сноска: «Bei der Kettenschaltung gibt es keine Rücktrittfunktion, bei der Nabenschaltung ist sie optional.» Для BDU450 CX таблицы нет. | 2026-11-05 |
| 5 | Патент Robert Bosch GmbH US 12,687,206 B2, «Drive device for a drive unit of an electric bike…», приоритет DE 10 2022 202 694.2 от 2022-03-18. https://patents.google.com/patent/US12687206B2/en, раздел «Summary» и описание к фиг. 1–5 | Звезда «co-rotationally connected» с выходным валом. Роликовая муфта стоит на выходе редуктора мотора, между его последним зубчатым колесом и валом. Она передаёт момент, пока колесо мотора вращается вперёд не медленнее вала, и размыкается, когда вал быстрее. При вращении вала назад ролики свободны, и мотор назад не тянется. Связь вала с шатуном оставлена открытой: жёсткая, одна деталь или «a further freewheeling assembly … between the shaft and the pedal crankshaft», чтобы «prevent any conjoint rotation of the pedal crankshaft». | 2026-11-05 |
| 6 | Патент Robert Bosch GmbH EP 4 610 157 A1, «Antriebseinheit eines Fahrzeugs mit einer Freilaufanordnung», приоритет DE 10 2024 105 418.2 от 2024-02-27. https://patents.google.com/patent/EP4610157A1/de | Звезда закреплена прямо на валу шатунов: «an welchem ein Abtriebselement 107, welches insbesondere ein Kettenblatt ist, drehfest befestigbar ist». Муфта одна, «Motorfreilauf», между последним колесом редуктора и валом: «Zwischen dem Abtriebselement 2 und der Welle 3 befindet sich ein Freilauf 4». Муфта двусторонняя: «kann in beide relativen Drehrichtungen … öffnen». Сепаратор поворачивается трением о корпус. | 2026-11-05 |
| 7 | Патент Robert Bosch GmbH EP 4 598 804 A1, «Antriebseinheit eines mit Muskelkraft und/oder Motorkraft betreibbaren Fahrzeugs», приоритет DE 10 2022 210 636 от 2022-10-07. https://patents.google.com/patent/EP4598804A1/de | Звезда на валу шатунов. Муфта 5 стоит между колесом редуктора и валом и «sperrt … in Antriebsrichtung des Motors». Вариант без чертежа: звезда на полом валу, муфта работает «als Fahrerfreilauf». | 2026-11-05 |
| 8 | Патент Shimano Inc. US 11,167,817 B2, «Bicycle drive unit», приоритет JP 2017-199779 от 2017-10-13. https://patents.google.com/patent/US11167817B2/en | Первая муфта 72 стоит между шатунным валом и полым выходом со звездой. При вращении вперёд она передаёт момент, при вращении назад «does not transmit the rotation of the crankshaft 64 to the output part 68». Вторая муфта 74 стоит между колесом редуктора мотора и выходом, она «prevents rotational force of the crankshaft and the output part from being transmitted to the propulsion assist motor». Первую муфту «can be omitted», тогда вал и выход жёстко связаны. | 2026-11-05 |
| 9 | Shimano, «Dealer's Manual EP800 Series» DM-EP800-01 (DU-EP800). Копия: https://bikefun.hu/downloadFile/36812/EP8_Seria%20EP800_DM-EP800-01-ENG.pdf | О муфтах, о вращении шатунов при помощи при ведении и о кручении назад не сказано ничего. Официальные копии DM-EP800-04 и UM-7HC0B на si.shimano.com вернули отказ доступа (403). | 2026-11-05 |
| 10 | Патент Brose Fahrzeugteile WO 2013/156445 A1 (семейство: US 9,302,734 B2, EP 2 838 783 B1), «Antriebseinrichtung für ein Elektrorad», приоритет DE 10 2012 103 355 от 2012-04-17. https://patents.google.com/patent/WO2013156445A1/de | Шатунный вал связан с полым валом со звездой через муфту 81 с клиньями, она замыкается при вращении вперёд. Мотор передаёт момент на полый вал через вторую муфту 82, когда его колесо быстрее полого вала. При работе одного мотора: «Dabei kann sich die Tretkurbelwelle 2 im Stillstand befinden», «so dass kein Drehmoment vom elektromotorischen Antrieb auf die Tretkurbelwelle 2 übertragen wird». Для ножного тормоза муфту 81 нужно перемыкать: она не пропускает вращение назад. | 2026-11-05 |
| 11 | FIT (Smartscale NewCo AG), «Betriebsanleitung Motor Brose S Mag FIT», V2, стр. 5. https://fit-ebike.com/media/77/ec/9a/1750766151/FIT_Betriebsanleitung_Motor_Brose_S_Mag_V2_de.pdf | «Wenn die Schiebehilfe eingeschaltet ist, drehen sich möglicherweise die Pedale mit.» Это документ сборщика велосипеда, а не Brose. | 2026-11-05 |
| 12 | Bosch eBike, «New features for May 2026», https://www.bosch-ebike.com/us/news/newfeatures-2026-05 | Функция Drivetrain Tensioner: программа «tension the drivetrain» и не даёт «the motor and rear wheel from freewheeling». Какая муфта имеется в виду, не сказано. | 2026-11-05 |
| 13 | Пресса о Bosch CX поколения 5 со ссылкой на пресс-релиз Bosch: velojournal.ch (https://www.velojournal.ch/aktuell/nachrichten/detail/bosch-performance-line-cx-neues-system-am-start/), bikeboard.at «PM Bosch», 10.10.2024 (https://bikeboard.at/magazin/bosch-performance-line-cx-motor-gen5-th11113) | «Beim Bergabfahren ist der externe Antriebsstrang (Kette, Kettenblatt, Kassette) vom internen Antriebsstrang (Freilauf, Getriebe, Motor) entkoppelt.» Сам пресс-релиз на bosch-presse.de найти не удалось. | 2026-11-05 |
| 14 | Форум EMTB Forums: «Bosch Gen 4 free wheel» (https://www.emtbforums.com/threads/bosch-gen-4-free-wheel.34900/) и «Pedals spinning freely clockwise» (https://www.emtbforums.com/threads/pedals-spinning-freely-clockwise.34999/), BDU450 CX | Ремонтник (Bearing Man) и владельцы пишут, что у исправного мотора поколения 4 шатуны крутятся назад без движения звезды и цепи, а храповик звонко щёлкает. В моторе есть «freewheel sleeve» на шатунном валу и отдельная роликовая муфта мотора. Неисправность: шатуны вперёд прокручиваются без звезды. | 2026-11-05 |
| 15 | Обзоры Bosch CX поколения 4: MBR, 2019-06-18 (https://www.mbr.co.uk/news/bosch-performance-line-cx-388235), Flow MTB (https://flowmountainbike.com/tests/tuning-tips-bosch-smart-system-emtb-review/) | MBR: мотор полностью отсоединяется от передачи, когда не помогает. Flow: стук на спусках вызывает цепь, которая тянет назад «the internal freewheel mechanism». | 2026-11-05 |

## Разбор

### Схема модели сейчас

Схема описана в `native/src/model/physical_topology.cpp:117-136` и `examples/research/viewer_physics_welded.toml:26-37`.
- При `motor_clutch = false` шатун жёстко связан со звездой: передача на заднюю втулку идёт прямо от `crank_spin`, и
  муфты `crank_clutch` нет.
- Мотор `mid_drive` приложен к оси шатуна. При `rotor_inertia_kgm2 = 0` ротора как тела нет, и муфта мотора сводится к
  условию «момент мотора ≥ 0» (`ctrlrange` от 0 до наибольшего момента).

Целевая схема в `simulation-decisions.md`, пункт 1: муфта шатун → звезда, вперёд зацепление, назад свободно; мотор
крутит звезду при стоящих педалях.

### Вопрос 1. Стоит ли муфта между шатунами и звездой?

- **Shimano:** да [8]. Ссылка на модель EP8 в патенте не указана, но заявка 2017 года подана до выхода EP8 (2020).
- **Brose:** да [10]. Устройство с ремнём и планетарным рядом похоже на Brose Drive, но прямой привязки к модели S Mag
  в патенте нет.
- **Bosch CX поколения 4:** не найдено.
  - За «да»: подсказки [14] и косвенно [3]. В поколении 3 педали при помощи при ведении вращались только у
    велосипедов с ножным тормозом, а это возможно лишь при муфте у шатуна.
  - За «нет»: поздний патент Bosch [6], в котором звезда жёстко закреплена на валу шатунов. С моделью поколения 4
    он не связан.
  - Не решает: [5] допускает оба варианта.

### Вопрос 2. Крутится ли звезда при кручении педалей назад?

- **Shimano:** нет. Муфта 72 не передаёт вращение назад [8].
- **Brose:** нет. Муфта 81 не пропускает вращение назад, для ножного тормоза её перемыкают [10].
- **Bosch CX поколения 4:** не найдено.
  - Подсказки [14] говорят «нет»: звезда и цепь стоят.
  - Обратное косвенно следует из [6] (жёсткая связь), но это другая конструкция.

### Вопрос 3. Может ли мотор крутить звезду, когда шатуны стоят?

- **Brose:** да, сказано прямо [10].
- **Shimano:** да. Это следует из устройства муфты 72, которая передаёт только вращение шатуна вперёд [8]. Прямой фразы
  о стоящих шатунах в прочитанной части патента нет.
- **Bosch CX поколения 4:** не найдено.
  - Руководства [1], [2] пишут «педали, возможно, вращаются» при помощи при ведении. Это не решает вопрос: при муфте у
    шатуна педали могут увлекаться трением.
  - В [6] (поздняя конструкция) мотор крутит звезду только вместе с шатунами.
  - Такие же уклончивые слова есть у Brose [11], хотя по патенту [10] шатун там отсоединён.

### Вопрос 4. Есть ли вторая муфта между мотором и звездой?

- **Shimano:** да [8], муфта 74 прямо для того, чтобы педали не крутили мотор.
- **Brose:** да [10], муфта 82.
- **Bosch:** во всех найденных патентах Bosch муфта мотора стоит между редуктором и валом звезды [5], [6], [7]. В [5]
  и [6] она к тому же не тянет мотор назад при обратном вращении вала.
  - Для поколения 4 прямого документа нет. Подсказки [13] и [15] говорят «да».

### Итог по схеме

- Схема «вперёд — зацепление, назад — свободно, мотор крутит звезду при стоящих педалях» совпадает с патентами
  Shimano и Brose. Для Bosch CX поколения 4 она правдоподобна, но не доказана.
- Во всех трёх приводах, кроме муфты у шатуна, есть муфта мотора. При нулевой инерции ротора модель её уже учитывает
  условием «момент ≥ 0».

## Не найдено / неуверенно

- **Схема Bosch CX поколения 4.** Сервисного руководства Bosch и патента, прямо относящегося к поколению 4 (BDU450 CX,
  BDU374x), найти не удалось. Пресс-релиз Bosch 2019 года и каталог Bosch 2020 года не нашлись. Каталог 2021 года
  слишком велик для загрузки (больше 10 МБ). Вывод по Bosch держится на подсказках [14] и косвенном [3].
- **Связь патентов с моделями.** Патент не говорит, в какой серийный мотор он вошёл. Связь [8] с EP8, [10] с Brose
  Drive S Mag и [5], [6] с Bosch CX поколения 5 не подтверждена.
- **Руководство Shimano.** Руководства пользователя Shimano EP8 (UM-7HC0B) и новая редакция руководства для
  дилеров на сайте Shimano недоступны (403). Вращаются ли шатуны при помощи при ведении у EP8, по документу Shimano
  не проверено.
- **Документ Brose.** Руководство [11] составил сборщик FIT, а не Brose. Документ самого Brose об устройстве муфт не
  найден.
- **Пресс-релиз Bosch о поколении 5** известен только в пересказе прессы [13].
- **Параметры моторов** (момент, мощность, задержка, коэффициенты режимов) по условию задачи не искались.
