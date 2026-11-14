# План старта, часть 4: включение плана, критерий Б, строка потерь упора, мягкий старт уходит — план реализации

> **Для исполнителя:** задачи выполняются по одной, в порядке номеров (`superpowers:subagent-driven-development`).
> Шаги помечены `- [ ]`. Финальное ревью ветки делает оркестратор.

Редакция 4 (2026-11-14). Между задачами 4 и 5 вставлена часть 4б
(`docs/superpowers/plans/2026-11-13-launch-4b-crank-hold-damping.md`, коммиты `42a34cef`…`81a3ee26`): удержание
шатуна при поиске позы, активное гашение ноги, угол −0,8 рад и пять клеток таблиц. Решения 5 и 9 и задачи 5–9
поправлены под неё по ревью части 4б (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/part-review-1.md`, I3).

Редакция 3 (2026-11-12; ревью 2 `plan-review-2.md` — «готов с правками», правки N1–N6, m1–m7, D2–D3 внесены. Редакция 2 — после ревью qoder `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/plan-review-1.md`:
«не готов», 4 критических, 8 важных, 7 мелких; исправлены все, кроме B2 — это прямое решение пользователя П13).
Восстановленные списки 33 финальных поездок калибровки проверены побайтным повтором:
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/parity-baseline/`. Факты кода: `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/facts.md` (снят на `22dc1b0d`).
Калибровка: `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/gradecap/report.md` (разделы 14–17), критерий Б:
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/calibration-handB/report.md` (раздел 8), упор корпуса:
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/energy-diag/report.md` (раздел 5), риски:
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/risks2/report.md`.

**Цель.** Включить план старта во всех исследовательских профилях с утверждёнными таблицами трёх типов райдера,
перенести в код три меры, проверенные в пробных поездках (предел нажима по уклону, его подъём после разгона,
повтор просьбы о переключении), заменить признак рук на критерий Б, вывести работу анатомического упора райдера в
отдельную строку потерь, убрать мягкий старт и закрепить пороги тестами поездок.

**Устройство.** Пробные поездки калибровки шли на правках `launch_plan.cpp` и `front_lift_reaction.cpp` через
переменные окружения (`gradecap/_diff_plan_r3.txt`, `gradecap/_diff_reaction.txt`), с историей в статических
переменных файла. Здесь та же арифметика переносится в поля таблиц и в состояние узлов, с кодеками и проверкой
восстановления. Главная проверка переноса — побайтное совпадение записей пробных поездок (решение 9).

**Стек.** C++23, CMake/CTest, `bike_test::run`, кодеки `bike::io::Value`.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md` («Проверки», «Таблица опыта»).
Дорожная карта: `docs/superpowers/plans/2026-11-11-rider-launch-plan.md`, задачи 12–15 (здесь они заменяются).

## Решения плана

Решения пользователя 2026-11-12 (журнал `/Volumes/SSD_1TB_M2/work/mujoco_clean-scratch/devin/2026-11-05-physics-queue/ledger.md`)
пронумерованы П1–П13; решения плана ссылаются на них.

- П1. Опрокидывание на старте убирают поля таблицы: предел, звезда, наклон корпуса.
- П2. Мотор не меняется.
- П3. Признак рук — критерий Б.
- П4. Предел 60 Н·м остаётся пределом среднего за оборот.
- П5. Пороги задачи 13 дорожной карты остаются; добавляется поле «предел по уклону».
- П6. Работа анатомического упора — отдельная строка потерь; сам упор пока не меняется.
- П7. Предел поднимается после разгона (2,5 м/с).
- П8. Подъём колеса после передачи управления — отдельная задача очереди.
- П9. Переключение вниз перед остановкой — отдельная задача очереди; тесты рестарта уходят туда.
- П10. Агрессивный райдер — открытое окно: на 15 и 20 % без опрокидывания.
- П11. Окно старта кончается входом в `handover`. Откат считается от начала `release`. Время до 20 км/ч проверяется
  только у обычного райдера.
- П12. Осторожный на 20 % проверяется по правилу обычного: каждый отрыв не дольше 0,3 с, без опрокидывания.
- П13 (вечер 2026-11-12).
  - Пределы по покрытиям — проверенные, по типам (таблица в решении 5).
  - Опрокидывание на 25 % (обычный: стоит 6 с, передача по таймеру, подъём 1,2 с) — в задачу очереди «предел
    нажима в обычной езде на подъёмах». Здесь 22 и 25 % — информационные строки.
  - Промежуточные уклоны 7/9/11/14/17 % на асфальте, мокром и рыхлом проверяются теми же правилами. Провал —
    стоп и пользователь; пределы не подбираются.

Решения плана:

1. **Предел нажима по уклону.** В `[rider_launch]` два новых поля:
   - `effort_cap_grades` — узлы уклона, строго по возрастанию;
   - `effort_cap_grade_nm` — предел в узлах, Н·м, положительный.

   Между узлами — линейно, за краями — крайние значения, как у `release_torque_nm`. До подъёма потолок плана равен
   `min(предел покрытия, предел по уклону(grade_ahead))`.

   Уклон — задержанный `grade_ahead` из ощущений: так было в пробе.
2. **Подъём предела.** Новое поле `effort_rise_speed_mps`. С первого тика, на котором задержанная скорость
   `senses.speed_mps ≥ effort_rise_speed_mps`, часы подъёма `rise_ticks` (новое поле состояния, −1 — не начат)
   считают тики. Потолок тогда равен
   `min(предел покрытия, предел по уклону + effort_rate_nm_s · rise_ticks · period_s)`.
   - Если скорость падает ниже порога, потолок возвращается к значению «до подъёма», а часы идут дальше.
   - Часы обнуляет только новый старт (`arm`) или сброс состояния.

   Так в пробе: `Rise.start/now`.
3. **Повтор просьбы о переключении.** Новое поле `shift_retry_s` (на сетке такта, положительное) и новое поле
   состояния `retry_tick` (−1 — нет).
   - Отказ автомата (`rejected`) снимает просьбу, запоминает звезду и ставит `retry_tick = launch_ticks + ticks(shift_retry_s)`.
   - Принятое переключение ставит `retry_tick = −1`: звезда заблокирована, пока ощущаемая звезда не сменится, как
     в части 3b.
   - Новая просьба на той же звезде разрешена, когда `retry_tick ≥ 0 && launch_ticks ≥ retry_tick`.
   - Выдача просьбы ставит `retry_tick = −1`.
4. **Критерий Б.** В `[front_lift_reaction]` поля `hand_unload_n` и `unload_dwell_s` удаляются, поле состояния
   `unload_ticks` тоже. Добавляются поля:
   - `hand_window_s` — наибольшее окно, по умолчанию полоборота шатуна;
   - `hand_lag_s` — запаздывание опоры;
   - `hand_drop_ratio` — доля, (0, 1].

   Признак рук на тике с ощущениями:
   - угол шатуна копится как `crank_angle += crank_rpm · (2π/60) · period_s`;
   - окно — образцы после самого нового образца, отстающего по углу не меньше чем на π;
   - если такого нет или окно длиннее `window_ticks`, берётся `min(window_ticks, число образцов)`;
   - среднее нажима по окну кладётся в историю средних;
   - опора — среднее `lag_ticks` тиков назад;
   - признак есть, когда опора > 0 и среднее < `hand_drop_ratio · опора`; без выдержки.

   Признак реакции = признак рук И прежний признак наклона. Фазы, решение, действия не меняются.

   В состоянии хранятся:
   - последние `window_ticks + 1` образцов нажима и угла;
   - последние `lag_ticks + 1` средних;
   - накопленный угол.

   Арифметика и порядок суммирования — как в пробе. Ограниченные очереди дают тот же результат, что и пробные
   неограниченные: самый новый подходящий образец либо лежит в окне, либо окно длиннее предела. Это доказывает
   тест-реплика (задача 2). Таблица Б у трёх типов одна: 0,5 / 0,5 / 0,7 — так калибровали.
5. **Таблицы трёх типов** — дословно блоки `[rider_launch]` и `[front_lift_reaction]` пробных профилей
   `gradecap/profiles/g_{careful,normal,aggressive}_leanm.toml`, а не сводка §17.3: она неверно называла пределы
   покрытий общими (П13). Плюс новые поля. Остальные разделы пробных профилей побайтно равны
   `examples/research/viewer_physics_welded.toml` (проверено вне комментариев и пустых строк).

   | поле | осторожный | обычный | агрессивный |
   |---|---|---|---|
   | `start_cog_teeth` (границы .03/.08/.13/.18) | [28,39,45,39,39] | [24,33,39,33,39] | [21,28,33,28,33] |
   | `start_pedal_angle_rad` | −0.8 (было 0.9) | −0.8 (было 1.12) | −0.8 (было 1.24) |
   | `start_lean_grades` / `start_lean_rad` | [0,0.15] / [0,0.35] | то же | то же |
   | `effort_rate_nm_s` | 40 | 80 | 200 |
   | пределы асфальт/плотный/рыхлый/мокрый/осыпь | 45/45/35/11/30 (мокрый был 35) | 45/60/45/13/40 (мокрый был 45) | 45/60/60/60/60 |
   | `effort_cap_grades` | [0,.05,.08,.12,.13,.15,.2] | [0,.05,.08,.10,.12,.13,.15,.2] | [0,.05,.08,.12,.13,.15,.2] |
   | `effort_cap_grade_nm` | [26,14,11.5,13,14,13,14] (было …,15,18) | [38.5,15,14.5,13.2,14,15.5,19.5,18.5] (было 18 в узле 0,05) | [50,32,24,26,33,39.5,35] |

   Угол и пять клеток изменила часть 4б, решение 6 (2026-11-13, решения пользователя), коммит `3a0048b8`.
   | `effort_rise_speed_mps` | 2.5 | 2.5 | 2.5 |
   | `release_torque_grades` / `_nm` | [0,.05,.1,.15,.2] / [0,7,7,7,7] | … / [0,9,9,9,8] | … / [0,12,12,12,12] |
   | `brake_release_s` / `max_hold_s` | 0.2 / 2.0 | 0.15 / 1.5 | 0.1 / 1.0 |
   | `upshift_crank_rpm` / `shift_retry_s` | 72 / 0.25 | 82 / 0.25 | 95 / 0.25 |
   | `end_speed_mps` / `end_crank_rpm` / `end_hold_s` / `max_launch_s` | 3 / 60 / 0.3 / 6 | то же | то же |
   | `handover_s` | 1.2 | 0.9 | 0.6 |
   | `pitch_rate_up_rad_s` / `pitch_rad` / `decision_s` | 0.105 / 0.06 / 0.28 | 0.15 / 0.09 / 0.2 | 0.21 / 0.12 / 0.1 |
   | `ease_fraction` / `ease_rate_per_s` | 0.9 / 4 | 0.7 / 5 | 0.5 / 6 |
   | `lean_add_rad` / `lean_rate_rad_s` / `clear_s` | 0.08 / 0.5 / 0.65 | 0.15 / 1.0 / 0.4 | 0.2 / 1.5 / 0.25 |
   | `hand_window_s` / `hand_lag_s` / `hand_drop_ratio` | 0.5 / 0.5 / 0.7 | то же | то же |
   | `ease_enabled` / `lean_enabled` / `plan_enabled` | true / true / true | то же | то же |

   Обычный райдер — в трёх исследовательских профилях (`viewer_physics_{welded,distributed,pneumatic}.toml`).
   Осторожный и агрессивный — в новых `examples/research/rider_type_{careful,aggressive}.toml`. Это полные копии
   сварного профиля, отличаются только этими двумя разделами, каждое отличие прокомментировано. Тест
   `test_welded_profile_config` проверяет, что копии отличаются только этими разделами.

   Финальный осторожный набор взят из `ovr/env/ck12_13_c.env` (`.12:13`). `ckfinal_c.env` (`.12:12`) устарел:
   `runs/fck_c_g12.csv` побайтно равен `runs/ck13_c_g12.csv`, в журнале запуска `fck_c_g00` — знамя
   `grade_cap_knots=[0:26,…,0.12:13,…]`.
6. **Встроенный профиль и программные конфигурации остаются выключенными.** Сюда входят:
   - `default_physics()` — встроенный велосипед 34/24 «сегодняшнего старта», не тип райдера;
   - программные конфигурации в `native/src/validation/{rider,drivetrain,scenarios}.cpp`;
   - `native/src/cli/configuration.cpp:211-215`;
   - тестовые документы `tests/fixtures/**` — замороженные входы своих тестов.

   Все они получают новые поля с инертными значениями: `plan_enabled` остаётся `false`, предел по уклону выше всех
   пределов покрытий, подъём и повтор — числа обычного. Значения — в задачах 1 и 2.
7. **Строка потерь упора (П6).** Строки ограничений MuJoCo типа `mjCNSTR_LIMIT_JOINT` у шарнира, имя которого
   начинается с `rider_`, получают ключ `rider_end_stop:<id>` вместо `joint_limit:<id>`. Это ровно шарниры с
   анатомическим пределом (`articulated_rider.cpp:38-53`), доказывает тест задачи 3.
   - `raw_work` не кладёт эти строки ни в численную работу ограничений, ни в работу замыканий.
   - `account_energy` считает `end_stop = −Σ мощность · dt`, прибавляет к `loss_j` и к публикуемому
     `loss_step_j`, публикует новый ключ `rider_end_stop_loss_step_j`.
   - Работа упора — пружина и демпфер MuJoCo по умолчанию (`solreflimit (0.02, 1)`, B ≈ 2770). Около 97 % —
     рассеяние. Возврат пружины виден как короткая отрицательная потеря шага.
   - Нового поля состояния нет; невязка (`residual_j`) остаётся точной.
   - Демпфер упора противоречит правилу «демпфирования в шарнирах нет». Это записывается в документ решений как
     известное отступление до решения пользователя по самому упору.
8. **Мягкий старт уходит после включения** (задача 5): поля `seated_climb.launch_*`, ключи состояния политики
   `launch_ramp_s` и `launch_decided`, код `intent.cpp:362-369` и `:383-388`, проверка `intent.cpp:252-265`, решение 42
   заменяется ссылкой на план.

   Где он сейчас действует, а план — нет: множитель мягкого старта доходит до педалей, только когда
   `rider_enabled` и у вызывающего нет своего нажима (`intent.cpp:575-581`). План не включается там же и ещё при
   `!ctx.effort_mode` (`intent.cpp:558`). Вторая дыра: мягкий старт взводится без проверки скорости
   (`intent.cpp:362-369`: уклон ≥ 3 % и задержанный шатун ≤ 1 рад/с), а план — только с места (скорость ≤ 0,1 м/с
   и шатун ≤ 5 об/мин). Поэтому старт на ходу на подъёме и накат на уклоне со стоящим шатуном сегодня защищены
   мягким стартом, а без него — нет.

   Задача 5 перечисляет поездки и тесты с обеими дырами. Непустой список — стоп и пользователь.
9. **Перенос доказывается побайтно.** После части 4б образец для повторных сверок — поездки части 4б,
   задача 4, шаг 3: драйвер `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/parity/` и образцы
   `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-pneumatic-reach/f1/runs/kd88/m/sweep/` (96 поездок, правило gzip — там
   же, решение 7 части 4б). `gradecap/runs/` описывает поездки до части 4б и больше не образец. Исходная запись
   решения для задачи 4:

   Пробник `gradecap/harness/probe.cpp`, собранный без правок, на коде задачи 4
   и на профилях задачи 4 даёт записи поездок. Они должны совпасть с `gradecap/runs/` у 33 финальных
   поездок: файлы фаз побайтно, числовые записи — по всем столбцам, кроме `energy.*`, которые по замыслу меняет
   задача 3 (порядок сверки — задача 4, шаг 3):
   - `fck_c_*` — осторожный;
   - `rv25_n_*` — обычный;
   - `rv25_a_*` — агрессивный;
   - по 11 случаев: g00, g05, g08, g10, g12, g15, g20, loose10, wet10, stop05, stop10.

   Разошедшийся файл — стоп с первой разной строкой и столбцом. После задачи 5 сравнение повторяется: в пробе
   мягкий старт был. Расхождение после задачи 5 — не стоп, но каждый разошедшийся файл называется в отчёте с
   разницей метрик `cal_metrics.py --window-handover --rb-from-release`.
10. **Тесты поездок** (задача 6) — три исполняемых файла с меткой `slow`, по одному на тип; общий помощник метрик
    повторяет `gradecap/harness/cal_metrics.py`.

    Определения метрик:
    - отрыв колеса — `tires.front.normal_load_n ≤ 50` дольше 0,05 с;
    - буксовка — `|tires.rear.tangent_force_n| / (friction_coefficient · normal_load_n) > 0.9` дольше 0,05 с,
      при нормальной силе > 1 Н;
    - каденс — экспоненциальное среднее ног с τ = 0,35 с по `|sensors.encoders_rad_s.crank| · 60/2π`;
    - откат — наименьшее `x_m` после начала `release` минус `x_m` в этот момент (П11). Одобренные вердикты
      калибровки считали откат от входа в `hold` (`verdicts2.py:22-23`). Основа от `release` строже, разница на
      калиброванных случаях до 1,5 мм, все в пределах 10 мм (рецензия 2, §2.6);
    - «нынешние ограничения на каждом шаге» спецификации (`spec:284-286`) проверяются так:
      - опора седла и стоп — минимумы нормальных сил седла и обеих педалей в окне не ниже −6 Н (задача 13 дорожной
        карты, `verdicts2.py:57-58`);
      - бюджет усилия и равновесие — `rider_effort_budget_exceeded` и `rider_balance.balance_lost` в окне;
      - нажим поднимающейся ноги, силы рук и сила и мощность суставов — метки каналов `attachment_violations`,
        `rider_joint_power_violations`, `rider_joint_speed_violations`, `rider_strength_violations` по всей поездке;
      - баланс энергии — строгий режим: допуск невязки по умолчанию (0,05) обрывает поездку с `failure`;
      - рывки отдельным каналом не записываются и здесь не проверяются; это записывается в документ решений;
    - окно для контроллера (агрессивный, П10) — `cal_metrics.py:341-353` построчно: от начала отрыва до первого
      `acting` реакции, а без него — до опрокидывания; не меньше 0,4 с. Если не было ни того, ни другого, окно открыто
      и проходит. Довод — замеры калибровки (`_h_rv25_a_g15/g20`: отрывы 169 и 296 мс, реакция не действовала,
      опрокидывания нет), а не вердикт `verdicts2.py:62-64`: тот для агрессивного не мог провалиться;
    - время до 20 км/ч — по всей поездке от t = 0, не по окну. Порог 3,25 с — время на `f616f525` (3,254 с),
      округлённое в строгую сторону.

    Окно — от входа в `hold` до входа в `handover` (П11). Опрокидывание и прочий исход смотрятся по всей поездке
    (20 с). Поездки строгие: `stop_on_model_violation = true`, допуск энергии по умолчанию, как в пробе.

    Каждый тест сначала проезжает все случаи, печатает таблицу, потом требует. Падение называет все провалившиеся
    случаи.
11. **Эталоны.** Задачи 1–3 меняют форму документов. Задачи 4–5 меняют поведение поездок.

    Правило перезаписи частей 2–3b (только зелёные выходы, только новые ключи, `check_rerecord.py`) действует в
    задачах 1–3.

    В задачах 4–5 прежние значения записей меняются по замыслу. Исполнитель:
    - ничего не перезаписывает;
    - составляет список покрасневших тестов и изменившихся причин известных красных;
    - для каждого пишет файл, ключи и причину.

    Перезапись — задача 7, только после «да» пользователя.
12. **`test_realistic_pedelec_acceptance`** (задача 4).
    - Проверки ближайшей звезды и полосы каденса — только для переключений после входа плана в `blend`. С этого
      момента переключает автомат.
    - Каждое переключение раньше, пока план владеет переключениями, проверяется как ответ на просьбу райдера:
      направление «вверх», в этом или предыдущем интервале канал `rider_commands` просил «вверх».
    - Требование «хотя бы одно переключение вверх на плоском старте» остаётся.

## Общие ограничения

Как в части 3b (`docs/superpowers/plans/2026-11-12-launch-3b-launch-plan.md`, «Общие ограничения»), с уточнениями:

- **Пороги, пределы, допуски тестов, фильтры, задержки, силы, `strength`, `pedal_torque_ripple`, запас 25 Н** не
  меняются. Числа таблиц — только из решения 5. Пороги тестов поездок — только из решения 10 и П10–П13.
- Значений по умолчанию в коде для полей профиля нет. Читателей старых форм нет. Профиль без поля не загружается и
  называет поле.
- Мотор не меняется (П2). Запрет 41 остаётся.
- Красные тесты: 22 обычных и 10 медленных (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/final/reds_output.log`).
  Нельзя переставлять или ослаблять проверки в красном тесте.
- Входные документы переводятся везде, красные тесты включительно. Инструменты —
  `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/tools/`; копировать в `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/tools/`
  и править копию.
- Без отладчиков, с ограничением времени (`perl -e 'alarm 2400; exec @ARGV' --`), без `git stash`, без push.
  `.claude/` и `.agents/` не трогать, чистого `git status` не добиваться.
- Последняя строка коммита ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`, без строк про Devin и
  «Generated with».
- Временные файлы и прогоны — в `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/`. Каталоги
  `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/**` только читаются.
- Сборка пробника собирает список объектов через `find`, не `ninja -t` (он переписывает `.ninja_log`).
- Параметры Bosch в интернете не искать.

## Проверка задачи

После коммита задачи:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
BASELOG=/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/baseline-before-task<N>.log \
  EXPECT=<список задачи или пусто> \
  bash /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/check_task.sh <база задачи> \
  /Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/verify-task<N> [ab]
```

- `ab` — в задачах 1–3: поза успокоения не меняется, план выключен, действия реакции выключены, учёт энергии не
  трогает динамику.
- В задачах 4–6 — без `ab`: наклон старта меняет позу успокоения на уклоне по замыслу.
- Оркестратор кладёт `baseline-before-task<N>.log`; для задачи 1 это
  `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/final/reds_output.log`.
- Конец вывода — `check_task exit 0`. В задачах 4 и 5 допустим стоп «new red tests» или «causes differ» только по
  тестам из списка решения 11. Исполнитель тогда останавливается с этим списком, не перезаписывая.

## Бюджет тестов

| Что | Время |
|---|---|
| целевые тесты задачи | 3–8 мин |
| `check_task.sh` (инкрементальная сборка) | 10–30 мин |
| задача 4: сборка пробника и 33 поездки по 20 с | около 10 мин |
| задача 6: три файла поездок, около 80 поездок по 20 с, параллельно | около 5 мин на прогон |
| задача 9: `run_tests.sh full` и `rough_uphill_extreme` | около 35 мин |

Итого около 4 ч машинного времени без повторов.

## Опись

- **`[rider_launch]` (задача 1).**
  - TOML-файлы: `git grep -l '^\[rider_launch\]' -- '*.toml'`.
  - Встроенные TOML-строки тестов: `git grep -l '\[rider_launch\]' -- native/tests`.
  - Программные конфигурации: `git grep -n 'rider_launch *=\|rider_launch={' -- native` (семь мест: `validate.cpp`
    `default_physics`, `cli/configuration.cpp:212`, `validation/rider.cpp:24,36`, `validation/drivetrain.cpp:27`,
    `validation/scenarios.cpp:113,168`) и `git grep -n 'LaunchConfig{' -- native`.
  - JSON-документы — набор из 52 файлов в `facts.md` A.6.
  - `test_schema.cpp`: число полей `rider_launch` 24 → 28.
- **`[front_lift_reaction]` (задача 2)** — то же, `git grep -l 'hand_unload_n'` (26 файлов кода и тестов, плюс
  документы). Число полей 13 → 14.
- **Состояние плана** (задача 1) и **состояние реакции** (задача 2) — документы `native_rider_intent` (`facts.md`
  B.5 п. 5), `tests/fixtures/state/rider_intent{,.canonical}.json`.
- **Мягкий старт (задача 5)** — `facts.md` E: схема `model_config.hpp:217-219`, загрузчик `validate.cpp:819-843`,
  умолчания `validate.cpp:1363-1365`, `intent.{hpp,cpp}`, кодек политики `state_codec.cpp:1029-1051` и
  `:2373-2376`, списки стирания `equilibrium_cache.cpp:181`, `initial_state.cpp:71`, пять программных
  конфигураций, тесты, 52 документа.

---

### Задача 1. Предел по уклону, подъём предела и повтор просьбы в плане старта

**Файлы:**
- Изменить: `native/src/rider/launch_config.{hpp,cpp}`, `native/src/rider/launch_plan.{hpp,cpp}`,
  `native/src/rider/senses_codec.cpp` (конфигурация и состояние плана), `native/src/configuration/model_config.hpp`
  (строки схемы), семь программных конфигураций и все документы описи
- Тест: `native/tests/reference/test_launch_plan.cpp`, `native/tests/reference/test_launch_preparation.cpp`
  (правила таблицы), `native/tests/config/test_schema.cpp`

**Interfaces:**
- Produces: `LaunchConfig::{effort_cap_grades, effort_cap_grade_nm, effort_rise_speed_mps, shift_retry_s}`;
  `rider::effort_grade_cap_nm(const LaunchConfig&, double grade)`; `LaunchPlan::State::{rise_ticks, retry_tick}`;
  ключи кодека `rise_ticks`, `retry_tick`.

Поведение поездок не меняется: план во всех документах выключен.

- [ ] **Шаг 1. Тесты (падают).** В `test_launch_plan.cpp`:
  - в `table()` после `t.effort_cap_scree_nm = 60.;` добавить строки ниже, а после `t.upshift_crank_rpm = 80.;` —
    `t.shift_retry_s = 10 * kPeriod;`;
  - текст ожидания «a rejected cog is not asked again» остаётся: проверка идёт 5 тиков, меньше 10;
  - добавить тесты ниже и вызвать их в `main` после `test_upshift_request_follows_the_shifter()`.

```cpp
    // Launch part 4: a grade cap that never binds below the 60 N·m surface
    // caps and a rise speed no helper reaches keep the 3b tests exact.
    t.effort_cap_grades = {0.};
    t.effort_cap_grade_nm = {1000.};
    t.effort_rise_speed_mps = 100.;
```

```cpp
// Launch part 4: the grade cap — piecewise linear over the knots, held at
// the end values, the lower of it and the surface cap binds the ramp.
void test_grade_cap_binds_below_the_surface_cap() {
    auto t = table();
    t.effort_cap_grades = {0., .1, .2};
    t.effort_cap_grade_nm = {30., 10., 20.};
    bt::require(rider::effort_grade_cap_nm(t, -.05) == 30. && rider::effort_grade_cap_nm(t, .3) == 20.,
                "the grade cap holds its end values outside the knots");
    bt::require(std::abs(rider::effort_grade_cap_nm(t, .05) - 20.) < 1e-12 &&
                    std::abs(rider::effort_grade_cap_nm(t, .15) - 15.) < 1e-12,
                "the grade cap is linear between knots, also where it rises again");
    rider::LaunchPlan plan(t, kPeriod);
    auto p = standing(.1);
    rider::LaunchCommand cmd;
    for (int i = 0; i < 40; ++i) cmd = plan.update(in(p));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.effort_nm == 10.,
                "the 10 % grade cap binds below the 60 N·m surface cap");
    // A fresh plan: the 50-tick hold limit would end the first hold.
    rider::LaunchPlan loose(t, kPeriod);
    auto q = standing(0.);
    q.senses.surface_front = rider::SurfaceLabel::loose;
    for (int i = 0; i < 30; ++i) cmd = loose.update(in(q));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.effort_nm == 20.,
                "the 20 N·m loose cap binds below the 30 N·m flat grade cap");
}

// Launch part 4: once the felt speed reaches the rise speed the ceiling
// rises from the grade cap at effort_rate_nm_s per second to the surface
// cap; below the rise speed it is the grade cap again while the rise
// clock keeps running; a new start resets the clock.
void test_cap_rises_after_the_rise_speed() {
    auto t = table();
    t.effort_cap_grades = {0.};
    t.effort_cap_grade_nm = {10.};
    t.effort_rise_speed_mps = 2.;
    rider::LaunchPlan plan(t, kPeriod);
    auto p = standing();
    (void)plan.update(in(p));
    p.human_crank_torque_nm = 10.;
    for (int i = 0; i < 5; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::accelerate && plan.command().effort_nm == 5.,
                "accelerate starts below the 10 N·m grade cap");
    p.senses.speed_mps = 1.;
    rider::LaunchCommand cmd;
    for (int i = 0; i < 10; ++i) cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 10. && plan.state().rise_ticks == -1, "below the rise speed the grade cap binds");
    p.senses.speed_mps = 2.;
    cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 10. && plan.state().rise_ticks == 0, "the rise starts at the grade cap");
    for (int i = 0; i < 5; ++i) cmd = plan.update(in(p));
    bt::require(std::abs(cmd.effort_nm - 15.) < 1e-12 && plan.state().rise_ticks == 5,
                "the ceiling rises one rate step per tick");
    p.senses.speed_mps = 1.;
    cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 10. && plan.state().rise_ticks == 6,
                "below the rise speed the ceiling is the grade cap again; the clock runs on");
    p.senses.speed_mps = 2.;
    for (int i = 0; i < 100; ++i) cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 60., "the rise ends at the surface cap");
}

// Launch part 4: a rejected request repeats on the same cog after
// shift_retry_s; a committed shift keeps the old cog blocked until the
// felt cog changes.
void test_rejected_request_repeats_after_the_retry_time() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 1.;
    p.crank_rpm = 85.;
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = 3, .cut_active = false};
    auto cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "the table cadence raises a request");
    fb.response = rider::ShiftResponse::rejected;
    cmd = plan.update(in(p, fb));
    const std::int64_t rejected_at = plan.state().launch_ticks;
    bt::require(!cmd.shift_up && plan.state().retry_tick == rejected_at + 10,
                "a rejection retires the request and sets the retry tick");
    fb.response = rider::ShiftResponse::none;
    for (int i = 0; i < 9; ++i) {
        cmd = plan.update(in(p, fb));
        bt::require(!cmd.shift_up, "the same cog waits for the retry time");
    }
    cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up && plan.state().retry_tick == -1 && plan.state().launch_ticks == rejected_at + 10,
                "the same cog is asked again at the retry tick");
    fb.shift_count = 4;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up && plan.state().retry_tick == -1, "a committed shift sets no retry");
    for (int i = 0; i < 20; ++i) cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "after a committed shift the old cog stays blocked");
}
```

  Расчёт `test_cap_rises_after_the_rise_speed`. Темп 100 Н·м/с при такте 0,01 с — 1 Н·м за тик.
  - Вызов 1 взводит удержание (нажим 0).
  - Вызов 2 — нажим 1, ощущаемый момент 10 ≥ 10, переход в `release`.
  - Вызовы 3–6 — четыре тика `release`, нажим 5, на шестом тормоз 0 и `accelerate`. Как в помощнике `accelerating()`.
  - Если проверка «`accelerate` и 5» не сходится, исполнитель печатает фактические `phase`, `effort_nm`,
    `launch_ticks` и правит только число вызовов в подготовке, не ожидания про потолок.
  - После этого 10 тиков на 1 м/с: потолок 10.
  - Первый тик на 2 м/с: `rise_ticks = 0`, потолок 10. Ещё 5 тиков: `rise_ticks = 5`, потолок 10 + 1 · 5 = 15.
    Нажим растёт на 1 за тик и тоже 15.
  - Тик на 1 м/с: потолок 10, нажим режется до 10, часы 6.
  - 100 тиков на 2 м/с: потолок 10 + 1 · k растёт до 60, нажим тоже; предел покрытия 60.

  В `test_codec_round_trip_continuation_and_rejected_restores` добавить отклонённые состояния (рядом с прочими
  `bad`):

```cpp
    bad = hold.state();
    bad.rise_ticks = -2;
    rejects([&] { fresh.restore(bad); }, "rise_ticks", "a rise clock below -1");
    bad = hold.state();
    bad.retry_tick = 5;
    rejects([&] { fresh.restore(bad); }, "retry_tick", "a retry tick in the hold");
```

  (имя переменной с состоянием удержания взять из теста; если его нет — состояние после
  `plan.update(in(standing()))`).

  В `test_launch_preparation.cpp`, в тесте правил таблицы, добавить отклонения:
  - пустой `effort_cap_grades` — «needs at least one knot»;
  - разная длина — «one cap per effort_cap_grades knot»;
  - невозрастающие узлы — «ascend strictly»;
  - нулевой предел — «effort_cap_grade_nm»;
  - `effort_rise_speed_mps = 0` и `shift_retry_s = 0` — «positive»;
  - `shift_retry_s` вне сетки — «whole number of intention periods».

  `test_schema.cpp`: число полей `rider_launch` 24 → 28.

- [ ] **Шаг 2. Запуск — падает.** `mise run test 'launch_plan|launch_preparation|config_schema'`. Ожидается ошибка
  сборки: нет полей.

- [ ] **Шаг 3. Таблица.** В `launch_config.hpp` после `effort_cap_scree_nm`:

```cpp
    // Launch part 4: the launch ceiling over the felt grade ahead —
    // piecewise linear over strictly ascending knots, held at the end
    // values outside them; the plan's ceiling is the lower of it and the
    // surface cap until the felt speed first reaches effort_rise_speed_mps,
    // then it rises from the grade cap at effort_rate_nm_s to the surface cap.
    std::vector<double> effort_cap_grades;
    std::vector<double> effort_cap_grade_nm;
    double effort_rise_speed_mps = std::numeric_limits<double>::quiet_NaN();
```

  после `upshift_crank_rpm`:

```cpp
    // Launch part 4: a rejected upshift request may repeat on the same cog
    // after this time.
    double shift_retry_s = std::numeric_limits<double>::quiet_NaN();
```

  после объявления `release_torque_nm`:

```cpp
// The launch grade cap the table gives for one grade (same interpolation
// as the release torque). Validates the table first.
[[nodiscard]] double effort_grade_cap_nm(const LaunchConfig &table, double grade);
```

  В `launch_config.cpp`:
  - в безымянном пространстве после `strictly_ascending`:

```cpp
// Piecewise linear over strictly ascending knots, held at the end values —
// the arithmetic release_torque_nm used since part 3b.
double knot_value(const std::vector<double> &knots, const std::vector<double> &values, double grade) {
    if (grade >= knots.back()) return values.back();
    if (grade <= knots.front()) return values.front();
    const auto upper = static_cast<std::size_t>(
        std::distance(knots.begin(), std::ranges::upper_bound(knots, grade)));
    const double t = (grade - knots[upper - 1]) / (knots[upper] - knots[upper - 1]);
    return values[upper - 1] + t * (values[upper] - values[upper - 1]);
}
```

  - в `validate_launch_table` после `positive(table.effort_cap_scree_nm, …)`:

```cpp
    if (table.effort_cap_grades.empty()) reject("effort_cap_grades", "needs at least one knot");
    if (table.effort_cap_grade_nm.size() != table.effort_cap_grades.size())
        reject("effort_cap_grade_nm", "needs one cap per effort_cap_grades knot");
    strictly_ascending(table.effort_cap_grades, "effort_cap_grades");
    for (const double cap : table.effort_cap_grade_nm) positive(cap, "effort_cap_grade_nm");
    positive(table.effort_rise_speed_mps, "effort_rise_speed_mps");
```

    и после `positive(table.upshift_crank_rpm, …)` — `positive(table.shift_retry_s, "shift_retry_s");`.
  - `release_torque_nm` после проверки уклона возвращает `knot_value(table.release_torque_grades, table.release_torque_nm, grade);`.
    Арифметика та же, побайтно.
  - новая функция:

```cpp
double effort_grade_cap_nm(const LaunchConfig &table, double grade) {
    validate_launch_table(table);
    if (!std::isfinite(grade)) throw std::invalid_argument("launch grade cap grade must be finite");
    return knot_value(table.effort_cap_grades, table.effort_cap_grade_nm, grade);
}
```

    Для узла — внутренняя `grade_cap_unchecked(const LaunchConfig&, double)` без проверки таблицы: таблица
    проверена в конструкторе узла. Объявить её в `launch_config.hpp` в `namespace rider::detail` с комментарием
    «the plan's per-tick path; the table is validated at construction».
  - в `validate_launch_plan_context` добавить `on_grid(table.shift_retry_s, "shift_retry_s");`.

- [ ] **Шаг 4. Узел.** В `launch_plan.hpp` в `State` после `preload_missed`:

```cpp
        // Launch part 4: intention ticks since the felt speed first reached
        // effort_rise_speed_mps in this launch; -1 = not yet.
        std::int64_t rise_ticks = -1;
        // launch_ticks at which a rejected request's same-cog block ends;
        // -1 = the block lasts until the felt cog changes.
        std::int64_t retry_tick = -1;
```

  - Сигнатура `ramp` становится `[[nodiscard]] double ramp(double effort, const Percepts &percepts, State &n) const;`.
  - Новый член `std::int64_t retry_ticks_ = 0;` в список тиков.

  В `launch_plan.cpp`:
  - в списке инициализации конструктора `retry_ticks_(ticks(config_.shift_retry_s, period_s_))`;
  - `ramp`:

```cpp
double LaunchPlan::ramp(double effort, const Percepts &p, State &n) const {
    // Launch part 4, decisions 1-2: the lower of the surface cap and the
    // grade cap; from the first tick at the rise speed the grade cap rises at
    // the effort rate (the calibration probe's arithmetic, operand order kept).
    const double surface = cap(p.senses.surface_front);
    const double grade = detail::grade_cap_unchecked(config_, p.senses.grade_ahead);
    double ceiling = std::min(surface, grade);
    if (p.senses.speed_mps >= config_.effort_rise_speed_mps) {
        if (n.rise_ticks < 0) n.rise_ticks = 0;
        ceiling = std::min(surface, grade + config_.effort_rate_nm_s * static_cast<double>(n.rise_ticks) * period_s_);
    }
    return std::min(effort + config_.effort_rate_nm_s * period_s_, ceiling);
}
```

  - все вызовы `ramp(n.effort_nm, *p)` → `ramp(n.effort_nm, *p, n)`;
  - в `update` сразу после `State n = s_;`:

```cpp
    // The rise clock counts every eligible tick once started (probe: Rise.now).
    if (n.rise_ticks >= 0) ++n.rise_ticks;
```

  - `shifting`:

```cpp
void LaunchPlan::shifting(State &n, const Percepts &p, const ShiftFeedback &fb) const {
    if (n.shift_up && (fb.shift_count > n.request_count || fb.response == ShiftResponse::rejected)) {
        n.shift_up = false;
        n.blocked_teeth = n.request_teeth;
        // Launch part 4, decision 3: a rejected cog may be asked again after
        // shift_retry_s; a committed shift keeps it blocked until the felt cog changes.
        n.retry_tick = fb.response == ShiftResponse::rejected ? n.launch_ticks + retry_ticks_ : -1;
    }
    if (!n.shift_up && !fb.cut_active && p.crank_rpm >= config_.upshift_crank_rpm &&
        (p.senses.rear_teeth != n.blocked_teeth || (n.retry_tick >= 0 && n.launch_ticks >= n.retry_tick))) {
        n.shift_up = true;
        n.request_count = fb.shift_count;
        n.request_teeth = p.senses.rear_teeth;
        n.retry_tick = -1;
    }
}
```

  - `restore`: после проверки `request_teeth`:

```cpp
    if (s.rise_ticks < -1) bad_state("rise_ticks", "below -1");
    if (s.retry_tick < -1) bad_state("retry_tick", "below -1");
    if (s.retry_tick >= 0 &&
        (s.shift_up || s.blocked_teeth < 3 || s.retry_tick < retry_ticks_ || s.retry_tick > s.launch_ticks + retry_ticks_))
        bad_state("retry_tick", "a retry without the rejected request it follows");
```

    и в ветке `hold` и `release` — `if (s.retry_tick != -1) bad_state("retry_tick", "a retry before accelerate");`.
    Ветки `idle`/`done` уже требуют `State{.phase = …}`, то есть −1 у обоих.

  Эквивалентность пробе (для ревью):
  - Проба держала срок в `phase_ticks` и сбрасывала его в 0 при просьбе. До первой отмены условие `rear_teeth != blocked_teeth (=0)`
    истинно, поэтому второе слагаемое ничего не решает.
  - После отмены проба и этот код ставят одинаковый срок; INT64_MAX пробы — это −1 здесь.
  - Часы подъёма в пробе: `now` растёт на каждом допущенном тике, `start` ставится при первом достижении
    скорости, `arm` сбрасывает `start`. Здесь −1 сбрасывают `arm`, сброс по неприменимости и `done`.

- [ ] **Шаг 5. Кодеки и схема.**
  - `encode_launch_config`/`decode_launch_config`: три новых ключа после `effort_cap_scree_nm` (`real_array` /
    `real_list` для узлов), `shift_retry_s` после `upshift_crank_rpm`; список `ncd::exact` — 28 имён.
  - `encode_launch_state`/`decode_launch_state`: `rise_ticks`, `retry_tick` в конце (`integer`); список
    `ncd::exact` — 15 имён.
  - Схема `model_config.hpp`: четыре строки в том же порядке, по образцу соседних (`RealTuple` для узлов, `Real`
    для чисел, `required=true`).

- [ ] **Шаг 6. Документы и программные конфигурации.**
  - Каждый документ описи с `[rider_launch]` и каждое программное место получает четыре поля.
  - Значения (решение 6, инертные при выключенном плане): `effort_cap_grades = [0.0]`,
    `effort_cap_grade_nm = [1000.0]`, `effort_rise_speed_mps = 2.5`, `shift_retry_s = 0.25`.
  - Исключение — три исследовательских профиля `examples/research/viewer_physics_*.toml`: там значения обычного
    райдера из решения 5, с комментарием «launch part 4 — the normal rider's calibrated grade cap, rise and
    retry; inert until plan_enabled».
  - Документы состояния с объектом `launch` получают `"rise_ticks": -1, "retry_tick": -1`.
  - Бинарные `.bin` эталоны `native_rider_intent` — `migrate_rider_intent_bins.py` (копия в `launch4/tools`).

- [ ] **Шаг 7. Целевые тесты.**
  `mise run test 'launch_plan|launch_preparation|launch_intent|config_schema|welded_profile_config|rider_senses|front_lift_reaction'`.
  Ожидается: зелёные — прежние зелёные; `config_schema` — известный красный с прежней причиной.

- [ ] **Шаг 8. Перезапись** — только зелёные выходы, только новые ключи:
  `python3 /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch1/check_rerecord.py --repo . --ref <база задачи>` на каждый
  изменённый JSON (вывод — в отчёт). Изменённое прежнее значение — стоп.

- [ ] **Шаг 9. Коммит.** `feat(rider): the launch grade cap, its rise and the shift retry`.

- [ ] **Шаг 10. Проверка задачи** (с `ab`, `EXPECT` пусто).

---

### Задача 2. Критерий Б в реакции на подъём колеса

**Файлы:**
- Изменить: `native/src/rider/front_lift_reaction.{hpp,cpp}`, `native/src/rider/senses_codec.{hpp,cpp}`,
  `native/src/configuration/model_config.hpp`, `validate.cpp` (`default_physics`), семь программных конфигураций,
  документы описи
- Тест: `native/tests/reference/test_front_lift_reaction.cpp` и все тесты из `git grep -l hand_unload_n -- native/tests`

**Interfaces:**
- Consumes: `Percepts::{senses.hand_press_n, crank_rpm, pitch_rate_up_rad_s, pitch_rad}`.
- Produces: `FrontLiftReactionConfig::{hand_window_s, hand_lag_s, hand_drop_ratio}` (вместо `hand_unload_n`,
  `unload_dwell_s`); `FrontLiftReaction::State::{hand_n, crank_angle_rad, hand_average_n, crank_angle_total_rad}`
  (вместо `unload_ticks`); ключи кодека с теми же именами.

Действия реакции во всех документах пока выключены (`ease_enabled = lean_enabled = false`), поэтому динамика поездок
не меняется. Меняются значения канала `rider_reaction` (`predicate`, `phase`) в записанных поездках.

- [ ] **Шаг 1. Тест-реплика (падает).** В `test_front_lift_reaction.cpp` добавить дословную реплику пробы —
  неограниченные очереди, как в `gradecap/_diff_reaction.txt` — и сравнение по тикам:

```cpp
// The calibration probe's criterion B (gradecap/_diff_reaction.txt) with
// unbounded histories, kept verbatim as the oracle for the bounded state.
struct ProbeHandB {
    std::deque<double> hand, phase_at, avg;
    double phase = 0.;
    std::int64_t fcap = 0, lag = 0;
    double thr = 0.;
    bool term(const rider::Percepts &p, double period_s) {
        hand.push_back(p.senses.hand_press_n);
        phase += p.crank_rpm * (2. * std::numbers::pi / 60.) * period_s;
        phase_at.push_back(phase);
        std::int64_t span = -1;
        for (std::size_t i = phase_at.size(); i-- > 0;)
            if (phase - phase_at[i] >= std::numbers::pi) {
                span = static_cast<std::int64_t>(phase_at.size() - 1 - i);
                break;
            }
        if (span < 0 || span > fcap) span = std::min<std::int64_t>(fcap, static_cast<std::int64_t>(hand.size()));
        double sum = 0.;
        const auto n = static_cast<std::int64_t>(hand.size());
        for (std::int64_t i = n - span; i < n; ++i) sum += hand[static_cast<std::size_t>(i)];
        const double a = sum / static_cast<double>(span);
        avg.push_back(a);
        const auto j = static_cast<std::int64_t>(avg.size()) - 1;
        if (j < lag) return false;
        const double ref = avg[static_cast<std::size_t>(j - lag)];
        return ref > 0. && a < thr * ref;
    }
};

// Criterion B on the bounded state equals the probe's unbounded replay on
// every tick: steady cadence, a stall (the fallback window), a cadence
// that leaves no sample half a revolution back inside the window, hands
// that rise out of a pull (negative and zero references) and absent
// percepts; the pitch term is always on so the predicate is the hand term.
void test_criterion_b_equals_the_probe_replay() {
    auto t = table();
    t.decision_s = 0.;
    // The profile's 0.5 s window and lag (400 ticks): at 90 rpm half a
    // revolution is about 267 ticks, inside the window; 40 rpm and a stall
    // take the fallback.
    t.hand_window_s = 400 * kPeriod;
    t.hand_lag_s = 400 * kPeriod;
    rider::FrontLiftReaction reaction(t, kPeriod);
    ProbeHandB probe{.fcap = 400, .lag = 400, .thr = .7};
    for (int tick = 0; tick < 3000; ++tick) {
        if (tick % 997 == 500) {
            (void)reaction.update(std::nullopt);   // absent percepts leave the history alone
            continue;
        }
        rider::Percepts p = percept(0., 1., 0.);
        p.crank_rpm = tick < 800 ? 90. : tick < 1200 ? 0. : tick < 1600 ? 3. : 70. + 30. * std::sin(tick * .01);
        p.senses.hand_press_n = 60. + 50. * std::sin(tick * .037) - (tick > 2200 && tick < 2400 ? 120. : 0.);
        const bool expected = probe.term(p, kPeriod);
        const auto cmd = reaction.update(p);
        bt::require(cmd.predicate == expected, "tick " + std::to_string(tick) + ": the bounded criterion B equals the probe");
    }
}
```

  Здесь окно и опора по 400 тиков, как в профиле, порог 0,7. Ожидаемые предикаты не выписываются: их даёт
  реплика.

  Остальные тесты файла переводятся на критерий Б. Свойство критерия, которое определяет перевод: опора — то же
  среднее `lag` тиков назад. Поэтому при **ступеньке** нажима (80 Н → 5 Н и дальше постоянно) признак держится
  ограниченное время, а потом гаснет сам: через `lag` тиков опора тоже становится лёгкой. Расчёт рецензии для
  окна и опоры по 40 тиков: признак есть на лёгких образцах k = 13…78. Тесты, которым нужен долгий `acting`,
  получают **плавную разгрузку**: нажим `80 · 0.99^k`. Признак встаёт на k = 62 и держится без перерыва до
  k = 299 (расчёт оркестратора и рецензии 2); отношение средних окна к опоре стремится к `0.99^40 ≈ 0.669 < 0.7`.

  Помощники:

```cpp
// 40 loaded samples fill the window and the 41st completes the reference:
// the window average and the reference are both 80 N.
void loaded(rider::FrontLiftReaction &reaction, int ticks) {
    for (int tick = 0; tick < ticks; ++tick) (void)reaction.update(percept(80., 0., 0.));
}

// A steadily unloading grip with the pitch term on: the press falls by 1 %
// per tick. After 41 loaded samples criterion B first holds at k = 62 and
// then without a gap (the window/reference ratio settles at 0.99^40 = 0.669);
// a step to light hands would extinguish itself once the reference is light too.
rider::Percepts pulling(int k) { return percept(80. * std::pow(.99, k), 1., 0.); }
```

  Правило для каждого теста:
  - Каждый прогон, который раньше начинался с «лёгких рук» `percept(5., …)` с тика 0, теперь сначала проходит
    `loaded(reaction, 41)`.
  - Где тест проверяет точные индексы — короткая ступенька `percept(5., …)` и индексы по формуле ниже.
  - Где тест ждёт установившегося `acting` — `pulling(k)` на 300 тиков (k = 0…299; нажим в конце около 3,9 Н, > 0).
  - Тесты с проверками `unload_ticks` вместо них проверяют историю (шаг 4).

  Формула ступеньки. Шатун стоит (`crank_rpm = 0`), поэтому окно — запасное, `min(40, число образцов)`. После
  41 нагруженного образца k-й лёгкий образец (k = 1, 2, …) даёт среднее `(80 · (40 − k) + 5k) / 40`; оно < 56 при
  k ≥ 13. Опора при k ≤ 40 — 80. Значит признак встаёт на 13-м лёгком образце и держится до k = 78.

  `test_predicate_dwell_and_decision_timing` → `test_predicate_window_and_decision_timing`:

```cpp
// Criterion B: after 41 loaded samples (80 N, the 40-tick reference full)
// the light-hand average falls below 0.7 of the reference on the 13th light
// sample; the action starts after the decision time (8 ticks); the first
// acting tick already moves both outputs one rate step.
void test_predicate_window_and_decision_timing() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    loaded(reaction, 41);
    const auto lifting = percept(5., 1., 0.);
    std::vector<rider::ReactionCommand> out;
    out.reserve(30);
    for (int tick = 0; tick < 30; ++tick) out.push_back(reaction.update(lifting));
    for (std::size_t tick = 0; tick < 12; ++tick)
        bt::require(!out[tick].predicate, "no predicate before the window average drops below 0.7");
    bt::require(out[12].predicate && out[12].phase == rider::ReactionPhase::deciding,
                "the predicate rises on the 13th light sample");
    for (std::size_t tick = 12; tick < 20; ++tick)
        bt::require(out[tick].phase == rider::ReactionPhase::deciding && out[tick].effort_factor == 1.,
                    "the rider is still deciding");
    bt::require(out[20].phase == rider::ReactionPhase::acting, "the action starts after the decision time");
    bt::require(std::abs(out[20].effort_factor - (1. - 5. * kPeriod)) < 1e-15 &&
                    std::abs(out[20].lean_add_rad - .5 * kPeriod) < 1e-15,
                "acting moves both outputs one rate step");
}

// A step to light hands extinguishes itself: once the reference 40 ticks
// back is light too, the ratio returns to 1 and the predicate ends (k = 79).
void test_a_hand_step_extinguishes_itself() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    loaded(reaction, 41);
    std::vector<bool> predicate;
    for (int k = 1; k <= 90; ++k) predicate.push_back(reaction.update(percept(5., 1., 0.)).predicate);
    for (int k = 1; k <= 90; ++k)
        bt::require(predicate[static_cast<std::size_t>(k - 1)] == (k >= 13 && k <= 78),
                    "light sample " + std::to_string(k) + ": the predicate holds exactly for k = 13..78");
}
```

  В `test_held_pitch_triggers_and_light_hands_alone_do_not`:
  - «held» — `loaded(held, 41)`, затем 13 тиков `percept(5., .01, .3)`; признак есть.
  - «light» — 400 тиков `percept(5., .01, .01)` без нагрузки. Постоянные руки — отношение 1, признака нет. Ожидание
    «light hands alone never trigger» остаётся.
  - Добавить случай «руки разгружены, наклона нет»: `loaded(…, 41)`, затем 40 тиков `percept(5., 0., 0.)`; признака
    нет.
  - Добавить защиту опоры: 41 тик `percept(-10., 1., 0.)`, затем 40 тиков `percept(-30., 1., 0.)`. Опора ≤ 0,
    признака нет.

  В `test_short_predicate_is_dropped_while_deciding`:
  - `loaded(…, 41)`, затем 14 тиков `percept(5., 1., 0.)` — `deciding` (вход на k = 13, k = 14 — `decision_ticks = 1`);
  - затем тик `percept(80., 0., 0.)` — `idle`: наклон пропал.

  В `test_each_action_has_its_own_switch` и `test_recovery_after_clear` вместо прежних 200 тиков
  `percept(5., 1., 0.)` — `loaded(…, 41)` и `pulling(k)` для k = 0…299.
  - `acting` начинается около k ≈ 70 и держится до конца: ослаблению до 0,3 нужно 112 тиков, наклону до 0,1 —
    160 тиков.
  - Затем, как раньше, 16 тиков `percept(80., 0., 0.)` в `acting` (наклона нет, признака нет, идёт отсчёт), на 17-м
    — `releasing`.

  `test_codec_round_trip_and_continuation`: вход — 41 тик `percept(80., 0., 0.)`, затем `pulling(k)` на 300 тиков,
  затем `percept(80., 0., 0.)`. Точки разреза тест **находит сам** по прогону-образцу той же последовательности:
  первый тик фазы `deciding` плюс 2 и первый тик фазы `releasing` плюс 20. Найденные тики тест печатает и требует
  `phase == deciding` и `phase == releasing` на них. Числа на сложение со старыми точками 7 и 230 не выводятся.

  `test_restore_rejects_unreachable_states`:
  - Реакция `target` (оба действия выключены): `loaded(target, 41)`, затем `percept(5., 1., 0.)`, пока фаза не
    станет `deciding` (не больше 40 тиков, иначе тест падает), и ещё один тик.
  - Реакция `on`: `loaded(on, 41)` и `pulling(k)` на 300 тиков → `acting` под признаком; затем 20 тиков
    `percept(80., 0., 0.)` → `releasing`.
  - Отклонённые состояния вместо `unload_ticks`:
    - `bad.hand_n.push_back(1.)` — «hand_n», длины истории не совпадают;
    - `bad.predicate = true` при неполной истории средних (состояние `target` после 30 нагруженных тиков) —
      «predicate»;
    - `bad.crank_angle_total_rad = 1.` при пустой истории (свежая реакция) — «crank_angle»;
    - acting с `percepts_seen = 48`: нужно 41 + 8 = 49 — «percepts_seen».
  - Строка `bad.unload_ticks = 5;` в случае «releasing with the predicate» удаляется. Признак без падения рук
    отклоняет новая проверка с тем же словом «predicate».

  Тест на уровне намерений `test_reacting_resolver_timing_slew_probe_and_restore` (`:425-480`). Входы `inputs()`:
  60 Н, на шагах 161…399 — 5 Н; шатун 6 рад/с, то есть 57 об/мин. Полоборота — 419 тиков, больше окна 40, окно
  запасное.
  - Реакция видит 60 Н до задержанного шага 280. k-й лёгкий образец даёт среднее `(60 · (40 − k) + 5k) / 40 < 42`
    при k ≥ 14. Признак — шаг 281 + 13 = 294, `acting` — 302.
  - Строка `const std::int64_t onset = 161 + delay + 4 + 8;` → `161 + delay + 13 + 8` с комментарием «the 14th
    light sample against the 60 N reference (ratio .7, fallback window 40)».
  - Снимок `onset - 3` (299) остаётся в `deciding`.
  - По расчёту рецензии признак гаснет на шаге 359, `releasing` идёт с 375 по 446. Снимок «while releasing» на
    шаге 560 становится шагом 400.
  - Комментарий над тестом переписывается под новую хронологию. Так же устаревшие комментарии `:489` («acting from
    293» → 302) и `:596` — проверить и переписать.

  Если пересчитанный индекс не сходится, исполнитель печатает фактическую хронологию фаз и сообщает. Ожидания он не
  ослабляет.

- [ ] **Шаг 2. Таблица теста.** `table()`: `.hand_window_s = 40 * kPeriod, .hand_lag_s = 40 * kPeriod,
  .hand_drop_ratio = .7` вместо `hand_unload_n`/`unload_dwell_s`, остальное как было. В правилах таблицы (`test_table_rules`)
  отклоняются:
  - нулевое окно, нулевая опора и время вне сетки — «whole multiple»;
  - доля 0 и доля 1.5 — «hand_drop_ratio».

  Доля 1 допустима.

- [ ] **Шаг 3. Запуск — падает.** `mise run test 'front_lift_reaction'`.

- [ ] **Шаг 4. Узел.** `front_lift_reaction.hpp`:

```cpp
struct FrontLiftReactionConfig {
    // Launch part 4, criterion B: the hands are unloading while their press
    // averaged over the last half crank revolution (at most hand_window_s)
    // is below hand_drop_ratio of the same average hand_lag_s earlier and
    // that earlier average is positive.
    double hand_window_s = std::numeric_limits<double>::quiet_NaN();
    double hand_lag_s = std::numeric_limits<double>::quiet_NaN();
    double hand_drop_ratio = std::numeric_limits<double>::quiet_NaN();
    double pitch_rate_up_rad_s = std::numeric_limits<double>::quiet_NaN();
    // … остальные поля без изменений
};
```

  `State`: убрать `unload_ticks`, после `percepts_seen` добавить:

```cpp
        // Criterion B history, one entry per percept: the delayed hand press
        // and the crank angle integrated from the delayed cadence (the last
        // window + 1 of each), the window averages (the last lag + 1) and
        // the integrated angle itself.
        std::deque<double> hand_n;
        std::deque<double> crank_angle_rad;
        std::deque<double> hand_average_n;
        double crank_angle_total_rad = 0.;
```

  - Комментарий к `percepts_cap`: «lag + 1 + decision + clear + 1 ticks».
  - Частные члены: `std::int64_t window_ticks_ = 0, lag_ticks_ = 0, decision_ticks_ = 0, clear_ticks_ = 0;`
    (вместо `dwell_ticks_`) и `bool hand_term(State &n, const Percepts &p) const;`.
  - `#include <deque>`.

  `front_lift_reaction.cpp`:

```cpp
void validate_reaction_table(const FrontLiftReactionConfig &t) {
    finite_positive(t.hand_window_s, "hand_window_s");
    finite_positive(t.hand_lag_s, "hand_lag_s");
    if (!std::isfinite(t.hand_drop_ratio) || t.hand_drop_ratio <= 0. || t.hand_drop_ratio > 1.)
        reject("hand_drop_ratio", "must lie in (0, 1]");
    finite_positive(t.pitch_rate_up_rad_s, "pitch_rate_up_rad_s");
    // … остальные проверки без изменений
}
```

  - В `validate_reaction_context` и в конструкторе `unload_dwell_s` заменяется на `hand_window_s` и `hand_lag_s`:
    `window_ticks_ = ticks(config_.hand_window_s, …)`, `lag_ticks_ = ticks(config_.hand_lag_s, …)`. Положительное
    время на сетке даёт не меньше одного тика.
  - Признак:

```cpp
// Launch part 4, decision 4 — the calibration probe's criterion B on a
// bounded history (gradecap/_diff_reaction.txt; same arithmetic and
// summation order). The newest sample at least half a revolution back is
// inside the last window + 1 samples whenever the probe's span fits the
// window, so the bounded search finds the same span or falls back the same way.
bool FrontLiftReaction::hand_term(State &n, const Percepts &p) const {
    n.hand_n.push_back(p.senses.hand_press_n);
    n.crank_angle_total_rad += p.crank_rpm * (2. * std::numbers::pi / 60.) * period_s_;
    n.crank_angle_rad.push_back(n.crank_angle_total_rad);
    if (std::ssize(n.hand_n) > window_ticks_ + 1) {
        n.hand_n.pop_front();
        n.crank_angle_rad.pop_front();
    }
    std::int64_t span = -1;
    for (std::size_t i = n.crank_angle_rad.size(); i-- > 0;)
        if (n.crank_angle_total_rad - n.crank_angle_rad[i] >= std::numbers::pi) {
            span = static_cast<std::int64_t>(n.crank_angle_rad.size() - 1 - i);
            break;
        }
    if (span < 0 || span > window_ticks_) span = std::min<std::int64_t>(window_ticks_, std::ssize(n.hand_n));
    double sum = 0.;
    const std::int64_t count = std::ssize(n.hand_n);
    for (std::int64_t i = count - span; i < count; ++i) sum += n.hand_n[static_cast<std::size_t>(i)];
    const double average = sum / static_cast<double>(span);
    n.hand_average_n.push_back(average);
    if (std::ssize(n.hand_average_n) > lag_ticks_ + 1) n.hand_average_n.pop_front();
    if (std::ssize(n.hand_average_n) < lag_ticks_ + 1) return false;
    const double reference = n.hand_average_n.front();
    return reference > 0. && average < config_.hand_drop_ratio * reference;
}
```

  - В `update`, ветка без ощущений — только `n.predicate = false;`. Ветка с ощущениями:

```cpp
        n.percepts_seen = std::min(n.percepts_seen + 1, percepts_cap());
        // The hand term runs first on every percept so the history always advances.
        const bool hands = hand_term(n, *percepts);
        n.predicate = hands && (percepts->pitch_rate_up_rad_s > config_.pitch_rate_up_rad_s ||
                                percepts->pitch_rad > config_.pitch_rad);
```

  - `percepts_cap()` возвращает `lag_ticks_ + 1 + decision_ticks_ + clear_ticks_ + 1`.

  `restore`:
  - строки про `unload_ticks` заменяются проверками истории:

```cpp
    if (s.hand_n.size() != s.crank_angle_rad.size()) bad("hand_n", "hand and crank histories differ in length");
    const std::int64_t hands = std::ssize(s.hand_n), averages = std::ssize(s.hand_average_n);
    const bool saturated = s.percepts_seen == percepts_cap();
    if (saturated ? hands < std::min(percepts_cap(), window_ticks_ + 1) || hands > window_ticks_ + 1
                  : hands != std::min(s.percepts_seen, window_ticks_ + 1))
        bad("hand_n", "history length off its percept count");
    if (averages != std::min(s.percepts_seen, lag_ticks_ + 1)) bad("hand_average_n", "history length off its percept count");
    for (const double v : s.hand_n) if (!std::isfinite(v)) bad("hand_n", "must be finite");
    for (const double v : s.hand_average_n) if (!std::isfinite(v)) bad("hand_average_n", "must be finite");
    for (const double v : s.crank_angle_rad) if (!std::isfinite(v)) bad("crank_angle_rad", "must be finite");
    if (!std::isfinite(s.crank_angle_total_rad) ||
        (s.crank_angle_rad.empty() ? s.crank_angle_total_rad != 0. : s.crank_angle_rad.back() != s.crank_angle_total_rad))
        bad("crank_angle_total_rad", "must equal the newest crank angle");
    // update() sets the predicate only on a full reference and a hand drop.
    if (s.predicate && (averages != lag_ticks_ + 1 || !(s.hand_average_n.front() > 0.) ||
                        !(s.hand_average_n.back() < config_.hand_drop_ratio * s.hand_average_n.front())))
        bad("predicate", "set without a hand drop against a full reference");
```

    Так как `percepts_cap() > lag_ticks_ + 1`, насыщенный счётчик всегда значит полную историю средних.
  - `dwell_ticks_ + 1` в ранних счётчиках фаз (`acting_from`, проверка `deciding`) заменяется на `lag_ticks_ + 1`.
  - Комментарий над `acting_from`: «the predicate first holds at lag + 1 percepts».

- [ ] **Шаг 5. Кодеки.**
  - `encode_reaction_state`: ключи по порядку
    `phase, predicate, decision_ticks, clear_ticks, effort_factor, lean_add_rad, percepts_seen, hand_n,
    crank_angle_rad, hand_average_n, crank_angle_total_rad`. Массивы — `real_array` от `std::vector<double>(deque.begin(), deque.end())`.
  - Весь безымянный блок `senses_codec.cpp:529-554` (`real_array`, `real_list`, `int_list`) переносится над кодек
    реакции, без копий.
  - `decode_reaction_state` — тот же список `ncd::exact`. Массивы читаются `real_list` в `std::deque`.
  - Комментарий-список ключей в `senses_codec.hpp:41` обновить.
  - Конфигурация: `hand_window_s, hand_lag_s, hand_drop_ratio, pitch_rate_up_rad_s, …` — 14 ключей.
  - Схема `model_config.hpp:360`: `FLR_REAL(hand_window_s), FLR_REAL(hand_lag_s), FLR_REAL(hand_drop_ratio)` вместо
    двух прежних.

- [ ] **Шаг 6. Документы и программные конфигурации.** Во всех документах описи и программных местах:
  - убрать `hand_unload_n` и `unload_dwell_s`, добавить `hand_window_s = 0.5`, `hand_lag_s = 0.5`,
    `hand_drop_ratio = 0.7` на месте первых двух;
  - в документах состояния реакции убрать `unload_ticks` и добавить пустую историю
    (`"hand_n": [], "crank_angle_rad": [], "hand_average_n": [], "crank_angle_total_rad": 0.0`), если документ —
    состояние до первого тика (`percepts_seen = 0`);
  - документ состояния с `percepts_seen > 0` нельзя перевести без истории. Такие документы (эталоны
    `native_rider_intent`) регенерируются своим генератором. Если генератора нет — стоп со списком.

- [ ] **Шаг 7. Целевые тесты.**
  `mise run test 'front_lift_reaction|launch_intent|launch_plan|rider_senses|native_rider_intent|config_schema|welded_profile_config'`.
  - `rider_senses` печатает «reaction ride … acting reached/not reached». Это значение записать в отчёт; ожидания
    теста не трогать.

- [ ] **Шаг 8. Перезапись** — правило задачи 1, шаг 8.
  - Записи зелёных тестов с изменившимся `rider_reaction.predicate`/`phase` — стоп со списком: прежнее значение
    изменилось.
  - Записи красных тестов не трогаются.

- [ ] **Шаг 9. Коммит.** `feat(rider): criterion B for the hands in the front-lift reaction`.

- [ ] **Шаг 10. Проверка задачи** (с `ab`). Изменения причин известных красных — только канал `rider_reaction`;
  каждое — в `EXPECT` с объяснением (`launch4/expect-task2.tsv`). Другое изменение — стоп.

---

### Задача 3. Работа анатомического упора райдера — отдельная строка потерь

**Файлы:**
- Изменить: `native/src/runtime/step.cpp` (`numerical_constraint_powers`, около `:931-943`),
  `native/src/runtime/accounting.cpp` (`raw_work` `:73-98`, `account_energy` `:162-210`),
  `native/src/recording/catalog_data.inc`, `native/resources/recording/catalog-v1.json`
- Тест: `native/tests/reference/test_energy_ledger.cpp`

**Interfaces:**
- Produces: ключ строки ограничения `rider_end_stop:<joint id>`; ключ канала `energy.rider_end_stop_loss_step_j`.

- [ ] **Шаг 1. Тесты (падают).** В `test_energy_ledger.cpp`:

```cpp
// Launch part 4, decision 7: joint-limit rows of rider joints are the
// anatomical end stops — exactly the limited joints whose name starts with
// "rider_" — and the welded model limits exactly the envelope joints.
void test_rider_end_stops_are_the_envelope_joints() {
    // Compile the welded articulated model from viewer_physics_welded.toml
    // (its envelope map and its lock_ankles value, as the ride does). From the
    // compiled mjModel collect the joints with jnt_limited set and a name
    // starting with "rider_"; require that set to equal the rider_-prefixed
    // joints of configuration::kRiderJoints (rider_spec.hpp:128-133) that
    // the model contains (ankles are absent when lock_ankles holds), and
    // require that no non-rider joint name starts with "rider_".
}

// The end-stop rows leave the numerical constraint work and become a loss:
// loss_j and loss_step_j carry -sum(power) * dt of the rider_end_stop rows,
// rider_end_stop_loss_step_j publishes it, and constraint_absolute_j no
// longer counts them; joint_limit rows of bike joints stay numerical.
void test_end_stop_work_is_a_loss() {
    // RawStep with numerical_constraint_power_w =
    //   {{"rider_end_stop:12", -4.}, {"joint_limit:3", .5}, {"equality:0", 1.}},
    // dt = .001: rider_end_stop_loss_step_j = .004, loss_step_j = raw.loss_step_j + .004,
    // constraint_absolute_j = .0005, closure_signed_j = .001.
}
```

  - Тела тестов исполнитель пишет по образцу соседних в файле (`test_energy_ledger.cpp:300-460` строит `RawStep`
    и зовёт учёт) с ровно этими числами.
  - Список шарниров — `configuration::kRiderJoints` (`rider_spec.hpp:128-133`). Отдельного списка оболочки нет:
    предел ставит `articulated_rider.cpp:42-52`, только при карте оболочки и не для лодыжек при `lock_ankles`.
    Поэтому сравнение идёт по скомпилированной модели.

- [ ] **Шаг 2. Запуск — падает.** `mise run test 'energy_ledger'`.

- [ ] **Шаг 3. Ключ строки.** В `numerical_constraint_powers` для `mjCNSTR_LIMIT_JOINT`:

```cpp
            case mjCNSTR_LIMIT_JOINT: {
                // Launch part 4, decision 7: a rider joint's limit is the
                // anatomical end stop (articulated_rider.cpp:38-53), charged as
                // a loss in accounting.cpp, not as numerical constraint work.
                const char *joint = mj_id2name(model, mjOBJ_JOINT, efc_id[static_cast<std::size_t>(i)]);
                kind = joint != nullptr && std::string_view(joint).starts_with("rider_") ? "rider_end_stop"
                                                                                          : "joint_limit";
                break;
            }
```

- [ ] **Шаг 4. Учёт.**
  - `raw_work`: строки с префиксом `rider_end_stop:` не идут ни в `constraint`, ни в `closure`. Комментарий над
    циклом дополнить: «rider_end_stop rows are the anatomical end stops: a loss, account_energy».
  - `account_energy`:

```cpp
    // Launch part 4, decision 7: the rider's anatomical end stops (MuJoCo's
    // default limit spring-damper on the envelope joints) are a loss line.
    double end_stop_power = 0.;
    for (const auto &[name, power] : raw.numerical_constraint_power_w)
        if (name.starts_with("rider_end_stop:")) end_stop_power += power;
    const double end_stop_loss = -end_stop_power * config.timestep_s;
    e.loss_j += raw.loss_step_j + end_stop_loss;
```

    вместо `e.loss_j += raw.loss_step_j;`.
  - В `result`: `{"loss_step_j", Wire(raw.loss_step_j + end_stop_loss)}` и сразу после —
    `{"rider_end_stop_loss_step_j", Wire(end_stop_loss)}`.
  - Каталог канала `energy` получает новый ключ по образцу `loss_step_j` в обоих файлах:
    `native/src/recording/catalog_data.inc` (около `:838-855`) и `native/resources/recording/catalog-v1.json`.
  - Разбиение сил `constraint_components` (`step.cpp:889-900`, группа `joint_limits`) не меняется: упор
    переклассифицируется только в учёте работы. Тесты этой группы (`test_energy_ledger.cpp:455`,
    `strictness_reference_support.hpp:48`) не трогаются.

- [ ] **Шаг 5. Целевые тесты.** `mise run test 'energy_ledger|rider_senses|launch_intent|realistic_pedelec_acceptance'`.
  - Выходы зелёных тестов — только новый ключ (`check_rerecord.py`).
  - Если в зелёной записи меняется `loss_j`, `loss_step_j` или `constraint_*` (поездка касалась упора) — стоп со
    списком.

- [ ] **Шаг 6. Коммит.** `feat(runtime): the rider's anatomical end stops are a loss line`.

- [ ] **Шаг 7. Проверка задачи** (с `ab`). Изменения причин известных красных — ключ `rider_end_stop_loss_step_j`,
  значения энергии в поездках с упором, исход поездок, которые раньше обрывались на `energy.constraint_work`. Каждое —
  в `launch4/expect-task3.tsv` с объяснением. Другое — стоп.

---

### Задача 4. Включение: таблицы трёх типов, план и действия реакции

**Файлы:**
- Изменить: `examples/research/viewer_physics_{welded,distributed,pneumatic}.toml` — разделы `[rider_launch]` и
  `[front_lift_reaction]`
- Создать: `examples/research/rider_type_careful.toml`, `examples/research/rider_type_aggressive.toml`
- Изменить: `native/tests/reference/test_welded_profile_config.cpp`,
  `native/tests/reference/test_realistic_pedelec_acceptance.cpp`
- Создать (вне репозитория): `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/parity/`

- [ ] **Шаг 1. Тест профилей (падает).** В `test_welded_profile_config.cpp`:
  - три профиля и две копии типов загружаются;
  - их `rider_launch` и `front_lift_reaction` равны таблице решения 5, поле за полем;
  - текст `rider_type_{careful,aggressive}.toml` без комментариев и пустых строк совпадает с текстом
    `viewer_physics_welded.toml` везде, кроме двух разделов.

  Сравнение разделов строится разбором TOML (`load_physics` трёх файлов и сравнение всех остальных записей
  `SimulationPhysicsConfig`); если такого равенства нет, — построчно по тексту вне двух разделов.

- [ ] **Шаг 2. Профили.**
  - В трёх исследовательских профилях блоки `[rider_launch]` и `[front_lift_reaction]` заменяются на блоки
    обычного райдера: дословно `gradecap/profiles/g_normal_leanm.toml` плюс поля задач 1–2. Комментарии над
    блоками: «launch part 4 — the normal rider, calibrated (gradecap §17, user rulings 2026-11-12)».
  - Копии типов — `viewer_physics_welded.toml` целиком, разделы — дословно из `g_careful_leanm.toml` и
    `g_aggressive_leanm.toml` плюс поля задач 1–2. Предел по уклону — из решения 5. Над каждым отличием от
    обычного — комментарий с числом обычного.

- [ ] **Шаг 3. Побайтная сверка с пробными поездками.**
  - Скопировать `gradecap/harness/{probe.cpp,build_probe.sh,run_one.sh,cal_metrics.py}` в `launch4/parity/`.
  - В копии `build_probe.sh` убрать компиляцию и ссылки на оба объекта правок.
  - Профили для пробника — новые файлы репозитория, сами профили, не копии: обычный — `viewer_physics_welded.toml`,
    осторожный — `rider_type_careful.toml`, агрессивный — `rider_type_aggressive.toml`.
  - Трассы и параметры остановки — из `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/parity-baseline/manifest_fck.txt`
    и `manifest_rv25.txt`. Оркестратор восстановил их и проверил: 33 поездки старого пробника по ним побайтно равны
    `gradecap/runs/` (`parity-baseline/run.sh`). В этих списках заменяются столбец профиля (файл репозитория по
    типу) и столбец окружения (`-`: правок больше нет).
  - Собрать на текущей сборке ветки, прогнать 33 поездки.
  - `runs/<имя>_meta.csv` сравнить с `gradecap/runs/` через `cmp`.
  - В `runs/<имя>.csv` столбцы `energy.*` меняет задача 3 (новый ключ, работа упора уходит в потери). Поэтому
    столбцы сопоставляются **по имени**, сравнивается текст значений `%.10g` всех столбцов, кроме `energy.*`.
    Лишний или недостающий столбец не из `energy.*` — тоже расхождение.
  - Столбцы `energy.*` отдельно. Изменились только `energy.loss_j`, `energy.loss_step_j`, `energy.residual_j`,
    `energy.solver_constraint_work_j`, `energy.constraint_*` (`constraint_signed_j`, `constraint_absolute_j`,
    `constraint_work_ratio`, `constraint_work_ok`) и новый `energy.rider_end_stop_loss_step_j`. Изменения — в
    отчёт: число строк и наибольшая разница.
  - Всё совпадает — шаг пройден. Иначе — стоп: имя, первая разная строка, столбец, оба значения.

- [ ] **Шаг 4. `test_realistic_pedelec_acceptance`** — решение 12.
  - Интервалам (`IntervalRow`) добавить фазу плана из канала `rider_launch.phase` и просьбу из `rider_commands`
    (имя поля взять из записи канала).
  - Проверки ближайшей звезды и полосы — только для переключений с фазой `blend` или `done`.
  - Для переключений в `hold`/`release`/`accelerate`/`handover` — новое требование: направление «up», в этом или
    предыдущем интервале просьба «up».
  - Прочие требования не меняются.

- [ ] **Шаг 5. Целевые тесты.**
  `mise run test 'welded_profile_config|realistic_pedelec_acceptance|seated_pedaling_cycle|seated_climb_surge|rider_senses|launch_intent|front_lift_reaction'`.
  - Провал теста, не упомянутого в шагах 1 и 4, — стоп с числами. Ожидания не трогать.
  - Строку `launch ride:` из `rider_senses` записать в отчёт.

- [ ] **Шаг 6. Список эталонов** (решение 11).
  - Прогнать `ctest -R` по известным красным и по тестам-читателям эталонов (`facts.md` H).
  - Составить `launch4/golden-list-task4.md`: тест, файл, ключи, первая разная строка, причина («план включён»,
    «наклон старта», «действия реакции»).
  - Ничего не перезаписывать.

- [ ] **Шаг 7. Коммит.** `feat(profiles): calibrated launch tables for the careful, normal and aggressive riders`.

- [ ] **Шаг 8. Проверка задачи** (без `ab`). Стопы «new red tests» и «causes differ» сверить со списком шага 6;
  совпадение — отчёт, несовпадение — стоп.

---

### Задача 5. Мягкий старт уходит

**Файлы:**
- Изменить: всё из описи «мягкий старт»
- Тест: `native/tests/reference/test_seated_climb_surge.cpp`, `test_welded_profile_config.cpp:232-255` и тесты
  из `git grep -l 'launch_effort_ramp_s\|launch_ramp_s\|launch_decided' -- native/tests`

- [ ] **Шаг 1. Где мягкий старт действует без плана** (решение 8).
  - Перечислить:
    - тесты и программные конфигурации, где `seated_climb.enabled`, `rider_enabled`, нет нажима вызывающего и
      `drive_mode` не `crank_effort`/`articulated_effort`;
    - поездки на встроенном профиле (`default_physics`, план выключен по решению 6), где поездка стартует с места
      (`initial_speed_mps = 0`) на уклоне ≥ 3 %;
    - любая поездка, тест или программная конфигурация с включённым намерением, где на первом задержанном тике
      уклон предпросмотра ≥ 3 % и шатун ≤ 1 рад/с **при любой скорости**. Сюда входят старты на ходу
      (`initial_speed_mps > 0`, в том числе умолчание CLI 2 м/с, `cli/configuration.cpp:205`) на трассах с
      подъёмом в начале. Искать по программам `validation/*.cpp`, тестам с `initial_speed_mps` и трассам.
  - Результат — `launch4/soft-start-holes.md` со ссылками `файл:строка`. Непустой список — стоп.

- [ ] **Шаг 2. Тест.**
  - В `test_seated_climb_surge.cpp` проверки мягкого старта удаляются. Вместо них — тест: на 15 % с места при
    профиле `viewer_physics_welded.toml` предел нажима в первые 1,5 с — нажим плана (`rider_launch.effort_nm`),
    а не прежняя доля `t/1.5` от предела политики. Проверка: `effort_ceiling_nm` намерения равен
    `rider_launch.effort_nm` на каждом интервале с весом 1.
  - В `test_welded_profile_config.cpp:232-255` проверки трёх полей заменяются проверкой, что профиль с любым из
    них не загружается и называет поле.

- [ ] **Шаг 3. Код.**
  - Удалить поля, проверку `intent.cpp:252-265`, взвод и часы `:362-369`, множитель `:383-388` (потолок —
    `effort_nm_`), члены `intent.hpp:187-188, 227-228` и комментарий `:60-61`.
  - Убрать ключи `launch_ramp_s`, `launch_decided` из кодека политики, строки схемы, загрузчика, умолчаний и
    списков стирания.
  - Удалить поля из пяти программных конфигураций.

- [ ] **Шаг 4. Документы.** Ключи удаляются из всех 52 документов и встроенных TOML (копия `migrate_json.py`).
  Бинарные `.bin` — `migrate_rider_intent_bins.py` (копия).

- [ ] **Шаг 5. Повтор сверки части 4б, задача 4, шаг 3** — драйвер `launch4b/parity/` на новой сборке, 96 поездок
  против образцов `pneumatic-reach/f1/runs/kd88/` (решение 9). Совпадение или расхождения с разницей метрик
  `cal_metrics.py --window-handover --rb-from-release` — в отчёт.

- [ ] **Шаг 5а. `reference.rider_brake_channel`.** Тест `test_flags_match_old_semantics_without_commands`
  (`native/tests/reference/test_rider_brake_channel.cpp:295`) проверяет флаги вызывающего без команд райдера. С
  задачи 4 план старта включён в профиле, на 15 % он держит тормоз в фазе `hold`, и тест красный
  (`FAIL every resolved flag equals the single old predicate`). Тест едет с выключенным планом
  (`rider_launch.plan_enabled = false` в физике его поездки, как `test_launch_preparation.cpp:352`) и с
  комментарием об этом. Проверки не ослабляются и не переставляются. После шага тест зелёный.

- [ ] **Шаг 6. Целевые тесты** — набор задачи 4, шаг 5, плюс `seated_climb_surge`. Список эталонов дополняется
  (`launch4/golden-list-task5.md`).

- [ ] **Шаг 7. Коммит.** `refactor(rider): the launch plan replaces the soft start`.

- [ ] **Шаг 8. Проверка задачи** (без `ab`) — как в задаче 4, шаг 8.

---

### Задача 6. Тесты поездок с утверждёнными порогами

**Файлы:**
- Изменить: `native/tests/reference/launch_ride_support.hpp` (окружение с профилем и строгими проверками, трасса с
  покрытием)
- Создать: `native/tests/reference/launch_ride_metrics.hpp`,
  `native/tests/reference/test_rider_launch_rides_{normal,careful,aggressive}.cpp`
- Изменить: `native/tests/reference/Phase3Ports.cmake` — три теста с метками `launch rider slow`

- [ ] **Шаг 1. Окружение.** В `launch_ride_support.hpp`:

```cpp
[[nodiscard]] inline terrain::TrackSpec track(std::string_view knots, double length, std::string_view surface) {
    return terrain::track_from_dict(terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = " + std::to_string(length) + "\nsurface = \"" + std::string(surface) +
        "\"\ngrade_profile = { knots = " + std::string(knots) + " }\n"));
}
```

  - Прежний `track(knots, length)` зовёт новый с `"hardpack"`.
  - Новое `launch_environment(std::string_view profile, terrain::TrackSpec track, double duration_s)` — тело
    `environment()` с тремя отличиями:
    - физика из `repo_file(profile)`;
    - строки `launch_ride_support.hpp:86-87` (`maximum_energy_residual_ratio = 1e9`,
      `stop_on_model_violation = false`) в новое тело **не копируются**. Оба поля остаются умолчаниями запроса
      (`research/environment.hpp:22`: 0,05 и `true`), как в пробнике. `launch_environment` сам требует это до
      передачи запроса в `prepare_environment`:
      `bt::require(request.experiment.stop_on_model_violation && request.experiment.maximum_energy_residual_ratio == research::ExperimentConfig{}.maximum_energy_residual_ratio, …)`
      (имя типа взять из `environment.hpp`);
    - `record_decimation = 1`, режим записи по умолчанию.
  - `PedelecEnv` получает `root_pitch_qpos` (из `root_pitch`) для `ColumnLayout`.

- [ ] **Шаг 2. Метрики.** `launch_ride_metrics.hpp`: перенос `gradecap/harness/cal_metrics.py` (`summarize` с
  `--window-handover --rb-from-release`) на интервалы `runtime->recorded_intervals()`:
  - числа — из `row->flat_columns(layout)`, теми же именами, что столбцы CSV пробника;
  - фаза — из канала `rider_launch.phase`.

```cpp
struct LaunchCase {
    std::string name;
    double grade = 0.;
    std::string surface;   // asphalt | wet | loose
};

struct LaunchMetrics {
    std::string outcome;        // runtime snapshot outcome or "none"
    bool crashed = false;       // outcome != "none" (crash:loop_out, crash:* and the rest) — verdicts2.py:60-61
    std::optional<std::string> failure;   // first model violation / invalid step, strict mode
    // Inside the launch window (hold entry .. handover entry, plan decision 10):
    std::vector<std::pair<double, double>> lifts;   // nf <= 50 N longer than 0.05 s: start, length
    std::vector<std::pair<double, double>> slips;   // util > 0.9 longer than 0.05 s
    double slip_peak = 0.;
    double rollback_m = 0.;     // min x from the release entry - x at the release entry (<= 0)
    double leg_ema_peak_rpm = 0.;
    double saddle_min_n = 0., front_foot_min_n = 0., rear_foot_min_n = 0.;   // rider_welds.*.normal_n minima
    bool budget_exceeded = false;   // rider_effort_budget_exceeded > 0.5 on a window row (cal_metrics.py:378)
    bool balance_lost = false;      // rider_balance.balance_lost > 0.5 on a window row (:380)
    // Whole ride, not windowed (cal_metrics.py:359-362): the labels of the
    // string-array channels attachment_violations, rider_joint_power_violations,
    // rider_joint_speed_violations, rider_strength_violations.
    std::vector<std::string> violations;
    // Controller window, cal_metrics.py:341-353 literally: the lift start is the
    // first lift episode longer than 0.05 s, else the first nf <= 50 N episode
    // at all; the reaction's first acting tick is searched from the hold entry
    // over the whole ride; window = acting - lift when acting >= lift, else
    // crash time - lift when the ride crashed, else open (nullopt).
    std::optional<double> lift_start_s;
    std::optional<double> controller_window_s;
    // Whole ride from t = 0, not clipped (3.031 s after part 4b is full-run):
    std::optional<double> t20_s;
    std::optional<double> t20_from_release_s;   // printed only (spec: second basis)
    std::optional<double> hold_s, release_s, accelerate_s, handover_s;
};

// Rides `profile` on a 150 m constant-grade track of `c` for 20 s, the
// calibration probe's drive (caller control empty, no caller brake).
[[nodiscard]] LaunchMetrics ride(std::string_view profile, const LaunchCase &c);
```

  Правила совпадают с `cal_metrics.py` (`summarize` с `--window-handover --rb-from-release`):
  - эпизоды — `episodes()`;
  - окно — строки от входа в `hold` до входа в `handover` (`cal_metrics.py:164-171`, обрезка строк). Без
    `handover` — до конца поездки;
  - все оконные метрики считаются **на обрезанных строках**. Экспоненциальное среднее ног —
    `prev + min(1, dt/0.35) · (v − prev)` по обрезанным строкам с начала поездки, пик — от входа в `hold`
    (`:308-314`);
  - `util` считается при `nn > 1 && mu > 0`;
  - опоры — минимумы `rider_welds.{saddle,front_pedal,rear_pedal}.normal_n` в окне (`:372-375`);
  - эпизоды отрыва и буксовки берутся, если их **конец** в окне (`i1e >= i0`, `:265-267`, `:284-287`);
  - нарушения — метки строковых каналов `attachment_violations`, `rider_joint_power_violations`,
    `rider_joint_speed_violations`, `rider_strength_violations` **по всей поездке** (`:359-362`). Они не входят в
    `flat_columns()` (строки и строковые массивы отбрасываются). Читать их из `row->channels` через
    `sample_wire::find`, как `probe.cpp:289-348`, и делить по `'|'`, если это строка;
  - опоры, бюджет усилия и потеря равновесия — по строкам окна (`:364-380`). Нарушения — по всей поездке;
  - окно для контроллера — `:341-353` построчно, с запасным `eps50`, условием `t_act >= t_lift` и предпочтением
    `acting` перед опрокидыванием.

  Исключение из построчного правила — `t20_s`: первая строка всей поездки (не обрезанной) со скоростью
  ≥ 20 км/ч, отсчёт от t = 0. Так в отчёте калибровки получено 3,239 с («full-run t20; window clips before
  20 km/h», §17.4). В оконном `cal_metrics.py` для этого случая — `n/a`. Порог 3,25 с — время на `f616f525`
  (3,254 с, часть 1, задача 1, `1fe7d0ee`), округлённое в строгую сторону; так он стоит в задаче 13 дорожной
  карты. `t20_from_release_s` (от входа в `accelerate`) только печатается — вторая основа спецификации («Время
  разгона»).

  Трасса: `[[0.0, g], [150.0, g]]`, длина 150, покрытие из случая. Каждый тест печатает строку на случай: имя, исход,
  отрывы, буксовка, пик, откат, каденс, опоры, окно, t20 (обе основы), фазы.

  Проверка переноса метрик (не в ctest). Сначала пробник задачи 4 (`launch4/parity/`) на коде **после задачи 5**
  проезжает `normal g00`, `normal wet10`, `careful g20`, `aggressive g15` в новый каталог `launch4/parity-t6/`.
  Затем строки теста для этих случаев сравниваются с `cal_metrics.py --window-handover --rb-from-release` на этих
  записях: код один, поездки одни. Сравниваются: отрывы, буксовка, пик, откат, пик каденса, опоры, окно. `t20_s` для `normal g00`
  сравнивается с полным прогоном: первая строка `speed_mps ≥ 20/3.6` в CSV, 3,03125 (после части 4б; до неё 3,23875). Совпадение до напечатанного
  знака — в отчёт; расхождение — стоп.

- [ ] **Шаг 3. Тесты.** Общая проверка «чистого старта»:

```cpp
// The seated limits every launch keeps (spec «Проверки», roadmap task 13,
// verdicts2.py:31-61): over the ride no crash, no strict-mode failure and no
// violation label; inside the window no support pull beyond -6 N (saddle and
// both feet), no effort-budget excess, no balance loss.
[[nodiscard]] std::vector<std::string> seated_limit_failures(const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out;
    if (m.crashed || m.failure) out.push_back(c.name + ": " + m.outcome + (m.failure ? " " + *m.failure : ""));
    if (m.saddle_min_n < -6. || m.front_foot_min_n < -6. || m.rear_foot_min_n < -6.)
        out.push_back(c.name + ": a saddle or pedal support below -6 N");
    if (m.budget_exceeded) out.push_back(c.name + ": rider effort budget exceeded");
    if (m.balance_lost) out.push_back(c.name + ": balance lost");
    for (const auto &v : m.violations) out.push_back(c.name + ": violation " + v);
    return out;
}

// The approved thresholds (spec «Проверки», plan decision 10): the seated
// limits, and inside the launch window no lift, no wheelspin, rollback from
// release at most 10 mm, leg cadence at most 110 rpm.
[[nodiscard]] std::vector<std::string> clean_start_failures(const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out = seated_limit_failures(c, m);
    if (!m.lifts.empty()) out.push_back(c.name + ": front lift in the launch window");
    if (!m.slips.empty()) out.push_back(c.name + ": rear wheelspin in the launch window");
    if (m.rollback_m < -.010) out.push_back(c.name + ": rollback beyond 10 mm");
    if (m.leg_ema_peak_rpm > 110.) out.push_back(c.name + ": leg cadence above 110 rpm");
    return out;
}

// 20 % (and careful 20 % under the normal rule, П12): the seated limits,
// every in-window lift at most 0.3 s.
[[nodiscard]] std::vector<std::string> short_lift_failures(const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out = seated_limit_failures(c, m);
    for (const auto &[start, length] : m.lifts)
        if (length > .3) out.push_back(c.name + ": a lift longer than 0.3 s at " + std::to_string(start));
    return out;
}

// The aggressive rider (П10, spec «Проверки»): the seated limits, and when
// the front lifts the controller window (cal_metrics.py:341-353) is at least
// 0.4 s; an open window passes. Calibration measurement (gradecap
// _h_rv25_a_g15/g20 ctrl_window): lifts 169 and 296 ms, the reaction never
// acted, no crash — open. The calibration verdict for these rows
// (verdicts2.py:62-64) was unconditional and is not the evidence.
[[nodiscard]] std::vector<std::string> controller_window_failures(const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out = seated_limit_failures(c, m);
    if (m.lift_start_s && m.controller_window_s && *m.controller_window_s < .4)
        out.push_back(c.name + ": controller window " + std::to_string(*m.controller_window_s) + " s");
    return out;
}
```

  Случаи:
  - **обычный** (`viewer_physics_welded.toml`):
    - чистый старт на асфальте 0, 5, 8, 10, 12, 15 %, на мокром и рыхлом 10 %;
    - на асфальте 0 % дополнительно `t20_s ≤ 3.25`;
    - 20 % на асфальте — правило короткого отрыва;
    - промежуточные (П13): чистый старт на 7, 9, 11, 14, 17 % × асфальт, мокрый, рыхлый;
    - информация, без требований: 22 и 25 % на асфальте;
  - **осторожный** (`rider_type_careful.toml`) — те же случаи и правила, без `t20`; 20 % — правило короткого
    отрыва (П12);
  - **агрессивный** (`rider_type_aggressive.toml`):
    - на асфальте 15 и 20 % и промежуточном 17 % (асфальт, мокрый, рыхлый) — `controller_window_failures` (П10);
    - остальные случаи — только `seated_limit_failures`; прочие метрики печатаются. Это **новое требование**:
      калибровка не проверяла у агрессивного опоры и нарушения (вердикт был безусловным), хотя метки нарушений во
      всех 33 финальных поездках — «none». Провал — стоп и пользователь, как у промежуточных уклонов;
    - 22 и 25 % — информация.

  Тест проезжает все случаи, печатает таблицу, затем `bt::require(failures.empty(), <все строки через "; ">)`.

  Ожидаемый итог по калибровке — пройдены все 9 + 9 + 2 калиброванных случаев. Промежуточные уклоны и 22/25 % не
  калибровались. Провал любого требуемого случая — стоп: таблица случаев пользователю. Пороги и таблицы не
  меняются.

- [ ] **Шаг 4. Регистрация** в `Phase3Ports.cmake` рядом с `launch_intent`:

```cmake
# Launch part 4 — the launch rides against the approved thresholds, one
# executable per rider type (about 26 rides of 20 s each).
bike_add_reference_test(rider_launch_rides_normal ENGINE
  SOURCES test_rider_launch_rides_normal.cpp LIBRARIES bike_app bike_validation
  LABELS launch rider slow)
bike_add_reference_test(rider_launch_rides_careful ENGINE
  SOURCES test_rider_launch_rides_careful.cpp LIBRARIES bike_app bike_validation
  LABELS launch rider slow)
bike_add_reference_test(rider_launch_rides_aggressive ENGINE
  SOURCES test_rider_launch_rides_aggressive.cpp LIBRARIES bike_app bike_validation
  LABELS launch rider slow)
```

  Если в репозитории ведётся заявленная длительность тестов (`<git-common-dir>/bike-gate/timings.tsv`,
  комментарии `Phase3Ports.cmake`), — заявить по образцу соседних медленных.

- [ ] **Шаг 5. Запуск.** `mise run test 'rider_launch_rides'` (в отчёт — три таблицы целиком).

- [ ] **Шаг 6. Коммит.** `test(rider): launch rides against the approved thresholds`.

- [ ] **Шаг 7. Проверка задачи** (без `ab`) — как в задаче 4, шаг 8. Три новых медленных теста не входят в
  обычный набор `check_change`.

---

### Задача 7. Перезапись эталонов (только после «да» пользователя)

- [ ] **Шаг 1.** Оркестратор показывает пользователю сводный список: `golden-list-task4.md`,
  `golden-list-task5.md`, списки части 4б `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/golden-list-task{2,3,4}.md`
  и записанные выходы `launch4b/inventory/excluded-recorded.txt`. Сюда же поимённо: начальные состояния
  `tests/fixtures/initial_state/**`, `tests/fixtures/golden/*/*/*/initial_state.json` и
  `tests/fixtures/config/poses/savage_pneumatic.npz`. Их контракт после части 4б не совпадает, тесты-читатели
  красные.
- [ ] **Шаг 2.** После «да»:
  - каждый тест из списка перезаписывается своим способом записи (`BIKE_RECORD=1` или генератор теста — как
    принято у этого теста);
  - для каждого файла `git diff --stat` и причина — в отчёт;
  - файлы вне списка не трогаются;
  - красный тест после перезаписи становится зелёным, только если в списке указана единственная причина
    «поведение»; иначе прежние причины сохраняются и сверяются `causes.py`.
- [ ] **Шаг 3. Коммит.** `test(golden): re-record launch-affected trajectories`.
- [ ] **Шаг 4. Проверка задачи** (без `ab`); новый список известных красных — `launch4/reds-after-task7.txt`.

---

### Задача 8. Документ решений, дорожная карта, очередь

- [ ] **Шаг 1.** `docs/agents/simulation-decisions.md`, «Действует». Записи части 4б (`483511f3`, `81a3ee26`:
  пункты 9, 26, 33 и «Пробовали» 13–16) уже есть, их не переписывать, а ссылаться на них:
  - правило старта: план, таблицы трёх типов (решение 5), пороги (решение 10, П10–П13), предел по уклону, подъём
    на 2,5 м/с, повтор просьбы 0,25 с;
  - реакция: критерий Б (0,5 / 0,5 / 0,7, опора > 0, без выдержки, И наклон). При ступеньке разгрузки признак
    сам гаснет через время опоры;
  - окно для контроллера агрессивного райдера: требование 0,4 с остаётся. Окно без действия реакции и без
    опрокидывания считается открытым (П10). Калибровка: реакция на отрывы агрессивного на 15 и 20 % не срабатывает,
    отрывы 169–296 мс проходят сами;
  - время до 20 км/ч на ровном — по всей поездке, порог 3,25 с вместо «не медленнее `f616f525`» (3,254 с);
  - тесты поездок старта проверяют опоры, бюджет усилия, равновесие, метки нарушений и строгий баланс энергии;
    рывки отдельно не проверяются (канала нет). Откат считается от входа в `release` (П11), строже основы
    калибровки (от `hold`);
  - строка потерь анатомического упора и известное отступление: демпфер MuJoCo по умолчанию на шарнирах оболочки
    (B ≈ 2770) против правила «демпфирования в шарнирах нет», до решения пользователя по упору;
  - пункт 42 (мягкий старт) заменяется ссылкой на план старта.
- [ ] **Шаг 2.** «Решено — не сделано»:
  - пункт 2 — сделаны «ослабить» и «наклониться», «короткое торможение задним» осталось;
  - новые пункты:
    - переключение вниз перед остановкой (П9, рестарты ползут около 6 с, затем подъём колеса после передачи по
      таймеру);
    - предел нажима в обычной езде на подъёмах (П8, П13: подъём колеса после передачи, опрокидывание на 25 %);
    - осторожный на 20 %: структурный подъём колеса 172–225 мс (§17.5).
- [ ] **Шаг 3.** «Пробовали — не сработало»:
  - предел, умноженный на долю трения покрытия (мокрый не исправился);
  - подъём на 1,5 м/с (новый подъём колеса 699 мс на рестарте);
  - темп осторожного 25–35 Н·м/с на 20 % (не исправил);
  - пределы ≤ 17 Н·м на 20 % у осторожного (остановка).
- [ ] **Шаг 4.** Строка статуса дорожной карты `2026-11-11-rider-launch-plan.md`: часть 4 выполнена, коммиты,
  задачи 12–15 заменены. Строку задачи 13 «Нужные трассы с постоянным уклоном добавить в `tests/fixtures/tracks/`»
  пометить: заменено трассами в коде (`launch_ride::track`). Новые трассы запрещены пунктом 4 «Трассы»
  документа решений.
- [ ] **Шаг 5. Коммит.** `docs: record the rider launch plan`.

---

### Задача 9. Итоговая проверка

- [ ] **Шаг 1.** `rough_uphill_extreme` на регрессию:
  `bike-measure-realtime --out /Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/final/extreme --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_extreme.toml --duration 60 --warmup 0 --runs 1`.
  - В отчёт: `.runs[0].prefix`, время реального счёта и число срабатываний реакции (фаза `acting`) за поездку.
  - Сравнение — с тем же прогоном на базе `22dc1b0d` (оркестратор кладёт его до задачи).
- [ ] **Шаг 2.** Информационные поездки `distributed` и `pneumatic`: обычный райдер, 0, 10, 15 % на асфальте,
  `launch_ride_metrics` через маленький исполняемый файл в `launch4/final/` (не в репозитории). Таблица — в отчёт.
- [ ] **Шаг 3.** `bash tools/run_tests.sh full`. Ожидается: красных не больше списка задачи 7, медленных — тоже.
- [ ] **Шаг 4.** Отчёт оркестратору: итог, время, три таблицы задачи 6, сверки задач 4–5.
