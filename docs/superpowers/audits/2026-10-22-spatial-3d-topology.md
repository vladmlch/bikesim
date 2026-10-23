# Пространственная топология — журнал проверки W13–W19

Статус: **исходники и проверки написаны; verification pending V03/V05**.
Дата записи: 2026-10-23. Сборка, конфигурация, компиляция, syntax-only,
исполнение тестов, MuJoCo и симуляции в W-фазе не выполнялись. Численные
значения ниже — критерии будущей проверки, а не результаты измерений.

Основание: [план W13–W19](../plans/2026-10-22-spatial-3d-native-02-topology.md),
[план верификации](../plans/2026-10-22-spatial-3d-native-09-verification.md),
frozen declarations W00 и контракты C2/C4/C5/C7.0.

## Область исходников

| Работа | Написанная граница | Будущий потребитель evidence |
| --- | --- | --- |
| W13 | Именованная топология из `mjModel`; отдельные qpos/dof spans; free-root 7/6; wheel/hub и steer bindings; SHA-256 модели и топологии; преобразования и body-origin velocity | V03, G1-core |
| W14 | Отдельные planar/constrained/free ветки MJCF; физическая рулевая шарнирная пара и actuator; IMU; collision masks; модельный mass/COM/inertia oracle | V03, G1-core |
| W15 | Scalar joints внутри смешанной free/ball модели; Jacobian геометрической цепи по nv; три drivetrain family; relative-spin тормоз; C4 rolling-loss ownership | V03, G1-core |
| W16 | Независимый pelvis free-root, shoulder swivel, positional grip connects, полные spatial support Jacobians, versioned shoulder state | V03, G1-core/G2-startup |
| W17 | Семейства runtime/writer/bootstrap `(1,1\|2,1)` и `(2,3,2)`; causal spatial inventory; полная tyre identity; cold bind/restore boundaries | V03, G2-startup |
| W18 | Constrained bridge, control/writer equivalence и длинные сценарии; отсутствие автоматической квалификации free-root | V05, G1-core/G2-startup |
| W19 | Интегрированная adversarial matrix, preservation полного состояния и реальных публикаций, covariance terrain/forces/load, закрытый release reporting | V05, G1-core/G2-startup |

Здесь `написано` не означает `implemented=true`, `qualified=true` или
пройденный gate. Такие флаги могут измениться только по результатам отдельной
V-фазы и подключения недостающих producer-компонентов.

## Матрица отрицательных входов

Файл: `native/tests/spatial/test_topology_negatives.cpp`. Каждая строка
вызывает настоящий cold decoder/binder/restore и требует `std::invalid_argument`
с именем поля. Проверяется побайтовая неизменность полного MJB и
`mjSTATE_INTEGRATION`, состояния настоящего runtime, canonical codec-состояния
причинных владельцев и фактического числа pending/retained publications.

| Вход | Проверяемая граница/поле | Результат выполнения |
| --- | --- | --- |
| Free root + planar tyre | `generate_mjcf`, native TOML, writer3 saved-family validator; `tires.backend` / `config.tire.backend` | pending V05 |
| Constrained root + roll-heading controller | Direct request, TOML, saved spatial config; `spatial_3d.steer.controller` | pending V05 |
| Неверный kind `root_free` | `bind_topology`; `topology.root_free` | pending V03/V05 |
| Неверная ось constrained root | `bind_topology`; `topology.root_y` | pending V03/V05 |
| Повтор joint name | `bind_topology`; `topology.front_wheel_spin` | pending V03/V05 |
| Steering actuator на другом joint | `bind_topology`; `topology.steer_torque` | pending V03/V05 |
| Одинаковые размеры, иной body/joint order | Разные MJB/topology hashes; `state.spatial.topology_fingerprint` | pending V05 |
| Stale terrain digest | `validate_spatial_identity`; `state.spatial.terrain_sha256` | pending V05 |
| Несовместимые runtime/writer/bootstrap schema | `validate_schema_family`, synthetic replay/import `compose_bootstrap`; `bootstrap.schema_family` | pending V05 |
| Невалидная quaternion каждого free/ball joint | `decode_bootstrap_state` на bootstrap2 с настоящим W27 material codec; `state.integration_state.qpos` | pending W27 + V05 |
| Другой shoulder branch/profile/layout | `SpindleController::restore`; `rider_controller.rider.spatial` | pending V03/V05 |

Публикационный witness — существующий `bootstrap/welded`, созданный до этой
работы. Он остаётся в t=0 и не служит spatial qualification. Снимок использует
`RuntimeRigAccess::inspect` и production `capture_initialized`; модель,
integration state, drive/rider/controller/intent/signals, tyre row, accounting,
presentation и ожидающие публикации сравниваются как владеющие данные.
Отсутствующий `model.mjb` или другой прежний capture — ошибка проверки, а не
пустой fixture, фиктивный счётчик или успешно пропущенный test.

Bootstrap2 quaternion control отдельно обозначен как **cold decoder fixture**:
материальные массивы fast-tier имеют действительные размеры, кодируются
`encode_tyre_state` и сначала проходят `decode_tyre_state`. Остальные staged
documents берутся из прежнего capture. Такой control не объявляется физически
запускаемым bootstrap смешанной модели и не заменяет continuation-тест W27.

## Численные критерии и будущие измерения

| Измерение | Acceptance | Измерено |
| --- | --- | --- |
| Quaternion rotation, inverse, sign equivalence | `1e-14` | null |
| Body-origin/world angular velocity | `1e-12` | null |
| Free/constrained physical mass, COM, inertia tensor | `1e-9` | null |
| Generalized/world virtual work | `1e-10 * max(1, abs(world_power))` | null |
| Внутренние force/reaction pairs, net wrench | `1e-10` | null |
| Chain length при общем translation/yaw/roll | `< 1e-12 m` | null |
| Manifold central-difference chain Jacobian | `5e-8`, шаг `1e-6` | null |
| Drive/brake local torque при heading +90° | `1e-12 Nm` | null |
| Normal-law load, point/force covariance, power при общем yaw +90° | `1e-9` | null |
| Body-frame speed при yaw +90°, оба направления движения | `1e-9` | null |
| Attitude около pitch singularity | `valid=false`, все поля конечны | null |
| Grip gap в заявленном диапазоне | `<= 0.001 m` | null |
| Grip constraint rank | 6 независимых positional constraints | null |
| Grip steering mobility | `> 0` | null |
| Internal rider power residual | `1e-8 * max(1, abs(drive_power))` | null |
| Планарный native MJCF и force-staging golden | точные прежние байты/f64 | null |
| Constrained bridge / long bridge | критерии W18/V05 | null |

Covariance test использует две независимо заданные аналитические плоскости
`z=.08x-.03y` и её world-Z поворот `z=.03x+.08y`, явный поворот входных
кинематик/скоростей и настоящий `query_geometry` → `update_carcass`.
Ненулевая нагрузка обязательна, поэтому нулевой или airborne ответ не может
ложно подтвердить covariance. Coefficients имеют `provenance_status=synthetic`;
они не являются измерениями MTB-шины. Полный production writer/ride load
response и continuation остаются зависимостями W25/W27, V04/V05.

## Frozen-contract различия и незакрытые evidence-зависимости

1. **Top-level rider BALL.** В frozen `RootBinding` есть planar3/constrained4/free6,
   но нет BALL kind и его 4/3 span. Binder принимает независимые посторонние
   BALL joints, учитывает их разные qpos/nv offsets и явно отвергает BALL
   rider root с диагностикой C10. FREE rider root реализован. Нельзя молча
   маркировать BALL как FREE или расширять W00 в этой записи; необходим C10
   перед поддержкой такой rider topology.
2. **Origin body velocity.** Поза хранит `xpos/xquat` regular body frame.
   Для согласованного начала скорости вызывается `mjOBJ_XBODY`: `mjOBJ_BODY`
   использует inertial frame `xipos/ximat`. Тест специально смещает inertial
   COM относительно body origin. Это исправляет буквальное указание BODY
   в W13, сохраняя физический контракт world velocity at body origin.
3. **Euler margin.** Frozen API называет параметр `singularity_margin_rad`.
   Реализация сравнивает `abs(pitch)` с `pi/2 - margin_rad`. Плановый пример
   `abs(sin(pitch)) > 1-margin` имеет другое размерностное значение. Изменять
   frozen декларацию не потребовалось; диагностическая validity использует
   радианный margin и конечные углы.
4. **Independent planar MJCF baseline.** В предоставленном дереве не найден
   прежний byte-identical native MJCF capture для T16 input. Существующий
   `migration/model/welded.xml` — Python/compiled-array oracle, он не выдан
   за native byte oracle. `tests/fixtures/spatial3d/model/manifest.json`
   содержит `native_mjcf_sha256_hex:null` и ожидаемый путь. Соответствующий тест
   закрыт до независимого capture; новый output не используется как expected.
5. **W20/W21/W25/W27 ещё не являются частью W06–W19.** W19 вызывает frozen
   реальные geometry/carcass/state APIs; production bootstrap2 runtime также
   требует настоящий tyre writer. Отсутствие определений/fixture не закрывает
   gate. Новые символы не заменены stub-функциями и фиктивными нулевыми rows.
6. **Внутренняя chain geometry.** Spatial branch переносит реальные sprocket
   sites в frame и вычисляет производные через 3×nv point/angular Jacobians.
   Для текущей геометрии sprockets остаются в одной bike-local плоскости;
   непараллельная axle или lateral departure отвергается явно. Общие rigid
   yaw/roll/translation не превращаются в длину/работу внутренней передачи.

## Заполняется только в V03/V05

```json
{
  "status": "pending_verification",
  "v03_run_id": null,
  "v05_run_id": null,
  "build_identity": null,
  "platform": null,
  "model_sha256": null,
  "terrain_sha256": null,
  "normal_load_covariance_error_n": null,
  "virtual_work_residual_w": null,
  "grip_max_gap_m": null,
  "planar_mjcf_oracle_sha256": null,
  "g1_core_passed": false,
  "g2_startup_passed": false,
  "free_root_qualified": false,
  "articulated_rider_qualified": false
}
```

После исполнения каждый результат должен указывать run manifest, точный
binary/source identity и independent oracle. Constrained bridge не переводит
free-root в `implemented`, а rigid-rider модель не квалифицирует articulated.
