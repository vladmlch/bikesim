# Native multi-contact tyre: portable reference and C++ port of `distributed_2d_reference` — design

Дата: 2026-10-11. Ветка (будущая): `feat/native-distributed-tire`. Статус: дизайн согласован в чате,
ждёт ревью файла. **Исполнение — только после Фазы 0 (спайк) и решения «C»** в
`docs/superpowers/plans/2026-10-11-multi-contact-tire.md`.

## 1. Цель

Дать нативному бэкенду шину, которая опирается на дорогу в нескольких точках сразу (станции по
ободу обжимают камень, как реальная покрышка), чтобы альпийские трассы могли нести реалистичные
камни (`examples/research/alpine_rocky_genspec.toml`) и исследоваться интерактивно на 1× и в
headless-батчах — с той же планкой корректности, что у портированной `compliant_2d`.

Не цель: новая модель шины, её калибровка или валидация (статус `experimental_unvalidated`
зашит в `DistributedWheelMaterial` и сохраняется), standalone CLI (не существует), изменения HUD.

## 2. Решения

| Ветвь | Решение |
|---|---|
| Что портируем | Существующую `distributed_2d_reference`: `station_count` станций по ободу, hinge-density радиальный отклик, brush на станцию, энергоучёт |
| Планка | Побитовый per-state паритет writer'а к Python-референсу, предварительно переписанному в переносимую скалярную форму; ≤ 4 ulp — только с документированием per-component (правило P2) |
| Состояние | На станцию `xi` + `tangent[3]` (`_StationState`); NaN для неактивных; колонки `front.s017.xi`, `front.s017.tangent.0..2`; ширина `station_count × 2 × 4` |
| Диагностика | Списки удалены; скаляры/счётчики: `station_count`, `active_station_count`, `nonsmooth_station_count`, `max_patch_count`, `penetration_m`, `normal_load_n`, `radial_energy_j`, `shear_energy_j`, `brush_loss_j`, `outside_material_load_range`, `outside_material_deflection_range`, `multi_support`, `supports_multiple_contacts = true`, `calibration_status`, `capability_status` |
| Конфиг | `schema` = 2; `tire.distributed` обязательна iff `backend == 'distributed_2d_reference'`; `front/rear.material` для этого backend'а не проецируется (материал — density) |
| C++ | Отдельный `DistributedTireWriter` с гранями `TireWriter`; общие brush/friction/SurfaceMap — разделяемые хелперы; `Stepper` выбирает по backend'у; тёплое ядро без аллокаций, скретч по `station_count` (валидация `16 ≤ n ≤ 512`); `step.cpp` пишет `supports_multiple_contacts` по backend'у |
| Golden | Один: `alpine_rocky` seed 0, 1 с, dt 0.00125, `viewer_physics_distributed.toml`; per-state gate + trajectory-эквивалентность |
| Скорость | Гейт: mean achieved RTF ≥ 0.95 при 1× на `alpine_rocky` при дефолтном `station_count` (из спайка); batch измеряется, не гейтится; перф-контракт расширяется |
| Заморозка (план A) | Пресет `alpine_rocky` + `examples/research/alpine_rocky.toml` + `examples/research/viewer_physics_distributed.toml` |
| Формулы станции | Скалярные `station_density`/`station_response` рядом с векторными; стенды/`fit_density` не меняются |
| Сдвиг референса | До/после: `contact_matrix` + `fidelity_sweep`, diff в аудите; затем старая форма удаляется |
| Планы | A (Python, `quick` + стенды) → B (C++, `native`/`full`) |

## 3. Факты, на которые опирается дизайн

- ADR 0001 / план P2: per-state эквивалентность писателей побитовая (`np.array_equal`), `-ffp-contract=off`,
  Accelerate CBLAS; `numeric_sum.hpp` даёт побитовые `np.sum` (pairwise) и `sum()` (Neumaier);
  `tyre/profile.*` — `np.interp`, `math.hypot`, `ddot`, `dgemv`.
- Тёплое ядро `TireWriter` — без аллокаций (RTSan, `BIKE_NONBLOCKING`), `kMaxSnapshotPatches = 1`
  с комментарием о будущем multi-patch backend'е.
- `status.cpp:119` уже вычисляет невалидность как `multi_support && !supports_multiple_contacts`;
  `step.cpp:1912` пишет `supports_multiple_contacts = false` константой.
- Python-референс (`sim/ride/distributed_tire_forces.py`, 155 строк) векторизован numpy; состояние —
  dict `(side, station) → _BrushState`; следующим шагом читаются только `xi` и `tangent`.
  `physics/distributed_tire.py` (153) — density/response, `flat_load`, `fit_density`;
  `terrain/profile_distance.py` (80) — `SignedProfile` (точный signed distance, `nonsmooth`).
- Потребителей списков `active_station_ids`/`road_segment_ids`/`road_projection_m` в `src/` и
  `tests/` нет.
- Состояние distributed-шины уже сериализуется в `equilibrium_cache._state_arrays`/`restore_state`
  и `initial_state.project_stations` как массивы `state_tire_{sides,xi,tangent,tangent_valid,
  point,point_valid,segment,center,center_valid}` — план A обязан перевести их на урезанную схему.
- `tools/golden_episode.py` снимает `tire_state` через `asdict(state)` по `tire.states` —
  план A переводит захват на `tire.state_dict()`.
- `bike_sim/native/setup.py:376` передаёт `physical.tire.state_dict()` в bootstrap — у
  distributed-референса этого метода нет.
- Стенды: `validation/contact_manifold_rigs.py` (flat/step/two_support/edge_unload/recontact/
  bump/incline), `validation/contact_acceptance.py::contact_matrix`, `validation/fidelity_sweep.py`
  с `--backend distributed_2d_reference --stations`.

## 4. Дизайн — план A: переносимый Python-референс

### 4.1 Состояние

```python
@dataclass
class _StationState:
    xi: float = 0.0
    tangent: np.ndarray | None = None
```

`DistributedTireForceApplier.states: dict[tuple[str, int], _StationState]`. `state_dict()` отдаёт
`{'names': [...], 'row': ndarray}` в порядке `side ∈ ('front','rear')`, станция `0..N-1`, листья
`xi, tangent.0, tangent.1, tangent.2`; неактивная станция — четыре NaN. `set_state(names, row)` —
обратная операция, требует полный канонический набор имён. `equilibrium_cache._state_arrays` для
distributed пишет только `sides, xi, tangent, tangent_valid`; `restore_state` для distributed
принимает ровно этот набор; `initial_state.project_stations` заполняет те же четыре массива.

### 4.2 Скалярные формулы (в `physics/distributed_tire.py`)

```python
def station_density(delta, material) -> tuple[float, float]:   # (elastic_force_per_rad, energy_per_rad)
def station_response(delta, rate, weight, material) -> tuple[float, float]:  # (normal_load_n, energy_j)
```

Та же математика, что `density_response`/`normal_station_response` для одного элемента: суммы по
узлам — последовательные (3 узла < 8 → numpy pairwise = последовательная сумма), `max(0, …)`,
`delta > 0` гейт. Векторные функции и стенды не меняются.

### 4.3 Переносимый `compute_qfrc`

Порядок, который C++ воспроизводит 1:1 (все суммы — явные скалярные циклы в указанном порядке):

1. `center`, `rotation` (`xmat` тела), для каждой станции `s`: `point_s = center + R @ local_s`
   (dgemv-эквивалент как в `profile.cpp::compiled_profile_vertices`); `h_s = profile.height(x_s)`;
   активна iff `z_s < h_s`.
2. Для активных `s` (по возрастанию): `SignedProfile.query` одной точки → `distance, normal2, point2,
   segment, nonsmooth`.
3. `jp_center, jr = point_jacobian(center)`; `center_velocity = jp_center @ qvel` (dgemv).
4. На станцию: `arm = point_s − center`; `J_s = jp_center + skew(jr)…` — покомпонентно:
   `J_s[k] = jp_center[k] + (jr[(k+1)%3] * arm[(k+2)%3] − jr[(k+2)%3] * arm[(k+1)%3])`
   (раскрытое `np.cross(jr.T, arm)`); `v_s = J_s @ qvel` (dgemv, 3 строки); `rate_s = −(v_s · n_s)`
   как `−(v0 n0 + v1 n1 + v2 n2)` в порядке numpy `np.sum(velocities*normals, axis=1)` для n=3
   (последовательно).
5. `normal_load_s, energy_s = station_response(−distance_s, rate_s, w_s, material)`;
   `elastic_density_s = station_density(−distance_s)[0]`;
   `radial_power += max(0, (normal_load_s − w_s · elastic_density_s) · rate_s)`.
6. Brush: `k = tangent_k · w_s`; перенос `xi` через `old.tangent · tangent` (3-членный dot как
   `float(a @ b)` → `ddot`), `losses += max(0, .5 k (xi² − xi_t²))`; `slip = v_s · tangent`
   (ddot); `mu` через `effective_friction`; `_brush_step(...)`; `shear += .5 k xi_new²`.
7. `force_s = normal_load_s · n_s + tangent_force_s · t_s`; `result += J_s.T @ force_s`
   (dgemv transposed, затем поэлементное `+=` в порядке станций).
8. Отключённые станции: `losses += .5 · tangent_k · w · xi_old²` в порядке ключей старого dict.
9. Редукции: `normal = Σ normal_load_s` и `radial_energy = Σ energy_s` — **pairwise `np.sum`**
   (сохраняется как есть: `numeric_sum.hpp` воспроизводит); `max_depth = max(penetration)`;
   `distinct` — двойной цикл по нормалям с `cos(distinct_normal_deg)`; `separate` — по `ids`.
10. Снимок, диагностика (§2), энергии, `last_time_s`.

`probe` (advance=False) — копия с чистыми часами, как сейчас.

### 4.4 Файлы плана A

`sim/ride/distributed_tire_forces.py` (переписать), `physics/distributed_tire.py` (+ скалярные),
`sim/ride/equilibrium_cache.py`, `sim/ride/initial_state.py` (урезанная схема), `tools/golden_episode.py`
(`state_dict`), `terrain/presets.py` + `terrain/alpine.py` (пресет `alpine_rocky`),
`examples/research/alpine_rocky.toml`, `examples/research/viewer_physics_distributed.toml`,
тесты: `tests/reference/test_distributed_tire_reference.py` (новый), `test_alpine_track.py` (пресет),
golden-артефакт в каталоге golden'ов проекта (там же, где savage).

## 5. Дизайн — план B: C++

### 5.1 Файлы

| Файл | Ответственность |
|---|---|
| `native/src/tyre/distance.{hpp,cpp}` | Порт `SignedProfile` (`height` через `np_interp`, `query` одной точки: `searchsorted`, окно по `upper`, проекции, ties `≤ best + 2e-15·max(1,best)`, знак, `nonsmooth`) |
| `native/src/tyre/station.{hpp,cpp}` | `station_angles`, `station_density`, `station_response` — скалярные, порядок операций §4.2 |
| `native/src/writers/tire_common.{hpp,cpp}` | Вынос из `tire.cpp`: `brush_step`, `effective_friction`, `SurfaceMap::at`, `TirePatch`/`TireSnapshot`/`TireDiagnostics` (+ новые поля диагностики с дефолтами) |
| `native/src/writers/distributed_tire.{hpp,cpp}` | `DistributedTireWriter`: ctor (геомы, радиусы, `local_` станций, скретч), `qfrc`/`compute_into`/`try_compute_into`, `set_state`/`state_names`/`state`, `probe_*`, `stored_energy`, `snapshots`/`diagnostics`, `reset`, `restart_clock` |
| `native/src/config_types.hpp`, `config_validation.hpp`, `config.hpp` | `DistributedConfig {station_count, density{knots, stiffness, damping, provenance}, front/rear material domains}`; валидация `16 ≤ station_count ≤ 512`, монотонные узлы, неотрицательные жёсткости |
| `native/src/stepper.cpp/.hpp` | Выбор writer'а по backend'у через `std::variant<TireWriter, DistributedTireWriter>` или общий интерфейс |
| `native/src/runtime/step.cpp` | `supports_multiple_contacts` по backend'у; wire новых диагностических полей |
| `src/bike_sim/native/config.py`, `schema.py` | Проекция `tire.distributed`; `_tire_side` без `TireSpec` для distributed |
| `tests/reference/test_native_distributed_tire.py` | Per-state gate, state round-trip, probe, допуски, RTSan-маркировка |
| `tests/reference/test_golden_episode*.py` | Новый golden `alpine_rocky` в матрице |
| `tests/reference/test_native_runtime_performance_contract.py` | Контракт для distributed-пути |
| `docs/RIDE.md` | Фраза о возможностях native |

### 5.2 Инварианты тёплого ядра

- Скретч (`points_`, `active_`, `normals_`, `tangents_`, `jac_station_` (3×nv), `loads_`,
  `energies_`, патчи снимка ёмкостью `station_count`) — размечается в ctor; `try_compute_into`
  не аллоцирует и `BIKE_NONBLOCKING`.
- Состояние — `std::array`/`std::vector<StationState>` размером `2 × station_count`, активность —
  флаг, не наличие ключа; порядок обхода «отключённых» станций (§4.3 п.8) — по возрастанию
  `(side, station)`, и Python-референс обязан использовать тот же порядок (план A фиксирует
  `sorted(self.states)`).
- Коммит снимков/диагностики — атомарный после обоих колёс, как в `TireWriter`.

### 5.3 Численный контракт

Побитово: все скалярные выражения (`-ffp-contract=off`), `np.interp` → `np_interp`, `a @ b` →
`cblas_ddot`, `(3×nv) @ qvel` → `cblas_dgemv`, `J.T @ f` → `cblas_dgemv(Trans)`, pairwise `np.sum`
→ `numeric::numpy_pairwise_sum`, `np.linalg.norm(v)` → `sqrt(ddot)`, `np.cos(np.radians(x))` →
та же libm. Любое отклонение — per-component ulp-таблица в тесте и в аудите.

### 5.4 Тесты плана B

- Per-state gate: для каждого шага golden `alpine_rocky` — `set_state` + `mj_forward` на
  записанном состоянии → `qfrc` писателя побитово равен записанному компоненту `tires`.
- State round-trip: `state()` → `set_state()` → `state()` побитово; неполный набор имён — ошибка.
- Probe не меняет состояние/часы.
- Trajectory: эпизод native vs golden до измеренного горизонта (как у `compliant_2d`).
- Стенды: `contact_matrix` на native stepper'е (адаптер) — те же границы.
- Перф-контракт; RTSan/ASan/coverage сборки; `check_*` свипы; оба компилятора.

## 6. Ошибки и границы

- `backend == distributed_2d_reference` без `tire.distributed` → ошибка конфига (как `surface_map`).
- `station_count` вне `[16, 512]` → ошибка валидации (Python и C++).
- Неполный `set_state` → ошибка с именем колонки.
- `multi_support` при distributed — информационный; невалидность не наступает (`status.cpp`
  без изменений, `supports_multiple_contacts = true` на проводе).

## 7. Верификация

План A: `bash tools/run_tests.sh quick`, `contact_matrix`, `fidelity_sweep --backend
distributed_2d_reference` до/после (diff в аудите). План B: `bash tools/run_tests.sh native`,
затем `full`; интерактивный гейт RTF ≥ 0.95 при 1× на `alpine_rocky`.

## 8. Вне скоупа

Калибровка/валидация материала (`fit_density` + dataset — отдельный проект); standalone CLI;
HUD-поля; изменение модели шины; `TabulatedTireSpec` в native.
