# Spatial 3D Native — 03. Шина и контакт (W-фаза: W20–W28)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Все задачи — **one-shot write** против frozen headers W00 (`00-foundation.md` §W00.7, W00.11 `tyre_fixture.hpp`). `pneumatic3d/state.hpp` полностью задан W00 и здесь **не меняется**. Запрещены configure/build/compile/ctest/запуск. Проверка и acceptance — `09-verification.md` V04 (S20–S27) и V06 (S28).

**Goal:** Написать пространственную pneumatic-шину: геометрию тора, нормальный finite-volume закон, patch lifecycle, fast/detailed brush, writer, тормоз, codec состояния и qualification rig — целиком, без запуска.

**Architecture:** Kernels `bike::pneumatic3d` без MuJoCo; `Pneumatic3dModel` связывает их; `writers/pneumatic_3d_tire.*` связывает модель с engine через существующий owning stepper. Старый `pneumatic` остаётся отдельным planar dispatch.

**Tech Stack:** C++23, `Terrain3dQueryView` (W10), `bike::spatial` frames (W13), `bt` harness, `spatial_tests::tyre_fixture`.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md` §§5–8; contracts C4, C4.1, C7.0.

## Global Constraints

- Все coordinates — SI, vectors в названных frames. Нормальная нагрузка — скаляр вдоль terrain normal. Силы применяются вместе с плечом и couple.
- Force evaluation/probe (`advance=false`) не мутирует committed state; material state продвигается ровно один раз на accepted interval. Нет allocation/исключений/IO в warm core.
- Engine friction/contact шины с hfield отключён для `pneumatic_3d` (W14 masks); контакты рамы/райдера — отдельная policy.
- Численные пороги — в V04/V06 таблицах; W пишет их в тесты как есть. Синтетические коэффициенты помечаются `provenance_status="synthetic"`.
- Buffers предоставляет вызывающий код; `Pneumatic3dModel` выделяет всё в конструкторе.

## Общий формат тестов

`native/tests/spatial/<slug>.cpp`; includes `support/migration_test.hpp`, `support/tyre_fixture.hpp`, `support/terrain_fixture.hpp`, `pneumatic3d/*.hpp`. Регистрация `bike_add_spatial_test(<slug> SOURCES … LIBRARIES bike_pneumatic3d bike_terrain bike_spatial)`; production sources — в новую `bike_pneumatic3d` STATIC в root `CMakeLists.txt` (W20 создаёт target; W21–W27 добавляют свои `.cpp`).

---

## W20 — Тор, контактные элементы и касательные frames (← S20)

**Dependencies:** W00, W10, W13. **Create:** `native/src/pneumatic3d/geometry.cpp`, `native/tests/spatial/support/tyre_fixture.cpp` (`synthetic_spec`, `wheel_at`, `full_torus_area`, `full_torus_volume`; `potential_energy_fd` пишет W21), `native/tests/spatial/tyre_geometry.cpp`. **Modify:** root `CMakeLists.txt` (`add_library(bike_pneumatic3d STATIC …)`), spatial CMake.

**Interfaces:** W00.7 `TorusDims`, `tier_dims`, `TorusGrid3d`, `make_torus_grid`, `WheelKinematics3d`, `FanFrame`, `fan_frame`, `ContactElement3d`, `query_geometry`.

- [ ] `tier_dims`: fast `{64,7,256}`, detailed `{256,15,1024}`.
- [ ] `make_torus_grid(dims, outer_radius, casing_width)`: `r=width/2`, `R=outer−r` (`R>r>0` иначе throw); contact fan θ∈[−75°,75°] (`n_theta_fan` узлов midpoint rule, непериодичен), φ∈[−70°,70°] (`n_phi` midpoint); material grid θ∈[0,2π) периодичен `n_theta_material`; `local_position_fan[it*n_phi+ip] = ((R+r cosφ) sinθ, r sinφ, −(R+r cosφ) cosθ)` в **fan frame** (axle +Y, down −Z, forward +X); `geometric_area_m2 = r*(R+r cosφ)*dθ*dφ`.
- [ ] `fan_frame(kin)`: `axle = kin.axle_world` (unit), `down = normalize(−Z − (−Z·axle)axle)`; `|proj|<1e-6` → `valid=false`; `forward = down × axle`. Spin не вращает fan.
- [ ] `query_geometry(grid, kin, view, out)`: `out.size()==grid.element_count()` иначе `invalid_input`; frame невалиден → `invalid_geometry` с `reason=degenerate_down_projection` в элементах; для каждого элемента: world point `p = hub + F·local`, ray origin `p + n_out*clearance` вдоль внешней нормали тора (`n_out = F·normalize(cosφ sinθ, sinφ, −cosφ cosθ)` при φ-зависимом радиусе — вычислить аналитически), direction `−n_out`, `max_distance = 2*clearance`; hit → `penetration = clearance − distance` (>0 контакт), `point_world`, `normal_world`, `surface_id`, `material_cell = material index` для текущего `wheel_phase_rad` (signed mapping θ_fan + phase → [0,2π)), `projected_area = geometric_area * max(0, dot(n_out, −normal))`; статусы `origin_below_surface/budget_exhausted/invalid_input` → `valid=false` с `reason` (`origin_below_surface`, `terrain_budget_exhausted`, `terrain_query_failed`); `miss/outside` → `valid=false`, `reason=none`, `penetration=0`. Tangents: `longitudinal = normalize(forward − (forward·n)n)`; при `|proj|<1e-9` использовать предыдущий valid frame элемента (хранится в `out[i]` до вызова) только если `|proj|>1e-12`, иначе элемент `valid=false, reason=nonfinite_geometry`; `lateral = normal × longitudinal`. Возврат `ok` если все элементы вычислены (включая невалидные с причиной), `invalid_geometry` если frame невалиден.
- [ ] `tyre_fixture.cpp`: `synthetic_spec(tier)` — `outer_radius={.3675,.3493}`, `casing_width={.061,.061}`, `CarcassSpec3d{pressure_pa=150000, k_area=1e6, maxwell_k=2e5, maxwell_tau=.02, rim_clearance=.04, rim_stiffness=1e8, constant_pressure=true, provenance_status="synthetic"}`, `BrushSpec3d{kx=ky=5e6, relaxation .3/.3, camber_coefficient .8, pneumatic_trail .02, r_eff=outer}`; `wheel_at(hub,yaw,camber,speed,spin)` — quaternion из yaw/camber (roll), `axle_world = R·{0,1,0}`, `linear_velocity = R·{speed,0,0}`; `full_torus_area/volume` — отдельная full-circle quadrature `θ,φ∈[0,2π)` с 1024×256 узлами.
- [ ] `tyre_geometry.cpp`: `|dot(longitudinal, lateral)|<=1e-12` и нормы `1±1e-12` для всех valid элементов на flat/slope/yaw 0/37/90°/camber ±5/15/45°; `|full_torus_area − 4π²Rr| <= 1e-8`, `|full_torus_volume − 2π²Rr²| <= 1e-8`; fan analytic `r*Δθ*(R*Δφ + r*(sinφ1−sinφ0))` vs `Σ geometric_area` (1e-10); contact point/normal vs аналитическая плоскость (ошибка ≤ допуска ray 1e-6); поворот всей установки вокруг Z на 90° сохраняет wheel-local результаты (penetration, projected area) с 1e-12; один камень (`bump_grid`) и разделённые пятна — число valid элементов >0 в двух несмежных θ-зонах; `origin_below_surface` элементы `valid=false` с нужным reason; mirror camber меняет знак `lateral·axle`.
- [ ] Коммит `feat(tyre): W20 spatial torus contact geometry and frames`, `Verification: pending V04`.

```cpp
bt::require(std::abs(dot(patch.longitudinal_world, patch.lateral_world)) <= 1e-12, "patch tangent axes are orthogonal");
bt::require(std::abs(surface_area - 4.0*pi*pi*R*r) <= 1e-8, "torus quadrature has the geometric area");
```

## W21 — Нормальная finite-volume law, carcass и давление (← S21)

**Dependencies:** W20. **Create:** `native/src/pneumatic3d/carcass.cpp`, `native/tests/spatial/tyre_carcass.cpp`, `docs/research/spatial-tyre-normal-law.md` (вывод закона, формулы, статус `derivation written; numerical validation pending V04`). **Modify:** `tyre_fixture.cpp` (+`potential_energy_fd`). Читать, не менять `native/src/pneumatic/carcass.cpp:425–470`.

**Interfaces:** W00.7 `CarcassSpec3d`, `NormalElement3d`, `update_carcass`, `CarcassState3d`.

- [ ] Закон (документ + код): `U_p = p_eff * V_displaced` при `constant_pressure`; иначе `U_p = ∫ p_eff(V) dV` с изотермической моделью `p(V)=p0*V0/V` и `initial_volume_m3`. Carcass potential `U_c = ½ k_area ∫ δ_n² dA_contact`. На элементе: `V_displaced_i = δ_i * A_i` (geometric area), `F_pressure_i = p_eff * A_i * H(δ_i)`, `F_elastic_i = k_area * δ_i * A_i`, `F_rim_i = rim_stiffness * max(0, δ_i − rim_clearance) * A_i`; энергия `stored = ½ k_area δ² A + rim term`, `pressure work` отдельно. Силы — производные потенциала по rigid displacement; footprint меняется с rotation.
- [ ] Maxwell primitive: `q_new = e^(−dt/τ) q_old + k τ (1 − e^(−dt/τ)) δ̇` через `std::expm1`; `F_rate = q * A`; losses `= (F_rate * δ̇ − dE_branch/dt) dt ≥ 0` до clamp; stiffness/τ > 0. Advection: при `advance` состояние material cell сдвигается по θ на `Δphase` (signed), interpolation по соседям, фиксированный порядок.
- [ ] `update_carcass(spec, elements, dt, advance, state, out)`: `out.size()==elements.size()`; probe (`advance=false` или `dt==0`) не меняет `state` (вычисления в локальных временных на стеке: без allocation — использовать поля `out` как scratch); `advance=true` коммитит `deflection_m`, `maxwell_q`, `previous_penetration`, `active`, `material_phase`, energy totals. Невалидный элемент → `active=false`, все силы 0 (не максимум).
- [ ] `tyre_carcass.cpp`: synthetic cell `p=0, A=.01, δ=.001, k=1e6` → `|force−10|<=1e-10`, `|stored−0.005|<=1e-12`; force–potential derivative: `potential_energy_fd` (central difference h=1e-6 по вертикальному смещению) vs `Σ F_n` — относительная ошибка `<=1e-5` вне смены contact topology; compression/release цикл не создаёт энергию сверх `1e-9 J`; dt-halving Maxwell ramp сходится (разница `<=` C·dt); 2× θ/φ refinement меняет суммарную силу `<=2%`; material rotation/reversal переносит `maxwell_q` на соседние cells; probe bytes прежние (`std::memcmp` сериализованного state до/после); flat/no-camber compatibility: нагрузка/прогиб vs старый planar закон (вызов существующего `pneumatic::carcass` на тех же параметрах) `<=2%` на сетке `Fz∈{100,300,600,900} N`, `p∈{1,1.5,2} bar`.
- [ ] Коммит `feat(tyre): W21 energy-accounted spatial carcass model`, `Verification: pending V04`.

## W22 — Connected patches, split/merge и история (← S22)

**Dependencies:** W20, W21. **Create:** `native/src/pneumatic3d/patches.cpp`, `native/tests/spatial/tyre_patches.cpp`.

**Interfaces:** W00.7 `Patch3d`, `assign_patches`, `PatchState3d`.

- [ ] Adjacency по **непериодической** fan-сетке (θ-соседи кроме крайних, φ-соседи); connected components — итеративный BFS, очередь которого размещается в `state.element_to_patch_slot` (размер `element_count`, выделен cold в `Pneumatic3dModel`) и перезаписывается результатом после обхода; новые component labels пишутся в `out[k].elements` через `patch_element_index_` модели (span передаётся как `out[k].elements` до вызова, заполненный индексами). Разные `surface_id` не объединяются (граница материала разрывает компонент); порог normal discontinuity `dot(n_i,n_j) < cos(30°)` разрывает.
- [ ] Идентичность: overlap weight `Σ load_i` по общим material cells между старым и новым компонентом; largest-overlap сохраняет ID (tie-break — минимальный element index); остальные получают `next_id++`; merge — survivor тем же правилом. Capacity: `state.capacity` (из конструктора модели, default = доказанный максимум `ceil(element_count/2)`); переполнение → `invalid_geometry` без изменения `state` (все изменения на staging полях `out`, коммит в конце).
- [ ] Patch wrench: centroid load-weighted; `force = Σ F_i`, `couple = Σ (p_i − centroid) × F_i` (residual couple); `area = Σ A_i`; normal = load-weighted normalized; tangents через `longitudinal/lateral` среднего направления.
- [ ] Airborne cells: `active=false`; история остаётся на material cells (carcass), patch-level relaxation — в brush state (W23) с consistent IDs.
- [ ] `tyre_patches.cpp`: moving double bump 1→2→1 — IDs сохраняются/назначаются по правилу; `split.total_force_world == before.total_force_world` побитно при identity-only reindex; `|split.total_stored_j − before.total_stored_j| <= 1e-12`; material boundary → два patch; ring wrap через material phase не ломает fan adjacency; overflow → `state_bytes_after == state_bytes_before`; reversed element enumeration oracle даёт те же IDs.
- [ ] Коммит `feat(tyre): W22 spatial patch identity and material history`, `Verification: pending V04`.

## W23 — Fast brush: combined slip, camber и passivity (← S23)

**Dependencies:** W21, W22. **Create:** `native/src/pneumatic3d/brush.cpp`, `native/tests/spatial/tyre_brush_fast.cpp`, `docs/research/spatial-tyre-tangential-law.md` (fast tier раздел; detailed дописывает W24).

**Interfaces:** W00.7 `BrushSpec3d`, `PatchWrench`, `update_fast_brush`, `BrushState3d`.

- [ ] Slip: скорость материала относительно неподвижной дороги в patch frame. `point_velocity_world_mps[k]` — скорость материальной точки обода в центроиде patch (уже включает spin, вычисляется writer'ом через `point_velocity_world` колеса); `vx = v_point·longitudinal`, `vy = v_point·lateral`; `v_roll = |v_hub·longitudinal|`. Соглашение span: `point_velocity_world_mps[0..count)` — centroid velocities, `point_velocity_world_mps[count]` — hub velocity (span имеет длину `count+1`); `Pneumatic3dModel` формирует его так, `brush.hpp` документирует комментарием.
- [ ] Elastic brush: `u̇ = −v_rel_t − (|v_roll|/ℓ) u` (convective decay), интегрирование implicit Euler: `u_trial = (u + dt*(−v))/(1 + dt*|v_roll|/ℓ)`; trial `f = −K u_trial` (`K = k * area`); ellipse `s = min(1, 1/sqrt((fx/(μx Fz))² + (fy/(μy Fz))²))`; `Fz==0` → `u=0, f=0`; return mapping: `u_new = s*u_trial`, `f = −K u_new`, `loss = ½ K (|u_trial|² − |u_new|²) ≥ 0` плюс `convective = (|v_roll|/ℓ) K |u|² dt`.
- [ ] Camber: `bγ = r_eff * Cγ * sin(γ) / (Ky * ℓy)`, lateral source `+bγ * ω_spin` добавляется к `vy` со знаком; conjugate spin couple `bγ * Fy` входит в `couple·axle` и power. Mirror меняет знак lateral/camber.
- [ ] Pneumatic trail: точка приложения `point = centroid + trail * longitudinal * sign(v_roll)`; `Mz = fy * trail` (один раз); `mz_nm` записывается.
- [ ] `update_fast_brush`: per patch; `advance=false` → `state` неизменен (вычисления в локальных переменных; `BrushState3d` имеет `ux_m/uy_m` по patch slot). `camber_state` переносится.
- [ ] `tyre_brush_fast.cpp`: ellipse `<= 1+1e-12` при combined braking+cornering+camber; `loss_j >= −1e-12`; zero speed/reverse — конечные силы; mirror error `<=1e-10`; `μ=0`, `Fz=0` → нулевые силы; energy closure `|ΔU + loss_sliding + loss_convective − W_force| <= 1e-6*scale + 1e-9` на управляемом цикле stop/start/reverse; steady pure slip cornering stiffness ≈ `K ℓ`-производная.
- [ ] Коммит `feat(tyre): W23 passive spatial fast brush and combined grip`, `Verification: pending V04`.

```cpp
const double ellipse = std::pow(out.fx_n/(mux*fz),2) + std::pow(out.fy_n/(muy*fz),2);
bt::require(ellipse <= 1.0 + 1e-12, "camber and braking share the friction ellipse");
bt::require(out.loss_j >= -1e-12, "return mapping does not generate dissipative energy");
```

## W24 — Detailed bristles и сопоставление tiers (← S24)

**Dependencies:** W20–W23. **Create:** `native/src/pneumatic3d/bristles.cpp`, `native/tests/spatial/tyre_brush_detailed.cpp`. **Modify:** `docs/research/spatial-tyre-tangential-law.md` (+detailed раздел и таблица fast-vs-detailed наблюдаемых, значения `pending V04`).

**Interfaces:** W00.7 `update_detailed_brush`; `BrushState3d::bristle_*` по material cell.

- [ ] На каждом material element: 2D displacement `(bux,buy)` и энергия в локальном tangent frame; при смене frame (поворот) — перенос вектора через старый/новый базис (`u_new = B_newᵀ B_old u_old`), а не копирование чисел.
- [ ] Advection по full-circle material grid в направлении signed spin: `Δθ = ω_spin dt`, interpolation weights фиксированы, порядок обхода источников фиксирован; fan читает соответствующую область; `dt→0` нет продвижения; выход из контакта — release `loss += ½ K|u|²`, `u=0`.
- [ ] Local load из `NormalElement3d.normal_load_n`, локальное μ из `surface_id`, return mapping каждой bristle; `Fx,Fy,Mz` интеграция в фиксированном material order; `element_wrench_world[i]` на элемент (point = contact point).
- [ ] `tyre_brush_detailed.cpp`: brush conveyor analytic case (steady slip → линейная u(θ) до насыщения), forward/reverse, rotating frame: `|rotated.energy − reference.energy| <= 1e-12`, `norm(rotated.force_in_reference_frame − reference.force) <= 1e-9`; contact exit/reentry; refinement: удвоение `n_theta_material` уменьшает ошибку плато хотя бы вдвое; fast vs detailed steady curves на одинаковой постановке — печать разницы (acceptance ≤10% проверяется V04 как measured row).
- [ ] Коммит `feat(tyre): W24 material-resolved spatial brush tier`, `Verification: pending V04`.

## W25 — Writer, generalized forces и один владелец контакта (← S25)

**Dependencies:** W11, W13–W17, W20–W24. **Create:** `native/src/pneumatic3d/model.cpp`, `native/src/writers/pneumatic_3d_tire.cpp`, `native/tests/spatial/tyre_writer.cpp`. **Modify:** `native/src/stepper.{hpp,cpp}`, `runtime/step.{hpp,cpp}`, `model/physical_topology.cpp`, `writers/writer_types.hpp` (append-only telemetry lanes), `runtime/capabilities.cpp` (row `tyre:pneumatic_3d` → `implemented=true` только для constrained4 с `qualification_pending=true`).

**Interfaces:** W00.7 `Pneumatic3dModel`, `Pneumatic3dTireWriter`.

- [ ] `Pneumatic3dModel` ctor: `tier_dims`, `make_torus_grid` на колесо, sizing `elements_/normals_/patches_/wrench_/patch_element_index_`, `capacity = spec.patch_capacity ? : ceil(element_count/2)`, `committed_ = staging_ = zero state с identity`. `evaluate(kins, point_velocity, dt, advance)`: на колесо — `query_geometry → update_carcass → assign_patches → update_fast_brush | update_detailed_brush`; все на `staging_`; при `advance` и всех `ok` — `committed_ = staging_` (копия фиксированного размера, без allocation), `accepted_interval++`; при `advance` того же interval повторно → `invalid_input`, `reason=duplicate_advance`; любой `!ok` → `committed_` не трогается, `reason_` установлен. `restore(state)`: identity mismatch → `invalid_input`; иначе `committed_=state`, `staging_=state`.
- [ ] `Pneumatic3dTireWriter` ctor: bind по `Topology.wheels`, проверить masks (`geom_contype` колёс не пересекаются с hfield → иначе throw `engine_contact_enabled`), sizing `jacp_/jacr_` `3*nv`, `view = terrain.query_view()` один раз. `try_compute_into(data, dt, advance, qfrc)`: `WheelKinematics3d` из `body_pose` колеса/хаба и `axis_rate`; patch point velocities через `point_velocity_world`; `kernels_.evaluate`; для каждого valid wrench `mj_jac(model,data,jacp,jacr,point,wheel_body)` → `qfrc += Jpᵀ F + Jrᵀ M`; `axle_moment_[w] = Σ dot(axle, cross(point−hub,F)+M)`; телеметрия в append-only lanes.
- [ ] `stepper.cpp/step.cpp`: dispatch `backend=="pneumatic_3d"` → этот writer в существующем `TireWriterView`; probe/advance semantics прокинуты; все readers видят один accepted generation.
- [ ] `tyre_writer.cpp`: virtual work `|dot(qfrc,qvel) − Σ(F·v_point + M·ω)| <= 1e-8` на произвольных `qvel`, yaw/roll/steer, с rider моделью; `state_before_probe == writer.state_bytes()` после probe; второй advance того же interval → status ≠ ok и state неизменен; construction с неправильной mask → `bt::invalid`; warm allocations = 0 (использовать существующий `allocation_faults.hpp` hook: `bt::require(allocations_during_compute==0)`); planar golden regression — существующий planar fixture через `bt::golden_f64` неизменен.
- [ ] Коммит `feat(runtime): W25 spatial tyre writer through owned stepper`, `Verification: pending V04`.

```cpp
bt::require(std::abs(dot(qfrc, qvel)-wrench_power_w) <= 1e-8, "generalized force preserves virtual work");
bt::require(state_before_probe == writer.state_bytes(), "force probe is observational");
```

## W26 — Статика тормоза, contact anchors и общий grip (← S26)

**Dependencies:** W23–W25. **Create:** `native/src/spatial/brake_state.cpp`, `native/tests/spatial/brake_static_3d.cpp`. **Modify:** `native/src/runtime/static_brake.{hpp,cpp}` (+ `capture_spatial_brake() const -> bike::spatial::BrakeState`, `restore_spatial_brake(const BrakeState&)`), `runtime/step.{hpp,cpp}`, `runtime/accounting.{hpp,cpp}` — только spatial-selected path.

- [ ] Сверить текущий `static_brake.cpp` с dirty-снимком (читать; расхождения — в `contract-deviations.md`). Для 3D latch привязан к wheel angle относительно hub (`unwrapped_angle_rad` накапливается через `axis_rate*dt`), не к world.
- [ ] Общий cap: `|τ_frictionloss + τ_hold| <= ceiling*demand`; engaged hold: `τ_hold = −k(θ−θ_latch) − c θ̇`, остаток budget → frictionloss; release — весь budget снова friction; знаки для обоих направлений вращения.
- [ ] Ground stiction принадлежит brush (W23) — anchor на material contact; никакого world spring от root к точке; на bank/slope brake и ground traction делят ellipse.
- [ ] Energy: `0.5*k*(θ−θ_latch)²` отдельно, damping/friction heat отдельно, release work отдельно; tyre loss не содержит brake-hinge work.
- [ ] `brake_state.cpp`: codecs (ключи см. W17) + `canonical_released_brake`.
- [ ] `brake_static_3d.cpp` на constrained rig (блокирует опрокидывание): slope 5°, μ=.8 → drift `<=1 mm/10 s` при объявленном dt; `tan(slope)>μ` → скольжение независимо от ceiling; поворот установки вокруг Z на 0/90° меняет только world представление (1e-12 локально); stop/start/reverse; release без скачка world force (`|ΔF|<=` одного шага friction change); bank с lateral gravity; `|τ_friction+τ_hold| <= ceiling*demand+1e-12`; `restarted.latch == uninterrupted.latch` после capture/restore; energy residual `<=0.1% + 1e-6 J`.
- [ ] Коммит `fix(spatial): W26 internal brake bite separated from ground traction`, `Verification: pending V04`.

## W27 — Версионированный tyre-state codec и atomic restore (← S27)

**Dependencies:** W17, W21–W26. **Create:** `native/src/pneumatic3d/state.cpp`, `native/tests/spatial/tyre_state.cpp`. **Modify:** `model.cpp` (restore staging), `native/src/runtime/bootstrap_io.cpp` (вызов `decode_tyre_state` в ветке bootstrap2 `tire`), `runtime/step.{hpp,cpp}`, `initialization/capture.cpp`.

**Interfaces:** W00.7 `encode_tyre_state`, `decode_tyre_state`, `TyreStateIdentity3d`, `tyre_identity_to_wire/from_wire`, `base64_canonical/base64_decode_strict`.

- [ ] Binary layout `encode_tyre_state` (LE): header `magic "BKS3"`, `u32 version=1`, `u32 tier`, `u32 n_theta_fan,n_phi,n_theta_material`, `u32 patch_capacity`, четыре hex-строки identity как `u32 len + bytes`; затем на колесо: carcass (`deflection[n_mat]`, `maxwell_q[n_mat]`, `previous_penetration[n_mat]`, `active[n_mat] u8`, `material_phase`, `stored`, `dissipated`), patches (`ids[capacity] u64`, `element_to_patch_slot[n_fan] u32`, `slot_used[capacity] u8`, `next_id u64`, `capacity u32`), brush (`ux[capacity],uy[capacity],camber_state[capacity]`, `bristle_ux[n_mat],bristle_uy[n_mat],bristle_energy[n_mat]`, три energy totals); `i64 accepted_interval`; без padding/pointers. Inactive slots canonical zero, NaN запрещён.
- [ ] `decode_tyre_state(bytes, identity)`: version → sizes/overflow → identity equality (все поля) → finite → ID uniqueness/`next_id > max id` → ranges (`active ∈{0,1}`, `slot < capacity`) → energy consistency (`stored>=0`); только полностью проверенный temp публикуется (возвращается); иначе `unexpected(error с путём)`.
- [ ] `tyre_identity_to_wire/from_wire`: exact keys `schema,tier,dims{n_theta_fan,n_phi,n_theta_material},patch_capacity,topology_fingerprint,config_fingerprint,terrain_sha256,calibration_sha256`; `base64_canonical` — padded, без whitespace; `base64_decode_strict` отвергает whitespace/неканонический padding.
- [ ] `bootstrap_io.cpp`: `tire{pneumatic3d_state=1,identity,encoding="base64",payload}` → decode → `Pneumatic3dTireWriter::restore`; отказ не меняет live writer.
- [ ] `tyre_state.cpp`: `encode(decode(bytes)) == bytes`; continuation fixture: ride на constrained rig mid-corner + braking после split/merge → capture, затем 1000 accepted steps continuous vs restored на том же binary — `resumed.accepted_states == continuous.accepted_states` (побайтно все `state_bytes()`); corrupt byte / tier mismatch / changed terrain sha / truncated last section → `!decode.has_value()` и writer state неизменен; raw render snapshot (integration only) как restore → reject.
- [ ] Коммит `feat(state): W27 transactional spatial tyre state codec`, `Verification: pending V04`.

```cpp
bt::require(encode_tyre_state(*decode_tyre_state(bytes, identity)) == bytes, "canonical tyre state round-trips exactly");
bt::require(resumed.accepted_states == continuous.accepted_states, "same-build restart retains full material history");
```

## W28 — Qualification rig и coefficient profile registry (← S28)

**Dependencies:** W20–W27. **Create:** `native/tests/spatial/tyre_qualification.cpp` (label `slow`), `native/tests/spatial/data/tyre_profiles.json`, `docs/research/spatial-tyre-qualification.md` (методика, матрица, targets; результаты `pending V06`). **Modify:** `native/src/configuration/{physical_config.hpp,schema.cpp,validate.cpp}` — реестр `lateral_coefficients` профилей с `status ∈ {measured,calibrated,inferred,provisional}` и digest.

- [ ] `tyre_profiles.json`: профиль `mtb-magic-mary-29x2.4-front` и `mtb-hans-dampf-27.5x2.4-rear` (identity из существующего config snapshot), все семейства коэффициентов со `status:"provisional"`, `source_measurements:0`, `digest:null`; dataset metadata поля (`load, pressure, casing, surface, speed, temperature, camber/slip domain`) — `null`.
- [ ] Матрица в тесте: `Fz ∈{100,300,600,900} N`, `p∈{1,1.5,2} bar`, `v∈{0,.1,1,3,8} m/s`, `κ∈{0,±.02,±.1,±.3}`, `α∈{0,±1,±3,±6}°`, `γ∈{0,±5,±15,±30}°`; вне measured domain → `exploratory`.
- [ ] Fit/withheld: тест реализует разделение measured rows (из `tyre-lateral.json` W03) на fit/withheld и вычисляет RMS; при `source_measurements==0` соответствующие `bt::require` обёрнуты в проверку `profile.status != "measured"` (т.е. тест подтверждает, что провenance не повышена) — не skip, а другая обязательная проверка.
- [ ] Отдельно от fit: conservation, passivity, mirror/yaw invariance, μ→0, pressure/load boundary, step-halving, grid refinement, split/merge, start/stop/reverse — все на `Pneumatic3dModel` без MuJoCo; `report.invalid_cases==0`.
- [ ] Коммит `test(spatial): W28 tyre qualification rig and profile registry`, `Verification: pending V06`.

```cpp
bt::require(report.invalid_cases == 0, "all qualified samples remain valid");
bt::require(profile.status != "measured" || report.source_measurements > 0, "provenance cannot promote invented coefficients");
```

## Передача

W25 передаёт writer runtime-владельцу (W29–W36); W27 — state schema recording/replay (W42–W44); W28 — capability и calibration domain qualification (V06/V10). Числовые acceptance (1e-12 ортонормированность, 1e-5 force–potential, 2% planar compatibility, 1e-8 virtual work, 1 mm/10 s drift) проверяются только в V04/V06.
