# Spatial 3D Native — 01. Terrain (W-фаза: W06–W12)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Все задачи — **one-shot write** против frozen headers W00 (`00-foundation.md` §W00.5–W00.6, W00.11). Запрещены configure/build/compile/ctest/запуск/генерация fixtures прогоном. Проверка и acceptance — `09-verification.md` V02 (S06–S10, S12) и V05 (S11).

**Goal:** Написать native terrain asset/codec, bicubic sampling, bounded ray casting, C++-генератор, exact profile extrusion, startup ownership и render projection — целиком, с тестами, без запуска.

**Architecture:** Новые типы живут в `bike::terrain` рядом с существующим `Point = array<double,2>` и planar генератором. Все публичные declarations уже в W00; W06–W12 пишут `.cpp`, тесты, fixtures, CMake-регистрацию.

**Tech Stack:** C++23, `bike::io` JSON/hash, существующий RNG (`native/src/random`), MuJoCo hfield API в W11, `bt` harness + `spatial_tests::*` support.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md` §§5–6; contracts C3, C3.1, C3.2.

## Global Constraints

- Grid по умолчанию binary64, `dx=0.005 m`, `dy=0.010 m`, corridor `y∈[-1.5,1.5] m`. Unit-fixtures маленькие (4×4 … 64×32); 300 m grid создаётся только в V02.
- Любая арифметика размера файла/памяти проверяется до allocation (`__builtin_mul_overflow` или деление-проверка). Выход за domain → `outside`, не продолжение дороги.
- `sample`/`ray_cast` после создания не выделяют память, не блокируют, не меняют состояние. `budget_exhausted ≠ miss`.
- Отображение использует ту же мировую систему координат и datum, но отдельное разрешение; колесо получает силы только из physical surface.
- Регистрация тестов: каждая W добавляет в `native/tests/spatial/CMakeLists.txt` строку `bike_add_spatial_test(<slug> SOURCES <file>.cpp LIBRARIES bike_terrain …)` и production sources — в root `CMakeLists.txt` (библиотека `bike_terrain` или существующий owner terrain-кода; читать текущий список перед правкой).

## Общий формат тестов

Каждый тест — `native/tests/spatial/<slug>.cpp`: `#include "support/migration_test.hpp"`, `#include "support/spatial_fixture.hpp"`, `#include "support/terrain_fixture.hpp"`, нужные production headers; `int main(){ return bt::run([]{ ... }); }`. Допуски берутся из V02-таблицы и пишутся в тест как есть.

---

## W06 — Owned grid и versioned terrain asset (← S06)

**Dependencies:** W00. **Create:** `native/src/terrain/grid3d.cpp` (конструктор/`create`, accessors, `sha256_hex`; `sample`/`ray_cast` здесь только делегируют в `interpolation3d`/`raycast3d`, реализуемые W07/W08 — до них `.cpp` W06 содержит их вызовы, не заглушки), `native/src/terrain/asset3d.cpp`, `native/tests/spatial/support/terrain_fixture.cpp` (реализация `plane_grid/polynomial_grid/asymmetric_4x4/bump_grid`; `polynomial_ray_roots_highprecision` пишет W08), `native/tests/spatial/terrain_codec.cpp`, fixtures `tests/fixtures/spatial3d/terrain/{asym4x4.terrain3d.json, corrupt_cases.json}` (JSON-описание grid для построения в тесте; бинарные байты строятся в тесте через `write_terrain3d`, не хранятся).

**Interfaces:** W00.5 `Grid3d::create`, `read_terrain3d`, `write_terrain3d`, `Terrain3dLoadLimits`, `kTerrain3d*`.

- [ ] `write_terrain3d`: header ровно по таблице C3.1 (112 bytes, LE, offsets 0/4/8/12/16/24/32/40/48/56/64/72/80/88/96/104), затем elevation `[ny][nx]` (`index = iy*nx+ix`) как f64 LE через `std::bit_cast<std::uint64_t>` и побайтовую запись; material `u32[(ny-1)*(nx-1)]`; canonical surface JSON (массив записей `id,name,mu_peak,mu_slide,slip_stiffness_per_load,stribeck_speed_mps[,description]` в author order) и metadata JSON (`generator_version,resolved_spec,seed_decimal,units,height_datum_m,provenance`) через существующий `bike::io` canonical serializer. Никаких `reinterpret_cast` на packed struct, никакого padding.
- [ ] `read_terrain3d`: проверка magic/version/header_bytes/dtype; `nx,ny>=4`; `dx,dy>0` finite; overflow-safe `height_bytes==nx*ny*(dtype==0?8:4)` и `material_bytes==4*(nx-1)*(ny-1)`; total length == 112+sum sections, без trailing bytes; `total <= limits.max_total_bytes`, `nx*ny <= limits.max_nodes`; все elevation finite; surface IDs уникальны; каждый material ID ссылается на существующую surface; `mu_*>=0`, `stribeck_speed_mps>0`; diagnostic с полным путём (`"terrain3d.elevation[17]: non-finite"`, `"terrain3d.header.height_bytes: mismatch"`). f32 dtype читается в f64 без изменения чисел.
- [ ] `Grid3d::create`: те же проверки на уже типизированных входах; `surface_index_by_id_` таблица; `sha256_hex_ = sha256(write_terrain3d(*this))` вычисляется один раз в create (без рекурсии: собрать bytes из уже проверенных полей).
- [ ] `terrain_fixture.cpp`: `asymmetric_4x4()` — elevation `z(ix,iy) = 0.1*ix + 0.37*iy + 0.01*ix*iy` (разные по осям), два surfaces (id 1,7), materials чередуются; `plane_grid(a,b,c,…)`; `polynomial_grid` по коэффициентам `c00,c10,c01,c20,c11,c02,c30,c21,c12,c03`; `bump_grid` — гауссов бугор на плоскости.
- [ ] `terrain_codec.cpp` — обязательные проверки: round-trip `write(read(write(g)))==write(g)` побайтно; `decoded` воспроизводит elevation/materials/surfaces exact; изменение одного `mu_peak` меняет `sha256_hex`; transpose-detector: `read` asymmetric grid и сверка `elevation()[iy*nx+ix]` в трёх несимметричных узлах; corrupt cases — для каждого поля header по одному повреждению (неверный magic, version 2, header_bytes 100, dtype 5, nx=3, dx=0, dx=NaN, height_bytes±8, material_bytes±4, surface_json_bytes>payload, reserved≠0), усечённый payload, trailing byte, duplicate surface id, material ref на несуществующий id, отрицательное трение — каждый через `bt::require(!read_terrain3d(bytes).has_value(), …)` с проверкой подстроки пути в `error()`.
- [ ] CMake: `bike_add_spatial_test(terrain_codec SOURCES terrain_codec.cpp LIBRARIES bike_terrain)`; `terrain_fixture.cpp` добавить в `bike_spatial_support` (нужен link на `bike_terrain`).
- [ ] Коммит `feat(terrain): W06 owned spatial grid asset codec`, `Verification: pending V02`.

```cpp
const auto encoded = write_terrain3d(fixture);
const auto decoded = read_terrain3d(encoded);
bt::require(decoded.has_value(), "valid 4x4 grid decodes");
bt::require(write_terrain3d(*decoded) == encoded, "terrain canonical bytes round-trip exactly");
```

## W07 — Bicubic sampling, нормаль и границы (← S07)

**Dependencies:** W06. **Create:** `native/src/terrain/interpolation3d.cpp`, `native/tests/spatial/terrain_sampling.cpp`. **Modify:** `grid3d.cpp` — тело `Grid3d::sample`.

**Interfaces:** W00.5 `Catmull`, `BicubicPatch`, `bicubic_patch`, `evaluate`, `GridSample3d`.

- [ ] `Catmull::value`: `a0=p1`, `a1=(p2-p0)/2`, `a2=p0-2.5*p1+2*p2-.5*p3`, `a3=(p3-p0)/2+1.5*(p1-p2)`; `value=((a3*t+a2)*t+a1)*t+a0`; `derivative=(3*a3*t+2*a2)*t+a1`.
- [ ] `bicubic_patch(grid,ix,iy)`: 4×4 окрестность `ix-1..ix+2`, `iy-1..iy+2` с ghost-значениями `p[-1]=2*p[0]-p[1]`, `p[n]=2*p[n-1]-p[n-2]` (линейная экстраполяция, не clamp); tensor-product коэффициенты `c[j*4+i]` для `Σ c_ij u^i v^j`; одинаковые правила по x и y.
- [ ] `evaluate(patch,u,v)`: Horner по u затем v; `dz_du`, `dz_dv` аналитически.
- [ ] `Grid3d::sample(x,y)`: NaN/Inf → `invalid_input`; `x<x0 || x>x_max || y<y0 || y>y_max` → `outside` (закрытый domain, `nextafter` снаружи → outside); `ix=min(floor((x-x0)/dx), nx-2)`, `u=(x-x0)/dx-ix` (на правой границе `u=1`); высота из `evaluate`; `dzdx=dz_du/dx`, `dzdy=dz_dv/dy`; `normal_world = normalize({-dzdx,-dzdy,1})`; material cell `(min(ix,nx-2), min(iy,ny-2))` — half-open справа/сверху, последняя граница включена в последнюю cell; `surface_id = surfaces()[material index].id`. Без allocation.
- [ ] `terrain_sampling.cpp`: plane `z=.1x-.2y+.3` на `plane_grid(.1,-.2,.3, 16, 12, .25, .5)` — узлы, interior, все четыре границы, четыре угла: `|height-(…)|<=1e-12`, `normal` vs `{-0.1,0.2,1}/sqrt(1.05)` по компонентам `<=1e-12`, `|norm(normal)-1|<=1e-12`; `nextafter` снаружи каждой границы → `outside`; NaN → `invalid_input`; polynomial fixture (кубический по x и y) — измерить ошибку при `h,h/2,h/4` и `bt::require` монотонного убывания (без претензии на порядок); continuity на внутренних швах: `sample(x_seam-1e-9)` vs `sample(x_seam+1e-9)` высота и нормаль `<=1e-9`; material: граница cell справа/сверху half-open; mirror `z(x,-y)` даёт `normal.y` с противоположным знаком; yaw-поворот аналитической плоскости воспроизводится.
- [ ] CMake: `bike_add_spatial_test(terrain_sampling SOURCES terrain_sampling.cpp LIBRARIES bike_terrain)`.
- [ ] Коммит `feat(terrain): W07 spatial bicubic sampling and analytic normals`, `Verification: pending V02`.

## W08 — Первый ray hit, касания, бюджет (← S08)

**Dependencies:** W07. **Create:** `native/src/terrain/raycast3d.cpp`, `native/tests/spatial/terrain_raycast.cpp`. **Modify:** `grid3d.cpp` — тело `Grid3d::ray_cast` делегирует `ray_cast_grid`; `support/terrain_fixture.cpp` — `polynomial_ray_roots_highprecision` (long double Durand–Kerner, 200 итераций, возвращает действительные корни в `[0,1]` с `|Im|<1e-14`).

**Interfaces:** W00.5 `Poly6`, `line_minus_surface`, `earliest_root_bernstein`, `ray_cast_grid`, `RayBudget`.

- [ ] Входная проверка: finite origin/direction/max_distance, `max_distance>=0`, `|norm(dir)-1|<=1e-9` иначе `invalid_input`. `t` в метрах.
- [ ] Slab clipping XY domain → `[t_enter,t_exit]∩[0,tmax]`; пусто → `outside`. Вертикальный луч (`|dir.x|,|dir.y| < 1e-15`) — отдельная ветка без деления: одна cell, прямая оценка `z(origin.xy)`; `origin.z < z` → `origin_below_surface`; `dir.z<0` и `origin.z>=z` → hit `distance=origin.z-z`; `dir.z>=0` → `miss`.
- [ ] Origin внутри domain: `sample(origin.xy).height > origin.z` → `origin_below_surface`. Origin снаружи: проверка в точке `t_enter` после clipping.
- [ ] DDA по возрастающему t: bound `ceil(|dir.x|*tmax/dx)+ceil(|dir.y|*tmax/dy)+2`, не более `budget.max_cells`; старт на grid line и отрицательные направления обработаны (`tMaxX/tDeltaX` схема Amanatides–Woo); corner crossing посещается один раз.
- [ ] В каждой cell: `Poly6 = line_minus_surface(patch, uv0, duv, z0, dz)` — коэффициенты `z_line(s) - Σ c_ij u(s)^i v(s)^j` при `u=u0+du*s`, `v=v0+dv*s`, `s∈[0,1]` на отрезке луча внутри cell (степень ≤6). `earliest_root_bernstein`: перевод в Bernstein-базис, convex-hull sign test (все коэффициенты строго одного знака → нет корня), иначе bisection — сначала левая половина; fixed-capacity stack `budget.stack_capacity`; остановка при ширине интервала `<= root_interval_m/segment_length`; уточнение 3 итерациями Newton на исходной поверхности; принять при `|residual|<=budget.residual_m`. Касание: отдельно изолировать стационарные точки производного polynomial и проверить residual; малый интервал без доказанной смены знака/residual → не hit. Исчерпание `max_subdivisions`/stack → `budget_exhausted`, не `miss`.
- [ ] Результат: `distance_m = t`, `point_world_m`, `normal_world` из `sample` в точке, `surface_id`.
- [ ] `terrain_raycast.cpp`: plane oracle (аналитический t) для vertical down/up, oblique (yaw 37°, pitch −30°), origin below, `t=0` boundary, луч вдоль grid edge, outside entry; `bump_grid` — луч с двумя пересечениями внутри cell при одинаковых знаках концов (oracle: `polynomial_ray_roots_highprecision` на том же `Poly6`, берётся минимальный корень); exact tangent (луч, касающийся вершины бугра: `dir={1,0,0}`, `origin.z = bump height`) → `ok` с `|distance - t_oracle|<=1e-6`; near-miss `clearance>=2e-8 m` → `miss`; искусственно малый budget (`max_cells=1`, длинный луч) → `budget_exhausted`; `|dir|≠1` → `invalid_input`. Плоскость: `|hit.distance - 1.|<=1e-6` для `ray_cast({0,0,1},{0,0,-1},2)`.
- [ ] CMake: `bike_add_spatial_test(terrain_raycast SOURCES terrain_raycast.cpp LIBRARIES bike_terrain)`.
- [ ] Коммит `feat(terrain): W08 bounded first-hit spatial ray casting`, `Verification: pending V02`.

## W09 — C++-генератор бокового рельефа (← S09)

**Dependencies:** W06–W08. **Create:** `native/src/terrain/spec3d.cpp`, `native/src/terrain/generator3d.cpp`, `native/tests/spatial/terrain_generator.cpp`, `examples/research/alpine_rocky_3d_genspec.toml`, `examples/research/rough_uphill_savage_3d_genspec.toml`, `tests/fixtures/spatial3d/terrain/genspec/{single_rock.toml,single_rut.toml,constant_bank.toml,mirrored.toml}`, `tests/fixtures/spatial3d/terrain/genspec/manifest.json` (`expected_sha256:null`, `status:"pending_verification"` per platform). **Modify:** `native/src/terrain/spec_io.hpp`, `native/src/terrain/trackfile.cpp` — additive dispatch `[terrain3d]` section → `Track3dSpec`.

**Interfaces:** W00.6 `Track3dSpec`, `read_track3d_spec`, `write_track3d_spec`, `quintic_smoothstep`, `generate_grid3d`.

- [ ] `read_track3d_spec`: exact keys `terrain3d.{seed,dx_m,dy_m,half_width_m,lead_in_m,lead_out_m,generator_version,banks[],roughness{sigma_m,ell_m},features[],surfaces[],zones[]}` + ссылка на base track; валидация: `|bank.angle_deg|<=15`, feature kinds ∈ {rock,rut,root}, rock amplitude 0.04–0.15 m, rut 0.02–0.06 m (author ranges), `ell_m>0`, `half_width_m>0`, длина трассы `>= lead_in+lead_out+1 m`; unknown key → `std::invalid_argument` с путём.
- [ ] `generate_grid3d`: `nx=floor(L/dx)+1`, `ny=floor(2*half_width/dy)+1`; base `h(x)` из существующего longitudinal helper (exact nodes, прежний порядок операций); `+ y*tan(bank(x))` с ramp `quintic_smoothstep((x-start)/ramp)`; lead-in `[0,4 m]` и lead-out `[L-5,L]` — bank, roughness и features обнулены (flat lateral); roughness: deterministic white noise из существующего `SeedSequence/NumpyRng` отдельным stream (seed decimal + `"terrain3d/roughness"`), separable Gaussian: `radius=ceil(4*ell/spacing)`, веса `exp(-.5*(k*spacing/ell)^2)` нормированы в фиксированном порядке (сумма от `-radius` до `+radius`), mirror boundary; генерация strips по `y` с halo `radius` строк — не более двух full-size буферов одновременно; features — гладкие finite-support `A*exp(-((x'/rx)^2+(y'/ry)^2))*(1-((x'/rx)^2+(y'/ry)^2))^2` внутри эллипса (0 снаружи) в author coordinates с yaw; rut — отрицательная амплитуда; surface `z(x,y)` однозначна. После сложения — проверка `|slope|<=tan(60°)` и finite; иначе `std::invalid_argument("terrain3d.features[k]: resulting slope exceeds limit")`. Material zones по `MaterialZone3dSpec` (last-wins, half-open); default surface id — первая в `surfaces`. Metadata: resolved spec, seed, `generator_version`, platform numerics identity (из существующего provenance helper), units.
- [ ] `terrain_generator.cpp`: `write(generate(spec))==write(generate(spec))` для `single_rock`; rock расположен в authored (x,y) с глубиной `|A|±1e-9` при нулевой roughness; rut отрицателен; `constant_bank`: `z(x,+y)-z(x,-y)=2*y*tan(angle)` вне lead-in/out и `=0` внутри; `mirrored`: `z_mirror(x,y)==z(x,-y)` побитно при симметричном spec без roughness; слишком короткая трасса → `bt::invalid`; `|bank|>15°` → `bt::invalid`; все elevation finite; material order last-wins на пересечении zones.
- [ ] Genspec examples: оба `.toml` с `[terrain3d]` секцией и ссылкой на существующие `alpine_rocky.toml`/`rough_uphill_savage.toml`.
- [ ] CMake: `bike_add_spatial_test(terrain_generator SOURCES terrain_generator.cpp LIBRARIES bike_terrain)`.
- [ ] Коммит `feat(terrain): W09 spatial track generator in C++`, `Verification: pending V02`.

```cpp
const auto a = write_terrain3d(generate_grid3d(spec));
const auto b = write_terrain3d(generate_grid3d(spec));
bt::require(a == b, "same resolved 3D spec and seed reproduce bytes");
```

## W10 — Exact extrusion и Terrain3dSource (← S10)

**Dependencies:** W06, W07. **Create:** `native/src/terrain/extrusion3d.cpp`, `native/src/terrain/source3d.cpp`, `native/tests/spatial/terrain_extrusion.cpp`. **Modify:** `native/src/terrain/trackfile.cpp` — загрузка legacy `track.toml` в `ProfileExtrusion3d::create` по явному `Terrain3dKind::profile_extrusion`.

**Interfaces:** W00.6 `ProfileExtrusion3d`, `Terrain3dQueryView`, `Terrain3dSource`.

- [ ] `ProfileExtrusion3d::create`: knots строго возрастают по x, finite, `segment_surface_ids.size()==knots.size()-1`, `corridor_half_width>0`; `fingerprint_hex = sha256(canonical{knots_x,knots_z,segment ids,surfaces,corridor,source bytes})` — два профиля с одинаковыми sampled nodes, но разными knots дают разные fingerprint.
- [ ] `sample(x,y)`: `|y|>half_width` или `x` вне `[kx.front(),kx.back()]` → `outside`; иначе `height = height_legacy(x)` — вызов **существующего** piecewise-linear evaluator planar пути с тем же порядком операций (не переписывать формулу); `normal = normalize({-slope_legacy(x),0,1})`, `normal[1]==0.` точно; `surface_id` из segment map.
- [ ] `ray_cast`: XY slab clipping по `[kx.front(),kx.back()]×[-hw,hw]`; `dir.y==0` → **прежний delegated путь** planar ray solver без дополнительной нормировки (bitwise bridge); общий случай: `s=hypot(dir.x,dir.z)`; `s==0` (pure lateral) → если `origin.z` ниже surface → `origin_below_surface`, если `|origin.z-height|<=residual` → hit при `t=0`, иначе `miss` в пределах corridor; `s>0` → `dir_xz/s`, legacy solve в XZ на clipped interval, `t_world=t_xz/s` + origin offset; `outside` при пустом clip.
- [ ] `Terrain3dSource`: конструкторы заполняют variant; `query_view()` — `context=&std::get<…>(source_)`, `sample_fn/ray_fn` — static thunks на `Grid3d::sample/ray_cast` или `ProfileExtrusion3d::*`; `fingerprint_hex()` — `sha256_hex()` grid или `fingerprint_hex()` extrusion; `x_min/x_max/half_width` по kind.
- [ ] `terrain_extrusion.cpp`: загрузить три existing tracks (`flat`, `rough_uphill_savage.toml`, `alpine_rocky.toml` — пути через `bt::fixture`/examples) → для исходных nodes и 1001 межузловой точки (включая knots/edges) `std::bit_cast<std::uint64_t>(e.height_m)==std::bit_cast<std::uint64_t>(legacy_height(x))` при `y=0.7`; `e.normal_world[1]==0.`; `sample(x,+y)==sample(x,-y)` побитно; `|y|>hw` → `outside`; наклонный луч (yaw 30°, pitch −20°) — `t_world` сравнить с аналитикой на flat (`<=1e-9`); pure lateral; outside entry; `dir.y==0` путь побитно равен legacy ray; два профиля с равными sampled nodes/разными knots → разные fingerprint; `query_view().context!=nullptr && sample_fn && ray_fn`; `kind()` соответствует.
- [ ] CMake: `bike_add_spatial_test(terrain_extrusion SOURCES terrain_extrusion.cpp LIBRARIES bike_terrain)`.
- [ ] Коммит `feat(terrain): W10 exact planar profile extrusion and terrain source`, `Verification: pending V02`.

## W11 — Startup ownership и render asset (← S11)

**Dependencies:** W09, W10, W14, W17 (читать их правки `builder.hpp`/`setup_owner.cpp` перед своей; порядок записи — master §5.3). **Create:** `native/src/terrain/render3d.cpp`, `native/tests/spatial/terrain_startup.cpp`. **Modify:** `native/src/model/builder.hpp` (+ `std::shared_ptr<const bike::terrain::Terrain3dSource> terrain3d` в `BuildRequest`, additive; existing `field` не трогать), `native/src/model/terrain.cpp` (новый overload hfield для прямоугольных данных; старый не менять), `native/src/initialization/context.hpp` (+ `std::shared_ptr<const Terrain3dSource> terrain3d; std::optional<RenderGrid3d> render3d;`), `native/src/initialization/setup_owner.cpp` (проверка domain/corridor/backend/fingerprint до live runtime; одна загрузка/генерация asset по `TerrainRequest3d`), `native/src/initialization/research_startup.cpp` (тот же owner).

**Interfaces:** W00.6 `RenderGrid3d`, `make_render_grid`; W00.2 `TerrainRequest3d`.

- [ ] `make_render_grid`: area-averaging downsample physical grid в `nrow×ncol` (extrusion — sampling `height(x)` на ncol точек, тиражирование по nrow); `z_min/z_max` по результату; `normalized = (z - z_min)/(z_max - z_min)` или 0 при `z_max==z_min`; bounds из source; `physical_asset_sha256_hex = source.fingerprint_hex()`.
- [ ] `model/terrain.cpp` новый overload принимает `RenderGrid3d` и заполняет **все** `nrow*ncol` (существующий `heightfield.cpp:25–33` копирует одну строку — не использовать); hfield `size` = `{(x1-x0)/2,(y1-y0)/2, z_max-z_min, base}`, geom translation `{(x0+x1)/2,(y0+y1)/2,z_min}`.
- [ ] Collision masks: при `tires.pneumatic_3d` wheel geoms получают `contype/conaffinity` без hfield; frame/rider crash geoms по topology-контракту W14 (явные masks, не «всё выключено»).
- [ ] `setup_owner.cpp`: по `TerrainRequest3d.kind` — `grid_asset`: `read_terrain3d(bytes, {max_asset_bytes})`; `genspec`: `generate_grid3d(read_track3d_spec(...))`; `profile_extrusion`: `ProfileExtrusion3d::create` из legacy track; `none` при `mode==spatial3d` → `std::invalid_argument("spatial_3d.terrain: required")`. Проверки: `half_width == corridor_half_width_m`, domain содержит start zone, fingerprint записан в setup identity. Один `shared_ptr<const Terrain3dSource>` разделяется BuildRequest/runtime/render.
- [ ] `terrain_startup.cpp`: startup по сохранённому asset и по genspec того же spec → одинаковый `fingerprint_hex`, одинаковые `sample` в 50 точках; `render.physical_asset_sha256_hex == terrain.fingerprint_hex()`; изменение `RenderResolution` не меняет `sample` результаты (`before_samples == after_render_resample_samples`) и physics identity, но меняет render provenance; constant-height grid → normalized все 0; hfield заполнен во всех строках (`nrow` различных строк для asymmetric grid); planar `BuildRequest` без `terrain3d` даёт прежний MJCF (`planar_xml_before == planar_xml_after` — берётся из существующего planar fixture generator); masks: wheel geom не коллидирует с hfield при pneumatic_3d.
- [ ] CMake: `bike_add_spatial_test(terrain_startup SOURCES terrain_startup.cpp LIBRARIES bike_terrain bike_model bike_initialization)` (имена existing targets — читать root CMake).
- [ ] Коммит `feat(startup): W11 own spatial terrain and derive native render fields`, `Verification: pending V05`.

## W12 — Terrain acceptance suite и документация статуса (← S12)

**Dependencies:** W06–W11. **Create:** `native/tests/spatial/terrain_acceptance.cpp`, `native/tests/spatial/terrain_budget_300m.cpp` (label `slow`), `examples/research/terrain3d_manifest.json`, `docs/superpowers/audits/2026-10-22-spatial-3d-native-terrain.md` (скелет evidence-таблицы со статусами `pending_verification`). **Modify:** `docs/TESTING.md` (раздел spatial terrain tests, labels), `docs/RIDE.md` (честный статус: terrain3d implemented-in-source, qualification pending).

- [ ] `terrain_acceptance.cpp`: объединяет малые fixtures: все границы sampler/ray на `plane`, `asymmetric_4x4`, `bump`; extreme slopes `tan(60°)` — no NaN, no OOB, `budget_exhausted_count==0` на зарегистрированных fixtures; подсчёт cells/subdivisions из `RootSearchResult` для статистики (печать в stdout как JSON-строка для V02).
- [ ] `terrain_budget_300m.cpp`: `generate_grid3d` для 300 m×3 m, 5×10 mm → `bt::require(grid.nx()==60001 && grid.ny()==301)`; `18 060 301` samples; измерение peak RSS через существующий platform helper (если есть) иначе `getrusage`; `bt::require(rss_peak_bytes <= 512ull<<20)` для генератора и `elevation bytes == 144 482 408`; immutable owner `<= 256 MiB`; два прогона seeds 0–4 — одинаковые `sha256_hex` внутри процесса; результаты печатаются JSON для manifest. Регистрируется с `LABELS slow`.
- [ ] `terrain3d_manifest.json`: записи reference seeds 0–4 для обоих genspec с `sha256:null`, `platform:null`, `status:"pending_verification"`.
- [ ] CMake: `bike_add_spatial_test(terrain_acceptance SOURCES terrain_acceptance.cpp LIBRARIES bike_terrain)`, `bike_add_spatial_test(terrain_budget_300m SOURCES terrain_budget_300m.cpp LIBRARIES bike_terrain LABELS slow)`.
- [ ] Коммит `test(terrain): W12 terrain acceptance suite and status docs`, `Verification: pending V02`.

```cpp
bt::require(grid.nx() == 60001 && grid.ny() == 301, "declared 300m physical terrain resolution is retained");
bt::require(query_budget_exhausted_count == 0, "registered terrain fixtures fit the explicit ray budget");
```

## Передача

W20 (tyre geometry) использует `Terrain3dQueryView` из W10 и fixtures `terrain_fixture.hpp`; W11 ownership потребляется W25/W31. Acceptance numbers (1e-12 плоскость, 1e-6 ray, memory budgets, within-platform hashes) проверяются только в V02/V05.
