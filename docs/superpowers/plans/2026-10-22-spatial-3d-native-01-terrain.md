# Spatial 3D Native — 01. Terrain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Если эти skills недоступны, сохраняйте тот же порядок реализации, независимого ревью и доказательной приёмки.

**Goal:** Получать из native-конфигурации неизменяемую физическую поверхность `z(x,y)`, корректно пересекать её лучами шины и передавать тот же terrain asset в startup, recording и viewer.

**Architecture:** Новые типы живут в существующем `bike::terrain`; текущий `Point = array<double,2>` и planar генератор сохраняются. Физический `Grid3d`, точная planar extrusion и отдельное представление для отображения имеют явные владельцы и fingerprint. Генерация и загрузка выполняются до начала realtime-пути.

**Tech Stack:** C++23, существующие native TOML/JSON, RNG и hash-модули, MuJoCo из квалифицированного manifest текущей C++-ветки; CTest. Python не требуется ни для новых fixtures, ни для генерации.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`, §§5–6; `2026-10-22-spatial-3d-native-contracts.md`. Исторический P2 сохраняет происхождение требований; индекс и глобальные gates — `2026-10-22-spatial-3d-native-master.md`.

## Global Constraints

- В этой задаче написания плана ничего из проекта не запускалось. Все команды ниже предназначены для последующей реализации на разрешённой квалифицированной машине.
- `G0-source` фиксирует SHA, dirty-снимок, provenance зависимостей и набор действующих planar golden. `G1-core` разрешает интеграционные C++-задачи. G0–G6 — upstream readiness PFC по master; в текущей работе они не проверены, spatial-задачи их потребляют, но не закрывают своими tests.
- Grid по умолчанию binary64, `dx=0.005 m`, `dy=0.010 m`, прямой corridor `y∈[-1.5,1.5] m`. Короткие аналитические fixtures используют меньшие размеры; 300 m grid не создаётся каждым unit test.
- Любая арифметика размера файла и памяти проверяется до выделения. Выход физического запроса за domain не превращается в искусственное продолжение дороги с краевым height.
- `sample`/`ray_cast` после создания не выделяют память, не блокируют, не меняют terrain или глобальное состояние. Ошибки имеют различимые статусы; `budget_exhausted` не равен `miss`.
- Отображение использует ту же мировую систему координат и datum, но отдельное разрешение. Колесо получает силы только из physical surface; collision policy для рамы/райдера задаётся явно в topology-фазе.
- Старые документы P2, где generator остаётся Python, и P1 с трёхстрочным bicubic «точным» bridge больше не являются инструкцией к реализации.

## Общие файлы и команды будущей проверки

Тесты ниже создаются в `native/tests/spatial/`. Использовать существующие `native/tests/support/migration_test.hpp` и `model_fixture.hpp`; helper-функции конкретного fixture объявлять в том же тестовом `.cpp` или в `native/tests/spatial/terrain_fixture.hpp`. `bt::require` уже есть в проекте. Основной C++ graph — корневой `CMakeLists.txt`; registry spatial tests — новый `native/tests/spatial/CMakeLists.txt`. Каждый slug даёт executable `bike_spatial_<slug>` и CTest `spatial.<slug>`; source для analysis sweeps добавляется тем же коммитом. Transitional binding graph остаётся только для собственных неизменённых gates PFC.

```bash
cmake --build --preset release --target bike_spatial_terrain_codec
ctest --test-dir build/native-release --no-tests=error -R '^spatial\.terrain_codec$' --output-on-failure
```

Это пример для S06; в остальных задачах slug указан точно. Preset/build directory сверяются с manifest квалифицированной PFC-базы, и любое отличие документируется. Никакого configure/build в текущем задании не требуется.

## S06. Owned grid и версионированный terrain asset

**Depends on:** S01–S05; для автономного codec достаточно замороженных contracts и native IO-зависимостей, общий rollout не нужен.

**Files:** Create `native/src/terrain/grid3d.hpp`, `native/src/terrain/grid3d.cpp`, `native/src/terrain/asset3d.hpp`, `native/src/terrain/asset3d.cpp`, `native/tests/spatial/terrain_codec.cpp`, `native/tests/spatial/terrain_fixture.hpp`; Modify root `CMakeLists.txt` и `native/tests/spatial/CMakeLists.txt` из S04. Не переопределять `terrain::Point` в `types.hpp`.

**Consumes:** Канонические frames, `GridSample3d`, `RayHit3d`, `QueryStatus3d` из contracts; существующий native hash/IO.

**Produces:** `read_terrain3d(std::span<const std::byte>) -> std::expected<Grid3d,std::string>` и `write_terrain3d(const Grid3d&) -> std::vector<std::byte>`. `Grid3d` владеет elevation, materials и metadata; публичные spans read-only. Все значения конечны, `dx,dy>0`, `nx,ny>=4` для bicubic. Asset fingerprint включает canonical bytes, generator version, resolved spec и seed.

- [ ] Использовать canonical binary layout из `2026-10-22-spatial-3d-native-contracts.md`, раздел terrain binary header: magic `BKT3`, initial version 1, header 112 bytes, little-endian fixed-width integers; explicit JSON surface/metadata sections; elevation `index=iy*nx+ix`; `uint32_t` material ID на physical cells. Размеры, origin/spacing/datum, dtype и длины sections не описывать второй несовместимой структурой. У каждой переменной таблицы есть count/length, неизвестные trailing bytes запрещены.
- [ ] До allocation проверить overflow `nx*ny*sizeof(T)`, верхний budget, несовпадение length, дубли ID, несуществующий material reference, NaN/Inf, отрицательное трение и invalid domain. Diagnostic содержит полный путь, например `terrain3d.elevation[17]`.
- [ ] Создать 4×4 fixture с разными x/y значениями: он обнаруживает transpose/row order, который симметричная плоскость не обнаружит. Добавить corrupt cases каждого header field и усечённого payload.
- [ ] Реализовать codec без `reinterpret_cast` на packed struct и зависимости от host endian/alignment. Не записывать padding C++-структур. Owned immutable publication происходит только после полной проверки.
- [ ] Round-trip сохраняет все elevation bytes и material IDs; изменение одного material coefficient изменяет fingerprint. f32 reader допускается как формат, но генератор S09 выдаёт f64 до отдельной квалификации quantization.
- [ ] Выполнить будущий `spatial.terrain_codec`; ревью границ allocation, фиксированный коммит `feat(terrain): add owned spatial grid asset codec`.

```cpp
const auto encoded = write_terrain3d(fixture);
const auto decoded = read_terrain3d(encoded);
bt::require(decoded.has_value(), "valid 4x4 grid decodes");
bt::require(write_terrain3d(*decoded) == encoded,
            "terrain canonical bytes round-trip exactly");
```

**Acceptance:** Codec round-trip побитовый; все повреждённые fixtures отклонены до публикации. Этот тест не доказывает физическую квалификацию поверхности и не закрывает upstream `G1-core`.

## S07. Bicubic sampling, нормаль и границы

**Depends on:** S06.

**Files:** Modify `native/src/terrain/grid3d.{hpp,cpp}`; Create `native/src/terrain/interpolation3d.hpp`, `native/src/terrain/interpolation3d.cpp`, `native/tests/spatial/terrain_sampling.cpp`.

**Consumes:** Неизменяемый `Grid3d`, SI coordinates и единый datum.

**Produces:** `Grid3d::sample(double x_m,double y_m) const noexcept -> GridSample3d`; normal получается из производных той же поверхности, `normalize({-dzdx,-dzdy,1})`. `inside`/status и material выбираются независимо от extrapolation, используемой для ghost nodes.

- [ ] Выписать формулу Catmull–Rom с tension 0.5 в `interpolation3d.hpp`: для `p0…p3` коэффициенты `a0=p1`, `a1=(p2-p0)/2`, `a2=p0-2.5*p1+2*p2-.5*p3`, `a3=(p3-p0)/2+1.5*(p1-p2)`; производная `a1+2*a2*t+3*a3*t²`. Двумерный tensor-product использует одинаковые правила по x/y.
- [ ] Для border выбрать линейные ghost values `p[-1]=2*p[0]-p[1]`, `p[n]=2*p[n-1]-p[n-2]`. Не использовать clamped индексы, одновременно обещая точную наклонную плоскость на краю. Вне закрытого domain вернуть `outside`, без физического контакта с продолжением ghost surface.
- [ ] На plane `z=.1x-.2y+.3` проверять узлы, interior, все границы и углы. Нормаль сравнивать с `{-0.1,0.2,1}/sqrt(1.05)`. Нулевая ширина, NaN coordinates и `nextafter` по обе стороны границы имеют отдельные случаи.
- [ ] Добавить явные polynomial fixtures, finite-difference normal check и continuity по внутренним швам. Не заявлять O(h⁴) для Catmull–Rom с finite-difference derivatives; измерять реальную ошибку выбранного polynomial при h,h/2,h/4.
- [ ] Политика material rectangles/cells — half-open справа/сверху, последняя граница включена; при overlap побеждает последний resolved слой по стабильному author order. Normal не меняется от смены material, коэффициенты не интерполируются через разрыв.
- [ ] Сравнить mirror/yaw-transformed аналитические поверхности, проверить отсутствие allocation; будущий target `bike_spatial_terrain_sampling`, commit `feat(terrain): implement spatial bicubic sampling and analytic normals`.

```cpp
const auto s = plane.sample(0.25, -0.4);
bt::require(s.status == QueryStatus3d::ok, "plane query is inside");
bt::require(std::abs(s.height_m - 0.405) <= 1e-12,
            "bicubic reproduces an affine plane");
```

**Acceptance:** Абсолютная ошибка высоты≤1e−12 m на плоскости метрового масштаба; `abs(norm(normal)-1)≤1e−12`; normal непрерывна на внутренних швах. Текущая planar interpolation и её байты не изменены.

## S08. Первый ray hit, касания и исчерпание бюджета

**Depends on:** S07.

**Files:** Create `native/src/terrain/raycast3d.hpp`, `native/src/terrain/raycast3d.cpp`, `native/tests/spatial/terrain_raycast.cpp`; Modify `grid3d.{hpp,cpp}`.

**Consumes:** S07 interpolation polynomial и immutable grid.

**Produces:** `Grid3d::ray_cast(std::array<double,3> origin_m,std::array<double,3> unit_direction,double max_distance_m) const noexcept -> RayHit3d`; `QueryStatus3d::{ok,miss,outside,invalid_input,budget_exhausted,origin_below_surface}`. Hit содержит distance, world point, terrain normal и material ID. Начало ниже поверхности получает `origin_below_surface`, не `ok` с нулевым distance и не автоматически «полное сжатие шины».

- [ ] Проверить конечные inputs, `max_distance_m>=0`, единичную direction в установленной tolerance. `t` измеряется в метрах. Выполнить slab clipping XY domain; вертикальный луч обрабатывается отдельно, без деления на нулевой dx/dy.
- [ ] DDA идёт по возрастающему t. Его bound выводится из `ceil(abs(dir.x)*tmax/dx)+ceil(abs(dir.y)*tmax/dy)+2`; одинаковый corner-crossing посещается один раз. Обработать старт на grid line и направление в сторону negative axis.
- [ ] В каждой cell подставить line в bicubic: получаем polynomial степени не выше шести. Isolate earliest root с deterministic Bernstein interval subdivision: convex hull одинакового строгого знака исключает интервал; иначе делить пополам, сначала левый. Касание проверяется как minimum-distance interval без требования смены знака. Сохранить fixed-capacity stack и точный iteration budget в query config.
- [ ] При ширине root interval≤1e−6 m уточнить hit на исходной поверхности; принять его только при residual≤1e−8 m и доказанной interval-проверкой возможности корня. Для касания отдельно изолировать stationary points производного polynomial и проверить residual: малый интервал сам по себе не доказывает hit. Неопределённость, stack/iteration exhaustion возвращают `budget_exhausted`, не `miss`; normal берётся S07.
- [ ] Fixtures: vertical/down/up, origin below, t=0 boundary; oblique plane; bump с двумя пересечениями внутри cell при одинаковом знаке концов; exact tangent и near-miss с clearance≥2e−8 m; луч вдоль grid edge; outside entry; искусственно малый budget. Plane oracle аналитический, polynomial oracle — независимые high-precision roots; тест не вызывает тот же алгоритм как эталон.
- [ ] Будущий `spatial.terrain_raycast`; фиксировать max cells/subdivisions и отсутствие allocation; commit `feat(terrain): add bounded first-hit spatial ray casting`.

```cpp
const auto hit = plane.ray_cast({0.,0.,1.}, {0.,0.,-1.}, 2.);
bt::require(hit.status == QueryStatus3d::ok, "vertical ray hits");
bt::require(std::abs(hit.distance_m - 1.) <= 1e-6,
            "unit ray parameter is physical distance");
```

**Acceptance:** Ошибка ближайшего hit≤1e−6 m для независимо закреплённых fixtures; ни один tangent/two-root fixture не даёт ложного miss. Limit status проверяется вызывающей tyre-транзакцией S25.

## S09. C++-генератор бокового рельефа и provenance

**Depends on:** S06–S08 и существующий квалифицированный native 1D terrain/RNG контракт.

**Files:** Create `native/src/terrain/generator3d.hpp`, `native/src/terrain/generator3d.cpp`, `native/src/terrain/spec3d.hpp`, `native/src/terrain/spec3d.cpp`, `native/tests/spatial/terrain_generator.cpp`, `examples/research/alpine_rocky_3d_genspec.toml`, `examples/research/rough_uphill_savage_3d_genspec.toml`; Modify `native/src/terrain/spec_io.hpp`, `native/src/terrain/trackfile.cpp` additive dispatch only.

**Consumes:** Resolved `TrackSpec`, существующие longitudinal helpers, отдельный 3D seed и validated dimensions. Новые RNG stream не потребляют числа из planar stream.

**Produces:** `generate_grid3d(const Track3dSpec&) -> Grid3d` cold API; `Track3dSpec` содержит base track, seed, grid, bank/rut/rock/root/roughness sections, material zones и `generator_version`. Результат имеет reproducible hash в пределах закреплённого numerics profile.

- [ ] Начать с exact base elevation `h(x)`; добавить `y*tan(bank(x))`, где bank ramp — quintic smoothstep `6u^5-15u^4+10u^3`, |bank|≤15°. Гарантировать flat lateral lead-in 4 m и lead-out 5 m; слишком короткая трасса отклоняется.
- [ ] Для roughness использовать native separable Gaussian convolution над deterministic white-noise grid: kernel radius `ceil(4*ell/spacing)`, веса `exp(-.5*(k*spacing/ell)^2)`, нормировка в фиксированном порядке, mirror boundary. Это явная замена старого Python FFT-пути, с собственной version; не обещать старый NumPy seed output. Делить генерацию на strips с halo, чтобы не хранить ещё несколько full-size grids.
- [ ] Задать rock/rut/root как smooth finite-support functions с author coordinates и amplitude; physical surface однозначна z(x,y), overhang/cave не поддерживаются. Проверять результирующие slope/curvature после сложения features, а не только каждый feature отдельно. `rock` 40–150 mm и `rut` 20–60 mm являются allowed author ranges, не требованием случайно использовать максимумы в каждом seed.
- [ ] Создать отдельные feature fixtures: один shoulder rock, один rut, постоянный bank, mirrored spec. Проверить authored location/depth, lead-in, material order и отсутствие NaN. Не выбирать seed только потому, что он «проходит» ride.
- [ ] Сохранить complete resolved spec, seed, generator algorithm/version, platform numerics identity и SHA256. Для коротких reference tracks commit маленькие manifest; большие `.terrain3d` создаются в cache/output.
- [ ] Будущий `spatial.terrain_generator`; commit `feat(terrain): generate spatial tracks entirely in C++`.

```cpp
const auto a = write_terrain3d(generate_grid3d(spec));
const auto b = write_terrain3d(generate_grid3d(spec));
bt::require(a == b, "same resolved 3D spec and seed reproduce bytes");
```

**Acceptance:** Генерация полностью native; одинаковый seed/spec/version даёт одинаковый asset; memory budget соблюдён; bank/lead-in/feature-domain проверки явны. На разных платформах допускаются отдельные qualified manifests, пока общая побитовая RNG/math совместимость не доказана.

## S10. Точная extrusion как отдельный compatibility adapter

**Depends on:** S06 и S07; существующий qualified `biketyre::RoadProfile`/planar contact path. Может идти параллельно S08–S09.

**Files:** Create `native/src/terrain/extrusion3d.hpp`, `native/src/terrain/extrusion3d.cpp`, `native/src/terrain/source3d.hpp`, `native/src/terrain/source3d.cpp`, `native/tests/spatial/terrain_extrusion.cpp`; Modify `native/src/terrain/trackfile.cpp` и startup terrain dispatch через явный asset kind.

**Consumes:** Исходный planar profile и его исходный datum; corridor half-width.

**Produces:** Полный C3.2 contract: owned immutable `ProfileExtrusion3d` с `sample(double,double) const noexcept -> GridSample3d` и `ray_cast(array3,array3,double) const noexcept -> RayHit3d`; `Terrain3dKind`, `Terrain3dQueryView`, owned-variant `Terrain3dSource` с `query_view() const noexcept`. Новые общие типы объявляются в `source3d.hpp`; S11 лишь подключает владельца к runtime. Planar-matched contact adapter сохраняет прежний piecewise-linear solver.

- [ ] Сохранить исходные x/z nodes и прежний порядок операций при evaluation по x. Не пропускать profile через f32, Gaussian smoothing или Catmull–Rom: совпадение узлов не означает совпадения поверхности между ними.
- [ ] `height(x,y)=height_legacy(x)` внутри corridor; `normal_y=0`, material lookup использует прежнюю longitudinal surface map. Вне corridor status=`outside`; x-boundaries повторяют явно записанную legacy-политику только внутри declared source extent.
- [ ] Для regression bridge сохранить прежний путь при `dir.y==0`. Общий spatial ray сначала XY-clips interval, затем переводит normalized XZ parameter в world distance: `s=hypot(dir.x,dir.z)`, `t_world=t_xz/s` с origin offset. Pure-lateral `s=0`, outside-entry и below-clipped-origin получают явные C3.2 outcomes. Grid export служит visual preview/tolerant integration и не называется `bitwise bridge`.
- [ ] На трех неизменённых tracks проверить исходные nodes и 1001 межузловую точку, включая knots/edges; сравнивать через `std::bit_cast<uint64_t>`, фиксируя policy signed zero. Два профиля с одинаковыми sampled grid nodes, но разными knots, должны иметь разные fingerprint.
- [ ] Проверить `sample(x,+y)==sample(x,-y)`, `normal_y=0`, world-distance наклонного луча, pure lateral и конечную ширину. Bound query view имеет non-null context/function pointers и выдаётся только от окончательно размещённого immutable owner, который живёт дольше writers. Kind/fingerprint хранятся в provenance/replay; owner нельзя перемещать после выдачи view.
- [ ] Будущий `spatial.terrain_extrusion`; commit `feat(terrain): preserve exact planar profile extrusion semantics`.

```cpp
for (double x : probe_x) {
    const auto e = extrusion.sample(x, 0.7);
    bt::require(std::bit_cast<std::uint64_t>(e.height_m) ==
                std::bit_cast<std::uint64_t>(legacy_height(x)),
                "exact extrusion delegates to legacy height arithmetic");
    bt::require(e.normal_world[1] == 0., "extruded normal has no lateral part");
}
```

**Acceptance:** обещанная byte equality ограничена adapter-семантикой; полноценный 3D pneumatic writer проходит физический tolerance bridge, не наследует эту гарантию автоматически.

## S11. Startup ownership и render asset без повторного terrain decode

**Depends on:** S09, S10, S14, S17; подтверждённые producer contracts G1. S11 готовит model/setup ownership; последующая S31 потребляет upstream `G2-startup` и проверяет spatial startup. S11 не зависит от завершения S18/S19/S31.

**Files:** Create `native/src/terrain/render3d.hpp`, `native/src/terrain/render3d.cpp`, `native/tests/spatial/terrain_startup.cpp`; Modify `native/src/model/builder.hpp`, `native/src/model/terrain.cpp`, `native/src/initialization/context.hpp`, `native/src/initialization/setup_owner.cpp`, `native/src/initialization/research_startup.cpp`.

**Consumes:** Owned `Terrain3dSource`, `bike::model::BuildRequest`, `spatial::Config`, существующий `SetupOwner` lifecycle.

**Produces:** Один immutable terrain owner, разделяемый запросами шины, initial placement, artifact capture и viewer. `RenderGrid3d` содержит nrow/ncol, normalized elevation, world bounds/datum и physical asset fingerprint; не является источником tyre forces.

- [ ] Добавить spatial terrain field к BuildRequest additive способом. Existing `field` и planar MJCF ordering не менять. SetupOwner проверяет matching domain/corridor, backend и asset fingerprint до создания live runtime.
- [ ] Render downsample по площади, с вычисленными world bounds и height offset. Для constant-height grid обходить деление на zero range; для minimum/maximum сохранить физическую z-позицию через hfield `size` и geom translation.
- [ ] Заполнить все nrow×ncol: нынешний `terrain/heightfield.cpp:25–33` копирует одну строку, поэтому его planar overload не использовать как «готовый 3D». Новый overload принимает прямоугольные данные и не меняет старый.
- [ ] Wheel geom masks исключают engine ground contact, когда включён `pneumatic_3d`; hfield rendering не должен незаметно давать вторую нормальную силу. Для body/rider crash geoms topology-контракт явно определяет, используется ли отдельная collision surface; test проверяет конкретные masks.
- [ ] Проверить startup по сохранённому asset и по genspec: одинаковый resolved terrain fingerprint, совпадающие world heights, no second generation/decode. Изменение render resolution не меняет physical tyre samples или physics hash, но меняет render provenance.
- [ ] Будущий `spatial.terrain_startup`; commit `feat(startup): own spatial terrain and derive native render fields`.

```cpp
bt::require(render.physical_asset_sha256 == terrain.sha256(),
            "viewer geometry identifies the actual physical surface");
bt::require(before_samples == after_render_resample_samples,
            "render resolution cannot change physical terrain queries");
```

**Acceptance:** native config→asset→BuildRequest→SetupOwner проходит без Python; исходный planar MJCF сохраняет прежние байты. Viewer pixels и realtime скорость проверяются позже, этой задачей они не считаются подтверждёнными.

## S12. Terrain qualification, budgets и evidence

**Depends on:** S06–S11.

**Files:** Create `native/tests/spatial/terrain_acceptance.cpp`, `examples/research/terrain3d_manifest.json`, `docs/superpowers/audits/2026-10-22-spatial-3d-native-terrain.md`; Modify `docs/TESTING.md`, `docs/RIDE.md` с честным статусом capability.

**Consumes:** codec/query/generator tests и реальные manifests выбранного baseline.

**Produces:** Проверяемая terrain gate matrix: analytic accuracy, robustness, determinism, actual resident/peak memory, throughput и ограничения represented surfaces.

- [ ] Объединить маленькие fixtures в `spatial.terrain_acceptance`; тяжёлый 300 m профиль вынести в явно named test/benchmark, исключённый из inner loop, но обязательный один раз для phase acceptance.
- [ ] Для 300 m×3 m, 5×10 mm сетки ожидать 60 001×301=18 060 301 samples. Только f64 elevations занимают 144 482 408 bytes; прибавить materials, manifests и scratch отдельно. Зафиксировать memory budget ≤512 MiB peak для генератора одного такого asset и ≤256 MiB immutable elevation+materials owner; если не укладывается, использовать strip generation, а не менять physical resolution.
- [ ] Проверить sampler/ray tests на всех краях и extreme allowed slopes; no NaN, no out-of-bounds, no silent budget exhaustion. Отдельно измерить число cells/subdivisions и распределение ошибок, а не заявлять ожидаемые «100 ns/ray» как gate.
- [ ] Два раза сгенерировать reference seeds 0–4 на каждой квалифицируемой platform lane; сравнить within-platform hashes. Cross-platform сравнение пометить strict или numerical согласно G1 numerics contract; различие libm не скрывать заменой manifest.
- [ ] f32 study факультативна и не блокирует f64: высота ≤1e−5 m и normal angle ≤1e−4 rad на registered fixtures; относительная slope error без absolute floor запрещена, поскольку flat slope равен нулю. Переход на f32 требует отдельного physics review и нового asset version/fingerprint.
- [ ] Сохранить commands/toolchain/SHA/counts/max errors/RSS и все unqualified rows. Future native analysis/ASan/RTSan gates выбираются по действующим правилам репозитория; tests не запускаются в текущем задании. Commit `test(terrain): qualify spatial terrain contracts and budgets`.

```cpp
bt::require(grid.nx() == 60001 && grid.ny() == 301,
            "declared 300m physical terrain resolution is retained");
bt::require(query_budget_exhausted_count == 0,
            "registered terrain fixtures fit the explicit ray budget");
```

**Acceptance:** все обязательные строки имеют actual evidence; memory numbers и скорость помечены measured только после запуска. Этот gate допускает tyre integration, но не доказывает lateral grip, balance или пригодность симуляции для anti-wheelie исследования.

