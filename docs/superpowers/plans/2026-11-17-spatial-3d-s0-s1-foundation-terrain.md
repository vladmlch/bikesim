# Spatial 3D Phase 1 (Slices S0 & S1: Foundation, SO(3) Frames & Terrain3D) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Создать в `.worktrees/spatial-3d` изолированное ядро `bike_spatial_core` (`SO(3)` фреймы + строгий контракт конфигурации `[spatial]` без влияния на 2D) и полный модуль 3D-рельефа `Terrain3D` (`ExtrudedTrack3d` с побитовым 1D-мостом + кэш-локальный `Grid3d` `ix * ny + iy` с монотонной бикубической интерполяцией Стивенса без выбросов Гиббса на уступах 100 мм + запрос кратчайшей точки тора + 3D-оверлей существующих трасс).

**Architecture:** Новые пространственные примитивы собираются в отдельную статическую библиотеку `bike_spatial_core`, а 3D-рельеф добавляется новыми единицами трансляции в `bike_terrain`, не меняя ни одного существующего 1D-файла и не раздувая `bike_core`. Все существующие плоские конфигурации, сериализации и эталоны остаются побитово нетронутыми.

**Tech Stack:** C++23 (`AppleClang` + вторичный фронтенд `g++-16`), CMake ≥ 3.27, Ninja, `-Werror -ffp-contract=off -Wunsafe-buffer-usage`, тестовый харнесс `bt::run` (`native/tests/support/fixture_test.hpp`).

**Spec:** [docs/superpowers/specs/2026-11-17-spatial-3d-design.md](../specs/2026-11-17-spatial-3d-design.md) и [docs/agents/simulation-decisions.md](../../agents/simulation-decisions.md) (пп. 3, 12, 26, 45).

## Global Constraints

- Работать строго внутри `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/spatial-3d` (ветка `spatial-3d` от `0502660d`).
- **Побитовый щит 2D:** не модифицировать существующие формулы `native/src/terrain/{profile,obstacles,alpine,heightfield,surface}.cpp` и не добавлять новых полей в сериализацию плоского `SimulationPhysicsConfig` при отсутствии `[spatial]`.
- **Флаги компиляции и контракты:** никакой адресной арифметики по сырым указателям (`-Wunsafe-buffer-usage` — использовать `std::array` и `std::span`), никаких свободных функций вне `namespace { ... }` без объявления в заголовке (`-Wmissing-declarations`), никаких стековых кадров > 8192 байт (`-Wframe-larger-than=8192`), каждый заголовок обязан компилироваться автономно (`check_headers`), один `.cpp` входит ровно в один таргет (`odr`).
- **Задержки и такты:** в `SpatialConfig` нет скрытых дефолтов таймингов в C++; каждая задержка и период проверяются на точную целочисленную кратность `timestep_s` (допуск округления $\le 10^{-12}$, никакого `std::llround` произвольных чисел).
- **Бюджет тестов на весь план `S0 + S1` (~5 мин суммарно):** внутри задач 1–5 запускать только целевой CTest (`cmake --build build/native-release --target <test_target> && ctest --test-dir build/native-release -R '^<test_target>$' --no-tests=error --output-on-failure`, по ~3–8 с); полный срез проверить **один раз** в конце Задачи 5 командой `bash tools/check_change.sh 0502660d` (~2.5–3 мин). Не запускать `run_tests.sh native` или `full` на промежуточных шагах.

---

## Карта создаваемых и изменяемых файлов

| Файл | Действие | Ответственность |
|---|---|---|
| `native/src/spatial/frames.hpp` | Create | Чистые типы `Vec3`, `Quat`, `EulerZyx`, `EulerRatesZyx`, `BodyPose` и объявления функций `SO(3)` |
| `native/src/spatial/frames.cpp` | Create | Операции кватернионов (`wxyz`), ZYX Эйлер с порогом сингулярности `80°`, кинематика скоростей Эйлера, единичный вектор горизонтального курса |
| `native/src/spatial/config.hpp` | Create | Типизированная структура `SpatialConfig` без скрытых дефолтов таймингов и интерфейс парсинга/валидации |
| `native/src/spatial/config.cpp` | Create | Строгий декодер `Value` $\leftrightarrow$ `SpatialConfig`, проверка точной кратности шагу `timestep_s` и допустимых пресетов |
| `native/src/terrain/terrain3d_types.hpp` | Create | Общие типы `Terrain3D`: `QueryStatus3d`, `TerrainSample3d`, `TorusClosestPoint3d`, `TerrainBounds3d` |
| `native/src/terrain/extrusion3d.hpp` | Create | Класс `ExtrudedTrack3d` — аналитическая 3D-экструзия существующего `TrackSpec` |
| `native/src/terrain/extrusion3d.cpp` | Create | Реализация `ExtrudedTrack3d`: делегирование в `biketyre::np_interp`, `SignedProfile`, `SurfaceMap` и потоковый `Sha256` |
| `native/src/terrain/interpolation3d.hpp` | Create | Монотонная бикубическая интерполяция Эрмита (схема Стивенса) на ячейке 4×4 |
| `native/src/terrain/interpolation3d.cpp` | Create | Вычисление монотонных производных $f_x, f_y, f_{xy}$ без выбросов Гиббса и аналитической высоты/нормали |
| `native/src/terrain/grid3d.hpp` | Create | Регулярная сетка `Grid3d` в порядке `ix * ny + iy` с предвычисленными `z_min/z_max` ячеек |
| `native/src/terrain/grid3d.cpp` | Create | Валидация сетки, потоковый SHA-256 без копирования буфера, `sample(x, y)` и поиск ближайшей точки `closest_point` |
| `native/src/terrain/source3d.hpp` | Create | Полиморфный владелец `Terrain3dSource` и легковесный `Terrain3dQueryView` (`noexcept`) |
| `native/src/terrain/source3d.cpp` | Create | Диспетчеризация `ExtrudedTrack3d` / `Grid3d` без виртуальных аллокаций |
| `native/src/terrain/generator3d.hpp` | Create | Спецификация `CrossSlopeSegment3d`, `Terrain3dOverlaySpec` и генератор поверх `TrackSpec` |
| `native/src/terrain/generator3d.cpp` | Create | Построение `Grid3d` поверх любой трассы `main` с сохранением `SurfaceSection` и гладким гашением в `lead_in`/`lead_out` |
| `CMakeLists.txt` | Modify | Объявление статической библиотеки `bike_spatial_core` |
| `native/cmake/NativeStartup.cmake` | Modify | Добавление `*3d*.cpp` в `bike_terrain` и линковка `bike_spatial_core` при необходимости |
| `native/tests/CMakeLists.txt` | Modify | Подключение подкаталога `spatial` |
| `native/tests/spatial/CMakeLists.txt` | Create | Регистрация контрактных тестов `spatial_frames` и `spatial_config` |
| `native/tests/spatial/test_frames.cpp` | Create | Тесты `SO(3)`, антиподальной инвариантности `q/-q`, сингулярности `80°`, кинематики гироскопа и курса |
| `native/tests/spatial/test_config.cpp` | Create | Тесты обязательных полей `[spatial]`, отказа на некратных задержках и побитовой неизменности 2D конфига |
| `native/tests/terrain/CMakeLists.txt` | Modify | Добавление `terrain_extrusion3d`, `terrain_grid3d`, `terrain_generator3d` |
| `native/tests/terrain/test_extrusion3d.cpp` | Create | Тесты побитового моста `ExtrudedTrack3d == 1D` на всех 5 рабочих трассах `main` |
| `native/tests/terrain/test_grid3d.cpp` | Create | Тесты порядка памяти, `0%` overshoot на `SquareEdge` 100 мм и `Pothole` 140 мм, непрерывности нормалей и `closest_point` |
| `native/tests/terrain/test_generator3d.cpp` | Create | Тесты генерации косогора на `savage` и `alpine_rocky`, сохранения секций `scree` и зеркальной симметрии |

---

### Task 1: Срез S0a — библиотека `bike_spatial_core` и математика `SO(3)` фреймов

**Files:**
- Create: `native/src/spatial/frames.hpp`
- Create: `native/src/spatial/frames.cpp`
- Modify: `CMakeLists.txt`
- Modify: `native/tests/CMakeLists.txt`
- Create: `native/tests/spatial/CMakeLists.txt`
- Create: `native/tests/spatial/test_frames.cpp`

**Interfaces:**
- Consumes: только стандартная библиотека C++23 (`<array>`, `<cmath>`, `<numbers>`) и `bike_io` (для будущих задач в `bike_spatial_core`).
- Produces: `bike::spatial::{Vec3, Quat, EulerZyx, EulerRatesZyx, BodyPose, quat_multiply, quat_conjugate, quat_normalized, rotate_body_to_world, rotate_world_to_body, geodesic_angle_rad, euler_zyx, body_gyro_to_euler_zyx_rates, horizontal_heading_unit, point_velocity_world}` в статической библиотеке `bike_spatial_core`.

- [x] **Step 1: Создать `native/src/spatial/frames.hpp` и `native/tests/spatial/test_frames.cpp` (сначала заголовок и тест)**

Создать `native/src/spatial/frames.hpp`:
```cpp
#pragma once

#include <array>
#include <numbers>

namespace bike::spatial {

using Vec3 = std::array<double, 3>;
// MuJoCo convention: (w, x, y, z), rotating body-frame vectors to world-frame.
using Quat = std::array<double, 4>;

inline constexpr double kDefaultGimbalMarginRad = std::numbers::pi / 18.; // 10 deg -> |pitch| < 80 deg

struct EulerZyx {
    double roll_rad = 0.;
    double pitch_rad = 0.;
    double yaw_rad = 0.;
    bool valid = false;
};

struct EulerRatesZyx {
    double roll_rate_rad_s = 0.;
    double pitch_rate_rad_s = 0.;
    double yaw_rate_rad_s = 0.;
    bool valid = false;
};

struct BodyPose {
    Vec3 position_world_m{};
    Quat quat_world_from_body_wxyz{1., 0., 0., 0.};
    Vec3 linear_velocity_world_mps{};
    Vec3 angular_velocity_world_rad_s{};
};

[[nodiscard]] Quat quat_multiply(const Quat& a, const Quat& b) noexcept;
[[nodiscard]] Quat quat_conjugate(const Quat& q) noexcept;
[[nodiscard]] Quat quat_normalized(const Quat& q) noexcept;
[[nodiscard]] Quat quat_from_euler_zyx(double roll_rad, double pitch_rad, double yaw_rad) noexcept;

[[nodiscard]] Vec3 rotate_body_to_world(const Quat& q_world_from_body, const Vec3& v_body) noexcept;
[[nodiscard]] Vec3 rotate_world_to_body(const Quat& q_world_from_body, const Vec3& v_world) noexcept;

[[nodiscard]] double geodesic_angle_rad(const Quat& a, const Quat& b) noexcept;
[[nodiscard]] EulerZyx euler_zyx(const Quat& q_world_from_body,
                                 double gimbal_margin_rad = kDefaultGimbalMarginRad) noexcept;
[[nodiscard]] EulerRatesZyx body_gyro_to_euler_zyx_rates(
    const EulerZyx& attitude, const Vec3& omega_body_rad_s) noexcept;

// Unit horizontal heading vector (hx, hy, 0) of body +X projected onto world XY.
// Returns valid=false when body +X is within gimbal_margin_rad of world +-Z.
struct HorizontalHeading {
    double hx = 1.;
    double hy = 0.;
    double heading_rad = 0.;
    bool valid = false;
};
[[nodiscard]] HorizontalHeading horizontal_heading_unit(
    const Quat& q_world_from_body,
    double gimbal_margin_rad = kDefaultGimbalMarginRad) noexcept;

[[nodiscard]] Vec3 point_velocity_world(const BodyPose& pose, const Vec3& point_world_m) noexcept;

} // namespace bike::spatial
```

Создать `native/tests/spatial/test_frames.cpp`:
```cpp
#include "spatial/frames.hpp"
#include "support/fixture_test.hpp"

#include <cmath>
#include <limits>
#include <numbers>

int main() {
    return bt::run([] {
        using namespace bike::spatial;
        constexpr double pi = std::numbers::pi;

        // 1. Right-handed MuJoCo convention: +roll tilts body +Z toward world -Y (right).
        const Quat q_roll_right = quat_from_euler_zyx(pi / 6., 0., 0.);
        const Vec3 up_world = rotate_body_to_world(q_roll_right, {0., 0., 1.});
        bt::require(std::abs(up_world[0]) <= 1e-15, "pure roll keeps world X zero");
        bt::require(std::abs(up_world[1] - (-0.5)) <= 1e-15, "+30 deg roll tilts top right (-Y)");
        bt::require(std::abs(up_world[2] - std::cos(pi / 6.)) <= 1e-15, "roll Z cosine");

        // 2. Antipodal invariance: q and -q produce identical rotated vectors and Euler angles.
        const Quat q = quat_from_euler_zyx(0.25, -0.18, 1.35);
        const Quat neg_q{-q[0], -q[1], -q[2], -q[3]};
        const Vec3 v{1.2, -0.4, 0.7};
        const Vec3 r_pos = rotate_body_to_world(q, v);
        const Vec3 r_neg = rotate_body_to_world(neg_q, v);
        for (std::size_t i = 0; i < 3; ++i) {
            bt::require(r_pos[i] == r_neg[i], "q and -q rotate identically bitwise");
        }
        const Vec3 back = rotate_world_to_body(q, r_pos);
        for (std::size_t i = 0; i < 3; ++i) {
            bt::require(std::abs(back[i] - v[i]) <= 1e-14, "body->world->body roundtrip");
        }

        // 3. ZYX Euler roundtrip and 80-deg gimbal guard.
        const EulerZyx e = euler_zyx(q);
        bt::require(e.valid, "moderate pitch attitude is valid");
        bt::require(std::abs(e.roll_rad - 0.25) <= 1e-14, "roll recovered");
        bt::require(std::abs(e.pitch_rad - (-0.18)) <= 1e-14, "pitch recovered");
        bt::require(std::abs(e.yaw_rad - 1.35) <= 1e-14, "yaw recovered");

        const EulerZyx near_pole = euler_zyx(quat_from_euler_zyx(0.1, 82. * pi / 180., 0.3));
        bt::require(!near_pole.valid, "pitch >= 80 deg marked invalid before tan blow-up");

        // 4. Body gyro -> Euler rates finite-difference consistency.
        const Vec3 omega_b{0.7, -0.35, 0.45};
        const EulerRatesZyx rates = body_gyro_to_euler_zyx_rates(e, omega_b);
        bt::require(rates.valid, "Euler rates valid on valid attitude");
        constexpr double dt = 1e-7;
        const Quat dq{1., 0.5 * omega_b[0] * dt, 0.5 * omega_b[1] * dt, 0.5 * omega_b[2] * dt};
        const EulerZyx e_next = euler_zyx(quat_normalized(quat_multiply(q, dq)));
        bt::require(std::abs((e_next.roll_rad - e.roll_rad) / dt - rates.roll_rate_rad_s) <= 1e-6,
                    "roll rate matches quaternion kinematic derivative");
        bt::require(std::abs((e_next.pitch_rad - e.pitch_rad) / dt - rates.pitch_rate_rad_s) <= 1e-6,
                    "pitch rate matches quaternion kinematic derivative");
        bt::require(std::abs((e_next.yaw_rad - e.yaw_rad) / dt - rates.yaw_rate_rad_s) <= 1e-6,
                    "yaw rate matches quaternion kinematic derivative");

        // 5. Horizontal heading at arbitrary yaw (+90 deg, +180 deg) and point velocity.
        const HorizontalHeading h90 = horizontal_heading_unit(quat_from_euler_zyx(0.2, -0.1, pi / 2.));
        bt::require(h90.valid, "heading valid at 90 deg yaw");
        bt::require(std::abs(h90.hx) <= 1e-14 && std::abs(h90.hy - 1.) <= 1e-14, "yaw +90 deg points +Y");

        const BodyPose pose{
            .position_world_m = {1., 2., 0.5},
            .quat_world_from_body_wxyz = {1., 0., 0., 0.},
            .linear_velocity_world_mps = {3., 0., 0.},
            .angular_velocity_world_rad_s = {0., 0., 2.},
        };
        const Vec3 vp = point_velocity_world(pose, {1., 2.5, 0.5});
        bt::require(std::abs(vp[0] - 2.) <= 1e-15 && std::abs(vp[1] - 0.) <= 1e-15,
                    "lever-arm velocity v + w x r");

        // 6. Nonfinite inputs fail closed.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        bt::require(!euler_zyx({nan, 0., 0., 0.}).valid, "NaN quaternion rejected");
    });
}
```

- [x] **Step 2: Подключить `bike_spatial_core` и `spatial_frames` в CMake (пока без `frames.cpp` — проверить ожидаемую ошибку сборки)**

В `CMakeLists.txt` сразу после блока `bike_random` (после строки 507) добавить:
```cmake
add_library(bike_spatial_core STATIC
  "${_bike_native_dir}/src/spatial/frames.hpp"
  "${_bike_native_dir}/src/spatial/frames.cpp")
set_target_properties(bike_spatial_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
bike_apply_native_policy(bike_spatial_core)
target_include_directories(bike_spatial_core SYSTEM PRIVATE
  "${_bike_native_dir}/src")
target_link_libraries(bike_spatial_core PUBLIC bike_io)
```

В `native/tests/CMakeLists.txt` добавить `spatial` в `foreach(_domain IN ITEMS ...)`:
```cmake
foreach(_domain IN ITEMS batch cli config drivetrain engine initialization
    io kinematics model numerics platform random recording replay
    research rider runtime spatial terrain validation viewer viz)
  add_subdirectory(${_domain})
endforeach()
```

Создать `native/tests/spatial/CMakeLists.txt`:
```cmake
foreach(_spatial_test IN ITEMS spatial_frames)
  string(REGEX REPLACE "^spatial_" "" _spatial_src "${_spatial_test}")
  add_executable(${_spatial_test} "test_${_spatial_src}.cpp")
  bike_startup_target_policy(${_spatial_test} "${_test_repo_root}/native")
  target_link_libraries(${_spatial_test} PRIVATE bike_test_fixtures bike_spatial_core)
  if(BUILD_TESTING)
    add_test(NAME ${_spatial_test} COMMAND ${_spatial_test})
    set_tests_properties(${_spatial_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_spatial_test}>")
  endif()
endforeach()
```

Проверить ожидаемый отказ (отсутствует `frames.cpp`):
Run: `cmake --preset release`
Expected: FAIL (`Cannot find source file: .../native/src/spatial/frames.cpp`)

- [x] **Step 3: Написать реализацию `native/src/spatial/frames.cpp`**

```cpp
#include "spatial/frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace bike::spatial {
namespace {
[[nodiscard]] bool all_finite(const auto& arr) noexcept {
    return std::ranges::all_of(arr, [](double x) { return std::isfinite(x); });
}
} // namespace

Quat quat_multiply(const Quat& a, const Quat& b) noexcept {
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
}

Quat quat_conjugate(const Quat& q) noexcept {
    return {q[0], -q[1], -q[2], -q[3]};
}

Quat quat_normalized(const Quat& q) noexcept {
    const double scale = std::max({std::abs(q[0]), std::abs(q[1]), std::abs(q[2]), std::abs(q[3])});
    if (!(scale > 0.) || !std::isfinite(scale)) {
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        return {invalid, invalid, invalid, invalid};
    }
    Quat r{q[0] / scale, q[1] / scale, q[2] / scale, q[3] / scale};
    const double norm = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
    for (double& c : r) {
        c /= norm;
    }
    return r;
}

Quat quat_from_euler_zyx(double roll_rad, double pitch_rad, double yaw_rad) noexcept {
    if (!std::isfinite(roll_rad) || !std::isfinite(pitch_rad) || !std::isfinite(yaw_rad)) {
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        return {invalid, invalid, invalid, invalid};
    }
    const Quat qx{std::cos(0.5 * roll_rad), std::sin(0.5 * roll_rad), 0., 0.};
    const Quat qy{std::cos(0.5 * pitch_rad), 0., std::sin(0.5 * pitch_rad), 0.};
    const Quat qz{std::cos(0.5 * yaw_rad), 0., 0., std::sin(0.5 * yaw_rad)};
    return quat_normalized(quat_multiply(quat_multiply(qz, qy), qx));
}

Vec3 rotate_body_to_world(const Quat& q, const Vec3& v) noexcept {
    const Vec3 t{
        2. * (q[2] * v[2] - q[3] * v[1]),
        2. * (q[3] * v[0] - q[1] * v[2]),
        2. * (q[1] * v[1] - q[2] * v[0]),
    };
    return {
        v[0] + q[0] * t[0] + q[2] * t[2] - q[3] * t[1],
        v[1] + q[0] * t[1] + q[3] * t[0] - q[1] * t[2],
        v[2] + q[0] * t[2] + q[1] * t[1] - q[2] * t[0],
    };
}

Vec3 rotate_world_to_body(const Quat& q, const Vec3& v) noexcept {
    return rotate_body_to_world(quat_conjugate(q), v);
}

double geodesic_angle_rad(const Quat& a, const Quat& b) noexcept {
    const Quat na = quat_normalized(a);
    const Quat nb = quat_normalized(b);
    if (!std::isfinite(na[0]) || !std::isfinite(nb[0])) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Quat d = quat_multiply(na, quat_conjugate(nb));
    return 2. * std::atan2(std::hypot(d[1], d[2], d[3]), std::abs(d[0]));
}

EulerZyx euler_zyx(const Quat& q, double gimbal_margin_rad) noexcept {
    EulerZyx out{};
    if (!all_finite(q) || !std::isfinite(gimbal_margin_rad) || gimbal_margin_rad <= 0. ||
        gimbal_margin_rad >= 0.5 * std::numbers::pi) {
        return out;
    }
    const Quat n = quat_normalized(q);
    if (!std::isfinite(n[0])) {
        return out;
    }
    const double sine = std::clamp(2. * (n[0] * n[2] - n[3] * n[1]), -1., 1.);
    out.pitch_rad = std::asin(sine);
    out.roll_rad = std::atan2(2. * (n[0] * n[1] + n[2] * n[3]),
                              1. - 2. * (n[1] * n[1] + n[2] * n[2]));
    out.yaw_rad = std::atan2(2. * (n[0] * n[3] + n[1] * n[2]),
                             1. - 2. * (n[2] * n[2] + n[3] * n[3]));
    out.valid = std::abs(out.pitch_rad) < (0.5 * std::numbers::pi - gimbal_margin_rad);
    return out;
}

EulerRatesZyx body_gyro_to_euler_zyx_rates(const EulerZyx& attitude,
                                           const Vec3& omega_body_rad_s) noexcept {
    EulerRatesZyx out{};
    if (!attitude.valid || !all_finite(omega_body_rad_s)) {
        return out;
    }
    const double cp = std::cos(attitude.pitch_rad);
    if (!(cp > 0.)) {
        return out;
    }
    const double sp = std::sin(attitude.pitch_rad);
    const double sr = std::sin(attitude.roll_rad);
    const double cr = std::cos(attitude.roll_rad);
    const double q_sin_r_cos = sr * omega_body_rad_s[1] + cr * omega_body_rad_s[2];
    out.roll_rate_rad_s = omega_body_rad_s[0] + (sp / cp) * q_sin_r_cos;
    out.pitch_rate_rad_s = cr * omega_body_rad_s[1] - sr * omega_body_rad_s[2];
    out.yaw_rate_rad_s = q_sin_r_cos / cp;
    if (!std::isfinite(out.roll_rate_rad_s) || !std::isfinite(out.pitch_rate_rad_s) ||
        !std::isfinite(out.yaw_rate_rad_s)) {
        return {};
    }
    out.valid = true;
    return out;
}

HorizontalHeading horizontal_heading_unit(const Quat& q, double gimbal_margin_rad) noexcept {
    HorizontalHeading out{};
    if (!all_finite(q) || !std::isfinite(gimbal_margin_rad) || gimbal_margin_rad <= 0. ||
        gimbal_margin_rad >= 0.5 * std::numbers::pi) {
        return out;
    }
    const Quat n = quat_normalized(q);
    if (!std::isfinite(n[0])) {
        return out;
    }
    const Vec3 forward = rotate_body_to_world(n, {1., 0., 0.});
    const double horiz = std::hypot(forward[0], forward[1]);
    if (!(horiz > std::sin(gimbal_margin_rad)) || !std::isfinite(horiz)) {
        return out;
    }
    out.hx = forward[0] / horiz;
    out.hy = forward[1] / horiz;
    out.heading_rad = std::atan2(out.hy, out.hx);
    out.valid = true;
    return out;
}

Vec3 point_velocity_world(const BodyPose& pose, const Vec3& point_world_m) noexcept {
    const Vec3 r{
        point_world_m[0] - pose.position_world_m[0],
        point_world_m[1] - pose.position_world_m[1],
        point_world_m[2] - pose.position_world_m[2],
    };
    const Vec3& w = pose.angular_velocity_world_rad_s;
    const Vec3& v = pose.linear_velocity_world_mps;
    return {
        v[0] + w[1] * r[2] - w[2] * r[1],
        v[1] + w[2] * r[0] - w[0] * r[2],
        v[2] + w[0] * r[1] - w[1] * r[0],
    };
}

} // namespace bike::spatial
```

- [x] **Step 4: Собрать и запустить `spatial_frames`, затем закоммитить**

Run:
```bash
cmake --preset release && cmake --build build/native-release --target spatial_frames && ctest --test-dir build/native-release -R '^spatial_frames$' --no-tests=error --output-on-failure
```
Expected: PASS (`1/1 Test #...: spatial_frames ... Passed`)

Commit:
```bash
git add CMakeLists.txt native/tests/CMakeLists.txt native/src/spatial/frames.hpp native/src/spatial/frames.cpp native/tests/spatial/CMakeLists.txt native/tests/spatial/test_frames.cpp docs/agents/simulation-decisions.md docs/superpowers/specs/2026-11-17-spatial-3d-design.md docs/superpowers/plans/2026-11-17-spatial-3d-s0-s1-foundation-terrain.md
git commit -m "feat(spatial): add isolated bike_spatial_core and SO(3) frame kinematics"
```

---

### Task 2: Срез S0b — строгий контракт конфигурации `[spatial]` и побитовый щит 2D

**Files:**
- Create: `native/src/spatial/config.hpp`
- Create: `native/src/spatial/config.cpp`
- Modify: `CMakeLists.txt`
- Modify: `native/tests/spatial/CMakeLists.txt`
- Create: `native/tests/spatial/test_config.cpp`

**Interfaces:**
- Consumes: `bike::io::Value`, `bike::spatial::Vec3`, `bike::configuration::SimulationPhysicsConfig` (только для теста побитовой неизменности 2D).
- Produces: `bike::spatial::{SpatialTireBackend, SpatialTirePreset, SpatialStartMode, SpatialSteerSpec, SpatialStartSupportSpec, SpatialConfig, parse_spatial_config, validate_spatial_config, spatial_config_to_value}` с проверкой обязательных таймингов кратно `timestep_s`.

- [ ] **Step 1: Написать `native/src/spatial/config.hpp` и тест `native/tests/spatial/test_config.cpp`**

Создать `native/src/spatial/config.hpp`:
```cpp
#pragma once

#include "io/value.hpp"
#include "spatial/frames.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace bike::spatial {

enum class SpatialTireBackend : std::uint8_t { compliant_3d, distributed_3d };
enum class SpatialTirePreset : std::uint8_t { mtb_2p8_1p2bar, mtb_2p4_standard };
enum class SpatialStartMode : std::uint8_t { moving, standstill };

struct SpatialSteerSpec {
    double range_deg = -1.;
    double control_period_s = -1.;
    double reaction_delay_s = -1.;
    double max_torque_nm = -1.;
};

struct SpatialStartSupportSpec {
    double release_speed_mps = -1.;
    double lateral_stiffness_n_per_m = -1.;
    double lateral_damping_ns_per_m = -1.;
    double roll_stiffness_nm_per_rad = -1.;
    double roll_damping_nms_per_rad = -1.;
};

struct SpatialConfig {
    SpatialTireBackend tire_backend = SpatialTireBackend::compliant_3d;
    SpatialTirePreset tire_preset = SpatialTirePreset::mtb_2p8_1p2bar;
    SpatialStartMode start_mode = SpatialStartMode::moving;
    double corridor_half_width_m = -1.;
    double capsize_roll_rad = -1.;
    SpatialSteerSpec steer{};
    std::optional<SpatialStartSupportSpec> start_support{};
    Vec3 imu_mount_pos_m{0., 0., 0.};
    Vec3 imu_mount_rpy_rad{0., 0., 0.};
    Vec3 imu_misalignment_rad{0., 0., 0.};
};

// Returns std::nullopt when root_object has no "spatial" key (planar mode).
[[nodiscard]] std::optional<SpatialConfig> parse_spatial_config(
    const bike::io::Value::Object& root_object, double timestep_s);

void validate_spatial_config(const SpatialConfig& config, double timestep_s);
[[nodiscard]] bike::io::Value spatial_config_to_value(const SpatialConfig& config);

// Exact integer-multiple check (Decision 12): ratio = duration_s / step_s must be
// an integer >= min_ticks within 1e-12 relative tolerance; rejects silent rounding.
[[nodiscard]] std::int64_t require_exact_ticks(
    double duration_s, double step_s, std::int64_t min_ticks, const char* field_path);

} // namespace bike::spatial
```

Создать `native/tests/spatial/test_config.cpp`:
```cpp
#include "configuration/load.hpp"
#include "configuration/physical_config.hpp"
#include "configuration/schema.hpp"
#include "io/json.hpp"
#include "spatial/config.hpp"
#include "support/fixture_test.hpp"

int main() {
    return bt::run([] {
        using namespace bike::spatial;
        using bike::io::Value;

        constexpr double dt = 0.00125;

        // 1. Planar table without "spatial" returns nullopt and keeps 2D schema/JSON untouched.
        const Value::Object empty_root{};
        bt::require(!parse_spatial_config(empty_root, dt).has_value(), "absent spatial -> planar nullopt");

        const bike::configuration::SimulationPhysicsConfig planar_default{};
        const std::string json_before = bike::io::dump_json(bike::configuration::to_value(planar_default));
        bt::require(json_before.find("spatial") == std::string::npos,
                    "planar SimulationPhysicsConfig wire contains zero spatial keys");

        // 2. Valid moving-start SpatialConfig roundtrips cleanly.
        Value::Object steer_obj{
            {"range_deg", Value{45.0}},
            {"control_period_s", Value{0.005}},
            {"reaction_delay_s", Value{0.100}},
            {"max_torque_nm", Value{35.0}},
        };
        Value::Object spatial_obj{
            {"tire_backend", Value{std::string("compliant_3d")}},
            {"tire_preset", Value{std::string("2p8_1p2bar")}},
            {"start_mode", Value{std::string("moving")}},
            {"corridor_half_width_m", Value{1.5}},
            {"capsize_roll_rad", Value{1.0471975511965976}},
            {"steer", Value{steer_obj}},
        };
        Value::Object root_with_spatial{{"spatial", Value{spatial_obj}}};
        const auto parsed = parse_spatial_config(root_with_spatial, dt);
        bt::require(parsed.has_value(), "valid spatial config parsed");
        bt::require(parsed->tire_backend == SpatialTireBackend::compliant_3d, "backend decoded");
        bt::require(parsed->tire_preset == SpatialTirePreset::mtb_2p8_1p2bar, "2.8 preset decoded");
        bt::require(require_exact_ticks(parsed->steer.control_period_s, dt, 1, "steer.control_period_s") == 4,
                    "5 ms is exact 4 ticks of 1.25 ms");
        bt::require(require_exact_ticks(parsed->steer.reaction_delay_s, parsed->steer.control_period_s, 0,
                                        "steer.reaction_delay_s") == 20,
                    "100 ms is exact 20 control ticks");

        const Value serialized = spatial_config_to_value(*parsed);
        const auto reparsed = parse_spatial_config(
            Value::Object{{"spatial", serialized}}, dt);
        bt::require(reparsed.has_value(), "serialized spatial config reparses");
        bt::require(bike::io::dump_json(serialized) ==
                        bike::io::dump_json(spatial_config_to_value(*reparsed)),
                    "canonical JSON roundtrip is bitwise deterministic");

        // 3. Decision 12: non-multiple delay or missing required timing fails closed.
        auto bad_delay = spatial_obj;
        auto bad_steer = steer_obj;
        bad_steer["reaction_delay_s"] = Value{0.103}; // not a multiple of 0.005 s!
        bad_delay["steer"] = Value{bad_steer};
        bt::invalid([&] {
            (void)parse_spatial_config(Value::Object{{"spatial", Value{bad_delay}}}, dt);
        });

        // 4. Standstill mode requires explicit start_support block; moving mode forbids unknown keys.
        auto standstill_missing_support = spatial_obj;
        standstill_missing_support["start_mode"] = Value{std::string("standstill")};
        bt::invalid([&] {
            (void)parse_spatial_config(Value::Object{{"spatial", Value{standstill_missing_support}}}, dt);
        });

        auto unknown_key = spatial_obj;
        unknown_key["damping_nms_rad"] = Value{0.5}; // Passive steer damping is banned!
        bt::invalid([&] {
            (void)parse_spatial_config(Value::Object{{"spatial", Value{unknown_key}}}, dt);
        });
    });
}
```

- [ ] **Step 2: Добавить `config.cpp` и `spatial_config` в CMake и проверить ожидаемый отказ до создания `config.cpp`**

В `CMakeLists.txt` обновить список источников `bike_spatial_core`:
```cmake
add_library(bike_spatial_core STATIC
  "${_bike_native_dir}/src/spatial/frames.hpp"
  "${_bike_native_dir}/src/spatial/frames.cpp"
  "${_bike_native_dir}/src/spatial/config.hpp"
  "${_bike_native_dir}/src/spatial/config.cpp")
```

В `native/tests/spatial/CMakeLists.txt` обновить список тестов (и прилинковать `bike_configuration` для проверки 2D конфига):
```cmake
foreach(_spatial_test IN ITEMS spatial_frames spatial_config)
  string(REGEX REPLACE "^spatial_" "" _spatial_src "${_spatial_test}")
  add_executable(${_spatial_test} "test_${_spatial_src}.cpp")
  bike_startup_target_policy(${_spatial_test} "${_test_repo_root}/native")
  target_link_libraries(${_spatial_test} PRIVATE
    bike_test_fixtures bike_spatial_core bike_configuration)
  if(BUILD_TESTING)
    add_test(NAME ${_spatial_test} COMMAND ${_spatial_test})
    set_tests_properties(${_spatial_test} PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_spatial_test}>")
  endif()
endforeach()
```

Run: `cmake --preset release`
Expected: FAIL (`Cannot find source file: .../native/src/spatial/config.cpp`)

- [ ] **Step 3: Написать `native/src/spatial/config.cpp`**

```cpp
#include "spatial/config.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string_view>

namespace bike::spatial {
namespace {

using bike::io::Value;

const Value& require_field(const Value::Object& obj, std::string_view key, const std::string& prefix) {
    const auto it = obj.find(std::string(key));
    if (it == obj.end()) {
        throw std::invalid_argument(prefix + "." + std::string(key) + ": required field missing");
    }
    return it->second;
}

void reject_unknown_keys(const Value::Object& obj, const std::set<std::string_view>& allowed,
                         const std::string& prefix) {
    for (const auto& [key, _] : obj) {
        if (!allowed.contains(key)) {
            throw std::invalid_argument(prefix + "." + key + ": unknown field");
        }
    }
}

double require_real(const Value& v, const std::string& path) {
    if (const auto* d = std::get_if<double>(&v.storage)) {
        if (!std::isfinite(*d)) {
            throw std::invalid_argument(path + ": finite real number required");
        }
        return *d;
    }
    if (const auto* i = std::get_if<std::int64_t>(&v.storage)) {
        return static_cast<double>(*i);
    }
    throw std::invalid_argument(path + ": real number required");
}

const std::string& require_string(const Value& v, const std::string& path) {
    const auto* s = std::get_if<std::string>(&v.storage);
    if (!s) {
        throw std::invalid_argument(path + ": string required");
    }
    return *s;
}

const Value::Object& require_object(const Value& v, const std::string& path) {
    const auto* o = std::get_if<Value::Object>(&v.storage);
    if (!o) {
        throw std::invalid_argument(path + ": table/object required");
    }
    return *o;
}

Vec3 parse_vec3_or_default(const Value::Object& obj, std::string_view key, const Vec3& fallback,
                           const std::string& prefix) {
    const auto it = obj.find(std::string(key));
    if (it == obj.end()) {
        return fallback;
    }
    const std::string path = prefix + "." + std::string(key);
    const auto* arr = std::get_if<Value::Array>(&it->second.storage);
    if (!arr || arr->size() != 3) {
        throw std::invalid_argument(path + ": expected 3-element real array");
    }
    return {
        require_real((*arr)[0], path + "[0]"),
        require_real((*arr)[1], path + "[1]"),
        require_real((*arr)[2], path + "[2]"),
    };
}

Value vec3_to_value(const Vec3& v) {
    return Value{Value::Array{Value{v[0]}, Value{v[1]}, Value{v[2]}}};
}

} // namespace

std::int64_t require_exact_ticks(double duration_s, double step_s, std::int64_t min_ticks,
                                 const char* field_path) {
    if (!std::isfinite(duration_s) || !std::isfinite(step_s) || !(step_s > 0.)) {
        throw std::invalid_argument(std::string(field_path) + ": finite duration and positive step required");
    }
    const double ratio = duration_s / step_s;
    if (!std::isfinite(ratio) || ratio < static_cast<double>(min_ticks) - 1e-12 || ratio > 1e9) {
        throw std::invalid_argument(std::string(field_path) + ": duration below minimum ticks or out of range");
    }
    const double nearest = std::nearbyint(ratio);
    const double scale = std::max(1., std::abs(ratio));
    if (std::abs(ratio - nearest) > 1e-12 * scale) {
        throw std::invalid_argument(std::string(field_path) +
                                    ": must be an exact integer multiple of its clock period");
    }
    const auto ticks = static_cast<std::int64_t>(nearest);
    if (ticks < min_ticks) {
        throw std::invalid_argument(std::string(field_path) + ": below minimum tick count");
    }
    return ticks;
}

void validate_spatial_config(const SpatialConfig& config, double timestep_s) {
    if (!std::isfinite(timestep_s) || !(timestep_s > 0.)) {
        throw std::invalid_argument("spatial.timestep_s: positive finite timestep required");
    }
    if (!std::isfinite(config.corridor_half_width_m) || !(config.corridor_half_width_m >= 0.25)) {
        throw std::invalid_argument("spatial.corridor_half_width_m: must be >= 0.25 m");
    }
    if (!std::isfinite(config.capsize_roll_rad) || !(config.capsize_roll_rad >= 0.1) ||
        !(config.capsize_roll_rad < 0.5 * std::numbers::pi - kDefaultGimbalMarginRad)) {
        throw std::invalid_argument("spatial.capsize_roll_rad: must lie in [0.1, 80 deg)");
    }
    if (!std::isfinite(config.steer.range_deg) || !(config.steer.range_deg > 0.) ||
        !(config.steer.range_deg <= 75.)) {
        throw std::invalid_argument("spatial.steer.range_deg: must lie in (0, 75] deg");
    }
    if (!std::isfinite(config.steer.max_torque_nm) || !(config.steer.max_torque_nm > 0.)) {
        throw std::invalid_argument("spatial.steer.max_torque_nm: positive torque cap required");
    }
    (void)require_exact_ticks(config.steer.control_period_s, timestep_s, 1,
                              "spatial.steer.control_period_s");
    (void)require_exact_ticks(config.steer.reaction_delay_s, config.steer.control_period_s, 0,
                              "spatial.steer.reaction_delay_s");

    for (const double v : config.imu_mount_pos_m) {
        if (!std::isfinite(v)) throw std::invalid_argument("spatial.imu_mount_pos_m: nonfinite");
    }
    for (const double v : config.imu_mount_rpy_rad) {
        if (!std::isfinite(v)) throw std::invalid_argument("spatial.imu_mount_rpy_rad: nonfinite");
    }
    for (const double v : config.imu_misalignment_rad) {
        if (!std::isfinite(v)) throw std::invalid_argument("spatial.imu_misalignment_rad: nonfinite");
    }

    if (config.start_mode == SpatialStartMode::standstill) {
        if (!config.start_support.has_value()) {
            throw std::invalid_argument(
                "spatial.start_support: required when spatial.start_mode == 'standstill'");
        }
        const auto& s = *config.start_support;
        if (!std::isfinite(s.release_speed_mps) || !(s.release_speed_mps >= 0.5) ||
            !(s.release_speed_mps <= 4.0)) {
            throw std::invalid_argument("spatial.start_support.release_speed_mps: must lie in [0.5, 4.0]");
        }
        if (!std::isfinite(s.lateral_stiffness_n_per_m) || !(s.lateral_stiffness_n_per_m > 0.) ||
            !std::isfinite(s.lateral_damping_ns_per_m) || !(s.lateral_damping_ns_per_m >= 0.) ||
            !std::isfinite(s.roll_stiffness_nm_per_rad) || !(s.roll_stiffness_nm_per_rad >= 1500.) ||
            !std::isfinite(s.roll_damping_nms_per_rad) || !(s.roll_damping_nms_per_rad >= 0.)) {
            throw std::invalid_argument(
                "spatial.start_support: positive stiffness (roll >= 1500 N m/rad) and nonnegative damping required");
        }
    } else if (config.start_support.has_value()) {
        throw std::invalid_argument(
            "spatial.start_support: forbidden when spatial.start_mode == 'moving'");
    }
}

std::optional<SpatialConfig> parse_spatial_config(const Value::Object& root_object, double timestep_s) {
    const auto it = root_object.find("spatial");
    if (it == root_object.end()) {
        return std::nullopt;
    }
    const std::string prefix = "spatial";
    const auto& obj = require_object(it->second, prefix);
    reject_unknown_keys(obj,
                        {"tire_backend", "tire_preset", "start_mode", "corridor_half_width_m",
                         "capsize_roll_rad", "steer", "start_support", "imu_mount_pos_m",
                         "imu_mount_rpy_rad", "imu_misalignment_rad"},
                        prefix);

    SpatialConfig out{};
    const std::string& backend = require_string(require_field(obj, "tire_backend", prefix),
                                                "spatial.tire_backend");
    if (backend == "compliant_3d") {
        out.tire_backend = SpatialTireBackend::compliant_3d;
    } else if (backend == "distributed_3d") {
        out.tire_backend = SpatialTireBackend::distributed_3d;
    } else {
        throw std::invalid_argument("spatial.tire_backend: expected 'compliant_3d' or 'distributed_3d'");
    }

    const std::string& preset = require_string(require_field(obj, "tire_preset", prefix),
                                               "spatial.tire_preset");
    if (preset == "2p8_1p2bar") {
        out.tire_preset = SpatialTirePreset::mtb_2p8_1p2bar;
    } else if (preset == "2p4_standard") {
        out.tire_preset = SpatialTirePreset::mtb_2p4_standard;
    } else {
        throw std::invalid_argument("spatial.tire_preset: expected '2p8_1p2bar' or '2p4_standard'");
    }

    const std::string& mode = require_string(require_field(obj, "start_mode", prefix),
                                             "spatial.start_mode");
    if (mode == "moving") {
        out.start_mode = SpatialStartMode::moving;
    } else if (mode == "standstill") {
        out.start_mode = SpatialStartMode::standstill;
    } else {
        throw std::invalid_argument("spatial.start_mode: expected 'moving' or 'standstill'");
    }

    out.corridor_half_width_m = require_real(require_field(obj, "corridor_half_width_m", prefix),
                                             "spatial.corridor_half_width_m");
    out.capsize_roll_rad = require_real(require_field(obj, "capsize_roll_rad", prefix),
                                        "spatial.capsize_roll_rad");

    const std::string steer_prefix = "spatial.steer";
    const auto& steer_obj = require_object(require_field(obj, "steer", prefix), steer_prefix);
    reject_unknown_keys(steer_obj,
                        {"range_deg", "control_period_s", "reaction_delay_s", "max_torque_nm"},
                        steer_prefix);
    out.steer.range_deg = require_real(require_field(steer_obj, "range_deg", steer_prefix),
                                       "spatial.steer.range_deg");
    out.steer.control_period_s = require_real(
        require_field(steer_obj, "control_period_s", steer_prefix), "spatial.steer.control_period_s");
    out.steer.reaction_delay_s = require_real(
        require_field(steer_obj, "reaction_delay_s", steer_prefix), "spatial.steer.reaction_delay_s");
    out.steer.max_torque_nm = require_real(require_field(steer_obj, "max_torque_nm", steer_prefix),
                                           "spatial.steer.max_torque_nm");

    if (const auto sup_it = obj.find("start_support"); sup_it != obj.end()) {
        const std::string sup_prefix = "spatial.start_support";
        const auto& sup_obj = require_object(sup_it->second, sup_prefix);
        reject_unknown_keys(sup_obj,
                            {"release_speed_mps", "lateral_stiffness_n_per_m",
                             "lateral_damping_ns_per_m", "roll_stiffness_nm_per_rad",
                             "roll_damping_nms_per_rad"},
                            sup_prefix);
        SpatialStartSupportSpec sup{};
        sup.release_speed_mps = require_real(
            require_field(sup_obj, "release_speed_mps", sup_prefix),
            "spatial.start_support.release_speed_mps");
        sup.lateral_stiffness_n_per_m = require_real(
            require_field(sup_obj, "lateral_stiffness_n_per_m", sup_prefix),
            "spatial.start_support.lateral_stiffness_n_per_m");
        sup.lateral_damping_ns_per_m = require_real(
            require_field(sup_obj, "lateral_damping_ns_per_m", sup_prefix),
            "spatial.start_support.lateral_damping_ns_per_m");
        sup.roll_stiffness_nm_per_rad = require_real(
            require_field(sup_obj, "roll_stiffness_nm_per_rad", sup_prefix),
            "spatial.start_support.roll_stiffness_nm_per_rad");
        sup.roll_damping_nms_per_rad = require_real(
            require_field(sup_obj, "roll_damping_nms_per_rad", sup_prefix),
            "spatial.start_support.roll_damping_nms_per_rad");
        out.start_support = sup;
    }

    out.imu_mount_pos_m = parse_vec3_or_default(obj, "imu_mount_pos_m", {0., 0., 0.}, prefix);
    out.imu_mount_rpy_rad = parse_vec3_or_default(obj, "imu_mount_rpy_rad", {0., 0., 0.}, prefix);
    out.imu_misalignment_rad = parse_vec3_or_default(obj, "imu_misalignment_rad", {0., 0., 0.}, prefix);

    validate_spatial_config(out, timestep_s);
    return out;
}

Value spatial_config_to_value(const SpatialConfig& config) {
    std::string backend_str;
    switch (config.tire_backend) {
    case SpatialTireBackend::compliant_3d: backend_str = "compliant_3d"; break;
    case SpatialTireBackend::distributed_3d: backend_str = "distributed_3d"; break;
    }
    std::string preset_str;
    switch (config.tire_preset) {
    case SpatialTirePreset::mtb_2p8_1p2bar: preset_str = "2p8_1p2bar"; break;
    case SpatialTirePreset::mtb_2p4_standard: preset_str = "2p4_standard"; break;
    }
    std::string mode_str;
    switch (config.start_mode) {
    case SpatialStartMode::moving: mode_str = "moving"; break;
    case SpatialStartMode::standstill: mode_str = "standstill"; break;
    }

    Value::Object steer_obj{
        {"range_deg", Value{config.steer.range_deg}},
        {"control_period_s", Value{config.steer.control_period_s}},
        {"reaction_delay_s", Value{config.steer.reaction_delay_s}},
        {"max_torque_nm", Value{config.steer.max_torque_nm}},
    };
    Value::Object out{
        {"tire_backend", Value{std::move(backend_str)}},
        {"tire_preset", Value{std::move(preset_str)}},
        {"start_mode", Value{std::move(mode_str)}},
        {"corridor_half_width_m", Value{config.corridor_half_width_m}},
        {"capsize_roll_rad", Value{config.capsize_roll_rad}},
        {"steer", Value{std::move(steer_obj)}},
        {"imu_mount_pos_m", vec3_to_value(config.imu_mount_pos_m)},
        {"imu_mount_rpy_rad", vec3_to_value(config.imu_mount_rpy_rad)},
        {"imu_misalignment_rad", vec3_to_value(config.imu_misalignment_rad)},
    };
    if (config.start_support.has_value()) {
        const auto& s = *config.start_support;
        out["start_support"] = Value{Value::Object{
            {"release_speed_mps", Value{s.release_speed_mps}},
            {"lateral_stiffness_n_per_m", Value{s.lateral_stiffness_n_per_m}},
            {"lateral_damping_ns_per_m", Value{s.lateral_damping_ns_per_m}},
            {"roll_stiffness_nm_per_rad", Value{s.roll_stiffness_nm_per_rad}},
            {"roll_damping_nms_per_rad", Value{s.roll_damping_nms_per_rad}},
        }};
    }
    return Value{std::move(out)};
}

} // namespace bike::spatial
```

- [ ] **Step 4: Собрать и запустить `spatial_config`, закоммитить**

Run:
```bash
cmake --preset release && cmake --build build/native-release --target spatial_frames spatial_config && ctest --test-dir build/native-release -R '^spatial_(frames|config)$' --no-tests=error --output-on-failure
```
Expected: PASS (`2/2 tests passed`)

Commit:
```bash
git add CMakeLists.txt native/tests/spatial/CMakeLists.txt native/src/spatial/config.hpp native/src/spatial/config.cpp native/tests/spatial/test_config.cpp
git commit -m "feat(spatial): add strict SpatialConfig decoder with exact-tick validation"
```

---

### Task 3: Срез S1a — `Terrain3D` базовые типы, `ExtrudedTrack3d` и побитовый мост с 1D на всех рабочих трассах

**Files:**
- Create: `native/src/terrain/terrain3d_types.hpp`
- Create: `native/src/terrain/extrusion3d.hpp`
- Create: `native/src/terrain/extrusion3d.cpp`
- Modify: `native/cmake/NativeStartup.cmake`
- Modify: `native/tests/terrain/CMakeLists.txt`
- Create: `native/tests/terrain/test_extrusion3d.cpp`

**Interfaces:**
- Consumes: `bike::terrain::{TrackSpec, SurfaceMap, SignedProfile, build_profile, DEFAULT_RESOLUTION_M}`, `rider::SurfaceLabel`, `biketyre::np_interp`, `bike::io::Sha256`.
- Produces: `bike::terrain::{QueryStatus3d, TerrainSample3d, TorusClosestPoint3d, TerrainBounds3d, ExtrudedTrack3d}` в `bike_terrain`.

- [ ] **Step 1: Создать `terrain3d_types.hpp`, `extrusion3d.hpp` и `test_extrusion3d.cpp`**

Создать `native/src/terrain/terrain3d_types.hpp`:
```cpp
#pragma once

#include "rider/senses.hpp"

#include <array>
#include <cstdint>

namespace bike::terrain {

using Vec3d = std::array<double, 3>;

enum class QueryStatus3d : std::uint8_t {
    ok,
    outside,
    invalid_input,
};

struct TerrainBounds3d {
    double x_min_m = 0.;
    double x_max_m = 0.;
    double y_min_m = 0.;
    double y_max_m = 0.;
    double z_min_m = 0.;
    double z_max_m = 0.;
};

struct TerrainSample3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double height_m = 0.;
    Vec3d normal_world{0., 0., 1.};
    rider::SurfaceLabel surface = rider::SurfaceLabel::hardpack;
};

// Closest point on z = h(x, y) within [x - window, x + window] x [y - window, y + window]
// to a torus tube-section centre `center_world_m` (3D generalization of SignedProfile).
// `signed_distance_m` > 0 when center is above the surface, < 0 when below;
// torus indentation is `r_tube - signed_distance_m`.
struct TorusClosestPoint3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double signed_distance_m = 0.;
    Vec3d surface_point_world_m{};
    Vec3d normal_world{0., 0., 1.};
    rider::SurfaceLabel surface = rider::SurfaceLabel::hardpack;
};

} // namespace bike::terrain
```

Создать `native/src/terrain/extrusion3d.hpp`:
```cpp
#pragma once

#include "terrain/profile.hpp"
#include "terrain/terrain3d_types.hpp"
#include "terrain/types.hpp"

#include <string>
#include <vector>

namespace bike::terrain {

class ExtrudedTrack3d {
public:
    ExtrudedTrack3d(const TrackSpec& track, double half_width_m, double datum_z_offset_m = 0.,
                    double resolution_m = DEFAULT_RESOLUTION_M);

    [[nodiscard]] const TerrainBounds3d& bounds() const noexcept { return bounds_; }
    [[nodiscard]] const std::string& sha256_hex() const noexcept { return sha256_hex_; }
    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    struct LabelSpan {
        double start_m = 0.;
        double end_m = 0.;
        rider::SurfaceLabel label = rider::SurfaceLabel::hardpack;
    };

    [[nodiscard]] rider::SurfaceLabel label_at(double x_m) const noexcept;

    std::vector<double> xs_;
    std::vector<double> zs_;
    SignedProfile signed_profile_;
    rider::SurfaceLabel default_label_ = rider::SurfaceLabel::hardpack;
    std::vector<LabelSpan> sections_;
    TerrainBounds3d bounds_{};
    std::string sha256_hex_{};
};

} // namespace bike::terrain
```

Создать `native/tests/terrain/test_extrusion3d.cpp`:
```cpp
#include "support/fixture_test.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/trackfile.hpp"

#include <cmath>
#include <filesystem>

int main() {
    return bt::run([] {
        using namespace bike::terrain;

        // Repo-root research tracks (5 canonical tracks via relative path from fixture root ../..).
        const std::filesystem::path repo_root = bt::fixture("config").parent_path().parent_path();
        const std::array<const char*, 5> track_files{
            "examples/research/rough_uphill.toml",
            "examples/research/rough_uphill_extreme.toml",
            "examples/research/rough_uphill_savage.toml",
            "examples/research/alpine_scree.toml",
            "examples/research/alpine_rocky.toml",
        };

        for (const char* rel : track_files) {
            const TrackSpec spec = load_track_spec(repo_root / rel);
            const ExtrudedTrack3d ext(spec, 1.5, -0.402);
            bt::require(ext.sha256_hex().size() == 64, "64-hex SHA-256 digest");

            // Check bitwise height parity with 1D biketyre::np_interp and Y-invariance across [-1.5, 1.5].
            const auto pts1d = profile(spec, DEFAULT_RESOLUTION_M);
            std::vector<double> xs(pts1d.size()), zs(pts1d.size());
            for (std::size_t i = 0; i < pts1d.size(); ++i) {
                xs[i] = pts1d[i][0];
                zs[i] = pts1d[i][1] - 0.402;
            }
            const SurfaceMap smap = spec.surface_map();

            for (double x : {0.0, 1.234, 12.5, spec.length_m * 0.5, spec.length_m - 0.321}) {
                if (x > spec.length_m) continue;
                const double expected_z = biketyre::np_interp(x, xs, zs);
                const auto expected_lbl = rider::surface_label(smap.at(x).name);
                bt::require(expected_lbl.has_value(), "known surface label");

                for (double y : {-1.5, -0.37, 0.0, 0.89, 1.5}) {
                    const TerrainSample3d s = ext.sample(x, y);
                    bt::require(s.status == QueryStatus3d::ok, "sample inside corridor ok");
                    bt::require(s.height_m == expected_z, "bitwise height match with 1D np_interp");
                    bt::require(s.normal_world[1] == 0.0, "zero lateral normal on extruded track");
                    bt::require(s.surface == *expected_lbl, "surface section preserved (including scree)");
                }
            }

            bt::require(ext.sample(-0.01, 0.0).status == QueryStatus3d::outside, "x < 0 outside");
            bt::require(ext.sample(1.0, 1.51).status == QueryStatus3d::outside, "|y| > half_width outside");

            // Check closest_point parity with 1D SignedProfile on sharp obstacles (e.g. SquareEdge).
            std::vector<Point> verts(xs.size());
            for (std::size_t i = 0; i < xs.size(); ++i) verts[i] = {xs[i], zs[i]};
            const SignedProfile sp(std::move(verts));
            const Point probe_xz{spec.length_m * 0.4, zs[zs.size() * 2 / 5] + 0.035};
            const ProfileDistance d1 = sp.query(std::span<const Point>(&probe_xz, 1));
            const TorusClosestPoint3d d3 = ext.closest_point({probe_xz[0], -0.42, probe_xz[1]}, 0.08);
            bt::require(d3.status == QueryStatus3d::ok, "closest_point ok");
            bt::require(std::abs(d3.signed_distance_m - d1.distance[0]) <= 1e-14,
                        "3D closest_point matches 1D SignedProfile distance");
            bt::require(std::abs(d3.normal_world[0] - d1.normal[0][0]) <= 1e-14 &&
                            d3.normal_world[1] == 0.0 &&
                            std::abs(d3.normal_world[2] - d1.normal[0][1]) <= 1e-14,
                        "3D closest_point normal matches 1D SignedProfile normal");
        }
    });
}
```

- [ ] **Step 2: Зарегистрировать `extrusion3d.cpp` в `NativeStartup.cmake` и `terrain_extrusion3d` в `native/tests/terrain/CMakeLists.txt`**

В `native/cmake/NativeStartup.cmake` добавить `"${native_dir}/src/terrain/extrusion3d.cpp"` в `bike_terrain` и `bike_io` в `target_link_libraries(bike_terrain PUBLIC ...)`:
```cmake
  add_library(bike_terrain STATIC
    "${native_dir}/src/terrain/profile.cpp"
    "${native_dir}/src/terrain/pothole_report.cpp"
    "${native_dir}/src/terrain/heightfield.cpp"
    "${native_dir}/src/terrain/surface.cpp"
    "${native_dir}/src/terrain/obstacles.cpp"
    "${native_dir}/src/terrain/contact_safety.cpp"
    "${native_dir}/src/terrain/wheelpath.cpp"
    "${native_dir}/src/terrain/placement.cpp"
    "${native_dir}/src/terrain/road.cpp"
    "${native_dir}/src/terrain/generator.cpp"
    "${native_dir}/src/terrain/alpine.cpp"
    "${native_dir}/src/terrain/presets.cpp"
    "${native_dir}/src/terrain/trackfile.cpp"
    "${native_dir}/src/terrain/extrusion3d.cpp")
  bike_startup_target_policy(bike_terrain "${native_dir}")
  target_link_libraries(bike_terrain PUBLIC bike_configuration bike_geometry bike_random bike_io)
```

В `native/tests/terrain/CMakeLists.txt` добавить `terrain_extrusion3d` в `foreach(_terrain_test IN ITEMS ...)`.

Run: `cmake --preset release`
Expected: FAIL (`Cannot find source file: .../native/src/terrain/extrusion3d.cpp`)

- [ ] **Step 3: Написать `native/src/terrain/extrusion3d.cpp`**

```cpp
#include "terrain/extrusion3d.hpp"

#include "io/hash.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace bike::terrain {
namespace {

void hash_f64(bike::io::Sha256& h, double v) noexcept {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(v == 0. ? 0. : v);
    std::array<std::byte, 8> le{};
    for (std::size_t i = 0; i < 8; ++i) {
        le[i] = static_cast<std::byte>((bits >> (8U * i)) & 0xffU);
    }
    h.update(le);
}

void hash_u8(bike::io::Sha256& h, std::uint8_t v) noexcept {
    const std::byte b = static_cast<std::byte>(v);
    h.update(std::span<const std::byte>(&b, 1));
}

std::string hex_digest(const std::array<std::uint8_t, 32>& digest) {
    static constexpr std::array<char, 16> kHex{
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
    };
    std::string out(64, '0');
    for (std::size_t i = 0; i < 32; ++i) {
        out[2 * i] = kHex[(digest[i] >> 4U) & 0x0fU];
        out[2 * i + 1] = kHex[digest[i] & 0x0fU];
    }
    return out;
}

std::vector<Point> build_shifted_vertices(const TrackSpec& track, double datum_z_offset_m,
                                          double resolution_m) {
    track.validate();
    if (!std::isfinite(datum_z_offset_m) || !std::isfinite(resolution_m) || !(resolution_m > 0.)) {
        throw std::invalid_argument("ExtrudedTrack3d: finite datum and positive resolution required");
    }
    std::vector<Point> pts = profile(track, resolution_m);
    for (Point& p : pts) {
        p[1] += datum_z_offset_m;
    }
    return pts;
}

} // namespace

ExtrudedTrack3d::ExtrudedTrack3d(const TrackSpec& track, double half_width_m,
                                 double datum_z_offset_m, double resolution_m)
    : signed_profile_(build_shifted_vertices(track, datum_z_offset_m, resolution_m)) {
    if (!std::isfinite(half_width_m) || !(half_width_m > 0.)) {
        throw std::invalid_argument("ExtrudedTrack3d: positive finite half_width_m required");
    }
    const auto& verts = signed_profile_.vertices();
    xs_.resize(verts.size());
    zs_.resize(verts.size());
    double z_min = verts.front()[1];
    double z_max = verts.front()[1];
    for (std::size_t i = 0; i < verts.size(); ++i) {
        xs_[i] = verts[i][0];
        zs_[i] = verts[i][1];
        z_min = std::min(z_min, zs_[i]);
        z_max = std::max(z_max, zs_[i]);
    }

    const auto def_lbl = rider::surface_label(track.surface);
    if (!def_lbl.has_value()) {
        throw std::invalid_argument("ExtrudedTrack3d: unknown default surface '" + track.surface + "'");
    }
    default_label_ = *def_lbl;

    const SurfaceMap smap = track.surface_map();
    sections_.reserve(smap.sections().size());
    for (const SurfaceSection& sec : smap.sections()) {
        const auto lbl = rider::surface_label(sec.surface);
        if (!lbl.has_value()) {
            throw std::invalid_argument("ExtrudedTrack3d: unknown section surface '" + sec.surface + "'");
        }
        sections_.push_back(LabelSpan{
            .start_m = sec.start_m,
            .end_m = sec.end_m,
            .label = *lbl,
        });
    }

    bounds_ = TerrainBounds3d{
        .x_min_m = xs_.front(),
        .x_max_m = xs_.back(),
        .y_min_m = -half_width_m,
        .y_max_m = half_width_m,
        .z_min_m = z_min,
        .z_max_m = z_max,
    };

    bike::io::Sha256 hasher;
    hash_u8(hasher, 1U);
    hash_f64(hasher, half_width_m);
    hash_u8(hasher, static_cast<std::uint8_t>(default_label_));
    for (std::size_t i = 0; i < xs_.size(); ++i) {
        hash_f64(hasher, xs_[i]);
        hash_f64(hasher, zs_[i]);
    }
    for (const LabelSpan& sec : sections_) {
        hash_f64(hasher, sec.start_m);
        hash_f64(hasher, sec.end_m);
        hash_u8(hasher, static_cast<std::uint8_t>(sec.label));
    }
    sha256_hex_ = hex_digest(hasher.finish());
}

rider::SurfaceLabel ExtrudedTrack3d::label_at(double x_m) const noexcept {
    rider::SurfaceLabel out = default_label_;
    for (const LabelSpan& sec : sections_) {
        if (x_m >= sec.start_m && x_m < sec.end_m) {
            out = sec.label;
        }
    }
    return out;
}

TerrainSample3d ExtrudedTrack3d::sample(double x_m, double y_m) const noexcept {
    if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    if (x_m < bounds_.x_min_m || x_m > bounds_.x_max_m ||
        y_m < bounds_.y_min_m || y_m > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }
    const double h = biketyre::np_interp(x_m, xs_, zs_);
    const auto upper = std::ranges::upper_bound(xs_, x_m);
    std::size_t seg = 0;
    if (upper == xs_.begin()) {
        seg = 0;
    } else if (upper == xs_.end()) {
        seg = xs_.size() - 2;
    } else {
        seg = static_cast<std::size_t>(upper - xs_.begin()) - 1;
    }
    const double dx = xs_[seg + 1] - xs_[seg];
    const double dz = zs_[seg + 1] - zs_[seg];
    const double len = std::hypot(dx, dz);
    const Vec3d normal = len > 0. ? Vec3d{-dz / len, 0., dx / len} : Vec3d{0., 0., 1.};
    return TerrainSample3d{
        .status = QueryStatus3d::ok,
        .height_m = h,
        .normal_world = normal,
        .surface = label_at(x_m),
    };
}

TorusClosestPoint3d ExtrudedTrack3d::closest_point(const Vec3d& center_world_m,
                                                   double search_radius_m) const noexcept {
    if (!std::isfinite(center_world_m[0]) || !std::isfinite(center_world_m[1]) ||
        !std::isfinite(center_world_m[2]) || !std::isfinite(search_radius_m) ||
        !(search_radius_m > 0.)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    if (center_world_m[0] < bounds_.x_min_m || center_world_m[0] > bounds_.x_max_m ||
        center_world_m[1] < bounds_.y_min_m || center_world_m[1] > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }
    try {
        const Point query_xz{center_world_m[0], center_world_m[2]};
        const ProfileDistance res = signed_profile_.query(std::span<const Point>(&query_xz, 1));
        const double px = res.point[0][0];
        return TorusClosestPoint3d{
            .status = QueryStatus3d::ok,
            .signed_distance_m = res.distance[0],
            .surface_point_world_m = {px, center_world_m[1], res.point[0][1]},
            .normal_world = {res.normal[0][0], 0., res.normal[0][1]},
            .surface = label_at(px),
        };
    } catch (...) {
        return {.status = QueryStatus3d::invalid_input};
    }
}

} // namespace bike::terrain
```

- [ ] **Step 4: Собрать и запустить `terrain_extrusion3d`, закоммитить**

Run:
```bash
cmake --preset release && cmake --build build/native-release --target terrain_extrusion3d && ctest --test-dir build/native-release -R '^terrain_extrusion3d$' --no-tests=error --output-on-failure
```
Expected: PASS (`1/1 Test ... terrain_extrusion3d ... Passed`)

Commit:
```bash
git add native/cmake/NativeStartup.cmake native/tests/terrain/CMakeLists.txt native/src/terrain/terrain3d_types.hpp native/src/terrain/extrusion3d.hpp native/src/terrain/extrusion3d.cpp native/tests/terrain/test_extrusion3d.cpp
git commit -m "feat(terrain): add ExtrudedTrack3d with bitwise 1D parity across all tracks"
```

---

### Task 4: Срез S1b — кэш-локальный `Grid3d` (`ix * ny + iy`), монотонная бикубика Стивенса (`0%` overshoot) и `Terrain3dSource`

**Files:**
- Create: `native/src/terrain/interpolation3d.hpp`
- Create: `native/src/terrain/interpolation3d.cpp`
- Create: `native/src/terrain/grid3d.hpp`
- Create: `native/src/terrain/grid3d.cpp`
- Create: `native/src/terrain/source3d.hpp`
- Create: `native/src/terrain/source3d.cpp`
- Modify: `native/cmake/NativeStartup.cmake`
- Modify: `native/tests/terrain/CMakeLists.txt`
- Create: `native/tests/terrain/test_grid3d.cpp`

**Interfaces:**
- Consumes: `bike::terrain::{TerrainSample3d, TorusClosestPoint3d, TerrainBounds3d, ExtrudedTrack3d}`, `bike::io::Sha256`.
- Produces: `bike::terrain::{MonotoneBicubicCell, evaluate_monotone_bicubic, Grid3d, Terrain3dSource, Terrain3dQueryView}`.

- [ ] **Step 1: Создать заголовки `interpolation3d.hpp`, `grid3d.hpp`, `source3d.hpp` и тест `test_grid3d.cpp`**

Создать `native/src/terrain/interpolation3d.hpp`:
```cpp
#pragma once

#include <array>

namespace bike::terrain {

struct BicubicEval3d {
    double height_m = 0.;
    double dz_dx = 0.;
    double dz_dy = 0.;
};

// Evaluates a C^1 monotone bicubic Hermite patch over normalized cell coordinates
// u, v in [0, 1] from a 4x4 stencil z[ix_local][iy_local] (ix_local, iy_local in 0..3,
// cell spans stencil indices [1..2] x [1..2]) using Steffen (1990) slope limiting.
// Guarantees zero Gibbs overshoot on steps/plateaus and exact reproduction of tilted planes.
[[nodiscard]] BicubicEval3d evaluate_steffen_bicubic(
    const std::array<std::array<double, 4>, 4>& stencil_xy,
    double dx_m, double dy_m, double u, double v) noexcept;

} // namespace bike::terrain
```

Создать `native/src/terrain/grid3d.hpp`:
```cpp
#pragma once

#include "terrain/terrain3d_types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace bike::terrain {

struct Grid3dSpec {
    std::size_t nx = 0;
    std::size_t ny = 0;
    double x0_m = 0.;
    double y0_m = 0.;
    double dx_m = 0.;
    double dy_m = 0.;
};

class Grid3d {
public:
    // `elevations_xy` has length nx * ny in transverse-slice order: index = ix * ny + iy.
    // `cell_surfaces_xy` has length (nx - 1) * (ny - 1) in order: cell = ix * (ny - 1) + iy.
    Grid3d(Grid3dSpec spec, std::vector<double> elevations_xy,
           std::vector<rider::SurfaceLabel> cell_surfaces_xy);

    [[nodiscard]] const Grid3dSpec& spec() const noexcept { return spec_; }
    [[nodiscard]] const TerrainBounds3d& bounds() const noexcept { return bounds_; }
    [[nodiscard]] const std::string& sha256_hex() const noexcept { return sha256_hex_; }
    [[nodiscard]] std::span<const double> elevations() const noexcept { return elevations_xy_; }

    [[nodiscard]] double node_z(std::size_t ix, std::size_t iy) const noexcept {
        return elevations_xy_[ix * spec_.ny + iy];
    }

    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    struct CellZRange {
        double z_min = 0.;
        double z_max = 0.;
    };

    Grid3dSpec spec_{};
    std::vector<double> elevations_xy_;
    std::vector<rider::SurfaceLabel> cell_surfaces_xy_;
    std::vector<CellZRange> cell_z_ranges_;
    TerrainBounds3d bounds_{};
    std::string sha256_hex_{};
};

} // namespace bike::terrain
```

Создать `native/src/terrain/source3d.hpp`:
```cpp
#pragma once

#include "terrain/extrusion3d.hpp"
#include "terrain/grid3d.hpp"

#include <string_view>
#include <variant>

namespace bike::terrain {

class Terrain3dSource;

class Terrain3dQueryView {
public:
    Terrain3dQueryView() noexcept = default;
    explicit Terrain3dQueryView(const Terrain3dSource* owner) noexcept : owner_(owner) {}

    [[nodiscard]] bool valid() const noexcept { return owner_ != nullptr; }
    [[nodiscard]] TerrainBounds3d bounds() const noexcept;
    [[nodiscard]] std::string_view sha256_hex() const noexcept;
    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    const Terrain3dSource* owner_ = nullptr;
};

class Terrain3dSource {
public:
    using Storage = std::variant<ExtrudedTrack3d, Grid3d>;

    explicit Terrain3dSource(ExtrudedTrack3d extruded) : storage_(std::move(extruded)) {}
    explicit Terrain3dSource(Grid3d grid) : storage_(std::move(grid)) {}

    Terrain3dSource(const Terrain3dSource&) = delete;
    Terrain3dSource& operator=(const Terrain3dSource&) = delete;
    Terrain3dSource(Terrain3dSource&&) = delete;
    Terrain3dSource& operator=(Terrain3dSource&&) = delete;
    ~Terrain3dSource() = default;

    [[nodiscard]] Terrain3dQueryView view() const noexcept { return Terrain3dQueryView(this); }
    [[nodiscard]] const TerrainBounds3d& bounds() const noexcept;
    [[nodiscard]] const std::string& sha256_hex() const noexcept;
    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    Storage storage_;
};

} // namespace bike::terrain
```

Создать `native/tests/terrain/test_grid3d.cpp`:
```cpp
#include "support/fixture_test.hpp"
#include "terrain/source3d.hpp"

#include <cmath>
#include <vector>

int main() {
    return bt::run([] {
        using namespace bike::terrain;

        // 1. Exact reproduction of tilted plane z = 0.18 * x - 0.09 * y + 0.5.
        const Grid3dSpec plane_spec{.nx = 41, .ny = 21, .x0_m = 0., .y0_m = -1., .dx_m = 0.05, .dy_m = 0.1};
        std::vector<double> plane_z(plane_spec.nx * plane_spec.ny);
        for (std::size_t ix = 0; ix < plane_spec.nx; ++ix) {
            const double x = plane_spec.x0_m + static_cast<double>(ix) * plane_spec.dx_m;
            for (std::size_t iy = 0; iy < plane_spec.ny; ++iy) {
                const double y = plane_spec.y0_m + static_cast<double>(iy) * plane_spec.dy_m;
                plane_z[ix * plane_spec.ny + iy] = 0.18 * x - 0.09 * y + 0.5;
            }
        }
        std::vector<rider::SurfaceLabel> plane_surf((plane_spec.nx - 1) * (plane_spec.ny - 1),
                                                    rider::SurfaceLabel::scree);
        const Terrain3dSource plane_src(Grid3d(plane_spec, std::move(plane_z), std::move(plane_surf)));
        const Terrain3dQueryView view = plane_src.view();
        const double inv_norm = 1. / std::sqrt(1. + 0.18 * 0.18 + 0.09 * 0.09);
        for (double x : {0.173, 0.85, 1.42}) {
            for (double y : {-0.73, 0.0, 0.64}) {
                const TerrainSample3d s = view.sample(x, y);
                bt::require(s.status == QueryStatus3d::ok, "plane sample ok");
                bt::require(std::abs(s.height_m - (0.18 * x - 0.09 * y + 0.5)) <= 1e-13,
                            "Steffen bicubic reproduces tilted plane to machine precision");
                bt::require(std::abs(s.normal_world[0] - (-0.18 * inv_norm)) <= 1e-12 &&
                                std::abs(s.normal_world[1] - (0.09 * inv_norm)) <= 1e-12 &&
                                std::abs(s.normal_world[2] - inv_norm) <= 1e-12,
                            "plane normal exact");
                bt::require(s.surface == rider::SurfaceLabel::scree, "surface preserved");
            }
        }

        // 2. Regression against 3d-new Gibbs overshoot on 100 mm SquareEdge at dx = 5 mm AND 140 mm Pothole!
        const Grid3dSpec step_spec{.nx = 201, .ny = 11, .x0_m = 0., .y0_m = -0.1, .dx_m = 0.005, .dy_m = 0.02};
        std::vector<double> step_z(step_spec.nx * step_spec.ny, 0.0);
        for (std::size_t ix = 0; ix < step_spec.nx; ++ix) {
            const double x = static_cast<double>(ix) * step_spec.dx_m;
            double z = 0.0;
            if (x >= 0.30 && x <= 0.55) {
                z = 0.100; // 100 mm SquareEdge (slope = 20.0 >> tan(60 deg)!)
            } else if (x >= 0.70 && x <= 0.85) {
                z = -0.140; // 140 mm sharp Pothole
            }
            for (std::size_t iy = 0; iy < step_spec.ny; ++iy) {
                step_z[ix * step_spec.ny + iy] = z;
            }
        }
        std::vector<rider::SurfaceLabel> step_surf((step_spec.nx - 1) * (step_spec.ny - 1),
                                                   rider::SurfaceLabel::asphalt);
        const Grid3d step_grid(step_spec, std::move(step_z), std::move(step_surf));

        // Sample every 0.5 mm across both plateaus and transitions: ZERO overshoot/undershoot!
        for (int k = 0; k <= 2000; ++k) {
            const double x = 0.0005 * static_cast<double>(k);
            const TerrainSample3d s = step_grid.sample(x, 0.0);
            bt::require(s.status == QueryStatus3d::ok, "step sample ok");
            if (x <= 0.60) {
                bt::require(s.height_m >= -1e-15 && s.height_m <= 0.100 + 1e-15,
                            "zero Gibbs pit/lip around 100 mm SquareEdge");
            } else {
                bt::require(s.height_m >= -0.140 - 1e-15 && s.height_m <= 1e-15,
                            "zero Gibbs overshoot around 140 mm Pothole");
            }
        }

        // 3. C^1 seam continuity across cell boundary and frontal -Fx normal on 100 mm SquareEdge!
        const TerrainSample3d left_seam = step_grid.sample(0.2975 - 1e-9, 0.01);
        const TerrainSample3d right_seam = step_grid.sample(0.2975 + 1e-9, 0.01);
        bt::require(std::abs(left_seam.normal_world[0] - right_seam.normal_world[0]) <= 1e-6,
                    "continuous C^1 normal across cell boundary");

        // Torus station at (x = 0.275 m, z = 0.055 m) approaching the 100 mm step at x = 0.300 m:
        // must see the step face with negative normal_x (frontal impact resistance), never origin_below_surface!
        const TorusClosestPoint3d hit = step_grid.closest_point({0.275, 0.0, 0.055}, 0.06);
        bt::require(hit.status == QueryStatus3d::ok, "closest_point near 100 mm step succeeds");
        bt::require(hit.signed_distance_m < 0.035 && hit.normal_world[0] < -0.5,
                    "torus station sees frontal -X normal of 100 mm SquareEdge");
    });
}
```

- [ ] **Step 2: Добавить новые файлы в `NativeStartup.cmake` и `native/tests/terrain/CMakeLists.txt`**

В `native/cmake/NativeStartup.cmake` добавить в `bike_terrain`:
```cmake
    "${native_dir}/src/terrain/interpolation3d.cpp"
    "${native_dir}/src/terrain/grid3d.cpp"
    "${native_dir}/src/terrain/source3d.cpp"
```
В `native/tests/terrain/CMakeLists.txt` добавить `terrain_grid3d` в `foreach(_terrain_test IN ITEMS ...)`.

Run: `cmake --preset release`
Expected: FAIL (`Cannot find source file: .../interpolation3d.cpp`)

- [ ] **Step 3: Реализовать `interpolation3d.cpp`, `grid3d.cpp` и `source3d.cpp`**

Создать `native/src/terrain/interpolation3d.cpp`:
```cpp
#include "terrain/interpolation3d.hpp"

#include <algorithm>
#include <cmath>

namespace bike::terrain {
namespace {

// Steffen (1990), "A simple method for monotonic interpolation in one dimension",
// Astron. Astrophys. 239, 443-450: C^1 monotone slope at interior node 1 from (y0, y1, y2)
// on uniform spacing h. Zero slope at extrema and adjacent to flat plateaus.
[[nodiscard]] double steffen_slope(double y0, double y1, double y2, double inv_h) noexcept {
    const double s0 = (y1 - y0) * inv_h;
    const double s1 = (y2 - y1) * inv_h;
    if (s0 * s1 <= 0.) {
        return 0.;
    }
    const double p = 0.5 * (s0 + s1);
    const double bound = 2. * std::min(std::abs(s0), std::abs(s1));
    return std::copysign(std::min(std::abs(p), bound), p);
}

struct Hermite1d {
    double val = 0.;
    double deriv = 0.;
};

[[nodiscard]] Hermite1d eval_cubic_hermite(double f0, double f1, double d0_scaled,
                                           double d1_scaled, double t) noexcept {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2. * t3 - 3. * t2 + 1.;
    const double h10 = t3 - 2. * t2 + t;
    const double h01 = -2. * t3 + 3. * t2;
    const double h11 = t3 - t2;

    const double dh00 = 6. * (t2 - t);
    const double dh10 = 3. * t2 - 4. * t + 1.;
    const double dh01 = 6. * (t - t2);
    const double dh11 = 3. * t2 - 2. * t;

    return Hermite1d{
        .val = h00 * f0 + h10 * d0_scaled + h01 * f1 + h11 * d1_scaled,
        .deriv = dh00 * f0 + dh10 * d0_scaled + dh01 * f1 + dh11 * d1_scaled,
    };
}

} // namespace

BicubicEval3d evaluate_steffen_bicubic(
    const std::array<std::array<double, 4>, 4>& z,
    double dx_m, double dy_m, double u, double v) noexcept {
    const double inv_dx = 1. / dx_m;
    const double inv_dy = 1. / dy_m;

    // Step 1: Along X for each of the 4 Y-columns (iy = 0..3), evaluate height and dz/dx at u.
    std::array<double, 4> col_val{};
    std::array<double, 4> col_dzdx{};
    for (std::size_t iy = 0; iy < 4; ++iy) {
        const double sx1 = steffen_slope(z[0][iy], z[1][iy], z[2][iy], inv_dx);
        const double sx2 = steffen_slope(z[1][iy], z[2][iy], z[3][iy], inv_dx);
        const Hermite1d hx = eval_cubic_hermite(z[1][iy], z[2][iy], sx1 * dx_m, sx2 * dx_m, u);
        col_val[iy] = hx.val;
        col_dzdx[iy] = hx.deriv * inv_dx;
    }

    // Step 2: Monotone interpolation along Y of col_val(v) -> height and dz/dy.
    const double sy1 = steffen_slope(col_val[0], col_val[1], col_val[2], inv_dy);
    const double sy2 = steffen_slope(col_val[1], col_val[2], col_val[3], inv_dy);
    const Hermite1d hy = eval_cubic_hermite(col_val[1], col_val[2], sy1 * dy_m, sy2 * dy_m, v);

    // Exact dz/dx of the separable monotone surface: differentiate col_val[iy](u) through
    // the active slope limiter along Y, or evaluate smooth Hermite of col_dzdx along Y.
    // Using finite-free Hermite of col_dzdx with Steffen slopes preserves C^1 continuity across seams:
    const double dsy1 = steffen_slope(col_dzdx[0], col_dzdx[1], col_dzdx[2], inv_dy);
    const double dsy2 = steffen_slope(col_dzdx[1], col_dzdx[2], col_dzdx[3], inv_dy);
    const Hermite1d hy_dx = eval_cubic_hermite(col_dzdx[1], col_dzdx[2], dsy1 * dy_m, dsy2 * dy_m, v);

    return BicubicEval3d{
        .height_m = hy.val,
        .dz_dx = hy_dx.val,
        .dz_dy = hy.deriv * inv_dy,
    };
}

} // namespace bike::terrain
```

Создать `native/src/terrain/grid3d.cpp`:
```cpp
#include "terrain/grid3d.hpp"

#include "io/hash.hpp"
#include "terrain/interpolation3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::terrain {
namespace {

void hash_u64(bike::io::Sha256& h, std::uint64_t v) noexcept {
    std::array<std::byte, 8> le{};
    for (std::size_t i = 0; i < 8; ++i) {
        le[i] = static_cast<std::byte>((v >> (8U * i)) & 0xffU);
    }
    h.update(le);
}

void hash_f64(bike::io::Sha256& h, double v) noexcept {
    hash_u64(h, std::bit_cast<std::uint64_t>(v == 0. ? 0. : v));
}

std::string hex_digest(const std::array<std::uint8_t, 32>& digest) {
    static constexpr std::array<char, 16> kHex{
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
    };
    std::string out(64, '0');
    for (std::size_t i = 0; i < 32; ++i) {
        out[2 * i] = kHex[(digest[i] >> 4U) & 0x0fU];
        out[2 * i + 1] = kHex[digest[i] & 0x0fU];
    }
    return out;
}

[[nodiscard]] std::size_t clamped_index(std::ptrdiff_t idx, std::size_t count) noexcept {
    if (idx <= 0) return 0;
    const auto u = static_cast<std::size_t>(idx);
    return u >= count ? count - 1 : u;
}

} // namespace

Grid3d::Grid3d(Grid3dSpec spec, std::vector<double> elevations_xy,
               std::vector<rider::SurfaceLabel> cell_surfaces_xy)
    : spec_(spec),
      elevations_xy_(std::move(elevations_xy)),
      cell_surfaces_xy_(std::move(cell_surfaces_xy)) {
    if (spec_.nx < 4 || spec_.ny < 4 || spec_.nx > 500000 || spec_.ny > 10000) {
        throw std::invalid_argument("Grid3d: nx and ny must be >= 4 and within bounded limits");
    }
    if (!std::isfinite(spec_.x0_m) || !std::isfinite(spec_.y0_m) ||
        !std::isfinite(spec_.dx_m) || !std::isfinite(spec_.dy_m) ||
        !(spec_.dx_m > 0.) || !(spec_.dy_m > 0.)) {
        throw std::invalid_argument("Grid3d: finite origin and positive grid spacings required");
    }
    // Verify strictly increasing binary64 node coordinates on both axes.
    for (std::size_t ix = 1; ix < spec_.nx; ++ix) {
        const double prev = spec_.x0_m + static_cast<double>(ix - 1) * spec_.dx_m;
        const double curr = spec_.x0_m + static_cast<double>(ix) * spec_.dx_m;
        if (!(curr > prev) || !std::isfinite(curr)) {
            throw std::invalid_argument("Grid3d: collapsed or nonfinite X coordinate progression");
        }
    }
    for (std::size_t iy = 1; iy < spec_.ny; ++iy) {
        const double prev = spec_.y0_m + static_cast<double>(iy - 1) * spec_.dy_m;
        const double curr = spec_.y0_m + static_cast<double>(iy) * spec_.dy_m;
        if (!(curr > prev) || !std::isfinite(curr)) {
            throw std::invalid_argument("Grid3d: collapsed or nonfinite Y coordinate progression");
        }
    }

    const std::size_t expected_nodes = spec_.nx * spec_.ny;
    const std::size_t expected_cells = (spec_.nx - 1) * (spec_.ny - 1);
    if (elevations_xy_.size() != expected_nodes || cell_surfaces_xy_.size() != expected_cells) {
        throw std::invalid_argument("Grid3d: elevation or surface array size mismatch");
    }

    double z_min = elevations_xy_.front();
    double z_max = elevations_xy_.front();
    for (const double z : elevations_xy_) {
        if (!std::isfinite(z)) {
            throw std::invalid_argument("Grid3d: nonfinite elevation value");
        }
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
    }

    // Because Steffen interpolation is strictly monotone within cell corner bounds,
    // the exact cell Z-AABB equals min/max of its 4 corners!
    cell_z_ranges_.resize(expected_cells);
    for (std::size_t ix = 0; ix + 1 < spec_.nx; ++ix) {
        for (std::size_t iy = 0; iy + 1 < spec_.ny; ++iy) {
            const double z00 = node_z(ix, iy);
            const double z10 = node_z(ix + 1, iy);
            const double z01 = node_z(ix, iy + 1);
            const double z11 = node_z(ix + 1, iy + 1);
            cell_z_ranges_[ix * (spec_.ny - 1) + iy] = CellZRange{
                .z_min = std::min({z00, z10, z01, z11}),
                .z_max = std::max({z00, z10, z01, z11}),
            };
        }
    }

    bounds_ = TerrainBounds3d{
        .x_min_m = spec_.x0_m,
        .x_max_m = spec_.x0_m + static_cast<double>(spec_.nx - 1) * spec_.dx_m,
        .y_min_m = spec_.y0_m,
        .y_max_m = spec_.y0_m + static_cast<double>(spec_.ny - 1) * spec_.dy_m,
        .z_min_m = z_min,
        .z_max_m = z_max,
    };

    // Streaming SHA-256 without copying a second multi-MB buffer.
    bike::io::Sha256 hasher;
    hash_u64(hasher, 2U);
    hash_u64(hasher, spec_.nx);
    hash_u64(hasher, spec_.ny);
    hash_f64(hasher, spec_.x0_m);
    hash_f64(hasher, spec_.y0_m);
    hash_f64(hasher, spec_.dx_m);
    hash_f64(hasher, spec_.dy_m);
    for (const double z : elevations_xy_) {
        hash_f64(hasher, z);
    }
    for (const rider::SurfaceLabel s : cell_surfaces_xy_) {
        const auto b = static_cast<std::byte>(s);
        hasher.update(std::span<const std::byte>(&b, 1));
    }
    sha256_hex_ = hex_digest(hasher.finish());
}

TerrainSample3d Grid3d::sample(double x_m, double y_m) const noexcept {
    if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    if (x_m < bounds_.x_min_m || x_m > bounds_.x_max_m ||
        y_m < bounds_.y_min_m || y_m > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }

    const double fx = (x_m - spec_.x0_m) / spec_.dx_m;
    const double fy = (y_m - spec_.y0_m) / spec_.dy_m;
    const std::size_t ix = std::min(static_cast<std::size_t>(std::max(0., std::floor(fx))), spec_.nx - 2);
    const std::size_t iy = std::min(static_cast<std::size_t>(std::max(0., std::floor(fy))), spec_.ny - 2);
    const double u = std::clamp(fx - static_cast<double>(ix), 0., 1.);
    const double v = std::clamp(fy - static_cast<double>(iy), 0., 1.);

    std::array<std::array<double, 4>, 4> stencil{};
    const auto ix_base = static_cast<std::ptrdiff_t>(ix) - 1;
    const auto iy_base = static_cast<std::ptrdiff_t>(iy) - 1;
    for (std::size_t rx = 0; rx < 4; ++rx) {
        const std::size_t sx = clamped_index(ix_base + static_cast<std::ptrdiff_t>(rx), spec_.nx);
        for (std::size_t ry = 0; ry < 4; ++ry) {
            const std::size_t sy = clamped_index(iy_base + static_cast<std::ptrdiff_t>(ry), spec_.ny);
            stencil[rx][ry] = node_z(sx, sy);
        }
    }

    const BicubicEval3d ev = evaluate_steffen_bicubic(stencil, spec_.dx_m, spec_.dy_m, u, v);
    const double inv_len = 1. / std::sqrt(1. + ev.dz_dx * ev.dz_dx + ev.dz_dy * ev.dz_dy);
    return TerrainSample3d{
        .status = QueryStatus3d::ok,
        .height_m = ev.height_m,
        .normal_world = {-ev.dz_dx * inv_len, -ev.dz_dy * inv_len, inv_len},
        .surface = cell_surfaces_xy_[ix * (spec_.ny - 1) + iy],
    };
}

TorusClosestPoint3d Grid3d::closest_point(const Vec3d& center_world_m,
                                          double search_radius_m) const noexcept {
    if (!std::isfinite(center_world_m[0]) || !std::isfinite(center_world_m[1]) ||
        !std::isfinite(center_world_m[2]) || !std::isfinite(search_radius_m) ||
        !(search_radius_m > 0.)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    const double cx = center_world_m[0];
    const double cy = center_world_m[1];
    const double cz = center_world_m[2];
    if (cx < bounds_.x_min_m || cx > bounds_.x_max_m ||
        cy < bounds_.y_min_m || cy > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }

    // Seed with vertical sample directly below/above (cx, cy).
    const TerrainSample3d vertical = sample(cx, cy);
    if (vertical.status != QueryStatus3d::ok) {
        return {.status = vertical.status};
    }
    double best_d2 = (cz - vertical.height_m) * (cz - vertical.height_m);
    double best_x = cx;
    double best_y = cy;
    TerrainSample3d best_sample = vertical;

    // Scan candidate cells inside [cx - R, cx + R] x [cy - R, cy + R] with exact Z-AABB pruning.
    const double x_lo = std::max(bounds_.x_min_m, cx - search_radius_m);
    const double x_hi = std::min(bounds_.x_max_m, cx + search_radius_m);
    const double y_lo = std::max(bounds_.y_min_m, cy - search_radius_m);
    const double y_hi = std::min(bounds_.y_max_m, cy + search_radius_m);

    const std::size_t ix0 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((x_lo - spec_.x0_m) / spec_.dx_m))),
        spec_.nx - 2);
    const std::size_t ix1 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((x_hi - spec_.x0_m) / spec_.dx_m))),
        spec_.nx - 2);
    const std::size_t iy0 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((y_lo - spec_.y0_m) / spec_.dy_m))),
        spec_.ny - 2);
    const std::size_t iy1 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((y_hi - spec_.y0_m) / spec_.dy_m))),
        spec_.ny - 2);

    for (std::size_t ix = ix0; ix <= ix1; ++ix) {
        const double cell_x0 = spec_.x0_m + static_cast<double>(ix) * spec_.dx_m;
        const double cell_x1 = cell_x0 + spec_.dx_m;
        const double dx_box = cx < cell_x0 ? (cell_x0 - cx) : (cx > cell_x1 ? (cx - cell_x1) : 0.);
        for (std::size_t iy = iy0; iy <= iy1; ++iy) {
            const double cell_y0 = spec_.y0_m + static_cast<double>(iy) * spec_.dy_m;
            const double cell_y1 = cell_y0 + spec_.dy_m;
            const double dy_box = cy < cell_y0 ? (cell_y0 - cy) : (cy > cell_y1 ? (cy - cell_y1) : 0.);
            const CellZRange zr = cell_z_ranges_[ix * (spec_.ny - 1) + iy];
            const double dz_box = cz < zr.z_min ? (zr.z_min - cz) : (cz > zr.z_max ? (cz - zr.z_max) : 0.);
            if (dx_box * dx_box + dy_box * dy_box + dz_box * dz_box >= best_d2) {
                continue;
            }
            // Refine closest point on cell [cell_x0, cell_x1] x [cell_y0, cell_y1] via projected Gauss-Newton.
            double px = std::clamp(cx, cell_x0, cell_x1);
            double py = std::clamp(cy, cell_y0, cell_y1);
            for (int iter = 0; iter < 5; ++iter) {
                const TerrainSample3d s = sample(px, py);
                const double rz = s.height_m - cz;
                const double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy) + rz * rz;
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_x = px;
                    best_y = py;
                    best_sample = s;
                }
                // Gradient of 0.5 * ((x-cx)^2 + (y-cy)^2 + (h(x,y)-cz)^2); dz_dx = -nx/nz, dz_dy = -ny/nz.
                const double hx = -s.normal_world[0] / s.normal_world[2];
                const double hy = -s.normal_world[1] / s.normal_world[2];
                const double gx = (px - cx) + rz * hx;
                const double gy = (py - cy) + rz * hy;
                const double hxx = 1. + hx * hx;
                const double hyy = 1. + hy * hy;
                const double hxy = hx * hy;
                const double det = hxx * hyy - hxy * hxy;
                if (!(det > 1e-12)) break;
                const double step_x = (hyy * gx - hxy * gy) / det;
                const double step_y = (hxx * gy - hxy * gx) / det;
                px = std::clamp(px - step_x, cell_x0, cell_x1);
                py = std::clamp(py - step_y, cell_y0, cell_y1);
            }
            const TerrainSample3d s_end = sample(px, py);
            const double d2_end = (px - cx) * (px - cx) + (py - cy) * (py - cy) +
                                  (s_end.height_m - cz) * (s_end.height_m - cz);
            if (d2_end < best_d2) {
                best_d2 = d2_end;
                best_x = px;
                best_y = py;
                best_sample = s_end;
            }
        }
    }

    const Vec3d q_surf{best_x, best_y, best_sample.height_m};
    const Vec3d diff{cx - q_surf[0], cy - q_surf[1], cz - q_surf[2]};
    const double dist = std::sqrt(best_d2);
    const double sign = cz >= vertical.height_m ? 1. : -1.;
    Vec3d normal = best_sample.normal_world;
    if (dist > 1e-12) {
        normal = {sign * diff[0] / dist, sign * diff[1] / dist, sign * diff[2] / dist};
        if (normal[2] < 0.) {
            normal = best_sample.normal_world;
        }
    }
    return TorusClosestPoint3d{
        .status = QueryStatus3d::ok,
        .signed_distance_m = sign * dist,
        .surface_point_world_m = q_surf,
        .normal_world = normal,
        .surface = best_sample.surface,
    };
}

} // namespace bike::terrain
```

Создать `native/src/terrain/source3d.cpp`:
```cpp
#include "terrain/source3d.hpp"

namespace bike::terrain {

const TerrainBounds3d& Terrain3dSource::bounds() const noexcept {
    return std::visit([](const auto& impl) -> const TerrainBounds3d& { return impl.bounds(); },
                      storage_);
}

const std::string& Terrain3dSource::sha256_hex() const noexcept {
    return std::visit([](const auto& impl) -> const std::string& { return impl.sha256_hex(); },
                      storage_);
}

TerrainSample3d Terrain3dSource::sample(double x_m, double y_m) const noexcept {
    return std::visit([x_m, y_m](const auto& impl) { return impl.sample(x_m, y_m); }, storage_);
}

TorusClosestPoint3d Terrain3dSource::closest_point(const Vec3d& center_world_m,
                                                   double search_radius_m) const noexcept {
    return std::visit(
        [&center_world_m, search_radius_m](const auto& impl) {
            return impl.closest_point(center_world_m, search_radius_m);
        },
        storage_);
}

TerrainBounds3d Terrain3dQueryView::bounds() const noexcept {
    return owner_ ? owner_->bounds() : TerrainBounds3d{};
}

std::string_view Terrain3dQueryView::sha256_hex() const noexcept {
    return owner_ ? std::string_view(owner_->sha256_hex()) : std::string_view{};
}

TerrainSample3d Terrain3dQueryView::sample(double x_m, double y_m) const noexcept {
    return owner_ ? owner_->sample(x_m, y_m) : TerrainSample3d{};
}

TorusClosestPoint3d Terrain3dQueryView::closest_point(const Vec3d& center_world_m,
                                                      double search_radius_m) const noexcept {
    return owner_ ? owner_->closest_point(center_world_m, search_radius_m) : TorusClosestPoint3d{};
}

} // namespace bike::terrain
```

- [ ] **Step 4: Собрать и запустить `terrain_grid3d`, закоммитить**

Run:
```bash
cmake --preset release && cmake --build build/native-release --target terrain_grid3d && ctest --test-dir build/native-release -R '^terrain_grid3d$' --no-tests=error --output-on-failure
```
Expected: PASS (`1/1 Test ... terrain_grid3d ... Passed`)

Commit:
```bash
git add native/cmake/NativeStartup.cmake native/tests/terrain/CMakeLists.txt native/src/terrain/interpolation3d.hpp native/src/terrain/interpolation3d.cpp native/src/terrain/grid3d.hpp native/src/terrain/grid3d.cpp native/src/terrain/source3d.hpp native/src/terrain/source3d.cpp native/tests/terrain/test_grid3d.cpp
git commit -m "feat(terrain): add cache-local Grid3d with Steffen monotone bicubic sampling"
```

---

### Task 5: Срез S1c — процедурный 3D-оверлей `generator3d` поверх существующих трасс `main` и гейт среза Level 1

**Files:**
- Create: `native/src/terrain/generator3d.hpp`
- Create: `native/src/terrain/generator3d.cpp`
- Modify: `native/cmake/NativeStartup.cmake`
- Modify: `native/tests/terrain/CMakeLists.txt`
- Create: `native/tests/terrain/test_generator3d.cpp`

**Interfaces:**
- Consumes: `bike::terrain::{TrackSpec, Grid3d, DEFAULT_RESOLUTION_M}`, `bike::random::NumpyRng` (`random/numpy_compat.hpp`).
- Produces: `bike::terrain::{CrossSlopeSegment3d, Terrain3dOverlaySpec, build_overlay_grid3d}` — строит 3D-рельеф поверх любой из существующих трасс `main` (включая `rough_uphill_savage` и `alpine_rocky`) без потери секций покрытий и с плоскими стартовыми/финишными зонами `lead_in`/`lead_out`.

- [ ] **Step 1: Создать `generator3d.hpp` и `test_generator3d.cpp`**

Создать `native/src/terrain/generator3d.hpp`:
```cpp
#pragma once

#include "terrain/grid3d.hpp"
#include "terrain/types.hpp"

#include <string>
#include <vector>

namespace bike::terrain {

struct CrossSlopeSegment3d {
    double start_m = 0.;
    double end_m = 0.;
    double transition_m = 1.0;
    // Positive bank_rad raises +Y (left) and lowers -Y (right): dz = tan(bank_rad) * y.
    double bank_rad = 0.;
};

struct Terrain3dOverlaySpec {
    double half_width_m = 1.5;
    double dx_m = DEFAULT_RESOLUTION_M; // 0.005 m (matches 1D profile nodes)
    double dy_m = 0.020;                // 0.020 m (151 transverse nodes -> ~32.6 MB on 120 m track)
    double datum_z_offset_m = 0.;
    double lead_in_m = 4.0;
    double lead_out_m = 4.0;
    double lateral_roughness_amplitude_m = 0.;
    std::string seed = "0";
    std::vector<CrossSlopeSegment3d> cross_slopes{};
};

void validate_overlay_spec(const TrackSpec& base_track, const Terrain3dOverlaySpec& overlay);
[[nodiscard]] Grid3d build_overlay_grid3d(const TrackSpec& base_track,
                                          const Terrain3dOverlaySpec& overlay);

} // namespace bike::terrain
```

Создать `native/tests/terrain/test_generator3d.cpp`:
```cpp
#include "support/fixture_test.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/trackfile.hpp"

#include <cmath>
#include <filesystem>
#include <numbers>

int main() {
    return bt::run([] {
        using namespace bike::terrain;
        const std::filesystem::path repo_root = bt::fixture("config").parent_path().parent_path();

        // 1. Both tracks that crashed 3d-new (rough_uphill_savage 100 mm SquareEdge & alpine_rocky scree + SquareEdge)
        // must generate cleanly with a 5-deg cross-slope and preserve exact lead-in flatness + surface sections!
        for (const char* rel : {"examples/research/rough_uphill_savage.toml",
                                "examples/research/alpine_rocky.toml"}) {
            const TrackSpec track = load_track_spec(repo_root / rel);
            Terrain3dOverlaySpec ov{};
            ov.half_width_m = 1.5;
            ov.dx_m = 0.005;
            ov.dy_m = 0.020;
            ov.lead_in_m = 4.0;
            ov.lead_out_m = 4.0;
            ov.cross_slopes.push_back(CrossSlopeSegment3d{
                .start_m = 10.0,
                .end_m = 60.0,
                .transition_m = 3.0,
                .bank_rad = 5.0 * std::numbers::pi / 180.0,
            });

            const Grid3d grid = build_overlay_grid3d(track, ov);
            bt::require(grid.spec().ny == 151, "151 transverse nodes at dy=20 mm");

            // Lead-in zone [0, 3.9 m] has exact zero lateral slope everywhere across [-1.5, 1.5].
            const TerrainSample3d s_left = grid.sample(2.0, 1.2);
            const TerrainSample3d s_right = grid.sample(2.0, -1.2);
            bt::require(s_left.status == QueryStatus3d::ok && s_right.status == QueryStatus3d::ok,
                        "lead-in samples ok");
            bt::require(s_left.height_m == s_right.height_m && s_left.normal_world[1] == 0.0,
                        "lead-in halo keeps interpolated surface strictly flat laterally");

            // Inside full cross-slope at x = 30 m: dz/dy == tan(5 deg).
            const TerrainSample3d bank_pos = grid.sample(30.0, 0.5);
            const TerrainSample3d bank_neg = grid.sample(30.0, -0.5);
            const double expected_diff = std::tan(5.0 * std::numbers::pi / 180.0) * 1.0;
            bt::require(std::abs((bank_pos.height_m - bank_neg.height_m) - expected_diff) <= 1e-12,
                        "5-deg cross-slope elevation difference exact");

            // Surface sections of base_track (e.g. scree on alpine_rocky, asphalt on savage) preserved!
            const SurfaceMap smap = track.surface_map();
            for (double x : {1.0, 25.0, 55.0, 90.0}) {
                const auto expected = rider::surface_label(smap.at(x).name);
                bt::require(grid.sample(x, 0.4).surface == *expected,
                            "base_track surface sections preserved in 3D overlay");
            }
        }

        // 2. Mirror symmetry: flipping bank_rad -> -bank_rad mirrors y -> -y bitwise.
        const TrackSpec flat_track = flat(40.0);
        Terrain3dOverlaySpec pos_ov{.half_width_m = 1.5, .dx_m = 0.01, .dy_m = 0.05,
                                    .lead_in_m = 4.0, .lead_out_m = 4.0,
                                    .cross_slopes = {{.start_m = 8., .end_m = 30., .transition_m = 2., .bank_rad = 0.1}}};
        Terrain3dOverlaySpec neg_ov = pos_ov;
        neg_ov.cross_slopes[0].bank_rad = -0.1;

        const Grid3d g_pos = build_overlay_grid3d(flat_track, pos_ov);
        const Grid3d g_neg = build_overlay_grid3d(flat_track, neg_ov);
        for (double x : {6.0, 9.0, 18.5, 31.0}) {
            for (double y : {-1.1, -0.35, 0.0, 0.72, 1.3}) {
                const TerrainSample3d a = g_pos.sample(x, y);
                const TerrainSample3d b = g_neg.sample(x, -y);
                bt::require(std::abs(a.height_m - b.height_m) <= 1e-15, "mirror height symmetry");
                bt::require(std::abs(a.normal_world[1] + b.normal_world[1]) <= 1e-15,
                            "mirror lateral normal antisymmetry");
            }
        }
    });
}
```

- [ ] **Step 2: Подключить `generator3d.cpp` в `NativeStartup.cmake` и `terrain_generator3d` в `native/tests/terrain/CMakeLists.txt`**

В `native/cmake/NativeStartup.cmake` добавить `"${native_dir}/src/terrain/generator3d.cpp"` в `bike_terrain`.  
В `native/tests/terrain/CMakeLists.txt` добавить `terrain_generator3d` в `foreach(_terrain_test IN ITEMS ...)`.

Run: `cmake --preset release`
Expected: FAIL (`Cannot find source file: .../generator3d.cpp`)

- [ ] **Step 3: Реализовать `native/src/terrain/generator3d.cpp`**

```cpp
#include "terrain/generator3d.hpp"

#include "random/numpy_compat.hpp"
#include "terrain/profile.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace bike::terrain {
namespace {

[[nodiscard]] double quintic_smoothstep(double t) noexcept {
    const double c = std::clamp(t, 0., 1.);
    return c * c * c * (c * (c * 6. - 15.) + 10.);
}

[[nodiscard]] double cross_slope_tan_at(double x_m, const CrossSlopeSegment3d& seg) noexcept {
    if (x_m <= seg.start_m || x_m >= seg.end_m || seg.bank_rad == 0.) {
        return 0.;
    }
    const double slope = std::tan(seg.bank_rad);
    if (!(seg.transition_m > 0.)) {
        return slope;
    }
    const double up = quintic_smoothstep((x_m - seg.start_m) / seg.transition_m);
    const double down = quintic_smoothstep((seg.end_m - x_m) / seg.transition_m);
    return slope * std::min(up, down);
}

[[nodiscard]] double lead_window(double x_m, double length_m, const Terrain3dOverlaySpec& ov) noexcept {
    // Keep 2 extra dx spacings of exact zero inside lead-in/out so 4-point stencils stay strictly flat.
    const double halo = 2. * ov.dx_m;
    const double in_end = ov.lead_in_m + halo;
    const double out_start = length_m - ov.lead_out_m - halo;
    if (x_m <= in_end || x_m >= out_start) {
        return 0.;
    }
    constexpr double kFadeM = 1.0;
    const double w_in = quintic_smoothstep((x_m - in_end) / kFadeM);
    const double w_out = quintic_smoothstep((out_start - x_m) / kFadeM);
    return std::min(w_in, w_out);
}

} // namespace

void validate_overlay_spec(const TrackSpec& base_track, const Terrain3dOverlaySpec& overlay) {
    base_track.validate();
    if (!std::isfinite(overlay.half_width_m) || !(overlay.half_width_m >= 0.25) ||
        !std::isfinite(overlay.dx_m) || !(overlay.dx_m > 0.) ||
        !std::isfinite(overlay.dy_m) || !(overlay.dy_m > 0.) ||
        !std::isfinite(overlay.datum_z_offset_m) ||
        !std::isfinite(overlay.lead_in_m) || !(overlay.lead_in_m >= 0.) ||
        !std::isfinite(overlay.lead_out_m) || !(overlay.lead_out_m >= 0.) ||
        !std::isfinite(overlay.lateral_roughness_amplitude_m) ||
        !(overlay.lateral_roughness_amplitude_m >= 0.)) {
        throw std::invalid_argument("Terrain3dOverlaySpec: invalid dimensions or nonfinite parameter");
    }
    const double ny_real = std::nearbyint((2. * overlay.half_width_m) / overlay.dy_m) + 1.;
    if (ny_real < 5. || std::abs((ny_real - 1.) * overlay.dy_m - 2. * overlay.half_width_m) > 1e-9) {
        throw std::invalid_argument(
            "Terrain3dOverlaySpec.dy_m: 2 * half_width_m must be an exact multiple of dy_m with ny >= 5");
    }
    for (const CrossSlopeSegment3d& seg : overlay.cross_slopes) {
        if (!std::isfinite(seg.start_m) || !std::isfinite(seg.end_m) || !(seg.end_m > seg.start_m) ||
            !std::isfinite(seg.transition_m) || !(seg.transition_m >= 0.) ||
            !std::isfinite(seg.bank_rad) || std::abs(seg.bank_rad) >= std::numbers::pi / 4.) {
            throw std::invalid_argument("Terrain3dOverlaySpec.cross_slopes: invalid segment or |bank| >= 45 deg");
        }
    }
}

Grid3d build_overlay_grid3d(const TrackSpec& base_track, const Terrain3dOverlaySpec& overlay) {
    validate_overlay_spec(base_track, overlay);

    const std::vector<Point> base_pts = profile(base_track, overlay.dx_m);
    const std::size_t nx = base_pts.size();
    const auto ny = static_cast<std::size_t>(std::nearbyint((2. * overlay.half_width_m) / overlay.dy_m)) + 1U;

    std::vector<double> elevations(nx * ny, 0.);
    std::vector<double> roughness(nx * ny, 0.);
    if (overlay.lateral_roughness_amplitude_m > 0.) {
        bike::random::SeedSequence seq(overlay.seed);
        bike::random::NumpyRng rng(seq);
        for (double& r : roughness) {
            r = overlay.lateral_roughness_amplitude_m * rng.standard_normal();
        }
    }

    for (std::size_t ix = 0; ix < nx; ++ix) {
        const double x = base_pts[ix][0];
        const double base_z = base_pts[ix][1] + overlay.datum_z_offset_m;
        const double fade = lead_window(x, base_track.length_m, overlay);
        double slope_y = 0.;
        if (fade > 0.) {
            for (const CrossSlopeSegment3d& seg : overlay.cross_slopes) {
                slope_y += cross_slope_tan_at(x, seg);
            }
        }
        for (std::size_t iy = 0; iy < ny; ++iy) {
            const double y = -overlay.half_width_m + static_cast<double>(iy) * overlay.dy_m;
            const double lat = slope_y * y + roughness[ix * ny + iy];
            elevations[ix * ny + iy] = fade == 0. ? base_z : (base_z + fade * lat);
        }
    }

    const SurfaceMap smap = base_track.surface_map();
    std::vector<rider::SurfaceLabel> cell_surfaces((nx - 1) * (ny - 1), rider::SurfaceLabel::hardpack);
    for (std::size_t ix = 0; ix + 1 < nx; ++ix) {
        const double x_mid = 0.5 * (base_pts[ix][0] + base_pts[ix + 1][0]);
        const auto lbl = rider::surface_label(smap.at(x_mid).name);
        if (!lbl.has_value()) {
            throw std::invalid_argument("build_overlay_grid3d: unknown surface '" + smap.at(x_mid).name + "'");
        }
        for (std::size_t iy = 0; iy + 1 < ny; ++iy) {
            cell_surfaces[ix * (ny - 1) + iy] = *lbl;
        }
    }

    return Grid3d(
        Grid3dSpec{
            .nx = nx,
            .ny = ny,
            .x0_m = base_pts.front()[0],
            .y0_m = -overlay.half_width_m,
            .dx_m = overlay.dx_m,
            .dy_m = overlay.dy_m,
        },
        std::move(elevations), std::move(cell_surfaces));
}

} // namespace bike::terrain
```

- [ ] **Step 4: Собрать и запустить все целевые тесты `S0 + S1`, затем выполнить единый гейт Level 1 (`check_change.sh 0502660d`) и закоммитить**

Run targeted tests first:
```bash
cmake --preset release && cmake --build build/native-release --target spatial_frames spatial_config terrain_extrusion3d terrain_grid3d terrain_generator3d && ctest --test-dir build/native-release -R '^(spatial_(frames|config)|terrain_(extrusion3d|grid3d|generator3d))$' --no-tests=error --output-on-failure
```
Expected: PASS (`5/5 tests passed`)

Run the single Stage-Completion Gate (Level 1 per `AGENTS.md`):
```bash
bash tools/check_change.sh 0502660d
```
Expected: PASS (exit 0: incremental build + ordinary CTest suite + `cppcheck` / `frontends` (`g++-16`) / `clang-tidy` on all new/modified TUs).

Commit:
```bash
git add native/cmake/NativeStartup.cmake native/tests/terrain/CMakeLists.txt native/src/terrain/generator3d.hpp native/src/terrain/generator3d.cpp native/tests/terrain/test_generator3d.cpp
git commit -m "feat(terrain): add 3D track overlay generator preserving existing track surfaces"
```
