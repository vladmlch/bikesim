// model_access.hpp — checked (pointer, extent) → span construction plus
// model-ID and element-product validation shared by the native boundary
// translation units.
//
// mjModel/mjData expose raw pointer + mjtSize extent pairs. Forming a span
// from an unchecked pair is undefined behavior when the extent is negative
// or the pointer is null with positive extent: these helpers reject both
// with std::invalid_argument before any range is created. Read-only struct
// access uses std::span<const T>; mutable engine arrays (ctrl, applied
// force rows) use std::span<T>. Callers that still carry a local
// drivetrain::buffer-style alias delegate here — this header is the shared
// home other TUs adopt as they gain checked extents.
//
// Ownership contract: the helpers validate extents and IDs, never model/
// data pairing. That a given mjData* was created from the paired mjModel*
// is a documented caller precondition carried by the owning Stepper
// context; dimension equality alone does not prove pairing.
#pragma once

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace model_access {
    // Engine-shape element counts — a body/geom position is XYZ and a
    // rotation matrix is 3x3. Use them only where the count IS that
    // spatial shape: a coefficient-table width (4 hfield_size elements,
    // 2D profile columns, Jacobian row pairs) is not a spatial dimension
    // and keeps its literal.
    inline constexpr int kXYZ = 3;
    inline constexpr int kRotationElements = 9;
    inline constexpr int kQuaternionElements = 4;
    namespace detail {
        // Shared extent gate: mjtSize is signed, so negative counts and a
        // null pointer with positive extent are rejected before any span is
        // formed. Returns the extent as an element count.
        inline std::size_t checked_extent(const void *data, mjtSize count,
                                          std::string_view field) {
            if (count < 0)
                throw std::invalid_argument(
                    std::string(field) + ": negative extent");
            if (data == nullptr && count > 0)
                throw std::invalid_argument(
                    std::string(field) +
                    ": null buffer with positive extent");
            return static_cast<std::size_t>(count);
        }
    } // namespace detail

    // Read-only span over an engine array of `count` elements. A zero
    // extent with a null pointer is the declared empty buffer.
    // checked_extent() guarantees count >= 0, so the ptrdiff_t narrowing
    // below can only discard a magnitude no mjtSize could carry — the
    // explicit cast keeps every frontend's sign-conversion gate honest.
    template<typename T>
    [[nodiscard]] std::span<const T>
    readonly_buffer(const T *data, mjtSize count,
                    std::string_view field = "model buffer") {
        return std::views::counted(
            data, static_cast<std::ptrdiff_t>(
                      detail::checked_extent(data, count, field)));
    }

    // Mutable span over an engine array of `count` elements, same extent
    // contract as readonly_buffer.
    template<typename T>
    [[nodiscard]] std::span<T>
    mutable_buffer(T *data, mjtSize count,
                   std::string_view field = "model buffer") {
        return std::views::counted(
            data, static_cast<std::ptrdiff_t>(
                      detail::checked_extent(data, count, field)));
    }

    // Model-array element ID: require 0 <= id < count with both sides
    // signed (a negative count rejects every ID).
    inline void require_id(int id, mjtSize count, std::string_view field) {
        if (id < 0 || static_cast<mjtSize>(id) >= count)
            throw std::invalid_argument(
                std::string(field) + ": invalid model ID");
    }

    // a * b, required to stay within `limit` declared elements. The divide
    // side runs before the product is formed, so the multiply itself can
    // never wrap for a representable limit.
    [[nodiscard]] inline std::size_t checked_product(std::size_t a,
                                                     std::size_t b,
                                                     std::size_t limit) {
        if (a != 0 && b > limit / a)
            throw std::invalid_argument(
                "buffer extent exceeds declared storage");
        return a * b;
    }

    // a + b, required to stay within `limit` declared elements. The
    // comparisons run before the sum is formed; a > limit is rejected
    // rather than wrapping `limit - a` negative.
    [[nodiscard]] inline std::size_t checked_sum(std::size_t a, std::size_t b,
                                                 std::size_t limit) {
        if (a > limit || b > limit - a)
            throw std::invalid_argument(
                "buffer extent exceeds declared storage");
        return a + b;
    }

    // resolve_id (physical_mapping.py:6-11): name → model ID or the
    // shared "model has no '<name>'" rejection. `physical` makes the
    // nonzero-ID domain explicit — pass it when the resolved element
    // must be a physical body (the world body, ID 0, is never a
    // joint/force site); resolvers of other model kinds leave it off
    // and keep their own downstream topology checks. Sites whose
    // rejection text is a different Python message (braking.py's
    // joint/geom/actuator lookups) keep their local resolvers — this
    // helper is only the shared contract.
    [[nodiscard]] inline int resolve_id(const mjModel *m, mjtObj kind,
                                        const char *name,
                                        bool physical = false) {
        const int id = mj_name2id(m, kind, name);
        if (id < 0 || (physical && id == 0))
            throw std::invalid_argument("model has no '" +
                                        std::string(name) + "'");
        return id;
    }

    // Body-domain contract for the point-Jacobian boundary — an explicit
    // parameter so each port keeps its recorded gate:
    enum class JacobianBody : std::uint8_t {
        // Split gate: an out-of-range ID reports "<field>: invalid model
        // ID" through require_id; the world body reports the
        // physical-body message. (tire writer's recorded contract)
        physical = 0,
        // Single gate matching physical_mapping.py:36 literally — any ID
        // outside 1..nbody reports the physical-body message.
        // (resistance writer's recorded contract)
        physical_strict = 1
    };

    // Checked boundary for the jacp-only point Jacobian
    // (physical_mapping.py:24-37 point_jacobian_into): the world point
    // must be exactly 3 finite coordinates, the body must satisfy
    // `body_domain`, nv must be nonnegative, and the destination must be
    // exactly 3*nv elements — the engine ABI receives .data() only after
    // every check, with nullptr for the unused rotational half like the
    // Python helper's None. `field` labels the body check inside
    // rejections; the width-contract messages are the fixed physical
    // mapping labels below.
    inline void point_jacobian_into(const mjModel *m, const mjData *d,
                                    int body_id,
                                    std::span<const mjtNum> point,
                                    std::span<mjtNum> jacp,
                                    JacobianBody body_domain,
                                    std::string_view field) {
        if (point.size() != static_cast<std::size_t>(kXYZ) ||
            !std::ranges::all_of(
                point, [](mjtNum v) { return std::isfinite(v); }))
            throw std::invalid_argument("invalid world point");
        if (body_domain == JacobianBody::physical_strict) {
            if (!(0 < body_id && body_id < m->nbody))
                throw std::invalid_argument(
                    "point Jacobian requires a physical body");
        } else {
            require_id(body_id, m->nbody, field);
            if (body_domain == JacobianBody::physical && body_id == 0)
                throw std::invalid_argument(
                    "point Jacobian requires a physical body");
        }
        if (m->nv < 0)
            throw std::invalid_argument(
                "point Jacobian needs a nonnegative model width");
        if (jacp.size() != checked_product(
                static_cast<std::size_t>(kXYZ),
                static_cast<std::size_t>(m->nv),
                std::numeric_limits<std::size_t>::max()))
            throw std::invalid_argument(
                "point Jacobian destination must be 3*nv elements");
        mj_jac(m, d, jacp.data(), nullptr, point.data(), body_id);
    }
} // namespace model_access
