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

#include <cstddef>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace model_access {
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
} // namespace model_access
