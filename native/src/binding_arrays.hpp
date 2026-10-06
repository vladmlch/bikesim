#pragma once
#include <algorithm>
#include <cstddef>
#include <memory>
#include <ranges>
#include <span>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace wire {
    namespace nb = nanobind;

    template <typename T>
    nb::ndarray<nb::numpy, T, nb::shape<-1>> owned_array(std::span<const T> values) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) ndarray requires runtime-sized contiguous primitive storage
        auto storage = std::make_unique<T[]>(values.size());
        std::ranges::copy(values, storage.get());
        nb::capsule const owner(storage.get(), [](void *value) noexcept {
            // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) capsule owns the released unique_ptr array
            delete[] static_cast<T *>(value);
        });
        T *const data = storage.release();
        return {data, {values.size()}, owner};
    }

    inline nb::ndarray<nb::numpy, bool, nb::shape<-1>> owned_flags(std::span<const char> values) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) vector<bool> has no contiguous bool storage
        auto storage = std::make_unique<bool[]>(values.size());
        const std::span<bool> destination = std::views::counted(storage.get(),
            static_cast<std::ptrdiff_t>(values.size()));
        for (std::size_t i = 0; i < values.size(); ++i) destination[i] = values[i] != 0;
        nb::capsule const owner(storage.get(), [](void *value) noexcept {
            // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) capsule owns the released unique_ptr array
            delete[] static_cast<bool *>(value);
        });
        bool *const data = storage.release();
        return {data, {values.size()}, owner};
    }
}
