#pragma once
#include <algorithm>
#include <cstddef>
#include <memory>
#include <ranges>
#include <span>
#include <utility>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace wire {
    namespace nb = nanobind;

    namespace detail {
        // Ownership staging shared by every owned array (and by the
        // contract test's forced-failure seam): make_owner is invoked while
        // the unique_ptr still holds the allocation, so a throwing owner
        // constructor can never leak the buffer. release() runs only after
        // the owner exists; the returned pair carries (released data,
        // owner) for the caller to hand to the ndarray.
        template <typename Storage, typename MakeOwner>
        [[nodiscard]] auto release_with_owner(Storage &&storage,
                                              MakeOwner &&make_owner) {
            auto owner = std::forward<MakeOwner>(make_owner)(storage.get());
            auto *const data = std::forward<Storage>(storage).release();
            return std::pair{data, std::move(owner)};
        }
    }

    template <typename T>
    nb::ndarray<nb::numpy, T, nb::shape<-1>> owned_array(std::span<const T> values) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) ndarray requires runtime-sized contiguous primitive storage
        auto storage = std::make_unique<T[]>(values.size());
        std::ranges::copy(values, storage.get());
        auto [data, owner] = detail::release_with_owner(std::move(storage), [](T *value) {
            // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) capsule owns the released unique_ptr array
            return nb::capsule(value, [](void *p) noexcept { delete[] static_cast<T *>(p); });
        });
        return {data, {values.size()}, owner};
    }

    inline nb::ndarray<nb::numpy, bool, nb::shape<-1>> owned_flags(std::span<const char> values) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) vector<bool> has no contiguous bool storage
        auto storage = std::make_unique<bool[]>(values.size());
        const std::span<bool> destination = std::views::counted(storage.get(),
            static_cast<std::ptrdiff_t>(values.size()));
        for (std::size_t i = 0; i < values.size(); ++i) destination[i] = values[i] != 0;
        auto [data, owner] = detail::release_with_owner(std::move(storage), [](bool *value) {
            // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) capsule owns the released unique_ptr array
            return nb::capsule(value, [](void *p) noexcept { delete[] static_cast<bool *>(p); });
        });
        return {data, {values.size()}, owner};
    }
}
