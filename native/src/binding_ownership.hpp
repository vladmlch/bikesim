#pragma once
#include <utility>

namespace wire::detail {
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
} // namespace wire::detail
