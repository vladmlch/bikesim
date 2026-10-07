#include "allocation_faults.hpp"
#include <cstdlib>
#include <new>

// Overrides of the throwing global allocation operators for the test
// executable only. Replacements are permitted by [replacement.functions];
// the nothrow and placement forms stay implicit, which is what the
// production build relies on: the compiler-rt defaults forward to these
// replacements, so every throwing allocation in the test binary funnels
// through this file while the disarmed path remains an exact malloc/free
// implementation.
//
// Alignment: __STDCPP_DEFAULT_NEW_ALIGNMENT__ (16 on this platform) is the
// boundary between the scalar forms and the align_val_t overloads
// ([new.delete.single]); over-aligned requests must round the size up to
// an alignment multiple for std::aligned_alloc.
namespace {
    struct Budget {
        bool armed = false;
        std::size_t remaining = 0;
        std::size_t performed = 0;
    };
    // thread_local confines the sweep to the thread that armed it — helper
    // threads and library-internal allocations never consult it.
    thread_local Budget budget; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables) the injector IS thread-local mutable state

    constexpr std::size_t default_alignment = __STDCPP_DEFAULT_NEW_ALIGNMENT__;

    // aligned_alloc requires size % alignment == 0; round up and keep the
    // zero-size edge nonzero so it cannot surface as a spurious nullptr.
    std::size_t padded(std::size_t size, std::size_t alignment) noexcept {
        const std::size_t rounded = (size / alignment + (size % alignment != 0)) * alignment;
        return rounded != 0 ? rounded : alignment;
    }

    void *allocate(std::size_t size, std::size_t alignment) {
        if (budget.armed) {
            if (budget.remaining == 0) throw std::bad_alloc();
            --budget.remaining;
            ++budget.performed;
        }
        // A replacement operator new must return a raw allocation; malloc is
        // the only primitive that satisfies the contract without recursion.
        void *pointer = alignment <= default_alignment
                            ? std::malloc(size == 0 ? 1 : size)          // NOLINT(cppcoreguidelines-no-malloc)
                            : std::aligned_alloc(alignment,              // NOLINT(cppcoreguidelines-no-malloc)
                                                 padded(size, alignment));
        if (pointer == nullptr) throw std::bad_alloc();
        return pointer;
    }

    // The matching raw release for every replacement delete overload —
    // NOLINTNEXTLINE applies per line because each operator needs its own.
    void release(void *pointer) noexcept { std::free(pointer); }         // NOLINT(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)
} // namespace

void allocation_faults::arm(std::size_t successful_allocations) {
    budget = {.armed = true, .remaining = successful_allocations, .performed = 0};
}

void allocation_faults::disarm() noexcept { budget.armed = false; }

std::size_t allocation_faults::allocated() noexcept { return budget.performed; }

void *operator new(std::size_t size) { return allocate(size, default_alignment); }
void *operator new[](std::size_t size) { return allocate(size, default_alignment); }
void *operator new(std::size_t size, std::align_val_t alignment) {
    return allocate(size, static_cast<std::size_t>(alignment));
}
void *operator new[](std::size_t size, std::align_val_t alignment) {
    return allocate(size, static_cast<std::size_t>(alignment));
}
void operator delete(void *pointer) noexcept { release(pointer); }
void operator delete[](void *pointer) noexcept { release(pointer); }
void operator delete(void *pointer, std::size_t) noexcept { release(pointer); }
void operator delete[](void *pointer, std::size_t) noexcept { release(pointer); }
void operator delete(void *pointer, std::align_val_t) noexcept { release(pointer); }
void operator delete[](void *pointer, std::align_val_t) noexcept { release(pointer); }
void operator delete(void *pointer, std::size_t, std::align_val_t) noexcept { release(pointer); }
void operator delete[](void *pointer, std::size_t, std::align_val_t) noexcept { release(pointer); }
