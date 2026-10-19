#include "allocation_faults.hpp"
#include <cstdlib>
#include <new>

// Overrides of the throwing AND nothrow global allocation operators for
// the test executable only. Replacements are permitted by
// [replacement.functions]. The nothrow forms must funnel through the same
// malloc/free pair: under AddressSanitizer the compiler-rt defaults do
// NOT forward to these replacements — an unreplaced operator
// new(nothrow) (what libstdc++ get_temporary_buffer uses) allocates
// through the sanitizer's bookkeeping while the replaced operator delete
// releases via std::free, which ASan reports as alloc-dealloc-mismatch.
// Only the placement forms stay implicit; the disarmed path remains an
// exact malloc/free implementation either way.
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

    struct Activity {
        std::size_t allocated = 0;
        std::size_t freed = 0;
    };
    // The allocation_counter window: separate thread_local counters bumped
    // unconditionally by allocate()/release() so the zero-allocation cases
    // measure the warm core without touching the injector's budget state.
    thread_local Activity activity; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables) same contract as budget above

    constexpr std::size_t default_alignment = __STDCPP_DEFAULT_NEW_ALIGNMENT__;

    // aligned_alloc requires size % alignment == 0; round up and keep the
    // zero-size edge nonzero so it cannot surface as a spurious nullptr.
    std::size_t padded(std::size_t size, std::size_t alignment) noexcept {
        const std::size_t rounded = (size / alignment + (size % alignment != 0)) * alignment;
        return rounded != 0 ? rounded : alignment;
    }

    void *allocate(std::size_t size, std::size_t alignment) {
        ++activity.allocated; // unconditional: attempts count too
        if (budget.armed) {
            if (budget.remaining == 0)
                throw std::bad_alloc();
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

    // The nothrow operator news share the funnel but translate the two
    // throwing exits (budget exhaustion, raw-allocation failure) into the
    // contract's nullptr return.
    void *allocate_nothrow(std::size_t size, std::size_t alignment) noexcept {
        try {
            return allocate(size, alignment);
        } catch (...) {
            return nullptr;
        }
    }

    // The matching raw release for every replacement delete overload —
    // NOLINTNEXTLINE applies per line because each operator needs its own.
    // nullptr releases count too: a delete call is still allocator activity.
    void release(void *pointer) noexcept {
        ++activity.freed;
        std::free(pointer);                                              // NOLINT(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)
    }
} // namespace

void allocation_faults::arm(std::size_t successful_allocations) {
    budget = {.armed = true, .remaining = successful_allocations, .performed = 0};
}

void allocation_faults::disarm() noexcept { budget.armed = false; }

std::size_t allocation_faults::allocated() noexcept { return budget.performed; }

void allocation_counter::reset() noexcept { activity = {}; }

std::size_t allocation_counter::count() noexcept { return activity.allocated; }

std::size_t allocation_counter::freed() noexcept { return activity.freed; }

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

// Nothrow forms — see the file-level comment: leaving these implicit lets
// compiler-rt allocate through its own bookkeeping while the deletes above
// release via std::free, which AddressSanitizer reports as a mismatch.
void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
    return allocate_nothrow(size, default_alignment);
}
void *operator new[](std::size_t size, const std::nothrow_t &) noexcept {
    return allocate_nothrow(size, default_alignment);
}
void *operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t &) noexcept {
    return allocate_nothrow(size, static_cast<std::size_t>(alignment));
}
void *operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t &) noexcept {
    return allocate_nothrow(size, static_cast<std::size_t>(alignment));
}
void operator delete(void *pointer, const std::nothrow_t &) noexcept { release(pointer); }
void operator delete[](void *pointer, const std::nothrow_t &) noexcept { release(pointer); }
void operator delete(void *pointer, std::align_val_t,
                     const std::nothrow_t &) noexcept { release(pointer); }
void operator delete[](void *pointer, std::align_val_t,
                       const std::nothrow_t &) noexcept { release(pointer); }
