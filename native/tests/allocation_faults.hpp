#pragma once
#include <cstddef>

// Test-only thread-local global-new injector for native_contract_tests.
//
// The operators in allocation_faults.cpp are compiled only into the test
// executable — never into bike_native — so injection is scoped to the test
// binary's own translation units. Library images (libmujoco, libc++) bind
// their own operator new under Mach-O two-level namespaces, so engine
// internals and the standard library pass through untouched, as does the
// engine's mju_user_malloc plumbing. While disarmed the operators delegate
// to the C runtime exactly like the stock allocation functions, so a
// disarmed injector cannot alter production allocation behavior.
namespace allocation_faults {
    // Arm the injector: the next `successful_allocations` throwing
    // allocation calls on THIS thread succeed; the following one throws
    // std::bad_alloc. arm() itself performs no allocation — callers may
    // arm while holding no other injector state. The count is measured by
    // running the operation once under arm(SIZE_MAX) and reading
    // allocated() afterwards.
    void arm(std::size_t successful_allocations);

    // Restore production allocation behavior on this thread. noexcept so
    // scope-exit cleanup is legal on every exception path.
    void disarm() noexcept;

    // Throwing allocations performed since the most recent arm() — the
    // measurement used to size an enumeration sweep. Must be read before
    // the next arm()/disarm() — disarm leaves the counter readable.
    [[nodiscard]] std::size_t allocated() noexcept;

    // Scope-bound disarm: test code builds snapshots and assertion output
    // while disarmed, arms only for the attempt, and lets the guard cover
    // the unexpected-exception paths a local catch would miss.
    class Guard {
    public:
        Guard() = default;
        Guard(const Guard &) = delete;
        Guard &operator=(const Guard &) = delete;
        Guard(Guard &&) = delete;
        Guard &operator=(Guard &&) = delete;
        ~Guard() { disarm(); }
    };
} // namespace allocation_faults

// Always-on companion to the injector above: the same replacement operators
// keep a second pair of thread_local counters that increment on EVERY
// allocation and deallocation call — armed or not, successful or throwing.
// The warm-core zero-allocation contract reads these: E4's `allocated()`
// only measures allocations performed under an armed budget, while
// count()/freed() see every call the test binary's own TUs make, including
// aligned and sized forms (library images still bind their own operators).
namespace allocation_counter {
    // Zero both counters. Test cases reset AFTER fixtures, output buffers
    // and the measured warmup are fully built so the core-path window sees
    // only per-tick activity.
    void reset() noexcept;

    // operator new calls since the most recent reset(), including calls
    // that went on to throw — a throwing attempt is still allocation
    // activity the warm core must not contain.
    [[nodiscard]] std::size_t count() noexcept;

    // operator delete calls since the most recent reset(), nullptr
    // releases included.
    [[nodiscard]] std::size_t freed() noexcept;
} // namespace allocation_counter
