// signals.cpp — deferred-SIGINT shield. The installed handler is
// async-signal-safe by construction: it only performs a lock-free atomic
// pointer load and a sig_atomic_t increment. No allocation, no locks, no
// filesystem work — the owning thread honors deferred() after the critical
// section, mirroring recording/interrupt.py's InterruptShield.
#include "platform/signals.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <csignal>

#if defined(__APPLE__)
#include <pthread.h>
#elif defined(__linux__)
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace {

// The registry is a fixed-size table — no allocation, so the handler path
// can never block on a lock or touch the heap. The "current" target is a
// single lock-free pointer the handler reads on every delivery.
constexpr std::size_t kShieldSlots = 16;

struct ShieldSlot {
    std::atomic<bool> occupied {false};
    volatile std::sig_atomic_t *previous_target = nullptr;
    struct sigaction previous_action {};
};

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
// Justified: the signal handler must reach this table from any thread at
// any time; it is fixed-size, lock-free and written only on the main
// thread outside the handler.
std::array<ShieldSlot, kShieldSlots> g_shield_slots;
std::atomic<volatile std::sig_atomic_t *> g_current_target {nullptr};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

extern "C" void shield_sigint_handler(int /*signum*/) {
    volatile std::sig_atomic_t *target =
        g_current_target.load(std::memory_order_relaxed);
    if (target != nullptr) {
        // C++20 deprecates compound ops (++/--/op=) on volatile, not a
        // plain read + single store — the handler stays sig_atomic_t only.
        *target = *target + 1;
    }
}

bool on_main_thread() {
#if defined(__APPLE__)
    return ::pthread_main_np() != 0;
#elif defined(__linux__)
    return ::getpid() == static_cast<pid_t>(::syscall(SYS_gettid));
#else
    return false; // conservative: only the owning thread may swap handlers
#endif
}

} // namespace

namespace bike::platform {

InterruptShield::InterruptShield() {
    // Off the main thread the shield degrades to a flag holder — the same
    // degrade path the source takes for non-main scopes, so an off-main
    // critical section never silently claims process signal state.
    if (!on_main_thread()) return;
    for (std::size_t i = 0; i < kShieldSlots; ++i) {
        bool expected = false;
        if (g_shield_slots[i].occupied.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
            slot_ = static_cast<int>(i);
            break;
        }
    }
    if (slot_ < 0) return; // registry exhausted — still a flag holder

    ShieldSlot &slot = g_shield_slots[static_cast<std::size_t>(slot_)];
    slot.previous_target = g_current_target.load(std::memory_order_relaxed);
    struct sigaction action {};
    action.sa_handler = &shield_sigint_handler;
    (void) sigemptyset(&action.sa_mask); // sigemptyset is a macro on macOS
    action.sa_flags = 0;
    if (::sigaction(SIGINT, &action, &slot.previous_action) != 0) {
        slot.occupied.store(false, std::memory_order_release);
        slot_ = -1;
        return;
    }
    // Publish the target AFTER the handler is in place; an interrupt
    // between the two steps is attributed to the previous scope exactly as
    // the source's handler chain attributes it.
    g_current_target.store(&deferred_, std::memory_order_relaxed);
}

InterruptShield::~InterruptShield() {
    if (slot_ < 0) return;
    ShieldSlot &slot = g_shield_slots[static_cast<std::size_t>(slot_)];
    // Repoint the counter first: a delivery between this store and the
    // sigaction restore lands on the previous scope's counter, matching
    // the source's unwind order.
    g_current_target.store(slot.previous_target, std::memory_order_relaxed);
    (void) ::sigaction(SIGINT, &slot.previous_action, nullptr);
    slot.previous_target = nullptr;
    slot.occupied.store(false, std::memory_order_release);
}

void InterruptShield::note() noexcept { deferred_ = deferred_ + 1; }

int InterruptShield::deferred() const noexcept { return deferred_; }

bool InterruptShield::installed() const noexcept { return slot_ >= 0; }

} // namespace bike::platform
