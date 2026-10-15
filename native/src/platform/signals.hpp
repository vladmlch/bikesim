// signals.hpp — deferred-SIGINT shield (task T06), the direct port of
// src/bike_sim/recording/interrupt.py's InterruptShield semantics.
//
// Seal+finalize is a critical section: a second Ctrl-C delivered mid-write
// can strand journaled state — so for the scope of one shield the process
// SIGINT handler only records the interrupt (a sig_atomic_t counter), and
// the owning thread honors deferred() AFTER the section completes. The
// handler itself does no allocation, no filesystem or database work and no
// logging; all finalization happens later on the owning thread. Repeated
// SIGINT cannot interrupt the critical section — it only raises the
// deferred count.
//
// On the main thread construction installs the counting handler (the
// previous action is always restored on destruction); off the main thread
// the shield degrades to a flag holder exactly like the source — POSIX
// allows installing from any thread, but the contract reserves handler
// swaps for the owning thread so an off-main scope cannot silently claim
// process signal state. note() is the injected-hook seam a caller (or a
// test without real signal delivery) uses to record a deferred interrupt.
//
// Scopes unwind in LIFO order, matching the source's handler-chain model:
// the innermost installed shield owns the signal while active and the
// previous disposition resumes as each scope exits.
#pragma once

#include <csignal>

namespace bike::platform {

class InterruptShield {
  public:
    InterruptShield();
    ~InterruptShield();
    // The registered counter lives at a fixed address for the handler's
    // lifetime — moving or copying a shield would dangle the signal target.
    InterruptShield(const InterruptShield &) = delete;
    InterruptShield &operator=(const InterruptShield &) = delete;
    InterruptShield(InterruptShield &&) = delete;
    InterruptShield &operator=(InterruptShield &&) = delete;

    // Record one deferred operator interrupt (the source's note()).
    void note() noexcept;
    // Interrupts received while this scope owned the handler plus any
    // note() calls — the caller decides how a nonzero count is honored.
    [[nodiscard]] int deferred() const noexcept;
    // True when this scope actually installed the process handler (main
    // thread and registry capacity); false means flag-holder mode.
    [[nodiscard]] bool installed() const noexcept;

  private:
    volatile std::sig_atomic_t deferred_ = 0;
    int slot_ = -1;
};

} // namespace bike::platform
