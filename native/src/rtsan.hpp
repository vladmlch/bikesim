#pragma once

// RealtimeSanitizer marking: BIKE_NONBLOCKING declares the per-tick hot path
// allocation/lock-free. Under -fsanitize=realtime (NATIVE_RTSAN, brew clang
// only — Apple clang lacks the sanitizer) a malloc/free/lock inside a marked
// function aborts with a report; under every other build the attribute is
// compiled out entirely.
//
// Contract for marked functions: no allocation, no locks, and no throwing —
// exceptions stay outside nonblocking kernels. Invalid core state inside a
// marked region is reported through a status return (engine::ErrorBuffer on
// Stepper::try_step/try_forward) that the caller translates into an
// exception only after leaving the marked region. Functions that can still
// throw or allocate must NOT carry the mark or noexcept.
// Verified arm64-apple: mj_forward and mj_step do not allocate, so
// Stepper::forward/step may carry the mark today. As the P4 tick lands, move
// the mark to the whole-tick entry point and keep it honest.
#if defined(__has_feature)
#  if __has_feature(realtime_sanitizer)
#    define BIKE_NONBLOCKING [[clang::nonblocking]]
#  endif
#endif
#ifndef BIKE_NONBLOCKING
#  define BIKE_NONBLOCKING
#endif
