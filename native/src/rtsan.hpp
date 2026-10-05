#pragma once

// RealtimeSanitizer marking: BIKE_NONBLOCKING declares the per-tick hot path
// allocation/lock-free. Under -fsanitize=realtime (NATIVE_RTSAN, brew clang
// only — Apple clang lacks the sanitizer) a malloc/free/lock inside a marked
// function aborts with a report; under every other build the attribute is
// compiled out entirely.
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
