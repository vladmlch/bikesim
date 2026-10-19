#pragma once

// RealtimeSanitizer marking: BIKE_NONBLOCKING declares the per-tick hot path
// allocation/lock-free. Under -fsanitize=realtime (NATIVE_RTSAN, upstream
// clang only — Apple clang and GCC lack the sanitizer) a malloc/free/lock
// inside a marked function aborts with a report; under every other build
// the attribute is compiled out entirely.
//
// Qualification (T16): cmake/BikeCompilerPolicy.cmake never infers support
// from the compiler family or version — it compile-probes -fsanitize=realtime
// together with this header's __has_feature query and a BIKE_NONBLOCKING
// function, proves the annotation is load-bearing via the function-effects
// checker, resolves the selected toolchain's runtime through
// -print-file-name, and runs both a clean control and the
// tests/rtsan_control.cpp forbidden-allocation violation control (which
// must die with the real RealtimeSanitizer report) before the requested
// lane may configure. Since LLVM 20 [[clang::nonblocking]] is a function
// *type* attribute: place the macro in the trailing declarator position
// (`void f() BIKE_NONBLOCKING`), never before the declaration.
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
