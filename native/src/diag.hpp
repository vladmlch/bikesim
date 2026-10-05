#pragma once

// Scoped warning suppression that is a no-op under GCC: the frontends sweep
// compiles these headers with -Werror, and GCC rejects clang-specific names
// inside #pragma GCC diagnostic. Usage:
//   NATIVE_DIAG_PUSH
//   NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
//   ...
//   NATIVE_DIAG_POP
#if defined(__clang__)
#define NATIVE_PRAGMA_(x) _Pragma(#x)
#define NATIVE_DIAG_PUSH NATIVE_PRAGMA_(clang diagnostic push)
#define NATIVE_DIAG_POP NATIVE_PRAGMA_(clang diagnostic pop)
#define NATIVE_DIAG_IGNORE(flag) NATIVE_PRAGMA_(clang diagnostic ignored flag)
#else
#define NATIVE_DIAG_PUSH
#define NATIVE_DIAG_POP
#define NATIVE_DIAG_IGNORE(flag)
#endif
