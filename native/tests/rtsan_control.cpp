// T16 realtime-sanitizer control pair.
//
// This translation unit is linked into the `native_rtsan_control` target —
// CTest runs only its clean mode — and the reference/portability suites
// compile it in a throwaway directory with the selected toolchain's real
// RTSan flags to run the `forbidden-allocation` mode in a fresh subprocess.
// The violation mode exists so the negative control can prove the real
// RealtimeSanitizer diagnostic fires: a clean exit, a loader error or an
// unrelated signal is not a sanitizer result.
//
// BIKE_NONBLOCKING sits in the trailing declarator position — the same
// position every production annotation uses (src/writers/*.hpp declare
// `... noexcept BIKE_NONBLOCKING` and the definitions repeat it before the
// body). Since LLVM 20, [[clang::nonblocking]] is a function *type*
// attribute: placed before the declaration it is a hard error ("cannot be
// applied to a declaration"), not a silent no-op.

#include <array>
#include <cstdlib>
#include <span>
#include <string_view>

#include "../src/rtsan.hpp"

namespace {

// Positive control: plain arithmetic over a caller-provided buffer inside
// the marked region — no allocation, no locks, no libc calls. It must exit
// clean under an instrumented build and under every ordinary build alike.
int nonblocking_accumulate(std::span<const int> values) BIKE_NONBLOCKING {
    int total = 0;
    for (const int value : values) {
        total += value;
    }
    return total;
}

// Negative control: heap traffic inside the marked region. Compiled with
// -fsanitize=realtime the RTSan runtime reports `unsafe-library-call` on
// the intercepted malloc and aborts; compiled without the flag,
// BIKE_NONBLOCKING expands empty and this exits normally — which is also
// the load-bearing proof that the annotation, not the source, causes the
// report.
void forbidden_allocation() BIKE_NONBLOCKING {
    void* volatile pointer = std::malloc(32);              // NOLINT(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory) — the manual alloc/free IS the violation under test
    std::free(pointer);                                     // NOLINT(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)
}

}  // namespace

int main(int argc, char** argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the
    // sentinel — the same std::span wrap the other test mains use.
    const std::span<char*> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() > 1
        && std::string_view(arguments[1]) == "forbidden-allocation") {
        forbidden_allocation();
        return 0;
    }
    const std::array<int, 2> values{7, 11};
    return nonblocking_accumulate(values) == 18 ? 0 : 1;
}
