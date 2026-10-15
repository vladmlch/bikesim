// T15 sanitizer controls.
//
// This translation unit IS linked into the `native_sanitizer_control`
// target — CTest runs only the clean mode — and the portability suite
// (tests/portability/test_native_runtime.py) additionally compiles it in
// a throwaway directory with the selected toolchain's sanitizer flags to
// run each mode in a fresh subprocess under the runtime environment that
// tools/native_runtime.py computes. The `use-after-free` mode exists so
// the negative control can prove the real sanitizer diagnostic fires — a
// loader error or unrelated crash is not accepted as a sanitizer result.

#include <cstddef>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

namespace {

// Positive control: heap allocate, write, read, release — must exit 0
// under every instrumentation lane (ASan, UBSan, RTSan and the
// unsanitized build). std::vector keeps the allocation on the heap while
// staying inside the -Wunsafe-buffer-usage contract.
int clean_control() {
    std::vector<int> values(4);
    values[0] = 7;
    values[3] = 11;
    const int sum = values[0] + values[3];
    values.clear();
    return sum == 18 ? 0 : 1;
}

// Negative control (portability plan task T15): an escaped pointer reads
// storage after its owning unique_ptr released it. The volatile pointer
// keeps the read observable under optimization so the sanitizer — not the
// compiler — owns the outcome; ASan reports heap-use-after-free. Every
// buffer operation in this function is the injected defect's machinery,
// so clang's compile-time buffer-usage analysis is scoped out of the
// whole body (GCC knows no such warning group).
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage"
#endif
int use_after_free_control(std::size_t index) {
    auto values = std::make_unique<int[]>(4);
    values[0] = 7;
    volatile int* escaped = values.get();
    values.reset();
    return escaped[index];
}
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

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
        && std::string_view(arguments[1]) == "use-after-free") {
        return use_after_free_control(arguments.size() - 1);
    }
    return clean_control();
}
