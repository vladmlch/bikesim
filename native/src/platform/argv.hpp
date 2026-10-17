#pragma once
#include <cstddef>
#include <span>
#include <stdexcept>

namespace bike::platform {
// Use only with the argc/argv pair supplied to a native main entry point.
// The C runtime supplies exactly argc accessible argument pointers. Keep
// that one ABI-to-container conversion out of ordinary parser code.
[[nodiscard]] inline std::span<char*> main_arguments(int argc,char** argv) {
    if(argc<0||(argc>0&&argv==nullptr))throw std::invalid_argument("invalid native argument envelope");
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    const std::span<char*> arguments{argv,static_cast<std::size_t>(argc)};
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    return arguments;
}
} // namespace bike::platform
