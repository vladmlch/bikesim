// runtime/wire.cpp — the wire-tree constructors declared in control.hpp.
// Split out so the contract-core object library (which links samples.cpp
// but not control.cpp) resolves them without pulling the runtime helper
// layer. Ordering of the visitor arms mirrors the Python statement it
// names; that ordering is the FP contract.
#include "control.hpp"

namespace runtime {

Wire wire_array(std::span<const double> values) {
    WireArray out;
    out.reserve(values.size());
    for (const double v : values) out.emplace_back(v);
    return out;
}

Wire wire_diagnostics(
    const std::map<std::string,
                   std::variant<std::monostate, double, int, bool,
                                std::string>> &values) {
    WireObject out;
    out.reserve(values.size());
    for (const auto &[name, v] : values) {
        out.emplace_back(name, std::visit(
                                   [](const auto &x) -> Wire {
                                       using T = std::decay_t<decltype(x)>;
                                       if constexpr (std::is_same_v<
                                                         T, int>)
                                           return Wire(
                                               static_cast<std::int64_t>(
                                                   x));
                                       else if constexpr (std::is_same_v<
                                                              T,
                                                              std::monostate>)
                                           return Wire{nullptr};
                                       else
                                           return Wire{x};
                                   },
                                   v));
    }
    return out;
}

} // namespace runtime
