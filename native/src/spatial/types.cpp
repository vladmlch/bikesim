#include "types.hpp"

namespace bike::spatial {

std::string_view to_string(Mode value) noexcept {
    switch (value) {
    case Mode::planar: return "planar";
    case Mode::spatial3d: return "3d";
    }
    return {};
}
std::optional<Mode> parse_mode(std::string_view value) noexcept {
    if (value == "planar") return Mode::planar;
    if (value == "3d") return Mode::spatial3d;
    return std::nullopt;
}
std::string_view to_string(RootKind value) noexcept {
    switch (value) {
    case RootKind::planar3: return "planar";
    case RootKind::constrained4: return "constrained";
    case RootKind::free6: return "free";
    }
    return {};
}
std::optional<RootKind> parse_root_kind(std::string_view value) noexcept {
    if (value == "planar") return RootKind::planar3;
    if (value == "constrained") return RootKind::constrained4;
    if (value == "free") return RootKind::free6;
    return std::nullopt;
}
std::string_view to_string(TyreTier value) noexcept {
    switch (value) {
    case TyreTier::fast: return "fast";
    case TyreTier::detailed: return "detailed";
    }
    return {};
}
std::optional<TyreTier> parse_tyre_tier(std::string_view value) noexcept {
    if (value == "fast") return TyreTier::fast;
    if (value == "detailed") return TyreTier::detailed;
    return std::nullopt;
}

} // namespace bike::spatial
