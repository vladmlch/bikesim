#pragma once
#include "input.hpp"
#include "../runtime/research.hpp"

namespace bike::replay {
// Explicit compatibility projection. This owner detaches every growing
// stream before output is touched, and survives runtime reset()/close().
struct LegacySnapshot {
    ReplayInput data;
    runtime::SampleColumns columns;
    std::vector<io::Value> intervals;
};
[[nodiscard]] LegacySnapshot capture_legacy(runtime::OwnedResearchEpisode);
[[nodiscard]] LegacySnapshot capture_legacy(runtime::NativeResearchRuntime const&);
// Publishes the historical schema-2 filenames and hashes, manifest last.
// Native executable provenance stays intact. It is never forged into the
// former Python-extension execution schema; the result states compatibility.
[[nodiscard]] io::Value export_legacy(const LegacySnapshot&, const std::filesystem::path&, bool overwrite = false);
[[nodiscard]] io::Value export_legacy(const runtime::NativeResearchRuntime&, const std::filesystem::path&, bool overwrite = false);
} // namespace bike::replay
