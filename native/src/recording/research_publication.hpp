#pragma once
#include "publish.hpp"
#include "../research/lifecycle.hpp"

namespace bike::recording {
// Constructed only when --record was explicitly requested. Metadata/catalog
// are frozen before stepping; publish adapts sealed RAM receipts exactly once.
[[nodiscard]] std::shared_ptr<research::ResearchPublication> make_research_publication(
    RunMetadata metadata, Catalog catalog, std::filesystem::path output_root,
    FinalizeOptions options = {});
} // namespace bike::recording
