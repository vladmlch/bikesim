#pragma once
#include "environment.hpp"
#include "../runtime/status.hpp"

namespace bike::research {
struct EnergyQuality {
    double residual_ratio = 0., electrical_residual_j = 0.;
    bool acceptable = true;
};
[[nodiscard]] EnergyQuality energy_quality(const runtime::WireObject&, double maximum_ratio = .05);
[[nodiscard]] std::vector<std::string> model_violations(const runtime::PhysicalSampleData&,
    double maximum_compression_fraction = .15, double maximum_linkage_error_m = .002);
[[nodiscard]] io::Value wire_value(const runtime::Wire&);
// The physical identity is common to an ordinary ride and a research
// episode. An omitted seed remains null; a present seed keeps all digits.
[[nodiscard]] io::Value physical_configuration_metadata(const initialization::SetupRequest&,
    const initialization::ResearchStartup&, const runtime::RuntimeBootstrap&, const io::Value& setup_metadata,
    const std::optional<std::string>& seed_decimal = std::nullopt);
[[nodiscard]] io::Value configuration_metadata(const ResearchRequest&,
    const initialization::ResearchStartup&, const runtime::RuntimeBootstrap&, const io::Value& setup_metadata);
// Reset changes sensor/policy entropy; it does not regenerate terrain or rider.
[[nodiscard]] io::Value reseed_metadata(const io::Value&, std::string_view seed_decimal);
// Re-check compiled-source, executable/provider and referenced profile identities.
// Missing captured identities stay unknown, and cannot qualify for replay.
[[nodiscard]] io::Value identity_receipt(const io::Value&);
[[nodiscard]] io::Value research_recipe(const ResearchRequest&, const io::Value& metadata);
[[nodiscard]] runtime::ResearchAssets snapshot_research_assets(const ResearchRequest&,
    const initialization::ResearchStartup&, const io::Value& metadata, const runtime::RuntimeBootstrap&);
[[nodiscard]] std::vector<runtime::ResearchAsset> snapshot_spatial_assets(
    const initialization::SetupRequest&, const runtime::RuntimeBootstrap&, const io::Value& metadata);
[[nodiscard]] runtime::ResearchAssets reseed_assets(const runtime::ResearchAssets&, const io::Value& metadata);
// Adds stable logical IDs without altering the historical T10 batch shape.
[[nodiscard]] runtime::WireArray recording_batch(const runtime::OwnedResearchEpisode&, std::string_view stream,
                                                std::int64_t start, std::int64_t limit);
[[nodiscard]] io::Value physical_summary(const runtime::OwnedResearchEpisode&);
[[nodiscard]] io::Value episode_metrics(const runtime::OwnedResearchEpisode&);
[[nodiscard]] io::Value summarize(const runtime::OwnedResearchEpisode&);
[[nodiscard]] std::string physical_run_dir_name(std::string_view track_name, const io::Value& metadata);
[[nodiscard]] int episode_exit_code(const runtime::WireObject& status);
} // namespace bike::research
