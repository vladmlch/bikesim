#pragma once
#include "../app/prepare.hpp"
#include "../recording/contracts.hpp"
#include <string_view>

namespace bike::viewer { struct PacingReport; }
namespace bike::cli {
// Pure presentation/finalization projections, shared by the CLI and its
// source-level contract tests. These helpers do not advance a runtime.
struct ViewerFinalizationTimes {
    double seal_s = 0., preview_s = 0., publication_s = 0., export_s = 0., close_s = 0.;
};
[[nodiscard]] runtime::WireObject physical_preview_row(const runtime::RenderFrame&,
    const terrain::TrackSpec&, std::optional<double> achieved_rtf);
[[nodiscard]] int physical_ride_exit_code(std::string_view reason, const io::Value& summary);
[[nodiscard]] io::Value viewer_performance_value(const viewer::PacingReport&, const io::Value& summary,
    const ViewerFinalizationTimes&, bool fast_profile, bool strict, bool recording_enabled);
[[nodiscard]] recording::Catalog model_catalog(const runtime::ModelImage&);
[[nodiscard]] io::Value physical_summary(runtime::NativeRideRuntime const&, const io::Value&, std::string_view);
[[nodiscard]] std::vector<recording::Asset> physical_assets(const initialization::SetupRequest&,
    const app::PreparedSimulation&, const io::Value& launch_metadata);
[[nodiscard]] io::Value launch_metadata(const initialization::SetupRequest&, const app::PreparedSimulation&,
    const std::optional<std::string>& seed_decimal = std::nullopt);
[[nodiscard]] std::vector<recording::Asset> physical_final_assets(std::vector<recording::Asset>,
    const runtime::NativeRideRuntime&, const io::Value& launch_metadata);
} // namespace bike::cli
