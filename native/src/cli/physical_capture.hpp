#pragma once
#include "../app/prepare.hpp"
#include "../recording/contracts.hpp"
#include <string_view>

namespace bike::cli {
[[nodiscard]] recording::Catalog model_catalog(const runtime::ModelImage&);
[[nodiscard]] io::Value physical_summary(runtime::NativeRideRuntime&, const io::Value&, std::string_view);
[[nodiscard]] std::vector<recording::Asset> physical_assets(const initialization::SetupRequest&,
    const app::PreparedSimulation&, const io::Value& launch_metadata);
[[nodiscard]] io::Value launch_metadata(const initialization::SetupRequest&, const app::PreparedSimulation&,
    const std::optional<std::string>& seed_decimal = std::nullopt);
} // namespace bike::cli
