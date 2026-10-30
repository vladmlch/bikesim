#pragma once
#include "../app/prepare.hpp"
#include "../recording/contracts.hpp"
#include "../research/lifecycle.hpp"
#include <exception>
#include <functional>
#include <string_view>

namespace bike::cli {
[[nodiscard]] recording::Catalog model_catalog(const runtime::ModelImage&);
[[nodiscard]] io::Value physical_summary(runtime::NativeRideRuntime const&, const io::Value&, std::string_view);
[[nodiscard]] std::vector<recording::Asset> physical_assets(const initialization::SetupRequest&,
    const app::PreparedSimulation&, const io::Value& launch_metadata);
[[nodiscard]] io::Value launch_metadata(const initialization::SetupRequest&, const app::PreparedSimulation&,
    const std::optional<std::string>& seed_decimal = std::nullopt);

// The physical run's finalization order as captured callables. seal and
// runtime_close are mandatory for a real run; empty optionals mark absent
// preview/publication/plots. Production-time conditions (capture enabled and
// sealed, a produced publication) stay inside the captured actions.
struct PhysicalFinalizationActions {
    std::function<void()> seal;
    std::function<void()> preview_close;
    std::function<void()> recording_publish;
    std::function<void()> plot_publish;
    std::function<void()> runtime_close;
};

// Runs the finalization order once: seal (at most one retry over the already
// committed prefix — never a physics advance), preview_close,
// recording_publish, plot_publish, runtime_close. The passed primary keeps
// its identity; every later failure lands as a secondary
// "operation: cause" note, so a cleanup error can never mask the primary.
// This helper never logs and never touches physics.
[[nodiscard]] research::FailureReport finalize_physical_run(
    const PhysicalFinalizationActions& actions,
    std::exception_ptr primary = {});
} // namespace bike::cli
