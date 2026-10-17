#pragma once
#include "arguments.hpp"
#include "../app/config.hpp"
#include "../initialization/setup_owner.hpp"
#include "../research/environment.hpp"
#include <filesystem>
#include <span>
#include <string_view>

namespace bike::cli {
struct ResolvedRide { Arguments arguments; initialization::SetupRequest setup; };
[[nodiscard]] ResolvedRide resolve_ride(Arguments, bool settling = false);
[[nodiscard]] terrain::TrackSpec resolve_track(std::string_view, std::optional<std::string> seed = {},
    std::optional<double> length = {});
[[nodiscard]] research::ResearchRequest resolve_research(const Arguments&);
void validate_research_arguments(const Arguments&);
[[nodiscard]] int ride_command(const Arguments&);
[[nodiscard]] int research_command(const Arguments&);
[[nodiscard]] int run_research(research::ResearchRequest, const Arguments&, bool viewer);
[[nodiscard]] int settle_command(const Arguments&);
[[nodiscard]] int export_command(const Arguments&);
[[nodiscard]] int simulation_command(const Arguments&);
[[nodiscard]] int playground_command();
[[nodiscard]] int replay_command(const Arguments&);
[[nodiscard]] int batch_command(const Arguments&);
[[nodiscard]] int dispatch(std::string_view executable, std::span<const std::string_view> arguments);
[[nodiscard]] std::filesystem::path executable_path();
} // namespace bike::cli
