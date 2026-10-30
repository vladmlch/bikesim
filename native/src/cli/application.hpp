#pragma once
#include "arguments.hpp"
#include "../app/config.hpp"
#include "../initialization/setup_owner.hpp"
#include "../research/environment.hpp"
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <string_view>

namespace bike::cli {
struct ResolvedRide { Arguments arguments; initialization::SetupRequest setup; };
[[nodiscard]] std::unique_ptr<ResolvedRide> resolve_ride(Arguments, bool settling = false);
[[nodiscard]] terrain::TrackSpec resolve_track(std::string_view, std::optional<std::string> seed = {},
    std::optional<double> length = {});
[[nodiscard]] std::unique_ptr<research::ResearchRequest> resolve_research(const Arguments&);
void validate_research_arguments(const Arguments&);
[[nodiscard]] int ride_command(const Arguments&);
[[nodiscard]] int research_command(const Arguments&);
[[nodiscard]] int run_research(std::unique_ptr<research::ResearchRequest>, const Arguments&, bool viewer);
[[nodiscard]] int settle_command(const Arguments&);
[[nodiscard]] int export_command(const Arguments&);
[[nodiscard]] int simulation_command(const Arguments&);
[[nodiscard]] int playground_command();
[[nodiscard]] int replay_command(const Arguments&);
[[nodiscard]] int batch_command(const Arguments&);
// Human diagnostic for a captured failure: first-party exception text is the
// message; untrusted plugin payloads (research::PolicyFailure) degrade to a
// fixed description with a stable error_code and the rejected policy
// identifier — plugin-provided text never reaches the human stream. The
// exception object is never rewritten. The formatter's stream failure is
// reported by its own return value and never retried.
void emit_failure_diagnostic(std::ostream& output, std::string_view event,
    const std::exception_ptr& failure,
    std::optional<std::string_view> policy = std::nullopt);
[[nodiscard]] int dispatch(std::string_view executable, std::span<const std::string_view> arguments);
[[nodiscard]] std::filesystem::path executable_path();
} // namespace bike::cli
