#pragma once
#include "../io/value.hpp"
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <string_view>
#include <vector>

namespace bike::batch {
struct Job {
    std::string id;
    std::vector<std::string> argv;
    std::size_t index = 0;
    io::Value params{io::Value::Object{}};
    std::filesystem::path output{};
    // User tokens before the native executable/output suffix is attached.
    std::optional<std::vector<std::string>> requested_argv{};
};
struct Result {
    std::string id;
    int exit_code = 0;
    io::Value summary;
    double wall_s = 0.;
    std::string out, err;
};
struct ExecuteOptions {
    std::filesystem::path cwd = std::filesystem::current_path();
    std::stop_token cancellation;
    std::function<bool()> cancelled;
    std::function<void(std::size_t, const Result&)> completed;
};
[[nodiscard]] std::vector<Job> expand_spec(const io::Value&);
// Preserve arbitrary-width TOML integer values and source table/key order.
// The existing ordered TOML parser remains authoritative for syntax/types.
[[nodiscard]] io::Value parse_spec_toml(std::string_view, std::string_view origin = "batch");
[[nodiscard]] std::vector<Result> execute(std::span<const Job>, unsigned workers,
    const ExecuteOptions& options = {});
[[nodiscard]] unsigned performance_cores();
[[nodiscard]] io::Value tail_json(std::string_view);
[[nodiscard]] std::vector<std::string> parse_seeds(std::string_view);
[[nodiscard]] io::Value result_record(const Job&, const Result&);
[[nodiscard]] io::Value summarize(std::span<const Job>, std::span<const Result>,
    double elapsed_s, const std::filesystem::path&, bool interrupted);
[[nodiscard]] int research_sweep(std::span<const std::string_view>, const std::filesystem::path& executable);
} // namespace bike::batch
