#pragma once
#include "../io/value.hpp"
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace bike::batch {
struct Job {
    // Spec-scoped identity: `id` is the expanded job's unique name within
    // this batch and `index` its expansion order — the report's "name" and
    // "index" keys serialize them verbatim; no global run id is invented.
    std::string id;
    std::vector<std::string> argv;
    std::size_t index = 0;
    io::Value params{io::Value::Object{}};
    std::filesystem::path output{};
    // User tokens before the native executable/output suffix is attached.
    std::optional<std::vector<std::string>> requested_argv{};
};
enum class SummaryStatus : std::uint8_t { missing, malformed, valid };
struct ParsedSummary {
    SummaryStatus status = SummaryStatus::missing;
    io::Value value;
};
struct Result {
    std::string id;
    int exit_code = 0;
    io::Value summary;
    SummaryStatus summary_status = SummaryStatus::missing;
    double wall_duration_s = 0.;
    std::string stdout_text, stderr_text;
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
[[nodiscard]] unsigned default_worker_count();
// Extract the terminal summary candidate from worker stdout: the last line
// that begins '{' plus the rest of the stream. `missing` means no candidate
// existed; `malformed` means the candidate was rejected (including trailing
// text after valid JSON — no earlier "convenient" JSON is sought).
[[nodiscard]] ParsedSummary parse_stdout_summary(std::string_view stdout_text);
[[nodiscard]] std::vector<std::string> parse_seeds(std::string_view);
[[nodiscard]] io::Value result_record(const Job&, const Result&);
[[nodiscard]] io::Value summarize(std::span<const Job>, std::span<const Result>,
    double elapsed_s, const std::filesystem::path&, bool interrupted,
    unsigned workers = 0);
[[nodiscard]] int research_sweep(std::span<const std::string_view>, const std::filesystem::path& executable);
} // namespace bike::batch
