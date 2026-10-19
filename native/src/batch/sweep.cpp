#include "plan.hpp"
#include "log.hpp"
#include "../cli/arguments.hpp"
#include "../io/json.hpp"
#include "../platform/signals.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <thread>
#include <utility>

namespace bike::batch {
int research_sweep(std::span<const std::string_view> input, const std::filesystem::path& executable) {
    using namespace runtime::setupvalue;
    cli::Arguments args;
    args.set("jobs", std::to_string(std::max(1U, std::thread::hardware_concurrency())));
    std::vector<std::string> passthrough;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const auto token = input[i];
        if (token == "-h" || token == "--help") {
            std::cout << "usage: bike-research-sweep --out PATH --seeds 1-8 [--jobs N] [--dry-run] -- [bike-research arguments]\n";
            return 0;
        }
        if (token == "--") { for (++i; i < input.size(); ++i) passthrough.emplace_back(input[i]); break; }
        if (token == "--dry-run") { args.set("dry_run", "true"); continue; }
        auto name = token; std::optional<std::string_view> value;
        if (const auto equals = token.find('='); equals != std::string_view::npos) { name = token.substr(0, equals); value = token.substr(equals + 1); }
        if (name != "--out" && name != "--seeds" && name != "--jobs") throw cli::ArgumentError("unknown sweep option: " + std::string(name));
        if (!value) { if (++i == input.size()) throw cli::ArgumentError("missing sweep option value"); value = input[i]; }
        args.set(std::string(name.substr(2)), std::string(*value));
    }
    if (!args.has("out") || !args.has("seeds")) throw cli::ArgumentError("--out and --seeds are required");
    const auto seeds = parse_seeds(args.text("seeds"));
    const auto count = std::max<std::int64_t>(1, args.integer("jobs"));
    if (std::cmp_greater(count , std::numeric_limits<unsigned>::max())) throw cli::ArgumentError("sweep worker count exceeds range");
    const auto root = std::filesystem::path(args.text("out"));
    std::vector<Job> jobs;
    for (const auto& seed : seeds) {
        Job job; job.id = "seed-" + seed; job.index = jobs.size(); job.output = root / job.id;
        job.params = Value{Object{{"seed", Value{io::Number{seed}}}}};
        job.argv = passthrough; job.argv.insert(job.argv.end(), {"--seed", seed, "--out", job.output.string()});
        std::vector<std::string_view> views; views.reserve(job.argv.size());
for (const auto& arg : job.argv) views.push_back(arg);
        (void)cli::parse_arguments("bike-research", views);
        job.requested_argv = job.argv; job.argv.insert(job.argv.begin(), executable.string());
        jobs.push_back(std::move(job));
    }
    if (args.flag("dry_run")) {
        for (const auto& job : jobs) { for (std::size_t i = 0; i < job.argv.size(); ++i) std::cout << (i ? " " : "") << job.argv[i]; std::cout << '\n'; }
        return 0;
    }
    // A sweep has no --overwrite flag: a collision must be resolved before
    // dispatching any child. One destination is owned by exactly one seed.
    if (std::filesystem::exists(root) && !std::filesystem::is_empty(root)) throw cli::ArgumentError("sweep output already exists: " + root.string());
    std::filesystem::create_directories(root);
    platform::InterruptShield interrupt;
    ExecuteOptions options;
    options.cancelled = [&] { return interrupt.deferred() != 0; };
    options.completed = [&](std::size_t index, const Result& result) {
        write_log(root / (jobs[index].id + ".log"), result.out + result.err, false);
        std::cout << "seed " << seeds[index] << ": rc=" << result.exit_code << " wall=" << result.wall_s << "s\n" << std::flush;
    };
    const auto results = execute(jobs, static_cast<unsigned>(count), options);
    Array runs, passed;
    for (const auto& token : passthrough) passed.push_back(text(token));
    for (std::size_t i = 0; i < results.size(); ++i)
        runs.push_back(Value{Object{{"seed", Value{io::Number{seeds[i]}}}, {"returncode", integer(results[i].exit_code)}, {"wall_s", real(results[i].wall_s)}}});
    write_log(root / "sweep.json", io::recording_json(Value{Object{{"jobs", integer(args.integer("jobs"))},
        {"passthrough", Value{std::move(passed)}}, {"runs", Value{std::move(runs)}}}}) + "\n", false);
    return interrupt.deferred() ? 130 : std::ranges::all_of(results, [](const auto& r) { return r.exit_code == 0; }) ? 0 : 1;
}
} // namespace bike::batch
