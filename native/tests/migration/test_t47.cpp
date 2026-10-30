#include "support/migration_test.hpp"
#include "cli/application.hpp"
#include "cli/physical_capture.hpp"
#include "cli/viewer_options.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "platform/argv.hpp"
#include "platform/process.hpp"
#include "research/lifecycle.hpp"
#include "research/policy.hpp"
#include "runtime/setup_values.hpp"
#include "terrain/profile.hpp"
#include <algorithm>
#include <array>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <vector>

namespace {
bike::cli::Arguments parse(std::string_view name, std::initializer_list<std::string_view> input) {
    const auto arguments = std::vector<std::string_view>(input);
    return bike::cli::parse_arguments(name, std::span<const std::string_view>(arguments));
}
void rejected_close_on_finish(std::initializer_list<std::string_view> input, std::string_view message) {
    bool rejected = false;
    try {
        (void)bike::cli::resolve_ride(parse("bike-ride", input));
    } catch (const bike::cli::ArgumentError& error) {
        rejected = true;
        bt::require(std::string_view(error.what()).find(message) != std::string_view::npos,
            "close-on-finish reports the violated CLI contract");
    }
    bt::require(rejected, "invalid close-on-finish request throws ArgumentError");
}
void close_on_finish_contract() {
    using namespace bike::cli;
    rejected_close_on_finish({"--physics", "physical", "--close-on-finish"},
        "--close-on-finish requires --duration");
    constexpr std::string_view interactive = "--close-on-finish requires an interactive physical or research run";
    rejected_close_on_finish({"--physics", "physical", "--duration", "1", "--headless", "--close-on-finish"}, interactive);
    rejected_close_on_finish({"--physics", "physical", "--duration", "1", "--preview", "--close-on-finish"}, interactive);
    rejected_close_on_finish({"--physics", "legacy", "--duration", "1", "--close-on-finish"}, interactive);
    bt::invalid([] { (void)parse("bike-ride", {"--close-on-finish=true"}); });
    bt::invalid([] { (void)resolve_ride(parse("bike-ride", {"--physics", "physical", "--duration", "0", "--close-on-finish"})); });
    const auto physical = resolve_ride(parse("bike-ride", {"--physics", "physical", "--duration", "1", "--close-on-finish"}));
    const auto physical_options = ride_viewer_options(physical->arguments, .00125);
    bt::require(physical_options.close_on_terminal && physical_options.duration_s == std::optional<double>(1.) &&
        physical_options.mode == bike::viewer::Mode::physical,
        "physical launch receives automatic terminal closure and its duration");
    const auto research = resolve_ride(parse("bike-ride", {"--physics", "physical", "--drive", "crank_effort",
        "--research", "--duration", "1", "--close-on-finish"}));
    const auto research_options = ride_viewer_options(research->arguments, .00125, bike::viewer::Mode::research);
    bt::require(research_options.close_on_terminal && !research_options.duration_s &&
        research_options.mode == bike::viewer::Mode::research,
        "research launch closes at its runtime-owned terminal boundary without a second duration cap");
    const auto held = resolve_ride(parse("bike-ride", {"--physics", "physical", "--duration", "1"}));
    bt::require(!held->arguments.flag("close_on_finish") && !ride_viewer_options(held->arguments, .00125).close_on_terminal,
        "interactive terminal hold remains the default");
    bt::require(help_text("bike-ride").find("--close-on-finish") != std::string::npos,
        "native ride help advertises close-on-finish");
}
void compiled_capture() {
    using namespace runtime::setupvalue;
    auto owned_request=bike::cli::resolve_ride(parse("bike-ride",{"--physics","physical","--drive","ideal_speed_control","--speed","18","--record"}));
    auto& request=owned_request->setup;
    request.track=bike::terrain::flat(12.);
    request.track.obstacles.push_back(bike::terrain::Obstacle{.shape=bike::terrain::Bump{.start_m=8.,.height_m=.031234567,.bump_length_m=.7},.integer_scalar_fields={}});
    request.model.field=bike::terrain::HeightFieldSpec::for_track(request.track);
    request.model.field.ncol=401;
    request.model.physics.value().equilibrium_cache_enabled=false; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    const auto prepared=bike::app::prepare(std::make_unique<bike::initialization::SetupRequest>(request));
    const auto metadata=bike::cli::launch_metadata(request,*prepared);
    const auto& resolved=field(metadata,"resolved_config");
    bt::require(bike::io::as_double(field(resolved,"target_speed_kmh"),"target")==18. &&
        bike::io::as_bool(field(metadata,"external_speed_controller"),"speed controller") &&
        bike::io::is_null(field(metadata,"seed")),"ordinary physical identity keeps the actual ideal-speed target and omitted seed");
    for(const auto key:{"mass_budget","rider","active_suspension","compiled_contact_solver","compiled_equality_solref","compiled_equality_solimp"})
        bt::require(find(resolved,key)!=nullptr,"ordinary capture includes resolved and compiled physical settings");
    const auto assets=bike::cli::physical_assets(request,*prepared,metadata);
    const auto vertex_asset=std::ranges::find_if(assets,[](const auto& asset){return asset.name=="terrain_vertices.npy";});
    bt::require(vertex_asset!=assets.end(),"physical capture owns terrain bytes");
    const auto vertices=bike::io::parse_npy(vertex_asset->content);
    bt::require(vertices.shape==std::vector<std::size_t>{401,2},"recorded vertices use compiled heightfield columns rather than an arbitrary authored profile resolution");
    bt::require(bike::io::sha256(vertices.bytes)==bike::io::as_string(field(metadata,"terrain_sha256"),"terrain hash") &&
        vertex_asset->sha256==bike::io::sha256(vertex_asset->content),"raw compiled double vertices and encoded NPY bytes retain their distinct actual hashes");
    auto wrong=metadata;set(wrong,"terrain_sha256",text(std::string(64,'f')));
    bt::invalid([&]{(void)bike::cli::physical_assets(request,*prepared,wrong);});
}
[[gnu::noinline]] void failure_preservation_contract() {
    // The emit seam keeps untrusted plugin payloads out of the human
    // stream: a synthetic PolicyFailure carries a fixed description, a
    // stable error code and the rejected policy identifier — never the
    // plugin-provided text. The exception itself is not rewritten.
    {
        std::ostringstream human;
        bike::cli::emit_failure_diagnostic(human, "research.run.failed",
            std::make_exception_ptr(bike::research::PolicyFailure(
                "policy act threw: token=secret-sentinel")),
            std::string_view("plugin:/tmp/fake.so"));
        const std::string line = human.str();
        bt::require(line.find("secret-sentinel") == std::string::npos,
            "plugin-provided payloads never reach the human stream");
        bt::require(line.starts_with("ERROR research.policy.rejected: native policy rejected") &&
            line.find("error_code=\"policy_failure\"") != std::string::npos &&
            line.find("field_path=\"plugin:/tmp/fake.so\"") != std::string::npos,
            "the bounded policy diagnostic carries a stable code and safe identifier");
        std::ostringstream column;
        bike::cli::emit_failure_diagnostic(column, "research.run.failed",
            std::make_exception_ptr(std::invalid_argument(
                "recording requires column 'demand_nm'")));
        bt::require(column.str() ==
            "ERROR research.run.failed: recording requires column 'demand_nm'\n",
            "first-party messages are preserved and identify the bad column");
    }
    // The finalization seam keeps the first exception as primary and
    // records every later failure as a distinguishable secondary
    // "operation: cause" — close always runs, seal retries at most once.
    {
        bike::research::FailureReport failures;
        const auto primary = std::make_exception_ptr(
            std::invalid_argument("primary field"));
        failures.add("simulation", primary);
        failures.add("preview_close",
            std::make_exception_ptr(std::runtime_error("disk full")));
        bt::require(failures.primary == primary,
            "primary identity is preserved");
        bt::require(failures.secondary == std::vector<std::string>{
            "preview_close: disk full"},
            "secondary keeps operation and cause");
        bt::invalid([&] { failures.rethrow_if_failed(); });
    }
    {
        int seals = 0;
        bool closed = false;
        bike::cli::PhysicalFinalizationActions actions;
        actions.seal = [&] {
            if (++seals == 1) throw std::runtime_error("seal first");
        };
        actions.preview_close = [] [[noreturn]] () {
            throw std::runtime_error("preview full");
        };
        actions.runtime_close = [&] { closed = true; };
        const auto primary = std::make_exception_ptr(
            std::invalid_argument("simulation"));
        const auto report = bike::cli::finalize_physical_run(actions,
            primary);
        bt::require(report.primary == primary && seals == 2 && closed,
            "retry only seal and always close without replacing primary");
        bt::require(report.secondary == std::vector<std::string>{
            "seal: seal first", "preview_close: preview full"},
            "successful seal retry does not discard the first failed attempt");
    }
    {
        bool closed = false;
        bike::cli::PhysicalFinalizationActions actions;
        actions.seal = [] {};
        actions.recording_publish = [] [[noreturn]] () {
            throw std::runtime_error("publish denied");
        };
        actions.runtime_close = [&] { closed = true; };
        const auto primary = std::make_exception_ptr(
            std::runtime_error("simulation ended"));
        const auto report = bike::cli::finalize_physical_run(actions,
            primary);
        bt::require(report.primary == primary && closed &&
            report.secondary == std::vector<std::string>{
                "recording_publish: publish denied"},
            "a publication failure is a distinguishable secondary");
    }
    {
        bike::cli::PhysicalFinalizationActions actions;
        actions.seal = [] {};
        actions.runtime_close = [] [[noreturn]] () {
            throw std::invalid_argument("close broke");
        };
        const auto report = bike::cli::finalize_physical_run(actions);
        bt::require(report.primary && report.secondary.empty(),
            "a lone cleanup failure becomes the primary");
        bt::invalid([&] { report.rethrow_if_failed(); });
    }
    {
        bool sealed = false, closed = false;
        bike::cli::PhysicalFinalizationActions actions;
        actions.seal = [&] { sealed = true; };
        actions.runtime_close = [&] { closed = true; };
        const auto report = bike::cli::finalize_physical_run(actions);
        bt::require(!report.primary && report.secondary.empty() &&
            sealed && closed,
            "a clean finalization produces no diagnostic events");
    }
}
}
int main(int argc, char** argv) {
    // bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
    // cppcheck-suppress throwInEntryPoint
    return bt::run([&] {
        using namespace bike::cli;
        const auto off = parse("bike-ride", {"--physics", "physical", "--headless", "--duration", ".02", "--out", "résultats/путь с пробелами"});
        bt::require(!off.flag("record") && off.text("out") == "résultats/путь с пробелами", "output path never enables recording and retains UTF-8/whitespace");
        const auto resolved = resolve_ride(off);
        bt::require(resolved->setup.runtime.record_retention == runtime::RetentionMode::off && resolved->setup.runtime.strict,
            "unrecorded headless physical run still checks model limits");
        const auto recorded = resolve_ride(parse("bike-ride", {"--physics", "physical", "--record"}));
        bt::require(recorded->setup.runtime.record_retention == runtime::RetentionMode::full && !recorded->setup.runtime.strict,
            "recording policy is independent of interactive diagnostic checks");
        const auto research = parse("bike-research", {"--out", "unchanged"});
        bt::require(!research.flag("record") && research.text("backend") == "native", "native research default and recording opt-in");
        const auto explicit_research=parse("bike-research",{"--scenario","rough_uphill","--rider","articulated_planar","--check-policy","strict","--posture","neutral","--transmission","ideal_mid_drive"});
        bt::require(explicit_research.text("scenario")=="rough_uphill" && explicit_research.text("rider")=="articulated_planar" &&
            explicit_research.text("check_policy")=="strict" && explicit_research.text("posture")=="neutral" &&
            explicit_research.text("transmission")=="ideal_mid_drive","captured string choices are decoded values rather than quoted repr tokens");
        bt::invalid([]{(void)parse("bike-research",{"--scenario","'rough_uphill'"});});
        bt::require(parse("bike-export", {"-oresult", "--json-only"}).text("output_dir") == "result", "short attached path");
        bt::require(parse("bike-batch", {"--spec", "s.toml", "--out", "r", "-j2"}).integer("jobs") == 2, "short worker syntax");
        bt::require(parse("bike-replay", {"recording", "--time-scale", " +02 "}).integer("time_scale") == 2, "numeric choices compare the converted integer value");
        bt::require(parse("bike-research", {"--seed", "1_000"}).text("seed") == "1000", "integer flags preserve accepted separator syntax without narrowing");
        bt::require(parse("bike-ride", {"--duration=1", "--duration", "2"}).real("duration") == 2., "last override wins");
        bt::require(parse("bike-replay", {"--", "--recording-directory"}).text("recording") == "--recording-directory", "positional option terminator");
        bt::invalid([] { (void)parse("bike-ride", {"--backend", "python"}); });
        bt::invalid([] { (void)parse("bike-replay", {"recording", "--time-scale", "3"}); });
        bt::invalid([] { (void)parse("bike-settle", {"--physics", "physical"}); });
        bt::invalid([] { (void)parse("bike-ride", {"--record=true"}); });
        bt::invalid([] { (void)parse("bike-research", {"--duration", "nan"}); });
        bt::invalid([] { (void)resolve_ride(parse("bike-ride", {"--record"})); });
        bt::invalid([] { (void)resolve_ride(parse("bike-ride", {"--physics", "physical", "--headless", "--time-scale", "2", "--duration", ".1"})); });
        close_on_finish_contract();
        // Real command dispatch is registered for later execution. No simulator
        // is constructed by help, including the standalone analyst executable.
        bt::require(argc == 2, "command directory is supplied by CTest");
        const auto raw=bike::platform::main_arguments(argc,argv);
        const auto bin = std::filesystem::path(raw[1]);
        for (const auto& name : command_names()) {
            const auto help = bike::platform::run({(bin / name).string(), "--help"}, bin);
            bt::require(help.exit_code == 0 && help.stdout_text.find("usage:") != std::string::npos && help.stderr_text.empty(),
                name + " help dispatch writes product text only to stdout");
        }
        const auto rejected = bike::platform::run({(bin / "bike-research").string(), "--duration", "not-a-number"}, bin);
        bt::require(rejected.exit_code == 2 && rejected.stdout_text.empty() &&
            rejected.stderr_text.starts_with("ERROR cli.argument_error: ") &&
            rejected.stderr_text.find("--duration") != std::string::npos &&
            rejected.stderr_text.find("command=\"bike-research\"") != std::string::npos,
            "invalid input is a status 2 'LEVEL event: message key=value' stderr diagnostic with empty stdout");
        const auto runtime_failure = bike::platform::run({(bin / "bike-ride").string(), "--physics", "physical",
            "--headless", "--duration", ".001", "--track", "definitely-missing-track-file.toml"}, bin);
        bt::require(runtime_failure.exit_code == 1 && runtime_failure.stdout_text.empty() &&
            runtime_failure.stderr_text.starts_with("ERROR cli.command_failed: ") &&
            runtime_failure.stderr_text.find("platform::open_regular") != std::string::npos &&
            runtime_failure.stderr_text.find("command=\"bike-ride\"") != std::string::npos,
            "a runtime failure is a distinct status 1 'LEVEL event: message key=value' diagnostic, not a parse error");
        failure_preservation_contract();
#ifdef BIKE_HAVE_VALIDATE
        // bike-validate answers invalid input with a machine-readable JSON
        // error envelope on stderr — a product contract that is never routed
        // through human diagnostic prose. The executable is registered only
        // in the root graph, where CMake defines BIKE_HAVE_VALIDATE.
        const auto validated = bike::platform::run({(bin / "bike-validate").string(), "definitely-missing-module"}, bin);
        bt::require(validated.exit_code == 2 && validated.stdout_text.empty(),
            "validator input failure keeps status 2 and clean stdout");
        const auto envelope = bike::io::parse_json(validated.stderr_text);
        bt::require(!bike::io::as_bool(runtime::setupvalue::field(envelope, "accepted"), "accepted") &&
            bike::io::as_string(runtime::setupvalue::field(envelope, "error"), "error")
                .find("unknown validation case") != std::string::npos,
            "machine-readable stderr is a parseable error envelope");
#endif
        // Cancellation is signal-encoded (128 + SIGINT), distinct from parse
        // (2) and runtime (1) statuses.
        std::stop_source stop;
        stop.request_stop();
        const auto cancelled = bike::platform::run({(bin / "bike-test-child").string(), "sleep", "5000"}, bin,
            bike::platform::ProcessOptions{.cancellation = stop.get_token(),
                                           .timeout_s = std::nullopt});
        bt::require(cancelled.exit_code == 130 && cancelled.stdout_text.empty() && cancelled.stderr_text.empty(),
            "a pre-cancelled subprocess reports the cancellation status without stream bytes");
        bike::BikeConfig first, second;
        first.damper_clicks.fork_hsc = 0;
        bt::require(second.damper_clicks.fork_hsc == 2, "BikeConfig owns independent nested defaults");
        compiled_capture();
    });
}
