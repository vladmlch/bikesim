// test_ride_backend_cli.cpp — C++ port of
// tests/reference/test_ride_backend_cli.py (T52 batch 3.24b).
//
// The oracle drives bike_sim.cli.{ride,research,replay} parse_args/main
// with monkeypatched route stubs; the native port drives the production
// seams — cli::parse_arguments + cli::resolve_ride (the pair every
// executable main() reaches through cli::dispatch) — and, for the route
// and missing-input records, the produced bike-ride/bike-research/
// bike-replay binaries of the same build through bike::platform::run (the
// bounded argv boundary: no shell, concurrent stream drain, killpg
// cleanup, monotonic deadline). The generated bike_ride_cli_paths.hpp
// supplies the three executable paths — never a quoted compile
// definition (the bike_native_bench_path.hpp pattern).
//
// Oracle→native semantic deltas, each discharged at its assertion:
// - The oracle's --backend accepts ('python','native') and defaults to
//   'python'; the native contract declares the single choice 'native'
//   (default 'native'), so a Python-backend spelling is an invalid choice
//   — the stronger structural form of the oracle's backend assertions.
// - argparse rejection is SystemExit(2); natively that is
//   cli::ArgumentError (a std::invalid_argument) at the parse/resolve seam
//   and exit code 2 at the dispatch boundary.
// - The oracle's missing-artifact surface is the Python extension build
//   directory (BIKE_NATIVE_BUILD_PATH, dev.py repair commands, python=/
//   soabi= tags); the native executable IS the artifact, so the ported
//   contract asserts a missing required input file exits nonzero naming
//   the requested path.
#include "support/migration_test.hpp"
#include "support/golden.hpp"
#include "support/temporary_directory.hpp"
#include "cli/application.hpp"
#include "cli/arguments.hpp"
#include "cli/viewer_options.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "io/json.hpp"
#include "platform/process.hpp"
#include "viewer/session.hpp"

#include <filesystem>
#include <initializer_list>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "bike_ride_cli_paths.hpp"

#if !defined(BIKE_RIDE_PATH) || !defined(BIKE_RESEARCH_PATH) || !defined(BIKE_REPLAY_PATH)
#error "bike_ride_cli_paths.hpp must define BIKE_RIDE_PATH, BIKE_RESEARCH_PATH and BIKE_REPLAY_PATH"
#endif

namespace {

namespace cli = bike::cli;
namespace io = bike::io;

// REPO_ROOT in the pytest module: the checkout root holding
// examples/research/viewer_physics_welded.toml (the oracle's PROFILE).
// Derived from the CMake-pinned golden root — never the process cwd.
[[nodiscard]] const std::filesystem::path &repo_root() {
    static const std::filesystem::path root = std::filesystem::weakly_canonical(
        bt::oracle::root() / ".." / ".." / "..");
    return root;
}

const std::string PROFILE =
    (repo_root() / "examples/research/viewer_physics_welded.toml").string();

cli::Arguments parse(std::string_view command,
                     std::initializer_list<std::string_view> input) {
    const auto arguments = std::vector<std::string_view>(input);
    return cli::parse_arguments(command,
                                std::span<const std::string_view>(arguments));
}

// parse_args(argv) that lands the post-resolution semantics — resolve_ride
// validates combinations and fills the resolved request like the oracle's
// parse_args fills resolved_physics/resolved_check_policy.
std::unique_ptr<cli::ResolvedRide>
resolve(std::initializer_list<std::string_view> input) {
    return cli::resolve_ride(parse("bike-ride", input));
}

// pytest.raises(..., match=...) — exception type plus a message fragment.
template <class Ex, class F>
void throws(F &&action, std::string_view needle, std::string_view message) {
    try {
        std::forward<F>(action)();
    } catch (const Ex &failure) {
        bt::require(needle.empty() ||
                        std::string_view(failure.what()).find(needle) !=
                            std::string_view::npos,
                    std::string(message) + ": unexpected message '" +
                        failure.what() + "'");
        return;
    }
    bt::require(false, std::string(message) + ": no exception was raised");
}

// The resolved setup request always engages model.physics for the physical
// profiles used here; .value() is the loud contract on the request schema.
double timestep(const cli::ResolvedRide &resolved) {
    return resolved.setup.model.physics.value().timestep_s; // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
}

// Whole-physics comparison the oracle writes as resolved_physics ==
// resolved_physics: the canonical plain document of the resolved config.
std::string physics_document(const cli::ResolvedRide &resolved) {
    return io::canonical_json(bike::configuration::detail::plain_document(
        bike::configuration::to_value(
            resolved.setup.model.physics.value()))); // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
}

struct CliResult {
    int code = -1;
    std::string out;
    std::string err;
};

// subprocess.run([binary, *args]) — argv boundary, repo-root cwd, bounded
// deadline. The oracle's spawned surface is a Python module main; the
// native surface is the produced executable of the same build.
[[nodiscard]] CliResult run_cli(const std::filesystem::path &binary,
                               std::initializer_list<std::string> args) {
    std::vector<std::string> argv;
    argv.reserve(args.size() + 1);
    argv.push_back(binary.string());
    argv.insert(argv.end(), args.begin(), args.end());
    bike::platform::ProcessOptions options;
    options.timeout_s = 120.;
    const bike::platform::ProcessResult result =
        bike::platform::run(argv, repo_root(), options);
    return {.code = result.exit_code, .out = result.out, .err = result.err};
}

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

// ---------------------------------------------------------------------------
// tests/reference/test_ride_backend_cli.py::test_default_backend_and_profile_timestep
void test_default_backend_and_profile_timestep() {
    const auto args = resolve({"--physics-config", PROFILE});
    bt::require(args->arguments.text("backend") == "native",
                "the resolved default backend is the only spellable choice");
    bt::require(args->arguments.text("time_scale") == "1",
                "playback scale defaults to 1");
    bt::require(timestep(*args) == .00125,
                "the welded profile resolves its declared timestep");
    // The oracle's default backend is 'python'. Natively 'python' is not a
    // spellable backend — the contract declares the single 'native' choice,
    // so the Python-backend route cannot be selected at all (the structural
    // form of the oracle's backend check; also the discharge for the
    // dropped test_python_command_never_loads_the_extension record).
    bt::invalid([] { (void)parse("bike-ride", {"--backend", "python"}); });
}

// tests/reference/test_ride_backend_cli.py::test_native_visual_scale_keeps_all_physics_settings
void test_native_visual_scale_keeps_all_physics_settings() {
    const auto normal = resolve({"--physics-config", PROFILE});
    const auto fast = resolve({"--backend", "native", "--physics-config",
                               PROFILE, "--time-scale", "4"});
    bt::require(fast->arguments.integer("time_scale") == 4,
                "an explicit interactive scale is retained");
    bt::require(physics_document(*fast) == physics_document(*normal),
                "--time-scale 4 changes only the viewer clock; every "
                "resolved physics setting is unchanged");
}

// tests/reference/test_ride_backend_cli.py::test_explicit_scale_is_invalid_headless_even_one[scale]
void test_explicit_scale_is_invalid_headless_even_one(std::string_view scale) {
    throws<cli::ArgumentError>(
        [&] {
            (void)resolve({"--physics-config", PROFILE, "--headless",
                           "--time-scale", std::string(scale)});
        },
        "--time-scale requires an interactive physical or research run",
        "explicit --time-scale " + std::string(scale) +
            " is invalid for a headless run");
}

// tests/reference/test_ride_backend_cli.py::test_legacy_scale_and_native_rejected
void test_legacy_scale_and_native_rejected() {
    throws<cli::ArgumentError>(
        [] { (void)resolve({"--physics", "legacy", "--time-scale", "1"}); },
        "--time-scale requires an interactive physical or research run",
        "legacy physics rejects an explicit playback scale");
    throws<cli::ArgumentError>(
        [] { (void)resolve({"--physics", "legacy", "--backend", "native"}); },
        "--backend native requires physical or research mode",
        "legacy physics rejects the native backend");
}

// tests/reference/test_ride_backend_cli.py::test_headless_native_profile_does_not_replace_timestep
void test_headless_native_profile_does_not_replace_timestep() {
    const auto args = resolve({"--backend", "native", "--physics-config",
                               PROFILE, "--headless", "--duration", "1",
                               "--no-plots"});
    bt::require(args->arguments.text("time_scale") == "1",
                "a headless native run keeps the implicit playback scale");
    bt::require(timestep(*args) == .00125,
                "the headless native run does not replace the profile "
                "timestep");
}

// tests/reference/test_ride_backend_cli.py::test_research_cli_has_backend_but_no_playback_scale
void test_research_cli_has_backend_but_no_playback_scale() {
    bt::require(parse("bike-research", {}).text("backend") == "native",
                "research resolves its only spellable backend by default");
    bt::require(
        parse("bike-research", {"--backend", "native"}).text("backend") ==
            "native",
        "the explicit native backend is accepted");
    bt::invalid([] { (void)parse("bike-research", {"--backend", "python"}); });
    throws<cli::ArgumentError>(
        [] { (void)parse("bike-research", {"--time-scale", "1"}); },
        "unrecognized arguments: --time-scale",
        "the research command has no playback-scale option");
}

// tests/reference/test_ride_backend_cli.py::test_replay_requires_viewer_for_explicit_scale
void test_replay_requires_viewer_for_explicit_scale() {
    // Oracle: replay.main(['missing-recording', '--time-scale', '1']) exits
    // SystemExit(2) — validation precedes opening the recording. Natively
    // the ArgumentError fires inside replay_command before a
    // ReplaySession/PlaybackSession exists; the missing recording would
    // otherwise exit 1 (REPLAY ERROR), so the exit-2 spelling through the
    // produced binary proves the ordering too.
    throws<cli::ArgumentError>(
        [] {
            (void)cli::replay_command(
                parse("bike-replay", {"missing-recording", "--time-scale", "1"}));
        },
        "--time-scale requires --viewer",
        "replay validation refuses a scale without --viewer");
    const auto result =
        run_cli(BIKE_REPLAY_PATH, {"missing-recording", "--time-scale", "1"});
    bt::require(result.code == 2 &&
                    contains(result.err, "--time-scale requires --viewer"),
                "the executable maps the validation to exit 2 ahead of the "
                "missing recording");
}

// tests/reference/test_ride_backend_cli.py::test_native_preflight_failure_precedes_ride_setup
void test_native_preflight_failure_precedes_ride_setup() {
    // Oracle: a refused native-artifact preflight propagates before
    // build_physical_simulation. Natively the preflight is resolve_ride's
    // validation stage (cli/configuration.cpp:116-160), which runs before
    // the setup-request stage (track/physics resolution at :164-167) and
    // long before physical_run constructs the runtime. Probe: a rejected
    // flag combination must win over a setup-stage failure — the error is
    // the validation message, never the missing-track load failure.
    bt::TemporaryDirectory temporary("ride-cli-preflight");
    const auto missing_track =
        (temporary.path() / "no-such-track.toml").string();
    throws<cli::ArgumentError>(
        [&] {
            (void)resolve({"--physics-config", PROFILE, "--backend", "native",
                           "--headless", "--duration", "1", "--no-plots",
                           "--time-scale", "1", "--track", missing_track});
        },
        "--time-scale requires an interactive physical or research run",
        "resolve-stage validation precedes setup-stage track resolution");
    // The same refusal through the produced binary is the oracle's exit(2)
    // surface — it returns before any model build would run.
    const auto result = run_cli(
        BIKE_RIDE_PATH, {"--physics-config", PROFILE, "--backend", "native",
                         "--headless", "--duration", "1", "--no-plots",
                         "--time-scale", "1"});
    bt::require(result.code == 2 &&
                    contains(result.err, "--time-scale requires an "
                                         "interactive physical or research "
                                         "run"),
                "the executable refuses at the preflight stage with the "
                "validation message");
}

// tests/reference/test_ride_backend_cli.py::test_missing_artifact_names_directory_commands_and_code
void test_missing_artifact_names_directory_commands_and_code() {
    // Oracle: BIKE_NATIVE_BUILD_PATH=<empty dir> exits 2 and stderr names
    // the directory plus the dev.py doctor/configure/build repair commands
    // and python=/soabi= tags — the Python extension-artifact surface. The
    // native executable IS the artifact, so the surviving contract is that
    // a missing required input file fails nonzero naming the requested
    // path (the repair-spelling tokens have no native analog; dispatch maps
    // the acquisition failure to exit 1, not argparse's 2).
    bt::TemporaryDirectory temporary("ride-cli-missing");
    const auto missing = (temporary.path() / "no-such-physics.toml").string();
    const auto result = run_cli(
        BIKE_RIDE_PATH,
        {"--physics-config", missing, "--backend", "native", "--headless",
         "--duration", "1", "--no-plots",
         "--out", (temporary.path() / "out").string()});
    bt::require(result.code == 1, "a missing physics config exits nonzero");
    bt::require(contains(result.err, missing),
                "stderr names the missing input path");
}

// tests/reference/test_ride_backend_cli.py::test_cli_dispatches_every_live_and_headless_route
void test_cli_dispatches_every_live_and_headless_route() {
    bt::TemporaryDirectory temporary("ride-cli-dispatch");
    const auto out = (temporary.path() / "out").string();

    // Viewer route: the oracle's run_viewer stub observed (driver,
    // out_root, time_scale=2, record=False). Natively the interactive
    // branch of physical_run consumes the same resolved arguments through
    // ride_viewer_options — the projection is the seam, not a stub (a real
    // viewer is not spawned in a headless test environment).
    const auto interactive =
        resolve({"--backend", "native", "--physics-config", PROFILE,
                 "--time-scale", "2", "--out", out});
    const auto view = cli::ride_viewer_options(interactive->arguments,
                                               timestep(*interactive));
    bt::require(view.time_scale == 2 &&
                    view.mode == bike::viewer::Mode::physical,
                "the viewer route receives the explicit playback scale");
    bt::require(!interactive->arguments.flag("headless") &&
                    !interactive->arguments.flag("record") &&
                    interactive->arguments.text("out") == out,
                "the viewer route keeps --out and leaves recording off");

    // Headless route through the real binary: the oracle's stub returned 7
    // and main propagated it; natively the headless loop's own outcome code
    // is the exit — 2 on both qualified platforms (the welded profile's
    // constraint-work violation marks the run model-invalid; a clean run
    // would be 0, so the code window is asserted while the summary contract
    // pins the route itself).
    const auto headless = run_cli(
        BIKE_RIDE_PATH,
        {"--backend", "native", "--physics-config", PROFILE, "--headless",
         "--duration", "0.02", "--no-plots", "--check-policy", "diagnostic",
         "--out", out});
    bt::require((headless.code == 0 || headless.code == 2) &&
                    contains(headless.out, "\"reason\":\"duration_reached\"") &&
                    contains(headless.out, "\"backend\":\"native\"") &&
                    contains(headless.out, "\"drive_mode\":\"articulated_effort\"") &&
                    contains(headless.out, "\"timestep_s\":0.00125"),
                "the headless route runs the physical episode to the "
                "duration boundary and prints its summary");

    // --research route through the real binary: ride_command hands the
    // resolved request to run_research (interactive=false under --headless);
    // the run-result document is the research emission, not the physical
    // ride summary.
    const auto research = run_cli(
        BIKE_RIDE_PATH,
        {"--physics-config", PROFILE, "--research", "--headless",
         "--duration", "0.02", "--check-policy", "diagnostic", "--out", out});
    bt::require((research.code == 0 || research.code == 2) &&
                    contains(research.out, "\"outcome\":\"duration\"") &&
                    contains(research.out, "\"recording\":\"off\""),
                "the research route runs the accounted episode to the "
                "duration boundary");

    // Standalone bike-research main: the oracle's research_cli.main stub
    // returned episode_exit_code (9); natively research_command returns the
    // real episode code — 0 on both qualified platforms for the lumped
    // rider profile (a model-violating episode would be 2). --rider lumped
    // keeps the equilibrium probe inside the unit-test budget; the dispatch
    // contract being asserted is rider-independent.
    const auto research_main = run_cli(
        BIKE_RESEARCH_PATH, {"--rider", "lumped", "--duration", "0.02",
                             "--out", out});
    bt::require((research_main.code == 0 || research_main.code == 2) &&
                    contains(research_main.out, "\"outcome\":\"duration\"") &&
                    contains(research_main.out, "\"recording\":\"off\""),
                "the research executable dispatches to its headless episode "
                "and returns the episode code");
}

} // namespace

int main() {
    return bt::run([] {
        test_default_backend_and_profile_timestep();
        test_native_visual_scale_keeps_all_physics_settings();
        for (const std::string_view scale : {"1", "2", "4", "8"})
            test_explicit_scale_is_invalid_headless_even_one(scale);
        test_legacy_scale_and_native_rejected();
        test_headless_native_profile_does_not_replace_timestep();
        test_research_cli_has_backend_but_no_playback_scale();
        test_replay_requires_viewer_for_explicit_scale();
        test_native_preflight_failure_precedes_ride_setup();
        test_missing_artifact_names_directory_commands_and_code();
        test_cli_dispatches_every_live_and_headless_route();
    });
}
