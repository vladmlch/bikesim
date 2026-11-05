#include "support/fixture_test.hpp"
#include "support/temporary_directory.hpp"
#include "cli/application.hpp"
#include "cli/physical_capture.hpp"
#include "cli/viewer_options.hpp"
#include "io/hash.hpp"
#include "io/npy.hpp"
#include "platform/argv.hpp"
#include "platform/process.hpp"
#include "runtime/setup_values.hpp"
#include "terrain/profile.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
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
void write_file(const std::filesystem::path& path, std::string_view body) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << body;
    bt::require(static_cast<bool>(out), "test writes " + path.string());
}
// The pose bike-settle publishes is the file a ride pins through
// physics.equilibrium_state_file. The fingerprint binds the settle contract
// (max_steps, tolerance), so bike-settle's defaults must be the ones a ride
// settles with; otherwise every bike-settle pose is "another configuration".
void settled_pose_loads_into_ride(const std::filesystem::path& bin) {
    const bt::TemporaryDirectory dir("cli_settle_pose");
    for (const auto* name : {"rider_joint_envelope_anatomical.json", "rider_strength_reference.json"})
        std::filesystem::copy_file(bt::fixture(std::string("config/") + name), dir.path() / name);
    // Cache off on both sides: the settle must solve and the ride must take
    // the pose from the pinned file, never from a shared auto-cache entry.
    auto physics = bt::text(bt::fixture("config/viewer_physics_welded.toml"));
    const std::string cached = "equilibrium_cache_enabled = true";
    const auto at = physics.find(cached);
    bt::require(at != std::string::npos, "welded fixture declares the equilibrium cache switch");
    physics.replace(at, cached.size(), "equilibrium_cache_enabled = false");
    write_file(dir.path() / "settle.toml", physics);
    write_file(dir.path() / "ride.toml", "equilibrium_state_file = \"pose.npz\"\n" + physics);

    const auto settle = bike::platform::run({(bin / "bike-settle").string(),
        "--physics-config", (dir.path() / "settle.toml").string(), "--out", (dir.path() / "pose.npz").string()}, dir.path());
    bt::require(settle.exit_code == 0 && std::filesystem::exists(dir.path() / "pose.npz"),
        "bike-settle publishes the pose (" + settle.err + ")");
    const auto ride = bike::platform::run({(bin / "bike-ride").string(), "--backend", "native", "--physics", "physical",
        "--headless", "--duration", "0.00125", "--no-plots", "--check-policy", "diagnostic",
        "--physics-config", (dir.path() / "ride.toml").string(), "--out", (dir.path() / "ride").string()}, dir.path());
    bt::require(ride.err.find("fingerprint mismatch") == std::string::npos,
        "a ride accepts the pose bike-settle wrote for the same configuration (" + ride.err + ")");
    bt::require(ride.exit_code == 0, "the pinned ride completes (" + ride.err + ")");
}
}
int main(int argc, char** argv) {
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
        bt::invalid([] { (void)resolve_ride(parse("bike-ride", {"--physics", "physical", "--headless", "--time-scale", "2", "--duration", ".1"})); });
        close_on_finish_contract();
        // The stand simulator was removed (2026-11-04): bike-sim has no
        // --playground option and bike-playground is not a built command.
        bt::invalid([] { (void)parse("bike-sim", {"--playground"}); });
        bt::require(!std::ranges::contains(command_names(), "bike-playground"),
            "bike-playground is no longer a built command");
        // Real command dispatch is registered for later execution. No simulator
        // is constructed by help, including the standalone analyst executable.
        bt::require(argc == 3, "command directory and bike-native-ride are supplied by CTest");
        const auto raw=bike::platform::main_arguments(argc,argv);
        const auto bin = std::filesystem::path(raw[1]);
        for (const auto& name : command_names()) {
            const auto help = bike::platform::run({(bin / name).string(), "--help"}, bin);
            bt::require(help.exit_code == 0 && help.out.find("usage:") != std::string::npos, name + " help dispatch");
        }
        const auto rejected = bike::platform::run({(bin / "bike-research").string(), "--duration", "not-a-number"}, bin);
        bt::require(rejected.exit_code == 2 && !rejected.err.empty(), "configuration errors keep CLI exit 2");
        // bike-native-ride owns its option parser: the removed legacy-only
        // flags are unknown options now, and help drops the legacy block.
        const auto native_ride = std::filesystem::path(raw[2]);
        for (const auto* flag : {"--legacy-drive", "--assist-mode", "--legacy-tyre", "--legacy-tyre-tier"}) {
            const auto removed = bike::platform::run({native_ride.string(),
                "--physics-config", bt::fixture("config/viewer_physics_welded.toml").string(), flag, "motor"}, bin);
            bt::require(removed.exit_code == 2 && removed.err.find("unknown option") != std::string::npos,
                std::string("bike-native-ride rejects ") + flag + " as unknown (" + removed.err + ")");
        }
        const auto native_help = bike::platform::run({native_ride.string(), "--help"}, bin);
        bt::require(native_help.exit_code == 0 && native_help.out.find("Legacy mode") == std::string::npos,
            "bike-native-ride help lists no legacy option block");
        settled_pose_loads_into_ride(bin);
        bike::BikeConfig first, second;
        first.damper_clicks.fork_hsc = 0;
        bt::require(second.damper_clicks.fork_hsc == 2, "BikeConfig owns independent nested defaults");
        compiled_capture();
    });
}
