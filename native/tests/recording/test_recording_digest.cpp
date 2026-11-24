#include "cli/recording_digest.hpp"
#include "io/json.hpp"
#include "platform/files.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"
#include "recording/capture.hpp"
#include "recording/digest.hpp"
#include "recording/duckdb.hpp"
#include "recording/publish.hpp"
#include "recording/reader.hpp"
#include "recording/value_util.hpp"
#include "recording_reader_test_support.hpp"
#include "support/fixture_test.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace rec = bike::recording;
namespace io = bike::io;
namespace fs = std::filesystem;

rec::Catalog three_joint_catalog() {
    const std::array<rec::JointCoords, 3> joints{{
        {.name = "root_x", .joint_type = "slide", .qposadr = 0, .dofadr = 0},
        {.name = "root_z", .joint_type = "slide", .qposadr = 1, .dofadr = 1},
        {.name = "root_pitch", .joint_type = "hinge", .qposadr = 2, .dofadr = 2},
    }};
    return rec::build_catalog(joints, std::span<const rec::ScalarAlias>{});
}

runtime::OwnedNativeEpisode make_episode(
    std::size_t count, std::optional<std::size_t> bump_qpos_interval,
    std::optional<std::size_t> bump_force_interval, bool include_forces) {
    std::vector<runtime::SamplePtr> samples;
    samples.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        auto sample = std::make_shared<runtime::PhysicalSampleData>();
        sample->interval_id = static_cast<std::int64_t>(i);
        sample->time_s = static_cast<double>(i) * 0.00125;
        sample->end_time_s = static_cast<double>(i + 1) * 0.00125;
        double root_z = -0.05 + static_cast<double>(i) * 1e-4;
        if (bump_qpos_interval && i == *bump_qpos_interval) {
            root_z = std::nextafter(root_z, std::numeric_limits<double>::infinity());
        }
        sample->qpos = {static_cast<double>(i) * 0.002, root_z, -0.01};
        sample->qvel = {1.6, 0.02, -0.15};
        if (include_forces) {
            double power = 45.0 + static_cast<double>(i) * 0.25;
            if (bump_force_interval && i == *bump_force_interval) {
                power = std::nextafter(power, std::numeric_limits<double>::infinity());
            }
            sample->forces = {{"chain", {12.0, -3.5, 4.25}}};
            sample->powers_w = {{"chain", runtime::Wire{power}}};
        }
        samples.push_back(std::move(sample));
    }
    return runtime::OwnedNativeEpisode::seal(
        1, samples,
        runtime::ColumnLayout{.root_x_qpos = 0, .root_x_dof = 0, .root_pitch_qpos = 2});
}

rec::Publication publish_case(
    const fs::path &root, std::size_t count,
    std::optional<std::size_t> bump_qpos_interval,
    std::optional<std::size_t> bump_force_interval, bool include_forces) {
    const rec::RunMetadata meta{
        .run_uuid = "00000000-0000-4000-8000-000000000042",
        .started_utc = "2026-11-17T12:00:00.000000+00:00",
        .backend = "native",
        .mode = "ride",
        .values = io::Value{io::Value::Object{{"fixture", io::Value{true}}}},
        .wall_origin_s = 0.0,
    };
    rec::RunCapture capture(meta, true);
    const auto ev = std::make_shared<rec::EpisodeEvidence>();
    ev->episode_id = 0;
    ev->generation = 1;
    ev->physical = rec::make_physical_source(
        make_episode(count, bump_qpos_interval, bump_force_interval, include_forces),
        three_joint_catalog());
    ev->summary = io::Value{io::Value::Object{
        {"duration_s", rec::value::real(static_cast<double>(count) * 0.00125)},
        {"outcome", io::Value{io::Value::Object{{"reason", rec::value::text("fixture")}}}},
    }};
    capture.add_episode(ev);
    return rec::finalize(capture, root);
}

struct MissingForcesCopySpec {
    fs::path src_dir;
    fs::path dst_dir;
};

fs::path create_missing_forces_copy(const MissingForcesCopySpec &spec) {
    fs::create_directories(spec.dst_dir);
    for (const auto &entry : fs::directory_iterator(spec.src_dir)) {
        fs::copy_file(entry.path(), spec.dst_dir / entry.path().filename());
    }
    {
        const rec::Database db(spec.dst_dir / "run.duckdb", false);
        db.connection().execute("DROP TABLE forces");
        db.connection().execute("CHECKPOINT");
    }
    auto manifest = io::parse_recording_json(
        rec::detail::read_text(spec.dst_dir / "manifest.json"));
    auto file_hashes = rec::detail::field(manifest, "file_sha256");
    rec::detail::set(file_hashes, "run.duckdb",
                     rec::detail::text(bike::platform::file_sha256(spec.dst_dir / "run.duckdb")));
    rec::detail::set(manifest, "file_sha256", std::move(file_hashes));
    fs::remove(spec.dst_dir / "manifest.json");
    recording_reader_test::write(spec.dst_dir / "manifest.json", io::recording_json(manifest));
    return spec.dst_dir;
}

fs::path find_digest_binary() {
    const fs::path build_root =
        bike::platform::executable_path()
            .parent_path()
            .parent_path()
            .parent_path()
            .parent_path();
    return build_root / "bin" / "bike-recording-digest";
}

void test_digest_and_cli() {
    recording_reader_test::Directory const tmp;
    const auto base_pub = publish_case(tmp.path / "base", 50, std::nullopt, std::nullopt, true);
    const auto mut_qpos_pub = publish_case(tmp.path / "mut_qpos", 50, 42, std::nullopt, true);
    const auto mut_force_pub = publish_case(tmp.path / "mut_force", 50, std::nullopt, 17, true);
    const auto mut_coupled_pub = publish_case(tmp.path / "mut_coupled", 50, 11, 10, true);
    const auto mut_tie_pub = publish_case(tmp.path / "mut_tie", 50, 10, 10, true);
    const auto empty_force_pub = publish_case(tmp.path / "empty_force", 50, std::nullopt, std::nullopt, false);
    const auto missing_forces_dir = create_missing_forces_copy(
        {.src_dir = base_pub.directory, .dst_dir = tmp.path / "missing_forces"});

    const auto rec_a = rec::open_recording(base_pub.directory);
    const auto rec_b = rec::open_recording(mut_qpos_pub.directory);
    const auto rec_f = rec::open_recording(mut_force_pub.directory);
    const auto rec_coupled = rec::open_recording(mut_coupled_pub.directory);
    const auto rec_tie = rec::open_recording(mut_tie_pub.directory);
    const auto rec_empty_f = rec::open_recording(empty_force_pub.directory);

    const auto d_a = rec::compute_recording_digest(*rec_a);
    const auto d_b = rec::compute_recording_digest(*rec_b);
    const auto d_f = rec::compute_recording_digest(*rec_f);
    const auto d_coupled = rec::compute_recording_digest(*rec_coupled);
    const auto d_tie = rec::compute_recording_digest(*rec_tie);
    const auto d_empty_f = rec::compute_recording_digest(*rec_empty_f);

    bt::require(d_a.schema_version == 1, "digest preserves schema_version == 1");
    bt::require(d_a.tables.size() == 2, "digest covers physics and forces tables");
    for (const auto &tbl : d_a.tables) {
        bt::require(std::ranges::is_sorted(tbl.columns, {}, &rec::ColumnDigest::name),
                    "columns are emitted in canonical sorted order");
    }

    const auto diff_qpos = rec::compare_recording_digests(d_a, d_b);
    bt::require(!diff_qpos.equal, "1-ULP flip in root_z_m must fail comparison");
    bt::require(diff_qpos.table == "physics", "1-ULP flip reports physics table");
    bt::require(diff_qpos.column == "root_z_m", "1-ULP flip reports root_z_m column");
    bt::require(diff_qpos.interval_id == 42, "1-ULP flip reports interval_id 42");

    const auto diff_force = rec::compare_recording_digests(d_a, d_f);
    bt::require(!diff_force.equal, "1-ULP flip in forces.power_w must fail comparison");
    bt::require(diff_force.table == "forces", "1-ULP flip reports forces table");
    bt::require(diff_force.column == "power_w", "1-ULP flip reports power_w column");
    bt::require(diff_force.interval_id == 17, "1-ULP flip reports interval_id 17");

    // Coupled drift: forces diverges at 10 and physics diverges at 11 -> must report forces at 10
    const auto diff_coupled = rec::compare_recording_digests(d_a, d_coupled);
    bt::require(!diff_coupled.equal, "coupled forces(10)/physics(11) drift must fail comparison");
    bt::require(diff_coupled.table == "forces",
                "earlier forces drift at interval 10 beats later physics drift at interval 11");
    bt::require(diff_coupled.column == "power_w", "coupled drift reports forces.power_w");
    bt::require(diff_coupled.interval_id == 10, "coupled drift reports interval_id 10");

    // Equal-interval tie: both forces and physics diverge at 10 -> physics wins tie-break
    const auto diff_tie = rec::compare_recording_digests(d_a, d_tie);
    bt::require(!diff_tie.equal, "same-interval tie drift must fail comparison");
    bt::require(diff_tie.table == "physics", "same-interval tie reports physics first");
    bt::require(diff_tie.column == "root_z_m", "same-interval tie reports root_z_m");
    bt::require(diff_tie.interval_id == 10, "same-interval tie reports interval_id 10");

    // Column ordering invariance
    auto d_reordered = d_a;
    for (auto &tbl : d_reordered.tables) {
        std::ranges::reverse(tbl.columns);
    }
    std::ranges::reverse(d_reordered.tables);
    const auto diff_reordered = rec::compare_recording_digests(d_a, d_reordered);
    bt::require(diff_reordered.equal, "comparison is invariant to table/column ordering");

    // JSON round-trip (both parse_json and parse_recording_json)
    const std::string json_text = io::canonical_json(rec::digest_to_json(d_a));
    const auto d_rt1 = rec::digest_from_json(io::parse_json(json_text));
    const auto d_rt2 = rec::digest_from_json(io::parse_recording_json(json_text));
    bt::require(rec::compare_recording_digests(d_a, d_rt1).equal, "parse_json round-trip matches");
    bt::require(rec::compare_recording_digests(d_a, d_rt2).equal, "parse_recording_json round-trip matches");

    // Empty forces table is supported cleanly and differs from non-empty forces
    const auto diff_empty_f = rec::compare_recording_digests(d_a, d_empty_f);
    bt::require(!diff_empty_f.equal && diff_empty_f.table == "forces",
                "empty forces table differs from populated forces table");

    // Missing forces table in DB is rejected cleanly
    recording_reader_test::rejects([&] { (void)rec::open_recording(missing_forces_dir); });

    // CLI in-process checks
    {
        std::ostringstream out, err;
        const std::array<const char *, 2> argv{"bike-recording-digest", "--help"};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 0,
                    "--help exits 0");
        bt::require(!out.str().empty() && err.str().empty(), "--help writes to out only");
    }
    {
        std::ostringstream out, err;
        const std::array<const char *, 1> argv{"bike-recording-digest"};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 2,
                    "missing --run exits 2");
    }
    {
        std::ostringstream out, err;
        const std::string bad_flag = "--bogus";
        const std::array<const char *, 2> argv{"bike-recording-digest", bad_flag.c_str()};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 2,
                    "unknown option exits 2");
    }

    const fs::path golden_json = tmp.path / "golden.json";
    const std::string base_dir_str = base_pub.directory.string();
    const std::string base_duckdb = (base_pub.directory / "run.duckdb").string();
    const std::string mut_duckdb = (mut_qpos_pub.directory / "run.duckdb").string();
    const std::string missing_duckdb = (missing_forces_dir / "run.duckdb").string();
    const std::string golden_str = golden_json.string();

    {
        std::ostringstream out, err;
        const std::array<const char *, 3> argv{
            "bike-recording-digest", "--run", base_dir_str.c_str()};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 0,
                    "--run <dir> exits 0");
        const auto parsed = rec::digest_from_json(io::parse_json(out.str()));
        bt::require(rec::compare_recording_digests(d_a, parsed).equal,
                    "--run <dir> prints matching digest JSON to stdout");
    }

    {
        std::ostringstream out, err;
        const std::array<const char *, 5> argv{
            "bike-recording-digest", "--run", base_duckdb.c_str(), "--out", golden_str.c_str()};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 0,
                    "--run .../run.duckdb --out exits 0");
        bt::require(fs::exists(golden_json), "--out writes golden JSON file");
    }
    {
        std::ostringstream out, err;
        const std::array<const char *, 5> argv{
            "bike-recording-digest", "--run", base_duckdb.c_str(), "--verify", golden_str.c_str()};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 0,
                    "--verify on matching run exits 0");
    }
    {
        std::ostringstream out, err;
        const std::array<const char *, 5> argv{
            "bike-recording-digest", "--run", mut_duckdb.c_str(), "--verify", golden_str.c_str()};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 1,
                    "--verify on 1-ULP mutated run exits 1");
        const std::string msg = err.str();
        bt::require(msg.find("physics") != std::string::npos &&
                        msg.find("root_z_m") != std::string::npos &&
                        msg.find("42") != std::string::npos,
                    "mismatch stderr prints table, column, and interval_id");
    }
    {
        std::ostringstream out, err;
        const std::array<const char *, 3> argv{
            "bike-recording-digest", "--run", missing_duckdb.c_str()};
        bt::require(bike::cli::run_recording_digest(static_cast<int>(argv.size()), argv.data(), out, err) == 2,
                    "missing forces container exits 2");
    }

    // Standalone binary checks (Ruling 1)
    const fs::path bin = find_digest_binary();
    bt::require(fs::exists(bin), "bin/bike-recording-digest exists");
    const fs::path build_root = bin.parent_path().parent_path();

    const auto help_proc = bike::platform::run({bin.string(), "--help"}, build_root);
    bt::require(help_proc.exit_code == 0, "standalone bin/bike-recording-digest --help exits 0");

#if defined(__APPLE__)
    const auto otool = bike::platform::run({"otool", "-L", bin.string()}, build_root);
    bt::require(otool.exit_code == 0, "otool -L succeeds on bike-recording-digest");
    bt::require(otool.out.find("libmujoco") == std::string::npos,
                "bike-recording-digest has zero libmujoco linkage");
#else
    const auto readelf = bike::platform::run({"readelf", "-d", bin.string()}, build_root);
    bt::require(readelf.exit_code == 0, "readelf -d succeeds on bike-recording-digest");
    bt::require(readelf.out.find("libmujoco") == std::string::npos,
                "bike-recording-digest has zero libmujoco linkage");
#endif

    const auto verify_ok = bike::platform::run(
        {bin.string(), "--run", base_duckdb, "--verify", golden_str}, build_root);
    bt::require(verify_ok.exit_code == 0, "subprocess --verify match exits 0");

    const auto verify_fail = bike::platform::run(
        {bin.string(), "--run", mut_duckdb, "--verify", golden_str}, build_root);
    bt::require(verify_fail.exit_code == 1, "subprocess --verify mismatch exits 1");
}
} // namespace

int main() {
    return bt::run(test_digest_and_cli);
}
