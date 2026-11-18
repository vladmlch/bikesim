#include "cli/gpu_export.hpp"
#include "configuration/load.hpp"
#include "golden/step_closure_bundle.hpp"
#include "io/json.hpp"
#include "model/builder.hpp"
#include "model_access.hpp"
#include "platform/argv.hpp"
#include "platform/files.hpp"
#include "platform/process.hpp"
#include "support/fixture_test.hpp"
#include "support/temporary_directory.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <ranges>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

[[nodiscard]] fs::path repo_root() { return {BIKE_REPO_ROOT}; }

void verify_profile(const std::string &name, const fs::path &physics_toml,
                    const fs::path &track_toml, std::size_t n_steps) {
    const bt::TemporaryDirectory tmp("step-closure-" + name);
    const fs::path out_dir = tmp.path() / "bundle";

    const bike::golden::StepClosureCaptureSpec spec{
        .physics = bike::configuration::load_physics(physics_toml),
        .track = bike::terrain::load_track(track_toml, std::nullopt, std::nullopt),
        .duration_s = std::nullopt,
        .max_steps = n_steps,
        .seed_decimal = "0",
        .physics_config_label = physics_toml.string(),
        .track_label = track_toml.string(),
        .asset_root = physics_toml.parent_path(),
        .strict = false,
    };

    const auto bundle = bike::golden::capture_step_closure_bundle(spec);
    bt::require(bundle->rows.size() == n_steps,
                name + ": captured expected step count");

    // Confirm braking release (dof_frictionloss changes), gear shift
    // (wrap_prm_freehub / cog_index changes), and component decomposition.
    bool saw_brake = false;
    bool saw_unbraked = false;
    std::set<int> cogs_seen;
    std::set<double> wrap_seen;
    double max_component_sum_err = 0.;
    for (const auto &row : bundle->rows) {
        if (row.dof_frictionloss_wheels[0] > 0. &&
            row.dof_frictionloss_wheels[1] > 0.) {
            saw_brake = true;
        }
        if (row.dof_frictionloss_wheels[0] == 0. &&
            row.dof_frictionloss_wheels[1] == 0.) {
            saw_unbraked = true;
        }
        cogs_seen.insert(row.cog_index);
        wrap_seen.insert(row.wrap_prm_freehub);
        for (std::size_t j = 0; j < row.qfrc_applied.size(); ++j) {
            const double sum =
                row.qfrc_tires[j] + row.qfrc_suspension[j] +
                row.qfrc_resistance[j] + row.qfrc_rider_joint_envelope[j] +
                row.qfrc_brake_hold[j] + row.qfrc_injected_other[j];
            max_component_sum_err =
                std::max(max_component_sum_err, std::abs(sum - row.qfrc_applied[j]));
        }
    }
    bt::require(saw_brake && saw_unbraked,
                name + ": exercises both brake hold and brake release");
    bt::require(cogs_seen.size() >= 2 && wrap_seen.size() >= 2,
                name + ": exercises gear shift (wrap_prm_freehub & cog_index)");
    bt::require(max_component_sum_err <= 1e-12,
                name + ": force component decomposition sums to qfrc_applied");

    bike::golden::write_step_closure_bundle(*bundle, out_dir);
    for (const char *fname :
         {"manifest.json", "model.xml", "model_12gears.xml",
          "hfield_data.bin", "physics_config.json", "step_closure.bin"}) {
        bt::require(fs::is_regular_file(out_dir / fname),
                    name + ": bundle contains " + std::string(fname));
    }

    const auto m12 = bike::model::compile_mjcf(bundle->model_12gears_xml, out_dir);
    bt::require(m12->ntendon == 13,
                name + ": model_12gears.xml has ntendon == 13");

    const auto manifest =
        bike::io::parse_json(bt::text(out_dir / "manifest.json"));
    const auto &mobj = bike::io::as_object(manifest, "manifest");
    const auto *sha_val = bike::io::find_member(mobj, "step_closure_sha256");
    bt::require(sha_val != nullptr &&
                    bike::io::as_string(*sha_val, "step_closure_sha256") ==
                        bike::platform::file_sha256(out_dir / "step_closure.bin"),
                name + ": streaming step_closure_sha256 matches file_sha256");

    const auto verification = bike::golden::verify_bundle_closure(out_dir);
    bt::require(verification.ok(),
                name + ": 100% 0-bit cold closure on next_qpos and next_qvel");
    bt::require(verification.zero_bit_qacc_steps == verification.total_steps,
                name + ": 100% 0-bit cold closure on next_qacc");
    bt::require(verification.steady_gear_checked > 0,
                name + ": steady-gear equivalence checked on >= 1 step");
    bt::require(verification.steady_gear_max_qpos_diff == 0.0 &&
                    verification.steady_gear_max_qvel_diff == 0.0,
                name + ": model_12gears.xml matches model.xml (0.0 diff) on steady gear");
}

void verify_cli_surface(const fs::path &gpu_export_bin) {
    {
        const std::array<const char *, 2> argv{"bike-gpu-export", "--help"};
        std::ostringstream out;
        std::ostringstream err;
        const int rc = bike::cli::run_gpu_export(
            static_cast<int>(argv.size()), argv.data(), out, err);
        bt::require(rc == 0, "run_gpu_export --help returns 0");
        bt::require(out.str().find("bike-gpu-export") != std::string::npos,
                    "run_gpu_export --help prints usage");
    }

    if (!gpu_export_bin.empty() && fs::exists(gpu_export_bin)) {
        const bt::TemporaryDirectory tmp("gpu-export-cli");
        const fs::path out_dir = tmp.path() / "cli_bundle";
        const auto res = bike::platform::run(
            {gpu_export_bin.string(),
             "--physics-config",
             (repo_root() / "examples/research/viewer_physics_welded.toml")
                 .string(),
             "--track",
             "flat",
             "--duration",
             "0.25",
             "--verify",
             "--out",
             out_dir.string()},
            repo_root());
        bt::require(res.exit_code == 0,
                    "bike-gpu-export subprocess --verify exited 0");
        bt::require(fs::is_regular_file(out_dir / "step_closure.bin"),
                    "bike-gpu-export wrote step_closure.bin");
    }

    {
        const std::string too_many_steps = std::to_string(
            bike::golden::max_step_closure_steps(1U, 1U, 0U, 0U, 1U) + 1U);
        const std::array<const char *, 9> argv{
            "bike-gpu-export", "--physics-config", "unused.toml",
            "--track", "flat", "--steps", too_many_steps.c_str(),
            "--out", "unused"};
        std::ostringstream out;
        std::ostringstream err;
        const int rc = bike::cli::run_gpu_export(
            static_cast<int>(argv.size()), argv.data(), out, err);
        bt::require(rc == 2,
                    "run_gpu_export rejects --steps above bundle byte cap");
    }
}

void verify_nonzero_start_grade() {
    const bt::TemporaryDirectory tmp("step-closure-start-grade");
    const fs::path out_dir = tmp.path() / "bundle";
    const fs::path physics_toml =
        repo_root() / "examples/research/viewer_physics_welded.toml";
    auto track = bike::terrain::flat(40.0);
    track.name = "const_10pct";
    track.grade_profile =
        bike::terrain::GradeProfile{{{0.0, 0.10}, {40.0, 0.10}}};

    const bike::golden::StepClosureCaptureSpec spec{
        .physics = bike::configuration::load_physics(physics_toml),
        .track = track,
        .duration_s = std::nullopt,
        .max_steps = 5U,
        .seed_decimal = "0",
        .physics_config_label = physics_toml.string(),
        .track_label = track.name,
        .asset_root = physics_toml.parent_path(),
        .strict = false,
    };

    const auto bundle = bike::golden::capture_step_closure_bundle(spec);
    bike::golden::write_step_closure_bundle(*bundle, out_dir);

    const auto physics_doc =
        bike::io::parse_json(bt::text(out_dir / "physics_config.json"));
    const auto &pobj = bike::io::as_object(physics_doc, "physics_config");
    const auto *drive_val = bike::io::find_member(pobj, "drive");
    bt::require(drive_val != nullptr, "physics_config.json has drive");
    const auto *gearing_val = bike::io::find_member(
        bike::io::as_object(*drive_val, "drive"), "gearing");
    bt::require(gearing_val != nullptr, "physics_config.json has drive.gearing");
    const auto *rear_val = bike::io::find_member(
        bike::io::as_object(*gearing_val, "drive.gearing"), "rear_teeth");
    bt::require(rear_val != nullptr,
                "physics_config.json has drive.gearing.rear_teeth");
    bt::require(bike::io::as_int64(*rear_val, "rear_teeth") == 39,
                "10% start grade sets physics_config.json rear_teeth == 39");

    const auto m =
        bike::model::compile_mjcf(bt::text(out_dir / "model.xml"), out_dir);
    const auto wprm = model_access::readonly_buffer(
        m->wrap_prm, m->nwrap, "wrap_prm");
    const double model_ratio =
        wprm[static_cast<std::size_t>(bundle->freehub_wrap_adr)];
    bt::require(std::abs(model_ratio - 22.0 / 39.0) <= 1e-12,
                "10% start grade model.xml wrap_prm matches 22/39");
    bt::require(std::abs(model_ratio - bundle->rows[0].wrap_prm_freehub) <= 1e-12,
                "10% start grade model.xml wrap_prm matches rows[0].wrap_prm_freehub");

    const auto verification = bike::golden::verify_bundle_closure(out_dir);
    bt::require(verification.ok(),
                "10% start grade bundle achieves 100% 0-bit cold closure");
}

void write_bytes_overwrite(const fs::path &path,
                           std::span<const std::byte> bytes) {
    fs::remove(path);
    auto h = bike::platform::open_exclusive(path);
    bike::platform::write_all(h, bytes);
    h.close();
}

template <typename MutateFn>
std::string mutate_trace_header(std::string text, MutateFn &&fn) {
    std::array<std::byte, sizeof(bike::golden::StepClosureFileHeader)> buf{};
    std::ranges::copy(
        std::as_bytes(std::span<const char>{text}.first(buf.size())),
        buf.begin());
    auto hdr = std::bit_cast<bike::golden::StepClosureFileHeader>(buf);
    std::forward<MutateFn>(fn)(hdr);
    const auto out_chars = std::bit_cast<
        std::array<char, sizeof(bike::golden::StepClosureFileHeader)>>(hdr);
    std::ranges::copy(out_chars, text.begin());
    return text;
}

void verify_negative_bundle_validation() {
    const bt::TemporaryDirectory tmp("step-closure-negative");
    const fs::path valid_dir = tmp.path() / "valid";
    const fs::path physics_toml =
        repo_root() / "examples/research/viewer_physics_welded.toml";
    const bike::golden::StepClosureCaptureSpec spec{
        .physics = bike::configuration::load_physics(physics_toml),
        .track = bike::terrain::flat(40.0),
        .duration_s = std::nullopt,
        .max_steps = 2U,
        .seed_decimal = "0",
        .physics_config_label = physics_toml.string(),
        .track_label = "flat",
        .asset_root = physics_toml.parent_path(),
        .strict = false,
    };
    const auto bundle = bike::golden::capture_step_closure_bundle(spec);
    bike::golden::write_step_closure_bundle(*bundle, valid_dir);

    const std::string valid_trace_text = bt::text(valid_dir / "step_closure.bin");
    const auto copy_case = [&](const std::string &tag) -> fs::path {
        const fs::path d = tmp.path() / tag;
        fs::copy(valid_dir, d, fs::copy_options::recursive);
        return d;
    };

    {
        const fs::path d = copy_case("short_header");
        const auto bytes = std::as_bytes(
            std::span<const char>{valid_trace_text}.first(40U));
        write_bytes_overwrite(d / "step_closure.bin", bytes);
        bt::invalid([&] { (void)bike::golden::load_step_closure_bundle(d); });
    }
    {
        const fs::path d = copy_case("truncated_payload");
        const auto bytes = std::as_bytes(
            std::span<const char>{valid_trace_text}
                .first(valid_trace_text.size() - 8U));
        write_bytes_overwrite(d / "step_closure.bin", bytes);
        bt::invalid([&] { (void)bike::golden::load_step_closure_bundle(d); });
    }
    {
        const fs::path d = copy_case("bad_magic");
        std::string mutated = valid_trace_text;
        mutated[0] = 'X';
        write_bytes_overwrite(
            d / "step_closure.bin",
            std::as_bytes(std::span<const char>{mutated}));
        bt::invalid([&] { (void)bike::golden::load_step_closure_bundle(d); });
    }
    {
        const fs::path d = copy_case("nsteps_overflow");
        const std::string mutated = mutate_trace_header(
            valid_trace_text, [](bike::golden::StepClosureFileHeader &hdr) {
                hdr.n_steps = std::numeric_limits<std::uint64_t>::max();
            });
        write_bytes_overwrite(
            d / "step_closure.bin",
            std::as_bytes(std::span<const char>{mutated}));
        bt::invalid([&] { (void)bike::golden::load_step_closure_bundle(d); });
    }
    {
        const fs::path d = copy_case("oversized");
        fs::resize_file(d / "step_closure.bin",
                        bike::golden::kMaxStepClosureBytes + 8U);
        bt::invalid([&] { (void)bike::golden::load_step_closure_bundle(d); });
    }
    {
        const fs::path d = copy_case("bad_dof_idx");
        const std::string mutated = mutate_trace_header(
            valid_trace_text, [](bike::golden::StepClosureFileHeader &hdr) {
                hdr.front_wheel_dof = static_cast<std::int32_t>(hdr.nv);
            });
        write_bytes_overwrite(
            d / "step_closure.bin",
            std::as_bytes(std::span<const char>{mutated}));
        bt::invalid([&] { (void)bike::golden::load_step_closure_bundle(d); });
    }
    {
        const fs::path d = copy_case("bad_wrap_adr");
        const std::string mutated = mutate_trace_header(
            valid_trace_text, [](bike::golden::StepClosureFileHeader &hdr) {
                hdr.freehub_wrap_adr = 999999;
            });
        write_bytes_overwrite(
            d / "step_closure.bin",
            std::as_bytes(std::span<const char>{mutated}));
        bt::invalid([&] { (void)bike::golden::verify_bundle_closure(d); });
    }
}

} // namespace

int main(int argc, char **argv) {
    return bt::run([&] {
        const auto args = bike::platform::main_arguments(argc, argv);
        const fs::path gpu_export_bin =
            args.size() > 1 && args[1] != nullptr ? fs::path(args[1]) : fs::path{};
        const fs::path track =
            repo_root() / "examples/research/rough_uphill_extreme.toml";
        // 1300 steps (1.625 s) covers standstill braking, release, climb, and
        // the first cadence upshift around step ~1029.
        verify_profile("welded",
                       repo_root() /
                           "examples/research/viewer_physics_welded.toml",
                       track, 1300U);
        verify_profile("distributed",
                       repo_root() /
                           "examples/research/viewer_physics_distributed.toml",
                       track, 1300U);
        verify_nonzero_start_grade();
        verify_negative_bundle_validation();
        verify_cli_surface(gpu_export_bin);
    });
}
