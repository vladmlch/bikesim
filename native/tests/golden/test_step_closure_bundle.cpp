#include "cli/gpu_export.hpp"
#include "configuration/load.hpp"
#include "golden/step_closure_bundle.hpp"
#include "model/builder.hpp"
#include "platform/argv.hpp"
#include "platform/process.hpp"
#include "support/fixture_test.hpp"
#include "support/temporary_directory.hpp"
#include "terrain/trackfile.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <set>
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

    const auto m12 = bike::model::compile_mjcf(bundle->model_12gears_xml);
    bt::require(m12->ntendon == 13,
                name + ": model_12gears.xml has ntendon == 13");

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
        verify_cli_surface(gpu_export_bin);
    });
}
