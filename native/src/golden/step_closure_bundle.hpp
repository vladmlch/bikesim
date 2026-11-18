#pragma once

#include "../configuration/model_config.hpp"
#include "../configuration/physical_config.hpp"
#include "../io/value.hpp"
#include "../terrain/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace bike::golden {

inline constexpr std::array<char, 8> kStepClosureMagic{
    {'B', 'G', 'S', 'C', '0', '0', '0', '1'}};
inline constexpr std::uint32_t kStepClosureVersion = 1;
inline constexpr std::size_t kMaxStepClosureBytes =
    std::size_t{1024} * 1024U * 1024U; // 1 GiB

// Maximum step_closure.bin row count that fits within kMaxStepClosureBytes.
[[nodiscard]] std::size_t max_step_closure_steps(
    std::size_t nq, std::size_t nv, std::size_t na,
    std::size_t nu, std::size_t ntendon) noexcept;

// Fixed 80-byte little-endian binary header at offset 0 of step_closure.bin.
// Followed immediately by n_steps * row_doubles IEEE-754 binary64 scalars.
struct StepClosureFileHeader {
    std::array<char, 8> magic = kStepClosureMagic;
    std::uint32_t version = kStepClosureVersion;
    std::uint32_t header_bytes = 80;
    std::uint64_t n_steps = 0;
    std::uint32_t row_doubles = 0;
    std::uint32_t nq = 0;
    std::uint32_t nv = 0;
    std::uint32_t na = 0;
    std::uint32_t nu = 0;
    std::uint32_t ntendon = 0;
    std::int32_t front_wheel_dof = -1;
    std::int32_t rear_wheel_dof = -1;
    std::int32_t freehub_wrap_adr = -1;
    std::int32_t freehub_tendon_id = -1;
    std::int32_t clutch_tendon_id = -1;
    std::uint32_t reserved0 = 0;
    std::uint64_t reserved1 = 0;
};
static_assert(sizeof(StepClosureFileHeader) == 80,
              "StepClosureFileHeader must remain 80 bytes and 8-byte aligned");

// Complete per-step cold-closure snapshot captured from NativeRideRuntime.
struct StepClosureRow {
    std::int64_t step_index = 0;
    double time_s = 0.;
    double dt_s = 0.;

    // Pre-step engine state.
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> act;
    std::vector<double> qacc_warmstart;

    // Applied actuators and generalized forces (total + per-component split).
    std::vector<double> ctrl;
    std::vector<double> qfrc_applied;
    std::vector<double> qfrc_tires;
    std::vector<double> qfrc_suspension;
    std::vector<double> qfrc_resistance;
    std::vector<double> qfrc_rider_joint_envelope;
    std::vector<double> qfrc_brake_hold;
    std::vector<double> qfrc_injected_other;

    // Mid-step mjModel mutations required for 0-bit cold closure.
    std::array<double, 2> dof_frictionloss_wheels{};
    double wrap_prm_freehub = 0.;
    std::vector<double> tendon_range;
    int cog_index = -1;

    // Post-step solver diagnostics and ground-truth next state.
    int ncon = 0;
    int nefc = 0;
    int solver_niter = 0;
    std::vector<double> next_qpos;
    std::vector<double> next_qvel;
    std::vector<double> next_qacc;
};

struct StepClosureBundle {
    std::string model_xml;
    std::string model_12gears_xml;
    std::vector<double> hfield_data;
    io::Value physics_config_json;
    std::string physics_config_label;
    std::string track_label;
    double dt_s = 0.;
    int nq = 0;
    int nv = 0;
    int na = 0;
    int nu = 0;
    int ntendon = 0;
    int front_wheel_dof = -1;
    int rear_wheel_dof = -1;
    int freehub_wrap_adr = -1;
    int freehub_tendon_id = -1;
    int clutch_tendon_id = -1;
    int front_teeth = 0;
    std::vector<int> cassette;
    std::vector<StepClosureRow> rows;
};

struct StepClosureCaptureSpec {
    configuration::SimulationPhysicsConfig physics;
    terrain::TrackSpec track;
    std::optional<double> duration_s;
    std::optional<std::size_t> max_steps;
    std::string seed_decimal = "0";
    std::string physics_config_label;
    std::string track_label;
    std::filesystem::path asset_root;
    std::string rider_variant = "articulated_planar";
    double rider_mass_kg = 80.0;
    double rider_height_m = 1.8;
    bool strict = false;
};

struct ClosureVerificationResult {
    std::size_t total_steps = 0;
    std::size_t zero_bit_qpos_steps = 0;
    std::size_t zero_bit_qvel_steps = 0;
    std::size_t zero_bit_qacc_steps = 0;
    double max_qpos_err = 0.;
    double max_qvel_err = 0.;
    double max_qacc_err = 0.;
    std::size_t steady_gear_checked = 0;
    double steady_gear_max_qpos_diff = 0.;
    double steady_gear_max_qvel_diff = 0.;
    int first_fail_step = -1;

    [[nodiscard]] bool ok() const noexcept {
        return total_steps > 0 && zero_bit_qpos_steps == total_steps &&
               zero_bit_qvel_steps == total_steps && first_fail_step < 0;
    }
};

// Expand the single ideal_mid_drive_freehub tendon into N static cassette
// tendons freehub_cog_00..11 with inactive range [-1e12, +1e12] (ntendon=13).
[[nodiscard]] std::string expand_model_12gears_xml(
    std::string_view model_xml,
    const configuration::SimulationPhysicsConfig &physics);

// Run NativeRideRuntime deterministically and capture a cold-closure bundle.
[[nodiscard]] std::unique_ptr<StepClosureBundle> capture_step_closure_bundle(
    const StepClosureCaptureSpec &spec);

// Write manifest.json, model.xml, model_12gears.xml, hfield_data.bin,
// physics_config.json, and step_closure.bin into out_dir.
void write_step_closure_bundle(const StepClosureBundle &bundle,
                               const std::filesystem::path &out_dir);

// Load an exported bundle directory back into memory.
[[nodiscard]] std::unique_ptr<StepClosureBundle> load_step_closure_bundle(
    const std::filesystem::path &bundle_dir);

// Verify 0-bit cold 1-step FP64 closure from disk + steady-gear 12-gear parity.
[[nodiscard]] ClosureVerificationResult verify_bundle_closure(
    const std::filesystem::path &bundle_dir);

} // namespace bike::golden
