// native/tests/gpu/test_gpu_tire.cpp — Standalone C++23 unit test suite for
// native/src/gpu/tire_kernel.hpp.
#include "../../src/gpu/tire_kernel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace {

using namespace bike::gpu;

void check(bool cond, const char *msg, int line) {
    if (!cond) {
        std::fprintf(stderr, "FAIL [line %d]: %s\n", line, msg);
        std::exit(1);
    }
}

#define CHECK(cond) check((cond), #cond, __LINE__)

// Exact double-precision reference from native/src/tyre/station.cpp:52-73.
std::pair<double, double> ref_station_density(double delta, const GpuHingeDensity &d) {
    double force = 0.0;
    double energy = 0.0;
    for (int i = 0; i < d.num_knots; ++i) {
        const double x = std::max(delta - static_cast<double>(d.knots_m[i]), 0.0);
        const double k = static_cast<double>(d.stiffness_n_m[i]);
        force += k * x;
        energy += k * x * x;
    }
    return {force, 0.5 * energy};
}

// Exact double-precision reference from native/src/tyre/station.cpp:78-95.
std::pair<double, double> ref_station_response(
    double delta,
    double rate,
    double weight,
    const GpuHingeDensity &d
) {
    const auto [elastic, energy] = ref_station_density(delta, d);
    const double force = delta > 0.0
        ? std::max(0.0, elastic + static_cast<double>(d.damping_ns_m) * rate)
        : 0.0;
    return {weight * force, weight * energy};
}

double rel_err(double got, double expected) {
    const double denom = std::max(std::abs(expected), 1e-12);
    return std::abs(got - expected) / denom;
}

void test_station_density_and_response() {
    GpuHingeDensity d{};
    d.num_knots = 3;
    d.knots_m[0] = 0.0f;
    d.knots_m[1] = 0.008f;
    d.knots_m[2] = 0.018f;
    d.stiffness_n_m[0] = 55000.0f;
    d.stiffness_n_m[1] = 85000.0f;
    d.stiffness_n_m[2] = 140000.0f;
    d.damping_ns_m = 420.0f;

    // 1. Negative and zero penetrations
    for (float delta : {-0.025f, -0.001f, 0.0f}) {
        const GpuVec2 dens = gpu_station_density(delta, d);
        CHECK(dens.x == 0.0f);
        CHECK(dens.y == 0.0f);

        const GpuVec2 resp = gpu_station_response(delta, 1.5f, 0.19634954f, d);
        CHECK(resp.x == 0.0f);
        CHECK(resp.y == 0.0f);
    }

    // 2. Multi-knot positive penetrations and varying rates
    constexpr std::array<float, 5> kDeltas = {0.002f, 0.008f, 0.012f, 0.018f, 0.032f};
    constexpr std::array<float, 4> kRates = {-5.0f, -0.2f, 0.0f, 1.25f};
    const float weight = kTwoPi / 32.0f;

    for (float delta : kDeltas) {
        const GpuVec2 gpu_d = gpu_station_density(delta, d);
        const auto [ref_f, ref_e] = ref_station_density(static_cast<double>(delta), d);
        CHECK(rel_err(static_cast<double>(gpu_d.x), ref_f) < 1e-6);
        CHECK(rel_err(static_cast<double>(gpu_d.y), ref_e) < 1e-6);

        for (float rate : kRates) {
            const GpuVec2 gpu_r = gpu_station_response(delta, rate, weight, d);
            const auto [ref_rf, ref_re] = ref_station_response(
                static_cast<double>(delta),
                static_cast<double>(rate),
                static_cast<double>(weight),
                d
            );
            if (ref_rf == 0.0) {
                CHECK(std::abs(gpu_r.x) < 1e-6f);
            } else {
                CHECK(rel_err(static_cast<double>(gpu_r.x), ref_rf) < 1e-6);
            }
            CHECK(rel_err(static_cast<double>(gpu_r.y), ref_re) < 1e-6);
        }
    }
}

void test_heightfield_1d() {
    // 1. Flat terrain: nx = 0, nz = 1
    constexpr std::array<float, 5> kFlatHeights = {0.25f, 0.25f, 0.25f, 0.25f, 0.25f};
    const GpuHeightfield1D flat_hf{
        .heights_m = kFlatHeights.data(),
        .num_samples = static_cast<int>(kFlatHeights.size()),
        .x_min_m = -2.0f,
        .dx_m = 1.0f,
        .inv_dx_m = 1.0f
    };

    for (float x : {-3.0f, -1.5f, 0.0f, 1.25f, 5.0f}) {
        const GpuVec3 s = gpu_sample_heightfield_1d(flat_hf, x);
        CHECK(std::abs(s.x - 0.25f) < 1e-6f);
        CHECK(std::abs(s.y - 0.0f) < 1e-6f);
        CHECK(std::abs(s.z - 1.0f) < 1e-6f);
        CHECK(std::abs((s.y * s.y + s.z * s.z) - 1.0f) < 1e-6f);
    }

    // 2. Sloped terrain with dh/dx = 0.25
    constexpr std::array<float, 5> kSlopeHeights = {0.0f, 0.25f, 0.50f, 0.75f, 1.00f};
    const GpuHeightfield1D slope_hf{
        .heights_m = kSlopeHeights.data(),
        .num_samples = static_cast<int>(kSlopeHeights.size()),
        .x_min_m = 0.0f,
        .dx_m = 1.0f,
        .inv_dx_m = 1.0f
    };

    const GpuVec3 s_mid = gpu_sample_heightfield_1d(slope_hf, 1.5f);
    const float expected_nx = -0.25f / std::sqrt(1.0f + 0.25f * 0.25f);
    const float expected_nz = 1.0f / std::sqrt(1.0f + 0.25f * 0.25f);
    CHECK(std::abs(s_mid.x - 0.375f) < 1e-6f);
    CHECK(std::abs(s_mid.y - expected_nx) < 1e-6f);
    CHECK(std::abs(s_mid.z - expected_nz) < 1e-6f);
    CHECK(std::abs((s_mid.y * s_mid.y + s_mid.z * s_mid.z) - 1.0f) < 1e-6f);
}

void test_evaluate_wheel_contact_and_qfrc() {
    constexpr std::array<float, 5> kFlatGround = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const GpuHeightfield1D hf{
        .heights_m = kFlatGround.data(),
        .num_samples = static_cast<int>(kFlatGround.size()),
        .x_min_m = -10.0f,
        .dx_m = 5.0f,
        .inv_dx_m = 0.2f
    };

    GpuTireParams params{};
    params.nominal_radius_m = 0.351f;
    params.section_radius_m = 0.030f;
    params.num_stations = 32;
    params.num_lateral_ribs = 1;

    // 1. Wheel above ground -> zero load
    {
        GpuWheelStationState state{};
        const GpuWheelContactOutput out = gpu_evaluate_wheel_contact(
            0.0f, 0.50f, 2.0f, 0.0f, 5.0f, 0.001f, params, hf, &state);
        CHECK(out.active_stations == 0);
        CHECK(out.normal_load_n == 0.0f);
        CHECK(out.fx_world_n == 0.0f);
        CHECK(out.fz_world_n == 0.0f);
        CHECK(out.spin_torque_nm == 0.0f);
        CHECK(out.effective_radius_m == 0.0f);
    }

    // 2. Wheel penetrating ground -> positive normal load and effective_radius_m < nominal_radius_m
    {
        GpuWheelStationState state{};
        const float wheel_z = 0.330f; // 21 mm bottom penetration
        const float vx = 4.0f;
        const float omega_free = vx / params.nominal_radius_m;
        const GpuWheelContactOutput out = gpu_evaluate_wheel_contact(
            0.0f, wheel_z, vx, 0.0f, omega_free, 0.001f, params, hf, &state);
        CHECK(out.active_stations > 1);
        CHECK(out.normal_load_n > 0.0f);
        CHECK(out.fz_world_n > 0.0f);
        CHECK(out.effective_radius_m > 0.0f);
        CHECK(out.effective_radius_m < params.nominal_radius_m);
    }

    // 3. Driving vs braking spin torque when wheel_omega * r != vx
    {
        GpuWheelStationState state_drive{};
        const float wheel_z = 0.330f;
        const float vx = 3.0f;
        const float omega_drive = (vx + 2.0f) / params.nominal_radius_m; // omega * r > vx
        const GpuWheelContactOutput out_drive = gpu_evaluate_wheel_contact(
            0.0f, wheel_z, vx, 0.0f, omega_drive, 0.001f, params, hf, &state_drive);
        CHECK(out_drive.fx_world_n > 0.0f);
        CHECK(out_drive.spin_torque_nm < 0.0f);

        GpuWheelStationState state_brake{};
        const float omega_brake = (vx - 2.0f) / params.nominal_radius_m; // omega * r < vx
        const GpuWheelContactOutput out_brake = gpu_evaluate_wheel_contact(
            0.0f, wheel_z, vx, 0.0f, omega_brake, 0.001f, params, hf, &state_brake);
        CHECK(out_brake.fx_world_n < 0.0f);
        CHECK(out_brake.spin_torque_nm > 0.0f);
    }

    // 4. Multi-rib 3D ray scaling: num_stations = 60, num_lateral_ribs = 15 (900 rays)
    //    and num_stations = 110, num_lateral_ribs = 70 (7700 rays)
    for (const auto [nst, nrib] : std::array<std::pair<int, int>, 2>{{{60, 15}, {110, 70}}}) {
        GpuTireParams params_3d = params;
        params_3d.num_stations = nst;
        params_3d.num_lateral_ribs = nrib;
        GpuWheelStationState state_3d{};
        const GpuWheelContactOutput out_3d = gpu_evaluate_wheel_contact(
            0.0f, 0.325f, 3.5f, -0.1f, 12.0f, 0.001f, params_3d, hf, &state_3d);
        CHECK(out_3d.active_stations > 0);
        CHECK(gpu_isfinite(out_3d.fx_world_n));
        CHECK(gpu_isfinite(out_3d.fz_world_n));
        CHECK(gpu_isfinite(out_3d.spin_torque_nm));
        CHECK(gpu_isfinite(out_3d.normal_load_n));
        CHECK(gpu_isfinite(out_3d.effective_radius_m));
        CHECK(out_3d.normal_load_n > 0.0f);
        CHECK(out_3d.effective_radius_m > 0.0f);
        CHECK(out_3d.effective_radius_m < params_3d.nominal_radius_m);
    }

    // 5. Both-wheels qfrc projection
    {
        GpuModelIndices idx{};
        GpuWheelStationState front_state{};
        GpuWheelStationState rear_state{};
        std::array<float, kMaxNv> qfrc{};
        GpuWheelContactOutput front_out{};
        GpuWheelContactOutput rear_out{};

        gpu_write_both_wheels_qfrc(
            0.0f, 0.55f,
            0.60f, 0.330f, 3.0f, 0.0f, 12.0f,
            -0.55f, 0.330f, 3.0f, 0.0f, 12.0f,
            0.001f,
            params, params,
            hf, idx,
            &front_state, &rear_state,
            qfrc.data(),
            &front_out, &rear_out
        );

        CHECK(qfrc[idx.root_x_dof] > 0.0f);
        CHECK(qfrc[idx.root_z_dof] > 0.0f);
        CHECK(gpu_isfinite(qfrc[idx.root_pitch_dof]));
        CHECK(std::abs(qfrc[idx.front_wheel_spin_dof] - front_out.spin_torque_nm) < 1e-6f);
        CHECK(std::abs(qfrc[idx.rear_wheel_spin_dof] - rear_out.spin_torque_nm) < 1e-6f);
    }
}

} // namespace

int main() {
    test_station_density_and_response();
    test_heightfield_1d();
    test_evaluate_wheel_contact_and_qfrc();
    std::printf("All GPU tire kernel tests passed.\n");
    return 0;
}
