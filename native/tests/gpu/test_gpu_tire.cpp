// native/tests/gpu/test_gpu_tire.cpp — Standalone C++23 unit test suite for
// native/src/gpu/tire_kernel.hpp.
#include "../../src/gpu/tire_kernel.hpp"
// The station.cpp reference is linked from bike_core (the one compile of
// that TU): the ODR sweep links every selected TU into one image, so a
// textual #include of the .cpp here would define its symbols twice.
#include "../../src/tyre/station.hpp"
#include "../../src/platform/argv.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

using namespace bike::gpu;

using TestFunction = void (*)();

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition)
        throw std::runtime_error(std::string(message));
}

// Numeric helper used instead of printf-style messages. Templated on the
// arithmetic type so float comparisons keep their original float semantics
// instead of silently promoting to double.
template <class T>
void require_near(T actual, T expected, T tol, std::string_view what) {
    if (!(std::abs(actual - expected) <= tol))
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " tol=" + std::to_string(tol));
}

// GpuModelIndices addresses are int for the kernel ABI (a disabled slot is
// -1); container subscripts take size_t, so the explicit conversion lives in
// one place.
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

// Exact double-precision reference from native/src/tyre/station.cpp:52-73.
std::pair<double, double> ref_station_density(double delta, const GpuHingeDensity &d) {
    const std::span<const float> knots{d.knots_m};
    const std::span<const float> stiffness{d.stiffness_n_m};
    double force = 0.0;
    double energy = 0.0;
    for (int i = 0; i < d.num_knots; ++i) {
        const double x = std::max(delta - static_cast<double>(knots[to_index(i)]), 0.0);
        const double k = static_cast<double>(stiffness[to_index(i)]);
        force += k * x;
        energy += k * x * x;
    }
    return {force, 0.5 * energy};
}

// (delta, rate, weight) are all doubles: as positional parameters they would
// be trivially swappable at the call site, so the query is packed instead.
struct RefStationQuery {
    double delta;
    double rate;
    double weight;
};

// Exact double-precision reference from native/src/tyre/station.cpp:78-95.
std::pair<double, double> ref_station_response(
    const RefStationQuery &q,
    const GpuHingeDensity &d
) {
    const auto [elastic, energy] = ref_station_density(q.delta, d);
    const double force = q.delta > 0.0
        ? std::max(0.0, elastic + static_cast<double>(d.damping_ns_m) * q.rate)
        : 0.0;
    return {q.weight * force, q.weight * energy};
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

    // Equivalent nativecfg::HingeDensity for authoritative native biketyre::station_response
    nativecfg::HingeDensity native_d{};
    const std::span<const float> gpu_knots{d.knots_m};
    const std::span<const float> gpu_stiffness{d.stiffness_n_m};
    const std::size_t num_knots = to_index(d.num_knots);
    native_d.knots_m.assign(
        gpu_knots.first(num_knots).begin(), gpu_knots.first(num_knots).end());
    native_d.stiffness_n_m.assign(
        gpu_stiffness.first(num_knots).begin(), gpu_stiffness.first(num_knots).end());
    native_d.damping_ns_m = static_cast<double>(d.damping_ns_m);

    // 1. Negative and zero penetrations
    for (const float delta : {-0.025f, -0.001f, 0.0f}) {
        const GpuVec2 dens = gpu_station_density(delta, d);
        require(dens.x == 0.0f, "station density force is zero at non-positive penetration");
        require(dens.y == 0.0f, "station density energy is zero at non-positive penetration");

        const GpuVec2 resp = gpu_station_response(delta, 1.5f, 0.19634954f, d);
        require(resp.x == 0.0f, "station response force is zero at non-positive penetration");
        require(resp.y == 0.0f, "station response energy is zero at non-positive penetration");

        const GpuVec2 resp_alias = gpu_tire_station_response(delta, 1.5f, 0.19634954f, d);
        require(resp_alias.x == 0.0f, "tire alias response force is zero at non-positive penetration");
        require(resp_alias.y == 0.0f, "tire alias response energy is zero at non-positive penetration");

        const auto [authoritative_rf, authoritative_re] = biketyre::station_response(
            static_cast<double>(delta), 1.5, 0.19634954, native_d);
        require(authoritative_rf == 0.0,
                "authoritative station response force is zero at non-positive penetration");
        require(authoritative_re == 0.0,
                "authoritative station response energy is zero at non-positive penetration");
    }

    // 2. Multi-knot positive penetrations and varying rates
    constexpr std::array<float, 5> kDeltas = {0.002f, 0.008f, 0.012f, 0.018f, 0.032f};
    constexpr std::array<float, 4> kRates = {-5.0f, -0.2f, 0.0f, 1.25f};
    const float weight = kTwoPi / 32.0f;

    for (const float delta : kDeltas) {
        const GpuVec2 gpu_d = gpu_station_density(delta, d);
        const auto [ref_f, ref_e] = ref_station_density(static_cast<double>(delta), d);
        require_near(rel_err(static_cast<double>(gpu_d.x), ref_f), 0.0, 1e-6,
                     "station density force tracks the double reference");
        require_near(rel_err(static_cast<double>(gpu_d.y), ref_e), 0.0, 1e-6,
                     "station density energy tracks the double reference");

        for (const float rate : kRates) {
            const GpuVec2 gpu_r = gpu_station_response(delta, rate, weight, d);
            const GpuVec2 gpu_alias = gpu_tire_station_response(delta, rate, weight, d);
            require(gpu_alias.x == gpu_r.x, "tire alias response force matches station response");
            require(gpu_alias.y == gpu_r.y, "tire alias response energy matches station response");

            const auto [ref_rf, ref_re] = ref_station_response(
                {.delta = static_cast<double>(delta),
                 .rate = static_cast<double>(rate),
                 .weight = static_cast<double>(weight)},
                d
            );
            if (ref_rf == 0.0) {
                require_near(gpu_r.x, 0.0f, 1e-6f,
                             "station response force is zero when the reference clamps");
            } else {
                require_near(rel_err(static_cast<double>(gpu_r.x), ref_rf), 0.0, 1e-6,
                             "station response force tracks the double reference");
            }
            require_near(rel_err(static_cast<double>(gpu_r.y), ref_re), 0.0, 1e-6,
                         "station response energy tracks the double reference");

            // Authoritative comparison against biketyre::station_response (native/src/tyre/station.cpp)
            const auto [authoritative_rf, authoritative_re] = biketyre::station_response(
                static_cast<double>(delta),
                static_cast<double>(rate),
                static_cast<double>(weight),
                native_d
            );
            if (authoritative_rf == 0.0) {
                require_near(gpu_r.x, 0.0f, 1e-6f,
                             "station response force is zero when the authoritative reference clamps");
            } else {
                require_near(rel_err(static_cast<double>(gpu_r.x), authoritative_rf), 0.0, 1e-6,
                             "station response force tracks the authoritative reference");
            }
            require_near(rel_err(static_cast<double>(gpu_r.y), authoritative_re), 0.0, 1e-6,
                         "station response energy tracks the authoritative reference");
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

    for (const float x : {-3.0f, -1.5f, 0.0f, 1.25f, 5.0f}) {
        const GpuVec3 s = gpu_sample_heightfield_1d(flat_hf, x);
        require_near(s.x, 0.25f, 1e-6f, "flat heightfield height is the constant elevation");
        require_near(s.y, 0.0f, 1e-6f, "flat heightfield normal x is zero");
        require_near(s.z, 1.0f, 1e-6f, "flat heightfield normal z is one");
        require_near(s.y * s.y + s.z * s.z, 1.0f, 1e-6f,
                     "flat heightfield normal is unit length");
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
    require_near(s_mid.x, 0.375f, 1e-6f, "sloped heightfield interpolates the midpoint height");
    require_near(s_mid.y, expected_nx, 1e-6f, "sloped heightfield normal x is -dh/dx scaled");
    require_near(s_mid.z, expected_nz, 1e-6f, "sloped heightfield normal z is unit scaled");
    require_near(s_mid.y * s_mid.y + s_mid.z * s_mid.z, 1.0f, 1e-6f,
                 "sloped heightfield normal is unit length");
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
        require(out.active_stations == 0, "lifted wheel has no active stations");
        require(out.normal_load_n == 0.0f, "lifted wheel carries no normal load");
        require(out.fx_world_n == 0.0f, "lifted wheel produces no longitudinal force");
        require(out.fz_world_n == 0.0f, "lifted wheel produces no vertical force");
        require(out.spin_torque_nm == 0.0f, "lifted wheel produces no spin torque");
        require(out.effective_radius_m == 0.0f, "lifted wheel reports zero effective radius");
    }

    // 2. Wheel penetrating ground -> positive normal load and effective_radius_m < nominal_radius_m
    {
        GpuWheelStationState state{};
        const float wheel_z = 0.330f; // 21 mm bottom penetration
        const float vx = 4.0f;
        const float omega_free = vx / params.nominal_radius_m;
        const GpuWheelContactOutput out = gpu_evaluate_wheel_contact(
            0.0f, wheel_z, vx, 0.0f, omega_free, 0.001f, params, hf, &state);
        require(out.active_stations > 1, "penetrating wheel activates multiple stations");
        require(out.normal_load_n > 0.0f, "penetrating wheel carries positive normal load");
        require(out.fz_world_n > 0.0f, "penetrating wheel produces positive vertical force");
        require(out.effective_radius_m > 0.0f, "penetrating wheel reports a positive effective radius");
        require(out.effective_radius_m < params.nominal_radius_m,
                "penetrating wheel effective radius drops below nominal");
    }

    // 3. Driving vs braking spin torque when wheel_omega * r != vx
    {
        GpuWheelStationState state_drive{};
        const float wheel_z = 0.330f;
        const float vx = 3.0f;
        const float omega_drive = (vx + 2.0f) / params.nominal_radius_m; // omega * r > vx
        const GpuWheelContactOutput out_drive = gpu_evaluate_wheel_contact(
            0.0f, wheel_z, vx, 0.0f, omega_drive, 0.001f, params, hf, &state_drive);
        require(out_drive.fx_world_n > 0.0f, "overdriven wheel pushes forward");
        require(out_drive.spin_torque_nm < 0.0f, "overdriven wheel drags on the spin DOF");

        GpuWheelStationState state_brake{};
        const float omega_brake = (vx - 2.0f) / params.nominal_radius_m; // omega * r < vx
        const GpuWheelContactOutput out_brake = gpu_evaluate_wheel_contact(
            0.0f, wheel_z, vx, 0.0f, omega_brake, 0.001f, params, hf, &state_brake);
        require(out_brake.fx_world_n < 0.0f, "underdriven wheel drags backward");
        require(out_brake.spin_torque_nm > 0.0f, "underdriven wheel pulls on the spin DOF");
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
        require(out_3d.active_stations > 0, "multi-rib wheel activates stations");
        require(gpu_isfinite(out_3d.fx_world_n), "multi-rib longitudinal force is finite");
        require(gpu_isfinite(out_3d.fz_world_n), "multi-rib vertical force is finite");
        require(gpu_isfinite(out_3d.spin_torque_nm), "multi-rib spin torque is finite");
        require(gpu_isfinite(out_3d.normal_load_n), "multi-rib normal load is finite");
        require(gpu_isfinite(out_3d.effective_radius_m), "multi-rib effective radius is finite");
        require(out_3d.normal_load_n > 0.0f, "multi-rib wheel carries positive normal load");
        require(out_3d.effective_radius_m > 0.0f, "multi-rib wheel reports a positive effective radius");
        require(out_3d.effective_radius_m < params_3d.nominal_radius_m,
                "multi-rib effective radius drops below nominal");
    }

    // 5. Both-wheels qfrc projection
    {
        const GpuModelIndices idx{};
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

        require(qfrc[to_index(idx.root_x_dof)] > 0.0f, "both wheels drive root_x forward");
        require(qfrc[to_index(idx.root_z_dof)] > 0.0f, "both wheels push root_z upward");
        require(gpu_isfinite(qfrc[to_index(idx.root_pitch_dof)]),
                "root pitch torque is finite");
        require_near(qfrc[to_index(idx.front_wheel_spin_dof)], front_out.spin_torque_nm, 1e-6f,
                     "front spin DOF receives the front spin torque");
        require_near(qfrc[to_index(idx.rear_wheel_spin_dof)], rear_out.spin_torque_nm, 1e-6f,
                     "rear spin DOF receives the rear spin torque");
    }
}

void test_gpu_write_both_wheels_suspension_coupling() {
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

    const GpuModelIndices idx{};
    require(idx.fork_travel_dof >= 0, "fork travel DOF is enabled");
    require(idx.main_pivot_dof >= 0, "main pivot DOF is enabled");
    require(idx.horst_pivot_dof >= 0, "horst pivot DOF is enabled");

    // 1. Front wheel in contact: check fork_travel_dof projection (e_fork.x * fx + e_fork.y * fz)
    {
        std::array<float, static_cast<std::size_t>(kMaxNbody) * 3> xpos{};
        const float com_x = 0.0f;
        const float com_z = 0.55f;
        xpos[to_index(3 * idx.frame_body + 0)] = com_x;
        xpos[to_index(3 * idx.frame_body + 2)] = com_z;

        // Steer and fork lower define fork axis
        xpos[to_index(3 * idx.steer_body + 0)] = 0.50f;
        xpos[to_index(3 * idx.steer_body + 2)] = 0.80f;
        xpos[to_index(3 * idx.fork_lower_body + 0)] = 0.60f;
        xpos[to_index(3 * idx.fork_lower_body + 2)] = 0.50f;

        // Front wheel embedded 21 mm into terrain
        xpos[to_index(3 * idx.front_wheel_body + 0)] = 0.60f;
        xpos[to_index(3 * idx.front_wheel_body + 2)] = 0.330f;

        // Rear wheel lifted safely above ground
        xpos[to_index(3 * idx.rear_wheel_body + 0)] = -0.55f;
        xpos[to_index(3 * idx.rear_wheel_body + 2)] = 0.60f;

        std::array<float, kMaxNv> qvel{};
        qvel[to_index(idx.root_x_dof)] = 2.0f;
        qvel[to_index(idx.front_wheel_spin_dof)] = 2.0f / params.nominal_radius_m;

        std::array<float, kMaxNv> qfrc{};
        GpuWheelStationState front_state{};
        GpuWheelStationState rear_state{};
        GpuWheelContactOutput front_out{};
        GpuWheelContactOutput rear_out{};

        gpu_write_both_wheels_qfrc(
            xpos.data(),
            qvel.data(),
            0.001f,
            params, params,
            hf, idx,
            &front_state, &rear_state,
            qfrc.data(),
            &front_out, &rear_out
        );

        require(front_out.active_stations > 0, "front wheel activates stations");
        require(front_out.normal_load_n > 0.0f, "front wheel carries positive normal load");
        require(front_out.fz_world_n > 0.0f, "front wheel produces positive vertical force");
        require(rear_out.active_stations == 0, "lifted rear wheel has no active stations");

        const GpuVec2 e_fork = gpu_fork_axis_world(idx, xpos.data(), 0.0f);
        const float expected_fork_f = e_fork.x * front_out.fx_world_n + e_fork.y * front_out.fz_world_n;
        require(expected_fork_f != 0.0f, "fork projection of the contact force is non-zero");
        require_near(qfrc[to_index(idx.fork_travel_dof)], expected_fork_f, 1e-5f,
                     "fork travel DOF receives the projected fork-axis force");
    }

    // 2. Rear wheel in contact: check main_pivot_dof and horst_pivot_dof torques
    {
        std::array<float, static_cast<std::size_t>(kMaxNbody) * 3> xpos{};
        const float com_x = 0.0f;
        const float com_z = 0.55f;
        xpos[to_index(3 * idx.frame_body + 0)] = com_x;
        xpos[to_index(3 * idx.frame_body + 2)] = com_z;

        // Front wheel lifted
        xpos[to_index(3 * idx.front_wheel_body + 0)] = 0.60f;
        xpos[to_index(3 * idx.front_wheel_body + 2)] = 0.60f;

        // Rear pivot anchors
        xpos[to_index(3 * idx.chainstay_body + 0)] = -0.05f;
        xpos[to_index(3 * idx.chainstay_body + 2)] = 0.58f;
        xpos[to_index(3 * idx.seatstay_body + 0)] = -0.38f;
        xpos[to_index(3 * idx.seatstay_body + 2)] = 0.57f;

        // Rear wheel in contact (21 mm penetration)
        xpos[to_index(3 * idx.rear_wheel_body + 0)] = -0.55f;
        xpos[to_index(3 * idx.rear_wheel_body + 2)] = 0.330f;

        std::array<float, kMaxNv> qvel{};
        qvel[to_index(idx.root_x_dof)] = 2.0f;
        qvel[to_index(idx.rear_wheel_spin_dof)] = 2.0f / params.nominal_radius_m;

        std::array<float, kMaxNv> qfrc{};
        GpuWheelStationState front_state{};
        GpuWheelStationState rear_state{};
        GpuWheelContactOutput front_out{};
        GpuWheelContactOutput rear_out{};

        gpu_write_both_wheels_qfrc(
            xpos.data(),
            qvel.data(),
            0.001f,
            params, params,
            hf, idx,
            &front_state, &rear_state,
            qfrc.data(),
            &front_out, &rear_out
        );

        require(front_out.active_stations == 0, "lifted front wheel has no active stations");
        require(rear_out.active_stations > 0, "rear wheel activates stations");
        require(rear_out.normal_load_n > 0.0f, "rear wheel carries positive normal load");
        require(rear_out.fz_world_n > 0.0f, "rear wheel produces positive vertical force");

        const GpuVec2 a_main = gpu_rear_pivot_anchor_world(
            idx.chainstay_body, xpos.data(), com_x, com_z, kMainPivotLocalDxM, kMainPivotLocalDzM);
        const GpuVec2 a_horst = gpu_horst_pivot_anchor_world(
            idx.seatstay_body, xpos.data(), com_x, com_z, 0.0f, 0.0f);

        const float expected_main_torque =
            (rear_out.contact_z_m - a_main.y) * rear_out.fx_world_n -
            (rear_out.contact_x_m - a_main.x) * rear_out.fz_world_n;
        const float expected_horst_torque =
            (rear_out.contact_z_m - a_horst.y) * rear_out.fx_world_n -
            (rear_out.contact_x_m - a_horst.x) * rear_out.fz_world_n;

        require(expected_main_torque != 0.0f, "main pivot torque arm is non-zero");
        require(expected_horst_torque != 0.0f, "horst pivot torque arm is non-zero");
        require_near(qfrc[to_index(idx.main_pivot_dof)], expected_main_torque, 1e-5f,
                     "main pivot DOF receives the contact torque about its anchor");
        require_near(qfrc[to_index(idx.horst_pivot_dof)], expected_horst_torque, 1e-5f,
                     "horst pivot DOF receives the contact torque about its anchor");
    }

    // 3. Normal damping load sensitivity to suspension joint velocity
    {
        std::array<float, static_cast<std::size_t>(kMaxNbody) * 3> xpos{};
        const float com_x = 0.0f;
        const float com_z = 0.55f;
        xpos[to_index(3 * idx.frame_body + 0)] = com_x;
        xpos[to_index(3 * idx.frame_body + 2)] = com_z;

        xpos[to_index(3 * idx.steer_body + 0)] = 0.50f;
        xpos[to_index(3 * idx.steer_body + 2)] = 0.80f;
        xpos[to_index(3 * idx.fork_lower_body + 0)] = 0.60f;
        xpos[to_index(3 * idx.fork_lower_body + 2)] = 0.50f;
        xpos[to_index(3 * idx.front_wheel_body + 0)] = 0.60f;
        xpos[to_index(3 * idx.front_wheel_body + 2)] = 0.330f;

        xpos[to_index(3 * idx.chainstay_body + 0)] = -0.05f;
        xpos[to_index(3 * idx.chainstay_body + 2)] = 0.58f;
        xpos[to_index(3 * idx.seatstay_body + 0)] = -0.38f;
        xpos[to_index(3 * idx.seatstay_body + 2)] = 0.57f;
        xpos[to_index(3 * idx.rear_wheel_body + 0)] = -0.55f;
        xpos[to_index(3 * idx.rear_wheel_body + 2)] = 0.330f;

        // Front fork velocity coupling:
        std::array<float, kMaxNv> qvel_v0{};
        std::array<float, kMaxNv> qvel_vfork{};
        qvel_vfork[to_index(idx.fork_travel_dof)] = 1.0f;

        std::array<float, kMaxNv> qfrc_v0{};
        std::array<float, kMaxNv> qfrc_vfork{};
        GpuWheelStationState st_f0{}, st_r0{}, st_fv{}, st_rv{};
        GpuWheelContactOutput out_f0{}, out_r0{}, out_fv{}, out_rv{};

        gpu_write_both_wheels_qfrc(
            xpos.data(), qvel_v0.data(), 0.001f, params, params, hf, idx,
            &st_f0, &st_r0, qfrc_v0.data(), &out_f0, &out_r0);
        gpu_write_both_wheels_qfrc(
            xpos.data(), qvel_vfork.data(), 0.001f, params, params, hf, idx,
            &st_fv, &st_rv, qfrc_vfork.data(), &out_fv, &out_rv);

        // Positive fork slide velocity compresses fork lower, moving front wheel downwards,
        // increasing penetration rate and damping normal load
        require(std::abs(out_fv.normal_load_n - out_f0.normal_load_n) > 5.0f,
                "fork slide velocity changes the front normal load through damping");

        // Rear suspension main pivot velocity coupling:
        std::array<float, kMaxNv> qvel_vmain{};
        qvel_vmain[to_index(idx.main_pivot_dof)] = 2.0f;
        std::array<float, kMaxNv> qfrc_vmain{};
        GpuWheelStationState st_f_main{}, st_r_main{};
        GpuWheelContactOutput out_f_main{}, out_r_main{};

        gpu_write_both_wheels_qfrc(
            xpos.data(), qvel_vmain.data(), 0.001f, params, params, hf, idx,
            &st_f_main, &st_r_main, qfrc_vmain.data(), &out_f_main, &out_r_main);

        require(std::abs(out_r_main.normal_load_n - out_r0.normal_load_n) > 5.0f,
                "main pivot velocity changes the rear normal load through damping");

        // Rear suspension horst pivot velocity coupling:
        std::array<float, kMaxNv> qvel_vhorst{};
        qvel_vhorst[to_index(idx.horst_pivot_dof)] = 2.0f;
        std::array<float, kMaxNv> qfrc_vhorst{};
        GpuWheelStationState st_f_horst{}, st_r_horst{};
        GpuWheelContactOutput out_f_horst{}, out_r_horst{};

        gpu_write_both_wheels_qfrc(
            xpos.data(), qvel_vhorst.data(), 0.001f, params, params, hf, idx,
            &st_f_horst, &st_r_horst, qfrc_vhorst.data(), &out_f_horst, &out_r_horst);

        require(std::abs(out_r_horst.normal_load_n - out_r0.normal_load_n) > 5.0f,
                "horst pivot velocity changes the rear normal load through damping");
    }
}

void test_fork_axis_world_top_out_and_pitch() {
    const GpuModelIndices idx{};

    // Blocker 3 regression:
    // In model.xml at zero pitch, steer_body is at (+0.532605, +0.538145) relative to frame_body,
    // and fork_travel slide axis is (-0.43837105, +0.89879410).
    // When fork_travel <= 0 (top-out), xpos[fork_lower] - xpos[steer] vanishes or reverses sign.
    // Verify gpu_fork_axis_world NEVER flips sign across compressed, zero, and top-out states.
    std::array<float, static_cast<std::size_t>(kMaxNbody) * 3> xpos{};
    xpos[to_index(3 * idx.frame_body + 0)] = 2.0f;
    xpos[to_index(3 * idx.frame_body + 2)] = 0.56f;
    xpos[to_index(3 * idx.steer_body + 0)] = 2.0f + 0.532605f;
    xpos[to_index(3 * idx.steer_body + 2)] = 0.56f + 0.538145f;

    for (const float travel_m : {0.060f, 0.0f, -0.006f, -0.025f}) {
        xpos[to_index(3 * idx.fork_lower_body + 0)] =
            xpos[to_index(3 * idx.steer_body + 0)] - 0.43837105f * travel_m;
        xpos[to_index(3 * idx.fork_lower_body + 2)] =
            xpos[to_index(3 * idx.steer_body + 2)] + 0.89879410f * travel_m;

        const GpuVec2 e_fork = gpu_fork_axis_world(idx, xpos.data(), 0.0f);
        require_near(e_fork.x, -0.43837105f, 1e-4f,
                     "fork axis x never flips across compressed and top-out states");
        require_near(e_fork.y, 0.89879410f, 1e-4f,
                     "fork axis z never flips across compressed and top-out states");
        // Upward normal force (+700 N) must ALWAYS project to positive (compressive) fork_travel force
        const float f_fork = e_fork.y * 700.0f;
        require(f_fork > 600.0f, "upward load projects to a compressive fork force");
    }

    // Fallback when xpos is nullptr must match (-0.43837105, 0.89879410) at zero pitch
    // and rotate consistently with root_pitch_rad.
    const GpuVec2 e_fb0 = gpu_fork_axis_world(idx, nullptr, 0.0f);
    require_near(e_fb0.x, -0.43837105f, 1e-5f, "fork axis fallback x at zero pitch");
    require_near(e_fb0.y, 0.89879410f, 1e-5f, "fork axis fallback z at zero pitch");

    const float pitch = 0.25f;
    const GpuVec2 e_fb_p = gpu_fork_axis_world(idx, nullptr, pitch);
    const float exp_fx = std::cos(pitch) * (-0.43837105f) + std::sin(pitch) * 0.89879410f;
    const float exp_fz = -std::sin(pitch) * (-0.43837105f) + std::cos(pitch) * 0.89879410f;
    require_near(e_fb_p.x, exp_fx, 1e-5f, "fork axis fallback x rotates with root pitch");
    require_near(e_fb_p.y, exp_fz, 1e-5f, "fork axis fallback z rotates with root pitch");

    // Rear pivot fallback must also rotate frame-local offsets by root_pitch_rad
    const GpuVec2 a_main_p = gpu_rear_pivot_anchor_world(
        idx.chainstay_body, nullptr, 1.0f, 0.5f, kMainPivotLocalDxM, kMainPivotLocalDzM, pitch);
    const float exp_mx = 1.0f + std::cos(pitch) * kMainPivotLocalDxM + std::sin(pitch) * kMainPivotLocalDzM;
    const float exp_mz = 0.5f - std::sin(pitch) * kMainPivotLocalDxM + std::cos(pitch) * kMainPivotLocalDzM;
    require_near(a_main_p.x, exp_mx, 1e-5f, "main pivot anchor x rotates with root pitch");
    require_near(a_main_p.y, exp_mz, 1e-5f, "main pivot anchor z rotates with root pitch");
}

// Seatstay (horst) anchor without xpos when main_pivot != 0. The fallback must rotate the seatstay
// offset by root_pitch + main_pivot (gpu_horst_pivot_anchor_fallback). Checks:
//  1. the fallback equals an independent double-precision evaluation of the chain
//     R(root_pitch) * kMainPivotLocal + R(root_pitch + main_pivot) * kSeatstayFromChainstay;
//  2. gpu_horst_pivot_anchor_world with xpos == nullptr returns the same anchor;
//  3. project_rear_wheel_force_to_qfrc with xpos == nullptr writes the horst torque about that anchor;
//  4. the pre-fix anchor (kHorstPivotLocal rotated by root_pitch only) is about 17 mm away at
//     main_pivot = 0.05 rad, so checks 1-3 fail for it.
void test_horst_pivot_fallback_main_pivot() {
    const GpuModelIndices idx{};
    constexpr float kComX = 0.10f;
    constexpr float kComZ = 0.55f;
    constexpr float kPx = -0.52f;
    constexpr float kPz = 0.33f;
    constexpr float kFx = 300.0f;
    constexpr float kFz = 150.0f;

    for (const float pitch : {0.0f, 0.08f, -0.12f}) {
        for (const float main_pivot : {0.0f, 0.05f, -0.05f}) {
            const double rp = static_cast<double>(pitch);
            const double total = rp + static_cast<double>(main_pivot);
            const double ref_x = static_cast<double>(kComX)
                + std::cos(rp) * static_cast<double>(kMainPivotLocalDxM)
                + std::sin(rp) * static_cast<double>(kMainPivotLocalDzM)
                + std::cos(total) * static_cast<double>(kSeatstayFromChainstayDxM)
                + std::sin(total) * static_cast<double>(kSeatstayFromChainstayDzM);
            const double ref_z = static_cast<double>(kComZ)
                - std::sin(rp) * static_cast<double>(kMainPivotLocalDxM)
                + std::cos(rp) * static_cast<double>(kMainPivotLocalDzM)
                - std::sin(total) * static_cast<double>(kSeatstayFromChainstayDxM)
                + std::cos(total) * static_cast<double>(kSeatstayFromChainstayDzM);

            const GpuVec2 a = gpu_horst_pivot_anchor_fallback(kComX, kComZ, pitch, main_pivot);
            require_near(static_cast<double>(a.x), ref_x, 1e-6,
                         "horst fallback anchor x matches the double-precision chain");
            require_near(static_cast<double>(a.y), ref_z, 1e-6,
                         "horst fallback anchor z matches the double-precision chain");

            const GpuVec2 w = gpu_horst_pivot_anchor_world(
                idx.seatstay_body, nullptr, kComX, kComZ, pitch, main_pivot);
            require_near(w.x, a.x, 1e-7f, "horst world anchor x matches the fallback");
            require_near(w.y, a.y, 1e-7f, "horst world anchor z matches the fallback");

            std::array<float, kMaxNv> qfrc{};
            project_rear_wheel_force_to_qfrc(
                idx, nullptr, kComX, kComZ, kPx, kPz, kFx, kFz, qfrc.data(), pitch, main_pivot);
            const float expected_horst = (kPz - a.y) * kFx - (kPx - a.x) * kFz;
            require_near(qfrc[to_index(idx.horst_pivot_dof)], expected_horst,
                         1e-4f * std::max(1.0f, std::abs(expected_horst)),
                         "horst qfrc torque matches the moment about the fallback anchor");
        }
    }

    const GpuVec2 a_pre_fix = gpu_rear_pivot_anchor_world(
        -1, nullptr, kComX, kComZ, kHorstPivotLocalDxM, kHorstPivotLocalDzM, 0.0f);
    const GpuVec2 a_new = gpu_horst_pivot_anchor_fallback(kComX, kComZ, 0.0f, 0.05f);
    require(std::hypot(a_new.x - a_pre_fix.x, a_new.y - a_pre_fix.y) > 1e-2f,
            "the corrected anchor is more than 10 mm from the pre-fix anchor");
}

constexpr std::array<TestCase, 6> cases{{
    {.name = "station_density_and_response",
     .run = test_station_density_and_response},
    {.name = "heightfield_1d", .run = test_heightfield_1d},
    {.name = "evaluate_wheel_contact_and_qfrc",
     .run = test_evaluate_wheel_contact_and_qfrc},
    {.name = "gpu_write_both_wheels_suspension_coupling",
     .run = test_gpu_write_both_wheels_suspension_coupling},
    {.name = "fork_axis_world_top_out_and_pitch",
     .run = test_fork_axis_world_top_out_and_pitch},
    {.name = "horst_pivot_fallback_main_pivot",
     .run = test_horst_pivot_fallback_main_pivot},
}};

int run_case(const TestCase &test_case) {
    try {
        test_case.run();
        std::cout << "PASS " << test_case.name << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
        return 1;
    }
}
} // namespace

int main(int argc, char **argv) {
    try {
        const std::span<char *> arguments =
            bike::platform::main_arguments(argc, argv);
        if (arguments.size() == 2 &&
            std::string_view(arguments[1]) == "--list") {
            for (const TestCase &test_case: cases)
                std::cout << test_case.name << '\n';
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_tire [case-name|--list]\n";
            return 2;
        }

        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &test_case: cases) {
            if (requested == "all" || requested == test_case.name) {
                found = true;
                if (run_case(test_case) != 0)
                    status = 1;
            }
        }
        if (!found) {
            std::cerr << "unknown gpu tire case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        // A throwing argument envelope is a harness failure, not a usage error.
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
