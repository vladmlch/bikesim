// writers/tire_common.hpp — tyre-writer internals shared between
// TireWriter (writers/tire.cpp, compliant_2d), DistributedTireWriter
// (writers/distributed_tire.cpp, distributed_2d_reference) and
// PneumaticTyreModel (pneumatic/model.cpp, pneumatic): the brush
// step, the effective-friction resolution and the SurfaceMap lookup that
// used to live inside tire.cpp, plus the snapshot/patch/diagnostics
// structs the writers publish.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../config_types.hpp"
#include "../contact/laws.hpp"

namespace bike::physics_core {
template <class Real>
struct SurfaceSpecPod;
template <class Real>
struct SurfaceMapView;
} // namespace bike::physics_core

// ContactPatch (contact_state.py:19-76). Applied tire writers use the
// planar force reconstruction; native_reference retains the complete
// engine wrench, including its contact couple and out-of-plane force.
struct TirePatch {
    std::array<double, 3> point_m;
    std::array<double, 3> normal;
    double normal_load_n;
    double tangent_force_n;
    double slip_mps;
    bool working_surface;
    std::optional<std::array<double, 3>> native_world_force_n;
    std::array<double, 3> couple_world_nm{};
};

// WheelContactSnapshot (contact_state.py:79-190) for one side, plus the
// effective_radius_m property precomputed the way contact_state.py
// computes it (load-weighted mean contact drop; unit weights when every
// patch is unloaded; 0 when there is no patch).
struct TireSnapshot {
    double time_s{};
    std::int64_t interval_id{};
    std::string backend;
    bool geometric_contact{};
    std::array<double, 3> wheel_axis_m{};
    std::vector<TirePatch> patches;
    double effective_radius_m{};
};

// The per-side diagnostics dict the Python writer assembles inside the
// loop (tire_forces.py:190-199) — exposed for parity checks/debugging.
struct TireDiagnostics {
    bool multi_support;
    double penetration_m;
    double normal_speed_mps;
    double slip_mps;
    double normal_load_n;
    double tangent_force_n;
    double friction_coefficient;
    std::string surface;
    double branch_release_loss_j;
    double brush_loss_j;
    double radial_energy_j;
    double shear_energy_j;
    bool outside_material_load_range;
    bool supports_multiple_contacts{};
    int station_count{}, active_station_count{},
            nonsmooth_station_count{}, max_patch_count{};
    bool outside_material_deflection_range{};
    std::string calibration_status, capability_status;
    double support_n{}, slip_ratio{}, transient_slip_ratio{},
            contact_length_m{};
    int patch_count{};
    bool fully_sliding{}, airborne{}, coverage_event{},
            rim_strike_active{};
    double rim_event_x_m{}, rim_event_speed_mps{},
            rim_event_peak_load_n{}, rim_event_peak_force_n{},
            rim_event_energy_j{};
    double pressure_bar{}, dissipated_power_w{};
};

namespace tirecommon {
    // brush_step's result channels (tire.py:157-190): the transported
    // brush deflection, the tangential force and the step's dissipation.
    struct BrushResult {
        double xi, force, loss;
    };

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror tire.py brush step
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] BrushResult
    brush_step(double xi, double u, double v_roll, double fn, double k,
               double mu, double length, double dt);
    // NOLINTEND(bugprone-easily-swappable-parameters)

    // surface.py:72-88 — SurfaceSpec.mu: mu_slide + (peak-slide) *
    // exp(-|slip|/stribeck_speed_mps).
    [[nodiscard]] double surface_mu(const nativecfg::SurfaceSpec &s,
                                    double slip);

    // surface.py:194-206 — SurfaceMap.at: finite query, first matching
    // half-open section wins, else the default material.
    [[nodiscard]] const nativecfg::SurfaceSpec &
    surface_at(const nativecfg::SurfaceMap &map, double x_m);

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] std::pair<double, std::string_view>
    effective_friction(double tire_mu, const nativecfg::SurfaceMap *map,
                       double contact_x, double slip);
    // NOLINTEND(bugprone-easily-swappable-parameters)

    [[nodiscard]] bike::physics_core::SurfaceSpecPod<double>
    to_surface_spec_pod(const nativecfg::SurfaceSpec &s) noexcept;

    [[nodiscard]] bool
    fill_surface_map_view(const nativecfg::SurfaceMap *map,
                          bike::physics_core::SurfaceMapView<double> &out) noexcept;

    inline constexpr double kSurfaceReferenceLoadN = 500.0;
    inline constexpr double kSurfaceLoadSmoothDelta = 0.1;
    inline constexpr double kReferenceSlipStiffnessPerLoad = 15.0;
    inline constexpr double kWheelRollingTaperRadps = 0.2;

    // Stage A evaluated track-surface sample (Spec §2.4). `name` borrows
    // storage from the queried SurfaceSpec / SurfaceMap, which must outlive
    // the sample.
    struct SurfaceSample {
        std::string_view name{};
        double mu_peak = 0.0;
        double mu_slide = 0.0;
        double slip_stiffness_per_load = 0.0;
        double stribeck_speed_mps = 4.5;
        double crr_delta = 0.0;
        double load_sensitivity = 0.0;
    };

    [[nodiscard]] SurfaceSample
    sample_from_spec(const nativecfg::SurfaceSpec &s) noexcept;

    // Precondition: map passed nativecfg::validate and sections and
    // micro_spans are start_m-sorted (writer constructors guarantee both).
    [[nodiscard]] SurfaceSample
    eval_surface(const nativecfg::SurfaceMap &map, double x_m);

    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    [[nodiscard]] const nativecfg::SurfaceSpec *
    uniform_surface_for_span(const nativecfg::SurfaceMap &map, double x_min_m,
                             double x_max_m);

    [[nodiscard]] double load_sensitivity_factor(double wheel_normal_load_n,
                                                 double k_z) noexcept;

    [[nodiscard]] double sample_mu(const SurfaceSample &s,
                                   double sliding_speed_mps,
                                   double wheel_normal_load_n);
    // NOLINTEND(bugprone-easily-swappable-parameters)
} // namespace tirecommon
