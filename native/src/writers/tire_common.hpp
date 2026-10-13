// writers/tire_common.hpp — tyre-writer internals shared between
// TireWriter (writers/tire.cpp, compliant_2d) and DistributedTireWriter
// (writers/distributed_tire.cpp, distributed_2d_reference): the brush
// step, the effective-friction resolution and the SurfaceMap lookup that
// used to live inside tire.cpp, plus the snapshot/patch/diagnostics
// structs both writers publish.
//
// The helper bodies were moved verbatim — every floating-point
// expression keeps its statement, operation order and rounding, so the
// extraction cannot change TireWriter's proved warm-core results.
// brush_step's only implementation is contactlaw::brush_step in
// contact/laws.hpp (already shared with the rider-contact writer); the
// tirecommon face repacks its tuple into named fields, which adds no
// arithmetic and is bitwise-transparent.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../config_types.hpp"
#include "../contact/laws.hpp"

// ContactPatch (contact_state.py:19-76) — tire patches always carry
// source 'terrain' (working_surface=true), no couple, no native force.
struct TirePatch {
    std::array<double, 3> point_m;
    std::array<double, 3> normal;
    double normal_load_n;
    double tangent_force_n;
    double slip_mps;
    bool working_surface;
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
// The trailing fields are the distributed backend's scalars and status
// labels (design spec §table "Диагностика"): the compliant_2d writer
// never assigns them, so their zero defaults keep its staged diagnostics
// — and the wire, whose emitter reads only the fields above — identical.
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
    // The pneumatic backend's per-side dict (pneumatic_tire_forces.py
    // :426-455) — same all-default contract as the distributed fields
    // above: neither existing backend assigns them, so their staged
    // zeros change nothing on the compliant/distributed wire.
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
    [[nodiscard]] inline BrushResult
    brush_step(double xi, double u, double v_roll, double fn, double k,
               double mu, double length, double dt) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        // contactlaw::brush_step is the single implementation — it was
        // already shared with rider_contacts.cpp when this extraction
        // was planned. Repacking the tuple adds no arithmetic.
        const auto [new_xi, force, loss] =
                contactlaw::brush_step(xi, u, v_roll, fn, k, mu, length,
                                       dt);
        return {.xi = new_xi, .force = force, .loss = loss};
    }

    // surface.py:72-88 — SurfaceSpec.mu: mu_slide + (peak-slide) *
    // exp(-|slip|/stribeck_speed_mps). Exported for the pneumatic
    // writer's zero-load `surface_map.at(hub_x).mu(0.0)` fallback;
    // the body moved out of the anonymous namespace unchanged.
    [[nodiscard]] double surface_mu(const nativecfg::SurfaceSpec &s,
                                    double slip);

    // surface.py:194-206 — SurfaceMap.at: finite query, first matching
    // half-open section wins, else the default material.
    [[nodiscard]] const nativecfg::SurfaceSpec &
    surface_at(const nativecfg::SurfaceMap &map, double x_m);

    // tire_forces.py:56-61 — effective_friction: the configured mu when
    // there is no surface map, else the queried section's Stribeck mu
    // capped by the tire's own mu. The returned name is a view into the
    // map's storage (or the static 'configured' literal) — it borrows
    // from *map, which must outlive the view's use.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror the call site's Python order
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] std::pair<double, std::string_view>
    effective_friction(double tire_mu, const nativecfg::SurfaceMap *map,
                       double contact_x, double slip);
    // NOLINTEND(bugprone-easily-swappable-parameters)
} // namespace tirecommon
