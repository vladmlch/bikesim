// writers/compliant_3d_tire.hpp — 3D toroidal crown-circle compliant tire
// writer (`compliant_3d` backend). Implements the full TireWriterView surface
// plus midpoint-velocity `step_loss_j(qvel_mid, dt)` for exact 3D energy
// ledger closure, with zero `io/*` / `platform/*` includes and zero link
// symbol dependencies on `bike_terrain`.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "rtsan.hpp"
#include "spatial/types.hpp"
#include "terrain/terrain3d_types.hpp"
#include "writers/tire_common.hpp"
#include "writers/writer_types.hpp"

// Type-erased, noexcept, allocation-free terrain query callbacks so `bike_core`
// has zero link-time dependency on `bike_terrain`.
struct Compliant3dTerrainCallbacks {
    const void *ctx = nullptr;
    bike::terrain::TerrainSample3d (*sample_fn)(const void *, double,
                                                double) noexcept = nullptr;
    bike::terrain::TorusClosestPoint3d (*closest_point_fn)(
        const void *, const bike::terrain::Vec3d &, double) noexcept = nullptr;

    [[nodiscard]] bool valid() const noexcept {
        return ctx != nullptr && sample_fn != nullptr &&
               closest_point_fn != nullptr;
    }
};

template <class Source>
[[nodiscard]] Compliant3dTerrainCallbacks
make_terrain3d_callbacks(const Source &src) noexcept {
    return Compliant3dTerrainCallbacks{
        .ctx = &src,
        .sample_fn = [](const void *c, double x,
                        double y) noexcept -> bike::terrain::TerrainSample3d {
            return static_cast<const Source *>(c)->sample(x, y);
        },
        .closest_point_fn = [](const void *c,
                               const bike::terrain::Vec3d &center,
                               double search_radius_m) noexcept
            -> bike::terrain::TorusClosestPoint3d {
            return static_cast<const Source *>(c)->closest_point(
                center, search_radius_m);
        },
    };
}

template <class Source>
Compliant3dTerrainCallbacks
make_terrain3d_callbacks(const Source &&) = delete;

// Per-wheel calibrated physical parameters (spec §1.2).
struct Compliant3dWheelParams {
    double pressure_bar = 1.20;
    double outer_radius_m = 0.3790;
    double rim_radius_m = 0.3200;
    double crown_radius_m = 0.0355;
    double compressed_casing_m = 0.0060;
    double k_pneumatic_n_m = 53290.0;
    double delta_0_m = 0.00025;
    double k_carcass_n_m = 18650.0;
    double delta_carcass_m = 0.0357;
    double k_rim_n_m = 350000.0;
    double delta_rim_0_m = 0.0020;
    double loss_factor = 0.0550;
    double hysteresis_rate_eps_mps = 0.0100;
    double c_alpha_per_fz = 13.90;
    double k_brush_x_n_m = 83600.0;
    double k_brush_y_n_m = 36300.0;
    double mu_cap = 1.10;
    std::array<double, 2> valid_load_range_n{0.0, 3500.0};

    [[nodiscard]] constexpr double major_radius_m() const noexcept {
        return outer_radius_m - crown_radius_m;
    }

    [[nodiscard]] constexpr double rim_strike_deflection_m() const noexcept {
        return (outer_radius_m - rim_radius_m) - compressed_casing_m;
    }
};

struct Compliant3dTireConfig {
    Compliant3dWheelParams front{};
    Compliant3dWheelParams rear{};
    std::string surface_mode = "track";
    Compliant3dTerrainCallbacks terrain{};
    double distinct_normal_deg = 15.0;
    std::optional<double> configured_mu{};
    double configured_c_kappa_per_fz = 12.0;
};

[[nodiscard]] Compliant3dTireConfig make_compliant_3d_tire_config(
    bike::spatial::SpatialTirePreset preset,
    Compliant3dTerrainCallbacks terrain = {});

// Persistent 2D parallel-transported bristle state per wheel (spec §1.6.1).
struct BrushState3d {
    bool active = false;
    double ux = 0.0;
    double uy = 0.0;
    std::array<double, 3> tx{1.0, 0.0, 0.0};
    std::array<double, 3> ty{0.0, 1.0, 0.0};
    std::array<double, 3> point{};
    std::array<double, 3> center{};
};

class Compliant3dTireWriter {
public:
    Compliant3dTireWriter(const mjModel *m, const mjData *d,
                          Compliant3dTireConfig config);

    void reset();

    [[nodiscard]] std::vector<double> qfrc(const mjData *d, double dt);
    void compute_into(const mjData *d, double dt, std::span<double> out);
    [[nodiscard]] CoreStatus
    try_compute_into(const mjData *d, double dt,
                     std::span<double> out) noexcept BIKE_NONBLOCKING;

    void set_state(std::span<const std::string> names,
                   std::span<const double> row);
    void prepare_state(std::span<const std::string> names,
                       std::span<const double> row);
    void commit_prepared_state() noexcept;

    [[nodiscard]] static const std::vector<std::string> &state_names();
    [[nodiscard]] std::vector<double> state() const;

    [[nodiscard]] std::array<std::optional<TireSnapshot>, 2>
    snapshots() const {
        std::array<std::optional<TireSnapshot>, 2> out{};
        if (committed_) {
            for (std::size_t i = 0; i < 2; ++i) out[i] = snapshots_[i];
        }
        return out;
    }

    [[nodiscard]] std::array<std::optional<TireDiagnostics>, 2>
    diagnostics() const {
        std::array<std::optional<TireDiagnostics>, 2> out{};
        if (committed_) {
            for (std::size_t i = 0; i < 2; ++i) out[i] = diagnostics_[i];
        }
        return out;
    }

    [[nodiscard]] std::vector<double> probe_qfrc(const mjData *d, double dt);
    void probe_compute_into(const mjData *d, double dt, std::span<double> out);

    [[nodiscard]] std::array<std::optional<TireSnapshot>, 2>
    probe_snapshots() const {
        std::array<std::optional<TireSnapshot>, 2> out{};
        if (probe_committed_) {
            for (std::size_t i = 0; i < 2; ++i) out[i] = probe_snapshots_[i];
        }
        return out;
    }

    [[nodiscard]] const std::array<double, 2> &radii() const noexcept {
        return radii_;
    }

    void restart_clock() noexcept { last_time_s_.reset(); }

    [[nodiscard]] double stored_energy(const mjData *d) const;

    [[nodiscard]] const std::array<TireSnapshot, 2> &
    snapshot_storage() const noexcept {
        return snapshots_;
    }
    [[nodiscard]] bool snapshots_committed() const noexcept {
        return committed_;
    }
    [[nodiscard]] const std::array<TireDiagnostics, 2> &
    diagnostic_storage() const noexcept {
        return diagnostics_;
    }
    [[nodiscard]] const std::array<TireSnapshot, 2> &
    probe_snapshot_storage() const noexcept {
        return probe_snapshots_;
    }
    [[nodiscard]] bool probe_committed() const noexcept {
        return probe_committed_;
    }

    [[nodiscard]] double elastic_energy_j() const noexcept {
        return elastic_energy_j_;
    }
    [[nodiscard]] double brush_loss_step_j() const noexcept {
        return brush_loss_step_j_;
    }
    void clear_settled_loss() noexcept { brush_loss_step_j_ = 0.0; }

    [[nodiscard]] double radial_dissipation_power_w() const noexcept {
        return radial_dissipation_power_w_;
    }

    // Midpoint-velocity step dissipation ledger (spec §1.8.1).
    [[nodiscard]] double step_loss_j(std::span<const double> qvel_mid,
                                     double dt) const noexcept;

private:
    const mjModel *m_ = nullptr;
    Compliant3dTireConfig cfg_;
    int nv_ = 0;
    std::array<int, 2> geoms_{};
    std::array<int, 2> bodies_{};
    std::array<double, 2> radii_{};

    std::vector<mjtNum> jac_contact_;
    std::vector<mjtNum> jac_center_;
    std::vector<double> tmp_;
    std::vector<double> qfrc_out_;
    std::vector<double> qfrc_dissipative_;
    std::vector<double> qfrc_diss_stage_;

    std::array<TireSnapshot, 2> snapshot_stage_{};
    std::array<TireDiagnostics, 2> diagnostics_stage_{};
    static constexpr std::size_t kMaxSnapshotPatches = 1;

    std::array<BrushState3d, 2> states_{};
    std::array<BrushState3d, 2> prepared_states_{};
    std::array<TireSnapshot, 2> snapshots_{};
    std::array<TireDiagnostics, 2> diagnostics_{};
    bool committed_ = false;
    std::array<TireSnapshot, 2> probe_snapshots_{};
    bool probe_committed_ = false;

    double elastic_energy_j_ = 0.0;
    double brush_loss_step_j_ = 0.0;
    double radial_dissipation_power_w_ = 0.0;
    double dissipative_power_start_w_ = 0.0;
    std::optional<double> last_time_s_;

    [[nodiscard]] CoreStatus
    accumulate_core(const mjData *d, double dt, std::span<double> out,
                    const char *&err_msg, bool &overflow_err);
};
