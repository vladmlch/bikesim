// writers/distributed_3d_tire.hpp — 128-station Lagrangian toroidal 3D
// distributed tire writer (`distributed_3d` backend). Implements the full
// TireWriterView surface plus midpoint-velocity `step_loss_j(qvel_mid, dt)`
// for exact 3D energy ledger closure, with zero `io/*` / `platform/*`
// includes and zero link symbol dependencies on `bike_terrain`.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "rtsan.hpp"
#include "spatial/types.hpp"
#include "terrain/terrain3d_types.hpp"
#include "tyre/station.hpp"
#include "writers/compliant_3d_tire.hpp"
#include "writers/tire_common.hpp"
#include "writers/writer_types.hpp"

// Per-wheel calibrated physical parameters for the 128-station Lagrangian
// toroidal distributed tire (S5 spec §3.2).
struct Distributed3dWheelParams {
    int station_count = 128;
    double pressure_bar = 1.20;
    double outer_radius_m = 0.3790;
    double rim_radius_m = 0.3200;
    double crown_radius_m = 0.0355;
    double compressed_casing_m = 0.0060;
    double kappa_p_n_m_rad = 201000.0;
    double delta_0_m = 0.00050;
    double kappa_c_n_m_rad = 15000.0;
    double delta_carcass_m = 0.0200;
    double kappa_rim_n_m_rad = 1400000.0;
    double delta_rim_0_m = 0.0020;
    double loss_factor = 0.0550;
    double hysteresis_rate_eps_mps = 0.0100;
    double kappa_brush_x_n_m_rad = 200000.0;
    double kappa_brush_y_n_m_rad = 185300.0;
    double c_kappa_ref_per_fz = 15.0;
    double mu_cap = 1.10;
    std::array<double, 2> valid_load_range_n{0.0, 3500.0};
    std::array<double, 2> valid_deflection_range_m{0.0, 0.060};

    [[nodiscard]] constexpr double major_radius_m() const noexcept {
        return outer_radius_m - crown_radius_m;
    }

    [[nodiscard]] constexpr double rim_strike_deflection_m() const noexcept {
        return (outer_radius_m - rim_radius_m) - compressed_casing_m;
    }
};

struct Distributed3dTireConfig {
    Distributed3dWheelParams front{};
    Distributed3dWheelParams rear{};
    std::string surface_mode = "track";
    Compliant3dTerrainCallbacks terrain{};
    double distinct_normal_deg = 15.0;
    std::optional<double> configured_mu{};
    double configured_c_kappa_per_fz = 12.0;
};

[[nodiscard]] Distributed3dTireConfig make_distributed_3d_tire_config(
    bike::spatial::SpatialTirePreset preset,
    Compliant3dTerrainCallbacks terrain = {});

// Persistent 2D parallel-transported bristle state per station (S5 spec §3.4.3).
struct StationState3d {
    bool active = false;
    double ux = 0.0;
    double uy = 0.0;
    std::array<double, 3> tx{1.0, 0.0, 0.0};
    std::array<double, 3> ty{0.0, 1.0, 0.0};
};

class Distributed3dTireWriter {
public:
    Distributed3dTireWriter(const mjModel *m, const mjData *d,
                            Distributed3dTireConfig config);

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

    [[nodiscard]] const std::vector<std::string> &
    state_names() const noexcept {
        return state_names_;
    }
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

    // Midpoint-velocity step dissipation ledger (S5 spec §3.4.3).
    [[nodiscard]] double step_loss_j(std::span<const double> qvel_mid,
                                     double dt) const noexcept;

private:
    const mjModel *m_ = nullptr;
    Distributed3dTireConfig cfg_;
    int nv_ = 0;
    std::array<int, 2> geoms_{};
    std::array<int, 2> bodies_{};
    std::array<double, 2> radii_{};

    std::array<biketyre::StationAngles, 2> stations_{};
    std::array<std::vector<std::array<double, 3>>, 2> local_{};
    std::array<std::vector<StationState3d>, 2> states_{};
    std::array<std::vector<StationState3d>, 2> states_stage_{};
    std::array<std::vector<StationState3d>, 2> prepared_states_{};

    std::vector<std::string> state_names_{};
    std::unordered_map<std::string, std::size_t> canonical_{};
    std::vector<double> wire_cols_{};
    std::vector<char> wire_seen_{};

    mutable std::vector<double> points_{};
    std::vector<int> active_ids_{};
    std::vector<double> normals_{};
    std::vector<mjtNum> jac_center_{};
    std::vector<mjtNum> jr_{};
    std::vector<mjtNum> jac_station_{};
    std::vector<double> tmp_{};
    std::vector<double> qfrc_out_{};
    std::vector<double> qfrc_dissipative_{};
    std::vector<double> qfrc_diss_stage_{};

    std::array<TireSnapshot, 2> snapshot_stage_{};
    std::array<TireDiagnostics, 2> diagnostics_stage_{};

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
