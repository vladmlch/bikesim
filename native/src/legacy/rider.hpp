#pragma once
#include <array>
#include <optional>
#include <span>
#include <string_view>
#include <vector>
#include "../model/rider_geometry.hpp"
#include "../writers/rider_forces.hpp"

namespace bike::legacy {
struct RiderLoads {
    double saddle_n = 0., gap_m = 0., torso_n = 0., bar_n = 0.;
    double front_pedal_n = 0., rear_pedal_n = 0.;
};
class RiderForces {
public:
    RiderForces(const mjModel&, const std::optional<model::SeatedPose>&);
    [[nodiscard]] bool active() const noexcept { return !paths_.empty(); }
    void set_pedal_offsets(double front_m, double rear_m);
    void compute_into(const mjData&, std::span<double> force);
    [[nodiscard]] RiderLoads loads() const;
    [[nodiscard]] double stored_energy(const mjData&) const;
private:
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    const mjModel& model_;
    std::vector<RiderForcesWriter::Path> paths_;
    std::vector<RiderForcesWriter::Telemetry> telemetry_, staging_;
};
struct MechanicalEnergy { double kinetic_j, gravitational_j, elastic_j; };
struct SystemMomentum { std::array<double,3> linear, angular; };
[[nodiscard]] MechanicalEnergy mechanical_energy_terms(const mjModel&, mjData&, double elastic_energy_j);
[[nodiscard]] SystemMomentum system_momentum(const mjModel&, mjData&);
class EnergyLedger {
public:
    explicit EnergyLedger(double initial_energy_j);
    [[nodiscard]] double residual(double energy_j, double active_work_j,
        double external_work_j, double loss_j) const;
private:
    double initial_;
};
} // namespace bike::legacy
