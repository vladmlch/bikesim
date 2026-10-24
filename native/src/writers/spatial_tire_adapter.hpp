#pragma once

#include "writers/pneumatic_3d_tire.hpp"
#include "writers/tire_common.hpp"
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace bike::pneumatic3d {
// Cold error translation keeps invalid spatial contact distinct from an
// airborne success and preserves a closed reason for the outcome owner.
class SpatialTireEvaluationError final : public std::runtime_error {
public:
    SpatialTireEvaluationError(CoreStatus status_value,ReasonCode3d reason_value)
        :std::runtime_error("pneumatic_3d: force evaluation rejected"),status(status_value),reason(reason_value){}
    CoreStatus status;
    ReasonCode3d reason;
};
}

// Internal bridge to the established TireWriterView surface. Physics and the
// canonical spatial state remain owned by Pneumatic3dTireWriter. These cached
// snapshots are projections, never an independent force/contact producer.
class SpatialTireAdapter {
public:
    SpatialTireAdapter(const mjModel&,const bike::spatial::Topology&,
                      bike::pneumatic3d::Pneumatic3dTireWriter&,
                      bike::terrain::Terrain3dQueryView);
    CoreStatus try_compute_into(const mjData*,double,std::span<double>) noexcept;
    void compute_into(const mjData*,double,std::span<double>);
    void probe_compute_into(const mjData*,double,std::span<double>);
    std::vector<double> qfrc(const mjData*,double);
    std::vector<double> probe_qfrc(const mjData*,double);
    std::array<std::optional<TireSnapshot>,2> snapshots() const;
    std::array<std::optional<TireSnapshot>,2> probe_snapshots() const;
    std::array<std::optional<TireDiagnostics>,2> diagnostics() const;
    const std::array<TireSnapshot,2>& snapshot_storage() const noexcept{return snapshots_;}
    const std::array<TireSnapshot,2>& probe_snapshot_storage() const noexcept{return probes_;}
    const std::array<TireDiagnostics,2>& diagnostic_storage() const noexcept{return diagnostics_;}
    bool snapshots_committed() const noexcept{return committed_;}
    bool probe_committed() const noexcept{return probed_;}
    const std::array<double,2>& radii() const noexcept{return radii_;}
    double stored_energy(const mjData*) const noexcept{return writer_->model().stored_energy_j();}
    double elastic_energy_j() const noexcept{return writer_->model().stored_energy_j();}
    double brush_loss_step_j() const noexcept{return brush_loss_;}
    double radial_dissipation_power_w() const noexcept{return radial_power_;}
    void set_state(std::span<const std::string>,std::span<const double>);
    void prepare_state(std::span<const std::string>,std::span<const double>);
    void commit_prepared_state() noexcept{}
    std::vector<double> state() const;
    const std::vector<std::string>& state_names() const noexcept{return no_planar_columns_;}
    void reset();
    void restart_clock();
    void clear_settled_loss() noexcept{brush_loss_=radial_power_=0.;}
    void invalidate_projection() noexcept;
private:
    CoreStatus capture_motion(const mjData&) noexcept;
    CoreStatus project(const mjData&) noexcept;
    void publish_probe(const mjData&) noexcept;
    const mjModel* model_;
    bike::spatial::Topology topology_;
    bike::pneumatic3d::Pneumatic3dTireWriter* writer_;
    bike::terrain::Terrain3dQueryView terrain_;
    std::array<bike::spatial::BodyPose,2> wheel_motion_{};
    std::array<bike::spatial::Vec3,2> forward_{};
    bike::pneumatic3d::TyreState3d initial_;
    std::array<TireSnapshot,2> snapshots_,probes_,projection_stage_;
    std::array<TireDiagnostics,2> diagnostics_{};
    std::array<double,2> radii_{};
    std::vector<std::string> no_planar_columns_;
    bool committed_=false,probed_=false;
    double brush_loss_=0.,radial_power_=0.;
};
