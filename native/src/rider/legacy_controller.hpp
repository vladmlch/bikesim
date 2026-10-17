#pragma once
#include "spindle_controller.hpp"
#include "support_dispatch.hpp"

namespace bike::rider {
// Implementation branch of the same model-owned runtime controller.
// Mutable activation/warm-start/recovery state stays in its façade and is
// included in SpindleController::State; this object owns only topology.
class LegacyController {
public:
    explicit LegacyController(::rider::SpindleController& owner);
    [[nodiscard]] ::rider::NamedEntries<double> compute(mjData&,const ::rider::RiderCommand&,
        bool advance,double dt,bool steady,const std::array<bool,4>& supports,bool recovery);
    void initialize(mjData&);
    void initialize_velocity(mjData&);
private:
    [[nodiscard]] std::vector<double> targets(const mjData&,std::size_t side,double depth,
        double shear,const ::rider::RiderPosture* posture);
    [[nodiscard]] std::vector<double> upper_targets(const mjData&,const ::rider::RiderPosture*,
        const std::array<double,3>* grip_force);
    [[nodiscard]] std::array<double,2> estimate(const mjData&,std::size_t side) const;
    [[nodiscard]] ResponseRequest allocation_request(const mjData&,std::span<const double> intent,
        double dt,bool steady,std::optional<double> crank) const;
    [[nodiscard]] ResponseAllocation allocate_uncoupled(const mjData&,const ResponseRequest&,
        const PedalingSupportTargets&,const std::array<std::array<double,2>,2>& estimates);
    ::rider::SpindleController& c_;
    std::vector<AllocationAttachment> attachments_;
    std::vector<int> rider_dofs_,bike_dofs_;
    std::array<std::array<double,3>,2> sole_offsets_{};
    struct LegGeometry {double upper,lower,a0,b0;int branch;};
    std::array<LegGeometry,2> legs_{};
    std::array<FootTargetDiagnostics,2> target_diagnostics_{};
};
} // namespace bike::rider
