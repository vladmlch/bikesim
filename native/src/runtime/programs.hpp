// Research programs: explicit field ownership and oracle operation order.
#pragma once
#include <vector>
#include "samples.hpp"
namespace runtime {
[[nodiscard]] WireObject research_control_wire(const RideControl &control);
[[nodiscard]] bool controls_equal(const RideControl &a, const RideControl &b);
[[nodiscard]] RideControl research_control_from_wire(const WireObject &value);
struct RiderKeyframe {
    double time_s = 0.;
    rider::RiderPosture posture;
    std::optional<double> human_torque_nm;
};
class ResearchPrograms {
public:
    std::vector<RiderKeyframe> rider_frames;
    std::vector<std::array<double, 2>> demand_frames;
    double reaction_delay_s = 0.;
    void validate() const;
    void validate_control(const RideControl &control) const;
    [[nodiscard]] RideControl apply(const RideControl &control, double time_s) const;
    [[nodiscard]] std::optional<double> demand_at(double time_s) const;
    [[nodiscard]] static ResearchPrograms from_wire(const Wire &rider_program,
                                                   const Wire &demand_program);
};
} // namespace runtime
