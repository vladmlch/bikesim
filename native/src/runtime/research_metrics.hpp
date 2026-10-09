// Physics-rate research truth and evaluation, never policy inputs or forces.
#pragma once
#include <map>
#include "samples.hpp"
namespace runtime {
struct TruthGeometry {
    std::vector<double> x, z;
    double front_radius = 0., rear_radius = 0.;
    std::size_t root_x_qpos = 0, root_x_dof = 0, root_pitch_qpos = 0, root_pitch_dof = 0;
    void validate(std::size_t nq, std::size_t nv) const;
};
struct WheelieTruth {
    double time_s = 0., front_load_n = 0., rear_load_n = 0.;
    double front_clearance_m = 0., rear_clearance_m = 0., relative_pitch_rad = 0.;
    double position_m = 0., speed_mps = 0., pitch_up_rad = 0., pitch_rate_up_rad_s = 0.;
    double axle_pitch_rad = 0., road_pitch_rad = 0., front_slip_mps = 0., rear_slip_mps = 0.;
    double com_x_m = 0., com_z_m = 0.;
    [[nodiscard]] WireObject as_wire() const;
};
[[nodiscard]] WheelieTruth research_truth(const PhysicalSampleData &sample, const TruthGeometry &geometry);
struct WheelieEpisode {
    double start_s = 0., end_s = 0.;
    bool confirmed = false;
    double max_relative_pitch_rad = 0., max_front_clearance_m = 0., min_front_load_n = 0.;
    WireObject onset;
    [[nodiscard]] WireObject as_wire() const;
};
class WheelieTracker {
public:
    explicit WheelieTracker(double persistence_s);
    void update(const WheelieTruth &truth, double dt, double delivered, std::optional<double> applied);
    [[nodiscard]] const std::string &state() const noexcept { return state_; }
    [[nodiscard]] WireObject metrics() const;
private:
    double persistence_;
    std::optional<double> last_end_;
    double candidate_s_ = 0.;
    bool active_ = false;
    std::string state_ = "uninitialized";
    std::map<std::string, double> state_times_;
    std::optional<WheelieEpisode> episode_;
    std::vector<WheelieEpisode> episodes_;
    double duration_ = 0., candidate_time_ = 0., wheelie_time_ = 0.;
    std::int64_t wheelie_episodes_ = 0, fraction_count_ = 0;
    double front_unloaded_ = 0., front_lift_ = 0., flight_ = 0.;
    double max_clearance_ = 0., max_pitch_ = 0., rear_slip_ = 0.;
    std::optional<double> min_front_load_, min_fraction_;
    double fraction_sum_ = 0., max_pitch_rate_ = 0.;
};
struct ResearchQuality {
    double residual_ratio = 0.;
    bool acceptable = true;
};
[[nodiscard]] ResearchQuality research_quality(const WireObject &energy, double maximum_ratio);
} // namespace runtime
