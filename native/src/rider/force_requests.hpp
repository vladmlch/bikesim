#pragma once
#include <array>

namespace bike::rider {
using Force3=std::array<double,3>;
struct PedalingRequests {std::array<Force3,2> force{};std::array<double,2> weights{};};
[[nodiscard]] Force3 stance_force(double phase,double mean_nm,double crank_m);
[[nodiscard]] Force3 feasible_pedal_force_request(const Force3&,const Force3& normal,double mu,double measured_load);
[[nodiscard]] double pedal_torque_waveform(double mean_nm,double phase_rad,double ripple);
[[nodiscard]] PedalingRequests pedaling_force_requests(double phase,double mean_nm,double crank_m,
    const std::array<Force3,2>& normals,const std::array<double,2>& loads,double mu,double ripple=.35,
    double return_foot_preload_n=0.,const std::array<bool,2>& preload_sides={true,true});
} // namespace bike::rider
