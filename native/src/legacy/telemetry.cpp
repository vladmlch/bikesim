#include "playground.hpp"
#include "../runtime/setup_values.hpp"
#include "../numeric_sum.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <ranges>
#include <stdexcept>
namespace bike::legacy {
SimModelHandles SimModelHandles::from_model(const mjModel& m){
    const auto joint=[&](const char* name){return mj_name2id(&m,mjOBJ_JOINT,name);};const auto site=[&](const char* name){return mj_name2id(&m,mjOBJ_SITE,name);};
    return {joint("main_pivot"),joint("shock_stroke"),joint("fork_travel"),site("site_P3_ss"),site("site_P3_rocker"),site("site_BB"),site("site_PFA"),site("site_PRA")};
}
TelemetryProvider::TelemetryProvider(const geometry::BikeSpecs& specs,const kinematics::HorstLinkageSolver& solver,const model::ForkAirSpring& air,const model::BikeSuspensionSystem& suspension)
    :specs_(specs),solver_(solver),air_(air),suspension_(suspension){}
io::Value TelemetryProvider::compute(const mjModel& m,const mjData& d,double target,bool rider,const SimModelHandles& h,std::string_view camera)const {
    using namespace runtime::setupvalue;
    for(int id:{h.main_pivot,h.shock,h.fork})if(id>=m.njnt)throw std::invalid_argument("telemetry joint outside model");
    for(int id:{h.p3_ss,h.p3_rocker,h.bb,h.pfa,h.pra})if(id>=m.nsite)throw std::invalid_argument("telemetry site outside model");
    const auto qa=std::views::counted(m.jnt_qposadr,m.njnt),va=std::views::counted(m.jnt_dofadr,m.njnt);
    const auto q=std::views::counted(d.qpos,m.nq),v=std::views::counted(d.qvel,m.nv);
    const auto joint_q=[&](int id){return id>=0?q[static_cast<std::size_t>(qa[static_cast<std::size_t>(id)])]:0.;};
    const auto joint_v=[&](int id){return id>=0?v[static_cast<std::size_t>(va[static_cast<std::size_t>(id)])]:0.;};
    double rear=0.,shock=0.,leverage=0.;
    if(h.main_pivot>=0){const auto state=solver_.solve_state_from_chainstay_angle(solver_.theta_cs_0()-joint_q(h.main_pivot));rear=std::clamp(state.wheel_travel,0.,specs_.rear_wheel_travel);shock=std::clamp(state.shock_stroke,0.,specs_.shock_stroke);leverage=state.leverage_ratio;}
    else if(h.shock>=0){shock=std::clamp(joint_q(h.shock)*1000.,0.,specs_.shock_stroke);const auto state=solver_.solve_state_from_shock_stroke(shock);rear=state.wheel_travel;leverage=state.leverage_ratio;}
    else {const auto state=solver_.solve_state_from_wheel_travel(target);rear=target;shock=state.shock_stroke;leverage=state.leverage_ratio;}
    const double fork=std::max(0.,joint_q(h.fork)*1000.),vf=joint_v(h.fork),vs=joint_v(h.shock);
    SuspensionController controller(specs_,air_,suspension_);const auto ff=controller.compute_fork_force(fork,vf),sf=controller.compute_shock_force(shock,vs);
    const auto site=[&](int id){std::array<double,3> result{};if(id>=0)std::ranges::copy(std::views::counted(d.site_xpos,3*m.nsite).subspan(static_cast<std::size_t>(3*id),3),result.begin());return result;};
    double residual=0.;if(h.p3_ss>=0&&h.p3_rocker>=0){const auto a=site(h.p3_ss),b=site(h.p3_rocker);const std::array delta{a[0]-b[0],a[1]-b[1],a[2]-b[2]};residual=geometry::detail::np_norm3(delta)*1000.;}
    const int frame=mj_name2id(&m,mjOBJ_BODY,"frame"),root=std::max(0,frame);numeric::PythonSum mass;
    for(double x:std::views::counted(m.body_mass,m.nbody).subspan(static_cast<std::size_t>(root)))mass.add(x);
    const auto com=std::views::counted(d.subtree_com,3*m.nbody).subspan(static_cast<std::size_t>(3*root),3);
    const auto bb=site(h.bb);
    const double front_x=h.pfa>=0?site(h.pfa)[0]:.833,rear_x=h.pra>=0?site(h.pra)[0]:-.448,base=std::max(.5,front_x-rear_x);
    const double front_pct=std::clamp((com[0]-rear_x)/base*100.,0.,100.),rear_pct=100.-front_pct,weight=mass.total()*9.81;
    const auto& fd=suspension_.fork_damper;const auto& sd=suspension_.shock_damper;
    const std::array<std::string_view,5> presets{"BASE","PLUSH","ENDURO","PARK","CUSTOM"};
    return Value{Object{{"speed_kmh",real(0.)},{"speed_mps",real(0.)},{"pos_x_m",real(0.)},{"camera_mode",text(camera=="3d"?"3D Isometric Chase":"2D Side Tracking")},{"camera_mode_id",text(camera=="3d"?"3d":"2d")},{"rider_active",Value{rider}},
        {"rear_travel_mm",real(rear)},{"fork_travel_mm",real(fork)},{"shock_stroke_mm",real(shock)},{"shock_compression_pct",real(std::min(100.,shock/specs_.shock_stroke*100.))},{"leverage_ratio",real(leverage)},
        {"v_fork_mps",real(vf)},{"v_shock_mps",real(vs)},{"fork_force_n",real(ff.total_n)},{"fork_damper_n",real(ff.damper_n)},{"fork_k_n_mm",real(air_.compute_instantaneous_stiffness(fork))},{"shock_force_n",real(sf.total_n)},{"shock_damper_n",real(sf.damper_n)},
        {"fork_air_psi",real(air_.gauge_pressure_psi())},{"fork_tokens",integer(air_.num_tokens())},{"fork_hsc",integer(fd.hsc_clicks)},{"fork_lsc",integer(fd.lsc_clicks)},{"fork_reb",integer(fd.rebound_clicks)},
        {"shock_hsc",integer(sd.hsc_clicks)},{"shock_lsc",integer(sd.lsc_clicks)},{"shock_reb",integer(sd.rebound_clicks)},{"shock_hbo",integer(sd.hbo_clicks)},{"shock_lockout",Value{sd.lockout_firm}},
        {"damper_preset",text(presets.at(static_cast<std::size_t>(suspension_.current_preset)))},{"p3_residual_mm",real(residual)},{"total_mass_kg",real(mass.total())},{"cg_x_mm",real((com[0]-bb[0])*1000.)},{"cg_z_mm",real((com[2]-bb[2])*1000.)},
        {"front_load_pct",real(front_pct)},{"rear_load_pct",real(rear_pct)},{"front_load_n",real(weight*(front_pct/100.))},{"rear_load_n",real(weight*(rear_pct/100.))}}};
}
} // namespace bike::legacy
