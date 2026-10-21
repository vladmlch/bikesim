#pragma once
#include "common.hpp"
#include "../model/builder.hpp"
#include "../engine_call.hpp"
#include <array>
#include <memory>

namespace bike::validation {
struct RigResult { Value metrics; Value bounds; };
struct DataDeleter { void operator()(mjData* data)const noexcept {if(data)mj_deleteData(data);} };
struct Stand {
    model::ModelOwner model;
    std::unique_ptr<mjData,DataDeleter> data;
    explicit Stand(std::string_view xml):model(model::compile_mjcf(xml)),data(engine::make_data(model.get())){}
};
[[nodiscard]] std::string xml_number(double x);
[[nodiscard]] double vector_dot(std::span<const double> a,std::span<const double> b);
[[nodiscard]] double vector_norm(std::span<const double> a);
[[nodiscard]] int object_id(const mjModel* model,mjtObj kind,const char* name);
[[nodiscard]] Value radial_rig(double load_n,double dt,double duration_s=2.,bool native_reference=false);
[[nodiscard]] Value wheel_inertia_rig(double dt,double duration_s=.5);
[[nodiscard]] Value incline_rig(double dt,double duration_s=2.,double angle_deg=30.);
[[nodiscard]] Value brake_rig(double dt,double duration_s=5.);
[[nodiscard]] Value chain_locked_rig(double dt,double duration_s=1.);
[[nodiscard]] Value chain_geometry_rig();
[[nodiscard]] Value contact_event_rig(double dt);
[[nodiscard]] Value compiled_chain_motion_rig();
[[nodiscard]] double quasistatic_front_load(double mass,double wheelbase,double com_x,double com_h,
    double slope,double acceleration=0.,double gravity=9.81);
[[nodiscard]] double necessary_traction_margin(double mu,double rear_fraction,double grade);
[[nodiscard]] RigResult load_transfer_rig(double dt,double slope,double com_x,double com_h);
[[nodiscard]] RigResult shaft_ratio_rig(double dt,double ratio=34./51.,bool overrun=false);
[[nodiscard]] RigResult distributed_flat_rig(double dt,int stations,double load);
[[nodiscard]] RigResult distributed_step_rig(double dt,int stations,double dx);
[[nodiscard]] RigResult distributed_incline_rig(double dt,int stations=128,double angle_deg=15.);
[[nodiscard]] std::vector<std::array<double,2>> contact_case_vertices(std::string_view name,double dx);
[[nodiscard]] std::pair<Value,Value> distributed_road_trace(double dt,int stations,double dx,
    std::string_view name="step",double duration=.4);
[[nodiscard]] Value contact_matrix(const Value& configuration);
[[nodiscard]] RigResult drive_suspension_rig(double dt,std::string_view mode,double torque,
    double shock_stroke=.015,double duration=.05);
[[nodiscard]] Value drive_suspension_comparison(const Value& configuration,const std::filesystem::path& root);
struct Momentum { std::array<double,3> com{},linear{},angular{}; };
[[nodiscard]] Momentum system_momentum(const mjModel* model,const mjData* data);
[[nodiscard]] RigResult momentum_rig(double dt,bool active,std::string_view transmission="geometric_ideal_mid_drive",
    std::optional<bool> rider_active={});
[[nodiscard]] RigResult airborne_internal_actuation(double dt);
[[nodiscard]] RigResult suspension_cycle(double dt);
} // namespace bike::validation
