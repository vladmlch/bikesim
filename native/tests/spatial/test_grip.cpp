#include "support/rider_fixture.hpp"
#include "kinematics/solver.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/spatial_support.hpp"
#include "spatial/frames.hpp"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

namespace {
namespace sp=bike::spatial;
constexpr double radians(double degrees) { return degrees*std::numbers::pi/180.; }
constexpr std::array<const char*,5> rider_equalities{
    "connect_grip_left","connect_grip_right","connect_foot_front",
    "connect_foot_rear","connect_saddle"};

double link_angle(const bike::geometry::Vec3& end,const bike::geometry::Vec3& begin) {
    return std::atan2(end[2]-begin[2],end[0]-begin[0]);
}
void suspension(spatial_tests::RiderRig& rig,const bike::kinematics::HorstLinkageSolver& solver,
                const bike::kinematics::KinematicState& reference,double fraction) {
    const auto current=solver.solve_state_from_shock_stroke(fraction*rig.request->bike.shock_stroke);
    const double chainstay=reference.theta_cs-current.theta_cs;
    const double seatstay=link_angle(reference.P3,reference.P2)-link_angle(current.P3,current.P2);
    const double rocker=link_angle(reference.P3,reference.P5)-link_angle(current.P3,current.P5);
    const double yoke=link_angle(reference.P6,reference.P4)-link_angle(current.P6,current.P4);
    rig.scalar("main_pivot",chainstay);
    rig.scalar("horst_pivot",seatstay-chainstay);
    rig.scalar("rocker_frame_pivot",rocker);
    rig.scalar("yoke_pivot",yoke-rocker);
    rig.scalar("shock_stroke",(current.shock_stroke-reference.shock_stroke)/1000.);
    rig.scalar("fork_travel",fraction*rig.request->bike.fork_travel/1000.);
}

sp::Vec3 site_position(const mjModel& model,const mjData& data,int site) {
    const auto positions=model_access::readonly_buffer(data.site_xpos,3*model.nsite);
    const auto offset=3*static_cast<std::size_t>(site);
    return {positions[offset],positions[offset+1],positions[offset+2]};
}
Eigen::MatrixXd grip_jacobian(const mjModel& model,const mjData& data,
                             bike::rider::GripRigResult& result) {
    Eigen::MatrixXd jacobian(6,model.nv);
    std::vector<double> hand_jac(static_cast<std::size_t>(3*model.nv));
    std::vector<double> bar_jac(hand_jac.size());
    const auto bodies=model_access::readonly_buffer(model.site_bodyid,model.nsite);
    for(std::size_t side=0;side<2;++side) {
        const std::string suffix=side==0?"left":"right";
        const int hand=bike::model::required_id(model,mjOBJ_SITE,"site_rider_grip_"+suffix);
        const int bar=bike::model::required_id(model,mjOBJ_SITE,"site_grip_"+suffix);
        const auto hand_point=site_position(model,data,hand),bar_point=site_position(model,data,bar);
        const double gap=std::hypot(hand_point[0]-bar_point[0],hand_point[1]-bar_point[1],hand_point[2]-bar_point[2]);
        (side==0?result.left_gap_m:result.right_gap_m)=gap;
        mj_jac(&model,&data,hand_jac.data(),nullptr,hand_point.data(),bodies[static_cast<std::size_t>(hand)]);
        mj_jac(&model,&data,bar_jac.data(),nullptr,bar_point.data(),bodies[static_cast<std::size_t>(bar)]);
        for(Eigen::Index row=0;row<3;++row) for(Eigen::Index col=0;col<model.nv;++col) {
            const auto index=static_cast<std::size_t>(row*model.nv+col);
            jacobian(static_cast<Eigen::Index>(3*side)+row,col)=hand_jac[index]-bar_jac[index];
        }
    }
    return jacobian;
}

// Read the current engine arena, including its sparse/dense representation,
// through the same checked engine product used by production. Linkage, feet,
// saddle and both grips all participate in the velocity null space.
Eigen::MatrixXd equality_jacobian(const mjModel& model,const mjData& data,
                                 const ::rider::EqualityReactions& reactions) {
    std::vector<std::int64_t> rows;
    for(const auto& [id,indices]:reactions.equality_rows(&data)) {
        (void)id;rows.insert(rows.end(),indices.begin(),indices.end());
    }
    bt::require(!rows.empty(),"grip rig has active equality rows");
    Eigen::MatrixXd jacobian(static_cast<Eigen::Index>(rows.size()),model.nv);
    std::vector<double> basis(static_cast<std::size_t>(model.nv),0.);
    std::vector<double> product(static_cast<std::size_t>(data.nefc),0.);
    for(Eigen::Index col=0;col<model.nv;++col) {
        basis[static_cast<std::size_t>(col)]=1.;
        engine::mul_jac_vec(&model,&data,product.data(),basis.data());
        basis[static_cast<std::size_t>(col)]=0.;
        for(std::size_t row=0;row<rows.size();++row)
            jacobian(static_cast<Eigen::Index>(row),col)=product[static_cast<std::size_t>(rows[row])];
    }
    return jacobian;
}

struct Measurement {
    bike::rider::GripRigResult result;
    double drive_power_w=0.;
    bool qualified=false;
    std::string reason;
};
Measurement measure(spatial_tests::RiderRig& rig,double lean_degrees) {
    auto& model=*rig.rig.model;auto* data=rig.rig.data.get();
    Measurement out;
    out.result.left_gap_m=out.result.right_gap_m=std::numeric_limits<double>::infinity();
    out.result.internal_power_residual_w=std::numeric_limits<double>::infinity();
    ::rider::RiderPosture posture;posture.torso_lean_rad=radians(lean_degrees);
    try {
        rig.controller->initialize(data,&posture);
    } catch(const std::invalid_argument& error) {
        // Preserve the actual gap and rank for an unreachable seed. A clipped
        // IK target never turns an unqualified row into a successful solve.
        out.reason=error.what();engine::forward(&model,data);
    }
    const auto grips=grip_jacobian(model,*data,out.result);
    Eigen::JacobiSVD<Eigen::MatrixXd> grip_svd(grips);
    grip_svd.setThreshold(1e-12);
    out.result.grip_jacobian_rank=static_cast<int>(grip_svd.rank());
    ::rider::EqualityReactions reactions(&model);
    for(const char* name:rider_equalities) {
        const int id=bike::model::required_id(model,mjOBJ_EQUALITY,name);
        bt::require(reactions.rows_of(data,id).size()==3,"each rider connect contributes exactly three active rows");
    }
    const auto constraints=equality_jacobian(model,*data,reactions);
    Eigen::JacobiSVD<Eigen::MatrixXd> all_svd(constraints,Eigen::ComputeFullV);
    all_svd.setThreshold(1e-12);
    const auto row_basis=all_svd.matrixV().leftCols(all_svd.rank()).eval();
    const int steer=rig.rig.dof("steer_joint"),crank=rig.rig.dof("crank_spin");
    Eigen::VectorXd steering=Eigen::VectorXd::Zero(model.nv);steering[steer]=1.;
    steering-=row_basis*(row_basis.transpose()*steering).eval();
    // A one-column projection has exactly one singular value: its 2-norm.
    out.result.steering_mobility=steering.norm();
    Eigen::VectorXd velocity=Eigen::VectorXd::Zero(model.nv);velocity[steer]=.7;velocity[crank]=1.1;
    velocity-=row_basis*(row_basis.transpose()*velocity).eval();
    const auto qvel=model_access::mutable_buffer(data->qvel,model.nv);
    for(Eigen::Index i=0;i<model.nv;++i) qvel[static_cast<std::size_t>(i)]=velocity[i];
    // Request nonzero crank work without introducing a second steering owner.
    model_access::mutable_buffer(data->qfrc_applied,model.nv)[static_cast<std::size_t>(crank)]=10.;
    engine::forward(&model,data);
    out.drive_power_w=10.*qvel[static_cast<std::size_t>(crank)];
    out.result.internal_power_residual_w=0.;
    std::vector<double> qfrc(static_cast<std::size_t>(model.nv));
    for(const char* name:rider_equalities) {
        reactions.equality_qfrc_into(data,bike::model::required_id(model,mjOBJ_EQUALITY,name),qfrc);
        for(std::size_t i=0;i<qfrc.size();++i) out.result.internal_power_residual_w+=qfrc[i]*qvel[i];
    }
    const auto& r=out.result;
    out.qualified=out.reason.empty() && r.left_gap_m<=.001 && r.right_gap_m<=.001 &&
        r.grip_jacobian_rank==6 && std::isfinite(r.steering_mobility) && r.steering_mobility>1e-12 &&
        std::abs(r.internal_power_residual_w)<=1e-8*std::max(1.,std::abs(out.drive_power_w));
    if(!out.qualified && out.reason.empty()) out.reason="gap/rank/mobility/power acceptance";
    return out;
}

void common_rotation(spatial_tests::RiderRig& rig) {
    auto& model=*rig.rig.model;auto* data=rig.rig.data.get();
    engine::reset_data(&model,data);rig.scalar("steer_joint",radians(15.));
    ::rider::RiderPosture posture;posture.torso_lean_rad=radians(10.);
    rig.controller->initialize(data,&posture);
    bike::rider::GripRigResult before;
    (void)grip_jacobian(model,*data,before);
    constexpr std::array<const char*,5> names{
        "rider_torso_hinge","rider_shoulder_left","rider_shoulder_right","shoulder_swivel_l","shoulder_swivel_r"};
    std::array<double,names.size()> values{};
    auto positions=model_access::mutable_buffer(data->qpos,model.nq);
    for(std::size_t i=0;i<names.size();++i)
        values[i]=positions[static_cast<std::size_t>(bike::model::joint_address(model,names[i]).first)];
    const sp::Quat yaw{std::cos(.37),0.,0.,std::sin(.37)};
    const sp::Quat roll{std::cos(.21),std::sin(.21),0.,0.};
    const auto rotation=sp::quat_multiply(yaw,roll);
    for(const char* name:{"root_free","rider_root_free"}) {
        const auto qa=static_cast<std::size_t>(bike::model::joint_address(model,name).first);
        const auto point=sp::rotate_body_to_world(rotation,{positions[qa],positions[qa+1],positions[qa+2]});
        const auto orientation=sp::quat_multiply(rotation,{positions[qa+3],positions[qa+4],positions[qa+5],positions[qa+6]});
        for(std::size_t i=0;i<3;++i) positions[qa+i]=point[i]+std::array<double,3>{2.,-3.,1.}[i];
        for(std::size_t i=0;i<4;++i) positions[qa+3+i]=orientation[i];
    }
    engine::forward(&model,data);rig.controller->initialize(data,&posture);
    bike::rider::GripRigResult after;(void)grip_jacobian(model,*data,after);
    for(std::size_t i=0;i<names.size();++i)
        bt::require(std::abs(values[i]-positions[static_cast<std::size_t>(bike::model::joint_address(model,names[i]).first)])<1e-10,
            "spatial IK uses body-local geometry under common yaw/roll");
    bt::require(std::abs(before.left_gap_m-after.left_gap_m)<1e-10 && std::abs(before.right_gap_m-after.right_gap_m)<1e-10,
        "world rotation and translation preserve both physical hand gaps");
}

void reachability_grid(spatial_tests::RiderRig& rig) {
    const bike::kinematics::HorstLinkageSolver solver(rig.request->bike);
    const auto reference=solver.solve_state_from_wheel_travel(0.);
    std::size_t rows=0,unqualified=0;
    std::cout<<"steer_deg,crank_deg,lean_deg,suspension_fraction,left_gap_m,right_gap_m,grip_rank,steering_mobility,internal_power_w,drive_power_w,status,reason\n";
    std::cout<<std::setprecision(17);
    for(double steer:{-60.,-30.,-15.,-5.,5.,15.,30.,60.})
        for(double phase:{0.,90.,180.,270.}) for(double lean:{-10.,0.,10.}) for(double fraction:{0.,.5,1.}) {
            engine::reset_data(rig.rig.model.get(),rig.rig.data.get());
            suspension(rig,solver,reference,fraction);
            rig.scalar("steer_joint",radians(steer));rig.scalar("crank_spin",radians(phase));
            rig.scalar("pedal_front_spin",-radians(phase));rig.scalar("pedal_rear_spin",-radians(phase));
            const auto sample=measure(rig,lean);const auto& r=sample.result;
            ++rows;if(!sample.qualified) ++unqualified;
            std::cout<<steer<<','<<phase<<','<<lean<<','<<fraction<<','<<r.left_gap_m<<','<<r.right_gap_m<<','
                <<r.grip_jacobian_rank<<','<<r.steering_mobility<<','<<r.internal_power_residual_w<<','
                <<sample.drive_power_w<<','<<(sample.qualified?"qualified":"not_qualified")<<','<<sample.reason<<'\n';
        }
    bt::require(rows==288,"all steering, crank, lean and suspension combinations were measured");
    // This is the requested +/-60 degree acceptance gate. The profile remains
    // provisional until this future executable passes; no table is fabricated
    // during source-only authoring, and an unreachable row is never skipped.
    bt::require(unqualified==0,"declared +/-60 degree grip range is not qualified; inspect the complete table");
}
} // namespace

int main() { return bt::run([] {
    auto rig=std::make_unique<spatial_tests::RiderRig>();
    reachability_grid(*rig);common_rotation(*rig);
}); }
