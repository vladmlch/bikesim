#include "support/migration_test.hpp"
#include "support/startup_fixture.hpp"
#include "drivetrain/chain.hpp"
#include "engine_call.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"
#include "spatial/frames.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <numeric>
#include <random>
#include <span>
#include <vector>

namespace {
using Vec3=bike::spatial::Vec3;
double dot(const Vec3& a,const Vec3& b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
Vec3 cross(const Vec3& a,const Vec3& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
void pose(spatial_tests::RigModel& rig,double yaw,double roll,const Vec3& translation) {
    auto q=model_access::mutable_buffer(rig.data->qpos,rig.model->nq);
    const auto adr=static_cast<std::size_t>(rig.topology.root.free_qpos_adr);
    for(std::size_t k=0;k<3;++k) q[adr+k]=translation[k];
    // Independent analytic product Rz(yaw)*Rx(roll), wxyz.
    q[adr+3]=std::cos(yaw/2.)*std::cos(roll/2.);
    q[adr+4]=std::cos(yaw/2.)*std::sin(roll/2.);
    q[adr+5]=std::sin(yaw/2.)*std::sin(roll/2.);
    q[adr+6]=std::sin(yaw/2.)*std::cos(roll/2.);
    engine::forward(rig.model,rig.data);
}
std::array<double,6> object_velocity(const spatial_tests::RigModel& rig,int body) {
    std::array<double,6> result{};
    mj_objectVelocity(rig.model,rig.data,mjOBJ_XBODY,body,result.data(),0);
    return result;
}
void virtual_work(spatial_tests::RigModel& rig) {
    const int body=rig.topology.wheels[0].wheel_body_id;
    const auto positions=model_access::readonly_buffer(rig.data->xpos,3*rig.model->nbody);
    const auto index=3*static_cast<std::size_t>(body);
    const Vec3 offset{.13,-.09,.07};
    const Vec3 point{positions[index]+offset[0],positions[index+1]+offset[1],positions[index+2]+offset[2]};
    const Vec3 force{17.,-23.,41.},moment{-3.,5.,7.};
    const auto generalized=bike::model::map_wrench(*rig.model,*rig.data,body,point,force,moment);
    const auto velocity=object_velocity(rig,body);
    const Vec3 angular{velocity[0],velocity[1],velocity[2]};
    const auto lever=cross(angular,offset);
    const Vec3 point_velocity{velocity[3]+lever[0],velocity[4]+lever[1],velocity[5]+lever[2]};
    const double world_power=dot(force,point_velocity)+dot(moment,angular);
    const auto qvel=model_access::readonly_buffer(rig.data->qvel,rig.model->nv);
    const double generalized_power=std::inner_product(generalized.begin(),generalized.end(),qvel.begin(),0.);
    bt::require(std::abs(generalized_power-world_power)<=1e-10*std::max(1.,std::abs(world_power)),
        "world wrench and generalized force satisfy virtual work");
    const auto jacobian=bike::model::point_jacobian(*rig.model,*rig.data,body,point);
    for(std::size_t col=0;col<qvel.size();++col) {
        double explicit_force=0.;
        for(std::size_t row=0;row<3;++row)
            explicit_force+=jacobian.translation[row*qvel.size()+col]*force[row]+
                jacobian.rotation[row*qvel.size()+col]*moment[row];
        bt::require(std::abs(explicit_force-generalized[col])<=1e-10,"mapping retains all three translation and rotation rows");
    }
}
void internal_torque_pair(spatial_tests::RigModel& rig) {
    const auto& wheel=rig.topology.wheels[1];
    const auto axes=model_access::readonly_buffer(rig.data->xaxis,3*rig.model->njnt);
    const auto at=3*static_cast<std::size_t>(wheel.spin.joint_id);
    const Vec3 axis{axes[at],axes[at+1],axes[at+2]};
    constexpr double torque=11.;
    const Vec3 moment{torque*axis[0],torque*axis[1],torque*axis[2]};
    const Vec3 reaction{-moment[0],-moment[1],-moment[2]};
    auto drive=bike::model::map_wrench(*rig.model,*rig.data,wheel.wheel_body_id,{}, {},moment);
    const auto hub=bike::model::map_wrench(*rig.model,*rig.data,wheel.hub_body_id,{}, {},reaction);
    for(std::size_t k=0;k<drive.size();++k) drive[k]+=hub[k];
    for(std::size_t k=0;k<drive.size();++k)
        bt::require(std::abs(drive[k]-(k==static_cast<std::size_t>(wheel.spin.dof_adr)?torque:0.))<=1e-10,
            "internal motor pair produces only relative scalar torque and zero net root wrench");
    const auto child=object_velocity(rig,wheel.wheel_body_id),parent=object_velocity(rig,wheel.hub_body_id);
    const double rate=bike::spatial::axis_rate({child[0],child[1],child[2]},{parent[0],parent[1],parent[2]},axis);
    const auto qvel=model_access::readonly_buffer(rig.data->qvel,rig.model->nv);
    const double power=std::inner_product(drive.begin(),drive.end(),qvel.begin(),0.);
    bt::require(std::abs(power-torque*rate)<=1e-10*std::max(1.,std::abs(power)),"internal torque power uses wheel minus hub rate");
}
void chain_invariance(spatial_tests::RigModel& rig) {
    const int crank=bike::model::required_id(*rig.model,mjOBJ_BODY,"crank");
    const int rear=rig.topology.wheels[1].wheel_body_id;
    const int frame=rig.topology.root.body_id;
    const drivetrain::GearingConfig gear{.front_teeth=34,.rear_teeth=24,.chain_pitch_m=.0127};
    drivetrain::GeometryWorkspace geometry(rig.model);
    auto q=model_access::mutable_buffer(rig.data->qpos,rig.model->nq);
    q[static_cast<std::size_t>(bike::model::joint_address(*rig.model,"crank_spin").first)]=1.3;
    q[static_cast<std::size_t>(rig.topology.wheels[1].spin.qpos_adr)]=.7;
    pose(rig,0.,0.,{0.,0.,1.});
    const auto original=geometry.evaluate(rig.model,rig.data,gear,crank,rear,frame);
    for(const auto angles:{std::array{std::numbers::pi/2.,0.},std::array{-.8,.37},std::array{2.3,-.61}}) {
        pose(rig,angles[0],angles[1],{3.5,-1.7,2.1});
        const auto rotated=geometry.evaluate(rig.model,rig.data,gear,crank,rear,frame);
        bt::require(std::abs(rotated-original)<1e-12,"rigid rotation and translation preserve internal chain length");
        const auto adr=static_cast<std::size_t>(rig.topology.root.free_dof_adr);
        for(std::size_t k=0;k<6;++k) bt::require(std::abs(geometry.jacobian[adr+k])<1e-12,"internal chain Jacobian annihilates rigid root motion");
    }
    const std::vector<double> centre(q.begin(),q.end());
    const std::vector<double> jacobian=geometry.jacobian;
    const auto velocity=model_access::readonly_buffer(rig.data->qvel,rig.model->nv);
    constexpr double epsilon=1e-6;
    mj_integratePos(rig.model,q.data(),velocity.data(),epsilon);
    engine::forward(rig.model,rig.data);
    const double plus=geometry.evaluate(rig.model,rig.data,gear,crank,rear,frame);
    std::ranges::copy(centre,q.begin());
    mj_integratePos(rig.model,q.data(),velocity.data(),-epsilon);
    engine::forward(rig.model,rig.data);
    const double minus=geometry.evaluate(rig.model,rig.data,gear,crank,rear,frame);
    const double predicted=std::inner_product(jacobian.begin(),jacobian.end(),velocity.begin(),0.);
    bt::require(std::abs((plus-minus)/(2.*epsilon)-predicted)<5e-8,"chain Jacobian differentiates engine velocity coordinates");
    std::ranges::copy(centre,q.begin());engine::forward(rig.model,rig.data);
}
} // namespace

int main() {return bt::run([]{
    auto rig=spatial_tests::mixed_joint_rig();
    std::mt19937_64 random(0x5725);
    std::uniform_real_distribution<double> velocity(-3.,3.);
    auto qvel=model_access::mutable_buffer(rig.data->qvel,rig.model->nv);
    for(const auto angles:{std::array{0.,0.},std::array{std::numbers::pi/2.,0.},std::array{.9,-.45}})
        for(int trial=0;trial<8;++trial) {
            for(auto& value:qvel) value=velocity(random);
            pose(rig,angles[0],angles[1],{.3,-.2,1.1});
            virtual_work(rig);internal_torque_pair(rig);
        }
    chain_invariance(rig);
});}
