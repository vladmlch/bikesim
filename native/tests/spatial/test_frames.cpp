#include "support/migration_test.hpp"
#include "support/startup_fixture.hpp"
#include "engine_call.hpp"
#include "spatial/frames.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"

#include <cmath>
#include <numbers>

int main() { return bt::run([] {
    using namespace bike::spatial;
    const Vec3 v{.3,-.8,1.2};
    for(double angle:{-std::numbers::pi/2.,0.,std::numbers::pi/2.}) {
        const Quat q{std::cos(angle/2.),0.,0.,std::sin(angle/2.)};
        const auto r=rotate_body_to_world(q,v);
        const Vec3 expected{std::cos(angle)*v[0]-std::sin(angle)*v[1],std::sin(angle)*v[0]+std::cos(angle)*v[1],v[2]};
        const auto inverse=rotate_world_to_body(q,r);
        for(std::size_t k=0;k<3;++k) {
            bt::require(std::abs(r[k]-expected[k])<=1e-14,"yaw rotation follows world matrix");
            bt::require(std::abs(inverse[k]-v[k])<=1e-14,"rotation inverse");
        }
    }
    constexpr double yaw=.71,roll=.43;
    const auto combined=quat_multiply({std::cos(yaw/2.),0.,0.,std::sin(yaw/2.)},
        {std::cos(roll/2.),std::sin(roll/2.),0.,0.});
    const auto rotated=rotate_body_to_world(combined,v);
    const Vec3 matrix_product{
        std::cos(yaw)*v[0]-std::sin(yaw)*std::cos(roll)*v[1]+std::sin(yaw)*std::sin(roll)*v[2],
        std::sin(yaw)*v[0]+std::cos(yaw)*std::cos(roll)*v[1]-std::cos(yaw)*std::sin(roll)*v[2],
        std::sin(roll)*v[1]+std::cos(roll)*v[2]};
    for(std::size_t k=0;k<3;++k)
        bt::require(std::abs(rotated[k]-matrix_product[k])<=1e-14,"combined roll/yaw rotation matches the authored matrix product");
    const auto q=quat_normalized({.2,.3,.4,.5});const Quat negative{-q[0],-q[1],-q[2],-q[3]};
    const auto positive_rotation=rotate_body_to_world(q,v),negative_rotation=rotate_body_to_world(negative,v);
    for(std::size_t k=0;k<3;++k) bt::require(std::abs(positive_rotation[k]-negative_rotation[k])<=1e-14,"quaternion sign invariant rotation");
    bt::require(geodesic_angle_rad(q,negative)<=1e-14,"quaternion geodesic is sign invariant");
    bt::require(!euler_zyx({std::sqrt(.5),0.,std::sqrt(.5),0.}).valid,"Euler singularity is explicit");
    BodyPose pose;pose.linear_velocity_world_mps={1.,2.,3.};pose.angular_velocity_world_rad_s={0.,0.,2.};
    const auto point=point_velocity_world(pose,{3.,4.,0.});
    bt::require(point==Vec3{-7.,8.,3.},"point velocity includes lever arm");
    bt::require(axis_rate({1.,4.,3.},{1.,1.,3.},{0.,1.,0.})==3.,"relative axis rate");
    Config cfg;cfg.root=RootKind::free6;auto rig=spatial_tests::free_rig(cfg,false);
    const auto adr=static_cast<std::size_t>(rig.topology.root.free_dof_adr);
    auto velocity=model_access::mutable_buffer(rig.data->qvel,rig.model->nv);
    velocity[adr]=.4;velocity[adr+1]=-.2;velocity[adr+2]=.1;
    velocity[adr+3]=.3;velocity[adr+4]=-.7;velocity[adr+5]=.5;
    engine::forward(rig.model,rig.data);
    const auto measured=body_pose(*rig.model,*rig.data,rig.topology.root);
    for(std::size_t k=0;k<3;++k) bt::require(std::abs(measured.angular_velocity_world_rad_s[k]-velocity[adr+3+k])<=1e-12,"engine world angular velocity at identity");
    const auto at_origin=bike::model::point_velocity(*rig.model,*rig.data,rig.topology.root.body_id,measured.position_world_m);
    for(std::size_t k=0;k<3;++k) bt::require(std::abs(at_origin[k]-measured.linear_velocity_world_mps[k])<=1e-12,"body pose velocity is at xpos even with an offset inertial centre");
}); }
