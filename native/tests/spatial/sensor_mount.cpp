#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "runtime/spatial_sensors.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {
bike::spatial::Topology binding(const mjModel& model) {
    bike::spatial::Topology topology;
    topology.root.body_id=mj_name2id(&model,mjOBJ_BODY,"frame");
    topology.imu_site_id=mj_name2id(&model,mjOBJ_SITE,"imu_site_spatial");return topology;
}
}
int main(){return bt::run([]{
    using namespace runtime::spatial;
    const Matrix3 identity{1.,0.,0.,0.,1.,0.,0.,0.,1.};const Vector3 vector{2.,-3.,9.81};
    bt::require(rotate_vector(identity,vector)==vector,"identity mount preserves the declared vector");
    const Matrix3 quarter_turn{0.,-1.,0.,1.,0.,0.,0.,0.,1.};
    bt::require(rotate_vector(quarter_turn,vector)==Vector3{3.,2.,9.81},"90 degree rotation preserves orientation");
    auto improper=identity;improper[0]=-1.;bt::require(!is_rotation_matrix(improper),"reflection rejected");
    bt::invalid([&]{(void)rotate_vector(improper,vector);});
    bt::ModelFixture stationary(R"(<mujoco><compiler angle="radian"/><option gravity="0 0 -9.81"/><worldbody><body name="frame"><geom size=".1" mass="1"/><site name="imu_site_spatial" euler="0 1.5707963267948966 0"/></body></worldbody><sensor><accelerometer name="imu_acc_spatial" site="imu_site_spatial"/><gyro name="imu_gyro_spatial" site="imu_site_spatial"/></sensor></mujoco>)");
    bike::spatial::Config config;engine::forward(stationary.model.get(),stationary.data.get());
    const auto mount=bind_imu_mount(*stationary.model,config,binding(*stationary.model));
    const auto gravity=read_imu_truth(*stationary.model,*stationary.data,mount,0.);
    bt::require(std::abs(gravity.specific_force_sensor_mps2[0]+9.81)<=1e-12&&std::abs(gravity.specific_force_sensor_mps2[2])<=1e-12,
        "stationary tilted site measures gravity in sensor frame");
    bt::invalid([&]{(void)read_imu_truth(*stationary.model,*stationary.data,mount,-1.);});
    bt::invalid([&]{(void)read_imu_truth(*stationary.model,*stationary.data,mount,std::numeric_limits<double>::quiet_NaN());});
    auto wrong=mount;wrong.gyro_adr=static_cast<int>(stationary.model->nsensordata);
    bt::invalid([&]{(void)read_imu_truth(*stationary.model,*stationary.data,wrong,0.);});
    wrong=mount;wrong.body_id=-1;
    bt::invalid([&]{(void)read_imu_truth(*stationary.model,*stationary.data,wrong,0.);});
    wrong=mount;wrong.sensor_from_nominal=improper;
    bt::invalid([&]{(void)read_imu_truth(*stationary.model,*stationary.data,wrong,0.);});
    auto misaligned=config;misaligned.imu.axis_misalignment_rad={0.,0.,1.5707963267948966};
    const auto calibrated=bind_imu_mount(*stationary.model,misaligned,binding(*stationary.model));
    const auto rotated_gravity=read_imu_truth(*stationary.model,*stationary.data,calibrated,0.);
    bt::require(std::abs(rotated_gravity.specific_force_sensor_mps2[0])<=1e-12&&
        std::abs(rotated_gravity.specific_force_sensor_mps2[1]+9.81)<=1e-12,
        "configured Rz Ry Rx misalignment rotates the measured site-frame vector once");
    bt::ModelFixture moving(R"(<mujoco><option gravity="0 0 0"/><worldbody><body name="frame"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/><joint axis="0 1 0"/><site name="imu_site_spatial" pos=".3 0 .2"/></body></worldbody><sensor><accelerometer name="imu_acc_spatial" site="imu_site_spatial"/><gyro name="imu_gyro_spatial" site="imu_site_spatial"/><gyro name="sensor_frame_gyro" site="imu_site_spatial"/></sensor></mujoco>)");
    model_access::mutable_buffer(moving.data->qvel,moving.model->nv)[0]=2.;
    model_access::mutable_buffer(moving.data->qfrc_applied,moving.model->nv)[0]=3.;
    engine::forward(moving.model.get(),moving.data.get());
    const auto rotating=bind_imu_mount(*moving.model,config,binding(*moving.model));
    const auto truth=read_imu_truth(*moving.model,*moving.data,rotating,0.);
    bt::require(std::abs(truth.specific_force_sensor_mps2[0]+.6)<=1e-9&&std::abs(truth.specific_force_sensor_mps2[2]+1.7)<=1e-9,
        "offset includes alpha cross r and omega cross omega cross r exactly once");
    const auto count=engine::state_size(moving.model.get(),mjSTATE_INTEGRATION);std::vector<double> before(static_cast<std::size_t>(count)),after(before.size());
    engine::get_state(moving.model.get(),moving.data.get(),before.data(),mjSTATE_INTEGRATION);
    for(int i=0;i<5;++i)(void)read_imu_truth(*moving.model,*moving.data,rotating,0.);
    engine::get_state(moving.model.get(),moving.data.get(),after.data(),mjSTATE_INTEGRATION);
    bt::require(before==after,"repeated sensor reads do not advance engine state");
    const int legacy=mj_name2id(moving.model.get(),mjOBJ_SENSOR,"sensor_frame_gyro");
    const auto addresses=model_access::readonly_buffer(moving.model->sensor_adr,moving.model->nsensor);
    const auto sensors=model_access::readonly_buffer(moving.data->sensordata,moving.model->nsensordata);
    const double pitch_up=-sensors[static_cast<std::size_t>(addresses[static_cast<std::size_t>(legacy)])+1];
    bt::require(std::bit_cast<std::uint64_t>(pitch_up)==std::bit_cast<std::uint64_t>(-truth.angular_rate_sensor_rad_s[1]),
        "V1 pitch-up is bitwise negative V2 gyro Y at the same site/stage");
});}
