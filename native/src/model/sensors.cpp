#include "builder_internal.hpp"
#include <array>

namespace bike::model::detail {
void build_sensors(Element& root,const BuildContext& c) {
    auto& sensors=root.add("sensor");
    struct Sensor { const char* tag; const char* name; const char* joint; };
    constexpr auto joint_sensors = std::to_array<Sensor>({
        {.tag="jointpos",.name="sensor_shock_stroke",.joint="shock_stroke"},
        {.tag="jointvel",.name="sensor_shock_velocity",.joint="shock_stroke"},
        {.tag="jointpos",.name="sensor_fork_travel",.joint="fork_travel"},
        {.tag="jointvel",.name="sensor_fork_velocity",.joint="fork_travel"},
        {.tag="jointpos",.name="sensor_steer_angle",.joint="steer_joint"},
        {.tag="jointvel",.name="sensor_steer_velocity",.joint="steer_joint"},
        {.tag="jointpos",.name="sensor_chainstay_angle",.joint="main_pivot"},
        {.tag="jointpos",.name="sensor_horst_angle",.joint="horst_pivot"},
        {.tag="jointpos",.name="sensor_rocker_angle",.joint="rocker_frame_pivot"},
        {.tag="jointpos",.name="sensor_yoke_angle",.joint="yoke_pivot"},
        {.tag="jointvel",.name="sensor_front_wheel_speed",.joint="front_wheel_spin"},
        {.tag="jointvel",.name="sensor_rear_wheel_speed",.joint="rear_wheel_spin"},
    });
    for(const auto& item:joint_sensors) sensors.add(item.tag,{{"name",item.name},{"joint",item.joint}});
    sensors.add("accelerometer",{{"name","sensor_frame_accel"},{"site","site_BB"}});
    sensors.add("gyro",{{"name","sensor_frame_gyro"},{"site","site_BB"}});
    sensors.add("framepos",{{"name","sensor_rear_axle_pos"},{"objtype","site"},{"objname","site_PRA"}});
    sensors.add("framepos",{{"name","sensor_front_axle_pos"},{"objtype","site"},{"objname","site_PFA"}});
    sensors.add("subtreecom",{{"name","sensor_subtree_com"},{"body","frame"}});
    if(c.request.mode=="ride") {
        sensors.add("accelerometer",{{"name","sensor_bar_accel"},{"site","site_handlebar"}});
        sensors.add("accelerometer",{{"name","sensor_saddle_accel"},{"site","site_seatpost_top"}});
    }
    if(c.rider.variant=="seated") {
        sensors.add("accelerometer",{{"name","sensor_rider_torso_accel"},{"site","site_rider_torso"}});
        sensors.add("accelerometer",{{"name","sensor_rider_pelvis_accel"},{"site","site_rider_pelvis"}});
    }
}
} // namespace bike::model::detail
