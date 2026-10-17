#include "builder_internal.hpp"

namespace bike::model::detail {
void build_sensors(Element& root,const BuildContext& c) {
    auto& sensors=root.add("sensor");
    struct Sensor { const char* tag; const char* name; const char* joint; };
    constexpr Sensor joint_sensors[]{
        {"jointpos","sensor_shock_stroke","shock_stroke"},
        {"jointvel","sensor_shock_velocity","shock_stroke"},
        {"jointpos","sensor_fork_travel","fork_travel"},
        {"jointvel","sensor_fork_velocity","fork_travel"},
        {"jointpos","sensor_steer_angle","steer_joint"},
        {"jointvel","sensor_steer_velocity","steer_joint"},
        {"jointpos","sensor_chainstay_angle","main_pivot"},
        {"jointpos","sensor_horst_angle","horst_pivot"},
        {"jointpos","sensor_rocker_angle","rocker_frame_pivot"},
        {"jointpos","sensor_yoke_angle","yoke_pivot"},
        {"jointvel","sensor_front_wheel_speed","front_wheel_spin"},
        {"jointvel","sensor_rear_wheel_speed","rear_wheel_spin"},
    };
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
