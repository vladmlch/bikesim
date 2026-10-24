#include "spatial_sensors.hpp"
#include "../model_access.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace runtime::spatial {
bool is_rotation_matrix(const Matrix3& r, double tolerance) noexcept {
    if (!std::isfinite(tolerance) || tolerance < 0.) return false;
    for (double x : r) if (!std::isfinite(x)) return false;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) {
            double dot = 0.;
            for (std::size_t k = 0; k < 3; ++k) dot += r[i * 3 + k] * r[j * 3 + k];
            if (std::abs(dot - (i == j ? 1. : 0.)) > tolerance) return false;
        }
    const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) -
        r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::abs(determinant - 1.) <= tolerance;
}
Vector3 rotate_vector(const Matrix3& r, const Vector3& value) {
    if (!is_rotation_matrix(r) || std::ranges::any_of(value, [](double x) { return !std::isfinite(x); }))
        throw std::invalid_argument("spatial.imu.rotation: finite vector and proper orthonormal matrix required");
    Vector3 out{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) out[i] += r[3*i+j] * value[j];
    if(std::ranges::any_of(out,[](double x){return !std::isfinite(x);}))
        throw std::invalid_argument("spatial.imu.rotation: rotated vector is not finite");
    return out;
}
ImuMountBinding bind_imu_mount(const mjModel& model, const bike::spatial::Config& config,
                              const bike::spatial::Topology& topology) {
    ImuMountBinding out;
    out.site_id = mj_name2id(&model, mjOBJ_SITE, "imu_site_spatial");
    out.accel_sensor_id = mj_name2id(&model, mjOBJ_SENSOR, "imu_acc_spatial");
    out.gyro_sensor_id = mj_name2id(&model, mjOBJ_SENSOR, "imu_gyro_spatial");
    if (out.site_id < 0 || out.site_id != topology.imu_site_id || out.accel_sensor_id < 0 || out.gyro_sensor_id < 0)
        throw std::invalid_argument("spatial.imu: declared site and sensors must exist in the bound topology");
    const auto site_bodies=model_access::readonly_buffer(model.site_bodyid,model.nsite);
    const auto types=model_access::readonly_buffer(model.sensor_type,model.nsensor);
    const auto dimensions=model_access::readonly_buffer(model.sensor_dim,model.nsensor);
    const auto objects=model_access::readonly_buffer(model.sensor_objtype,model.nsensor);
    const auto ids=model_access::readonly_buffer(model.sensor_objid,model.nsensor);
    const auto addresses=model_access::readonly_buffer(model.sensor_adr,model.nsensor);
    out.body_id = site_bodies[static_cast<std::size_t>(out.site_id)];
    if (out.body_id != topology.root.body_id)
        throw std::invalid_argument("spatial.imu.site: mount must belong to the bicycle frame body");
    const auto bind = [&](int id, int type) {
        const auto index=static_cast<std::size_t>(id);
        if (types[index] != type || dimensions[index] != 3 ||
            objects[index] != mjOBJ_SITE || ids[index] != out.site_id ||
            addresses[index] < 0 || addresses[index] > model.nsensordata - 3)
            throw std::invalid_argument("spatial.imu.sensor: incompatible type, width, site or address");
        return addresses[index];
    };
    out.accel_adr = bind(out.accel_sensor_id, mjSENS_ACCELEROMETER);
    out.gyro_adr = bind(out.gyro_sensor_id, mjSENS_GYRO);
    const auto& a = config.imu.axis_misalignment_rad;
    for (double x : a) if (!std::isfinite(x)) throw std::invalid_argument("spatial.imu.axis_misalignment_rad: nonfinite angle");
    const double cx=std::cos(a[0]), sx=std::sin(a[0]), cy=std::cos(a[1]), sy=std::sin(a[1]), cz=std::cos(a[2]), sz=std::sin(a[2]);
    out.sensor_from_nominal = {cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx,
        sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx, -sy, cy*sx, cy*cx};
    if (!is_rotation_matrix(out.sensor_from_nominal))
        throw std::invalid_argument("spatial.imu.axis_misalignment_rad: invalid rotation");
    out.calibration_revision = config.imu.calibration_revision;
    return out;
}
ImuTruth read_imu_truth(const mjModel& model, const mjData& data, const ImuMountBinding& binding, double time) {
    if (!std::isfinite(time) || time < 0. || binding.accel_adr < 0 || binding.gyro_adr < 0 ||
        binding.accel_adr > model.nsensordata - 3 || binding.gyro_adr > model.nsensordata - 3 ||
        binding.site_id<0||binding.site_id>=model.nsite||binding.accel_sensor_id<0||binding.accel_sensor_id>=model.nsensor||
        binding.gyro_sensor_id<0||binding.gyro_sensor_id>=model.nsensor)
        throw std::invalid_argument("spatial.imu.truth: invalid source time or sensor address");
    const auto addresses=model_access::readonly_buffer(model.sensor_adr,model.nsensor);
    const auto dimensions=model_access::readonly_buffer(model.sensor_dim,model.nsensor);
    const auto types=model_access::readonly_buffer(model.sensor_type,model.nsensor);
    const auto ids=model_access::readonly_buffer(model.sensor_objid,model.nsensor);
    const auto objects=model_access::readonly_buffer(model.sensor_objtype,model.nsensor);
    const auto site_bodies=model_access::readonly_buffer(model.site_bodyid,model.nsite);
    const auto a=static_cast<std::size_t>(binding.accel_sensor_id),g=static_cast<std::size_t>(binding.gyro_sensor_id);
    if(addresses[a]!=binding.accel_adr||addresses[g]!=binding.gyro_adr||dimensions[a]!=3||dimensions[g]!=3||
        types[a]!=mjSENS_ACCELEROMETER||types[g]!=mjSENS_GYRO||objects[a]!=mjOBJ_SITE||objects[g]!=mjOBJ_SITE||
        ids[a]!=binding.site_id||ids[g]!=binding.site_id||site_bodies[static_cast<std::size_t>(binding.site_id)]!=binding.body_id)
        throw std::invalid_argument("spatial.imu.truth: binding differs from model sensor descriptors");
    const auto sensors=model_access::readonly_buffer(data.sensordata,model.nsensordata);
    Vector3 acc{}, gyro{};
    for (std::size_t i = 0; i < 3; ++i) {
        acc[i] = sensors[static_cast<std::size_t>(binding.accel_adr)+i];
        gyro[i] = sensors[static_cast<std::size_t>(binding.gyro_adr)+i];
    }
    // MuJoCo's site-frame accelerometer already includes the mount offset's
    // tangential and centripetal acceleration. Do not add them a second time.
    return {.time_s=time, .specific_force_sensor_mps2=rotate_vector(binding.sensor_from_nominal, acc),
        .angular_rate_sensor_rad_s=rotate_vector(binding.sensor_from_nominal, gyro)};
}
} // namespace runtime::spatial
