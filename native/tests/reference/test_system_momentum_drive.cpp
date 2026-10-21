// Task 3.19: tests/reference/test_system_momentum_drive.py, both slow stands.
#include "support/migration_test.hpp"
#include "validation/rigs.hpp"
#include "io/value.hpp"

#include <string>
#include <string_view>

namespace {
double metric(const bike::validation::RigResult& result, std::string_view name) {
    const auto* value = bike::io::find_member(bike::io::as_object(result.metrics, "momentum metrics"), name);
    bt::require(value != nullptr, "momentum metric exists: " + std::string(name));
    return bike::io::as_double(*value, name);
}
void test_airborne_motor_stand_delivers_work_without_root_actuation() {
    const auto result = bike::validation::momentum_rig(.00125, true, "geometric_ideal_mid_drive", false);
    bt::require(metric(result, "peak_motor_torque_nm") > 10., "active stand delivers actual motor torque");
    bt::require(metric(result, "motor_shaft_work_j") > 0., "active motor does positive shaft work");
    bt::require(metric(result, "joint_positive_work_j") == 0., "disabled rider produces no joint work");
    for (auto name : {"impulse_residual_ratio", "angular_residual_ratio", "shaft_power_identity_error_w",
                      "root_actuator_force_n", "ground_contact_count"}) {
        const auto* value = bike::io::find_member(bike::io::as_object(result.bounds, "momentum bounds"), name);
        bt::require(value != nullptr, "momentum bound exists: " + std::string(name));
        const auto& limits = bike::io::as_array(*value, name);
        bt::require(limits.size() == 2, "momentum bound has two endpoints");
        const auto measured = metric(result, name);
        bt::require(bike::io::as_double(limits[0], name) <= measured && measured <= bike::io::as_double(limits[1], name),
                    "momentum stand bound: " + std::string(name));
    }
}
void test_passive_airborne_stand_has_no_motor_or_joint_work() {
    const auto result = bike::validation::momentum_rig(.00125, false);
    for (auto name : {"peak_motor_torque_nm", "motor_shaft_work_j", "joint_positive_work_j"})
        bt::require(metric(result, name) == 0., "passive stand has no motor or joint work: " + std::string(name));
}
} // namespace
int main() { return bt::run([] {
    test_airborne_motor_stand_delivers_work_without_root_actuation();
    test_passive_airborne_stand_has_no_motor_or_joint_work();
}); }
