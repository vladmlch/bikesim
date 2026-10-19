// Real dynamic-library negative controls plus source-derived policy traces.
// Authored without compiling or loading any plugin in this session.
#include "support/migration_test.hpp"
#include "research/policy.hpp"
#include "io/json.hpp"
#include <cmath>
#include <filesystem>
#include <limits>
#include <ranges>
#include <type_traits>

namespace {
using namespace bike::research;
template<class F> void policy_failure(F&& call, std::string_view fragment) {
    bool rejected = false;
    try { std::forward<F>(call)(); }
    catch (const PolicyFailure& error) { rejected = std::string(error.what()).find(fragment) != std::string::npos; }
    bt::require(rejected, std::string("expected controlled policy failure: ") + std::string(fragment));
}
void builtin_trace() {
    runtime::SensorObservation observation;
    auto zero = load_policy("builtin:zero"); zero->reset("999999999999999999999999999999");
    bt::require(zero->act(observation, 80.).motor_torque_nm == 0., "zero policy preserves explicit zero");
    auto bare = load_policy("passthrough");
    auto factory = load_policy("bike_sim.sim.research.policies:passthrough_factory");
    bt::require(bare->act(observation, std::nullopt).motor_torque_nm == 0. &&
        !factory->act(observation, std::nullopt).motor_torque_nm, "historical callable/factory missing-demand distinction");
    auto fixed = load_policy("bike_sim.sim.research.policies:FixedLimitPolicy");
    const auto fixed_command = fixed->act(observation, std::nullopt);
    bt::require(fixed_command.motor_limit_nm == 40. && !fixed_command.motor_torque_nm, "fixed-limit class alias from T01 inventory");
    for (const auto class_name : {"PassthroughPolicy", "ZeroPolicy", "FixedLimitPolicy", "TractionControlPolicy"})
        bt::require(load_policy(std::string("bike_sim.sim.research.policies:") + class_name) != nullptr,
            "every T01 policy inventory entry resolves natively");
    bt::invalid([] { (void)load_policy("unknown_module:factory"); });
    bt::invalid([] { (void)load_policy("plugin:relative.so"); });
    bt::invalid([] { (void)load_policy("builtin:zero", bike::io::parse_json("{\"unknown\":1}")); });
    TractionControlConfig parameters;
    parameters.front_radius_m = parameters.rear_radius_m = 1.;
    parameters.slip_target_mps = 1.; parameters.cut_gain_nm_s_per_mps = 2.; parameters.recover_nm_s = 1.;
    TractionControlPolicy traction(parameters);
    traction.reset("2"); observation.values[5] = 3.;
    bt::require(!traction.act(observation, 20.).motor_limit_nm, "first observation has zero elapsed time");
    observation.time_s = .5;
    bt::require(traction.act(observation, 20.).motor_limit_nm == 18., "slip cap integrates its exact excess");
    observation.time_s = 1.; observation.values[4] = observation.values[5] = 1.;
    bt::require(traction.act(observation, 20.).motor_limit_nm == 18.5, "re-grip recovers cap at fixed rate");
    observation.time_s = 1.5; observation.valid = false;
    bt::require(!traction.act(observation, 20.).motor_limit_nm && traction.limit_nm() == 18.5,
        "invalid observation advances time without touching cap");
    observation.time_s = 2.; observation.valid = true;
    bt::require(traction.act(observation, 20.).motor_limit_nm == 19., "recovery excludes invalid elapsed interval");
    observation.time_s = 4.;
    bt::require(!traction.act(observation, 20.).motor_limit_nm && !traction.limit_nm(), "fully recovered cap disengages");
    traction.reset("2"); observation = {};
    (void)traction.act(observation, 20.); observation.time_s = .8;
    bt::require(!traction.act(observation, 20.).motor_torque_nm, "standstill releases explicit request after threshold");
    observation.time_s = .9; observation.values[5] = 1.;
    bt::require(traction.act(observation, 20.).motor_torque_nm == 20., "moving rear wheel restores base command");
    runtime::RideControl rider_owned; rider_owned.human_torque_nm = 0.;
    bt::invalid([&] { validate_motor_policy_control(rider_owned); });
    observation.values[0] = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { (void)zero->act(observation, 0.); });
}
int create_stub(const char*, std::size_t, void**, BikeErrorV1*) { return 0; }
int reset_stub(void*, const char*, std::size_t, BikeErrorV1*) { return 0; }
int act_stub(void*, const BikeObservationV1*, const BikeDemandV1*, BikeControlV1*, BikeErrorV1*) { return 0; }
void destroy_stub(void*) {}
void abi_contract() {
    static_assert(std::is_standard_layout_v<BikeObservationV1> && std::is_standard_layout_v<BikeControlV1>);
    static_assert(BIKE_SENSOR_VALUE_COUNT == 9);
    BikePolicyV1 const good{.abi_version=BIKE_POLICY_ABI_VERSION, .struct_size=sizeof(BikePolicyV1), .required_capabilities=0, .create=create_stub, .reset=reset_stub, .act=act_stub, .destroy=destroy_stub,
        .sensor_schema_version=BIKE_SENSOR_SCHEMA_VERSION};
    validate_policy_abi(&good);
    policy_failure([] { validate_policy_abi(nullptr); }, "null ABI");
    auto invalid = good; invalid.abi_version = 999;
    policy_failure([&] { validate_policy_abi(&invalid); }, "ABI version");
    invalid = good; invalid.struct_size = 8;
    policy_failure([&] { validate_policy_abi(&invalid); }, "too small");
    invalid = good; invalid.required_capabilities = 1;
    policy_failure([&] { validate_policy_abi(&invalid); }, "capabilities");
    invalid = good; invalid.sensor_schema_version = 99;
    policy_failure([&] { validate_policy_abi(&invalid); }, "sensor schema");
    invalid = good; invalid.act = nullptr;
    policy_failure([&] { validate_policy_abi(&invalid); }, "callback");
    BikeControlV1 output{}; output.struct_size = sizeof(output); output.rider_enabled = output.use_saddle = 1;
    output.present_mask = BIKE_CONTROL_MOTOR_TORQUE; output.motor_torque_nm = 0.;
    bt::require(policy_control_from_abi(output).motor_torque_nm == 0., "presence bit distinguishes zero from auto assistance");
    output.motor_torque_nm = std::numeric_limits<double>::quiet_NaN();
    policy_failure([&] { (void)policy_control_from_abi(output); }, "invalid control");
    output.present_mask = 0;
    bt::require(!policy_control_from_abi(output).motor_torque_nm, "absent optional payload is not read");
    output.present_mask = 1U << 31U;
    policy_failure([&] { (void)policy_control_from_abi(output); }, "presence mask");
    output.present_mask = 0; output.rider_enabled = 2;
    policy_failure([&] { (void)policy_control_from_abi(output); }, "nonboolean");
    runtime::SensorObservation sensor; sensor.valid = false;
    const auto observation = policy_observation_to_abi(sensor);
    bt::require(observation.struct_size == sizeof(BikeObservationV1) && observation.sensor_schema_version == 1 &&
        observation.valid == 0 && observation.valid_mask == 0, "ABI transmits only nine sensor channels and explicit validity");
}
void dynamic_boundary(const std::filesystem::path& library, const std::filesystem::path& no_api) {
    const std::string reference = "plugin:" + library.string();
    const auto load = [&](std::string_view scenario) {
        return load_policy(reference, bike::io::parse_json(R"({"case":")" + std::string(scenario) + "\"}"));
    };
    policy_failure([&] { (void)load_policy("plugin:" + no_api.string()); }, "missing bike_policy_v1");
    policy_failure([&] { (void)load_policy("plugin:" + (library.parent_path() / "absent-policy.so").string()); }, "cannot load");
    policy_failure([&] { (void)load("create_fail"); }, "create failed");
    auto reset = load("reset_throw");
    policy_failure([&] { reset->reset("0"); }, "reset threw");
    for (const auto scenario : {"act_throw", "bad_number", "bad_size", "bad_error"}) {
        auto policy = load(scenario); policy->reset("0");
        policy_failure([&] { (void)policy->act(runtime::SensorObservation{}, 0.); }, "policy");
    }
    auto destroy = load("destroy_throw");
    policy_failure([&] { destroy->close(); }, "destroy threw");
    destroy->close(); // pointer was consumed before invoking destroy
    auto valid = load("good"); valid->reset("18446744073709551616000");
    bt::require(valid->act(runtime::SensorObservation{}, 10.).motor_torque_nm == 0., "valid dynamic control");
    const auto metadata = bike::io::canonical_json(valid->metadata());
    bt::require(metadata.find("binary_sha256") != std::string::npos && metadata.find("configuration_sha256") != std::string::npos,
        "plugin binary and configuration identities are retained");
    valid->close();
    policy_failure([&] { (void)valid->act(runtime::SensorObservation{}, 0.); }, "closed");
}
} // namespace
int main(int argc, char** argv) {
    return bt::run([&] {
        bt::require(argc == 3, "T35 requires the callback fixture and missing-symbol fixture paths");
        const auto arguments = std::views::counted(argv, argc);
        builtin_trace(); abi_contract(); dynamic_boundary(arguments[1], arguments[2]);
    });
}
