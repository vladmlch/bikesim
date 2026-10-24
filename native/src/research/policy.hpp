#pragma once
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include "../io/value.hpp"
#include "../runtime/sensors.hpp"
#include "../runtime/spatial_sensor_pipeline.hpp"
#include "../../include/bike/policy_v1.h"
#include "../../include/bike/policy_v2.h"

namespace bike::research {
enum class PolicyAbi : std::uint8_t { v1, v2 };

class PolicyFailure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
    ~PolicyFailure() override;
    PolicyFailure(const PolicyFailure&) = default;
    PolicyFailure& operator=(const PolicyFailure&) = default;
    PolicyFailure(PolicyFailure&&) = default;
    PolicyFailure& operator=(PolicyFailure&&) = default;
};

class Policy {
public:
    virtual ~Policy() = default;
    Policy() = default;
    Policy(const Policy&) = delete;
    Policy& operator=(const Policy&) = delete;
    Policy(Policy&&) = delete;
    Policy& operator=(Policy&&) = delete;
    virtual void reset(std::string_view seed_decimal) = 0;
    [[nodiscard]] virtual runtime::RideControl act(const runtime::SensorObservation&, std::optional<double> demand_nm) = 0;
    [[nodiscard]] virtual runtime::RideControl act_v2(const runtime::spatial::SensorObservationV2&, std::optional<double> demand_nm);
    [[nodiscard]] virtual io::Value metadata() const;
    virtual void close() {}
};

struct TractionControlConfig {
    double front_radius_m = .372, rear_radius_m = .352;
    double slip_target_mps = .2, cut_gain_nm_s_per_mps = 200.;
    double recover_nm_s = 25., min_limit_nm = 8.;
    double standstill_wheel_rad_s = .3, standstill_release_s = .8;
    void validate() const;
};

class TractionControlPolicy final : public Policy {
public:
    explicit TractionControlPolicy(TractionControlConfig config = {}, std::unique_ptr<Policy> base = {});
    ~TractionControlPolicy() override;
    TractionControlPolicy(const TractionControlPolicy&) = delete;
    TractionControlPolicy& operator=(const TractionControlPolicy&) = delete;
    TractionControlPolicy(TractionControlPolicy&&) = delete;
    TractionControlPolicy& operator=(TractionControlPolicy&&) = delete;
    void reset(std::string_view seed_decimal) override;
    [[nodiscard]] runtime::RideControl act(const runtime::SensorObservation&, std::optional<double>) override;
    [[nodiscard]] runtime::RideControl act_v2(const runtime::spatial::SensorObservationV2&, std::optional<double>) override;
    [[nodiscard]] io::Value metadata() const override;
    void close() override;
    [[nodiscard]] std::optional<double> limit_nm() const noexcept { return cap_nm_; }
private:
    TractionControlConfig config_;
    std::unique_ptr<Policy> base_;
    std::optional<double> cap_nm_, last_time_;
    double standstill_s_ = 0.;
};

[[nodiscard]] std::unique_ptr<Policy> load_policy(std::string_view specification);
[[nodiscard]] std::unique_ptr<Policy> load_policy(std::string_view specification, const io::Value& config);
[[nodiscard]] std::unique_ptr<Policy> load_policy(std::string_view specification, PolicyAbi);
[[nodiscard]] std::unique_ptr<Policy> load_policy(std::string_view specification, const io::Value& config, PolicyAbi);
[[nodiscard]] std::unique_ptr<Policy> make_builtin_policy(std::string_view specification, const io::Value& config, PolicyAbi = PolicyAbi::v1);
[[nodiscard]] std::unique_ptr<Policy> load_plugin_policy(std::string_view filename, const io::Value& config);
[[nodiscard]] std::unique_ptr<Policy> load_plugin_policy_v2(std::string_view filename, const io::Value& config);
void validate_policy_input(const runtime::SensorObservation&, std::optional<double> demand_nm);
void validate_policy_input(const runtime::spatial::SensorObservationV2&, std::optional<double> demand_nm);
void validate_motor_policy_control(const runtime::RideControl&);
// Shared loader seam permits ABI negative controls without loading a test library.
void validate_policy_abi(const BikePolicyV1*);
[[nodiscard]] runtime::RideControl policy_control_from_abi(const BikeControlV1&);
[[nodiscard]] BikeObservationV1 policy_observation_to_abi(const runtime::SensorObservation&);
void validate_policy_abi_v2(const BikePolicyV2*);
[[nodiscard]] BikeObservationV2 policy_observation_to_abi_v2(const runtime::spatial::SensorObservationV2&);

} // namespace bike::research
