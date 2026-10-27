#include "policy.hpp"
#include "environment.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <cmath>
#include <ranges>

namespace bike::research {
namespace {
using namespace runtime::setupvalue;
class SimplePolicy final : public Policy {
public:
    ~SimplePolicy() override;
    SimplePolicy(const SimplePolicy&) = delete;
    SimplePolicy& operator=(const SimplePolicy&) = delete;
    SimplePolicy(SimplePolicy&&) = delete;
    SimplePolicy& operator=(SimplePolicy&&) = delete;
    SimplePolicy(std::string name, bool zero, bool limit, bool zero_missing)
        : name_(std::move(name)), zero_(zero), limit_(limit), zero_missing_(zero_missing) {}
    void reset(std::string_view seed) override { (void)validate_seed(seed); }
    runtime::RideControl act(const runtime::SensorObservation& observation, std::optional<double> demand) override {
        validate_policy_input(observation, demand);
        runtime::RideControl control;
        control.motor_torque_nm = zero_ ? std::optional(0.) : zero_missing_ ? std::optional(demand.value_or(0.)) : demand;
        if (limit_) control.motor_limit_nm = 40.;
        return control;
    }
    runtime::RideControl act_v2(const runtime::spatial::SensorObservationV2& observation,std::optional<double> demand) override {
        validate_policy_input(observation,demand);runtime::RideControl control;
        control.motor_torque_nm=zero_?std::optional(0.):zero_missing_?std::optional(demand.value_or(0.)):demand;
        if(limit_)control.motor_limit_nm=40.;
        return control;
    }
    io::Value metadata() const override {
        return Value{Object{{"reference", text(name_)}, {"class", text("native::" + name_)},
            {"kind", text("builtin")}, {"missing_demand", text(zero_missing_ ? "zero" : "automatic_assistance")}}};
    }
private:
    std::string name_;
    bool zero_, limit_, zero_missing_;
};
SimplePolicy::~SimplePolicy() = default;
Value traction_config_value(const TractionControlConfig& c) {
    return Value{Object{{"front_radius_m", real(c.front_radius_m)}, {"rear_radius_m", real(c.rear_radius_m)},
        {"slip_target_mps", real(c.slip_target_mps)}, {"cut_gain_nm_s_per_mps", real(c.cut_gain_nm_s_per_mps)},
        {"recover_nm_s", real(c.recover_nm_s)}, {"min_limit_nm", real(c.min_limit_nm)},
        {"standstill_wheel_rad_s", real(c.standstill_wheel_rad_s)}, {"standstill_release_s", real(c.standstill_release_s)}}};
}
} // namespace

void TractionControlConfig::validate() const {
    for (const double value : {front_radius_m, rear_radius_m, cut_gain_nm_s_per_mps, recover_nm_s})
        if (!std::isfinite(value) || value <= 0.) throw std::invalid_argument("traction radii/cut/recovery must be positive and finite");
    for (const double value : {slip_target_mps, min_limit_nm, standstill_wheel_rad_s, standstill_release_s})
        if (!std::isfinite(value) || value < 0.) throw std::invalid_argument("traction thresholds must be nonnegative and finite");
}
TractionControlPolicy::TractionControlPolicy(TractionControlConfig config, std::unique_ptr<Policy> base)
    : config_(config), base_(std::move(base)) { config_.validate(); }
TractionControlPolicy::~TractionControlPolicy() = default;
void TractionControlPolicy::reset(std::string_view seed) {
    (void)validate_seed(seed);
    if (base_) base_->reset(seed);
    cap_nm_.reset(); last_time_.reset(); standstill_s_ = 0.;
}
runtime::RideControl TractionControlPolicy::act(const runtime::SensorObservation& observation, std::optional<double> demand) {
    validate_policy_input(observation, demand);
    runtime::RideControl control;
    if (base_) control = base_->act(observation, demand);
    else control.motor_torque_nm = demand;
    runtime::control_validate_for(control, true);
    const double elapsed = last_time_ ? std::max(0., observation.time_s - *last_time_) : 0.;
    last_time_ = observation.time_s;
    if (!observation.valid) return control;
    const double slip = config_.rear_radius_m * observation.values[5] - config_.front_radius_m * observation.values[4];
    const double ceiling = std::max(0., control.motor_limit_nm.value_or(control.motor_torque_nm.value_or(observation.values[7])));
    auto cap = cap_nm_;
    if (!cap) { if (slip > config_.slip_target_mps) cap = ceiling; }
    else cap = std::min(*cap, ceiling);
    if (cap) {
        if (slip > config_.slip_target_mps) *cap -= config_.cut_gain_nm_s_per_mps * (slip - config_.slip_target_mps) * elapsed;
        else *cap += config_.recover_nm_s * elapsed;
        *cap = std::min(std::max(*cap, std::min(config_.min_limit_nm, ceiling)), ceiling);
        if (*cap >= ceiling) cap.reset();
        cap_nm_ = cap;
        if (cap) control.motor_limit_nm = cap;
    }
    const bool stopped = std::abs(observation.values[5]) < config_.standstill_wheel_rad_s;
    if (stopped && control.motor_torque_nm) standstill_s_ += elapsed;
    else if (!stopped) standstill_s_ = 0.;
    if (stopped && standstill_s_ >= config_.standstill_release_s) control.motor_torque_nm.reset();
    runtime::control_validate_for(control, true);
    return control;
}
io::Value TractionControlPolicy::metadata() const {
    auto config = traction_config_value(config_);
    if (base_) set(config, "base", base_->metadata());
    const auto bytes = io::canonical_json(config);
    return Value{Object{{"reference", text("builtin:traction_control")}, {"class", text("native::TractionControlPolicy")},
        {"kind", text("builtin")}, {"configuration", config},
        {"configuration_sha256", text(io::sha256(std::as_bytes(std::span{bytes.data(), bytes.size()})))}}};
}
runtime::RideControl TractionControlPolicy::act_v2(const runtime::spatial::SensorObservationV2& observation,std::optional<double> demand) {
    validate_policy_input(observation,demand);runtime::RideControl control;
    if(base_)control=base_->act_v2(observation,demand);else control.motor_torque_nm=demand;
    runtime::control_validate_for(control,true);
    const double elapsed=last_time_?std::max(0.,observation.time_s-*last_time_):0.;last_time_=observation.time_s;
    constexpr std::uint16_t required=static_cast<std::uint16_t>((1U<<6U)|(1U<<7U)|(1U<<9U));
    if(!observation.valid||(observation.valid_mask&required)!=required)return control;
    const double slip=config_.rear_radius_m*observation.values[7]-config_.front_radius_m*observation.values[6];
    const double ceiling=std::max(0.,control.motor_limit_nm.value_or(control.motor_torque_nm.value_or(observation.values[9])));
    auto cap=cap_nm_;if(!cap){if(slip>config_.slip_target_mps)cap=ceiling;}else cap=std::min(*cap,ceiling);
    if(cap){if(slip>config_.slip_target_mps)*cap-=config_.cut_gain_nm_s_per_mps*(slip-config_.slip_target_mps)*elapsed;
        else *cap+=config_.recover_nm_s*elapsed;
        *cap=std::min(std::max(*cap,std::min(config_.min_limit_nm,ceiling)),ceiling);if(*cap>=ceiling)cap.reset();
        cap_nm_=cap;if(cap)control.motor_limit_nm=cap;}
    const bool stopped=std::abs(observation.values[7])<config_.standstill_wheel_rad_s;
    if(stopped&&control.motor_torque_nm)standstill_s_+=elapsed;else if(!stopped)standstill_s_=0.;
    if(stopped&&standstill_s_>=config_.standstill_release_s)control.motor_torque_nm.reset();
    runtime::control_validate_for(control,true);return control;
}
void TractionControlPolicy::close() { if (base_) base_->close(); }

std::unique_ptr<Policy> make_builtin_policy(std::string_view name, const io::Value& config, PolicyAbi abi) {
    constexpr std::string_view module = "bike_sim.sim.research.policies:";
    const bool factory = name.starts_with(module);
    const std::string reference(name);
    if (factory) name.remove_prefix(module.size());
    // The T01 inventory also records these unambiguous no-argument class
    // constructors, accepted by the historical module:factory loader.
    if (factory && name == "PassthroughPolicy") name = "passthrough_factory";
    else if (factory && name == "ZeroPolicy") name = "zero_factory";
    else if (factory && name == "FixedLimitPolicy") name = "fixed_limit_40_factory";
    else if (factory && name == "TractionControlPolicy") name = "traction_control_factory";
    const bool explicit_builtin = name.starts_with("builtin:");
    if (explicit_builtin) name.remove_prefix(8);
    const bool zero_missing = !factory; // historical batch callable semantics
    if ((!factory && (name == "passthrough" || name == "zero" || name == "fixed_limit_40")) ||
        (factory && (name == "passthrough_factory" || name == "zero_factory" || name == "fixed_limit_40_factory"))) {
        if (!io::as_object(config, "policy configuration").empty())
            throw std::invalid_argument("this builtin policy accepts no configuration fields");
        return std::make_unique<SimplePolicy>(reference, name == "zero" || name == "zero_factory",
            name == "fixed_limit_40" || name == "fixed_limit_40_factory", zero_missing);
    }
    if (!((!factory && name == "traction_control") || (factory && name == "traction_control_factory"))) return {};
    TractionControlConfig out;
    const auto allowed = {"front_radius_m", "rear_radius_m", "slip_target_mps", "cut_gain_nm_s_per_mps",
        "recover_nm_s", "min_limit_nm", "standstill_wheel_rad_s", "standstill_release_s", "base", "base_config"};
    for (const auto& [key, ignored] : io::as_object(config, "traction configuration")) {
        (void)ignored;
        if (std::ranges::find(allowed, key) == allowed.end()) throw std::invalid_argument("unknown traction configuration field: " + key);
    }
    const auto assign = [&](std::string_view key, double& value) { if (const auto* x = find(config, key)) value = io::as_double(*x, key); };
    assign("front_radius_m", out.front_radius_m); assign("rear_radius_m", out.rear_radius_m);
    assign("slip_target_mps", out.slip_target_mps); assign("cut_gain_nm_s_per_mps", out.cut_gain_nm_s_per_mps);
    assign("recover_nm_s", out.recover_nm_s); assign("min_limit_nm", out.min_limit_nm);
    assign("standstill_wheel_rad_s", out.standstill_wheel_rad_s); assign("standstill_release_s", out.standstill_release_s);
    std::unique_ptr<Policy> base;
    if (const auto* spec = find(config, "base"))
        base = load_policy(io::as_string(*spec, "traction base"), find(config, "base_config") ? field(config, "base_config") : Value{Object{}},abi);
    else if (find(config, "base_config")) throw std::invalid_argument("base_config requires a base policy");
    return std::make_unique<TractionControlPolicy>(out, std::move(base));
}
} // namespace bike::research
