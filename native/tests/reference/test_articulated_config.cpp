// Ports tests/reference/test_articulated_config.py — topology
// normalization and gate budgets belong to their configuration.
//
// The frozen dataclass's __post_init__ maps onto detail::post_init on
// the built POD record: the field is assigned like the keyword argument,
// then post_init runs the oracle's normalize+domain gate in place.
// Parametrized pytest cases unroll as the same per-field loops. The
// oracle's 'bolts' non-real input has no statically typed analogue —
// the optional_real decode gate it exercises is covered by the wire
// decoder's own validation suite.
#include "support/fixture_test.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include "rider/intent.hpp"
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace cfg = bike::configuration;
using cfg::ArticulatedConfig;
using cfg::SeatedClimbConfig;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void post_init(ArticulatedConfig &config) {
    cfg::detail::post_init(config);
}

// test_spindle_and_connect_are_valid_attachments
void test_spindle_and_connect_are_valid_attachments() {
    ArticulatedConfig config;
    config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
    config.pedal_attachment = "spindle";
    config.saddle_attachment = "pin";
    config.grip_attachment = "connect";
    post_init(config);
    bt::require(config.pedal_attachment == "spindle",
                "the spindle pedal attachment is valid");
    bt::require(config.grip_attachment == "connect",
                "the connect grip attachment is valid");
}

// test_grip_weld_alias_normalises_to_connect
void test_grip_weld_alias_normalises_to_connect() {
    ArticulatedConfig config;
    config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
    config.grip_attachment = "weld";
    post_init(config);
    bt::require(config.grip_attachment == "connect",
                "'weld' normalises to 'connect' like the oracle's alias");
}

// test_budget_comes_from_config_fields
void test_budget_comes_from_config_fields() {
    ArticulatedConfig config;
    config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
    config.pedal_min_normal_n = 25.;
    config.foot_mu = .8;
    config.saddle_mu = .5;
    config.grip_pull_per_hand_n = 350.;
    config.link_max_gap_m = .004;
    post_init(config);
    const auto budget = config.attachment_budget();
    bt::require(budget.foot_min_normal_n == 25. && budget.foot_mu == .8 &&
                    budget.saddle_mu == .5 && budget.grip_pull_n == 350. &&
                    budget.max_gap_m == .004,
                "the budget carries the configured fields");
    // The flat-pad default attachments keep the unilateral-cone weld
    // fields at their disabled sentinels.
    bt::require(budget.saddle_min_normal_n == 0. &&
                    !budget.foot_max_normal_n.has_value() &&
                    !budget.saddle_max_normal_n.has_value() &&
                    budget.foot_shear_n == 0. &&
                    budget.saddle_shear_n == 0.,
                "flat attachments keep the weld envelope disabled");
}

// test_delivered_power_budgets_fall_back_to_capability_caps
void test_delivered_power_budgets_fall_back_to_capability_caps() {
    // Unset: the monitor verdict reads the capability bound bitwise.
    {
        ArticulatedConfig config;
        config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
        config.active_positive_power_limit_w = 450.;
        post_init(config);
        bt::require(config.delivered_positive_budget_w().has_value() &&
                        *config.delivered_positive_budget_w() == 450.,
                    "the delivered positive budget falls back to the "
                    "capability cap");
        bt::require(config.delivered_joint_budget_w() ==
                        config.joint_power_limit_w,
                    "the delivered joint budget falls back to the joint "
                    "capability limit");
    }
    {
        ArticulatedConfig config;
        config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
        post_init(config);
        bt::require(!config.delivered_positive_budget_w().has_value(),
                    "no capability cap leaves the monitor budget unset");
    }
    // Set: the monitor re-bands the delivered envelope while the command
    // cap — plant physics — is untouched.
    {
        ArticulatedConfig config;
        config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
        config.active_positive_power_limit_w = 450.;
        config.delivered_positive_power_budget_w = 750.;
        config.delivered_joint_power_budget_w = 375.;
        post_init(config);
        bt::require(config.delivered_positive_budget_w().value() == 750., // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    "the declared monitor budget wins");
        bt::require(config.delivered_joint_budget_w() == 375.,
                    "the declared joint monitor budget wins");
        bt::require(*config.active_positive_power_limit_w == 450.,
                    "the command cap stays at the declared bound");
        bt::require(config.joint_power_limit_w == 250.,
                    "the joint capability cap is untouched");
    }
}

// test_delivered_power_budgets_validate_like_other_budgets
void test_delivered_power_budgets_validate_like_other_budgets() {
    for (const double value : {-1., kNan, kInf}) {
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
            config.delivered_positive_power_budget_w = value;
            post_init(config);
        });
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
            config.delivered_joint_power_budget_w = value;
            post_init(config);
        });
    }
}

// test_weld_normal_bounds_must_be_finite_when_set
void test_weld_normal_bounds_must_be_finite_when_set() {
    const std::vector<std::optional<double> ArticulatedConfig::*>
        fields{&ArticulatedConfig::weld_foot_min_normal_n,
               &ArticulatedConfig::weld_foot_max_normal_n,
               &ArticulatedConfig::weld_saddle_min_normal_n,
               &ArticulatedConfig::weld_saddle_max_normal_n};
    for (const auto field : fields)
        for (const double value : {kNan, kInf, -kInf})
            bt::invalid([&] {
                ArticulatedConfig config;
                config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
                config.*field = value;
                post_init(config);
            });
}

// test_weld_normal_bounds_accept_tension_retention
void test_weld_normal_bounds_accept_tension_retention() {
    const std::vector<std::optional<double> ArticulatedConfig::*>
        fields{&ArticulatedConfig::weld_foot_min_normal_n,
               &ArticulatedConfig::weld_foot_max_normal_n,
               &ArticulatedConfig::weld_saddle_min_normal_n,
               &ArticulatedConfig::weld_saddle_max_normal_n};
    for (const auto field : fields) {
        ArticulatedConfig config;
        config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
        config.*field = -1050.;
        post_init(config);
        bt::require(config.*field == -1050.,
                    "a negative finite bound is tension retention, not a "
                    "validation failure");
    }
}

// test_weld_shear_capacity_is_nonnegative
void test_weld_shear_capacity_is_nonnegative() {
    for (const double value : {-1., kNan}) {
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
            config.weld_foot_shear_n = value;
            post_init(config);
        });
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
            config.weld_saddle_shear_n = value;
            post_init(config);
        });
    }
}

// test_balance_defaults
void test_balance_defaults() {
    ArticulatedConfig config;
    config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
    post_init(config);
    bt::require(config.balance_floor_kmh == 4. &&
                    config.balance_dwell_s == .5 &&
                    config.balance_grace_s == 3.,
                "the balance defaults are 4 km/h, .5 s dwell, 3 s grace");
}

// test_invalid_balance_thresholds_are_rejected
void test_invalid_balance_thresholds_are_rejected() {
    for (const double value : {-1., 0., kNan}) {
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
            config.balance_floor_kmh = value;
            post_init(config);
        });
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
            config.balance_dwell_s = value;
            post_init(config);
        });
    }
}

// test_surge_and_trim_are_validated_in_climb_config
void test_surge_and_trim_are_validated_in_climb_config() {
    {
        SeatedClimbConfig config;
        config.surge_budget_s = 15.; config.trim_dead_time_s = .2;
        config.period_s = .00125;
        config.reaction_delay_s = .15;
        config.lean_rate_rad_s = .5;
        config.orientation_tau_s = .5;
        config.min_elbow_flexion_deg = 15.;
        cfg::detail::post_init(config);
        bt::require(config.surge_power_w == 400. &&
                        config.surge_grade == .2 &&
                        config.surge_budget_s == 15. &&
                        config.surge_recovery_rate == 1. / 3.,
                    "the climb defaults are 400 W, grade .2, 15 s, 1/3");
    }
    for (const auto field : {&SeatedClimbConfig::surge_power_w,
                             &SeatedClimbConfig::surge_budget_s})
        bt::invalid([&] {
            SeatedClimbConfig config;
            config.surge_budget_s = 15.; config.trim_dead_time_s = .2;
            config.period_s = .00125;
            config.reaction_delay_s = .15;
            config.lean_rate_rad_s = .5;
            config.orientation_tau_s = .5;
            config.min_elbow_flexion_deg = 15.;
            config.*field = 0.;
            cfg::detail::post_init(config);
        });
    for (const auto field :
         {&SeatedClimbConfig::surge_grade,
          &SeatedClimbConfig::surge_recovery_rate,
          &SeatedClimbConfig::front_load_share_target,
          &SeatedClimbConfig::lean_trim_gain_rad_s,
          &SeatedClimbConfig::lean_trim_limit_rad})
        bt::invalid([&] {
            SeatedClimbConfig config;
            config.surge_budget_s = 15.; config.trim_dead_time_s = .2;
            config.period_s = .00125;
            config.reaction_delay_s = .15;
            config.lean_rate_rad_s = .5;
            config.orientation_tau_s = .5;
            config.min_elbow_flexion_deg = 15.;
            config.*field = -1.;
            cfg::detail::post_init(config);
        });
}

// test_balance_grace_must_be_finite_and_nonnegative
void test_balance_grace_must_be_finite_and_nonnegative() {
    for (const double value : {-1., kNan, kInf})
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = .5; config.balance_grace_s = 3.; config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.; config.leg_active_damping_nms_rad = 35.;
            config.balance_grace_s = value;
            post_init(config);
        });
}

// Launch part 4b: the posture PD gains come from the profile — an unset
// gain is not a silent 600 / 15, it fails validation and names the field,
// and neither 0 nor non-finite values are accepted.
[[gnu::noinline]] void test_posture_gains_are_required() {
    const auto without_gain = [](double ArticulatedConfig::*field) {
        ArticulatedConfig config;
        config.balance_dwell_s = .5;
        config.balance_grace_s = 3.;
        config.leg_active_damping_nms_rad = 35.;
        if (field == &ArticulatedConfig::joint_kp_nm_rad) {
            config.joint_kd_nms_rad = 15.;
        } else {
            config.joint_kp_nm_rad = 600.;
        }
        return config;
    };
    for (const auto &[field, name] : {
             std::pair{&ArticulatedConfig::joint_kp_nm_rad, "joint_kp_nm_rad"},
             std::pair{&ArticulatedConfig::joint_kd_nms_rad, "joint_kd_nms_rad"}}) {
        const auto expect_rejected = [&](ArticulatedConfig config, std::string_view reason) {
            bool rejected = false;
            try {
                post_init(config);
            } catch (const std::invalid_argument &error) {
                rejected = std::string_view(error.what()).find(name) != std::string_view::npos;
            }
            bt::require(rejected, std::string(name) + " " + std::string(reason));
        };
        expect_rejected(without_gain(field), "unset default fails validation and names the field");
        for (const double bad : {-1., 0., kNan, kInf}) {
            ArticulatedConfig config = without_gain(field);
            config.*field = bad;
            expect_rejected(config, "non-positive or non-finite gain fails validation and names the field");
        }
    }
}

// Launch part 4b: the active leg damping comes from the profile and is
// never negative — a negative gain would pump the 18-24 Hz leg mode, while
// zero and positive finite gains are valid and an unset default is rejected.
[[gnu::noinline]] void test_leg_damping_is_required_and_nonnegative() {
    const auto with_other_required = [] {
        ArticulatedConfig config;
        config.balance_dwell_s = .5;
        config.balance_grace_s = 3.;
        config.joint_kp_nm_rad = 600.;
        config.joint_kd_nms_rad = 15.;
        return config;
    };
    const auto expect_rejected = [](ArticulatedConfig config, std::string_view reason) {
        bool rejected = false;
        try {
            post_init(config);
        } catch (const std::invalid_argument &error) {
            rejected = std::string(error.what()).find("leg_active_damping_nms_rad") != std::string::npos;
        }
        bt::require(rejected, reason);
    };
    {
        ArticulatedConfig config = with_other_required();
        config.leg_active_damping_nms_rad = 0.;
        post_init(config);   // zero is a valid gain
        config.leg_active_damping_nms_rad = -1.;
        expect_rejected(config, "a negative leg damping fails validation and names the field");
    }
    expect_rejected(with_other_required(), "an unset leg damping fails validation and names the field");
    for (const double bad : {-0.1, kNan, kInf}) {
        ArticulatedConfig config = with_other_required();
        config.leg_active_damping_nms_rad = bad;
        expect_rejected(config, "an out-of-range or non-finite leg damping fails validation and names the field");
    }
    {
        ArticulatedConfig config = with_other_required();
        config.leg_active_damping_nms_rad = 35.;
        post_init(config);   // nominal profile value is accepted
    }
}

} // namespace

int main() {
    return bt::run([] {
        test_spindle_and_connect_are_valid_attachments();
        test_grip_weld_alias_normalises_to_connect();
        test_budget_comes_from_config_fields();
        test_delivered_power_budgets_fall_back_to_capability_caps();
        test_delivered_power_budgets_validate_like_other_budgets();
        test_weld_normal_bounds_must_be_finite_when_set();
        test_weld_normal_bounds_accept_tension_retention();
        test_weld_shear_capacity_is_nonnegative();
        test_balance_defaults();
        test_invalid_balance_thresholds_are_rejected();
        test_surge_and_trim_are_validated_in_climb_config();
        test_balance_grace_must_be_finite_and_nonnegative();
        test_posture_gains_are_required();
        test_leg_damping_is_required_and_nonnegative();
    });
}
