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
#include "support/migration_test.hpp"
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
    config.grip_attachment = "weld";
    post_init(config);
    bt::require(config.grip_attachment == "connect",
                "'weld' normalises to 'connect' like the oracle's alias");
}

// test_budget_comes_from_config_fields
void test_budget_comes_from_config_fields() {
    ArticulatedConfig config;
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
        post_init(config);
        bt::require(!config.delivered_positive_budget_w().has_value(),
                    "no capability cap leaves the monitor budget unset");
    }
    // Set: the monitor re-bands the delivered envelope while the command
    // cap — plant physics — is untouched.
    {
        ArticulatedConfig config;
        config.active_positive_power_limit_w = 450.;
        config.delivered_positive_power_budget_w = 750.;
        config.delivered_joint_power_budget_w = 375.;
        post_init(config);
        bt::require(*config.delivered_positive_budget_w() == 750.,
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
            config.delivered_positive_power_budget_w = value;
            post_init(config);
        });
        bt::invalid([&] {
            ArticulatedConfig config;
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
            config.weld_foot_shear_n = value;
            post_init(config);
        });
        bt::invalid([&] {
            ArticulatedConfig config;
            config.weld_saddle_shear_n = value;
            post_init(config);
        });
    }
}

// test_balance_defaults
void test_balance_defaults() {
    ArticulatedConfig config;
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
            config.balance_floor_kmh = value;
            post_init(config);
        });
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_dwell_s = value;
            post_init(config);
        });
    }
}

// test_surge_and_trim_are_validated_in_climb_config
void test_surge_and_trim_are_validated_in_climb_config() {
    {
        SeatedClimbConfig config;
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
            config.*field = -1.;
            cfg::detail::post_init(config);
        });
}

// test_balance_grace_must_be_finite_and_nonnegative
void test_balance_grace_must_be_finite_and_nonnegative() {
    for (const double value : {-1., kNan, kInf})
        bt::invalid([&] {
            ArticulatedConfig config;
            config.balance_grace_s = value;
            post_init(config);
        });
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
    });
}
