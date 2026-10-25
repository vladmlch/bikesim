// Completion 3.14b (T52). Source: tests/reference/test_attachment_budget.py.
// attachment_budget.py's violation classifier replayed against the literal
// oracle expectations — the file is fully analytic (every expected tuple
// is a source literal), so it captures no golden entries and the port is
// the authoritative gate from the start.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "configuration/physical_config.hpp"
#include "diag.hpp"
#include "support/migration_test.hpp"
#include "rider/attachment_wrench.hpp"
#include "runtime/period_buffer.hpp"

namespace {
using bike::configuration::ArticulatedConfig;
using rider::AttachmentSample;
using runtime::AttachmentBudget;
using runtime::attachment_violations;
using Violations = std::vector<std::string>;

// BUDGET = ArticulatedConfig().attachment_budget()
AttachmentBudget budget() {
    const auto b = ArticulatedConfig{}.attachment_budget();
    return {.foot_min_normal_n = b.foot_min_normal_n, .foot_mu = b.foot_mu,
            .saddle_mu = b.saddle_mu, .grip_pull_n = b.grip_pull_n,
            .max_gap_m = b.max_gap_m};
}
const AttachmentBudget kBudget = budget();

bool has(const Violations &v, std::string_view name) {
    return std::find(v.begin(), v.end(), name) != v.end();
}

void test_violations_use_the_supplied_budget() {
    const AttachmentSample sample{.kind = "foot", .normal_n = 19.};
    bt::require(has(attachment_violations(
                        sample, AttachmentBudget{.foot_min_normal_n = 20.,
                                                 .foot_mu = .9, .saddle_mu = .6,
                                                 .grip_pull_n = 300.,
                                                 .max_gap_m = .005}),
                    "normal"),
                "supplied 20 N floor flags");
    bt::require(!has(attachment_violations(
                         sample, AttachmentBudget{.foot_min_normal_n = 10.,
                                                  .foot_mu = .9, .saddle_mu = .6,
                                                  .grip_pull_n = 300.,
                                                  .max_gap_m = .005}),
                     "normal"),
                "supplied 10 N floor passes");
}

void test_platform_cannot_pull_or_scrape_without_normal() {
    const AttachmentSample bad{.kind = "foot", .normal_n = -1., .tangent_n = 10.};
    bt::require(has(attachment_violations(bad, kBudget), "normal"),
                "negative normal flags");
    const AttachmentSample good{.kind = "foot", .normal_n = 20., .tangent_n = 18.,
                                .gap_m = 0.005};
    bt::require(attachment_violations(good, kBudget).empty(),
                "in-budget foot reports nothing");
}

void test_a_weld_cannot_supply_a_free_unlimited_couple() {
    const AttachmentSample sample{.kind = "foot", .normal_n = 100.,
                                  .moment_nm = 10., .half_patch_m = 0.025};
    bt::require(has(attachment_violations(sample, kBudget), "cop"),
                "free couple flagged via cop");
}

void test_per_hand_pull_and_saddle_friction() {
    bt::require(has(attachment_violations(
                        AttachmentSample{.kind = "grip", .pull_n = 300.01},
                        kBudget),
                    "pull"),
                "grip pull flagged");
    bt::require(has(attachment_violations(
                        AttachmentSample{.kind = "saddle", .normal_n = 100.,
                                         .tangent_n = 60.01},
                        kBudget),
                    "friction"),
                "saddle friction flagged");
    bt::require(has(attachment_violations(AttachmentSample{.kind = "saddle"},
                                          kBudget),
                    "normal"),
                "saddle without normal flagged");
}

void test_nonfinite_measurements_are_rejected() {
    for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
        const Violations v = attachment_violations(
            AttachmentSample{.kind = "foot", .normal_n = value}, kBudget);
        bt::require(v == Violations{"nonfinite"}, "nonfinite reports nonfinite");
    }
}

void test_foot_boundaries() {
    struct Case {
        double normal, tangent, moment, gap, pull, patch;
        Violations expected;
    };
    const std::array<Case, 9> cases{{
        {19.99, 0., 0., 0., 0., 0., {"normal"}},
        {20.0, 0., 0., 0.005001, 0., 0., {"gap"}},
        {100., 90.01, 0., 0., 0., 0., {"friction"}},
        {100., -90.01, 0., 0., 0., 0., {"friction"}},
        {100., 89.99, 0., 0., 0., 0., {}},
        {100., -89.99, 0., 0., 0., 0., {}},
        {100., 0., 2.501, 0., 0., 0.025, {"cop"}},
        {100., 0., -2.501, 0., 0., 0.025, {"cop"}},
        {100., 0., 2.499, 0., 0., 0.025, {}},
    }};
    for (const Case &c : cases) {
        const AttachmentSample sample{
            .kind = "foot", .normal_n = c.normal, .tangent_n = c.tangent,
            .moment_nm = c.moment, .gap_m = c.gap, .pull_n = c.pull,
            .half_patch_m = c.patch};
        bt::require(attachment_violations(sample, kBudget) == c.expected,
                    "foot boundary tuple matches");
    }
}

void test_saddle_pin_reports_no_couple_channel() {
    const AttachmentSample sample{.kind = "saddle", .normal_n = 100.,
                                  .moment_nm = 5.};
    bt::require(has(attachment_violations(sample, kBudget), "cop"),
                "pin moment flagged");
}

void test_unknown_attachment_kind_is_an_error() {
    bt::invalid([] {
        static_cast<void>(attachment_violations(
            AttachmentSample{.kind = "elbow"}, kBudget));
    });
}

void test_negative_measurement_fields_are_rejected() {
    bt::require(attachment_violations(
                    AttachmentSample{.kind = "foot", .normal_n = 100.,
                                     .gap_m = -0.001},
                    kBudget) == Violations{"invalid_measurement"},
                "negative gap rejected");
    bt::require(attachment_violations(
                    AttachmentSample{.kind = "foot", .normal_n = 100.,
                                     .half_patch_m = -0.01},
                    kBudget) == Violations{"invalid_measurement"},
                "negative half patch rejected");
}
} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        test_violations_use_the_supplied_budget();
        test_platform_cannot_pull_or_scrape_without_normal();
        test_a_weld_cannot_supply_a_free_unlimited_couple();
        test_per_hand_pull_and_saddle_friction();
        test_nonfinite_measurements_are_rejected();
        test_foot_boundaries();
        test_saddle_pin_reports_no_couple_channel();
        test_unknown_attachment_kind_is_an_error();
        test_negative_measurement_fields_are_rejected();
    });
}
