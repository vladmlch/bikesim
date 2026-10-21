// Task 3.19: tests/reference/test_motor_profile.py. Native assist owns the
// permission/finite-input boundary formerly exposed as two Python helpers.
#include "drive_reference_support.hpp"
#include "io/json.hpp"
#include "runtime/setup_values.hpp"

#include <array>
#include <functional>
#include <limits>
#include <string>
#include <type_traits>

namespace {
namespace cfg = bike::configuration;
using drive_reference::near;
using drive_reference::run_assist;
using drivetrain::AssistController;

void test_bosch_profile_values_are_the_recalled_unverified_set() {
    const auto* profile = cfg::find_motor_profile("bosch_cx_gen4");
    bt::require(profile == &cfg::kBoschCxGen4, "named registry resolves recalled profile");
    bt::require(profile->peak_torque_nm == 85. && profile->rated_power_w == 250. && profile->peak_power_w == 600.,
                "recalled torque and power data");
    bt::require(profile->torque_tau_s == .04, "recalled torque lag");
    near(profile->cutoff_mps, 25. / 3.6, "recalled cutoff");
    near(profile->taper_width_mps, 2. / 3.6, "recalled taper");
    near(profile->gate_min_crank_rad_s, 5. * std::numbers::pi / 180., "recalled crank gate");
    bt::require(profile->provenance.find("unverified") != std::string::npos &&
        profile->provenance.find("no web lookup") != std::string::npos, "unverified provenance is retained");
}
void test_mode_gains(const drivetrain::AssistConfig& config) {
    struct Row { std::string_view mode; double human; double expected; };
    constexpr std::array rows{
        Row{"eco", 30., .6}, Row{"tour", 30., 1.4}, Row{"turbo", 30., 3.4}, Row{"turbo", 0., 3.4},
        Row{"emtb", 0., 1.4}, Row{"emtb", 20., 2.4}, Row{"emtb", 40., 3.4}, Row{"emtb", 80., 3.4},
        Row{"emtb", -5., 1.4}};
    bt::require(config.profile.has_value(), "projected profile supplies gain table");
    for (const auto& row : rows) {
        const auto mode = drivetrain::profile_mode(row.mode);
        bt::require(mode.has_value(), "declared gain mode parses");
        near(drivetrain::profile_gain(*config.profile, *mode, row.human), row.expected, "declared mode gain");
    }
}
void test_unknown_mode_is_rejected(const drivetrain::AssistConfig& config) {
    bt::require(!drivetrain::profile_mode("sport"), "unknown profile mode does not parse");
    auto malformed = config; malformed.mode = "sport";
    bt::invalid([&] { (void)AssistController(malformed); });
}
void test_profile_is_frozen_and_copies_the_gain_mapping(const drivetrain::AssistConfig& config) {
    auto gains = cfg::kBoschCxGen4.mode_gains;
    auto source = cfg::kBoschCxGen4; source.mode_gains = gains;
    const auto profile = source;
    gains.turbo = 99.; source.mode_gains.turbo = 99.;
    bt::require(profile.mode_gains.turbo == 3.4, "gain value is detached from caller storage");
    bt::require(std::is_const_v<decltype(cfg::kBoschCxGen4)>, "published registry profile is immutable");
    auto input = config; AssistController controller(input);
    bt::require(input.profile.has_value(), "owned profile exists");
    input.profile->turbo = 99.;
    (void)controller.step(10., 60., 0., false, .001);
    near(controller.state().last_gain, 3.4, "controller owns profile data independently of caller");
    // FrozenInstanceError/TypeError on Python attribute/item assignment are
    // language-only assertions; C++ const registry plus value ownership is
    // their replacement. No unsupported mutation is executed as a test.
}
void test_profile_rejects_invalid_data(const bike::io::Value& valid_writer) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<std::function<void(cfg::MotorProfile&)>, 9> changes{
        [](auto& p) { p.name.clear(); }, [](auto& p) { p.provenance = "  "; },
        [](auto& p) { p.taper_width_mps = 8.; }, [](auto& p) { p.peak_torque_nm = 0.; },
        [](auto& p) { p.rated_power_w = 700.; }, [nan](auto& p) { p.torque_tau_s = nan; },
        [inf](auto& p) { p.gate_min_crank_rad_s = inf; },
        [](auto& p) { p.mode_gains.emtb = {3.4, 1.4}; }, [nan](auto& p) { p.mode_gains.eco = nan; }};
    for (const auto& change : changes) {
        auto profile = cfg::kBoschCxGen4; change(profile);
        bt::invalid([&] { cfg::detail::post_init(profile); });
    }
    // The fixed C++ record cannot express missing keys or wrong tuple kinds.
    // Exercise those source parameters at the actual dynamic writer boundary.
    using namespace runtime::setupvalue;
    for (const auto malformed : {
        R"({"eco":0.5})",
        R"({"eco":[0.6,0.9],"tour":1.4,"emtb":[1.4,3.4],"turbo":3.4})",
        R"({"eco":0.6,"tour":1.4,"emtb":2.0,"turbo":3.4})"}) {
        auto writer = valid_writer;
        auto drive = field(writer, "drive"); auto assist = field(drive, "assist"); auto profile = field(assist, "profile");
        set(profile, "mode_gains", bike::io::parse_json(malformed));
        set(assist, "profile", std::move(profile)); set(drive, "assist", std::move(assist)); set(writer, "drive", std::move(drive));
        bt::invalid([&] { (void)nativecfg::decode(writer); });
    }
}
void test_pedelec_cap_is_a_permission_not_a_throttle(const drivetrain::AssistConfig& config) {
    const double gate_rpm = 5. / 6.;
    const double one_radian_per_second_rpm = 60. / (2. * std::numbers::pi);
    // The native API returns delivered torque, so the uncapped permission is
    // observed as the full 3.4*10 Nm rider request. It exposes no infinity cap.
    auto delivered = [&](double human, double cadence, std::optional<double> cap, bool braking) {
        AssistController c(config);
        (void)run_assist(c, human, cadence, 0., 1., cap);
        return c.step(human, cadence, 0., braking, .001, cap);
    };
    near(delivered(10., one_radian_per_second_rpm, std::nullopt, false), 34., "unconstrained permission preserves rider request");
    near(delivered(10., one_radian_per_second_rpm, 30., false), 30., "explicit permission ceiling");
    bt::require(delivered(10., one_radian_per_second_rpm, 0., false) == 0., "zero ceiling");
    bt::require(delivered(10., one_radian_per_second_rpm, std::nullopt, true) == 0., "braking denies permission");
    bt::require(delivered(0., one_radian_per_second_rpm, std::nullopt, false) == 0., "zero effort denies permission");
    bt::require(delivered(-3., one_radian_per_second_rpm, std::nullopt, false) == 0., "negative effort denies permission");
    bt::require(delivered(10., gate_rpm, std::nullopt, false) == 0., "exact crank gate denies permission");
    bt::require(delivered(10., -one_radian_per_second_rpm, std::nullopt, false) == 0., "backward crank denies permission");
    near(delivered(10., gate_rpm * 1.01, std::nullopt, false), 34., "above crank gate grants full support");
}
void test_pedelec_cap_rejects_invalid_inputs(const drivetrain::AssistConfig& config) {
    AssistController c(config);
    bt::invalid([&] { (void)c.step(std::numeric_limits<double>::quiet_NaN(), 60., 0., false, .001); });
    bt::invalid([&] { (void)c.step(10., std::numeric_limits<double>::infinity(), 0., false, .001); });
    auto bad = config; bad.gate_min_crank_rad_s = -.1;
    bt::invalid([&] { (void)AssistController(bad); });
    for (double cap : {-1., std::numeric_limits<double>::infinity()})
        bt::invalid([&] { (void)c.step(10., 60., 0., false, .001, cap); });
    // The Python-only `braking=1` rejection tested strict Python bool
    // marshalling. The native signature owns a bool and has no such input
    // representation; that parameter is a documented drop, not a passed gate.
}
void test_gain_rejects_nonfinite_human_torque(const drivetrain::AssistConfig& config) {
    auto input = config; input.mode = "emtb"; AssistController c(input);
    bt::invalid([&] { (void)c.step(std::numeric_limits<double>::quiet_NaN(), 60., 0., false, .001); });
}
} // namespace
int main() { return bt::run([] {
    const auto writer = drive_reference::projected_writer(drive_reference::turbo_source());
    const auto decoded = std::make_unique<nativecfg::WriterConfig>(nativecfg::decode(writer));
    bt::require(decoded->drive.has_value(), "profile writer has drivetrain");
    const auto& config = decoded->drive->policies.assist;
    test_bosch_profile_values_are_the_recalled_unverified_set();
    test_mode_gains(config);
    test_unknown_mode_is_rejected(config);
    test_profile_is_frozen_and_copies_the_gain_mapping(config);
    test_profile_rejects_invalid_data(writer);
    test_pedelec_cap_is_a_permission_not_a_throttle(config);
    test_pedelec_cap_rejects_invalid_inputs(config);
    test_gain_rejects_nonfinite_human_torque(config);
}); }
