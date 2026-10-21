// Task 3.18: scalar policy contracts and frozen oracle comparisons.
// Capture-only source changes give every tree leaf a stable label. This test
// intentionally fails when either platform's real capture is still missing.
#include "policy_reference_support.hpp"

namespace {
using namespace policy_reference;

// tests/reference/test_native_drive_policies.py::test_native_policy_api_exists
void test_native_policy_api_exists(const Value& baseline) {
    auto policy = policies(baseline);
    bt::require(sorted_keys(state(policy)) == std::vector<std::string>{"assist", "battery", "hub", "pedaling", "shifting"},
                "native policy owner exposes all five complete snapshots");
}

// tests/reference/test_native_drive_policies.py::test_crank_torque
void test_crank_torque() {
    auto golden = node("test_crank_torque");
    for (double mean : {-10., -0., 0., 30.})
        for (double phase : {-23., -std::numbers::pi, -0., 0., .71, std::numbers::pi / 2., 15.})
            for (double ripple : {0., .35, .999})
                scalar(golden, drivetrain::human_crank_torque(mean, phase, ripple));
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_battery_draw_matches_python
void test_battery_draw_matches_python(const Value& baseline) {
    auto golden = node("test_battery_draw_matches_python"); tree(golden, baseline);
    auto policy = policies(baseline);
    for (const auto& [power, dt] : std::array<std::array<double, 2>, 4>{{{5., .002}, {1000., .01}, {1e9, .1}, {0., .01}}}) {
        scalar(golden, policy.battery.draw(power, dt)); tree(golden, state(policy));
    }
    golden.done();
}

struct PedalCase { double phase, rate, required, effort, dt; bool enabled = true, braking = false; };
void pedal_case(bt::oracle::Node& golden, PolicySet& policy, const PedalCase& item) {
    tree(golden, pedaling_result(policy.pedaling.update(item.phase, item.rate, item.required,
        item.effort, item.dt, item.enabled, item.braking)));
    tree(golden, state(policy));
}

// tests/reference/test_native_drive_policies.py::test_pedaling_sequences
void test_pedaling_sequences(const Value& baseline) {
    auto golden = node("test_pedaling_sequences"); tree(golden, baseline);
    auto policy = policies(baseline);
    const std::array cases{
        PedalCase{0., 0., 0., 20., .01}, PedalCase{2., 4., 40., 20., .04},
        PedalCase{3., 15., 120., 30., .04}, PedalCase{4., 11., 80., 30., .01},
        PedalCase{4., -8., -30., 30., .4}, PedalCase{5., -5., -60., 30., .01, true, true},
        PedalCase{6., -5., 0., 0., .1}, PedalCase{6., 0., 0., 30., .1, false, false}};
    for (const auto& item : cases) pedal_case(golden, policy, item);
    bike::random::NumpyRng random(934);
    for (int i = 0; i < 200; ++i) {
        const PedalCase item{random.uniform(-40., 40.), random.uniform(-15., 15.),
            random.uniform(-100., 130.), random.uniform(0., 65.), .007};
        pedal_case(golden, policy, item);
    }
    golden.done();
}

struct ShiftCase {
    double cadence, required, dt;
    bool pedaling = true, braking = false, contact = true;
    std::optional<double> slip = std::nullopt;
};
void shift_case(bt::oracle::Node& golden, PolicySet& policy, const ShiftCase& item) {
    shifted(golden, policy.shifting.update(item.cadence, item.required, item.dt,
        item.pedaling, item.braking, item.contact, item.slip));
    tree(golden, shifting_result(policy)); tree(golden, state(policy));
}

// tests/reference/test_native_drive_policies.py::test_shifting[raw/ema-signed/magnitude]
void test_shifting() {
    for (double tau : {0., .35}) for (auto mode : {"legacy_signed", "magnitude"}) {
        const auto id = std::string(tau == 0. ? "raw-" : "ema-") + (std::string_view(mode) == "legacy_signed" ? "signed" : "magnitude");
        auto golden = node("test_shifting", id);
        auto input = source(false); input.shifting.enabled = true;
        input.shifting.upshift_slip_mode = mode; input.shifting.cadence_smoothing_tau_s = tau;
        auto policy = policies(config(input));
        const std::array cases{
            ShiftCase{100., 100., .5, true, false, true, 1.},
            ShiftCase{100., 100., .5, true, false, true, -1.},
            ShiftCase{60., 60., .01}, ShiftCase{60., 60., .5},
            ShiftCase{60., 120., .5}, ShiftCase{120., 30., .5},
            ShiftCase{100., 100., .5, true, false, false},
            ShiftCase{100., 100., .5, false}, ShiftCase{100., 100., .5, true, true},
            ShiftCase{-10., -10., .5}};
        for (const auto& item : cases) shift_case(golden, policy, item);
        bike::random::NumpyRng random(11);
        for (int i = 0; i < 250; ++i) {
            const ShiftCase item{random.uniform(30., 130.), random.uniform(30., 130.), .01};
            shift_case(golden, policy, item);
        }
        golden.done();
    }
}

struct AssistCase {
    double human, cadence, speed; bool braking; double dt;
    std::optional<double> cap, shaft;
};
void assist_case(bt::oracle::Node& golden, PolicySet& policy, const AssistCase& item) {
    scalar(golden, policy.assist.step(item.human, item.cadence, item.speed, item.braking, item.dt, item.cap, item.shaft));
    tree(golden, state(policy));
}

// tests/reference/test_native_drive_policies.py::test_assist[custom/eco/tour/emtb/turbo]
void test_assist() {
    for (auto mode : {"custom", "eco", "tour", "emtb", "turbo"}) {
        auto golden = node("test_assist", mode);
        auto input = source(false); input.assist.mode = mode;
        if (std::string_view(mode) != "custom") input.assist.profile = "bosch_cx_gen4";
        input.assist.torque_curve = std::vector<std::array<double, 2>>{{0., 80.}, {43., 71.}, {100., 35.}, {150., 0.}};
        auto policy = policies(config(input));
        for (double shaft : {-10., 0., 10., 43., 77.7, 100., 149., 150., 170.})
            for (double speed : {0., 6.5, -6.7, 7.}) pair(golden, policy.assist.ceiling(shaft, speed));
        const std::array cases{
            AssistCase{4., 80., 0., false, .005, {}, {}}, AssistCase{30., 0., 0., false, .005, {}, {}},
            AssistCase{30., 80., 0., true, .005, {}, {}}, AssistCase{10., 80., 0., false, .005, 0., {}},
            AssistCase{30., 80., 6.7, false, .005, 2., 15.}, AssistCase{40., 80., 0., false, .005, {}, 150.},
            AssistCase{100., 80., 0., false, .005, {}, {}}, AssistCase{-1., 80., 0., false, .005, {}, {}}};
        for (const auto& item : cases) assist_case(golden, policy, item);
        bike::random::NumpyRng random(75);
        for (int i = 0; i < 250; ++i) {
            const AssistCase item{random.uniform(0., 100.), random.uniform(-5., 150.), random.uniform(-7., 7.),
                false, .003, {}, random.uniform(-5., 150.)};
            assist_case(golden, policy, item);
        }
        golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_energy_helpers[disabled/enabled-lossless/copper]
void test_energy_helpers(const Value& baseline) {
    for (bool enabled : {false, true}) for (double copper : {0., .02}) {
        const auto id = std::string(enabled ? "enabled-" : "disabled-") + (copper == 0. ? "lossless" : "copper");
        auto golden = node("test_energy_helpers", id); tree(golden, baseline);
        auto input = baseline;
        put(input, "battery", "enabled", Value{enabled}); put(input, "battery", "copper_w_per_nm2", real(copper));
        put(input, "battery", "speed_w_per_rad_s2", real(.03)); put(input, "battery", "idle_w", real(5.));
        auto policy = policies(input);
        for (double torque : {-0., 0., 5., 37.13, 80.}) for (double omega : {-7.3, -0., 0., 5.2}) {
            scalar(golden, electrical(policy, torque, omega, enabled));
            for (double budget : {0., 4.99, 5., 6., 72., 500.}) scalar(golden, energy_limit(policy, torque, omega, budget));
        }
        golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_freehub
void test_freehub(const Value& baseline) {
    auto golden = node("test_freehub"); tree(golden, baseline);
    auto policy = policies(baseline);
    constexpr std::array<std::array<double, 4>, 6> cases{{
        {0., 0., 0., 0.}, {.1, 0., 2., 0.}, {.3, .5, 0., 3.},
        {.4, .5, 4., 0.}, {8., 3., 2., 3.}, {8., 10., 2., 3.}}};
    for (const auto& item : cases) {
        scalar(golden, policy.hub.update(item[0], item[1], item[2], item[3])); tree(golden, state(policy));
    }
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_replay_reset_and_ownership
void test_replay_reset_and_ownership(const Value& baseline) {
    auto golden = node("test_replay_reset_and_ownership"); tree(golden, baseline);
    auto input = baseline; auto policy = policies(input); auto old = state(policy);
    (void)policy.assist.step(30., 80., 0., false, .01); restore(policy, old); tree(golden, state(policy));
    put(input, "assist", "gain", real(999.)); put(input, "shifting", "cassette", Value{Value::Array{}}); input = Value{};
    scalar(golden, policy.assist.step(30., 80., 0., false, .01));
    policy.reset(); tree(golden, state(policy));
    put(old, "battery", "energy_j", real(1.));
    bt::require(policy.battery.state().energy_j != 1., "restored source snapshot does not alias live state");
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_invalid_restore_is_atomic[section]
void test_invalid_restore_is_atomic(const Value& baseline) {
    for (auto section : {"pedaling", "shifting", "assist", "battery", "hub"}) {
        auto golden = node("test_invalid_restore_is_atomic", section); tree(golden, baseline);
        auto policy = policies(baseline); const auto old = state(policy); const auto before = state_text(policy);
        for (const auto& key : sorted_keys(field(old, section))) {
            const std::array invalid{Value{bike::io::Number{"nan"}}, Value{bike::io::Number{"inf"}},
                Value{std::string("invalid")}, Value{Value::Array{}}, Value{}};
            for (const auto& value : invalid) {
                if (bike::io::is_null(field(field(old, section), key)) && bike::io::is_null(value)) continue;
                auto bad = old; put(bad, section, key, value);
                bt::invalid([&] { restore(policy, bad); });
                bt::require(state_text(policy) == before, "invalid policy field cannot publish partial state");
                tree(golden, state(policy));
            }
            auto missing = old; erase(member(missing, section), key);
            bt::invalid([&] { restore(policy, missing); });
            bt::require(state_text(policy) == before, "missing policy field cannot publish partial state");
            tree(golden, state(policy));
        }
        golden.done();
    }
}

struct Mutation { std::string_view id, section, key, json; };
// tests/reference/test_native_drive_policies.py::test_invalid_restore_values[case]
void test_invalid_restore_values(const Value& baseline) {
    constexpr std::array cases{
        Mutation{"battery-negative", "battery", "energy_j", "-1.0"}, Mutation{"hub-negative", "hub", "energy_j", "-1.0"},
        Mutation{"cooldown-negative", "shifting", "cooldown_s", "-1.0"}, Mutation{"teeth-small", "shifting", "rear_teeth", "2"},
        Mutation{"count-negative", "shifting", "shift_count", "-1"}, Mutation{"teeth-fractional", "shifting", "rear_teeth", "3.5"},
        Mutation{"coasting-integer", "pedaling", "coasting", "1"}, Mutation{"ema-unpaired", "shifting", "cadence_ema", "10.0"}};
    for (const auto& item : cases) {
        auto golden = node("test_invalid_restore_values", item.id); tree(golden, baseline);
        auto policy = policies(baseline); auto bad = state(policy); const auto before = state_text(policy);
        put(bad, item.section, item.key, bike::io::parse_json(item.json));
        bt::invalid([&] { restore(policy, bad); });
        bt::require(state_text(policy) == before, "invalid restored value is atomic"); tree(golden, state(policy));
        golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_config_validation
void test_config_validation(const Value& baseline) {
    auto golden = node("test_config_validation"); tree(golden, baseline);
    for (auto section : {"gearing", "pedaling", "shifting", "assist", "battery"})
        for (const auto& [key, value] : bike::io::as_object(field(baseline, section), section)) {
            if (key == "profile" || key == "torque_curve" || key.starts_with("rollback_")) continue;
            auto bad = baseline; erase(member(bad, section), key);
            bt::invalid([&] { (void)policies(bad); });
            if (const auto* numeric = std::get_if<bike::io::Number>(&value.data);
                numeric && !bike::io::is_integer_lexeme(numeric->lexeme)) {
                put(bad, section, key, Value{bike::io::Number{"nan"}});
                bt::invalid([&] { (void)policies(bad); });
            }
        }
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_projection_resolves_runtime
void test_projection_resolves_runtime() {
    auto input = source(); input.assist.profile = "bosch_cx_gen4"; input.assist.mode = "emtb";
    auto policy = policies(config(input)); const auto& a = policy.config.assist;
    bt::require(a.max_torque == 85. && a.slew == 2125., "profile projection resolves runtime torque and slew");
    bt::require(a.profile.has_value(), "profile gain map survives projection");
    const auto& gains = *a.profile;
    bt::require(gains.eco == .6 && gains.tour == 1.4 && gains.emtb_low == 1.4 && gains.emtb_high == 3.4 && gains.turbo == 3.4,
                "profile projection preserves all declared gains");
    const auto document = config(input);
    bt::require(sorted_keys(field(document, "pedaling")) == std::vector<std::string>{
        "coast_above_rpm", "coast_cadence_tau_s", "effort_slew_nm_s", "enabled", "mash_cadence_rpm", "mash_torque_nm",
        "resume_below_rpm", "rollback_brake", "rollback_demand", "rollback_engage_mps", "rollback_release_mps", "stop_time_s"},
        "projected pedaling shape includes rollback controls");
    input = bike::configuration::PhysicalDriveConfig{};
    bt::require(policy.assist.step(30., 80., 0., false, .01) > 0., "projected owner survives source destruction");
}

// tests/reference/test_native_drive_policies.py::test_projection_normalizes_numeric_leaves
void test_projection_normalizes_numeric_leaves() {
    auto input = source(false); input.battery.energy_j = 123.; input.battery.copper_w_per_nm2 = .02;
    input.shifting.shift_cooldown_s = .4;
    const auto document = config(input);
    std::function<void(const Value&)> plain = [&](const Value& value) {
        if (const auto* object = std::get_if<Value::Object>(&value.data)) {
            for (const auto& [key, child] : *object) { (void)key; plain(child); }
        } else if (const auto* array = std::get_if<Value::Array>(&value.data)) {
            for (const auto& child : *array) plain(child);
        } else bt::require(std::holds_alternative<std::monostate>(value.data) || std::holds_alternative<bike::io::Number>(value.data) ||
            std::holds_alternative<bool>(value.data) || std::holds_alternative<std::string>(value.data), "projected leaf is an owned scalar");
    };
    plain(document);
    const auto policy = policies(document);
    bt::require(policy.battery.state().energy_j == 123., "plain projected numeric value is preserved");
}

// tests/reference/test_native_drive_policies.py::test_complete_reset_and_atomic_multi_policy_restore
void test_complete_reset_and_atomic_multi_policy_restore(const Value& baseline) {
    auto golden = node("test_complete_reset_and_atomic_multi_policy_restore"); tree(golden, baseline);
    auto policy = policies(baseline);
    (void)policy.pedaling.update(1., 13., 120., 30., .04);
    (void)policy.shifting.update(100., 100., .01);
    (void)policy.assist.step(30., 80., 0., false, .01);
    (void)policy.battery.draw(80., .01);
    (void)policy.hub.update(0., 0., 2., 0.); (void)policy.hub.update(.1, 0., 2., 0.);
    auto bad = state(policy); const auto before = state_text(policy);
    put(bad, "pedaling", "_effort", real(17.)); put(bad, "shifting", "rear_teeth", integer(33));
    put(bad, "battery", "energy_j", real(8.)); put(bad, "hub", "torque_nm", Value{bike::io::Number{"inf"}});
    bt::invalid([&] { restore(policy, bad); });
    bt::require(state_text(policy) == before, "late invalid hub value cannot commit earlier policy fields");
    tree(golden, state(policy)); policy.reset(); tree(golden, state(policy)); golden.done();
}

// tests/reference/test_native_drive_policies.py::test_restored_finite_snapshots_preserve_bits
void test_restored_finite_snapshots_preserve_bits(const Value& baseline) {
    auto golden = node("test_restored_finite_snapshots_preserve_bits"); tree(golden, baseline);
    auto policy = policies(baseline); auto document = state(policy);
    put(document, "pedaling", "target_phase_rad", real(-1000.));
    put(document, "pedaling", "target_rate_rad_s", real(-0.)); put(document, "pedaling", "_cadence_ema", real(-10.));
    put(document, "shifting", "rear_teeth", integer(51)); put(document, "shifting", "from_teeth", integer(45));
    put(document, "shifting", "cadence_ema", real(-3.)); put(document, "shifting", "required_ema", real(-5.));
    put(document, "assist", "torque", real(1e8)); put(document, "assist", "last_gain", real(1e9));
    const double initial = policy.battery.state().initial_energy_j;
    put(document, "battery", "energy_j", real(initial * .5)); put(document, "battery", "drawn_energy_j", real(initial * .75));
    put(document, "hub", "boundary", real(-1e9)); put(document, "hub", "torque_nm", real(1e9));
    restore(policy, document); tree(golden, state(policy));
    bt::require(std::bit_cast<std::uint64_t>(policy.pedaling.state().target_rate_rad_s) == UINT64_C(0x8000000000000000),
                "restoration preserves negative-zero target rate");
    put(document, "pedaling", "target_phase_rad", real(0.));
    bt::require(policy.pedaling.state().target_phase_rad == std::optional<double>(-1000.), "restored state owns its optional value");
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_disabled_shifter_initial_teeth_outside_cassette
void test_disabled_shifter_initial_teeth_outside_cassette() {
    auto golden = node("test_disabled_shifter_initial_teeth_outside_cassette");
    auto input = source(false); input.gearing.rear_teeth = 31;
    auto policy = policies(config(input)); policy.restore(policy.snapshot());
    tree(golden, state(policy)); golden.done();
}

// tests/reference/test_native_drive_policies.py::test_invalid_config_values[case]
void test_invalid_config_values(const Value& baseline) {
    constexpr std::array cases{
        Mutation{"front-bool", "gearing", "front_teeth", "true"}, Mutation{"rear-fractional", "gearing", "rear_teeth", "3.1"},
        Mutation{"pitch-zero", "gearing", "chain_pitch_m", "0.0"}, Mutation{"pedaling-bool", "pedaling", "enabled", "1"},
        Mutation{"stop-zero", "pedaling", "stop_time_s", "0.0"}, Mutation{"resume-band", "pedaling", "resume_below_rpm", "120.0"},
        Mutation{"shifting-bool", "shifting", "enabled", "1"}, Mutation{"cassette-duplicate", "shifting", "cassette", "[10,10]"},
        Mutation{"cassette-bool", "shifting", "cassette", "[true,24]"}, Mutation{"cassette-empty", "shifting", "cassette", "[]"},
        Mutation{"torque-factor", "shifting", "torque_factor", "1.1"}, Mutation{"shift-cut", "shifting", "shift_cut_duration_s", "1.0"},
        Mutation{"slip-mode", "shifting", "upshift_slip_mode", "\"other\""}, Mutation{"tau-zero", "assist", "tau", "0.0"},
        Mutation{"slew-zero", "assist", "slew", "0.0"}, Mutation{"gain-negative", "assist", "gain", "-1.0"},
        Mutation{"mode-integer", "assist", "mode", "1"}, Mutation{"curve-short", "assist", "torque_curve", "[[0.0,1.0]]"},
        Mutation{"curve-duplicate", "assist", "torque_curve", "[[0.0,1.0],[0.0,2.0]]"},
        Mutation{"curve-negative", "assist", "torque_curve", "[[0.0,1.0],[2.0,-1.0]]"},
        Mutation{"profile-incomplete", "assist", "profile", "{\"mode_gains\":{}}"},
        Mutation{"battery-bool", "battery", "enabled", "1"}, Mutation{"energy-negative", "battery", "energy_j", "-1.0"}};
    for (const auto& item : cases) {
        auto golden = node("test_invalid_config_values", item.id); tree(golden, baseline);
        auto bad = baseline; put(bad, item.section, item.key, bike::io::parse_json(item.json));
        bt::invalid([&] { (void)policies(bad); }); golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_missing_config_sections[key]
void test_missing_config_sections(const Value& baseline) {
    for (auto key : {"gearing", "pedaling", "shifting", "assist", "battery", "hub_stiffness_nm_rad", "hub_damping_nm_s"}) {
        auto golden = node("test_missing_config_sections", key); tree(golden, baseline);
        auto bad = baseline; erase(bad, key); bt::invalid([&] { (void)policies(bad); }); golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_random_curve_interpolation
void test_random_curve_interpolation(const Value& baseline) {
    auto golden = node("test_random_curve_interpolation"); tree(golden, baseline);
    auto input = baseline; put(input, "assist", "max_torque", real(1e6)); put(input, "assist", "max_power", real(1e9));
    put(input, "assist", "torque_curve", bike::io::parse_json("[[0.0,80.0],[43.0,71.0],[100.0,35.0],[150.0,0.0]]"));
    auto policy = policies(input);
    for (double rpm : {0., 43., 100., 150., -0., -1., 151.}) pair(golden, policy.assist.ceiling(rpm, 0.));
    bike::random::NumpyRng random(541);
    for (int i = 0; i < 1000; ++i) pair(golden, policy.assist.ceiling(random.uniform(-10., 170.), 0.));
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_battery_nonzero_idle_remainder
void test_battery_nonzero_idle_remainder(const Value& baseline) {
    auto golden = node("test_battery_nonzero_idle_remainder"); tree(golden, baseline);
    auto policy = policies(baseline); auto restored = policy.snapshot(); restored.battery.energy_j = .001; policy.restore(restored);
    bt::require(energy_limit(policy, 80., 0., .001 / .01) == 0., "idle loss can consume the available power budget");
    bt::require(policy.battery.state().energy_j == .001, "energy-limit query does not debit battery"); golden.done();
}

// tests/reference/test_native_drive_policies.py::test_config_and_python_object_deletion
void test_config_and_python_object_deletion() {
    auto golden = node("test_config_and_python_object_deletion");
    auto policy = [] {
        auto input = source(false); input.assist.profile = "bosch_cx_gen4"; input.assist.mode = "emtb";
        input.assist.torque_curve = std::vector<std::array<double, 2>>{{0., 85.}, {120., 0.}};
        return policies(config(input));
    }();
    scalar(golden, policy.assist.step(27., 72., 6.7, false, .01)); tree(golden, state(policy)); golden.done();
}

// tests/reference/test_native_drive_policies.py::test_pedaling_parameter_branches[raw/mash/filtered]
void test_pedaling_parameter_branches() {
    struct Parameters { std::string_view id; double tau, slew, mash; };
    constexpr std::array parameters{Parameters{"raw", 0., 0., 0.}, Parameters{"mash", 0., 0., 60.}, Parameters{"filtered", .03, 300., 0.}};
    constexpr std::array cases{PedalCase{0., 0., 0., 20., .01}, PedalCase{1., 12., 110., 30., .03},
        PedalCase{2., 10., 100., 30., .03}, PedalCase{3., 8., 80., 30., .04},
        PedalCase{4., -14., -90., 30., .04}, PedalCase{5., -6., -10., 0., .5}};
    for (const auto& item : parameters) {
        auto golden = node("test_pedaling_parameter_branches", item.id);
        auto input = source(false); input.pedaling.enabled = true; input.pedaling.coast_cadence_tau_s = item.tau;
        input.pedaling.effort_slew_nm_s = item.slew; input.pedaling.mash_torque_nm = item.mash;
        auto policy = policies(config(input));
        for (const auto& row : cases) pedal_case(golden, policy, row);
        golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_optional_config_sections
void test_optional_config_sections(const Value& baseline) {
    auto golden = node("test_optional_config_sections"); tree(golden, baseline);
    auto input = baseline; erase(member(input, "assist"), "profile"); erase(member(input, "assist"), "torque_curve");
    const auto policy = policies(input);
    bt::require(!policy.config.assist.profile && !policy.config.assist.torque_curve, "absent optional assist fields remain absent");
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_extra_and_missing_state_sections
void test_extra_and_missing_state_sections(const Value& baseline) {
    auto golden = node("test_extra_and_missing_state_sections"); tree(golden, baseline);
    auto policy = policies(baseline); const auto old = state(policy); const auto before = state_text(policy);
    std::array<Value, 3> candidates{old, old, old};
    set(candidates[0], "extra", Value{Value::Object{}}); erase(candidates[1], "hub"); put(candidates[2], "pedaling", "extra", real(0.));
    for (const auto& value : candidates) {
        bt::invalid([&] { restore(policy, value); });
        bt::require(state_text(policy) == before, "state schema rejects extra or missing members atomically"); tree(golden, state(policy));
    }
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_invalid_calls_preserve_state[case]
void test_invalid_calls_preserve_state(const Value& baseline) {
    struct Case { std::string_view id; std::function<void(PolicySet&)> invoke; };
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    const std::array cases{
        Case{"pedal-nan", [nan](auto& p) { (void)p.pedaling.update(nan, 0., 0., 30., .01); }},
        Case{"pedal-zero-dt", [](auto& p) { (void)p.pedaling.update(0., 0., 0., 30., 0.); }},
        Case{"shift-zero-dt", [](auto& p) { (void)p.shifting.update(80., 80., 0.); }},
        Case{"shift-infinite", [inf](auto& p) { (void)p.shifting.update(80., inf, .01); }},
        Case{"assist-zero-dt", [](auto& p) { (void)p.assist.step(30., 80., 0., false, 0.); }},
        Case{"assist-infinite", [inf](auto& p) { (void)p.assist.step(inf, 80., 0., false, .01); }},
        Case{"battery-infinite", [inf](auto& p) { (void)p.battery.draw(inf, .01); }},
        Case{"battery-zero-dt", [](auto& p) { (void)p.battery.draw(1., 0.); }},
        Case{"hub-infinite", [inf](auto& p) { (void)p.hub.update(0., inf, 0., 0.); }},
        Case{"power-infinite", [inf](auto& p) { (void)electrical(p, inf, 0., true); }},
        Case{"limit-negative", [](auto& p) { (void)energy_limit(p, -1., 0., 100.); }}};
    for (const auto& item : cases) {
        auto golden = node("test_invalid_calls_preserve_state", item.id); tree(golden, baseline);
        auto policy = policies(baseline); const auto before = state_text(policy);
        bt::invalid([&] { item.invoke(policy); });
        bt::require(state_text(policy) == before, "invalid scalar call preserves all policy state"); tree(golden, state(policy)); golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_invalid_human_torque[nan/infinite/negative/one]
void test_invalid_human_torque() {
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    const std::array<std::array<double, 3>, 4> cases{{{nan, 0., .35}, {1., inf, .35}, {1., 0., -1.}, {1., 0., 1.}}};
    for (const auto& item : cases)
        bt::invalid([&] { (void)drivetrain::human_crank_torque(item[0], item[1], item[2]); });
}

// tests/reference/test_native_drive_policies.py::test_shifter_count_boundary_rejects_successful_increment_atomically
void test_shifter_count_boundary_rejects_successful_increment_atomically(const Value& baseline) {
    auto golden = node("test_shifter_count_boundary_rejects_successful_increment_atomically"); tree(golden, baseline);
    auto policy = policies(baseline); auto restored = policy.snapshot();
    restored.shifting.shift_count = std::numeric_limits<std::int32_t>::max(); restored.shifting.direction = "up"; policy.restore(restored);
    const auto before = state_text(policy);
    overflow([&] { (void)policy.shifting.update(100., 100., .01); });
    bt::require(state_text(policy) == before, "successful-shift counter overflow is atomic"); tree(golden, state(policy));
    bt::require(!policy.shifting.update(75., 75., .01), "non-shifting interval remains legal at maximum count");
    bt::require(policy.shifting.state().shift_count == std::numeric_limits<std::int32_t>::max(), "non-shift preserves maximum count");
    golden.done();
}

// tests/reference/test_native_drive_policies.py::test_unrepresentable_count_restore_is_atomic
void test_unrepresentable_count_restore_is_atomic(const Value& baseline) {
    auto golden = node("test_unrepresentable_count_restore_is_atomic"); tree(golden, baseline);
    auto policy = policies(baseline); auto bad = state(policy); const auto before = state_text(policy);
    put(bad, "shifting", "shift_count", Value{bike::io::Number{"2147483648"}});
    bt::invalid([&] { restore(policy, bad); });
    bt::require(state_text(policy) == before, "unrepresentable restore count is atomic"); tree(golden, state(policy)); golden.done();
}

// tests/reference/test_native_drive_policies.py::test_assist_computed_crank_overflow_matches_python_state[shaft-cadence-profile]
void test_assist_computed_crank_overflow_matches_python_state() {
    for (bool shaft_override : {false, true}) for (double cadence : {-1e308, 1e308}) for (bool profiled : {false, true}) {
        const auto id = std::string(shaft_override ? "shaft-" : "crank-") + (cadence < 0. ? "negative-" : "positive-") + (profiled ? "emtb" : "plain");
        auto golden = node("test_assist_computed_crank_overflow_matches_python_state", id);
        auto input = source(false);
        if (profiled) { input.assist.profile = "bosch_cx_gen4"; input.assist.mode = "emtb"; }
        auto policy = policies(config(input));
        scalar(golden, policy.assist.step(30., 80., 0., false, .01));
        auto restored = policy.snapshot(); restored.assist.last_gain = 123.; policy.restore(restored);
        const auto before = state_text(policy);
        overflow([&] { (void)policy.assist.step(11., cadence, 0., false, .01, std::nullopt,
            shaft_override ? std::optional<double>(80.) : std::nullopt); });
        bt::require(state_text(policy) == before, "crank-derived overflow cannot publish gain or torque");
        // The source records the Python torque-vs-previous assertion before
        // comparing the native whole snapshot; consume both in that order.
        scalar(golden, policy.assist.state().torque); tree(golden, state(policy));
        scalar(golden, policy.assist.step(20., 80., 0., false, .01)); tree(golden, state(policy)); golden.done();
    }
}

// tests/reference/test_native_drive_policies.py::test_assist_braking_resets_before_computed_crank_overflow
void test_assist_braking_resets_before_computed_crank_overflow(const Value& baseline) {
    auto golden = node("test_assist_braking_resets_before_computed_crank_overflow"); tree(golden, baseline);
    auto policy = policies(baseline);
    scalar(golden, policy.assist.step(30., 80., 0., false, .01));
    scalar(golden, policy.assist.step(30., 1e308, 0., true, .01));
    tree(golden, state(policy)); golden.done();
}
} // namespace

int main() { return bt::run([] {
    const auto baseline = policy_reference::config(policy_reference::source());
    test_native_policy_api_exists(baseline);
    test_crank_torque();
    test_battery_draw_matches_python(baseline);
    test_pedaling_sequences(baseline);
    test_shifting();
    test_assist();
    test_energy_helpers(baseline);
    test_freehub(baseline);
    test_replay_reset_and_ownership(baseline);
    test_invalid_restore_is_atomic(baseline);
    test_invalid_restore_values(baseline);
    test_config_validation(baseline);
    test_projection_resolves_runtime();
    test_projection_normalizes_numeric_leaves();
    test_complete_reset_and_atomic_multi_policy_restore(baseline);
    test_restored_finite_snapshots_preserve_bits(baseline);
    test_disabled_shifter_initial_teeth_outside_cassette();
    test_invalid_config_values(baseline);
    test_missing_config_sections(baseline);
    test_random_curve_interpolation(baseline);
    test_battery_nonzero_idle_remainder(baseline);
    test_config_and_python_object_deletion();
    test_pedaling_parameter_branches();
    test_optional_config_sections(baseline);
    test_extra_and_missing_state_sections(baseline);
    test_invalid_calls_preserve_state(baseline);
    test_invalid_human_torque();
    test_shifter_count_boundary_rejects_successful_increment_atomically(baseline);
    test_unrepresentable_count_restore_is_atomic(baseline);
    test_assist_computed_crank_overflow_matches_python_state();
    test_assist_braking_resets_before_computed_crank_overflow(baseline);
}); }
