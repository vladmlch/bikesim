#include "../src/drivetrain/pedaling.hpp"
#include "../src/drivetrain/shifting.hpp"
#include "../src/drivetrain/motor.hpp"
#include "../src/drivetrain/freehub.hpp"
#include "../src/drivetrain/transmission.hpp"
#include "../src/binding_arrays.hpp"
#include "../src/engine_call.hpp"
#include "../src/engine_abi_312.hpp"
#include "../src/diag.hpp"
#include "../src/interval_clock.hpp"
#include "../src/writers/cruise.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cfenv>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
    using TestFunction = void (*)();

    struct TestCase {
        std::string_view name;
        TestFunction run;
    };

    void require(bool condition, std::string_view message) {
        if (!condition) throw std::runtime_error(std::string(message));
    }

    void test_human_crank_torque() {
        require(drivetrain::human_crank_torque(50., 0., .35) == 32.5,
                "human_crank_torque at phase zero");
        require(std::abs(drivetrain::human_crank_torque(50., std::numbers::pi / 2., .35) - 67.5) < 1e-12,
                "human_crank_torque at half turn");
    }

    void test_pedaling_policy_valid_transition() {
        const drivetrain::PedalingConfig config{
            .enabled = true,
            .coast_above_rpm = 110.,
            .resume_below_rpm = 80.,
            .stop_time_s = 1.,
            .coast_cadence_tau_s = 0.,
            .mash_cadence_rpm = 25.,
            .mash_torque_nm = 25.,
            .effort_slew_nm_s = 0.,
        };
        drivetrain::PedalingPolicy policy(config);
        const drivetrain::PedalingState state = policy.update(0., 0., 30., 20., .01);
        require(state.mode == "pedaling", "pedaling policy mode");
        require(state.effort_nm == 20., "pedaling policy effort");
        require(!state.target_phase_rad, "pedaling policy target phase");
    }

    // C1: invokes a callable, records that exactly std::invalid_argument
    // escapes, and fails through require if it returns or throws another type.
    template<class F>
    void require_throws_invalid_argument(const F &f, std::string_view message) {
        try {
            f();
        } catch (const std::invalid_argument &) {
            return;
        } catch (...) {
            require(false, message);
        }
        require(false, message);
    }

    drivetrain::PedalingConfig valid_pedaling_config() {
        return {
            .enabled = true, .coast_above_rpm = 110., .resume_below_rpm = 80.,
            .stop_time_s = 1., .coast_cadence_tau_s = 0., .mash_cadence_rpm = 25.,
            .mash_torque_nm = 25., .effort_slew_nm_s = 0.
        };
    }

    drivetrain::GearingConfig valid_gearing_config() {
        return {.front_teeth = 34, .rear_teeth = 18, .chain_pitch_m = .0127};
    }

    drivetrain::ShiftingConfig valid_shifting_config() {
        return {
            .enabled = true, .cassette = {11, 13, 15, 18, 21, 24, 28},
            .target_cadence_min_rpm = 70., .target_cadence_max_rpm = 95.,
            .shift_cooldown_s = .5, .shift_cut_duration_s = .15,
            .torque_factor = .5, .cadence_smoothing_tau_s = .3,
            .upshift_slip_limit_mps = .05, .upshift_slip_mode = "legacy_signed"
        };
    }

    drivetrain::AssistConfig valid_assist_config() {
        return {
            .gain = 1.5, .max_torque = 90., .max_power = 600., .tau = .08,
            .slew = 400., .engage_torque_nm = 5., .gate_min_crank_rad_s = 1.,
            .cutoff_mps = 11.1, .taper_width_mps = 1.5,
            .torque_curve = std::nullopt, .profile = std::nullopt, .mode = "turbo"
        };
    }

    void test_pedaling_ctor_domain() {
        {   // Valid control (same fields the Python suite parses).
            const drivetrain::PedalingPolicy policy(valid_pedaling_config());
            require(!policy.state().coasting && !policy.state().cadence_ema,
                    "fresh pedaling policy starts from the reset state");
        }
        const auto invalid = [](auto mutate) {
            auto config = valid_pedaling_config();
            mutate(config);
            require_throws_invalid_argument(
                [&] { [[maybe_unused]] const drivetrain::PedalingPolicy p(config); },
                "PedalingPolicy ctor must reject the mutated config");
        };
        invalid([](auto &c) { c.stop_time_s = 0.; });
        invalid([](auto &c) { c.stop_time_s = -1.; });
        invalid([](auto &c) { c.stop_time_s = std::numeric_limits<double>::quiet_NaN(); });
        invalid([](auto &c) { c.coast_above_rpm = 0.; });
        invalid([](auto &c) { c.coast_above_rpm = std::numeric_limits<double>::infinity(); });
        invalid([](auto &c) { c.resume_below_rpm = c.coast_above_rpm; });
        invalid([](auto &c) { c.resume_below_rpm = -1.; });
        invalid([](auto &c) { c.coast_cadence_tau_s = -1.; });
        invalid([](auto &c) { c.mash_torque_nm = -1.; });
        invalid([](auto &c) { c.mash_cadence_rpm = -1.; });
        invalid([](auto &c) { c.mash_cadence_rpm = 0.; }); // nonzero mash torque needs cadence
        invalid([](auto &c) { c.effort_slew_nm_s = -1.; });
        {   // Zero-disable control: slew, coast EMA tau and the mash feature
            // all accept 0 as "off" (mash cadence may be 0 only with torque 0).
            auto config = valid_pedaling_config();
            config.effort_slew_nm_s = 0.;
            config.coast_cadence_tau_s = 0.;
            config.mash_torque_nm = 0.;
            config.mash_cadence_rpm = 0.;
            const drivetrain::PedalingPolicy policy(config);
            require(!policy.state().coasting, "zero-disable pedaling config must construct");
        }
    }

    void test_shifter_ctor_domain() {
        {   // Valid control: enabled auto-shifter on a cassette gear.
            const drivetrain::CadenceShifter shifter(valid_gearing_config(), valid_shifting_config());
            require(shifter.state().rear_teeth == 18,
                    "fresh shifter sits on the configured rear gear");
        }
        {   // Valid control: unique teeth are the domain; ordering is parser
            // normalization (policy_binding.cpp sorts the wire cassette).
            auto shifting = valid_shifting_config();
            shifting.cassette = {28, 11, 21, 18, 13, 15, 24};
            const drivetrain::CadenceShifter shifter(valid_gearing_config(), shifting);
            require(shifter.state().rear_teeth == 18,
                    "unsorted but unique cassette stays in the valid domain");
        }
        const auto invalid_gearing = [](auto mutate) {
            auto gearing = valid_gearing_config();
            mutate(gearing);
            require_throws_invalid_argument(
                [&] {
                    [[maybe_unused]] const drivetrain::CadenceShifter s(gearing, valid_shifting_config());
                },
                "CadenceShifter ctor must reject an invalid gearing config");
        };
        invalid_gearing([](auto &g) { g.rear_teeth = 0; });
        invalid_gearing([](auto &g) { g.rear_teeth = 2; });
        invalid_gearing([](auto &g) { g.front_teeth = -3; });
        invalid_gearing([](auto &g) { g.chain_pitch_m = 0.; });
        invalid_gearing([](auto &g) { g.chain_pitch_m = std::numeric_limits<double>::quiet_NaN(); });
        const auto invalid_shifting = [](auto mutate) {
            auto shifting = valid_shifting_config();
            mutate(shifting);
            require_throws_invalid_argument(
                [&] {
                    [[maybe_unused]] const drivetrain::CadenceShifter s(valid_gearing_config(), shifting);
                },
                "CadenceShifter ctor must reject an invalid shifting config");
        };
        invalid_shifting([](auto &s) { s.cassette.clear(); });
        invalid_shifting([](auto &s) { s.cassette = {11, 18, 18, 24}; });       // duplicate teeth
        invalid_shifting([](auto &s) { s.cassette = {11, 18, 2}; });           // tooth below 3
        invalid_shifting([](auto &s) { s.cassette = {11, 13, 21}; });          // enabled gear 18 absent
        invalid_shifting([](auto &s) { s.target_cadence_max_rpm = s.target_cadence_min_rpm; });
        invalid_shifting([](auto &s) { s.target_cadence_min_rpm = 0.; });
        invalid_shifting([](auto &s) { s.shift_cut_duration_s = s.shift_cooldown_s + 1.; });
        invalid_shifting([](auto &s) { s.torque_factor = 1.5; });
        invalid_shifting([](auto &s) { s.shift_cooldown_s = -1.; });
        invalid_shifting([](auto &s) { s.cadence_smoothing_tau_s = -1.; });
        invalid_shifting([](auto &s) {
            s.upshift_slip_limit_mps = std::numeric_limits<double>::infinity(); });
        invalid_shifting([](auto &s) { s.upshift_slip_mode = "mystery"; });
        {   // Zero-disable control: cooldown/cut/smoothing/slip-limit 0 is legal.
            auto shifting = valid_shifting_config();
            shifting.shift_cooldown_s = 0.;
            shifting.shift_cut_duration_s = 0.;
            shifting.cadence_smoothing_tau_s = 0.;
            shifting.upshift_slip_limit_mps = 0.;
            const drivetrain::CadenceShifter shifter(valid_gearing_config(), shifting);
            require(shifter.state().rear_teeth == 18,
                    "zero-disable shifting config must construct");
        }
        {   // Disabled shifter skips the cassette-membership requirement.
            auto gearing = valid_gearing_config();
            auto shifting = valid_shifting_config();
            gearing.rear_teeth = 34;
            shifting.cassette = {11, 13, 21};
            shifting.enabled = false;
            const drivetrain::CadenceShifter shifter(gearing, shifting);
            require(shifter.state().rear_teeth == 34,
                    "disabled shifter accepts a gear outside the cassette");
        }
    }

    // One rejection check per call; keeping the mutated construction inside
    // a helper stops the test frame from accumulating every case's config.
    template<class Mutate>
    void require_invalid_assist_config(Mutate mutate) {
        auto config = valid_assist_config();
        mutate(config);
        require_throws_invalid_argument(
            [&] { [[maybe_unused]] const drivetrain::AssistController c(config); },
            "AssistController ctor must reject the mutated config");
    }

    void assist_ctor_scalar_domain() {
        require_invalid_assist_config([](auto &c) { c.tau = 0.; });
        require_invalid_assist_config([](auto &c) { c.tau = -1.; });
        require_invalid_assist_config([](auto &c) { c.slew = 0.; });
        require_invalid_assist_config([](auto &c) { c.slew = -1.; });
        require_invalid_assist_config([](auto &c) { c.taper_width_mps = 0.; });
        require_invalid_assist_config([](auto &c) { c.taper_width_mps = c.cutoff_mps + 1.; });
        require_invalid_assist_config([](auto &c) { c.gain = -1.; });
        require_invalid_assist_config([](auto &c) { c.max_torque = -1.; });
        require_invalid_assist_config([](auto &c) { c.max_power = std::numeric_limits<double>::quiet_NaN(); });
        require_invalid_assist_config([](auto &c) { c.engage_torque_nm = -1.; });
        require_invalid_assist_config([](auto &c) { c.gate_min_crank_rad_s = -1.; });
        require_invalid_assist_config([](auto &c) { c.cutoff_mps = -1.; });
    }

    void assist_ctor_curve_domain() {
        require_invalid_assist_config([](auto &c) {
            c.torque_curve = std::vector<std::array<double, 2>>{}; });
        require_invalid_assist_config([](auto &c) {
            c.torque_curve = std::vector<std::array<double, 2>>{{60., 50.}}; });
        require_invalid_assist_config([](auto &c) {
            c.torque_curve = std::vector<std::array<double, 2>>{{120., 40.}, {60., 90.}}; });
        require_invalid_assist_config([](auto &c) {
            c.torque_curve = std::vector<std::array<double, 2>>{{60., 90.}, {60., 50.}}; });
        require_invalid_assist_config([](auto &c) {
            c.torque_curve = std::vector<std::array<double, 2>>{{-5., 90.}, {60., 50.}}; });
        require_invalid_assist_config([](auto &c) {
            c.torque_curve = std::vector<std::array<double, 2>>{{0., -1.}, {60., 50.}}; });
        require_invalid_assist_config([](auto &c) {
            c.torque_curve = std::vector<std::array<double, 2>>{
                {0., std::numeric_limits<double>::quiet_NaN()}, {60., 50.}}; });
    }

    void assist_ctor_profile_domain() {
        require_invalid_assist_config([](auto &c) {
            c.profile = drivetrain::MotorProfile{
                .eco = .5, .tour = 1., .emtb_low = 2., .emtb_high = 1.,
                .turbo = 2.5, .emtb_full_gain_at_nm = 50.};
            c.mode = "emtb"; });
        require_invalid_assist_config([](auto &c) {
            c.profile = drivetrain::MotorProfile{
                .eco = .5, .tour = 1., .emtb_low = .8, .emtb_high = 2.,
                .turbo = 2.5, .emtb_full_gain_at_nm = 50.};
            c.mode = "supersport"; });
        require_invalid_assist_config([](auto &c) {
            c.profile = drivetrain::MotorProfile{
                .eco = .5, .tour = 1., .emtb_low = .8, .emtb_high = 2.,
                .turbo = 2.5, .emtb_full_gain_at_nm = 0.};
            c.mode = "turbo"; });
    }

    // The inlined mutation cases keep every config's exception-cleanup slot
    // live, so this one frame legitimately exceeds the 8KB guard whose target
    // is the per-step realtime path — same rationale as the binding glue.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wframe-larger-than")
    void test_assist_ctor_domain() {
        {   // Valid control: empty-optional torque curve and no profile.
            const drivetrain::AssistController controller(valid_assist_config());
            require(controller.state().torque == 0. && !controller.state().pedaling,
                    "fresh assist controller starts empty");
        }
        {   // Valid control: a sorted two-row torque curve.
            auto config = valid_assist_config();
            config.torque_curve = std::vector<std::array<double, 2>>{{0., 90.}, {120., 40.}};
            const drivetrain::AssistController controller(config);
            require(controller.state().last_gain == 0., "torque-curve config must construct");
        }
        assist_ctor_scalar_domain();
        assist_ctor_curve_domain();
        assist_ctor_profile_domain();
        {   // Valid control: profile engaged with a supported mode.
            auto config = valid_assist_config();
            config.profile = drivetrain::MotorProfile{
                .eco = .5, .tour = 1., .emtb_low = .8, .emtb_high = 2.,
                .turbo = 2.5, .emtb_full_gain_at_nm = 50.};
            config.mode = "emtb";
            const drivetrain::AssistController controller(config);
            require(!controller.state().pedaling, "profiled assist config must construct");
        }
    }
    NATIVE_DIAG_POP

    void test_battery_ctor_domain() {
        {
            const drivetrain::Battery battery(500.);
            require(battery.state().energy_j == 500. &&
                    battery.state().initial_energy_j == 500. &&
                    battery.state().drawn_energy_j == 0.,
                    "battery starts from its configured store");
        }
        {   // Zero-disable control: an empty store is a legal (nonnegative) config.
            const drivetrain::Battery battery(0.);
            require(battery.state().energy_j == 0., "empty battery store is valid");
        }
        require_throws_invalid_argument(
            [] { [[maybe_unused]] const drivetrain::Battery b(-1.); },
            "negative battery energy must be rejected");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const drivetrain::Battery b(
                    std::numeric_limits<double>::quiet_NaN());
            },
            "nonfinite battery energy must be rejected");
    }

    void test_freehub_ctor_domain() {
        {
            const drivetrain::Freehub hub(3000., 15.);
            require(hub.state().torque_nm == 0. && !hub.state().boundary,
                    "fresh freehub starts disengaged");
        }
        {   // Zero-disable control: an undamped freehub is legal.
            const drivetrain::Freehub hub(3000., 0.);
            require(hub.state().energy_j == 0., "zero freehub damping is valid");
        }
        require_throws_invalid_argument(
            [] { [[maybe_unused]] const drivetrain::Freehub h(0., 1.); },
            "zero freehub stiffness must be rejected");
        require_throws_invalid_argument(
            [] { [[maybe_unused]] const drivetrain::Freehub h(-1., 1.); },
            "negative freehub stiffness must be rejected");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const drivetrain::Freehub h(
                    std::numeric_limits<double>::quiet_NaN(), 1.);
            },
            "nonfinite freehub stiffness must be rejected");
        require_throws_invalid_argument(
            [] { [[maybe_unused]] const drivetrain::Freehub h(1., -1.); },
            "negative freehub damping must be rejected");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const drivetrain::Freehub h(
                    1., std::numeric_limits<double>::infinity());
            },
            "nonfinite freehub damping must be rejected");
    }

    void test_writer_config_domains() {
        // The model-carrying writer ctors validate cfg_ with these exact
        // overloads as their first config action (writers/resistance.cpp,
        // brake.cpp, rider_forces.cpp, suspension.cpp) — they cannot be
        // instantiated here without an mjModel, so the config domain is
        // exercised through the same calls their constructors make.
        {
            const nativecfg::BrakeConfig config{.torque_ceiling_nm = 200., .taper_radps = 5.};
            nativecfg::validate(config);
            auto zero = config;             // zero ceiling is a legal disable.
            zero.torque_ceiling_nm = 0.;
            nativecfg::validate(zero);
            auto negative = config;
            negative.torque_ceiling_nm = -1.;
            require_throws_invalid_argument([&] { nativecfg::validate(negative); },
                                            "negative brake ceiling");
            auto flat = config;
            flat.taper_radps = 0.;
            require_throws_invalid_argument([&] { nativecfg::validate(flat); },
                                            "zero brake taper");
            auto nonfinite = config;
            nonfinite.taper_radps = std::numeric_limits<double>::quiet_NaN();
            require_throws_invalid_argument([&] { nativecfg::validate(nonfinite); },
                                            "nonfinite brake taper");
        }
        {   // Air (drag) writer fields.
            const nativecfg::ResistanceConfig config{
                .crr = .005, .rolling_taper_rad_s = 3., .rho_kg_m3 = 1.225,
                .cda_m2 = .4, .wind_world_mps = {0., 0., 0.},
                .point_body_m = {.1, 0., .05}, .frame_body = "frame",
                .front_wheel_body = "front_wheel", .rear_wheel_body = "rear_wheel"};
            nativecfg::validate(config);
            const auto invalid = [&config](auto mutate) {
                auto copy = config;
                mutate(copy);
                require_throws_invalid_argument(
                    [&] { nativecfg::validate(copy); },
                    "resistance config must reject the mutated field");
            };
            invalid([](auto &c) { c.crr = -.1; });
            invalid([](auto &c) { c.crr = std::numeric_limits<double>::quiet_NaN(); });
            invalid([](auto &c) { c.rho_kg_m3 = std::numeric_limits<double>::quiet_NaN(); });
            invalid([](auto &c) { c.cda_m2 = -.1; });
            invalid([](auto &c) { c.rolling_taper_rad_s = 0.; });
            invalid([](auto &c) { c.wind_world_mps[0] = std::numeric_limits<double>::quiet_NaN(); });
            invalid([](auto &c) { c.wind_world_mps[1] = .1; });           // planar gate
            invalid([](auto &c) { c.point_body_m[1] = .1; });             // planar gate
            invalid([](auto &c) { c.point_body_m[2] = std::numeric_limits<double>::infinity(); });
        }
        {   // Rider-force path fields.
            const nativecfg::RiderForcesConfig config{
                .paths = {{
                    .joint = "saddle_z", .stiffness_n_m = 5000.,
                    .damping_ns_m = 50., .preload_deflection_m = .01,
                    .offset_m = 0., .unilateral = true}}};
            nativecfg::validate(config);
            {   // Empty path list is the inert pose=None control.
                const nativecfg::RiderForcesConfig empty{.paths = {}};
                nativecfg::validate(empty);
            }
            const auto invalid = [&config](auto mutate) {
                auto copy = config;
                mutate(copy.paths.front());
                require_throws_invalid_argument(
                    [&] { nativecfg::validate(copy); },
                    "rider path config must reject the mutated field");
            };
            invalid([](auto &p) { p.stiffness_n_m = -1.; });
            invalid([](auto &p) { p.damping_ns_m = -1.; });
            invalid([](auto &p) { p.preload_deflection_m = -1.; });
            invalid([](auto &p) { p.offset_m = std::numeric_limits<double>::quiet_NaN(); });
        }
        {   // Cruise gains/ceiling positive, target speed finite in [15, 45].
            const nativecfg::CruiseConfig config{
                .target_speed_kmh = 30., .kp_nm_per_mps = 30.,
                .ki_nm_per_mps_s = 10., .torque_ceiling_nm = 120.};
            nativecfg::validate(config);
            const auto invalid = [&config](auto mutate) {
                auto copy = config;
                mutate(copy);
                require_throws_invalid_argument(
                    [&] { nativecfg::validate(copy); },
                    "cruise config must reject the mutated field");
            };
            invalid([](auto &c) { c.kp_nm_per_mps = 0.; });
            invalid([](auto &c) { c.ki_nm_per_mps_s = -1.; });
            invalid([](auto &c) { c.torque_ceiling_nm = 0.; });
            invalid([](auto &c) { c.target_speed_kmh = 14.9; });
            invalid([](auto &c) { c.target_speed_kmh = 45.1; });
            invalid([](auto &c) {
                c.target_speed_kmh = std::numeric_limits<double>::quiet_NaN(); });
        }
        {   // Damper maxima: unordered bounds and negative maxima both fail.
            const nativecfg::DamperCore core{
                .max_hsc = 12, .max_lsc = 12, .max_reb = 20, .hsc_clicks = 0,
                .lsc_clicks = 0, .rebound_clicks = 0, .c_lsc_min = 1.,
                .c_lsc_max = 10., .c_hsc_min = 2., .c_hsc_max = 20.,
                .c_reb_min = 1., .c_reb_max = 15., .v_knee_comp = 1.,
                .v_knee_reb = 1.};
            nativecfg::validate(core);
            auto low_max = core;
            low_max.c_lsc_max = low_max.c_lsc_min - 1.;
            require_throws_invalid_argument([&] { nativecfg::validate(low_max); },
                                            "unordered LSC damping bounds");
            auto neg_max = core;
            neg_max.c_hsc_max = -1.;
            require_throws_invalid_argument([&] { nativecfg::validate(neg_max); },
                                            "negative HSC damping maximum");
            auto nan_max = core;
            nan_max.c_reb_max = std::numeric_limits<double>::quiet_NaN();
            require_throws_invalid_argument([&] { nativecfg::validate(nan_max); },
                                            "nonfinite rebound damping maximum");
        }
    }

    void test_transmission_lifecycle_invariants() {
        // C1: value semantics are deleted — ownership stays unique_ptr and a
        // moved-from transmission cannot exist by construction.
        static_assert(!std::is_copy_constructible_v<drivetrain::Transmission>);
        static_assert(!std::is_copy_assignable_v<drivetrain::Transmission>);
        static_assert(!std::is_move_constructible_v<drivetrain::Transmission>);
        static_assert(!std::is_move_assignable_v<drivetrain::Transmission>);
        static_assert(std::is_move_constructible_v<
                      std::unique_ptr<drivetrain::Transmission>>);
        // Declared reset states: the snapshot optionals start disengaged and
        // absent_id is the named invalid ID (never the world body's 0).
        const drivetrain::TransmissionSnapshot fresh;
        require(!fresh.boundary && !fresh.prepared && !fresh.shift_pending,
                "transmission snapshot optionals default to disengaged");
        require(drivetrain::absent_id == -1,
                "absent_id is the mj_name2id miss sentinel");
    }

    // C1 acceptance: the typed-ctor domain and the wire-parsed domain agree.
    // parse_drive_policy_config (drivetrain/policy_binding.cpp) enforces the
    // same per-field predicates (positive/nonnegative, relational bands,
    // unique cassette teeth, ordered torque curves), then DrivePolicies'
    // ctor invokes the same drivetrain::validate overloads the typed ctors
    // call — so ctor acceptance must equal validator acceptance here.
    void test_typed_ctor_matches_validator_domain() {
        const auto validator = [](const auto &check) {
            try {
                check();
            } catch (const std::invalid_argument &) {
                return false;
            }
            return true;
        };
        const std::array pedaling_cases{
            valid_pedaling_config(), [] {
                auto c = valid_pedaling_config();
                c.stop_time_s = 0.;
                return c;
            }(), [] {
                auto c = valid_pedaling_config();
                c.mash_torque_nm = 0.;
                c.mash_cadence_rpm = 0.;
                return c;
            }(), [] {
                auto c = valid_pedaling_config();
                c.resume_below_rpm = 200.;
                return c;
            }()};
        for (const auto &config: pedaling_cases) {
            const bool validator_ok =
                    validator([&] { drivetrain::validate(config); });
            const bool ctor_ok = validator([&] {
                [[maybe_unused]] const drivetrain::PedalingPolicy p(config);
            });
            require(validator_ok == ctor_ok,
                    "PedalingPolicy ctor domain must equal validate() domain");
        }
        const std::array assist_cases{
            valid_assist_config(), [] {
                auto c = valid_assist_config();
                c.torque_curve = std::vector<std::array<double, 2>>{{0., 90.}, {120., 40.}};
                return c;
            }(), [] {
                auto c = valid_assist_config();
                c.slew = 0.;
                return c;
            }(), [] {
                auto c = valid_assist_config();
                c.taper_width_mps = 99.;
                return c;
            }()};
        for (const auto &config: assist_cases) {
            const bool validator_ok =
                    validator([&] { drivetrain::validate(config); });
            const bool ctor_ok = validator([&] {
                [[maybe_unused]] const drivetrain::AssistController c(config);
            });
            require(validator_ok == ctor_ok,
                    "AssistController ctor domain must equal validate() domain");
        }
        // CadenceShifter's ctor domain = validate(gearing) + validate(shifting)
        // + the enabled-membership relation, mirrored here as the spec check.
        const auto shift_spec = [](const drivetrain::GearingConfig &g,
                                   const drivetrain::ShiftingConfig &s) {
            drivetrain::validate(g);
            drivetrain::validate(s);
            if (s.enabled &&
                std::ranges::find(s.cassette, g.rear_teeth) == s.cassette.end())
                throw std::invalid_argument("enabled gear must belong to cassette");
        };
        const std::array shift_cases{
            std::pair{valid_gearing_config(), valid_shifting_config()}, [] {
                auto g = valid_gearing_config();
                g.rear_teeth = 0;
                return std::pair{g, valid_shifting_config()};
            }(), [] {
                auto s = valid_shifting_config();
                s.cassette.clear();
                return std::pair{valid_gearing_config(), s};
            }(), [] {
                auto s = valid_shifting_config();
                s.cassette = {11, 13, 21};
                return std::pair{valid_gearing_config(), s};
            }()};
        for (const auto &[gearing, shifting]: shift_cases) {
            const bool spec_ok = validator([&] { shift_spec(gearing, shifting); });
            const bool ctor_ok = validator([&] {
                [[maybe_unused]] const drivetrain::CadenceShifter s(gearing, shifting);
            });
            require(spec_ok == ctor_ok,
                    "CadenceShifter ctor domain must equal its spec domain");
        }
    }

    struct FatalContext { const char *message; };
    // Operation uses the required mutable void* ABI despite reading its context.
    // NOLINTNEXTLINE(misc-const-correctness)
    void fatal_operation(void *raw) noexcept {
        const auto *context = static_cast<const FatalContext *>(raw);
        // This case exercises MuJoCo's real variadic fatal-error entry point.
        // Every context.message below names a NUL-terminated string literal.
#if defined(__clang__)
#pragma clang diagnostic push
#if __has_warning("-Wunsafe-buffer-usage-in-format-attr-call")
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-format-attr-call"
#endif
#endif
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,clang-diagnostic-unsafe-buffer-usage-in-format-attr-call)
        mju_error("%s", context->message);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    }

    void no_operation(void *) noexcept {}

    void test_engine_fatal_status_and_reuse() {
        engine::ErrorBuffer error;
        FatalContext context{.message = "contract fatal A"};
        require(!engine::invoke(&fatal_operation, &context, error),
                "fatal operation must return failure status");
        require(error.kind == engine::ErrorKind::fatal, "fatal status kind");
        require(std::string_view(error.message.data()).find(context.message) != std::string_view::npos,
                "fatal message survives C frames");
        require(engine::invoke(&no_operation, nullptr, error),
                "handler must be reusable after a fatal error");
        require(error.kind == engine::ErrorKind::none, "successful status clears fatal kind");
    }

    struct NestedContext { bool inner_failed = false; };
    void nested_operation(void *raw) noexcept {
        auto *context = static_cast<NestedContext *>(raw);
        engine::ErrorBuffer inner;
        FatalContext fatal{.message = "nested fatal"};
        context->inner_failed =
            !engine::invoke(&fatal_operation, &fatal, inner) &&
            inner.kind == engine::ErrorKind::fatal;
    }

    void test_nested_engine_frames() {
        NestedContext context;
        engine::ErrorBuffer outer;
        require(engine::invoke(&nested_operation, &context, outer),
                "outer engine operation survives inner fatal error");
        require(context.inner_failed, "inner fatal error reached its own frame");
    }

    void test_thread_local_engine_frames() {
        engine::ErrorBuffer first;
        engine::ErrorBuffer second;
        FatalContext first_context{.message = "thread fatal A"};
        FatalContext second_context{.message = "thread fatal B"};
        bool first_failed = false;
        bool second_failed = false;
        std::thread first_thread([&] {
            first_failed = !engine::invoke(&fatal_operation, &first_context, first);
        });
        std::thread second_thread([&] {
            second_failed = !engine::invoke(&fatal_operation, &second_context, second);
        });
        first_thread.join();
        second_thread.join();
        require(first_failed && second_failed, "both fatal errors must be caught");
        require(std::string_view(first.message.data()).find(first_context.message) != std::string_view::npos,
                "first thread must keep its own error");
        require(std::string_view(second.message.data()).find(second_context.message) != std::string_view::npos,
                "second thread must keep its own error");
    }

    // Thread-local counter observes forwarding without shared mutable state.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    thread_local int forwarded_warnings = 0;
    void warning_handler(const mjLogMessage *message) {
        if (message->level == mjLOG_WARNING) ++forwarded_warnings;
    }
    void warning_operation(void *) noexcept {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        mju_warning("contract warning");
    }

    void test_previous_tls_handler_restored() {
        forwarded_warnings = 0;
        const mjfLogHandler original = _mjPRIVATE_setTlsLogHandler(&warning_handler);
        engine::ErrorBuffer error;
        const bool succeeded = engine::invoke(&warning_operation, nullptr, error);
        const mjfLogHandler restored = _mjPRIVATE_setTlsLogHandler(original);
        require(succeeded, "warning operation must succeed");
        require(forwarded_warnings == 1, "nonfatal warning must reach prior handler");
        require(restored == &warning_handler, "prior TLS handler must be restored");
    }

    // F4 capsule-creation-failure seam. Production owned arrays stage the
    // buffer through wire::detail::release_with_owner (binding_arrays.hpp):
    // the owner is built while the unique_ptr still holds the allocation,
    // so a throwing owner frees instead of leaking. nb::capsule cannot run
    // in this Python-free executable, so OwnerSeam follows the same
    // acquire/deleter protocol and can be armed to throw exactly where
    // PyCapsule_New could fail.
    struct TrackedCell {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) test-local live-instance counter
        static inline int live = 0;
        TrackedCell() { ++live; }
        TrackedCell(const TrackedCell &) = delete;
        TrackedCell &operator=(const TrackedCell &) = delete;
        TrackedCell(TrackedCell &&) = delete;
        TrackedCell &operator=(TrackedCell &&) = delete;
        ~TrackedCell() { --live; }
    };
    void delete_cell(void *pointer) noexcept {
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) seam deleter owns the staged cell
        delete static_cast<TrackedCell *>(pointer);
    }
    void delete_cells(void *pointer) noexcept {
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) seam deleter owns the staged array
        delete[] static_cast<TrackedCell *>(pointer);
    }
    struct OwnerSeam {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) armed only inside the case
        static inline bool armed = false;
        void *pointer = nullptr;
        void (*deleter)(void *) noexcept = nullptr;
        OwnerSeam(void *p, void (*d)(void *) noexcept)
            : pointer(p), deleter(d) {
            if (armed)
                throw std::runtime_error("seamed owner construction failure");
        }
        OwnerSeam(const OwnerSeam &) = delete;
        OwnerSeam &operator=(const OwnerSeam &) = delete;
        OwnerSeam(OwnerSeam &&other) noexcept
            : pointer(std::exchange(other.pointer, nullptr)),
              deleter(std::exchange(other.deleter, nullptr)) {}
        OwnerSeam &operator=(OwnerSeam &&) = delete;
        ~OwnerSeam() { if (pointer) deleter(pointer); }
    };
    template <typename Stage>
    void require_owner_failure_frees(const Stage &stage, int &calls) {
        try {
            static_cast<void>(stage());
        } catch (const std::runtime_error &) {
            ++calls;
        }
    }
    void test_owned_staging_owner_failure_frees_storage() {
        const auto single_stage = [] {
            return wire::detail::release_with_owner(
                std::make_unique<TrackedCell>(),
                [](TrackedCell *cell) { return OwnerSeam(cell, &delete_cell); });
        };
        const auto array_stage = [] {
            // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) seam exercises the delete[] staging path
            auto cells = std::make_unique<TrackedCell[]>(2);
            return wire::detail::release_with_owner(
                std::move(cells),
                [](TrackedCell *array) { return OwnerSeam(array, &delete_cells); });
        };
        OwnerSeam::armed = true;
        int calls = 0;
        require_owner_failure_frees(single_stage, calls);
        require(calls == 1 && TrackedCell::live == 0,
                "single-object owner failure must free staged storage");
        require_owner_failure_frees(array_stage, calls);
        require(calls == 2 && TrackedCell::live == 0,
                "array owner failure must free staged storage");
        OwnerSeam::armed = false;
        {
            const auto &[data, owner] = single_stage();
            require(data != nullptr && TrackedCell::live == 1,
                    "successful staging releases storage to the owner");
        }
        require(TrackedCell::live == 0,
                "owner destruction must run the single-object deleter");
        {
            const auto &[data, owner] = array_stage();
            require(data != nullptr && TrackedCell::live == 2,
                    "successful array staging releases storage to the owner");
        }
        require(TrackedCell::live == 0,
                "array owner destruction must run the deleter");
    }

    void *fail_allocation(std::size_t) noexcept { return nullptr; }
    struct AllocationFailureContext {
        const char *path = nullptr;
        const mjModel *model = nullptr;
        mjModel *loaded = nullptr;
        mjData *data = nullptr;
    };
    void load_with_failed_allocation(void *raw) noexcept {
        auto *context = static_cast<AllocationFailureContext *>(raw);
        mju_user_malloc = &fail_allocation;
        context->loaded = mj_loadModel(context->path, nullptr);
    }
    void make_data_with_failed_allocation(void *raw) noexcept {
        auto *context = static_cast<AllocationFailureContext *>(raw);
        mju_user_malloc = &fail_allocation;
        context->data = mj_makeData(context->model);
    }

    int run_allocation_failure(const char *path, bool during_make_data) {
        try {
            const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                during_make_data ? engine::load_model(path) : nullptr,
                &mj_deleteModel);
            if (during_make_data) require(model != nullptr, "valid control model load");
            AllocationFailureContext context{
                .path = path, .model = model.get()
            };
            engine::ErrorBuffer error;
            const auto previous_allocator = mju_user_malloc;
            const bool succeeded = engine::invoke(
                during_make_data ? &make_data_with_failed_allocation
                                 : &load_with_failed_allocation,
                &context, error);
            mju_user_malloc = previous_allocator;
            if (context.data) mj_deleteData(context.data);
            if (context.loaded) mj_deleteModel(context.loaded);
            require(!succeeded, "MuJoCo allocation failure must be intercepted");
            require(error.kind == engine::ErrorKind::fatal,
                    "allocation failure must have fatal status");
            require(std::string_view(error.message.data()).find("Could not allocate memory") !=
                        std::string_view::npos,
                    "allocation failure must preserve engine message");
            std::cout << "PASS " << (during_make_data ? "makeData" : "loadModel")
                      << " fatal allocation\n";
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL fatal allocation: " << error.what() << '\n';
            return 1;
        }
    }

    struct LateAllocationState {
        int calls = 0;
        int outstanding = 0;
        std::array<void *, 8> pointers{};
    };
    // The process-isolated allocation probe installs this only inside invoke.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    thread_local LateAllocationState *late_allocation = nullptr;
    void *late_fail_allocation(std::size_t size) noexcept {
        auto &state = *late_allocation;
        if (++state.calls == 2) return nullptr;
        const std::size_t rounded = (size + 63U) & ~std::size_t{63U};
        // MuJoCo's allocator callback transfers ownership through void*.
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
        void *pointer = std::aligned_alloc(64, rounded);
        if (pointer != nullptr) {
            state.pointers[static_cast<std::size_t>(state.outstanding)] = pointer;
            ++state.outstanding;
        }
        return pointer;
    }
    void late_free_allocation(void *pointer) noexcept {
        if (pointer == nullptr) return;
        auto &state = *late_allocation;
        for (void *&owned : state.pointers) {
            if (owned == pointer) {
                owned = nullptr;
                --state.outstanding;
                break;
            }
        }
        // Paired with the raw allocator callback above, after MuJoCo releases it.
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc)
        std::free(pointer);
    }
    struct RawAllocationContext { mjData *data; const mjModel *model; };
    void raw_data_with_late_failure(void *raw) noexcept {
        auto *context = static_cast<RawAllocationContext *>(raw);
        mju_user_malloc = &late_fail_allocation;
        mju_user_free = &late_free_allocation;
        mj_makeRawData(&context->data, context->model);
    }
    int run_late_data_allocation_failure(const char *path) {
        try {
            const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                engine::load_model(path), &mj_deleteModel);
            require(model != nullptr, "valid control model load");
            auto *data = static_cast<mjData *>(mju_malloc(sizeof(mjData)));
            require(data != nullptr, "test mjData allocation");
            std::construct_at(data);
            LateAllocationState state;
            late_allocation = &state;
            RawAllocationContext context{.data = data, .model = model.get()};
            engine::ErrorBuffer error;
            const auto previous_malloc = mju_user_malloc;
            const auto previous_free = mju_user_free;
            const bool succeeded = engine::invoke(&raw_data_with_late_failure,
                                                  &context, error);
            mj_deleteData(data);
            mju_user_malloc = previous_malloc;
            mju_user_free = previous_free;
            late_allocation = nullptr;
            require(!succeeded && error.kind == engine::ErrorKind::fatal,
                    "late raw-data allocation must return fatal status");
            require(state.calls == 2 && state.outstanding == 0,
                    "owned raw-data buffers must be freed after late fatal");
            std::cout << "PASS late raw-data fatal allocation and cleanup\n";
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL late fatal allocation: " << error.what() << '\n';
            return 1;
        }
    }

    // E2 staging fixtures: the same MJCF topologies the Python reference
    // tests use (scalar planar joints; a joint wrap for every coordinate).
    constexpr std::string_view geometric_freehub_xml = R"XML(
<mujoco><option timestep=".0002" gravity="0 0 0"/>
<default><geom type="sphere" size=".05" mass="1" contype="0" conaffinity="0"/><joint damping="0"/></default>
<worldbody><body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><joint name="frame_pitch" axis="0 1 0"/><geom/>
  <body name="crank"><joint name="crank_spin" axis="0 1 0"/><geom/></body>
  <body name="rear_wheel" pos="-.5 0 .1"><joint name="rear_carrier" type="slide" axis="0 0 1" limited="true" range="-.01 .01"/><joint name="rear_wheel_spin" axis="0 1 0"/><geom/></body>
  <body name="front_wheel" pos=".6 0 0"><joint name="front_wheel_spin" axis="0 1 0"/><geom/></body>
  <body name="pedal_front"><joint name="pedal_front_spin" axis="0 1 0"/><geom/></body>
  <body name="pedal_rear"><joint name="pedal_rear_spin" axis="0 1 0"/><geom/></body>
</body></worldbody>
<tendon><fixed name="geometric_mid_drive_freehub" limited="true" range="-100 0">
<joint joint="root_x" coef=".1"/><joint joint="frame_pitch" coef=".1"/><joint joint="crank_spin" coef=".1"/><joint joint="rear_carrier" coef=".1"/><joint joint="rear_wheel_spin" coef=".1"/><joint joint="front_wheel_spin" coef=".1"/><joint joint="pedal_front_spin" coef=".1"/><joint joint="pedal_rear_spin" coef=".1"/>
</fixed></tendon></mujoco>
)XML";
    constexpr std::string_view ideal_freehub_xml = R"XML(
<mujoco><option timestep=".0002" gravity="0 0 0"/>
<default><geom type="sphere" size=".05" mass="1" contype="0" conaffinity="0"/><joint damping="0"/></default>
<worldbody><body name="frame"><geom/>
  <body name="crank"><joint name="crank_spin" axis="0 1 0"/><geom/></body>
  <body name="rear_wheel" pos="-.5 0 .1"><joint name="rear_wheel_spin" axis="0 1 0"/><geom/></body>
</body></worldbody>
<tendon><fixed name="ideal_mid_drive_freehub" limited="true" range="-100 0"><joint joint="crank_spin" coef="1.4166666666666667"/><joint joint="rear_wheel_spin" coef="-1"/></fixed></tendon>
</mujoco>
)XML";

    mjModel *load_xml_model(std::string_view xml) {
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        require(mj_addBufferVFS(&vfs, "contract.xml", xml.data(),
                                static_cast<int>(xml.size())) == 0,
                "contract MJCF mounts into the VFS");
        std::array<char, 1024> error{};
        mjModel *model = mj_loadXML("contract.xml", &vfs, error.data(),
                                    static_cast<int>(error.size()));
        mj_deleteVFS(&vfs);
        require(model != nullptr, "contract MJCF loads");
        return model;
    }

    std::vector<double> rows(const double *first, mjtSize count) {
        const auto view = drivetrain::buffer(first, count);
        return {view.begin(), view.end()};
    }

    // E2: a staged ratio whose candidate geometry is invalid leaves the
    // transmission snapshot AND the live model byte-identical, a repeated
    // attempt raises the same rejection, and staging alone never publishes.
    void test_transmission_staged_ratio_atomicity() {
        const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            load_xml_model(geometric_freehub_xml), &mj_deleteModel);
        const drivetrain::OwnedData data(engine::make_data(model.get()));
        require(data != nullptr, "geometric contract data");
        // crank_spin — nontrivial tangent geometry.
        drivetrain::buffer(data->qpos, model->nq)[2] = .31;
        const drivetrain::GearingConfig gear{
            .front_teeth = 34, .rear_teeth = 24, .chain_pitch_m = .0127};
        drivetrain::Transmission transmission(model.get(), gear, true,
                                              "geometric_mid_drive_freehub");
        transmission.reset(data.get());
        const drivetrain::TransmissionSnapshot before = transmission.state();
        const auto generations = transmission.prepared_generations();
        const std::vector prm_before = rows(model->wrap_prm, model->nwrap);
        const std::vector range_before = rows(model->tendon_range, 2 * model->ntendon);
        // Derived constants: tendon_length0 is recomputed by mj_setConst —
        // an unchanged row proves no model write ran at all on rejection.
        const std::vector length0_before =
                rows(model->tendon_length0, model->ntendon);
        require_throws_invalid_argument([&] {
            static_cast<void>(transmission.stage_ratio(data.get(), 34. / 1000.));
        }, "impossible-sprocket staged ratio must be rejected");
        const drivetrain::TransmissionSnapshot rejected = transmission.state();
        require(rejected.ratio == before.ratio && rejected.rear_teeth == before.rear_teeth &&
                rejected.boundary == before.boundary && rejected.range == before.range &&
                rejected.coefficients == before.coefficients &&
                rejected.shift_pending == before.shift_pending &&
                rejected.shift_parameter_work_j == before.shift_parameter_work_j &&
                rejected.shift_constraint_work_j == before.shift_constraint_work_j &&
                rejected.last_tension_n == before.last_tension_n,
                "rejected stage_ratio leaves the logical snapshot untouched");
        require(rejected.prepared && before.prepared &&
                rejected.prepared->phi == before.prepared->phi &&
                rejected.prepared->time == before.prepared->time &&
                rejected.prepared->jacobian == before.prepared->jacobian &&
                rejected.prepared->qpos == before.prepared->qpos,
                "rejected stage_ratio leaves prepared geometry untouched");
        require(rows(model->wrap_prm, model->nwrap) == prm_before &&
                rows(model->tendon_range, 2 * model->ntendon) == range_before &&
                rows(model->tendon_length0, model->ntendon) == length0_before,
                "rejected stage_ratio leaves live model rows untouched");
        require(transmission.prepared_generations() == generations,
                "rejected stage_ratio preserves prepared storage identity");
        require_throws_invalid_argument([&] {
            static_cast<void>(transmission.stage_ratio(data.get(), 34. / 1000.));
        }, "a repeated staged ratio rejection is identical");
        // Staging alone never publishes: a valid update is invisible until commit.
        auto staged = transmission.stage_ratio(data.get(), 34. / 28.);
        require(transmission.state().ratio == before.ratio &&
                transmission.state().rear_teeth == before.rear_teeth,
                "staging a valid ratio must not publish before commit");
        transmission.commit(staged);
        const drivetrain::TransmissionSnapshot shifted = transmission.state();
        require(shifted.ratio == 34. / 28. && shifted.rear_teeth == 28 &&
                shifted.shift_pending && shifted.prepared,
                "committed staged shift publishes gear, ratio, and prepared state");
        // The ideal transmission shares the transaction shape.
        const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> ideal_model(
            load_xml_model(ideal_freehub_xml), &mj_deleteModel);
        const drivetrain::OwnedData ideal_data(engine::make_data(ideal_model.get()));
        require(ideal_data != nullptr, "ideal contract data");
        drivetrain::Transmission ideal(ideal_model.get(), gear);
        ideal.reset(ideal_data.get());
        const drivetrain::TransmissionSnapshot ideal_before = ideal.state();
        const std::vector ideal_prm = rows(ideal_model->wrap_prm, ideal_model->nwrap);
        require_throws_invalid_argument([&] {
            static_cast<void>(ideal.stage_ratio(ideal_data.get(), -1.));
        }, "negative staged ratio is rejected");
        require_throws_invalid_argument([&] {
            static_cast<void>(ideal.stage_ratio(ideal_data.get(), 0.));
        }, "zero staged ratio is rejected");
        require(ideal.state().ratio == ideal_before.ratio &&
                ideal.state().boundary == ideal_before.boundary &&
                rows(ideal_model->wrap_prm, ideal_model->nwrap) == ideal_prm,
                "rejected ideal stage_ratio leaves state and model untouched");
        auto ideal_update = ideal.stage_ratio(ideal_data.get(), 1.5);
        ideal.commit(ideal_update);
        require(ideal.state().ratio == 1.5 && ideal.state().coefficients[0] == 1.5,
                "committed ideal staged ratio publishes coefficient and ratio");
    }

    template <typename Exception, typename F>
    void require_throws(const F &f, std::string_view message) {
        try {
            f();
        } catch (const Exception &) {
            return;
        } catch (...) {
            require(false, message);
        }
        require(false, message);
    }

    // F5: interval_id ties land on the even neighbour, domain violations are
    // invalid_argument while an out-of-int64 quotient is overflow_error, and
    // the conversion never consults the process rounding mode. Exact binary64
    // quotients (halves representable in every mode) isolate the rounding
    // step from the mode-dependent division.
    void test_interval_clock_contracts() {
        using interval_clock::interval_id;
        require(interval_id(0.5, 1.0) == 0 && interval_id(1.5, 1.0) == 2 &&
                    interval_id(2.5, 1.0) == 2 && interval_id(3.5, 1.0) == 4,
                "half-even ties");
        require(interval_id(-0.0, 1.0) == 0, "negative zero time");
        // Largest representable quotient below 0x1p63 stays in range.
        require(interval_id(0x1.fffffffffffffp62, 1.0) ==
                    std::int64_t{9223372036854774784LL},
                "int64 upper edge");
        require_throws<std::invalid_argument>(
            [] { static_cast<void>(interval_id(-1.0, 1.0)); }, "negative time");
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        require_throws<std::invalid_argument>(
            [] { static_cast<void>(interval_id(nan, 1.0)); },
            "nonfinite time");
        require_throws<std::invalid_argument>(
            [] { static_cast<void>(interval_id(1.0, 0.0)); }, "zero dt");
        require_throws<std::invalid_argument>(
            [] { static_cast<void>(interval_id(1.0, -1.0)); }, "negative dt");
        require_throws<std::invalid_argument>(
            [] { static_cast<void>(interval_id(1.0, nan)); }, "nonfinite dt");
        require_throws<std::overflow_error>(
            [] { static_cast<void>(interval_id(0x1p63, 1.0)); },
            "quotient at the int64 boundary");
        require_throws<std::overflow_error>(
            [] { static_cast<void>(interval_id(1.0, 1e-300)); },
            "nonfinite quotient");
        const int original_mode = std::fegetround();
        {
            // Restore the caller's rounding mode even when a check fails.
            struct FeGuard {
                int saved;
                explicit FeGuard(int restore_mode) : saved(restore_mode) {}
                FeGuard(const FeGuard &) = delete;
                FeGuard &operator=(const FeGuard &) = delete;
                FeGuard(FeGuard &&) = delete;
                FeGuard &operator=(FeGuard &&) = delete;
                ~FeGuard() { std::fesetround(saved); }
            };
            const FeGuard guard(original_mode);
            for (const int candidate:
                 {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO}) {
                if (std::fesetround(candidate) != 0)
                    continue;
                require(interval_id(2.5, 1.0) == 2 &&
                            interval_id(1.5, 1.0) == 2 &&
                            interval_id(0.5, 1.0) == 0,
                        "interval_id ignores the FP rounding mode");
            }
        }
        require(std::fegetround() == original_mode,
                "rounding mode restored after the sweep");
    }

    // F5: CruiseWriter reads model->opt.timestep live for each integrate —
    // mutating the model between calls rescales the integral increment, and
    // a nonpositive timestep is rejected only when the branch runs.
    constexpr std::string_view cruise_xml = R"XML(
<mujoco><option timestep=".001"/>
<default><geom type="sphere" size=".05" mass="1" contype="0" conaffinity="0"/></default>
<worldbody><body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><geom/></body></worldbody>
</mujoco>
)XML";

    void test_cruise_live_timestep() {
        const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            load_xml_model(cruise_xml), &mj_deleteModel);
        const drivetrain::OwnedData data(engine::make_data(model.get()));
        require(data != nullptr, "cruise contract data");
        const nativecfg::CruiseConfig config{
            .target_speed_kmh = 36., .kp_nm_per_mps = 1.,
            .ki_nm_per_mps_s = 2., .torque_ceiling_nm = 100.};
        CruiseWriter writer(model.get(), config);
        // target 10 m/s, qvel 0 -> error 10; integral accumulates error*dt.
        static_cast<void>(writer.compute(data.get(), true, false, std::nullopt));
        require(writer.state().integral_mps_s == 10. * 0.001,
                "first integrate uses the ctor timestep");
        model->opt.timestep = 0.003;
        static_cast<void>(writer.compute(data.get(), true, false, std::nullopt));
        require(writer.state().integral_mps_s == 10. * 0.001 + 10. * 0.003,
                "a changed opt.timestep is picked up live");
        model->opt.timestep = 0.0;
        // Traction-limited calls skip the integral branch — no rejection.
        static_cast<void>(writer.compute(data.get(), true, true, std::nullopt));
        require_throws<std::invalid_argument>(
            [&] { static_cast<void>(writer.compute(data.get(), true, false,
                                                   std::nullopt)); },
            "nonpositive live timestep rejected when integrating");
        // Not engaged at all -> timestep is never read.
        writer.reset();
        static_cast<void>(writer.compute(data.get(), false, false,
                                         std::nullopt));
    }

    constexpr std::array<TestCase, 18> cases{{
        {.name = "human_crank_torque", .run = test_human_crank_torque},
        {.name = "pedaling_policy_valid_transition", .run = test_pedaling_policy_valid_transition},
        {.name = "pedaling_ctor_domain", .run = test_pedaling_ctor_domain},
        {.name = "shifter_ctor_domain", .run = test_shifter_ctor_domain},
        {.name = "assist_ctor_domain", .run = test_assist_ctor_domain},
        {.name = "battery_ctor_domain", .run = test_battery_ctor_domain},
        {.name = "freehub_ctor_domain", .run = test_freehub_ctor_domain},
        {.name = "writer_config_domains", .run = test_writer_config_domains},
        {.name = "transmission_lifecycle_invariants", .run = test_transmission_lifecycle_invariants},
        {.name = "typed_ctor_matches_validator_domain", .run = test_typed_ctor_matches_validator_domain},
        {.name = "engine_fatal_status_and_reuse", .run = test_engine_fatal_status_and_reuse},
        {.name = "nested_engine_frames", .run = test_nested_engine_frames},
        {.name = "thread_local_engine_frames", .run = test_thread_local_engine_frames},
        {.name = "previous_tls_handler_restored", .run = test_previous_tls_handler_restored},
        {.name = "owned_staging_owner_failure_frees_storage", .run = test_owned_staging_owner_failure_frees_storage},
        {.name = "transmission_staged_ratio_atomicity", .run = test_transmission_staged_ratio_atomicity},
        {.name = "interval_clock_contracts", .run = test_interval_clock_contracts},
        {.name = "cruise_live_timestep", .run = test_cruise_live_timestep},
    }};

    int run_case(const TestCase &test_case) {
        try {
            test_case.run();
            std::cout << "PASS " << test_case.name << '\n';
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
            return 1;
        }
    }
} // namespace

int main(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the sentinel.
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() == 2 && std::string_view(arguments[1]) == "--require-failure") {
        try {
            require(false, "deliberate require failure");
        } catch (const std::exception &error) {
            std::cerr << error.what() << '\n';
            return 1;
        }
        return 0;
    }
    if (arguments.size() == 2 && std::string_view(arguments[1]) == "--list") {
        for (const TestCase &test_case : cases) std::cout << test_case.name << '\n';
        return 0;
    }
    if (arguments.size() == 3 &&
        std::string_view(arguments[1]) == "--load-allocation-failure")
        return run_allocation_failure(arguments[2], false);
    if (arguments.size() == 3 &&
        std::string_view(arguments[1]) == "--make-data-allocation-failure")
        return run_allocation_failure(arguments[2], true);
    if (arguments.size() == 3 &&
        std::string_view(arguments[1]) == "--late-data-allocation-failure")
        return run_late_data_allocation_failure(arguments[2]);
    if (arguments.size() > 2) {
        std::cerr << "usage: native_contract_tests [case-name|--list|--require-failure]\n";
        return 2;
    }

    const std::string_view requested =
        arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
    bool found = requested == "all";
    int status = 0;
    for (const TestCase &test_case : cases) {
        if (requested == "all" || requested == test_case.name) {
            found = true;
            if (run_case(test_case) != 0) status = 1;
        }
    }
    if (!found) {
        std::cerr << "unknown native contract case: " << requested << '\n';
        return 2;
    }
    return status;
}
