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
#include "../src/model_access.hpp"
#include "../src/tyre/profile.hpp"
#include "../src/rider/support_geometry.hpp"
#include "../src/cblas_abi.hpp"
#include "../src/numeric_norm.hpp"
#include "../src/writers/cruise.hpp"
#include "../src/writers/drivetrain.hpp"
#include "allocation_faults.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
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

    // C2: model_access extent/ID/product gates plus the readonly-element
    // compile-time contract and the drivetrain::buffer delegation alias.
    void test_model_access_contracts() {
        static_assert(std::is_same_v<
                      decltype(model_access::readonly_buffer(
                          std::declval<const double *>(), 0)),
                      std::span<const double>>,
                      "readonly_buffer keeps the const element type");
        static_assert(std::is_same_v<
                      decltype(model_access::mutable_buffer(
                          std::declval<double *>(), 0)),
                      std::span<double>>,
                      "mutable_buffer keeps the mutable element type");
        static_assert(std::is_same_v<
                      decltype(drivetrain::buffer(
                          std::declval<const double *>(), 0)),
                      std::span<const double>>,
                      "drivetrain::buffer delegates readonly reads");
        static_assert(std::is_same_v<
                      decltype(drivetrain::buffer(std::declval<double *>(), 0)),
                      std::span<double>>,
                      "drivetrain::buffer delegates mutable reads");
        require(model_access::readonly_buffer(
                    static_cast<const double *>(nullptr), 0)
                .empty(),
                "null pointer with a zero extent is the empty buffer");
        require_throws<std::invalid_argument>(
            [] {
                static_cast<void>(model_access::readonly_buffer(
                    static_cast<const double *>(nullptr), 1));
            }, "null pointer with positive extent must be rejected");
        require_throws<std::invalid_argument>(
            [] {
                static_cast<void>(model_access::mutable_buffer(
                    static_cast<double *>(nullptr), -1));
            }, "negative mutable extent must be rejected");
        const std::array storage{1., 2.};
        const auto view =
                model_access::readonly_buffer(storage.data(), 2);
        require(view.size() == 2 && view[0] == 1. && view[1] == 2.,
                "positive extent exposes the underlying storage");
        require_throws<std::invalid_argument>(
            [&] {
                static_cast<void>(
                    model_access::readonly_buffer(storage.data(), -1));
            }, "negative readonly extent must be rejected");
        require_throws<std::invalid_argument>(
            [] { model_access::require_id(-1, 5, "id"); },
            "require_id rejects -1");
        require_throws<std::invalid_argument>(
            [] { model_access::require_id(5, 5, "id"); },
            "require_id rejects the one-past-end ID");
        require_throws<std::invalid_argument>(
            [] { model_access::require_id(0, -1, "id"); },
            "require_id rejects a negative extent");
        model_access::require_id(0, 5, "id");
        model_access::require_id(4, 5, "id"); // boundary in-range controls
        require(model_access::checked_product(3, 4, 12) == 12,
                "product inside the storage limit");
        require(model_access::checked_product(0, 99, 0) == 0,
                "a zero factor stays inside a zero limit");
        require_throws<std::invalid_argument>(
            [] {
                static_cast<void>(
                    model_access::checked_product(3, 5, 12));
            }, "product beyond the storage limit must be rejected");
        require(model_access::checked_sum(4, 8, 12) == 12,
                "sum inside the storage limit");
        require_throws<std::invalid_argument>(
            [] {
                static_cast<void>(model_access::checked_sum(9, 4, 12));
            }, "sum beyond the storage limit must be rejected");
        require_throws<std::invalid_argument>(
            [] {
                static_cast<void>(model_access::checked_sum(13, 0, 12));
            }, "base beyond the storage limit must be rejected");
    }

    // C2: geometry evaluate() rejects non-scalar topology (nq != nv is a
    // rejected model shape, not a qpos-width overflow) and every sprocket/
    // frame ID goes through require_id before any model row is indexed.
    constexpr std::string_view free_topology_xml = R"XML(
<mujoco><worldbody>
<body name="floater"><joint type="free"/><geom type="sphere" size=".05" mass="1"/></body>
</worldbody></mujoco>
)XML";

    void test_geometry_topology_and_ids() {
        const drivetrain::GearingConfig gear{
            .front_teeth = 34, .rear_teeth = 18, .chain_pitch_m = .0127};
        {   // free joint: nq=7, nv=6 — nq >= nv is valid elsewhere but is
            // not the scalar planar topology geometry requires.
            const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                load_xml_model(free_topology_xml), &mj_deleteModel);
            const drivetrain::OwnedData data(
                engine::make_data(model.get()));
            require(data != nullptr, "free-topology contract data");
            require(model->nq != model->nv,
                    "free joint fixture must have nq != nv");
            drivetrain::GeometryWorkspace workspace(model->nv);
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(workspace.evaluate(
                        model.get(), data.get(), gear, 1, 1, 1));
                }, "nq != nv topology must be rejected as a model shape");
        }
        {   // Scalar planar control model (geometric fixture): invalid
            // sprocket/frame IDs are rejected before any array index.
            const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                load_xml_model(geometric_freehub_xml), &mj_deleteModel);
            const drivetrain::OwnedData data(
                engine::make_data(model.get()));
            require(data != nullptr, "geometric contract data");
            require(model->nq == model->nv,
                    "geometric fixture keeps the scalar topology");
            drivetrain::GeometryWorkspace workspace(model->nv);
            for (const int bad: {-1, static_cast<int>(model->nbody)}) {
                require_throws<std::invalid_argument>(
                    [&] {
                        static_cast<void>(workspace.evaluate(
                            model.get(), data.get(), gear, bad, 3, 1,
                            std::nullopt, std::nullopt, false));
                    }, "out-of-range front ID must be rejected");
                require_throws<std::invalid_argument>(
                    [&] {
                        static_cast<void>(workspace.evaluate(
                            model.get(), data.get(), gear, 2, bad, 1,
                            std::nullopt, std::nullopt, false));
                    }, "out-of-range rear ID must be rejected");
                require_throws<std::invalid_argument>(
                    [&] {
                        static_cast<void>(workspace.evaluate(
                            model.get(), data.get(), gear, 2, 3, bad,
                            std::nullopt, std::nullopt, false));
                    }, "out-of-range frame ID must be rejected");
            }
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(
                        workspace.angle(model.get(), data.get(), 0, false));
                }, "the world body is not a sprocket body");
        }
    }

    // C2: nonfinite transform entries are rejected before the planar-frame
    // tolerance comparisons — NaN would otherwise slip through |x| > eps.
    void test_geometry_nonfinite_transform() {
        const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            load_xml_model(geometric_freehub_xml), &mj_deleteModel);
        const drivetrain::OwnedData data(engine::make_data(model.get()));
        require(data != nullptr, "geometric contract data");
        mj_forward(model.get(), data.get());
        const int crank = mj_name2id(model.get(), mjOBJ_BODY, "crank");
        require(crank > 0, "crank body resolves");
        drivetrain::GeometryWorkspace workspace(model->nv);
        static_cast<void>(workspace.angle(model.get(), data.get(), crank,
                                          false));
        const auto xmat =
                model_access::mutable_buffer(data->xmat, 9 * model->nbody);
        const std::size_t base = 9 * static_cast<std::size_t>(crank);
        const mjtNum saved = xmat[base];
        xmat[base] = std::numeric_limits<mjtNum>::quiet_NaN();
        require_throws<std::invalid_argument>(
            [&] {
                static_cast<void>(
                    workspace.angle(model.get(), data.get(), crank, false));
            }, "a NaN rotation entry must be rejected before tolerances");
        xmat[base] = saved;
        xmat[base + 4] = std::numeric_limits<mjtNum>::infinity();
        require_throws<std::invalid_argument>(
            [&] {
                static_cast<void>(
                    workspace.angle(model.get(), data.get(), crank, false));
            }, "an infinite rotation entry must be rejected");
        xmat[base + 4] = 1.;
        static_cast<void>(workspace.angle(model.get(), data.get(), crank,
                                          false));
    }

    // C2: interpolation/candidate/heightfield boundaries — empty or
    // mismatched tables are explicit rejections, np.arange(lo, hi) declares
    // hi <= lo the empty interval, and raster extents go through checked
    // products/sums against nhfielddata (engine mjtSize limits).
    constexpr std::string_view hfield_xml = R"XML(
<mujoco><asset><hfield name="hf" nrow="3" ncol="4" size="1 1 .1 .1"/></asset>
<worldbody><geom name="terrain" type="hfield" hfield="hf"/></worldbody></mujoco>
)XML";

    void test_profile_contract_boundaries() {
        {   // np_interp boundary: empty or width-mismatched tables.
            const std::array xs{0., 1.}, ys{0., 2.};
            require(biketyre::np_interp(.5, xs, ys) == 1.,
                    "valid interpolation control");
            const std::span<const double> empty;
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(biketyre::np_interp(.5, empty, ys));
                }, "empty interpolation table must be rejected");
            const std::array short_ys{0.};
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(
                        biketyre::np_interp(.5, xs, short_ys));
                }, "mismatched interpolation widths must be rejected");
        }
        {   // ProfileQuery ctor validates the profile shape once; the
            // contact path then runs the unchecked inner loop.
            const biketyre::ProfileQuery control({0., 1., 2.}, {0., 0., 0.},
                                                 .001, .5, 30.);
            static_cast<void>(control);
            require_throws<std::invalid_argument>(
                [] {
                    const biketyre::ProfileQuery q({0., 1.}, {0.}, .001, .5,
                                                   30.);
                }, "mismatched profile widths must be rejected");
            require_throws<std::invalid_argument>(
                [] {
                    const biketyre::ProfileQuery q({0.}, {0.}, .001, .5, 30.);
                }, "a single-point profile must be rejected");
            require_throws<std::invalid_argument>(
                [] {
                    const biketyre::ProfileQuery q({0., 0., 1.}, {0., 0., 0.},
                                                   .001, .5, 30.);
                }, "nonmonotonic abscissas must be rejected");
            require_throws<std::invalid_argument>(
                [] {
                    const biketyre::ProfileQuery q(
                        {0., std::numeric_limits<double>::quiet_NaN(), 1.},
                        {0., 0., 0.}, .001, .5, 30.);
                }, "nonfinite vertices must be rejected");
        }
        {   // detail::candidates interval contract: hi <= lo is the declared
            // empty interval (np.arange semantics), never a negative reserve.
            const std::array px{0., 1., 2.}, pz{0., 0., 0.};
            const std::array sx{1., 1.}, sz{0., 0.}, sl{1., 1.};
            auto empty = biketyre::detail::candidates(px, pz, sx, sz, sl,
                                                    {.5, 1.}, 2, 1);
            require(empty.ids.empty() && empty.t.empty() &&
                    empty.points.empty() && empty.distances.empty(),
                    "lo > hi is the declared empty interval");
            empty = biketyre::detail::candidates(px, pz, sx, sz, sl,
                                                 {.5, 1.}, 1, 1);
            require(empty.ids.empty(), "lo == hi is the empty interval");
            const auto cand = biketyre::detail::candidates(
                px, pz, sx, sz, sl, {.5, 1.}, 0, 2);
            require(cand.ids.size() == 2,
                    "full window produces one candidate per segment");
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(biketyre::detail::candidates(
                        px, pz, sx, sz, sl, {.5, 1.}, -1, 2));
                }, "a window below the table must be rejected");
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(biketyre::detail::candidates(
                        px, pz, sx, sz, sl, {.5, 1.}, 0, 3));
                }, "a window past the table must be rejected");
            const std::array mismatch{1.};
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(biketyre::detail::candidates(
                        px, pz, mismatch, sz, sl, {.5, 1.}, 0, 1));
                }, "mismatched candidate tables must be rejected");
            const std::span<const double> blank;
            require_throws<std::invalid_argument>(
                [&] {
                    static_cast<void>(biketyre::detail::candidates(
                        blank, blank, blank, blank, blank, {.5, 1.}, 0, 1));
                }, "empty candidate tables must be rejected");
        }
        {   // Heightfield extents: checked product/sum against nhfielddata,
            // invalid geom/data IDs, and the non-hfield rejection.
            const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                load_xml_model(hfield_xml), &mj_deleteModel);
            const drivetrain::OwnedData data(
                engine::make_data(model.get()));
            require(data != nullptr, "heightfield contract data");
            mj_forward(model.get(), data.get());
            const auto vertices = biketyre::compiled_profile_vertices(
                model.get(), data.get(), "terrain");
            require(vertices.size() == 4,
                    "valid heightfield control yields ncol vertices");
            const int geom = mj_name2id(model.get(), mjOBJ_GEOM, "terrain");
            require(geom >= 0, "terrain geom resolves");
            // Each mutation runs inside the lambda so the saved originals
            // are restored for the next case — invalid models exist only
            // inside this test. Model-field access itself goes through the
            // checked buffers (-Wunsafe-buffer-usage).
            const auto nrow_buf = model_access::mutable_buffer(
                model->hfield_nrow, model->nhfield);
            const auto ncol_buf = model_access::mutable_buffer(
                model->hfield_ncol, model->nhfield);
            const auto adr_buf = model_access::mutable_buffer(
                model->hfield_adr, model->nhfield);
            const auto type_buf = model_access::mutable_buffer(
                model->geom_type, model->ngeom);
            const auto dataid_buf = model_access::mutable_buffer(
                model->geom_dataid, model->ngeom);
            const std::size_t gi = static_cast<std::size_t>(geom);
            const auto rejected = [&](const auto &mutate) {
                const int rows = nrow_buf[0];
                const int cols = ncol_buf[0];
                const int adr = adr_buf[0];
                const int type = type_buf[gi];
                const int dataid = dataid_buf[gi];
                mutate();
                require_throws<std::invalid_argument>(
                    [&] {
                        static_cast<void>(biketyre::compiled_profile_vertices(
                            model.get(), data.get(), "terrain"));
                    }, "corrupt heightfield extents must be rejected");
                nrow_buf[0] = rows;
                ncol_buf[0] = cols;
                adr_buf[0] = adr;
                type_buf[gi] = type;
                dataid_buf[gi] = dataid;
            };
            rejected([&] { nrow_buf[0] = -2; });
            rejected([&] { ncol_buf[0] = 1; });
            rejected([&] { ncol_buf[0] = std::numeric_limits<int>::max(); });
            rejected([&] { adr_buf[0] = -1; });
            rejected([&] {
                adr_buf[0] = static_cast<int>(model->nhfielddata);
            });
            rejected([&] { type_buf[gi] = mjGEOM_SPHERE; });
            rejected([&] { dataid_buf[gi] = -1; });
        }
    }

    // C2: the support-model walk validates every geom/body ID and enforces
    // the ordered-parent invariant 0 <= parent < body — a corrupt or cyclic
    // parent chain is a bounded rejection, never an unbounded loop — and
    // nonfinite joint axes fail before the axis-layout tolerances.
    constexpr std::string_view support_xml = R"XML(
<mujoco><default><geom type="sphere" size=".05" mass="1" contype="0" conaffinity="0"/></default>
<worldbody>
<body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><geom/>
  <body name="pedal"><joint name="pedal_z" type="slide" axis="0 0 1"/>
    <geom name="pedal_geom" type="box" size=".1 .05 .01"/></body>
</body></worldbody></mujoco>
)XML";

    void test_support_geometry_parent_contracts() {
        const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            load_xml_model(support_xml), &mj_deleteModel);
        const drivetrain::OwnedData data(engine::make_data(model.get()));
        require(data != nullptr, "support contract data");
        mj_forward(model.get(), data.get());
        const int geom = mj_name2id(model.get(), mjOBJ_GEOM, "pedal_geom");
        require(geom >= 0, "pedal geom resolves");
        const std::array good{geom};
        rider::validate_planar_support_model(model.get(), data.get(), good);
        for (const int bad: {-1, static_cast<int>(model->ngeom)}) {
            const std::array ids{bad};
            require_throws<std::invalid_argument>(
                [&] {
                    rider::validate_planar_support_model(model.get(),
                                                         data.get(), ids);
                }, "out-of-range support geom must be rejected");
        }
        const int pedal = mj_name2id(model.get(), mjOBJ_BODY, "pedal");
        require(pedal > 0, "pedal body resolves");
        const std::size_t pi = static_cast<std::size_t>(pedal);
        const std::size_t gi = static_cast<std::size_t>(geom);
        const auto parent_buf = model_access::mutable_buffer(
            model->body_parentid, model->nbody);
        const auto geom_body_buf = model_access::mutable_buffer(
            model->geom_bodyid, model->ngeom);
        {   // Self-parent: parent >= body violates the ordered invariant.
            const int saved = parent_buf[pi];
            parent_buf[pi] = pedal;
            require_throws<std::invalid_argument>(
                [&] {
                    rider::validate_planar_support_model(model.get(),
                                                         data.get(), good);
                }, "a cyclic body parent must be rejected");
            parent_buf[pi] = saved;
        }
        {   // Parent outside the body table is invalid, not an OOB index.
            const int saved = parent_buf[pi];
            parent_buf[pi] = -2;
            require_throws<std::invalid_argument>(
                [&] {
                    rider::validate_planar_support_model(model.get(),
                                                         data.get(), good);
                }, "an invalid body parent must be rejected");
            parent_buf[pi] = saved;
        }
        {   // geom_bodyid pointing outside nbody fails before indexing.
            const int saved = geom_body_buf[gi];
            geom_body_buf[gi] = static_cast<int>(model->nbody);
            require_throws<std::invalid_argument>(
                [&] {
                    rider::validate_planar_support_model(model.get(),
                                                         data.get(), good);
                }, "an out-of-range geom body must be rejected");
            geom_body_buf[gi] = saved;
        }
        const auto xaxis =
                model_access::mutable_buffer(data->xaxis, 3 * model->njnt);
        {   // A NaN axis entry on an ancestor joint is rejected before the
            // axis-layout tolerance comparisons.
            const mjtNum saved = xaxis[0];
            xaxis[0] = std::numeric_limits<mjtNum>::quiet_NaN();
            require_throws<std::invalid_argument>(
                [&] {
                    rider::validate_planar_support_model(model.get(),
                                                         data.get(), good);
                }, "a nonfinite joint axis must be rejected");
            xaxis[0] = saved;
        }
        {   // A nonfinite geom transform entry fails via validate_box.
            const auto gxmat = model_access::mutable_buffer(
                data->geom_xmat, 9 * model->ngeom);
            const std::size_t base = 9 * static_cast<std::size_t>(geom);
            const mjtNum saved = gxmat[base];
            gxmat[base] = std::numeric_limits<mjtNum>::infinity();
            require_throws<std::invalid_argument>(
                [&] {
                    rider::validate_planar_support_model(model.get(),
                                                         data.get(), good);
                }, "a nonfinite geom transform must be rejected");
            gxmat[base] = saved;
        }
        // Restored model remains a valid control.
        rider::validate_planar_support_model(model.get(), data.get(), good);
    }

    // ---------- E4: exactly-once settlement + exhaustive allocation sweep --

    // Bitwise logical-state equality. An injected allocation failure must
    // leave the whole drivetrain snapshot byte-identical to the armed
    // baseline, and the disarmed retry must reproduce the clean control run
    // exactly — value equality would tolerate drift the transaction must
    // not have.
    bool same(double a, double b) {
        return std::bit_cast<std::uint64_t>(a) ==
               std::bit_cast<std::uint64_t>(b);
    }
    // Generic optional lift — declared up front so every comparator below
    // can call it, defined after the concrete overloads so the dependent
    // payload lookup resolves each same(T, T) above.
    template <typename T>
    bool same(const std::optional<T> &a, const std::optional<T> &b);
    bool same(const std::vector<double> &a, const std::vector<double> &b) {
        if (a.size() != b.size())
            return false;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (!same(a[i], b[i]))
                return false;
        return true;
    }
    bool same(const drivetrain::DiagnosticValue &a,
              const drivetrain::DiagnosticValue &b) {
        if (a.index() != b.index())
            return false;
        return std::visit(
            [](const auto &x, const auto &y) {
                using T = std::decay_t<decltype(x)>;
                using U = std::decay_t<decltype(y)>;
                if constexpr (!std::is_same_v<T, U>)
                    return false;
                else if constexpr (std::is_same_v<T, double>)
                    return same(x, y);
                else if constexpr (std::is_same_v<T, std::monostate>)
                    return true;
                else
                    return x == y; // int, bool, std::string
            },
            a, b);
    }
    bool same(const drivetrain::Diagnostics &a,
              const drivetrain::Diagnostics &b) {
        if (a.size() != b.size())
            return false;
        for (auto ia = a.begin(), ib = b.begin(); ia != a.end(); ++ia, ++ib)
            if (ia->first != ib->first || !same(ia->second, ib->second))
                return false;
        return true;
    }
    bool same(const drivetrain::PreparedTransmission &a,
              const drivetrain::PreparedTransmission &b) {
        return same(a.phi, b.phi) && same(a.time, b.time) &&
               same(a.jacobian, b.jacobian) && same(a.qpos, b.qpos);
    }
    bool same(const drivetrain::TransmissionSnapshot &a,
              const drivetrain::TransmissionSnapshot &b) {
        return same(a.ratio, b.ratio) && a.rear_teeth == b.rear_teeth &&
               same(a.boundary, b.boundary) && same(a.prepared, b.prepared) &&
               same(a.diagnostics, b.diagnostics) &&
               a.shift_pending == b.shift_pending &&
               same(a.shift_parameter_work_j, b.shift_parameter_work_j) &&
               same(a.shift_constraint_work_j, b.shift_constraint_work_j) &&
               same(a.last_tension_n, b.last_tension_n) &&
               same(a.range[0], b.range[0]) && same(a.range[1], b.range[1]) &&
               same(a.coefficients, b.coefficients);
    }
    bool same(const drivetrain::PedalingSnapshot &a,
              const drivetrain::PedalingSnapshot &b) {
        return a.coasting == b.coasting &&
               same(a.target_phase_rad, b.target_phase_rad) &&
               same(a.target_rate_rad_s, b.target_rate_rad_s) &&
               same(a.deceleration_rad_s2, b.deceleration_rad_s2) &&
               same(a.effort, b.effort) && same(a.cadence_ema, b.cadence_ema);
    }
    bool same(const drivetrain::ShiftingSnapshot &a,
              const drivetrain::ShiftingSnapshot &b) {
        return a.rear_teeth == b.rear_teeth && a.from_teeth == b.from_teeth &&
               a.shift_count == b.shift_count &&
               same(a.cooldown_s, b.cooldown_s) &&
               same(a.cut_remaining_s, b.cut_remaining_s) &&
               a.direction == b.direction &&
               same(a.cadence_ema, b.cadence_ema) &&
               same(a.required_ema, b.required_ema);
    }
    bool same(const drivetrain::AssistSnapshot &a,
              const drivetrain::AssistSnapshot &b) {
        return same(a.torque, b.torque) && same(a.last_gain, b.last_gain) &&
               a.pedaling == b.pedaling;
    }
    bool same(const drivetrain::BatterySnapshot &a,
              const drivetrain::BatterySnapshot &b) {
        return same(a.initial_energy_j, b.initial_energy_j) &&
               same(a.energy_j, b.energy_j) &&
               same(a.drawn_energy_j, b.drawn_energy_j);
    }
    bool same(const drivetrain::FreehubSnapshot &a,
              const drivetrain::FreehubSnapshot &b) {
        return same(a.boundary, b.boundary) && same(a.energy_j, b.energy_j) &&
               same(a.torque_nm, b.torque_nm);
    }
    bool same(const drivetrain::PendingActuation &a,
              const drivetrain::PendingActuation &b) {
        return same(a.requested, b.requested) && same(a.omega, b.omega) &&
               same(a.dt, b.dt) && a.enabled == b.enabled;
    }
    template <typename T>
    bool same(const std::optional<T> &a, const std::optional<T> &b) {
        return a.has_value() == b.has_value() && (!a || same(*a, *b));
    }
    bool same(const drivetrain::DriveSnapshot &a,
              const drivetrain::DriveSnapshot &b) {
        return same(a.pedaling, b.pedaling) && same(a.shifting, b.shifting) &&
               same(a.assist, b.assist) && same(a.battery, b.battery) &&
               same(a.hub, b.hub) && same(a.shift_time_s, b.shift_time_s) &&
               same(a.last_time_s, b.last_time_s) &&
               same(a.reference, b.reference) && same(a.psi, b.psi) &&
               same(a.angles, b.angles) && same(a.last, b.last) &&
               same(a.probe_last, b.probe_last) &&
               same(a.pending_actuation, b.pending_actuation) &&
               same(a.ideal_hub, b.ideal_hub) && same(a.clutch, b.clutch) &&
               same(a.freewheel, b.freewheel);
    }

    // The operation's observable result folded into comparable bits. Boxing
    // it sits inside the armed window — exactly where the binding places
    // owned()/result_dict — so a result-boxing allocation failure is
    // enumerated like every production allocation.
    struct Digest {
        std::vector<double> numbers;
        std::vector<std::string> strings;
    };
    bool same(const Digest &a, const Digest &b) {
        return a.strings == b.strings && same(a.numbers, b.numbers);
    }

    struct ModelRows {
        std::vector<double> wrap_prm, tendon_range, tendon_length0;
    };
    ModelRows model_rows(const mjModel &model) {
        return {.wrap_prm = rows(model.wrap_prm, model.nwrap),
                .tendon_range = rows(model.tendon_range, 2 * model.ntendon),
                .tendon_length0 = rows(model.tendon_length0, model.ntendon)};
    }
    bool same(const ModelRows &a, const ModelRows &b) {
        return same(a.wrap_prm, b.wrap_prm) &&
               same(a.tendon_range, b.tendon_range) &&
               same(a.tendon_length0, b.tendon_length0);
    }

    // Mirrors tests/reference/test_native_drivetrain.py::model_xml for the
    // crank_effort physical chain: scalar planar joints plus each auxiliary
    // one-way hub topology. The spec wrapper keeps the two same-typed
    // selector strings from silently swapping at call sites.
    struct DriveSpec {
        std::string_view kind, topology;
    };
    std::string drive_mjcf(const DriveSpec &spec) {
        const std::string_view kind = spec.kind, topology = spec.topology;
        std::vector<std::string> names{
            "root_x", "frame_pitch", "crank_spin", "rear_carrier",
            "rear_wheel_spin", "front_wheel_spin", "pedal_front_spin",
            "pedal_rear_spin"};
        std::string extra, tendon;
        if (kind == "elastic_chain") {
            extra += "<body name=\"cassette\" pos=\"-.5 0 0\"><joint "
                     "name=\"cassette_spin\" axis=\"0 1 0\"/><geom size=\".04\" "
                     "mass=\"1\"/></body>";
            names.emplace_back("cassette_spin");
        }
        if (topology == "clutch") {
            extra += "<body name=\"drive_shaft\"><joint "
                     "name=\"drive_shaft_spin\" axis=\"0 1 0\"/><geom "
                     "size=\".04\" mass=\"1\"/></body>";
            names.emplace_back("drive_shaft_spin");
        }
        if (topology == "rotor") {
            extra += "<body name=\"rotor\"><joint name=\"rotor_spin\" "
                     "axis=\"0 1 0\"/><inertial pos=\"0 0 0\" mass=\"1\" "
                     "diaginertia=\".2 .2 .2\"/></body>";
            names.emplace_back("rotor_spin");
        }
        const std::string_view driver =
                topology == "clutch" ? "drive_shaft_spin" : "crank_spin";
        if (kind == "ideal_mid_drive") {
            tendon += "<fixed name=\"ideal_mid_drive_freehub\" limited=\"true\" "
                      "range=\"-100 0\"><joint joint=\"";
            tendon += driver;
            tendon += "\" coef=\"1.4166666666666667\"/><joint "
                      "joint=\"rear_wheel_spin\" coef=\"-1\"/></fixed>";
        } else if (kind == "geometric_ideal_mid_drive") {
            tendon += "<fixed name=\"geometric_mid_drive_freehub\" "
                      "limited=\"true\" range=\"-100 0\">";
            for (const std::string &name: names) {
                tendon += R"(<joint joint=")" + name + R"(" coef=".1"/>)";
            }
            tendon += "</fixed>";
        }
        if (topology == "clutch")
            tendon += "<fixed name=\"crank_clutch\" limited=\"true\" "
                      "range=\"-100 0\"><joint joint=\"crank_spin\" "
                      "coef=\"1\"/><joint joint=\"drive_shaft_spin\" "
                      "coef=\"-1\"/></fixed>";
        if (topology == "rotor")
            tendon += "<fixed name=\"motor_freewheel\" limited=\"true\" "
                      "range=\"-100 0\"><joint joint=\"rotor_spin\" "
                      "coef=\"1\"/><joint joint=\"crank_spin\" "
                      "coef=\"-1\"/></fixed>";
        const std::string_view motor_joint =
                topology == "rotor" ? "rotor_spin" : driver;
        std::string xml =
                "<mujoco><option timestep=\".0002\" gravity=\"0 0 0\"/>"
                "<default><geom type=\"sphere\" size=\".05\" mass=\"1\" "
                "contype=\"0\" conaffinity=\"0\"/><joint damping=\"0\"/></default>"
                "<worldbody><body name=\"frame\"><joint name=\"root_x\" "
                "type=\"slide\" axis=\"1 0 0\"/><joint name=\"frame_pitch\" "
                "axis=\"0 1 0\"/><geom/>"
                "<body name=\"crank\"><joint name=\"crank_spin\" axis=\"0 1 "
                "0\"/><geom/></body>"
                "<body name=\"rear_wheel\" pos=\"-.5 0 .1\"><joint "
                "name=\"rear_carrier\" type=\"slide\" axis=\"0 0 1\" "
                "limited=\"true\" range=\"-.01 .01\"/><joint "
                "name=\"rear_wheel_spin\" axis=\"0 1 0\"/><geom/></body>"
                "<body name=\"front_wheel\" pos=\".6 0 0\"><joint "
                "name=\"front_wheel_spin\" axis=\"0 1 0\"/><geom/></body>"
                "<body name=\"pedal_front\"><joint name=\"pedal_front_spin\" "
                "axis=\"0 1 0\"/><geom/></body>"
                "<body name=\"pedal_rear\"><joint name=\"pedal_rear_spin\" "
                "axis=\"0 1 0\"/><geom/></body>";
        xml += extra;
        xml += "</body></worldbody><tendon>";
        xml += tendon;
        xml += "</tendon><actuator><motor name=\"human_crank\" "
               "joint=\"crank_spin\"/><motor name=\"mid_drive\" joint=\"";
        xml += motor_joint;
        xml += "\"/></actuator></mujoco>";
        return xml;
    }

    int joint_dof(const mjModel *m, const char *name) {
        const int id = mj_name2id(m, mjOBJ_JOINT, name);
        require(id >= 0, "fixture joint resolves");
        return drivetrain::buffer(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(id)];
    }
    int joint_qpos(const mjModel *m, const char *name) {
        const int id = mj_name2id(m, mjOBJ_JOINT, name);
        require(id >= 0, "fixture joint resolves");
        return drivetrain::buffer(m->jnt_qposadr, m->njnt)[static_cast<std::size_t>(id)];
    }

    drivetrain::DriveConfig drive_config(std::string_view kind,
                                         std::string_view topology,
                                         bool assist_enabled, bool shifting_enabled) {
        drivetrain::ShiftingConfig shifting{
            .enabled = shifting_enabled, .cassette = {24, 28},
            .target_cadence_min_rpm = 65., .target_cadence_max_rpm = 85.,
            .shift_cooldown_s = .4, .shift_cut_duration_s = .2,
            .torque_factor = .3, .cadence_smoothing_tau_s = .35,
            .upshift_slip_limit_mps = .5,
            .upshift_slip_mode = "legacy_signed"};
        drivetrain::AssistConfig assist = valid_assist_config();
        if (!assist_enabled) {
            assist.gain = 0.;
            assist.max_torque = 0.;
            assist.max_power = 0.;
        }
        return {
            .policies = {
                .gearing = {.front_teeth = 34, .rear_teeth = 24,
                            .chain_pitch_m = .0127},
                .pedaling = valid_pedaling_config(),
                .shifting = std::move(shifting),
                .assist = std::move(assist),
                .battery = {.enabled = true, .energy_j = 1800000.,
                            .copper_w_per_nm2 = .02, .speed_w_per_rad_s2 = 0.,
                            .idle_w = 5.},
                .hub_stiffness_nm_rad = 1000., .hub_damping_nm_s = .5},
            .drive_mode = "crank_effort",
            .transmission_model = std::string(kind),
            .human_torque_nm = 20., .torque_ripple = .35,
            .crank_phase_rad = 0., .chain_k_n_m = 200000.,
            .chain_c_ns_m = 10., .bearing_c_nms_rad = .03,
            .rotor_inertia_kgm2 = topology == "rotor" ? .2 : 0.,
            .motor_clutch = topology == "clutch"};
    }

    // The pairing-fixture model/data/writer trio, seeded exactly like the
    // Python reference pair(): posed carrier, spinning crank/wheel, then
    // kinematics + reset so every op starts from a real drivetrain state.
    struct DriveFixture {
        drivetrain::OwnedModel model;
        drivetrain::OwnedData data;
        drivetrain::DrivetrainWriter writer;

        DriveFixture(std::string_view kind, std::string_view topology,
                     drivetrain::DriveConfig config)
            : model(load_xml_model(
                  drive_mjcf({.kind = kind, .topology = topology}))),
              data(engine::make_data(model.get())),
              writer(model.get(), data.get(), std::move(config)) {
            require(data != nullptr, "drive fixture data");
            mjData *d = data.get();
            const auto qpos = drivetrain::buffer(d->qpos, model->nq);
            const auto qvel = drivetrain::buffer(d->qvel, model->nv);
            qpos[static_cast<std::size_t>(joint_qpos(model.get(), "rear_carrier"))] = .013;
            qvel[static_cast<std::size_t>(joint_dof(model.get(), "crank_spin"))] = 4.;
            qvel[static_cast<std::size_t>(joint_dof(model.get(), "rear_wheel_spin"))] = 5.;
            if (topology == "clutch")
                qvel[static_cast<std::size_t>(joint_dof(model.get(), "drive_shaft_spin"))] = 4.5;
            if (topology == "rotor")
                qvel[static_cast<std::size_t>(joint_dof(model.get(), "rotor_spin"))] = 4.5;
            mj_forward(model.get(), d);
            writer.reset();
        }
    };

    // Reserve a pending actuation on a nonzero delivered torque, then let the
    // engine settle the reserved ctrl into actuator_force — the same state a
    // settle() sees in production.
    void prime_pending(DriveFixture &fixture) {
        static_cast<void>(fixture.writer.components(
            {}, .002, 2., false, true, true, 20., std::nullopt, true,
            std::nullopt));
        mj_forward(fixture.model.get(), fixture.data.get());
    }

    // Operation digests — staged calls with the result boxed between stage
    // and commit, mirroring each binding's actual transaction boundary.
    Digest prepare_op(DriveFixture &fixture) {
        auto tick = fixture.writer.stage_prepare(
            {.control = {}, .dt = .002, .braking = false, .active = true,
             .advance = true, .contact = true, .slip = std::nullopt,
             .ceiling = std::nullopt});
        Digest digest{
            .numbers = {tick.result.effort_nm,
                        tick.result.required_cadence_rpm,
                        tick.result.target_phase_rad ? 1. : 0.,
                        tick.result.target_phase_rad.value_or(0.),
                        tick.result.target_rate_rad_s},
            .strings = {tick.result.mode, tick.result.reason}};
        fixture.writer.commit(tick);
        return digest;
    }
    Digest components_op(DriveFixture &fixture, bool advance) {
        auto tick = fixture.writer.stage_components(
            {.control = {}, .dt = .002, .speed = 2., .sensed = 20.,
             .braking = false, .active = true, .advance = advance,
             .contact = true, .pedaling = std::nullopt, .slip = std::nullopt});
        Digest digest;
        for (const auto &[name, row]: tick.components) {
            digest.strings.push_back(name);
            digest.numbers.insert(digest.numbers.end(), row.begin(), row.end());
        }
        fixture.writer.commit(tick);
        return digest;
    }
    Digest advance_op(DriveFixture &fixture) {
        return components_op(fixture, true);
    }
    Digest probe_op(DriveFixture &fixture) {
        return components_op(fixture, false);
    }
    Digest settle_op(DriveFixture &fixture) {
        auto settlement = fixture.writer.stage_settle();
        Digest digest;
        digest.numbers.assign(settlement.force.begin(),
                              settlement.force.end());
        fixture.writer.commit(settlement);
        return digest;
    }
    Digest reset_op(DriveFixture &fixture) {
        fixture.writer.reset();
        return {};
    }

    // The enumeration driver from the plan: run the control once, measure
    // the operation's allocation surface once, then inject at every position
    // — each failure must publish nothing, and the disarmed retry must match
    // the control bitwise.
    template <typename Op>
    void sweep_allocations(std::string_view label, DriveFixture &fixture,
                           const drivetrain::DriveSnapshot &armed,
                           const Op &op) {
        const auto fail = [&label](const char *what) {
            require(false, std::string(label) + ": " + what);
        };
        fixture.writer.restore(armed);
        const Digest control_result = op(fixture);
        const drivetrain::DriveSnapshot control_post = fixture.writer.state();
        const ModelRows control_model = model_rows(*fixture.model);
        const std::vector<double> control_ctrl =
                rows(fixture.data->ctrl, fixture.model->nu);
        std::size_t count = 0;
        fixture.writer.restore(armed);
        {
            const allocation_faults::Guard guard;
            allocation_faults::arm(std::numeric_limits<std::size_t>::max());
            const Digest measured = op(fixture);
            count = allocation_faults::allocated();
            if (!(same(measured, control_result) &&
                  same(fixture.writer.state(), control_post) &&
                  same(model_rows(*fixture.model), control_model) &&
                  rows(fixture.data->ctrl, fixture.model->nu) == control_ctrl))
                fail("the armed measurement run deviates from the control");
        }
        if (count == 0)
            fail("the operation exposes no enumerable allocations");
        // Compact positions table for the enumeration report.
        std::cout << "  positions " << label << " = " << count << '\n';
        for (std::size_t fail_at = 0; fail_at < count; ++fail_at) {
            fixture.writer.restore(armed);
            const drivetrain::DriveSnapshot pre = fixture.writer.state();
            if (!same(pre, armed))
                fail("restore must reproduce the armed baseline bitwise");
            const ModelRows pre_model = model_rows(*fixture.model);
            const std::vector<double> pre_ctrl =
                    rows(fixture.data->ctrl, fixture.model->nu);
            bool injected = false;
            {
                const allocation_faults::Guard guard;
                allocation_faults::arm(fail_at);
                try {
                    static_cast<void>(op(fixture));
                } catch (const std::bad_alloc &) {
                    injected = true;
                }
            }
            if (!injected)
                fail("an armed allocation position ran to completion");
            if (!(same(fixture.writer.state(), pre) &&
                  same(model_rows(*fixture.model), pre_model) &&
                  rows(fixture.data->ctrl, fixture.model->nu) == pre_ctrl))
                fail("an injected failure published partial state");
            const Digest retry = op(fixture);
            if (!(same(retry, control_result) &&
                  same(fixture.writer.state(), control_post) &&
                  same(model_rows(*fixture.model), control_model) &&
                  rows(fixture.data->ctrl, fixture.model->nu) == control_ctrl))
                fail("the disarmed retry deviates from the control");
        }
    }

    // The A12 reproduction: restore a state whose `last` is the sparse {} a
    // partial restore leaves, with the pending actuation still reserved, then
    // sweep every settlement allocation position. The failed settles publish
    // nothing — battery, pending, telemetry, transmissions — and the retry
    // debits exactly once, never twice.
    void test_settlement_sparse_last_atomicity() {
        // Heap — a full writer plus several snapshots exceeds the frame
        // budget several times over.
        const auto fixture = std::make_unique<DriveFixture>(
            "ideal_mid_drive", "plain",
            drive_config("ideal_mid_drive", "plain", true, false));
        prime_pending(*fixture);
        drivetrain::DriveSnapshot armed = fixture->writer.state();
        require(armed.pending_actuation && armed.battery.drawn_energy_j == 0.,
                "fixture primes a pending actuation on a full battery");
        armed.last.clear();
        fixture->writer.restore(armed);
        require(fixture->writer.state().last.empty() &&
                fixture->writer.state().pending_actuation,
                "sparse restore leaves last={} with the pending reserved");
        // The clean control: one settlement, one debit, pending released.
        static_cast<void>(settle_op(*fixture));
        const drivetrain::DriveSnapshot settled = fixture->writer.state();
        require(settled.battery.drawn_energy_j > 0. &&
                !settled.pending_actuation && !settled.last.empty(),
                "control settlement debits once and releases pending");
        const double drawn_once = settled.battery.drawn_energy_j;
        static_cast<void>(settle_op(*fixture));
        require(fixture->writer.state().battery.drawn_energy_j == drawn_once,
                "a settlement with no pending actuation debits nothing");
        sweep_allocations("sparse last settlement", *fixture, armed,
                          settle_op);
        const drivetrain::DriveSnapshot post = fixture->writer.state();
        require(post.battery.energy_j == settled.battery.energy_j &&
                post.battery.drawn_energy_j == settled.battery.drawn_energy_j,
                "injected-failure retries debit the battery exactly once");
    }

    // Every supported drivetrain topology, every transaction shape: each
    // operation's full allocation surface is enumerated from a restored
    // armed baseline.
    void test_drivetrain_allocation_enumeration() {
        struct Variant {
            const char *kind, *topology;
            bool assist, shifting;
        };
        for (const Variant variant: {
                 Variant{.kind = "elastic_chain", .topology = "plain",
                         .assist = true, .shifting = false},
                 Variant{.kind = "ideal_mid_drive", .topology = "plain",
                         .assist = true, .shifting = false},
                 Variant{.kind = "ideal_mid_drive", .topology = "plain",
                         .assist = true, .shifting = true},
                 Variant{.kind = "ideal_mid_drive", .topology = "clutch",
                         .assist = true, .shifting = false},
                 Variant{.kind = "ideal_mid_drive", .topology = "rotor",
                         .assist = true, .shifting = false},
                 Variant{.kind = "ideal_mid_drive", .topology = "plain",
                         .assist = false, .shifting = false},
                 Variant{.kind = "geometric_ideal_mid_drive",
                         .topology = "plain", .assist = true,
                         .shifting = false},
                 Variant{.kind = "geometric_ideal_mid_drive",
                         .topology = "plain", .assist = true,
                         .shifting = true},
                 Variant{.kind = "geometric_ideal_mid_drive",
                         .topology = "clutch", .assist = true,
                         .shifting = false},
                 Variant{.kind = "geometric_ideal_mid_drive",
                         .topology = "rotor", .assist = true,
                         .shifting = false},
                 Variant{.kind = "geometric_ideal_mid_drive",
                         .topology = "plain", .assist = false,
                         .shifting = false}}) {
            const std::string prefix =
                    std::string(variant.kind) + "/" + variant.topology +
                    (variant.shifting ? "/shift" : "") +
                    (variant.assist ? "" : "/noassist");
            // Heap — the writer plus baseline snapshots far exceed the
            // frame budget.
            const auto fixture = std::make_unique<DriveFixture>(
                variant.kind, variant.topology,
                drive_config(variant.kind, variant.topology, variant.assist,
                             variant.shifting));
            const drivetrain::DriveSnapshot armed_clean =
                    fixture->writer.state();
            prime_pending(*fixture);
            const drivetrain::DriveSnapshot armed_primed =
                    fixture->writer.state();
            // reset() reverts a pending-bearing, possibly-shifted state.
            sweep_allocations(prefix + " reset", *fixture, armed_primed,
                              reset_op);
            // restore() commits a pending-bearing candidate onto a clean
            // baseline — the revert-and-retry shape the enumeration needs.
            sweep_allocations(prefix + " restore", *fixture, armed_clean,
                              [&armed_primed](DriveFixture &f) -> Digest {
                                  f.writer.restore(armed_primed);
                                  return {};
                              });
            sweep_allocations(prefix + " prepare", *fixture, armed_clean,
                              prepare_op);
            sweep_allocations(prefix + " advance", *fixture, armed_clean,
                              advance_op);
            // Probe with the pending still reserved exercises the
            // probe_last publication on top of live pending state.
            sweep_allocations(prefix + " probe", *fixture, armed_primed,
                              probe_op);
            sweep_allocations(prefix + " settle", *fixture, armed_primed,
                              settle_op);
            // The sparse-last{} settlement — the A12 double-debit shape.
            drivetrain::DriveSnapshot armed_sparse = armed_primed;
            armed_sparse.last.clear();
            sweep_allocations(prefix + " settle sparse", *fixture,
                              armed_sparse, settle_op);
        }
    }

    void test_cblas_and_norm() {
        // The typed enum faces must reach the linked CBLAS symbols —
        // each call below distinguishes routing from a decorative
        // pass-through by changing the buffer interpretation.
        const std::array a3{1., 2., 3.}, b3{4., 5., 6.};
        require(blas::ddot(3, a3.data(), 1, b3.data(), 1) == 32.,
                "cblas ddot dot product");
        require(blas::ddot(0, a3.data(), 1, b3.data(), 1) == 0.,
                "cblas ddot zero length");

        const std::array amat{1., 2., 3., 4., 5., 6.};
        const std::array x3{1., 1., 1.};
        std::array y2{0., 0.};
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 2, 3, 1.,
                    amat.data(), 3, x3.data(), 1, 0., y2.data(), 1);
        require(y2[0] == 6. && y2[1] == 15., "dgemv row-major no-transpose");
        const std::array x2{1., 1.};
        std::array y3{0., 0., 0.};
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 2, 3, 1.,
                    amat.data(), 3, x2.data(), 1, 0., y3.data(), 1);
        require(y3[0] == 5. && y3[1] == 7. && y3[2] == 9.,
                "dgemv row-major transpose");
        // The same buffer under column_major reads as [[1,3,5],[2,4,6]].
        std::array z2{0., 0.};
        blas::dgemv(blas::Order::column_major, blas::Transpose::no, 2, 3,
                    1., amat.data(), 2, x3.data(), 1, 0., z2.data(), 1);
        require(z2[0] == 9. && z2[1] == 12.,
                "dgemv column-major interpretation");

        const std::array a22{1., 2., 3., 4.}, b22{5., 6., 7., 8.};
        std::array c22{0., 0., 0., 0.};
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::no, 2, 2, 2, 1., a22.data(), 2,
                    b22.data(), 2, 0., c22.data(), 2);
        require(c22[0] == 19. && c22[1] == 22. && c22[2] == 43. &&
                        c22[3] == 50.,
                "dgemm row-major product");

        // CPython vector_norm (math.hypot) oracle corpus — expected bits
        // generated by CPython 3.14 math.hypot on this platform; covers
        // zero-length-adjacent singles, signed zero, the subnormal
        // rescale path, huge inputs that would overflow a naive sum,
        // near-cancellation, and repeated values.
        struct NormCase {
            std::vector<double> values;
            std::uint64_t expected_bits;
        };
        const std::array<NormCase, 13> norm_cases{{
            {.values = {0.0}, .expected_bits = 0x0000000000000000ULL},
            {.values = {-0.0}, .expected_bits = 0x0000000000000000ULL},
            {.values = {3.4}, .expected_bits = 0x400b333333333333ULL},
            {.values = {1.7976931348623157e308},
             .expected_bits = 0x7fefffffffffffffULL},
            {.values = {5e-324}, .expected_bits = 0x0000000000000001ULL},
            {.values = {2.2250738585072014e-308},
             .expected_bits = 0x0010000000000000ULL},
            {.values = {3.0, 4.0}, .expected_bits = 0x4014000000000000ULL},
            {.values = {1e308, 1e308},
             .expected_bits = 0x7fe92c80954c51f5ULL},
            {.values = {1e308, -1e308},
             .expected_bits = 0x7fe92c80954c51f5ULL},
            {.values = {1.0000000000000002, 1.0},
             .expected_bits = 0x3ff6a09e667f3bcdULL},
            {.values = {1e-300, 1e-300, 1e-300},
             .expected_bits = 0x01b28f1f70999505ULL},
            {.values = {5e-324, 5e-324, 5e-324, 5e-324},
             .expected_bits = 0x0000000000000002ULL},
            {.values = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
             .expected_bits = 0x3fd43d136248490fULL},
        }};
        for (const NormCase &norm_case: norm_cases) {
            const double got = numeric::python_vector_norm(norm_case.values);
            require(std::bit_cast<std::uint64_t>(got) ==
                            norm_case.expected_bits,
                    "python_vector_norm oracle bytes");
        }
        require(numeric::python_vector_norm({}) == 0.,
                "python_vector_norm empty span");
        const std::array nanv{1., std::numeric_limits<double>::quiet_NaN()};
        require(std::isnan(numeric::python_vector_norm(nanv)),
                "python_vector_norm nan propagation");
        const std::array infv{std::numeric_limits<double>::infinity(), 0.};
        require(std::isinf(numeric::python_vector_norm(infv)),
                "python_vector_norm inf short-circuit");
    }

    // C4: the closed-mode enums are compile-time contracts — the parsers
    // map exactly the declared wire labels, profile_gain dispatches every
    // enumerator, and the declared noexcept surface holds. A wrong mapping
    // or a widened domain fails the build, not just this case.
    void test_closed_mode_enums() {
        using namespace drivetrain;

        static_assert(profile_mode("eco") == ProfileMode::eco &&
                      profile_mode("tour") == ProfileMode::tour &&
                      profile_mode("emtb") == ProfileMode::emtb &&
                      profile_mode("turbo") == ProfileMode::turbo &&
                      !profile_mode("supersport") && !profile_mode(""),
                      "profile_mode maps exactly the four profiled labels");
        static_assert(slip_mode("legacy_signed") == SlipMode::legacy_signed &&
                      slip_mode("magnitude") == SlipMode::magnitude &&
                      !slip_mode("mystery") && !slip_mode(""),
                      "slip_mode maps exactly the two slip labels");
        static_assert(shift_direction("none") == ShiftDirection::none &&
                      shift_direction("up") == ShiftDirection::up &&
                      shift_direction("down") == ShiftDirection::down &&
                      !shift_direction("sideways") && !shift_direction(""),
                      "shift_direction maps exactly the three wire labels");
        static_assert(shift_direction_name(ShiftDirection::none) == "none" &&
                      shift_direction_name(ShiftDirection::up) == "up" &&
                      shift_direction_name(ShiftDirection::down) == "down",
                      "shift_direction_name round-trips the wire strings");

        static_assert(noexcept(profile_mode(std::string_view{})) &&
                      noexcept(slip_mode(std::string_view{})) &&
                      noexcept(shift_direction(std::string_view{})) &&
                      noexcept(shift_direction_name(ShiftDirection::none)) &&
                      noexcept(force_component_index(ForceComponent::chain)),
                      "enum parsers and index helpers stay noexcept");

        constexpr MotorProfile profile{
            .eco = .5, .tour = 1., .emtb_low = .8, .emtb_high = 2.,
            .turbo = 2.5, .emtb_full_gain_at_nm = 50.};
        static_assert(profile_gain(profile, ProfileMode::eco, 10.) == .5 &&
                      profile_gain(profile, ProfileMode::tour, 10.) == 1. &&
                      profile_gain(profile, ProfileMode::turbo, 10.) == 2.5 &&
                      profile_gain(profile, ProfileMode::emtb, 0.) == .8 &&
                      profile_gain(profile, ProfileMode::emtb, 50.) == 2.,
                      "profile_gain dispatches every enumerator");
        static_assert(noexcept(profile_gain(profile, ProfileMode::eco, 0.)),
                      "profile_gain is declared noexcept");
        // The eMTB ramp interpolates between the low/high gains with the
        // sensed rider torque, saturating at emtb_full_gain_at_nm.
        require(profile_gain(profile, ProfileMode::emtb, 25.) ==
                        profile.emtb_low +
                            (profile.emtb_high - profile.emtb_low) *
                                std::min(1., std::max(0., 25.) /
                                             profile.emtb_full_gain_at_nm),
                "emtb ramp keeps the declared interpolation order");

        // A profiled controller resolves the enum once at construction and
        // dispatches on it — last_gain is the profiled gain, not the scalar.
        {
            auto config = valid_assist_config();
            config.profile = profile;
            config.mode = "turbo";
            config.gain = 9.9; // must not leak into the profiled path
            AssistController controller(config);
            const double torque =
                    controller.step(10., 50., 0., false, .01);
            require(torque > 0. && controller.state().last_gain == 2.5,
                    "profiled step reports the enum-resolved turbo gain");
        }
        // Unknown labels are rejected only while a profile is installed;
        // the same label without a profile stays an open custom mode that
        // uses the configured scalar gain.
        {
            auto config = valid_assist_config();
            config.profile = profile;
            config.mode = "supersport";
            require_throws_invalid_argument(
                [&] { [[maybe_unused]] const AssistController c(config); },
                "profiled assist rejects an unknown mode");
        }
        {
            auto config = valid_assist_config();
            config.mode = "supersport"; // custom profile-less label
            config.gain = 1.5;
            AssistController controller(config);
            const double torque =
                    controller.step(10., 50., 0., false, .01);
            require(torque > 0. && controller.state().last_gain == 1.5,
                    "profile-less custom label keeps the scalar gain path");
        }

        // Slip-mode parsing drives the upshift gate: "magnitude" compares
        // |slip| against the limit, "legacy_signed" keeps the sign.
        {
            auto shifting = valid_shifting_config();
            shifting.upshift_slip_mode = "magnitude";
            CadenceShifter shifter(valid_gearing_config(), shifting);
            require(!shifter.update(100., 85., .01, true, false, true, -.3),
                    "magnitude mode blocks an upshift on |slip| over limit");
            require(shifter.state().rear_teeth == 18 &&
                        shifter.state().direction == "none",
                    "blocked shift keeps gear and none direction");
        }
        {
            CadenceShifter shifter(valid_gearing_config(),
                                   valid_shifting_config()); // legacy_signed
            require(shifter.update(100., 85., .01, true, false, true, -.3),
                    "legacy_signed mode ignores negative slip");
            require(shifter.state().rear_teeth == 15 &&
                        shifter.state().direction == "up",
                    "upshift lands on the next smaller sprocket");
        }
        {
            CadenceShifter shifter(valid_gearing_config(),
                                   valid_shifting_config());
            require(!shifter.update(100., 85., .01, true, false, true, .3),
                    "legacy_signed still blocks positive slip over limit");
        }
        {   // A downshift serializes "down" through the same enum path.
            CadenceShifter shifter(valid_gearing_config(),
                                   valid_shifting_config());
            require(shifter.update(50., 80., .01, true, false, true,
                                   std::nullopt),
                    "low cadence downshifts");
            require(shifter.state().rear_teeth == 21 &&
                        shifter.state().direction == "down",
                    "downshift lands on the next larger sprocket");
        }
        {   // set_state accepts only the serialized direction domain.
            CadenceShifter shifter(valid_gearing_config(),
                                   valid_shifting_config());
            ShiftingSnapshot snapshot = shifter.state();
            snapshot.direction = "sideways";
            require_throws_invalid_argument(
                [&] { shifter.set_state(snapshot); },
                "set_state rejects an unknown direction");
            snapshot.direction = "up";
            shifter.set_state(snapshot);
            require(shifter.state().direction == "up",
                    "set_state accepts a declared direction");
        }
    }

    // C4: the serialized component rows are addressed by the ForceComponent
    // table, not by emplace order — the emitted names and their order are
    // checked against the declared table, and the bearing row carries its
    // force at the right dof (external output, not private structure).
    void test_force_component_layout() {
        using namespace drivetrain;

        static_assert(force_component_specs.size() == 4 &&
                      force_component_index(ForceComponent::chain) == 0 &&
                      force_component_index(ForceComponent::freehub) == 1 &&
                      force_component_index(ForceComponent::drive_bearings) == 2 &&
                      force_component_index(ForceComponent::ideal_transmission) == 3,
                      "component slots match the declared enum positions");
        static_assert(force_component_specs[0].name == "chain" &&
                      force_component_specs[1].name == "freehub" &&
                      force_component_specs[2].name == "drive_bearings" &&
                      force_component_specs[3].name == "ideal_transmission",
                      "component names match the serialized layout");

        {   // Physical chain: three rows, in declared order.
            const auto fixture = std::make_unique<DriveFixture>(
                "elastic_chain", "plain",
                drive_config("elastic_chain", "plain", true, false));
            auto tick = fixture->writer.stage_components(
                {.control = {}, .dt = .002, .speed = 2., .sensed = 20.,
                 .braking = false, .active = true, .advance = true,
                 .contact = true, .pedaling = std::nullopt,
                 .slip = std::nullopt});
            require(tick.components.size() == 3,
                    "elastic chain emits exactly three component rows");
            for (std::size_t i = 0; i < tick.components.size(); ++i)
                require(tick.components[i].first ==
                                std::string(force_component_specs[i].name),
                        "emitted name matches the declared table slot");
            // qvel rear_wheel_spin = 5 → bearing force -0.03*5 at its dof.
            const int wheel = joint_dof(fixture->model.get(),
                                        "rear_wheel_spin");
            const auto &bearings =
                    tick.components[force_component_index(
                                        ForceComponent::drive_bearings)]
                        .second;
            require(bearings[static_cast<std::size_t>(wheel)] < 0.,
                    "bearing row carries force at the wheel dof");
            // The snapshot telemetry keeps the wire strings — enum plumbing
            // must not leak into serialization.
            require(std::get<std::string>(
                        tick.snapshot.last.at("assist_mode")) == "turbo" &&
                    std::get<std::string>(
                        tick.snapshot.last.at("shift_direction")) == "none",
                    "telemetry keeps the public serialization strings");
            fixture->writer.commit(tick);
        }
        {   // Simplified topology appends ideal_transmission last.
            const auto fixture = std::make_unique<DriveFixture>(
                "ideal_mid_drive", "plain",
                drive_config("ideal_mid_drive", "plain", true, false));
            auto tick = fixture->writer.stage_components(
                {.control = {}, .dt = .002, .speed = 2., .sensed = 20.,
                 .braking = false, .active = true, .advance = true,
                 .contact = true, .pedaling = std::nullopt,
                 .slip = std::nullopt});
            require(tick.components.size() == 4 &&
                        tick.components[force_component_index(
                                            ForceComponent::ideal_transmission)]
                                    .first == "ideal_transmission",
                    "simplified model appends the ideal_transmission row");
            for (std::size_t i = 0; i < tick.components.size(); ++i)
                require(tick.components[i].first ==
                                std::string(force_component_specs[i].name),
                        "emitted order follows the declared table");
            fixture->writer.commit(tick);
        }
    }

    constexpr std::array<TestCase, 28> cases{{
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
        {.name = "model_access_contracts", .run = test_model_access_contracts},
        {.name = "geometry_topology_and_ids", .run = test_geometry_topology_and_ids},
        {.name = "geometry_nonfinite_transform", .run = test_geometry_nonfinite_transform},
        {.name = "profile_contract_boundaries", .run = test_profile_contract_boundaries},
        {.name = "support_geometry_parent_contracts", .run = test_support_geometry_parent_contracts},
        {.name = "settlement_sparse_last_atomicity", .run = test_settlement_sparse_last_atomicity},
        {.name = "drivetrain_allocation_enumeration", .run = test_drivetrain_allocation_enumeration},
        {.name = "cblas_and_norm", .run = test_cblas_and_norm},
        {.name = "closed_mode_enums", .run = test_closed_mode_enums},
        {.name = "force_component_layout", .run = test_force_component_layout},
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
