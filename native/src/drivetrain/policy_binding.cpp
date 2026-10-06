#include "policy_binding.hpp"
#include "../diag.hpp"
#include "../binding_readers.hpp"
#include "pedaling.hpp"
#include "shifting.hpp"
#include "motor.hpp"
#include "freehub.hpp"
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <algorithm>
#include <limits>

namespace nb = nanobind;
using namespace drivetrain;

namespace {
    double number(nb::handle value, const char *key) { return wire::finite_real(value, key); }
    double num(const wire::Dict &d, const char *key) { return wire::finite_real(field(d, key), d.child(key)); }
    bool boolean(const wire::Dict &d, const char *key) { return wire::boolean(field(d, key), d.child(key)); }
    int integer(nb::handle value, const char *key) { return wire::integer32(value, key); }
    int integer(const wire::Dict &d, const char *key) { return wire::integer32(field(d, key), d.child(key)); }
    std::string string(const wire::Dict &d, const char *key) { return wire::string(field(d, key), d.child(key)); }
    std::optional<double> optional_num(const wire::Dict &d, const char *key) {
        const auto value = field(d, key);
        return value.is_none() ? std::nullopt : std::optional(wire::finite_real(value, d.child(key)));
    }
    nb::list sequence(nb::handle value, const char *key) { return wire::sequence(value, key); }

    nb::dict pedaling_dict(const PedalingSnapshot &s) {
        nb::dict d;
        d["coasting"] = s.coasting;
        d["target_phase_rad"] = nb::cast(s.target_phase_rad);
        d["target_rate_rad_s"] = s.target_rate_rad_s;
        d["deceleration_rad_s2"] = s.deceleration_rad_s2;
        d["_effort"] = s.effort;
        d["_cadence_ema"] = nb::cast(s.cadence_ema);
        return d;
    }

    nb::dict shifting_dict(const ShiftingSnapshot &s) {
        nb::dict d;
        d["rear_teeth"] = s.rear_teeth;
        d["from_teeth"] = s.from_teeth;
        d["cooldown_s"] = s.cooldown_s;
        d["cut_remaining_s"] = s.cut_remaining_s;
        d["shift_count"] = s.shift_count;
        d["direction"] = s.direction;
        d["cadence_ema"] = nb::cast(s.cadence_ema);
        d["required_ema"] = nb::cast(s.required_ema);
        return d;
    }

    nb::dict result_dict(const PedalingState &s) {
        nb::dict d;
        d["mode"] = s.mode;
        d["reason"] = s.reason;
        d["effort_nm"] = s.effort_nm;
        d["required_cadence_rpm"] = s.required_cadence_rpm;
        d["target_phase_rad"] = nb::cast(s.target_phase_rad);
        d["target_rate_rad_s"] = s.target_rate_rad_s;
        return d;
    }

    class DrivePolicies {
    public:
        explicit DrivePolicies(nb::handle input) : config(parse_drive_policy_config(wire::mapping(input, "config"))),
                                                        pedaling(config.pedaling),
                                                        shifting(config.gearing, config.shifting),
                                                        assist(config.assist),
                                                        battery(config.battery.energy_j),
                                                        hub(config.hub_stiffness_nm_rad, config.hub_damping_nm_s) {
        }

        void reset() {
            pedaling.reset();
            shifting.reset();
            assist.reset();
            battery.reset();
            hub.reset();
        }

        nb::dict state() const {
            nb::dict d, a, b, h;
            const auto &as = assist.state();
            const auto &bs = battery.state();
            const auto &hs = hub.state();
            a["torque"] = as.torque;
            a["pedaling"] = as.pedaling;
            a["last_gain"] = as.last_gain;
            b["initial_energy_j"] = bs.initial_energy_j;
            b["energy_j"] = bs.energy_j;
            b["drawn_energy_j"] = bs.drawn_energy_j;
            h["boundary"] = nb::cast(hs.boundary);
            h["energy_j"] = hs.energy_j;
            h["torque_nm"] = hs.torque_nm;
            d["pedaling"] = pedaling_dict(pedaling.state());
            d["shifting"] = shifting_dict(shifting.state());
            d["assist"] = a;
            d["battery"] = b;
            d["hub"] = h;
            return d;
        }

        void set_state(nb::handle raw) {
            const wire::Dict input{.value = wire::mapping(raw, "state"), .path = "state"};
            wire::exact(input, wire::keys("pedaling", "shifting", "assist", "battery", "hub"));
            const auto p = section(input, "pedaling"), s = section(input, "shifting"), a = section(input, "assist"),
                    b = section(input, "battery"), h = section(input, "hub");
            // Restore into typed copies: every field is parsed/validated before commit.
            auto next = *this;
            next.pedaling.set_state({
                .coasting = boolean(p, "coasting"), .target_phase_rad = optional_num(p, "target_phase_rad"),
                .target_rate_rad_s = num(p, "target_rate_rad_s"), .deceleration_rad_s2 = num(p, "deceleration_rad_s2"),
                .effort = num(p, "_effort"), .cadence_ema = optional_num(p, "_cadence_ema")
            });
            next.shifting.set_state({
                .rear_teeth = integer(s, "rear_teeth"), .from_teeth = integer(s, "from_teeth"),
                .shift_count = integer(s, "shift_count"),
                .cooldown_s = num(s, "cooldown_s"), .cut_remaining_s = num(s, "cut_remaining_s"),
                .direction = string(s, "direction"), .cadence_ema = optional_num(s, "cadence_ema"),
                .required_ema = optional_num(s, "required_ema")
            });
            next.assist.set_state({
                .torque = num(a, "torque"), .last_gain = num(a, "last_gain"), .pedaling = boolean(a, "pedaling")
            });
            next.battery.set_state({
                .initial_energy_j = num(b, "initial_energy_j"), .energy_j = num(b, "energy_j"),
                .drawn_energy_j = num(b, "drawn_energy_j")
            });
            next.hub.set_state({
                .boundary = optional_num(h, "boundary"), .energy_j = num(h, "energy_j"),
                .torque_nm = num(h, "torque_nm")
            });
            wire::exact(p, wire::keys("coasting", "target_phase_rad", "target_rate_rad_s", "deceleration_rad_s2", "_effort", "_cadence_ema"));
            wire::exact(s, wire::keys("rear_teeth", "from_teeth", "cooldown_s", "cut_remaining_s", "shift_count", "direction", "cadence_ema", "required_ema"));
            wire::exact(a, wire::keys("torque", "pedaling", "last_gain"));
            wire::exact(b, wire::keys("initial_energy_j", "energy_j", "drawn_energy_j"));
            wire::exact(h, wire::keys("boundary", "energy_j", "torque_nm"));
            *this = std::move(next);
        }

        DrivePolicyConfig config;
        PedalingPolicy pedaling;
        CadenceShifter shifting;
        AssistController assist;
        Battery battery;
        Freehub hub;
    };
}

DrivePolicyConfig parse_drive_policy_config(const nb::dict &raw, std::span<const std::string_view> extra_root_keys) {
    const wire::Dict input{.value = raw, .path = extra_root_keys.empty() ? "config" : "config.drive"};
    constexpr auto root = wire::keys("gearing", "pedaling", "shifting", "assist", "battery", "hub_stiffness_nm_rad", "hub_damping_nm_s");
    wire::exact(input, root, extra_root_keys);
    DrivePolicyConfig out;
    const auto g = section(input, "gearing"), p = section(input, "pedaling"), s = section(input, "shifting"),
            a = section(input, "assist"), b = section(input, "battery");
    wire::exact(g, wire::keys("front_teeth","rear_teeth","chain_pitch_m"), wire::keys());
    wire::exact(p, wire::keys("enabled","coast_above_rpm","resume_below_rpm","stop_time_s","coast_cadence_tau_s","mash_cadence_rpm","mash_torque_nm","effort_slew_nm_s"), wire::keys());
    wire::exact(s, wire::keys("enabled","cassette","target_cadence_min_rpm","target_cadence_max_rpm","shift_cooldown_s","shift_cut_duration_s","torque_factor","cadence_smoothing_tau_s","upshift_slip_limit_mps","upshift_slip_mode"), wire::keys());
    wire::exact(a, wire::keys("gain","max_torque","max_power","tau","slew","engage_torque_nm","gate_min_crank_rad_s","cutoff_mps","taper_width_mps","mode"), wire::keys("profile","torque_curve"));
    wire::exact(b, wire::keys("enabled","energy_j","copper_w_per_nm2","speed_w_per_rad_s2","idle_w"), wire::keys());
    out.gearing = {
        .front_teeth = integer(g, "front_teeth"), .rear_teeth = integer(g, "rear_teeth"),
        .chain_pitch_m = positive(num(g, "chain_pitch_m"), "chain_pitch_m")
    };
    if (out.gearing.front_teeth < 3 || out.gearing.rear_teeth < 3) throw std::invalid_argument("gearing teeth");
    out.pedaling = {
        .enabled = boolean(p, "enabled"), .coast_above_rpm = positive(num(p, "coast_above_rpm"), "coast_above_rpm"),
        .resume_below_rpm = nonnegative(num(p, "resume_below_rpm"), "resume_below_rpm"),
        .stop_time_s = positive(num(p, "stop_time_s"), "stop_time_s"),
        .coast_cadence_tau_s = nonnegative(num(p, "coast_cadence_tau_s"), "coast_cadence_tau_s"),
        .mash_cadence_rpm = nonnegative(num(p, "mash_cadence_rpm"), "mash_cadence_rpm"),
        .mash_torque_nm = nonnegative(num(p, "mash_torque_nm"), "mash_torque_nm"),
        .effort_slew_nm_s = nonnegative(num(p, "effort_slew_nm_s"), "effort_slew_nm_s")
    };
    if (out.pedaling.resume_below_rpm >= out.pedaling.coast_above_rpm || (
            out.pedaling.mash_torque_nm > 0. && out.pedaling.mash_cadence_rpm <= 0.)) throw std::invalid_argument(
        "pedaling cadence band");
    auto &sh = out.shifting;
    sh.enabled = boolean(s, "enabled");
    for (nb::handle const value: sequence(field(s, "cassette"), s.child("cassette").c_str())) {
        const int teeth = integer(value, s.child("cassette").c_str());
        if (teeth < 3) throw std::invalid_argument("cassette");
        sh.cassette.push_back(teeth);
    }
    std::ranges::sort(sh.cassette);
    if (sh.cassette.empty() || std::ranges::adjacent_find(sh.cassette) != sh.cassette.end()) throw
            std::invalid_argument("cassette");
    sh.target_cadence_min_rpm = positive(num(s, "target_cadence_min_rpm"), "target_cadence_min_rpm");
    sh.target_cadence_max_rpm = positive(num(s, "target_cadence_max_rpm"), "target_cadence_max_rpm");
    sh.shift_cooldown_s = nonnegative(num(s, "shift_cooldown_s"), "shift_cooldown_s");
    sh.shift_cut_duration_s = nonnegative(num(s, "shift_cut_duration_s"), "shift_cut_duration_s");
    sh.torque_factor = nonnegative(num(s, "torque_factor"), "torque_factor");
    sh.cadence_smoothing_tau_s = nonnegative(num(s, "cadence_smoothing_tau_s"), "cadence_smoothing_tau_s");
    sh.upshift_slip_limit_mps = nonnegative(num(s, "upshift_slip_limit_mps"), "upshift_slip_limit_mps");
    sh.upshift_slip_mode = string(s, "upshift_slip_mode");
    if (sh.upshift_slip_mode != "legacy_signed" && sh.upshift_slip_mode != "magnitude")
        wire::invalid(s.child("upshift_slip_mode"), "unsupported mode");
    if (sh.target_cadence_max_rpm <= sh.target_cadence_min_rpm || sh.shift_cut_duration_s > sh.shift_cooldown_s ||
        sh.torque_factor > 1.) throw std::invalid_argument("shifting config");
    if (sh.enabled && std::ranges::find(sh.cassette, out.gearing.rear_teeth) == sh.cassette.end()) throw
            std::invalid_argument("initial rear teeth");
    auto &ac = out.assist;
    ac.gain = nonnegative(num(a, "gain"), "gain");
    ac.max_torque = nonnegative(num(a, "max_torque"), "max_torque");
    ac.max_power = nonnegative(num(a, "max_power"), "max_power");
    ac.tau = positive(num(a, "tau"), "tau");
    ac.slew = positive(num(a, "slew"), "slew");
    ac.engage_torque_nm = nonnegative(num(a, "engage_torque_nm"), "engage_torque_nm");
    ac.gate_min_crank_rad_s = nonnegative(num(a, "gate_min_crank_rad_s"), "gate_min_crank_rad_s");
    ac.cutoff_mps = nonnegative(num(a, "cutoff_mps"), "cutoff_mps");
    ac.taper_width_mps = positive(num(a, "taper_width_mps"), "taper_width_mps");
    ac.mode = string(a, "mode");
    if (ac.taper_width_mps > ac.cutoff_mps) throw std::invalid_argument("taper_width_mps");
    if (a.contains("torque_curve") && !a["torque_curve"].is_none()) {
        std::vector<std::array<double, 2> > rows;
        for (nb::handle const row: sequence(a["torque_curve"], a.child("torque_curve").c_str())) {
            const auto values = sequence(row, a.child("torque_curve").c_str());
            if (values.size() != 2) throw std::invalid_argument("torque_curve row");
            const double rpm = nonnegative(number(values[0], a.child("torque_curve").c_str()), "curve rpm");
            const double torque = nonnegative(number(values[1], a.child("torque_curve").c_str()), "curve torque");
            if (!rows.empty() && rpm <= rows.back()[0]) throw std::invalid_argument("curve rpm order");
            rows.push_back({rpm, torque});
        }
        if (rows.size() < 2) throw std::invalid_argument("torque_curve");
        ac.torque_curve = std::move(rows);
    }
    if (a.contains("profile") && !a["profile"].is_none()) {
        const auto profile = section(a, "profile"), gains = section(profile, "mode_gains");
        const auto emtb = sequence(field(gains, "emtb"), gains.child("emtb").c_str());
        wire::exact(profile, wire::keys("mode_gains", "emtb_full_gain_at_nm"));
        wire::exact(gains, wire::keys("eco", "tour", "emtb", "turbo"));
        if (emtb.size() != 2) throw std::invalid_argument("mode_gains");
        ac.profile = MotorProfile{
            .eco = nonnegative(num(gains, "eco"), "eco"), .tour = nonnegative(num(gains, "tour"), "tour"),
            .emtb_low = nonnegative(number(emtb[0], gains.child("emtb").c_str()), "emtb low"),
            .emtb_high = nonnegative(number(emtb[1], gains.child("emtb").c_str()), "emtb high"),
            .turbo = nonnegative(num(gains, "turbo"), "turbo"),
            .emtb_full_gain_at_nm = positive(num(profile, "emtb_full_gain_at_nm"), "emtb_full_gain_at_nm")
        };
        if (ac.mode != "eco" && ac.mode != "tour" && ac.mode != "emtb" && ac.mode != "turbo")
            wire::invalid(a.child("mode"), "unsupported profiled mode");
        if (ac.profile->emtb_low > ac.profile->emtb_high)
            wire::invalid(gains.child("emtb"), "unordered mode gains");
    }
    out.battery = {
        .enabled = boolean(b, "enabled"), .energy_j = nonnegative(num(b, "energy_j"), "energy_j"),
        .copper_w_per_nm2 = nonnegative(num(b, "copper_w_per_nm2"), "copper_w_per_nm2"),
        .speed_w_per_rad_s2 = nonnegative(num(b, "speed_w_per_rad_s2"), "speed_w_per_rad_s2"),
        .idle_w = nonnegative(num(b, "idle_w"), "idle_w")
    };
    out.hub_stiffness_nm_rad = positive(num(input, "hub_stiffness_nm_rad"), "hub_stiffness_nm_rad");
    out.hub_damping_nm_s = nonnegative(num(input, "hub_damping_nm_s"), "hub_damping_nm_s");
    return out;
}

// Import-time binding glue; see bind_drivetrain for the frame-size note.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
// NOLINTBEGIN(bugprone-easily-swappable-parameters) fixed Python signatures; raw handles preserve original input types
void bind_drive_policies(nb::module_ &module) {
    module.def("human_crank_torque", [](nb::handle mean, nb::handle phase, nb::handle ripple) {
        return human_crank_torque(wire::finite_real(mean, "human_crank_torque.mean_nm"),
                                  wire::finite_real(phase, "human_crank_torque.phase_rad"),
                                  wire::finite_real(ripple, "human_crank_torque.ripple"));
    }, nb::arg("mean_nm").none(), nb::arg("phase_rad").none(),
               nb::arg("ripple").none() = .35);
    nb::class_<DrivePolicies>(module, "DrivePolicies")
            .def(nb::init<nb::handle>(), nb::arg("config").none())
            .def("reset", &DrivePolicies::reset)
            .def("state", &DrivePolicies::state)
            .def("set_state", &DrivePolicies::set_state, nb::arg("state").none())
            .def("pedaling_update", [](DrivePolicies &self, nb::handle phase, nb::handle rate, nb::handle required, nb::handle effort,
                                       nb::handle dt, nb::handle enabled, nb::handle braking) {
                     const auto checked_phase = wire::finite_real(phase, "pedaling_update.phase_rad");
                     const auto checked_rate = wire::finite_real(rate, "pedaling_update.rate_rad_s");
                     const auto checked_required = wire::finite_real(required, "pedaling_update.required_cadence_rpm");
                     const auto checked_effort = wire::finite_real(effort, "pedaling_update.effort_nm");
                     const auto checked_dt = wire::finite_real(dt, "pedaling_update.dt");
                     const auto checked_enabled = wire::boolean(enabled, "pedaling_update.enabled");
                     const auto checked_braking = wire::boolean(braking, "pedaling_update.braking");
                     return result_dict(self.pedaling.update(checked_phase, checked_rate, checked_required, checked_effort, checked_dt, checked_enabled, checked_braking));
                 }, nb::arg("phase_rad").none(), nb::arg("rate_rad_s").none(), nb::arg("required_cadence_rpm").none(), nb::arg("effort_nm").none(),
                 nb::arg("dt").none(), nb::arg("enabled").none() = true, nb::arg("braking").none() = false)
            .def("shifting_update", [](DrivePolicies &self, nb::handle cadence, nb::handle required, nb::handle dt, nb::handle pedaling,
                                       nb::handle braking, nb::handle contact, nb::handle slip) {
                     const auto checked_cadence = wire::finite_real(cadence, "shifting_update.cadence_rpm");
                     const auto checked_required = wire::finite_real(required, "shifting_update.required_cadence_rpm");
                     const auto checked_dt = wire::finite_real(dt, "shifting_update.dt");
                     const auto checked_pedaling = wire::boolean(pedaling, "shifting_update.pedaling");
                     const auto checked_braking = wire::boolean(braking, "shifting_update.braking");
                     const auto checked_contact = wire::boolean(contact, "shifting_update.rear_in_contact");
                     const auto checked_slip = wire::optional_real(slip, "shifting_update.rear_slip_mps");
                     return self.shifting.update(checked_cadence, checked_required, checked_dt, checked_pedaling, checked_braking, checked_contact, checked_slip);
                 }, nb::arg("cadence_rpm").none(), nb::arg("required_cadence_rpm").none(), nb::arg("dt").none(), nb::arg("pedaling").none() = true,
                 nb::arg("braking").none() = false, nb::arg("rear_in_contact").none() = true, nb::arg("rear_slip_mps").none() = nb::none())
            .def("shifting_diagnostics", [](const DrivePolicies &self) {
                auto d = shifting_dict(self.shifting.state());
                d["gear_ratio"] = self.shifting.gear_ratio();
                d["torque_factor"] = self.shifting.torque_factor();
                return d;
            })
            .def("assist_ceiling",
                 [](const DrivePolicies &self, nb::handle rpm, nb::handle speed) {
                     const auto checked_rpm = wire::finite_real(rpm, "assist_ceiling.shaft_rpm");
                     const auto checked_speed = wire::finite_real(speed, "assist_ceiling.speed_mps"); return self.assist.ceiling(checked_rpm, checked_speed); },
                 nb::arg("shaft_rpm").none(), nb::arg("speed_mps").none())
            .def("assist_step", [](DrivePolicies &self, nb::handle human, nb::handle rpm, nb::handle speed, nb::handle braking, nb::handle dt,
                                   nb::handle request, nb::handle shaft) {
                     const auto checked_human = wire::finite_real(human, "assist_step.human_nm");
                     const auto checked_rpm = wire::finite_real(rpm, "assist_step.cadence_rpm");
                     const auto checked_speed = wire::finite_real(speed, "assist_step.speed_mps");
                     const auto checked_braking = wire::boolean(braking, "assist_step.braking");
                     const auto checked_dt = wire::finite_real(dt, "assist_step.dt");
                     const auto checked_request = wire::optional_real(request, "assist_step.torque_request_nm");
                     const auto checked_shaft = wire::optional_real(shaft, "assist_step.shaft_rpm");
                     return self.assist.step(checked_human, checked_rpm, checked_speed, checked_braking, checked_dt, checked_request, checked_shaft);
                 }, nb::arg("human_nm").none(), nb::arg("cadence_rpm").none(), nb::arg("speed_mps").none(), nb::arg("braking").none(),
                 nb::arg("dt").none(),
                 nb::arg("torque_request_nm").none() = nb::none(), nb::arg("shaft_rpm").none() = nb::none())
            .def("battery_draw",
                 [](DrivePolicies &self, nb::handle power, nb::handle dt) {
                     const auto checked_power = wire::finite_real(power, "battery_draw.requested_power_w");
                     const auto checked_dt = wire::finite_real(dt, "battery_draw.dt"); return self.battery.draw(checked_power, checked_dt); },
                 nb::arg("requested_power_w").none(), nb::arg("dt").none())
            .def("electrical_power", [](const DrivePolicies &self, nb::handle torque, nb::handle omega, nb::handle enabled) {
                     const auto checked_torque = wire::finite_real(torque, "electrical_power.torque");
                     const auto checked_omega = wire::finite_real(omega, "electrical_power.omega");
                     const auto checked_enabled = wire::boolean(enabled, "electrical_power.enabled");
                const auto &b = self.config.battery;
                return motor_electrical_power(checked_torque, checked_omega, b.copper_w_per_nm2, b.speed_w_per_rad_s2, b.idle_w,
                                              checked_enabled);
            }, nb::arg("torque").none(), nb::arg("omega").none(), nb::arg("enabled").none())
            .def("energy_limit", [](const DrivePolicies &self, nb::handle request, nb::handle omega, nb::handle budget) {
                     const auto checked_request = wire::finite_real(request, "energy_limit.request");
                     const auto checked_omega = wire::finite_real(omega, "energy_limit.omega");
                     const auto checked_budget = wire::finite_real(budget, "energy_limit.budget_w");
                const auto &b = self.config.battery;
                return limit_torque_by_energy(checked_request, checked_omega, b.copper_w_per_nm2, b.speed_w_per_rad_s2, b.idle_w,
                                              checked_budget);
            }, nb::arg("request").none(), nb::arg("omega").none(), nb::arg("budget_w").none())
            .def("freehub_torque",
                 [](DrivePolicies &self, nb::handle pc, nb::handle pw, nb::handle wc, nb::handle ww) {
                     const auto checked_pc = wire::finite_real(pc, "freehub_torque.phi_c");
                     const auto checked_pw = wire::finite_real(pw, "freehub_torque.phi_w");
                     const auto checked_wc = wire::finite_real(wc, "freehub_torque.omega_c");
                     const auto checked_ww = wire::finite_real(ww, "freehub_torque.omega_w");
                     return self.hub.update(checked_pc, checked_pw, checked_wc, checked_ww);
                 }, nb::arg("phi_c").none(), nb::arg("phi_w").none(), nb::arg("omega_c").none(), nb::arg("omega_w").none());
}

// NOLINTEND(bugprone-easily-swappable-parameters)
NATIVE_DIAG_POP
