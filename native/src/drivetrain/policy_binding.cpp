#include "policy_binding.hpp"
#include "../diag.hpp"
#include "../binding_readers.hpp"
#include "../config.hpp"
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
    double num(const wire::Dict &d, const char *key) { return wire::finite_real(field(d, key), d.child(key)); }
    bool boolean(const wire::Dict &d, const char *key) { return wire::boolean(field(d, key), d.child(key)); }
    int integer(const wire::Dict &d, const char *key) { return wire::integer32(field(d, key), d.child(key)); }
    std::string string(const wire::Dict &d, const char *key) { return wire::string(field(d, key), d.child(key)); }
    std::optional<double> optional_num(const wire::Dict &d, const char *key) {
        const auto value = field(d, key);
        return value.is_none() ? std::nullopt : std::optional(wire::finite_real(value, d.child(key)));
    }

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
        // Closed enums materialize their wire strings only here at the
        // FFI boundary — the core carries PedalMode/CoastReason ordinals.
        d["mode"] = std::string(pedal_mode_name(s.mode));
        d["reason"] = std::string(coast_reason_name(s.reason));
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
            const ShiftingSnapshot shift{
                .rear_teeth = integer(s, "rear_teeth"), .from_teeth = integer(s, "from_teeth"),
                .shift_count = integer(s, "shift_count"),
                .cooldown_s = num(s, "cooldown_s"), .cut_remaining_s = num(s, "cut_remaining_s"),
                .direction = string(s, "direction"), .cadence_ema = optional_num(s, "cadence_ema"),
                .required_ema = optional_num(s, "required_ema")
            };
            next.shifting.set_state(shift);
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
            // Relational checks: a candidate must agree with the config and
            // with itself, not merely carry field-valid values.
            const bool shifted = shift.shift_count > 0;
            if ((shift.direction != "none") != shifted ||
                (!shifted && (shift.cooldown_s != 0. || shift.cut_remaining_s != 0.)))
                throw std::invalid_argument("shift count/direction mismatch");
            const auto &cassette = config.shifting.cassette;
            if (config.shifting.enabled &&
                (!std::ranges::contains(cassette, shift.rear_teeth) ||
                 !std::ranges::contains(cassette, shift.from_teeth)))
                throw std::invalid_argument("shifter teeth outside the cassette");
            const auto &batt = next.battery.state();
            if (batt.initial_energy_j != config.battery.energy_j ||
                batt.energy_j > batt.initial_energy_j || batt.drawn_energy_j > batt.initial_energy_j)
                throw std::invalid_argument("battery energy outside the configured store");
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

// The policy-subtree decode is the shared generic parser
// (writer_config.hpp::decode_drive_policies); this entry stays only as the
// nanobind-dict adapter so DrivePolicies(dict) keeps its wire:: reader
// strictness and 'config'-rooted paths.
DrivePolicyConfig parse_drive_policy_config(const nb::dict &raw, std::span<const std::string_view> extra_root_keys) {
    return nativecfg::detail::decode_drive_policies(
        nativecfg::detail::Dict<nativecfg::detail::NbNode>{
            .value = nativecfg::detail::NbNode{nb::borrow<nb::object>(raw)},
            .path = extra_root_keys.empty() ? "config" : "config.drive"},
        extra_root_keys);
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
