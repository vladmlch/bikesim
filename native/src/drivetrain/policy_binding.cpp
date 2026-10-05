#include "policy_binding.hpp"
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
nb::handle field(const nb::dict& d, const char* key) {
    if (!d.contains(key)) throw std::invalid_argument(key);
    return d[key];
}
nb::dict section(const nb::dict& d, const char* key) {
    const auto value = field(d, key);
    if (!nb::isinstance<nb::dict>(value)) throw std::invalid_argument(key);
    return nb::borrow<nb::dict>(value);
}
double number(nb::handle value, const char* key) {
    if ((!nb::isinstance<nb::float_>(value) && !nb::isinstance<nb::int_>(value)) || nb::isinstance<nb::bool_>(value)) throw std::invalid_argument(key);
    try { return finite(nb::cast<double>(value), key); }
    catch (const nb::cast_error&) { throw std::invalid_argument(key); }
}
double num(const nb::dict& d, const char* key) { return number(field(d, key), key); }
bool boolean(const nb::dict& d, const char* key) {
    const auto v = field(d, key);
    if (!nb::isinstance<nb::bool_>(v)) throw std::invalid_argument(key);
    return nb::cast<bool>(v);
}
int integer(nb::handle v, const char* key) {
    if (!nb::isinstance<nb::int_>(v) || nb::isinstance<nb::bool_>(v)) throw std::invalid_argument(key);
    try { return nb::cast<int>(v); }
    catch (const nb::cast_error&) { throw std::invalid_argument(key); }
}
int integer(const nb::dict& d, const char* key) { return integer(field(d, key), key); }
std::string string(const nb::dict& d, const char* key) {
    const auto v = field(d, key);
    if (!nb::isinstance<nb::str>(v)) throw std::invalid_argument(key);
    return nb::cast<std::string>(v);
}
std::optional<double> optional_num(const nb::dict& d, const char* key) {
    const auto v = field(d, key);
    return v.is_none() ? std::nullopt : std::optional<double>(number(v, key));
}
nb::list sequence(nb::handle value, const char* key) {
    if (!nb::isinstance<nb::list>(value) && !nb::isinstance<nb::tuple>(value)) throw std::invalid_argument(key);
    return nb::list(value);
}
void width(const nb::dict& d, std::size_t size) {
    if (d.size() != size) throw std::invalid_argument("state fields");
}
nb::dict pedaling_dict(const PedalingSnapshot& s) {
    nb::dict d;
    d["coasting"] = s.coasting; d["target_phase_rad"] = nb::cast(s.target_phase_rad);
    d["target_rate_rad_s"] = s.target_rate_rad_s; d["deceleration_rad_s2"] = s.deceleration_rad_s2;
    d["_effort"] = s.effort; d["_cadence_ema"] = nb::cast(s.cadence_ema); return d;
}
nb::dict shifting_dict(const ShiftingSnapshot& s) {
    nb::dict d;
    d["rear_teeth"] = s.rear_teeth; d["from_teeth"] = s.from_teeth;
    d["cooldown_s"] = s.cooldown_s; d["cut_remaining_s"] = s.cut_remaining_s;
    d["shift_count"] = s.shift_count; d["direction"] = s.direction;
    d["cadence_ema"] = nb::cast(s.cadence_ema); d["required_ema"] = nb::cast(s.required_ema); return d;
}
nb::dict result_dict(const PedalingState& s) {
    nb::dict d;
    d["mode"] = s.mode; d["reason"] = s.reason; d["effort_nm"] = s.effort_nm;
    d["required_cadence_rpm"] = s.required_cadence_rpm;
    d["target_phase_rad"] = nb::cast(s.target_phase_rad); d["target_rate_rad_s"] = s.target_rate_rad_s; return d;
}
class DrivePolicies {
public:
    explicit DrivePolicies(const nb::dict& input) : config(parse_drive_policy_config(input)),
        pedaling(config.pedaling), shifting(config.gearing, config.shifting), assist(config.assist),
        battery(config.battery.energy_j), hub(config.hub_stiffness_nm_rad, config.hub_damping_nm_s) {}
    void reset() { pedaling.reset(); shifting.reset(); assist.reset(); battery.reset(); hub.reset(); }
    nb::dict state() const {
        nb::dict d, a, b, h;
        const auto& as = assist.state(); const auto& bs = battery.state(); const auto& hs = hub.state();
        a["torque"] = as.torque; a["pedaling"] = as.pedaling; a["last_gain"] = as.last_gain;
        b["initial_energy_j"] = bs.initial_energy_j; b["energy_j"] = bs.energy_j; b["drawn_energy_j"] = bs.drawn_energy_j;
        h["boundary"] = nb::cast(hs.boundary); h["energy_j"] = hs.energy_j; h["torque_nm"] = hs.torque_nm;
        d["pedaling"] = pedaling_dict(pedaling.state()); d["shifting"] = shifting_dict(shifting.state());
        d["assist"] = a; d["battery"] = b; d["hub"] = h; return d;
    }
    void set_state(const nb::dict& input) {
        width(input, 5);
        const auto p = section(input, "pedaling"), s = section(input, "shifting"), a = section(input, "assist"),
                   b = section(input, "battery"), h = section(input, "hub");
        // Restore into typed copies: every field is parsed/validated before commit.
        auto next = *this;
        next.pedaling.set_state({boolean(p,"coasting"), optional_num(p,"target_phase_rad"),
            num(p,"target_rate_rad_s"), num(p,"deceleration_rad_s2"), num(p,"_effort"), optional_num(p,"_cadence_ema")});
        next.shifting.set_state({integer(s,"rear_teeth"), integer(s,"from_teeth"), integer(s,"shift_count"),
            num(s,"cooldown_s"), num(s,"cut_remaining_s"), string(s,"direction"), optional_num(s,"cadence_ema"), optional_num(s,"required_ema")});
        next.assist.set_state({num(a,"torque"), num(a,"last_gain"), boolean(a,"pedaling")});
        next.battery.set_state({num(b,"initial_energy_j"), num(b,"energy_j"), num(b,"drawn_energy_j")});
        next.hub.set_state({optional_num(h,"boundary"), num(h,"energy_j"), num(h,"torque_nm")});
        width(p, 6); width(s, 8); width(a, 3); width(b, 3); width(h, 3);
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

DrivePolicyConfig parse_drive_policy_config(const nb::dict& input) {
    DrivePolicyConfig out;
    const auto g = section(input,"gearing"), p = section(input,"pedaling"), s = section(input,"shifting"),
               a = section(input,"assist"), b = section(input,"battery");
    out.gearing = {integer(g,"front_teeth"), integer(g,"rear_teeth"), positive(num(g,"chain_pitch_m"),"chain_pitch_m")};
    if (out.gearing.front_teeth < 3 || out.gearing.rear_teeth < 3) throw std::invalid_argument("gearing teeth");
    out.pedaling = {boolean(p,"enabled"), positive(num(p,"coast_above_rpm"),"coast_above_rpm"),
        nonnegative(num(p,"resume_below_rpm"),"resume_below_rpm"), positive(num(p,"stop_time_s"),"stop_time_s"),
        nonnegative(num(p,"coast_cadence_tau_s"),"coast_cadence_tau_s"), nonnegative(num(p,"mash_cadence_rpm"),"mash_cadence_rpm"),
        nonnegative(num(p,"mash_torque_nm"),"mash_torque_nm"), nonnegative(num(p,"effort_slew_nm_s"),"effort_slew_nm_s")};
    if (out.pedaling.resume_below_rpm >= out.pedaling.coast_above_rpm || (out.pedaling.mash_torque_nm > 0. && out.pedaling.mash_cadence_rpm <= 0.)) throw std::invalid_argument("pedaling cadence band");
    auto& sh = out.shifting;
    sh.enabled = boolean(s,"enabled");
    for (nb::handle value : sequence(field(s,"cassette"),"cassette")) {
        const int teeth = integer(value,"cassette");
        if (teeth < 3) throw std::invalid_argument("cassette");
        sh.cassette.push_back(teeth);
    }
    std::ranges::sort(sh.cassette);
    if (sh.cassette.empty() || std::adjacent_find(sh.cassette.begin(),sh.cassette.end()) != sh.cassette.end()) throw std::invalid_argument("cassette");
    sh.target_cadence_min_rpm = positive(num(s,"target_cadence_min_rpm"),"target_cadence_min_rpm");
    sh.target_cadence_max_rpm = positive(num(s,"target_cadence_max_rpm"),"target_cadence_max_rpm");
    sh.shift_cooldown_s = nonnegative(num(s,"shift_cooldown_s"),"shift_cooldown_s");
    sh.shift_cut_duration_s = nonnegative(num(s,"shift_cut_duration_s"),"shift_cut_duration_s");
    sh.torque_factor = nonnegative(num(s,"torque_factor"),"torque_factor");
    sh.cadence_smoothing_tau_s = nonnegative(num(s,"cadence_smoothing_tau_s"),"cadence_smoothing_tau_s");
    sh.upshift_slip_limit_mps = nonnegative(num(s,"upshift_slip_limit_mps"),"upshift_slip_limit_mps");
    sh.upshift_slip_mode = string(s,"upshift_slip_mode");
    if (sh.target_cadence_max_rpm <= sh.target_cadence_min_rpm || sh.shift_cut_duration_s > sh.shift_cooldown_s || sh.torque_factor > 1. ||
        (sh.upshift_slip_mode != "legacy_signed" && sh.upshift_slip_mode != "magnitude")) throw std::invalid_argument("shifting config");
    if (sh.enabled && std::ranges::find(sh.cassette,out.gearing.rear_teeth) == sh.cassette.end()) throw std::invalid_argument("initial rear teeth");
    auto& ac = out.assist;
    ac.gain = nonnegative(num(a,"gain"),"gain"); ac.max_torque = nonnegative(num(a,"max_torque"),"max_torque");
    ac.max_power = nonnegative(num(a,"max_power"),"max_power"); ac.tau = positive(num(a,"tau"),"tau");
    ac.slew = positive(num(a,"slew"),"slew"); ac.engage_torque_nm = nonnegative(num(a,"engage_torque_nm"),"engage_torque_nm");
    ac.gate_min_crank_rad_s = nonnegative(num(a,"gate_min_crank_rad_s"),"gate_min_crank_rad_s");
    ac.cutoff_mps = nonnegative(num(a,"cutoff_mps"),"cutoff_mps"); ac.taper_width_mps = positive(num(a,"taper_width_mps"),"taper_width_mps");
    ac.mode = string(a,"mode");
    if (ac.taper_width_mps > ac.cutoff_mps) throw std::invalid_argument("taper_width_mps");
    if (a.contains("torque_curve") && !a["torque_curve"].is_none()) {
        std::vector<std::array<double,2>> rows;
        for (nb::handle row : sequence(a["torque_curve"],"torque_curve")) {
            const auto values = sequence(row,"torque_curve row");
            if (values.size() != 2) throw std::invalid_argument("torque_curve row");
            const double rpm = nonnegative(number(values[0],"curve rpm"),"curve rpm");
            const double torque = nonnegative(number(values[1],"curve torque"),"curve torque");
            if (!rows.empty() && rpm <= rows.back()[0]) throw std::invalid_argument("curve rpm order");
            rows.push_back({rpm,torque});
        }
        if (rows.size() < 2) throw std::invalid_argument("torque_curve");
        ac.torque_curve = std::move(rows);
    }
    if (a.contains("profile") && !a["profile"].is_none()) {
        const auto profile = section(a,"profile"), gains = section(profile,"mode_gains");
        const auto emtb = sequence(field(gains,"emtb"),"emtb");
        if (emtb.size() != 2 || gains.size() != 4) throw std::invalid_argument("mode_gains");
        ac.profile = MotorProfile{nonnegative(num(gains,"eco"),"eco"), nonnegative(num(gains,"tour"),"tour"),
            nonnegative(number(emtb[0],"emtb low"),"emtb low"), nonnegative(number(emtb[1],"emtb high"),"emtb high"),
            nonnegative(num(gains,"turbo"),"turbo"), positive(num(profile,"emtb_full_gain_at_nm"),"emtb_full_gain_at_nm")};
        if (ac.profile->emtb_low > ac.profile->emtb_high || (ac.mode != "eco" && ac.mode != "tour" && ac.mode != "emtb" && ac.mode != "turbo")) throw std::invalid_argument("profile mode/gains");
    }
    out.battery = {boolean(b,"enabled"), nonnegative(num(b,"energy_j"),"energy_j"),
        nonnegative(num(b,"copper_w_per_nm2"),"copper_w_per_nm2"), nonnegative(num(b,"speed_w_per_rad_s2"),"speed_w_per_rad_s2"), nonnegative(num(b,"idle_w"),"idle_w")};
    out.hub_stiffness_nm_rad = positive(num(input,"hub_stiffness_nm_rad"),"hub_stiffness_nm_rad");
    out.hub_damping_nm_s = nonnegative(num(input,"hub_damping_nm_s"),"hub_damping_nm_s");
    return out;
}

void bind_drive_policies(nb::module_& module) {
    module.def("human_crank_torque", &human_crank_torque, nb::arg("mean_nm"), nb::arg("phase_rad"), nb::arg("ripple") = .35);
    nb::class_<DrivePolicies>(module,"DrivePolicies")
        .def(nb::init<const nb::dict&>(), nb::arg("config"))
        .def("reset", &DrivePolicies::reset)
        .def("state", &DrivePolicies::state)
        .def("set_state", &DrivePolicies::set_state, nb::arg("state"))
        .def("pedaling_update", [](DrivePolicies& self, double phase, double rate, double required, double effort, double dt, bool enabled, bool braking) {
            return result_dict(self.pedaling.update(phase,rate,required,effort,dt,enabled,braking));
        }, nb::arg("phase_rad"), nb::arg("rate_rad_s"), nb::arg("required_cadence_rpm"), nb::arg("effort_nm"), nb::arg("dt"), nb::arg("enabled") = true, nb::arg("braking") = false)
        .def("shifting_update", [](DrivePolicies& self, double cadence, double required, double dt, bool pedaling, bool braking, bool contact, std::optional<double> slip) {
            return self.shifting.update(cadence,required,dt,pedaling,braking,contact,slip);
        }, nb::arg("cadence_rpm"), nb::arg("required_cadence_rpm"), nb::arg("dt"), nb::arg("pedaling") = true, nb::arg("braking") = false, nb::arg("rear_in_contact") = true, nb::arg("rear_slip_mps") = nb::none())
        .def("shifting_diagnostics", [](const DrivePolicies& self) {
            auto d = shifting_dict(self.shifting.state()); d["gear_ratio"] = self.shifting.gear_ratio(); d["torque_factor"] = self.shifting.torque_factor(); return d;
        })
        .def("assist_ceiling", [](const DrivePolicies& self, double rpm, double speed) { return self.assist.ceiling(rpm,speed); }, nb::arg("shaft_rpm"), nb::arg("speed_mps"))
        .def("assist_step", [](DrivePolicies& self, double human, double rpm, double speed, bool braking, double dt, std::optional<double> request, std::optional<double> shaft) {
            return self.assist.step(human,rpm,speed,braking,dt,request,shaft);
        }, nb::arg("human_nm"), nb::arg("cadence_rpm"), nb::arg("speed_mps"), nb::arg("braking"), nb::arg("dt"), nb::arg("torque_request_nm") = nb::none(), nb::arg("shaft_rpm") = nb::none())
        .def("battery_draw", [](DrivePolicies& self, double power, double dt) { return self.battery.draw(power,dt); }, nb::arg("requested_power_w"), nb::arg("dt"))
        .def("electrical_power", [](const DrivePolicies& self, double torque, double omega, bool enabled) {
            const auto& b = self.config.battery; return motor_electrical_power(torque,omega,b.copper_w_per_nm2,b.speed_w_per_rad_s2,b.idle_w,enabled);
        }, nb::arg("torque"), nb::arg("omega"), nb::arg("enabled"))
        .def("energy_limit", [](const DrivePolicies& self, double request, double omega, double budget) {
            const auto& b = self.config.battery; return limit_torque_by_energy(request,omega,b.copper_w_per_nm2,b.speed_w_per_rad_s2,b.idle_w,budget);
        }, nb::arg("request"), nb::arg("omega"), nb::arg("budget_w"))
        .def("freehub_torque", [](DrivePolicies& self, double pc, double pw, double wc, double ww) { return self.hub.update(pc,pw,wc,ww); }, nb::arg("phi_c"), nb::arg("phi_w"), nb::arg("omega_c"), nb::arg("omega_w"));
}
