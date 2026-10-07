#include "drive_binding.hpp"
#include "../diag.hpp"
#include "../binding_readers.hpp"
#include "../binding_arrays.hpp"
#include "../stepper.hpp"
#include "chain.hpp"
#include "policy_binding.hpp"
#include "transmission.hpp"
#include <algorithm>
#include <cstdint>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <set>
#include <type_traits>
namespace nb = nanobind;
using namespace drivetrain;

namespace {
    double number(nb::handle value, const char *key) { return wire::finite_real(value, key); }
    double num(const wire::Dict &d, const char *key) { return wire::finite_real(field(d, key), d.child(key)); }
    bool boolean(nb::handle value, const char *key) { return wire::boolean(value, key); }
    bool boolean(const wire::Dict &d, const char *key) { return wire::boolean(field(d, key), d.child(key)); }
    int integer(const wire::Dict &d, const char *key) { return wire::integer32(field(d, key), d.child(key)); }
    std::string string(const wire::Dict &d, const char *key) { return wire::string(field(d, key), d.child(key)); }
    std::optional<double> optional_num(const wire::Dict &d, const char *key) {
        const auto value = field(d, key);
        return value.is_none() ? std::nullopt : std::optional(wire::finite_real(value, d.child(key)));
    }
    std::vector<double> vector(nb::handle value, const char *key) { return wire::vector(value, key); }

    nb::ndarray<nb::numpy, double, nb::shape<-1> > owned(std::span<const double> values) {
        return wire::owned_array<double>(values);
    }

    nb::dict diagnostic_dict(const Diagnostics &values) {
        nb::dict d;
        for (const auto &[k, v]: values)
            std::visit(
                [&](const auto &x) {
                    using T = std::decay_t<decltype(x)>;
                    if constexpr (std::is_same_v<T, std::monostate>)
                        d[k.c_str()] = nb::none();
                    else
                        d[k.c_str()] = nb::cast(x);
                },
                v);
        return d;
    }

    Diagnostics parse_diagnostics(const wire::Dict &d) {
        Diagnostics out;
        const std::set<std::string> bools = {
            "omits_suspension_coupling",
            "freehub_engaged",
            "assist_demand_gated",
            "motor_freewheel_engaged",
            "safety_limited",
            "motor_enabled",
            "energy_limited",
            "battery_empty",
            "shift_active",
            "crank_clutch_engaged"
        };
        const std::set<std::string> integers = {
            "gear_front_teeth", "gear_rear_teeth",
            "shift_from_teeth", "shift_count"
        };
        const std::set<std::string> strings = {
            "transmission_model", "rider_mode", "coasting_reason",
            "motor_control_source", "assist_mode", "shift_direction",
            "transmission_reference_status"
        };
        const std::set<std::string> optional = {
            "human_setpoint_nm", "crank_target_phase_rad", "motor_setpoint_nm",
            "motor_limit_nm", "shift_time_s"
        };
        const std::set<std::string> numeric = {
            "chain_extension_m",
            "chain_extension_rate_mps",
            "chain_tension_n",
            "chain_energy_j",
            "chain_dissipation_power_w",
            "freehub_torque_nm",
            "freehub_energy_j",
            "freehub_deflection_rad",
            "freehub_dissipation_power_w",
            "cadence_rpm",
            "crank_rad_s",
            "drive_shaft_rad_s",
            "human_torque_nm",
            "human_sensor_nm",
            "assist_sensor_nm",
            "human_command_nm",
            "required_cadence_rpm",
            "crank_target_rate_rad_s",
            "motor_request_nm",
            "motor_torque_nm",
            "motor_freewheel_torque_nm",
            "motor_freewheel_dissipation_power_w",
            "motor_limited_request_nm",
            "motor_shaft_power_w",
            "electrical_power_w",
            "battery_energy_j",
            "assist_gain",
            "gear_ratio",
            "shift_torque_factor",
            "crank_clutch_torque_nm",
            "crank_clutch_dissipation_power_w",
            "transmission_phi_m",
            "transmission_boundary_m",
            "transmission_gap_m",
            "transmission_tension_n",
            "transmission_constraint_defect_m",
            "transmission_interval_work_j",
            "transmission_reaction_error_n",
            "shift_parameter_work_j",
            "shift_interval_constraint_work_j",
            "shift_constraint_work_cumulative_j"
        };
        for (auto const item: d.value) {
            const std::string k = wire::string(item.first, d.path);
            const auto v = item.second;
            if (!bools.contains(k) && !integers.contains(k) && !strings.contains(k) &&
                !optional.contains(k) && !numeric.contains(k))
                wire::invalid(d.child(k.c_str()), "unknown diagnostic key");
            if (bools.contains(k))
                out[k] = boolean(v, d.child(k.c_str()).c_str());
            else if (integers.contains(k)) {
                const int value = integer(d, k.c_str());
                if (value < (k == "shift_count" ? 0 : 3))
                    throw std::invalid_argument("diagnostic integer range");
                out[k] = value;
            } else if (strings.contains(k)) {
                const auto value = string(d, k.c_str());
                if ((k == "shift_direction" && value != "none" && value != "up" &&
                     value != "down") ||
                    (k == "transmission_model" && value != "elastic_chain" &&
                     value != "ideal_mid_drive" && value != "geometric_ideal_mid_drive") ||
                    (k == "rider_mode" && value != "disabled" && value != "pedaling" &&
                     value != "coasting") ||
                    (k == "motor_control_source" && value != "assist" &&
                     value != "external_request") ||
                    (k == "transmission_reference_status" &&
                     value != "experimental_geometric_reduction"))
                    throw std::invalid_argument("diagnostic enum");
                out[k] = value;
            } else if (v.is_none() && optional.contains(k))
                out[k] = std::monostate{};
            else {
                const double value = number(v, d.child(k.c_str()).c_str());
                if (k == "gear_ratio")
                    positive(value, "diagnostic gear ratio");
                out[k] = value;
            }
        }
        return out;
    }

    // Result boxing runs on the staged candidate BEFORE commit — an ndarray
    // or dict allocation failure cannot leave the tick half published.
    nb::dict box_components(const ForceComponents &components) {
        nb::dict out;
        for (const auto &[key, value]: components)
            out[key.c_str()] = owned(value);
        return out;
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

    PedalingState parse_result(const wire::Dict &d) {
        wire::exact(d, wire::keys("mode", "reason", "effort_nm", "required_cadence_rpm", "target_phase_rad", "target_rate_rad_s"));
        PedalingState s{
            .mode = string(d, "mode"),
            .reason = string(d, "reason"),
            .effort_nm = num(d, "effort_nm"),
            .required_cadence_rpm = num(d, "required_cadence_rpm"),
            .target_phase_rad = optional_num(d, "target_phase_rad"),
            .target_rate_rad_s = num(d, "target_rate_rad_s")
        };
        if (s.mode != "pedaling" && s.mode != "coasting" && s.mode != "disabled")
            throw std::invalid_argument("pedaling mode");
        nonnegative(s.effort_nm, "pedaling effort");
        return s;
    }

    RideControl control(nb::handle raw) {
        if (!nb::isinstance<nb::dict>(raw))
            throw std::invalid_argument("control");
        const wire::Dict d{.value = nb::borrow<nb::dict>(raw), .path = "control"};
        RideControl c;
        wire::exact(d, wire::keys(), wire::keys("motor_torque_nm", "motor_limit_nm", "human_torque_nm", "rider_enabled"));
        for (const char *key: {"motor_torque_nm", "motor_limit_nm", "human_torque_nm"})
            if (d.contains(key)) {
                auto x = optional_num(d, key);
                if (x)
                    nonnegative(*x, key);
                if (std::string(key) == "motor_torque_nm")
                    c.motor_torque_nm = x;
                else if (std::string(key) == "motor_limit_nm")
                    c.motor_limit_nm = x;
                else
                    c.human_torque_nm = x;
            }
        if (d.contains("rider_enabled"))
            c.rider_enabled = boolean(d, "rider_enabled");
        return c;
    }

    nb::dict policy_dict(const DriveSnapshot &s) {
        nb::dict d, p, sh, a, b, h;
        const auto &ps = s.pedaling;
        const auto &ss = s.shifting;
        p["coasting"] = ps.coasting;
        p["target_phase_rad"] = nb::cast(ps.target_phase_rad);
        p["target_rate_rad_s"] = ps.target_rate_rad_s;
        p["deceleration_rad_s2"] = ps.deceleration_rad_s2;
        p["_effort"] = ps.effort;
        p["_cadence_ema"] = nb::cast(ps.cadence_ema);
        sh["rear_teeth"] = ss.rear_teeth;
        sh["from_teeth"] = ss.from_teeth;
        sh["shift_count"] = ss.shift_count;
        sh["cooldown_s"] = ss.cooldown_s;
        sh["cut_remaining_s"] = ss.cut_remaining_s;
        sh["direction"] = ss.direction;
        sh["cadence_ema"] = nb::cast(ss.cadence_ema);
        sh["required_ema"] = nb::cast(ss.required_ema);
        a["torque"] = s.assist.torque;
        a["pedaling"] = s.assist.pedaling;
        a["last_gain"] = s.assist.last_gain;
        b["initial_energy_j"] = s.battery.initial_energy_j;
        b["energy_j"] = s.battery.energy_j;
        b["drawn_energy_j"] = s.battery.drawn_energy_j;
        d["pedaling"] = p;
        d["shifting"] = sh;
        d["assist"] = a;
        d["battery"] = b;
        if (s.hub) {
            h["boundary"] = nb::cast(s.hub->boundary);
            h["energy_j"] = s.hub->energy_j;
            h["torque_nm"] = s.hub->torque_nm;
            d["hub"] = h;
        } else
            d["hub"] = nb::none();
        return d;
    }

    void parse_policies(const wire::Dict &d, DriveSnapshot &out) {
        wire::exact(d, wire::keys("pedaling", "shifting", "assist", "battery", "hub"));
        const auto p = section(d, "pedaling"), s = section(d, "shifting"),
                a = section(d, "assist"), b = section(d, "battery");
        wire::exact(p, wire::keys("coasting", "target_phase_rad", "target_rate_rad_s", "deceleration_rad_s2", "_effort", "_cadence_ema"));
        wire::exact(s, wire::keys("rear_teeth", "from_teeth", "cooldown_s", "cut_remaining_s", "shift_count", "direction", "cadence_ema", "required_ema"));
        wire::exact(a, wire::keys("torque", "pedaling", "last_gain"));
        wire::exact(b, wire::keys("initial_energy_j", "energy_j", "drawn_energy_j"));
        out.pedaling = {
            .coasting = boolean(p, "coasting"), .target_phase_rad = optional_num(p, "target_phase_rad"),
            .target_rate_rad_s = num(p, "target_rate_rad_s"), .deceleration_rad_s2 = num(p, "deceleration_rad_s2"),
            .effort = num(p, "_effort"), .cadence_ema = optional_num(p, "_cadence_ema")
        };
        out.shifting = {
            .rear_teeth = integer(s, "rear_teeth"), .from_teeth = integer(s, "from_teeth"),
            .shift_count = integer(s, "shift_count"), .cooldown_s = num(s, "cooldown_s"),
            .cut_remaining_s = num(s, "cut_remaining_s"), .direction = string(s, "direction"),
            .cadence_ema = optional_num(s, "cadence_ema"), .required_ema = optional_num(s, "required_ema")
        };
        out.assist = {.torque = num(a, "torque"), .last_gain = num(a, "last_gain"), .pedaling = boolean(a, "pedaling")};
        out.battery = {
            .initial_energy_j = num(b, "initial_energy_j"), .energy_j = num(b, "energy_j"),
            .drawn_energy_j = num(b, "drawn_energy_j")
        };
        if (!field(d, "hub").is_none()) {
            const auto h = section(d, "hub");
            wire::exact(h, wire::keys("boundary", "energy_j", "torque_nm"));
            out.hub = FreehubSnapshot{
                .boundary = optional_num(h, "boundary"), .energy_j = num(h, "energy_j"),
                .torque_nm = num(h, "torque_nm")
            };
        }
    }

    nb::object transmission_dict(const std::optional<TransmissionSnapshot> &maybe) {
        if (!maybe)
            return nb::none();
        const auto &s = *maybe;
        nb::dict d;
        d["ratio"] = s.ratio;
        d["rear_teeth"] = s.rear_teeth;
        d["boundary"] = nb::cast(s.boundary);
        d["diagnostics"] = diagnostic_dict(s.diagnostics);
        d["shift_pending"] = s.shift_pending;
        d["shift_parameter_work_j"] = s.shift_parameter_work_j;
        d["shift_constraint_work_j"] = s.shift_constraint_work_j;
        d["last_tension_n"] = s.last_tension_n;
        d["range"] = nb::cast(s.range);
        d["coefficients"] = owned(s.coefficients);
        if (s.prepared) {
            nb::dict p;
            p["phi"] = s.prepared->phi;
            p["time"] = s.prepared->time;
            p["jacobian"] = owned(s.prepared->jacobian);
            p["qpos"] = owned(s.prepared->qpos);
            d["prepared"] = p;
        } else
            d["prepared"] = nb::none();
        return d;
    }

    std::optional<TransmissionSnapshot> parse_transmission(nb::handle input, const std::string &path) {
        if (input.is_none())
            return std::nullopt;
        if (!nb::isinstance<nb::dict>(input))
            throw std::invalid_argument("transmission state");
        const wire::Dict d{.value = nb::borrow<nb::dict>(input), .path = path};
        wire::exact(d, wire::keys("ratio", "rear_teeth", "boundary", "diagnostics", "shift_pending", "shift_parameter_work_j", "shift_constraint_work_j", "last_tension_n", "range", "coefficients", "prepared"));
        TransmissionSnapshot s;
        s.ratio = num(d, "ratio");
        s.rear_teeth = integer(d, "rear_teeth");
        s.boundary = optional_num(d, "boundary");
        s.diagnostics = parse_diagnostics(section(d, "diagnostics"));
        s.shift_pending = boolean(d, "shift_pending");
        s.shift_parameter_work_j = num(d, "shift_parameter_work_j");
        s.shift_constraint_work_j = num(d, "shift_constraint_work_j");
        s.last_tension_n = num(d, "last_tension_n");
        const auto r = vector(field(d, "range"), d.child("range").c_str());
        if (r.size() != 2)
            throw std::invalid_argument("range width");
        s.range = {r[0], r[1]};
        s.coefficients = vector(field(d, "coefficients"), d.child("coefficients").c_str());
        if (!field(d, "prepared").is_none()) {
            const auto p = section(d, "prepared");
            wire::exact(p, wire::keys("phi", "time", "jacobian", "qpos"));
            s.prepared = PreparedTransmission{
                .phi = num(p, "phi"), .time = num(p, "time"),
                .jacobian = vector(field(p, "jacobian"), p.child("jacobian").c_str()),
                .qpos = vector(field(p, "qpos"), p.child("qpos").c_str())
            };
        }
        return s;
    }

    nb::dict state_dict(const DriveSnapshot &s) {
        nb::dict d;
        d["policies"] = policy_dict(s);
        d["shift_time_s"] = nb::cast(s.shift_time_s);
        d["last_time_s"] = nb::cast(s.last_time_s);
        d["reference"] = nb::cast(s.reference);
        d["psi"] = nb::cast(s.psi);
        d["angles"] = nb::cast(s.angles);
        d["last"] = diagnostic_dict(s.last);
        d["probe_last"] = s.probe_last
                              ? nb::object(diagnostic_dict(*s.probe_last))
                              : nb::object(nb::none());
        if (s.pending_actuation) {
            nb::dict p;
            p["requested"] = s.pending_actuation->requested;
            p["omega"] = s.pending_actuation->omega;
            p["dt"] = s.pending_actuation->dt;
            p["enabled"] = s.pending_actuation->enabled;
            d["pending_actuation"] = p;
        } else
            d["pending_actuation"] = nb::none();
        d["ideal_hub"] = transmission_dict(s.ideal_hub);
        d["clutch"] = transmission_dict(s.clutch);
        d["freewheel"] = transmission_dict(s.freewheel);
        return d;
    }

    DriveSnapshot parse_state(const wire::Dict &d) {
        wire::exact(d, wire::keys("policies", "shift_time_s", "last_time_s", "reference", "psi", "angles", "last", "probe_last", "pending_actuation", "ideal_hub", "clutch", "freewheel"));
        DriveSnapshot s;
        parse_policies(section(d, "policies"), s);
        s.shift_time_s = optional_num(d, "shift_time_s");
        s.last_time_s = optional_num(d, "last_time_s");
        s.reference = optional_num(d, "reference");
        s.psi = optional_num(d, "psi");
        if (!field(d, "angles").is_none())
            s.angles = vector(field(d, "angles"), d.child("angles").c_str());
        s.last = parse_diagnostics(section(d, "last"));
        if (!field(d, "probe_last").is_none())
            s.probe_last = parse_diagnostics(section(d, "probe_last"));
        if (!field(d, "pending_actuation").is_none()) {
            const auto p = section(d, "pending_actuation");
            wire::exact(p, wire::keys("requested", "omega", "dt", "enabled"));
            s.pending_actuation = PendingActuation{
                .requested = num(p, "requested"), .omega = num(p, "omega"),
                .dt = num(p, "dt"), .enabled = boolean(p, "enabled")
            };
            if (s.pending_actuation->dt <= 0.)
                wire::invalid(p.child("dt"), "dt must be positive");
        }
        s.ideal_hub = parse_transmission(field(d, "ideal_hub"), d.child("ideal_hub"));
        s.clutch = parse_transmission(field(d, "clutch"), d.child("clutch"));
        s.freewheel = parse_transmission(field(d, "freewheel"), d.child("freewheel"));
        return s;
    }
} // namespace
DriveConfig parse_drive_config(const nb::dict &input) {
    const wire::Dict d{.value = input, .path = "config.drive"};
    DriveConfig c;
    constexpr auto extra_keys = wire::keys("drive_mode","transmission_model","human_torque_nm","torque_ripple","crank_phase_rad","chain_k_n_m","chain_c_ns_m","bearing_c_nms_rad","rotor_inertia_kgm2","motor_clutch");
    c.policies = parse_drive_policy_config(d.value, extra_keys);
    c.drive_mode = string(d, "drive_mode");
    c.transmission_model = string(d, "transmission_model");
    if (c.drive_mode != "crank_effort" && c.drive_mode != "articulated_effort" &&
        c.drive_mode != "coast" && c.drive_mode != "ideal_speed_control")
        wire::invalid(d.child("drive_mode"), "unsupported mode");
    if (c.transmission_model != "elastic_chain" &&
        c.transmission_model != "ideal_mid_drive" &&
        c.transmission_model != "geometric_ideal_mid_drive")
        wire::invalid(d.child("transmission_model"), "unsupported model");
    c.human_torque_nm = nonnegative(num(d, "human_torque_nm"), "human_torque_nm");
    c.torque_ripple = nonnegative(num(d, "torque_ripple"), "torque_ripple");
    if (c.torque_ripple >= 1.)
        throw std::invalid_argument("torque_ripple");
    c.crank_phase_rad = num(d, "crank_phase_rad");
    c.chain_k_n_m = positive(num(d, "chain_k_n_m"), "chain_k_n_m");
    c.chain_c_ns_m = nonnegative(num(d, "chain_c_ns_m"), "chain_c_ns_m");
    c.bearing_c_nms_rad = nonnegative(num(d, "bearing_c_nms_rad"), "bearing_c_nms_rad");
    c.rotor_inertia_kgm2 =
            nonnegative(num(d, "rotor_inertia_kgm2"), "rotor_inertia_kgm2");
    c.motor_clutch = boolean(d, "motor_clutch");
    if (c.motor_clutch && c.rotor_inertia_kgm2 > 0.)
        throw std::invalid_argument("rotor and clutch are exclusive");
    if (c.transmission_model == "elastic_chain" &&
        (c.motor_clutch || c.rotor_inertia_kgm2 > 0. || c.policies.shifting.enabled))
        throw std::invalid_argument(
            "rotor/clutch/shifting requires ideal transmission");
    if ((c.drive_mode == "coast" || c.drive_mode == "ideal_speed_control") &&
        (c.motor_clutch || c.rotor_inertia_kgm2 > 0.))
        throw std::invalid_argument("passive drive cannot use rotor/clutch");
    return c;
}

// Import-time binding glue; its ~15KB frame is nanobind temporaries,
// so the 8KB guard keeps covering the per-step code elsewhere.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void bind_drivetrain(nb::module_ &module, nb::class_<Stepper> &cls) {
    cls.def("_drive_prepared_storage", [](Stepper &owner) {
        const auto *transmission = owner.drive().transmission_storage();
        const auto *storage = transmission ? transmission->prepared_storage() : nullptr;
        if (!storage)
            return nb::object(nb::none());
        const auto [jacobian_generation, qpos_generation] =
                transmission->prepared_generations();
        nb::dict const result;
        // Storage identity crosses the FFI as generation counters, never
        // as raw addresses: a generation bumps only when its buffer's
        // data() pointer changes (see Transmission::prepared_storage).
        result["jacobian_generation"] = jacobian_generation;
        result["qpos_generation"] = qpos_generation;
        result["jacobian_capacity"] = storage->jacobian.capacity();
        result["qpos_capacity"] = storage->qpos.capacity();
        return nb::object(result);
    });
    // NOLINTNEXTLINE(bugprone-throw-keyword-missing,bugprone-unused-raii,misc-const-correctness) RAII translator registration, see bind_rider_contact_math
    nb::exception<ArithmeticError>(module, "DriveArithmeticError",
                                   PyExc_ArithmeticError);
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python geometric argument order
    module.def("chain_geometry", [](nb::handle front, nb::handle rear, nb::handle rf, nb::handle rr, nb::handle up, nb::handle ref) {
        auto const geometry = chain_geometry(wire::fixed<2>(front, "chain_geometry.front"),
            wire::fixed<2>(rear, "chain_geometry.rear"), wire::finite_real(rf, "chain_geometry.front_radius"),
            wire::finite_real(rr, "chain_geometry.rear_radius"), wire::fixed<2>(up, "chain_geometry.up"),
            wire::optional_real(ref, "chain_geometry.reference"));
        return std::pair{geometry.length, geometry.psi};
    }, nb::arg("front").none(), nb::arg("rear").none(), nb::arg("rf").none(), nb::arg("rr").none(),
       nb::arg("up").none(), nb::arg("reference").none());
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python geometric argument order
    module.def("chain_center_gradient", [](nb::handle front, nb::handle rear, nb::handle rf, nb::handle rr, nb::handle up, nb::handle ref) {
        return chain_center_gradient(wire::fixed<2>(front, "chain_center_gradient.front"),
            wire::fixed<2>(rear, "chain_center_gradient.rear"), wire::finite_real(rf, "chain_center_gradient.front_radius"),
            wire::finite_real(rr, "chain_center_gradient.rear_radius"), wire::fixed<2>(up, "chain_center_gradient.up"),
            wire::optional_real(ref, "chain_center_gradient.reference"));
    }, nb::arg("front").none(), nb::arg("rear").none(), nb::arg("rf").none(), nb::arg("rr").none(),
       nb::arg("up").none(), nb::arg("reference").none());
    cls.def("drive_reset", [](Stepper &s) {
                s.mutate([&] { s.drive().reset(); });
            })
            .def("drive_restart_clock", [](Stepper &s) {
                s.mutate([&] { s.drive().restart_clock(); });
            })
            .def(
                "drive_prepare_pedaling",
                // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional kwargs mirror the Python def
                [](Stepper &s, nb::handle raw, nb::handle dt, nb::handle braking,
                   nb::handle active, nb::handle advance, nb::handle contact,
                   nb::handle slip, nb::handle ceiling) {
                    return s.mutate([&] {
                        auto tick = s.drive().stage_prepare(
                            {.control = control(raw),
                             .dt = wire::finite_real(dt, "drive_prepare_pedaling.dt"),
                             .braking = boolean(braking, "drive_prepare_pedaling.braking"),
                             .active = boolean(active, "drive_prepare_pedaling.active"),
                             .advance = boolean(advance, "drive_prepare_pedaling.advance"),
                             .contact = boolean(contact, "drive_prepare_pedaling.rear_in_contact"),
                             .slip = wire::optional_real(slip, "drive_prepare_pedaling.rear_slip_mps"),
                             .ceiling = wire::optional_real(ceiling, "drive_prepare_pedaling.effort_ceiling_nm")});
                        nb::dict const out = result_dict(tick.result);
                        s.drive().commit(tick);
                        return out;
                    });
                },
                nb::arg("control").none(), nb::arg("dt").none(), nb::arg("braking").none() = false,
                nb::arg("active").none() = true, nb::arg("advance").none() = true,
                nb::arg("rear_in_contact").none() = true, nb::arg("rear_slip_mps").none() = nb::none(),
                nb::arg("effort_ceiling_nm").none() = nb::none())
            .def(
                "drive_components",
                [](Stepper &s, nb::handle raw, nb::handle dt, nb::handle speed, nb::handle braking,
                   nb::handle active, nb::handle advance, nb::handle sensed, nb::handle ps,
                   nb::handle contact, nb::handle slip) {
                    // NOLINTEND(bugprone-easily-swappable-parameters)
                    return s.mutate([&] {
                        std::optional<PedalingState> state;
                        if (!ps.is_none()) {
                            if (!nb::isinstance<nb::dict>(ps))
                                throw std::invalid_argument("pedaling_state");
                            state = parse_result({.value = nb::borrow<nb::dict>(ps), .path = "pedaling_state"});
                        }
                        auto tick = s.drive().stage_components(
                            {.control = control(raw),
                             .dt = wire::finite_real(dt, "drive_components.dt"),
                             .speed = wire::finite_real(speed, "drive_components.speed_mps"),
                             .sensed = wire::finite_real(sensed, "drive_components.sensed_human_torque_nm"),
                             .braking = boolean(braking, "drive_components.braking"),
                             .active = boolean(active, "drive_components.active"),
                             .advance = boolean(advance, "drive_components.advance"),
                             .contact = boolean(contact, "drive_components.rear_in_contact"),
                             .pedaling = state,
                             .slip = wire::optional_real(slip, "drive_components.rear_slip_mps")});
                        nb::dict const out = box_components(tick.components);
                        s.drive().commit(tick);
                        return out;
                    });
                },
                nb::arg("control").none(), nb::arg("dt").none(), nb::arg("speed_mps").none(),
                nb::arg("braking").none() = false, nb::arg("active").none() = true,
                nb::arg("advance").none() = true, nb::arg("sensed_human_torque_nm").none() = 0.,
                nb::arg("pedaling_state").none() = nb::none(), nb::arg("rear_in_contact").none() = true,
                nb::arg("rear_slip_mps").none() = nb::none())
            .def("drive_settle_actuation",
                 [](Stepper &s) {
                     // The staged settlement owns everything commit()
                     // publishes; boxing the force array first means an
                     // ndarray allocation failure cannot leave the
                     // drivetrain half settled (battery debited, pending
                     // consumed, telemetry written).
                     return s.mutate([&] {
                         auto settlement = s.drive().stage_settle();
                         auto out = owned(settlement.force);
                         s.drive().commit(settlement);
                         return out;
                     });
                 })
            .def(
                "drive_diagnostics",
                [](Stepper &s, nb::handle probe) {
                    return diagnostic_dict(s.drive().diagnostics(boolean(probe, "drive_diagnostics.probe")));
                },
                nb::arg("probe").none() = false)
            .def("drive_stored_energy",
                 [](Stepper &s) { return diagnostic_dict(s.drive().stored_energy()); })
            .def("drive_state", [](Stepper &s) { return state_dict(s.drive().state()); })
            .def(
                "set_drive_state",
                [](Stepper &s, nb::handle state) {
                    s.mutate([&] { s.drive().restore(parse_state({.value = wire::mapping(state, "state"), .path = "state"})); });
                },
                nb::arg("state").none())
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python ctrl/force signature
            .def("set_inputs", [](Stepper &s, nb::handle ctrl, nb::handle force) {
                const auto controls = wire::vector(ctrl, "set_inputs.ctrl");
                const auto forces = wire::vector(force, "set_inputs.qfrc_applied");
                s.set_inputs(controls, forces);
            }, nb::arg("ctrl").none(), nb::arg("qfrc_applied").none())
            // Zero-copy views over mjData's fixed buffers — same declared
            // owner policy as the views in binding.cpp: reference_internal
            // makes the Stepper the array's owner (see the header comment
            // there for the live-window contract).
            .def_prop_ro("qfrc_applied",
                         [](Stepper &s) {
                             auto const v = s.qfrc_applied();
                             return nb::ndarray<nb::numpy, const double, nb::shape<-1> >(
                                 v.data(), {v.size()}, nb::handle());
                         }, nb::rv_policy::reference_internal)
            .def_prop_ro("actuator_force", [](Stepper &s) {
                auto const v = s.actuator_force();
                return nb::ndarray<nb::numpy, const double, nb::shape<-1> >(
                    v.data(), {v.size()}, nb::handle());
            }, nb::rv_policy::reference_internal);
    cls.def(
        "_drive_core",
        [](Stepper &owner, const std::string &kind, const std::string &operation,
           double ratio) {
            // Private per-call oracle adapter. The kind set is closed —
            // the transient Transmission maps exactly onto the two ideal
            // models; anything else (including 'elastic_chain') is an
            // argument error, never a silent ideal fallback.
            if (kind != "ideal_mid_drive" && kind != "geometric_ideal_mid_drive")
                throw std::invalid_argument(
                    "_drive_core kind must be 'ideal_mid_drive' or "
                    "'geometric_ideal_mid_drive'");
            // reset/prepare/set_ratio mutate the shared model (wrap_prm,
            // tendon_range, mj_setConst): on a Stepper with an installed
            // drive writer the transient Transmission would desynchronize
            // the writer's caches. Bare Steppers remain legal.
            if (owner.has_drive())
                throw std::logic_error(
                    "_drive_core mutates the live model behind the installed "
                    "drive writer; use a bare Stepper (no drive config)");
            return owner.mutate([&] {
                const bool geometric = kind == "geometric_ideal_mid_drive";
                Transmission t(owner.model(), {.front_teeth = 34, .rear_teeth = 24, .chain_pitch_m = .0127}, geometric,
                               geometric
                                   ? "geometric_mid_drive_freehub"
                                   : "ideal_mid_drive_freehub");
                if (operation == "reset")
                    t.reset(owner.data());
                else if (operation == "prepare")
                    t.prepare(owner.data());
                else if (operation == "ratio") {
                    t.reset(owner.data());
                    t.set_ratio(owner.data(), ratio);
                } else
                    throw std::invalid_argument("core operation");
                return transmission_dict(t.state());
            });
        },
        nb::arg("kind"), nb::arg("operation"), nb::arg("ratio") = 34. / 24.);
}

NATIVE_DIAG_POP
