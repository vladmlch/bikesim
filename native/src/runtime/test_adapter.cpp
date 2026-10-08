// runtime/test_adapter.cpp — NativeTestAdapter binding (plan A2).
//
// Constructor contract: NativeTestAdapter(mjb_path, probes) where
// `probes` is a dict of per-subsystem sections. `static_brake` carries
// {front_dof, rear_dof, ceiling_nm}; `spindle_controller` carries the
// production wire shape ({config, pose, crank_length_m}) and builds the
// SpindleController the A2.3 parity tests compare against the Python
// oracle. Unknown sections are rejected — the adapter never guesses
// which subsystem a test wanted.
#include "test_adapter.hpp"

#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include "../binding_arrays.hpp"
#include "../diag.hpp"
#include "../binding_readers.hpp"
#include "../model_access.hpp"
#include "../rider/intent_wire.hpp"
#include "../rider/spindle_controller.hpp"
#include "../rider/spindle_wire.hpp"
#include "../stepper.hpp"
#include "static_brake.hpp"

namespace nb = nanobind;

namespace runtime {
namespace {

class NativeTestAdapter {
public:
    NativeTestAdapter(const std::string &mjb_path, nb::handle config)
        : stepper_(mjb_path, nb::dict()) {
        const nb::dict probes = config.is_none()
                                    ? nb::dict()
                                    : wire::mapping(config, "adapter");
        wire::exact_keys(probes, {},
                         wire::keys("static_brake", "spindle_controller",
                                    "rider_intent"),
                         "adapter");
        if (probes.contains("static_brake")) {
            const nb::dict section = wire::mapping(
                probes["static_brake"], "adapter.static_brake");
            wire::exact_keys(section,
                             wire::keys("front_dof", "rear_dof", "ceiling_nm"),
                             {}, "adapter.static_brake");
            brake_.emplace(
                wire::integer32(section["front_dof"],
                                "adapter.static_brake.front_dof"),
                wire::integer32(section["rear_dof"],
                                "adapter.static_brake.rear_dof"),
                wire::finite_real(section["ceiling_nm"],
                                  "adapter.static_brake.ceiling_nm"));
        }
        if (probes.contains("spindle_controller")) {
            const nb::dict section = wire::mapping(
                probes["spindle_controller"],
                "adapter.spindle_controller");
            wire::exact_keys(
                section, wire::keys("config", "pose", "crank_length_m"),
                {}, "adapter.spindle_controller");
            const auto spindle_path =
                std::string("adapter.spindle_controller");
            spindle_.emplace(
                stepper_.model(),
                rider::parse_spindle_pose(section["pose"],
                                          spindle_path + ".pose"),
                rider::parse_spindle_config(section["config"],
                                            spindle_path + ".config"),
                wire::finite_real(section["crank_length_m"],
                                  spindle_path + ".crank_length_m"));
        }
        if (probes.contains("rider_intent")) {
            const nb::dict section = wire::mapping(
                probes["rider_intent"], "adapter.rider_intent");
            wire::exact_keys(section, wire::keys("config", "dt_s"), {},
                             "adapter.rider_intent");
            intent_.emplace(
                rider::parse_intent_config(
                    section["config"], "adapter.rider_intent.config"),
                wire::real(section["dt_s"], "adapter.rider_intent.dt_s"));
        }
    }

    [[nodiscard]] rider::SpindleController &require_spindle() {
        if (!spindle_)
            throw std::logic_error(
                "adapter was built without a spindle_controller section");
        return *spindle_;
    }

    [[nodiscard]] rider::RiderIntent &require_intent() {
        if (!intent_)
            throw std::logic_error(
                "adapter was built without a rider_intent section");
        return *intent_;
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    nb::dict intent_resolve(nb::handle control, nb::handle signals,
                            nb::handle step, nb::handle road_grade,
                            nb::handle preview_grade,
                            nb::handle lean_limit_rad, bool active,
                            bool advance) {
        auto &intent = require_intent();
        // resolve() args stay lazily validated: road_grade/preview/lean are
        // only checked by policy.update when the clock ticks — the oracle
        // accepts NaN on non-tick steps, so the wire decodes raw reals.
        const auto parsed_control =
            rider::parse_intent_control(control, "resolve.control");
        const auto parsed_signals =
            rider::parse_intent_signals(signals, "resolve.signals");
        const double grade = wire::real(road_grade, "resolve.road_grade");
        const auto preview = preview_grade.is_none()
                                 ? std::optional<double>()
                                 : std::optional(wire::real(
                                       preview_grade, "resolve.preview_grade"));
        const auto lean =
            lean_limit_rad.is_none()
                ? std::optional<double>()
                : std::optional(
                    wire::real(lean_limit_rad, "resolve.lean_limit_rad"));
        // rider_intent.py resolve() checks enabled/active before
        // `type(step) is int` — a disabled or inactive call returns the
        // control untouched even for a bool or numpy step, so the strict
        // integer decode follows that same ordering here.
        const std::int64_t parsed_step =
            (intent.config().enabled && active)
                ? rider::parse_intent_step(step, "resolve.step")
                : 0;
        return rider::intent_control_dict(
            intent.resolve(parsed_control, parsed_signals, parsed_step, grade,
                           preview, lean, active, advance));
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    nb::dict intent_update(nb::handle signals, nb::handle dt_s,
                           nb::handle road_grade, nb::handle preview_grade,
                           nb::handle lean_limit_rad) {
        auto &intent = require_intent();
        const auto parsed_signals =
            rider::parse_intent_signals(signals, "update.signals");
        const double dt = wire::real(dt_s, "update.dt_s");
        const double grade = wire::real(road_grade, "update.road_grade");
        const auto preview = preview_grade.is_none()
                                 ? std::optional<double>()
                                 : std::optional(wire::real(
                                       preview_grade, "update.preview_grade"));
        const auto lean =
            lean_limit_rad.is_none()
                ? std::optional<double>()
                : std::optional(
                    wire::real(lean_limit_rad, "update.lean_limit_rad"));
        return rider::seated_intent_dict(
            intent.policy().update(parsed_signals, dt, grade, preview, lean));
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    double intent_program_update(nb::handle time_s, nb::handle road_grade,
                                 nb::handle dt_s, nb::handle front_load_share,
                                 nb::handle lean_limit_rad,
                                 nb::handle dead_time_s) {
        auto &intent = require_intent();
        const auto optional_real = [](nb::handle value, const char *path) {
            return value.is_none() ? std::optional<double>()
                                   : std::optional(wire::real(value, path));
        };
        return intent.policy().program().update(
            wire::real(time_s, "program.time_s"),
            wire::real(road_grade, "program.road_grade"),
            wire::real(dt_s, "program.dt_s"),
            optional_real(front_load_share, "program.front_load_share"),
            optional_real(lean_limit_rad, "program.lean_limit_rad"),
            optional_real(dead_time_s, "program.dead_time_s"));
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    double intent_power_target(nb::handle preview_grade, nb::handle dt_s) {
        return require_intent().policy().power_target_w(
            wire::real(preview_grade, "power_target.preview_grade"),
            wire::real(dt_s, "power_target.dt_s"));
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    void intent_schedule_pulse(double start_s, double duration_s,
                               double amplitude_rad) {
        require_intent().schedule_pulse(start_s, duration_s, amplitude_rad);
    }

    void intent_reset() { require_intent().reset(); }

    nb::dict intent_state() {
        return rider::intent_state_dict(require_intent().state());
    }

    void intent_restore(nb::handle state) {
        require_intent().restore(
            rider::parse_intent_state(state, "state"));
    }

    nb::dict spindle_compute(nb::handle command, bool advance,
                             nb::handle dt_s, bool steady_state) {
        auto &controller = require_spindle();
        const auto parsed = rider::parse_rider_command(command, "command");
        std::optional<double> dt;
        if (!dt_s.is_none())
            dt = wire::finite_real(dt_s, "compute.dt_s");
        const auto torques = controller.compute(
            stepper_.data(), parsed, advance, dt, steady_state);
        nb::dict out;
        nb::dict torque_dict;
        for (const auto &[name, value] : torques)
            torque_dict[name.c_str()] = value;
        out["torques"] = torque_dict;
        out["state"] = rider::spindle_state_dict(controller.state());
        return out;
    }

    void spindle_initialize() {
        stepper_.mutate(
            [&] { require_spindle().initialize(stepper_.data()); });
    }

    void spindle_initialize_velocity() {
        stepper_.mutate([&] {
            require_spindle().initialize_velocity(stepper_.data());
        });
    }

    void spindle_reset_activation() {
        require_spindle().reset_activation();
    }

    nb::dict spindle_envelope_forces() {
        const auto [force, stored] =
            require_spindle().envelope_forces(stepper_.data());
        nb::dict out;
        out["force"] = wire::owned_array<double>(
            std::span<const double>(force));
        out["stored_j"] = stored;
        return out;
    }

    void spindle_write(nb::handle torques) {
        const auto parsed = wire::mapping(torques, "spindle_write.torques");
        rider::NamedEntries<double> values;
        for (const auto item : parsed)
            values.emplace_back(
                wire::string(item.first, "spindle_write.torques"),
                wire::finite_real(item.second, "spindle_write.torques"));
        stepper_.mutate(
            [&] { require_spindle().write(stepper_.data(), values); });
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional wire signature
    nb::dict spindle_solved_effort(nb::handle qpos, nb::handle qvel,
                                   double dt_s) {
        const auto incoming_qpos = wire::vector(qpos, "solved_effort.qpos");
        const auto incoming_qvel = wire::vector(qvel, "solved_effort.qvel");
        if (incoming_qpos.size() !=
                static_cast<std::size_t>(stepper_.model()->nq) ||
            incoming_qvel.size() !=
                static_cast<std::size_t>(stepper_.model()->nv))
            throw std::invalid_argument(
                "solved_effort incoming state width mismatch");
        const auto diagnostics = require_spindle().solved_effort(
            stepper_.data(), incoming_qpos, incoming_qvel, dt_s);
        return rider::effort_diagnostics_dict(diagnostics);
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional wire signature
    nb::dict spindle_strength_limited(nb::handle torques, nb::handle qpos,
                                      nb::handle qvel) {
        const auto torque_v = wire::vector(torques, "strength.torques");
        const auto qpos_v = wire::vector(qpos, "strength.qpos");
        const auto qvel_v = wire::vector(qvel, "strength.qvel");
        const auto [clipped, limited] =
            require_spindle().strength_limited(torque_v, qpos_v, qvel_v);
        nb::dict out;
        out["torques"] =
            wire::owned_array<double>(std::span<const double>(clipped));
        nb::list names;
        for (const auto &name : limited) names.append(name);
        out["limited"] = names;
        return out;
    }

    double spindle_capacity(nb::handle name, double angle, double velocity,
                            double torque) {
        return require_spindle().strength_capacity(
            wire::string(name, "capacity.name"), angle, velocity, torque);
    }

    double spindle_anatomical(nb::handle name, double qpos_value) {
        return require_spindle().anatomical_joint_angle(
            wire::string(name, "anatomical.name"), qpos_value);
    }

    nb::dict spindle_state() {
        return rider::spindle_state_dict(require_spindle().state());
    }

    void spindle_restore(nb::handle state) {
        require_spindle().restore(
            rider::parse_spindle_state(state, "state"));
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional wire signature
    void set_state(nb::handle qpos, nb::handle qvel, nb::handle act,
                   nb::handle warmstart, double time) {
        const auto qpos_v = wire::vector(qpos, "set_state.qpos");
        const auto qvel_v = wire::vector(qvel, "set_state.qvel");
        const auto act_v = wire::vector(act, "set_state.act");
        const auto warmstart_v =
            wire::vector(warmstart, "set_state.warmstart");
        stepper_.mutate(
            [&] { stepper_.set_state(qpos_v, qvel_v, act_v, warmstart_v,
                                     time); });
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    data_view(const mjtNum *base, mjtSize width, const char *field) const {
        const auto row =
            model_access::readonly_buffer(base, width, field);
        return wire::owned_array<double>(
            std::vector<double>(row.begin(), row.end()));
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    qpos() const {
        return data_view(stepper_.data()->qpos, stepper_.model()->nq,
                         "qpos");
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    qvel() const {
        return data_view(stepper_.data()->qvel, stepper_.model()->nv,
                         "qvel");
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    qacc() const {
        return data_view(stepper_.data()->qacc, stepper_.model()->nv,
                         "qacc");
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    qfrc_passive() const {
        return data_view(stepper_.data()->qfrc_passive,
                         stepper_.model()->nv, "qfrc_passive");
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    actuator_force() const {
        return data_view(stepper_.data()->actuator_force,
                         stepper_.model()->nu, "actuator_force");
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    ctrl() const {
        return data_view(stepper_.data()->ctrl, stepper_.model()->nu,
                         "ctrl");
    }

    void apply_static_brake(nb::handle front, nb::handle rear) {
        if (!brake_)
            throw std::logic_error(
                "adapter was built without a static_brake section");
        stepper_.mutate([&] {
            brake_->apply(stepper_.model(),
                          wire::finite_real(front,
                                            "apply_static_brake.front_demand"),
                          wire::finite_real(rear,
                                            "apply_static_brake.rear_demand"));
        });
    }

    [[nodiscard]] nb::dict static_brake_solved() {
        if (!brake_)
            throw std::logic_error(
                "adapter was built without a static_brake section");
        const auto [front, rear] =
            brake_->solved_components(stepper_.model(), stepper_.data());
        nb::dict out;
        out["front_static_brake"] = wire::owned_array<double>(front);
        out["rear_static_brake"] = wire::owned_array<double>(rear);
        return out;
    }

    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    dof_frictionloss() const {
        const mjModel *model = stepper_.model();
        const auto row = model_access::readonly_buffer(
            model->dof_frictionloss, model->nv, "dof_frictionloss");
        return wire::owned_array<double>(
            std::vector<double>(row.begin(), row.end()));
    }

    void set_inputs(nb::handle ctrl, nb::handle force) {
        stepper_.set_inputs(wire::vector(ctrl, "set_inputs.ctrl"),
                            wire::vector(force, "set_inputs.qfrc_applied"));
    }

    void forward() { stepper_.forward(); }

private:
    Stepper stepper_;
    std::optional<StaticBrake> brake_;
    std::optional<rider::SpindleController> spindle_;
    std::optional<rider::RiderIntent> intent_;
};

} // namespace

// Import-time binding glue; the frame is nanobind temporaries, so the
// 8KB guard keeps covering the per-step code elsewhere.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void bind_test_adapter(const nb::module_ &module) {
    nb::class_<NativeTestAdapter>(module, "NativeTestAdapter")
        .def(nb::init<const std::string &, nb::handle>(),
             nb::arg("mjb_path"), nb::arg("probes").none())
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed front/rear Python signature
        .def("apply_static_brake", &NativeTestAdapter::apply_static_brake,
             nb::arg("front_demand").none(), nb::arg("rear_demand").none())
        .def("static_brake_solved", &NativeTestAdapter::static_brake_solved)
        .def_prop_ro("dof_frictionloss",
                     &NativeTestAdapter::dof_frictionloss,
                     nb::rv_policy::move)
        .def("set_inputs", &NativeTestAdapter::set_inputs,
             nb::arg("ctrl").none(), nb::arg("qfrc_applied").none())
        .def("spindle_compute", &NativeTestAdapter::spindle_compute,
             nb::arg("command"), nb::arg("advance"),
             nb::arg("dt_s").none(), nb::arg("steady_state"))
        .def("spindle_initialize", &NativeTestAdapter::spindle_initialize)
        .def("spindle_initialize_velocity",
             &NativeTestAdapter::spindle_initialize_velocity)
        .def("spindle_reset_activation",
             &NativeTestAdapter::spindle_reset_activation)
        .def("spindle_envelope_forces",
             &NativeTestAdapter::spindle_envelope_forces)
        .def("spindle_write", &NativeTestAdapter::spindle_write,
             nb::arg("torques"))
        .def("spindle_solved_effort",
             &NativeTestAdapter::spindle_solved_effort, nb::arg("qpos"),
             nb::arg("qvel"), nb::arg("dt_s"))
        .def("spindle_strength_limited",
             &NativeTestAdapter::spindle_strength_limited,
             nb::arg("torques"), nb::arg("qpos"), nb::arg("qvel"))
        .def("spindle_capacity", &NativeTestAdapter::spindle_capacity,
             nb::arg("name"), nb::arg("angle"), nb::arg("velocity"),
             nb::arg("torque"))
        .def("spindle_anatomical", &NativeTestAdapter::spindle_anatomical,
             nb::arg("name"), nb::arg("qpos_value"))
        .def("spindle_state", &NativeTestAdapter::spindle_state)
        .def("spindle_restore", &NativeTestAdapter::spindle_restore,
             nb::arg("state"))
        .def("intent_resolve", &NativeTestAdapter::intent_resolve,
             nb::arg("control"), nb::arg("signals"), nb::arg("step"),
             nb::arg("road_grade"), nb::arg("preview_grade").none(),
             nb::arg("lean_limit_rad").none(), nb::arg("active"),
             nb::arg("advance"))
        .def("intent_update", &NativeTestAdapter::intent_update,
             nb::arg("signals"), nb::arg("dt_s"), nb::arg("road_grade"),
             nb::arg("preview_grade").none(),
             nb::arg("lean_limit_rad").none())
        .def("intent_program_update",
             &NativeTestAdapter::intent_program_update, nb::arg("time_s"),
             nb::arg("road_grade"), nb::arg("dt_s"),
             nb::arg("front_load_share").none(),
             nb::arg("lean_limit_rad").none(),
             nb::arg("dead_time_s").none())
        .def("intent_power_target", &NativeTestAdapter::intent_power_target,
             nb::arg("preview_grade"), nb::arg("dt_s"))
        .def("intent_schedule_pulse",
             &NativeTestAdapter::intent_schedule_pulse, nb::arg("start_s"),
             nb::arg("duration_s"), nb::arg("amplitude_rad"))
        .def("intent_reset", &NativeTestAdapter::intent_reset)
        .def("intent_state", &NativeTestAdapter::intent_state)
        .def("intent_restore", &NativeTestAdapter::intent_restore,
             nb::arg("state"))
        .def("set_state", &NativeTestAdapter::set_state, nb::arg("qpos"),
             nb::arg("qvel"), nb::arg("act"), nb::arg("warmstart"),
             nb::arg("time"))
        .def_prop_ro("qpos", &NativeTestAdapter::qpos,
                     nb::rv_policy::move)
        .def_prop_ro("qvel", &NativeTestAdapter::qvel,
                     nb::rv_policy::move)
        .def_prop_ro("qacc", &NativeTestAdapter::qacc,
                     nb::rv_policy::move)
        .def_prop_ro("qfrc_passive", &NativeTestAdapter::qfrc_passive,
                     nb::rv_policy::move)
        .def_prop_ro("actuator_force",
                     &NativeTestAdapter::actuator_force,
                     nb::rv_policy::move)
        .def_prop_ro("ctrl", &NativeTestAdapter::ctrl,
                     nb::rv_policy::move)
        .def("forward", &NativeTestAdapter::forward);
}
NATIVE_DIAG_POP

} // namespace runtime
