// Held-GIL validation/boxing. Only advance_control releases the GIL; its C++
// scheduler has no Python callback, NumPy import or RNG operation.
#include "research_binding.hpp"
#include "research.hpp"
#include "samples_binding.hpp"
#include "../binding_readers.hpp"
#include "../rider/intent_wire.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <nanobind/stl/string.h>

namespace runtime {
namespace {
namespace nb = nanobind;
using namespace sample_wire;
// NOLINTNEXTLINE(misc-no-recursion) bounded-depth recursion mirrors plain() tree conversion
Wire read_plain(nb::handle value, const std::string &path, unsigned depth = 0) {
    if (depth > 64) wire::invalid(path, "plain-data nesting exceeds 64");
    if (value.is_none()) return nullptr;
    if (nb::isinstance<nb::bool_>(value)) return wire::boolean(value, path);
    if (nb::isinstance<nb::str>(value)) return wire::string(value, path);
    if (nb::isinstance<nb::int_>(value)) return wire::integer(value, path);
    if (nb::isinstance<nb::float_>(value)) return wire::finite_real(value, path);
    if (nb::isinstance<nb::dict>(value)) {
        WireObject result;
        const auto d = wire::mapping(value, path);
        result.reserve(d.size());
        for (const auto item : d) {
            const auto key = wire::string(item.first, path);
            std::string child = path;
            child += '.';
            child += key;
            result.emplace_back(key, read_plain(item.second, child, depth + 1));
        }
        return result;
    }
    if (nb::isinstance<nb::list>(value) || nb::isinstance<nb::tuple>(value)) {
        WireArray result;
        const auto items = wire::sequence(value, path);
        result.reserve(items.size());
        for (std::size_t i = 0; i < items.size(); ++i) {
            std::string child = path;
            child += '[';
            child += std::to_string(i);
            child += ']';
            result.push_back(read_plain(items[i], child, depth + 1));
        }
        return result;
    }
    wire::invalid(path, "expected plain finite scalar/dict/list/tuple");
}
SensorConfig parse_sensors(nb::handle value) {
    const wire::Dict d{.value = wire::mapping(value, "sensors"), .path = "sensors"};
    wire::exact(d, wire::keys("latency_s", "acceleration_std_mps2", "gyro_std_rad_s", "encoder_std_rad_s",
        "torque_std_nm", "imu_enabled", "sample_period_s", "acceleration_bias_mps2", "gyro_bias_rad_s",
        "dropout_probability", "maximum_age_s"));
    SensorConfig c;
    c.latency_s = wire::finite_real(d["latency_s"], d.child("latency_s"));
    c.acceleration_bias = wire::fixed<3>(d["acceleration_bias_mps2"], d.child("acceleration_bias_mps2"));
    c.gyro_bias = wire::finite_real(d["gyro_bias_rad_s"], d.child("gyro_bias_rad_s"));
    c.dropout_probability = wire::finite_real(d["dropout_probability"], d.child("dropout_probability"));
    c.maximum_age_s = wire::finite_real(d["maximum_age_s"], d.child("maximum_age_s"));
    c.imu_enabled = wire::boolean(d["imu_enabled"], d.child("imu_enabled"));
    for (const char *key : {"acceleration_std_mps2", "gyro_std_rad_s", "encoder_std_rad_s", "torque_std_nm"})
        if (wire::finite_real(d[key], d.child(key)) < 0.) wire::invalid(d.child(key), "must be nonnegative");
    if (wire::finite_real(d["sample_period_s"], d.child("sample_period_s")) <= 0.)
        wire::invalid(d.child("sample_period_s"), "must be positive");
    c.validate();
    return c;
}
NoiseTape parse_tape(nb::handle noise, nb::handle dropout) {
    wire::numeric_array_type(noise, "noise_tape.noise");
    wire::numeric_array_type(dropout, "noise_tape.dropout_uniform");
    // Shape checks also cover empty arrays, for which row iteration alone
    // cannot distinguish (0, 9) from an invalid (0, 8) or (0,).
    const auto np = nb::module_::import_("numpy");
    const nb::object array_noise = np.attr("asarray")(noise);
    const nb::object array_drop = np.attr("asarray")(dropout);
    if (wire::integer(array_noise.attr("ndim"), "noise.ndim") != 2 ||
        wire::integer(array_drop.attr("ndim"), "dropout.ndim") != 1)
        wire::invalid("noise_tape", "requires (count, 9) and (count,) arrays");
    const auto shape = wire::sequence(array_noise.attr("shape"), "noise.shape");
    if (wire::integer(shape[1], "noise.shape[1]") != 9)
        wire::invalid("noise_tape.noise", "must have nine columns");
    NoiseTape result;
    const auto rows = wire::sequence(noise, "noise_tape.noise");
    result.noise.reserve(rows.size());
    for (const nb::handle row : rows) {
        // wire::vector keeps the same sequence/finite checks as wire::fixed
        // without instantiating a 72-byte pass-by-value return for N=9.
        const auto values = wire::vector(row, "noise_tape.noise");
        if (values.size() != 9) wire::invalid("noise_tape.noise", "incorrect sequence width");
        std::array<double, 9> parsed{};
        std::copy_n(values.begin(), parsed.size(), parsed.begin());
        result.noise.push_back(parsed);
    }
    result.dropout_uniform = wire::vector(dropout, "noise_tape.dropout_uniform");
    result.validate();
    return result;
}
SensorState parse_sensor_state(nb::handle value) {
    const wire::Dict d{.value = wire::mapping(value, "sensor_state"), .path = "sensor_state"};
    wire::exact(d, wire::keys("queue", "last_time", "delivery_time", "startup", "cursor",
                              "samples_attempted", "samples_dropped"));
    SensorState s;
    s.last_time = wire::optional_real(d["last_time"], d.child("last_time"));
    s.delivery_time = wire::optional_real(d["delivery_time"], d.child("delivery_time"));
    const auto count = [&d](const char *key) {
        const auto v = wire::integer(d[key], d.child(key));
        if (v < 0 || std::cmp_greater(v, std::numeric_limits<std::size_t>::max()))
            wire::invalid(d.child(key), "must be a representable nonnegative count");
        return static_cast<std::size_t>(v);
    };
    s.cursor = count("cursor");
    s.samples_attempted = count("samples_attempted");
    s.samples_dropped = count("samples_dropped");
    if (!d["startup"].is_none())
        s.startup = SensorObservation::from_wire(object(read_plain(d["startup"], d.child("startup"))));
    for (const nb::handle item : wire::sequence(d["queue"], d.child("queue")))
        s.queue.push_back(SensorObservation::from_wire(object(read_plain(item, d.child("queue")))));
    return s;
}
ResearchConfig parse_config(nb::handle value) {
    const wire::Dict d{.value = wire::mapping(value, "research"), .path = "research"};
    wire::exact(d, wire::keys("timestep_s", "control_steps", "delay_steps", "max_steps", "sensor_steps",
        "record_decimation", "stop_on_model_violation", "maximum_energy_residual_ratio", "wheelie_persistence_s",
        "track_length_m", "start_position_m", "sensors", "geometry", "startup_control", "rider_program", "demand_program"));
    ResearchConfig c;
    c.timestep_s = wire::finite_real(d["timestep_s"], d.child("timestep_s"));
    c.control_steps = wire::integer(d["control_steps"], d.child("control_steps"));
    c.delay_steps = wire::integer(d["delay_steps"], d.child("delay_steps"));
    c.max_steps = wire::integer(d["max_steps"], d.child("max_steps"));
    c.sensor_steps = wire::integer(d["sensor_steps"], d.child("sensor_steps"));
    c.record_decimation = wire::integer32(d["record_decimation"], d.child("record_decimation"));
    c.stop_on_model_violation = wire::boolean(d["stop_on_model_violation"], d.child("stop_on_model_violation"));
    c.maximum_energy_residual_ratio = wire::finite_real(d["maximum_energy_residual_ratio"], d.child("maximum_energy_residual_ratio"));
    c.wheelie_persistence_s = wire::finite_real(d["wheelie_persistence_s"], d.child("wheelie_persistence_s"));
    c.track_length_m = wire::finite_real(d["track_length_m"], d.child("track_length_m"));
    c.start_position_m = wire::finite_real(d["start_position_m"], d.child("start_position_m"));
    c.sensors = parse_sensors(d["sensors"]);
    const auto sensors = wire::mapping(d["sensors"], d.child("sensors"));
    const double sample_period = wire::finite_real(sensors["sample_period_s"], "sensors.sample_period_s");
    if (c.timestep_s <= 0. || std::abs(sample_period / c.timestep_s - static_cast<double>(c.sensor_steps)) > 1e-8)
        wire::invalid("research.sensor_steps", "must match sensors.sample_period_s / timestep_s");
    c.startup_control = rider::parse_intent_control(d["startup_control"], d.child("startup_control"));
    c.programs = ResearchPrograms::from_wire(read_plain(d["rider_program"], d.child("rider_program")),
                                            read_plain(d["demand_program"], d.child("demand_program")));
    const auto g = wire::section(d, "geometry");
    wire::exact(g, wire::keys("vertices", "front_radius", "rear_radius", "root_x_qpos", "root_x_dof",
                              "root_pitch_qpos", "root_pitch_dof"));
    for (const nb::handle row : wire::sequence(g["vertices"], g.child("vertices"))) {
        const auto v = wire::fixed<2>(row, g.child("vertices"));
        c.geometry.x.push_back(v[0]); c.geometry.z.push_back(v[1]);
    }
    c.geometry.front_radius = wire::finite_real(g["front_radius"], g.child("front_radius"));
    c.geometry.rear_radius = wire::finite_real(g["rear_radius"], g.child("rear_radius"));
    const auto address = [&g](const char *key) {
        const auto v = wire::integer(g[key], g.child(key));
        if (v < 0 || std::cmp_greater(v, std::numeric_limits<std::size_t>::max()))
            wire::invalid(g.child(key), "invalid model address");
        return static_cast<std::size_t>(v);
    };
    c.geometry.root_x_qpos = address("root_x_qpos");
    c.geometry.root_x_dof = address("root_x_dof");
    c.geometry.root_pitch_qpos = address("root_pitch_qpos");
    c.geometry.root_pitch_dof = address("root_pitch_dof");
    return c;
}
// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional handles mirror the fixed Python API
void bind_sensor_pipeline(const nb::module_ &module) {
    nb::class_<SensorPipeline>(module, "NativeSensorPipeline")
        .def("__init__", [](SensorPipeline *self, nb::handle config, nb::handle noise, nb::handle dropout) {
            const auto c = parse_sensors(config);
            auto tape = parse_tape(noise, dropout);
            new (self) SensorPipeline(c, std::move(tape));
        }, nb::arg("config"), nb::arg("noise"), nb::arg("dropout_uniform"))
        .def("reset", [](SensorPipeline &self, nb::handle raw) {
            self.reset(SensorObservation::from_wire(object(read_plain(raw, "initial"))));
        }, nb::arg("initial"))
        .def("import_state", [](SensorPipeline &self, nb::handle state) { self.import_state(parse_sensor_state(state)); })
        .def("push", [](SensorPipeline &self, nb::handle raw) {
            self.push(SensorObservation::from_wire(object(read_plain(raw, "raw"))));
        }, nb::arg("raw"))
        .def("read", [](SensorPipeline &self, nb::handle time) {
            // read() only mutates the delivery cursor after validation. Box a
            // copy first to retain the original clock on a boxing failure.
            SensorPipeline staged = self;
            auto result = wire_object_to_python(staged.read(wire::finite_real(time, "time_s")).as_wire());
            self = std::move(staged);
            return result;
        }, nb::arg("time_s"))
        .def("state_dict", [](const SensorPipeline &self) { return wire_object_to_python(self.state_wire()); });
}
// NOLINTEND(bugprone-easily-swappable-parameters)
} // namespace
void bind_research(nb::module_ &module) {
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional handles mirror the fixed Python API
    bind_sensor_pipeline(module);
    module.def("research_program_at", [](nb::handle rider_program, nb::handle demand_program,
                                           nb::handle control, nb::handle time) {
        const double time_s = wire::finite_real(time, "time_s");
        const auto programs = ResearchPrograms::from_wire(read_plain(rider_program, "rider_program"),
                                                          read_plain(demand_program, "demand_program"));
        const auto command = programs.apply(rider::parse_intent_control(control, "control"), time_s);
        const auto demand = programs.demand_at(time_s);
        return wire_object_to_python({{"control", research_control_wire(command)},
                                      {"demand_nm", demand ? Wire(*demand) : Wire(nullptr)}});
    }, nb::arg("rider_program").none(), nb::arg("demand_program").none(),
       nb::arg("control"), nb::arg("time_s"));
    nb::class_<NativeResearchRuntime>(module, "NativeResearchRuntime")
        .def("__init__", [](NativeResearchRuntime *self, nb::handle model, nb::handle physical_config,
                            nb::handle physical_state, nb::handle research_config, nb::handle noise,
                            nb::handle dropout, nb::handle sensor_state) {
            auto config = parse_config(research_config);
            auto tape = parse_tape(noise, dropout);
            auto startup = parse_sensor_state(sensor_state);
            auto ride = std::make_unique<NativeRideRuntime>(model, physical_config, physical_state);
            new (self) NativeResearchRuntime(std::move(ride), std::move(config), std::move(tape), std::move(startup));
        }, nb::arg("model_path"), nb::arg("config"), nb::arg("state"), nb::arg("research"),
           nb::arg("noise"), nb::arg("dropout_uniform"), nb::arg("sensor_state"))
        .def("begin_control", [](NativeResearchRuntime &self, nb::handle control, nb::handle front, nb::handle rear) {
            const auto command = rider::parse_intent_control(control, "control");
            self.begin_control(command, wire::finite_real(front, "front_brake_demand"),
                               wire::finite_real(rear, "rear_brake_demand"));
        }, nb::arg("control"), nb::arg("front_brake_demand") = 0., nb::arg("rear_brake_demand") = 0.)
        .def("advance_control", [](NativeResearchRuntime &self, nb::handle wall_budget,
                                  nb::handle target) -> nb::object {
            const auto budget = wire::optional_real(wall_budget, "wall_budget_s");
            const auto target_step = target.is_none() ? std::optional<std::int64_t>{} :
                std::optional<std::int64_t>{wire::integer(target, "target_step")};
            std::optional<ResearchTransition> result;
            {
                const nb::gil_scoped_release release;
                result = self.advance_control(budget, target_step);
            }
            self.emit_warning();
            return result ? nb::object(wire_object_to_python(result->as_wire())) : nb::none();
        }, nb::arg("wall_budget_s") = nb::none(), nb::arg("target_step") = nb::none())
        .def("acknowledge_control", &NativeResearchRuntime::acknowledge_control)
        .def("snapshot", &NativeResearchRuntime::snapshot)
        .def("status", [](const NativeResearchRuntime &self) { return wire_object_to_python(self.status()); })
        .def("commands_requested", [](const NativeResearchRuntime &self) { return wire_to_python(self.commands_requested()); })
        .def("commands_applied", [](const NativeResearchRuntime &self) { return wire_to_python(self.commands_applied()); })
        .def("observations", [](const NativeResearchRuntime &self) { return wire_to_python(self.observations()); })
        .def("transitions", [](const NativeResearchRuntime &self) { return wire_to_python(self.transitions()); })
        .def("recorded_columns", [](const NativeResearchRuntime &self) { return columns_to_python(self.recorded_columns()); })
        .def("recorded_intervals", [](const NativeResearchRuntime &self) { return wire_to_python(self.recorded_intervals()); })
        .def("reset", [](NativeResearchRuntime &self, nb::handle noise, nb::handle dropout, nb::handle startup) {
            auto tape = parse_tape(noise, dropout);
            auto state = parse_sensor_state(startup);
            self.reset(std::move(tape), std::move(state));
        }, nb::arg("noise"), nb::arg("dropout_uniform"), nb::arg("sensor_state"))
        .def("_test_fail_at_step", &NativeResearchRuntime::test_fail_at_step, nb::arg("step"))
        .def("stop", &NativeResearchRuntime::stop)
        .def("fail_policy", &NativeResearchRuntime::fail_policy)
        .def("set_error_text", &NativeResearchRuntime::set_error_text)
        .def("close", &NativeResearchRuntime::close)
        .def_prop_ro("closed", &NativeResearchRuntime::closed);
    // NOLINTEND(bugprone-easily-swappable-parameters)
}
} // namespace runtime
