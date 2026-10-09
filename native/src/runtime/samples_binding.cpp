#include "samples_binding.hpp"
#include "accounting.hpp"
#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"
#include "../diag.hpp"
#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <stdexcept>
#include <vector>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
namespace runtime {

nb::dict wire_object_to_python(const WireObject &value) {
    nb::dict result;
    for (const auto &[key, item] : value)
        result[nb::str(key.c_str(), key.size())] = wire_to_python(item);
    return result;
}

// NOLINTNEXTLINE(misc-no-recursion) boxing follows a finite owned value tree
nb::object wire_to_python(const Wire &value) {
    return std::visit([](const auto &item) -> nb::object {
        using T = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<T, std::monostate>) return nb::none();
        else if constexpr (std::is_same_v<T, bool>) return nb::bool_(item);
        else if constexpr (std::is_same_v<T, std::int64_t>) return nb::int_(item);
        else if constexpr (std::is_same_v<T, double>) return nb::float_(item);
        else if constexpr (std::is_same_v<T, std::string>) return nb::str(item.c_str(), item.size());
        else if constexpr (std::is_same_v<T, WireArray>) {
            nb::list result;
            for (const auto &entry : item) result.append(wire_to_python(entry));
            return result;
        } else return wire_object_to_python(item);
    }, value.value);
}

nb::dict columns_to_python(const SampleColumns &columns) {
    nb::dict result;
    for (const auto &[name, values] : columns)
        result[nb::str(name.c_str(), name.size())] = wire::owned_array<double>(values);
    return result;
}

namespace {
nb::list batch_rows(const RuntimeSampleBatch &batch) {
    nb::list result;
    for (const auto &sample : batch.samples)
        result.append(wire_object_to_python(sample->as_wire()));
    return result;
}

void bind_sample_value(const nb::module_ &module) {
    nb::class_<RuntimeSample>(module, "RuntimeSample")
        .def_prop_ro("interval_id", [](const RuntimeSample &s) { return s.value->interval_id; })
        .def_prop_ro("time_s", [](const RuntimeSample &s) { return s.value->time_s; })
        .def_prop_ro("end_time_s", [](const RuntimeSample &s) { return s.value->end_time_s; })
        .def_prop_ro("qpos", [](const RuntimeSample &s) {
            return wire::owned_array<double>(s.value->qpos);
        }, nb::rv_policy::move)
        .def_prop_ro("qvel", [](const RuntimeSample &s) {
            return wire::owned_array<double>(s.value->qvel);
        }, nb::rv_policy::move)
        .def_prop_ro("forces", [](const RuntimeSample &s) {
            nb::dict result;
            for (const auto &[name, force] : s.value->forces)
                result[nb::str(name.c_str(), name.size())] = wire::owned_array<double>(force);
            return result;
        })
        .def_prop_ro("channels", [](const RuntimeSample &s) {
            return wire_object_to_python(s.value->channels);
        })
        .def("as_dict", [](const RuntimeSample &s) { return wire_object_to_python(s.value->as_wire()); });
}

void bind_batch_value(const nb::module_ &module) {
    nb::class_<RuntimeSampleBatch>(module, "RuntimeSampleBatch")
        .def_ro("generation", &RuntimeSampleBatch::generation)
        .def_prop_ro("size", [](const RuntimeSampleBatch &batch) { return batch.samples.size(); })
        .def_prop_ro("interval_ids", [](const RuntimeSampleBatch &batch) {
            std::vector<std::int64_t> values;
            values.reserve(batch.samples.size());
            for (const auto &sample : batch.samples) values.push_back(sample->interval_id);
            return wire::owned_array<std::int64_t>(values);
        }, nb::rv_policy::move)
        .def_prop_ro("columns", [](const RuntimeSampleBatch &batch) {
            return columns_to_python(batch.columns);
        })
        .def("as_dict_rows", &batch_rows);
}

PhysicalSampleData history_sample(nb::handle value) {
    const wire::Dict row{.value = wire::mapping(value, "history interval"),
                         .path = "history interval"};
    wire::exact(row, wire::keys("interval_id", "time_s", "end_time_s", "powers_w", "tires"));
    PhysicalSampleData sample;
    sample.interval_id = wire::integer(wire::field(row, "interval_id"), row.child("interval_id"));
    sample.time_s = wire::finite_real(wire::field(row, "time_s"), row.child("time_s"));
    sample.end_time_s = wire::finite_real(wire::field(row, "end_time_s"), row.child("end_time_s"));
    for (const auto entry : wire::mapping(wire::field(row, "powers_w"), row.child("powers_w")))
        sample.powers_w.emplace_back(wire::string(entry.first, "power name"),
                                      Wire(wire::finite_real(entry.second, "power")));
    const auto tires = wire::section(row, "tires");
    WireObject captured;
    for (const auto *side : {"front", "rear"}) {
        if (!tires.contains(side)) continue;
        const auto tire = wire::section(tires, side);
        WireObject evidence;
        if (tire.contains("patches")) {
            WireArray patches;
            for (const nb::handle item :
                 wire::sequence(tire["patches"], tire.child("patches"))) {
                const wire::Dict patch{.value = wire::mapping(item, "patch"),
                                       .path = "patch"};
                patches.emplace_back(WireObject{
                    {"normal_load_n", Wire(wire::finite_real(wire::field(patch, "normal_load_n"), "normal_load_n"))},
                    {"source_geom", Wire(wire::string(wire::field(patch, "source_geom"), "source_geom"))}});
            }
            evidence.emplace_back("patches", Wire(std::move(patches)));
        }
        captured.emplace_back(side, Wire(std::move(evidence)));
    }
    sample.channels.emplace_back("tires", Wire(std::move(captured)));
    return sample;
}

// Pure production kernels surfaced for independent, deliberately adversarial
// accounting tests. They neither step an engine nor synthesize oracle results.
void bind_work_probes(nb::module_ &module) {
    module.def("runtime_work_history", [](nb::handle intervals) {
        WorkHistory history;
        for (const nb::handle item : wire::sequence(intervals, "intervals"))
            history.add(history_sample(item));
        return wire_object_to_python(history.as_wire());
    }, nb::arg("intervals"));
    module.def("runtime_step_work", [](nb::handle muscle, double motor,
                                        nb::handle constraints, double dt_s) {
        const auto muscle_values = wire::vector(muscle, "muscle_power_w");
        const auto constraint_values = wire::vector(constraints, "constraint_power_w");
        return wire_object_to_python(step_work(muscle_values, motor, constraint_values, dt_s).as_wire());
    }, nb::arg("muscle_power_w"), nb::arg("motor_power_w"),
       nb::arg("constraint_power_w"), nb::arg("dt_s"));
    module.def("runtime_constraint_work_ok", &constraint_work_ok,
               nb::arg("absolute_j"), nb::arg("source_positive_j"), nb::arg("roundoff_j") = 1e-8);
    module.def("runtime_validate_intervals", [](nb::handle intervals, int capacity) {
        if (capacity < 1) throw std::invalid_argument("period capacity must be positive");
        PeriodBuffer buffer(static_cast<std::size_t>(capacity));
        for (const nb::handle item : wire::sequence(intervals, "intervals")) {
            const auto values = wire::sequence(item, "interval");
            if (nb::len(values) != 3)
                throw std::invalid_argument("interval must contain ID, start and end");
            RawStep raw;
            raw.interval_id = wire::integer(values[0], "interval.id");
            raw.time_s = wire::finite_real(values[1], "interval.time_s");
            raw.end_time_s = wire::finite_real(values[2], "interval.end_time_s");
            buffer.push(std::move(raw));
            if (buffer.full()) buffer.clear();
        }
    }, nb::arg("intervals"), nb::arg("capacity"));
}
} // namespace

void bind_samples(nb::module_ &module) {
    // Registration is RAII: the held type lives in the module, not the handle.
    const nb::exception<InvalidReferenceRun> invalid_reference_run(
        module, "InvalidReferenceRun", PyExc_RuntimeError);
    (void)invalid_reference_run;
    bind_sample_value(module);
    bind_batch_value(module);
    bind_work_probes(module);
}
} // namespace runtime
