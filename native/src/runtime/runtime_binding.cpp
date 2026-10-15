// runtime_binding.cpp — the nanobind adapter over the pure
// runtime::NativeRideRuntime (task T09).
//
// Everything Python-shaped lives here and only here: the handle
// materializer that lowers the capture_bootstrap config/state objects into
// owned bike::io::Value documents (the owned_copy twin), the model.mjb
// artifact read behind os.fspath resolution, control-dict decoding,
// held-GIL boxing and the RuntimeWarning emission. The pure core keeps
// validation order, diagnostics and accounting verbatim — the adapter only
// stages inputs and boxes outputs.
//
// Materialization shape notes (the Value tree the staged decoders consume):
//   dict            -> Object (keys must be str — same "expected string"
//                      gate the exact() passes ran)
//   list/tuple      -> Array; ndarray -> its .tolist() shape (the C-order
//                      nested list, Python scalars — the same element
//                      domain nb::list(ndarray) iteration surfaced)
//   str             -> String; bytes-likes -> Bytes
//   bool            -> Bool; Integral -> integer lexeme via PyNumber_Index
//                      + str() (arbitrary precision survives the boundary)
//   Real            -> shortest round-trip float lexeme; a nonfinite one
//                      becomes the recording Tagged{"float64"} sentinel
//   dataclass       -> its dataclasses.asdict() object (member access ==
//                      the readers' attribute reads on the instance)
//   other objects   -> rejected like owned_copy's "expected plain data"
#include "runtime_binding.hpp"
#include "samples_binding.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nanobind/stl/string.h>

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"
#include "../diag.hpp"
#include "../io/hash.hpp"
#include "../rider/intent_wire.hpp"
#include "../rider/spindle_math.hpp"

namespace nb = nanobind;

namespace runtime {
namespace {

namespace bio = bike::io;

// ---- handle -> bike::io::Value materialization --------------------------
// The acceptance set mirrors owned_copy + the wire:: readers exactly at
// the staging position: numbers keep their domains through lexeme spelling
// (integer tokens vs float spellings), and the two spellings Python could
// not carry through JSON — nonfinite floats and wide ints — survive as the
// recording sentinel and the full decimal lexeme respectively.

struct MaterializeTypes {
    nb::object integral;
    nb::object real;
    nb::object np_generic;
    nb::object np_ndarray;
    nb::object dc_is_dataclass;
    nb::object dc_asdict;
};

// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the document shape
[[nodiscard]] bio::Value value_from_python(nb::handle value,
                                           const MaterializeTypes &types,
                                           std::string_view path) {
    if (value.is_none()) return bio::Value{};
    // Strict bools only — wire::boolean rejected np.bool_ (it requires the
    // literal Py_True/Py_False), and wire::integer rejected the Integral
    // alias on bool; the same exclusions land before the numeric lanes.
    if (value.ptr() == Py_True) return bio::Value{true};
    if (value.ptr() == Py_False) return bio::Value{false};
    if (nb::isinstance<nb::str>(value))
        return bio::Value{wire::string(value, path)};
    // np.generic unboxes to its Python value first — np.bool_ is neither a
    // PyBool nor a numbers lane, mirroring samples_binding's authored
    // python_to_wire.
    if (nb::isinstance(value, types.np_generic))
        return value_from_python(value.attr("item")(), types, path);
    if (nb::isinstance(value, types.integral)) {
        // PyNumber_Index collapses IntEnum and friends to the int domain;
        // str() spells the full decimal — wide Python ints arrive whole
        // and fail vread::integer's range check at the same position the
        // PyLong_AsLongLong overflow used to.
        nb::object const indexed =
            nb::steal<nb::object>(PyNumber_Index(value.ptr()));
        if (!indexed.is_valid()) wire::conversion_error(path);
        return bio::Value{
            bio::Number{nb::cast<std::string>(nb::str(indexed))}};
    }
    if (nb::isinstance(value, types.real)) {
        const double result = PyFloat_AsDouble(value.ptr());
        if (PyErr_Occurred()) wire::conversion_error(path);
        if (!std::isfinite(result)) {
            const char *spelling = std::isnan(result) ? "nan"
                                   : result > 0.    ? "+inf"
                                                    : "-inf";
            return bio::Value{
                bio::Tagged{.kind = "float64", .fields = {{"value", spelling}}}};
        }
        std::array<char, 32> buffer{};
        const auto written = std::to_chars(
            buffer.data(), std::to_address(buffer.end()), result);
        if (written.ec != std::errc())
            throw std::runtime_error("float lexeme encode failed");
        std::string lexeme(buffer.data(),
                           static_cast<std::size_t>(written.ptr -
                                                    buffer.data()));
        // A float must never spell an integer token — vread::integer is
        // the Integral gate, and a "3"-spelled float would alias it.
        if (lexeme.find_first_of(".eE") == std::string::npos)
            lexeme += ".0";
        return bio::Value{bio::Number{std::move(lexeme)}};
    }
    if (nb::isinstance(value, types.np_ndarray)) {
        // owned_copy's dtype gate first — non-numeric arrays never
        // entered the owned copy.
        const auto kind =
            wire::string(value.attr("dtype").attr("kind"), path);
        if (kind != "b" && kind != "i" && kind != "u" && kind != "f" &&
            kind != "c")
            wire::invalid(path, "expected a numeric array dtype");
        // .tolist() produces the C-order nested plain-data tree — Python
        // scalars (float/int/bool), the identical element domain the old
        // nb::list(ndarray) iteration surfaced.
        return value_from_python(value.attr("tolist")(), types, path);
    }
    if (nb::isinstance<nb::dict>(value)) {
        bio::Value::Object out;
        for (const auto item : nb::borrow<nb::dict>(value)) {
            // owned_copy's key gate, verbatim: non-str keys never entered
            // the owned copy, so they cannot enter the owned Value.
            if (!nb::isinstance<nb::str>(item.first))
                wire::invalid(path, "expected string keys");
            out.emplace_back(wire::string(item.first, path),
                             value_from_python(item.second, types, path));
        }
        return bio::Value{std::move(out)};
    }
    if (nb::isinstance<nb::list>(value) ||
        nb::isinstance<nb::tuple>(value)) {
        bio::Value::Array out;
        for (const nb::handle item : wire::sequence(value, path))
            out.push_back(value_from_python(item, types, path));
        return bio::Value{std::move(out)};
    }
    if (nb::isinstance<nb::bytes>(value) ||
        nb::isinstance<nb::bytearray>(value)) {
        // Bytearray materializes through the same immutable-bytes copy the
        // Python codecs read it as.
        const nb::bytes bytes =
            nb::isinstance<nb::bytes>(value)
                ? nb::borrow<nb::bytes>(value)
                : nb::bytes(nb::module_::import_("builtins")
                                .attr("bytes")(value));
        const std::string_view view(bytes.c_str(), bytes.size());
        bio::Bytes out;
        out.value.reserve(view.size());
        for (const char unit : view)
            out.value.push_back(static_cast<std::byte>(unit));
        return bio::Value{std::move(out)};
    }
    // Dataclass instances read through attribute access on the binding
    // side; the owned document carries the member object instead —
    // dataclasses.asdict() performs the same recursive lowering.
    if (nb::bool_(types.dc_is_dataclass(value)) &&
        !nb::isinstance<nb::type_object>(value))
        return value_from_python(types.dc_asdict(value), types, path);
    wire::invalid(path,
                  "expected plain data (dict/list/tuple/scalar/array)");
}

[[nodiscard]] std::string artifact_path(nb::handle value) {
    nb::object resolved;
    if (nb::isinstance<nb::str>(value)) {
        resolved = nb::borrow<nb::object>(value);
    } else {
        resolved = nb::module_::import_("os").attr("fspath")(value);
        if (!nb::isinstance<nb::str>(resolved))
            throw std::invalid_argument(
                "model_path: expected a str or os.PathLike");
    }
    return wire::string(resolved, "model_path");
}

[[nodiscard]] std::vector<std::byte> artifact_bytes(const std::string &path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file)
        throw std::invalid_argument("model_path: cannot read artifact file");
    const std::streampos end = file.tellg();
    if (end <= 0 || end > std::streampos(std::numeric_limits<int>::max()))
        throw std::invalid_argument("model_path: cannot read artifact file");
    std::vector<char> raw(static_cast<std::size_t>(end));
    file.seekg(0);
    if (!file.read(raw.data(), static_cast<std::streamsize>(raw.size())))
        throw std::invalid_argument("model_path: cannot read artifact file");
    std::vector<std::byte> bytes(raw.size());
    std::ranges::copy(std::as_bytes(std::span{raw}), bytes.begin());
    return bytes;
}

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional handles mirror the fixed Python API
RuntimeBootstrap compose_bootstrap_handles(nb::handle model_path,
                                           nb::handle config,
                                           nb::handle state) {
    const nb::module_ numpy = nb::module_::import_("numpy");
    const nb::module_ numbers = nb::module_::import_("numbers");
    const nb::module_ dataclasses = nb::module_::import_("dataclasses");
    const MaterializeTypes types{
        .integral = nb::object(numbers.attr("Integral")),
        .real = nb::object(numbers.attr("Real")),
        .np_generic = nb::object(numpy.attr("generic")),
        .np_ndarray = nb::object(numpy.attr("ndarray")),
        .dc_is_dataclass = nb::object(dataclasses.attr("is_dataclass")),
        .dc_asdict = nb::object(dataclasses.attr("asdict"))};
    // The old ctor's ordinal positions, verbatim: the config envelope
    // decoded FIRST (init-list position), then artifact_path + the file
    // read, then the state document's digest comparison and staged
    // sections inside compose_bootstrap. Materialization is structural
    // only — every field-level validation keeps its ordinal position.
    const bio::Value config_doc =
        value_from_python(config, types, "config");
    RuntimeConfig decoded = decode_runtime_config(config_doc);
    const std::string path = artifact_path(model_path);
    ModelImage image{.mjb = artifact_bytes(path), .sha256 = {}};
    image.sha256 = bio::sha256(image.mjb);
    const bio::Value state_doc =
        value_from_python(state, types, "state");
    return compose_bootstrap(std::move(decoded), std::move(image),
                             state_doc);
}

void emit_reference_warning(NativeRideRuntime &runtime) {
    const auto message = runtime.take_pending_warning();
    if (message.has_value() &&
        PyErr_WarnEx(PyExc_RuntimeWarning, message->c_str(), 1) < 0)
        throw nb::python_error();
}

namespace {

// nanobind's class builders carry heavy template frames; each binding
// group gets its own noinline function so the per-function stack budget
// holds on unoptimized builds. Sanitizer instrumentation still pushes
// the chained def() temporaries past the guard — binding-time cost only.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void bind_snapshot_class(const nb::module_ &module) {
    nb::class_<RuntimeSnapshot>(module, "RuntimeSnapshot")
        .def_ro("generation", &RuntimeSnapshot::generation)
        .def_ro("step", &RuntimeSnapshot::step)
        .def_ro("time_s", &RuntimeSnapshot::time_s)
        // Every access mints a fresh capsule-owned array — the stored
        // vector stays an internal copy the caller can never reach.
        .def_prop_ro("integration_state",
                     [](const RuntimeSnapshot &s) {
                         return wire::owned_array<double>(
                             s.integration_state);
                     },
                     nb::rv_policy::move)
        .def_prop_ro("latest_sample", [](const RuntimeSnapshot &s) -> nb::object {
            return s.latest_sample == nullptr ? nb::none() :
                nb::cast(RuntimeSample{.value = s.latest_sample});
        })
        .def_prop_ro("view", [](const RuntimeSnapshot &s) { return wire_object_to_python(s.view); })
        .def_prop_ro("model_status", [](const RuntimeSnapshot &s) {
            return wire_object_to_python(s.model_status);
        })
        .def_prop_ro("first_failure", [](const RuntimeSnapshot &s) {
            return wire_to_python(s.first_failure.has_value() ? s.first_failure->as_wire() : Wire(nullptr));
        })
        .def_prop_ro("outcome",
                     [](const RuntimeSnapshot &s) -> nb::object {
                         return s.outcome.has_value()
                                    ? nb::object(nb::str(s.outcome->c_str(),
                                                         s.outcome->size()))
                                    : nb::none();
                     });
}
NATIVE_DIAG_POP

void bind_result_class(const nb::module_ &module) {
    nb::class_<AdvanceResult>(module, "NativeAdvanceResult")
        .def_ro("step", &AdvanceResult::step)
        .def_ro("time_s", &AdvanceResult::time_s)
        .def_ro("reason", &AdvanceResult::reason)
        .def_prop_ro("outcome",
                     [](const AdvanceResult &r) -> nb::object {
                         return r.outcome.has_value()
                                    ? nb::object(nb::str(r.outcome->c_str(),
                                                         r.outcome->size()))
                                    : nb::none();
                     });
}

// The A4 additions pushed the chained def() temporaries past the frame
// guard on unoptimized clang builds — binding-time cost only, like
// bind_drivetrain.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void bind_runtime_class(const nb::module_ &module) {
    nb::class_<NativeRideRuntime>(module, "NativeRideRuntime")
        .def("__init__",
             [](NativeRideRuntime *self, nb::handle model_path,
                nb::handle config, nb::handle state) {
                 new (self)
                     NativeRideRuntime(compose_bootstrap_handles(
                         model_path, config, state));
             },
             nb::arg("model_path"), nb::arg("config"), nb::arg("state"))
        .def("advance",
             [](NativeRideRuntime &self, std::int64_t target_step,
                nb::handle control, double front_brake_demand,
                double rear_brake_demand,
                nb::handle wall_budget_s) -> AdvanceResult {
                 // require_open() ran before the control decode in the
                 // monolithic binding — keep that rejection first.
                 if (self.closed())
                     throw std::logic_error("NativeRideRuntime is closed");
                 const RideControl command =
                     rider::parse_intent_control(control, "control");
                 if (!std::isfinite(front_brake_demand))
                     throw std::invalid_argument(
                         "front_brake_demand must be finite");
                 if (!std::isfinite(rear_brake_demand))
                     throw std::invalid_argument(
                         "rear_brake_demand must be finite");
                 std::optional<double> budget;
                 if (!wall_budget_s.is_none()) {
                     budget =
                         wire::finite_real(wall_budget_s, "wall_budget_s");
                     if (*budget < 0.)
                         throw std::invalid_argument(
                             "wall_budget_s must be nonnegative");
                 }
                 AdvanceResult result;
                 {
                     const nb::gil_scoped_release release;
                     result = self.advance(target_step, command,
                                           front_brake_demand,
                                           rear_brake_demand, budget);
                 }
                 emit_reference_warning(self);
                 return result;
             },
             nb::arg("target_step"), nb::arg("control"),
             nb::arg("front_brake_demand") = 0.,
             nb::arg("rear_brake_demand") = 0.,
             nb::arg("wall_budget_s") = nb::none())
        .def("probe_step_inputs",
             [](NativeRideRuntime &self, nb::handle control,
                double front_brake_demand,
                double rear_brake_demand) -> nb::dict {
                 if (self.closed())
                     throw std::logic_error("NativeRideRuntime is closed");
                 const RideControl command =
                     rider::parse_intent_control(control, "control");
                 if (!std::isfinite(front_brake_demand))
                     throw std::invalid_argument(
                         "front_brake_demand must be finite");
                 if (!std::isfinite(rear_brake_demand))
                     throw std::invalid_argument(
                         "rear_brake_demand must be finite");
                 PhysicalStep::ProbeResult probe;
                 {
                     const nb::gil_scoped_release release;
                     probe = self.probe_step_inputs(
                         command, front_brake_demand, rear_brake_demand);
                 }
                 nb::dict out;
                 nb::dict components;
                 for (const auto &[name, force] : probe.components)
                     components[nb::str(name.c_str(), name.size())] =
                         wire::owned_array<double>(
                             std::span<const double>(force));
                 out["components"] = components;
                 out["ctrl"] = wire::owned_array<double>(
                     std::span<const double>(probe.ctrl));
                 out["front_brake_bound_nm"] = probe.front_brake_bound_nm;
                 out["rear_brake_bound_nm"] = probe.rear_brake_bound_nm;
                 out["drive"] = wire_object_to_python(probe.drive);
                 out["contact_probe"] =
                     probe.has_contact_probe
                         ? nb::object(wire_object_to_python(
                               probe.contact_probe))
                         : nb::none();
                 out["sensors"] = wire_object_to_python(probe.sensors);
                 out["intent_signals"] =
                     wire_object_to_python(probe.intent_signals);
                 out["intent_inclination_rad"] =
                     probe.intent_inclination_rad;
                 return out;
             },
             nb::arg("control"), nb::arg("front_brake_demand") = 0.,
             nb::arg("rear_brake_demand") = 0.)
        .def("snapshot", &NativeRideRuntime::snapshot)
        .def("flush",
             [](NativeRideRuntime &self) {
                 {
                     const nb::gil_scoped_release release;
                     self.flush();
                 }
                 emit_reference_warning(self);
             })
        .def("prepare_samples", &NativeRideRuntime::prepare_samples)
        .def("acknowledge_samples", &NativeRideRuntime::acknowledge_samples,
             nb::arg("batch"))
        .def("accounting_state",
             [](const NativeRideRuntime &self) {
                 return wire_object_to_python(self.accounting_state());
             })
        .def("recorded_columns",
             [](const NativeRideRuntime &self) {
                 return columns_to_python(self.recorded_columns());
             })
        .def("recorded_intervals",
             [](const NativeRideRuntime &self) {
                 nb::list result;
                 for (const auto &sample : self.recorded_intervals())
                     result.append(wire_object_to_python(sample->as_wire()));
                 return result;
             })
        .def("seal_episode",
             [](NativeRideRuntime &self) {
                 OwnedNativeEpisode episode;
                 {
                     const nb::gil_scoped_release release;
                     episode = self.seal_episode();
                 }
                 emit_reference_warning(self);
                 return episode;
             })
        .def("release_samples", &NativeRideRuntime::release_samples)
        .def("set_finish_position",
             &NativeRideRuntime::set_finish_position,
             nb::arg("position_m"))
        .def("_test_fail_at_step", &NativeRideRuntime::test_fail_at_step,
             nb::arg("step"))
        .def("reset",
             [](NativeRideRuntime &self) {
                 {
                     const nb::gil_scoped_release release;
                     self.reset();
                 }
                 emit_reference_warning(self);
             })
        .def("close", &NativeRideRuntime::close)
        .def_prop_ro("closed", &NativeRideRuntime::closed);
}
NATIVE_DIAG_POP

} // namespace

void bind_runtime(nb::module_ &module) {
    // Production spindle kernels exposed as parity probes (plan A2):
    // same inputs, same arithmetic, same rejections as the Python helpers.
    module.def(
        "spindle_torque_waveform",
        [](double mean_nm, double phase_rad, double ripple) {
            return spindle::spindle_torque_waveform(mean_nm, phase_rad,
                                                   ripple);
        },
        nb::arg("mean_nm"), nb::arg("phase_rad"), nb::arg("ripple"));
    module.def(
        "crank_effort_ceiling",
        [](double power_w, double torque_limit_nm, double crank_rate_rad_s) {
            return spindle::crank_effort_ceiling(power_w, torque_limit_nm,
                                                 crank_rate_rad_s);
        },
        nb::arg("power_w"), nb::arg("torque_limit_nm"),
        nb::arg("crank_rate_rad_s"));
    bind_samples(module);
    bind_snapshot_class(module);
    bind_result_class(module);
    bind_runtime_class(module);
}

} // namespace runtime
