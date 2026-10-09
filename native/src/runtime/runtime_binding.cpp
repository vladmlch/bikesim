// runtime_binding.cpp — NativeRideRuntime: the owned C++ ride runtime.
//
// Construction order (every failure before Stepper construction leaves
// nothing behind; a later failure unwinds the unique_ptr):
//   1. config envelope validation (runtime_schema=1, all required fields)
//   2. artifact identity — sha256(model.mjb) == state.model_digest
//   3. Stepper construction (schema-2 writer_config parse happens inside)
//   4. bootstrap decode — model_dims/digest, integration width, resolved
//      model coefficients, mechanical snapshots, staged controller state
//   5. one ordered restore: reset_data -> model coefficients -> set_const
//      -> integration vector -> writer snapshots -> runtime counters
//      (mj_setConst rewrites qpos, so the integration vector lands after it)
//
// The integration vector is restored verbatim — mj_getState/mj_setState
// round-trip qacc_warmstart exactly, so no forward() runs here; derived
// mjData fields stay at reset values until the first forward (the same
// deferral Stepper::set_state documents).
#include "runtime_binding.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <mujoco/mujoco.h>
#include <nanobind/stl/string.h>

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"
#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../rider/intent_wire.hpp"
#include "../rider/spindle_math.hpp"
#include "../rider/spindle_wire.hpp"
#include "../stepper.hpp"
#include "../tyre/profile.hpp"

namespace nb = nanobind;

namespace {

// ---- FIPS 180-4 SHA-256 ------------------------------------------------
// The project links no crypto library; this compact implementation covers
// the single digest the bootstrap verifies (model artifact identity).

class Sha256 {
public:
    void update(std::span<const std::byte> bytes) noexcept {
        message_bytes_ += bytes.size();
        feed(bytes);
    }

    [[nodiscard]] std::string hex() {
        const std::uint64_t bit_length = message_bytes_ * 8U;
        const std::byte start{0x80};
        feed(std::span<const std::byte>{&start, 1});
        const std::byte zero{0x00};
        while (block_used_ != 56U)
            feed(std::span<const std::byte>{&zero, 1});
        std::array<std::byte, 8> length_bytes{};
        for (std::size_t i = 0; i < 8; ++i)
            length_bytes[7U - i] =
                static_cast<std::byte>(static_cast<unsigned char>(
                    (bit_length >> (i * 8U)) & 0xffU));
        feed(length_bytes);
        static constexpr std::array<char, 16> digits{'0', '1', '2', '3', '4',
            '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
        std::string out;
        out.reserve(64);
        for (const std::uint32_t word : h_)
            for (const int shift : {28, 24, 20, 16, 12, 8, 4, 0})
                out.push_back(digits[static_cast<std::size_t>(
                    (word >> static_cast<unsigned>(shift)) & 0xfU)]);
        return out;
    }

private:
    void feed(std::span<const std::byte> bytes) noexcept {
        while (!bytes.empty()) {
            const std::size_t take =
                std::min(bytes.size(), block_.size() - block_used_);
            std::ranges::copy(bytes.first(take),
                              std::span(block_).subspan(block_used_).begin());
            block_used_ += take;
            bytes = bytes.subspan(take);
            if (block_used_ == block_.size()) {
                compress();
                block_used_ = 0;
            }
        }
    }

    void compress() noexcept {
        std::array<std::uint32_t, 64> w{};
        for (std::size_t i = 0; i < 16; ++i)
            w[i] = std::to_integer<std::uint32_t>(block_[i * 4]) << 24U |
                   std::to_integer<std::uint32_t>(block_[i * 4 + 1]) << 16U |
                   std::to_integer<std::uint32_t>(block_[i * 4 + 2]) << 8U |
                   std::to_integer<std::uint32_t>(block_[i * 4 + 3]);
        for (std::size_t i = 16; i < 64; ++i)
            w[i] = w[i - 16] +
                   (std::rotr(w[i - 15], 7) ^ std::rotr(w[i - 15], 18) ^
                    (w[i - 15] >> 3U)) +
                   w[i - 7] +
                   (std::rotr(w[i - 2], 17) ^ std::rotr(w[i - 2], 19) ^
                    (w[i - 2] >> 10U));
        std::array<std::uint32_t, 8> v = h_;
        for (std::size_t i = 0; i < 64; ++i) {
            const std::uint32_t s1 = std::rotr(v[4], 6) ^
                                     std::rotr(v[4], 11) ^
                                     std::rotr(v[4], 25);
            const std::uint32_t ch =
                (v[4] & v[5]) ^ (~v[4] & v[6]);
            const std::uint32_t t1 =
                v[7] + s1 + ch + kRound[i] + w[i];
            const std::uint32_t s0 = std::rotr(v[0], 2) ^
                                     std::rotr(v[0], 13) ^
                                     std::rotr(v[0], 22);
            const std::uint32_t maj =
                (v[0] & v[1]) ^ (v[0] & v[2]) ^
                (v[1] & v[2]);
            const std::uint32_t t2 = s0 + maj;
            v[7] = v[6];
            v[6] = v[5];
            v[5] = v[4];
            v[4] = v[3] + t1;  // e <- d + t1, before d shifts out
            v[3] = v[2];
            v[2] = v[1];
            v[1] = v[0];
            v[0] = t1 + t2;
        }
        for (std::size_t i = 0; i < 8; ++i) h_[i] += v[i];
    }

    static constexpr std::array<std::uint32_t, 64> kRound{
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
        0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
        0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
        0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
        0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
        0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
        0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
        0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
        0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
        0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
    std::array<std::uint32_t, 8> h_{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                    0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                    0x1f83d9abU, 0x5be0cd19U};
    std::array<std::byte, 64> block_{};
    std::size_t block_used_ = 0;
    std::uint64_t message_bytes_ = 0;
};

[[nodiscard]] std::string sha256_file(const std::string &path) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
        throw std::invalid_argument("model_path: cannot read artifact file");
    Sha256 hash;
    // Heap buffer: a stack copy would dominate the caller's frame when this
    // one-shot reader gets inlined.
    const auto buffer = std::make_unique<std::array<char, 16384>>();
    const std::span<char> view(*buffer);
    while (file) {
        file.read(view.data(), static_cast<std::streamsize>(view.size()));
        const auto got = static_cast<std::size_t>(file.gcount());
        hash.update(std::as_bytes(view).first(got));
    }
    return hash.hex();
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

// ---- shared field readers ----------------------------------------------

[[nodiscard]] double positive_field(const wire::Dict &d, const char *key) {
    const double value =
        wire::finite_real(wire::field(d, key), d.child(key));
    if (!(value > 0.)) wire::invalid(d.child(key), "expected positive value");
    return value;
}

[[nodiscard]] double nonnegative_field(const wire::Dict &d, const char *key) {
    const double value =
        wire::finite_real(wire::field(d, key), d.child(key));
    if (value < 0.)
        wire::invalid(d.child(key), "expected nonnegative value");
    return value;
}

[[nodiscard]] int positive_int_field(const wire::Dict &d, const char *key) {
    const int value = wire::integer32(wire::field(d, key), d.child(key));
    if (value <= 0)
        wire::invalid(d.child(key), "expected positive integer");
    return value;
}

[[nodiscard]] runtime::JointReference joint_reference(const wire::Dict &d,
                                                      const char *key) {
    const wire::Dict ref = wire::section(d, key);
    wire::exact(ref, wire::keys("joint", "dof_index"));
    runtime::JointReference out{
        .joint = wire::string(wire::field(ref, "joint"), ref.child("joint")),
        .dof_index = wire::integer32(wire::field(ref, "dof_index"),
                                     ref.child("dof_index"))};
    if (out.dof_index < 0)
        wire::invalid(ref.child("dof_index"), "expected nonnegative index");
    return out;
}

[[nodiscard]] std::optional<std::string> optional_string_field(
    const wire::Dict &d, const char *key) {
    const nb::object value = wire::field(d, key);
    if (value.is_none()) return std::nullopt;
    return wire::string(value, d.child(key));
}

// owned dict-copy of a required mapping field.
[[nodiscard]] nb::object owned_mapping(const wire::Dict &d, const char *key) {
    const nb::dict source =
        wire::mapping(wire::field(d, key), d.child(key));
    return runtime::owned_copy(source, d.child(key));
}

[[nodiscard]] nb::object owned_optional(const wire::Dict &d, const char *key) {
    const nb::object value = wire::field(d, key);
    if (value.is_none()) return nb::borrow<nb::object>(value);
    return runtime::owned_copy(
        wire::mapping(value, d.child(key)), d.child(key));
}

// ---- Wire <-> Python ----------------------------------------------------
// The step emits pure-C++ Wire trees so the GIL-free loop never touches a
// Python object; these converters run strictly at the held-GIL boundary.

nb::object wire_to_python(const runtime::Wire &w);

[[nodiscard]] nb::dict wire_object_to_python(
    const runtime::WireObject &object) {
    nb::dict out;
    for (const auto &[key, value] : object)
        out[nb::str(key.c_str(), key.size())] = wire_to_python(value);
    return out;
}

nb::object wire_to_python(const runtime::Wire &w) {
    return std::visit(
        [](const auto &value) -> nb::object {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, std::monostate>)
                return nb::none();
            else if constexpr (std::is_same_v<T, bool>)
                return nb::object(nb::bool_(value));
            else if constexpr (std::is_same_v<T, std::int64_t>)
                return nb::object(nb::int_(value));
            else if constexpr (std::is_same_v<T, double>)
                return nb::object(nb::float_(value));
            else if constexpr (std::is_same_v<T, std::string>)
                return nb::object(nb::str(value.c_str(), value.size()));
            else if constexpr (std::is_same_v<T, runtime::WireArray>) {
                nb::list out;
                for (const auto &element : value)
                    out.append(wire_to_python(element));
                return out;
            } else {
                return wire_object_to_python(value);
            }
        },
        w.value);
}

// plain() inverse for the wire sections the step state round-trips
// verbatim (elastic_energy_j's named-term dict is the only consumer).
// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the plain-data tree
[[nodiscard]] runtime::Wire python_to_wire(nb::handle value,
                                           std::string_view path) {
    if (value.is_none()) return runtime::Wire{nullptr};
    if (nb::isinstance<nb::bool_>(value) || wire::numeric_boolean(value))
        return runtime::Wire{nb::cast<bool>(value)};
    const auto numbers = nb::module_::import_("numbers");
    if (nb::isinstance(value, numbers.attr("Integral")))
        return runtime::Wire{wire::integer(value, path)};
    if (nb::isinstance(value, numbers.attr("Real")))
        return runtime::Wire{wire::real(value, path)};
    if (nb::isinstance<nb::str>(value))
        return runtime::Wire{wire::string(value, path)};
    if (nb::isinstance<nb::dict>(value)) {
        runtime::WireObject out;
        for (const auto item : nb::borrow<nb::dict>(value))
            out.emplace_back(wire::string(item.first, path),
                             python_to_wire(item.second, path));
        return runtime::Wire{std::move(out)};
    }
    if (nb::isinstance<nb::list>(value) ||
        nb::isinstance<nb::tuple>(value)) {
        runtime::WireArray out;
        for (const nb::handle item : wire::sequence(value, path))
            out.push_back(python_to_wire(item, path));
        return runtime::Wire{std::move(out)};
    }
    wire::invalid(path, "expected plain data (dict/list/scalar)");
}

// ---- state.runtime section decoders --------------------------------------
// Every state_dict() shape validated key-for-key against setup.py's
// emission; rejected input leaves the live step untouched (the restore
// happens only after the whole StepState decoded).

[[nodiscard]] double real_field(const wire::Dict &d, const char *key) {
    return wire::real(wire::field(d, key), d.child(key));
}

[[nodiscard]] std::optional<double>
optional_field(const wire::Dict &d, const char *key) {
    return wire::optional_real(wire::field(d, key), d.child(key));
}

[[nodiscard]] bool bool_field(const wire::Dict &d, const char *key) {
    return wire::boolean(wire::field(d, key), d.child(key));
}

[[nodiscard]] std::int64_t int_field(const wire::Dict &d, const char *key) {
    return wire::integer(wire::field(d, key), d.child(key));
}

[[nodiscard]] runtime::GroundedFilter::State
parse_grounded(const wire::Dict &d) {
    wire::exact(d, wire::keys("last_time_s", "last_loaded_s", "value"));
    return {.last_time_s = optional_field(d, "last_time_s"),
            .last_loaded_s = optional_field(d, "last_loaded_s"),
            .value = bool_field(d, "value")};
}

[[nodiscard]] runtime::BridgedLoad::State
parse_bridged(const wire::Dict &d) {
    wire::exact(d, wire::keys("held_load_n", "unloaded_steps"));
    return {.held_load_n = real_field(d, "held_load_n"),
            .unloaded_steps = int_field(d, "unloaded_steps")};
}

[[nodiscard]] runtime::RuntimeContactQuery::State
parse_query_state(const wire::Dict &d) {
    wire::exact(d,
                wire::keys("front_load", "rear_load",
                           "front_controller_grounded",
                           "rear_controller_grounded"));
    return {.front_load = parse_bridged(wire::section(d, "front_load")),
            .rear_load = parse_bridged(wire::section(d, "rear_load")),
            .front_controller = parse_grounded(
                wire::section(d, "front_controller_grounded")),
            .rear_controller = parse_grounded(
                wire::section(d, "rear_controller_grounded"))};
}

[[nodiscard]] runtime::BalanceMonitor::State
parse_balance(const wire::Dict &d) {
    wire::exact(d, wire::keys("event", "low_speed_s", "grace_left_s",
                              "started_s", "low_started_s", "last_time_s"));
    runtime::BalanceMonitor::State out;
    const nb::object event = wire::field(d, "event");
    if (!event.is_none()) {
        const wire::Dict e{
            .value = wire::mapping(event, d.child("event")),
            .path = d.child("event")};
        wire::exact(e, wire::keys("time_s", "position_m", "speed_mps"));
        out.event = runtime::BalanceLostEvent{
            .time_s = real_field(e, "time_s"),
            .position_m = real_field(e, "position_m"),
            .speed_mps = real_field(e, "speed_mps")};
    }
    out.low_speed_s = real_field(d, "low_speed_s");
    out.grace_left_s = real_field(d, "grace_left_s");
    out.started_s = optional_field(d, "started_s");
    out.low_started_s = optional_field(d, "low_started_s");
    out.last_time_s = optional_field(d, "last_time_s");
    return out;
}

[[nodiscard]] std::optional<runtime::CrashEvent>
parse_crash(const wire::Dict &d) {
    wire::exact(d, wire::keys("event"));
    const nb::object event = wire::field(d, "event");
    if (event.is_none()) return std::nullopt;
    const wire::Dict e{.value = wire::mapping(event, d.child("event")),
                       .path = d.child("event")};
    wire::exact(e,
                wire::keys("cause", "time_s", "position_m", "pitch_rad"));
    return runtime::CrashEvent{
        .cause = wire::string(wire::field(e, "cause"), e.child("cause")),
        .time_s = real_field(e, "time_s"),
        .position_m = real_field(e, "position_m"),
        .pitch_rad = real_field(e, "pitch_rad")};
}

[[nodiscard]] runtime::RuntimeContacts
parse_contacts(const wire::Dict &d) {
    wire::exact(d,
                wire::keys("front_load_n", "rear_load_n", "front_support_n",
                           "rear_support_n", "handlebar_load_n",
                           "front_controller_grounded",
                           "rear_controller_grounded", "front_slip_mps",
                           "rear_slip_mps"));
    runtime::RuntimeContacts out;
    out.front_load_n = real_field(d, "front_load_n");
    out.rear_load_n = real_field(d, "rear_load_n");
    out.front_support_n = real_field(d, "front_support_n");
    out.rear_support_n = real_field(d, "rear_support_n");
    out.handlebar_load_n = real_field(d, "handlebar_load_n");
    // _contacts_state emits plain bools for the controller-grounded pair.
    out.front_controller_grounded = bool_field(d, "front_controller_grounded");
    out.rear_controller_grounded = bool_field(d, "rear_controller_grounded");
    out.front_slip_mps = optional_field(d, "front_slip_mps");
    out.rear_slip_mps = optional_field(d, "rear_slip_mps");
    return out;
}

[[nodiscard]] runtime::EnergyState
parse_energy(const wire::Dict &d) {
    wire::exact(d, wire::keys(
                       "initial_energy_j", "energy_scale_j", "loss_j",
                       "active_work_j", "external_work_j", "solver_work_j",
                       "muscle_signed_j", "muscle_positive_j",
                       "motor_signed_j", "motor_positive_j",
                       "constraint_absolute_j", "initial_battery_j",
                       "electrical_work_j", "mechanical_energy_j",
                       "elastic_energy_j", "residual_j"));
    runtime::EnergyState out;
    out.initial_energy_j = optional_field(d, "initial_energy_j");
    out.energy_scale_j = real_field(d, "energy_scale_j");
    out.loss_j = real_field(d, "loss_j");
    out.active_work_j = real_field(d, "active_work_j");
    out.external_work_j = real_field(d, "external_work_j");
    out.solver_work_j = real_field(d, "solver_work_j");
    out.muscle_signed_j = real_field(d, "muscle_signed_j");
    out.muscle_positive_j = real_field(d, "muscle_positive_j");
    out.motor_signed_j = real_field(d, "motor_signed_j");
    out.motor_positive_j = real_field(d, "motor_positive_j");
    out.constraint_absolute_j = real_field(d, "constraint_absolute_j");
    out.initial_battery_j = optional_field(d, "initial_battery_j");
    out.electrical_work_j = real_field(d, "electrical_work_j");
    out.mechanical_energy_j = real_field(d, "mechanical_energy_j");
    const runtime::Wire elastic =
        python_to_wire(wire::field(d, "elastic_energy_j"),
                       d.child("elastic_energy_j"));
    if (const auto *object = std::get_if<runtime::WireObject>(
            &elastic.value))
        out.elastic_energy_j = *object;
    else
        wire::invalid(d.child("elastic_energy_j"), "expected dict");
    out.residual_j = real_field(d, "residual_j");
    return out;
}

// state.runtime + the sibling top-level sections (signals, rider_intent,
// rider_controller) -> the step's full restore input. history/
// model_status/monitor stay owned blobs on the BootstrapState until A4.
[[nodiscard]] runtime::StepState
parse_step_state(const runtime::BootstrapState &bootstrap) {
    const wire::Dict rt{.value = wire::mapping(bootstrap.runtime,
                                              "state.runtime"),
                        .path = "state.runtime"};
    wire::exact(rt, wire::keys(
                        "step", "generation", "record_decimation",
                        "applied_control", "held_control",
                        "held_rider_terms", "rollback_hold",
                        "research_accounting_valid", "initializing",
                        "filters", "balance", "crash", "contacts",
                        "contact_query", "probe_query", "energy", "history",
                        "model_status", "monitor"));
    runtime::StepState out;
    out.step = int_field(rt, "step");
    if (out.step < 0)
        wire::invalid(rt.child("step"), "expected nonnegative integer");
    out.generation =
        wire::integer32(wire::field(rt, "generation"),
                        rt.child("generation"));
    if (out.generation <= 0)
        wire::invalid(rt.child("generation"), "expected positive integer");
    out.record_decimation =
        wire::integer32(wire::field(rt, "record_decimation"),
                        rt.child("record_decimation"));
    if (out.record_decimation <= 0)
        wire::invalid(rt.child("record_decimation"),
                      "expected positive integer");
    out.applied_control = rider::parse_intent_control(
        wire::field(rt, "applied_control"),
        rt.child("applied_control"));
    const nb::object held = wire::field(rt, "held_control");
    if (!held.is_none())
        out.held_control = rider::parse_named_reals(
            held, rt.child("held_control"));
    const nb::object terms = wire::field(rt, "held_rider_terms");
    if (!terms.is_none()) {
        const wire::Dict map{
            .value = wire::mapping(terms, rt.child("held_rider_terms")),
            .path = rt.child("held_rider_terms")};
        rider::NamedEntries<rider::JointTerms> decoded;
        for (const auto item : map.value) {
            const std::string joint = wire::string(item.first, map.path);
            decoded.emplace_back(
                joint,
                rider::parse_joint_terms(item.second,
                                         map.child(joint.c_str())));
        }
        out.held_rider_terms = std::move(decoded);
    }
    out.rollback_hold = bool_field(rt, "rollback_hold");
    out.research_accounting_valid =
        bool_field(rt, "research_accounting_valid");
    out.initializing = bool_field(rt, "initializing");
    const wire::Dict filters = wire::section(rt, "filters");
    wire::exact(filters, wire::keys("front", "rear"));
    out.filters = {parse_grounded(wire::section(filters, "front")),
                   parse_grounded(wire::section(filters, "rear"))};
    out.balance = parse_balance(wire::section(rt, "balance"));
    out.crash = parse_crash(wire::section(rt, "crash"));
    out.contacts = parse_contacts(wire::section(rt, "contacts"));
    out.contact_query =
        parse_query_state(wire::section(rt, "contact_query"));
    out.probe_query = parse_query_state(wire::section(rt, "probe_query"));
    out.energy = parse_energy(wire::section(rt, "energy"));
    out.signals =
        rider::parse_intent_signals(bootstrap.signals, "state.signals");
    out.rider_intent =
        rider::parse_intent_state(bootstrap.rider_intent,
                                  "state.rider_intent");
    if (!bootstrap.rider_controller.is_none())
        out.rider_controller = rider::parse_spindle_state(
            bootstrap.rider_controller, "state.rider_controller");
    return out;
}

// environment.py crash_reason — the latched CrashEvent's outcome label.
[[nodiscard]] std::optional<std::string>
crash_outcome(const std::optional<runtime::CrashEvent> &event) {
    if (!event.has_value()) return std::nullopt;
    if (event->cause == "pitch_over")
        return std::string(event->pitch_rad < 0. ? "crash:loop_out"
                                                 : "crash:endo");
    return "crash:" + event->cause;
}

// Reentrancy guard — advance() and reset() are the two entries that can
// run without the GIL; a nested or concurrent call fails fast instead of
// observing a half-advanced runtime.
class AdvancementGuard {
public:
    explicit AdvancementGuard(std::atomic<bool> &flag) : flag_(flag) {
        bool expected = false;
        if (!flag_.compare_exchange_strong(expected, true))
            throw std::runtime_error(
                "advance already in progress on this runtime");
    }
    ~AdvancementGuard() { flag_.store(false); }
    AdvancementGuard(const AdvancementGuard &) = delete;
    AdvancementGuard &operator=(const AdvancementGuard &) = delete;
    AdvancementGuard(AdvancementGuard &&) = delete;
    AdvancementGuard &operator=(AdvancementGuard &&) = delete;

private:
    std::atomic<bool> &flag_;
};

} // namespace

namespace runtime {

// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the plain-data tree
nb::object owned_copy(nb::handle value, std::string_view path) {
    if (value.is_none() || nb::isinstance<nb::str>(value) ||
        nb::isinstance<nb::bool_>(value) ||
        wire::numeric_boolean(value))
        return nb::borrow<nb::object>(value);
    const auto numbers = nb::module_::import_("numbers");
    if (nb::isinstance(value, numbers.attr("Integral")) ||
        nb::isinstance(value, numbers.attr("Real")))
        return nb::borrow<nb::object>(value);
    if (nb::isinstance<nb::dict>(value)) {
        nb::dict out;
        for (const auto item : nb::borrow<nb::dict>(value)) {
            if (!nb::isinstance<nb::str>(item.first))
                wire::invalid(path, "expected string keys");
            out[nb::handle(item.first)] = owned_copy(item.second, path);
        }
        return out;
    }
    if (nb::isinstance<nb::list>(value)) {
        nb::list out;
        for (nb::handle const item : nb::borrow<nb::list>(value))
            out.append(owned_copy(item, path));
        return out;
    }
    if (nb::isinstance<nb::tuple>(value)) {
        nb::list stage;
        for (nb::handle const item : nb::borrow<nb::tuple>(value))
            stage.append(owned_copy(item, path));
        return nb::steal<nb::object>(PySequence_Tuple(stage.ptr()));
    }
    if (nb::isinstance(value, nb::module_::import_("numpy").attr("ndarray"))) {
        const auto kind =
            wire::string(value.attr("dtype").attr("kind"), path);
        if (kind != "b" && kind != "i" && kind != "u" && kind != "f" &&
            kind != "c")
            wire::invalid(path, "expected a numeric array dtype");
        return value.attr("copy")();
    }
    wire::invalid(path, "expected plain data (dict/list/tuple/scalar/array)");
}

RuntimeConfig parse_runtime_config(nb::handle raw) {
    const wire::Dict root{.value = wire::mapping(raw, "config"),
                          .path = "config"};
    constexpr auto required = wire::keys(
        "runtime_schema", "timestep_s", "control_period_s",
        "control_period_steps", "physics_mode", "drive_mode", "strict",
        "record_decimation", "writer_config", "rider_controller",
        "rider_intent", "monitors", "geometry");
    wire::exact(root, required);
    if (wire::integer32(wire::field(root, "runtime_schema"),
                        root.child("runtime_schema")) != kRuntimeSchema)
        wire::invalid(root.child("runtime_schema"),
                      "unsupported runtime schema");

    RuntimeConfig out;
    out.timestep_s = positive_field(root, "timestep_s");
    out.control_period_s = positive_field(root, "control_period_s");
    out.control_period_steps =
        positive_int_field(root, "control_period_steps");
    if (std::abs(out.timestep_s * out.control_period_steps -
                 out.control_period_s) > 1e-9 * out.control_period_s)
        wire::invalid(root.child("control_period_s"),
                      "control_period_steps * timestep_s mismatch");
    if (wire::string(wire::field(root, "physics_mode"),
                     root.child("physics_mode")) != "physical")
        wire::invalid(root.child("physics_mode"), "unsupported physics mode");
    if (wire::string(wire::field(root, "drive_mode"),
                     root.child("drive_mode")) != "articulated_effort")
        wire::invalid(root.child("drive_mode"), "unsupported drive mode");
    out.strict =
        wire::boolean(wire::field(root, "strict"), root.child("strict"));
    out.record_decimation = positive_int_field(root, "record_decimation");

    out.writer_config = owned_mapping(root, "writer_config");
    out.rider_controller = owned_mapping(root, "rider_controller");
    out.rider_intent = owned_mapping(root, "rider_intent");

    const wire::Dict monitors = wire::section(root, "monitors");
    wire::exact(monitors,
                wire::keys("balance_floor_mps", "balance_dwell_s",
                           "balance_grace_s", "grounded_hold_s"));
    out.monitors = MonitorConfig{
        .balance_floor_mps = positive_field(monitors, "balance_floor_mps"),
        .balance_dwell_s = positive_field(monitors, "balance_dwell_s"),
        .balance_grace_s = nonnegative_field(monitors, "balance_grace_s"),
        .grounded_hold_s = nonnegative_field(monitors, "grounded_hold_s")};

    const wire::Dict geometry = wire::section(root, "geometry");
    wire::exact(geometry,
                wire::keys("crank_length_m", "brake_dofs", "wheel_bodies",
                           "com_marker_site", "pose"));
    out.geometry.crank_length_m = positive_field(geometry, "crank_length_m");
    const wire::Dict brake = wire::section(geometry, "brake_dofs");
    wire::exact(brake, wire::keys("front", "rear"));
    out.geometry.front_brake = joint_reference(brake, "front");
    out.geometry.rear_brake = joint_reference(brake, "rear");
    const wire::Dict bodies = wire::section(geometry, "wheel_bodies");
    wire::exact(bodies, {}, wire::keys("front", "rear"));
    out.geometry.front_wheel_body =
        bodies.contains("front")
            ? optional_string_field(bodies, "front")
            : std::nullopt;
    out.geometry.rear_wheel_body =
        bodies.contains("rear") ? optional_string_field(bodies, "rear")
                                : std::nullopt;
    out.geometry.com_marker_site =
        optional_string_field(geometry, "com_marker_site");
    out.geometry.pose = owned_optional(geometry, "pose");
    return out;
}

std::string expected_model_digest(nb::handle raw_state) {
    const wire::Dict root{.value = wire::mapping(raw_state, "state"),
                          .path = "state"};
    return wire::string(wire::field(root, "model_digest"),
                        root.child("model_digest"));
}

// ---- bootstrap section parsers ------------------------------------------
// Each decoder owns its wire::Dict temporaries; keeping them in separate
// functions holds every frame under the 8 KiB guard.
namespace {

void parse_model_dims(const wire::Dict &root, const mjModel &model) {
    const wire::Dict dims = wire::section(root, "model_dims");
    wire::exact(dims,
                wire::keys("nq", "nv", "nu", "na", "nbody", "njnt", "ntendon",
                           "nwrap", "nsite", "ngeom", "neq", "nsensor",
                           "nsensordata"));
    const std::array<std::pair<std::string_view, mjtSize>, 13> actual_dims{{
        {"nq", model.nq},         {"nv", model.nv},
        {"nu", model.nu},         {"na", model.na},
        {"nbody", model.nbody},   {"njnt", model.njnt},
        {"ntendon", model.ntendon}, {"nwrap", model.nwrap},
        {"nsite", model.nsite},   {"ngeom", model.ngeom},
        {"neq", model.neq},       {"nsensor", model.nsensor},
        {"nsensordata", model.nsensordata},
    }};
    for (const auto &[name, actual] : actual_dims) {
        const std::string key(name);
        const auto expected = wire::integer(
            wire::field(dims, key.c_str()), dims.child(key.c_str()));
        if (!std::cmp_equal(expected, actual))
            wire::invalid(dims.child(key.c_str()),
                          "model dimension mismatch");
    }
}

std::vector<double> parse_integration_state(const wire::Dict &root,
                                            const mjModel &model) {
    std::vector<double> values = wire::vector(
        wire::field(root, "integration_state"), "state.integration_state");
    const mjtSize width =
        engine::state_size(&model, static_cast<int>(mjSTATE_INTEGRATION));
    if (!std::cmp_equal(values.size(), width))
        wire::invalid("state.integration_state",
                      "width does not match the model");
    return values;
}

ModelMutableState parse_model_mutable(const wire::Dict &root,
                                      const mjModel &model) {
    ModelMutableState out;
    const wire::Dict mutable_section = wire::section(root, "model_mutable");
    wire::exact(mutable_section, wire::keys("dof_frictionloss", "site_pos"));
    const wire::Dict frictionloss =
        wire::section(mutable_section, "dof_frictionloss");
    // mjModel carries no per-joint dof count — it derives from jnt_type.
    const std::span<const int> joint_types =
        std::views::counted(model.jnt_type, model.njnt);
    const auto dof_count = [&joint_types](int joint) {
        switch (joint_types[static_cast<std::size_t>(joint)]) {
        case mjJNT_FREE: return 6;
        case mjJNT_BALL: return 3;
        default: return 1;
        }
    };
    const std::span<const int> dofadr =
        std::views::counted(model.jnt_dofadr, model.njnt);
    for (const auto item : frictionloss.value) {
        const std::string side =
            wire::string(item.first, frictionloss.path);
        const wire::Dict entry{
            .value = wire::mapping(item.second, frictionloss.child(side.c_str())),
            .path = frictionloss.child(side.c_str())};
        wire::exact(entry, wire::keys("joint", "dof_index", "value"));
        FrictionLossEntry row{
            .joint = wire::string(wire::field(entry, "joint"),
                                  entry.child("joint")),
            .dof_index = wire::integer32(wire::field(entry, "dof_index"),
                                         entry.child("dof_index")),
            .dof_adr = 0,
            .value = wire::finite_real(wire::field(entry, "value"),
                                       entry.child("value"))};
        const int joint =
            mj_name2id(&model, mjOBJ_JOINT, row.joint.c_str());
        if (joint < 0)
            wire::invalid(entry.child("joint"), "unknown joint");
        if (row.dof_index < 0 ||
            row.dof_index >= dof_count(joint))
            wire::invalid(entry.child("dof_index"), "out of range");
        row.dof_adr = dofadr[static_cast<std::size_t>(joint)] +
                      row.dof_index;
        out.dof_frictionloss.push_back(std::move(row));
    }
    const wire::Dict site_pos = wire::section(mutable_section, "site_pos");
    for (const auto item : site_pos.value) {
        const std::string name = wire::string(item.first, site_pos.path);
        const int site = mj_name2id(&model, mjOBJ_SITE, name.c_str());
        if (site < 0)
            wire::invalid(site_pos.child(name.c_str()), "unknown site");
        out.site_pos.push_back(SitePosEntry{
            .site = name, .site_id = site,
            .value = wire::fixed<3>(item.second, site_pos.child(name.c_str()))});
    }
    return out;
}

TireStateRow parse_tire_state(const wire::Dict &root) {
    TireStateRow out;
    const wire::Dict tire = wire::section(root, "tire");
    wire::exact(tire, wire::keys("names", "row"));
    for (nb::handle const value :
         wire::sequence(wire::field(tire, "names"), tire.child("names")))
        out.names.push_back(wire::string(value, tire.child("names")));
    // NaN marks unset brush columns — real, not finite_real.
    for (nb::handle const value :
         wire::sequence(wire::field(tire, "row"), tire.child("row")))
        out.row.push_back(wire::real(value, tire.child("row")));
    if (out.names.size() != out.row.size())
        wire::invalid(tire.child("row"), "names/row width mismatch");
    return out;
}

} // namespace

BootstrapState parse_bootstrap(nb::handle raw_state, const mjModel &model,
                               const Stepper &stepper) {
    const wire::Dict root{.value = wire::mapping(raw_state, "state"),
                          .path = "state"};
    constexpr auto required = wire::keys(
        "schema", "model_digest", "model_dims", "integration_state",
        "model_mutable", "tire", "drive", "rider_contacts",
        "rider_controller", "rider_intent", "signals", "runtime");
    wire::exact(root, required);
    if (wire::integer32(wire::field(root, "schema"), "state.schema") !=
        kRuntimeSchema)
        wire::invalid("state.schema", "unsupported bootstrap schema");

    BootstrapState out;
    out.model_digest = expected_model_digest(raw_state);
    parse_model_dims(root, model);
    out.integration_state = parse_integration_state(root, model);
    out.model_mutable = parse_model_mutable(root, model);
    out.tire = parse_tire_state(root);
    out.drive = parse_drive_snapshot(wire::field(root, "drive"),
                                     "state.drive");

    const nb::object contacts = wire::field(root, "rider_contacts");
    if (!contacts.is_none()) {
        try {
            out.rider_contacts =
                parse_rider_contacts_state(contacts, stepper.rider_contacts());
        } catch (const std::logic_error &) {
            wire::invalid("state.rider_contacts",
                          "the runtime config has no rider_contacts writer");
        }
    }

    out.rider_controller = owned_optional(root, "rider_controller");
    out.rider_intent = owned_mapping(root, "rider_intent");
    out.signals = owned_mapping(root, "signals");

    const wire::Dict runtime_section = wire::section(root, "runtime");
    wire::exact(runtime_section,
                wire::keys("step", "generation", "record_decimation",
                           "applied_control", "held_control",
                           "held_rider_terms", "rollback_hold",
                           "research_accounting_valid", "initializing",
                           "filters", "balance", "crash", "contacts",
                           "contact_query", "probe_query", "energy", "history",
                           "model_status", "monitor"));
    out.step = wire::integer(wire::field(runtime_section, "step"),
                             runtime_section.child("step"));
    if (out.step < 0)
        wire::invalid(runtime_section.child("step"),
                      "expected nonnegative integer");
    out.generation = wire::integer32(
        wire::field(runtime_section, "generation"),
        runtime_section.child("generation"));
    if (out.generation <= 0)
        wire::invalid(runtime_section.child("generation"),
                      "expected positive integer");
    out.runtime = runtime::owned_copy(runtime_section.value,
                                      runtime_section.path);
    return out;
}

// ---- NativeRideRuntime --------------------------------------------------

void NativeRideRuntime::require_open() const {
    if (closed_) throw std::logic_error("NativeRideRuntime is closed");
}

namespace {

// The step-owned slice of the runtime envelope — physics/drive modes are
// already pinned at parse_runtime_config.
[[nodiscard]] StepConfig step_config(const RuntimeConfig &config) {
    StepConfig out;
    out.timestep_s = config.timestep_s;
    out.control_period_s = config.control_period_s;
    out.record_decimation = config.record_decimation;
    out.strict = config.strict;
    out.crank_length_m = config.geometry.crank_length_m;
    const wire::Dict rider_cfg{
        .value = wire::mapping(config.rider_controller,
                               "config.rider_controller"),
        .path = "config.rider_controller"};
    out.road_lookahead_m = wire::finite_real(
        wire::field(rider_cfg, "road_lookahead_m"),
        rider_cfg.child("road_lookahead_m"));
    out.rider_present = !config.geometry.pose.is_none();
    out.monitors = config.monitors;
    out.intent = rider::parse_intent_config(config.rider_intent,
                                            "config.rider_intent");
    return out;
}

// StaticBrakeApplier — the geometry section's name-indexed brake refs plus
// the drive config's ceiling, resolved to DOF addresses once.
[[nodiscard]] StaticBrake static_brake(const RuntimeConfig &config,
                                       const mjModel &model,
                                       const Stepper &stepper) {
    const auto resolve = [&](const JointReference &ref, const char *what) {
        const int joint =
            mj_name2id(&model, mjOBJ_JOINT, ref.joint.c_str());
        if (joint < 0)
            throw std::invalid_argument(std::string("unknown ") + what +
                                        " brake joint '" + ref.joint + "'");
        const std::span<const int> joint_types =
            std::views::counted(model.jnt_type, model.njnt);
        const int dof_count = [&] {
            switch (joint_types[static_cast<std::size_t>(joint)]) {
            case mjJNT_FREE: return 6;
            case mjJNT_BALL: return 3;
            default: return 1;
            }
        }();
        if (ref.dof_index < 0 || ref.dof_index >= dof_count)
            throw std::invalid_argument(
                std::string("config.geometry.brake_dofs: ") + what +
                " dof_index out of range");
        const std::span<const int> dofadr =
            std::views::counted(model.jnt_dofadr, model.njnt);
        return dofadr[static_cast<std::size_t>(joint)] + ref.dof_index;
    };
    return {resolve(config.geometry.front_brake, "front"),
            resolve(config.geometry.rear_brake, "rear"),
            stepper.drive().config().brake_ceiling_nm};
}

// The PhysicalStep assembly — pose/config/vertices decode plus the
// constructor — carries a frame too large for the debug stack budget, so
// it owns a separate function scope.
std::unique_ptr<PhysicalStep>
make_physical_step(Stepper &stepper, const RuntimeConfig &config) {
    // The compiled terrain cross-section is constant model geometry — the
    // tire writer's own copy was read from this same post-forward data.
    std::vector<std::array<double, 2>> vertices =
        biketyre::compiled_profile_vertices(stepper.model(),
                                            stepper.data(), "terrain");
    std::optional<rider::SpindlePose> pose;
    std::optional<rider::SpindleConfig> rider_cfg;
    if (!config.geometry.pose.is_none()) {
        pose = rider::parse_spindle_pose(config.geometry.pose,
                                         "config.geometry.pose");
        rider_cfg = rider::parse_spindle_config(
            config.rider_controller, "config.rider_controller");
    }
    return std::make_unique<PhysicalStep>(
        stepper, step_config(config), std::move(vertices),
        static_brake(config, *stepper.model(), stepper), pose, rider_cfg);
}

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature is the spec'd Python API
NativeRideRuntime::NativeRideRuntime(nb::handle model_path,
                                     nb::handle config, nb::handle state)
    : config_(parse_runtime_config(config)) {
    init(model_path, state);
}

// One-shot bootstrap decode — like bind_drivetrain, its frame is decode
// temporaries rather than per-step code, so the frame guard is waived.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) decode handles, callee pins their roles
void NativeRideRuntime::init(nb::handle model_path, nb::handle state) {
    const std::string path = artifact_path(model_path);
    if (sha256_file(path) != expected_model_digest(state))
        throw std::invalid_argument(
            "state.model_digest: model artifact bytes do not match");
    stepper_ = std::make_unique<Stepper>(path, config_.writer_config);
    bootstrap_ = parse_bootstrap(state, *stepper_->model(), *stepper_);
    physical_ = make_physical_step(*stepper_, config_);
    bootstrap_step_state_ = parse_step_state(bootstrap_);
    // The captured controller state and the configured rider ensemble
    // arrive together (setup.py emits both or neither).
    if (bootstrap_step_state_.rider_controller.has_value() !=
        physical_->rider_present())
        throw std::invalid_argument(
            "state.rider_controller does not match the rider config");
    apply_bootstrap();
}
NATIVE_DIAG_POP

void NativeRideRuntime::apply_bootstrap() {
    mjModel *const model = stepper_->model();
    mjData *const data = stepper_->data();
    stepper_->mutate([&] {
        engine::reset_data(model, data);
        const std::span<double> frictionloss =
            std::views::counted(model->dof_frictionloss, model->nv);
        for (const FrictionLossEntry &e :
             bootstrap_.model_mutable.dof_frictionloss)
            frictionloss[static_cast<std::size_t>(e.dof_adr)] = e.value;
        const std::span<double> site_pos =
            std::views::counted(model->site_pos, model->nsite * 3);
        for (const SitePosEntry &e : bootstrap_.model_mutable.site_pos)
            for (std::size_t i = 0; i < 3; ++i)
                site_pos[static_cast<std::size_t>(e.site_id) * 3U + i] =
                    e.value.at(i);
        // mj_setConst derives rest-state constants from qpos0 — it rewrites
        // qpos — so the captured integration vector restores strictly after
        // it, and the writer snapshots (which never touch live mjData) can
        // sit on either side.
        engine::set_const(model, data);
        engine::set_state(model, data, bootstrap_.integration_state.data(),
                          static_cast<int>(mjSTATE_INTEGRATION));
        stepper_->drive().restore(bootstrap_.drive);
        if (bootstrap_.rider_contacts.has_value()) {
            // assign_state moves — hand it a copy so the stored bootstrap
            // stays replayable across reset().
            auto [contact_state, probe] = *bootstrap_.rider_contacts;
            stepper_->rider_contacts().assign_state(std::move(contact_state),
                                                    std::move(probe));
        }
        stepper_->set_tire_state(bootstrap_.tire.names,
                                 bootstrap_.tire.row);
        // The runtime counters land last (the docstring's ordered restore
        // ends with them): step/generation, clocks, filters, contacts,
        // energy and the controller/intent state faces.
        physical_->restore(bootstrap_step_state_);
    });
}

std::optional<std::string> NativeRideRuntime::outcome_reason() const {
    return crash_outcome(physical_->crash());
}

AdvanceResult
NativeRideRuntime::advance(std::int64_t target_step, nb::handle control,
                           double front_brake_demand,
                           double rear_brake_demand,
                           nb::handle wall_budget_s) {
    require_open();
    // ---- GIL held: decode + validate every Python input first ----
    // An argument rejection never mutates committed state.
    const RideControl command =
        rider::parse_intent_control(control, "control");
    if (!std::isfinite(front_brake_demand))
        throw std::invalid_argument("front_brake_demand must be finite");
    if (!std::isfinite(rear_brake_demand))
        throw std::invalid_argument("rear_brake_demand must be finite");
    std::optional<double> budget;
    if (!wall_budget_s.is_none()) {
        budget = wire::finite_real(wall_budget_s, "wall_budget_s");
        if (*budget < 0.)
            throw std::invalid_argument(
                "wall_budget_s must be nonnegative");
    }
    if (target_step < 0)
        throw std::invalid_argument("target_step must be nonnegative");
    // Reentrancy precedes every state-dependent check: while a concurrent
    // advance is in flight the committed step is a moving target, so
    // reading it (or advancing) must fail as reentrancy first.
    const AdvancementGuard guard(advancing_);
    if (target_step < physical_->step())
        throw std::invalid_argument(
            "target_step is behind the committed step");
    std::string reason;
    {
        // ---- GIL released: owned native advancement only ---------
        // The loop touches no Python objects: advance_physics' outputs
        // are pure-C++ Wire trees that stay inside the RawStep A4 owns.
        const nb::gil_scoped_release release;
        stepper_->refresh_time_callback_policy();
        const auto start = std::chrono::steady_clock::now();
        while (physical_->step() < target_step &&
               !physical_->crash().has_value()) {
            // `>=` keeps a zero budget a hard stop-before-first-step: two
            // back-to-back steady_clock reads can share a tick, so a strict
            // `>` could let one step slip through on budget=0.
            if (budget.has_value() &&
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start)
                        .count() >= *budget) {
                reason = "budget";
                break;
            }
            // The committed prefix survives a mid-range failure —
            // advance_physics increments only after a completed solve.
            (void)physical_->advance_physics(front_brake_demand,
                                             rear_brake_demand, std::nullopt,
                                             command);
        }
        if (reason.empty())
            reason = physical_->crash().has_value() ? "outcome" : "target";
    }
    // ---- GIL held: build the immutable result --------------------
    return {.step = physical_->step(),
            .time_s = stepper_->data()->time,
            .reason = std::move(reason),
            .outcome = outcome_reason()};
}

nb::dict
NativeRideRuntime::probe_step_inputs(nb::handle control,
                                     double front_brake_demand,
                                     double rear_brake_demand) {
    require_open();
    const RideControl command =
        rider::parse_intent_control(control, "control");
    if (!std::isfinite(front_brake_demand))
        throw std::invalid_argument("front_brake_demand must be finite");
    if (!std::isfinite(rear_brake_demand))
        throw std::invalid_argument("rear_brake_demand must be finite");
    const AdvancementGuard guard(advancing_);
    PhysicalStep::ProbeResult probe;
    {
        const nb::gil_scoped_release release;
        probe = physical_->probe_step_inputs(command, front_brake_demand,
                                             rear_brake_demand);
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
            ? nb::object(wire_object_to_python(probe.contact_probe))
            : nb::none();
    return out;
}

RuntimeSnapshot NativeRideRuntime::snapshot() const {
    require_open();
    const mjModel *const model = stepper_->model();
    const mjData *const data = stepper_->data();
    const mjtSize width =
        engine::state_size(model, static_cast<int>(mjSTATE_INTEGRATION));
    std::vector<double> values(static_cast<std::size_t>(width));
    engine::get_state(model, data, values.data(),
                      static_cast<int>(mjSTATE_INTEGRATION));
    return {.generation = physical_->generation(),
            .step = physical_->step(),
            .time_s = data->time,
            .integration_state = std::move(values),
            .outcome = outcome_reason()};
}

void NativeRideRuntime::reset() {
    require_open();
    const AdvancementGuard guard(advancing_);
    // PhysicalRuntime.reset() bumps generation monotonically; the stored
    // bootstrap's capture value belongs to the constructor only.
    const int next = physical_->generation() + 1;
    apply_bootstrap();
    physical_->set_generation(next);
}

void NativeRideRuntime::close() {
    physical_.reset();
    stepper_.reset();
    closed_ = true;
}

NativeRideRuntime::~NativeRideRuntime() = default;

namespace {

// nanobind's class builders carry heavy template frames; each binding
// group gets its own noinline function so the per-function stack budget
// holds on unoptimized builds.
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
        .def_prop_ro("outcome",
                     [](const RuntimeSnapshot &s) -> nb::object {
                         return s.outcome.has_value()
                                    ? nb::object(nb::str(s.outcome->c_str(),
                                                         s.outcome->size()))
                                    : nb::none();
                     });
}

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

void bind_runtime_class(const nb::module_ &module) {
    nb::class_<NativeRideRuntime>(module, "NativeRideRuntime")
        .def(nb::init<nb::handle, nb::handle, nb::handle>(),
             nb::arg("model_path"), nb::arg("config"), nb::arg("state"))
        .def("advance", &NativeRideRuntime::advance,
             nb::arg("target_step"), nb::arg("control"),
             nb::arg("front_brake_demand") = 0.,
             nb::arg("rear_brake_demand") = 0.,
             nb::arg("wall_budget_s") = nb::none())
        .def("probe_step_inputs", &NativeRideRuntime::probe_step_inputs,
             nb::arg("control"), nb::arg("front_brake_demand") = 0.,
             nb::arg("rear_brake_demand") = 0.)
        .def("snapshot", &NativeRideRuntime::snapshot)
        .def("reset", &NativeRideRuntime::reset)
        .def("close", &NativeRideRuntime::close)
        .def_prop_ro("closed", &NativeRideRuntime::closed);
}

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
    bind_snapshot_class(module);
    bind_result_class(module);
    bind_runtime_class(module);
}

} // namespace runtime
