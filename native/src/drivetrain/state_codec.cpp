// drivetrain/state_codec.cpp — the Python-free half of the drive-state
// wire (task T08). Decode walks the T05-owned bike::io::Value tree through
// the shared node protocol (nativecfg::detail): the VNode below is the
// Value-side twin of the nanobind adapter's node, so every scalar read
// throws std::invalid_argument spelling the wire:: readers' verbatim
// reasons — a Value document and its nanobind-dict twin decode through
// identical checks. See the header for the schema notes.
//
// Emission re-spells the typed snapshot as the wire document the binding's
// state_dict() produces: integer lanes keep integer lexemes, real lanes
// keep float spelling (whole-valued doubles emit "x.0" so the lexeme stays
// a float, and -0.0 keeps its sign), closed labels emit their names and
// explicit-None lanes emit null — presence state survives the round-trip.
//
// The codec is staging-only by construction: every rejection happens
// before a live object exists on this side of the boundary.
#include "state_codec.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <variant>
#include <vector>

#include "../validation.hpp"
#include "../writer_config.hpp"
#include "transmission.hpp"

namespace drivetrain {
namespace {

    namespace bio = bike::io;
    namespace ncd = nativecfg::detail;

    // ---- the Value-side decode node ----------------------------------------
    // Twin of writer_config.cpp's TU-local VNode — the node protocol is by
    // shape (nativecfg::ncd::Dict<N> drives it), so the copy stays local
    // to this TU and the codecs share only the generic readers.
    namespace vnode {
        [[nodiscard]] inline const bio::Value::Object &object(
            const bio::Value &v) {
            return *std::get_if<bio::Value::Object>(&v.data);
        }
        [[nodiscard]] inline const bio::Value::Array &array(
            const bio::Value &v) {
            return *std::get_if<bio::Value::Array>(&v.data);
        }
        [[nodiscard]] inline const bio::Value &missing() {
            static const bio::Value placeholder{};
            return placeholder;
        }
    } // namespace vnode

    struct VNode {
        const bio::Value *value;

        [[nodiscard]] bool is_none() const {
            return bio::is_null(*value);
        }
        [[nodiscard]] bool is_mapping() const {
            return std::holds_alternative<bio::Value::Object>(value->data);
        }
        [[nodiscard]] bool contains(std::string_view key) const {
            return bio::find_member(vnode::object(*value), key) != nullptr;
        }
        [[nodiscard]] VNode member(std::string_view key) const {
            const bio::Value *const found =
                bio::find_member(vnode::object(*value), key);
            return {found != nullptr ? found : &vnode::missing()};
        }
        // Object keys are strings by construction — the nb side's
        // non-string-key check has no analog.
        template <class F>
        void each_key(std::string_view /*path*/, const F &f) const {
            for (const auto &member: vnode::object(*value))
                f(member.first);
        }
        template <class F>
        void each_member(const F &f) const {
            for (const auto &member: vnode::object(*value))
                f(member.first, VNode{&member.second});
        }
        [[nodiscard]] bool boolean(std::string_view path) const {
            const auto *const flag = std::get_if<bool>(&value->data);
            if (flag == nullptr) ncd::invalid(path, "expected bool");
            return *flag;
        }
        [[nodiscard]] double real(std::string_view path) const {
            const auto *const number =
                std::get_if<bio::Number>(&value->data);
            if (number != nullptr) {
                bool ok = false;
                const double result =
                    bio::detail::float_from_lexeme(number->lexeme, ok);
                if (ok) return result;
                ncd::invalid(path, "expected finite real scalar");
            }
            // The resolved {"$nonfinite": ...} capture tag — real() admits
            // the sentinel exactly like the binding's wire::real passes a
            // nonfinite Python float to the finite-only field readers.
            if (const auto *const tagged =
                    std::get_if<bio::Tagged>(&value->data)) {
                if (tagged->kind == "float64" && tagged->fields.size() == 1 &&
                    tagged->fields.front().first == "value") {
                    const std::string &spelling = tagged->fields.front().second;
                    if (spelling == "nan")
                        return std::numeric_limits<double>::quiet_NaN();
                    if (spelling == "+inf" || spelling == "inf")
                        return std::numeric_limits<double>::infinity();
                    if (spelling == "-inf")
                        return -std::numeric_limits<double>::infinity();
                }
            }
            ncd::invalid(path, "expected finite real scalar");
        }
        [[nodiscard]] double finite_real(std::string_view path) const {
            const double result = real(path);
            if (!std::isfinite(result))
                ncd::invalid(path, "expected finite real scalar");
            return result;
        }
        [[nodiscard]] std::optional<double> optional_real(
            std::string_view path) const {
            return is_none() ? std::nullopt
                             : std::optional(finite_real(path));
        }
        [[nodiscard]] std::int64_t integer(std::string_view path) const {
            const auto *const number =
                std::get_if<bio::Number>(&value->data);
            if (number == nullptr ||
                !bio::is_integer_lexeme(number->lexeme))
                ncd::invalid(path, "expected integer");
            std::int64_t result = 0;
            const char *const end =
                std::to_address(number->lexeme.cend());
            const auto parsed =
                std::from_chars(number->lexeme.data(), end, result, 10);
            if (parsed.ec != std::errc() || parsed.ptr != end)
                ncd::invalid(path, "invalid value or out of range");
            return result;
        }
        [[nodiscard]] int integer32(std::string_view path) const {
            const std::int64_t result = integer(path);
            if (result < std::numeric_limits<int>::min() ||
                result > std::numeric_limits<int>::max())
                ncd::invalid(path, "integer out of range");
            return static_cast<int>(result);
        }
        [[nodiscard]] std::string string(std::string_view path) const {
            const auto *const text = std::get_if<std::string>(&value->data);
            if (text == nullptr) ncd::invalid(path, "expected string");
            return *text;
        }
        [[nodiscard]] ncd::Seq<VNode> sequence(
            std::string_view path) const {
            if (!std::holds_alternative<bio::Value::Array>(value->data))
                ncd::invalid(path, "expected ordered sequence");
            return {*this};
        }
        [[nodiscard]] std::size_t seq_size() const {
            return vnode::array(*value).size();
        }
        [[nodiscard]] VNode seq_at(std::size_t index) const {
            return {&vnode::array(*value)[index]};
        }
        // Value has no ndarray-style boolean-array pitfall — the per-
        // element finite_real gate covers mixed-type arrays anyway.
        void numeric_gate(std::string_view /*path*/) const noexcept {}
    };

    using Dict = ncd::Dict<VNode>;
    using ncd::exact;
    using ncd::field;
    using ncd::section;
    using ncd::vector;

    // wire::mapping on a bare node with an explicit public path (the
    // binding's "expected dict" family).
    [[nodiscard]] Dict mapping(const VNode &v, std::string_view path) {
        if (!v.is_mapping()) ncd::invalid(path, "expected dict");
        return {.value = v, .path = std::string(path)};
    }

    [[nodiscard]] double num(const Dict &d, std::string_view key) {
        return ncd::num(d, key);
    }
    [[nodiscard]] bool boolean(const Dict &d, std::string_view key) {
        return ncd::boolean(d, key);
    }
    [[nodiscard]] int integer(const Dict &d, std::string_view key) {
        return ncd::integer(d, key);
    }
    [[nodiscard]] std::string string(const Dict &d, std::string_view key) {
        return ncd::str(d, key);
    }
    [[nodiscard]] std::optional<double> optional_num(
        const Dict &d, std::string_view key) {
        return ncd::optional_num(d, key);
    }

    // ---- decode -------------------------------------------------------------

    // Schema-driven twin of the binding's parse_telemetry: kind, integer
    // floor and closed-label domains all come from telemetry_spec, so the
    // two sides can never disagree about a lane. Unknown keys reject with
    // the same "unknown diagnostic key" family.
    [[nodiscard]] DriveTelemetry decode_telemetry(const Dict &d) {
        DriveTelemetry out;
        d.value.each_member([&](std::string_view name, const VNode &v) {
            const std::string key(name);
            const std::optional<TelemetryField> field =
                telemetry_field(key);
            if (!field.has_value())
                ncd::invalid(d.child(key), "unknown diagnostic key");
            const TelemetrySpec &spec = telemetry_spec(*field);
            const std::string path = d.child(key);
            switch (spec.kind) {
            case TelemetryKind::flag:
                out.set(*field, v.boolean(path));
                break;
            case TelemetryKind::integer: {
                const int value = v.integer32(path);
                if (value < spec.integer_min)
                    throw std::invalid_argument("diagnostic integer range");
                out.set(*field, static_cast<std::int32_t>(value));
                break;
            }
            case TelemetryKind::label: {
                const auto value = v.string(path);
                const std::optional<std::uint8_t> ordinal =
                    telemetry_label(*field, value);
                if (!ordinal.has_value())
                    throw std::invalid_argument("diagnostic enum");
                out.set_label(*field, *ordinal);
                break;
            }
            case TelemetryKind::open:
                out.set_open(*field, v.string(path));
                break;
            case TelemetryKind::optional:
                if (v.is_none()) {
                    out.set_none(*field);
                    break;
                }
                [[fallthrough]];
            case TelemetryKind::real: {
                const double value = v.finite_real(path);
                if (key == "gear_ratio")
                    validation::positive(value, "diagnostic gear ratio");
                out.set(*field, value);
                break;
            }
            }
        });
        return out;
    }

    void decode_policies(const Dict &d, DriveSnapshot &out) {
        exact(d, ncd::keys("pedaling", "shifting", "assist", "battery",
                              "hub"));
        const auto p = section(d, "pedaling"), s = section(d, "shifting"),
                   a = section(d, "assist"), b = section(d, "battery");
        exact(p, ncd::keys("coasting", "target_phase_rad",
                              "target_rate_rad_s", "deceleration_rad_s2",
                              "_effort", "_cadence_ema"));
        exact(s, ncd::keys("rear_teeth", "from_teeth", "cooldown_s",
                              "cut_remaining_s", "shift_count", "direction",
                              "cadence_ema", "required_ema"));
        exact(a, ncd::keys("torque", "pedaling", "last_gain"));
        exact(b, ncd::keys("initial_energy_j", "energy_j",
                              "drawn_energy_j"));
        out.pedaling = {
            .coasting = boolean(p, "coasting"),
            .target_phase_rad = optional_num(p, "target_phase_rad"),
            .target_rate_rad_s = num(p, "target_rate_rad_s"),
            .deceleration_rad_s2 = num(p, "deceleration_rad_s2"),
            .effort = num(p, "_effort"),
            .cadence_ema = optional_num(p, "_cadence_ema")};
        out.shifting = {
            .rear_teeth = integer(s, "rear_teeth"),
            .from_teeth = integer(s, "from_teeth"),
            .shift_count = integer(s, "shift_count"),
            .cooldown_s = num(s, "cooldown_s"),
            .cut_remaining_s = num(s, "cut_remaining_s"),
            .direction = string(s, "direction"),
            .cadence_ema = optional_num(s, "cadence_ema"),
            .required_ema = optional_num(s, "required_ema")};
        out.assist = {.torque = num(a, "torque"),
                      .last_gain = num(a, "last_gain"),
                      .pedaling = boolean(a, "pedaling")};
        out.battery = {.initial_energy_j = num(b, "initial_energy_j"),
                       .energy_j = num(b, "energy_j"),
                       .drawn_energy_j = num(b, "drawn_energy_j")};
        if (!field(d, "hub").is_none()) {
            const auto h = section(d, "hub");
            exact(h, ncd::keys("boundary", "energy_j", "torque_nm"));
            out.hub = FreehubSnapshot{
                .boundary = optional_num(h, "boundary"),
                .energy_j = num(h, "energy_j"),
                .torque_nm = num(h, "torque_nm")};
        }
    }

    [[nodiscard]] std::optional<TransmissionSnapshot> decode_transmission(
        const VNode &input, const std::string &path) {
        if (input.is_none()) return std::nullopt;
        if (!input.is_mapping())
            throw std::invalid_argument("transmission state");
        const Dict d{.value = input, .path = path};
        exact(d, ncd::keys("ratio", "rear_teeth", "boundary",
                              "diagnostics", "shift_pending",
                              "shift_parameter_work_j",
                              "shift_constraint_work_j", "last_tension_n",
                              "range", "coefficients", "prepared"));
        TransmissionSnapshot s;
        s.ratio = num(d, "ratio");
        s.rear_teeth = integer(d, "rear_teeth");
        s.boundary = optional_num(d, "boundary");
        s.diagnostics = decode_telemetry(section(d, "diagnostics"));
        s.shift_pending = boolean(d, "shift_pending");
        s.shift_parameter_work_j = num(d, "shift_parameter_work_j");
        s.shift_constraint_work_j = num(d, "shift_constraint_work_j");
        s.last_tension_n = num(d, "last_tension_n");
        const auto r = vector(field(d, "range"), d.child("range"));
        if (r.size() != 2) throw std::invalid_argument("range width");
        s.range = {r[0], r[1]};
        s.coefficients = vector(field(d, "coefficients"),
                                d.child("coefficients"));
        if (!field(d, "prepared").is_none()) {
            const auto p = section(d, "prepared");
            exact(p, ncd::keys("phi", "time", "jacobian", "qpos"));
            s.prepared = PreparedTransmission{
                .phi = num(p, "phi"), .time = num(p, "time"),
                .jacobian = vector(field(p, "jacobian"),
                                   p.child("jacobian")),
                .qpos = vector(field(p, "qpos"), p.child("qpos"))};
        }
        return s;
    }

    // ---- encode -------------------------------------------------------------

    // Shortest-round-trip float spelling with the float marker preserved:
    // whole-valued and zero magnitudes gain ".0" so the lexeme stays a
    // JSON float ("0" would spell an integer) and -0.0 keeps its sign.
    [[nodiscard]] bio::Value real_value(double v) {
        std::array<char, 32> digits{};
        const auto converted =
            std::to_chars(digits.data(), std::to_address(digits.end()), v);
        std::string lexeme(digits.data(), converted.ptr);
        if (std::isfinite(v) &&
            lexeme.find_first_of(".eE") == std::string::npos)
            lexeme += ".0";
        return bio::Value{bio::Number{std::move(lexeme)}};
    }
    [[nodiscard]] bio::Value integer_value(std::int64_t v) {
        return bio::Value{bio::Number{std::to_string(v)}};
    }
    [[nodiscard]] bio::Value optional_value(std::optional<double> v) {
        return v.has_value() ? real_value(*v) : bio::Value{};
    }
    [[nodiscard]] bio::Value vector_value(std::span<const double> xs) {
        bio::Value::Array out;
        out.reserve(xs.size());
        for (const double x: xs) out.push_back(real_value(x));
        return bio::Value{std::move(out)};
    }
    // Typed-telemetry twin of the binding's diagnostic_dict — schema order
    // (the map's sorted sequence), presence 2 spelling null, labels as
    // their declared names.
    [[nodiscard]] bio::Value telemetry_value(const DriveTelemetry &values) {
        bio::Value::Object out;
        values.for_each([&](TelemetryField field, const TelemetryScalar &v) {
            const TelemetrySpec &spec = telemetry_spec(field);
            bio::Value member;
            std::visit(
                [&](const auto &x) {
                    using T = std::decay_t<decltype(x)>;
                    if constexpr (std::is_same_v<T, std::monostate>)
                        member = bio::Value{};
                    else if constexpr (std::is_same_v<T, std::int32_t>)
                        member = integer_value(x);
                    else if constexpr (std::is_same_v<T, std::uint8_t>)
                        member = bio::Value{std::string(
                            telemetry_label_name(field, x))};
                    else if constexpr (std::is_same_v<T, std::string_view>)
                        member = bio::Value{std::string(x)};
                    else if constexpr (std::is_same_v<T, bool>)
                        member = bio::Value{x};
                    else
                        member = real_value(x);
                },
                v);
            out.emplace_back(std::string(spec.name), std::move(member));
        });
        return bio::Value{std::move(out)};
    }

    [[nodiscard]] bio::Value policy_value(const DriveSnapshot &s) {
        bio::Value::Object p, sh, a, b;
        p.emplace_back("coasting", bio::Value{s.pedaling.coasting});
        p.emplace_back("target_phase_rad",
                       optional_value(s.pedaling.target_phase_rad));
        p.emplace_back("target_rate_rad_s",
                       real_value(s.pedaling.target_rate_rad_s));
        p.emplace_back("deceleration_rad_s2",
                       real_value(s.pedaling.deceleration_rad_s2));
        p.emplace_back("_effort", real_value(s.pedaling.effort));
        p.emplace_back("_cadence_ema",
                       optional_value(s.pedaling.cadence_ema));
        sh.emplace_back("rear_teeth", integer_value(s.shifting.rear_teeth));
        sh.emplace_back("from_teeth",
                        integer_value(s.shifting.from_teeth));
        sh.emplace_back("shift_count",
                        integer_value(s.shifting.shift_count));
        sh.emplace_back("cooldown_s", real_value(s.shifting.cooldown_s));
        sh.emplace_back("cut_remaining_s",
                        real_value(s.shifting.cut_remaining_s));
        sh.emplace_back("direction", bio::Value{s.shifting.direction});
        sh.emplace_back("cadence_ema",
                        optional_value(s.shifting.cadence_ema));
        sh.emplace_back("required_ema",
                        optional_value(s.shifting.required_ema));
        a.emplace_back("torque", real_value(s.assist.torque));
        a.emplace_back("pedaling", bio::Value{s.assist.pedaling});
        a.emplace_back("last_gain", real_value(s.assist.last_gain));
        b.emplace_back("initial_energy_j",
                       real_value(s.battery.initial_energy_j));
        b.emplace_back("energy_j", real_value(s.battery.energy_j));
        b.emplace_back("drawn_energy_j",
                       real_value(s.battery.drawn_energy_j));
        bio::Value::Object out;
        out.emplace_back("pedaling", bio::Value{std::move(p)});
        out.emplace_back("shifting", bio::Value{std::move(sh)});
        out.emplace_back("assist", bio::Value{std::move(a)});
        out.emplace_back("battery", bio::Value{std::move(b)});
        if (s.hub.has_value()) {
            bio::Value::Object h;
            h.emplace_back("boundary", optional_value(s.hub->boundary));
            h.emplace_back("energy_j", real_value(s.hub->energy_j));
            h.emplace_back("torque_nm", real_value(s.hub->torque_nm));
            out.emplace_back("hub", bio::Value{std::move(h)});
        } else {
            out.emplace_back("hub", bio::Value{});
        }
        return bio::Value{std::move(out)};
    }

    [[nodiscard]] bio::Value transmission_value(
        const std::optional<TransmissionSnapshot> &maybe) {
        if (!maybe.has_value()) return bio::Value{};
        const TransmissionSnapshot &s = *maybe;
        bio::Value::Object out;
        out.emplace_back("ratio", real_value(s.ratio));
        out.emplace_back("rear_teeth", integer_value(s.rear_teeth));
        out.emplace_back("boundary", optional_value(s.boundary));
        out.emplace_back("diagnostics", telemetry_value(s.diagnostics));
        out.emplace_back("shift_pending", bio::Value{s.shift_pending});
        out.emplace_back("shift_parameter_work_j",
                         real_value(s.shift_parameter_work_j));
        out.emplace_back("shift_constraint_work_j",
                         real_value(s.shift_constraint_work_j));
        out.emplace_back("last_tension_n",
                         real_value(s.last_tension_n));
        out.emplace_back("range", vector_value(s.range));
        out.emplace_back("coefficients", vector_value(s.coefficients));
        if (s.prepared.has_value()) {
            bio::Value::Object p;
            p.emplace_back("phi", real_value(s.prepared->phi));
            p.emplace_back("time", real_value(s.prepared->time));
            p.emplace_back("jacobian", vector_value(s.prepared->jacobian));
            p.emplace_back("qpos", vector_value(s.prepared->qpos));
            out.emplace_back("prepared", bio::Value{std::move(p)});
        } else {
            out.emplace_back("prepared", bio::Value{});
        }
        return bio::Value{std::move(out)};
    }

} // namespace

DriveSnapshot decode_drive_snapshot(const bio::Value &value) {
    const Dict d = mapping(VNode{&value}, "state.drive");
    exact(d, ncd::keys("policies", "shift_time_s", "last_time_s",
                          "reference", "psi", "angles", "last", "probe_last",
                          "pending_actuation", "ideal_hub", "clutch",
                          "freewheel"));
    DriveSnapshot s;
    decode_policies(section(d, "policies"), s);
    s.shift_time_s = optional_num(d, "shift_time_s");
    s.last_time_s = optional_num(d, "last_time_s");
    s.reference = optional_num(d, "reference");
    s.psi = optional_num(d, "psi");
    if (!field(d, "angles").is_none()) {
        const auto a = vector(field(d, "angles"), d.child("angles"));
        DriveAngles angles{.count = a.size()};
        if (a.size() > angles.values.size())
            throw std::invalid_argument("drive angle width");
        std::ranges::copy(a, angles.values.begin());
        s.angles = angles;
    }
    s.last = decode_telemetry(section(d, "last"));
    if (!field(d, "probe_last").is_none())
        s.probe_last = decode_telemetry(section(d, "probe_last"));
    if (!field(d, "pending_actuation").is_none()) {
        const auto p = section(d, "pending_actuation");
        exact(p, ncd::keys("requested", "omega", "dt", "enabled"));
        s.pending_actuation = PendingActuation{
            .requested = num(p, "requested"), .omega = num(p, "omega"),
            .dt = num(p, "dt"), .enabled = boolean(p, "enabled")};
        if (s.pending_actuation->dt <= 0.)
            ncd::invalid(p.child("dt"), "dt must be positive");
    }
    s.ideal_hub = decode_transmission(field(d, "ideal_hub"),
                                      d.child("ideal_hub"));
    s.clutch = decode_transmission(field(d, "clutch"), d.child("clutch"));
    s.freewheel = decode_transmission(field(d, "freewheel"),
                                      d.child("freewheel"));
    return s;
}

bio::Value encode_drive_snapshot(const DriveSnapshot &s) {
    bio::Value::Object out;
    out.emplace_back("policies", policy_value(s));
    out.emplace_back("shift_time_s", optional_value(s.shift_time_s));
    out.emplace_back("last_time_s", optional_value(s.last_time_s));
    out.emplace_back("reference", optional_value(s.reference));
    out.emplace_back("psi", optional_value(s.psi));
    // The fixed-width mirror expands back into the wire's variable list —
    // 1 entry for the simplified models, 2 for elastic_chain.
    if (s.angles.has_value()) {
        const auto values =
            std::span<const double>(s.angles->values).first(s.angles->count);
        out.emplace_back("angles", vector_value(values));
    } else {
        out.emplace_back("angles", bio::Value{});
    }
    out.emplace_back("last", telemetry_value(s.last));
    out.emplace_back("probe_last",
                     s.probe_last.has_value()
                         ? telemetry_value(*s.probe_last)
                         : bio::Value{});
    if (s.pending_actuation.has_value()) {
        bio::Value::Object p;
        p.emplace_back("requested",
                       real_value(s.pending_actuation->requested));
        p.emplace_back("omega", real_value(s.pending_actuation->omega));
        p.emplace_back("dt", real_value(s.pending_actuation->dt));
        p.emplace_back("enabled",
                       bio::Value{s.pending_actuation->enabled});
        out.emplace_back("pending_actuation", bio::Value{std::move(p)});
    } else {
        out.emplace_back("pending_actuation", bio::Value{});
    }
    out.emplace_back("ideal_hub", transmission_value(s.ideal_hub));
    out.emplace_back("clutch", transmission_value(s.clutch));
    out.emplace_back("freewheel", transmission_value(s.freewheel));
    return bio::Value{std::move(out)};
}

} // namespace drivetrain
