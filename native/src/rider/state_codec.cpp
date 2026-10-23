// rider/state_codec.cpp — the Python-free halves of the rider state
// wires (task T08). Every decoder is the bike::io::Value twin of the
// nanobind reader it names in spindle_wire.cpp / intent_wire.cpp /
// attachment_binding.cpp / rider_contact_binding.cpp — same key tables,
// same validation order, the same std::invalid_argument family spelling
// the wire:: readers' verbatim reasons under the same public paths.
//
// Reads run through the shared node protocol (nativecfg::detail): the
// VNode below is the Value-side node (a TU-local twin of the one in
// writer_config.cpp — the protocol is by shape). Type gates on the Value
// tree carry the Python semantics the wire contract needs:
//
//   * a Number member accepts only float()/int()-shaped lexemes — a bool
//     member is not numeric and a float-spelled lexeme ("2.0") is not an
//     integer, mirroring PyLong_CheckExact / numbers.Integral exactly;
//   * 'truthy' reproduces PyObject_IsTrue over the JSON-admissible kinds
//     (null->false, bools, zero/nonzero numbers, empty/nonempty strings
//     and containers) for the attachment records' observable/rotational
//     fields, which the oracle never type-checked beyond truthiness;
//   * dense-matrix members take the nested-row spelling ([[...], ...]) —
//     the JSON analog of the binding's 2-D float64 ndarray contract, with
//     a ragged or non-numeric member rejecting as "must be a 2-D float64
//     array".
//
// Emission mirrors the binding's dict-literal update order so canonical
// JSON output is shaped like the oracle's: float lexemes keep their
// spelling (whole-valued doubles emit "x.0"; -0.0 keeps its sign),
// optionals emit null, and tuples/lists both emit ordered arrays.
//
// Every decoder is staging-only: a throw anywhere abandons a candidate
// struct — nothing here touches a live writer, so rejection is atomic.
#include "state_codec.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include "../diag.hpp"
#include "../validation.hpp"
#include "../writer_config.hpp"
#include "spindle_math.hpp"
#include "../configuration/load.hpp"
#include "response_codec.hpp"

namespace rider {
namespace {

    namespace bio = bike::io;
    namespace ncd = nativecfg::detail;
    namespace wc = writers;

    using ncd::keys;

    // ---- the Value-side decode node ---------------------------------------
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
        void numeric_gate(std::string_view /*path*/) const noexcept {}
    };

    using Dict = ncd::Dict<VNode>;

    [[noreturn]] void invalid(std::string_view path,
                              std::string_view reason) {
        ncd::invalid(path, reason);
    }

    [[nodiscard]] Dict mapping(const VNode &v, std::string_view path) {
        if (!v.is_mapping()) invalid(path, "expected dict");
        return {.value = v, .path = std::string(path)};
    }

    void exact_keys(const VNode &v,
                    std::span<const std::string_view> required,
                    std::span<const std::string_view> optional,
                    std::string_view path) {
        const Dict d = mapping(v, path);
        ncd::exact_at(d, required, optional, path);
    }

    [[nodiscard]] std::vector<double> vector(const VNode &v,
                                             std::string_view path) {
        return ncd::vector(v, path);
    }

    template <std::size_t M>
    [[nodiscard]] std::array<double, M> fixed(const VNode &v,
                                              std::string_view path) {
        return ncd::fixed<M>(v, path);
    }

    [[nodiscard]] VNode field(const Dict &d, std::string_view key) {
        return ncd::field(d, key);
    }

    [[nodiscard]] Dict section(const Dict &d, std::string_view key) {
        return ncd::section(d, key);
    }

    // PyObject_IsTrue over the JSON-admissible kinds — the attachment
    // records' truthy fields (observable/rotational) were never
    // type-checked beyond this in the oracle.
    [[nodiscard]] bool truthy(const VNode &v) {
        const bio::Value &value = *v.value;
        if (const auto *const flag = std::get_if<bool>(&value.data))
            return *flag;
        if (const auto *const number =
                std::get_if<bio::Number>(&value.data)) {
            bool ok = false;
            const double parsed =
                bio::detail::float_from_lexeme(number->lexeme, ok);
            return ok && parsed != 0.;
        }
        if (const auto *const text = std::get_if<std::string>(&value.data))
            return !text->empty();
        if (const auto *const arr =
                std::get_if<bio::Value::Array>(&value.data))
            return !arr->empty();
        if (const auto *const obj =
                std::get_if<bio::Value::Object>(&value.data))
            return !obj->empty();
        return false;
    }

    // The VNode bodies the produced decode_named_reals/decode_joint_terms
    // delegate to — internal sections read members without re-boxing.
    [[nodiscard]] NamedEntries<double> decode_named_reals_node(
        const VNode &v, std::string_view path) {
        NamedEntries<double> out;
        const Dict d = mapping(v, path);
        d.value.each_member([&](std::string_view name, const VNode &item) {
            out.emplace_back(
                std::string(name),
                item.finite_real(std::string(path) + "." +
                                 std::string(name)));
        });
        return out;
    }

    [[nodiscard]] JointTerms decode_joint_terms_node(
        const VNode &v, std::string_view path) {
        const Dict d = mapping(v, path);
        ncd::exact(d,
                   keys("command_nm", "posture_nm",
                        "pedaling_nm", "saturated"),
                   keys("requested_nm", "active_request_nm", "active_delivered_nm",
                        "passive_damping_nm", "solved_force_nm",
                        "solved_active_nm", "solved_passive_nm", "support_nm", "tracking_nm", "hip_posture_nm"));
        JointTerms out;
        out.requested_present = d.contains("requested_nm");
        if (out.requested_present) out.requested_nm = d["requested_nm"].finite_real(path);
        out.command_nm = d["command_nm"].finite_real(path);
        out.posture_nm = d["posture_nm"].finite_real(path);
        out.pedaling_nm = d["pedaling_nm"].finite_real(path);
        out.saturated = d["saturated"].boolean(path);
        const auto optional = [&](std::string_view key) {
            if (!d.contains(key) || d[key].is_none())
                return std::optional<double>{};
            return std::optional<double>(d[key].finite_real(path));
        };
        out.active_request_nm = optional("active_request_nm");
        out.active_delivered_nm = optional("active_delivered_nm");
        out.passive_damping_nm = optional("passive_damping_nm");
        out.solved_force_nm = optional("solved_force_nm");
        out.solved_active_nm = optional("solved_active_nm");
        out.solved_passive_nm = optional("solved_passive_nm");
        out.support_nm = optional("support_nm");
        out.tracking_nm = optional("tracking_nm");
        out.hip_posture_nm = optional("hip_posture_nm");
        return out;
    }

    // ---- emission ----------------------------------------------------------

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
    [[nodiscard]] bio::Value index_vector_value(
        std::span<const std::int64_t> xs) {
        bio::Value::Array out;
        out.reserve(xs.size());
        for (const std::int64_t x: xs) out.push_back(integer_value(x));
        return bio::Value{std::move(out)};
    }
    // The 2-D float64 ndarray's JSON spelling — an array of row arrays.
    [[nodiscard]] bio::Value matrix_value(const DenseMatrix &m) {
        bio::Value::Array out;
        out.reserve(m.rows);
        for (std::size_t row = 0; row < m.rows; ++row) {
            bio::Value::Array entries;
            entries.reserve(m.columns);
            for (std::size_t column = 0; column < m.columns; ++column)
                entries.push_back(
                    real_value(m.values[row * m.columns + column]));
            out.push_back(bio::Value{std::move(entries)});
        }
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value optional_vector_value(
        const std::optional<std::vector<double>> &v) {
        return v.has_value() ? vector_value(*v) : bio::Value{};
    }
    [[nodiscard]] bio::Value named_reals_value(
        const NamedEntries<double> &entries) {
        bio::Value::Object out;
        for (const auto &[name, value]: entries)
            out.emplace_back(name, real_value(value));
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value named_bools_value(
        const NamedEntries<bool> &entries) {
        bio::Value::Object out;
        for (const auto &[name, value]: entries)
            out.emplace_back(name, bio::Value{value});
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value names_value(
        const std::vector<std::string> &names) {
        bio::Value::Array out;
        out.reserve(names.size());
        for (const auto &name: names) out.push_back(bio::Value{name});
        return bio::Value{std::move(out)};
    }
    void set_or_none(bio::Value::Object &out, std::string_view key,
                     const std::optional<double> &value) {
        out.emplace_back(std::string(key), optional_value(value));
    }

    // ---- spindle controller (spindle_wire.cpp twins) -----------------------

    [[nodiscard]] std::array<double, 3> point3(const VNode &v,
                                               std::string_view path) {
        return fixed<3>(v, path);
    }

    [[nodiscard]] spindle::TorqueCurve decode_curve(
        const VNode &v, std::string_view path) {
        const Dict d = mapping(v, path);
        ncd::exact(d,
                      keys("angles_rad", "torques_nm", "vmax_rad_s",
                           "hill_c", "eccentric_ratio", "source"));
        spindle::TorqueCurve curve{
            .angles_rad = vector(d["angles_rad"], path),
            .torques_nm = vector(d["torques_nm"], path),
            .vmax_rad_s = d["vmax_rad_s"].finite_real(path),
            .hill_c = d["hill_c"].finite_real(path),
            .eccentric_ratio = d["eccentric_ratio"].finite_real(path),
            .source = d["source"].string(path),
        };
        spindle::validate_torque_curve(curve, "strength curve");
        return curve;
    }

    [[nodiscard]] spindle::JointEnvelope decode_envelope(
        const VNode &v, std::string_view path) {
        const Dict d = mapping(v, path);
        ncd::exact(d,
                      keys("neutral_anatomical_rad", "direction",
                           "minimum_anatomical_rad",
                           "maximum_anatomical_rad", "provenance"));
        spindle::JointEnvelope envelope{
            .neutral_anatomical_rad =
                d["neutral_anatomical_rad"].finite_real(path),
            .direction = d["direction"].integer32(path),
            .minimum_anatomical_rad =
                d["minimum_anatomical_rad"].finite_real(path),
            .maximum_anatomical_rad =
                d["maximum_anatomical_rad"].finite_real(path),
            .provenance = d["provenance"].string(path),
        };
        spindle::validate_joint_envelope(envelope, "joint envelope");
        return envelope;
    }

    [[nodiscard]] std::map<std::string, spindle::JointEnvelope>
    decode_envelope_map(const VNode &v, std::string_view path) {
        std::map<std::string, spindle::JointEnvelope> out;
        if (v.is_none()) return out;
        const Dict d = mapping(v, path);
        d.value.each_member([&](std::string_view name, const VNode &item) {
            const auto sub = std::string(path) + "." + std::string(name);
            out.emplace(std::string(name), decode_envelope(item, sub));
        });
        return out;
    }

    [[nodiscard]] NamedEntries<bool>
    decode_named_bools(const VNode &v, std::string_view path,
                       std::span<const std::string_view> allowed) {
        NamedEntries<bool> out;
        const Dict d = mapping(v, path);
        d.value.each_member([&](std::string_view name, const VNode &item) {
            if (std::ranges::find(allowed, name) == allowed.end())
                invalid(std::string(path) + "." + std::string(name),
                        "unknown key");
            out.emplace_back(
                std::string(name),
                item.boolean(std::string(path) + "." + std::string(name)));
        });
        return out;
    }

    [[nodiscard]] std::vector<std::string>
    decode_name_list(const VNode &v, std::string_view path) {
        const auto items = v.sequence(path);
        std::vector<std::string> out;
        out.reserve(items.size());
        for (std::size_t i = 0; i < items.size(); ++i)
            out.push_back(items.at(i).string(path));
        return out;
    }

    [[nodiscard]] PedalRecoveryState decode_pedal_recovery(
        const VNode &v, std::string_view path) {
        const Dict d = mapping(v, path);
        ncd::exact(d, keys("stage", "direction", "release_offset_x_m"));
        PedalRecoveryState out;
        out.stage = d["stage"].string(path);
        if (out.stage != "none" && out.stage != "release" &&
            out.stage != "escape" && out.stage != "raise" &&
            out.stage != "return")
            invalid(path, "unknown pedal recovery stage");
        out.direction = d["direction"].finite_real(path);
        out.release_offset_x_m = d["release_offset_x_m"].finite_real(path);
        return out;
    }

    constexpr auto kEffortRequiredKeys = keys(
        "rider_active_request_nm", "rider_active_delivered_nm",
        "rider_positive_power_w", "rider_passive_power_w",
        "rider_activation_saturated", "rider_strength_limited");
    constexpr auto kEffortOptionalKeys = keys(
        "rider_effort_budget_exceeded", "rider_active_positive_power_limit_w", "rider_effort_observation",
        "rider_joint_positive_power_w", "rider_joint_power_violations",
        "rider_joint_speed_violations", "rider_positive_work_step_j",
        "rider_passive_work_step_j", "rider_strength_violations");

    void decode_effort_solved_fields(const Dict &d, std::string_view path,
                                     EffortDiagnostics &out) {
        if (d.contains("rider_joint_positive_power_w"))
            out.joint_positive_power_w = decode_named_reals_node(
                d["rider_joint_positive_power_w"], path);
        if (d.contains("rider_joint_power_violations"))
            out.joint_power_violations = decode_name_list(
                d["rider_joint_power_violations"], path);
        if (d.contains("rider_joint_speed_violations"))
            out.joint_speed_violations = decode_name_list(
                d["rider_joint_speed_violations"], path);
        if (d.contains("rider_positive_work_step_j"))
            out.positive_work_step_j =
                d["rider_positive_work_step_j"].finite_real(path);
        if (d.contains("rider_passive_work_step_j"))
            out.passive_work_step_j =
                d["rider_passive_work_step_j"].finite_real(path);
        if (d.contains("rider_strength_violations"))
            out.strength_violations = decode_name_list(
                d["rider_strength_violations"], path);
    }

    [[nodiscard]] EffortDiagnostics decode_effort_diagnostics(
        const VNode &v, std::string_view path) {
        if (v.is_none()) return {};
        const Dict d = mapping(v, path);
        if (vnode::object(*d.value.value).empty()) return {};
        EffortDiagnostics out;
        ncd::exact_at(d, kEffortRequiredKeys, kEffortOptionalKeys, path);
        out.present = true;
        out.active_request_nm =
            decode_named_reals_node(d["rider_active_request_nm"], path);
        out.active_delivered_nm =
            decode_named_reals_node(d["rider_active_delivered_nm"], path);
        out.positive_power_w = d["rider_positive_power_w"].finite_real(path);
        out.passive_power_w = d["rider_passive_power_w"].finite_real(path);
        out.activation_saturated =
            d["rider_activation_saturated"].boolean(path);
        out.strength_limited =
            decode_name_list(d["rider_strength_limited"], path);
        out.budget_present = d.contains("rider_effort_budget_exceeded");
        out.power_limit_present = d.contains("rider_active_positive_power_limit_w");
        out.observation_present = d.contains("rider_effort_observation");
        if (out.budget_present) out.budget_exceeded = d["rider_effort_budget_exceeded"].boolean(path);
        if (out.power_limit_present) out.positive_power_limit_w = d["rider_active_positive_power_limit_w"].optional_real(path);
        if (out.observation_present) out.observation = d["rider_effort_observation"].string(path);
        decode_effort_solved_fields(d, path, out);
        return out;
    }

    [[nodiscard]] AllocationDiagnostics decode_allocation_diagnostics(
        const VNode &v, std::string_view path) {
        if (v.is_none()) return {};
        const Dict d = mapping(v, path);
        if (vnode::object(*d.value.value).empty()) return {};
        AllocationDiagnostics out;
        if (d.contains("feasible")) {
            out.legacy = bike::rider::decode_response_allocation(*v.value,path);
            out.present = true;
            out.invalid_controller = !out.legacy->allocation.feasible;
            out.crank_task_nm = out.legacy->crank_torque_target_nm.value_or(0.);
            out.crank_task_shortfall_nm = std::max(0.,out.crank_task_nm-out.legacy->predicted_crank_torque_nm.value_or(0.));
            out.solution_excitation_nm = out.legacy->excitation;
            return out;
        }
        ncd::exact(d,
                      keys("invalid_controller", "crank_task_nm",
                           "crank_task_shortfall_nm",
                           "solution_excitation_nm"));
        out.present = true;
        out.invalid_controller = d["invalid_controller"].boolean(path);
        out.crank_task_nm = d["crank_task_nm"].finite_real(path);
        out.crank_task_shortfall_nm =
            d["crank_task_shortfall_nm"].finite_real(path);
        out.solution_excitation_nm =
            vector(d["solution_excitation_nm"], path);
        return out;
    }

    [[nodiscard]] SupportDiagnostics decode_support_diagnostics(
        const VNode &v, std::string_view path) {
        if (v.is_none()) return {};
        const Dict d = mapping(v, path);
        if (vnode::object(*d.value.value).empty()) return {};
        SupportDiagnostics out;
        if(d.contains("requested_vertical_forces_n")) {
            out.legacy=bike::rider::decode_support_diagnostics(*v.value,path);out.present=true;
            out.stance_front=out.legacy->stance[0];out.stance_rear=out.legacy->stance[1];return out;
        }
        ncd::exact(d, keys("stance"));
        const VNode stance_node = d["stance"];
        const std::string stance_path = std::string(path) + ".stance";
        exact_keys(stance_node, keys("front", "rear"), {}, stance_path);
        out.present = true;
        out.stance_front =
            stance_node.member("front").boolean(stance_path + ".front");
        out.stance_rear =
            stance_node.member("rear").boolean(stance_path + ".rear");
        return out;
    }

    [[nodiscard]] NamedEntries<SoleGoalDiagnostics>
    decode_sole_goal_diagnostics(const VNode &v, std::string_view path) {
        NamedEntries<SoleGoalDiagnostics> out;
        if (v.is_none()) return out;
        const Dict d = mapping(v, path);
        d.value.each_member([&](std::string_view name, const VNode &item) {
            if (name != "front" && name != "rear")
                invalid(std::string(path) + "." + std::string(name),
                        "unknown key");
            const auto sub = std::string(path) + "." + std::string(name);
            out.emplace_back(std::string(name), decode_sole_goal(*item.value, sub));
        });
        return out;
    }

    // ---- emission twins (spindle_state_dict etc.) ---------------------------

    [[nodiscard]] bio::Value effort_diagnostics_value(
        const EffortDiagnostics &diagnostics) {
        bio::Value::Object out;
        if (!diagnostics.present) return bio::Value{std::move(out)};
        out.emplace_back("rider_active_request_nm",
                         named_reals_value(diagnostics.active_request_nm));
        out.emplace_back("rider_active_delivered_nm",
                         named_reals_value(
                             diagnostics.active_delivered_nm));
        out.emplace_back("rider_positive_power_w",
                         real_value(diagnostics.positive_power_w));
        out.emplace_back("rider_passive_power_w",
                         real_value(diagnostics.passive_power_w));
        out.emplace_back("rider_activation_saturated",
                         bio::Value{diagnostics.activation_saturated});
        out.emplace_back("rider_strength_limited",
                         names_value(diagnostics.strength_limited));
        if (diagnostics.budget_present) out.emplace_back("rider_effort_budget_exceeded",
                         bio::Value{diagnostics.budget_exceeded});
        if (diagnostics.power_limit_present) set_or_none(out, "rider_active_positive_power_limit_w",
                    diagnostics.positive_power_limit_w);
        if (diagnostics.observation_present) out.emplace_back("rider_effort_observation",
                         bio::Value{diagnostics.observation});
        if (diagnostics.joint_positive_power_w.has_value())
            out.emplace_back("rider_joint_positive_power_w",
                             named_reals_value(
                                 *diagnostics.joint_positive_power_w));
        if (diagnostics.joint_power_violations.has_value())
            out.emplace_back("rider_joint_power_violations",
                             names_value(
                                 *diagnostics.joint_power_violations));
        if (diagnostics.joint_speed_violations.has_value())
            out.emplace_back("rider_joint_speed_violations",
                             names_value(
                                 *diagnostics.joint_speed_violations));
        if (diagnostics.positive_work_step_j.has_value())
            out.emplace_back("rider_positive_work_step_j",
                             real_value(*diagnostics.positive_work_step_j));
        if (diagnostics.passive_work_step_j.has_value())
            out.emplace_back("rider_passive_work_step_j",
                             real_value(*diagnostics.passive_work_step_j));
        if (diagnostics.strength_violations.has_value())
            out.emplace_back("rider_strength_violations",
                             names_value(*diagnostics.strength_violations));
        return bio::Value{std::move(out)};
    }

    [[nodiscard]] bio::Value allocation_diagnostics_value(
        const AllocationDiagnostics &diagnostics) {
        if (diagnostics.legacy) return bike::rider::encode_response_allocation(*diagnostics.legacy);
        bio::Value::Object out;
        if (!diagnostics.present) return bio::Value{std::move(out)};
        out.emplace_back("invalid_controller",
                         bio::Value{diagnostics.invalid_controller});
        out.emplace_back("crank_task_nm",
                         real_value(diagnostics.crank_task_nm));
        out.emplace_back("crank_task_shortfall_nm",
                         real_value(diagnostics.crank_task_shortfall_nm));
        out.emplace_back("solution_excitation_nm",
                         vector_value(diagnostics.solution_excitation_nm));
        return bio::Value{std::move(out)};
    }

    [[nodiscard]] bio::Value support_diagnostics_value(
        const SupportDiagnostics &diagnostics) {
        if(diagnostics.legacy)return bike::rider::encode_support_diagnostics(*diagnostics.legacy);
        bio::Value::Object out;
        if (!diagnostics.present) return bio::Value{std::move(out)};
        bio::Value::Object stance;
        stance.emplace_back("front",
                            bio::Value{diagnostics.stance_front});
        stance.emplace_back("rear", bio::Value{diagnostics.stance_rear});
        out.emplace_back("stance", bio::Value{std::move(stance)});
        return bio::Value{std::move(out)};
    }

    [[nodiscard]] bio::Value sole_goal_diagnostics_value(
        const NamedEntries<SoleGoalDiagnostics> &diagnostics) {
        bio::Value::Object out;
        for (const auto &[name, goal]: diagnostics) {
            out.emplace_back(name, encode_sole_goal(goal));
        }
        return bio::Value{std::move(out)};
    }

    [[nodiscard]] bio::Value last_terms_value(
        const NamedEntries<JointTerms> &terms) {
        bio::Value::Object out;
        for (const auto &[name, term]: terms) {
            bio::Value::Object entry;
            // Python dict-literal order: requested, command, posture,
            // pedaling, saturated; update() keys append afterwards.
            if (term.requested_present) entry.emplace_back("requested_nm",
                               real_value(term.requested_nm));
            entry.emplace_back("command_nm", real_value(term.command_nm));
            entry.emplace_back("posture_nm", real_value(term.posture_nm));
            entry.emplace_back("pedaling_nm",
                               real_value(term.pedaling_nm));
            entry.emplace_back("saturated", bio::Value{term.saturated});
            if (term.active_request_nm.has_value())
                entry.emplace_back("active_request_nm",
                                   real_value(*term.active_request_nm));
            if (term.active_delivered_nm.has_value())
                entry.emplace_back("active_delivered_nm",
                                   real_value(*term.active_delivered_nm));
            if (term.passive_damping_nm.has_value())
                entry.emplace_back("passive_damping_nm",
                                   real_value(*term.passive_damping_nm));
            if (term.solved_force_nm.has_value())
                entry.emplace_back("solved_force_nm",
                                   real_value(*term.solved_force_nm));
            if (term.solved_active_nm.has_value())
                entry.emplace_back("solved_active_nm",
                                   real_value(*term.solved_active_nm));
            if (term.solved_passive_nm.has_value())
                entry.emplace_back("solved_passive_nm",
                                   real_value(*term.solved_passive_nm));
            if (term.support_nm) entry.emplace_back("support_nm",real_value(*term.support_nm));
            if (term.tracking_nm) entry.emplace_back("tracking_nm",real_value(*term.tracking_nm));
            if (term.hip_posture_nm) entry.emplace_back("hip_posture_nm",real_value(*term.hip_posture_nm));
            out.emplace_back(name, bio::Value{std::move(entry)});
        }
        return bio::Value{std::move(out)};
    }

    // ---- spindle config (parse_spindle_config's frame split) ---------------

    constexpr auto kConfigRequiredKeys = keys(
        "activation_tau_s", "active_positive_power_limit_w",
        "arm_reach_fraction", "balance_dwell_s", "balance_floor_kmh",
        "balance_grace_s", "bar_support_fraction",
        "coasting_brake_d_nm_s_rad", "coasting_brake_limit_nm",
        "coupled_task_control", "foot_mu", "grip_attachment",
        "grip_c_ns_m", "grip_capture_distance_m", "grip_capture_speed_mps",
        "grip_k_n_m", "grip_pair_force_limit_n", "grip_pull_per_hand_n",
        "grip_release_distance_m", "joint_envelope_path",
        "joint_envelope_soft_k_nm_rad", "joint_envelope_soft_margin_rad",
        "joint_kd_nms_rad", "joint_kp_nm_rad", "joint_limit_nm",
        "joint_passive_damping_nms_rad", "joint_power_limit_w",
        "joint_speed_limit_rad_s", "joint_strength_path", "link_max_gap_m",
        "pedal_ankle_amplitude_rad", "pedal_attachment", "pedal_c_ns_m",
        "pedal_min_normal_n", "pedal_patch_half_length_m",
        "pedal_scrape_fraction", "pedal_support_fraction",
        "pedal_torque_ripple", "posture_pitch_d_nms_rad",
        "posture_pitch_k_nm_rad", "posture_pitch_limit_nm",
        "posture_sole_depth_m", "posture_translation_d_ns_m",
        "posture_translation_k_n_m", "posture_translation_limit_n",
        "return_foot_preload_n", "road_lookahead_m", "saddle_attachment",
        "saddle_mu", "saddle_patch_half_length_m",
        "saddle_reserve_weight_fraction", "stance_blend_load_n",
        "support_c_ns_m", "support_k_n_m", "support_length_m",
        "support_mu", "support_pad_radius_m", "support_tangent_k_n_m",
        "swing_clearance_m", "strength", "strength_coordinates",
        "envelopes");

    constexpr std::array kUnmodeledScalarKeys{
        "pedal_support_fraction", "bar_support_fraction",
        "posture_sole_depth_m", "swing_clearance_m",
        "stance_blend_load_n", "posture_pitch_k_nm_rad",
        "posture_pitch_d_nms_rad", "posture_pitch_limit_nm",
        "posture_translation_k_n_m", "posture_translation_d_ns_m",
        "posture_translation_limit_n", "saddle_patch_half_length_m",
        "support_k_n_m", "support_c_ns_m", "pedal_c_ns_m",
        "support_tangent_k_n_m", "support_mu", "support_length_m",
        "grip_k_n_m", "grip_c_ns_m", "grip_release_distance_m",
        "grip_capture_distance_m", "grip_capture_speed_mps",
        "grip_pull_per_hand_n", "foot_mu", "saddle_mu",
        "pedal_min_normal_n", "saddle_reserve_weight_fraction",
        "pedal_ankle_amplitude_rad", "pedal_scrape_fraction",
        "link_max_gap_m", "road_lookahead_m", "balance_floor_kmh",
        "balance_dwell_s", "balance_grace_s"};

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) key/path mirror the binding's (member, path) order
    void decode_optional_path(const Dict &d, std::string_view key,
                              std::string_view path) {
        const VNode v = d[key];
        if (!v.is_none()) (void)v.string(path);
    }

    // Other full-config fields are type-validated before branch-specific decode; the values
    // belong to surfaces the spindle path does not reach.
    void validate_unmodeled_scalars(const Dict &d, std::string_view path) {
        for (const char *key: kUnmodeledScalarKeys)
            (void)d[key].finite_real(path);
        (void)d["grip_pair_force_limit_n"].optional_real(path);
        (void)d["coupled_task_control"].boolean(path);
    }

    // The class implements exactly the supported spindle plant; other
    // attachments route through paths that do not exist natively.
    void validate_attachments(const Dict &d, std::string_view path) {
        const auto pedal_attachment = d["pedal_attachment"].string(path);
        if (pedal_attachment != "spindle" && pedal_attachment != "flat" && pedal_attachment != "weld")
            invalid(std::string(path) + ".pedal_attachment",
                    "supported: 'flat', 'weld', 'spindle'");
        const auto saddle_attachment = d["saddle_attachment"].string(path);
        if (saddle_attachment != "pin" && saddle_attachment != "flat" && saddle_attachment != "weld")
            invalid(std::string(path) + ".saddle_attachment",
                    "supported: 'flat', 'weld', 'pin'");
        const auto grip_attachment = d["grip_attachment"].string(path);
        if (grip_attachment != "connect" && grip_attachment != "spring" && grip_attachment != "weld")
            invalid(std::string(path) + ".grip_attachment",
                    "supported: 'spring', 'connect' (alias 'weld')");
    }

    void decode_actuation_scalars(const Dict &d, std::string_view path,
                                  SpindleConfig &config) {
        config.joint_kp_nm_rad = d["joint_kp_nm_rad"].finite_real(path);
        config.joint_kd_nms_rad = d["joint_kd_nms_rad"].finite_real(path);
        config.joint_limit_nm = d["joint_limit_nm"].finite_real(path);
        config.joint_speed_limit_rad_s =
            d["joint_speed_limit_rad_s"].finite_real(path);
        config.joint_power_limit_w =
            d["joint_power_limit_w"].finite_real(path);
        config.active_positive_power_limit_w =
            d["active_positive_power_limit_w"].optional_real(path);
        config.joint_passive_damping_nms_rad =
            d["joint_passive_damping_nms_rad"].optional_real(path);
        config.activation_tau_s = d["activation_tau_s"].finite_real(path);
    }

    void decode_geometry_scalars(const Dict &d, std::string_view path,
                                 SpindleConfig &config) {
        config.pedal_torque_ripple =
            d["pedal_torque_ripple"].finite_real(path);
        config.return_foot_preload_n =
            d["return_foot_preload_n"].finite_real(path);
        config.coasting_brake_d_nm_s_rad =
            d["coasting_brake_d_nm_s_rad"].finite_real(path);
        config.coasting_brake_limit_nm =
            d["coasting_brake_limit_nm"].finite_real(path);
        config.joint_envelope_soft_k_nm_rad =
            d["joint_envelope_soft_k_nm_rad"].finite_real(path);
        config.joint_envelope_soft_margin_rad =
            d["joint_envelope_soft_margin_rad"].finite_real(path);
        config.arm_reach_fraction =
            d["arm_reach_fraction"].finite_real(path);
        config.pedal_patch_half_length_m =
            d["pedal_patch_half_length_m"].finite_real(path);
        config.support_pad_radius_m =
            d["support_pad_radius_m"].finite_real(path);
    }

    void decode_strength_tables(const Dict &d, std::string_view path,
                                SpindleConfig &config) {
        const VNode strength = d["strength"];
        if (!strength.is_none()) {
            config.has_strength = true;
            const Dict table = mapping(strength, path);
            table.value.each_member(
                [&](std::string_view name, const VNode &item) {
                    const auto sub =
                        std::string(path) + "." + std::string(name);
                    const Dict pair = mapping(item, sub);
                    ncd::exact(pair, keys("positive", "negative"));
                    DirectionalStrength curves{
                        .positive =
                            decode_curve(pair["positive"], sub + ".positive"),
                        .negative = decode_curve(pair["negative"],
                                                 sub + ".negative")};
                    config.strength.emplace(std::string(name),
                                            std::move(curves));
                });
        }
        config.strength_coordinates =
            decode_envelope_map(d["strength_coordinates"], path);
        config.envelopes = decode_envelope_map(d["envelopes"], path);
    }

    // ---- spindle state sections ---------------------------------------------

    constexpr auto kStateRequiredKeys = keys(
        "enabled", "command_enabled", "active_state", "activation_time_s",
        "last_terms", "pd_split", "saturated_ik", "ik_reach_limited",
        "joint_torques_nm", "joint_capacity_nm", "lean_limit_rad",
        "last_branch", "last_solution", "pedal_recovery",
        "effort_diagnostics", "allocation_diagnostics",
        "support_diagnostics", "sole_goal_diagnostics");
    constexpr auto kIkJointKeys = keys("front", "rear", "torso", "arms");
    constexpr auto kSideKeys = keys("front", "rear");

    void decode_state_terms(const Dict &d, std::string_view path,
                            SpindleController::State &state) {
        const Dict terms = mapping(d["last_terms"], path);
        terms.value.each_member(
            [&](std::string_view name, const VNode &item) {
                state.last_terms.emplace_back(
                    std::string(name),
                    decode_joint_terms_node(
                        item,
                        std::string(path) + ".last_terms." +
                            std::string(name)));
            });
        const Dict split = mapping(d["pd_split"], path);
        split.value.each_member([&](std::string_view name,const VNode& item){
            state.pd_split.emplace_back(std::string(name),fixed<2>(item,path));
        });
    }

    void decode_state_ik(const Dict &d, std::string_view path,
                         SpindleController::State &state) {
        state.saturated_ik =
            decode_named_bools(d["saturated_ik"], path, kIkJointKeys);
        state.ik_reach_limited =
            decode_named_bools(d["ik_reach_limited"], path, kSideKeys);
        state.joint_torques_nm =
            decode_named_reals_node(d["joint_torques_nm"], path);
        state.joint_capacity_nm =
            decode_named_reals_node(d["joint_capacity_nm"], path);
        state.lean_limit_rad = d["lean_limit_rad"].optional_real(path);
        if (!d["last_branch"].is_none() || !d["last_solution"].is_none()) {
            state.allocation_memory.emplace();
            if (!d["last_branch"].is_none()) {
                const auto b=d["last_branch"].sequence(path);
                if(b.size()!=2)invalid(path,"allocator branch requires two booleans");
                state.allocation_memory->branch=std::array<bool,2>{b.at(0).boolean(path),b.at(1).boolean(path)};
            }
            if (!d["last_solution"].is_none()) state.allocation_memory->solution=vector(d["last_solution"],path);
        }
    }

    void decode_state_recovery(const Dict &d, std::string_view path,
                               SpindleController::State &state) {
        const Dict recovery = mapping(d["pedal_recovery"], path);
        ncd::exact_at(recovery, kSideKeys, {},
                         std::string(path) + ".pedal_recovery");
        for (const char *side: {"front", "rear"})
            state.pedal_recovery.emplace_back(
                side,
                decode_pedal_recovery(recovery[side],
                                      std::string(path) +
                                          ".pedal_recovery." + side));
        state.effort_diagnostics = decode_effort_diagnostics(
            d["effort_diagnostics"],
            std::string(path) + ".effort_diagnostics");
        state.allocation_diagnostics = decode_allocation_diagnostics(
            d["allocation_diagnostics"],
            std::string(path) + ".allocation_diagnostics");
        state.support_diagnostics = decode_support_diagnostics(
            d["support_diagnostics"],
            std::string(path) + ".support_diagnostics");
        state.sole_goal_diagnostics = decode_sole_goal_diagnostics(
            d["sole_goal_diagnostics"],
            std::string(path) + ".sole_goal_diagnostics");
    }

    // ---- intent stack (intent_wire.cpp twins) --------------------------------

    [[nodiscard]] IntentSignals decode_signals(const VNode &v,
                                               std::string_view path) {
        const Dict d = mapping(v, path);
        ncd::exact(d,
                      keys("pitch_rate_up_rad_s",
                           "specific_force_body_mps2", "crank_rate_rad_s",
                           "human_crank_torque_nm", "front_load_share"));
        IntentSignals signals;
        // SeatedClimbSignals.__post_init__ order: scalars, force array,
        // share.
        signals.pitch_rate_up_rad_s =
            d["pitch_rate_up_rad_s"].finite_real(path);
        signals.crank_rate_rad_s = d["crank_rate_rad_s"].finite_real(path);
        signals.human_crank_torque_nm =
            d["human_crank_torque_nm"].finite_real(path);
        signals.specific_force_body_mps2 =
            fixed<3>(d["specific_force_body_mps2"], path);
        const VNode share = d["front_load_share"];
        if (!share.is_none()) {
            const double parsed = share.finite_real(path);
            if (parsed < 0.) invalid(path, "invalid front load share");
            if (parsed > 1.)
                invalid(path, "front load share must not exceed one");
            signals.front_load_share = parsed;
        }
        return signals;
    }

    [[nodiscard]] std::pair<double, IntentSignals> decode_policy_sample(
        const VNode &v, std::string_view path) {
        const auto row = v.sequence(path);
        if (row.size() != 2)
            invalid(path, "incorrect sequence width");
        return {row.at(0).finite_real(path),
                decode_signals(row.at(1), std::string(path) + ".signals")};
    }

    [[nodiscard]] std::pair<double, std::optional<double>>
    decode_load_sample(const VNode &v, std::string_view path) {
        const auto row = v.sequence(path);
        if (row.size() != 2)
            invalid(path, "incorrect sequence width");
        return {row.at(0).finite_real(path),
                row.at(1).optional_real(path)};
    }

    [[nodiscard]] Intent decode_intent_block(const VNode &v,
                                             const std::string &path) {
        const Dict intent = mapping(v, path);
        ncd::exact(intent, keys("posture", "effort_ceiling_nm"));
        Intent block;
        block.posture =
            decode_rider_posture(*intent["posture"].value,
                                 path + ".posture");
        block.effort_ceiling_nm =
            intent["effort_ceiling_nm"].finite_real(path);
        return block;
    }

    [[nodiscard]] SeatedClimbPolicy::State decode_intent_policy(
        const VNode &v, const std::string &policy_path) {
        const Dict policy = mapping(v, policy_path);
        ncd::exact(policy,
                      keys("time_s", "inclination_rad", "lean_rad",
                           "effort_nm", "samples", "delayed",
                           "surge_budget_s_left"));
        SeatedClimbPolicy::State out;
        out.time_s = policy["time_s"].finite_real(policy_path);
        out.inclination_rad =
            policy["inclination_rad"].finite_real(policy_path);
        out.lean_rad = policy["lean_rad"].finite_real(policy_path);
        out.effort_nm = policy["effort_nm"].finite_real(policy_path);
        const auto samples = policy["samples"].sequence(policy_path);
        for (std::size_t i = 0; i < samples.size(); ++i)
            out.samples.push_back(
                decode_policy_sample(samples.at(i),
                                     policy_path + ".samples"));
        const VNode delayed = policy["delayed"];
        if (!delayed.is_none())
            out.delayed =
                decode_signals(delayed, policy_path + ".delayed");
        out.surge_budget_s_left =
            policy["surge_budget_s_left"].finite_real(policy_path);
        return out;
    }

    [[nodiscard]] SeatedPostureProgram::State decode_intent_program(
        const VNode &v, const std::string &program_path) {
        const Dict program = mapping(v, program_path);
        ncd::exact(program,
                      keys("lean_rad", "trim_rad", "load_samples",
                           "delayed_load_share", "pulses"));
        SeatedPostureProgram::State out;
        out.lean_rad = program["lean_rad"].finite_real(program_path);
        out.trim_rad = program["trim_rad"].finite_real(program_path);
        const auto load_samples =
            program["load_samples"].sequence(program_path);
        for (std::size_t i = 0; i < load_samples.size(); ++i)
            out.load_samples.push_back(decode_load_sample(
                load_samples.at(i), program_path + ".load_samples"));
        out.delayed_load_share =
            program["delayed_load_share"].optional_real(program_path);
        const auto pulses = program["pulses"].sequence(program_path);
        for (std::size_t i = 0; i < pulses.size(); ++i)
            out.pulses.push_back(
                fixed<3>(pulses.at(i), program_path + ".pulses"));
        return out;
    }

    // ---- attachment records (attachment_binding.cpp twins) ------------------

    // np.asarray(x, dtype=float) for the 1-D members — the nested-sequence
    // analog of the binding's caster, with the same rejection spelling.
    // Number members convert with float() semantics (nonfinite spellings
    // like 1e400 land as inf, matching the untyped Python cast).
    [[nodiscard]] std::vector<double> float_vector(const VNode &v,
                                                   std::string_view label) {
        if (!std::holds_alternative<bio::Value::Array>(v.value->data))
            throw std::invalid_argument(std::string(label) +
                                        " must be a 1-D float64 sequence");
        const auto items = v.sequence(label);
        std::vector<double> out;
        out.reserve(items.size());
        for (std::size_t i = 0; i < items.size(); ++i) {
            const auto *const number =
                std::get_if<bio::Number>(&items.at(i).value->data);
            if (number == nullptr)
                throw std::invalid_argument(
                    std::string(label) + " must be a 1-D float64 sequence");
            bool ok = false;
            const double parsed =
                bio::detail::float_from_lexeme(number->lexeme, ok);
            if (!ok)
                throw std::invalid_argument(
                    std::string(label) + " must be a 1-D float64 sequence");
            out.push_back(parsed);
        }
        return out;
    }

    [[nodiscard]] std::optional<std::vector<double>> optional_vector(
        const VNode &v, std::string_view label) {
        if (v.is_none()) return std::nullopt;
        return float_vector(v, label);
    }

    [[nodiscard]] std::vector<std::int64_t> index_vector(
        const VNode &v, std::string_view label) {
        if (!std::holds_alternative<bio::Value::Array>(v.value->data))
            throw std::invalid_argument(std::string(label) +
                                        " must be an integer sequence");
        const auto items = v.sequence(label);
        std::vector<std::int64_t> out;
        out.reserve(items.size());
        for (std::size_t i = 0; i < items.size(); ++i) {
            const auto *const number =
                std::get_if<bio::Number>(&items.at(i).value->data);
            if (number == nullptr ||
                !bio::is_integer_lexeme(number->lexeme))
                throw std::invalid_argument(
                    std::string(label) + " must be an integer sequence");
            std::int64_t parsed = 0;
            const char *const end =
                std::to_address(number->lexeme.cend());
            const auto result =
                std::from_chars(number->lexeme.data(), end, parsed, 10);
            if (result.ec != std::errc() || result.ptr != end)
                throw std::invalid_argument(
                    std::string(label) + " must be an integer sequence");
            out.push_back(parsed);
        }
        return out;
    }

    // The nested-row matrix — the binding's 2-D float64 ndarray analog.
    // Row-major DenseMatrix storage, ragged rows and non-numeric members
    // reject like a bad ndarray cast.
    [[nodiscard]] DenseMatrix dense_matrix_arg(const VNode &v,
                                               std::string_view label) {
        const auto bad = [&label]() -> std::invalid_argument {
            return std::invalid_argument(std::string(label) +
                                         " must be a 2-D float64 array");
        };
        if (!std::holds_alternative<bio::Value::Array>(v.value->data))
            throw bad();
        const auto &rows = vnode::array(*v.value);
        DenseMatrix m{.rows = rows.size(), .columns = 0, .values = {}};
        if (!rows.empty()) {
            const auto *const first =
                std::get_if<bio::Value::Array>(&rows.front().data);
            if (first == nullptr) throw bad();
            m.columns = first->size();
            m.values.reserve(m.rows * m.columns);
            for (const bio::Value &row: rows) {
                const auto *const entries =
                    std::get_if<bio::Value::Array>(&row.data);
                if (entries == nullptr || entries->size() != m.columns)
                    throw bad();
                for (const bio::Value &entry: *entries) {
                    const auto *const number =
                        std::get_if<bio::Number>(&entry.data);
                    if (number == nullptr) throw bad();
                    bool ok = false;
                    const double parsed = bio::detail::float_from_lexeme(
                        number->lexeme, ok);
                    if (!ok) throw bad();
                    m.values.push_back(parsed);
                }
            }
        }
        return m;
    }

    // Field access on the record document — the wire carries objects; the
    // binding's dataclass attribute fallback has no JSON analog, so a
    // non-object owner reports every field absent and the first typed
    // read rejects with the record's own diagnostic family.
    [[nodiscard]] VNode attachment_field(const VNode &owner,
                                         const char *name) {
        if (!owner.is_mapping()) return {&vnode::missing()};
        return owner.member(name);
    }

    [[nodiscard]] AttachmentGeometry decode_geometry_doc(
        const VNode &owner) {
        AttachmentGeometry g;
        {
            const VNode v = attachment_field(owner, "eq_id");
            const auto *const number =
                std::get_if<bio::Number>(&v.value->data);
            bool ok = false;
            std::int64_t parsed = 0;
            if (number != nullptr &&
                bio::is_integer_lexeme(number->lexeme)) {
                const char *const end =
                    std::to_address(number->lexeme.cend());
                const auto result =
                    std::from_chars(number->lexeme.data(), end, parsed, 10);
                ok = result.ec == std::errc() && result.ptr == end &&
                     parsed >= std::numeric_limits<int>::min() &&
                     parsed <= std::numeric_limits<int>::max();
            }
            if (!ok)
                throw std::invalid_argument("eq_id must be an integer");
            g.eq_id = static_cast<int>(parsed);
        }
        g.rider_jac = dense_matrix_arg(attachment_field(owner, "rider_jac"),
                                       "rider_jac");
        g.rider_columns = index_vector(
            attachment_field(owner, "rider_columns"), "rider_columns");
        g.bike_jac = dense_matrix_arg(attachment_field(owner, "bike_jac"),
                                      "bike_jac");
        g.bike_columns = index_vector(
            attachment_field(owner, "bike_columns"), "bike_columns");
        g.observable = truthy(attachment_field(owner, "observable"));
        g.normal =
            float_vector(attachment_field(owner, "normal"), "normal");
        {
            const VNode v = attachment_field(owner, "kind");
            const auto *const text =
                std::get_if<std::string>(&v.value->data);
            if (text == nullptr)
                throw std::invalid_argument("kind must be a string");
            g.kind = *text;
        }
        g.rotational = truthy(attachment_field(owner, "rotational"));
        {
            const VNode v = attachment_field(owner, "half_patch_m");
            const auto *const number =
                std::get_if<bio::Number>(&v.value->data);
            if (number == nullptr)
                throw std::invalid_argument("half_patch_m must be a float");
            bool ok = false;
            const double parsed =
                bio::detail::float_from_lexeme(number->lexeme, ok);
            if (!ok)
                throw std::invalid_argument("half_patch_m must be a float");
            g.half_patch_m = parsed;
        }
        g.pull_direction = optional_vector(
            attachment_field(owner, "pull_direction"), "pull_direction");
        return g;
    }

    [[nodiscard]] bio::Value raw_value(const AttachmentRaw &r) {
        bio::Value::Object out;
        out.emplace_back("rider_jac", matrix_value(r.rider_jac));
        out.emplace_back("rider_qfrc", vector_value(r.rider_qfrc));
        out.emplace_back("bike_jac", matrix_value(r.bike_jac));
        out.emplace_back("bike_qfrc", vector_value(r.bike_qfrc));
        out.emplace_back("observable", bio::Value{r.observable});
        out.emplace_back("normal", vector_value(r.normal));
        out.emplace_back("kind", bio::Value{r.kind});
        out.emplace_back("rotational", bio::Value{r.rotational});
        out.emplace_back("half_patch_m", real_value(r.half_patch_m));
        out.emplace_back("gap_m", real_value(r.gap_m));
        out.emplace_back("pull_direction",
                         optional_vector_value(r.pull_direction));
        return bio::Value{std::move(out)};
    }

    [[nodiscard]] bio::Value sample_value(const AttachmentSample &s) {
        bio::Value::Object out;
        out.emplace_back("kind", bio::Value{s.kind});
        out.emplace_back("normal_n", real_value(s.normal_n));
        out.emplace_back("tangent_n", real_value(s.tangent_n));
        out.emplace_back("moment_nm", real_value(s.moment_nm));
        out.emplace_back("gap_m", real_value(s.gap_m));
        out.emplace_back("pull_n", real_value(s.pull_n));
        out.emplace_back("half_patch_m", real_value(s.half_patch_m));
        return bio::Value{std::move(out)};
    }

    // ---- contacts state (rider_contact_binding.cpp twins) -------------------

    [[nodiscard]] std::array<bool, wc::kContactCount> decode_enabled(
        const VNode &v, const std::string &path) {
        const Dict d = mapping(v, path);
        ncd::exact_at(d, wc::kContactNames, {}, path);
        std::array<bool, wc::kContactCount> out{};
        std::string member_path(path);
        member_path += '.';
        const std::size_t stem = member_path.size();
        for (std::size_t i = 0; i < wc::kContactCount; ++i) {
            const std::string name(wc::kContactNames[i]);
            member_path.resize(stem);
            member_path += name;
            out[i] = d[name].boolean(member_path);
        }
        return out;
    }

    [[nodiscard]] wc::RiderContactPatchDiag decode_patch(
        const VNode &v, const std::string &path) {
        const Dict d = mapping(v, path);
        constexpr auto required =
            keys("in_platform", "normal_load_n", "gap_m", "point_m",
                 "force_on_rider_n", "radial_energy_j", "shear_energy_j");
        ncd::exact(d, required);
        const auto real = [&d, &path](const char *key) {
            return d[key].finite_real(path + "." + key);
        };
        const auto vec = [&d, &path](const char *key) {
            return fixed<3>(d[key], path + "." + key);
        };
        return {.in_platform = d["in_platform"].boolean(path + ".in_platform"),
                .normal_load_n = real("normal_load_n"),
                .gap_m = real("gap_m"),
                .point_m = vec("point_m"),
                .force_on_rider_n = vec("force_on_rider_n"),
                .radial_energy_j = real("radial_energy_j"),
                .shear_energy_j = real("shear_energy_j")};
    }

    [[nodiscard]] wc::RiderContactSupportDiag decode_support_diag(
        const VNode &v, bool welded, const std::string &path) {
        const Dict d = mapping(v, path);
        wc::RiderContactSupportDiag out{};
        out.welded = welded;
        const auto real = [&d, &path](const char *key) {
            return d[key].finite_real(path + "." + key);
        };
        const auto flag = [&d, &path](const char *key) {
            return d[key].boolean(path + "." + key);
        };
        const auto vec = [&d, &path](const char *key) {
            return fixed<3>(d[key], path + "." + key);
        };
        const bool detailed = d.contains("patches");
        if (!detailed) {
            if (welded) {
                constexpr auto base = keys(
                    "enabled", "in_platform", "normal_load_n",
                    "would_separate", "would_slip", "tangent_n", "gap_m",
                    "vertical_force_on_rider_n");
                ncd::exact(d, base);
            } else {
                constexpr auto base = keys(
                    "enabled", "in_platform", "normal_load_n", "gap_m",
                    "vertical_force_on_rider_n");
                ncd::exact(d, base);
            }
        } else {
            if (welded) {
                constexpr auto full = keys(
                    "enabled", "in_platform", "normal_load_n",
                    "would_separate", "would_slip", "tangent_n", "gap_m",
                    "vertical_force_on_rider_n", "tangent_force_n",
                    "patches", "force_on_rider_n", "force_on_bike_n",
                    "moment_about_rider_origin_nm", "radial_energy_j",
                    "shear_energy_j", "relative_power_w");
                ncd::exact(d, full);
            } else {
                constexpr auto full = keys(
                    "enabled", "in_platform", "normal_load_n", "gap_m",
                    "vertical_force_on_rider_n", "tangent_force_n",
                    "patches", "force_on_rider_n", "force_on_bike_n",
                    "moment_about_rider_origin_nm", "radial_energy_j",
                    "shear_energy_j", "relative_power_w");
                ncd::exact(d, full);
            }
        }
        out.enabled = flag("enabled");
        out.in_platform = flag("in_platform");
        out.normal_load_n = real("normal_load_n");
        if (welded) {
            out.would_separate = flag("would_separate");
            out.would_slip = flag("would_slip");
            out.tangent_n = real("tangent_n");
        }
        out.gap_m = real("gap_m");
        out.vertical_force_on_rider_n = real("vertical_force_on_rider_n");
        out.detailed = detailed;
        if (detailed) {
            if (welded)
                out.tangent_force_vec = vec("tangent_force_n");
            else
                out.tangent_force_scalar = real("tangent_force_n");
            const auto patches =
                d["patches"].sequence(path + ".patches");
            for (std::size_t index = 0; index < patches.size(); ++index)
                out.patches.push_back(
                    decode_patch(patches.at(index),
                                 path + ".patches." +
                                     std::to_string(index)));
            out.force_on_rider_n = vec("force_on_rider_n");
            out.force_on_bike_n = vec("force_on_bike_n");
            out.moment_nm = vec("moment_about_rider_origin_nm");
            out.radial_energy_j = real("radial_energy_j");
            out.shear_energy_j = real("shear_energy_j");
            out.relative_power_w = real("relative_power_w");
        }
        return out;
    }

    // The spring grip side dict's full field set (the connect layout's
    // extended block minus nothing — spring sides always carry it).
    constexpr auto kGripSideSpringKeys = keys(
        "enabled", "reachable", "overloaded", "trial_pair_force_n",
        "pair_force_limit_n", "release_loss_j", "shoulder_distance_m",
        "arm_reach_m", "hand_gap_m", "point_m", "force_on_rider_n",
        "force_on_bike_n", "elastic_energy_j");
    constexpr auto kGripSpringKeys = keys(
        "enabled", "reachable", "overloaded", "trial_pair_force_n",
        "pair_force_limit_n", "release_loss_j", "shoulder_distance_m",
        "arm_reach_m", "hand_gap_m", "force_on_rider_n", "force_on_bike_n",
        "elastic_energy_j");
    constexpr auto kGripSideWeldedBase =
        keys("enabled", "reachable", "hand_gap_m");
    constexpr auto kGripSideWeldedFull = keys(
        "enabled", "reachable", "hand_gap_m", "overloaded",
        "trial_pair_force_n", "pair_force_limit_n", "release_loss_j",
        "shoulder_distance_m", "arm_reach_m", "point_m", "force_on_rider_n",
        "force_on_bike_n", "elastic_energy_j");
    constexpr auto kGripWeldedFull = keys(
        "enabled", "reachable", "hand_gap_m", "overloaded",
        "trial_pair_force_n", "pair_force_limit_n", "release_loss_j",
        "shoulder_distance_m", "arm_reach_m", "force_on_rider_n",
        "force_on_bike_n", "elastic_energy_j");

    [[nodiscard]] wc::RiderContactGripSideDiag decode_grip_side(
        const VNode &v, bool welded, const std::string &path) {
        const Dict d = mapping(v, path);
        wc::RiderContactGripSideDiag out{};
        out.welded = welded;
        const bool extended = !welded || d.contains("overloaded");
        if (welded) {
            if (extended)
                ncd::exact(d, kGripSideWeldedFull);
            else
                ncd::exact(d, kGripSideWeldedBase);
        } else {
            ncd::exact(d, kGripSideSpringKeys);
        }
        const auto real = [&d, &path](const char *key) {
            return d[key].finite_real(path + "." + key);
        };
        const auto flag = [&d, &path](const char *key) {
            return d[key].boolean(path + "." + key);
        };
        out.enabled = flag("enabled");
        out.reachable = flag("reachable");
        out.hand_gap_m = real("hand_gap_m");
        out.extended = extended;
        if (extended) {
            out.overloaded = flag("overloaded");
            out.trial_pair_force_n = real("trial_pair_force_n");
            out.pair_force_limit_n =
                d["pair_force_limit_n"].optional_real(
                    path + ".pair_force_limit_n");
            out.release_loss_j = real("release_loss_j");
            out.shoulder_distance_m = real("shoulder_distance_m");
            out.arm_reach_m = real("arm_reach_m");
            out.point_m = fixed<3>(d["point_m"], path + ".point_m");
            out.force_on_rider_n =
                fixed<3>(d["force_on_rider_n"], path + ".force_on_rider_n");
            out.force_on_bike_n =
                fixed<3>(d["force_on_bike_n"], path + ".force_on_bike_n");
            out.elastic_energy_j = real("elastic_energy_j");
        }
        return out;
    }

    [[nodiscard]] wc::RiderContactGripDiag decode_grip(
        const VNode &v, bool welded, const std::string &path) {
        const Dict d = mapping(v, path);
        wc::RiderContactGripDiag out{};
        out.welded = welded;
        const bool extended = !welded || d.contains("overloaded");
        if (welded) {
            if (extended)
                ncd::exact(d, kGripWeldedFull);
            else
                ncd::exact(d, kGripSideWeldedBase);
        } else {
            ncd::exact(d, kGripSpringKeys);
        }
        const auto real = [&d, &path](const char *key) {
            return d[key].finite_real(path + "." + key);
        };
        const auto flag = [&d, &path](const char *key) {
            return d[key].boolean(path + "." + key);
        };
        out.enabled = flag("enabled");
        out.reachable = flag("reachable");
        out.hand_gap_m = real("hand_gap_m");
        out.extended = extended;
        if (extended) {
            out.overloaded = flag("overloaded");
            out.trial_pair_force_n = real("trial_pair_force_n");
            out.pair_force_limit_n =
                d["pair_force_limit_n"].optional_real(
                    path + ".pair_force_limit_n");
            out.release_loss_j = real("release_loss_j");
            out.shoulder_distance_m = real("shoulder_distance_m");
            out.arm_reach_m = real("arm_reach_m");
            out.force_on_rider_n =
                fixed<3>(d["force_on_rider_n"], path + ".force_on_rider_n");
            out.force_on_bike_n =
                fixed<3>(d["force_on_bike_n"], path + ".force_on_bike_n");
            out.elastic_energy_j = real("elastic_energy_j");
        }
        return out;
    }

    // The diagnostics tree the writer's own attachment layout produces —
    // the key sets are config-determined, so a foreign layout is rejected
    // rather than reinterpreted.
    [[nodiscard]] wc::RiderContactsDiagnostics decode_diagnostics(
        const VNode &v, const wc::RiderContactWriter &w,
        const std::string &path) {
        const Dict d = mapping(v, path);
        wc::RiderContactsDiagnostics out{};
        if (vnode::object(*d.value.value).empty())
            return out; // the unevaluated {} state
        out.evaluated = true;
        constexpr auto required =
            keys("saddle", "front_pedal", "rear_pedal", "grip_left",
                 "grip_right", "grip");
        ncd::exact(d, required);
        std::string member_path(path);
        member_path += '.';
        const std::size_t stem = member_path.size();
        for (std::size_t si = 0; si < wc::kSupportCount; ++si) {
            const std::string name(wc::kSupportNames[si]);
            const bool welded =
                si == 0 ? w.linked_saddle() : w.linked_pedals();
            member_path.resize(stem);
            member_path += name;
            out.supports[si] =
                decode_support_diag(d[name], welded, member_path);
        }
        for (std::size_t side = 0; side < wc::kSideCount; ++side) {
            std::string name("grip_");
            name += wc::kSideNames[side];
            member_path.resize(stem);
            member_path += name;
            out.grip_sides[side] =
                decode_grip_side(d[name], w.welded_grip(), member_path);
        }
        out.grip = decode_grip(d["grip"], w.welded_grip(), path + ".grip");
        return out;
    }

    // (welds dict, crank) — the settle_welds latch shape. Entry names
    // must be ones this writer's attachments could have produced.
    [[nodiscard]] std::optional<wc::RiderSettledWelds> decode_settled(
        const VNode &v, const wc::RiderContactWriter &w,
        const std::string &path) {
        if (v.is_none()) return std::nullopt;
        const auto pair = v.sequence(path);
        if (pair.size() != 2)
            invalid(path, "expected the (welds, crank) pair");
        const VNode welds_raw = pair.at(0);
        const VNode crank_raw = pair.at(1);
        const Dict welds = mapping(welds_raw, path + ".welds");
        wc::RiderSettledWelds out{};
        welds.value.each_member(
            [&](std::string_view name, const VNode &item) {
                const std::size_t support_index = [&]() -> std::size_t {
                    for (std::size_t i = 0; i < wc::kSupportCount; ++i)
                        if (name == wc::kSupportNames[i]) return i;
                    return wc::kSupportCount;
                }();
                const bool is_support =
                    support_index < wc::kSupportCount &&
                    (support_index == 0 ? w.linked_saddle()
                                        : w.linked_pedals());
                const bool is_grip =
                    w.welded_grip() &&
                    (name == "grip_left" || name == "grip_right");
                const std::string entry_path =
                    path + "." + std::string(name);
                if (!is_support && !is_grip)
                    invalid(entry_path, "unknown key");
                const Dict entry = mapping(item, entry_path);
                wc::RiderSettledEntry parsed{};
                parsed.support_fields = is_support;
                if (is_support)
                    ncd::exact(entry,
                                  keys("force_on_rider_n", "normal_n",
                                       "tangent_n", "would_separate",
                                       "would_slip"));
                else
                    ncd::exact(entry, keys("force_on_rider_n"));
                parsed.force_on_rider_n = fixed<3>(
                    entry["force_on_rider_n"],
                    entry_path + ".force_on_rider_n");
                if (is_support) {
                    parsed.normal_n = entry["normal_n"].finite_real(
                        entry_path + ".normal_n");
                    parsed.tangent_n = entry["tangent_n"].finite_real(
                        entry_path + ".tangent_n");
                    parsed.would_separate = entry["would_separate"].boolean(
                        entry_path + ".would_separate");
                    parsed.would_slip = entry["would_slip"].boolean(
                        entry_path + ".would_slip");
                }
                out.entries.emplace_back(std::string(name), parsed);
            });
        out.crank_torque_nm =
            crank_raw.finite_real(path + ".crank_torque_nm");
        return out;
    }

    // One last_attachment_samples entry: 'rider_jac' membership selects
    // the raw block over the budget sample; both faces carry their exact
    // key-set gate.
    [[nodiscard]] wc::RiderAttachmentEntry decode_entry(
        const VNode &v, const std::string &path) {
        if (!v.is_mapping())
            invalid(path, "expected dict");
        const bool is_raw =
            bio::find_member(vnode::object(*v.value), "rider_jac") !=
            nullptr;
        const Dict d = mapping(v, path);
        const auto field_at = [&](const char *name) -> VNode {
            if (!d.contains(name))
                invalid(path + "." + name, "missing required key");
            return d[name];
        };
        if (is_raw) {
            ncd::exact(d,
                          keys("rider_jac", "rider_qfrc", "bike_jac",
                               "bike_qfrc", "observable", "normal", "kind",
                               "rotational", "half_patch_m", "gap_m",
                               "pull_direction"));
            AttachmentRaw raw;
            raw.rider_jac = dense_matrix_arg(field_at("rider_jac"),
                                             path + ".rider_jac");
            raw.rider_qfrc = float_vector(field_at("rider_qfrc"),
                                          path + ".rider_qfrc");
            raw.bike_jac = dense_matrix_arg(field_at("bike_jac"),
                                            path + ".bike_jac");
            raw.bike_qfrc = float_vector(field_at("bike_qfrc"),
                                         path + ".bike_qfrc");
            raw.observable = truthy(field_at("observable"));
            raw.normal =
                float_vector(field_at("normal"), path + ".normal");
            raw.kind = field_at("kind").string(path + ".kind");
            raw.rotational = truthy(field_at("rotational"));
            raw.half_patch_m = field_at("half_patch_m")
                                   .finite_real(path + ".half_patch_m");
            raw.gap_m = field_at("gap_m").finite_real(path + ".gap_m");
            raw.pull_direction = optional_vector(
                field_at("pull_direction"), path + ".pull_direction");
            return raw;
        }
        ncd::exact(d,
                      keys("kind", "normal_n", "tangent_n", "moment_nm",
                           "gap_m", "pull_n", "half_patch_m"));
        AttachmentSample sample;
        sample.kind = field_at("kind").string(path + ".kind");
        const auto real = [&](const char *name) {
            return field_at(name).finite_real(path + "." + name);
        };
        sample.normal_n = real("normal_n");
        sample.tangent_n = real("tangent_n");
        sample.moment_nm = real("moment_nm");
        sample.gap_m = real("gap_m");
        sample.pull_n = real("pull_n");
        sample.half_patch_m = real("half_patch_m");
        return sample;
    }

    // ---- contacts emission ---------------------------------------------------

    [[nodiscard]] bio::Value vec3_value(const Vec3 &v) {
        return vector_value(std::span<const double>(v.data(), v.size()));
    }
    [[nodiscard]] bio::Value optional_vec3_value(
        const std::optional<Vec3> &v) {
        return v.has_value() ? vec3_value(*v) : bio::Value{};
    }
    [[nodiscard]] bio::Value enabled_value(
        const std::array<bool, wc::kContactCount> &e) {
        bio::Value::Object out;
        for (std::size_t i = 0; i < wc::kContactCount; ++i)
            out.emplace_back(std::string(wc::kContactNames[i]),
                             bio::Value{e[i]});
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value patch_value(
        const wc::RiderContactPatchDiag &p) {
        bio::Value::Object out;
        out.emplace_back("in_platform", bio::Value{p.in_platform});
        out.emplace_back("normal_load_n", real_value(p.normal_load_n));
        out.emplace_back("gap_m", real_value(p.gap_m));
        out.emplace_back("point_m", vec3_value(p.point_m));
        out.emplace_back("force_on_rider_n",
                         vec3_value(p.force_on_rider_n));
        out.emplace_back("radial_energy_j",
                         real_value(p.radial_energy_j));
        out.emplace_back("shear_energy_j", real_value(p.shear_energy_j));
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value support_diag_value(
        const wc::RiderContactSupportDiag &d) {
        bio::Value::Object out;
        out.emplace_back("enabled", bio::Value{d.enabled});
        out.emplace_back("in_platform", bio::Value{d.in_platform});
        out.emplace_back("normal_load_n", real_value(d.normal_load_n));
        if (d.welded) {
            out.emplace_back("would_separate",
                             bio::Value{d.would_separate});
            out.emplace_back("would_slip", bio::Value{d.would_slip});
            out.emplace_back("tangent_n", real_value(d.tangent_n));
        }
        out.emplace_back("gap_m", real_value(d.gap_m));
        out.emplace_back("vertical_force_on_rider_n",
                         real_value(d.vertical_force_on_rider_n));
        if (d.detailed) {
            if (d.welded)
                out.emplace_back("tangent_force_n",
                                 vec3_value(d.tangent_force_vec));
            else
                out.emplace_back("tangent_force_n",
                                 real_value(d.tangent_force_scalar));
            bio::Value::Array patches;
            for (const auto &p: d.patches)
                patches.push_back(patch_value(p));
            out.emplace_back("patches", bio::Value{std::move(patches)});
            out.emplace_back("force_on_rider_n",
                             vec3_value(d.force_on_rider_n));
            out.emplace_back("force_on_bike_n",
                             vec3_value(d.force_on_bike_n));
            out.emplace_back("moment_about_rider_origin_nm",
                             vec3_value(d.moment_nm));
            out.emplace_back("radial_energy_j",
                             real_value(d.radial_energy_j));
            out.emplace_back("shear_energy_j",
                             real_value(d.shear_energy_j));
            out.emplace_back("relative_power_w",
                             real_value(d.relative_power_w));
        }
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value grip_side_diag_value(
        const wc::RiderContactGripSideDiag &d) {
        bio::Value::Object out;
        out.emplace_back("enabled", bio::Value{d.enabled});
        out.emplace_back("reachable", bio::Value{d.reachable});
        if (!d.welded) {
            out.emplace_back("overloaded", bio::Value{d.overloaded});
            out.emplace_back("trial_pair_force_n",
                             real_value(d.trial_pair_force_n));
            out.emplace_back("pair_force_limit_n",
                             optional_value(d.pair_force_limit_n));
            out.emplace_back("release_loss_j",
                             real_value(d.release_loss_j));
            out.emplace_back("shoulder_distance_m",
                             real_value(d.shoulder_distance_m));
            out.emplace_back("arm_reach_m", real_value(d.arm_reach_m));
        }
        out.emplace_back("hand_gap_m", real_value(d.hand_gap_m));
        if (d.welded && d.extended) {
            out.emplace_back("overloaded", bio::Value{d.overloaded});
            out.emplace_back("trial_pair_force_n",
                             real_value(d.trial_pair_force_n));
            out.emplace_back("pair_force_limit_n",
                             optional_value(d.pair_force_limit_n));
            out.emplace_back("release_loss_j",
                             real_value(d.release_loss_j));
            out.emplace_back("shoulder_distance_m",
                             real_value(d.shoulder_distance_m));
            out.emplace_back("arm_reach_m", real_value(d.arm_reach_m));
        }
        if (!d.welded || d.extended) {
            out.emplace_back("point_m", vec3_value(d.point_m));
            out.emplace_back("force_on_rider_n",
                             vec3_value(d.force_on_rider_n));
            out.emplace_back("force_on_bike_n",
                             vec3_value(d.force_on_bike_n));
            out.emplace_back("elastic_energy_j",
                             real_value(d.elastic_energy_j));
        }
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value grip_diag_value(
        const wc::RiderContactGripDiag &d) {
        bio::Value::Object out;
        out.emplace_back("enabled", bio::Value{d.enabled});
        out.emplace_back("reachable", bio::Value{d.reachable});
        if (!d.welded) {
            out.emplace_back("overloaded", bio::Value{d.overloaded});
            out.emplace_back("trial_pair_force_n",
                             real_value(d.trial_pair_force_n));
            out.emplace_back("pair_force_limit_n",
                             optional_value(d.pair_force_limit_n));
            out.emplace_back("release_loss_j",
                             real_value(d.release_loss_j));
            out.emplace_back("shoulder_distance_m",
                             real_value(d.shoulder_distance_m));
            out.emplace_back("arm_reach_m", real_value(d.arm_reach_m));
        }
        out.emplace_back("hand_gap_m", real_value(d.hand_gap_m));
        if (d.welded && d.extended) {
            out.emplace_back("overloaded", bio::Value{d.overloaded});
            out.emplace_back("trial_pair_force_n",
                             real_value(d.trial_pair_force_n));
            out.emplace_back("pair_force_limit_n",
                             optional_value(d.pair_force_limit_n));
            out.emplace_back("release_loss_j",
                             real_value(d.release_loss_j));
            out.emplace_back("shoulder_distance_m",
                             real_value(d.shoulder_distance_m));
            out.emplace_back("arm_reach_m", real_value(d.arm_reach_m));
        }
        if (!d.welded || d.extended) {
            out.emplace_back("force_on_rider_n",
                             vec3_value(d.force_on_rider_n));
            out.emplace_back("force_on_bike_n",
                             vec3_value(d.force_on_bike_n));
            out.emplace_back("elastic_energy_j",
                             real_value(d.elastic_energy_j));
        }
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value diagnostics_value(
        const wc::RiderContactsDiagnostics &d) {
        bio::Value::Object out;
        if (!d.evaluated) return bio::Value{std::move(out)};
        for (std::size_t i = 0; i < wc::kSupportCount; ++i)
            out.emplace_back(std::string(wc::kSupportNames[i]),
                             support_diag_value(d.supports[i]));
        for (std::size_t side = 0; side < wc::kSideCount; ++side)
            out.emplace_back(
                "grip_" + std::string(wc::kSideNames[side]),
                grip_side_diag_value(d.grip_sides[side]));
        out.emplace_back("grip", grip_diag_value(d.grip));
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value settled_entry_value(
        const wc::RiderSettledEntry &e) {
        bio::Value::Object out;
        out.emplace_back("force_on_rider_n",
                         vec3_value(e.force_on_rider_n));
        if (e.support_fields) {
            out.emplace_back("normal_n", real_value(e.normal_n));
            out.emplace_back("tangent_n", real_value(e.tangent_n));
            out.emplace_back("would_separate",
                             bio::Value{e.would_separate});
            out.emplace_back("would_slip", bio::Value{e.would_slip});
        }
        return bio::Value{std::move(out)};
    }
    [[nodiscard]] bio::Value settled_value(
        const std::optional<wc::RiderSettledWelds> &w) {
        if (!w.has_value()) return bio::Value{};
        bio::Value::Object welds;
        for (const auto &[name, entry]: w->entries)
            welds.emplace_back(name, settled_entry_value(entry));
        bio::Value::Array pair;
        pair.push_back(bio::Value{std::move(welds)});
        pair.push_back(real_value(w->crank_torque_nm));
        return bio::Value{std::move(pair)};
    }
    [[nodiscard]] bio::Value attachment_entry_value(
        const wc::RiderAttachmentEntry &entry) {
        return std::visit(
            [](const auto &v) -> bio::Value {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, AttachmentRaw>)
                    return raw_value(v);
                else
                    return sample_value(v);
            },
            entry);
    }

} // namespace

// ---- produced boundary -----------------------------------------------------

SoleGoalDiagnostics decode_sole_goal(const bio::Value &value, std::string_view path) {
    const Dict entry = mapping(VNode{&value}, path);
    SoleGoalDiagnostics out;
    out.spindle = false;
    if (entry.contains("spindle")) {
        ncd::exact(entry, keys("spindle", "saturated", "limiting_reasons"));
        out.spindle = entry["spindle"].boolean(path);
    } else if (entry.contains("weld") || entry.contains("recovery")) {
        const char *tag = entry.contains("weld") ? "weld" : "recovery";
        if (entry.contains("weld"))
            ncd::exact(entry, keys("weld", "saturated", "limiting_reasons"));
        else ncd::exact(entry, keys("recovery", "saturated", "limiting_reasons"));
        if (!entry[tag].boolean(path)) invalid(path, "goal topology tag must be true");
        out.weld = entry.contains("weld");
        out.recovery = entry.contains("recovery");
    } else {
        ncd::exact(entry, keys("requested_compression_m", "applied_compression_m",
            "requested_shear_m", "applied_shear_m", "saturated", "limiting_reasons"));
        out.geometry = SoleGoalDiagnostic{
            .requested_compression_m=entry["requested_compression_m"].finite_real(path),
            .applied_compression_m=entry["applied_compression_m"].finite_real(path),
            .requested_shear_m=entry["requested_shear_m"].finite_real(path),
            .applied_shear_m=entry["applied_shear_m"].finite_real(path), .saturated=false, .limiting_reasons={}};
    }
    out.saturated = entry["saturated"].boolean(path);
    const auto reasons = entry["limiting_reasons"].sequence(path);
    for (std::size_t i = 0; i < reasons.size(); ++i)
        out.limiting_reasons.push_back(reasons.at(i).string(path));
    if (out.geometry) {
        out.geometry->saturated = out.saturated;
        out.geometry->limiting_reasons = out.limiting_reasons;
    } else if ((out.weld || out.recovery) && (out.saturated || !out.limiting_reasons.empty()))
        invalid(path, "linked/recovery goals cannot have a geometry projection limit");
    return out;
}

bio::Value encode_sole_goal(const SoleGoalDiagnostics &goal) {
    bio::Value::Object entry;
    const auto count = static_cast<unsigned>(goal.weld) + static_cast<unsigned>(goal.recovery)
        + static_cast<unsigned>(goal.geometry.has_value());
    if (count > 1 || (count != 0 && goal.spindle)) throw std::invalid_argument("sole goal must select one topology");
    if (count == 0) entry.emplace_back("spindle", bio::Value{goal.spindle});
    if (goal.geometry) {
        const auto &g = *goal.geometry;
        entry.emplace_back("requested_compression_m", real_value(g.requested_compression_m));
        entry.emplace_back("applied_compression_m", real_value(g.applied_compression_m));
        entry.emplace_back("requested_shear_m", real_value(g.requested_shear_m));
        entry.emplace_back("applied_shear_m", real_value(g.applied_shear_m));
    }
    entry.emplace_back("saturated", bio::Value{goal.saturated});
    entry.emplace_back("limiting_reasons", names_value(goal.limiting_reasons));
    if (goal.weld) entry.emplace_back("weld", bio::Value{true});
    if (goal.recovery) entry.emplace_back("recovery", bio::Value{true});
    return bio::Value{std::move(entry)};
}

SpindleConfig decode_spindle_config(const bio::Value &value) {
    const std::string path = "config.rider_controller";
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, kConfigRequiredKeys);
    SpindleConfig config;
    decode_actuation_scalars(d, path, config);
    decode_geometry_scalars(d, path, config);
    validate_unmodeled_scalars(d, path);
    validate_attachments(d, path);
    decode_optional_path(d, "joint_strength_path", path);
    decode_optional_path(d, "joint_envelope_path", path);
    decode_strength_tables(d, path, config);
    if (d["pedal_attachment"].string(path) != "spindle" ||
        d["saddle_attachment"].string(path) != "pin" ||
        d["grip_attachment"].string(path) != "connect" ||
        d["coupled_task_control"].boolean(path)) {
        bio::Value::Object members;
        for (const auto& [key, member] : std::get<bio::Value::Object>(value.data))
            if (key != "strength" && key != "strength_coordinates" && key != "envelopes")
                members.emplace_back(key,member);
        config.legacy = decode_articulated_config(bio::Value{std::move(members)});
    }
    return config;
}

bike::configuration::ArticulatedConfig decode_articulated_config(const bio::Value& value) {
    return bike::configuration::detail::construct_config(
        bio::Value{bio::Value::Object{{"articulated",value}}}).articulated;
}

// ---- spindle controller -----------------------------------------------------

SpindlePose decode_spindle_pose(const bio::Value &value,
                                std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d,
                  keys("saddle", "hip", "shoulder", "elbow", "grip",
                       "head_center", "knee_front", "knee_rear",
                       "ankle_front", "ankle_rear", "pedal_front",
                       "pedal_rear"));
    SpindlePose pose;
    pose.hip = point3(d["hip"], path);
    pose.shoulder = point3(d["shoulder"], path);
    pose.elbow = point3(d["elbow"], path);
    pose.grip = point3(d["grip"], path);
    pose.knee_front = point3(d["knee_front"], path);
    pose.knee_rear = point3(d["knee_rear"], path);
    pose.ankle_front = point3(d["ankle_front"], path);
    pose.ankle_rear = point3(d["ankle_rear"], path);
    pose.pedal_front = point3(d["pedal_front"], path);
    pose.pedal_rear = point3(d["pedal_rear"], path);
    // saddle/head_center/ankle_* are validated present by the key gate but
    // unused by the spindle path.
    return pose;
}

RiderPosture decode_rider_posture(const bio::Value &value,
                                  std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d,
                  keys("torso_lean_rad", "pelvis_pitch_rad",
                       "pelvis_offset_m", "use_saddle"));
    RiderPosture posture;
    posture.torso_lean_rad = d["torso_lean_rad"].finite_real(path);
    posture.pelvis_pitch_rad = d["pelvis_pitch_rad"].finite_real(path);
    if (std::abs(posture.torso_lean_rad) > .8)
        invalid(path, "torso_lean_rad must lie in [-0.8, 0.8]");
    if (std::abs(posture.pelvis_pitch_rad) > .5)
        invalid(path, "pelvis_pitch_rad must lie in [-0.5, 0.5]");
    const VNode offset = d["pelvis_offset_m"];
    if (!offset.is_none()) {
        const auto v = fixed<2>(offset, path);
        if (std::abs(v[0]) > .25 || v[1] < -.15 || v[1] > .25)
            invalid(path,
                    "hip offset exceeds the posture command envelope");
        posture.pelvis_offset_m = spindle::Vec2{v[0], v[1]};
    }
    posture.use_saddle = d["use_saddle"].boolean(path);
    return posture;
}

RiderCommand decode_rider_command(const bio::Value &value,
                                  std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d,
                  keys("mean_crank_torque_nm", "enabled", "posture",
                       "crank_target_phase_rad", "crank_target_rate_rad_s"));
    RiderCommand command;
    command.mean_crank_torque_nm =
        d["mean_crank_torque_nm"].finite_real(path);
    if (command.mean_crank_torque_nm < 0.)
        invalid(path, "rider effort must be nonnegative");
    command.enabled = d["enabled"].boolean(path);
    command.posture =
        decode_rider_posture(*d["posture"].value, std::string(path) + ".posture");
    command.crank_target_phase_rad =
        d["crank_target_phase_rad"].optional_real(path);
    command.crank_target_rate_rad_s =
        d["crank_target_rate_rad_s"].finite_real(path);
    if (command.crank_target_phase_rad.has_value() &&
        command.mean_crank_torque_nm > 0.)
        invalid(path, "coasting goals cannot request pedaling effort");
    return command;
}

NamedEntries<double> decode_named_reals(const bio::Value &value,
                                        std::string_view path) {
    return decode_named_reals_node(VNode{&value}, path);
}

JointTerms decode_joint_terms(const bio::Value &value,
                              std::string_view path) {
    return decode_joint_terms_node(VNode{&value}, path);
}

SpindleController::State decode_spindle_state(const bio::Value &value,
                                              std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, kStateRequiredKeys);
    SpindleController::State state;
    state.enabled = d["enabled"].boolean(path);
    state.command_enabled = d["command_enabled"].boolean(path);
    state.active_state = vector(d["active_state"], path);
    state.activation_time_s = d["activation_time_s"].optional_real(path);
    decode_state_terms(d, path, state);
    decode_state_ik(d, path, state);
    decode_state_recovery(d, path, state);
    return state;
}

bio::Value encode_rider_posture(const RiderPosture &posture) {
    bio::Value::Object out;
    out.emplace_back("torso_lean_rad", real_value(posture.torso_lean_rad));
    out.emplace_back("pelvis_pitch_rad",
                     real_value(posture.pelvis_pitch_rad));
    if (posture.pelvis_offset_m.has_value()) {
        bio::Value::Array offset;
        offset.push_back(real_value((*posture.pelvis_offset_m)[0]));
        offset.push_back(real_value((*posture.pelvis_offset_m)[1]));
        out.emplace_back("pelvis_offset_m", bio::Value{std::move(offset)});
    } else {
        out.emplace_back("pelvis_offset_m", bio::Value{});
    }
    out.emplace_back("use_saddle", bio::Value{posture.use_saddle});
    return bio::Value{std::move(out)};
}

bio::Value encode_spindle_state(const SpindleController::State &state) {
    bio::Value::Object out;
    out.emplace_back("enabled", bio::Value{state.enabled});
    out.emplace_back("command_enabled",
                     bio::Value{state.command_enabled});
    out.emplace_back("active_state", vector_value(state.active_state));
    set_or_none(out, "activation_time_s", state.activation_time_s);
    out.emplace_back("last_terms", last_terms_value(state.last_terms));
    bio::Value::Object split;
    for(const auto&[name,pair]:state.pd_split)split.emplace_back(name,vector_value(pair));
    out.emplace_back("pd_split", bio::Value{std::move(split)});
    out.emplace_back("saturated_ik", named_bools_value(state.saturated_ik));
    out.emplace_back("ik_reach_limited",
                     named_bools_value(state.ik_reach_limited));
    out.emplace_back("joint_torques_nm",
                     named_reals_value(state.joint_torques_nm));
    out.emplace_back("joint_capacity_nm",
                     named_reals_value(state.joint_capacity_nm));
    set_or_none(out, "lean_limit_rad", state.lean_limit_rad);
    if(state.allocation_memory && state.allocation_memory->branch) {
        const auto b=*state.allocation_memory->branch;
        out.emplace_back("last_branch",bio::Value{bio::Value::Array{bio::Value{b[0]},bio::Value{b[1]}}});
    } else out.emplace_back("last_branch", bio::Value{});
    out.emplace_back("last_solution",state.allocation_memory && state.allocation_memory->solution ? vector_value(*state.allocation_memory->solution) : bio::Value{});
    bio::Value::Object recovery;
    for (const auto &[name, pedal]: state.pedal_recovery) {
        bio::Value::Object entry;
        entry.emplace_back("stage", bio::Value{pedal.stage});
        entry.emplace_back("direction", real_value(pedal.direction));
        entry.emplace_back("release_offset_x_m",
                           real_value(pedal.release_offset_x_m));
        recovery.emplace_back(name, bio::Value{std::move(entry)});
    }
    out.emplace_back("pedal_recovery", bio::Value{std::move(recovery)});
    out.emplace_back("effort_diagnostics",
                     effort_diagnostics_value(state.effort_diagnostics));
    out.emplace_back("allocation_diagnostics",
                     allocation_diagnostics_value(
                         state.allocation_diagnostics));
    out.emplace_back("support_diagnostics",
                     support_diagnostics_value(state.support_diagnostics));
    out.emplace_back("sole_goal_diagnostics",
                     sole_goal_diagnostics_value(
                         state.sole_goal_diagnostics));
    return bio::Value{std::move(out)};
}

// ---- intent stack -----------------------------------------------------------

IntentConfig decode_intent_config(const bio::Value &value,
                                  std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d,
                  keys("enabled", "period_s", "reaction_delay_s",
                       "target_crank_power_w", "max_crank_torque_nm",
                       "torque_slew_nm_s", "lean_gain",
                       "max_forward_lean_rad", "max_backward_lean_rad",
                       "lean_rate_rad_s", "orientation_tau_s",
                       "surge_power_w", "surge_grade", "surge_budget_s",
                       "surge_recovery_rate", "front_load_share_target",
                       "lean_trim_gain_rad_s", "lean_trim_limit_rad",
                       "trim_dead_time_s"));
    IntentConfig config;
    // SeatedClimbConfig.__post_init__ order: enabled, positive fields,
    // nonnegative fields, share bound, posture envelope bound.
    config.enabled = d["enabled"].boolean(path);
    struct Bound {
        const char *key;
        double IntentConfig::*field;
        bool positive;
    };
    const std::array<Bound, 18> bounds{{
        {.key = "period_s", .field = &IntentConfig::period_s,
         .positive = true},
        {.key = "target_crank_power_w",
         .field = &IntentConfig::target_crank_power_w, .positive = true},
        {.key = "max_crank_torque_nm",
         .field = &IntentConfig::max_crank_torque_nm, .positive = true},
        {.key = "torque_slew_nm_s", .field = &IntentConfig::torque_slew_nm_s,
         .positive = true},
        {.key = "lean_rate_rad_s", .field = &IntentConfig::lean_rate_rad_s,
         .positive = true},
        {.key = "orientation_tau_s", .field = &IntentConfig::orientation_tau_s,
         .positive = true},
        {.key = "surge_power_w", .field = &IntentConfig::surge_power_w,
         .positive = true},
        {.key = "surge_budget_s", .field = &IntentConfig::surge_budget_s,
         .positive = true},
        {.key = "reaction_delay_s", .field = &IntentConfig::reaction_delay_s,
         .positive = false},
        {.key = "lean_gain", .field = &IntentConfig::lean_gain,
         .positive = false},
        {.key = "max_forward_lean_rad",
         .field = &IntentConfig::max_forward_lean_rad, .positive = false},
        {.key = "max_backward_lean_rad",
         .field = &IntentConfig::max_backward_lean_rad, .positive = false},
        {.key = "surge_grade", .field = &IntentConfig::surge_grade,
         .positive = false},
        {.key = "surge_recovery_rate",
         .field = &IntentConfig::surge_recovery_rate, .positive = false},
        {.key = "front_load_share_target",
         .field = &IntentConfig::front_load_share_target,
         .positive = false},
        {.key = "lean_trim_gain_rad_s",
         .field = &IntentConfig::lean_trim_gain_rad_s, .positive = false},
        {.key = "lean_trim_limit_rad",
         .field = &IntentConfig::lean_trim_limit_rad, .positive = false},
        {.key = "trim_dead_time_s", .field = &IntentConfig::trim_dead_time_s,
         .positive = false},
    }};
    for (const auto &[key, field, positive]: bounds) {
        const double parsed = d[key].finite_real(path);
        if (positive ? parsed <= 0. : parsed < 0.)
            invalid(std::string(path) + "." + key,
                    "invalid value or out of range");
        config.*field = parsed;
    }
    if (config.front_load_share_target > 1.)
        invalid(path, "front load share target must not exceed one");
    if (std::max(config.max_forward_lean_rad,
                 config.max_backward_lean_rad) > .8)
        invalid(path, "seated lean exceeds the posture envelope");
    return config;
}

IntentSignals decode_intent_signals(const bio::Value &value,
                                    std::string_view path) {
    return decode_signals(VNode{&value}, path);
}

std::int64_t decode_intent_step(const bio::Value &value,
                                std::string_view path) {
    // rider_intent.py resolve(): `type(step) is not int` — a float-spelled
    // lexeme, bool or non-number rejects exactly like PyLong_CheckExact.
    const auto *const number =
        std::get_if<bio::Number>(&value.data);
    if (number == nullptr || !bio::is_integer_lexeme(number->lexeme))
        invalid(path, "expected integer");
    std::int64_t result = 0;
    const char *const end = std::to_address(number->lexeme.cend());
    const auto parsed =
        std::from_chars(number->lexeme.data(), end, result, 10);
    if (parsed.ec != std::errc() || parsed.ptr != end)
        invalid(path, "invalid value or out of range");
    return result;
}

IntentControl decode_intent_control(const bio::Value &value,
                                    std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d,
                  keys("motor_torque_nm", "motor_limit_nm",
                       "human_torque_nm", "crank_target_rate_rad_s",
                       "posture", "rider_enabled"));
    IntentControl control;
    // RideControl.__post_init__ order: the four optional nonneg reals,
    // posture type, rider_enabled bool.
    for (const auto &[key, field]:
         std::array<std::pair<const char *,
                              std::optional<double> IntentControl::*>,
                    4>{{{"motor_torque_nm", &IntentControl::motor_torque_nm},
                        {"motor_limit_nm", &IntentControl::motor_limit_nm},
                        {"human_torque_nm",
                         &IntentControl::human_torque_nm},
                        {"crank_target_rate_rad_s",
                         &IntentControl::crank_target_rate_rad_s}}}) {
        const auto parsed = d[key].optional_real(path);
        if (parsed.has_value() && *parsed < 0.)
            invalid(std::string(path) + "." + key,
                    "invalid value or out of range");
        control.*field = parsed;
    }
    const VNode posture = d["posture"];
    if (!posture.is_none())
        control.posture = decode_rider_posture(
            *posture.value, std::string(path) + ".posture");
    control.rider_enabled = d["rider_enabled"].boolean(path);
    return control;
}

RiderIntent::State decode_intent_state(const bio::Value &value,
                                       std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, keys("last_tick_step", "intent", "policy", "program"));
    RiderIntent::State state;
    state.last_tick_step =
        decode_intent_step(*d["last_tick_step"].value,
                         std::string(path) + ".last_tick_step");
    state.intent =
        decode_intent_block(d["intent"], std::string(path) + ".intent");
    state.policy =
        decode_intent_policy(d["policy"], std::string(path) + ".policy");
    state.program =
        decode_intent_program(d["program"], std::string(path) + ".program");
    return state;
}

bio::Value encode_intent_signals(const IntentSignals &signals) {
    bio::Value::Object out;
    out.emplace_back("pitch_rate_up_rad_s",
                     real_value(signals.pitch_rate_up_rad_s));
    out.emplace_back("specific_force_body_mps2",
                     vector_value(signals.specific_force_body_mps2));
    out.emplace_back("crank_rate_rad_s",
                     real_value(signals.crank_rate_rad_s));
    out.emplace_back("human_crank_torque_nm",
                     real_value(signals.human_crank_torque_nm));
    out.emplace_back("front_load_share",
                     optional_value(signals.front_load_share));
    return bio::Value{std::move(out)};
}

bio::Value encode_intent_control(const IntentControl &control) {
    bio::Value::Object out;
    out.emplace_back("motor_torque_nm",
                     optional_value(control.motor_torque_nm));
    out.emplace_back("motor_limit_nm",
                     optional_value(control.motor_limit_nm));
    out.emplace_back("human_torque_nm",
                     optional_value(control.human_torque_nm));
    out.emplace_back("crank_target_rate_rad_s",
                     optional_value(control.crank_target_rate_rad_s));
    out.emplace_back("posture",
                     control.posture.has_value()
                         ? encode_rider_posture(*control.posture)
                         : bio::Value{});
    out.emplace_back("rider_enabled", bio::Value{control.rider_enabled});
    return bio::Value{std::move(out)};
}

bio::Value encode_intent_state(const RiderIntent::State &state) {
    bio::Value::Object out;
    out.emplace_back("last_tick_step",
                     integer_value(state.last_tick_step));
    {
        bio::Value::Object intent;
        intent.emplace_back("posture",
                            encode_rider_posture(state.intent.posture));
        intent.emplace_back("effort_ceiling_nm",
                            real_value(state.intent.effort_ceiling_nm));
        out.emplace_back("intent", bio::Value{std::move(intent)});
    }
    {
        bio::Value::Object policy;
        policy.emplace_back("time_s", real_value(state.policy.time_s));
        policy.emplace_back("inclination_rad",
                            real_value(state.policy.inclination_rad));
        policy.emplace_back("lean_rad", real_value(state.policy.lean_rad));
        policy.emplace_back("effort_nm",
                            real_value(state.policy.effort_nm));
        bio::Value::Array samples;
        for (const auto &[time_s, signals]: state.policy.samples) {
            bio::Value::Array row;
            row.push_back(real_value(time_s));
            row.push_back(encode_intent_signals(signals));
            samples.push_back(bio::Value{std::move(row)});
        }
        policy.emplace_back("samples", bio::Value{std::move(samples)});
        policy.emplace_back("delayed",
                            state.policy.delayed.has_value()
                                ? encode_intent_signals(*state.policy.delayed)
                                : bio::Value{});
        policy.emplace_back("surge_budget_s_left",
                            real_value(state.policy.surge_budget_s_left));
        out.emplace_back("policy", bio::Value{std::move(policy)});
    }
    {
        bio::Value::Object program;
        program.emplace_back("lean_rad",
                             real_value(state.program.lean_rad));
        program.emplace_back("trim_rad",
                             real_value(state.program.trim_rad));
        bio::Value::Array load_samples;
        for (const auto &[time_s, share]: state.program.load_samples) {
            bio::Value::Array row;
            row.push_back(real_value(time_s));
            row.push_back(optional_value(share));
            load_samples.push_back(bio::Value{std::move(row)});
        }
        program.emplace_back("load_samples",
                             bio::Value{std::move(load_samples)});
        program.emplace_back("delayed_load_share",
                             optional_value(
                                 state.program.delayed_load_share));
        bio::Value::Array pulses;
        for (const auto &[start, duration, amplitude]:
             state.program.pulses) {
            bio::Value::Array pulse;
            pulse.push_back(real_value(start));
            pulse.push_back(real_value(duration));
            pulse.push_back(real_value(amplitude));
            pulses.push_back(bio::Value{std::move(pulse)});
        }
        program.emplace_back("pulses", bio::Value{std::move(pulses)});
        out.emplace_back("program", bio::Value{std::move(program)});
    }
    return bio::Value{std::move(out)};
}

// ---- attachment records -------------------------------------------------------

AttachmentGeometry decode_attachment_geometry(const bio::Value &value,
                                              std::string_view /*path*/) {
    return decode_geometry_doc(VNode{&value});
}

wc::RiderAttachmentEntry decode_attachment_entry(const bio::Value &value,
                                                 std::string_view path) {
    return decode_entry(VNode{&value}, std::string(path));
}

bio::Value encode_attachment_geometry(const AttachmentGeometry &g) {
    bio::Value::Object out;
    out.emplace_back("eq_id", integer_value(g.eq_id));
    out.emplace_back("rider_jac", matrix_value(g.rider_jac));
    out.emplace_back("rider_columns", index_vector_value(g.rider_columns));
    out.emplace_back("bike_jac", matrix_value(g.bike_jac));
    out.emplace_back("bike_columns", index_vector_value(g.bike_columns));
    out.emplace_back("observable", bio::Value{g.observable});
    out.emplace_back("normal", vector_value(g.normal));
    out.emplace_back("kind", bio::Value{g.kind});
    out.emplace_back("rotational", bio::Value{g.rotational});
    out.emplace_back("half_patch_m", real_value(g.half_patch_m));
    out.emplace_back("pull_direction",
                     optional_vector_value(g.pull_direction));
    return bio::Value{std::move(out)};
}

bio::Value encode_attachment_entry(const wc::RiderAttachmentEntry &entry) {
    return attachment_entry_value(entry);
}

// ---- rider contacts state -----------------------------------------------------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::pair<wc::RiderContactsState, std::optional<wc::RiderContactsProbe>>
decode_rider_contacts_state(const bio::Value &value,
                            const wc::RiderContactWriter &w) {
    const std::string root_path = "state.rider_contacts";
    const Dict root = mapping(VNode{&value}, root_path);
    constexpr auto required = keys(
        "enabled", "supports", "grip_xi_local", "grip_anchor_local",
        "elastic_energy_j", "loss_step_j", "radial_dissipation_power_w",
        "delivered_crank_torque_nm", "last_time_s",
        "pending_release_loss_j", "diagnostics", "settled_welds",
        "last_attachment_samples", "last_attachment_errors",
        "probe_diagnostics", "probe_enabled",
        "probe_delivered_crank_torque_nm");
    ncd::exact(root, required);
    wc::RiderContactsState st{};
    const auto real = [&root](const char *key) {
        return field(root, key).finite_real(root.child(key));
    };
    st.enabled = decode_enabled(field(root, "enabled"),
                                root.child("enabled"));
    {
        const Dict supports = section(root, "supports");
        constexpr auto pad_keys =
            keys("saddle:0", "saddle:1", "front_pedal:0", "front_pedal:1",
                 "rear_pedal:0", "rear_pedal:1");
        ncd::exact(supports, pad_keys);
        for (std::size_t key = 0; key < wc::kPadStateCount; ++key) {
            const std::string name =
                std::string(wc::kSupportNames[key / wc::kPadsPerSupport]) +
                ":" + std::to_string(key % wc::kPadsPerSupport);
            const Dict entry = mapping(supports[name],
                                       supports.child(name));
            ncd::exact(entry, keys("xi", "tangent"));
            auto &target = st.supports[key];
            target.xi = field(entry, "xi").finite_real(entry.child("xi"));
            const VNode tangent = field(entry, "tangent");
            target.tangent =
                tangent.is_none()
                    ? std::nullopt
                    : std::optional<Vec3>(
                          fixed<3>(tangent, entry.child("tangent")));
        }
    }
    constexpr std::array<const char *, wc::kSideCount> side_keys = {
        "left", "right"};
    {
        const Dict xi = section(root, "grip_xi_local");
        ncd::exact(xi, keys("left", "right"));
        for (std::size_t side = 0; side < wc::kSideCount; ++side)
            st.grip_xi_local[side] =
                fixed<3>(field(xi, side_keys[side]),
                         xi.child(side_keys[side]));
    }
    {
        const Dict anchors = section(root, "grip_anchor_local");
        ncd::exact(anchors, keys("left", "right"));
        for (std::size_t side = 0; side < wc::kSideCount; ++side) {
            const char *const name = side_keys[side];
            const VNode anchor = field(anchors, name);
            st.grip_anchor_local[side] =
                anchor.is_none()
                    ? std::nullopt
                    : std::optional<Vec3>(
                          fixed<3>(anchor, anchors.child(name)));
        }
    }
    st.elastic_energy_j = real("elastic_energy_j");
    st.loss_step_j = real("loss_step_j");
    st.radial_dissipation_power_w = real("radial_dissipation_power_w");
    st.delivered_crank_torque_nm = real("delivered_crank_torque_nm");
    st.last_time_s =
        field(root, "last_time_s").optional_real(root.child("last_time_s"));
    st.pending_release_loss_j = real("pending_release_loss_j");
    st.diagnostics =
        decode_diagnostics(field(root, "diagnostics"), w,
                           root.child("diagnostics"));
    st.settled_welds =
        decode_settled(field(root, "settled_welds"), w,
                       root.child("settled_welds"));
    {
        const Dict samples = section(root, "last_attachment_samples");
        samples.value.each_member(
            [&](std::string_view name, const VNode &item) {
                st.attachment_samples.emplace_back(
                    std::string(name),
                    decode_entry(item, samples.child(name)));
            });
    }
    {
        const auto errors =
            field(root, "last_attachment_errors")
                .sequence(root.child("last_attachment_errors"));
        for (std::size_t i = 0; i < errors.size(); ++i)
            st.attachment_errors.push_back(errors.at(i).string(
                root.child("last_attachment_errors")));
    }
    const VNode probe_diag = field(root, "probe_diagnostics");
    const VNode probe_enabled = field(root, "probe_enabled");
    const VNode probe_torque = field(root, "probe_delivered_crank_torque_nm");
    const bool any_probe = !probe_diag.is_none() ||
                           !probe_enabled.is_none() ||
                           !probe_torque.is_none();
    const bool all_probe = !probe_diag.is_none() &&
                           !probe_enabled.is_none() &&
                           !probe_torque.is_none();
    if (any_probe && !all_probe)
        invalid(root.path, "probe fields must arrive together");
    std::optional<wc::RiderContactsProbe> probe;
    if (all_probe) {
        probe.emplace();
        probe->diagnostics =
            decode_diagnostics(probe_diag, w,
                               root.child("probe_diagnostics"));
        probe->enabled = decode_enabled(probe_enabled,
                                        root.child("probe_enabled"));
        probe->delivered_crank_torque_nm = probe_torque.finite_real(
            root.child("probe_delivered_crank_torque_nm"));
    }
    return {std::move(st), std::move(probe)};
}
NATIVE_DIAG_POP

bio::Value encode_rider_contacts_state(
    const wc::RiderContactsState &s,
    const std::optional<wc::RiderContactsProbe> &probe) {
    bio::Value::Object out;
    out.emplace_back("enabled", enabled_value(s.enabled));
    {
        bio::Value::Object supports;
        for (std::size_t si = 0; si < wc::kSupportCount; ++si) {
            for (std::size_t pad = 0; pad < wc::kPadsPerSupport; ++pad) {
                const auto &state =
                    s.supports[wc::kPadsPerSupport * si + pad];
                bio::Value::Object entry;
                entry.emplace_back("xi", real_value(state.xi));
                entry.emplace_back("tangent",
                                   optional_vec3_value(state.tangent));
                supports.emplace_back(
                    std::string(wc::kSupportNames[si]) + ":" +
                        std::to_string(pad),
                    bio::Value{std::move(entry)});
            }
        }
        out.emplace_back("supports", bio::Value{std::move(supports)});
    }
    {
        bio::Value::Object xi;
        for (std::size_t side = 0; side < wc::kSideCount; ++side)
            xi.emplace_back(std::string(wc::kSideNames[side]),
                            vec3_value(s.grip_xi_local[side]));
        out.emplace_back("grip_xi_local", bio::Value{std::move(xi)});
    }
    {
        bio::Value::Object anchors;
        for (std::size_t side = 0; side < wc::kSideCount; ++side)
            anchors.emplace_back(
                std::string(wc::kSideNames[side]),
                optional_vec3_value(s.grip_anchor_local[side]));
        out.emplace_back("grip_anchor_local",
                         bio::Value{std::move(anchors)});
    }
    out.emplace_back("elastic_energy_j", real_value(s.elastic_energy_j));
    out.emplace_back("loss_step_j", real_value(s.loss_step_j));
    out.emplace_back("radial_dissipation_power_w",
                     real_value(s.radial_dissipation_power_w));
    out.emplace_back("delivered_crank_torque_nm",
                     real_value(s.delivered_crank_torque_nm));
    out.emplace_back("last_time_s", optional_value(s.last_time_s));
    out.emplace_back("pending_release_loss_j",
                     real_value(s.pending_release_loss_j));
    out.emplace_back("diagnostics", diagnostics_value(s.diagnostics));
    out.emplace_back("settled_welds", settled_value(s.settled_welds));
    {
        bio::Value::Object samples;
        for (const auto &[name, entry]: s.attachment_samples)
            samples.emplace_back(name, attachment_entry_value(entry));
        out.emplace_back("last_attachment_samples",
                         bio::Value{std::move(samples)});
    }
    {
        bio::Value::Array errors;
        for (const std::string &e: s.attachment_errors)
            errors.push_back(bio::Value{e});
        out.emplace_back("last_attachment_errors",
                         bio::Value{std::move(errors)});
    }
    out.emplace_back("probe_diagnostics",
                     probe.has_value()
                         ? diagnostics_value(probe->diagnostics)
                         : bio::Value{});
    out.emplace_back("probe_enabled",
                     probe.has_value() ? enabled_value(probe->enabled)
                                       : bio::Value{});
    out.emplace_back(
        "probe_delivered_crank_torque_nm",
        probe.has_value() ? real_value(probe->delivered_crank_torque_nm)
                          : bio::Value{});
    return bio::Value{std::move(out)};
}

} // namespace rider
