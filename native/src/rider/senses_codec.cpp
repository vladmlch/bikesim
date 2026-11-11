// senses_codec.cpp — strict codecs for the rider senses channel.
//
// The readers share the shared node protocol (nativecfg::detail): the
// VNode below is the same TU-local copy state_codec.cpp carries — the
// node protocol is by shape, so each codec TU keeps its own and the
// "path: reason" diagnostics stay identical.
#include "senses_codec.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include "../writer_config.hpp"

namespace rider {
namespace {

    namespace bio = bike::io;
    namespace ncd = nativecfg::detail;

    using ncd::keys;

    // ---- the Value-side decode node ---------------------------------------
    // Twin of rider/state_codec.cpp's TU-local VNode — the node protocol
    // is by shape (nativecfg::ncd::Dict<N> drives it), so the copy stays
    // local to this TU and the codecs share only the generic readers.
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
            // The resolved {"$nonfinite": ...} capture tag — real()
            // admits the sentinel exactly like the binding's wire::real
            // passes a nonfinite Python float to the finite-only field
            // readers.
            if (const auto *const tagged =
                    std::get_if<bio::Tagged>(&value->data)) {
                if (tagged->kind == "float64" &&
                    tagged->fields.size() == 1 &&
                    tagged->fields.front().first == "value") {
                    const std::string &spelling =
                        tagged->fields.front().second;
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

    // The integer-vs-real spelling rules the capture layer relies on:
    // integral lexemes stay integers, everything else keeps its real
    // spelling (io/json's canonical emit already guarantees the '.0').
    [[nodiscard]] bio::Value real_value(double v) {
        std::array<char, 32> buffer{};
        const auto result = std::to_chars(buffer.data(),
                                        std::to_address(buffer.end()), v);
        if (result.ec != std::errc())
            throw std::runtime_error("rider senses float encode failed");
        std::string lexeme(buffer.data(), result.ptr);
        if (lexeme.find_first_of(".eE") == std::string::npos)
            lexeme += ".0";
        return bio::Value{bio::Number{std::move(lexeme)}};
    }
    [[nodiscard]] bio::Value integer_value(std::int64_t v) {
        return bio::Value{bio::Number{std::to_string(v)}};
    }

    // Decode one surface-label leaf: a registry name string or a
    // "path: reason" rejection — the enum IS the label domain.
    [[nodiscard]] SurfaceLabel label_field(const VNode &node,
                                           const std::string &path) {
        const std::string name = node.string(path);
        const std::optional<SurfaceLabel> label = surface_label(name);
        if (!label.has_value())
            invalid(path, "unknown surface label");
        return *label;
    }

} // namespace

SurfaceLabels::SurfaceLabels(SurfaceLabel default_label,
                             std::vector<SurfaceLabelSection> section_list)
    : surface(default_label), sections(std::move(section_list)) {
    std::ranges::stable_sort(sections, [](const auto &a, const auto &b) {
        return a.start_m < b.start_m;
    });
    // SurfaceMap ctor + validate's label-only twin: intervals stay sorted
    // and non-overlapping with finite, positive-width bounds.
    double previous = 0.;
    for (const auto &section : sections) {
        if (!std::isfinite(section.start_m) ||
            !std::isfinite(section.end_m) ||
            section.start_m >= section.end_m)
            throw std::invalid_argument(
                "SurfaceLabels.sections: invalid interval");
        if (section.start_m < previous)
            throw std::invalid_argument(
                "SurfaceLabels.sections: unordered or overlapping "
                "intervals");
        previous = section.end_m;
    }
}

SurfaceLabel SurfaceLabels::at(double x_m) const {
    if (!std::isfinite(x_m))
        throw std::invalid_argument("SurfaceLabels.x_m");
    for (const auto &section : sections)
        if (section.start_m <= x_m && x_m < section.end_m)
            return section.label;
    return surface;
}

RiderSenses decode_rider_senses(const bio::Value &value,
                                std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, keys("hand_press_n", "speed_mps", "rear_teeth",
                       "surface_front", "surface_ahead", "grade_ahead"));
    RiderSenses senses;
    senses.hand_press_n = d["hand_press_n"].finite_real(path);
    senses.speed_mps = d["speed_mps"].finite_real(path);
    const std::int64_t teeth = d["rear_teeth"].integer(path);
    // The strict reader's cog domain: a cassette never runs below 3
    // teeth, so 0/negative cogs reject here rather than downstream.
    if (teeth < 3 || teeth > std::numeric_limits<int>::max())
        invalid(std::string(path) + ".rear_teeth",
                "invalid value or out of range");
    senses.rear_teeth = static_cast<int>(teeth);
    senses.surface_front = label_field(d["surface_front"],
                                       std::string(path) + ".surface_front");
    senses.surface_ahead = label_field(d["surface_ahead"],
                                       std::string(path) + ".surface_ahead");
    senses.grade_ahead = d["grade_ahead"].finite_real(path);
    return senses;
}

SurfaceLabels decode_surface_labels(const bio::Value &value,
                                    std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, keys("surface", "sections"));
    const SurfaceLabel default_label =
        label_field(d["surface"], std::string(path) + ".surface");
    std::vector<SurfaceLabelSection> sections;
    const auto items = d["sections"].sequence(path);
    for (std::size_t i = 0; i < items.size(); ++i) {
        const Dict item = mapping(items.at(i),
                                  std::string(path) + ".sections");
        ncd::exact(item, keys("start_m", "end_m", "surface"));
        sections.push_back(SurfaceLabelSection{
            .start_m = item["start_m"].finite_real(path),
            .end_m = item["end_m"].finite_real(path),
            .label = label_field(item["surface"],
                                 std::string(path) + ".sections.surface")});
    }
    // The constructor re-runs the SurfaceMap ordering/overlap contract.
    return SurfaceLabels{default_label, std::move(sections)};
}

bio::Value encode_rider_senses(const RiderSenses &senses) {
    bio::Value::Object out;
    out.emplace_back("hand_press_n", real_value(senses.hand_press_n));
    out.emplace_back("speed_mps", real_value(senses.speed_mps));
    out.emplace_back("rear_teeth", integer_value(senses.rear_teeth));
    out.emplace_back("surface_front",
                     bio::Value{std::string(
                         surface_label_name(senses.surface_front))});
    out.emplace_back("surface_ahead",
                     bio::Value{std::string(
                         surface_label_name(senses.surface_ahead))});
    out.emplace_back("grade_ahead", real_value(senses.grade_ahead));
    return bio::Value{std::move(out)};
}

bio::Value encode_surface_labels(const SurfaceLabels &labels) {
    bio::Value::Object out;
    out.emplace_back("surface",
                     bio::Value{std::string(
                         surface_label_name(labels.surface))});
    bio::Value::Array sections;
    sections.reserve(labels.sections.size());
    for (const auto &section : labels.sections) {
        bio::Value::Object item;
        item.emplace_back("start_m", real_value(section.start_m));
        item.emplace_back("end_m", real_value(section.end_m));
        item.emplace_back("surface",
                          bio::Value{std::string(
                              surface_label_name(section.label))});
        sections.push_back(bio::Value{std::move(item)});
    }
    out.emplace_back("sections", bio::Value{std::move(sections)});
    return bio::Value{std::move(out)};
}

} // namespace rider
