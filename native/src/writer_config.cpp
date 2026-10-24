// writer_config.cpp — the owned bike::io::Value → WriterConfig decode
// (task T07). The node below adapts the strict T05 AST onto the shared
// node protocol in writer_config.hpp — every scalar read throws
// std::invalid_argument spelling the wire:: readers' verbatim reasons,
// so a Value input and its nanobind-dict twin decode through identical
// checks. The Python-value semantics the readers mirror:
//
//   * a JSON integer lexeme is numbers.Integral; a float lexeme (even a
//     whole-valued one like 2.0) is not — is_integer_lexeme carries the
//     int-vs-float spelling distinction the wire contract needs.
//   * a bool member is not a real/integer input — the Number alternative
//     excludes it exactly like wire::numeric_boolean.
//   * "expected ordered sequence" is the shared sequence rejection — the
//     nb side rejects mappings/str/bytes and non-Sequence non-ndarray
//     values; a Value Array is the only ordered wire form.
//   * number_double's float-from-lexeme is CPython float() semantics
//     (correctly rounded; overflow -> inf, underflow -> signed zero),
//     equal to PyFloat_AsDouble on the spellings JSON admits.
//
// Allocation-failure contract: decode is a pure function — every
// rejection or bad_alloc returns without installing anything.
#include "writer_config.hpp"
#include "spatial/types.hpp"
#include "runtime/spatial_tire_factory.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <system_error>
#include <utility>
#include <variant>

namespace nativecfg::detail {
    // TU-local: the node protocol contract is by shape, so VNode and its
    // accessors need no linkage beyond this file.
    namespace {
        namespace vnode {
            // Mapping/sequence ops are caller-gated (the parsers check
            // is_mapping()/is-sequence first); a misrouted kind read is a
            // plain wrong-type diagnostic, never a silent default. Deref
            // of the wrong alternative is unreachable by contract.
            [[nodiscard]] inline const bike::io::Value::Object &object(
                const bike::io::Value &v) {
                return *std::get_if<bike::io::Value::Object>(&v.data);
            }
            [[nodiscard]] inline const bike::io::Value::Array &array(
                const bike::io::Value &v) {
                return *std::get_if<bike::io::Value::Array>(&v.data);
            }
            [[nodiscard]] inline const bike::io::Value &missing() {
                static const bike::io::Value placeholder{};
                return placeholder;
            }
        } // namespace vnode

    // The bike::io::Value flavor of the shared decode node.
    struct VNode {
        const bike::io::Value *value;

        [[nodiscard]] bool is_none() const {
            return bike::io::is_null(*value);
        }
        [[nodiscard]] bool is_mapping() const {
            return std::holds_alternative<bike::io::Value::Object>(
                value->data);
        }
        [[nodiscard]] bool contains(std::string_view key) const {
            return bike::io::find_member(vnode::object(*value), key) !=
                   nullptr;
        }
        [[nodiscard]] VNode member(std::string_view key) const {
            const bike::io::Value *const found =
                bike::io::find_member(vnode::object(*value), key);
            return {found != nullptr ? found : &vnode::missing()};
        }
        // const& — the sink is invoked once per member, so it must stay
        // invocable for the whole iteration (no forwarding).
        template <class F>
        void each_key(std::string_view /*path*/, const F &f) const {
            // Object keys are strings by construction — the nb side's
            // "expected string" non-string-key check has no analog.
            for (const auto &member: vnode::object(*value))
                f(member.first);
        }
        [[nodiscard]] bool boolean(std::string_view path) const {
            const auto *const flag = std::get_if<bool>(&value->data);
            if (flag == nullptr) invalid(path, "expected bool");
            return *flag;
        }
        [[nodiscard]] double real(std::string_view path) const {
            const auto *const number =
                std::get_if<bike::io::Number>(&value->data);
            if (number == nullptr)
                invalid(path, "expected finite real scalar");
            bool ok = false;
            const double result =
                bike::io::detail::float_from_lexeme(number->lexeme, ok);
            if (!ok) invalid(path, "expected finite real scalar");
            return result;
        }
        [[nodiscard]] double finite_real(std::string_view path) const {
            const double result = real(path);
            if (!std::isfinite(result))
                invalid(path, "expected finite real scalar");
            return result;
        }
        [[nodiscard]] std::optional<double> optional_real(
            std::string_view path) const {
            return is_none() ? std::nullopt
                             : std::optional(finite_real(path));
        }
        [[nodiscard]] std::int64_t integer(std::string_view path) const {
            const auto *const number =
                std::get_if<bike::io::Number>(&value->data);
            if (number == nullptr ||
                !bike::io::is_integer_lexeme(number->lexeme))
                invalid(path, "expected integer");
            std::int64_t result = 0;
            const char *const end =
                std::to_address(number->lexeme.cend());
            const auto parsed =
                std::from_chars(number->lexeme.data(), end, result, 10);
            if (parsed.ec != std::errc() || parsed.ptr != end)
                invalid(path, "invalid value or out of range");
            return result;
        }
        [[nodiscard]] int integer32(std::string_view path) const {
            const std::int64_t result = integer(path);
            if (result < std::numeric_limits<int>::min() ||
                result > std::numeric_limits<int>::max())
                invalid(path, "integer out of range");
            return static_cast<int>(result);
        }
        [[nodiscard]] std::string string(std::string_view path) const {
            const auto *const text = std::get_if<std::string>(&value->data);
            if (text == nullptr) invalid(path, "expected string");
            return *text;
        }
        [[nodiscard]] Seq<VNode> sequence(std::string_view path) const {
            if (!std::holds_alternative<bike::io::Value::Array>(value->data))
                invalid(path, "expected ordered sequence");
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
    } // namespace

} // namespace nativecfg::detail

namespace nativecfg {

    namespace {
        using Value = bike::io::Value;

        const Value::Object& closed_spatial_object(
            const Value& value, std::string_view path,
            std::span<const std::string_view> required,
            std::span<const std::string_view> optional = {}) {
            const auto& object = bike::io::as_object(value, path);
            std::set<std::string_view> seen;
            for (const auto& [key, child] : object) {
                (void)child;
                if (!seen.insert(key).second)
                    detail::invalid(std::string(path) + "." + key, "duplicate key");
                if (std::ranges::find(required, key) == required.end() &&
                    std::ranges::find(optional, key) == optional.end())
                    detail::invalid(std::string(path) + "." + key, "unknown key");
            }
            for (const auto key : required)
                if (bike::io::find_member(object, key) == nullptr)
                    detail::invalid(std::string(path) + "." + std::string(key), "missing required key");
            return object;
        }

        const Value& spatial_member(const Value::Object& object,
                                    std::string_view key, std::string_view path) {
            const auto* value = bike::io::find_member(object, key);
            if (value == nullptr)
                detail::invalid(std::string(path) + "." + std::string(key), "missing required key");
            return *value;
        }

        void spatial_digest(const Value& value, std::string_view path) {
            const auto& digest = bike::io::as_string(value, path);
            if (digest.size() != 64 || !std::ranges::all_of(digest, [](char c) {
                return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
            })) detail::invalid(path, "expected 64 lowercase hexadecimal characters");
        }
    } // namespace

    void validate_spatial_writer_envelope(const bike::io::Value& value) {
        constexpr auto required = detail::keys("schema", "spatial");
        constexpr auto sections = detail::keys("drive", "cruise", "suspension", "brake",
            "resistance", "tire", "rider_forces", "rider_contacts");
        const auto& input = bike::io::as_object(value, "config");
        // Select a strictly integer version before interpreting the new key set.
        if (bike::io::as_int64(spatial_member(input, "schema", "config"), "config.schema") !=
            bike::spatial::kSpatialWriterSchema)
            detail::invalid("config.schema", "expected spatial writer schema 3");
        const auto& root = closed_spatial_object(value, "config", required, sections);
        const auto& spatial = closed_spatial_object(spatial_member(root, "spatial", "config"),
            "config.spatial", detail::keys("spatial_contract", "root_kind",
                "topology_fingerprint", "terrain_sha256", "numeric_profile"));
        if (bike::io::as_int64(spatial_member(spatial, "spatial_contract", "config.spatial"),
                               "config.spatial.spatial_contract") != bike::spatial::kSpatialContract)
            detail::invalid("config.spatial.spatial_contract", "unsupported spatial contract");
        const auto& root_kind = bike::io::as_string(
            spatial_member(spatial, "root_kind", "config.spatial"), "config.spatial.root_kind");
        if (root_kind != "constrained" && root_kind != "free")
            detail::invalid("config.spatial.root_kind", "expected 'constrained' or 'free'");
        spatial_digest(spatial_member(spatial, "topology_fingerprint", "config.spatial"),
                       "config.spatial.topology_fingerprint");
        spatial_digest(spatial_member(spatial, "terrain_sha256", "config.spatial"),
                       "config.spatial.terrain_sha256");
        if (bike::io::as_string(spatial_member(spatial, "numeric_profile", "config.spatial"),
                                "config.spatial.numeric_profile").empty())
            detail::invalid("config.spatial.numeric_profile", "must not be empty");
        for (const auto name : sections)
            if (const auto* section = bike::io::find_member(root, name))
                (void)bike::io::as_object(*section, "config." + std::string(name));
        if (const auto* value_tire = bike::io::find_member(root, "tire")) {
            const auto& tire = closed_spatial_object(*value_tire, "config.tire",
                detail::keys("backend", "pneumatic_3d"));
            if (bike::io::as_string(spatial_member(tire, "backend", "config.tire"),
                                    "config.tire.backend") != "pneumatic_3d")
                detail::invalid("config.tire.backend", "schema3 requires 'pneumatic_3d'");
            (void)bike::io::as_object(spatial_member(tire, "pneumatic_3d", "config.tire"),
                                     "config.tire.pneumatic_3d");
        }
    }

    drivetrain::DrivePolicyConfig decode_policy_config(const bike::io::Value& value) {
        if (!std::holds_alternative<bike::io::Value::Object>(value.data))
            detail::invalid("config", "expected dict");
        return detail::decode_drive_policies(
            detail::Dict<detail::VNode>{.value = detail::VNode{&value}, .path = "config"}, {});
    }

    WriterConfig decode(const bike::io::Value &value) {
        const auto *const root =
            std::get_if<bike::io::Value::Object>(&value.data);
        if (root == nullptr)
            detail::invalid("config", "expected dict");
        if (root->empty())
            return {};
        const auto* schema = bike::io::find_member(*root, "schema");
        const auto* schema_number = schema == nullptr
            ? nullptr : std::get_if<bike::io::Number>(&schema->data);
        const bool schema3 = schema_number != nullptr && schema_number->lexeme == "3";
        if (schema3 || bike::io::find_member(*root, "spatial") != nullptr) {
            if (schema == nullptr) detail::invalid("config.schema", "missing required key");
            const auto version = bike::io::as_int64(*schema, "config.schema");
            if (version == bike::spatial::kSpatialWriterSchema) {
                validate_spatial_writer_envelope(value);
                const auto& tire=bike::io::as_object(spatial_member(*root,"tire","config"),"config.tire");
                (void)runtime::decode_spatial_tire_config(spatial_member(tire,"pneumatic_3d","config.tire"));
                // Shared producers retain their exact schema2 contracts.
                // Spatial tyre construction also needs an owned terrain and
                // bound model, and is performed by the runtime cold factory.
                bike::io::Value shared=value;
                auto& members=std::get<bike::io::Value::Object>(shared.data);
                std::erase_if(members,[](const auto& entry){return entry.first=="spatial" || entry.first=="tire";});
                for(auto& entry:members)if(entry.first=="schema")entry.second=bike::io::Value{bike::io::Number{"2"}};
                return detail::decode_root(detail::Dict<detail::VNode>{.value=detail::VNode{&shared},.path="config"});
            }
            // Planar1/2 continue through their unchanged closed-key decoder;
            // their historical rejection of a spatial member is preserved.
            if (version != 1 && version != 2)
                detail::invalid("config.schema", "unsupported schema " + std::to_string(version));
        }
        return detail::decode_root(
            detail::Dict<detail::VNode>{
                .value = detail::VNode{&value}, .path = "config"});
    }

} // namespace nativecfg
