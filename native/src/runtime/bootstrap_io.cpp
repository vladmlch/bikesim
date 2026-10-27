// bootstrap_io.cpp — read_bootstrap(): the filesystem face of the owned
// runtime bootstrap (task T09).
//
// Layout: <directory>/{config.json, state.json, model.mjb}. The JSON
// documents carry the capture codec's externalized tags
// (tools/migration/capture_reference.py::encode_tree):
//
//   {"$array": rel, "dtype": ..., "shape": [...], "order": "C",
//    "sha256": ...}        -> a (possibly nested) Value::Array of exact
//                            Number lexemes; nonfinite <f8 elements carry
//                            the recording sentinel as Tagged{"float64"}
//   {"$nonfinite": "nan"|"inf"|"-inf"} -> Tagged{"float64", value:...}
//   {"$tuple": [...]} / {"$set": [...]} -> Value::Array
//   {"$dataclass": fqname, "fields": {...}} -> the fields Object
//                            (member access == the binding's attribute
//                            reads on the materialized dataclass)
//   {"$bytes": rel, "sha256": ..., "bytes": n} -> Bytes payload
//   {"$path"|"$datetime"|"$date"|"$uuid": str} -> Tagged pass-through —
//                            scalars the readers reject like the
//                            non-str Python objects they materialized
//
// Every externalized payload is resolved relative to the capture root:
// the declared path is fixture-root-relative ("bootstrap/<name>/..."),
// so the resolver climbs the bootstrap directory's ancestors and takes
// the first ancestor under which it resolves to a regular file —
// absolute paths and ".." segments never resolve.
#include "bootstrap_io.hpp"

#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../writer_config.hpp"
#include "runtime.hpp"
#include "value_readers.hpp"
#include "../terrain/spec_io.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>
#include <mujoco/mujoco.h>

namespace runtime {
namespace {

namespace bio = bike::io;
namespace fs = std::filesystem;

[[noreturn]] void io_invalid(std::string_view field, std::string_view reason) {
    throw std::invalid_argument(std::string("read_bootstrap: ") +
                                std::string(field) + ": " +
                                std::string(reason));
}

[[nodiscard]] std::vector<std::byte> read_file(const fs::path &path,
                                              std::string_view field) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) io_invalid(field, "cannot open file");
    const std::streampos end = file.tellg();
    if (end < 0) io_invalid(field, "cannot size file");
    // ifstream reads char storage; the byte vector is a unit conversion,
    // not a reinterpretation of the caller's bytes.
    std::vector<char> raw(static_cast<std::size_t>(end));
    file.seekg(0);
    if (!raw.empty() &&
        !file.read(raw.data(), static_cast<std::streamsize>(raw.size())))
        io_invalid(field, "cannot read file");
    std::vector<std::byte> bytes(raw.size());
    std::ranges::copy(std::as_bytes(std::span{raw}), bytes.begin());
    return bytes;
}

// ---- the declared capture-root-relative path ------------------------------

// One capture tag's externalized payload lives at <capture-root>/<rel>;
// <directory> itself is <capture-root>/bootstrap/<name>, so climbing the
// ancestors resolves the convention without trusting it blindly. Only
// relative, ..-free declared paths can resolve; the first ancestor with a
// matching regular file wins.
// NOLINTBEGIN(bugprone-easily-swappable-parameters) path, declared name, diagnostic field — the caller pins the roles
[[nodiscard]] fs::path resolve_declared(const fs::path &directory,
                                        std::string_view declared,
                                        std::string_view field) {
    const fs::path rel{std::string(declared)};
    if (rel.empty() || rel.is_absolute())
        io_invalid(field, "external payload path must be relative");
    for (const auto &component : rel)
        if (component == "..")
            io_invalid(field, "external payload path escapes the root");
    for (fs::path base = fs::absolute(directory);; base = base.parent_path()) {
        const fs::path candidate = base / rel;
        std::error_code ec;
        if (fs::is_regular_file(candidate, ec)) return candidate;
        if (!base.has_parent_path() || base == base.root_path()) break;
    }
    io_invalid(field, "external payload path does not resolve under any "
                      "ancestor of the bootstrap directory");
    // NOLINTEND(bugprone-easily-swappable-parameters)
}

// ---- array payload decode --------------------------------------------------

struct ArraySpec {
    std::string dtype;
    std::vector<std::int64_t> shape;
    std::string order;
    std::string sha256;
    std::string relpath;
};

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) dtype tag + diagnostic field, roles pinned by callers
[[nodiscard]] std::size_t dtype_itemsize(std::string_view dtype,
                                         std::string_view field) {
    if (dtype == "<f8" || dtype == "<i8" || dtype == "<u8") return 8;
    if (dtype == "<i4") return 4;
    if (dtype == "|u1") return 1;
    io_invalid(field, "unsupported array dtype");
}

// LE byte payload -> exact lexeme-bearing Number (or a Tagged nonfinite
// sentinel for a nonfinite <f8 element). Integer lexemes spell the raw
// signed/unsigned decimal the Python int carried. `bytes` spans exactly
// one element (the caller subspans with the dtype itemsize).
[[nodiscard]] bio::Value element_value(std::string_view dtype,
                                       std::span<const std::byte> bytes,
                                       std::string_view field) {
    if (dtype == "<f8") {
        std::uint64_t bits = 0;
        for (std::size_t i = 0; i < 8; ++i)
            bits |= static_cast<std::uint64_t>(
                        static_cast<unsigned char>(bytes[i]))
                    << (8U * i);
        const double value = std::bit_cast<double>(bits);
        if (!std::isfinite(value)) {
            const char *spelling = std::isnan(value) ? "nan"
                                   : value > 0       ? "+inf"
                                                     : "-inf";
            return bio::Value{bio::Tagged{
                .kind = "float64", .fields = {{"value", spelling}}}};
        }
        // Shortest round-trip spelling — bitwise-identical to the payload.
        std::array<char, 32> buffer{};
        const auto written =
            std::to_chars(buffer.data(),
                          std::to_address(buffer.end()), value);
        if (written.ec != std::errc())
            io_invalid(field, "cannot encode an array element");
        std::string lexeme(buffer.data(),
                           static_cast<std::size_t>(written.ptr - buffer.data()));
        // Python float lexemes always carry a decimal point or exponent —
        // an integral float spells "x.0", never "x" (integer lexemes are
        // the int domain).
        if (lexeme.find_first_of(".eE") == std::string::npos)
            lexeme += ".0";
        return bio::Value{bio::Number{std::move(lexeme)}};
    }
    if (dtype == "<i8" || dtype == "<i4" || dtype == "<u8" ||
        dtype == "|u1") {
        std::uint64_t raw = 0;
        const std::size_t width = dtype_itemsize(dtype, field);
        for (std::size_t i = 0; i < width; ++i)
            raw |= static_cast<std::uint64_t>(
                       static_cast<unsigned char>(bytes[i]))
                   << (8U * i);
        std::string lexeme;
        if (dtype == "<i8" || dtype == "<i4") {
            const std::int64_t signed_value =
                width == 8
                    ? std::bit_cast<std::int64_t>(raw)
                    : static_cast<std::int64_t>(
                          std::bit_cast<std::int32_t>(
                              static_cast<std::uint32_t>(raw)));
            lexeme = std::to_string(signed_value);
        } else {
            lexeme = std::to_string(raw);
        }
        return bio::Value{bio::Number{std::move(lexeme)}};
    }
    io_invalid(field, "unsupported array dtype");
}

// Assemble the (possibly nested) C-order array tree: the last shape
// dimension varies fastest.
// NOLINTNEXTLINE(misc-no-recursion) recursion depth is the declared shape's rank
[[nodiscard]] bio::Value::Array array_level(
    const ArraySpec &spec, std::span<const std::byte> payload,
    std::size_t dim, std::size_t &offset, std::string_view field) {
    const std::size_t itemsize = dtype_itemsize(spec.dtype, field);
    const std::int64_t extent = spec.shape.at(dim);
    bio::Value::Array out;
    out.reserve(static_cast<std::size_t>(extent));
    for (std::int64_t i = 0; i < extent; ++i) {
        if (dim + 1 == spec.shape.size()) {
            out.push_back(element_value(
                spec.dtype, payload.subspan(offset, itemsize), field));
            offset += itemsize;
        } else {
            out.push_back(bio::Value{array_level(spec, payload, dim + 1,
                                                 offset, field)});
        }
    }
    return out;
}

[[nodiscard]] bio::Value resolve_array(const bio::Value::Object &tag,
                                       const fs::path &directory,
                                       std::string_view field) {
    ArraySpec spec;
    const auto member = [&tag, field](std::string_view key)
        -> const bio::Value & {
        const bio::Value *const found = bio::find_member(tag, key);
        if (found == nullptr)
            io_invalid(field, "external array tag missing a field");
        return *found;
    };
    spec.relpath = bio::as_string(member("$array"), field);
    spec.dtype = bio::as_string(member("dtype"), field);
    spec.order = bio::as_string(member("order"), field);
    spec.sha256 = bio::as_string(member("sha256"), field);
    for (const bio::Value &dim : bio::as_array(member("shape"), field)) {
        const std::int64_t extent = bio::as_int64(dim, field);
        if (extent < 0)
            io_invalid(field, "negative array shape extent");
        spec.shape.push_back(extent);
    }
    if (spec.order != "C")
        io_invalid(field, "unsupported array order");
    const std::size_t itemsize = dtype_itemsize(spec.dtype, field);
    std::size_t count = 1;
    for (const std::int64_t extent : spec.shape) {
        if (count >
            std::numeric_limits<std::size_t>::max() /
                static_cast<std::size_t>(extent == 0 ? 1 : extent))
            io_invalid(field, "array shape overflows");
        count *= static_cast<std::size_t>(extent);
    }
    const fs::path payload_path =
        resolve_declared(directory, spec.relpath, field);
    const std::vector<std::byte> payload = read_file(payload_path, field);
    if (count > std::numeric_limits<std::size_t>::max() / itemsize ||
        payload.size() != count * itemsize)
        io_invalid(field, "external array payload size does not match "
                          "dtype and shape");
    if (bio::sha256(payload) != spec.sha256)
        io_invalid(field, "external array payload digest does not match");
    if (spec.shape.empty()) {
        if (payload.size() != itemsize)
            io_invalid(field, "scalar array payload size mismatch");
        return element_value(spec.dtype, std::span{payload}, field);
    }
    std::size_t offset = 0;
    return bio::Value{array_level(spec, payload, 0, offset, field)};
}

// ---- recursive tag resolution ----------------------------------------------

[[nodiscard]] bool tagged_member_set(const bio::Value::Object &object,
                                     std::string_view tag,
                                     std::initializer_list<std::string_view>
                                         extra) {
    if (bio::find_member(object, tag) == nullptr) return false;
    for (const auto &[key, item] : object) {
        if (key == tag) continue;
        if (std::ranges::find(extra, key) == extra.end())
            return false;
    }
    return true;
}

// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the document shape (bounded by parse depth)
bio::Value resolve_tags(const bio::Value &value, const fs::path &directory,
                        std::string_view field) {
    if (const auto *object = std::get_if<bio::Value::Object>(&value.data)) {
        // Tag dispatch runs on the member set — a document carrying a
        // '$'-keyed member that matches a tag shape decodes; anything else
        // stays an ordinary object whose members resolve recursively.
        if (tagged_member_set(*object, "$array",
                              {"dtype", "shape", "order", "sha256"}))
            return resolve_array(*object, directory, field);
        if (tagged_member_set(*object, "$nonfinite", {})) {
            const std::string &spelling =
                bio::as_string(*bio::find_member(*object, "$nonfinite"),
                               field);
            if (spelling == "nan" || spelling == "inf" ||
                spelling == "+inf" || spelling == "-inf")
                return bio::Value{bio::Tagged{
                    .kind = "float64",
                    .fields = {{"value",
                                spelling == "inf" ? "+inf" : spelling}}}};
            io_invalid(field, "unsupported nonfinite sentinel spelling");
        }
        if (tagged_member_set(*object, "$tuple", {}) ||
            tagged_member_set(*object, "$set", {})) {
            const char *key = tagged_member_set(*object, "$tuple", {})
                                  ? "$tuple"
                                  : "$set";
            const auto &items =
                bio::as_array(*bio::find_member(*object, key), field);
            bio::Value::Array out;
            out.reserve(items.size());
            for (const bio::Value &item : items)
                out.push_back(resolve_tags(item, directory, field));
            return bio::Value{std::move(out)};
        }
        if (tagged_member_set(*object, "$dataclass", {"fields"})) {
            const bio::Value &fields =
                *bio::find_member(*object, "fields");
            // The dataclass's member map — decoded recursively like every
            // ordinary object (the "$dataclass" name itself does not
            // survive; the readers' member access never consults it).
            return resolve_tags(fields, directory, field);
        }
        if (tagged_member_set(*object, "$bytes", {"sha256", "bytes"})) {
            const std::string &rel =
                bio::as_string(*bio::find_member(*object, "$bytes"), field);
            const std::string &digest =
                bio::as_string(*bio::find_member(*object, "sha256"), field);
            const std::int64_t declared =
                bio::as_int64(*bio::find_member(*object, "bytes"), field);
            const fs::path payload_path =
                resolve_declared(directory, rel, field);
            std::vector<std::byte> payload = read_file(payload_path, field);
            if (declared < 0 || payload.size() !=
                                    static_cast<std::size_t>(declared))
                io_invalid(field, "external bytes payload size mismatch");
            if (bio::sha256(payload) != digest)
                io_invalid(field, "external bytes payload digest does not "
                                  "match");
            return bio::Value{bio::Bytes{std::move(payload)}};
        }
        for (const char *scalar_tag :
             {"$path", "$datetime", "$date", "$uuid"}) {
            if (tagged_member_set(*object, scalar_tag, {})) {
                const std::string &spelling = bio::as_string(
                    *bio::find_member(*object, scalar_tag), field);
                return bio::Value{bio::Tagged{
                    .kind = std::string(scalar_tag).substr(1),
                    .fields = {{"value", spelling}}}};
            }
        }
        bio::Value::Object out;
        out.reserve(object->size());
        for (const auto &[key, item] : *object)
            out.emplace_back(key, resolve_tags(item, directory, field));
        return bio::Value{std::move(out)};
    }
    if (const auto *array = std::get_if<bio::Value::Array>(&value.data)) {
        bio::Value::Array out;
        out.reserve(array->size());
        for (const bio::Value &item : *array)
            out.push_back(resolve_tags(item, directory, field));
        return bio::Value{std::move(out)};
    }
    return value;
}

} // namespace

bike::io::Value resolve_bootstrap_document(const bike::io::Value &document,
                                           const fs::path &directory) {
    return resolve_tags(document, directory, "document");
}

namespace {
// Spatial identity/tier validation — extracted so compose_bootstrap stays
// under the stack-frame limit.
void validate_spatial_bootstrap(RuntimeConfig& config, const BootstrapState& state,
                                const mjModel& image_model) {
        if (!state.spatial || !state.tire3d)
            io_invalid("state.spatial.tire", "producer state missing");
        if (!config.terrain3d)
            config.terrain3d = bike::terrain::detail::load_terrain3d_source(config.spatial->terrain); // NOLINT(bugprone-unchecked-optional-access) the caller guards config.spatial
        const auto topology = bike::spatial::bind_topology(image_model, *config.spatial); // NOLINT(bugprone-unchecked-optional-access) the caller guards config.spatial
        validate_spatial_identity(config, *state.spatial, topology);
        if (state.tire3d->identity.tier != config.spatial->tyre.tier) // NOLINT(bugprone-unchecked-optional-access) the caller guards config.spatial
            io_invalid("state.tire.identity.tier", "differs from configured tyre tier");
}

// decode_bootstrap_state returns ~5.6 KiB by value; materializing that
// prvalue in compose_bootstrap's frame trips the 8 KiB frame budget, so the
// temp lives in this non-inlined helper and move-assigns into the caller's
// state.
[[gnu::noinline]] void decode_bootstrap_state_into(BootstrapState &dst,
                                 const bike::io::Value &state,
                                 const mjModel &image_model,
                                 const RuntimeConfig &config) {
    dst = decode_bootstrap_state(state, image_model);
    validate_schema_family(config, dst.schema);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) compose arguments, the roles are pinned by the API
// compose_bootstrap's counted frame is dominated by the 6.6 KiB sret return
// slot, so the body runs in this non-inlined helper where the parameters are
// reference slots: the wrapper then holds only the return object plus four
// references.
[[gnu::noinline]] void compose_bootstrap_into(RuntimeBootstrap &out, RuntimeConfig &&config,
                                              ModelImage &&model, const bike::io::Value &state) {
    // The nanobind ctor's validation order, verbatim — the config envelope
    // decode ran at the call site (ordinal position 1). Here:
    // state.model_digest read, artifact digest comparison, the schema-2
    // writer-config decode, THEN the model-bound state sections (which
    // needed the loaded model — the owned image loads a disposable model
    // here purely for validation; the runtime's Stepper compiles its own
    // from the same bytes).
    out.config = std::move(config);
    // Select the family before loading an engine or decoding a writer. The
    // planar branch below retains its historical decode order.
    if (std::cmp_equal(out.config.schema, bike::spatial::kSpatialRuntimeSchema) || out.config.spatial) {
        const auto& state_object = bio::as_object(state, "state");
        const auto* state_version = bio::find_member(state_object, "schema");
        if (!state_version) io_invalid("state.schema", "missing required key");
        const auto version = bio::as_int64(*state_version, "state.schema");
        if (version < 1 || version > 2) io_invalid("state.schema", "unsupported bootstrap schema");
        validate_schema_family(out.config, static_cast<int>(version));
    }
    const std::string expected = expected_model_digest(state);
    if (bio::sha256(model.mjb) != expected)
        throw std::invalid_argument(
            "state.model_digest: model artifact bytes do not match");
    if (!out.config.spatial) static_cast<void>(nativecfg::decode(out.config.writer_config));
    const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> image_model(
        engine::load_model_image(model.mjb),
        &mj_deleteModel);
    if (image_model == nullptr)
        throw std::invalid_argument(
            "model.mjb: failed to load the model image");
    decode_bootstrap_state_into(out.state, state, *image_model, out.config);
    if (out.config.spatial) validate_spatial_bootstrap(out.config, out.state, *image_model);
    out.model = std::move(model);
}
} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) compose arguments, the roles are pinned by the API
RuntimeBootstrap compose_bootstrap(RuntimeConfig config, ModelImage model,
                                   const bike::io::Value &state) {
    RuntimeBootstrap out;
    compose_bootstrap_into(out, std::move(config), std::move(model), state);
    return out;
}

RuntimeBootstrap read_bootstrap(const fs::path &directory) {
    std::error_code ec;
    if (!fs::is_directory(directory, ec))
        io_invalid(directory.string(), "not a bootstrap directory");
    const auto text = [](const std::vector<std::byte> &bytes) {
        std::string out;
        out.reserve(bytes.size());
        for (const std::byte unit : bytes)
            out.push_back(std::to_integer<char>(unit));
        return out;
    };
    // The same ordinal order as the adapter path: the config envelope
    // decodes before the state document's digest extraction or the
    // artifact read can fail.
    const bio::Value config_doc = resolve_tags(
        bio::parse_json(text(read_file(directory / "config.json",
                                     "config.json"))),
        directory, "config");
    RuntimeConfig config = decode_runtime_config(config_doc);
    const bio::Value state_doc = resolve_tags(
        bio::parse_json(text(read_file(directory / "state.json",
                                     "state.json"))),
        directory, "state");
    ModelImage model;
    model.mjb = read_file(directory / "model.mjb", "model.mjb");
    model.sha256 = bio::sha256(model.mjb);
    return compose_bootstrap(std::move(config), std::move(model), state_doc);
}

} // namespace runtime
