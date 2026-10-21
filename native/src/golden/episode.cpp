#include "episode.hpp"
#include "model_image.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include "../engine_call.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>

namespace bike::golden {
namespace {
using io::Value;
[[noreturn]] void bad(std::string_view message) {
    throw std::invalid_argument("golden episode: " + std::string(message));
}
std::string real_text(double value) {
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value < 0 ? "-inf" : "inf";
    std::array<char, 64> text{};
    const auto result = std::to_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{}) bad("number formatting failed");
    std::string out(text.data(), result.ptr);
    if (out.find_first_of(".eE") == std::string::npos) out += ".0";
    return out;
}
Value real(double value) {
    return std::isfinite(value) ? Value{io::Number{real_text(value)}} : Value{real_text(value)};
}
Value leaf_value(const Leaf &leaf) {
    return std::visit([](const auto &value) -> Value {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, double>) return real(value);
        else if constexpr (std::is_same_v<T, std::int64_t>) return Value{io::Number{std::to_string(value)}};
        else return Value{value};
    }, leaf);
}
Leaf scalar(const Value &value) {
    if (const auto *number = std::get_if<io::Number>(&value.data)) {
        if (io::is_integer_lexeme(number->lexeme)) return io::number_i64(*number, "episode leaf");
        return io::number_double(*number, "episode leaf");
    }
    if (const auto *text = std::get_if<std::string>(&value.data)) return *text;
    if (const auto *boolean = std::get_if<bool>(&value.data)) return *boolean;
    if (io::is_null(value)) return std::monostate{};
    bad("expected a scalar row leaf");
}
void set(Value::Object &object, std::string key, Value value) {
    for (auto &item : object) if (item.first == key) { item.second = std::move(value); return; }
    object.emplace_back(std::move(key), std::move(value));
}
void set(Row &row, std::string key, Leaf value) {
    for (auto &item : row) if (item.first == key) { item.second = std::move(value); return; }
    row.emplace_back(std::move(key), std::move(value));
}
const Value *member(const Value &value, std::string_view key) {
    return io::find_member(io::as_object(value, "episode object"), key);
}
const io::NpyArray *find(const io::NpzArchive &archive, std::string_view key) {
    for (const auto &item : archive) if (item.first == key) return &item.second;
    return nullptr;
}
const io::NpyArray &required(const io::NpzArchive &archive, std::string_view key) {
    if (const auto *value = find(archive, key)) return *value;
    bad("missing NPZ member " + std::string(key));
}
std::vector<std::string> string_vector(const io::NpyArray &array, std::string_view name) {
    if (array.shape.size() != 1) bad(std::string(name) + " must be a vector");
    return array.strings();
}
void unique(const std::vector<std::string> &names, std::string_view name) {
    const std::set<std::string> distinct(names.begin(), names.end());
    if (distinct.size() != names.size()) bad(std::string(name) + " contains duplicate names");
}
Value array_of_strings(const std::vector<std::string> &names) {
    Value::Array result;
    for (const auto &name : names) result.push_back(Value{name});
    return Value{std::move(result)};
}
std::size_t dimension(const io::NpzArchive &initial, std::string_view key) {
    const auto *array = find(initial, key);
    if (!array) return 0;
    if (array->shape.size() != 1) bad("initial state must be a vector");
    return array->shape.front();
}
io::NpyArray optional_array(const io::NpzArchive &archive, std::string_view key,
                            std::vector<std::size_t> shape) {
    if (const auto *array = find(archive, key)) return *array;
    return io::double_array(std::span<const double>{}, std::move(shape));
}
void flatten(const Value &value, const std::string &path, Row &out) {
    if (const auto *object = std::get_if<Value::Object>(&value.data)) {
        for (const auto &[name, child] : *object) flatten(child, path.empty() ? name : path + '.' + name, out);
    } else if (const auto *array = std::get_if<Value::Array>(&value.data)) {
        for (std::size_t i = 0; i < array->size(); ++i) flatten((*array)[i], path + '.' + std::to_string(i), out);
    } else set(out, path, scalar(value));
}
void flatten_storage(const Value &value, const std::string &path, Value::Object &out) {
    if (const auto *object = std::get_if<Value::Object>(&value.data)) {
        for (const auto &[name, child] : *object) flatten_storage(child, path.empty() ? name : path + '.' + name, out);
    } else set(out, path, value);
}
std::optional<double> numeric(const Leaf &leaf) {
    if (const auto *value = std::get_if<double>(&leaf)) return *value;
    if (const auto *value = std::get_if<std::int64_t>(&leaf)) return static_cast<double>(*value);
    if (const auto *value = std::get_if<bool>(&leaf)) return *value ? 1. : 0.;
    const auto *value = std::get_if<std::string>(&leaf);
    if (!value) return std::nullopt;
    std::string_view token(*value);
    constexpr std::string_view whitespace = " \t\r\n\v\f";
    const auto first = token.find_first_not_of(whitespace);
    if (first == std::string_view::npos) return std::nullopt;
    token = token.substr(first, token.find_last_not_of(whitespace) - first + 1);
    bool negative = false;
    if (token.starts_with('+') || token.starts_with('-')) {
        negative = token.front() == '-'; token.remove_prefix(1);
    }
    std::string lower;
    for (const auto c : token) lower.push_back(c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c);
    if (lower == "nan") return std::numeric_limits<double>::quiet_NaN();
    if (lower == "inf" || lower == "infinity")
        return negative ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
    // CPython float accepts digit separators but rejects C spellings such
    // as nan(payload). Gate its decimal grammar before from_chars; do not
    // allow an implementation-specific float parser to broaden equality.
    std::string decimal = negative ? "-" : "";
    std::size_t position = 0;
    const auto digits = [&]() {
        std::size_t count = 0;
        while (position < token.size()) {
            const auto c = token[position];
            if (c >= '0' && c <= '9') { decimal.push_back(c); ++position; ++count; }
            else if (c == '_' && count > 0 && position + 1 < token.size() &&
                     token[position + 1] >= '0' && token[position + 1] <= '9') { ++position; }
            else break;
        }
        return count;
    };
    auto count = digits();
    if (position < token.size() && token[position] == '.') { decimal.push_back('.'); ++position; count += digits(); }
    if (count == 0) return std::nullopt;
    if (position < token.size() && (token[position] == 'e' || token[position] == 'E')) {
        decimal.push_back('e'); ++position;
        if (position < token.size() && (token[position] == '+' || token[position] == '-')) decimal.push_back(token[position++]);
        if (digits() == 0) return std::nullopt;
    }
    if (position != token.size()) return std::nullopt;
    bool valid = false;
    const auto result = io::detail::float_from_lexeme(decimal, valid);
    return valid ? std::optional(result) : std::nullopt;
}
bool close(const Leaf &a, const Leaf &b, double rtol, double atol) {
    const auto x = numeric(a), y = numeric(b);
    if (x && y) return *x == *y || (std::isnan(*x) && std::isnan(*y)) ||
        (std::isfinite(*x) && std::isfinite(*y) && std::abs(*x - *y) <= atol + rtol * std::abs(*y));
    return a == b;
}
std::string display(const Leaf &leaf) {
    return std::visit([](const auto &value) -> std::string {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::monostate>) return "None";
        else if constexpr (std::is_same_v<T, bool>) return value ? "True" : "False";
        else if constexpr (std::is_same_v<T, double>) return real_text(value);
        else if constexpr (std::is_same_v<T, std::string>) return value;
        else return std::to_string(value);
    }, leaf);
}
void tolerances(double rtol, double atol) {
    if (!std::isfinite(rtol) || !std::isfinite(atol) || rtol < 0 || atol < 0)
        bad("comparison tolerances must be finite and nonnegative");
}
const Leaf &get(const Row &row, std::string_view name) {
    for (const auto &item : row) if (item.first == name) return item.second;
    static const Leaf missing = std::string("<missing>");
    return missing;
}
void write(const std::filesystem::path &path, std::span<const std::byte> bytes) {
    platform::write_exclusive(path, bytes);
}
std::string read_text(const std::filesystem::path &path) {
    const auto bytes = io::read_numpy_bytes(path);
    std::string result;
    result.reserve(bytes.size());
    for (const auto byte : bytes) result.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
    return result;
}
Value read_manifest(const std::filesystem::path &path) {
    // Python json.dumps permits these three non-JSON constants inside
    // list-valued non_numeric channels. Preserve their comparison meaning
    // using the same portable sentinels as scalar flatten_row. Only whole
    // unquoted value tokens are accepted; the ordinary strict JSON reader
    // still validates all structure, escaping and duplicate keys.
    const auto input = read_text(path);
    std::string normalized;
    normalized.reserve(input.size());
    bool quoted = false, escaped = false;
    constexpr std::array<std::pair<std::string_view, std::string_view>, 3> constants{{
        {"-Infinity", "\"-inf\""}, {"Infinity", "\"inf\""}, {"NaN", "\"nan\""}}};
    for (std::size_t i = 0; i < input.size();) {
        const char current = input[i];
        if (quoted) {
            normalized.push_back(current); ++i;
            if (escaped) escaped = false;
            else if (current == '\\') escaped = true;
            else if (current == '"') quoted = false;
            continue;
        }
        if (current == '"') { quoted = true; normalized.push_back(current); ++i; continue; }
        bool replaced = false;
        for (const auto &[token, sentinel] : constants) {
            if (!std::string_view(input).substr(i).starts_with(token)) continue;
            const auto end = i + token.size();
            if (end != input.size() && std::string_view(" \t\r\n,]}").find(input[end]) == std::string_view::npos) continue;
            normalized += sentinel; i = end; replaced = true; break;
        }
        if (!replaced) { normalized.push_back(current); ++i; }
    }
    return io::parse_json(normalized);
}
std::vector<io::Value> storage_rows(const EpisodeArtifact &episode) {
    if (!episode.stored_rows.empty()) {
        if (episode.stored_rows.size() != episode.rows.size()) bad("stored/comparison row counts differ");
        return episode.stored_rows;
    }
    std::vector<Value> result;
    for (const auto &row : episode.rows) {
        Value::Object value;
        for (const auto &[name, leaf] : row) value.emplace_back(name, leaf_value(leaf));
        result.push_back(Value{std::move(value)});
    }
    return result;
}
io::NpyArray normalized(const io::NpyArray &array, std::vector<std::size_t> shape) {
    if (array.dtype.empty()) return io::double_array(std::span<const double>{}, std::move(shape));
    array.validate();
    return array;
}
} // namespace

Row flatten_row(const io::Value &row) { Row out; flatten(row, "", out); return out; }
io::Value storage_row(const io::Value &row) { Value::Object out; flatten_storage(row, "", out); return Value{std::move(out)}; }

std::vector<std::byte> detail::model_image(const mjModel &model) {
    struct SizeCall { const mjModel *model{}; mjtSize bytes = 0; } size{.model=&model};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void *raw) noexcept {
        auto &call = *static_cast<SizeCall *>(raw);
        call.bytes = mj_sizeModel(call.model);
    }, &size, error)) engine::throw_failure(error);
    if (size.bytes <= 0 || std::cmp_greater(size.bytes, io::kMaxNumpyBytes)) bad("invalid model image size");
    std::vector<std::byte> image(static_cast<std::size_t>(size.bytes));
    struct SaveCall { const mjModel *model; void *bytes; int size; }
        call{.model=&model, .bytes=image.data(), .size=static_cast<int>(size.bytes)};
    if (!engine::invoke([](void *raw) noexcept {
        const auto &save_call = *static_cast<SaveCall *>(raw);
        mj_saveModel(save_call.model, nullptr, save_call.bytes, save_call.size);
    }, &call, error)) engine::throw_failure(error);
    return image;
}

void save(const EpisodeArtifact &episode, const std::filesystem::path &out_dir, const mjModel *model) {
    // Assemble/validate every payload before publishing any names. Existing
    // artifacts are never silently overwritten by a freeze invocation.
    unique(episode.channel_names, "channel_names");
    unique(episode.force_names, "force_names");
    unique(episode.tire_state_names, "tire_state_names");
    auto rows = storage_rows(episode);
    const auto width = episode.channel_names.size();
    if (width && rows.size() > io::kMaxNumpyBytes / sizeof(double) / width) bad("channel matrix too large");
    std::vector<double> channels(rows.size() * width, std::numeric_limits<double>::quiet_NaN());
    Value::Array non_numeric, absent;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto &object = io::as_object(rows[i], "episode row");
        Value::Object nonnum;
        std::vector<std::string> missing;
        for (std::size_t j = 0; j < width; ++j) {
            const auto *value = io::find_member(object, episode.channel_names[j]);
            if (!value) { missing.push_back(episode.channel_names[j]); continue; }
            if (const auto *number = std::get_if<io::Number>(&value->data)) channels[i * width + j] = io::number_double(*number, "channel");
            else if (const auto *boolean = std::get_if<bool>(&value->data)) channels[i * width + j] = *boolean ? 1. : 0.;
            else nonnum.emplace_back(episode.channel_names[j], *value);
        }
        non_numeric.push_back(Value{std::move(nonnum)});
        absent.push_back(array_of_strings(missing));
    }
    std::set<std::string> control_set;
    for (const auto &control : episode.controls)
        for (const auto &[name, unused] : io::as_object(control, "control")) { (void)unused; control_set.insert(name); }
    const std::vector<std::string> control_names(control_set.begin(), control_set.end());
    const auto control_width = control_names.size();
    if (control_width && episode.controls.size() > io::kMaxNumpyBytes / sizeof(double) / control_width) bad("control matrix too large");
    std::vector<double> controls(episode.controls.size() * control_width, std::numeric_limits<double>::quiet_NaN());
    for (std::size_t i = 0; i < episode.controls.size(); ++i) {
        for (std::size_t j = 0; j < control_width; ++j) {
            const auto *value = member(episode.controls[i], control_names[j]);
            if (!value) continue;
            if (const auto *number = std::get_if<io::Number>(&value->data)) controls[i * control_width + j] = io::number_double(*number, "control");
            else if (const auto *boolean = std::get_if<bool>(&value->data)) controls[i * control_width + j] = *boolean ? 1. : 0.;
        }
    }
    io::NpzArchive archive;
    archive.emplace_back("channels", io::double_array(channels, {rows.size(), width}));
    archive.emplace_back("channel_names", io::string_array(episode.channel_names, {width}));
    for (const auto &[name, value] : episode.initial) archive.emplace_back("init_" + name, value);
    archive.emplace_back("controls", io::double_array(controls, {episode.controls.size(), control_width}));
    archive.emplace_back("control_names", io::string_array(control_names, {control_width}));
    const auto nq = dimension(episode.initial, "qpos"), nv = dimension(episode.initial, "qvel");
    archive.emplace_back("state_qpos", normalized(episode.state_qpos, {0, nq}));
    archive.emplace_back("state_qvel", normalized(episode.state_qvel, {0, nv}));
    archive.emplace_back("state_act", normalized(episode.state_act, {0, dimension(episode.initial, "act")}));
    archive.emplace_back("state_warmstart", normalized(episode.state_warmstart, {0, nv}));
    archive.emplace_back("state_time", normalized(episode.state_time, {0}));
    archive.emplace_back("ctrl_written", normalized(episode.ctrl_written, {0, dimension(episode.initial, "ctrl")}));
    archive.emplace_back("qfrc_applied", normalized(episode.qfrc_applied, {0, nv}));
    archive.emplace_back("force_names", io::string_array(episode.force_names, {episode.force_names.size()}));
    archive.emplace_back("forces", normalized(episode.forces, {0, 0, nv}));
    archive.emplace_back("tire_state", normalized(episode.tire_state, {0, 0}));
    archive.emplace_back("tire_state_names", io::string_array(episode.tire_state_names, {episode.tire_state_names.size()}));
    const auto encoded = io::encode_npz(archive);
    auto manifest = io::as_object(episode.manifest, "manifest");
    set(manifest, "first_failure", episode.first_failure);
    set(manifest, "non_numeric", Value{std::move(non_numeric)});
    set(manifest, "absent", Value{std::move(absent)});
    set(manifest, "controls", Value{episode.controls});
    const auto manifest_text = io::canonical_json(Value{std::move(manifest)}) + '\n';
    auto image = episode.model_image;
    if (model != nullptr) image = detail::model_image(*model);
    if (image.empty()) bad("save requires a model image or mjModel");
    if (out_dir.empty()) bad("output directory is empty");
    if (!out_dir.parent_path().empty()) std::filesystem::create_directories(out_dir.parent_path());
    platform::create_directory(out_dir);
    try {
        write(out_dir / "model.mjb", image);
        write(out_dir / "episode.npz", encoded);
        write(out_dir / "manifest.json", std::as_bytes(std::span(manifest_text)));
        platform::sync_directory(out_dir);
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove_all(out_dir, ignored);
        throw;
    }
}

EpisodeArtifact load_episode(const std::filesystem::path &directory) {
    const auto archive = io::parse_npz(io::read_numpy_bytes(directory / "episode.npz"));
    EpisodeArtifact out;
    out.manifest = read_manifest(directory / "manifest.json");
    (void)io::as_object(out.manifest, "manifest");
    out.model_image = io::read_numpy_bytes(directory / "model.mjb");
    if (out.model_image.empty()) bad("model image is empty");
    out.channel_names = string_vector(required(archive, "channel_names"), "channel_names");
    unique(out.channel_names, "channel_names");
    const auto &channels = required(archive, "channels");
    if (channels.shape.size() != 2 || channels.shape[1] != out.channel_names.size()) bad("channels shape disagrees with names");
    const auto matrix = channels.doubles(true);
    const auto *nonnum_value = member(out.manifest, "non_numeric");
    const auto *absent_value = member(out.manifest, "absent");
    const Value::Array empty;
    const auto &nonnum = nonnum_value ? io::as_array(*nonnum_value, "non_numeric") : empty;
    const auto &absent = absent_value ? io::as_array(*absent_value, "absent") : empty;
    for (std::size_t i = 0; i < channels.shape[0]; ++i) {
        std::set<std::string> skip;
        if (i < absent.size()) for (const auto &name : io::as_array(absent[i], "absent row")) skip.insert(io::as_string(name, "absent channel"));
        Value::Object row;
        for (std::size_t j = 0; j < out.channel_names.size(); ++j)
            if (!skip.contains(out.channel_names[j])) row.emplace_back(out.channel_names[j], real(matrix[i * out.channel_names.size() + j]));
        if (i < nonnum.size()) for (const auto &[name, value] : io::as_object(nonnum[i], "non_numeric row")) set(row, name, value);
        out.stored_rows.push_back(Value{std::move(row)});
        out.rows.push_back(flatten_row(out.stored_rows.back()));
    }
    const auto *controls = member(out.manifest, "controls");
    if (controls && !io::is_null(*controls)) out.controls = io::as_array(*controls, "controls");
    else {
        const auto names = string_vector(required(archive, "control_names"), "control_names");
        const auto &values = required(archive, "controls");
        if (values.shape.size() != 2 || values.shape[1] != names.size()) bad("controls shape disagrees with names");
        const auto numeric_controls = values.doubles(true);
        for (std::size_t i = 0; i < values.shape[0]; ++i) {
            Value::Object control;
            for (std::size_t j = 0; j < names.size(); ++j) {
                const auto value = numeric_controls[i * names.size() + j];
                control.emplace_back(names[j], std::isnan(value) ? Value{} : real(value));
            }
            out.controls.push_back(Value{std::move(control)});
        }
    }
    for (const auto &control : out.controls) (void)io::as_object(control, "control");
    if (const auto *failure = member(out.manifest, "first_failure")) out.first_failure = *failure;
    for (const auto &[name, value] : archive) if (name.starts_with("init_")) out.initial.emplace_back(name.substr(5), value);
    const auto nq = dimension(out.initial, "qpos"), nv = dimension(out.initial, "qvel");
    out.state_qpos = optional_array(archive, "state_qpos", {0, nq});
    out.state_qvel = optional_array(archive, "state_qvel", {0, nv});
    out.state_act = optional_array(archive, "state_act", {0, dimension(out.initial, "act")});
    out.state_warmstart = optional_array(archive, "state_warmstart", {0, nv});
    out.state_time = optional_array(archive, "state_time", {0});
    out.ctrl_written = optional_array(archive, "ctrl_written", {0, dimension(out.initial, "ctrl")});
    out.qfrc_applied = optional_array(archive, "qfrc_applied", {0, nv});
    out.forces = optional_array(archive, "forces", {0, 0, nv});
    out.tire_state = optional_array(archive, "tire_state", {0, 0});
    if (const auto *names = find(archive, "force_names")) out.force_names = string_vector(*names, "force_names");
    if (const auto *names = find(archive, "tire_state_names")) out.tire_state_names = string_vector(*names, "tire_state_names");
    unique(out.force_names, "force_names"); unique(out.tire_state_names, "tire_state_names");
    return out;
}

std::vector<std::string> compare_rows(const std::vector<Row> &golden,
    const std::vector<Row> &candidate, double rtol, double atol) {
    tolerances(rtol, atol);
    std::vector<std::string> differences;
    if (golden.size() != candidate.size()) {
        differences.push_back("row count: golden=" + std::to_string(golden.size()) + " candidate=" + std::to_string(candidate.size()));
        return differences;
    }
    for (std::size_t i = 0; i < golden.size(); ++i) {
        std::vector<std::string> names;
        for (const auto &item : golden[i]) names.push_back(item.first);
        for (const auto &item : candidate[i]) if (std::ranges::find(names, item.first) == names.end()) names.push_back(item.first);
        for (const auto &name : names) {
            const auto &a = get(golden[i], name), &b = get(candidate[i], name);
            if (!close(a, b, rtol, atol)) differences.push_back("step=" + std::to_string(i) + " channel=" + name +
                " golden=" + display(a) + " candidate=" + display(b));
        }
        // Python stops after a whole row once there are more than 50 diffs.
        if (differences.size() > 50) { differences.push_back("…truncated"); break; }
    }
    return differences;
}
bool same_failure(const io::Value &golden, const io::Value &candidate, double rtol, double atol) {
    tolerances(rtol, atol);
    if (io::is_null(golden) || io::is_null(candidate)) return io::is_null(golden) && io::is_null(candidate);
    const auto &a = io::as_array(golden, "first_failure"), &b = io::as_array(candidate, "first_failure");
    if (a.size() != 2 || b.size() != 2) bad("first_failure must be [time, violations]");
    return close(scalar(a[0]), scalar(b[0]), rtol, atol) && io::canonical_json(a[1]) == io::canonical_json(b[1]);
}
} // namespace bike::golden
