// configuration/paths.cpp — the file-level half of T13: bounded no-follow
// TOML reads (the same 512 MiB regular-file contract the JSON loaders use)
// and resolution.py resolve_config_paths — relative asset spellings resolve
// once, against the declaring configuration file's directory.

#include "configuration/load.hpp"

#include <array>
#include <cerrno>
#include <string>
#include <system_error>
#include <variant>

#include <unistd.h>

#include "io/json.hpp"
#include "platform/files.hpp"

namespace bike::configuration {
namespace {

using bike::io::Value;

// Bounded whole-file read through the T06 boundary — identical contract to
// the validate.cpp JSON loaders: regular file, O_NOFOLLOW, 512 MiB cap.
std::string read_regular(const std::filesystem::path& path) {
    const platform::FileHandle handle = platform::open_regular(path);
    std::string content;
    std::array<char, 4096> chunk{};
    std::size_t total = 0;
    for (;;) {
        const ssize_t got = ::read(handle.get(), chunk.data(), chunk.size());
        if (got == 0) break;
        if (got < 0) {
            if (errno == EINTR) continue;
            throw std::system_error(
                std::error_code(errno, std::generic_category()),
                "read " + path.string());
        }
        total += static_cast<std::size_t>(got);
        if (total > bike::io::kMaxJsonBytes)
            throw std::invalid_argument(
                "configuration file exceeds the bounded input domain");
        content.append(chunk.data(), static_cast<std::size_t>(got));
    }
    return content;
}

// resolution.py's `(Path(directory)/file).resolve()` — canonical as far as
// the existing prefix allows, lexical normalization for the rest
// (weakly_canonical matches Path.resolve(strict=False)).
Value* find_mut(Value::Object& object, std::string_view key) {
    for (auto& member : object)
        if (member.first == key) return &member.second;
    return nullptr;
}

void resolve_member_in_place(Value::Object& object, std::string_view key,
                             const std::filesystem::path& directory) {
    Value* leaf = find_mut(object, key);
    if (leaf == nullptr || io::is_null(*leaf)) return;
    if (!std::holds_alternative<std::string>(leaf->data))
        io::detail::kind_error("bike::configuration::resolve_config_paths",
                               key, "string path");
    auto& spelling = std::get<std::string>(leaf->data);
    const std::filesystem::path file(spelling);
    if (file.is_absolute()) return;
    spelling = std::filesystem::weakly_canonical(directory / file).string();
}

} // namespace

namespace detail {

bike::io::Value parse_toml_file(const std::filesystem::path& file) {
    return parse_toml(read_regular(file), file.string());
}

void resolve_config_paths(bike::io::Value& values,
                          const std::filesystem::path& directory) {
    if (!std::holds_alternative<Value::Object>(values.data))
        io::detail::kind_error("bike::configuration::resolve_config_paths",
                               "values", "object");
    auto& root = std::get<Value::Object>(values.data);
    // result.get('articulated', {}).get(key) — a non-table articulated
    // section fails the attribute access in the source before construction.
    if (Value* articulated = find_mut(root, "articulated")) {
        if (!std::holds_alternative<Value::Object>(articulated->data))
            io::detail::kind_error(
                "bike::configuration::resolve_config_paths", "articulated",
                "object");
        auto& section = std::get<Value::Object>(articulated->data);
        resolve_member_in_place(section, "joint_envelope_path", directory);
        resolve_member_in_place(section, "joint_strength_path", directory);
    }
    resolve_member_in_place(root, "equilibrium_state_file", directory);
}

} // namespace detail
} // namespace bike::configuration
