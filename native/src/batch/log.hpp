#pragma once
#include "../platform/files.hpp"
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::batch {
// Explicit compatibility overwrite removes only the named regular log.
// New output is always O_EXCL/O_NOFOLLOW, never a truncating symlink open.
[[nodiscard]] inline platform::FileHandle open_log(const std::filesystem::path& path, bool overwrite) {
    const auto state = std::filesystem::symlink_status(path);
    if (std::filesystem::exists(state)) {
        if (!overwrite) throw std::invalid_argument("log already exists: " + path.string());
        if (!std::filesystem::is_regular_file(state)) throw std::invalid_argument("log is not a regular file: " + path.string());
        std::filesystem::remove(path);
    }
    return platform::open_exclusive(path);
}
// The range constructor keeps the byte count tied to the view; the (pointer,
// size) form is rejected by -Wunsafe-buffer-usage-in-container.
inline void append_log(const platform::FileHandle& file, std::string_view text) {
    platform::write_all(file, std::as_bytes(std::span<const char>(text)));
    platform::sync(file);
}
inline void write_log(const std::filesystem::path& path, std::string_view text, bool overwrite) {
    auto file = open_log(path, overwrite); append_log(file, text); file.close();
}
} // namespace bike::batch
