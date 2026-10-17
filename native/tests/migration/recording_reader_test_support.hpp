#pragma once
#include "support/migration_test.hpp"
#include "recording/analysis_detail.hpp"
#include "platform/files.hpp"
#include <cerrno>
#include <cstdlib>
#include <system_error>

namespace recording_reader_test {
struct Directory {
    std::filesystem::path path;
    Directory() {
        auto pattern = (std::filesystem::temp_directory_path() / "bike-recording-reader-test-XXXXXX").string();
        std::vector<char> bytes(pattern.begin(), pattern.end()); bytes.push_back('\0');
        const auto* made = ::mkdtemp(bytes.data());
        if (!made) throw std::system_error(errno, std::generic_category(), "test temporary directory");
        path = made;
    }
    ~Directory() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
};
inline void write(const std::filesystem::path& file, const std::string& text) {
    bike::platform::write_exclusive(file, std::as_bytes(std::span(text.data(), text.size())));
}
template<class Action> void rejects(Action&& action) {
    bool rejected = false;
    try { std::forward<Action>(action)(); } catch (const std::exception&) { rejected = true; }
    bt::require(rejected, "operation must reject invalid input");
}
} // namespace recording_reader_test
