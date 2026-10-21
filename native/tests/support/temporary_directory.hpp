#pragma once
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <string_view>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace bt {
class TemporaryDirectory {
public:
    explicit TemporaryDirectory(std::string_view label) {
        if (label.empty() || label.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != label.npos)
            throw std::invalid_argument("invalid temporary test directory label");
        const auto pattern = (std::filesystem::temp_directory_path() / ("bike-" + std::string(label) + "-XXXXXX")).string();
        std::vector<char> writable(pattern.begin(), pattern.end()); writable.push_back('\0');
        const char* made = ::mkdtemp(writable.data());
        if (made == nullptr) throw std::system_error(errno, std::generic_category(), "mkdtemp");
        path_ = made;
    }
    ~TemporaryDirectory() { std::error_code error; std::filesystem::remove_all(path_, error); }
    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
private:
    std::filesystem::path path_;
};
} // namespace bt
