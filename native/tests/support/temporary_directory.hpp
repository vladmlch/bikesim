#pragma once
#include <filesystem>
#include <random>
#include <string>
#include <string_view>
#include <stdexcept>
#include <system_error>

namespace bt {
class TemporaryDirectory {
public:
    explicit TemporaryDirectory(std::string_view label) {
        if (label.empty() || label.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != label.npos)
            throw std::invalid_argument("invalid temporary test directory label");
        std::random_device entropy;
        for (int attempt = 0; attempt < 100; ++attempt) {
            const auto candidate = std::filesystem::temp_directory_path() /
                ("bike-" + std::string(label) + "-" + std::to_string(entropy()));
            std::error_code error;
            if (std::filesystem::create_directory(candidate, error)) { path_ = candidate; return; }
            if (error) throw std::system_error(error, "create temporary test directory");
        }
        throw std::runtime_error("cannot create unique temporary test directory");
    }
    ~TemporaryDirectory() { std::error_code error; std::filesystem::remove_all(path_, error); }
    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;
    TemporaryDirectory(TemporaryDirectory&&) = delete;
    TemporaryDirectory& operator=(TemporaryDirectory&&) = delete;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
private:
    std::filesystem::path path_;
};
} // namespace bt
