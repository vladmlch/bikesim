#pragma once
#include "duckdb.hpp"
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bike::recording {
class RecordingError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
    ~RecordingError() override;
    RecordingError(const RecordingError&) = default;
    RecordingError& operator=(const RecordingError&) = default;
    RecordingError(RecordingError&&) = default;
    RecordingError& operator=(RecordingError&&) = default;
};
[[nodiscard]] io::Value read_manifest(const std::filesystem::path&);
class Recording {
public:
    explicit Recording(std::filesystem::path);
    ~Recording();
    void close() noexcept;
    Recording(const Recording&) = delete;
    Recording& operator=(const Recording&) = delete;
    Recording(Recording&&) = delete;
    Recording& operator=(Recording&&) = delete;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
    [[nodiscard]] const io::Value& manifest() const noexcept { return manifest_; }
    [[nodiscard]] std::int64_t schema_version() const noexcept { return 1; }
    [[nodiscard]] io::Value metadata() const;
    [[nodiscard]] std::vector<std::int64_t> episode_ids() const;
    [[nodiscard]] io::Value episode(std::optional<std::int64_t> = {}, bool research = false) const;
    [[nodiscard]] std::vector<io::Value::Object> rows(std::string_view,
        std::span<const io::Value> parameters = {}) const;
    [[nodiscard]] Cursor stream(std::string_view, std::span<const io::Value> parameters = {}) const;
    [[nodiscard]] io::Value query(std::string_view, std::uint64_t row_limit = 200,
        std::span<const io::Value> parameters = {}) const;
private:
    std::filesystem::path path_;
    io::Value manifest_;
    std::unique_ptr<Database> database_;
};
[[nodiscard]] std::unique_ptr<Recording> open_recording(const std::filesystem::path&);
[[nodiscard]] io::Value query(const Recording&, std::string_view, std::uint64_t row_limit);
[[nodiscard]] io::Value overview(const Recording&, std::uint64_t limit = 200);
[[nodiscard]] io::Value fields(const Recording&, std::optional<std::string> pattern = {}, std::uint64_t limit = 200);
[[nodiscard]] io::Value events(const Recording&, std::int64_t episode, std::optional<std::string> kind = {}, std::uint64_t limit = 200);
[[nodiscard]] io::Value window(const Recording&, std::int64_t episode, double start_s, double end_s,
    const std::optional<std::string>& fields = {}, std::uint64_t limit = 200);
[[nodiscard]] io::Value compare(const Recording&, const Recording&, std::int64_t left_episode, std::int64_t right_episode,
    std::string_view mode = "exact", const std::optional<std::string>& fields = {}, double atol = 1e-9, double rtol = 1e-9,
    std::uint64_t limit = 200);
} // namespace bike::recording
