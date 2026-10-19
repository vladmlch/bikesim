#pragma once
#include "input.hpp"
#include "../runtime/runtime.hpp"
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
namespace bike::replay {
struct ReplayResult {
    bool matched = false, physically_valid = false;
    std::string reason;
    io::Value details;
};
class IncompatibleExecution : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
    ~IncompatibleExecution() override;
    IncompatibleExecution(const IncompatibleExecution&) = default;
    IncompatibleExecution& operator=(const IncompatibleExecution&) = default;
    IncompatibleExecution(IncompatibleExecution&&) = default;
    IncompatibleExecution& operator=(IncompatibleExecution&&) = default;
};
struct Comparison {
    bool matched = true;
    std::string first_path, reason;
    double max_absolute_error = 0.;
};
[[nodiscard]] Comparison compare_evidence(const io::Value& actual, const io::Value& expected, const std::string& path = "evidence");
class ReplaySession {
public:
    explicit ReplaySession(const std::filesystem::path&, std::optional<std::int64_t> episode = {});
    ~ReplaySession();
    ReplaySession(const ReplaySession&) = delete;
    ReplaySession& operator=(const ReplaySession&) = delete;
    ReplaySession(ReplaySession&&) = delete;
    ReplaySession& operator=(ReplaySession&&) = delete;
    [[nodiscard]] bool advance(std::optional<double> wall_budget_s = {}, std::optional<std::int64_t> target_step = {});
    [[nodiscard]] bool done() const noexcept;
    [[nodiscard]] bool pending() const;
    [[nodiscard]] const runtime::RuntimeSnapshot& snapshot() const;
    [[nodiscard]] const runtime::RenderFrame& render_frame() const;
    [[nodiscard]] io::Value report() const;
    // Always available, including an early close or mismatch. `passed` is
    // false until report() can return the completed verification certificate.
    [[nodiscard]] io::Value diagnostics() const;
    [[nodiscard]] std::optional<std::string> error() const;
    [[nodiscard]] const std::vector<std::byte>& model_image() const;
    [[nodiscard]] double timestep_s() const;
    void restart();
    void close();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
[[nodiscard]] ReplayResult check(const std::filesystem::path&, std::int64_t episode);
} // namespace bike::replay
