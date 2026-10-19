#pragma once
#include "input.hpp"
#include "../research/environment.hpp"
namespace bike::replay {
// Only fixed validated assets are materialized. Their owner must outlive the
// prepared model/runtime, whose immutable config refers to profile filenames.
class FixedAssets {
public:
    explicit FixedAssets(const ReplayInput&);
    ~FixedAssets();
    FixedAssets(const FixedAssets&) = delete;
    FixedAssets& operator=(const FixedAssets&) = delete;
    FixedAssets(FixedAssets&&) = delete;
    FixedAssets& operator=(FixedAssets&&) = delete;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
private:
    std::filesystem::path path_;
};
[[nodiscard]] std::unique_ptr<research::ResearchRequest> request_from_input(const ReplayInput&, const FixedAssets&);
void verify_configuration(const ReplayInput&, const research::PreparedEnvironment&);
void check_execution(const ReplayInput&);
} // namespace bike::replay
