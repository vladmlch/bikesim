#pragma once
#include "capture.hpp"
#include <functional>

namespace bike::recording {
struct Publication {
    std::filesystem::path directory;
    std::string report_status;
};
using ReportGenerator = std::function<std::string(const std::filesystem::path& database)>;
// Hooks are cold-path fault-injection seams. Production callers leave them empty.
struct FinalizeOptions {
    std::size_t maximum_batch_rows = batch_rows;
    ReportGenerator report;
    std::function<void(std::string_view stage)> before_stage;
};
[[nodiscard]] Publication finalize(RunCapture&, const std::filesystem::path& output_root,
                                   const FinalizeOptions& options);
[[nodiscard]] Publication finalize(RunCapture&, const std::filesystem::path& output_root);
[[nodiscard]] Publication finalize_guarded(RunCapture&, const std::filesystem::path& output_root,
                                           const FinalizeOptions& options = {});
// Callers normally use finalize, which owns the state machine. This function
// performs one fresh attempt without consuming any captured evidence.
[[nodiscard]] Publication publish(const RunCapture&, const std::filesystem::path& output_root,
                                  const FinalizeOptions& options);
} // namespace bike::recording
