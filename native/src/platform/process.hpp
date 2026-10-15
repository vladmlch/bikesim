// process.hpp — bounded argv subprocess execution (task T06) for tool/CLI
// orchestration. Direct posix_spawn argument vectors only: no shell
// interpolation, no implicit parent working-directory mutation — the child
// chdir's inside its own spawn file actions. The child leads its own
// process group so cancellation kills and reaps the whole group, never a
// leftover grandchild tree. stdout and stderr are drained CONCURRENTLY
// through one bounded poll loop — a flood on one stream can never deadlock
// the other. POSIX only (macOS/Linux).
//
// exit_code preserves the wait status: a normal exit reports 0..255; a
// signal death reports 128 + signal number (the POSIX shell encoding), so
// both pieces of information survive in one field. Input-domain violations
// (empty argv/argv[0], embedded NUL, non-directory cwd, nonpositive
// timeout) are std::invalid_argument; spawn/OS failures are
// std::system_error; exceeding max_output_bytes kills the group, reaps it
// and throws std::runtime_error naming the bound — tool output is bounded,
// never silently truncated.
#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bike::platform {

struct ProcessResult {
    int exit_code = 0;
    std::string out;
    std::string err;
};

struct ProcessOptions {
    // Wall-clock-free deadline on the monotonic axis: when it expires the
    // whole process group receives SIGKILL and is reaped; the result keeps
    // the signal-encoded exit_code and the streams captured so far.
    std::optional<double> timeout_s;
    // Per-stream capture bound in bytes. Default 64 MiB; a larger child
    // output is a malfunction, reported loudly after kill+reap.
    std::size_t max_output_bytes = 67108864U;
};

// Runs argv in cwd to completion. stdin is /dev/null; the environment is
// inherited verbatim. Same EOF semantics as subprocess.communicate(): the
// call returns only after the child is reaped AND both pipes hit EOF.
[[nodiscard]] ProcessResult run(const std::vector<std::string> &argv,
                                const std::filesystem::path &cwd);
[[nodiscard]] ProcessResult run(const std::vector<std::string> &argv,
                                const std::filesystem::path &cwd,
                                const ProcessOptions &options);

} // namespace bike::platform
