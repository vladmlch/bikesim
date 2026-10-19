// process.cpp — POSIX argv subprocess execution: posix_spawnp file
// actions (no shell), a dedicated child process group, one poll loop that
// drains stdout and stderr concurrently under explicit byte bounds, and
// killpg+waitpid cleanup on deadline or overflow.
#include "platform/process.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <csignal>
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <crt_externs.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;

[[noreturn]] void fail_errno(std::string_view what, const int error) {
    throw std::system_error(std::error_code(error, std::generic_category()),
                            std::string(what));
}

// An owning descriptor for the brief window between pipe()/spawn() and the
// drain loop; exceptions during spawn setup must not leak descriptors.
struct ScopedFd {
    int fd = -1;
    ScopedFd() noexcept = default;
    ~ScopedFd() {
        if (fd >= 0) (void) ::close(fd);
    }
    ScopedFd(const ScopedFd &) = delete;
    ScopedFd &operator=(const ScopedFd &) = delete;
    ScopedFd(ScopedFd &&) = delete;
    ScopedFd &operator=(ScopedFd &&) = delete;
};

void make_pipe(ScopedFd &read_end, ScopedFd &write_end) {
    std::array<int, 2> fds {};
    if (::pipe(fds.data()) != 0) {
        const int error = errno;
        fail_errno("platform::run: pipe", error);
    }
    for (const int fd : fds)
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) fcntl is a vararg interface; FD_CLOEXEC is required for the parent ends
        if (::fcntl(fd, F_SETFD, FD_CLOEXEC) != 0) {
            const int error = errno;
            (void) ::close(fds[0]);
            (void) ::close(fds[1]);
            fail_errno("platform::run: pipe FD_CLOEXEC", error);
        }
    read_end.fd = fds[0];
    write_end.fd = fds[1];
}

class FileActions {
  public:
    FileActions() {
        if (const int error = ::posix_spawn_file_actions_init(&value_);
            error != 0)
            fail_errno("platform::run: posix_spawn_file_actions_init", error);
    }
    ~FileActions() { (void) ::posix_spawn_file_actions_destroy(&value_); }
    FileActions(const FileActions &) = delete;
    FileActions &operator=(const FileActions &) = delete;
    FileActions(FileActions &&) = delete;
    FileActions &operator=(FileActions &&) = delete;
    [[nodiscard]] posix_spawn_file_actions_t *ptr() { return &value_; }

  private:
    posix_spawn_file_actions_t value_ {};
};

class SpawnAttr {
  public:
    SpawnAttr() {
        if (const int error = ::posix_spawnattr_init(&value_); error != 0)
            fail_errno("platform::run: posix_spawnattr_init", error);
    }
    ~SpawnAttr() { (void) ::posix_spawnattr_destroy(&value_); }
    SpawnAttr(const SpawnAttr &) = delete;
    SpawnAttr &operator=(const SpawnAttr &) = delete;
    SpawnAttr(SpawnAttr &&) = delete;
    SpawnAttr &operator=(SpawnAttr &&) = delete;
    [[nodiscard]] posix_spawnattr_t *ptr() { return &value_; }

  private:
    posix_spawnattr_t value_ {};
};

void require_action(const int error, std::string_view what) {
    if (error != 0) fail_errno(what, error);
}

int add_chdir(posix_spawn_file_actions_t *actions, const char *path) {
#if defined(__APPLE__)
#if __MAC_OS_X_VERSION_MAX_ALLOWED >= 260000
    // POSIX.1-2024 name; the _np spelling is deprecated from macOS 26 SDKs.
    return ::posix_spawn_file_actions_addchdir(actions, path);
#else
    return ::posix_spawn_file_actions_addchdir_np(actions, path);
#endif
#elif defined(__linux__) && defined(__GLIBC__) && defined(__GLIBC_PREREQ)
#if __GLIBC_PREREQ(2, 38)
    return ::posix_spawn_file_actions_addchdir(actions, path);
#else
    return ::posix_spawn_file_actions_addchdir_np(actions, path);
#endif
#else
    return ::posix_spawn_file_actions_addchdir(actions, path);
#endif
}

char **process_environment() {
#if defined(__APPLE__)
    // environ is unreliable inside dyld-managed runtimes; the sanctioned
    // accessor always returns the real environment block.
    return *_NSGetEnviron();
#else
    return ::environ;
#endif
}

void validate_run_inputs(const std::vector<std::string> &argv,
                         const std::string &cwd_text,
                         const std::filesystem::path &cwd,
                         const bike::platform::ProcessOptions &options) {
    if (argv.empty())
        throw std::invalid_argument("platform::run: argv must not be empty");
    if (argv.front().empty())
        throw std::invalid_argument("platform::run: argv[0] must not be empty");
    for (const auto &arg : argv)
        if (arg.find('\0') != std::string::npos)
            throw std::invalid_argument(
                "platform::run: argv contains a NUL byte");
    if (cwd_text.empty())
        throw std::invalid_argument("platform::run: cwd must not be empty");
    if (cwd_text.find('\0') != std::string::npos)
        throw std::invalid_argument("platform::run: cwd contains a NUL byte");
    std::error_code ec;
    if (!std::filesystem::is_directory(cwd, ec))
        throw std::invalid_argument("platform::run: cwd is not a directory: " +
                                    cwd_text);
    if (options.timeout_s.has_value() &&
        (!std::isfinite(*options.timeout_s) || *options.timeout_s <= 0.0))
        throw std::invalid_argument(
            "platform::run: timeout_s must be positive and finite");
    if (options.max_output_bytes == 0)
        throw std::invalid_argument(
            "platform::run: max_output_bytes must be nonzero");
    if (!std::isfinite(options.cancellation_grace_s) || options.cancellation_grace_s < 0.)
        throw std::invalid_argument("platform::run: cancellation_grace_s must be finite and nonnegative");
}

} // namespace

namespace bike::platform {

ProcessResult run(const std::vector<std::string> &argv,
                  const std::filesystem::path &cwd) {
    return run(argv, cwd, ProcessOptions{});
}

ProcessResult run(const std::vector<std::string> &argv,
                  const std::filesystem::path &cwd,
                  const ProcessOptions &options) {
    const std::string &cwd_text = cwd.native();
    validate_run_inputs(argv, cwd_text, cwd, options);
    if (options.cancellation.stop_requested()) return {.exit_code=130, .out={}, .err={}};

    // On platforms without an atomic pipe(O_CLOEXEC), another concurrent
    // platform::run must not spawn between pipe() and fcntl(). Such a child
    // could retain a sibling's pipe and prevent EOF after that sibling exits.
    static std::mutex spawn_mutex;
    std::unique_lock spawn_guard(spawn_mutex);

    ScopedFd out_read, out_write, err_read, err_write;
    make_pipe(out_read, out_write);
    make_pipe(err_read, err_write);

    FileActions actions;
    require_action(
        ::posix_spawn_file_actions_addopen(actions.ptr(), STDIN_FILENO,
                                           "/dev/null", O_RDONLY, 0),
        "platform::run: file_actions addopen /dev/null");
    // The child enters its own process group FIRST so every later member of
    // the tree sits under one killable group id.
    SpawnAttr attr;
    require_action(::posix_spawnattr_setflags(attr.ptr(), POSIX_SPAWN_SETPGROUP),
                   "platform::run: attr setflags SETPGROUP");
    require_action(::posix_spawnattr_setpgroup(attr.ptr(), 0),
                   "platform::run: attr setpgroup");
    // dup2 the pipe write ends onto stdout/stderr, then close every other
    // inherited descriptor end inside the child.
    require_action(
        ::posix_spawn_file_actions_adddup2(actions.ptr(), out_write.fd,
                                           STDOUT_FILENO),
        "platform::run: file_actions dup2 stdout");
    require_action(
        ::posix_spawn_file_actions_adddup2(actions.ptr(), err_write.fd,
                                           STDERR_FILENO),
        "platform::run: file_actions dup2 stderr");
    for (const int fd : {out_read.fd, err_read.fd, out_write.fd, err_write.fd})
        require_action(
            ::posix_spawn_file_actions_addclose(actions.ptr(), fd),
            "platform::run: file_actions close");
    // cwd is applied inside the child image — the parent's working
    // directory is never touched.
    require_action(add_chdir(actions.ptr(), cwd_text.c_str()),
                   "platform::run: file_actions chdir");

    std::vector<char *> child_argv;
    child_argv.reserve(argv.size() + 1U);
    for (const auto &arg : argv)
        child_argv.push_back(const_cast<char *>(arg.c_str())); // NOLINT
    child_argv.push_back(nullptr);

    pid_t pid = -1;
    {
        const int error = ::posix_spawnp(&pid, child_argv.front(),
                                         actions.ptr(), attr.ptr(),
                                         child_argv.data(), process_environment());
        if (error != 0)
            fail_errno("platform::run: posix_spawnp " + argv.front(), error);
    }
    // Any allocation, read, poll or callback failure after spawn still reaps
    // the owned process tree. A destructor never lets an exception escape.
    struct ChildOwner {
        pid_t id{};
        bool released = false;
        explicit ChildOwner(pid_t pid) : id(pid) {}
        ~ChildOwner() {
            if (released) return;
            (void)::killpg(id, SIGKILL);
            int ignored = 0;
            while (::waitpid(id, &ignored, 0) < 0 && errno == EINTR) {}
        }
        ChildOwner(const ChildOwner&) = delete;
        ChildOwner& operator=(const ChildOwner&) = delete;
        ChildOwner(ChildOwner&&) = delete;
        ChildOwner& operator=(ChildOwner&&) = delete;
    } child{pid};
    // The child's copies are dup'd; the parent must drop its write ends so
    // EOF can arrive on the read ends.
    (void) ::close(out_write.fd);
    out_write.fd = -1;
    (void) ::close(err_write.fd);
    err_write.fd = -1;
    spawn_guard.unlock();

    Clock::time_point deadline = Clock::time_point::max();
    if (options.timeout_s.has_value())
        deadline = Clock::now() +
                   std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(*options.timeout_s));

    struct DrainEnd {
        int fd;
        ScopedFd* owner;
        std::string *sink;
    };
    ProcessResult result;
    std::array<DrainEnd, 2> drains {
        {DrainEnd{.fd = out_read.fd, .owner = &out_read, .sink = &result.out},
         DrainEnd{.fd = err_read.fd, .owner = &err_read, .sink = &result.err}}};

    int status = 0;
    int open_fds = 2;
    bool reaped = false;
    bool killed = false;
    bool overflowed = false;
    bool cancellation_sent = false;
    std::array<char, 4096> chunk {};

    while (open_fds > 0 || !reaped) {
        if (!cancellation_sent && options.cancellation.stop_requested()) {
            (void)::killpg(pid, SIGINT);
            cancellation_sent = true;
            deadline = std::min(deadline, Clock::now() +
                std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(options.cancellation_grace_s)));
        }
        // The deadline covers the whole run: a grandchild holding a pipe
        // open cannot outlive the budget either.
        if (!killed && Clock::now() >= deadline) {
            (void) ::killpg(pid, SIGKILL);
            killed = true;
        }
        if (!reaped) {
            const pid_t got = ::waitpid(pid, &status, WNOHANG);
            if (got == pid) {
                reaped = true;
            } else if (got < 0) {
                const int error = errno;
                if (error == EINTR) continue;
                fail_errno("platform::run: waitpid", error);
            }
        }
        if (reaped && open_fds == 0) break;

        std::array<pollfd, 2> poll_fds {};
        std::array<std::size_t, 2> poll_index {};
        nfds_t poll_count = 0;
        for (std::size_t i = 0; i < drains.size(); ++i) {
            if (drains[i].fd < 0) continue;
            poll_fds[poll_count] =
                pollfd{.fd = drains[i].fd, .events = POLLIN, .revents = 0};
            poll_index[poll_count] = i;
            ++poll_count;
        }
        int timeout_ms = 20;
        if (deadline != Clock::time_point::max()) {
            const auto left = deadline - Clock::now();
            if (left <= Clock::duration::zero()) {
                timeout_ms = 0;
            } else {
                const auto left_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(left)
                        .count();
                timeout_ms = left_ms < 20 ? (left_ms <= 0 ? 1 : static_cast<int>(left_ms)) : 20;
            }
        }
        const int ready =
            ::poll(poll_fds.data(), poll_count, timeout_ms);
        if (ready < 0) {
            const int error = errno;
            if (error == EINTR) continue;
            fail_errno("platform::run: poll", error);
        }
        if (ready <= 0) continue;
        for (nfds_t k = 0; k < poll_count; ++k) {
            if (poll_fds[k].revents == 0) continue;
            DrainEnd &end = drains[poll_index[k]];
            const ssize_t got =
                ::read(end.fd, chunk.data(), chunk.size());
            if (got < 0) {
                const int error = errno;
                if (error == EINTR) continue;
                fail_errno("platform::run: read", error);
            }
            if (got > 0) {
                const auto amount = static_cast<std::size_t>(got);
                if (end.sink->size() + amount > options.max_output_bytes) {
                    overflowed = true;
                    if (!killed) {
                        (void) ::killpg(pid, SIGKILL);
                        killed = true;
                    }
                } else {
                    end.sink->append(chunk.data(), amount);
                }
            }
            // got==0 is EOF; POLLNVAL means the descriptor died. A bare
            // POLLHUP with data pending must not close early — the pipe
            // buffer still holds bytes that read() returns first.
            if (got == 0 ||
                (static_cast<unsigned>(poll_fds[k].revents) &
                 static_cast<unsigned>(POLLNVAL)) != 0U) {
                (void) ::close(end.fd);
                end.owner->fd = -1;
                end.fd = -1;
                --open_fds;
            }
        }
    }

    child.released = true;
    if (overflowed)
        throw std::runtime_error(
            "platform::run: child output exceeded the max_output_bytes bound (" +
            std::to_string(options.max_output_bytes) + " bytes)");

    if (WIFEXITED(status)) {
        result.exit_code = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
        result.exit_code = 128 + WTERMSIG(status);
    } else {
        result.exit_code = 255;
    }
    return result;
}

} // namespace bike::platform
