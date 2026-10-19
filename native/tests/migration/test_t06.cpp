// test_t06.cpp — bounded filesystem, subprocess, signal and provenance
// primitives (task T06). The behavioral authorities are
// src/bike_sim/recording/interrupt.py (deferred-SIGINT shield: handlers
// only store sig_atomic_t state; finalization runs on the owning thread;
// repeated SIGINT cannot interrupt the critical section) and
// src/bike_sim/recording/storage.py (exclusive publication: exclusive-create
// staging/destination, validate the OPENED object, no-follow regular-file
// checks, hash after close, existing destinations never adopted).
#include "support/migration_test.hpp"
#include "io/hash.hpp"
#include "platform/files.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"
#include "platform/signals.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <utility>

#include <csignal>
#include <unistd.h>

// Packaging-axis expectation for this build graph (see the T06 block in
// native/tests/migration/CMakeLists.txt): 1 when the configured graph
// carries the optional adapter, 0 in the produced bindings-OFF build.
#ifndef BIKE_T06_EXPECT_BINDINGS
#define BIKE_T06_EXPECT_BINDINGS 0
#endif
#ifndef BIKE_T06_EXPECT_GUI
#define BIKE_T06_EXPECT_GUI 0
#endif

namespace {

namespace pf = bike::platform;

// Process-spawned child modes: the test re-executes itself so no shell or
// second binary is involved. Modes are dispatched in main() before bt::run.
constexpr std::string_view kModeSleep = "--t06-child-sleep";
constexpr std::string_view kModeFlood = "--t06-child-flood";
constexpr std::string_view kModeResource = "--t06-child-resource";
constexpr std::string_view kModeExit = "--t06-child-exit";

// An exclusive temp directory owned for the test section's scope; cleanup is
// best-effort and never converts a passing check into a failure.
struct TempDir {
    std::filesystem::path path;
    TempDir() {
        static std::atomic<int> counter{0};
        path = std::filesystem::temp_directory_path() /
               ("bike-t06-" + std::to_string(static_cast<long long>(::getpid())) +
                "-" + std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    TempDir(const TempDir &) = delete;
    TempDir &operator=(const TempDir &) = delete;
    TempDir(TempDir &&) = delete;
    TempDir &operator=(TempDir &&) = delete;
};

// std::system_error (OS/domain failures) and every other std::runtime_error
// lands here; input-domain violations stay on bt::invalid
// (std::invalid_argument).
template <class F> void throws_runtime(F &&action) {
    bool rejected = false;
    try {
        std::forward<F>(action)();
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    bt::require(rejected, "expected a runtime/system error");
}

bool hex64(std::string_view value) {
    if (value.size() != 64U) return false;
    for (const char c : value)
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    return true;
}

// ---- produced contract (step 2 of the work package) -------------------------
void checks_produced() {
    const auto p = pf::run({"/usr/bin/printf", "%s", "a b;not-a-command"}, "/");
    bt::require(p.exit_code == 0 && p.out == "a b;not-a-command",
                "argv is not a shell");
}

// ---- subprocess boundaries ---------------------------------------------------
void checks_process() {
    const std::filesystem::path self = pf::executable_path();
    // Child failure is preserved verbatim through the status word.
    {
        const auto p = pf::run({self.string(), std::string(kModeExit), "7"}, "/");
        bt::require(p.exit_code == 7, "child exit status preserved");
        bt::require(p.out == "child-out\n" && p.err == "child-err\n",
                    "child streams captured verbatim");
    }
    // cwd is an argv-level working directory, never an implicit mutation of
    // the parent. A path with a space exercises the vector path end to end.
    {
        const TempDir temp;
        const auto spaced = temp.path / "cwd with space";
        std::filesystem::create_directories(spaced);
        const auto p = pf::run({"/bin/pwd"}, spaced);
        std::string out = p.out;
        if (!out.empty() && out.back() == '\n') out.pop_back();
        bt::require(p.exit_code == 0 &&
                        std::filesystem::weakly_canonical(out) ==
                            std::filesystem::weakly_canonical(spaced),
                    "cwd honored without shell or parent chdir");
    }
    // Timeout: the whole process group is killed and reaped; the result
    // keeps the signal-encoded status (128+SIGKILL) and partial streams.
    {
        const pf::ProcessOptions options{
            .cancellation = {},
            .timeout_s = 0.25,
            .max_output_bytes = std::size_t{4U} * 1024U * 1024U};
        const auto started = std::chrono::steady_clock::now();
        const auto p =
            pf::run({self.string(), std::string(kModeSleep), "30"}, "/", options);
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
                .count();
        bt::require(p.exit_code == 128 + SIGKILL,
                    "timeout kills the child group (signal status)");
        bt::require(elapsed < 10.0, "kill/reap bounded the wait");
    }
    // stderr flood cannot deadlock stdout collection: the child writes
    // ~2 MiB to stderr (macOS pipes hold far less) and only then writes the
    // stdout marker — a serial drain would wedge the child on a full pipe.
    {
        const auto p = pf::run(
            {self.string(), std::string(kModeFlood), "2097152", "stdout-ok"}, "/");
        bt::require(p.exit_code == 0, "flooded child completed");
        bt::require(p.out == "stdout-ok", "stdout drained past the flood");
        bt::require(p.err.size() == 2097152U,
                    "stderr fully drained concurrently");
    }
    // Explicit output bounds: a flood past max_output_bytes fails loudly and
    // the runaway child is killed and reaped, never truncated silently.
    {
        const pf::ProcessOptions options{
            .cancellation = {},
            .timeout_s = std::nullopt,
            .max_output_bytes = 4096U};
        throws_runtime([&] {
            (void) pf::run(
                {self.string(), std::string(kModeFlood), "1048576", "x"}, "/",
                options);
        });
    }
    // Input-domain errors are std::invalid_argument.
    bt::invalid([] { (void) pf::run({}, "/"); });
    bt::invalid([] { (void) pf::run({""}, "/"); });
    bt::invalid([] {
        (void) pf::run({"/usr/bin/true", std::string("a\0b", 3U)}, "/");
    });
    bt::invalid([] { (void) pf::run({"/usr/bin/true"}, "/no/such/dir-t06"); });
    bt::invalid([] {
        (void) pf::run({"/usr/bin/true"}, "/",
                       pf::ProcessOptions{.cancellation = {}, .timeout_s = 0.0,
                                          .max_output_bytes = 16U});
    });
    // Nonfinite deadlines are the same input-domain violation: NaN and
    // +/-inf can never arm a monotonic deadline.
    bt::invalid([] {
        (void) pf::run(
            {"/usr/bin/true"}, "/",
            pf::ProcessOptions{
                .cancellation = {},
                .timeout_s = std::numeric_limits<double>::infinity(),
                .max_output_bytes = 16U});
    });
    bt::invalid([] {
        (void) pf::run(
            {"/usr/bin/true"}, "/",
            pf::ProcessOptions{
                .cancellation = {},
                .timeout_s = std::numeric_limits<double>::quiet_NaN(),
                .max_output_bytes = 16U});
    });
    // A zero output bound is rejected up front — capture is bounded by
    // contract and zero is a misconfiguration, not "capture nothing".
    bt::invalid([] {
        (void) pf::run({"/usr/bin/true"}, "/",
                       pf::ProcessOptions{.cancellation = {}, .timeout_s = std::nullopt,
                                          .max_output_bytes = 0U});
    });
}

// ---- bounded filesystem primitives -------------------------------------------
void checks_files() {
    TempDir temp;
    // Spaces, Unicode and shell metacharacters in every component — no part
    // of the path is ever interpolated or normalized away.
    const auto awkward_dir =
        temp.path / "dir with space ünïçødë ; $(not-a-command)";
    std::filesystem::create_directories(awkward_dir);
    const auto awkward = awkward_dir / "païd; $(x) b.bin";
    const std::string payload = "exclusive payload\n";
    const auto bytes = std::as_bytes(std::span(payload));
    pf::write_exclusive(awkward, bytes);
    bt::require(bt::text(awkward) == payload, "exclusive write round trip");
    bt::require(pf::file_sha256(awkward) == bike::io::sha256(bytes),
                "file_sha256 hashes the closed object");

    // Existing destinations are never adopted — including empty ones.
    {
        const auto empty_file = temp.path / "empty-dest";
        { const std::ofstream touch(empty_file); }
        throws_runtime([&] { pf::write_exclusive(empty_file, bytes); });
        bt::require(bt::text(empty_file).empty(),
                    "existing empty destination untouched");
        throws_runtime([&] { pf::write_exclusive(awkward, bytes); });
    }

    // Exclusive directory creation owns the name; collisions fail.
    {
        const auto staged = temp.path / "staging";
        pf::create_directory(staged);
        throws_runtime([&] { pf::create_directory(staged); });
        const auto empty_dir = temp.path / "empty-dir-dest";
        std::filesystem::create_directory(empty_dir);
        throws_runtime([&] { pf::create_directory(empty_dir); });
        throws_runtime(
            [&] { pf::create_directory(temp.path / "missing" / "nested"); });
    }

    // Exclusive hard links: the publication protocol's last step.
    {
        const auto linked = temp.path / "linked.bin";
        pf::link_new(awkward, linked);
        bt::require(bt::text(linked) == payload, "hard link publishes bytes");
        throws_runtime([&] { pf::link_new(awkward, linked); });
        throws_runtime(
            [&] { pf::link_new(temp.path / "absent", temp.path / "x"); });
    }

    // Symlink discipline: a leaf symlink is never followed (no-follow
    // regular-file validation), while a symlinked DIRECTORY resolves —
    // O_NOFOLLOW guards the opened object, not intermediate components.
    {
        const auto real_dir = temp.path / "real dir";
        std::filesystem::create_directories(real_dir);
        const auto link_dir = temp.path / "linkdir";
        std::filesystem::create_directory_symlink(real_dir, link_dir);
        const auto via_link = link_dir / "through.bin";
        pf::write_exclusive(via_link, bytes);
        bt::require(bt::text(real_dir / "through.bin") == payload,
                    "directory symlink resolves for exclusive create");

        const auto leaf = temp.path / "leaf-symlink";
        std::filesystem::create_symlink(awkward.filename(), leaf);
        throws_runtime([&] { pf::write_exclusive(leaf, bytes); });
        throws_runtime([&] { (void) pf::file_sha256(leaf); });
        throws_runtime([&] {
            (void) pf::open_regular(temp.path / "missing-file");
        });
    }

    // Explicit handle path: exclusive create + bounded write + sync + close.
    // A missing directory fails sync loudly rather than passing silently.
    {
        const auto second = temp.path / "second.bin";
        auto handle = pf::open_exclusive(second);
        pf::write_all(handle, bytes);
        pf::sync(handle);
        handle.close();
        bt::require(bt::text(second) == payload,
                    "write_all through an exclusive handle");
        pf::sync_directory(temp.path);
        throws_runtime(
            [&] { pf::sync_directory(temp.path / "missing-dir"); });
    }

    // /dev/null is a character device, not a regular file: the opened-object
    // check must reject it even though open() succeeds.
    throws_runtime([] { (void) pf::open_regular("/dev/null"); });
    throws_runtime([&] { pf::write_exclusive("/dev/null", bytes); });

    // Input-domain violations.
    bt::invalid([&] {
        pf::write_exclusive(std::filesystem::path{}, bytes);
    });
    bt::invalid([&] {
        pf::write_exclusive(std::filesystem::path(std::string("a\0b", 3U)),
                            bytes);
    });
    bt::invalid([] { pf::create_directory(std::filesystem::path{}); });
    bt::invalid([] { (void) pf::file_sha256(std::filesystem::path{}); });
}

// ---- deferred-SIGINT shield ----------------------------------------------------

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) async-signal-visible sentinel
volatile std::sig_atomic_t sentinel_count = 0;

void sentinel_handler(int /*signum*/) {
    // Sentinel only stores sig_atomic_t state — the same contract the shield
    // itself obeys (no allocation/SQL/logging inside a handler).
    sentinel_count = sentinel_count + 1;
}

void checks_signals() {
    // On the main thread the shield swaps the process SIGINT handler for a
    // counting one; interrupts are recorded and honored AFTER the section.
    struct sigaction sentinel {};
    sentinel.sa_handler = &sentinel_handler;
    sigemptyset(&sentinel.sa_mask);
    bt::require(::sigaction(SIGINT, &sentinel, nullptr) == 0,
                "sentinel handler installed");
    {
        pf::InterruptShield shield;
        bt::require(shield.installed(), "main-thread shield installs");
        const int raised0 = std::raise(SIGINT);
        const int raised1 = std::raise(SIGINT);
        bt::require(raised0 == 0 && raised1 == 0, "SIGINT delivered");
        bt::require(shield.deferred() == 2,
                    "SIGINT twice deferred, not delivered");
        bt::require(sentinel_count == 0,
                    "shielded section owned SIGINT exclusively");
        // The injected-hook seam mirrors interrupt.py's note().
        shield.note();
        bt::require(shield.deferred() == 3, "note() records a deferred hit");
    }
    // The previous handler is always restored on exit — a third raise now
    // reaches the sentinel, proving restoration without killing the test.
    bt::require(std::raise(SIGINT) == 0, "post-scope SIGINT delivered");
    bt::require(sentinel_count == 1,
                "previous handler restored on scope exit");
    struct sigaction dfl {};
    dfl.sa_handler = SIG_DFL;
    sigemptyset(&dfl.sa_mask);
    (void) ::sigaction(SIGINT, &dfl, nullptr);

    // Nested shields unwind LIFO like the source's handler chain: the inner
    // scope owns the signal while active, the outer resumes after.
    {
        const pf::InterruptShield outer;
        {
            const pf::InterruptShield inner;
            (void) std::raise(SIGINT);
            bt::require(inner.deferred() == 1 && outer.deferred() == 0,
                        "innermost shield owns the signal");
        }
        (void) std::raise(SIGINT);
        bt::require(outer.deferred() == 1,
                    "outer shield resumes after inner exit");
    }

    // Off the main thread the shield degrades to a flag holder — the exact
    // interrupt.py fallback — with note() as the only recording seam.
    int thread_installed = -1;
    int thread_deferred = -1;
    std::thread worker([&] {
        pf::InterruptShield shield;
        thread_installed = shield.installed() ? 1 : 0;
        shield.note();
        thread_deferred = shield.deferred();
    });
    worker.join();
    bt::require(thread_installed == 0, "off-main shield is a flag holder");
    bt::require(thread_deferred == 1, "flag holder still records note()");
}

// ---- provenance, build identity and clocks -------------------------------------
void checks_provenance() {
    const auto identity = pf::build_identity();
    bt::require(!identity.compiler.empty(), "compiler identity recorded");
    bt::require(!identity.compile_flags.empty(), "flag identity recorded");
    bt::require(!identity.platform.empty(), "platform identity recorded");
#if defined(_LIBCPP_HARDENING_MODE)
    // Compile definitions are part of the recorded flag surface: this TU and
    // bike_platform share the same policy definitions, so when this TU sees
    // the hardening-mode macro the recorded identity must carry it too —
    // two builds differing only in BIKE_LIBCPP_HARDENING must not produce an
    // identical flag string or execution identity.
    bt::require(identity.compile_flags.find("_LIBCPP_HARDENING_MODE=") !=
                    std::string::npos,
                "flag surface records target compile definitions");
#endif
    bt::require(hex64(identity.dependency_lock_sha256),
                "dependency lock hash recorded");
    bt::require(hex64(identity.source_manifest_sha256),
                "source manifest hash recorded");
#if defined(__APPLE__)
    bt::require(!identity.sdk.empty(), "SDK identity recorded");
#endif
    // The produced-executable hash is the running image itself — the same
    // bytes file_sha256 sees through the no-follow reader.
    const auto self = pf::executable_path();
    bt::require(identity.executable_sha256 == pf::file_sha256(self),
                "running image hashed through the bounded reader");
    bt::require(self.filename() == "migration_T06",
                "executable_path names the running image");
    // The numerical-execution axis excludes adapter/packaging fields, so it
    // is stable for this process and untouched by bindings deletion.
    bt::require(hex64(pf::execution_identity()),
                "execution identity digest is a 64-hex sha256");
    bt::require(pf::execution_identity() == pf::execution_identity(),
                "execution identity is stable");
    // The packaging axis is recorded separately from the execution axis —
    // bindings live in the transition extension graph, never in the
    // produced bindings-OFF build.
    bt::require(identity.python_bindings == (BIKE_T06_EXPECT_BINDINGS != 0) &&
                    identity.gui == (BIKE_T06_EXPECT_GUI != 0),
                "packaging axis matches the configured build graph");

    // Executable-relative resource lookup survives a moved install tree:
    // copy this binary plus a share/bike_sim resource into a new prefix and
    // run it there — the lookup anchors on the image's own location.
    {
        const TempDir temp;
        const auto moved = temp.path / "moved tree";
        const auto bin_dir = moved / "bin";
        const auto share_dir = moved / "share" / "bike_sim";
        std::filesystem::create_directories(bin_dir);
        std::filesystem::create_directories(share_dir);
        const auto moved_exe = bin_dir / self.filename();
        std::filesystem::copy_file(self, moved_exe);
        std::filesystem::permissions(
            moved_exe, std::filesystem::perms::owner_exec,
            std::filesystem::perm_options::add);
        pf::write_exclusive(share_dir / "t06-resource.txt",
                            std::as_bytes(std::span(std::string_view{
                                "t06-resource-payload\n"})));
        const auto p = pf::run({moved_exe.string(), std::string(kModeResource)},
                               moved);
        bt::require(p.exit_code == 0 && p.out == "resource-ok\n",
                    "moved install tree resolves resources by image path");
    }

    // Clocks: UTC metadata stamps and the monotonic receipt origin are
    // separate axes — the monotonic clock never substitutes wall time.
    const std::string stamp = pf::utc_now();
    bt::require(stamp.size() == 27U && stamp[4] == '-' && stamp[7] == '-' &&
                    stamp[10] == 'T' && stamp[13] == ':' && stamp[16] == ':' &&
                    stamp[19] == '.' && stamp[26] == 'Z',
                "utc_now emits an ISO-8601 UTC stamp");
    const double m0 = pf::monotonic_seconds();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const double m1 = pf::monotonic_seconds();
    bt::require(m0 >= 0.0 && m1 > m0 && m1 - m0 < 5.0,
                "monotonic origin advances monotonically");
}

} // namespace

int main(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the sentinel.
    const std::span<char *> args(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    // Child-mode dispatch runs before the harness: the same binary is the
    // subprocess fixture, so every case exercises the produced run() path.
    // The outer try is also the bugprone-exception-escape contract: a
    // throwing child path returns a distinct code instead of escaping main.
    try {
    if (args.size() > 1U) {
        const std::string_view mode{args[1]};
        if (mode == kModeSleep) {
            const double seconds =
                args.size() > 2U ? std::stod(args[2]) : 30.0;
            std::this_thread::sleep_for(
                std::chrono::duration<double>(seconds));
            return 0;
        }
        if (mode == kModeFlood) {
            const std::size_t count =
                args.size() > 2U ? std::stoull(args[2]) : 0U;
            const std::string_view marker =
                args.size() > 3U ? std::string_view{args[3]}
                                 : std::string_view{};
            const std::string chunk(65536U, 'e');
            std::size_t left = count;
            while (left > 0U) {
                const std::size_t part = std::min(left, chunk.size());
                std::cerr.write(chunk.data(),
                                static_cast<std::streamsize>(part));
                if (!std::cerr) return 90;
                left -= part;
            }
            std::cerr.flush();
            std::cout << marker;
            std::cout.flush();
            if (!std::cout) return 91;
            return 0;
        }
        if (mode == kModeResource) {
            const auto resource = bike::platform::install_prefix() /
                                  "share" / "bike_sim" / "t06-resource.txt";
            std::error_code error;
            const auto resolved =
                std::filesystem::weakly_canonical(resource, error);
            std::ifstream stream(resolved, std::ios::binary);
            const std::string content{std::istreambuf_iterator<char>{stream},
                                      std::istreambuf_iterator<char>{}};
            if (error || content != "t06-resource-payload\n") return 92;
            std::cout << "resource-ok\n" << std::flush;
            return std::cout ? 0 : 91;
        }
        if (mode == kModeExit) {
            const int code = args.size() > 2U ? std::stoi(args[2]) : 0;
            std::cout << "child-out\n" << std::flush;
            std::cerr << "child-err\n" << std::flush;
            return code;
        }
        return 2;
    }
    // cppcheck's throwInEntryPoint cannot see that bt::run catches every
    // test escape; the outer try covers std::function allocation failure.
    int failures = 0;
    failures += bt::run(checks_produced);
    failures += bt::run(checks_process);
    failures += bt::run(checks_files);
    failures += bt::run(checks_signals);
    failures += bt::run(checks_provenance);
    return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
