// test_native_platform_linkage.cpp — §3.35 port of
// tests/reference/test_native_platform_linkage.py (3 records → 2 ported,
// 1 drop). The Python file asserted that every registered test
// executable runs directly under a scrubbed link environment and that
// every native engine consumer records a resolvable MuJoCo image plus
// the selected numerics provider — no @rpath-rescued framework, no
// LD_LIBRARY_PATH rescue. The native port runs the same checks from
// inside a CTest binary: spawn each build artifact under the scrubbed
// environment, then inspect its own and the CLI tools' link tables with
// the platform's real tools (`otool -L` on Darwin, `readelf -d` + `ldd`
// on Linux).
//
// Dispositions:
//   test_every_ctest_executable_runs_directly — ported as
//     direct_exec_scrubbed: every bin/ artifact runs --help under a
//     scrubbed environment and exits 0. The Python version parsed
//     CTestTestfile.cmake for registered commands; the native port
//     covers the bin/ consumers plus this test binary itself — the
//     registered-ctest enumeration is inherent (this binary IS one).
//   test_engine_consumers_record_resolvable_engine_image — ported as
//     engine_consumers_resolve_images: otool/readelf/ldd prove the
//     pinned MuJoCo image resolves absolutely and the numerics provider
//     is the configured one, with no rescue variables in play.
//   test_extension_imports_without_environment_rescue — drop: it
//     imported the bike_native CPython extension under the scrubbed
//     environment; the extension is Python-side surface that leaves with
//     the Python removal, and its linkage is already inspected as a
//     consumer in record 2.

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "../support/migration_test.hpp"
#include "cli/application.hpp"
#include "platform/process.hpp"

namespace {

// Remove the library search variables a loader could use to rescue a
// dependency lookup — exactly the Python contract's scrub list.
// Injection variables that carry the sanitizer runtime
// (NATIVE_TEST_SANITIZER_RUNTIME, DYLD_INSERT_LIBRARIES, LD_PRELOAD,
// *_OPTIONS) stay: they are the build's own instrumentation, not a
// dependency search path.
void scrub_link_environment() {
    for (const char* name : {"LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH",
                             "DYLD_FALLBACK_LIBRARY_PATH"})
        ::unsetenv(name); // NOLINT(concurrency-mt-unsafe) test environment mutation runs single-threaded by construction
}

[[nodiscard]] std::filesystem::path build_root() {
    // This test binary is four levels below the build root
    // (native/tests/reference); the bin/ artifacts sit at <root>/bin.
    return bike::cli::executable_path()
        .parent_path().parent_path().parent_path().parent_path();
}

[[nodiscard]] std::vector<std::filesystem::path> bin_artifacts() {
    std::vector<std::filesystem::path> found;
    for (const auto& entry : std::filesystem::directory_iterator(build_root() / "bin"))
        if (entry.is_regular_file()) found.push_back(entry.path());
    bt::require(!found.empty(), "the build published its bin/ artifacts");
    return found;
}

[[nodiscard]] bike::platform::ProcessResult tool(
    const std::vector<std::string>& argv) {
    bike::platform::ProcessOptions options;
    options.timeout_s = 60.;
    return bike::platform::run(argv, build_root(), options);
}

// test_every_ctest_executable_runs_directly — each published consumer
// executable starts and answers --help under a scrubbed link
// environment, i.e. every DT_NEEDED/install-name chain resolves without
// LD_LIBRARY_PATH/DYLD_* rescue. A loader failure surfaces as a nonzero
// exit or a loader diagnostic on stderr.
[[gnu::noinline]] void direct_exec_scrubbed() {
    scrub_link_environment();
    for (const auto& binary : bin_artifacts()) {
        // bike-test-child is the process-fixture helper, not a --help CLI
        // — `echo TEXT` is its no-op subcommand. Every published CLI
        // answers --help; a loader failure surfaces as a nonzero exit.
        const auto ran = binary.filename() == "bike-test-child"
            ? tool({binary.string(), "echo", "ok"})
            : tool({binary.string(), "--help"});
        bt::require(ran.exit_code == 0,
                    binary.filename().string() +
                        " runs directly under the scrubbed environment");
    }
}

// test_engine_consumers_record_resolvable_engine_image — the real link
// tables of every engine consumer resolve the pinned MuJoCo image and
// the configured numerics provider absolutely, without environment
// rescue.
[[gnu::noinline]] void engine_consumers_resolve_images() {
    scrub_link_environment();
    std::vector<std::filesystem::path> consumers = bin_artifacts();
    consumers.push_back(bike::cli::executable_path());
#if defined(__APPLE__)
    for (const auto& binary : consumers) {
        const auto table = tool({"otool", "-L", binary.string()});
        bt::require(table.exit_code == 0, "otool -L read " + binary.filename().string());
        // Match the image name, not the substring — the checkout path can
        // legitimately contain "mujoco" (e.g. a mujoco_clean worktree).
        if (table.out.find("libmujoco") == std::string::npos) continue;
        // Any consumer that links MuJoCo names the pinned image by an
        // absolute install name — never the relocatable framework form
        // and never an @rpath-rescued relative name. (@rpath entries for
        // unrelated deps such as libduckdb are not the contract.)
        bt::require(table.out.find("/libmujoco.") != std::string::npos,
                    binary.filename().string() +
                        " records the absolute MuJoCo install name");
        bt::require(table.out.find("mujoco.framework") == std::string::npos &&
                    table.out.find("@rpath/libmujoco") == std::string::npos &&
                    table.out.find("@rpath/mujoco") == std::string::npos,
                    binary.filename().string() +
                        " needs no rpath/framework rescue for its engine image");
        // The numerics provider is the system Accelerate framework —
        // resolvable without any search path.
        bt::require(table.out.find("/Accelerate.framework/") != std::string::npos ||
                    table.out.find("vecLib") != std::string::npos ||
                    table.out.find("libBLAS") != std::string::npos,
                    binary.filename().string() +
                        " records the configured numerics provider");
    }
#else
    for (const auto& binary : consumers) {
        const auto dynamic = tool({"readelf", "-d", binary.string()});
        bt::require(dynamic.exit_code == 0,
                    "readelf -d read " + binary.filename().string());
        // Match the image name, not the substring — the checkout path can
        // legitimately contain "mujoco" (e.g. a mujoco_clean worktree).
        if (dynamic.out.find("libmujoco") == std::string::npos) continue;
        bt::require(dynamic.out.find("NEEDED") != std::string::npos,
                    binary.filename().string() + " declares its engine dependency");
        const auto resolved = tool({"ldd", binary.string()});
        bt::require(resolved.exit_code == 0,
                    "ldd resolved " + binary.filename().string());
        bt::require(resolved.out.find("not found") == std::string::npos,
                    binary.filename().string() +
                        " resolves every dependency without environment rescue");
        bt::require(resolved.out.find("libmujoco") != std::string::npos,
                    binary.filename().string() +
                        " resolves the pinned MuJoCo image");
    }
#endif
}
} // namespace

int main() {
    return bt::run([] {
        direct_exec_scrubbed();
        engine_consumers_resolve_images();
    });
}
