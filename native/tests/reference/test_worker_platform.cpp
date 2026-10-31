// Completion 3.36 tranche B (T53). Source: tests/portability/test_worker_platform.py.
//
// The Python suite drove bike_sim.cli.batch._performance_cores() through
// monkeypatched sys.platform/os.process_cpu_count/subprocess.run seams. The
// native owner is bike::batch::performance_cores()
// (native/src/batch/process_pool.cpp): Darwin reads
// sysctlbyname("hw.perflevel0.physicalcpu"), Linux reads
// sched_getaffinity(0), each falling back to
// std::thread::hardware_concurrency. The ported cases re-derive the expected
// value from the same live system primitives the function is allowed to use
// on this platform — the alternate-platform query cannot exist because that
// branch is not compiled.
//
// Dropped nodes (python-only):
//   * test_linux_spawn_and_retained_darwin_policy,
//     test_invalid_explicit_start_method_creates_no_resources,
//     test_missing_spawn_does_not_fall_back_to_fork — the fork/spawn
//     start-method axis belongs to multiprocessing; native pool workers are
//     std::jthread (process_pool.cpp) and the async runner is a std::thread
//     (native/tests/integration/test_async_close.cpp), so there is no
//     start method to select or reject.
//   * test_threaded_parent_worker_probe, test_mjpython_worker_probe — real
//     Python subprocess/mjpython probes of the interpreter worker; the
//     native interrupt->130 contract lives in cli/batch.cpp (deferred
//     interrupt returns 130) and process_pool.cpp's
//     cancelled-before-worker-launch path.
//   * test_ride_research_forwards_explicit_async_option — argparse plumbing;
//     the option lands on request.experiment.async_telemetry directly
//     (native/tests/integration/test_async_close.cpp research_seed).
//   * test_real_cli_async_entry_points — exit-parity across async/sync CLI
//     entry points is covered natively by
//     test_async_close.cpp::async_close_publishes_identical_samples, which
//     compares the published samples bitwise (stronger than exit parity).
#include "support/migration_test.hpp"
#include "batch/plan.hpp"
#include <algorithm>
#include <thread>
#if defined(__APPLE__)
#include <sys/sysctl.h>
#elif defined(__linux__)
#include <sched.h>
#endif

namespace {

// test_cpu_fallback_respects_process_availability param (None, None, 1):
// whatever the platform source reports, the worker count is positive.
void performance_cores_always_positive() {
    bt::require(bike::batch::performance_cores() >= 1U,
        "the worker count never degenerates to zero workers");
}

// test_linux_cpu_count_does_not_query_apple + the positive
// test_cpu_fallback_respects_process_availability arm: on Linux the count
// comes from sched_getaffinity alone — the sysctl branch is not even
// compiled — with hardware_concurrency as the only fallback. On Darwin the
// test_darwin_performance_core_query/'failure_uses_affinity_fallback' pair
// maps onto the same conditional: sysctl success (>0) governs, otherwise
// the hardware fallback applies.
void performance_cores_uses_platform_source() {
    const unsigned count = bike::batch::performance_cores();
#if defined(__APPLE__)
    int pcores = 0;
    std::size_t size = sizeof(pcores);
    const bool queried = ::sysctlbyname("hw.perflevel0.physicalcpu",
        &pcores, &size, nullptr, 0) == 0 && pcores > 0;
    const unsigned expected = queried ? static_cast<unsigned>(pcores)
        : std::max(1U, std::thread::hardware_concurrency());
    bt::require(count == expected,
        "Darwin worker count is the performance-core sysctl when it answers, "
        "else the hardware fallback");
#elif defined(__linux__)
    cpu_set_t available;
    CPU_ZERO(&available);
    unsigned expected;
    if (::sched_getaffinity(0, sizeof(available), &available) == 0 &&
        CPU_COUNT(&available) > 0) {
        expected = static_cast<unsigned>(CPU_COUNT(&available));
    } else {
        expected = std::max(1U, std::thread::hardware_concurrency());
    }
    bt::require(count == expected,
        "Linux worker count is the process-affinity count, never an Apple query");
#else
    bt::require(count == std::max(1U, std::thread::hardware_concurrency()),
        "unqualified platforms take the hardware fallback");
#endif
}

// The affinity mask — not the machine's total CPU count — is the source on
// Linux (Python param (4, 64, 4)): narrowing this process's affinity narrows
// the reported count, and restoring it restores the count.
void linux_affinity_governs_worker_count() {
#if defined(__linux__)
    cpu_set_t original;
    CPU_ZERO(&original);
    if (::sched_getaffinity(0, sizeof(original), &original) == 0 &&
        CPU_COUNT(&original) > 1) {
        int first = -1;
        for (int index = 0; index < CPU_SETSIZE; ++index) {
            if (CPU_ISSET(index, &original)) { first = index; break; }
        }
        cpu_set_t narrowed;
        CPU_ZERO(&narrowed);
        CPU_SET(first, &narrowed);
        if (::sched_setaffinity(0, sizeof(narrowed), &narrowed) == 0) {
            bt::require(bike::batch::performance_cores() == 1U,
                "a single-CPU affinity mask yields exactly one worker");
            (void)::sched_setaffinity(0, sizeof(original), &original);
            bt::require(
                bike::batch::performance_cores() ==
                    static_cast<unsigned>(CPU_COUNT(&original)),
                "restoring the affinity mask restores the worker count");
        }
    }
#endif
    bt::require(bike::batch::performance_cores() >= 1U,
        "worker count stays positive after the affinity check");
}

} // namespace

int main() {
    return bt::run([] {
        performance_cores_always_positive();
        performance_cores_uses_platform_source();
        linux_affinity_governs_worker_count();
    });
}
