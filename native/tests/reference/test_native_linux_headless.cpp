// test_native_linux_headless.cpp — §3.35 port of
// tests/reference/test_native_linux_headless.py (4 records → 2 ported, 2
// drops). The Python file verified that stepping never needs a display
// server and that the real CLI keeps its exit/outcome contract without
// DISPLAY/WAYLAND_DISPLAY/MUJOCO_GL. Native has no rendering or GL
// symbols at all, so the honest port drives the real `bike-ride`
// executable under a scrubbed display environment and asserts the same
// outcomes the Python subprocess probe observed.
//
// Dispositions:
//   test_four_step_driver_without_rendering[python] — drop: the
//     python-backend parameter exercised the (now migrating) Python
//     frontend, not a native contract.
//   test_four_step_driver_without_rendering[native] — ported as
//     driver_steps_without_display: a scrubbed-env `bike-ride` advances
//     the full requested duration with no display variables and leaves
//     no output residue.
//   test_native_cli_without_display_retains_real_outcome[strict /
//     diagnostic] — ported as cli_outcome_retained_without_display: the
//     violating physics run keeps its real model-invalid outcome and
//     exit status under either check policy.
//   test_headless_frontend_closes_without_initializing_rendering — drop:
//     Python frontend lifecycle (Qt/matplotlib cleanup); there is no
//     native frontend to close.
//   test_failed_native_model_construction_cleans_bootstrap_without_
//     rendering — drop: Python bootstrap tempdir mechanics; the native
//     runtime's failure-path cleanup is covered by
//     reference.native_platform_recording (crash staging record).

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../support/migration_test.hpp"
#include "../support/temporary_directory.hpp"
#include "recording_test_support.hpp"
#include "cli/application.hpp"
#include "io/json.hpp"
#include "platform/process.hpp"

namespace {
namespace rt = recording_test;

using bike::io::Value;

// Scrub every variable that could inject a display/GL backend or rescue a
// library lookup. Sanitizer injection variables (NATIVE_TEST_SANITIZER_RUNTIME
// and friends) are deliberately left alone — they carry the build's
// sanitizer runtime, not a display or dependency path.
void scrub_display_environment() {
    for (const char* name : {"DISPLAY", "WAYLAND_DISPLAY", "MIR_SOCKET",
                             "XDG_SESSION_TYPE", "MUJOCO_GL", "PYOPENGL_PLATFORM",
                             "LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH",
                             "DYLD_FALLBACK_LIBRARY_PATH"})
        ::unsetenv(name); // NOLINT(concurrency-mt-unsafe) test environment mutation runs single-threaded by construction
}

[[nodiscard]] const Value& reqv(const Value& row, const char* name) {
    const auto* member =
        bike::io::find_member(bike::io::as_object(row, name), name);
    bt::require(member != nullptr, std::string("member .") + name + " is present");
    return *member;
}

[[nodiscard]] std::filesystem::path bin_dir() {
    // bike-* CLIs land at <build-root>/bin; this test's binary is four
    // levels down in native/tests/reference (same layout as test_batch).
    return bike::cli::executable_path()
        .parent_path().parent_path().parent_path().parent_path() / "bin";
}

// The welded TOML staged with its rider-profile JSON siblings alongside —
// the paths inside the TOML are relative to the working directory the
// CLI runs in. Healthy twin of recording_test_support::violating_physics.
[[nodiscard]] std::filesystem::path staged_physics(
    const std::filesystem::path& directory) {
    const auto target = directory / "physics.toml";
    std::ofstream out(target);
    out << bt::text(rt::repo_file("examples/research/viewer_physics_welded.toml"));
    for (const char* sibling : {"rider_joint_envelope_anatomical.json",
                                "rider_strength_reference.json"})
        std::filesystem::copy_file(rt::repo_file("examples/research") / sibling,
                                   directory / sibling,
                                   std::filesystem::copy_options::overwrite_existing);
    return target;
}

[[nodiscard]] bike::platform::ProcessResult ride(
    const std::filesystem::path& physics, const std::filesystem::path& cwd,
    const char* duration, const char* policy,
    const std::filesystem::path& out) {
    const auto track = rt::repo_file("examples/research/rough_uphill_savage.toml");
    bike::platform::ProcessOptions options;
    options.timeout_s = 120.;
    return bike::platform::run(
        {(bin_dir() / "bike-ride").string(), "--physics-config", physics.string(),
         "--track", track.string(), "--headless", "--duration", duration,
         "--no-plots", "--check-policy", policy, "--decimate", "1",
         "--out", out.string()},
        cwd, options);
}

// test_four_step_driver_without_rendering[native] — a real `bike-ride`
// under a scrubbed display environment steps the whole requested horizon
// and reports a real sealed outcome. --no-plots plus --headless means no
// rendering path exists to initialise; no --record means the run must
// not leave an output payload behind.
[[gnu::noinline]] void driver_steps_without_display() {
    scrub_display_environment();
    bt::TemporaryDirectory const root("linux-headless-driver");
    const auto physics = staged_physics(root.path());
    const auto out = root.path() / "out";
    const auto result = ride(physics, root.path(), "0.005", "diagnostic", out);
    const auto summary = bike::io::parse_recording_json(result.out);
    bt::require(rt::as_text(reqv(reqv(summary, "outcome"), "reason")) ==
                    "duration_reached",
                "the display-free driver reaches the requested duration");
    bt::require(rt::as_real(reqv(summary, "duration_s")) == 0.005,
                "the full 4-step (dt 0.00125) horizon advanced");
    bt::require(!std::filesystem::exists(out),
                "an unrecorded headless run leaves no output residue");
}

// test_native_cli_without_display_retains_real_outcome — the real CLI
// keeps its outcome contract with no display variables: the violating
// physics is model-invalid under both policies. Strict reports the
// failure on stderr and writes no summary; diagnostic completes and
// seals the invalid model status into the summary JSON. Python expected
// reason=invalid_controller; the native CLI seals the same invalid-model
// contract as simulation_error naming — asserted on model_valid and the
// failure naming instead (see §3.35 divergence notes in
// test_native_platform_recording.cpp).
[[gnu::noinline]] void cli_outcome_retained_without_display() {
    scrub_display_environment();
    bt::TemporaryDirectory const root("linux-headless-outcome");
    const auto physics = rt::violating_physics(root.path() / "violating.toml");

    const auto strict = ride(physics, root.path(), "0.04", "strict",
                             root.path() / "strict-out");
    bt::require(strict.exit_code == 2,
                "the strict headless run keeps its nonzero exit");
    bt::require(strict.out.find("\"duration_s\"") == std::string::npos,
                "the strict failure writes no run summary");
    bt::require(strict.err.find("invalid reference step") != std::string::npos,
                "the strict failure names the real model violation");
    bt::require(!std::filesystem::exists(root.path() / "strict-out"),
                "the strict failure publishes no output directory");

    const auto diagnostic = ride(physics, root.path(), "0.01", "diagnostic",
                                 root.path() / "diag-out");
    bt::require(diagnostic.exit_code == 2,
                "the diagnostic headless run keeps the model-invalid exit");
    const auto summary = bike::io::parse_recording_json(diagnostic.out);
    bt::require(!rt::as_bool(reqv(reqv(summary, "model_status"), "model_valid")),
                "the sealed summary keeps the real invalid model status");
    bt::require(rt::as_text(reqv(reqv(summary, "outcome"), "reason")) ==
                    "duration_reached",
                "the diagnostic run seals its real duration_reached outcome");
}
} // namespace

int main() {
    return bt::run([] {
        driver_steps_without_display();
        cli_outcome_retained_without_display();
    });
}
