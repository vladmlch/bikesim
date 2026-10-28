// Port of tests/reference/test_pedelec_replay_contract.py — motor-only
// open-loop replay is explicitly unsupported by the pedelec permission gate
// at every entry point: schedule, replay report, CLI, fidelity case and
// sweep.
#include "replay_reference_support.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"
#include "validation/scenarios.hpp"

namespace {
namespace rr = replay_reference;
namespace bv = bike::validation;
using rr::field;
using rr::integer;
using rr::text;
using bike::io::Value;

// test_motor_only_schedule_is_explicitly_unsupported ×4.
void schedule_rejects(double time) {
    rr::rejects([&] { (void)bv::open_loop_schedule(time); }, "motor-only open-loop");
}

// test_unsupported_motor_replay_reports_failure_before_building — the
// open-loop diagnosis lands before any fixture file is read.
void replay_report() {
    rr::Directory tmp;
    const auto report = bv::run_replay(Value{Value::Object{
        {"mode", text("open-loop")},
        {"physics_profile", text((tmp.path / "missing-physics.toml").string())},
        {"track", text((tmp.path / "missing-track.toml").string())}}}, rr::repo_root());
    bt::require(rr::text_of(report, "mode") == "open-loop" &&
        rr::text_of(report, "diagnosis") == "unsupported_experiment" &&
        !rr::flag(report, "completed_requested_duration") &&
        bike::io::as_array(field(report, "rows"), "rows").empty(),
        "the unsupported diagnosis reports before any fixture is built");
    bt::require(rr::text_of(report, "error").find("motor-only open-loop") != std::string_view::npos,
        "the retained error names the motor-only gate");
    bt::require(!rr::flag(field(report, "support_evidence"), "valid_for_learning"),
        "the failed run carries no learning-valid support evidence");
}

// test_motor_only_cli_has_nonzero_exit_and_retains_error — bike-validate
// rider-replay exits 1 and prints the retained error.
void cli_exit() {
    rr::Directory tmp;
    // bike-validate lives at <build-root>/bin; the test binary sits four
    // levels down in native/tests/reference.
    const auto binary = bike::platform::executable_path()
        .parent_path().parent_path().parent_path().parent_path() / "bin" / "bike-validate";
    bt::require(std::filesystem::exists(binary), "the bike-validate artifact is built");
    bike::platform::ProcessOptions options;
    options.timeout_s = 120.;
    const auto result = bike::platform::run({binary.string(), "rider-replay",
        "--mode", "open-loop", "--physics-config", (tmp.path / "missing.toml").string(),
        "--track", (tmp.path / "missing-track.toml").string(),
        "--output", (tmp.path / "failure.json").string(),
        "--root", rr::repo_root().string()}, tmp.path, options);
    bt::require(result.exit_code == 1, "a failed case reports exit 1");
    bt::require(result.out.find("motor-only open-loop") != std::string::npos,
        "the printed report retains the motor-only diagnosis");
}

// test_motor_ramp_fidelity_rejects_before_loading_the_fixture and
// test_unsupported_fidelity_case_cannot_report_pass — the sweep's
// initialization_errors entry names the rejection; every series row is an
// initialization_failure and the report cannot pass.
void sweep_cannot_pass() {
    rr::Directory tmp;
    const auto report = bv::fidelity_sweep(Value{Value::Object{
        {"cases", Value{Value::Array{text("motor_ramp_grade")}}},
        {"physics_profile", text((tmp.path / "missing.toml").string())},
        {"time_steps", Value{Value::Array{rr::real(.00125), rr::real(.000625), rr::real(.0003125)}}},
        {"road_steps", Value{Value::Array{rr::real(.01), rr::real(.005), rr::real(.0025)}}},
        {"station_counts", Value{Value::Array{integer(128), integer(256), integer(512)}}},
        {"output", text((tmp.path / "out").string())}}}, rr::repo_root());
    bt::require(!rr::flag(report, "passed"),
        "an unsupported case cannot report pass");
    bt::require(rr::text_of(field(report, "initialization_errors"), "motor_ramp_grade")
        .find("motor-only open-loop") != std::string_view::npos,
        "the initialization error names the motor-only gate before fixture loading");
    const auto& series = bike::io::as_array(field(report, "series"), "series");
    bt::require(!series.empty(), "every candidate resolution produced a series row");
    for (const auto& row : series)
        bt::require(rr::text_of(row, "termination") == "initialization_failure",
            "every series row is an initialization failure");
}
} // namespace
int main() {
    return bt::run([] {
        schedule_rejects(0.);
        schedule_rejects(1.5);
        schedule_rejects(3.);
        schedule_rejects(5.);
        replay_report();
        cli_exit();
        sweep_cannot_pass();
    });
}
