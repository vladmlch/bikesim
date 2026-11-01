// Completion 3.31 (T53). Source: tests/reference/test_recording_cli.py.
//
// Ordinary ``bike-ride`` recording/check-policy parser contracts (R02, R22).
// The oracle drives ``bike_sim.cli.ride.parse_args``; the native port drives
// the production pair ``cli::parse_arguments`` + ``cli::resolve_ride`` —
// the same seam the executable main() uses. Rejection is ``ArgumentError``
// (argparse SystemExit(2)); ``resolved_check_policy`` lands on the resolved
// arguments like the oracle's namespace attribute.
//
// Python-only discharges: ``--backend`` accepts only ``native`` in the
// native contract (``--backend python`` is an unspellable invalid choice),
// so the oracle's python-backend record branch is covered by spec-level
// rejection here plus the shared non-physical gate.
#include "recording_test_support.hpp"
#include "cli/application.hpp"
#include "cli/arguments.hpp"

namespace {

using namespace recording_test;
namespace cli = bike::cli;

const std::string PROFILE = // NOLINT(bugprone-throwing-static-initialization,cert-err58-cpp) static fixture table built once at startup; a throw is a fatal defect
    repo_file("examples/research/viewer_physics_welded.toml").string();

// std::string_view elements accept both literals and the PROFILE string.
cli::Arguments parse(std::initializer_list<std::string_view> input) {
    const auto arguments = std::vector<std::string_view>(input);
    return cli::parse_arguments("bike-ride",
                                std::span<const std::string_view>(arguments));
}
// parse_args(argv) that lands the post-resolution semantics — resolve_ride
// validates combinations and fills resolved_check_policy like the oracle's
// parse_args does.
cli::Arguments resolve(std::initializer_list<std::string_view> input) {
    return cli::resolve_ride(parse(input))->arguments;
}
// The oracle's resolved namespace carried the effective physics mode;
// natively it lives on the resolved setup request, not the `physics`
// option (which is unset whenever --physics-config supplies the mode).
std::string resolved_physics(std::initializer_list<std::string_view> input) {
    auto resolved = cli::resolve_ride(parse(input));
    return resolved->setup.model.physics.value().physics_mode; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
}
void rejected(std::initializer_list<std::string_view> input,
              std::string_view needle, std::string_view message) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    throws<cli::ArgumentError>(
        [&] { (void)cli::resolve_ride(parse(input)); }, needle, message);
}

void test_record_defaults_off_and_check_policy_defaults_unset() {
    const auto args = resolve({"--physics-config", PROFILE});
    bt::require(!args.flag("record") && !args.present("check_policy") &&
                args.text("resolved_check_policy") == "diagnostic",
                "record defaults off; viewer policy resolves diagnostic");
}

void test_out_alone_never_enables_recording() {
    const auto args = resolve({"--physics-config", PROFILE,
                               "--backend", "native", "--out", "out/dir"});
    bt::require(!args.flag("record"), "--out alone never enables recording");
}

void test_record_is_accepted_for_physical_native_runs() {
    for (const bool headless : {false, true}) {
        const auto args = headless
            ? resolve({"--physics-config", PROFILE, "--backend", "native",
                       "--record", "--headless", "--duration", "1", "--no-plots"})
            : resolve({"--physics-config", PROFILE, "--backend", "native",
                       "--record"});
        bt::require(args.flag("record") && args.text("backend") == "native",
                    "record accepted for physical native runs");
        bt::require(resolved_physics(headless
                ? std::initializer_list<std::string_view>{
                    "--physics-config", PROFILE, "--backend", "native",
                    "--record", "--headless", "--duration", "1", "--no-plots"}
                : std::initializer_list<std::string_view>{
                    "--physics-config", PROFILE, "--backend", "native",
                    "--record"}) == "physical",
                "the welded config resolves the physical mode");
    }
}

void test_record_rejected_for_python_backend() {
    // Oracle argv 1: --record with the oracle's implicit python backend
    // rejected. The native default backend IS native, so the same spelling
    // is a valid physical native record — that acceptance is itself the
    // discharge (the rejected combination cannot be spelled natively).
    const auto args = resolve({"--physics-config", PROFILE, "--record"});
    bt::require(args.flag("record") && args.text("backend") == "native",
                "implicit-backend record resolves to the native backend");
    // The contract declares --backend with the single choice "native"; a
    // python-backend record is an invalid choice — the stronger structural
    // form of the oracle's SystemExit(2).
    throws<cli::ArgumentError>(
        [] {
            (void)parse({"--physics-config", PROFILE, "--backend", "python",
                         "--headless", "--duration", "1", "--record"});
        }, "invalid choice", "python backend + record is an invalid choice");
}

void test_record_rejected_for_legacy_physics() {
    rejected({"--physics", "legacy", "--record"}, "--record requires a physical run",
             "record rejected for legacy physics");
    rejected({"--record"}, "--record requires a physical run",
             "record rejected when legacy is the default physics mode");
}

void test_record_accepted_for_research() {
    // backend='python' is unspellable natively (see
    // test_record_rejected_for_python_backend); the shared research gate
    // applies to the surviving native backend.
    const auto args = resolve({"--physics-config", PROFILE, "--backend",
                               "native", "--research", "--record"});
    bt::require(args.flag("record") &&
                args.text("resolved_check_policy") == "diagnostic",
                "record accepted for research with diagnostic default");
}

void test_record_rejected_for_preview() {
    rejected({"--physics-config", PROFILE, "--backend", "native",
              "--preview", "--record"}, "--record requires a physical run",
             "record rejected for preview");
}

void test_check_policy_resolved_before_runtime_and_independent_of_record() {
    auto args = resolve({"--physics-config", PROFILE, "--backend", "native",
                         "--headless", "--duration", "1", "--check-policy",
                         "diagnostic"});
    bt::require(args.text("check_policy") == "diagnostic" &&
                args.text("resolved_check_policy") == "diagnostic" &&
                !args.flag("record"),
                "explicit diagnostic check policy, independent of record");
    args = resolve({"--physics-config", PROFILE, "--backend", "native",
                    "--check-policy", "strict", "--record"});
    bt::require(args.text("resolved_check_policy") == "strict" &&
                args.flag("record"),
                "strict policy independent of record");
    args = resolve({"--physics-config", PROFILE, "--backend", "native",
                    "--headless", "--duration", "1"});
    bt::require(args.text("resolved_check_policy") == "strict",
                "headless keeps its historical strict default");
}

void test_check_policy_accepted_for_python_backend_physical_runs() {
    const auto args = resolve({"--physics-config", PROFILE, "--check-policy",
                               "strict"});
    bt::require(args.text("check_policy") == "strict" &&
                args.text("resolved_check_policy") == "strict",
                "explicit strict check policy accepted");
}

void test_check_policy_rejected_outside_physical_and_research() {
    for (const char *policy : {"strict", "diagnostic"}) {
        rejected({"--physics", "legacy", "--check-policy", policy},
                 "--check-policy requires a physical run",
                 "check-policy rejected for legacy physics");
        const auto args = resolve({"--physics-config", PROFILE, "--research",
                                   "--check-policy", policy});
        bt::require(args.text("resolved_check_policy") == policy,
                    "check-policy accepted for research");
        rejected({"--physics-config", PROFILE, "--preview",
                  "--check-policy", policy},
                 "--check-policy requires a physical run",
                 "check-policy rejected for preview");
    }
}

void test_time_scale_remains_interactive_only() {
    rejected({"--physics-config", PROFILE, "--backend", "native", "--record",
              "--headless", "--duration", "1", "--time-scale", "2"},
             "--time-scale requires an interactive",
             "time-scale stays interactive-only");
    const auto args = resolve({"--physics-config", PROFILE, "--backend",
                               "native", "--record", "--time-scale", "2"});
    bt::require(args.integer("time_scale") == 2 && args.flag("record"),
                "interactive run keeps time-scale and record");
}

void test_unsupported_combination_names_itself() {
    // The oracle's backend-python branch asserts the error names 'native';
    // natively that combination is unspellable (invalid choice naming both
    // the rejected token and the only valid one).
    throws<cli::ArgumentError>(
        [] { (void)parse({"--physics-config", PROFILE, "--backend", "python",
                          "--record"}); },
        "native", "unspellable backend names the native choice");
    throws<cli::ArgumentError>(
        [] {
            (void)cli::resolve_ride(parse({"--physics", "legacy", "--record"}));
        }, "physical", "record + legacy names 'physical'");
    throws<cli::ArgumentError>(
        [] {
            (void)cli::resolve_ride(parse({"--physics-config", PROFILE,
                                           "--backend", "native", "--preview",
                                           "--record"}));
        }, "preview", "record + preview names 'preview'");
}

} // namespace

int main() {
    return bt::run([] {
        test_record_defaults_off_and_check_policy_defaults_unset();
        test_out_alone_never_enables_recording();
        test_record_is_accepted_for_physical_native_runs();
        test_record_rejected_for_python_backend();
        test_record_rejected_for_legacy_physics();
        test_record_accepted_for_research();
        test_record_rejected_for_preview();
        test_check_policy_resolved_before_runtime_and_independent_of_record();
        test_check_policy_accepted_for_python_backend_physical_runs();
        test_check_policy_rejected_outside_physical_and_research();
        test_time_scale_remains_interactive_only();
        test_unsupported_combination_names_itself();
    });
}
