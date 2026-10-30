// Completion 3.33b. Source: tests/reference/test_research_recording_lifecycle.py.
// Research CLI opt-in, the zero-step policy-load failure publication and the
// deferred-interrupt lifecycle probes. SIGINT delivery is real: the two
// second-Ctrl-C probes run as isolated child processes like the t36
// signal/fatal pattern.
//
// Substitution notes:
//  * research.main([...]) forwards to the native run_research entry — the
//    tests call cli::research_command on parsed bike-research arguments and
//    capture its streams the way capsys does.
//  * The monkeypatched load_policy failure is the native loader's own
//    invalid_argument at the same contract position inside run_research.
//  * finish(primary=..., stop=True) returns the path in the Python adapter;
//    the native owner rethrows the primary after publishing, so the port
//    catches the rethrow and reads the idempotent finish() — identical bytes.
//  * The monkeypatched env.seal_episode SIGINT injection is
//    PolicySession::seal_episode's current-policy-metadata probe: a policy
//    double that raises inside metadata() lands the second Ctrl-C inside the
//    shielded seal. The launch is therefore session-form — the same
//    ownership pair launch.attach(session) expresses in the source.
//  * The publish-seam injection (custom publisher raising SIGINT before
//    write_recording) is a ResearchPublication double delegating to the real
//    publication — raise first, then the owned write, exactly like the
//    source's `publish` wrapper.
//  * The Python '__notes__' probe is add_note on the KeyboardInterrupt; the
//    native twin is the OperatorInterrupted message text, which carries the
//    same 'operator interrupt deferred' note the FailureReport records.
//  * Python's Probe monkeypatch captures finish()'s own shield and notes it
//    from the publisher seam. The native shield is a concrete finish() local
//    nothing outside can name, and a real raise on a handler-less worker
//    would kill the child — so the KeyboardInterrupt-from-finish leg has no
//    native twin. The port pins the two halves that are real: the flag-holder
//    degrade and the note() seam it relies on, plus an off-main finish
//    completing and publishing.
//  * launch_failure.error_type is Python exception-class surface; the native
//    launch_failure record carries the exception message under 'error'.
#include "research_reference_support.hpp"
#include "cli/application.hpp"
#include "cli/arguments.hpp"
#include "io/json.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"
#include "platform/signals.hpp"
#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>
#include <ranges>
#include <sstream>
#include <thread>

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;
namespace rec = bike::recording;
namespace io = bike::io;
namespace rd = bike::recording::detail;
namespace cli = bike::cli;

// Runs cli::research_command(args) with its std streams captured — the
// capsys twin, keeping ERROR/summary lines off the CTest log while letting a
// failure message say what the command printed.
[[nodiscard]] int run_cli(const cli::Arguments &args,
                          std::string *out, std::string *err) {
    std::ostringstream out_stream, err_stream;
    auto *const cout_buf = std::cout.rdbuf(out_stream.rdbuf());
    auto *const cerr_buf = std::cerr.rdbuf(err_stream.rdbuf());
    const auto code = cli::research_command(args);
    std::cout.rdbuf(cout_buf);
    std::cerr.rdbuf(cerr_buf);
    if (out) *out = out_stream.str();
    if (err) *err = err_stream.str();
    return code;
}

// The metadata() probe PolicySession::seal_episode runs inside the shielded
// seal: arming this double delivers the second Ctrl-C there — the
// monkeypatched env.seal_episode injection's native twin.
class SealInterruptPolicy final : public CountingPolicy {
public:
    ~SealInterruptPolicy() override;
    bool armed = false;
    [[nodiscard]] io::Value metadata() const override {
        if (armed) (void)std::raise(SIGINT);
        return CountingPolicy::metadata();
    }
};
SealInterruptPolicy::~SealInterruptPolicy() = default;

// The custom publisher seam: raise the second Ctrl-C inside finalize, then
// run the real publication write.
class InterruptingPublication final : public research::ResearchPublication {
public:
    explicit InterruptingPublication(std::shared_ptr<research::ResearchPublication> inner)
        : inner_(std::move(inner)) {}
    ~InterruptingPublication() override;
    [[nodiscard]] std::filesystem::path
    publish(std::span<const research::EpisodeReceipt> receipts,
            std::span<const research::OperatorEvent> events) override {
        (void)std::raise(SIGINT);
        return inner_->publish(receipts, events);
    }
private:
    std::shared_ptr<research::ResearchPublication> inner_;
};
InterruptingPublication::~InterruptingPublication() = default;

// tests/reference/test_research_recording_lifecycle.py::
//   test_research_parser_defaults_off_and_resolves_check_policy
void test_research_parser_defaults_off_and_resolves_check_policy() {
    // research.resolve_check_policy(args): the explicit-policy value, or the
    // diagnostic/strict fallback — and the strict conflict rejection.
    const auto resolved = [](const cli::Arguments &args) {
        return args.text("check_policy",
                         args.flag("diagnostic_model_limits") ? "diagnostic" : "strict");
    };
    const auto empty = cli::parse_arguments("bike-research",
                                            std::span<const std::string_view>{});
    bt::require(!empty.flag("record"), "--record defaults off");
    cli::validate_research_arguments(empty);
    bt::require(resolved(empty) == "strict", "the default check policy resolves strict");
    for (const std::string_view policy : {"strict", "diagnostic"}) {
        const std::array<std::string_view, 3> argv{"--record", "--check-policy", policy};
        const auto args = cli::parse_arguments("bike-research", argv);
        bt::require(args.flag("record") && resolved(args) == policy,
                    "--record --check-policy resolves the explicit policy");
        cli::validate_research_arguments(args);
    }
    const std::array<std::string_view, 3> argv{"--check-policy", "strict",
                                               "--diagnostic-model-limits"};
    const auto conflict = cli::parse_arguments("bike-research", argv);
    invalid_with([&] { cli::validate_research_arguments(conflict); },
                 "conflicts with --check-policy strict");
}

// tests/reference/test_research_recording_lifecycle.py::
//   test_policy_import_failure_publishes_zero_step_evidence
void test_policy_import_failure_publishes_zero_step_evidence() {
    Directory directory;
    const auto output = directory.path / "run";
    const auto out_text = output.string();
    const std::vector<std::string_view> argv{
        "--record", "--out", out_text, "--scenario", "flat", "--rider", "lumped",
        "--duration", ".04", "--policy", "missing:factory"};
    const auto args = cli::parse_arguments("bike-research", argv);
    std::string out, err;
    const auto code = run_cli(args, &out, &err);
    bt::require(code == 3,
                "a policy factory load failure exits 3 after publishing: " + err);
    // manifests = list(output.glob('*/manifest.json'))
    std::vector<std::filesystem::path> manifests;
    for (const auto &entry : std::filesystem::directory_iterator(output))
        if (std::filesystem::is_regular_file(entry.path() / "manifest.json"))
            manifests.push_back(entry.path() / "manifest.json");
    bt::require(manifests.size() == 1, "exactly one manifest publishes");
    rec::Recording saved(manifests.front().parent_path());
    const auto episode = saved.episode();
    bt::require(io::as_int64(rd::field(episode, "physical_rows"), "physical_rows") == 0 &&
                io::as_int64(rd::field(episode, "complete_control_steps"),
                             "complete_control_steps") == 0,
                "the failed launch publishes zero-step evidence");
    bt::require(!io::as_bool(rd::field(episode, "replay_eligible"), "replay_eligible"),
                "policy-load failure evidence is not replay eligible");
    bt::require(row_count(saved, "SELECT count(*) AS n FROM commands_requested") == 0,
                "no command was ever requested");
    bt::require(row_count(saved, "SELECT count(*) AS n FROM observations") == 1,
                "only the startup observation is retained");
}

// tests/reference/test_research_recording_lifecycle.py::
//   test_out_without_record_never_publishes
void test_out_without_record_never_publishes() {
    Directory directory;
    const auto output = directory.path / "run";
    const auto out_text = output.string();
    const std::vector<std::string_view> argv{
        "--out", out_text, "--scenario", "flat", "--rider", "lumped",
        "--duration", ".01"};
    const auto args = cli::parse_arguments("bike-research", argv);
    std::string out, err;
    const auto code = run_cli(args, &out, &err);
    bt::require(code == 0, "a clean headless run exits 0: " + err);
    bt::require(!std::filesystem::exists(output),
                "without --record the CLI writes no publication");
    // The Python source asserts the retention legs on the env the patched
    // make_environment returned; the native CLI owns its env internally, so
    // the twin pins the same off-mode contract on a like-for-like env (the
    // capture parity test covers it exhaustively).
    auto env = environment({.duration_s = .01,
                            .capture = runtime::ResearchCaptureMode::off});
    (void)step(*env, motor_control(0.));
    bt::require(env->commands_requested().empty() && env->commands_applied().empty() &&
                env->observations().empty() && env->transitions().empty() &&
                sw::integer(sw::required(env->status(), "recorded_rows")) == 0,
                "off capture retains no history");
    env->close();
}

// tests/reference/test_research_recording_lifecycle.py::
//   test_finish_second_ctrl_c_during_seal_journals_and_publishes — child mode.
void test_finish_second_ctrl_c_during_seal_journals_and_publishes() {
    Directory directory;
    auto [env, publication] = recording_env(directory.path);
    auto owner = std::make_unique<SealInterruptPolicy>();
    auto *const policy = owner.get();
    research::PolicySession session(std::move(env), std::move(owner));
    research::ResearchRun launch(session, publication);
    (void)step(session.environment(), motor_control(0.));
    // The PolicySession constructor already captured its policy_metadata;
    // arming now lands the second Ctrl-C inside the shielded seal.
    policy->armed = true;
    bool interrupted = false;
    try {
        (void)launch.finish();
    } catch (const research::OperatorInterrupted &) {
        interrupted = true;
    }
    bt::require(interrupted, "the deferred interrupt is honored after publication");
    const auto path = launch.finish(); // launch.path twin
    bt::require(path.has_value(), "the publication path is retained");
    bt::require(std::filesystem::is_regular_file(*path / "manifest.json") &&
                std::filesystem::is_regular_file(*path / "run.duckdb"),
                "manifest and database publish");
    const auto kinds = event_kinds(*path);
    bt::require(!kinds.empty() && kinds.front() == "episode_started",
                "the first journaled event opens the episode");
    bt::require(std::ranges::find(kinds, "episode_ended") != kinds.end(),
                "the sealed episode end is journaled");
    bt::require(std::ranges::find(kinds, "operator_stop_deferred") != kinds.end(),
                "the interrupt deferred during seal is journaled");
    session.close();
}

// tests/reference/test_research_recording_lifecycle.py::
//   test_finish_second_ctrl_c_during_finalize_is_noted_and_publishes — child.
void test_finish_second_ctrl_c_during_finalize_is_noted_and_publishes() {
    Directory directory;
    auto [env, publication] = recording_env(directory.path);
    research::ResearchRun launch(
        *env, std::make_shared<InterruptingPublication>(publication));
    (void)step(*env, motor_control(0.));
    bool interrupted = false;
    std::string message;
    try {
        (void)launch.finish();
    } catch (const research::OperatorInterrupted &error) {
        interrupted = true;
        message = error.what();
    }
    bt::require(interrupted &&
                    message.find("operator interrupt deferred") != std::string::npos,
                "the deferred interrupt re-raises carrying the note");
    const auto path = launch.finish(); // launch.path twin
    bt::require(path.has_value() &&
                std::filesystem::is_regular_file(*path / "manifest.json"),
                "the publication lands before the interrupt is honored");
    const auto kinds = event_kinds(*path);
    bt::require(std::ranges::find(kinds, "operator_stop_deferred") == kinds.end(),
                "a deferred interrupt inside finalize is not journaled");
    env->close();
}

// tests/reference/test_research_recording_lifecycle.py::
//   test_finish_off_main_thread_shield_degrades_but_records
void test_finish_off_main_thread_shield_degrades_but_records() {
    // created[0]._previous is None: off the main thread the shield is a flag
    // holder that installs no handler, and the note() seam still records the
    // deferred interrupt deterministically.
    std::atomic<bool> installed{true};
    std::atomic<int> deferred{-1};
    {
        std::thread probe([&] {
            bike::platform::InterruptShield shield;
            installed.store(shield.installed(), std::memory_order_relaxed);
            shield.note();
            deferred.store(shield.deferred(), std::memory_order_relaxed);
        });
        probe.join();
    }
    bt::require(!installed.load(std::memory_order_relaxed) &&
                deferred.load(std::memory_order_relaxed) == 1,
                "off the main thread the shield is a flag holder and note() records");
    // finish() on a worker thread completes and publishes — the half of the
    // source test that has a native twin (see the header note for the
    // monkeypatched-shield leg that does not).
    Directory directory;
    auto [env, publication] = recording_env(directory.path);
    research::ResearchRun launch(*env, publication);
    (void)step(*env, motor_control(0.));
    std::optional<std::filesystem::path> path;
    std::exception_ptr failure;
    {
        std::thread worker([&] {
            try { path = launch.finish(); }
            catch (...) { failure = std::current_exception(); }
        });
        worker.join();
    }
    if (failure) std::rethrow_exception(failure);
    bt::require(path.has_value() &&
                std::filesystem::is_regular_file(*path / "manifest.json"),
                "an off-main finish publishes the recording");
    env->close();
}

// tests/reference/test_research_recording_lifecycle.py::
//   test_frontend_failure_between_windows_is_not_replay_eligible
void test_frontend_failure_between_windows_is_not_replay_eligible() {
    Directory directory;
    auto [env, publication] = recording_env(directory.path);
    research::ResearchRun launch(*env, publication);
    (void)step(*env, motor_control(0.));
    throws_runtime([&] {
        (void)launch.finish(std::make_exception_ptr(
                                std::runtime_error("frontend callback injection")), true); },
        "frontend callback injection");
    const auto path = launch.finish(); // launch.path twin
    env->close();
    bt::require(path.has_value(), "the failure seal still publishes");
    rec::Recording saved(*path);
    const auto episode = saved.episode();
    bt::require(io::as_int64(rd::field(episode, "complete_control_steps"),
                             "complete_control_steps") == 1,
                "the completed window is retained");
    bt::require(!io::as_bool(rd::field(episode, "replay_eligible"), "replay_eligible"),
                "a frontend failure seal is not replay eligible");
    const auto summary =
        io::parse_recording_json(rd::string(rd::field(episode, "summary_json")));
    const auto &research_node = rd::field(summary, "research");
    bt::require(!io::as_bool(rd::field(research_node, "valid_for_learning"),
                             "valid_for_learning"),
                "a launch failure poisons valid_for_learning");
    const auto &launch_failure =
        rd::field(rd::field(research_node, "run_metadata"), "launch_failure");
    bt::require(io::as_string(rd::field(launch_failure, "error"), "error") ==
                    "frontend callback injection",
                "the launch failure records the primary exception");
}

} // namespace

int main(int argc, char **argv) {
    return bt::run([&] {
        const auto arguments = std::views::counted(argv, argc);
        if (argc == 2 && std::string_view(arguments[1]) == "--seal-child") {
            test_finish_second_ctrl_c_during_seal_journals_and_publishes();
            return;
        }
        if (argc == 2 && std::string_view(arguments[1]) == "--finalize-child") {
            test_finish_second_ctrl_c_during_finalize_is_noted_and_publishes();
            return;
        }
        bt::require(argc == 1, "unexpected 3.33b lifecycle arguments");
        test_research_parser_defaults_off_and_resolves_check_policy();
        test_policy_import_failure_publishes_zero_step_evidence();
        test_out_without_record_never_publishes();
        test_finish_off_main_thread_shield_degrades_but_records();
        test_frontend_failure_between_windows_is_not_replay_eligible();
        for (const auto child : {"--seal-child", "--finalize-child"}) {
            const auto result = bike::platform::run(
                {bike::platform::executable_path().string(), child},
                std::filesystem::temp_directory_path(),
                {.cancellation = {}, .timeout_s = 300.,
                 .max_output_bytes = std::size_t{1024U} * std::size_t{1024U}});
            bt::require(result.exit_code == 0,
                        std::string(child) + ": " + result.stderr_text + result.stdout_text);
        }
    });
}
