#include "application.hpp"
#include "recording.hpp"
#include "../io/diagnostics.hpp"
#include "../platform/provenance.hpp"
#include "../research/policy.hpp"
#include <array>
#include <iostream>

namespace bike::cli {
std::filesystem::path executable_path() { return platform::executable_path(); }

void emit_failure_diagnostic(std::ostream& output, std::string_view event,
                             const std::exception_ptr& failure,
                             std::optional<std::string_view> policy) {
    try {
        if (failure) std::rethrow_exception(failure);
        (void)io::write_diagnostic(output, io::Severity::error, event,
            "run failed without a captured exception");
    } catch (const research::PolicyFailure&) {
        // Plugin-provided text is untrusted payload: the human line carries a
        // fixed description, a stable error code and the rejected policy
        // identifier — never the raw what(). The exception itself is not
        // rewritten; its full text stays in the propagated evidence.
        const io::DiagnosticField code{.key = io::DiagnosticKey::error_code,
            .value = std::string_view("policy_failure")};
        if (policy) {
            const std::array fields{code, io::DiagnosticField{
                .key = io::DiagnosticKey::field_path, .value = *policy}};
            (void)io::write_diagnostic(output, io::Severity::error,
                "research.policy.rejected", "native policy rejected", fields);
        } else {
            const std::array fields{code};
            (void)io::write_diagnostic(output, io::Severity::error,
                "research.policy.rejected", "native policy rejected", fields);
        }
    } catch (const std::exception& error) {
        (void)io::write_diagnostic(output, io::Severity::error, event,
            error.what());
    } catch (...) {
        (void)io::write_diagnostic(output, io::Severity::error, event,
            "non-standard exception");
    }
}

int dispatch(std::string_view executable, std::span<const std::string_view> input) {
    const auto command = std::filesystem::path(executable).filename().string();
    // The standalone analyst main never links this simulation dispatcher.
    if (command == "bike-recording") return recording_command(input);
    const std::array context{io::DiagnosticField{.key = io::DiagnosticKey::command,
        .value = std::string_view(command)}};
    try {
        const auto args = parse_arguments(command, input);
        if (args.help) { std::cout << help_text(command); return 0; }
        if (command == "bike-ride") return ride_command(args);
        if (command == "bike-research") return research_command(args);
        if (command == "bike-settle") return settle_command(args);
        if (command == "bike-export") return export_command(args);
        if (command == "bike-sim") return simulation_command(args);
        if (command == "bike-playground") return playground_command();
        if (command == "bike-replay") return replay_command(args);
        if (command == "bike-batch") return batch_command(args);
        throw ArgumentError("unknown executable: " + command);
    } catch (const ArgumentError& error) {
        (void)io::write_diagnostic(std::cerr, io::Severity::error,
            "cli.argument_error", error.what(), context);
        return 2;
    } catch (const runtime::InvalidReferenceRun& error) {
        (void)io::write_diagnostic(std::cerr, io::Severity::error,
            "cli.model_invalid", error.what(), context);
        return 2;
    } catch (const std::invalid_argument& error) {
        (void)io::write_diagnostic(std::cerr, io::Severity::error,
            "cli.argument_error", error.what(), context);
        return 2;
    } catch (const std::exception& error) {
        (void)io::write_diagnostic(std::cerr, io::Severity::error,
            "cli.command_failed", error.what(), context);
        return 1;
    }
}
} // namespace bike::cli
