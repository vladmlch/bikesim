// Renderer contract for the bounded human-diagnostic formatter
// (io/diagnostics.hpp): one logical event is one escaped line
// `LEVEL event: message key=value`, text is bounded at 4096 input bytes with
// an explicit truncation marker, escaping never splits a line or a UTF-8 code
// point, numbers are locale-independent, and wrong-typed/uncanonically-named
// events report false without touching the stream.
#include "io/diagnostics.hpp"
#include "support/migration_test.hpp"
#include <algorithm>
#include <array>
#include <sstream>
#include <string>
#include <string_view>

namespace {

using bike::io::DiagnosticField;
using bike::io::DiagnosticKey;
using bike::io::Severity;

void canonical_line() {
    std::ostringstream output;
    const std::array fields{DiagnosticField{
        .key = DiagnosticKey::episode_id, .value = std::int64_t{2}}};
    bt::require(bike::io::write_diagnostic(output, bike::io::Severity::warning,
        "physical.reference.warning", "invalid reference", fields),
        "the canonical write succeeds");
    bt::require(output.str() ==
        "WARN physical.reference.warning: invalid reference episode_id=2\n",
        "canonical event, severity, field type and one line");
    bt::require(bike::io::write_diagnostic(output, Severity::error,
        "cli.test.event", "cause"), "a well-formed write reports success");
}

void severity_spellings_and_ordering() {
    const std::array levels{Severity::trace, Severity::debug, Severity::info,
                            Severity::warning, Severity::error};
    const std::array<std::string_view, 5> tags{"TRACE", "DEBUG", "INFO",
                                             "WARN", "ERROR"};
    for (std::size_t i = 0; i < levels.size(); ++i) {
        std::ostringstream output;
        bt::require(bike::io::write_diagnostic(output, levels[i],
            "cli.test.event", "probe"), "each level writes");
        bt::require(output.str() ==
            std::string(tags[i]) + " cli.test.event: probe\n",
            "level spelling is canonical");
    }
    // Enabled iff severity >= minimum on the declared enum order.
    bt::require(bike::io::is_enabled(Severity::trace, Severity::trace) &&
        bike::io::is_enabled(Severity::error, Severity::trace) &&
        bike::io::is_enabled(Severity::warning, Severity::warning),
        "severity at or above the minimum is enabled");
    bt::require(!bike::io::is_enabled(Severity::debug, Severity::info) &&
        !bike::io::is_enabled(Severity::info, Severity::warning) &&
        !bike::io::is_enabled(Severity::trace, Severity::error),
        "severity below the minimum is disabled");
    // Expensive suppliers only run inside the is_enabled gate.
    int supplied = 0;
    for (const Severity level : {Severity::debug, Severity::info}) {
        if (bike::io::is_enabled(level, Severity::warning)) ++supplied;
    }
    bt::require(supplied == 0,
        "disabled severities never invoke the gated supplier");
    if (bike::io::is_enabled(Severity::warning, Severity::warning)) ++supplied;
    bt::require(supplied == 1, "the enabled severity reaches the supplier");
}

void escaping_keeps_one_line() {
    std::ostringstream output;
    const std::array fields{DiagnosticField{.key = DiagnosticKey::command,
        .value = std::string_view("bike-\nride")}};
    bt::require(bike::io::write_diagnostic(output, Severity::info,
        "cli.test.event",
        "first\nsecond\rcarriage\ttab\"quoted\"\x01\x7f end", fields),
        "escaped writes succeed");
    const std::string line = output.str();
    bt::require(!line.empty() && line.back() == '\n' &&
        std::count(line.begin(), line.end(), '\n') == 1,
        "one logical event produces exactly one output line");
    bt::require(line.find(
            R"(first\nsecond\rcarriage\ttab\"quoted\"\x01\x7F end)")
            != std::string::npos,
        "newlines, carriage returns, tabs, quotes and control bytes escape");
    bt::require(line.find(R"(command="bike-\nride")") != std::string::npos,
        "string field values are quoted and escaped");
}

void bounded_text_preserves_utf8() {
    const std::string oversized(5000, 'a');
    std::ostringstream output;
    bt::require(bike::io::write_diagnostic(output, Severity::warning,
        "cli.test.event", oversized), "oversized write succeeds");
    bt::require(output.str() == "WARN cli.test.event: " +
            std::string(4096, 'a') + "[truncated]\n",
        "oversized text cuts at the input bound with an explicit marker");
    // 4095 bytes + one 3-byte code point + tail: the point starts at byte
    // 4095 and ends past the bound, so the cut lands before it.
    const std::string utf8_tail =
        std::string(4095, 'a') + "\xE2\x82\xAC" + "tail";
    output.str("");
    output.clear();
    bt::require(bike::io::write_diagnostic(output, Severity::warning,
        "cli.test.event", utf8_tail), "utf8-boundary write succeeds");
    bt::require(output.str() == "WARN cli.test.event: " +
            std::string(4095, 'a') + "[truncated]\n",
        "truncation never splits a valid UTF-8 code point");
}

void field_types() {
    std::ostringstream output;
    const std::array fields{
        DiagnosticField{.key = DiagnosticKey::command,
            .value = std::string_view("bike-ride")},
        DiagnosticField{.key = DiagnosticKey::operation,
            .value = std::string_view("seal")},
        DiagnosticField{.key = DiagnosticKey::error_code,
            .value = std::string_view("io_error")},
        DiagnosticField{.key = DiagnosticKey::field_path,
            .value = std::string_view("episodes[0].demand_nm")},
        DiagnosticField{.key = DiagnosticKey::episode_id,
            .value = std::int64_t{-2}},
        DiagnosticField{.key = DiagnosticKey::generation,
            .value = std::uint64_t{7}},
        DiagnosticField{.key = DiagnosticKey::job_id,
            .value = std::string_view("job-42")},
        DiagnosticField{.key = DiagnosticKey::job_index,
            .value = std::int64_t{3}},
        DiagnosticField{.key = DiagnosticKey::duration_ms, .value = 1.5}};
    bt::require(bike::io::write_diagnostic(output, Severity::error,
        "cli.test.event", "failed", fields), "typed field write succeeds");
    bt::require(output.str() ==
        "ERROR cli.test.event: failed command=\"bike-ride\" "
        "operation=\"seal\" error_code=\"io_error\" "
        "field_path=\"episodes[0].demand_nm\" episode_id=-2 generation=7 "
        "job_id=\"job-42\" job_index=3 duration_ms=1.5\n",
        "string fields are quoted, integers stay bare and doubles are "
        "locale-independent");
}

void malformed_input_reports_false() {
    std::ostringstream output;
    const std::array wrong_type{DiagnosticField{.key = DiagnosticKey::episode_id,
        .value = std::string_view("2")}};
    bt::require(!bike::io::write_diagnostic(output, Severity::error,
        "cli.test.event", "cause", wrong_type),
        "a wrong-typed field value is a format error");
    const std::array boolean{DiagnosticField{.key = DiagnosticKey::command,
        .value = true}};
    bt::require(!bike::io::write_diagnostic(output, Severity::error,
        "cli.test.event", "cause", boolean),
        "no declared key accepts a boolean payload today");
    const std::array wrong_double{DiagnosticField{
        .key = DiagnosticKey::duration_ms, .value = std::int64_t{4}}};
    bt::require(!bike::io::write_diagnostic(output, Severity::error,
        "cli.test.event", "cause", wrong_double),
        "integers are not coerced into double keys");
    bt::require(!bike::io::write_diagnostic(output, Severity::error,
        "Not Canonical!", "cause"), "non-canonical event names are rejected");
    bt::require(!bike::io::write_diagnostic(output, Severity::error,
        "", "cause"), "empty event names are rejected");
    bt::require(output.str().empty(),
        "rejected writes leave the stream untouched");
}

void stream_failure_reports_false() {
    std::ostringstream failed;
    failed.setstate(std::ios::failbit);
    bt::require(!bike::io::write_diagnostic(failed, Severity::error,
        "cli.test.event", "cause"),
        "a failed stream reports false and is never retried");
}

} // namespace

int main() {
    return bt::run([] {
        canonical_line();
        severity_spellings_and_ordering();
        escaping_keeps_one_line();
        bounded_text_preserves_utf8();
        field_types();
        malformed_input_reports_false();
        stream_failure_reports_false();
    });
}
