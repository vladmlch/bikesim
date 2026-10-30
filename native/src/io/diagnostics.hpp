// Bounded human diagnostics for non-realtime application boundaries: one
// logical event becomes one escaped line `LEVEL event: message key=value`.
// There is no global level state, callback store, background thread, lock, or
// mutable shared state — callers gate expensive suppliers on is_enabled()
// themselves and event names stay compile-time literals at call sites. This
// is a narrow cold-path formatter, not a general logging framework.
#pragma once
#include <cstdint>
#include <ostream>
#include <span>
#include <string_view>
#include <variant>

namespace bike::io {

// Severity order is the threshold order: a minimum of `warning` disables
// trace/debug/info. Spelling in output is the canonical short tag.
enum class Severity : std::uint8_t { trace, debug, info, warning, error };

// The allowlist of structured context keys. Field types are fixed per key:
// ids/index/generation take integers, duration_ms takes a double, the rest
// take strings. A mismatched DiagnosticValue is a format error.
enum class DiagnosticKey : std::uint8_t {
    command, operation, error_code, field_path,
    episode_id, generation, job_id, job_index, duration_ms
};

using DiagnosticValue = std::variant<bool, std::int64_t, std::uint64_t,
                                     double, std::string_view>;

struct DiagnosticField {
    DiagnosticKey key;
    DiagnosticValue value;
};

[[nodiscard]] bool is_enabled(Severity severity, Severity minimum) noexcept;

// Writes one diagnostic line. Message and string field values are escaped
// (newlines, returns, tabs, quotes, control bytes) so one event is one line,
// and each is cut at 4096 input bytes on a UTF-8 boundary with an explicit
// `[truncated]` marker. Returns true only when the write succeeded; a stream
// failure or format error (uncanonical event name, wrong-typed field value)
// returns false and is never retried or rewritten.
[[nodiscard]] bool write_diagnostic(std::ostream& output, Severity severity,
    std::string_view event, std::string_view message,
    std::span<const DiagnosticField> fields = {}) noexcept;

} // namespace bike::io
