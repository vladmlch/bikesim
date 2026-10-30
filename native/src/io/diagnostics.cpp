#include "diagnostics.hpp"
#include "float_format.hpp"
#include <array>
#include <charconv>
#include <memory>
#include <utility>

namespace bike::io {
namespace {

constexpr std::size_t kMaxTextBytes = 4096;
constexpr std::string_view kTruncatedMark = "[truncated]";

[[nodiscard]] constexpr char hex_digit(unsigned int nibble) noexcept {
    return nibble < 10U ? static_cast<char>('0' + nibble)
                        : static_cast<char>('A' + nibble - 10U);
}

[[nodiscard]] constexpr std::string_view severity_tag(Severity severity) noexcept {
    switch (severity) {
        case Severity::trace:   return "TRACE";
        case Severity::debug:   return "DEBUG";
        case Severity::info:    return "INFO";
        case Severity::warning: return "WARN";
        case Severity::error:   return "ERROR";
    }
    return "ERROR";
}

[[nodiscard]] constexpr std::string_view key_name(DiagnosticKey key) noexcept {
    switch (key) {
        case DiagnosticKey::command:     return "command";
        case DiagnosticKey::operation:   return "operation";
        case DiagnosticKey::error_code:  return "error_code";
        case DiagnosticKey::field_path:  return "field_path";
        case DiagnosticKey::episode_id:  return "episode_id";
        case DiagnosticKey::generation:  return "generation";
        case DiagnosticKey::job_id:      return "job_id";
        case DiagnosticKey::job_index:   return "job_index";
        case DiagnosticKey::duration_ms: return "duration_ms";
    }
    return "unknown";
}

// Canonical event names are compile-time literals at call sites: lowercase
// dotted identifiers only, so user input can never smuggle a level, colon or
// line break into the event slot.
[[nodiscard]] bool canonical_event(std::string_view event) noexcept {
    if (event.empty() || event.front() < 'a' || event.front() > 'z') return false;
    for (const char c : event) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                        c == '.' || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

// Cuts text at the input bound without splitting a UTF-8 code point: the cut
// backs off continuation bytes to the last code-point boundary. The flag
// reports whether bytes were dropped.
[[nodiscard]] std::pair<std::string_view, bool>
bounded_text(std::string_view text) noexcept {
    if (text.size() <= kMaxTextBytes) return {text, false};
    std::size_t end = kMaxTextBytes;
    while (end > 0 &&
           (static_cast<unsigned char>(text[end]) & 0xC0U) == 0x80U) {
        --end;
    }
    // remove_suffix is noexcept — this helper can never throw inside the
    // noexcept write path.
    std::string_view head = text;
    head.remove_suffix(text.size() - end);
    return {head, true};
}

// Escape every byte that could break the one-line shape or the quoted-field
// shape; bytes >= 0x80 pass through so valid UTF-8 is preserved verbatim.
void write_escaped(std::ostream& output, std::string_view text) {
    for (const char c : text) {
        switch (const auto byte = static_cast<unsigned char>(c); byte) {
            case '\n': output << "\\n";  break;
            case '\r': output << "\\r";  break;
            case '\t': output << "\\t";  break;
            case '"':  output << "\\\""; break;
            default:
                if (byte < 0x20U || byte == 0x7FU) {
                    output << "\\x" << hex_digit(byte >> 4U)
                           << hex_digit(byte & 0x0FU);
                } else {
                    output << c;
                }
        }
    }
}

void write_bounded_escaped(std::ostream& output, std::string_view text) {
    const auto [view, cut] = bounded_text(text);
    write_escaped(output, view);
    if (cut) output << kTruncatedMark;
}

[[nodiscard]] bool value_matches(DiagnosticKey key,
                                 const DiagnosticValue& value) noexcept {
    switch (key) {
        case DiagnosticKey::command:
        case DiagnosticKey::operation:
        case DiagnosticKey::error_code:
        case DiagnosticKey::field_path:
        case DiagnosticKey::job_id:
            return std::holds_alternative<std::string_view>(value);
        case DiagnosticKey::episode_id:
        case DiagnosticKey::generation:
        case DiagnosticKey::job_index:
            return std::holds_alternative<std::int64_t>(value) ||
                   std::holds_alternative<std::uint64_t>(value);
        case DiagnosticKey::duration_ms:
            return std::holds_alternative<double>(value);
    }
    return false;
}

// std::to_chars is locale-independent for every supported scalar; doubles use
// the shared binary64 repr spelling so durations match recording output.
void write_value(std::ostream& output, const DiagnosticValue& value) {
    if (const auto* text = std::get_if<std::string_view>(&value)) {
        output << '"';
        write_bounded_escaped(output, *text);
        output << '"';
        return;
    }
    if (const auto* flag = std::get_if<bool>(&value)) {
        output << (*flag ? "true" : "false");
        return;
    }
    if (const auto* fractional = std::get_if<double>(&value)) {
        output << repr_binary64(*fractional);
        return;
    }
    std::array<char, 24> digits{};
    std::to_chars_result result{};
    if (const auto* number = std::get_if<std::int64_t>(&value)) {
        result = std::to_chars(digits.data(), std::to_address(digits.end()),
            *number);
    } else {
        result = std::to_chars(digits.data(), std::to_address(digits.end()),
            std::get<std::uint64_t>(value));
    }
    output << std::string_view(digits.data(), result.ptr);
}

} // namespace

bool is_enabled(Severity severity, Severity minimum) noexcept {
    return static_cast<int>(severity) >= static_cast<int>(minimum);
}

// event and message are distinct named slots in the mandated interface; the
// signature anchors a multi-line warning, so the suppression spans the block.
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
bool write_diagnostic(std::ostream& output, Severity severity,
                      std::string_view event, std::string_view message,
                      std::span<const DiagnosticField> fields) noexcept {
// NOLINTEND(bugprone-easily-swappable-parameters)
    try {
        if (!output || !canonical_event(event)) return false;
        for (const DiagnosticField& field : fields) {
            if (!value_matches(field.key, field.value)) return false;
        }
        output << severity_tag(severity) << ' ' << event;
        if (!message.empty()) {
            output << ": ";
            write_bounded_escaped(output, message);
        }
        for (const DiagnosticField& field : fields) {
            output << ' ' << key_name(field.key) << '=';
            write_value(output, field.value);
        }
        output << '\n' << std::flush;
        return output.good();
    } catch (...) {
        return false;
    }
}

} // namespace bike::io
