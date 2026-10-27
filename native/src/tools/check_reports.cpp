// check_reports.cpp — error types, JSON emission, clock, process records,
// diagnostics and suppression matching for the native check orchestrator
// (task T56). Ports tools/native_checks.py's reporting substrate:
// CheckError, _read_json, json.dumps(indent=2, sort_keys=True) summaries,
// _new_run_id/_run_process/_version_record, _DIAGNOSTIC/_LOCATED_DIAGNOSTIC
// classification and load_suppressions/apply_suppressions.
//
// Subprocess execution is exclusively bike::platform::run — argv arrays
// with no shell, concurrent bounded stdout/stderr capture, deadline
// termination of the whole process group. When a record asks for a
// replaced environment the argv is prefixed with `env -i K=V ...` (the
// POSIX argv-only equivalent of subprocess.run(env=dict(env))); core-dump
// suppression re-execs this binary under --check-child so RLIMIT_CORE=0 is
// applied between fork and exec, matching the oracle's preexec_fn.
#include "checks_internal.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <pwd.h>
#include <set>
#include <sstream>
#include <system_error>
#include <unistd.h>

#include "../platform/files.hpp"
#include "../platform/process.hpp"
#include "../platform/provenance.hpp"

#if defined(__APPLE__)
#include <crt_externs.h>
#else
// POSIX process environment. Declared at global scope deliberately: a
// block-scope `extern char **environ` inside bike::checks would declare
// bike::checks::environ — a namespaced entity that does not exist — and
// strict C++23 (__STRICT_ANSI__) hides unistd.h's own declaration.
extern char **environ; // NOLINT(readability-redundant-declaration)
#endif

namespace bike::checks {

CheckError::~CheckError() = default;
ContractError::~ContractError() = default;

Value vnull() { return Value{std::monostate{}}; }
Value vstr(std::string text) { return Value{std::move(text)}; }
Value vint(std::int64_t value) {
    return Value{io::Number{std::to_string(value)}};
}
Value vbool(bool value) { return Value{value}; }
Value varr(Array items) { return Value{std::move(items)}; }
Value vobj(Object members) { return Value{std::move(members)}; }

void set_member(Value &object, std::string key, Value value) {
    auto *members = std::get_if<Object>(&object.data);
    if (members == nullptr) return;
    for (auto &entry: *members)
        if (entry.first == key) {
            entry.second = std::move(value);
            return;
        }
    members->emplace_back(std::move(key), std::move(value));
}

const Value *member(const Object &object, std::string_view key) noexcept {
    return io::find_member(object, key);
}

const Value *member(const Value &object, std::string_view key) noexcept {
    const auto *members = std::get_if<Object>(&object.data);
    if (members == nullptr) return nullptr;
    return member(*members, key);
}

Value *member_mut(Value &object, std::string_view key) noexcept {
    auto *members = std::get_if<Object>(&object.data);
    if (members == nullptr) return nullptr;
    for (auto &entry: *members)
        if (entry.first == key) return &entry.second;
    return nullptr;
}

std::optional<std::string> opt_string(const Value &object,
                                      std::string_view key) {
    const Value *value = member(object, key);
    if (value == nullptr || io::is_null(*value)) return std::nullopt;
    const auto *text = std::get_if<std::string>(&value->data);
    if (text == nullptr)
        throw CheckError("context member " + std::string(key) +
                         " is not a string");
    return *text;
}

std::optional<bool> opt_bool(const Value &object, std::string_view key) {
    const Value *value = member(object, key);
    if (value == nullptr || io::is_null(*value)) return std::nullopt;
    const auto *flag = std::get_if<bool>(&value->data);
    if (flag == nullptr)
        throw CheckError("context member " + std::string(key) +
                         " is not a boolean");
    return *flag;
}

std::optional<std::int64_t> opt_int(const Value &object, std::string_view key) {
    const Value *value = member(object, key);
    if (value == nullptr || io::is_null(*value)) return std::nullopt;
    const auto *number = std::get_if<io::Number>(&value->data);
    if (number == nullptr)
        throw CheckError("context member " + std::string(key) +
                         " is not a number");
    return io::number_i64(*number, key);
}

// ---- JSON reading ----------------------------------------------------------

Value read_json(const fs::path &path, const std::string &label) {
    std::error_code ec;
    if (!fs::exists(path, ec)) throw CheckError("missing " + label + ": " + path.string());
    std::ifstream const stream(path, std::ios::binary);
    if (!stream) {
        const int error = errno;
        // strerror is process-wide but this tool is single-threaded at read time.
        throw CheckError("malformed " + label + ": " + path.string() + ": " +
                         std::string(std::strerror(error))); // NOLINT(concurrency-mt-unsafe)
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    if (stream.bad())
        throw CheckError("malformed " + label + ": " + path.string() +
                         ": read failed");
    try {
        return io::parse_json(buffer.str());
    } catch (const std::invalid_argument &error) {
        throw CheckError("malformed " + label + ": " + path.string() + ": " +
                         error.what());
    }
}

// ---- JSON writing (json.dumps(indent=2, sort_keys=True, ensure_ascii=True)) -

namespace {

void append_ascii_escaped(std::string &out, const std::string &text) {
    out += '"';
    std::size_t index = 0;
    while (index < text.size()) {
        const auto byte = static_cast<unsigned char>(text[index]);
        // Decode one UTF-8 scalar; malformed input is impossible here (all
        // producer strings passed through utf8_replace / parse validation)
        // but a defensive byte-level escape keeps the writer total.
        std::uint32_t scalar = byte;
        std::size_t width = 1;
        if (byte >= 0xC2 && byte < 0xE0 && index + 1 < text.size()) {
            scalar = ((byte & 0x1FU) << 6U) |
                     (static_cast<unsigned char>(text[index + 1]) & 0x3FU);
            width = 2;
        } else if (byte >= 0xE0 && byte < 0xF0 && index + 2 < text.size()) {
            scalar = ((byte & 0x0FU) << 12U) |
                     ((static_cast<unsigned char>(text[index + 1]) & 0x3FU) << 6U) |
                     (static_cast<unsigned char>(text[index + 2]) & 0x3FU);
            width = 3;
        } else if (byte >= 0xF0 && byte < 0xF5 && index + 3 < text.size()) {
            scalar = ((byte & 0x07U) << 18U) |
                     ((static_cast<unsigned char>(text[index + 1]) & 0x3FU) << 12U) |
                     ((static_cast<unsigned char>(text[index + 2]) & 0x3FU) << 6U) |
                     (static_cast<unsigned char>(text[index + 3]) & 0x3FU);
            width = 4;
        }
        index += width;
        const auto hex = [](std::uint32_t value, std::string &sink) {
            constexpr std::string_view digits = "0123456789abcdef";
            sink += "\\u";
            for (const std::uint32_t shift: {12U, 8U, 4U, 0U})
                sink += digits[(value >> shift) & 0xFU];
        };
        switch (scalar) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (scalar < 0x20 || (scalar >= 0x7F && scalar <= 0xFFFF)) {
                    hex(scalar, out);
                } else if (scalar < 0x7F) {
                    out += static_cast<char>(scalar);
                } else {
                    // ensure_ascii: astral scalars serialize as surrogate pairs.
                    const std::uint32_t shifted = scalar - 0x10000U;
                    hex(0xD800U + (shifted >> 10U), out);
                    hex(0xDC00U + (shifted & 0x3FFU), out);
                }
        }
    }
    out += '"';
}

// NOLINTNEXTLINE(misc-no-recursion) JSON is a tree; emission mirrors the value structure
void dump_json_into(std::string &out, const Value &value, unsigned level) {
    const std::string indent(static_cast<std::size_t>(level) * 2U, ' ');
    const std::string child_indent((static_cast<std::size_t>(level) + 1U) * 2U,
                                   ' ');
    if (io::is_null(value)) {
        out += "null";
    } else if (const auto *flag = std::get_if<bool>(&value.data)) {
        out += *flag ? "true" : "false";
    } else if (const auto *number = std::get_if<io::Number>(&value.data)) {
        out += number->lexeme;
    } else if (const auto *text = std::get_if<std::string>(&value.data)) {
        append_ascii_escaped(out, *text);
    } else if (const auto *array = std::get_if<Array>(&value.data)) {
        if (array->empty()) {
            out += "[]";
        } else {
            out += "[\n";
            for (std::size_t index = 0; index < array->size(); ++index) {
                out += child_indent;
                dump_json_into(out, (*array)[index], level + 1U);
                out += (index + 1 == array->size()) ? "\n" : ",\n";
            }
            out += indent + "]";
        }
    } else if (const auto *object = std::get_if<Object>(&value.data)) {
        if (object->empty()) {
            out += "{}";
        } else {
            // sort_keys=True: UTF-8 byte order equals code-point order.
            Object sorted(*object);
            std::ranges::sort(sorted,
                      [](const auto &left, const auto &right) {
                          return left.first < right.first;
                      });
            out += "{\n";
            for (std::size_t index = 0; index < sorted.size(); ++index) {
                out += child_indent;
                append_ascii_escaped(out, sorted[index].first);
                out += ": ";
                dump_json_into(out, sorted[index].second, level + 1U);
                out += (index + 1 == sorted.size()) ? "\n" : ",\n";
            }
            out += indent + "}";
        }
    } else {
        throw CheckError("summary contains a non-JSON value kind");
    }
}

} // namespace

std::string dump_json(const Value &value) {
    std::string out;
    dump_json_into(out, value, 0);
    return out;
}

std::string repr_str_map(
    const std::vector<std::pair<std::string, std::string>> &ordered) {
    // Python dict repr of a flat {str: str}: {'key': 'value', ...}
    const auto repr = [](const std::string &text) {
        std::string out = "'";
        for (const char ch: text) {
            if (ch == '\\' || ch == '\'') {
                out += '\\';
                out += ch;
            } else if (ch == '\n') {
                out += "\\n";
            } else {
                out += ch;
            }
        }
        out += '\'';
        return out;
    };
    std::string out = "{";
    bool first = true;
    for (const auto &[key, value]: ordered) {
        if (!first) out += ", ";
        first = false;
        out += repr(key);
        out += ": ";
        out += repr(value);
    }
    out += '}';
    return out;
}

void write_summary(const fs::path &build, std::string_view kind,
                   const Value &summary) {
    const fs::path directory = build / "native_check_summaries";
    std::error_code ec;
    fs::create_directories(directory, ec);
    const fs::path destination = directory / (std::string(kind) + ".json");
    const std::string payload = dump_json(summary) + "\n";
    std::ofstream stream(destination, std::ios::binary | std::ios::trunc);
    if (!stream)
        throw CheckError("cannot write native check summary: " +
                         destination.string());
    stream << payload;
    stream.flush();
    if (!stream)
        throw CheckError("failed writing native check summary: " +
                         destination.string());
}

void write_summary_atomic(const fs::path &build, const Value &summary) {
    const fs::path directory = build / "native_check_summaries";
    std::error_code ec;
    fs::create_directories(directory, ec);
    const fs::path destination = directory / "contracts.json";
    const fs::path temporary = destination.parent_path() /
                               (destination.filename().string() + ".tmp");
    {
        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
        if (!stream)
            throw CheckError("cannot write native check summary: " +
                             temporary.string());
        stream << dump_json(summary) << "\n";
    }
    ec.clear();
    fs::rename(temporary, destination, ec);
    if (ec)
        throw CheckError("cannot publish native check summary: " +
                         destination.string() + ": " + ec.message());
}

// ---- clock -------------------------------------------------------------------

namespace {

struct UtcParts {
    std::tm tm;
    long micros;
};

UtcParts utc_now_parts() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto micros =
        std::chrono::duration_cast<std::chrono::microseconds>(now - seconds)
            .count();
    const std::time_t epoch = std::chrono::system_clock::to_time_t(seconds);
    std::tm tm {};
    gmtime_r(&epoch, &tm);
    return {.tm=tm, .micros=static_cast<long>(micros)};
}

} // namespace

std::string new_run_id() {
    const auto [tm, micros] = utc_now_parts();
    std::ostringstream out;
    out << std::setfill('0') << std::setw(4) << tm.tm_year + 1900
        << std::setw(2) << tm.tm_mon + 1 << std::setw(2) << tm.tm_mday << 'T'
        << std::setw(2) << tm.tm_hour << std::setw(2) << tm.tm_min
        << std::setw(2) << tm.tm_sec << std::setw(6) << micros << 'Z';
    return out.str();
}

std::string utc_iso_now() {
    const auto [tm, micros] = utc_now_parts();
    std::ostringstream out;
    out << std::setfill('0') << std::setw(4) << tm.tm_year + 1900 << '-'
        << std::setw(2) << tm.tm_mon + 1 << '-' << std::setw(2) << tm.tm_mday
        << 'T' << std::setw(2) << tm.tm_hour << ':' << std::setw(2)
        << tm.tm_min << ':' << std::setw(2) << tm.tm_sec << '.'
        << std::setw(6) << micros << "+00:00";
    return out.str();
}

// ---- filesystem --------------------------------------------------------------

fs::path absolute_lexical(const fs::path &path) {
    if (path.is_absolute()) return path;
    return fs::current_path() / path;
}

fs::path expand_home(const std::string &value) {
    if (value.empty() || value.front() != '~') return {value};
    const std::size_t slash = value.find('/');
    const std::string user = value.substr(1, slash == std::string::npos
                                                 ? std::string::npos
                                                 : slash - 1);
    std::string home;
    if (user.empty()) {
        // getenv is process-wide; this tool resolves the home directory before
        // any worker threads exist.
        const char *env = std::getenv("HOME"); // NOLINT(concurrency-mt-unsafe)
        if (env == nullptr)
            throw CheckError("could not determine home directory for: " + value);
        home = env;
    } else {
        // getpwnam is process-wide; same single-threaded-read rationale.
        const passwd *entry =
            ::getpwnam(user.c_str()); // NOLINT(concurrency-mt-unsafe)
        if (entry == nullptr)
            throw CheckError("no home directory for user in path: " + value);
        home = entry->pw_dir;
    }
    const std::string rest = slash == std::string::npos
                                 ? std::string()
                                 : value.substr(slash);
    return {home + rest};
}

fs::path canonical_path(const fs::path &path, const fs::path *base) {
    fs::path candidate = path;
    if (candidate.is_relative() && base != nullptr) candidate = *base / candidate;
    if (candidate.is_relative()) candidate = fs::current_path() / candidate;
    std::error_code ec;
    fs::path const resolved = fs::weakly_canonical(candidate, ec);
    return ec ? candidate.lexically_normal() : resolved;
}

bool is_under(const fs::path &path, const std::vector<fs::path> &roots) {
    for (const fs::path &root: roots) {
        auto path_it = path.begin();
        auto root_it = root.begin();
        while (root_it != root.end() && path_it != path.end() &&
               *path_it == *root_it) {
            ++path_it;
            ++root_it;
        }
        if (root_it == root.end()) return true;
    }
    return false;
}

bool is_executable_file(const fs::path &path) {
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) return false;
    return ::access(path.c_str(), X_OK) == 0;
}

std::optional<fs::path> which(const std::string &name) {
    if (name.empty()) return std::nullopt;
    if (name.find('/') != std::string::npos) {
        const fs::path candidate(name);
        return fs::is_regular_file(candidate) && is_executable_file(candidate)
                   ? std::optional<fs::path>(candidate)
                   : std::nullopt;
    }
    // NOLINTNEXTLINE(concurrency-mt-unsafe) tool lookup happens before any threads spawn
    const char *path_env = std::getenv("PATH");
    const std::string path_text =
        path_env != nullptr ? path_env : "/bin:/usr/bin";
    std::size_t start = 0;
    while (true) {
        const std::size_t colon = path_text.find(':', start);
        std::string directory =
            path_text.substr(start, colon == std::string::npos
                                        ? std::string::npos
                                        : colon - start);
        if (directory.empty()) directory = ".";
        fs::path candidate = fs::path(directory) / name;
        if (fs::is_regular_file(candidate) && is_executable_file(candidate))
            return candidate;
        if (colon == std::string::npos) break;
        start = colon + 1;
    }
    return std::nullopt;
}

std::map<std::string, std::string> environ_copy() {
    std::map<std::string, std::string> result;
#if defined(__APPLE__)
    char *const *environ = *_NSGetEnviron();
#endif
    // POSIX environ is a null-terminated C array — the terminator bounds the
    // walk; there is no sized-container spelling for it. The pragma is clang
    // dialect: GCC's frontends sweep would reject it as an unknown pragma.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage"
#endif
    for (char *const *cursor = environ;
         cursor != nullptr && *cursor != nullptr;
         ++cursor) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic) environ is only addressable by pointer walk
        const std::string entry(*cursor);
        const std::size_t equals = entry.find('=');
        if (equals == std::string::npos) continue;
        result[entry.substr(0, equals)] = entry.substr(equals + 1);
    }
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    return result;
}

std::string getenv_string(const char *name) {
    // NOLINTNEXTLINE(concurrency-mt-unsafe) tool reads environment before spawning workers
    const char *value = std::getenv(name);
    return value != nullptr ? std::string(value) : std::string();
}

// ---- text / diagnostics --------------------------------------------------------

namespace {

const std::regex &diagnostic_pattern() {
    static const std::regex pattern(
        "\\b(?:warning|error):|fatal error:", std::regex_constants::icase);
    return pattern;
}

const std::regex &located_pattern() {
    static const std::regex pattern(
        "^(.*?):(\\d+):(\\d+):\\s+(warning|error):\\s*(.*?)"
        "(?:\\s+\\[([^\\]]+)\\])?\\s*$",
        std::regex_constants::icase);
    return pattern;
}

// _SNIPPET_CONTEXT: clang echoes the offending source line under each
// diagnostic with a "   <line> | <code>" gutter. Quoted code can contain
// text like "error:" (a loop variable named error, a label, a string
// literal), so collected diagnostic lines are not all severity headers.
const std::regex &snippet_context_pattern() {
    static const std::regex pattern("^\\s*\\d*\\s*\\|");
    return pattern;
}

// _DIAGNOSTIC_HEADER: a severity token only counts where a diagnostic header
// can put it: after a "file:line[:col]:" location, after a tool-name prefix
// ("clang++: error:"), or at the start of the line ("error: unable to open
// output file").
const std::regex &diagnostic_header_pattern() {
    static const std::regex pattern(
        "^(?:.*?:\\d+(?::\\d+)?:|[^\\s:][^:\\n]*:)?\\s*"
        "(warning|error|fatal error):",
        std::regex_constants::icase);
    return pattern;
}

// _diagnostic_severity: "warning"/"error" for a diagnostic header line, else
// nullopt; snippet/context lines can never fake a severity-bearing finding.
std::optional<std::string> diagnostic_severity(const std::string &line) {
    if (std::regex_search(line.begin(), line.end(), snippet_context_pattern()))
        return std::nullopt;
    std::match_results<std::string::const_iterator> match;
    if (!std::regex_search(line.begin(), line.end(), match,
                           diagnostic_header_pattern()))
        return std::nullopt;
    std::string severity = match[1].str();
    std::ranges::transform(severity, severity.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (severity == "fatal error") severity = "error";
    return severity;
}

} // namespace

bool is_diagnostic_line(std::string_view line) {
    return std::regex_search(line.begin(), line.end(), diagnostic_pattern());
}

std::optional<LocatedDiagnostic> parse_located_diagnostic(
    const std::string &line) {
    std::match_results<std::string::const_iterator> match;
    if (!std::regex_match(line.begin(), line.end(), match, located_pattern()))
        return std::nullopt;
    LocatedDiagnostic parsed;
    parsed.origin = match[1].str();
    parsed.severity = match[4].str();
    parsed.message = match[5].str();
    if (match[6].matched) parsed.checker = match[6].str();
    return parsed;
}

std::vector<std::string> split_lines(const std::string &text) {
    // str.splitlines() boundaries: \n \r \r\n \v \f \x1c-\x1e plus the UTF-8
    // forms of NEL (C2 85), LS (E2 80 A8) and PS (E2 80 A9).
    std::vector<std::string> lines;
    std::string current;
    const auto flush = [&]() {
        lines.push_back(std::move(current));
        current.clear();
    };
    std::size_t index = 0;
    while (index < text.size()) {
        const unsigned char byte = static_cast<unsigned char>(text[index]);
        std::size_t width = 0;
        if (byte == '\n' || byte == '\v' || byte == '\f' ||
            (byte >= 0x1C && byte <= 0x1E)) {
            width = 1;
        } else if (byte == '\r') {
            width = (index + 1 < text.size() && text[index + 1] == '\n') ? 2 : 1;
        } else if (byte == 0xC2 && index + 1 < text.size() &&
                   static_cast<unsigned char>(text[index + 1]) == 0x85) {
            width = 2;
        } else if (byte == 0xE2 && index + 2 < text.size() &&
                   static_cast<unsigned char>(text[index + 1]) == 0x80 &&
                   (static_cast<unsigned char>(text[index + 2]) == 0xA8 ||
                    static_cast<unsigned char>(text[index + 2]) == 0xA9)) {
            width = 3;
        }
        if (width == 0) {
            current += text[index];
            ++index;
        } else {
            flush();
            index += width;
        }
    }
    if (!current.empty()) lines.push_back(std::move(current));
    return lines;
}

bool has_warning(const std::vector<std::string> &diagnostics) {
    return std::ranges::any_of(diagnostics, [](const auto &line) {
        return diagnostic_severity(line) == "warning";
    });
}

bool has_error(const std::vector<std::string> &diagnostics) {
    return std::ranges::any_of(diagnostics, [](const auto &line) {
        return diagnostic_severity(line) == "error";
    });
}

// ---- suppressions --------------------------------------------------------------

std::vector<Suppression> load_suppressions(const fs::path &path) {
    const Value manifest = read_json(path, "analysis suppression manifest");
    const auto *schema = member(manifest, "schema_version");
    const bool supported =
        schema != nullptr && std::holds_alternative<io::Number>(schema->data) &&
        std::get<io::Number>(schema->data).lexeme == "1";
    if (!std::holds_alternative<Object>(manifest.data) || !supported)
        throw CheckError("unsupported analysis suppression manifest schema: " +
                         path.string());
    const Value *entries = member(manifest, "suppressions");
    if (entries == nullptr || !std::holds_alternative<Array>(entries->data))
        throw CheckError(
            "analysis suppression manifest has no suppression list: " +
            path.string());
    static const std::set<std::string> kFields = {
        "checker", "origin", "message", "tool_version",
        "reason", "reproducer", "remove_when",
    };
    std::vector<Suppression> normalized;
    std::size_t index = 0;
    for (const Value &entry: std::get<Array>(entries->data)) {
        const auto *object = std::get_if<Object>(&entry.data);
        const bool complete =
            object != nullptr &&
            std::ranges::all_of(kFields, [&](const auto &field) {
                const Value *value = member(*object, field);
                return value != nullptr &&
                       std::holds_alternative<std::string>(value->data);
            });
        if (!complete)
            throw CheckError("incomplete analysis suppression entry " +
                             std::to_string(index) + ": " + path.string());
        const auto text_of = [&](const std::string &field) {
            // `complete` already proved every field a string, but the lookup
            // itself must stay fail-closed — GCC 13 -Wnull-dereference also
            // cannot prove member() non-null through this lambda.
            const Value *value = member(*object, field);
            if (value == nullptr ||
                !std::holds_alternative<std::string>(value->data))
                throw CheckError("incomplete analysis suppression entry " +
                                 std::to_string(index) + ": " +
                                 path.string());
            return std::get<std::string>(value->data);
        };
        const bool nonempty =
            std::ranges::all_of(kFields,
                        [&](const auto &field) { return !text_of(field).empty(); });
        if (!nonempty)
            throw CheckError("analysis suppression entry " +
                             std::to_string(index) +
                             " has empty or nonstring fields: " + path.string());
        Suppression suppression{
            .checker = text_of("checker"),
            .origin = text_of("origin"),
            .message = text_of("message"),
            .tool_version = text_of("tool_version"),
            .reason = text_of("reason"),
            .reproducer = text_of("reproducer"),
            .remove_when = text_of("remove_when"),
            .origin_regex = {},
        };
        try {
            suppression.origin_regex = std::regex(suppression.origin);
        } catch (const std::regex_error &error) {
            throw CheckError("invalid origin pattern in analysis suppression "
                             "entry " +
                             std::to_string(index) + ": " + error.what());
        }
        normalized.push_back(std::move(suppression));
        ++index;
    }
    return normalized;
}

SuppressionFilter apply_suppressions(
    const std::vector<std::string> &diagnostics, const std::string &tool_version,
    const std::vector<Suppression> &suppressions) {
    SuppressionFilter result;
    for (const std::string &diagnostic: diagnostics) {
        std::string stripped = diagnostic;
        // The oracle applies .strip() before matching.
        const auto first = stripped.find_first_not_of(" \t\n\v\f\r");
        const auto last = stripped.find_last_not_of(" \t\n\v\f\r");
        stripped = first == std::string::npos
                       ? std::string()
                       : stripped.substr(first, last - first + 1);
        const auto parsed = parse_located_diagnostic(stripped);
        const Suppression *match = nullptr;
        if (parsed.has_value()) {
            for (const Suppression &entry: suppressions) {
                if (entry.checker == parsed->checker &&
                    tool_version.find(entry.tool_version) != std::string::npos &&
                    parsed->message.find(entry.message) != std::string::npos &&
                    std::regex_search(parsed->origin, entry.origin_regex)) {
                    match = &entry;
                    break;
                }
            }
        }
        if (match == nullptr) {
            result.remaining.push_back(diagnostic);
        } else if (parsed.has_value()) {
            const auto &located = *parsed;
            result.matched.push_back(vobj({
                {"diagnostic", vstr(diagnostic)},
                {"checker", vstr(located.checker)},
                {"origin", vstr(located.origin)},
                {"reason", vstr(match->reason)},
            }));
        }
    }
    return result;
}

// ---- process records -----------------------------------------------------------

namespace {

void write_log(const fs::path &path, const std::string &content) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (stream) stream << content;
}

// The sign-encoded exit code -> Python subprocess returncode convention:
// wait status 128+N means killed by signal N, reported as -N by CPython.
std::optional<std::int64_t> python_returncode(int exit_code) {
    if (exit_code > 128) return static_cast<std::int64_t>(128 - exit_code);
    return static_cast<std::int64_t>(exit_code);
}

std::string utf8_boundary_replace(std::string_view bytes) {
    std::string out;
    std::size_t index = 0;
    while (index < bytes.size()) {
        const auto first = static_cast<unsigned char>(bytes[index]);
        if (first < 0x80) {
            out += static_cast<char>(first);
            ++index;
            continue;
        }
        std::size_t width = 0;
        if (first >= 0xC2 && first <= 0xDF)
            width = 2;
        else if (first >= 0xE0 && first <= 0xEF)
            width = 3;
        else if (first >= 0xF0 && first <= 0xF4)
            width = 4;
        bool valid = width != 0 && index + width <= bytes.size();
        for (std::size_t offset = 1; valid && offset < width; ++offset) {
            const auto continuation =
                static_cast<unsigned char>(bytes[index + offset]);
            valid = (continuation & 0xC0U) == 0x80U;
        }
        if (valid) {
            // Reject overlongs and out-of-range scalars.
            if (width == 3) {
                const auto second = static_cast<unsigned char>(bytes[index + 1]);
                if (first == 0xE0 && second < 0xA0) valid = false;
                if (first == 0xED && second > 0x9F) valid = false;
            } else if (width == 4) {
                const auto second = static_cast<unsigned char>(bytes[index + 1]);
                if (first == 0xF0 && second < 0x90) valid = false;
                if (first == 0xF4 && second > 0x8F) valid = false;
            }
        }
        if (!valid) {
            out += "\xEF\xBF\xBD"; // U+FFFD
            ++index;
            continue;
        }
        out.append(bytes.substr(index, width));
        index += width;
    }
    return out;
}

} // namespace

std::string utf8_replace(const std::string &bytes) {
    return utf8_boundary_replace(bytes);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (cwd, log_directory) order mirrors the oracle signature
Value run_process(const std::vector<std::string> &argv, const fs::path &cwd,
                  const fs::path &log_directory, const std::string &name,
                  const std::map<std::string, std::string> *env,
                  bool disable_core_dumps, std::int64_t timeout_seconds) {
    std::error_code ec;
    fs::create_directories(log_directory, ec);
    const fs::path stdout_path = log_directory / (name + ".stdout.log");
    const fs::path stderr_path = log_directory / (name + ".stderr.log");
    const std::string started = utc_iso_now();

    std::vector<std::string> effective;
    if (env != nullptr) {
        const auto env_binary =
            which("env").value_or(fs::path("/usr/bin/env")).string();
        effective = {env_binary, "-i"};
        for (const auto &[key, value]: *env) {
            std::string assignment = key;
            assignment += '=';
            assignment += value;
            effective.push_back(std::move(assignment));
        }
        if (disable_core_dumps) {
            effective.push_back(platform::executable_path().string());
            effective.emplace_back("--check-child");
        }
        effective.insert(effective.end(), argv.begin(), argv.end());
    } else if (disable_core_dumps) {
        effective = {platform::executable_path().string(), "--check-child"};
        effective.insert(effective.end(), argv.begin(), argv.end());
    } else {
        effective = argv;
    }

    platform::ProcessOptions options;
    options.timeout_s = static_cast<double>(timeout_seconds);
    std::string out;
    std::string err;
    std::optional<std::int64_t> returncode;
    std::optional<std::string> launch_error;
    bool timed_out = false;
    const auto before = std::chrono::steady_clock::now();
    try {
        const platform::ProcessResult result =
            platform::run(effective, cwd, options);
        out = utf8_replace(result.out);
        err = utf8_replace(result.err);
        returncode = python_returncode(result.exit_code);
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - before)
                .count();
        // The deadline kill lands as SIGKILL (encoded 137) once the full
        // timeout elapsed — the observable signature of a timeout under the
        // sign-encoded exit contract.
        if (result.exit_code == 137 &&
            elapsed >= static_cast<double>(timeout_seconds) - 0.5)
            timed_out = true;
    } catch (const std::invalid_argument &error) {
        launch_error = error.what();
    } catch (const std::system_error &error) {
        launch_error = error.what();
    } catch (const std::runtime_error &error) {
        launch_error = error.what();
    }
    write_log(stdout_path, out);
    write_log(stderr_path, err);

    Array argv_values;
    argv_values.reserve(argv.size());
    for (const auto &argument: argv) argv_values.push_back(vstr(argument));

    const std::string combined = out + "\n" + err;
    Array diagnostics;
    for (const auto &line: split_lines(combined))
        if (is_diagnostic_line(line)) diagnostics.push_back(vstr(line));

    const bool failed =
        launch_error.has_value() || timed_out || !returncode.has_value() ||
        *returncode != 0;
    return vobj({
        {"argv", varr(std::move(argv_values))},
        {"cwd", vstr(cwd.string())},
        {"started_at", vstr(started)},
        {"returncode", returncode.has_value() ? vint(*returncode) : vnull()},
        {"launch_error",
         launch_error.has_value() ? vstr(*launch_error) : vnull()},
        {"timed_out", vbool(timed_out)},
        {"process_health", vstr(failed ? "failed" : "ok")},
        {"diagnostics", varr(std::move(diagnostics))},
        {"stdout_log", vstr(stdout_path.string())},
        {"stderr_log", vstr(stderr_path.string())},
        {"stdout", vstr(std::move(out))},
        {"stderr", vstr(std::move(err))},
    });
}

Value version_record(const fs::path &tool, const fs::path &log_directory) {
    Value record = run_process({tool.string(), "--version"}, fs::current_path(),
                               log_directory, "tool-version",
                               nullptr, false, 60);
    std::string version;
    if (const Value *out_text = member(record, "stdout"))
        version += std::get<std::string>(out_text->data);
    if (const Value *err_text = member(record, "stderr"))
        version += std::get<std::string>(err_text->data);
    const auto first = version.find_first_not_of(" \t\n\v\f\r");
    const auto last = version.find_last_not_of(" \t\n\v\f\r");
    version = first == std::string::npos ? std::string()
                                         : version.substr(first, last - first + 1);
    set_member(record, "version", vstr(std::move(version)));
    return record;
}

std::optional<std::int64_t> record_returncode(const Value &record) {
    const Value *value = member(record, "returncode");
    if (value == nullptr || io::is_null(*value)) return std::nullopt;
    const auto *number = std::get_if<io::Number>(&value->data);
    if (number == nullptr) return std::nullopt;
    return parse_int64(number->lexeme);
}

std::optional<std::int64_t> parse_int64(const std::string &text) {
    try {
        std::size_t consumed = 0;
        const std::int64_t value = std::stoll(text, &consumed);
        if (consumed != text.size()) return std::nullopt;
        return value;
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

bool record_health_ok(const Value &record) {
    const Value *value = member(record, "process_health");
    return value != nullptr && std::holds_alternative<std::string>(value->data) &&
           std::get<std::string>(value->data) == "ok";
}

std::vector<std::string> record_diagnostics(const Value &record) {
    std::vector<std::string> result;
    const Value *value = member(record, "diagnostics");
    if (value == nullptr) return result;
    if (const auto *array = std::get_if<Array>(&value->data))
        for (const auto &item: *array)
            if (const auto *text = std::get_if<std::string>(&item.data))
                result.push_back(*text);
    return result;
}

std::string record_stdout_text(const Value &record) {
    const Value *value = member(record, "stdout");
    return value != nullptr && std::holds_alternative<std::string>(value->data)
               ? std::get<std::string>(value->data)
               : std::string{};
}

std::string record_stderr_text(const Value &record) {
    const Value *value = member(record, "stderr");
    return value != nullptr && std::holds_alternative<std::string>(value->data)
               ? std::get<std::string>(value->data)
               : std::string{};
}

// NOLINTNEXTLINE(misc-no-recursion) JSON is a tree; equality mirrors the value structure
bool json_equal(const Value &left, const Value &right) {
    if (left.data.index() != right.data.index()) return false;
    if (std::holds_alternative<std::monostate>(left.data)) return true;
    if (const auto *flag = std::get_if<bool>(&left.data))
        return *flag == std::get<bool>(right.data);
    if (const auto *number = std::get_if<io::Number>(&left.data))
        return number->lexeme == std::get<io::Number>(right.data).lexeme;
    if (const auto *text = std::get_if<std::string>(&left.data))
        return *text == std::get<std::string>(right.data);
    if (const auto *array = std::get_if<Array>(&left.data)) {
        const auto &other = std::get<Array>(right.data);
        if (array->size() != other.size()) return false;
        for (std::size_t index = 0; index < array->size(); ++index)
            if (!json_equal((*array)[index], other[index])) return false;
        return true;
    }
    const auto &object = std::get<Object>(left.data);
    const auto &other = std::get<Object>(right.data);
    if (object.size() != other.size()) return false;
    for (const auto &[key, value]: object) {
        const Value *match = nullptr;
        for (const auto &[other_key, other_value]: other)
            if (other_key == key) match = &other_value; // last wins
        if (match == nullptr || !json_equal(value, *match)) return false;
    }
    return true;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (value, pattern) order mirrors Python fnmatchcase
bool fnmatchcase(const std::string &value, const std::string &pattern) {
    // fnmatch.fnmatchcase on POSIX: * -> .*, ? -> ., [seq] -> [seq]
    // (with [!seq] -> [^seq]), everything else literal; full-string match.
    std::string translated = "^";
    std::size_t index = 0;
    while (index < pattern.size()) {
        const char ch = pattern[index];
        switch (ch) {
            case '*': translated += ".*"; ++index; continue;
            case '?': translated += '.'; ++index; continue;
            case '[': {
                std::size_t close = index + 1;
                std::string body;
                if (close < pattern.size() &&
                    (pattern[close] == '!' || pattern[close] == '^')) {
                    body += '^';
                    ++close;
                }
                if (close < pattern.size() && pattern[close] == ']')
                    ++close;
                bool closed = false;
                for (; close < pattern.size(); ++close) {
                    if (pattern[close] == ']') {
                        closed = true;
                        break;
                    }
                    if (pattern[close] == '\\') body += "\\\\";
                    else body += pattern[close];
                }
                if (closed) {
                    translated += '[' + body + ']';
                    index = close + 1;
                    continue;
                }
                translated += "\\[";
                ++index;
                continue;
            }
            default:
                if (ch == '\\' && index + 1 < pattern.size()) {
                    // fnmatch's backslash is literal (not an escape), but a
                    // regex backslash must be doubled to stay literal.
                    translated += "\\\\";
                    ++index;
                    continue;
                }
                if (std::string_view(".^$+{}()|").find(ch) !=
                    std::string_view::npos)
                    translated += '\\';
                translated += ch;
                ++index;
                continue;
        }
    }
    translated += '$';
    try {
        const std::regex regex(translated);
        return std::regex_match(value, regex);
    } catch (const std::regex_error &) {
        return false;
    }
}

} // namespace bike::checks
