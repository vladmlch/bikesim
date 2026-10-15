// checks_internal.hpp — shared declarations for the native check
// orchestrator (task T56). This header is the internal boundary between
// the five compilation units that port tools/native_checks.py and
// tools/check_native_contracts.py:
//
//   check_reports.cpp   — errors, JSON emission, run_ids, process records,
//                         diagnostics and suppression matching
//   check_context.cpp   — context/manifest/compile-database loading and
//                         validation, tool resolution, argv rewriting
//   check_frontends.cpp — the five translation-unit sweeps plus the
//                         standalone header and shell-script sweeps
//   check_contracts.cpp — diagnostic controls, SDK/hardening context
//                         controls, and the structural contracts check
//   checks_main.cpp     — CLI parsing and dispatch (plus the exec wrapper
//                         used to disable core dumps in child processes)
//
// Source authorities (kept read-only until T63):
//   tools/native_checks.py, tools/check_native_contracts.py.
// Semantics preserved exactly: selection order, fail-closed validation
// order, stdout/stderr separation, exit codes, summary/log provenance.
#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "../io/json.hpp"
#include "../io/value.hpp"

namespace bike::checks {

namespace fs = std::filesystem;
using io::Value;
using Object = Value::Object;
using Array = Value::Array;

// ---- errors ---------------------------------------------------------------

// The ported CheckError(ValueError): every malformed-context, stale-manifest
// or selection violation is raised through this type and surfaces as the
// summary "errors" entry plus a stderr line.
class CheckError : public std::runtime_error {
  public:
    explicit CheckError(const std::string &message) : std::runtime_error(message) {}
    CheckError(const CheckError &) = default;
    CheckError(CheckError &&) = default;
    CheckError &operator=(const CheckError &) = default;
    CheckError &operator=(CheckError &&) = default;
    ~CheckError() override; // out-of-line key function — no weak vtable
};

// The ported ContractError(ValueError) from check_native_contracts.py.
class ContractError : public std::runtime_error {
  public:
    explicit ContractError(const std::string &message) : std::runtime_error(message) {}
    ContractError(const ContractError &) = default;
    ContractError(ContractError &&) = default;
    ContractError &operator=(const ContractError &) = default;
    ContractError &operator=(ContractError &&) = default;
    ~ContractError() override; // out-of-line key function — no weak vtable
};

// ---- value construction/access helpers ------------------------------------

[[nodiscard]] Value vnull();
[[nodiscard]] Value vstr(std::string text);
[[nodiscard]] Value vint(std::int64_t value);
[[nodiscard]] Value vbool(bool value);
[[nodiscard]] Value varr(Array items);
[[nodiscard]] Value vobj(Object members);

// Replace-or-append a member on an Object value (no-op on non-objects).
void set_member(Value &object, std::string key, Value value);
// Object member lookup that keeps absence distinct from explicit null.
[[nodiscard]] const Value *member(const Value &object, std::string_view key) noexcept;
[[nodiscard]] Value *member_mut(Value &object, std::string_view key) noexcept;
[[nodiscard]] const Value *member(const Object &object, std::string_view key) noexcept;
// Typed optional accessors — absent/null yield empty, other kinds throw
// CheckError to mirror the source's isinstance guards.
[[nodiscard]] std::optional<std::string> opt_string(const Value &object,
                                                   std::string_view key);
[[nodiscard]] std::optional<bool> opt_bool(const Value &object,
                                           std::string_view key);
[[nodiscard]] std::optional<std::int64_t> opt_int(const Value &object,
                                                 std::string_view key);

// ---- JSON ------------------------------------------------------------------

// Ports _read_json: missing -> "missing <label>: <path>", any read/parse
// failure -> "malformed <label>: <path>: <error>". The strict T05 parser is
// the port of json.loads for every context/manifest/database input.
[[nodiscard]] Value read_json(const fs::path &path, const std::string &label);

// Ports json.dumps(indent=2, sort_keys=True, ensure_ascii=True): object keys
// sorted by code point, two-space nesting, "key": value separators, ASCII
// escaping with surrogate pairs for astral characters, and [] / {} inline
// for empty containers. Used for summaries, temporary compile databases and
// generated fixtures so the artifacts match the oracle byte-for-byte in
// structure (number lexemes are emitted verbatim).
[[nodiscard]] std::string dump_json(const Value &value);

// Python repr() of a flat {str: str} mapping — used for the
// "native context SDK controls" console line so output parity is retained.
[[nodiscard]] std::string repr_str_map(const std::vector<std::pair<std::string, std::string>> &ordered);

// Ports _write_summary for sweep kinds: writes
// <build>/native_check_summaries/<kind>.json with a trailing newline.
void write_summary(const fs::path &build, std::string_view kind, const Value &summary);
// Ports check_native_contracts._write_summary: tmp file + os.replace().
void write_summary_atomic(const fs::path &build, const Value &summary);

// ---- clock -----------------------------------------------------------------

// Ports _new_run_id(): "%Y%m%dT%H%M%S%fZ" in UTC.
[[nodiscard]] std::string new_run_id();
// Ports datetime.now(timezone.utc).isoformat(): "YYYY-MM-DDTHH:MM:SS.ffffff+00:00".
[[nodiscard]] std::string utc_iso_now();

// ---- filesystem ------------------------------------------------------------

// Ports Path(path); resolve(strict=False): absolutize (against `base` when
// relative and base is set), resolve symlinks of the existing prefix, and
// lexically normalize the remainder — never throws for missing leaves.
[[nodiscard]] fs::path canonical_path(const fs::path &path, const fs::path *base = nullptr);
// Ports Path(value).expanduser(): "~" and "~user" expansion; an unknown
// user or missing HOME is a CheckError like the source's RuntimeError.
[[nodiscard]] fs::path expand_home(const std::string &value);
// Ports Path.absolute(): join with cwd when relative, no normalization.
[[nodiscard]] fs::path absolute_lexical(const fs::path &path);
// Ports _is_under: equality or component-wise containment under any root.
[[nodiscard]] bool is_under(const fs::path &path, const std::vector<fs::path> &roots);
// os.access(X_OK) on a regular file.
[[nodiscard]] bool is_executable_file(const fs::path &path);
// Ports shutil.which on POSIX: PATH entries are searched in order (empty
// entries mean the current directory); returns the located path unresolved.
[[nodiscard]] std::optional<fs::path> which(const std::string &name);
// The process environment as a {name: value} map (os.environ.copy()).
[[nodiscard]] std::map<std::string, std::string> environ_copy();
[[nodiscard]] std::string getenv_string(const char *name);

// ---- diagnostics -----------------------------------------------------------

// _DIAGNOSTIC: r"\b(?:warning|error):|fatal error:" (case-insensitive).
[[nodiscard]] bool is_diagnostic_line(std::string_view line);
// str.splitlines() for the tool streams (LF/CRLF/CR plus the vertical-tab,
// form-feed and NEL/LS/PS separators Python recognizes).
[[nodiscard]] std::vector<std::string> split_lines(const std::string &text);

// _LOCATED_DIAGNOSTIC: origin:line:column: severity: message [checker].
struct LocatedDiagnostic {
    std::string origin;
    std::string severity;
    std::string message;
    std::string checker; // "" when the bracket group is absent
};
[[nodiscard]] std::optional<LocatedDiagnostic> parse_located_diagnostic(
    const std::string &line);

[[nodiscard]] bool has_warning(const std::vector<std::string> &diagnostics);
[[nodiscard]] bool has_error(const std::vector<std::string> &diagnostics);

// ---- suppressions ----------------------------------------------------------

struct Suppression {
    std::string checker;
    std::string origin;
    std::string message;
    std::string tool_version;
    std::string reason;
    std::string reproducer;
    std::string remove_when;
    std::regex origin_regex;
};

// Ports load_suppressions(): schema_version==1, a suppressions list, and
// every entry carrying nonempty string checker/origin/message/tool_version/
// reason/reproducer/remove_when fields with a compilable origin regex.
[[nodiscard]] std::vector<Suppression> load_suppressions(const fs::path &path);

// Ports apply_suppressions(): a diagnostic is suppressed iff it parses as a
// located diagnostic whose checker equals the entry's, whose tool version
// contains the entry's tool_version substring, whose message contains the
// entry's message substring, and whose origin matches the entry's regex.
struct SuppressionFilter {
    std::vector<std::string> remaining;
    Array matched; // {"diagnostic","checker","origin","reason"} objects
};
[[nodiscard]] SuppressionFilter apply_suppressions(
    const std::vector<std::string> &diagnostics,
    const std::string &tool_version,
    const std::vector<Suppression> &suppressions);

// ---- subprocess records ----------------------------------------------------

// Ports _run_process(): argv-only (shell-free) execution through
// platform::run, separate stdout/stderr capture, per-run log files under
// log_directory/<name>.{stdout,stderr}.log, deadline kill, launch errors
// and timeouts recorded instead of thrown.
//   env                -> nullptr inherits; a pointer replaces the whole
//                         environment (dict(env) semantics).
//   disable_core_dumps -> re-exec this binary under --check-child, which
//                         sets RLIMIT_CORE=0 before exec (preexec_fn port).
//   timeout_seconds    -> whole-run deadline, like subprocess.run(timeout=).
[[nodiscard]] Value run_process(const std::vector<std::string> &argv,
                                const fs::path &cwd,
                                const fs::path &log_directory,
                                const std::string &name,
                                const std::map<std::string, std::string> *env = nullptr,
                                bool disable_core_dumps = false,
                                std::int64_t timeout_seconds = 1800);

// Ports _version_record(): "<tool> --version" with a 60 s deadline; the
// record gains a "version" member with the stripped combined output.
[[nodiscard]] Value version_record(const fs::path &tool,
                                   const fs::path &log_directory);

// Record accessors used by the sweeps (return defaults on absent members).
[[nodiscard]] std::optional<std::int64_t> record_returncode(const Value &record);
[[nodiscard]] bool record_health_ok(const Value &record);
[[nodiscard]] std::vector<std::string> record_diagnostics(const Value &record);
[[nodiscard]] std::string record_stdout_text(const Value &record);
[[nodiscard]] std::string record_stderr_text(const Value &record);

// ---- context / manifest / compile database ---------------------------------

// Ports load_context(): strict validation of every documented field; paths
// are canonicalized in place. The standalone context may carry
// "python_executable": null (or omit it) — the Python-free core does not
// require an interpreter; when a string is present it must be an executable
// file, exactly as the oracle requires.
[[nodiscard]] Value load_context(const fs::path &build);

// One compile_commands entry validated against context and manifest.
struct CompileEntry {
    fs::path source;
    fs::path directory;
    std::vector<std::string> arguments;
    std::optional<std::string> target;
    Value raw; // the raw JSON object (for _temporary_database's "output")
};

// Ports load_entries(): loads the context and manifest itself, validates the
// manifest (schema, roots, unique source/target/context identities),
// validates every compile-commands entry (argv form parity, compiler/sysroot/
// paths vs. context, target contexts, ISO C++23 mode), requires the manifest
// and database TU sets to agree, and returns entries ordered by source path
// then database order. `expected` is the caller's source set; an empty set
// or a mismatch fails closed.
[[nodiscard]] std::vector<CompileEntry> load_entries(
    const fs::path &build, const std::vector<fs::path> &expected);

// Reads native_sources.json and returns the manifest Value plus canonical
// first-party roots and declared source paths.
struct SourceManifest {
    Value document;
    std::vector<fs::path> first_party_roots;
    std::vector<fs::path> sources;
};
[[nodiscard]] SourceManifest load_source_manifest(const fs::path &build);

// Ports _resolve_tool(): env override -> context tool_overrides -> context
// tool_defaults -> PATH lookup; a path with a separator must exist and be
// executable. Returns {resolved tool path, override env-var name}.
[[nodiscard]] std::pair<fs::path, std::string> resolve_tool(
    const std::string &kind, const Value &context,
    const std::map<std::string, std::string> &environment);

// Ports _temporary_database(): <build>/native_check_work/<kind>/<run_id>/
// entry-NNNN/compile_commands.json containing exactly this entry.
[[nodiscard]] fs::path temporary_database(const fs::path &build,
                                          const std::string &kind,
                                          const std::string &run_id,
                                          std::size_t index,
                                          const CompileEntry &entry);

// Ports _replace_compile_source(): drop -o/-MF/-MT/-MQ pairs and
// -c/-MD/-MMD/-MP flags, substitute `replacement` for the entry source.
[[nodiscard]] std::vector<std::string> replace_compile_source(
    const CompileEntry &entry, const fs::path &replacement, bool syntax_only);

// Ports _gcc_context_args() for the gcc-side sweeps (frontends/odr).
[[nodiscard]] std::vector<std::string> gcc_context_args(const CompileEntry &entry,
                                                        const Value &context);

// Ports _standalone_header_roots(): <root>/src and <root>/tests per
// first-party root plus the repository tools/proto_native_bench tree.
[[nodiscard]] std::vector<fs::path> standalone_header_roots(
    const std::vector<fs::path> &first_party_roots);

// ---- sweeps -----------------------------------------------------------------

// run_sweep() for the per-translation-unit kinds, plus the header and
// scripts sweeps. Returns the process exit code (0 pass / 1 fail).
[[nodiscard]] int run_sweep(const std::string &kind, const fs::path &build);
[[nodiscard]] int run_header_sweep(const fs::path &build);
[[nodiscard]] int run_scripts(const fs::path &build,
                              const std::optional<fs::path> &tools_dir);
[[nodiscard]] int run_diagnostic_controls(const fs::path &build);
[[nodiscard]] int run_context_checks(const fs::path &build);
[[nodiscard]] int run_contracts(const fs::path &build,
                                const std::optional<fs::path> &repo_root);

// ---- misc -------------------------------------------------------------------

// fnmatch.fnmatchcase port used by the clang-tidy Checks policy validator.
[[nodiscard]] bool fnmatchcase(const std::string &value, const std::string &pattern);

// Deep structural equality over the Value model: null/bool/string/number by
// value (numbers compare lexemes), arrays elementwise, objects order-
// insensitively by key (duplicate keys behave like Python dict lookups —
// last wins). Used by the libc++ hardening sizeof evidence comparison.
[[nodiscard]] bool json_equal(const Value &left, const Value &right);

// The UTF-8 "replace" decoder port: malformed byte runs become U+FFFD so
// record fields are always valid UTF-8 for the summary writer.
[[nodiscard]] std::string utf8_replace(const std::string &bytes);

// Strict whole-string decimal integer parse — the bounds-safe replacement
// for strtoll on Number lexemes; malformed or overflowing text yields
// nullopt so callers fail closed or skip.
[[nodiscard]] std::optional<std::int64_t> parse_int64(const std::string &text);

} // namespace bike::checks
