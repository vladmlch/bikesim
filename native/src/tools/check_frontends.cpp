// check_frontends.cpp — the translation-unit sweeps plus the standalone
// header and shell-script sweeps for the native check orchestrator
// (task T56). Ports tools/native_checks.py: run_sweep for tidy/analyzer/
// cppcheck/frontends/odr (including the stable/alpha pass split and the
// report-only alpha contract), run_header_sweep, _tool_argv,
// _report_alpha_findings; and tools/check_scripts.sh (find tools/*.sh |
// sort | xargs shellcheck) as the `scripts` kind.
#include "checks_internal.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <set>
#include <sstream>

namespace bike::checks {
namespace {

constexpr std::array<std::string_view, 53> kGccWarningFlags{
    "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-Wconversion",
    "-Wsign-conversion", "-Wdouble-promotion", "-Wshadow", "-Wcast-qual",
    "-Wformat=2", "-Wundef", "-Wimplicit-fallthrough", "-Wnon-virtual-dtor",
    "-Wold-style-cast", "-Woverloaded-virtual", "-Wnull-dereference",
    "-Wlogical-op", "-Wduplicated-cond", "-Wuseless-cast",
    "-Wstringop-overflow=4", "-fanalyzer", "-Warith-conversion",
    "-Wduplicated-branches", "-Wrestrict", "-Wformat-signedness",
    "-Wformat-overflow=2", "-Wformat-truncation=2", "-Wextra-semi",
    "-Wredundant-tags", "-Wcast-function-type", "-Wcast-align=strict",
    "-Wcatch-value=3", "-Wconditionally-supported", "-Wdeprecated-copy-dtor",
    "-Wvolatile", "-Winit-self", "-Wsign-promo", "-Wctor-dtor-privacy",
    "-Wplacement-new=2", "-Wmismatched-new-delete", "-Wsized-deallocation",
    "-Winterference-size", "-Wsubobject-linkage", "-Wsuggest-override",
    "-Wstrict-null-sentinel", "-Walloca", "-Wvla", "-Warray-bounds=2",
    "-Waggressive-loop-optimizations", "-Wstack-usage=8192",
    "-Wframe-larger-than=8192", "-Wmissing-declarations", "-Wswitch-enum",
};

// zero-padded 4-wide index, matching the oracle's "%04d" record naming
std::string padded_index(std::size_t index) {
    std::ostringstream out;
    out << std::setfill('0') << std::setw(4) << index;
    return out.str();
}

// _tool_argv: per-kind process argv construction.
std::vector<std::string> tool_argv(const std::string &kind,
                                   const fs::path &tool,
                                   const CompileEntry &entry,
                                   const Value &context, const fs::path &build,
                                   const std::string &run_id, std::size_t index,
                                   const std::string &pass_name) {
    const fs::path &source = entry.source;
    if (kind == "tidy") {
        const fs::path database =
            temporary_database(build, kind, run_id, index, entry);
        std::vector<std::string> argv = {tool.string(), "-p", database.string(),
                                         "--extra-arg=-Wno-error", "--quiet"};
        // First-party sources outside native/ (e.g. tools/proto_native_bench)
        // do not inherit native/.clang-tidy through parent-directory
        // discovery — without it clang-tidy reports "no checks enabled".
        const auto &roots_array =
            std::get<Array>(member(context, "source_roots")->data);
        const fs::path policy =
            fs::path(std::get<std::string>(roots_array.front().data)) /
            ".clang-tidy";
        std::error_code ec;
        if (fs::is_regular_file(policy, ec))
            argv.push_back("--config-file=" + policy.string());
        argv.push_back(source.string());
        return argv;
    }
    if (kind == "analyzer") {
        std::vector<std::string> cleaned;
        cleaned.reserve(entry.arguments.size());
        std::size_t position = 0;
        while (position < entry.arguments.size()) {
            const std::string &argument = entry.arguments[position];
            if (position == 0) {
                cleaned.push_back(tool.string());
            } else if (argument == "-o") {
                position += 2;
                continue;
            } else if (argument == "-c" || argument == "-MD" ||
                       argument == "-MMD" || argument == "-MP") {
                position += 1;
                continue;
            } else {
                cleaned.push_back(argument);
            }
            position += 1;
        }
        if (cleaned.empty()) cleaned.push_back(tool.string());
        std::vector<std::string> extra = {"-Xanalyzer", "-analyzer-output=text"};
        if (pass_name == "stable") {
            extra.insert(extra.end(),
                         {"-Xanalyzer", "-analyzer-checker=optin", "-Xanalyzer",
                          "-analyzer-checker=cplusplus.Move"});
        } else {
            extra.insert(extra.end(),
                         {"-Xanalyzer", "-analyzer-checker=alpha", "-Xanalyzer",
                          "-analyzer-config", "-Xanalyzer",
                          "aggressive-binary-operation-simplification=true",
                          "-Wno-error"});
        }
        cleaned.emplace_back("--analyze");
        cleaned.insert(cleaned.end(), extra.begin(), extra.end());
        return cleaned;
    }
    if (kind == "cppcheck") {
        const fs::path database =
            temporary_database(build, kind, run_id, index, entry);
        std::vector<std::string> argv = {
            tool.string(),
            "--project=" + (database / "compile_commands.json").string(),
        };
        // _extra_dependency_args: every validated include path.
        for (const Value &item:
             std::get<Array>(member(context, "include_paths")->data)) {
            argv.emplace_back("-I");
            argv.push_back(std::get<std::string>(item.data));
        }
        const std::vector<std::string> fixed = {
            "--enable=warning,performance,portability",
            "--inconclusive",
            "--std=c++23",
            "--error-exitcode=1",
            "--inline-suppr",
            "-j1",
            "--suppress=missingIncludeSystem",
            "--suppress=uninitMemberVarNoCtor",
            "--suppress=passedByValue",
            "--suppress=returnByReference",
            "--suppress=normalCheckLevelMaxBranches",
            "--quiet",
        };
        argv.insert(argv.end(), fixed.begin(), fixed.end());
        return argv;
    }
    const std::vector<std::string> gcc_args = gcc_context_args(entry, context);
    if (kind == "frontends") {
        std::vector<std::string> argv = {tool.string(), "-std=c++23",
                                         "-fsyntax-only"};
        argv.insert(argv.end(), gcc_args.begin(), gcc_args.end());
        for (const std::string_view flag: kGccWarningFlags)
            argv.emplace_back(flag);
        argv.push_back(source.string());
        return argv;
    }
    if (kind == "odr") {
        const std::string object_name = padded_index(index) + ".o";
        const std::string rename_macro =
            "-Dmain=native_check_odr_entry_" + padded_index(index);
        const fs::path object_path =
            build / "native_check_work" / kind / run_id / object_name;
        std::error_code ec;
        fs::create_directories(object_path.parent_path(), ec);
        std::vector<std::string> argv = {
            tool.string(), "-std=c++23", "-O2", "-flto=auto",
            "-ffat-lto-objects", "-Wno-psabi",
            // Sweep TUs may each define `main`; rename it uniquely per TU so
            // the relocatable link sees no duplicate-symbol conflict.
            rename_macro,
        };
        argv.insert(argv.end(), gcc_args.begin(), gcc_args.end());
        argv.insert(argv.end(), {"-c", source.string(), "-o",
                                 object_path.string()});
        return argv;
    }
    throw CheckError("unknown native sweep kind: " + kind);
}

// _report_alpha_findings: alpha-pass warnings are report-only — printed to
// stderr, never blocking.
void report_alpha_findings(const Value &record, const fs::path &source) {
    std::vector<std::string> lines;
    if (const Value *unsuppressed = member(record, "unsuppressed_diagnostics");
        unsuppressed != nullptr) {
        for (const Value &item: std::get<Array>(unsuppressed->data))
            lines.push_back(std::get<std::string>(item.data));
    } else {
        lines = record_diagnostics(record);
    }
    std::vector<std::string> warnings;
    for (const auto &line: lines)
        if (has_warning({line})) warnings.push_back(line);
    if (!warnings.empty()) {
        std::cerr << "== alpha findings (report-only): " << source.string()
                  << "\n";
        for (const auto &line: warnings) std::cerr << line << "\n";
    }
}

} // namespace

int run_sweep(const std::string &kind, const fs::path &build) {
    const fs::path build_dir = canonical_path(build);
    std::error_code ec;
    if (!fs::is_directory(build_dir, ec)) {
        std::cerr << "native sweep failed: build directory does not exist: "
                  << build_dir.string() << "\n";
        return 1;
    }
    const std::string run_id = new_run_id();
    Value summary = vobj({
        {"schema_version", vint(1)},
        {"kind", vstr(kind)},
        {"build", vstr(build_dir.string())},
        {"run_id", vstr(run_id)},
        {"tool_health", vstr("failed")},
        {"finding_status", vstr("none")},
        {"status", vstr("failed")},
        {"unique_source_count", vint(0)},
        {"translation_unit_count", vint(0)},
        {"selected_sources", varr({})},
        {"records", varr({})},
        {"errors", varr({})},
    });
    Value context;
    std::vector<CompileEntry> entries;
    std::vector<Suppression> suppressions;
    fs::path tool;
    std::string override_name;
    try {
        context = load_context(build_dir);
        const SourceManifest manifest = load_source_manifest(build_dir);
        entries = load_entries(build_dir, manifest.sources);
        if (kind == "analyzer" || kind == "tidy")
            suppressions = load_suppressions(fs::path(std::get<std::string>(
                member(context, "analysis_suppressions_manifest")->data)));
        const auto environment = environ_copy();
        auto resolved = resolve_tool(kind, context, environment);
        tool = std::move(resolved.first);
        override_name = std::move(resolved.second);
        set_member(summary, "tool", vstr(tool.string()));
        set_member(summary, "tool_override", vstr(override_name));
        Object overrides;
        for (const std::string_view name:
             {"CLANG_TIDY", "ANALYZER_CLANG", "CPPCHECK", "GXX"})
            if (const auto found =
                    environment.find(std::string(name));
                found != environment.end())
                overrides.emplace_back(std::string(name),
                                       vstr(found->second));
        set_member(summary, "tool_overrides", vobj(std::move(overrides)));
        set_member(summary, "compiler_context",
                   *member(context, "compiler"));
        set_member(summary, "sdk", *member(context, "sdk"));
        set_member(summary, "python_executable",
                   member(context, "python_executable") != nullptr
                       ? *member(context, "python_executable")
                       : vnull());
        std::set<std::string> unique_sources;
        for (const auto &entry: entries) unique_sources.insert(entry.source.string());
        set_member(summary, "unique_source_count",
                   vint(static_cast<std::int64_t>(unique_sources.size())));
        set_member(summary, "translation_unit_count",
                   vint(static_cast<std::int64_t>(entries.size())));
        Array selected;
        for (const std::string &source: unique_sources)
            selected.push_back(vstr(source));
        set_member(summary, "selected_sources", varr(std::move(selected)));
        Array selected_units;
        for (const auto &entry: entries)
            selected_units.push_back(vobj({
                {"source", vstr(entry.source.string())},
                {"target", entry.target.has_value() ? vstr(*entry.target)
                                                    : vnull()},
            }));
        set_member(summary, "selected_translation_units",
                   varr(std::move(selected_units)));
        set_member(summary, "suppression_counts", vobj({}));
    } catch (const CheckError &error) {
        auto *errors = member_mut(summary, "errors");
        std::get<Array>(errors->data).push_back(vstr(error.what()));
        write_summary(build_dir, kind, summary);
        std::cerr << "native sweep failed: " << error.what() << "\n";
        return 1;
    } catch (const std::invalid_argument &error) {
        auto *errors = member_mut(summary, "errors");
        std::get<Array>(errors->data).push_back(vstr(error.what()));
        write_summary(build_dir, kind, summary);
        std::cerr << "native sweep failed: " << error.what() << "\n";
        return 1;
    } catch (const std::exception &error) {
        auto *errors = member_mut(summary, "errors");
        std::get<Array>(errors->data).push_back(vstr(error.what()));
        write_summary(build_dir, kind, summary);
        std::cerr << "native sweep failed: " << error.what() << "\n";
        return 1;
    }

    std::cout << "native " << kind << " selection: "
              << std::get<io::Number>(member(summary, "unique_source_count")->data)
                     .lexeme
              << " unique sources, "
              << std::get<io::Number>(
                     member(summary, "translation_unit_count")->data)
                     .lexeme
              << " translation units\n";
    for (const Value &source_value:
         std::get<Array>(member(summary, "selected_sources")->data))
        std::cout << "  " << std::get<std::string>(source_value.data) << "\n";
    std::cout << "native " << kind << " tool: " << tool.string() << "\n";

    const fs::path log_directory =
        build_dir / "native_check_logs" / kind / run_id;
    const Value version = version_record(tool, log_directory);
    std::string tool_version;
    if (const Value *version_text = member(version, "version"))
        tool_version = std::get<std::string>(version_text->data);
    set_member(summary, "tool_version", vstr(tool_version));
    Array records;
    if (!record_health_ok(version)) {
        // Python appends to summary["records"] here and later assigns
        // summary["records"] = records — the version record is therefore
        // dropped from the written summary. The append is reproduced against
        // the member so the same drop occurs.
        Value version_copy = version;
        Object version_record_members = std::get<Object>(version_copy.data);
        version_record_members.insert(version_record_members.begin(),
                                      {"pass", vstr("version")});
        std::get<Array>(member_mut(summary, "records")->data)
            .push_back(vobj(std::move(version_record_members)));
    }
    bool blocking_findings = false;
    bool report_only_findings = false;
    bool tool_health_failed = !record_health_ok(version);
    std::vector<fs::path> object_paths;
    if (kind == "odr") {
        for (std::size_t index = 0; index < entries.size(); ++index) {
            object_paths.push_back(build_dir / "native_check_work" / kind /
                                   run_id / (padded_index(index) + ".o"));
        }
    }

    for (std::size_t index = 0; index < entries.size(); ++index) {
        const CompileEntry &entry = entries[index];
        const std::vector<std::string> passes =
            kind == "analyzer" ? std::vector<std::string>{"stable", "alpha"}
                               : std::vector<std::string>{"stable"};
        bool stable_failed = false;
        for (const std::string &pass_name: passes) {
            if (kind == "analyzer" && pass_name == "alpha" && stable_failed)
                continue;
            const std::vector<std::string> argv =
                tool_argv(kind, tool, entry, context, build_dir, run_id, index,
                          pass_name);
            std::string name = padded_index(index);
            name += '-';
            name += entry.source.stem().string();
            name += '-';
            name += pass_name;
            Value record =
                run_process(argv, entry.directory, log_directory, name);
            set_member(record, "pass", vstr(pass_name));
            set_member(record, "source", vstr(entry.source.string()));
            set_member(record, "target",
                       entry.target.has_value() ? vstr(*entry.target)
                                                : vnull());
            std::vector<std::string> unsuppressed;
            if (kind == "analyzer" || kind == "tidy") {
                const auto filtered = apply_suppressions(
                    record_diagnostics(record), tool_version, suppressions);
                unsuppressed = filtered.remaining;
                set_member(record, "unsuppressed_diagnostics",
                           varr([&]() {
                               Array items;
                               for (const auto &line: unsuppressed)
                                   items.push_back(vstr(line));
                               return items;
                           }()));
                set_member(record, "suppressed_diagnostics",
                           varr(filtered.matched));
                for (const Value &item: filtered.matched) {
                    const std::string checker = std::get<std::string>(
                        member(item, "checker")->data);
                    Value *counts = member_mut(summary, "suppression_counts");
                    auto &counts_object = std::get<Object>(counts->data);
                    bool found = false;
                    for (auto &[key, value]: counts_object) {
                        if (key == checker) {
                            const std::int64_t count =
                                parse_int64(std::get<io::Number>(value.data)
                                                .lexeme)
                                    .value_or(0);
                            value = vint(count + 1);
                            found = true;
                        }
                    }
                    if (!found)
                        counts_object.emplace_back(checker, vint(1));
                }
            } else {
                unsuppressed = record_diagnostics(record);
            }
            records.push_back(record);
            if (!record_health_ok(record)) tool_health_failed = true;
            const bool warning = has_warning(unsuppressed);
            const bool error = has_error(unsuppressed);
            if (kind == "analyzer" && pass_name == "alpha") {
                if (warning) {
                    report_only_findings = true;
                    report_alpha_findings(record, entry.source);
                }
                if (error)
                    // An alpha finding may be noisy, but an error diagnostic
                    // is a broken analysis run.
                    tool_health_failed = true;
                continue;
            }
            if (warning || error) {
                blocking_findings = true;
                if (kind == "analyzer" && pass_name == "stable")
                    stable_failed = true;
            }
        }
    }

    if (kind == "odr" && !tool_health_failed) {
        std::vector<std::string> linker = {tool.string(), "-flto=auto",
                                           "-Wodr", "-Wno-psabi", "-r"};
        for (const fs::path &object: object_paths)
            linker.push_back(object.string());
        linker.emplace_back("-o");
        linker.push_back(
            (build_dir / "native_check_work" / kind / run_id / "combined.o")
                .string());
        Value link_record =
            run_process(linker, build_dir, log_directory, "odr-link");
        set_member(link_record, "pass", vstr("link"));
        set_member(link_record, "source", vnull());
        set_member(link_record, "target", vnull());
        if (!record_health_ok(link_record)) tool_health_failed = true;
        if (!record_diagnostics(link_record).empty()) blocking_findings = true;
        records.push_back(std::move(link_record));
    }

    set_member(summary, "records", varr(std::move(records)));
    set_member(summary, "tool_health",
               vstr(tool_health_failed ? "failed" : "ok"));
    set_member(summary, "finding_status",
               vstr(blocking_findings ? "blocking"
                    : report_only_findings ? "report-only"
                                           : "none"));
    set_member(summary, "status",
               vstr(!tool_health_failed && !blocking_findings ? "passed"
                                                              : "failed"));
    set_member(summary, "log_directory", vstr(log_directory.string()));
    write_summary(build_dir, kind, summary);

    const std::string tool_health =
        std::get<std::string>(member(summary, "tool_health")->data);
    const std::string finding_status =
        std::get<std::string>(member(summary, "finding_status")->data);
    std::cout << "native " << kind << " tool health: " << tool_health << "\n";
    std::cout << "native " << kind << " findings: " << finding_status << "\n";
    const Value *counts = member(summary, "suppression_counts");
    if (counts != nullptr && !std::get<Object>(counts->data).empty()) {
        std::map<std::string, std::int64_t> sorted_counts;
        for (const auto &[checker, value]: std::get<Object>(counts->data))
            sorted_counts[checker] =
                parse_int64(std::get<io::Number>(value.data).lexeme)
                    .value_or(0);
        std::string joined;
        for (const auto &[checker, count]: sorted_counts) {
            if (!joined.empty()) joined += ", ";
            joined += checker + "=" + std::to_string(count);
        }
        std::cout << "native " << kind << " suppressions: " << joined << "\n";
    }
    std::cout << "native " << kind << " summary: "
              << (build_dir / "native_check_summaries" / (kind + ".json"))
                     .string()
              << "\n";
    if (std::get<std::string>(member(summary, "status")->data) != "passed") {
        for (const Value &record: std::get<Array>(member(summary, "records")->data)) {
            if (!record_health_ok(record) ||
                !record_diagnostics(record).empty()) {
                for (const auto &line: record_diagnostics(record))
                    std::cerr << line << "\n";
                const Value *pass = member(record, "pass");
                const Value *source = member(record, "source");
                std::cerr << "  "
                          << (pass != nullptr &&
                                      std::holds_alternative<std::string>(
                                          pass->data)
                                  ? std::get<std::string>(pass->data)
                                  : std::string("null"))
                          << " "
                          << (source != nullptr &&
                                      std::holds_alternative<std::string>(
                                          source->data)
                                  ? std::get<std::string>(source->data)
                                  : std::string("null"))
                          << ": rc="
                          << (member(record, "returncode") != nullptr &&
                                          !io::is_null(
                                              *member(record, "returncode"))
                                      ? std::get<io::Number>(
                                            member(record, "returncode")->data)
                                            .lexeme
                                      : std::string("None"))
                          << " stdout="
                          << std::get<std::string>(
                                 member(record, "stdout_log")->data)
                          << " stderr="
                          << std::get<std::string>(
                                 member(record, "stderr_log")->data)
                          << "\n";
            }
        }
        return 1;
    }
    return 0;
}

namespace {

// The header sweep's coverage question — which first-party headers a build
// actually exercises — is decided per context kind. A bindings context
// (context["python_executable"] set, the transition graph) keeps the
// oracle's contract verbatim: every first-party header compiles under every
// covering target. A produced standalone context applies textual
// reachability instead: a header is swept iff at least one compiled
// translation unit reaches it through its #include chain, under the compile
// context of each reaching target. That keeps binding-side headers —
// reachable only from translation units the produced build does not
// compile — out of the sweep without weakening the check for anything the
// build actually includes; the excluded set is recorded in the summary.
struct IncludeDirective {
    std::string name;
    bool quoted; // "x" resolves relative to the including file first; <x> is path-only
};

// One scan per file: the directive list is content-local; resolution is
// per-context because each target carries its own include directories.
std::vector<IncludeDirective> scan_include_directives(
    const fs::path &file,
    std::map<std::string, std::vector<IncludeDirective>> &cache) {
    const std::string key = file.string();
    if (const auto found = cache.find(key); found != cache.end())
        return found->second;
    std::vector<IncludeDirective> directives;
    std::ifstream stream(file, std::ios::binary);
    std::string line;
    const std::regex pattern(
        R"(^[ \t]*#[ \t]*include[ \t]*(["<])([^">]+)[">])");
    while (std::getline(stream, line)) {
        std::smatch match;
        if (std::regex_search(line, match, pattern))
            directives.push_back({.name=match.str(2), .quoted=match.str(1) == "\""});
    }
    cache.emplace(key, directives);
    return directives;
}

// Include directories a compile entry searches: -I/-isystem/-iquote in both
// joined and separate spellings, resolved against the entry's directory.
std::vector<fs::path> entry_include_dirs(const CompileEntry &entry) {
    std::vector<fs::path> dirs;
    for (std::size_t index = 1; index < entry.arguments.size(); ++index) {
        const std::string &argument = entry.arguments[index];
        const auto push_dir = [&](const std::string &value) {
            if (value.empty()) return;
            dirs.push_back(canonical_path(value, &entry.directory));
        };
        if (argument == "-I" || argument == "-isystem" || argument == "-iquote") {
            if (index + 1 < entry.arguments.size()) push_dir(entry.arguments[++index]);
        } else if (argument.starts_with("-isystem")) {
            push_dir(argument.substr(8));
        } else if (argument.starts_with("-iquote")) {
            push_dir(argument.substr(7));
        } else if (argument.starts_with("-I")) {
            push_dir(argument.substr(2));
        }
    }
    return dirs;
}

// Resolve one directive: quoted forms try the including file's directory
// first (the "" search rule), then the include path; angle forms search the
// include path only. Returns the canonical file when it exists.
std::optional<fs::path> resolve_include(const IncludeDirective &directive,
                                        const fs::path &including_dir,
                                        const std::vector<fs::path> &dirs) {
    std::error_code ec;
    if (directive.quoted) {
        const fs::path adjacent = including_dir / directive.name;
        if (fs::is_regular_file(adjacent, ec))
            return canonical_path(adjacent);
    }
    for (const fs::path &dir: dirs) {
        const fs::path candidate = dir / directive.name;
        if (fs::is_regular_file(candidate, ec))
            return canonical_path(candidate);
    }
    return std::nullopt;
}

// header -> set of target names whose translation units reach it. The walk
// starts at every compiled TU; each visited header expands with the SAME
// entry's include path (the context it would compile under).
std::map<std::string, std::set<std::string>> reaching_targets(
    const std::vector<CompileEntry> &entries,
    const std::vector<fs::path> &header_roots,
    std::map<std::string, std::vector<IncludeDirective>> &cache) {
    std::map<std::string, std::set<std::string>> reaching;
    for (const CompileEntry &entry: entries) {
        const std::string target = entry.target.value_or("");
        const std::vector<fs::path> dirs = entry_include_dirs(entry);
        std::set<std::string> visited;
        std::vector<fs::path> frontier{entry.source};
        while (!frontier.empty()) {
            const fs::path file = frontier.back();
            frontier.pop_back();
            if (!visited.insert(file.string()).second) continue;
            const fs::path including_dir = file.parent_path();
            for (const IncludeDirective &directive:
                 scan_include_directives(file, cache)) {
                const std::optional<fs::path> resolved =
                    resolve_include(directive, including_dir, dirs);
                if (!resolved.has_value()) continue;
                const fs::path &header = *resolved;
                // Only first-party files are walked or recorded: third-party
                // headers never include the first-party surface back, so the
                // reachability index stays bounded and exact.
                if (is_under(header, header_roots)) {
                    reaching[header.string()].insert(target);
                    frontier.push_back(header);
                }
            }
        }
    }
    return reaching;
}

// The contexts one reachable header compiles under: the covering root's
// per-target entries restricted to targets whose TUs reach the header. When
// none of the reaching targets lives under that root (e.g. a bench TU
// pulling a native/src header), every covering context applies — identical
// to the oracle's all-contexts contract.
std::vector<std::pair<std::string, const CompileEntry *>> header_contexts(
    const fs::path &header, const fs::path &root,
    const std::map<std::string, std::set<std::string>> &reaching,
    const std::map<std::string, std::map<std::string, const CompileEntry *>> &
        entries_by_root) {
    const auto &contexts = entries_by_root.at(root.string());
    const auto found = reaching.find(header.string());
    std::vector<std::pair<std::string, const CompileEntry *>> selected;
    if (found != reaching.end())
        for (const auto &[target, entry]: contexts)
            if (found->second.contains(target))
                selected.emplace_back(target, entry);
    if (selected.empty())
        for (const auto &[target, entry]: contexts)
            selected.emplace_back(target, entry);
    return selected;
}

} // namespace

int run_header_sweep(const fs::path &build) {
    const fs::path build_dir = canonical_path(build);
    const std::string kind = "headers";
    std::error_code ec;
    if (!fs::is_directory(build_dir, ec)) {
        std::cerr << "native headers failed: build directory does not exist: "
                  << build_dir.string() << "\n";
        return 1;
    }
    const std::string run_id = new_run_id();
    Value summary = vobj({
        {"schema_version", vint(1)},
        {"kind", vstr(kind)},
        {"build", vstr(build_dir.string())},
        {"run_id", vstr(run_id)},
        {"tool_health", vstr("failed")},
        {"finding_status", vstr("none")},
        {"status", vstr("failed")},
        {"header_count", vint(0)},
        {"header_translation_unit_count", vint(0)},
        {"records", varr({})},
        {"errors", varr({})},
    });
    Value context;
    std::vector<CompileEntry> entries;
    std::vector<fs::path> headers;
    std::vector<fs::path> excluded_headers;
    std::map<std::string, std::set<std::string>> reaching;
    std::map<std::string, std::map<std::string, const CompileEntry *>>
        entries_by_root;
    std::vector<fs::path> header_roots;
    fs::path compiler;
    try {
        context = load_context(build_dir);
        const SourceManifest manifest = load_source_manifest(build_dir);
        entries = load_entries(build_dir, manifest.sources);
        header_roots = standalone_header_roots(manifest.first_party_roots);
        // Every header under the standalone roots, deduplicated and sorted by
        // resolved path (sorted(set(Path.rglob(...))) semantics).
        std::set<std::string> seen;
        std::vector<fs::path> collected;
        for (const fs::path &root: header_roots) {
            std::error_code iter_ec;
            for (fs::recursive_directory_iterator it(root, iter_ec), end;
                 !iter_ec && it != end; it.increment(iter_ec)) {
                const std::string extension = it->path().extension().string();
                if (extension != ".h" && extension != ".hh" &&
                    extension != ".hpp" && extension != ".hxx")
                    continue;
                std::error_code file_ec;
                if (!it->is_regular_file(file_ec)) continue;
                const fs::path resolved = canonical_path(it->path());
                if (seen.insert(resolved.string()).second)
                    collected.push_back(resolved);
            }
        }
        std::ranges::sort(collected,
                  [](const fs::path &a, const fs::path &b) {
                      return a.string() < b.string();
                  });
        headers = std::move(collected);
        if (headers.empty())
            throw CheckError(
                "no first-party headers found for standalone compilation");

        for (const fs::path &root: header_roots)
            for (const CompileEntry &entry: entries)
                if (is_under(entry.source, {root}))
                    // dict comprehension semantics: later entries overwrite
                    // the same target key.
                    entries_by_root[root.string()][entry.target.value_or("")] =
                        &entry;
        // Reachability scoping applies only to produced (Python-free)
        // contexts: binding-side headers are reachable exclusively from
        // translation units the standalone graph does not build, so they are
        // recorded, not swept. A bindings context keeps the oracle's exact
        // contract — every header under every covering target — because the
        // extension TU set compiles the whole first-party surface there.
        const bool bindings_expected =
            opt_string(context, "python_executable").has_value();
        if (!bindings_expected) {
            std::map<std::string, std::vector<IncludeDirective>>
                include_cache;
            reaching = reaching_targets(entries, header_roots, include_cache);
            std::vector<fs::path> swept;
            std::vector<fs::path> excluded;
            for (const fs::path &header: headers) {
                if (reaching.contains(header.string()))
                    swept.push_back(header);
                else
                    excluded.push_back(header);
            }
            excluded_headers = std::move(excluded);
            headers = std::move(swept);
            if (headers.empty())
                throw CheckError(
                    "no first-party headers are reachable from compiled "
                    "translation units");
        }

        for (const fs::path &header: headers) {
            const fs::path *root = nullptr;
            for (const fs::path &candidate: header_roots)
                if (is_under(header, {candidate})) {
                    root = &candidate;
                    break;
                }
            if (root == nullptr ||
                entries_by_root.find(root->string()) == entries_by_root.end() ||
                entries_by_root[root->string()].empty())
                throw CheckError(
                    "no configured target compile context for first-party "
                    "header: " +
                    header.string());
        }

        compiler = fs::path(std::get<std::string>(
            member(*member(context, "compiler"), "path")->data));
        set_member(summary, "tool", vstr(compiler.string()));
        const Value version = version_record(
            compiler, build_dir / "native_check_logs" / kind / run_id);
        if (const Value *text = member(version, "version"))
            set_member(summary, "tool_version",
                       vstr(std::get<std::string>(text->data)));
        set_member(summary, "header_count",
                   vint(static_cast<std::int64_t>(headers.size())));
        std::size_t tu_count = 0;
        for (const fs::path &header: headers) {
            for (const fs::path &candidate: header_roots)
                if (is_under(header, {candidate})) {
                    tu_count += header_contexts(
                        header, candidate, reaching, entries_by_root)
                                    .size();
                    break;
                }
        }
        set_member(summary, "header_translation_unit_count",
                   vint(static_cast<std::int64_t>(tu_count)));
        Array header_values;
        for (const fs::path &header: headers)
            header_values.push_back(vstr(header.string()));
        set_member(summary, "headers", varr(std::move(header_values)));
        // Provenance for the scoping decision: headers under the roots that
        // no compiled TU reaches are reported, never silently skipped.
        // Produced-only keys — a bindings context emits exactly the oracle's
        // summary shape.
        if (!bindings_expected) {
            Array excluded_values;
            for (const fs::path &header: excluded_headers)
                excluded_values.push_back(vstr(header.string()));
            set_member(summary, "unreachable_headers",
                       varr(std::move(excluded_values)));
            set_member(summary, "unreachable_header_count",
                       vint(static_cast<std::int64_t>(excluded_headers.size())));
        }
    } catch (const std::exception &error) {
        std::get<Array>(member_mut(summary, "errors")->data)
            .push_back(vstr(error.what()));
        write_summary(build_dir, kind, summary);
        std::cerr << "native headers failed: " << error.what() << "\n";
        return 1;
    }

    const fs::path work_root = build_dir / "native_check_work" / kind / run_id;
    fs::create_directories(work_root, ec);
    const fs::path log_directory =
        build_dir / "native_check_logs" / kind / run_id;
    Array records;
    bool tool_health_failed = false;
    bool blocking_findings = false;
    std::size_t index = 0;
    for (const fs::path &header: headers) {
        const fs::path *root = nullptr;
        for (const fs::path &candidate: header_roots)
            if (is_under(header, {candidate})) {
                root = &candidate;
                break;
            }
        for (const auto &[target, entry]:
             header_contexts(header, *root, reaching, entries_by_root)) {
            // tempfile.TemporaryDirectory(dir=work_root): a unique probe dir
            // per header/target probe, removed after the record.
            const fs::path probe_dir =
                work_root / ("header-" + std::to_string(index) + "-" +
                             new_run_id());
            fs::create_directories(probe_dir, ec);
            const fs::path probe = probe_dir / "header_probe.cpp";
            {
                std::ofstream stream(probe, std::ios::binary | std::ios::trunc);
                stream << "#include \"" << header.generic_string()
                       << "\"\nint main() { return 0; }\n";
            }
            const std::vector<std::string> argv =
                replace_compile_source(*entry, probe, true);
            std::string name = padded_index(index);
            name += '-';
            name += header.stem().string();
            name += '-';
            name += target;
            Value record =
                run_process(argv, entry->directory, log_directory, name);
            std::error_code rm_ec;
            fs::remove_all(probe_dir, rm_ec);
            set_member(record, "pass", vstr("header"));
            set_member(record, "source", vstr(header.string()));
            set_member(record, "target", vstr(target));
            if (!record_health_ok(record)) tool_health_failed = true;
            if (!record_diagnostics(record).empty()) blocking_findings = true;
            records.push_back(std::move(record));
            ++index;
        }
    }

    set_member(summary, "records", varr(std::move(records)));
    set_member(summary, "tool_health",
               vstr(tool_health_failed ? "failed" : "ok"));
    set_member(summary, "finding_status",
               vstr(blocking_findings ? "blocking" : "none"));
    set_member(summary, "status",
               vstr(!tool_health_failed && !blocking_findings ? "passed"
                                                              : "failed"));
    set_member(summary, "log_directory", vstr(log_directory.string()));
    write_summary(build_dir, kind, summary);
    std::cout << "native headers selection: "
              << std::get<io::Number>(member(summary, "header_count")->data)
                     .lexeme
              << " headers, "
              << std::get<io::Number>(
                     member(summary, "header_translation_unit_count")->data)
                     .lexeme
              << " header translation units\n";
    std::cout << "native headers tool: " << compiler.string() << "\n";
    std::cout << "native headers tool health: "
              << std::get<std::string>(member(summary, "tool_health")->data)
              << "\n";
    std::cout << "native headers findings: "
              << std::get<std::string>(member(summary, "finding_status")->data)
              << "\n";
    std::cout << "native headers summary: "
              << (build_dir / "native_check_summaries" / "headers.json")
                     .string()
              << "\n";
    if (std::get<std::string>(member(summary, "status")->data) != "passed") {
        for (const Value &record:
             std::get<Array>(member(summary, "records")->data)) {
            for (const auto &line: record_diagnostics(record))
                std::cerr << line << "\n";
            if (!record_health_ok(record) ||
                !record_diagnostics(record).empty()) {
                const Value *source = member(record, "source");
                const Value *target = member(record, "target");
                const Value *rc = member(record, "returncode");
                std::cerr << "  "
                          << (source != nullptr ? std::get<std::string>(source->data)
                                                : std::string())
                          << " ["
                          << (target != nullptr ? std::get<std::string>(target->data)
                                                : std::string())
                          << "]: rc="
                          << (rc != nullptr && !io::is_null(*rc)
                                  ? std::get<io::Number>(rc->data).lexeme
                                  : std::string("None"))
                          << " stdout="
                          << std::get<std::string>(
                                 member(record, "stdout_log")->data)
                          << " stderr="
                          << std::get<std::string>(
                                 member(record, "stderr_log")->data)
                          << "\n";
            }
        }
        return 1;
    }
    return 0;
}

int run_scripts(const fs::path &build,
                const std::optional<fs::path> &tools_dir_override) {
    // tools/check_scripts.sh: find -L "$TOOLS_DIR" -maxdepth 2 -type f -name '*.sh' |
    // LC_ALL=C sort -z | xargs -0 shellcheck — one argv over the sorted
    // file set, exit status propagated. The build/context validation is the
    // same fail-closed contract the other sweeps share unless --tools-dir
    // explicitly selects the tree (the legacy script's positional arg).
    const fs::path build_dir = canonical_path(build);
    const std::string kind = "scripts";
    std::error_code ec;
    const std::string run_id = new_run_id();
    Value summary = vobj({
        {"schema_version", vint(1)},
        {"kind", vstr(kind)},
        {"build", vstr(build_dir.string())},
        {"run_id", vstr(run_id)},
        {"tool_health", vstr("failed")},
        {"finding_status", vstr("none")},
        {"status", vstr("failed")},
        {"records", varr({})},
        {"errors", varr({})},
    });
    fs::path tools_dir;
    fs::path shellcheck;
    try {
        if (tools_dir_override.has_value()) {
            tools_dir = canonical_path(*tools_dir_override);
        } else {
            if (!fs::is_directory(build_dir, ec))
                throw CheckError("build directory does not exist: " +
                                 build_dir.string());
            const Value context = load_context(build_dir);
            const auto &roots =
                std::get<Array>(member(context, "source_roots")->data);
            tools_dir = canonical_path(
                fs::path(std::get<std::string>(roots.front().data))
                    .parent_path() /
                "tools");
        }
        if (!fs::is_directory(tools_dir, ec))
            throw CheckError("tools directory does not exist: " +
                             tools_dir.string());
        const auto environment = environ_copy();
        std::string requested;
        if (const auto found = environment.find("SHELLCHECK");
            found != environment.end() && !found->second.empty())
            requested = found->second;
        if (requested.empty()) requested = "shellcheck";
        if (requested.find('/') != std::string::npos) {
            shellcheck = canonical_path(expand_home(requested));
            if (!is_executable_file(shellcheck))
                throw CheckError(
                    "required tool is missing or not executable: " + requested);
        } else {
            const std::optional<fs::path> located = which(requested);
            if (!located.has_value())
                throw CheckError("required tool is unavailable: " + requested +
                                 " (set SHELLCHECK)");
            shellcheck = canonical_path(*located);
        }
    } catch (const std::exception &error) {
        std::get<Array>(member_mut(summary, "errors")->data)
            .push_back(vstr(error.what()));
        write_summary(build_dir, kind, summary);
        std::cerr << "native scripts failed: " << error.what() << "\n";
        return 1;
    }

    std::vector<std::string> scripts;
    try {
        for (fs::directory_iterator it(tools_dir, ec), end; !ec && it != end;
             it.increment(ec)) {
            std::error_code file_ec;
            if (it->is_regular_file(file_ec) &&
                it->path().extension() == ".sh")
                scripts.push_back(it->path().string());
            if (file_ec) throw CheckError("cannot inspect scripts path: " + it->path().string());
            if (it->is_directory(file_ec)) {
                std::error_code child_ec;
                for (fs::directory_iterator child(it->path(), child_ec), child_end;
                     !child_ec && child != child_end; child.increment(child_ec)) {
                    std::error_code entry_ec;
                    if (child->is_regular_file(entry_ec) && child->path().extension() == ".sh")
                        scripts.push_back(child->path().string());
                    if (entry_ec)
                        throw CheckError("cannot inspect scripts path: " + child->path().string());
                }
                if (child_ec)
                    throw CheckError("cannot read scripts directory: " + it->path().string());
            }
            if (file_ec) throw CheckError("cannot inspect scripts directory: " + it->path().string());
        }
        if (ec) throw CheckError("cannot read tools directory: " + tools_dir.string());
        if (scripts.empty()) throw CheckError("native scripts selection: 0 scripts");
    } catch (const std::exception &error) {
        std::get<Array>(member_mut(summary, "errors")->data).push_back(vstr(error.what()));
        set_member(summary, "script_count", vint(static_cast<std::int64_t>(scripts.size())));
        write_summary(build_dir, kind, summary);
        std::cerr << "native scripts failed: " << error.what() << '\n';
        return 1;
    }
    // sort -z under LC_ALL=C semantics: byte order.
    std::ranges::sort(scripts);
    std::vector<std::string> argv = {shellcheck.string()};
    argv.insert(argv.end(), scripts.begin(), scripts.end());
    const fs::path log_directory =
        build_dir / "native_check_logs" / kind / run_id;
    Value record = run_process(argv, tools_dir, log_directory, "shellcheck");
    const bool healthy = record_health_ok(record);
    const std::vector<std::string> diagnostics = record_diagnostics(record);
    const Value *rc = member(record, "returncode");
    const std::string returncode_text =
        rc != nullptr && !io::is_null(*rc)
            ? std::get<io::Number>(rc->data).lexeme
            : std::string("None");
    set_member(record, "pass", vstr("shellcheck"));
    set_member(summary, "records", varr({std::move(record)}));
    set_member(summary, "tool", vstr(shellcheck.string()));
    Array script_values;
    for (const auto &script: scripts) script_values.push_back(vstr(script));
    set_member(summary, "scripts", varr(std::move(script_values)));
    set_member(summary, "script_count",
               vint(static_cast<std::int64_t>(scripts.size())));
    set_member(summary, "tool_health", vstr(healthy ? "ok" : "failed"));
    set_member(summary, "finding_status",
               vstr(diagnostics.empty() ? "none" : "blocking"));
    set_member(summary, "status",
               vstr(healthy && diagnostics.empty() ? "passed" : "failed"));
    set_member(summary, "log_directory", vstr(log_directory.string()));
    write_summary(build_dir, kind, summary);
    std::cout << "native scripts selection: " << scripts.size()
              << " scripts\n";
    std::cout << "native scripts tool: " << shellcheck.string() << "\n";
    std::cout << "native scripts tool health: "
              << std::get<std::string>(member(summary, "tool_health")->data)
              << "\n";
    std::cout << "native scripts summary: "
              << (build_dir / "native_check_summaries" / "scripts.json")
                     .string()
              << "\n";
    if (std::get<std::string>(member(summary, "status")->data) != "passed") {
        for (const auto &line: diagnostics) std::cerr << line << "\n";
        std::cerr << "  shellcheck: rc=" << returncode_text << "\n";
        return 1;
    }
    std::cout << "shellcheck clean: " << tools_dir.string() << "\n";
    return 0;
}

} // namespace bike::checks
