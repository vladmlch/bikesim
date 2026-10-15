// check_contracts.cpp — diagnostic controls, SDK/hardening context sweeps
// and the structural native contracts check (task T56). Ports
// tools/native_checks.py::run_diagnostic_controls / run_context_checks and
// tools/check_native_contracts.py.
//
// Standalone-build deviations from the oracle, all fail-closed and recorded
// in the emitted summaries:
//   * context-kind case configures use the produced standalone root project
//     (context["configure_source_dir"], emitted by Checks.cmake) instead of
//     `uv run cmake -S <repo>/native` — the produced sweep requires no
//     interpreter; a legacy context without that field still wraps the
//     scratch configure in `uv run cmake` exactly like the oracle.
//   * contract checks that only exist for a binding artifact (loader
//     provenance, selected-artifact test loaders, contract executable) are
//     reported "not-applicable" when the artifact class is absent from a
//     Python-free build instead of failing on an absent extension; a
//     bindings context (python_executable non-null) keeps the oracle's
//     fail-closed "exactly one extension" requirement.
#include "checks_internal.hpp"

#include "../platform/process.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

namespace bike::checks {
namespace {

// check_native_contracts._run_contract_case substrate: subprocess.run with a
// full replacement environment, stdout/stderr captured as text. Launch
// failures become ContractError exactly like the oracle's OSError path.
struct PlainResult {
    std::int64_t exit_code = 0;
    std::string out;
    std::string err;
};

PlainResult run_plain(const std::vector<std::string> &argv,
                      const fs::path &cwd,
                      const std::map<std::string, std::string> &environment) {
    std::vector<std::string> effective;
    effective = {which("env").value_or(fs::path("/usr/bin/env")).string(),
                 "-i"};
    for (const auto &[key, value]: environment) {
        std::string assignment = key;
        assignment += '=';
        assignment += value;
        effective.push_back(std::move(assignment));
    }
    effective.insert(effective.end(), argv.begin(), argv.end());
    try {
        const platform::ProcessResult result =
            platform::run(effective, cwd);
        PlainResult out;
        out.exit_code = result.exit_code > 128
                            ? static_cast<std::int64_t>(128 - result.exit_code)
                            : static_cast<std::int64_t>(result.exit_code);
        out.out = utf8_replace(result.out);
        out.err = utf8_replace(result.err);
        return out;
    } catch (const std::exception &error) {
        throw ContractError(std::string("cannot run native contract "
                                        "executable: ") +
                            error.what());
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (label, stdout, stderr) order mirrors the oracle writer
void write_contract_logs(const fs::path &build, const std::string &label,
                         const std::string &stdout_text,
                         const std::string &stderr_text) {
    static const std::regex unsafe_label("[^A-Za-z0-9_.-]+");
    const std::string safe_label =
        std::regex_replace(label, unsafe_label, "_");
    const fs::path log_directory = build / "native_check_logs" / "contracts";
    std::error_code ec;
    fs::create_directories(log_directory, ec);
    std::ofstream(log_directory / (safe_label + ".stdout.log"),
                  std::ios::binary | std::ios::trunc)
        << stdout_text;
    std::ofstream(log_directory / (safe_label + ".stderr.log"),
                  std::ios::binary | std::ios::trunc)
        << stderr_text;
}

} // namespace

// ---- diagnostic controls -----------------------------------------------------

namespace {

std::string record_stderr(const Value &record) {
    const Value *value = member(record, "stderr");
    return value != nullptr && std::holds_alternative<std::string>(value->data)
               ? std::get<std::string>(value->data)
               : std::string{};
}

} // namespace

int run_diagnostic_controls(const fs::path &build) {
    const fs::path build_dir = canonical_path(build);
    const std::string kind = "diagnostic-controls";
    std::error_code ec;
    if (!fs::is_directory(build_dir, ec)) {
        std::cerr << "native diagnostic controls failed: build directory does "
                     "not exist: "
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
        {"controls", varr({})},
        {"suppression_counts", vobj({})},
        {"errors", varr({})},
    });
    Value context;
    std::vector<CompileEntry> entries;
    std::vector<Suppression> suppressions;
    fs::path compiler;
    fs::path analyzer;
    try {
        context = load_context(build_dir);
        const SourceManifest manifest = load_source_manifest(build_dir);
        entries = load_entries(build_dir, manifest.sources);
        suppressions = load_suppressions(fs::path(std::get<std::string>(
            member(context, "analysis_suppressions_manifest")->data)));
        compiler = fs::path(std::get<std::string>(
            member(*member(context, "compiler"), "path")->data));
        analyzer = resolve_tool("analyzer", context, environ_copy()).first;
    } catch (const std::exception &error) {
        std::get<Array>(member_mut(summary, "errors")->data)
            .push_back(vstr(error.what()));
        write_summary(build_dir, kind, summary);
        std::cerr << "native diagnostic controls failed: " << error.what()
                  << "\n";
        return 1;
    }

    const fs::path version_logs =
        build_dir / "native_check_logs" / kind / run_id;
    const Value compiler_version = version_record(compiler, version_logs);
    const Value analyzer_version = version_record(analyzer, version_logs);
    set_member(summary, "compiler", vstr(compiler.string()));
    set_member(summary, "compiler_version",
               member(compiler_version, "version") != nullptr
                   ? *member(compiler_version, "version")
                   : vstr(""));
    set_member(summary, "analyzer", vstr(analyzer.string()));
    set_member(summary, "analyzer_version",
               member(analyzer_version, "version") != nullptr
                   ? *member(analyzer_version, "version")
                   : vstr(""));
    bool tool_health_failed = !record_health_ok(compiler_version) ||
                              !record_health_ok(analyzer_version);

    const fs::path control_root =
        build_dir / "native_check_work" / kind / run_id;
    fs::create_directories(control_root, ec);
    const fs::path log_directory =
        build_dir / "native_check_logs" / kind / run_id;
    const CompileEntry &entry = entries.front();

    Array controls;
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) (name, source_text) and (flags, markers) pairs mirror the oracle call convention
    const auto compile_control =
        [&](const std::string &name, const std::string &source_text,
            const std::vector<std::string> &flags,
            const std::vector<std::string> &markers) {
            // NOLINTEND(bugprone-easily-swappable-parameters)
            const fs::path source = control_root / (name + ".cpp");
            {
                std::ofstream stream(source,
                                     std::ios::binary | std::ios::trunc);
                stream << source_text;
            }
            std::vector<std::string> argv =
                replace_compile_source(entry, source, true);
            argv.insert(argv.end(), flags.begin(), flags.end());
            Value record =
                run_process(argv, entry.directory, log_directory, name);
            std::vector<std::string> matched;
            for (const auto &line: record_diagnostics(record))
                for (const auto &marker: markers)
                    if (line.find(marker) != std::string::npos) {
                        matched.push_back(line);
                        break;
                    }
            const Value *launch_error = member(record, "launch_error");
            const bool launch_ok =
                launch_error == nullptr || io::is_null(*launch_error);
            const Value *timed_out = member(record, "timed_out");
            const bool timeout_clear =
                timed_out == nullptr ||
                (std::holds_alternative<bool>(timed_out->data) &&
                 !std::get<bool>(timed_out->data));
            const std::optional<std::int64_t> rc = record_returncode(record);
            const bool control_status =
                rc.has_value() && *rc == 0 && launch_ok && timeout_clear &&
                !matched.empty();
            if (!launch_ok || !timeout_clear || !rc.has_value() || *rc != 0)
                tool_health_failed = true;
            Array expected_markers;
            for (const auto &marker: markers)
                expected_markers.push_back(vstr(marker));
            Array matched_values;
            for (const auto &line: matched)
                matched_values.push_back(vstr(line));
            Object control = {
                {"name", vstr(name)},
                {"status", vstr(control_status ? "passed" : "failed")},
                {"expected_markers", varr(std::move(expected_markers))},
                {"matched_diagnostics", varr(std::move(matched_values))},
            };
            Object record_members = std::get<Object>(record.data);
            control.insert(control.end(),
                           std::make_move_iterator(record_members.begin()),
                           std::make_move_iterator(record_members.end()));
            controls.push_back(vobj(std::move(control)));
        };

    compile_control(
        "discarded_nodiscard",
        "[[nodiscard]] int result() { return 1; }\nint main() { result(); "
        "return 0; }\n",
        {"-Wunused-result", "-Wno-error=unused-result"}, {"unused-result"});
    compile_control(
        "bitwise_instead_of_logical",
        "bool left() { return true; } bool right() { return false; }\n"
        "int main() { if (left() & right()) return 1; return 0; }\n",
        {"-Wbitwise-instead-of-logical", "-Wno-error=bitwise-instead-of-logical"},
        {"bitwise-instead-of-logical"});
    compile_control("extra_semicolon",
                    "struct Control { int value;; };\nint main() { return 0; "
                    "}\n",
                    {"-Wextra-semi", "-Wno-error=extra-semi"}, {"extra-semi"});
    compile_control(
        "signed_bounds",
        "#include <vector>\nint main() { std::vector<int> values(2); int "
        "index = 0; return values[index]; }\n",
        {"-Wsign-conversion", "-Wno-error=sign-conversion"},
        {"sign-conversion", "signedness"});

    // cplusplus.Move positive control: alpha-adjacent checker kept stable by
    // the dedicated checker flag, with suppression filtering active.
    const fs::path move_source =
        control_root / "first_party_use_after_move.cpp";
    {
        std::ofstream stream(move_source, std::ios::binary | std::ios::trunc);
        stream << "#include <string>\n#include <utility>\n"
                  "void check_move() { std::string source{\"value\"}; "
                  "std::string destination{std::move(source)}; "
                  "(void)source.size(); }\n";
    }
    std::vector<std::string> move_argv =
        replace_compile_source(entry, move_source, false);
    if (!move_argv.empty()) move_argv[0] = analyzer.string();
    move_argv.insert(move_argv.end(),
                     {"--analyze", "-Xanalyzer", "-analyzer-output=text",
                      "-Xanalyzer", "-analyzer-checker=cplusplus.Move",
                      "-Wno-error"});
    Value move_record = run_process(move_argv, entry.directory, log_directory,
                                    "first_party_use_after_move");
    const std::string analyzer_version_text =
        member(analyzer_version, "version") != nullptr
            ? std::get<std::string>(member(analyzer_version, "version")->data)
            : std::string{};
    const auto filtered = apply_suppressions(
        record_diagnostics(move_record), analyzer_version_text, suppressions);
    std::vector<std::string> move_matched;
    for (const auto &line: filtered.remaining)
        if (line.find("cplusplus.Move") != std::string::npos)
            move_matched.push_back(line);
    for (const Value &item: filtered.matched) {
        const std::string checker = std::get<std::string>(
            member(item, "checker")->data);
        Value *counts = member_mut(summary, "suppression_counts");
        auto &counts_object = std::get<Object>(counts->data);
        bool found = false;
        for (auto &[key, value]: counts_object) {
            if (key == checker) {
                value = vint(parse_int64(
                                 std::get<io::Number>(value.data).lexeme)
                                 .value_or(0) +
                             1);
                found = true;
            }
        }
        if (!found) counts_object.emplace_back(checker, vint(1));
    }
    {
        const Value *launch_error = member(move_record, "launch_error");
        const bool launch_ok =
            launch_error == nullptr || io::is_null(*launch_error);
        const Value *timed_out = member(move_record, "timed_out");
        const bool timeout_clear =
            timed_out == nullptr ||
            (std::holds_alternative<bool>(timed_out->data) &&
             !std::get<bool>(timed_out->data));
        const std::optional<std::int64_t> rc = record_returncode(move_record);
        const bool move_status = rc.has_value() && *rc == 0 && launch_ok &&
                                 timeout_clear && !move_matched.empty();
        if (!launch_ok || !timeout_clear || !rc.has_value() || *rc != 0)
            tool_health_failed = true;
        Array matched_values;
        for (const auto &line: move_matched)
            matched_values.push_back(vstr(line));
        Object control = {
            {"name", vstr("first_party_use_after_move")},
            {"status", vstr(move_status ? "passed" : "failed")},
            {"expected_markers", varr({vstr("cplusplus.Move")})},
            {"matched_diagnostics", varr(std::move(matched_values))},
            {"suppressed_diagnostics", varr(filtered.matched)},
        };
        Object record_members = std::get<Object>(move_record.data);
        control.insert(control.end(),
                       std::make_move_iterator(record_members.begin()),
                       std::make_move_iterator(record_members.end()));
        controls.push_back(vobj(std::move(control)));
    }

    // The hardening-mode guard control must REJECT a libc++ configured with
    // no selected mode: nonzero rc plus the expected #error text on stderr.
    const fs::path fake_libcpp_include = control_root / "fake-libcpp";
    fs::create_directories(fake_libcpp_include, ec);
    {
        std::ofstream stream(fake_libcpp_include / "__config",
                             std::ios::binary | std::ios::trunc);
        stream << "#define _LIBCPP_VERSION 1\n";
    }
    const fs::path guard_source = control_root / "libcpp_hardening_guard.cpp";
    {
        std::ofstream stream(guard_source, std::ios::binary | std::ios::trunc);
        stream << "#include <__config>\n"
                  "#ifndef _LIBCPP_VERSION\n#error not libc++\n#endif\n"
                  "#ifndef _LIBCPP_HARDENING_MODE_EXTENSIVE\n#error missing "
                  "hardening mode macro\n#endif\n"
                  "#ifndef _LIBCPP_HARDENING_MODE\n#error missing hardening "
                  "selection macro\n#endif\n"
                  "int main() { return 0; }\n";
    }
    Value guard_record = run_process(
        {compiler.string(), "-std=c++23", "-I", fake_libcpp_include.string(),
         "-fsyntax-only", guard_source.string()},
        entry.directory, log_directory, "libcpp_probe_rejects_missing_mode");
    {
        const std::string guard_stderr = record_stderr(guard_record);
        const bool guard_error =
            guard_stderr.find("missing hardening mode macro") !=
            std::string::npos;
        const Value *launch_error = member(guard_record, "launch_error");
        const bool launch_ok =
            launch_error == nullptr || io::is_null(*launch_error);
        const Value *timed_out = member(guard_record, "timed_out");
        const bool timeout_clear =
            timed_out == nullptr ||
            (std::holds_alternative<bool>(timed_out->data) &&
             !std::get<bool>(timed_out->data));
        const std::optional<std::int64_t> rc =
            record_returncode(guard_record);
        const bool guard_status =
            rc.has_value() && *rc != 0 && guard_error && launch_ok;
        if (!launch_ok || !timeout_clear || !guard_status)
            tool_health_failed = true;
        Array matched;
        if (guard_error) matched.push_back(vstr(guard_stderr));
        Object control = {
            {"name", vstr("libcpp_probe_rejects_missing_mode")},
            {"status", vstr(guard_status ? "passed" : "failed")},
            {"expected_markers", varr({vstr("missing hardening mode macro")})},
            {"matched_diagnostics", varr(std::move(matched))},
            {"expected_nonzero", vbool(true)},
        };
        Object record_members = std::get<Object>(guard_record.data);
        control.insert(control.end(),
                       std::make_move_iterator(record_members.begin()),
                       std::make_move_iterator(record_members.end()));
        controls.push_back(vobj(std::move(control)));
    }

    set_member(summary, "controls", varr(controls));
    const Value &controls_value = *member(summary, "controls");
    std::vector<std::string> failures;
    for (const Value &control: std::get<Array>(controls_value.data)) {
        if (std::get<std::string>(member(control, "status")->data) != "passed")
            failures.push_back(
                std::get<std::string>(member(control, "name")->data));
    }
    set_member(summary, "tool_health",
               vstr(tool_health_failed ? "failed" : "ok"));
    set_member(summary, "status",
               vstr(failures.empty() && !tool_health_failed ? "passed"
                                                            : "failed"));
    set_member(summary, "log_directory", vstr(log_directory.string()));
    write_summary(build_dir, kind, summary);
    std::cout << "native diagnostic controls: "
              << std::get<Array>(controls_value.data).size() << " controls, "
              << failures.size() << " failed\n";
    std::cout << "native diagnostic control tool health: "
              << std::get<std::string>(member(summary, "tool_health")->data)
              << "\n";
    std::cout << "native diagnostic controls summary: "
              << (build_dir / "native_check_summaries" /
                  "diagnostic-controls.json")
                     .string()
              << "\n";
    if (!failures.empty()) {
        for (const Value &control: std::get<Array>(controls_value.data)) {
            if (std::get<std::string>(member(control, "status")->data) !=
                "passed") {
                const Value *rc = member(control, "returncode");
                std::cerr << "  "
                          << std::get<std::string>(
                                 member(control, "name")->data)
                          << ": rc="
                          << (rc != nullptr && !io::is_null(*rc)
                                  ? std::get<io::Number>(rc->data).lexeme
                                  : std::string("None"))
                          << " stdout="
                          << std::get<std::string>(
                                 member(control, "stdout_log")->data)
                          << " stderr="
                          << std::get<std::string>(
                                 member(control, "stderr_log")->data)
                          << "\n";
            }
        }
        return 1;
    }
    return 0;
}

// ---- context checks -----------------------------------------------------------

namespace {

std::string_view signal_name(std::int64_t signal_number) {
    switch (signal_number) {
        case 1: return "SIGHUP";
        case 2: return "SIGINT";
        case 3: return "SIGQUIT";
        case 4: return "SIGILL";
        case 5: return "SIGTRAP";
        case 6: return "SIGABRT";
        case 7: return "SIGBUS";
        case 8: return "SIGFPE";
        case 9: return "SIGKILL";
        case 10: return "SIGUSR1";
        case 11: return "SIGSEGV";
        case 12: return "SIGUSR2";
        case 13: return "SIGPIPE";
        case 14: return "SIGALRM";
        case 15: return "SIGTERM";
        default: return {};
    }
}

std::int64_t median_of(std::vector<std::int64_t> samples) {
    std::ranges::sort(samples);
    const std::size_t count = samples.size();
    if (count % 2 == 1) return samples[count / 2];
    // statistics.median averages the two middle values; int() truncates.
    return (samples[count / 2 - 1] + samples[count / 2]) / 2;
}

} // namespace

int run_context_checks(const fs::path &build) {
    const fs::path build_dir = canonical_path(build);
    const std::string kind = "context";
    std::error_code ec;
    if (!fs::is_directory(build_dir, ec)) {
        std::cerr << "native context checks failed: build directory does not "
                     "exist: "
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
        {"sdk_controls", vobj({})},
        {"hardening_evidence", varr({})},
        {"records", varr({})},
        {"errors", varr({})},
    });
    Value context;
    std::vector<CompileEntry> entries;
    fs::path cmake;
    fs::path compiler;
    std::optional<fs::path> uv;
    try {
        context = load_context(build_dir);
        const SourceManifest manifest = load_source_manifest(build_dir);
        entries = load_entries(build_dir, manifest.sources);
        // The produced context sweep configures the standalone root project
        // with the same CMake binary that configured this build (recorded by
        // Checks.cmake); a legacy context configures <repo>/native via the
        // `uv`-provided cmake — both fail closed when the tool is absent.
        const auto recorded_cmake = opt_string(context, "cmake");
        if (recorded_cmake.has_value() && !recorded_cmake->empty()) {
            cmake = canonical_path(expand_home(*recorded_cmake));
            if (!is_executable_file(cmake))
                throw CheckError("required tool is unavailable: " +
                                 *recorded_cmake + " (context 'cmake')");
        } else {
            const std::optional<fs::path> located = which("cmake");
            if (!located.has_value())
                throw CheckError("required tool is unavailable: cmake");
            cmake = canonical_path(*located);
        }
        // Legacy contexts (no produced dependency fields) configure the
        // bindings project through `uv run cmake`, exactly like the Python
        // oracle, so the venv interpreter and `uv`-provided cmake are on the
        // child environment; resolve `uv` eagerly to fail closed.
        if (!opt_string(context, "dependency_root").has_value()) {
            const std::optional<fs::path> located = which("uv");
            if (!located.has_value())
                throw CheckError("required tool is unavailable: uv");
            uv = canonical_path(*located);
        }
        compiler = fs::path(std::get<std::string>(
            member(*member(context, "compiler"), "path")->data));
    } catch (const std::exception &error) {
        std::get<Array>(member_mut(summary, "errors")->data)
            .push_back(vstr(error.what()));
        write_summary(build_dir, kind, summary);
        std::cerr << "native context checks failed: " << error.what() << "\n";
        return 1;
    }

    const fs::path work_root = build_dir / "native_check_work" / kind / run_id;
    fs::create_directories(work_root, ec);
    const fs::path log_directory =
        build_dir / "native_check_logs" / kind / run_id;
    const auto &roots =
        std::get<Array>(member(context, "source_roots")->data);
    const fs::path repository =
        fs::path(std::get<std::string>(roots.front().data)).parent_path();
    // Produced contexts name the configured project dir explicitly (the repo
    // root); legacy contexts fall back to the binding project directory.
    const auto recorded_source_dir = opt_string(context, "configure_source_dir");
    const fs::path project_source =
        recorded_source_dir.has_value()
            ? fs::path(*recorded_source_dir)
            : fs::path(std::get<std::string>(roots.front().data));

    Array records;
    std::vector<Value> cmake_records;

    const auto configure_case =
        [&](const std::string &name, const std::vector<std::string> &options,
            const std::map<std::string, std::string> *environment,
            const std::optional<fs::path> &configured_compiler,
            bool use_default_compiler,
            std::pair<Value, fs::path> &out) {
            const fs::path case_build = work_root / ("cmake-" + name);
            // Legacy parity: `uv run cmake -S <native> -B <build>` so the
            // scratch configure sees the project venv. Produced contexts
            // invoke the recorded cmake directly — no interpreter needed.
            std::vector<std::string> argv;
            if (uv.has_value())
                argv = {uv->string(), "run", "cmake", "-S",
                        project_source.string(), "-B", case_build.string()};
            else
                argv = {cmake.string(), "-S", project_source.string(), "-B",
                        case_build.string()};
            if (!use_default_compiler)
                argv.push_back("-DCMAKE_CXX_COMPILER=" +
                               configured_compiler.value_or(compiler).string());
            if (const auto dependency_root =
                    opt_string(context, "dependency_root");
                dependency_root.has_value()) {
                argv.emplace_back("-DBIKE_BUILD_PYTHON_BINDINGS=OFF");
                argv.push_back("-DBIKE_DEPENDENCY_ROOT=" + *dependency_root);
                if (const auto dependency_lock =
                        opt_string(context, "dependency_lock"))
                    argv.push_back("-DBIKE_DEPENDENCY_LOCK=" + *dependency_lock);
            }
            argv.insert(argv.end(), options.begin(), options.end());
            Value record = run_process(argv, repository, log_directory,
                                       "cmake-" + name, environment, false,
                                       300);
            set_member(record, "control", vstr(name));
            out = {record, case_build};
        };

    const auto configured_context =
        [&](const fs::path &case_build) -> Value {
        Value child_context = load_context(case_build);
        const SourceManifest child_manifest = load_source_manifest(case_build);
        const auto selected =
            load_entries(case_build, child_manifest.sources);
        if (selected.empty())
            throw CheckError("configured context selected no translation "
                             "units: " +
                             case_build.string());
        return child_context;
    };

    // Ordered control map — Python dict insertion order is part of the
    // printed "native context SDK controls" line.
    std::vector<std::pair<std::string, std::string>> sdk_controls;
    std::map<std::string, Value> sdk_details;
    const std::string sdk_request =
        opt_string(context, "sdk_requested").value_or("");

    {
        std::pair<Value, fs::path> result;
        configure_case("sdk-explicit",
                       {"-DCMAKE_OSX_SYSROOT=" + sdk_request}, nullptr,
                       std::nullopt, false, result);
        try {
            const Value explicit_context = configured_context(result.second);
            const auto rc = record_returncode(result.first);
            const bool explicit_ok =
                rc.has_value() && *rc == 0 &&
                opt_string(explicit_context, "sdk_source") == "explicit" &&
                canonical_path(std::get<std::string>(
                              member(explicit_context, "sdk")->data)) ==
                    canonical_path(std::get<std::string>(
                                  member(context, "sdk")->data)) &&
                canonical_path(std::get<std::string>(
                              member(*member(explicit_context, "compiler"),
                                     "path")
                                  ->data)) == canonical_path(compiler);
            sdk_controls.emplace_back("explicit_sdk",
                                      explicit_ok ? "passed" : "failed");
            sdk_details["explicit_sdk"] = vobj({
                {"requested", vstr(sdk_request)},
                {"selected", *member(explicit_context, "sdk")},
                {"compiler",
                 *member(*member(explicit_context, "compiler"), "path")},
            });
        } catch (const std::exception &error) {
            sdk_controls.emplace_back("explicit_sdk", "failed");
            sdk_details["explicit_sdk"] =
                vobj({{"error", vstr(error.what())}});
        }
        cmake_records.push_back(result.first);
    }

    {
        const fs::path missing_sdk = work_root / "missing-explicit-sdk";
        std::pair<Value, fs::path> result;
        configure_case("sdk-missing",
                       {"-DCMAKE_OSX_SYSROOT=" + missing_sdk.string()},
                       nullptr, std::nullopt, false, result);
        const std::string combined =
            record_stdout_text(result.first) + record_stderr(result.first);
        const auto rc = record_returncode(result.first);
        const bool missing_ok = rc.has_value() && *rc != 0 &&
                                combined.find(missing_sdk.string()) !=
                                    std::string::npos;
        sdk_controls.emplace_back("missing_sdk",
                                  missing_ok ? "rejected" : "failed");
        sdk_details["missing_sdk"] = vobj({
            {"requested", vstr(missing_sdk.string())},
            {"returncode",
             rc.has_value() ? vint(*rc) : vnull()},
        });
        cmake_records.push_back(result.first);
    }

    auto default_environment = environ_copy();
    default_environment.erase("SDKROOT");
    default_environment.erase("CMAKE_OSX_SYSROOT");
    {
        std::pair<Value, fs::path> result;
        configure_case("sdk-default", {}, &default_environment, std::nullopt,
                       true, result);
        try {
            const Value default_context = configured_context(result.second);
            const auto rc = record_returncode(result.first);
            const fs::path selected_sdk = fs::path(std::get<std::string>(
                member(default_context, "sdk")->data));
            std::error_code dir_ec;
            const bool default_ok =
                rc.has_value() && *rc == 0 &&
                opt_string(default_context, "sdk_source") == "default" &&
                fs::is_directory(selected_sdk, dir_ec);
            sdk_controls.emplace_back("default_sdk",
                                      default_ok ? "passed" : "failed");
            sdk_details["default_sdk"] = vobj({
                {"selected", *member(default_context, "sdk")},
                {"compiler",
                 *member(*member(default_context, "compiler"), "path")},
            });
        } catch (const std::exception &error) {
            sdk_controls.emplace_back("default_sdk", "failed");
            sdk_details["default_sdk"] =
                vobj({{"error", vstr(error.what())}});
        }
        cmake_records.push_back(result.first);
    }

    {
        const fs::path rejected_default_sdk =
            work_root / "incompatible-default-sdk";
        fs::create_directories(rejected_default_sdk, ec);
        const fs::path fake_bin = work_root / "fake-tool-bin";
        fs::create_directories(fake_bin, ec);
        const fs::path fake_xcrun = fake_bin / "xcrun";
        {
            std::ofstream stream(fake_xcrun,
                                 std::ios::binary | std::ios::trunc);
            stream << "#!/bin/sh\n"
                      "if [ \"${1:-}\" = \"--sdk\" ] && [ \"${2:-}\" = "
                      "\"macosx\" ] && [ \"${3:-}\" = \"--show-sdk-path\" ]; "
                      "then\n"
                      "  printf \"%s\\n\" \"$BIKE_TEST_BAD_SDK\"; exit 0;\n"
                      "fi\n"
                      "exec /usr/bin/xcrun \"$@\"\n";
        }
        fs::permissions(fake_xcrun,
                        fs::perms::owner_all | fs::perms::group_read |
                            fs::perms::group_exec | fs::perms::others_read |
                            fs::perms::others_exec,
                        ec);
        const fs::path reject_marker =
            work_root / "link-incompatibility-seen.txt";
        const fs::path compiler_wrapper =
            work_root / "compiler-link-control.sh";
        {
            std::ofstream stream(compiler_wrapper,
                                 std::ios::binary | std::ios::trunc);
            stream
                << "#!/bin/bash\n"
                   "args=(\"$@\")\n"
                   "selected_sdk=\"\"\n"
                   "syntax_only=0\n"
                   "for ((i=0; i<${#args[@]}; ++i)); do\n"
                   "  if [[ \"${args[i]}\" == \"-isysroot\" && $((i+1)) -lt "
                   "${#args[@]} ]]; then selected_sdk=\"${args[i+1]}\"; fi\n"
                   "  if [[ \"${args[i]}\" == \"-fsyntax-only\" ]]; then "
                   "syntax_only=1; fi\n"
                   "done\n"
                   "if [[ \"$selected_sdk\" == \"$BIKE_TEST_BAD_SDK\" && "
                   "$syntax_only == 0 ]]; then\n"
                   "  printf \"%s\\n\" \"link-rejected $selected_sdk\" >> "
                   "\"$BIKE_TEST_REJECT_MARKER\"\n"
                   "  printf \"%s\\n\" \"synthetic linker incompatibility for "
                   "$selected_sdk\" >&2\n"
                   "  exit 1\n"
                   "fi\n"
                   "exec /usr/bin/c++ \"${args[@]}\"\n";
        }
        fs::permissions(compiler_wrapper,
                        fs::perms::owner_all | fs::perms::group_read |
                            fs::perms::group_exec | fs::perms::others_read |
                            fs::perms::others_exec,
                        ec);
        auto fallback_environment = default_environment;
        fallback_environment["PATH"] =
            fake_bin.string() + ":" +
            (fallback_environment.count("PATH") ? fallback_environment["PATH"]
                                                : std::string{});
        fallback_environment["BIKE_TEST_BAD_SDK"] =
            rejected_default_sdk.string();
        fallback_environment["BIKE_TEST_REJECT_MARKER"] =
            reject_marker.string();
        std::pair<Value, fs::path> result;
        configure_case("sdk-default-link-fallback", {}, &fallback_environment,
                       compiler_wrapper, false, result);
        try {
            const Value fallback_context = configured_context(result.second);
            const auto rc = record_returncode(result.first);
            std::error_code marker_ec;
            const bool marker_exists = fs::is_regular_file(reject_marker, marker_ec);
            std::string marker_text;
            if (marker_exists) {
                std::ostringstream content;
                content << std::ifstream(reject_marker).rdbuf();
                marker_text = content.str();
            }
            const bool fallback_ok =
                rc.has_value() && *rc == 0 &&
                opt_string(fallback_context, "sdk_source") == "default" &&
                canonical_path(std::get<std::string>(
                              member(fallback_context, "sdk")->data)) !=
                    canonical_path(rejected_default_sdk) &&
                marker_exists &&
                marker_text.find("link-rejected") != std::string::npos;
            sdk_controls.emplace_back("default_sdk_fallback",
                                      fallback_ok ? "passed" : "failed");
            sdk_details["default_sdk_fallback"] = vobj({
                {"rejected", vstr(rejected_default_sdk.string())},
                {"selected", *member(fallback_context, "sdk")},
                {"compiler_wrapper", vstr(compiler_wrapper.string())},
                {"link_rejection_observed", vbool(marker_exists)},
            });
        } catch (const std::exception &error) {
            sdk_controls.emplace_back("default_sdk_fallback", "failed");
            sdk_details["default_sdk_fallback"] =
                vobj({{"error", vstr(error.what())}});
        }
        cmake_records.push_back(result.first);
    }

    Array hardening_evidence;
    const Value *hardening = member(context, "libcpp_hardening");
    const bool hardening_supported =
        member(*hardening, "supported") != nullptr &&
        std::holds_alternative<bool>(
            member(*hardening, "supported")->data) &&
        std::get<bool>(member(*hardening, "supported")->data);
    if (hardening_supported) {
        const fs::path hardening_source = work_root / "hardening_probe.cpp";
        {
            std::ofstream stream(hardening_source,
                                 std::ios::binary | std::ios::trunc);
            stream
                << "#include <__config>\n#include <chrono>\n#include "
                   "<cstdint>\n"
                   "#include <cstdio>\n#include <optional>\n#include "
                   "<string>\n#include <vector>\n"
                   "#if !defined(__STRICT_ANSI__) || __cplusplus < 202302L\n"
                   "#error Native first-party code must compile as strict "
                   "C++23\n#endif\n"
                   "#if _LIBCPP_HARDENING_MODE == "
                   "_LIBCPP_HARDENING_MODE_EXTENSIVE\n"
                   "constexpr int selected_mode = 1;\n"
                   "#elif _LIBCPP_HARDENING_MODE == "
                   "_LIBCPP_HARDENING_MODE_FAST\n"
                   "constexpr int selected_mode = 2;\n"
                   "#else\n#error Unsupported hardening probe mode\n#endif\n"
                   "[[clang::noinline]] std::uint64_t element(const "
                   "std::vector<std::uint64_t>& values, std::size_t index) { "
                   "return values[index]; }\n"
                   "int main(int argc, char**) {\n"
                   "  std::vector<std::uint64_t> values(1024);\n"
                   "  for (std::size_t i = 0; i < values.size(); ++i) "
                   "values[i] = i * 17u + 3u;\n"
                   "  if (argc > 1) { volatile std::uint64_t invalid = "
                   "values[values.size()]; return static_cast<int>(invalid); "
                   "}\n"
                   "  std::uint64_t checksum = 0;\n"
                   "  const auto start = std::chrono::steady_clock::now();\n"
                   "  for (std::uint64_t i = 0; i < 3000000; ++i) checksum += "
                   "element(values, static_cast<std::size_t>(i & 1023u));\n"
                   "  const auto end = std::chrono::steady_clock::now();\n"
                   "  const auto elapsed = "
                   "std::chrono::duration_cast<std::chrono::nanoseconds>(end - "
                   "start).count();\n"
                   "  std::printf(\"%d %zu %zu %zu %lld %llu\\n\", "
                   "selected_mode, sizeof(values), sizeof(std::string), "
                   "sizeof(std::optional<int>), static_cast<long "
                   "long>(elapsed), static_cast<unsigned long "
                   "long>(checksum));\n"
                   "  return 0;\n}\n";
        }
        std::vector<std::string> arch_args;
        {
            const auto &configured_args = entries.front().arguments;
            for (std::size_t position = 0; position < configured_args.size();
                 ++position) {
                if (configured_args[position] == "-arch" &&
                    position + 1 < configured_args.size()) {
                    arch_args.emplace_back("-arch");
                    arch_args.push_back(configured_args[position + 1]);
                    ++position;
                }
            }
        }
        // Python dict literal order for each evidence row.
        const std::vector<std::pair<std::string, std::vector<std::string>>>
            build_type_options = {
                {"Debug", {"-O0", "-g"}},
                {"Release", {"-O3", "-DNDEBUG"}},
                {"default", {}},
            };
        const fs::path &control_logs = log_directory;
        const fs::path control_work = work_root / "hardening-binaries";
        fs::create_directories(control_work, ec);
        bool hardening_failed = false;
        for (const auto &[build_type, optimization]: build_type_options) {
            Object row = {
                {"build_type", vstr(build_type)},
                {"sizeof", vobj({})},
                {"timings_ns", vobj({})},
                {"median_ns", vobj({})},
                {"assertion_active", vobj({})},
            };
            Array optimization_values;
            for (const auto &flag: optimization)
                optimization_values.push_back(vstr(flag));
            row.emplace_back("optimization_flags",
                             varr(std::move(optimization_values)));
            const auto nested = [&row](const std::string &key) -> Object & {
                for (auto &[name, value]: row)
                    if (name == key) return std::get<Object>(value.data);
                throw CheckError("context evidence row missing " + key);
            };
            const std::string lower_type = [&]() {
                std::string out = build_type;
                std::ranges::transform(out, out.begin(),
                               [](unsigned char ch) {
                                   return static_cast<char>(std::tolower(ch));
                               });
                return out;
            }();
            for (const std::string_view mode: {"EXTENSIVE", "FAST"}) {
                const std::string lower_mode = [&]() {
                    std::string lowered(mode);
                    std::ranges::transform(lowered,
                                   lowered.begin(), [](unsigned char ch) {
                                       return static_cast<char>(
                                           std::tolower(ch));
                                   });
                    return lowered;
                }();
                std::string control_stem = lower_type;
                control_stem += '-';
                control_stem += lower_mode;
                std::string control_name = "hardening-";
                control_name += control_stem;
                const fs::path binary = control_work / control_stem;
                std::vector<std::string> compile_argv = {
                    compiler.string(), "-std=c++23",
                };
                compile_argv.insert(compile_argv.end(), arch_args.begin(),
                                    arch_args.end());
                compile_argv.insert(
                    compile_argv.end(),
                    {"-isysroot",
                     std::get<std::string>(member(context, "sdk")->data),
                     "-stdlib=libc++",
                     "-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_" +
                         std::string(mode)});
                compile_argv.insert(compile_argv.end(), optimization.begin(),
                                    optimization.end());
                compile_argv.push_back(hardening_source.string());
                compile_argv.emplace_back("-o");
                compile_argv.push_back(binary.string());
                Value compile_record = run_process(
                    compile_argv, repository, control_logs,
                    control_name + "-compile", nullptr, false, 180);
                const bool compile_ok = record_health_ok(compile_record);
                records.push_back(std::move(compile_record));
                if (!compile_ok) {
                    hardening_failed = true;
                    continue;
                }
                Value assertion_record = run_process(
                    {binary.string(), "invalid-element-control"}, repository,
                    control_logs, control_name + "-assertion", nullptr, true,
                    30);
                const std::optional<std::int64_t> assertion_rc =
                    record_returncode(assertion_record);
                const bool assertion_active =
                    assertion_rc.has_value() && *assertion_rc == -5;
                const std::int64_t signalled =
                    assertion_rc.has_value() && *assertion_rc < 0
                        ? -*assertion_rc
                        : 0;
                {
                    Object extra = std::get<Object>(assertion_record.data);
                    extra.emplace_back("control",
                                       vstr(control_name + "-assertion"));
                    extra.emplace_back("assertion_active",
                                       vbool(assertion_active));
                    extra.emplace_back(
                        "assertion_category",
                        vstr("libc++ std::vector element access"));
                    records.push_back(vobj(std::move(extra)));
                }
                Object &assertion_active_obj = nested("assertion_active");
                assertion_active_obj.emplace_back(std::string(mode),
                                                  vbool(assertion_active));
                // row["assertion_evidence"][mode] via setdefault.
                Object *evidence_obj = nullptr;
                for (auto &[name, value]: row)
                    if (name == "assertion_evidence")
                        evidence_obj = &std::get<Object>(value.data);
                if (evidence_obj == nullptr) {
                    row.emplace_back("assertion_evidence", vobj({}));
                    evidence_obj = &std::get<Object>(row.back().second.data);
                }
                evidence_obj->emplace_back(
                    std::string(mode),
                    vobj({
                        {"returncode",
                         assertion_rc.has_value() ? vint(*assertion_rc)
                                                  : vnull()},
                        {"signal", signalled != 0
                                       ? vstr(std::string(signal_name(
                                             signalled)))
                                       : vnull()},
                        {"category",
                         vstr("libc++ std::vector element access")},
                        {"stderr_log",
                         member(assertion_record, "stderr_log") != nullptr
                             ? *member(assertion_record, "stderr_log")
                             : vnull()},
                    }));
                if (mode == "EXTENSIVE" && !assertion_active)
                    hardening_failed = true;
                for (int run_index = 0; run_index < 6; ++run_index) {
                    std::string run_name = control_name;
                    run_name += "-run-";
                    run_name += std::to_string(run_index);
                    Value timing_record = run_process(
                        {binary.string()}, repository, control_logs, run_name,
                        nullptr, false, 30);
                    const bool timing_ok = record_health_ok(timing_record);
                    const std::string timing_stdout = record_stdout_text(
                        timing_record);
                    records.push_back(std::move(timing_record));
                    if (!timing_ok) {
                        hardening_failed = true;
                        continue;
                    }
                    // stdout.strip().split() — whitespace-token fields.
                    std::vector<std::string> fields;
                    {
                        std::istringstream tokens(timing_stdout);
                        std::string field;
                        while (tokens >> field) fields.push_back(field);
                    }
                    if (fields.size() != 6) {
                        hardening_failed = true;
                        continue;
                    }
                    std::int64_t selected_mode = 0;
                    std::int64_t size_vector = 0;
                    std::int64_t size_string = 0;
                    std::int64_t size_optional = 0;
                    std::int64_t elapsed_ns = 0;
                    try {
                        selected_mode = std::stoll(fields[0]);
                        size_vector = std::stoll(fields[1]);
                        size_string = std::stoll(fields[2]);
                        size_optional = std::stoll(fields[3]);
                        elapsed_ns = std::stoll(fields[4]);
                        [[maybe_unused]] const std::int64_t checksum =
                            std::stoll(fields[5]);
                    } catch (const std::exception &) {
                        hardening_failed = true;
                        continue;
                    }
                    const std::int64_t expected_mode =
                        mode == "EXTENSIVE" ? 1 : 2;
                    if (selected_mode != expected_mode) hardening_failed = true;
                    nested("sizeof").emplace_back(
                        mode,
                        vobj({
                            {"vector_uint64", vint(size_vector)},
                            {"string", vint(size_string)},
                            {"optional_int", vint(size_optional)},
                        }));
                    if (run_index > 0) {
                        Object &timings = nested("timings_ns");
                        Array *samples = nullptr;
                        for (auto &[name, value]: timings)
                            if (name == mode)
                                samples = &std::get<Array>(value.data);
                        if (samples == nullptr) {
                            timings.emplace_back(mode, varr({}));
                            samples =
                                &std::get<Array>(timings.back().second.data);
                        }
                        samples->push_back(vint(elapsed_ns));
                    }
                }
                // row["median_ns"][mode] over collected timing samples.
                Object &timings = nested("timings_ns");
                for (auto &[name, value]: timings) {
                    if (name != mode) continue;
                    std::vector<std::int64_t> samples;
                    for (const Value &item: std::get<Array>(value.data))
                        samples.push_back(
                            parse_int64(std::get<io::Number>(item.data)
                                            .lexeme)
                                .value_or(0));
                    if (!samples.empty())
                        nested("median_ns")
                            .emplace_back(mode, vint(median_of(samples)));
                }
            }
            // sizeof equality between modes is required evidence.
            const Object &sizeofs = nested("sizeof");
            const Value *extensive = nullptr;
            const Value *fast = nullptr;
            for (const auto &[name, value]: sizeofs) {
                if (name == "EXTENSIVE") extensive = &value;
                if (name == "FAST") fast = &value;
            }
            const bool sizes_match =
                extensive != nullptr && fast != nullptr &&
                json_equal(*extensive, *fast);
            if (!sizes_match) hardening_failed = true;
            hardening_evidence.push_back(vobj(std::move(row)));
        }
        sdk_controls.emplace_back("libcpp_hardening",
                                  hardening_failed ? "failed" : "passed");
    } else {
        sdk_controls.emplace_back("libcpp_hardening", "not-applicable");
        set_member(summary, "hardening_limitation",
                   vstr("The selected compiler does not use libc++; native "
                        "libc++ hardening modes cannot be measured."));
    }

    Object sdk_controls_object;
    for (const auto &[name, status]: sdk_controls)
        sdk_controls_object.emplace_back(name, vstr(status));
    set_member(summary, "sdk_controls", vobj(std::move(sdk_controls_object)));
    Object sdk_details_object;
    for (const auto &[name, detail]: sdk_details)
        sdk_details_object.emplace_back(name, detail);
    set_member(summary, "sdk_control_details",
               vobj(std::move(sdk_details_object)));
    set_member(summary, "hardening_evidence",
               varr(std::move(hardening_evidence)));
    bool cmake_health_failed = false;
    for (const Value &record: cmake_records) {
        const Value *launch_error = member(record, "launch_error");
        const Value *timed_out = member(record, "timed_out");
        if ((launch_error != nullptr && !io::is_null(*launch_error)) ||
            (timed_out != nullptr &&
             std::holds_alternative<bool>(timed_out->data) &&
             std::get<bool>(timed_out->data)))
            cmake_health_failed = true;
    }
    set_member(summary, "tool_health",
               vstr(cmake_health_failed ? "failed" : "ok"));
    bool all_passed = true;
    for (const auto &[name, status]: sdk_controls)
        if (status != "passed" && status != "rejected" &&
            status != "not-applicable")
            all_passed = false;
    set_member(summary, "status",
               vstr(all_passed && !cmake_health_failed ? "passed" : "failed"));
    for (Value &record: cmake_records) records.push_back(std::move(record));
    set_member(summary, "records", varr(std::move(records)));
    set_member(summary, "log_directory", vstr(log_directory.string()));
    write_summary(build_dir, kind, summary);
    std::cout << "native context SDK controls: " << repr_str_map(sdk_controls)
              << "\n";
    std::cout << "native context tool health: "
              << std::get<std::string>(member(summary, "tool_health")->data)
              << "\n";
    std::cout << "native context summary: "
              << (build_dir / "native_check_summaries" / "context.json")
                     .string()
              << "\n";
    if (std::get<std::string>(member(summary, "status")->data) != "passed") {
        std::cerr << "native context controls failed; see retained control "
                     "logs above\n";
        for (const auto &[name, status]: sdk_controls)
            if (status != "passed" && status != "rejected" &&
                status != "not-applicable")
                std::cerr << "  " << name << ": " << status << "\n";
        return 1;
    }
    return 0;
}

// ---- structural contracts (check_native_contracts.py port) ---------------------

namespace {

constexpr std::array<std::string_view, 3> kSupportTestModules{
    "test_native_check_tools.py",
    "test_native_contract_checks.py",
    "test_native_loader.py",
};

Value read_contract_json(const fs::path &path, const std::string &label) {
    std::ifstream const stream(path, std::ios::binary);
    if (!stream.is_open()) {
        std::error_code ec;
        if (!fs::exists(path, ec))
            throw ContractError("missing " + label + ": " + path.string());
        throw ContractError("malformed " + label + ": " + path.string() +
                            ": unreadable");
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    try {
        return io::parse_json(buffer.str());
    } catch (const std::exception &error) {
        throw ContractError("malformed " + label + ": " + path.string() + ": " +
                            error.what());
    }
}

// importlib.machinery.EXTENSION_SUFFIXES equivalents on this platform: any
// bike_native<tag>.so regular file in the build dir.
std::vector<fs::path> extensions_in(const fs::path &build) {
    std::vector<fs::path> found;
    static const std::regex pattern("^bike_native.*\\.so$");
    std::error_code ec;
    for (fs::directory_iterator it(build, ec), end; !ec && it != end;
         it.increment(ec)) {
        std::error_code file_ec;
        if (it->is_regular_file(file_ec) &&
            std::regex_match(it->path().filename().string(), pattern))
            found.push_back(canonical_path(it->path()));
    }
    std::ranges::sort(found);
    return found;
}

fs::path selected_extension(const fs::path &build) {
    const std::vector<fs::path> extensions = extensions_in(build);
    if (extensions.size() != 1)
        throw ContractError("expected exactly one bike_native extension in " +
                            build.string() + ", found " +
                            std::to_string(extensions.size()));
    return extensions.front();
}

Value validate_provenance(
    const fs::path &build, const fs::path &extension,
    const std::map<std::string, std::string> &environment) {
    if (const auto selected = environment.find("NATIVE_TEST_BUILD_PATH");
        selected != environment.end()) {
        const fs::path selected_path(selected->second);
        if (!selected_path.is_absolute() || canonical_path(selected_path) != build)
            throw ContractError(
                "NATIVE_TEST_BUILD_PATH differs from requested build: " +
                selected->second + " != " + build.string());
    }
    fs::path provenance_path;
    if (const auto configured = environment.find("NATIVE_TEST_PROVENANCE_PATH");
        configured != environment.end())
        provenance_path = canonical_path(expand_home(configured->second));
    else
        provenance_path = build / "native_test_provenance.json";
    const Value provenance =
        read_contract_json(provenance_path, "native loader provenance");
    const Value *schema = member(provenance, "schema_version");
    const bool schema_one =
        schema != nullptr &&
        std::holds_alternative<io::Number>(schema->data) &&
        std::get<io::Number>(schema->data).lexeme == "1";
    if (!std::holds_alternative<Object>(provenance.data) || !schema_one)
        throw ContractError(
            "native loader provenance has an unsupported schema: " +
            provenance_path.string());
    const std::string selected_build =
        opt_string(provenance, "selected_build").value_or("");
    if (selected_build != build.string())
        throw ContractError(
            "native loader provenance selected build does not match: '" +
            selected_build + "' != '" + build.string() + "'");
    const Value *imported = member(provenance, "imported_extension");
    const auto *imported_text =
        imported != nullptr ? std::get_if<std::string>(&imported->data)
                            : nullptr;
    if (imported_text == nullptr || !fs::path(*imported_text).is_absolute() ||
        canonical_path(*imported_text) != extension)
        throw ContractError(
            "native loader provenance imported extension does not match "
            "selected artifact: '" +
            (imported_text != nullptr ? *imported_text : std::string("None")) +
            "' != '" + extension.string() + "'");
    if (const Value *count = member(provenance, "native_test_count");
        count != nullptr && !io::is_null(*count)) {
        const auto *number = std::get_if<io::Number>(&count->data);
        bool integer = number != nullptr;
        std::int64_t value = 0;
        if (integer) {
            const std::string &lexeme = number->lexeme;
            integer = !lexeme.empty() &&
                      std::ranges::all_of(lexeme,
                                  [](unsigned char ch) {
                                      return std::isdigit(ch);
                                  });
            if (integer) value = parse_int64(lexeme).value_or(-1);
        }
        if (!integer || value < 0)
            throw ContractError(
                "native loader provenance has an invalid native_test_count");
    }
    return provenance;
}

Value validate_language_and_hardening(const Value &context,
                                      const Value &sources) {
    const Value *target_contexts = member(context, "target_contexts");
    const Value *entries = member(sources, "sources");
    if (target_contexts == nullptr ||
        !std::holds_alternative<Object>(target_contexts->data) ||
        entries == nullptr ||
        !std::holds_alternative<Array>(entries->data) ||
        std::get<Array>(entries->data).empty())
        throw ContractError(
            "native target context or source manifest is missing");
    std::set<std::string> targets;
    bool saw_null_target = false;
    for (const Value &entry: std::get<Array>(entries->data)) {
        if (!std::holds_alternative<Object>(entry.data)) continue;
        const Value *target = member(entry, "target");
        if (target == nullptr || io::is_null(*target) ||
            !std::holds_alternative<std::string>(target->data))
            saw_null_target = true;
        else
            targets.insert(std::get<std::string>(target->data));
    }
    if (saw_null_target || targets.empty())
        throw ContractError(
            "native source manifest contains an invalid target list");
    const Value *hardening = member(context, "libcpp_hardening");
    if (hardening == nullptr ||
        !std::holds_alternative<Object>(hardening->data))
        throw ContractError(
            "native check context has no libc++ hardening record");
    const std::string mode =
        opt_string(*hardening, "mode").value_or("");
    const Value *supported_value = member(*hardening, "supported");
    const bool supported =
        supported_value != nullptr &&
        std::holds_alternative<bool>(supported_value->data) &&
        std::get<bool>(supported_value->data);
    if (supported_value == nullptr ||
        !std::holds_alternative<bool>(supported_value->data))
        throw ContractError(
            "native libc++ hardening support must be boolean");
    std::string required_definition;
    if (supported) {
        if (mode != "EXTENSIVE" && mode != "FAST")
            throw ContractError(
                "unsupported configured libc++ hardening mode: '" + mode +
                "'");
        required_definition =
            "_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_" + mode;
    }
    std::vector<std::string> violations;
    for (const std::string &target: targets) {
        const Value *target_context = member(*target_contexts, target);
        if (target_context == nullptr ||
            !std::holds_alternative<Object>(target_context->data)) {
            violations.push_back(target + ": no configured target context");
            continue;
        }
        const Value *standard = member(*target_context, "cxx_standard");
        const bool standard_23 =
            standard != nullptr &&
            std::holds_alternative<io::Number>(standard->data) &&
            std::get<io::Number>(standard->data).lexeme == "23";
        if (!standard_23)
            violations.push_back(target + ": C++ standard is not ISO C++23");
        const Value *extensions = member(*target_context, "cxx_extensions");
        const bool extensions_off =
            extensions != nullptr &&
            std::holds_alternative<bool>(extensions->data) &&
            !std::get<bool>(extensions->data);
        if (!extensions_off)
            violations.push_back(target +
                                 ": C++ language extensions are enabled");
        const Value *definitions =
            member(*target_context, "compile_definitions");
        if (definitions == nullptr ||
            !std::holds_alternative<Array>(definitions->data)) {
            violations.push_back(target + ": compile definitions are malformed");
        } else if (!required_definition.empty()) {
            bool present = false;
            for (const Value &item: std::get<Array>(definitions->data))
                if (std::holds_alternative<std::string>(item.data) &&
                    std::get<std::string>(item.data) == required_definition)
                    present = true;
            if (!present) {
                std::string violation = target;
                violation += ": missing libc++ hardening definition ";
                violation += required_definition;
                violations.push_back(std::move(violation));
            }
        }
    }
    if (!violations.empty()) {
        std::string joined;
        for (const auto &violation: violations) {
            if (!joined.empty()) joined += "; ";
            joined += violation;
        }
        throw ContractError(joined);
    }
    Array target_values;
    for (const auto &target: targets)
        target_values.push_back(vstr(target));
    return vobj({
        {"targets", varr(std::move(target_values))},
        {"cxx_standard", vint(23)},
        {"extensions", vbool(false)},
        {"hardening_mode", vstr(mode)},
    });
}

// _yaml_setting: top-level "key: value" with YAML block-scalar support.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (content, key) order mirrors the oracle helper
std::optional<std::string> yaml_setting(const std::string &content,
                                        const std::string &key) {
    const auto lines = split_lines(content);
    const std::regex header("^\\s*" + key + R"(\s*:\s*(.*?)\s*$)");
    for (std::size_t index = 0; index < lines.size(); ++index) {
        std::smatch match;
        if (!std::regex_match(lines[index], match, header)) continue;
        std::string value = match[1].str();
        if (value == ">" || value == "|" || value == ">-" || value == "|-") {
            const std::size_t setting_indent =
                lines[index].find_first_not_of(" \t");
            std::string block;
            for (std::size_t offset = index + 1; offset < lines.size();
                 ++offset) {
                const std::string &block_line = lines[offset];
                const auto first_nonspace =
                    block_line.find_first_not_of(" \t");
                if (first_nonspace == std::string::npos) continue;
                if (first_nonspace <= setting_indent) break;
                if (!block.empty()) block += ' ';
                block += block_line.substr(first_nonspace);
            }
            return block;
        }
        if (value.size() >= 2 &&
            (value.front() == '\'' || value.front() == '"') &&
            value.back() == value.front())
            value = value.substr(1, value.size() - 2);
        return value;
    }
    return std::nullopt;
}

Value validate_tidy_policy(const fs::path &repo_root) {
    const fs::path path = repo_root / "native" / ".clang-tidy";
    std::ifstream const stream(path, std::ios::binary);
    if (!stream.is_open())
        throw ContractError("cannot read native clang-tidy policy " +
                            path.string() + ": unreadable");
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    const std::string content = buffer.str();
    const auto checks_value = yaml_setting(content, "Checks");
    const auto header_filter = yaml_setting(content, "HeaderFilterRegex");
    if (!checks_value.has_value() || !header_filter.has_value())
        throw ContractError(
            "native clang-tidy policy must declare Checks and "
            "HeaderFilterRegex");
    bool move_enabled = false;
    {
        std::istringstream tokens(*checks_value);
        std::string token;
        while (std::getline(tokens, token, ',')) {
            const auto first = token.find_first_not_of(" \t");
            const auto last = token.find_last_not_of(" \t");
            if (first == std::string::npos) continue;
            token = token.substr(first, last - first + 1);
            const bool disabled = !token.empty() && token.front() == '-';
            const std::string pattern = disabled ? token.substr(1) : token;
            if (fnmatchcase("clang-analyzer-cplusplus.Move", pattern))
                move_enabled = !disabled;
        }
    }
    if (!move_enabled)
        throw ContractError(
            "clang-tidy must enable first-party "
            "clang-analyzer-cplusplus.Move findings");
    std::regex header_pattern;
    try {
        header_pattern = std::regex(*header_filter);
    } catch (const std::regex_error &error) {
        throw ContractError("native clang-tidy header filter is invalid: " +
                            std::string(error.what()));
    }
    const std::vector<std::string> required_paths = {
        "native/src/contract.hpp",
        "native/tests/test_contracts.cpp",
        "tools/proto_native_bench/bench.cpp",
    };
    for (const auto &required: required_paths)
        if (!std::regex_search(required, header_pattern))
            throw ContractError(
                "native clang-tidy header filter omits a portable "
                "first-party path");
    for (const auto &required: required_paths)
        if (!std::regex_search("/checkout/" + required, header_pattern))
            throw ContractError(
                "native clang-tidy header filter is not checkout portable");
    return vobj({
        {"move_analysis", vstr("enabled")},
        {"header_filter", vstr(*header_filter)},
    });
}

// Comment- and string-blanking view of a Python module: '#' outside string
// literals truncates the rest of the line and string contents are blanked
// (keeping newlines so statement boundaries stay aligned). The oracle uses
// ast.parse; this scan reproduces the same three facts for our own test
// files: a `from native_loader import ... load_native` statement, a
// `load_native(` call, and the absence of a raw `import bike_native` edge.
std::string python_code_view(const std::string &text) {
    std::string output;
    output.reserve(text.size());
    enum class Quote : std::uint8_t {
        none,
        single,
        double_,
        triple_single,
        triple_double
    };
    Quote quote = Quote::none;
    bool escaped = false;
    std::size_t index = 0;
    while (index < text.size()) {
        const char ch = text[index];
        if (quote == Quote::none) {
            if (ch == '#') {
                while (index < text.size() && text[index] != '\n') ++index;
                continue;
            }
            if (ch == '\'' && index + 2 < text.size() &&
                text[index + 1] == '\'' && text[index + 2] == '\'') {
                quote = Quote::triple_single;
                output += '\'';
                index += 3;
                continue;
            }
            if (ch == '"' && index + 2 < text.size() &&
                text[index + 1] == '"' && text[index + 2] == '"') {
                quote = Quote::triple_double;
                output += '"';
                index += 3;
                continue;
            }
            if (ch == '\'') {
                quote = Quote::single;
                output += ch;
                ++index;
                continue;
            }
            if (ch == '"') {
                quote = Quote::double_;
                output += ch;
                ++index;
                continue;
            }
            output += ch;
            ++index;
            continue;
        }
        if (escaped) {
            escaped = false;
            ++index;
            continue;
        }
        if (ch == '\\') {
            escaped = true;
            ++index;
            continue;
        }
        if (quote == Quote::single || quote == Quote::double_) {
            const char terminator = quote == Quote::single ? '\'' : '"';
            if (ch == '\n') {
                // Unterminated single-line string: treat as closed (the
                // oracle would raise SyntaxError; our files are valid).
                quote = Quote::none;
                output += '\n';
                ++index;
                continue;
            }
            if (ch == terminator) {
                quote = Quote::none;
                output += ch;
                ++index;
                continue;
            }
            output += ' ';
            ++index;
            continue;
        }
        const char t0 = quote == Quote::triple_single ? '\'' : '"';
        if (ch == t0 && index + 2 < text.size() && text[index + 1] == t0 &&
            text[index + 2] == t0) {
            quote = Quote::none;
            index += 3;
            continue;
        }
        if (ch == '\n') output += '\n';
        ++index;
    }
    return output;
}

Value validate_test_loaders(const fs::path &repo_root) {
    const fs::path reference_root = repo_root / "tests" / "reference";
    std::vector<fs::path> test_files;
    std::error_code ec;
    for (fs::directory_iterator it(reference_root, ec), end; !ec && it != end;
         it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.starts_with("test_native_") && name.ends_with(".py") &&
            std::ranges::find(kSupportTestModules, name) ==
                kSupportTestModules.end()) {
            std::error_code file_ec;
            if (it->is_regular_file(file_ec)) test_files.push_back(it->path());
        }
    }
    std::ranges::sort(test_files);
    if (test_files.empty())
        throw ContractError("no native extension tests found under " +
                            reference_root.string());
    std::vector<std::string> violations;
    for (const fs::path &path: test_files) {
        std::ifstream const stream(path, std::ios::binary);
        if (!stream.is_open()) {
            violations.push_back(path.filename().string() +
                                 ": cannot inspect module: unreadable");
            continue;
        }
        std::ostringstream buffer;
        buffer << stream.rdbuf();
        const std::string code = python_code_view(buffer.str());
        bool imports_loader = false;
        static const std::regex from_loader(
            "(^|\\n)[ \\t]*from[ \\t]+native_loader[ \\t]+import[ \\t]*("
            "\\([^)]*\\)|[^\\n]*)");
        for (std::sregex_iterator it(code.begin(), code.end(), from_loader),
                                  end;
             it != end; ++it) {
            static const std::regex name_pattern("\\bload_native\\b");
            if (std::regex_search((*it)[2].str(), name_pattern))
                imports_loader = true;
        }
        static const std::regex call_pattern(R"(\bload_native[ \t]*\()");
        const bool calls_loader = std::regex_search(code, call_pattern);
        static const std::regex raw_import(
            "(^|\\n)[ \\t]*(import[ \\t]+[^\\n]*\\bbike_native\\b|from[ \\t]+"
            "bike_native[ \\t]+import\\b)");
        const bool imports_raw_module = std::regex_search(code, raw_import);
        if (!imports_loader || !calls_loader || imports_raw_module)
            violations.push_back(
                path.filename().string() +
                ": native tests must use the selected-artifact loader");
    }
    if (!violations.empty()) {
        std::string joined;
        for (const auto &violation: violations) {
            if (!joined.empty()) joined += "; ";
            joined += violation;
        }
        throw ContractError(joined);
    }
    Array module_names;
    for (const auto &path: test_files)
        module_names.push_back(vstr(path.filename().string()));
    return vobj({
        {"module_count",
         vint(static_cast<std::int64_t>(test_files.size()))},
        {"modules", varr(std::move(module_names))},
    });
}

Value validate_contract_manifest(const Value &sources,
                                 const fs::path &repo_root) {
    const Value *entries = member(sources, "sources");
    const fs::path contract_source =
        canonical_path(repo_root / "native" / "tests" / "test_contracts.cpp");
    if (entries == nullptr || !std::holds_alternative<Array>(entries->data))
        throw ContractError("native source manifest has no source entries");
    std::size_t matches = 0;
    for (const Value &entry: std::get<Array>(entries->data)) {
        if (!std::holds_alternative<Object>(entry.data)) continue;
        const auto *path = member(entry, "path") != nullptr
                               ? std::get_if<std::string>(
                                     &member(entry, "path")->data)
                               : nullptr;
        const auto *target = member(entry, "target") != nullptr
                                 ? std::get_if<std::string>(
                                       &member(entry, "target")->data)
                                 : nullptr;
        if (path != nullptr && canonical_path(*path) == contract_source &&
            target != nullptr && *target == "native_contract_tests")
            ++matches;
    }
    if (matches != 1)
        throw ContractError(
            "native contract source is not uniquely manifested in "
            "native_contract_tests");
    return vobj({
        {"source", vstr(contract_source.string())},
        {"target", vstr("native_contract_tests")},
    });
}

fs::path contract_executable(const fs::path &build) {
    for (const fs::path &candidate:
         {build / "native_contract_tests",
          build / "Debug" / "native_contract_tests",
          build / "Release" / "native_contract_tests"}) {
        std::error_code ec;
        if (fs::is_regular_file(candidate, ec) && is_executable_file(candidate))
            return canonical_path(candidate);
    }
    throw ContractError(
        "missing executable native contract test binary in " + build.string());
}

PlainResult run_contract_case(
    const fs::path &executable, const std::vector<std::string> &arguments,
    const std::string &label, const fs::path &build) {
    auto environment = environ_copy();
    if (const auto runtime_value =
            environment.find("NATIVE_TEST_SANITIZER_RUNTIME");
        runtime_value != environment.end()) {
        const fs::path runtime(runtime_value->second);
        std::error_code ec;
        if (!runtime.is_absolute() || !fs::is_regular_file(runtime, ec))
            throw ContractError(
                "selected sanitizer runtime is not an absolute file: " +
                runtime.string());
        const std::string preload_name = "DYLD_INSERT_LIBRARIES";
        std::vector<std::string> existing;
        {
            std::istringstream tokens(environment[preload_name]);
            std::string item;
            while (std::getline(tokens, item, ':'))
                if (!item.empty()) existing.push_back(item);
        }
        if (std::ranges::find(existing, runtime.string()) ==
            existing.end()) {
            existing.push_back(runtime.string());
            std::string joined;
            for (const auto &item: existing) {
                if (!joined.empty()) joined += ':';
                joined += item;
            }
            environment[preload_name] = joined;
        }
    }
    std::vector<std::string> argv = {executable.string()};
    argv.insert(argv.end(), arguments.begin(), arguments.end());
    const PlainResult result = run_plain(argv, build, environment);
    write_contract_logs(build, label, result.out, result.err);
    if (result.exit_code != 0) {
        const auto strip = [](const std::string &text) {
            const auto first = text.find_first_not_of(" \t\n\v\f\r");
            const auto last = text.find_last_not_of(" \t\n\v\f\r");
            return first == std::string::npos
                       ? std::string()
                       : text.substr(first, last - first + 1);
        };
        const std::string detail = !strip(result.err).empty()
                                       ? strip(result.err)
                                       : strip(result.out);
        throw ContractError("native contract executable " + label +
                            " exited " + std::to_string(result.exit_code) +
                            ": " + detail);
    }
    return result;
}

Value validate_ctest_contracts(const fs::path &build) {
    const fs::path ctest_file = build / "CTestTestfile.cmake";
    std::ifstream const stream(ctest_file, std::ios::binary);
    if (!stream.is_open())
        throw ContractError("cannot read CTest registration file " +
                            ctest_file.string() + ": unreadable");
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    static const std::regex registration(
        "add_test\\([^\\n)]*native_contract_tests[^\\n)]*"
        "native_contract_tests[^\\n)]*\\)",
        std::regex_constants::icase);
    if (!std::regex_search(buffer.str(), registration))
        throw ContractError(
            "native_contract_tests is not registered with CTest");
    const fs::path executable = contract_executable(build);
    const PlainResult listed =
        run_contract_case(executable, {"--list"}, "case-list", build);
    std::vector<std::string> names;
    for (const auto &line: split_lines(listed.out)) {
        const auto first = line.find_first_not_of(" \t");
        const auto last = line.find_last_not_of(" \t");
        if (first != std::string::npos)
            names.push_back(line.substr(first, last - first + 1));
    }
    const std::set<std::string> unique(names.begin(), names.end());
    if (names.empty() || unique.size() != names.size())
        throw ContractError(
            "native contract executable lists no cases or contains duplicate "
            "case names");
    static const std::regex name_pattern("[A-Za-z0-9_]+");
    std::vector<std::string> invalid;
    for (const auto &name: names)
        if (!std::regex_match(name, name_pattern)) invalid.push_back(name);
    if (!invalid.empty()) {
        std::string joined;
        for (const auto &name: invalid) {
            if (!joined.empty()) joined += ", ";
            joined += name;
        }
        throw ContractError(
            "native contract case names are malformed: [" + joined + "]");
    }
    for (const auto &name: names)
        run_contract_case(executable, {name}, "case-" + name, build);
    Array case_values;
    for (const auto &name: names) case_values.push_back(vstr(name));
    return vobj({
        {"executable", vstr(executable.string())},
        {"cases", varr(std::move(case_values))},
        {"ctest_registered", vbool(true)},
    });
}

// _check_pending_rules port: engine-call boundary and benchmark manifest
// rules plus the T56 step-4.6 pure-core dependency policy (no Python/
// nanobind/NumPy include edges in pure-core library TUs).
Value check_pending_rules(const fs::path &repo_root, const Value &sources) {
    const Value *source_entries_value = member(sources, "sources");
    static const Array empty_entries;
    const Array &source_entries =
        source_entries_value != nullptr &&
                std::holds_alternative<Array>(source_entries_value->data)
            ? std::get<Array>(source_entries_value->data)
            : empty_entries;
    const fs::path native_src = repo_root / "native" / "src";
    const fs::path engine_source = native_src / "engine_call.cpp";
    const fs::path engine_header = native_src / "engine_call.hpp";
    const fs::path engine_abi = native_src / "engine_abi_312.hpp";
    const std::vector<fs::path> engine_files = {engine_source, engine_header,
                                                engine_abi};
    bool any_engine = false;
    bool all_engine = true;
    std::error_code ec;
    for (const auto &path: engine_files) {
        if (fs::exists(path, ec))
            any_engine = true;
        else
            all_engine = false;
    }
    Object engine_status;
    if (any_engine) {
        if (!all_engine)
            throw ContractError(
                "installed engine-call boundary is missing a source or ABI "
                "header");
        std::size_t matches = 0;
        static const std::set<std::string> engine_targets = {
            "bike_native_engine", "bike_engine"};
        for (const Value &entry: source_entries) {
            if (!std::holds_alternative<Object>(entry.data)) continue;
            const auto *path = member(entry, "path") != nullptr
                                   ? std::get_if<std::string>(
                                         &member(entry, "path")->data)
                                   : nullptr;
            const auto *target = member(entry, "target") != nullptr
                                     ? std::get_if<std::string>(
                                           &member(entry, "target")->data)
                                     : nullptr;
            if (path != nullptr && target != nullptr &&
                canonical_path(*path) == canonical_path(engine_source) &&
                engine_targets.contains(*target))
                ++matches;
        }
        if (matches != 1)
            throw ContractError(
                "engine_call.cpp is not uniquely manifested in "
                "bike_native_engine");
        std::ostringstream header_buffer;
        header_buffer << std::ifstream(engine_header).rdbuf();
        const std::string header_text = header_buffer.str();
        const std::vector<std::string> required_wrappers = {
            "invoke", "load_model", "make_data", "forward", "step",
            "reset_data", "set_const", "try_forward", "try_step",
        };
        std::vector<std::string> missing;
        for (const auto &name: required_wrappers) {
            const std::regex call_pattern("\\b" + name + "\\s*\\(");
            if (!std::regex_search(header_text, call_pattern))
                missing.push_back(name);
        }
        if (!missing.empty()) {
            std::string joined;
            for (const auto &name: missing) {
                if (!joined.empty()) joined += ", ";
                joined += name;
            }
            throw ContractError(
                "engine-call interface is missing wrappers: " + joined);
        }
        static const std::regex fatal_calls(
            "\\bmj_(?:loadModel(?:Buffer)?|makeData|forward|step|resetData|"
            "setConst)\\s*\\(");
        std::vector<std::string> unguarded;
        const fs::path resolved_engine = canonical_path(engine_source);
        for (fs::recursive_directory_iterator it(native_src, ec), end;
             !ec && it != end; it.increment(ec)) {
            std::error_code file_ec;
            if (!it->is_regular_file(file_ec) ||
                it->path().extension() != ".cpp")
                continue;
            if (canonical_path(it->path()) == resolved_engine) continue;
            std::ostringstream body;
            body << std::ifstream(it->path()).rdbuf();
            if (std::regex_search(body.str(), fatal_calls)) {
                std::error_code rel_ec;
                const fs::path relative =
                    fs::relative(it->path(), repo_root, rel_ec);
                unguarded.push_back(rel_ec ? it->path().string()
                                           : relative.string());
            }
        }
        std::ranges::sort(unguarded);
        if (!unguarded.empty()) {
            std::string joined;
            for (const auto &name: unguarded) {
                if (!joined.empty()) joined += ", ";
                joined += name;
            }
            throw ContractError("unguarded fatal-capable MuJoCo calls: " +
                                joined);
        }
        engine_status = {
            {"status", vstr("passed")},
            {"detail",
             vstr("pinned engine-call wrappers are manifested; direct "
                  "fatal-capable calls are confined to engine_call.cpp")},
        };
    } else {
        engine_status = {
            {"status", vstr("pending")},
            {"detail",
             vstr("no engine-call wrapper declaration manifest exists yet")},
        };
    }

    Object benchmark_status;
    {
        std::set<std::string> benchmark_targets;
        for (const Value &entry: source_entries) {
            if (!std::holds_alternative<Object>(entry.data)) continue;
            const auto *target = member(entry, "target") != nullptr
                                     ? std::get_if<std::string>(
                                           &member(entry, "target")->data)
                                     : nullptr;
            if (target != nullptr && *target == "native_bench")
                benchmark_targets.insert(*target);
        }
        if (!benchmark_targets.empty()) {
            std::set<std::string> benchmark_sources;
            const fs::path bench_dir =
                repo_root / "tools" / "proto_native_bench";
            for (fs::directory_iterator it(bench_dir, ec), end;
                 !ec && it != end; it.increment(ec)) {
                std::error_code file_ec;
                if (it->is_regular_file(file_ec) &&
                    it->path().extension() == ".cpp")
                    benchmark_sources.insert(canonical_path(it->path()).string());
            }
            if (benchmark_sources.empty())
                throw ContractError(
                    "native_bench target has no C++ sources under "
                    "tools/proto_native_bench");
            std::set<std::string> manifested;
            for (const Value &entry: source_entries) {
                if (!std::holds_alternative<Object>(entry.data)) continue;
                const auto *path = member(entry, "path") != nullptr
                                       ? std::get_if<std::string>(
                                             &member(entry, "path")->data)
                                       : nullptr;
                const auto *target = member(entry, "target") != nullptr
                                         ? std::get_if<std::string>(
                                               &member(entry, "target")->data)
                                         : nullptr;
                if (path != nullptr && target != nullptr &&
                    benchmark_targets.contains(*target))
                    manifested.insert(canonical_path(*path).string());
            }
            std::vector<std::string> missing_sources;
            for (const auto &source: benchmark_sources)
                if (!manifested.contains(source))
                    missing_sources.push_back(source);
            if (!missing_sources.empty()) {
                std::string joined;
                for (const auto &source: missing_sources) {
                    if (!joined.empty()) joined += ", ";
                    joined += source;
                }
                throw ContractError(
                    "benchmark sources are missing from CMake manifest: " +
                    joined);
            }
            benchmark_status = {
                {"status", vstr("passed")},
                {"detail",
                 vstr("configured benchmark sources are manifested")},
            };
        } else {
            benchmark_status = {
                {"status", vstr("pending")},
                {"detail",
                 vstr("no configured proto_native_bench target exists yet")},
            };
        }
    }

    // T56 step-4.6: pure-core library TUs must not gain Python/nanobind/NumPy
    // include edges — the executable-level dependency rule the removal gate
    // relies on.
    Object pure_core_status;
    {
        static const std::set<std::string> pure_core_targets = {
            "bike_core", "bike_io", "bike_platform", "bike_engine",
            "bike_native_contract_core", "bike_native_engine",
        };
        // The banned edge is the interpreter/binding surface itself —
        // <Python.h> (incl. versioned paths), nanobind/pybind11/numpy/pythran
        // package prefixes and cpython internals. A first-party header whose
        // NAME mentions one of them (numeric_numpy.hpp) is not an edge.
        static const std::regex include_target(
            R"(^[ \t]*#[ \t]*include[ \t]*[<"]([^>"]+)[>"])");
        const auto interpreter_edge = [](const std::string &target) {
            return target == "Python.h" || target.ends_with("/Python.h") ||
                   target.starts_with("nanobind") ||
                   target.starts_with("pybind11") ||
                   target.starts_with("numpy/") ||
                   target.find("/numpy/") != std::string::npos ||
                   target.starts_with("pythran") ||
                   target.find("cpython") != std::string::npos;
        };
        std::vector<std::string> violations;
        for (const Value &entry: source_entries) {
            if (!std::holds_alternative<Object>(entry.data)) continue;
            const auto *path = member(entry, "path") != nullptr
                                   ? std::get_if<std::string>(
                                         &member(entry, "path")->data)
                                   : nullptr;
            const auto *target = member(entry, "target") != nullptr
                                     ? std::get_if<std::string>(
                                           &member(entry, "target")->data)
                                     : nullptr;
            if (path == nullptr || target == nullptr ||
                !pure_core_targets.contains(*target))
                continue;
            std::ifstream stream(*path, std::ios::binary);
            if (!stream.is_open()) continue;
            std::string line;
            std::size_t line_number = 0;
            while (std::getline(stream, line)) {
                ++line_number;
                std::smatch match;
                if (std::regex_search(line, match, include_target) &&
                    interpreter_edge(match.str(1))) {
                    std::ostringstream detail;
                    detail << fs::path(*path).filename().string() << ":"
                           << line_number;
                    violations.push_back(detail.str());
                }
            }
        }
        if (!violations.empty()) {
            std::string joined;
            for (const auto &violation: violations) {
                if (!joined.empty()) joined += ", ";
                joined += violation;
            }
            throw ContractError(
                "pure-core translation units carry Python/nanobind/NumPy "
                "include edges: " +
                joined);
        }
        pure_core_status = {
            {"status", vstr("passed")},
            {"detail",
             vstr("pure-core library sources carry no interpreter/nanobind/"
                  "NumPy include edges")},
        };
    }

    return vobj({
        {"engine_call_wrappers", vobj(std::move(engine_status))},
        {"benchmark_manifest_membership",
         vobj(std::move(benchmark_status))},
        {"pure_core_python_free", vobj(std::move(pure_core_status))},
    });
}

} // namespace

int run_contracts(const fs::path &build,
                  const std::optional<fs::path> &repo_root_override) {
    const fs::path build_dir = canonical_path(expand_home(build.string()));
    const fs::path repo_root =
        repo_root_override.has_value()
            ? canonical_path(expand_home(repo_root_override->string()))
            : canonical_path(fs::path(__FILE__)
                            .parent_path()
                            .parent_path()
                            .parent_path());
    std::error_code ec;
    const std::map<std::string, std::string> environment = environ_copy();
    Value summary;
    try {
        if (!fs::is_directory(build_dir, ec))
            throw ContractError("build directory does not exist: " +
                                build_dir.string());
        const Value context = load_context(build_dir);
        const Value sources = read_contract_json(
            build_dir / "native_sources.json", "native source manifest");
        {
            const Value *schema = member(sources, "schema_version");
            const bool schema_one =
                schema != nullptr &&
                std::holds_alternative<io::Number>(schema->data) &&
                std::get<io::Number>(schema->data).lexeme == "1";
            if (!std::holds_alternative<Object>(sources.data) || !schema_one)
                throw ContractError(
                    "native source manifest has an unsupported schema");
        }
        const Value *source_entries = member(sources, "sources");
        if (source_entries == nullptr ||
            !std::holds_alternative<Array>(source_entries->data) ||
            std::get<Array>(source_entries->data).empty())
            throw ContractError(
                "native source manifest has an empty source set");
        std::vector<fs::path> expected;
        std::size_t index = 0;
        for (const Value &entry: std::get<Array>(source_entries->data)) {
            const bool valid =
                std::holds_alternative<Object>(entry.data) &&
                member(entry, "path") != nullptr &&
                std::holds_alternative<std::string>(
                    member(entry, "path")->data) &&
                member(entry, "target") != nullptr &&
                std::holds_alternative<std::string>(
                    member(entry, "target")->data) &&
                member(entry, "context") != nullptr &&
                std::holds_alternative<std::string>(
                    member(entry, "context")->data);
            if (!valid)
                throw ContractError("native source manifest entry " +
                                    std::to_string(index) + " is malformed");
            expected.push_back(canonical_path(
                std::get<std::string>(member(entry, "path")->data)));
            ++index;
        }
        std::vector<CompileEntry> compiled_entries;
        try {
            compiled_entries = load_entries(build_dir, expected);
        } catch (const CheckError &error) {
            throw ContractError(
                "native source manifest differs from selected translation "
                "units: " +
                std::string(error.what()));
        }

        Object checks;
        checks.emplace_back(
            "manifest_selection",
            vobj({
                {"status", vstr("passed")},
                {"source_count",
                 vint(static_cast<std::int64_t>(expected.size()))},
                {"translation_unit_context_count",
                 vint(static_cast<std::int64_t>(compiled_entries.size()))},
            }));

        // Binding-artifact checks: the Python-free build produces no
        // extension, so loader provenance / contract-executable checks report
        // not-applicable rather than failing on an intentionally absent
        // artifact; when an extension IS present the oracle path runs
        // unchanged. A bindings context (python_executable non-null) keeps
        // the oracle's fail-closed contract — an absent extension there is a
        // broken build, not an accepted shape.
        const bool bindings_expected =
            opt_string(context, "python_executable").has_value();
        const std::vector<fs::path> found_extensions =
            extensions_in(build_dir);
        if (found_extensions.empty() && bindings_expected)
            throw ContractError(
                "expected exactly one bike_native extension in " +
                build_dir.string() + ", found 0");
        if (!found_extensions.empty()) {
            const fs::path extension = selected_extension(build_dir);
            checks.emplace_back(
                "loader_provenance",
                vobj({
                    {"status", vstr("passed")},
                    {"provenance",
                     validate_provenance(build_dir, extension, environment)},
                }));
        } else {
            checks.emplace_back(
                "loader_provenance",
                vobj({
                    {"status", vstr("not-applicable")},
                    {"detail",
                     vstr("no bike_native extension in the Python-free "
                          "build")},
                }));
        }
        {
            Object fields = {{"status", vstr("passed")}};
            Object extra = std::get<Object>(
                validate_language_and_hardening(context, sources).data);
            fields.insert(fields.end(),
                          std::make_move_iterator(extra.begin()),
                          std::make_move_iterator(extra.end()));
            checks.emplace_back("iso_language_mode_and_hardening",
                                vobj(std::move(fields)));
        }
        {
            Object fields = {{"status", vstr("passed")}};
            Object extra =
                std::get<Object>(validate_tidy_policy(repo_root).data);
            fields.insert(fields.end(),
                          std::make_move_iterator(extra.begin()),
                          std::make_move_iterator(extra.end()));
            checks.emplace_back("analysis_policy", vobj(std::move(fields)));
        }
        {
            Object fields = {{"status", vstr("passed")}};
            Object extra =
                std::get<Object>(validate_test_loaders(repo_root).data);
            fields.insert(fields.end(),
                          std::make_move_iterator(extra.begin()),
                          std::make_move_iterator(extra.end()));
            checks.emplace_back("selected_artifact_assertions",
                                vobj(std::move(fields)));
        }
        {
            bool has_contract_target = false;
            for (const Value &entry: std::get<Array>(source_entries->data)) {
                if (!std::holds_alternative<Object>(entry.data)) continue;
                const auto *target = member(entry, "target") != nullptr
                                         ? std::get_if<std::string>(
                                               &member(entry, "target")->data)
                                         : nullptr;
                if (target != nullptr && *target == "native_contract_tests")
                    has_contract_target = true;
            }
            if (has_contract_target) {
                Object fields = {{"status", vstr("passed")}};
                Object extra = std::get<Object>(
                    validate_contract_manifest(sources, repo_root).data);
                fields.insert(fields.end(),
                              std::make_move_iterator(extra.begin()),
                              std::make_move_iterator(extra.end()));
                checks.emplace_back("contract_source_manifest_membership",
                                    vobj(std::move(fields)));
                Object ctest_fields = {{"status", vstr("passed")}};
                Object ctest_extra = std::get<Object>(
                    validate_ctest_contracts(build_dir).data);
                ctest_fields.insert(
                    ctest_fields.end(),
                    std::make_move_iterator(ctest_extra.begin()),
                    std::make_move_iterator(ctest_extra.end()));
                checks.emplace_back("ctest_contract_cases",
                                    vobj(std::move(ctest_fields)));
            } else {
                checks.emplace_back(
                    "contract_source_manifest_membership",
                    vobj({
                        {"status", vstr("not-applicable")},
                        {"detail",
                         vstr("native_contract_tests is not part of the "
                              "Python-free build graph")},
                    }));
                checks.emplace_back(
                    "ctest_contract_cases",
                    vobj({
                        {"status", vstr("not-applicable")},
                        {"detail",
                         vstr("no contract test executable in the "
                              "Python-free build")},
                    }));
            }
        }
        {
            Object pending =
                std::get<Object>(check_pending_rules(repo_root, sources).data);
            for (auto &[name, value]: pending)
                checks.emplace_back(name, std::move(value));
        }
        summary = vobj({
            {"schema_version", vint(1)},
            {"status", vstr("passed")},
            {"build", vstr(build_dir.string())},
            {"checks", vobj(std::move(checks))},
        });
    } catch (const std::exception &error) {
        summary = vobj({
            {"schema_version", vint(1)},
            {"status", vstr("failed")},
            {"build", vstr(build_dir.string())},
            {"failures", varr({vstr(error.what())})},
        });
        if (fs::is_directory(build_dir, ec))
            write_summary_atomic(build_dir, summary);
        std::cerr << "native contract checks failed: " << error.what() << "\n";
        return 1;
    }
    write_summary_atomic(build_dir, summary);
    std::cout << "native contract checks passed: " << build_dir.string()
              << "\n";
    for (const auto &[name, result]:
         std::get<Object>(member(summary, "checks")->data)) {
        std::cout << "  " << name << ": "
                  << std::get<std::string>(member(result, "status")->data)
                  << "\n";
    }
    return 0;
}

} // namespace bike::checks
