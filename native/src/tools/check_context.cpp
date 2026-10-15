// check_context.cpp — context, manifest and compilation-database loading
// and validation for the native check orchestrator (task T56). Ports
// tools/native_checks.py's fail-closed selection substrate: load_context,
// _parse_command, _check_path_arguments, _source_for_entry,
// _target_from_entry, _contains_source_argument, _resolve_command_executable,
// _validate_compilation_context, _compile_identity, _is_under,
// _validate_target_context, load_entries, _resolve_tool,
// _temporary_database, _replace_compile_source, _gcc_context_args and
// _standalone_header_roots.
//
// Ordering and error text are preserved field-for-field so a stale or
// tampered build fails on the same boundary the oracle used.
#include "checks_internal.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <utility>

namespace bike::checks {
namespace {

// shlex.split port (POSIX mode): whitespace separates; '...' is literal;
// "..." allows \" \\ \$ \` and backslash-newline; \x quotes the next
// character. A missing close quote is a ValueError in Python -> CheckError.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (command, label) order mirrors the oracle signature
std::vector<std::string> shlex_split(const std::string &command,
                                     const std::string &label) {
    std::vector<std::string> words;
    std::string current;
    bool in_word = false;
    enum class Quote : std::uint8_t { none, single, double_ } quote =
        Quote::none;
    std::size_t index = 0;
    const auto fail = [&]() -> CheckError {
        return CheckError("malformed command in " + label +
                          ": No closing quotation");
    };
    while (index < command.size()) {
        const char ch = command[index];
        if (quote == Quote::single) {
            if (ch == '\'') {
                quote = Quote::none;
            } else {
                current += ch;
            }
            ++index;
            continue;
        }
        if (quote == Quote::double_) {
            if (ch == '"') {
                quote = Quote::none;
                ++index;
                continue;
            }
            if (ch == '\\' && index + 1 < command.size() &&
                (command[index + 1] == '"' || command[index + 1] == '\\' ||
                 command[index + 1] == '$' || command[index + 1] == '`')) {
                current += command[index + 1];
                index += 2;
                continue;
            }
            if (ch == '\\' && index + 1 < command.size() &&
                command[index + 1] == '\n') {
                index += 2;
                continue;
            }
            current += ch;
            ++index;
            continue;
        }
        if (ch == '\\' && index + 1 < command.size() &&
            command[index + 1] == '\n') {
            index += 2;
            continue;
        }
        if (ch == '\\') {
            if (index + 1 >= command.size()) {
                current += ch;
                ++index;
                continue;
            }
            in_word = true;
            current += command[index + 1];
            index += 2;
            continue;
        }
        if (ch == '\'') {
            quote = Quote::single;
            in_word = true;
            ++index;
            continue;
        }
        if (ch == '"') {
            quote = Quote::double_;
            in_word = true;
            ++index;
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(ch))) {
            if (in_word) {
                words.push_back(std::move(current));
                current.clear();
                in_word = false;
            }
            ++index;
            continue;
        }
        in_word = true;
        current += ch;
        ++index;
    }
    if (quote != Quote::none) throw fail();
    if (in_word) words.push_back(std::move(current));
    return words;
}

fs::path require_existing_path(const Value &value, const std::string &label,
                               bool executable = false,
                               bool preserve_symlink = false) {
    const auto *text = std::get_if<std::string>(&value.data);
    if (text == nullptr || text->empty())
        throw CheckError(label + " is missing or is not a path");
    const fs::path expanded = expand_home(*text);
    // A C++ driver may be a symlink to the C driver binary; preserve the
    // spelling (absolute(), not resolve()) when the contract asks for it.
    const fs::path path =
        preserve_symlink ? absolute_lexical(expanded) : canonical_path(expanded);
    std::error_code ec;
    if (!fs::exists(path, ec))
        throw CheckError("missing " + label + ": " + path.string());
    if (executable && !is_executable_file(path))
        throw CheckError(label + " is not executable: " + path.string());
    return path;
}

std::string member_string(const Value &object, const std::string &key) {
    const Value *value = member(object, key);
    if (value == nullptr) return {};
    const auto *text = std::get_if<std::string>(&value->data);
    return text != nullptr ? *text : std::string{};
}

std::vector<std::string> member_string_list(const Value &object,
                                            const std::string &key) {
    std::vector<std::string> result;
    const Value *value = member(object, key);
    if (value == nullptr) return result;
    const auto *array = std::get_if<Array>(&value->data);
    if (array == nullptr) return result;
    for (const Value &item: *array)
        if (const auto *text = std::get_if<std::string>(&item.data))
            result.push_back(*text);
    return result;
}

} // namespace

Value load_context(const fs::path &build) {
    const fs::path build_dir = canonical_path(build);
    std::error_code ec;
    if (!fs::is_directory(build_dir, ec))
        throw CheckError("build directory does not exist: " + build_dir.string());
    Value context =
        read_json(build_dir / "native_check_context.json", "native check context");
    if (!std::holds_alternative<Object>(context.data))
        throw CheckError("native check context has an unsupported schema");
    const Value *schema = member(context, "schema_version");
    const bool schema_one =
        schema != nullptr &&
        std::holds_alternative<io::Number>(schema->data) &&
        std::get<io::Number>(schema->data).lexeme == "1";
    if (!schema_one)
        throw CheckError("native check context has an unsupported schema");

    Value *compiler = member_mut(context, "compiler");
    if (compiler == nullptr || !std::holds_alternative<Object>(compiler->data))
        throw CheckError("native check context has no compiler record");
    const Value *compiler_path_value = member(*compiler, "path");
    const fs::path compiler_path = require_existing_path(
        compiler_path_value != nullptr ? *compiler_path_value : Value{},
        "configured C++ compiler", true, true);
    set_member(*compiler, "path", vstr(compiler_path.string()));

    {
        const Value *python = member(context, "python_executable");
        if (python != nullptr && !io::is_null(*python)) {
            // Bindings-era contexts carry a configured interpreter; validate
            // it. The Python-free core context may carry explicit null.
            const fs::path python_path = require_existing_path(
                *python, "configured Python executable", true);
            set_member(context, "python_executable",
                       vstr(python_path.string()));
        } else {
            set_member(context, "python_executable", vnull());
        }
    }

    const Value *sdk_value = member(context, "sdk");
    const fs::path sdk_path = require_existing_path(
        sdk_value != nullptr ? *sdk_value : Value{}, "configured SDK");
    if (!fs::is_directory(sdk_path, ec))
        throw CheckError("configured SDK is not a directory: " + sdk_path.string());
    set_member(context, "sdk", vstr(sdk_path.string()));
    const Value *sdk_requested_raw = member(context, "sdk_requested");
    const auto *sdk_requested_text =
        sdk_requested_raw != nullptr
            ? std::get_if<std::string>(&sdk_requested_raw->data)
            : nullptr;
    if (sdk_requested_text == nullptr || sdk_requested_text->empty())
        throw CheckError("native check context has no requested SDK value");
    const std::string sdk_requested = *sdk_requested_text;
    const Value *sdk_source_raw = member(context, "sdk_source");
    const auto *sdk_source_text =
        sdk_source_raw != nullptr
            ? std::get_if<std::string>(&sdk_source_raw->data)
            : nullptr;
    const std::string sdk_source =
        sdk_source_text != nullptr ? *sdk_source_text : std::string{};
    if (sdk_source != "explicit" && sdk_source != "default")
        throw CheckError("native check context has an invalid SDK source");
    const fs::path requested_sdk_path = expand_home(sdk_requested);
    if (fs::exists(requested_sdk_path) &&
        canonical_path(requested_sdk_path) != sdk_path)
        throw CheckError(
            "requested SDK and selected SDK differ: requested=" +
            canonical_path(requested_sdk_path).string() + ", selected=" +
            sdk_path.string());

    const Value *hardening = member(context, "libcpp_hardening");
    if (hardening == nullptr || !std::holds_alternative<Object>(hardening->data))
        throw CheckError("native check context has no libc++ hardening record");
    const auto hardening_supported = opt_bool(*hardening, "supported");
    if (!hardening_supported.has_value())
        throw CheckError(
            "native check libc++ hardening support must be boolean");
    const std::string hardening_mode =
        opt_string(*hardening, "mode").value_or("");
    if (hardening_mode != "EXTENSIVE" && hardening_mode != "FAST" &&
        hardening_mode != "NOT_APPLICABLE")
        throw CheckError(
            "native check context has an invalid libc++ hardening mode");
    const std::string hardening_requested =
        opt_string(*hardening, "requested_mode").value_or("");
    if (hardening_requested != "EXTENSIVE" && hardening_requested != "FAST")
        throw CheckError(
            "native check context has an invalid requested libc++ hardening mode");
    if (*hardening_supported != (hardening_mode != "NOT_APPLICABLE"))
        throw CheckError(
            "native check libc++ hardening support and selected mode disagree");
    if (*hardening_supported && hardening_mode != hardening_requested)
        throw CheckError(
            "native check libc++ hardening mode differs from its configured request");

    const Value *suppressions_value =
        member(context, "analysis_suppressions_manifest");
    const fs::path suppressions_manifest = require_existing_path(
        suppressions_value != nullptr ? *suppressions_value : Value{},
        "analysis suppression manifest");
    if (!fs::is_regular_file(suppressions_manifest, ec))
        throw CheckError("analysis suppression manifest is not a file: " +
                         suppressions_manifest.string());
    set_member(context, "analysis_suppressions_manifest",
               vstr(suppressions_manifest.string()));

    for (const std::string_view field:
         {"include_paths", "library_paths", "required_files", "source_roots"}) {
        const Value *values = member(context, field);
        if (values == nullptr || !std::holds_alternative<Array>(values->data))
            throw CheckError("native check context field '" +
                             std::string(field) + "' must be a path list");
        Array resolved_values;
        for (const Value &item: std::get<Array>(values->data)) {
            const auto *text = std::get_if<std::string>(&item.data);
            if (text == nullptr)
                throw CheckError("native check context field '" +
                                 std::string(field) + "' must be a path list");
            const fs::path path = require_existing_path(
                item, "context " + std::string(field) + " entry");
            const bool require_file = field == "required_files";
            if (require_file && !fs::is_regular_file(path, ec))
                throw CheckError("required dependency is not a file: " +
                                 path.string());
            if ((field == "include_paths" || field == "source_roots") &&
                !fs::is_directory(path, ec))
                throw CheckError("context " + std::string(field) +
                                 " entry is not a directory: " + path.string());
            if (field == "library_paths" && !fs::is_regular_file(path, ec) &&
                !(fs::is_directory(path, ec) && path.extension() == ".framework"))
                throw CheckError(
                    "context library_paths entry is neither a library file nor a "
                    "framework directory: " +
                    path.string());
            resolved_values.push_back(vstr(path.string()));
        }
        set_member(context, std::string(field), varr(std::move(resolved_values)));
    }

    for (const std::string_view field: {"tool_defaults", "tool_overrides"}) {
        const Value *values = member(context, field);
        if (values == nullptr || !std::holds_alternative<Object>(values->data))
            throw CheckError("native check context field '" +
                             std::string(field) + "' must be a string map");
        for (const auto &[key, value]: std::get<Object>(values->data))
            if (!std::holds_alternative<std::string>(value.data))
                throw CheckError("native check context field '" +
                                 std::string(field) + "' must be a string map");
    }

    Value *target_contexts = member_mut(context, "target_contexts");
    if (target_contexts == nullptr) {
        set_member(context, "target_contexts", vobj({}));
        target_contexts = member_mut(context, "target_contexts");
    }
    if (!std::holds_alternative<Object>(target_contexts->data))
        throw CheckError(
            "native check context field 'target_contexts' must be an object");
    for (const auto &[target, target_context]:
         std::get<Object>(target_contexts->data)) {
        if (!std::holds_alternative<Object>(target_context.data))
            throw CheckError(
                "native check target contexts must map target names to objects");
        for (const std::string_view field: {"include_directories",
                                            "compile_definitions",
                                            "compile_options"}) {
            const Value *values =
                member(target_context, field);
            if (values != nullptr &&
                std::holds_alternative<Array>(values->data)) {
                bool strings = true;
                for (const Value &item: std::get<Array>(values->data))
                    if (!std::holds_alternative<std::string>(item.data))
                        strings = false;
                if (!strings)
                    throw CheckError("target context '" + target +
                                     "' field '" + std::string(field) +
                                     "' must be a string list");
            } else if (values != nullptr) {
                throw CheckError("target context '" + target + "' field '" +
                                 std::string(field) +
                                 "' must be a string list");
            }
        }
        const Value *standard = member(target_context, "cxx_standard");
        const bool standard_23 =
            standard != nullptr &&
            std::holds_alternative<io::Number>(standard->data) &&
            std::get<io::Number>(standard->data).lexeme == "23";
        if (!standard_23)
            throw CheckError("target context '" + target + "' must use C++23");
        const Value *extensions = member(target_context, "cxx_extensions");
        const bool extensions_off =
            extensions != nullptr &&
            std::holds_alternative<bool>(extensions->data) &&
            !std::get<bool>(extensions->data);
        if (!extensions_off)
            throw CheckError("target context '" + target +
                             "' must disable C++ extensions");
    }
    return context;
}

// ---- compilation database ----------------------------------------------------

namespace {

std::vector<std::string> parse_command(const Value &entry, std::size_t index) {
    const Value *arguments = member(entry, "arguments");
    const Value *command = member(entry, "command");
    std::optional<std::vector<std::string>> parsed_command;
    if (command != nullptr && std::holds_alternative<std::string>(command->data))
        parsed_command =
            shlex_split(std::get<std::string>(command->data),
                        "compilation database entry " + std::to_string(index));
    else if (command != nullptr)
        parsed_command = std::nullopt; // non-string command is ignored below
    std::vector<std::string> argv;
    if (arguments != nullptr) {
        if (!std::holds_alternative<Array>(arguments->data))
            throw CheckError("malformed arguments in compilation database entry " +
                             std::to_string(index));
        const auto &list = std::get<Array>(arguments->data);
        if (list.empty())
            throw CheckError("malformed arguments in compilation database entry " +
                             std::to_string(index));
        for (const Value &item: list)
            if (!std::holds_alternative<std::string>(item.data))
                throw CheckError(
                    "malformed arguments in compilation database entry " +
                    std::to_string(index));
        argv.reserve(list.size());
        for (const Value &item: list)
            argv.push_back(std::get<std::string>(item.data));
        if (parsed_command.has_value() && *parsed_command != argv)
            throw CheckError(
                "conflicting command and arguments in compilation database entry " +
                std::to_string(index));
    } else if (parsed_command.has_value()) {
        argv = *parsed_command;
    } else {
        throw CheckError("compilation database entry " + std::to_string(index) +
                         " needs command or arguments");
    }
    if (argv.empty() || argv.front().empty())
        throw CheckError("compilation database entry " + std::to_string(index) +
                         " has no compiler");
    return argv;
}

constexpr std::array<std::string_view, 12> kPathOptions{
    "-I", "-isystem", "-iquote", "-idirafter", "-isysroot",
    "--sysroot", "-L", "-F", "-iframework", "-include", "-imacros",
};

// NOLINTBEGIN(bugprone-easily-swappable-parameters) (directory, source) order mirrors the oracle signature
void check_path_arguments(const std::vector<std::string> &argv,
                          const fs::path &directory, const fs::path &source,
                          std::size_t index) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    std::size_t position = 1;
    while (position < argv.size()) {
        const std::string &argument = argv[position];
        std::optional<std::string> value;
        bool path_is_file = false;
        if (std::ranges::find(kPathOptions, argument) != kPathOptions.end()) {
            if (position + 1 >= argv.size())
                throw CheckError("missing path after " + argument +
                                 " in compilation database entry " +
                                 std::to_string(index));
            value = argv[position + 1];
            path_is_file = argument == "-include" || argument == "-imacros";
            position += 2;
        } else {
            for (const std::string_view prefix:
                 {"--sysroot=", "-isysroot", "-isystem", "-iquote",
                  "-idirafter", "-iframework", "-I", "-L", "-F"}) {
                if (argument.starts_with(prefix) && argument != prefix) {
                    value = argument.substr(prefix.size());
                    break;
                }
            }
            if (argument.starts_with("-fmodule-map-file=")) {
                value = argument.substr(std::string("-fmodule-map-file=").size());
                path_is_file = true;
            } else if (argument.starts_with("-resource-dir=")) {
                value = argument.substr(std::string("-resource-dir=").size());
            }
            position += 1;
        }
        if (value.has_value() && *value != "<built-in>" &&
            *value != "<command-line>") {
            const fs::path path = canonical_path(*value, &directory);
            if (path == source && path_is_file) continue;
            std::error_code ec;
            if (!fs::exists(path, ec) ||
                (path_is_file && !fs::is_regular_file(path, ec)))
                throw CheckError("missing dependency path in compilation "
                                 "database entry " +
                                 std::to_string(index) + ": " + path.string());
        }
    }
}

std::pair<fs::path, fs::path> source_for_entry(const Value &entry,
                                             std::size_t index) {
    const Value *directory_value = member(entry, "directory");
    const Value *file_value = member(entry, "file");
    const auto *directory_text = directory_value != nullptr
                                     ? std::get_if<std::string>(&directory_value->data)
                                     : nullptr;
    const auto *file_text = file_value != nullptr
                                ? std::get_if<std::string>(&file_value->data)
                                : nullptr;
    if (directory_text == nullptr || directory_text->empty())
        throw CheckError("compilation database entry " + std::to_string(index) +
                         " has no directory");
    if (file_text == nullptr || file_text->empty())
        throw CheckError("compilation database entry " + std::to_string(index) +
                         " has no source file");
    const fs::path directory = canonical_path(*directory_text);
    std::error_code ec;
    if (!fs::is_directory(directory, ec))
        throw CheckError("missing compilation directory in entry " +
                         std::to_string(index) + ": " + directory.string());
    return {directory, canonical_path(*file_text, &directory)};
}

std::optional<std::string> target_from_entry(const Value &entry,
                                             const std::vector<std::string> &argv) {
    static const std::regex target_pattern(
        R"((?:^|[/\\])CMakeFiles[/\\]([^/\\]+)\.dir(?:[/\\]|$))");
    std::vector<std::string> outputs;
    const Value *output = member(entry, "output");
    if (output != nullptr && std::holds_alternative<std::string>(output->data))
        outputs.push_back(std::get<std::string>(output->data));
    for (std::size_t index = 0; index + 1 < argv.size(); ++index)
        if (argv[index] == "-o") outputs.push_back(argv[index + 1]);
    for (const std::string &candidate: outputs) {
        std::smatch match;
        if (std::regex_search(candidate, match, target_pattern))
            return match[1].str();
    }
    return std::nullopt;
}

bool contains_source_argument(const std::vector<std::string> &argv,
                              const fs::path &directory,
                              const fs::path &source) {
    for (std::size_t index = 1; index < argv.size(); ++index) {
        const std::string &argument = argv[index];
        if (argument.starts_with('-')) continue;
        if (canonical_path(argument, &directory) == source) return true;
    }
    return false;
}

std::optional<fs::path> resolve_command_executable(const std::string &value,
                                                   const fs::path &directory) {
    if (value.find('/') != std::string::npos) {
        const fs::path candidate = canonical_path(value, &directory);
        std::error_code ec;
        return fs::is_regular_file(candidate, ec)
                   ? std::optional<fs::path>(candidate)
                   : std::nullopt;
    }
    const std::optional<fs::path> located = which(value);
    if (!located.has_value()) return std::nullopt;
    return canonical_path(*located);
}

void validate_compilation_context(const std::vector<std::string> &argv,
                                  const fs::path &directory,
                                  const Value &context, std::size_t index) {
    const Value *compiler = member(context, "compiler");
    const fs::path configured_compiler =
        canonical_path(std::get<std::string>(member(*compiler, "path")->data));
    std::vector<std::string> launcher;
    if (const Value *launcher_value = member(*compiler, "launcher")) {
        if (const auto *text =
                std::get_if<std::string>(&launcher_value->data)) {
            if (!text->empty()) launcher.push_back(*text);
        } else if (const auto *array =
                       std::get_if<Array>(&launcher_value->data)) {
            for (const Value &item: *array) {
                if (!std::holds_alternative<std::string>(item.data))
                    throw CheckError(
                        "configured compiler launcher must be a string list");
                launcher.push_back(std::get<std::string>(item.data));
            }
        } else {
            throw CheckError(
                "configured compiler launcher must be a string list");
        }
    }
    std::size_t compiler_index = 0;
    if (!launcher.empty()) {
        const auto configured_launcher =
            resolve_command_executable(launcher.front(), directory);
        const auto selected_launcher =
            resolve_command_executable(argv.front(), directory);
        if (!configured_launcher.has_value() ||
            selected_launcher != configured_launcher)
            throw CheckError("compilation database launcher differs from "
                             "configured C++ compiler in entry " +
                             std::to_string(index));
        compiler_index = launcher.size();
    }
    const std::optional<fs::path> selected_compiler =
        compiler_index < argv.size()
            ? resolve_command_executable(argv[compiler_index], directory)
            : std::nullopt;
    if (selected_compiler != configured_compiler)
        throw CheckError(
            "compilation database compiler differs from configured C++ "
            "compiler in entry " +
            std::to_string(index) + ": selected=" +
            (selected_compiler.has_value() ? selected_compiler->string()
                                           : std::string("None")) +
            ", configured=" + configured_compiler.string());

    std::vector<fs::path> selected_sysroots;
    std::size_t position = 1;
    while (position < argv.size()) {
        const std::string &argument = argv[position];
        if (argument == "-isysroot" || argument == "--sysroot") {
            if (position + 1 >= argv.size())
                throw CheckError("missing SDK path in compilation database entry " +
                                 std::to_string(index));
            selected_sysroots.push_back(canonical_path(argv[position + 1], &directory));
            position += 2;
        } else if (argument.starts_with("--sysroot=")) {
            selected_sysroots.push_back(
                canonical_path(argument.substr(std::string("--sysroot=").size()),
                          &directory));
            position += 1;
        } else if (argument.starts_with("-isysroot") && argument != "-isysroot") {
            selected_sysroots.push_back(canonical_path(
                argument.substr(std::string("-isysroot").size()), &directory));
            position += 1;
        } else {
            position += 1;
        }
    }
    const fs::path configured_sdk =
        canonical_path(std::get<std::string>(member(context, "sdk")->data));
    const bool mismatch =
        selected_sysroots.empty() ||
        std::ranges::any_of(selected_sysroots,
                    [&](const fs::path &path) { return path != configured_sdk; });
    if (mismatch) {
        std::string selected;
        for (const fs::path &path: selected_sysroots) {
            if (!selected.empty()) selected += ", ";
            selected += path.string();
        }
        throw CheckError("compilation database sysroot differs from configured "
                         "SDK in entry " +
                         std::to_string(index) + ": selected=[" + selected +
                         "], configured=" + configured_sdk.string());
    }
}

// (str(source), target, normalized argv minus -o/-c and the source argument)
struct CompileIdentity {
    std::string source;
    std::optional<std::string> target;
    std::vector<std::string> arguments;
    bool operator==(const CompileIdentity &other) const = default;
    auto operator<=>(const CompileIdentity &other) const = default;
};

CompileIdentity compile_identity(const fs::path &source,
                                 const std::optional<std::string> &target,
                                 const std::vector<std::string> &argv,
                                 const fs::path &directory) {
    CompileIdentity identity{.source=source.string(), .target=target, .arguments={}};
    std::size_t position = 0;
    while (position < argv.size()) {
        const std::string &argument = argv[position];
        if (argument == "-o") {
            position += 2;
            continue;
        }
        if (argument == "-c") {
            position += 1;
            continue;
        }
        if (position > 0 && !argument.starts_with('-')) {
            if (canonical_path(argument, &directory) == source) {
                position += 1;
                continue;
            }
        }
        identity.arguments.push_back(argument);
        position += 1;
    }
    return identity;
}

void validate_target_context(const std::vector<std::string> &argv,
                             const std::string &target, const Value &context,
                             const fs::path &directory, std::size_t index) {
    const Value *target_contexts = member(context, "target_contexts");
    const Value *target_context =
        target_contexts != nullptr ? member(*target_contexts, target) : nullptr;
    if (target_context == nullptr ||
        !std::holds_alternative<Object>(target_context->data))
        return;
    const auto contains = [&argv](const std::string &flag) {
        return std::ranges::find(argv, flag) != argv.end();
    };
    if (!contains("-std=c++23") || contains("-std=gnu++23"))
        throw CheckError("compilation database entry " + std::to_string(index) +
                         " for target " + target + " is not strict ISO C++23");
    const auto entry_message = [](std::size_t entry_index,
                                  const std::string &entry_target) {
        std::string message = "compilation database entry ";
        message += std::to_string(entry_index);
        message += " omits target ";
        message += entry_target;
        return message;
    };
    for (const std::string &expected:
         member_string_list(*target_context, "compile_options")) {
        if (expected.find("$<") == std::string::npos && !contains(expected)) {
            std::string message = entry_message(index, target);
            message += " compile option '";
            message += expected;
            message += '\'';
            throw CheckError(message);
        }
    }
    for (const std::string &expected:
         member_string_list(*target_context, "compile_definitions")) {
        if (expected.find("$<") != std::string::npos) continue;
        const std::string flag = "-D" + expected;
        bool present = contains(flag);
        for (std::size_t position = 0; !present && position < argv.size();
             ++position)
            if (argv[position] == "-D" && position + 1 < argv.size() &&
                argv[position + 1] == expected)
                present = true;
        if (!present) {
            std::string message = entry_message(index, target);
            message += " definition '";
            message += expected;
            message += '\'';
            throw CheckError(message);
        }
    }
    for (const std::string &expected:
         member_string_list(*target_context, "include_directories")) {
        if (expected.find("$<") != std::string::npos) continue;
        const std::string resolved = canonical_path(expected).string();
        bool present = contains(resolved);
        for (std::size_t position = 0; !present && position < argv.size();
             ++position) {
            if ((argv[position] == "-I" || argv[position] == "-isystem" ||
                 argv[position] == "-iquote" || argv[position] == "-idirafter") &&
                position + 1 < argv.size() &&
                canonical_path(argv[position + 1], &directory).string() == resolved)
                present = true;
        }
        if (!present) {
            for (const std::string &argument: argv) {
                for (const std::string_view prefix:
                     {"-isystem", "-iquote", "-idirafter", "-I"}) {
                    if (argument.starts_with(prefix) && argument != prefix &&
                        canonical_path(argument.substr(prefix.size()), &directory)
                                .string() == resolved) {
                        present = true;
                        break;
                    }
                }
                if (present) break;
            }
        }
        if (!present) {
            std::string message = entry_message(index, target);
            message += " include ";
            message += resolved;
            throw CheckError(message);
        }
    }
}

} // namespace

SourceManifest load_source_manifest(const fs::path &build) {
    const fs::path build_dir = canonical_path(build);
    SourceManifest manifest{.document=read_json(build_dir / "native_sources.json",
                                    "native source manifest"),
                            .first_party_roots={},
                            .sources={}};
    if (!std::holds_alternative<Object>(manifest.document.data))
        throw CheckError("native source manifest has an unsupported schema");
    const Value *schema = member(manifest.document, "schema_version");
    const bool schema_one =
        schema != nullptr &&
        std::holds_alternative<io::Number>(schema->data) &&
        std::get<io::Number>(schema->data).lexeme == "1";
    if (!schema_one)
        throw CheckError("native source manifest has an unsupported schema");
    const Value *roots_value = member(manifest.document, "first_party_roots");
    const Value *sources_value = member(manifest.document, "sources");
    const bool valid_roots =
        roots_value != nullptr &&
        std::holds_alternative<Array>(roots_value->data) &&
        !std::get<Array>(roots_value->data).empty() &&
        std::all_of(
            std::get<Array>(roots_value->data).begin(),
            std::get<Array>(roots_value->data).end(),
            [](const Value &item) {
                return std::holds_alternative<std::string>(item.data);
            });
    if (!valid_roots)
        throw CheckError("native source manifest has no valid first-party roots");
    if (sources_value == nullptr ||
        !std::holds_alternative<Array>(sources_value->data) ||
        std::get<Array>(sources_value->data).empty())
        throw CheckError(
            "native source manifest has an empty expected translation-unit set");
    for (const Value &root: std::get<Array>(roots_value->data))
        manifest.first_party_roots.push_back(
            canonical_path(std::get<std::string>(root.data)));
    for (const fs::path &root: manifest.first_party_roots) {
        std::error_code ec;
        if (!fs::is_directory(root, ec))
            throw CheckError("missing first-party source root: " + root.string());
    }
    for (const Value &item: std::get<Array>(sources_value->data)) {
        const Value *path = member(item, "path");
        if (path != nullptr && std::holds_alternative<std::string>(path->data))
            manifest.sources.push_back(
                canonical_path(std::get<std::string>(path->data)));
    }
    return manifest;
}

std::vector<CompileEntry> load_entries(const fs::path &build,
                                     const std::vector<fs::path> &expected) {
    const fs::path build_dir = canonical_path(build);
    std::error_code ec;
    if (!fs::is_directory(build_dir, ec))
        throw CheckError("build directory does not exist: " + build_dir.string());
    const Value context = load_context(build_dir);
    const SourceManifest manifest = load_source_manifest(build_dir);
    const auto &roots = manifest.first_party_roots;

    struct ExpectedContext {
        std::string target;
        std::string context;
    };
    std::map<std::string, std::vector<ExpectedContext>> expected_by_source;
    struct ManifestIdentity {
        std::string source;
        std::string target;
        std::string context;
        bool operator<(const ManifestIdentity &other) const {
            return std::tie(source, target, context) <
                   std::tie(other.source, other.target, other.context);
        }
    };
    std::set<ManifestIdentity> manifest_identities;
    const auto *sources_value =
        std::get_if<Array>(&member(manifest.document, "sources")->data);
    std::size_t manifest_index = 0;
    for (const Value &item: *sources_value) {
        if (!std::holds_alternative<Object>(item.data))
            throw CheckError("malformed native source manifest entry " +
                             std::to_string(manifest_index));
        const std::string source_value = member_string(item, "path");
        const std::string target = member_string(item, "target");
        const std::string compile_context = member_string(item, "context");
        if (source_value.empty() || target.empty() || compile_context.empty())
            throw CheckError("malformed native source manifest entry " +
                             std::to_string(manifest_index));
        const fs::path source = canonical_path(source_value);
        if (!is_under(source, roots))
            throw CheckError("manifest source is outside first-party roots: " +
                             source.string());
        if (!fs::is_regular_file(source, ec))
            throw CheckError("missing source file in native source manifest: " +
                             source.string());
        ManifestIdentity identity{.source=source.string(), .target=target, .context=compile_context};
        if (!manifest_identities.insert(std::move(identity)).second)
            throw CheckError(
                "duplicate source/target context in native source manifest: " +
                source.string() + " (" + target + ")");
        expected_by_source[source.string()].push_back({.target=target, .context=compile_context});
        ++manifest_index;
    }

    const Value *target_contexts = member(context, "target_contexts");
    const bool have_target_contexts =
        target_contexts != nullptr &&
        std::holds_alternative<Object>(target_contexts->data) &&
        !std::get<Object>(target_contexts->data).empty();
    std::set<std::string> expected_targets;
    for (const auto &[source, contexts]: expected_by_source)
        for (const auto &entry: contexts) expected_targets.insert(entry.target);
    std::set<std::string> missing_target_contexts;
    for (const std::string &target: expected_targets)
        if (target_contexts == nullptr ||
            member(*target_contexts, target) == nullptr)
            missing_target_contexts.insert(target);
    if (!missing_target_contexts.empty()) {
        std::string joined;
        for (const std::string &target: missing_target_contexts) {
            if (!joined.empty()) joined += ", ";
            joined += target;
        }
        throw CheckError(
            "native check context is missing CMake target contexts: " + joined);
    }

    std::set<std::string> expected_sources;
    for (const fs::path &path: expected)
        expected_sources.insert(canonical_path(path).string());
    if (expected_sources.empty())
        throw CheckError("expected first-party translation-unit set is empty");
    std::set<std::string> manifest_source_keys;
    for (const auto &[source, _]: expected_by_source)
        manifest_source_keys.insert(source);
    if (expected_sources != manifest_source_keys) {
        std::set<std::string> missing;
        for (const std::string &source: expected_sources)
            if (!manifest_source_keys.contains(source)) missing.insert(source);
        std::set<std::string> unexpected;
        for (const std::string &source: manifest_source_keys)
            if (!expected_sources.contains(source)) unexpected.insert(source);
        const auto join = [](const std::set<std::string> &items) {
            std::string out;
            for (const auto &item: items) {
                if (!out.empty()) out += ", ";
                out += item;
            }
            return out;
        };
        throw CheckError(
            "expected source set differs from CMake manifest; missing=[" +
            join(missing) + "]; unexpected=[" + join(unexpected) + "]");
    }

    const Value database =
        read_json(build_dir / "compile_commands.json", "compilation database");
    if (!std::holds_alternative<Array>(database.data))
        throw CheckError("compilation database must be a JSON array");
    const auto &database_entries = std::get<Array>(database.data);
    if (database_entries.empty())
        throw CheckError(
            "compilation database has an empty translation-unit selection");

    std::map<std::string, std::vector<CompileEntry>> actual_by_source;
    std::set<CompileIdentity> identities;
    std::size_t index = 0;
    for (const Value &raw_entry: database_entries) {
        if (!std::holds_alternative<Object>(raw_entry.data))
            throw CheckError("malformed compilation database entry " +
                             std::to_string(index));
        const auto [directory, source] = source_for_entry(raw_entry, index);
        const std::vector<std::string> argv = parse_command(raw_entry, index);
        if (!contains_source_argument(argv, directory, source))
            throw CheckError("compilation database entry " +
                             std::to_string(index) + " command does not compile " +
                             source.string());
        if (!is_under(source, roots)) {
            ++index;
            continue;
        }
        if (!expected_by_source.contains(source.string()))
            throw CheckError(
                "unexpected first-party translation unit in compilation "
                "database: " +
                source.string());
        if (!fs::is_regular_file(source, ec))
            throw CheckError(
                "missing source file referenced by compilation database: " +
                source.string());
        validate_compilation_context(argv, directory, context, index);
        check_path_arguments(argv, directory, source, index);

        const std::optional<std::string> target =
            target_from_entry(raw_entry, argv);
        if (have_target_contexts && !target.has_value())
            throw CheckError("target context is unavailable in compilation "
                             "database entry " +
                             std::to_string(index) + " for " + source.string());
        if (target.has_value()) {
            const auto &expected_contexts = expected_by_source[source.string()];
            const bool allowed =
                std::ranges::any_of(expected_contexts,
                            [&](const ExpectedContext &item) {
                                return item.target == *target;
                            });
            if (!allowed) {
                std::string allowed_list;
                std::set<std::string> sorted;
                for (const auto &item: expected_contexts)
                    sorted.insert(item.target);
                for (const auto &item: sorted) {
                    if (!allowed_list.empty()) allowed_list += ", ";
                    allowed_list += item;
                }
                throw CheckError("unexpected target context for " +
                                 source.string() + ": " + *target +
                                 "; expected [" + allowed_list + "]");
            }
            validate_target_context(argv, *target, context, directory, index);
        }

        const CompileIdentity identity =
            compile_identity(source, target, argv, directory);
        if (!identities.insert(identity).second)
            throw CheckError(
                "duplicate identical translation unit in compilation database: " +
                source.string());
        CompileEntry entry;
        entry.source = source;
        entry.directory = directory;
        entry.arguments = argv;
        entry.target = target;
        entry.raw = raw_entry;
        actual_by_source[source.string()].push_back(std::move(entry));
        ++index;
    }

    std::vector<std::string> missing_sources;
    for (const auto &[source, _]: expected_by_source)
        if (!actual_by_source.contains(source))
            missing_sources.push_back(fs::path(source).filename().string());
    if (!missing_sources.empty()) {
        std::string joined;
        for (const auto &name: missing_sources) {
            if (!joined.empty()) joined += ", ";
            joined += name;
        }
        throw CheckError("missing expected translation units: " + joined);
    }

    for (const auto &[source_key, expected_contexts]: expected_by_source) {
        const auto found = actual_by_source.find(source_key);
        const std::size_t actual_count =
            found == actual_by_source.end() ? 0 : found->second.size();
        if (actual_count != expected_contexts.size())
            throw CheckError("translation-unit context count mismatch for " +
                             source_key + ": expected " +
                             std::to_string(expected_contexts.size()) +
                             ", selected " + std::to_string(actual_count));
        std::set<std::string> known_targets;
        for (const auto &item: expected_contexts)
            known_targets.insert(item.target);
        std::vector<std::string> selected_targets;
        if (found != actual_by_source.end())
            for (const auto &entry: found->second)
                if (entry.target.has_value()) selected_targets.push_back(*entry.target);
        if (have_target_contexts &&
            (selected_targets.size() != actual_count ||
             std::set<std::string>(selected_targets.begin(),
                                   selected_targets.end()) != known_targets)) {
            const auto join = [](const std::set<std::string> &items) {
                std::string out;
                for (const auto &item: items) {
                    if (!out.empty()) out += ", ";
                    out += item;
                }
                return out;
            };
            throw CheckError("selected target contexts differ for " + source_key +
                             ": expected [" + join(known_targets) +
                             "], selected [" +
                             join(std::set<std::string>(selected_targets.begin(),
                                                       selected_targets.end())) +
                             "]");
        }
    }

    std::vector<CompileEntry> entries;
    for (const auto &[source_key, grouped]: actual_by_source)
        for (const auto &entry: grouped) entries.push_back(entry);
    if (entries.empty())
        throw CheckError("first-party translation-unit selection is empty");
    return entries;
}

std::pair<fs::path, std::string> resolve_tool(
    const std::string &kind, const Value &context,
    const std::map<std::string, std::string> &environment) {
    static const std::map<std::string, std::pair<std::string, std::string>>
        configuration = {
            {"tidy", {"CLANG_TIDY", "tidy"}},
            {"analyzer", {"ANALYZER_CLANG", "analyzer"}},
            {"cppcheck", {"CPPCHECK", "cppcheck"}},
            {"frontends", {"GXX", "gxx"}},
            {"odr", {"GXX", "gxx"}},
        };
    const auto found = configuration.find(kind);
    if (found == configuration.end())
        throw CheckError("unknown native sweep kind: " + kind);
    const auto &[override_name, default_key] = found->second;
    std::string requested;
    if (const auto env = environment.find(override_name);
        env != environment.end() && !env->second.empty())
        requested = env->second;
    if (requested.empty())
        requested = member_string(*member(context, "tool_overrides"), override_name);
    if (requested.empty())
        requested = member_string(*member(context, "tool_defaults"), default_key);
    if (requested.empty())
        throw CheckError("no executable configured for " + kind + "; set " +
                         override_name);
    fs::path executable;
    if (requested.find('/') != std::string::npos) {
        executable = canonical_path(expand_home(requested));
        if (!is_executable_file(executable))
            throw CheckError("required tool is missing or not executable: " +
                             requested);
    } else {
        const std::optional<fs::path> located = which(requested);
        if (!located.has_value())
            throw CheckError("required tool is unavailable: " + requested +
                             " (set " + override_name + ")");
        executable = canonical_path(*located);
    }
    return {executable, override_name};
}

fs::path temporary_database(const fs::path &build, const std::string &kind,
                            const std::string &run_id, std::size_t index,
                            const CompileEntry &entry) {
    std::ostringstream directory_name;
    directory_name << "entry-" << std::setfill('0') << std::setw(4) << index;
    const fs::path database_dir =
        build / "native_check_work" / kind / run_id / directory_name.str();
    std::error_code ec;
    fs::create_directories(database_dir, ec);
    Array arguments;
    arguments.reserve(entry.arguments.size());
    for (const auto &argument: entry.arguments)
        arguments.push_back(vstr(argument));
    Object database_entry{
        {"directory", vstr(entry.directory.string())},
        {"file", vstr(entry.source.string())},
        {"arguments", varr(std::move(arguments))},
    };
    if (const Value *output = member(entry.raw, "output");
        output != nullptr && std::holds_alternative<std::string>(output->data))
        database_entry.emplace_back("output",
                                    vstr(std::get<std::string>(output->data)));
    const Value document = varr({vobj(std::move(database_entry))});
    std::ofstream stream(database_dir / "compile_commands.json",
                         std::ios::binary | std::ios::trunc);
    stream << dump_json(document) << "\n";
    return database_dir;
}

std::vector<std::string> replace_compile_source(const CompileEntry &entry,
                                                const fs::path &replacement,
                                                bool syntax_only) {
    std::vector<std::string> output;
    std::size_t position = 0;
    const auto &argv = entry.arguments;
    while (position < argv.size()) {
        const std::string &argument = argv[position];
        if (argument == "-o" || argument == "-MF" || argument == "-MT" ||
            argument == "-MQ") {
            position += 2;
            continue;
        }
        if (argument == "-c" || argument == "-MD" || argument == "-MMD" ||
            argument == "-MP") {
            position += 1;
            continue;
        }
        if (!argument.starts_with('-') &&
            canonical_path(argument, &entry.directory) == entry.source) {
            output.push_back(replacement.string());
            position += 1;
            continue;
        }
        output.push_back(argument);
        position += 1;
    }
    if (std::ranges::find(output, replacement.string()) ==
        output.end())
        throw CheckError("could not replace translation unit " +
                         entry.source.string() + " in compile context");
    if (syntax_only) output.emplace_back("-fsyntax-only");
    return output;
}

std::vector<fs::path> standalone_header_roots(
    const std::vector<fs::path> &first_party_roots) {
    std::vector<fs::path> roots;
    for (const fs::path &first_party_root: first_party_roots) {
        for (const std::string_view relative: {"src", "tests"}) {
            const fs::path candidate = first_party_root / relative;
            std::error_code ec;
            if (fs::is_directory(candidate, ec))
                roots.push_back(canonical_path(candidate));
        }
        const fs::path benchmark_headers =
            first_party_root.parent_path() / "tools" / "proto_native_bench";
        std::error_code ec;
        if (fs::is_directory(benchmark_headers, ec))
            roots.push_back(canonical_path(benchmark_headers));
    }
    return roots;
}

std::vector<std::string> gcc_context_args(const CompileEntry &entry,
                                          const Value &context) {
    const auto &argv = entry.arguments;
    const fs::path &directory = entry.directory;
    const fs::path &source = entry.source;
    std::vector<std::string> output;
    static const std::set<std::string> accepted_pairs = {
        "-I", "-isystem", "-iquote", "-idirafter",
        "-isysroot", "-D", "-U", "-include",
    };
    static const std::vector<std::string> accepted_prefixes = {
        "-I", "-D", "-U", "-isystem", "-isysroot", "-iquote", "-idirafter",
    };
    std::size_t position = 1;
    while (position < argv.size()) {
        const std::string &argument = argv[position];
        if (argument == "-o" || argument == "-MF" || argument == "-MT" ||
            argument == "-MQ") {
            position += 2;
            continue;
        }
        if (argument == "-c" || argument == "-MD" || argument == "-MMD" ||
            argument == "-MP" || argument.starts_with("-W")) {
            position += 1;
            continue;
        }
        if (!argument.starts_with('-') &&
            canonical_path(argument, &directory) == source) {
            position += 1;
            continue;
        }
        if (accepted_pairs.contains(argument) && position + 1 < argv.size()) {
            output.push_back(argument);
            output.push_back(argv[position + 1]);
            position += 2;
            continue;
        }
        bool prefixed = false;
        for (const std::string &prefix: accepted_prefixes) {
            if (argument.starts_with(prefix) && argument != prefix) {
                output.push_back(argument);
                prefixed = true;
                break;
            }
        }
        if (prefixed) {
            position += 1;
            continue;
        }
        static const std::set<std::string> accepted_flags = {
            "-pthread", "-fPIC", "-fpic", "-ffp-contract=off",
            "-fvisibility=hidden", "-fvisibility-inlines-hidden",
        };
        if (accepted_flags.contains(argument)) output.push_back(argument);
        position += 1;
    }
    if (const Value *include_paths = member(context, "include_paths"))
        for (const Value &item: std::get<Array>(include_paths->data)) {
            const std::string &include = std::get<std::string>(item.data);
            if (std::ranges::find(output, include) == output.end()) {
                output.emplace_back("-isystem");
                output.push_back(include);
            }
        }
    return output;
}

} // namespace bike::checks
