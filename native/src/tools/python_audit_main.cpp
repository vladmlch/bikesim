// python_audit_main.cpp — bike-python-audit (produced by T56, enforced by
// T62). Three modes per the T62 contract:
//
//   bike-python-audit --root .            --mode source
//   bike-python-audit --root install-test --mode package
//   bike-python-audit --root .            --mode documentation
//
//   source        — first-party executable Python files/invocations/imports/
//                   build edges. During transition this reports remaining
//                   owned paths from the removal ledger plus interpreter
//                   invocations in first-party shell/CMake/CI surfaces; it
//                   exits nonzero while any remain and passes after T63.
//   package       — binaries, dynamic dependencies, shebangs, bytecode,
//                   wheels and venv markers under the audited root.
//   documentation — active fenced command blocks in live documentation;
//                   historical prose paths are exempt via policy.
//
// Every hit is reported as "<path>: <edge detail>" on stderr with the owning
// remediation task when known. No hit is ever silently absorbed.
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

#include "../io/json.hpp"
#include "../io/value.hpp"
#include "../platform/process.hpp"

namespace {

namespace fs = std::filesystem;
using bike::io::Value;
using Object = Value::Object;
using Array = Value::Array;

struct AuditError : std::runtime_error {
    explicit AuditError(const std::string &message)
        : std::runtime_error(message) {}
    AuditError(const AuditError &) = default;
    AuditError &operator=(const AuditError &) = default;
    AuditError(AuditError &&) = default;
    AuditError &operator=(AuditError &&) = default;
    ~AuditError() override; // out-of-line key function — no weak vtable
};
AuditError::~AuditError() = default;

const Value *member(const Value &object, const std::string &key) {
    const auto *map = std::get_if<Object>(&object.data);
    if (map == nullptr) return nullptr;
    for (const auto &[name, value]: *map)
        if (name == key) return &value;
    return nullptr;
}

std::vector<std::string> string_list(const Value &object,
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

std::optional<std::string> opt_string(const Value &object,
                                    const std::string &key) {
    const Value *value = member(object, key);
    if (value == nullptr) return std::nullopt;
    if (const auto *text = std::get_if<std::string>(&value->data))
        return *text;
    return std::nullopt;
}

Value read_json(const fs::path &path, const std::string &label) {
    std::ifstream const stream(path, std::ios::binary);
    if (!stream.is_open())
        throw AuditError("missing " + label + ": " + path.string());
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    try {
        return bike::io::parse_json(buffer.str());
    } catch (const std::exception &error) {
        throw AuditError("malformed " + label + ": " + path.string() + ": " +
                         error.what());
    }
}

// fnmatch-style globbing for the policy's exempt/match lists (same engine
// the check orchestrator shares; local copy keeps this binary independent
// of the checks translation units).
std::string glob_to_regex(const std::string &pattern) {
    std::string out = "^";
    std::size_t index = 0;
    while (index < pattern.size()) {
        const char ch = pattern[index];
        if (ch == '*') {
            if (index + 1 < pattern.size() && pattern[index + 1] == '*') {
                out += ".*";
                index += 2;
                continue;
            }
            out += "[^/]*";
            ++index;
            continue;
        }
        if (ch == '?') {
            out += "[^/]";
            ++index;
            continue;
        }
        if (ch == '[') {
            const std::size_t close = pattern.find(']', index + 1);
            if (close == std::string::npos) {
                out += "\\[";
                ++index;
                continue;
            }
            std::string_view body =
                std::string_view(pattern).substr(index + 1, close - index - 1);
            std::string klass = "[";
            if (!body.empty() && body.front() == '!') {
                klass += '^';
                body.remove_prefix(1);
            }
            for (const char item: body) {
                if (item == ']' || item == '\\' || item == '[' ||
                    item == '^')
                    klass += '\\';
                klass += item;
            }
            klass += ']';
            out += klass;
            index = close + 1;
            continue;
        }
        if (std::string("\\^$.|(){}+").find(ch) != std::string::npos)
            out += '\\';
        out += ch;
        ++index;
    }
    out += '$';
    return out;
}

bool glob_match(const std::string &value, const std::string &pattern) {
    try {
        return std::regex_match(value, std::regex(glob_to_regex(pattern)));
    } catch (const std::regex_error &) {
        return false;
    }
}

bool any_glob(const std::vector<std::string> &patterns,
              const std::string &relative) {
    for (const auto &pattern: patterns)
        if (glob_match(relative, pattern) ||
            (!pattern.empty() && pattern.back() == '/' &&
             relative.starts_with(pattern)))
            return true;
    return false;
}

struct Finding {
    std::string path;    // repo/scan-relative where possible
    std::string detail;  // the offending edge/artifact + remediation owner
};

std::vector<fs::path> walk_files(const fs::path &root) {
    std::vector<fs::path> files;
    std::error_code ec;
    if (!fs::is_directory(root, ec)) return files;
    for (fs::recursive_directory_iterator it(
             root, fs::directory_options::skip_permission_denied, ec),
         end;
         !ec && it != end; it.increment(ec)) {
        std::error_code file_ec;
        if (it->is_regular_file(file_ec) && !it->is_symlink(file_ec))
            files.push_back(it->path());
    }
    std::ranges::sort(files);
    return files;
}

std::vector<fs::path> walk_directories(const fs::path &root) {
    std::vector<fs::path> dirs;
    std::error_code ec;
    for (fs::recursive_directory_iterator it(
             root, fs::directory_options::skip_permission_denied, ec),
         end;
         !ec && it != end; it.increment(ec)) {
        std::error_code dir_ec;
        if (it->is_directory(dir_ec)) dirs.push_back(it->path());
    }
    return dirs;
}

std::string relative_to(const fs::path &path, const fs::path &root) {
    std::error_code ec;
    const fs::path relative = fs::relative(path, root, ec);
    return ec ? path.string() : relative.generic_string();
}

std::string read_head(const fs::path &path, std::size_t limit) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream.is_open()) return {};
    std::string head(static_cast<std::size_t>(limit), '\0');
    stream.read(head.data(), static_cast<std::streamsize>(limit));
    head.resize(static_cast<std::size_t>(stream.gcount()));
    return head;
}

// Mach-O magic bytes (thin 32/64, fat, fat-64) in wire order.
bool is_mach_o(const std::string &head) {
    if (head.size() < 4) return false;
    const auto b = [&](std::size_t i) {
        return static_cast<unsigned char>(head.at(i));
    };
    const std::uint32_t be = (static_cast<std::uint32_t>(b(0)) << 24U) |
                             (static_cast<std::uint32_t>(b(1)) << 16U) |
                             (static_cast<std::uint32_t>(b(2)) << 8U) |
                             static_cast<std::uint32_t>(b(3));
    const std::uint32_t le = static_cast<std::uint32_t>(b(0)) |
                             (static_cast<std::uint32_t>(b(1)) << 8U) |
                             (static_cast<std::uint32_t>(b(2)) << 16U) |
                             (static_cast<std::uint32_t>(b(3)) << 24U);
    return be == 0xFEEDFACE || be == 0xFEEDFACF || be == 0xCAFEBABE ||
           be == 0xCAFEBABF || le == 0xFEEDFACE || le == 0xFEEDFACF;
}

bool is_elf(const std::string &head) {
    return head.size() >= 4 && head[0] == 0x7F && head[1] == 'E' &&
           head[2] == 'L' && head[3] == 'F';
}

// ---- mode: source ---------------------------------------------------------

int audit_source(const fs::path &root, const Value &policy) {
    std::vector<Finding> findings;
    const Value *source_policy = member(policy, "source_mode");
    const auto inventory_path =
        opt_string(policy, "removal_inventory")
            .value_or("migration/python-removal.json");

    // Ledger rows whose path still exists are remaining owned Python files.
    {
        const fs::path inventory = root / inventory_path;
        std::error_code ec;
        if (fs::is_regular_file(inventory, ec)) {
            const Value ledger = read_json(inventory, "removal inventory");
            if (const auto *rows = std::get_if<Array>(&ledger.data)) {
                for (const Value &row: *rows) {
                    const auto path = opt_string(row, "path");
                    const auto owner = opt_string(row, "owner_task")
                                           .value_or("unassigned");
                    if (!path.has_value()) continue;
                    if (fs::exists(root / *path, ec))
                        findings.push_back(
                            {.path=*path,
                             .detail="tracked Python source still present "
                             "(owner: " +
                                 owner + ")"});
                }
            }
        }
    }

    // Interpreter invocation edges in first-party executable surfaces.
    const std::vector<std::string> invocation_globs =
        string_list(source_policy != nullptr ? *source_policy : Value{},
                    "invocation_globs");
    const std::vector<std::string> invocation_tokens =
        string_list(source_policy != nullptr ? *source_policy : Value{},
                    "invocation_tokens");
    const std::vector<std::string> exempt =
        string_list(source_policy != nullptr ? *source_policy : Value{},
                    "exempt_globs");
    const auto files = walk_files(root);
    for (const fs::path &file: files) {
        const std::string relative = relative_to(file, root);
        if (any_glob(exempt, relative)) continue;
        if (!any_glob(invocation_globs, relative)) continue;
        std::ifstream stream(file, std::ios::binary);
        if (!stream.is_open()) continue;
        std::string line;
        std::size_t line_number = 0;
        while (std::getline(stream, line)) {
            ++line_number;
            // Whole-line comments cannot hold an invocation edge.
            const auto first = line.find_first_not_of(" \t");
            if (first != std::string::npos && line[first] == '#') continue;
            for (const auto &token: invocation_tokens) {
                if (token.empty()) continue;
                static const std::string boundary_chars =
                    " \t\"'()$`|;&<>,={}[]";
                std::size_t position = 0;
                while ((position = line.find(token, position)) !=
                       std::string::npos) {
                    const bool left_ok =
                        position == 0 ||
                        boundary_chars.find(line.at(position - 1)) !=
                            std::string::npos ||
                        line.at(position - 1) == '/';
                    const std::size_t after = position + token.size();
                    const bool right_ok =
                        after >= line.size() ||
                        boundary_chars.find(line.at(after)) !=
                            std::string::npos;
                    if (left_ok && right_ok) {
                        findings.push_back(
                            {.path=relative + ":" + std::to_string(line_number),
                             .detail="interpreter invocation token '" + token +
                                 "': " +
                                 line.substr(
                                     first == std::string::npos ? 0 : first)});
                        break;
                    }
                    position = after;
                }
            }
        }
    }

    if (findings.empty()) {
        std::cout << "python source audit clean: " << root.string() << "\n";
        return 0;
    }
    for (const Finding &finding: findings)
        std::cerr << "python source audit: " << finding.path << " — "
                  << finding.detail << "\n";
    std::cerr << "python source audit: " << findings.size()
              << " remaining edge(s) under " << root.string() << "\n";
    return 1;
}

// ---- mode: package ---------------------------------------------------------

int audit_package(const fs::path &root, const Value &policy) {
    const Value *package_policy = member(policy, "package_mode");
    const Value empty_policy = Value{Object{}};
    const Value &resolved =
        package_policy != nullptr ? *package_policy : empty_policy;
    const std::vector<std::string> forbidden_files =
        string_list(resolved, "forbidden_file_globs");
    const std::vector<std::string> forbidden_dirs =
        string_list(resolved, "forbidden_directory_names");
    const std::vector<std::string> forbidden_dir_globs =
        string_list(resolved, "forbidden_directory_globs");
    const std::vector<std::string> shebang_tokens =
        string_list(resolved, "shebang_tokens");
    const std::vector<std::string> forbidden_loads =
        string_list(resolved, "mach_o_forbidden_loads");
    const std::vector<std::string> exempt =
        string_list(resolved, "exempt_globs");

    std::vector<Finding> findings;
    for (const fs::path &directory: walk_directories(root)) {
        const std::string name = directory.filename().string();
        const std::string relative = relative_to(directory, root);
        if (any_glob(exempt, relative)) continue;
        if (std::ranges::find(forbidden_dirs, name) !=
            forbidden_dirs.end())
            findings.push_back(
                {.path=relative, .detail="virtual-environment/package directory '" + name +
                               "' present"});
        else if (any_glob(forbidden_dir_globs, name))
            findings.push_back({.path=relative,
                                .detail="installed-package metadata directory '" +
                                    name + "' present"});
    }
    for (const fs::path &file: walk_files(root)) {
        const std::string relative = relative_to(file, root);
        if (any_glob(exempt, relative)) continue;
        const std::string name = file.filename().string();
        bool flagged = false;
        for (const auto &glob: forbidden_files) {
            if (glob_match(name, glob)) {
                findings.push_back({.path=relative, .detail="Python artifact '" + name +
                                                 "' present in package tree"});
                flagged = true;
                break;
            }
        }
        if (flagged) continue;
        const std::string head = read_head(file, 4096);
        if (head.starts_with("#!")) {
            const auto newline = head.find('\n');
            const std::string shebang = head.substr(
                0, newline == std::string::npos ? head.size() : newline);
            for (const auto &token: shebang_tokens) {
                if (shebang.find(token) != std::string::npos) {
                    findings.push_back(
                        {.path=relative, .detail="interpreter shebang '" + shebang + "'"});
                    flagged = true;
                    break;
                }
            }
            if (flagged) continue;
        }
        if (is_mach_o(head) || is_elf(head)) {
            // Dynamic-dependency audit via the platform tool: otool on
            // Mach-O, readelf on ELF. Tool absence fails loudly rather than
            // skipping the check.
            const std::vector<std::string> argv =
                is_mach_o(head)
                    ? std::vector<std::string>{"otool", "-L", file.string()}
                    : std::vector<std::string>{"readelf", "-d", file.string()};
            try {
                const bike::platform::ProcessResult result =
                    bike::platform::run(argv, root);
                if (result.exit_code != 0) continue; // not a loadable image
                std::istringstream lines(result.out + "\n" + result.err);
                std::string line;
                while (std::getline(lines, line)) {
                    for (const auto &load: forbidden_loads) {
                        if (line.find(load) != std::string::npos) {
                            findings.push_back(
                                {.path=relative, .detail="dynamic dependency edge: " +
                                               line});
                            break;
                        }
                    }
                }
            // NOLINTNEXTLINE(bugprone-empty-catch) a non-loadable binary is not a Mach-O/ELF image; no finding
            } catch (const std::exception &) {
            }
        }
    }

    if (findings.empty()) {
        std::cout << "python package audit clean: " << root.string() << "\n";
        return 0;
    }
    for (const Finding &finding: findings)
        std::cerr << "python package audit: " << finding.path << " — "
                  << finding.detail << "\n";
    std::cerr << "python package audit: " << findings.size()
              << " forbidden edge(s) under " << root.string() << "\n";
    return 1;
}

// ---- mode: documentation -----------------------------------------------------

int audit_documentation(const fs::path &root, const Value &policy) {
    const Value *doc_policy = member(policy, "documentation_mode");
    const Value empty_policy = Value{Object{}};
    const Value &resolved =
        doc_policy != nullptr ? *doc_policy : empty_policy;
    const std::vector<std::string> document_globs =
        string_list(resolved, "document_globs");
    const std::vector<std::string> exempt =
        string_list(resolved, "exempt_globs");
    const std::vector<std::string> tokens =
        string_list(resolved, "forbidden_command_tokens");

    std::vector<Finding> findings;
    for (const fs::path &file: walk_files(root)) {
        const std::string relative = relative_to(file, root);
        if (!any_glob(document_globs, relative) ||
            any_glob(exempt, relative))
            continue;
        std::ifstream stream(file, std::ios::binary);
        if (!stream.is_open()) continue;
        std::string line;
        std::size_t line_number = 0;
        bool in_fence = false;
        std::string fence_marker;
        while (std::getline(stream, line)) {
            ++line_number;
            const auto first = line.find_first_not_of(" \t");
            if (!in_fence) {
                if (first != std::string::npos &&
                    line.compare(first, 3, "```") == 0) {
                    in_fence = true;
                    fence_marker = "```";
                }
                continue;
            }
            if (first != std::string::npos &&
                line.compare(first, fence_marker.size(), fence_marker) == 0) {
                in_fence = false;
                continue;
            }
            // Inside an active command block: a forbidden token used as a
            // command word (line start, or after a shell metacharacter) is a
            // live invocation edge.
            std::string stripped =
                first == std::string::npos ? std::string{}
                                           : line.substr(first);
            if (stripped.empty() || stripped.front() == '#') continue;
            for (const auto &token: tokens) {
                if (token.empty()) continue;
                const auto position = stripped.find(token);
                if (position == std::string::npos) continue;
                const bool command_position =
                    position == 0 ||
                    std::string("$ `|;&(").find(stripped[position - 1]) !=
                        std::string::npos;
                if (command_position) {
                    std::string detail = "active command block invokes '";
                    detail += token;
                    detail += "': ";
                    detail += stripped;
                    findings.push_back(
                        {.path=relative + ":" + std::to_string(line_number),
                         .detail=std::move(detail)});
                }
            }
        }
    }

    if (findings.empty()) {
        std::cout << "python documentation audit clean: " << root.string()
                  << "\n";
        return 0;
    }
    for (const Finding &finding: findings)
        std::cerr << "python documentation audit: " << finding.path << " — "
                  << finding.detail << "\n";
    std::cerr << "python documentation audit: " << findings.size()
              << " active invocation(s) under " << root.string() << "\n";
    return 1;
}

std::string usage(const std::string &prog) {
    return "usage: " + prog +
           " --root DIR --mode {source,package,documentation} "
           "[--policy FILE]";
}

} // namespace

// AuditError escapes by design: the dispatch catch maps it to an exit code;
// a bad_alloc escaping main is an acceptable hard stop for an audit tool.
int main(int argc, char **argv) { // NOLINT(bugprone-exception-escape)
    // argv+argc is the ABI-guaranteed bound — the two-argument span
    // construction is correct here by definition of the C entry contract.
    // The pragma is clang dialect; GCC's frontends sweep would reject it.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    const std::string prog =
        !arguments.empty()
            ? std::filesystem::path(arguments.front()).filename().string()
            : "bike-python-audit";
    std::string root;
    std::string mode;
    std::string policy_path;
    for (std::size_t index = 1; index < arguments.size(); ++index) {
        const std::string argument = arguments[index];
        const auto take_value = [&](std::string &out) -> bool {
            if (index + 1 >= arguments.size()) return false;
            out = arguments[++index];
            return true;
        };
        if (argument == "--root" && take_value(root)) continue;
        if (argument.starts_with("--root=")) {
            root = argument.substr(7);
            continue;
        }
        if (argument == "--mode" && take_value(mode)) continue;
        if (argument.starts_with("--mode=")) {
            mode = argument.substr(7);
            continue;
        }
        if (argument == "--policy" && take_value(policy_path)) continue;
        if (argument.starts_with("--policy=")) {
            policy_path = argument.substr(9);
            continue;
        }
        if (argument == "-h" || argument == "--help") {
            std::cout << usage(prog) << "\n";
            return 0;
        }
        std::cerr << usage(prog) << "\n"
                  << prog << ": error: unrecognized arguments: "
                  << argument << "\n";
        return 2;
    }
    if (root.empty() || mode.empty()) {
        std::cerr << usage(prog) << "\n"
                  << prog << ": error: the following arguments are required: "
                  << (root.empty() ? (mode.empty() ? "--root, --mode"
                                                   : "--root")
                                   : "--mode")
                  << "\n";
        return 2;
    }
    if (mode != "source" && mode != "package" && mode != "documentation") {
        std::cerr << usage(prog) << "\n"
                  << prog << ": error: argument --mode: invalid choice: '"
                  << mode
                  << "' (choose from 'source', 'package', 'documentation')\n";
        return 2;
    }
    std::error_code ec;
    const fs::path resolved_root = fs::weakly_canonical(root, ec);
    if (ec || !fs::is_directory(resolved_root, ec)) {
        std::cerr << prog << ": error: audit root is not a directory: " << root
                  << "\n";
        return 2;
    }
    const fs::path resolved_policy =
        policy_path.empty()
            ? resolved_root / "migration" / "no-python-policy.json"
            : fs::weakly_canonical(policy_path);
    try {
        const Value policy =
            fs::is_regular_file(resolved_policy, ec)
                ? read_json(resolved_policy, "no-python policy")
                : Value{Object{}};
        if (mode == "source") return audit_source(resolved_root, policy);
        if (mode == "package") return audit_package(resolved_root, policy);
        return audit_documentation(resolved_root, policy);
    } catch (const std::exception &error) {
        std::cerr << prog << ": audit failed: " << error.what() << "\n";
        return 1;
    }
}
