// checks_main.cpp — bike-native-checks entry point (task T56). Ports the
// argparse contract of tools/native_checks.py::main plus the
// check_native_contracts.py runner, extended with the produced `scripts`
// and `contracts` kinds and the --check-child exec wrapper that replaces
// the oracle's subprocess preexec_fn (RLIMIT_CORE=0 for the real child).
//
//   bike-native-checks --kind KIND --build BUILD
//                      [--tools-dir DIR]      (scripts kind only)
//                      [--repo-root DIR]      (contracts kind only)
//                      [--validate-only | --list-controls] (context only)
//   bike-native-checks --check-child PROGRAM [ARG...]   (internal)
//
// argparse parity: missing required options or an invalid --kind print the
// usage line plus "prog: error: ..." on stderr and exit 2.
#include <array>
#include <cstring>
#include <iostream>
#include <span>
#include <string_view>
#include <string>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

#include "checks_internal.hpp"

namespace {

constexpr std::array<std::string_view, 10> kKinds{
    "tidy",      "analyzer",    "cppcheck", "frontends", "odr",
    "headers",   "diagnostic-controls", "context", "scripts", "contracts",
};

std::string program_name(const char *argv0) {
    const std::string text = argv0 != nullptr ? argv0 : "bike-native-checks";
    const auto slash = text.find_last_of('/');
    return slash == std::string::npos ? text : text.substr(slash + 1);
}

std::string usage_line(const std::string &prog) {
    std::string kinds;
    for (const std::string_view kind: kKinds) {
        if (!kinds.empty()) kinds += ',';
        kinds += kind;
    }
    return "usage: " + prog + " [-h] --kind {" + kinds + "} --build BUILD" +
           " [--tools-dir DIR] [--repo-root DIR] [--validate-only | --list-controls]";
}

int argparse_error(const std::string &prog, const std::string &message) {
    std::cerr << usage_line(prog) << "\n" << prog << ": error: " << message
              << "\n";
    return 2;
}

// --check-child: the preexec_fn port. Disable core dumps on this process,
// then exec the real program so the wrapped child inherits the limit
// (crash artifacts stay out of the workspace while failing probes still
// report their sign-encoded exit status through the process record).
[[noreturn]] void check_child(std::span<char *const> args) {
    rlimit const zero{.rlim_cur=0, .rlim_max=0}; // POSIX tag; 'struct rlimit' trips GCC -Wredundant-tags
    (void)setrlimit(RLIMIT_CORE, &zero);
    std::vector<char *> child_argv;
    child_argv.reserve(args.size() + 1);
    for (char *const argument: args) child_argv.push_back(argument);
    child_argv.push_back(nullptr);
    ::execvp(child_argv.front(), child_argv.data());
    // Only an exec failure lands here; the process record reports it as a
    // launch error through the 127 exit.
    std::cerr << "bike-native-checks: cannot exec " << child_argv.front()
              << ": " << std::strerror(errno) // NOLINT(concurrency-mt-unsafe) post-exec-failure path is single-threaded
              << "\n";
    _exit(127);
}

} // namespace

// CheckError/ContractError escape by design: the dispatch catch maps them to
// exit codes; argument parsing above it cannot throw in a way that matters —
// a bad_alloc escaping main is an acceptable hard stop for an orchestrator.
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
    if (arguments.size() >= 3 &&
        std::string(arguments[1]) == "--check-child")
        check_child(arguments.subspan(2));

    const std::string prog =
        program_name(!arguments.empty() ? arguments.front() : nullptr);
    std::string kind;
    std::string build;
    std::string tools_dir;
    std::string repo_root;
    bool validate_only = false;
    bool list_controls = false;

    for (std::size_t index = 1; index < arguments.size(); ++index) {
        const std::string argument = arguments[index];
        if (argument == "-h" || argument == "--help") {
            std::cout << usage_line(prog) << "\n\n"
                      << "native check orchestrator\n\n"
                      << "options:\n"
                      << "  -h, --help\n"
                      << "  --kind KIND\n"
                      << "  --build BUILD\n"
                      << "  --tools-dir DIR\n"
                      << "  --repo-root DIR\n"
                      << "  --validate-only  validate context without running controls\n"
                      << "  --list-controls  list this platform's context controls\n";
            return 0;
        }
        const auto take_value = [&](std::string &out) -> bool {
            if (index + 1 >= arguments.size()) return false;
            out = arguments[++index];
            return true;
        };
        if (argument == "--kind") {
            if (!take_value(kind))
                return argparse_error(prog,
                                      "argument --kind: expected one argument");
        } else if (argument.starts_with("--kind=")) {
            kind = argument.substr(std::string("--kind=").size());
        } else if (argument == "--build") {
            if (!take_value(build))
                return argparse_error(
                    prog, "argument --build: expected one argument");
        } else if (argument.starts_with("--build=")) {
            build = argument.substr(std::string("--build=").size());
        } else if (argument == "--tools-dir") {
            if (!take_value(tools_dir))
                return argparse_error(
                    prog, "argument --tools-dir: expected one argument");
        } else if (argument.starts_with("--tools-dir=")) {
            tools_dir = argument.substr(std::string("--tools-dir=").size());
        } else if (argument == "--repo-root") {
            if (!take_value(repo_root))
                return argparse_error(
                    prog, "argument --repo-root: expected one argument");
        } else if (argument.starts_with("--repo-root=")) {
            repo_root = argument.substr(std::string("--repo-root=").size());
        } else if (argument == "--validate-only") {
            validate_only = true;
        } else if (argument == "--list-controls") {
            list_controls = true;
        } else {
            return argparse_error(prog,
                                  "unrecognized arguments: " + argument);
        }
    }

    std::string missing;
    if (kind.empty()) missing = "--kind";
    if (build.empty())
        missing += (missing.empty() ? "--build" : ", --build");
    if (!missing.empty())
        return argparse_error(
            prog, "the following arguments are required: " + missing);

    bool known = false;
    for (const std::string_view candidate: kKinds)
        if (kind == candidate) known = true;
    if (!known) {
        std::string choices;
        for (const std::string_view candidate: kKinds) {
            if (!choices.empty()) choices += ", ";
            choices += '\'';
            choices += candidate;
            choices += '\'';
        }
        return argparse_error(prog, "argument --kind: invalid choice: '" + kind +
                                        "' (choose from " + choices + ")");
    }

    if ((validate_only || list_controls) && kind != "context")
        return argparse_error(prog, "--validate-only and --list-controls require --kind context");
    if (validate_only && list_controls)
        return argparse_error(prog, "--validate-only and --list-controls are mutually exclusive");

    const bike::checks::fs::path build_path(build);
    try {
        if (validate_only || list_controls) {
            const auto context = bike::checks::load_context(build_path);
            if (list_controls)
                for (const auto &control : bike::checks::context_control_names(context))
                    std::cout << control << '\n';
            return 0;
        }
        if (kind == "headers")
            return bike::checks::run_header_sweep(build_path);
        if (kind == "diagnostic-controls")
            return bike::checks::run_diagnostic_controls(build_path);
        if (kind == "context")
            return bike::checks::run_context_checks(build_path);
        if (kind == "scripts")
            return bike::checks::run_scripts(
                build_path,
                tools_dir.empty()
                    ? std::nullopt
                    : std::optional<bike::checks::fs::path>(tools_dir));
        if (kind == "contracts")
            return bike::checks::run_contracts(
                build_path,
                repo_root.empty()
                    ? std::nullopt
                    : std::optional<bike::checks::fs::path>(repo_root));
        return bike::checks::run_sweep(kind, build_path);
    } catch (const std::exception &error) {
        // A port-side invariant failure still fails closed; Python's runner
        // would propagate a traceback, the native binary reports cleanly.
        std::cerr << "native " << kind << " failed: " << error.what() << "\n";
        return 1;
    }
}
