#include "realtime_runner.hpp"
#include "provenance.hpp"
#include "setup_values.hpp"
#include "bike_measurement_paths.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../platform/process.hpp"
#include <cerrno>
#include <cstddef>
#include <sstream>
#include <system_error>
#include <sys/utsname.h>

namespace runtime::measurement {
namespace {
using namespace setupvalue;
namespace io = bike::io;
std::string read(const std::filesystem::path& path) {
    const auto bytes = io::read_numpy_bytes(path);
    std::string out; out.reserve(bytes.size());
    for (std::byte const value : bytes) out.push_back(static_cast<char>(value));
    return out;
}
Value optional(const Value& object, std::string_view key) {
    const auto* value = find(object, key); return value != nullptr ? *value : Value{};
}
std::string trim_line(std::string value) {
    while (!value.empty() && (value.back() == '\n' || value.back() == '\r')) value.pop_back();
    return value;
}
} // namespace
bike::io::Value native_build_provenance(const std::filesystem::path& build_directory) {
    Value out{Object{{"build_dir", text(build_directory.string())}}};
    try {
        const auto context = io::parse_json(read(build_directory / "native_check_context.json"));
        const auto targets = optional(context, "target_contexts");
        Value target{Object{}};
        if (!io::is_null(targets)) {
            for (std::string_view const name : {"bike-measure-realtime", "bike_native", "bike_core"}) {
                const auto* found = find(targets, name); if (found != nullptr) { target = *found; break; }
            }
        }
        set(out, "compiler", optional(context, "compiler")); set(out, "sdk", optional(context, "sdk"));
        set(out, "hardening", optional(context, "libcpp_hardening"));
        for (std::string_view const name : {"cxx_standard", "cxx_extensions", "compile_definitions", "compile_options"}) set(out, name, optional(target, name));
        set(out, "target_configuration", optional(target, "configuration"));
    } catch (const std::exception& error) {
        set(out, "context_error", text(std::string("cannot read native_check_context.json: ") + error.what()));
    }
    Value sanitizers{Object{{"address", Value{build::address_sanitizer}},
        {"realtime", Value{build::realtime_sanitizer}}, {"coverage", Value{build::coverage}}}};
    try {
        std::istringstream cache(read(build_directory / "CMakeCache.txt"));
        std::string line;
        while (std::getline(cache, line)) {
            for (const auto& [key, flag] : {std::pair{"NATIVE_SANITIZE:BOOL=", "address"},
                    std::pair{"NATIVE_RTSAN:BOOL=", "realtime"}, std::pair{"NATIVE_COVERAGE:BOOL=", "coverage"}})
                if (line.starts_with(key)) set(sanitizers, flag, Value{trim_line(line.substr(std::string_view(key).size())) == "ON"});
        }
    } catch (const std::exception& error) { set(out, "cache_error", text(error.what())); }
    set(out, "sanitizer_options", std::move(sanitizers));
    set(out, "sanitizer_runtime", Value{});
    // Retain the historical key without claiming a Python extension was loaded.
    set(out, "imported_extension", Value{});
    return out;
}
bike::io::Value native_provenance() {
    auto out = native_build_provenance(build::context_directory);
    const auto identity = runtime_identity();
    set(out, "native_source_sha256", text(identity.native_source_sha256));
    set(out, "build_context", io::parse_json(identity.build_context_json));
    set(out, "git_sha", Value{}); set(out, "working_tree_dirty", Value{});
    try {
        const std::filesystem::path source_root(build::source_root);
        if (!std::filesystem::is_regular_file(source_root / "native/src/runtime/realtime_driver.cpp"))
            throw std::runtime_error("native source checkout is not available");
        bike::platform::ProcessOptions process;
        process.timeout_s = 5.; process.max_output_bytes = static_cast<std::size_t>(1024U * 1024U);
        const auto sha = bike::platform::run({"git", "rev-parse", "HEAD"}, source_root, process);
        const auto dirty = bike::platform::run({"git", "status", "--porcelain", "--", "native", "CMakeLists.txt", "CMakePresets.json"}, source_root, process);
        if (sha.exit_code != 0 || dirty.exit_code != 0) throw std::runtime_error("git cannot identify the native source checkout");
        set(out, "git_sha", text(trim_line(sha.out))); set(out, "working_tree_dirty", Value{!dirty.out.empty()});
    } catch (const std::exception& error) { set(out, "git_unavailable_reason", text(error.what())); }
    utsname machine{};
    if (::uname(&machine) == 0) set(out, "machine", text(std::string(static_cast<const char *>(machine.sysname)) + "-" + static_cast<const char *>(machine.release) + "-" + static_cast<const char *>(machine.machine)));
    else { set(out, "machine", Value{}); set(out, "machine_unavailable_reason", text(std::system_error(errno, std::generic_category()).what())); }
    return out;
}
} // namespace runtime::measurement
