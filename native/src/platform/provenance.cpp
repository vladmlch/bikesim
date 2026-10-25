// provenance.cpp — running-image and build identity plus the two clock
// origins. Identity is baked at build time into platform_build_identity.hpp
// (the Track-B convention from research_build_identity.hpp): configured
// values are compiled in, never read from an editable JSON at runtime. The
// produced artifact itself is hashed at query time so the recorded image
// digest is the file actually on disk.
#include "platform/provenance.hpp"

#include "io/hash.hpp"
#include "platform/files.hpp"
#include "platform_build_identity.hpp"

#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <climits>
#include <unistd.h>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

namespace {

[[noreturn]] void fail_errno(const char *what, const int error) {
    throw std::system_error(std::error_code(error, std::generic_category()),
                            std::string(what));
}

} // namespace

namespace bike::platform {

std::filesystem::path executable_path() {
#if defined(__APPLE__)
    // _NSGetExecutablePath may return a relative or symlinked name; resolve
    // through realpath so a renamed launcher still names the on-disk image.
    std::vector<char> buffer(1024U);
    while (true) {
        auto size = static_cast<std::uint32_t>(buffer.size());
        if (::_NSGetExecutablePath(buffer.data(), &size) == 0) break;
        buffer.assign(static_cast<std::size_t>(size) + 1U, '\0');
    }
    std::array<char, PATH_MAX> resolved {};
    if (::realpath(buffer.data(), resolved.data()) == nullptr) {
        const int error = errno;
        fail_errno("platform::executable_path: realpath", error);
    }
    return std::filesystem::path{resolved.data()};
#elif defined(__linux__)
    std::array<char, PATH_MAX> buffer {};
    const ssize_t got =
        ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1U);
    if (got < 0) {
        const int error = errno;
        fail_errno("platform::executable_path: readlink", error);
    }
    return std::filesystem::path{
        std::string_view{buffer.data(), static_cast<std::size_t>(got)}};
#else
    throw std::runtime_error(
        "platform::executable_path: unsupported platform");
#endif
}

std::filesystem::path install_prefix() {
    const auto self = executable_path();
    const auto dir = self.parent_path();
    if (!self.has_filename() || dir.empty() || !dir.has_parent_path())
        throw std::runtime_error(
            "platform::install_prefix: no parent layout above " +
            self.native());
    // bin/.. convention: the prefix is the directory above the one holding
    // the image, so moved install trees keep executable-relative lookup.
    return std::filesystem::weakly_canonical(dir.parent_path());
}

std::string key() {
    // Baked at build time from CMAKE_SYSTEM_NAME-CMAKE_SYSTEM_PROCESSOR —
    // bitwise the same expression the test graphs lowercase into
    // BIKE_GOLDEN_PLATFORM, so the reader-side sibling lookup and the
    // captured fixture directories always agree.
    std::string result = &identity::platform[0];
    for (char &c : result)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return result;
}

BuildIdentity build_identity() {
    BuildIdentity identity;
    // &name[0] is the codebase's explicit-decay convention for generated
    // constexpr char[] constants (runtime/provenance.cpp does the same).
    identity.compiler = &identity::compiler_id[0];
    identity.compile_flags = &identity::numerical_flags[0];
    identity.sdk = &identity::sdk[0];
    identity.platform = &identity::platform[0];
    identity.build_type = &identity::build_type[0];
    identity.dependency_lock_sha256 = &identity::dependency_lock_sha256[0];
    identity.source_manifest_sha256 = &identity::source_manifest_sha256[0];
    identity.python_bindings = identity::python_bindings;
    identity.gui = identity::gui;
    // The produced artifact's own bytes — hashed after open through the
    // bounded no-follow reader, the same digest an auditor reproduces.
    identity.executable_sha256 = file_sha256(executable_path());
    return identity;
}

std::string execution_identity() {
    // Canonical field order; only the numerical-execution axis feeds the
    // digest — packaging flags (bindings/GUI) are deliberately absent so
    // deleting the adapter layer cannot move physical compatibility.
    const std::string canonical =
        std::string{"compiler="} + &identity::compiler_id[0] +
        "\nflags=" + &identity::numerical_flags[0] +
        "\nsdk=" + &identity::sdk[0] +
        "\nplatform=" + &identity::platform[0] +
        "\nbuild_type=" + &identity::build_type[0] +
        "\nlock=" + &identity::dependency_lock_sha256[0] +
        "\nsources=" + &identity::source_manifest_sha256[0] + "\n";
    return io::sha256(std::as_bytes(std::span(canonical)));
}

std::string utc_now() {
    const auto now = std::chrono::system_clock::now();
    const auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto micros =
        std::chrono::duration_cast<std::chrono::microseconds>(now - secs)
            .count();
    const std::time_t wall = std::chrono::system_clock::to_time_t(now);
    std::tm broken {};
    if (::gmtime_r(&wall, &broken) == nullptr)
        throw std::runtime_error("platform::utc_now: gmtime_r failed");
    // Manual digit assembly — no printf-family call in the stamp path.
    std::string stamp;
    stamp.reserve(27U);
    const auto append2 = [&stamp](const int value) {
        stamp += static_cast<char>('0' + (value / 10) % 10);
        stamp += static_cast<char>('0' + value % 10);
    };
    const auto append4 = [&stamp](const int value) {
        stamp += static_cast<char>('0' + (value / 1000) % 10);
        stamp += static_cast<char>('0' + (value / 100) % 10);
        stamp += static_cast<char>('0' + (value / 10) % 10);
        stamp += static_cast<char>('0' + value % 10);
    };
    append4(broken.tm_year + 1900);
    stamp += '-';
    append2(broken.tm_mon + 1);
    stamp += '-';
    append2(broken.tm_mday);
    stamp += 'T';
    append2(broken.tm_hour);
    stamp += ':';
    append2(broken.tm_min);
    stamp += ':';
    append2(broken.tm_sec);
    stamp += '.';
    for (long long place = 100000; place > 0; place /= 10)
        stamp += static_cast<char>('0' + (micros / place) % 10);
    stamp += 'Z';
    return stamp;
}

double monotonic_seconds() noexcept {
    // Process-local monotonic origin: the first observation defines the
    // receipt/measurement epoch; wall-clock time never substitutes.
    static const std::chrono::steady_clock::time_point origin =
        std::chrono::steady_clock::now();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         origin)
        .count();
}

} // namespace bike::platform
