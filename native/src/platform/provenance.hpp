// provenance.hpp — running-image and build identity plus the two clock
// origins (task T06).
//
// Build identity separates the NUMERICAL-EXECUTION axis from the
// ADAPTER/PACKAGING axis: compiler, flags, SDK, platform, the dependency
// lock hash and the first-party source-manifest hash determine the
// arithmetic the binary performs, while bindings/GUI flags only describe
// packaging — deleting the optional bindings must not invalidate physical
// compatibility, so execution_identity() digests the execution axis alone.
// The running image itself is hashed at query time (the actual produced
// artifact, not a build-time claim); a shared library's image hash is
// owned by that library's own identity contract (engine::identity covers
// libmujoco through the same dladdr-on-data technique).
//
// Clocks: utc_now() is the wall-clock UTC metadata stamp; monotonic_seconds()
// is a monotonic receipt/measurement origin. The two axes never substitute
// for each other — durations and deadlines are always monotonic.
#pragma once

#include <filesystem>
#include <string>

namespace bike::platform {

// Canonical absolute path of the running executable image — resolved
// through the OS image registry and realpath, so a renamed or symlinked
// launcher still names the actual file on disk.
[[nodiscard]] std::filesystem::path executable_path();

// The install-tree root this binary belongs to: the directory above the
// executable's own directory (the bin/.. convention). Resources resolve
// through it, so a moved install tree keeps executable-relative lookup
// working; in a developer build it resolves to the build root's parent.
[[nodiscard]] std::filesystem::path install_prefix();

// The lower-case "<system>-<machine>" key naming per-platform runtime
// artifacts — the same axis as Python's
// platform.system().lower() + "-" + platform.machine() and the test-side
// BIKE_GOLDEN_PLATFORM/initial_state/<platform>/ directory convention.
// Explicit t=0 transfer documents are platform-bound (their contract pins
// compiled-model arrays), so request readers prefer a capture inside a
// sibling directory named by this key.
[[nodiscard]] std::string key();

struct BuildIdentity {
    // ---- numerical-execution axis (digested by execution_identity()) ----
    std::string compiler;        // configured compiler id + version
    std::string compile_flags;   // strict-policy flag identity verbatim
    std::string sdk;             // resolved sysroot, "" when not applicable
    std::string platform;        // OS + architecture
    std::string build_type;      // CMake configuration
    std::string dependency_lock_sha256; // checked lock file (T03)
    std::string source_manifest_sha256; // first-party source manifest (T00)
    // ---- adapter/packaging axis (NOT part of execution identity) --------
    bool python_bindings = false;
    bool gui = false;
    // ---- produced artifact identity ------------------------------------
    std::string executable_sha256; // SHA-256 of the running image's bytes
};

// Resolves the running image's build record. Throws std::runtime_error or
// std::system_error when the image cannot be identified or hashed.
[[nodiscard]] BuildIdentity build_identity();

// 64-hex SHA-256 over the numerical-execution axis fields only. Two
// binaries that differ ONLY in the packaging axis (bindings/GUI flags)
// share one execution identity; any change to compiler, flags, SDK,
// platform, dependency lock or source manifest moves it.
[[nodiscard]] std::string execution_identity();

// Wall-clock UTC metadata stamp: "YYYY-MM-DDTHH:MM:SS.ffffffZ" (microsecond
// resolution, always Zulu). Metadata/reporting use this; never durations.
[[nodiscard]] std::string utc_now();

// Seconds on the monotonic receipt/measurement origin (steady_clock,
// process-local epoch captured at first use). Never wall-derived.
[[nodiscard]] double monotonic_seconds() noexcept;

} // namespace bike::platform
