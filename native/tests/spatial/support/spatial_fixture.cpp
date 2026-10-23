#include "spatial_fixture.hpp"

#include "fixture_paths.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <system_error>

namespace spatial_tests {
namespace {
constexpr std::uintmax_t kMaxFixtureBytes = 512ULL * 1024ULL * 1024ULL;

[[noreturn]] void fail(std::string_view detail) {
    throw std::invalid_argument("spatial fixture: " + std::string(detail));
}

void check_path_text(const std::filesystem::path& path) {
    if (path.empty() || path.native().find('\0') != std::string::npos)
        fail("path must be nonempty and contain no NUL");
}
} // namespace

std::filesystem::path fixture(std::string_view relative) {
    const std::filesystem::path requested{relative};
    check_path_text(requested);
    if (requested.is_absolute() || requested.has_root_name() || requested.has_root_directory())
        fail("path must be relative");
    for (const auto& part : requested)
        if (part == "..") fail("path must not contain '..'");

    std::error_code error;
    const auto root = std::filesystem::canonical(BIKE_SPATIAL_FIXTURE_ROOT, error);
    if (error) fail("fixture root is missing or inaccessible");
    const auto full = std::filesystem::canonical(root / requested, error);
    if (error) fail("fixture is missing or inaccessible: " + std::string(relative));
    const auto inside = full.lexically_relative(root);
    if (inside.empty() || inside.is_absolute()) fail("canonical path escaped fixture root");
    for (const auto& part : inside)
        if (part == "..") fail("canonical path escaped fixture root");
    return full;
}

std::string text(const std::filesystem::path& path) {
    check_path_text(path);
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error) || error)
        fail("payload must be a readable regular file: " + path.string());
    const auto size = std::filesystem::file_size(path, error);
    if (error) fail("cannot determine payload size: " + path.string());
    if (size > kMaxFixtureBytes) fail("payload exceeds 512 MiB read bound");

    std::ifstream stream(path, std::ios::binary);
    if (!stream.is_open()) fail("cannot open payload: " + path.string());
    std::string result(static_cast<std::size_t>(size), '\0');
    if (size != 0) {
        stream.read(result.data(), static_cast<std::streamsize>(size));
        if (stream.gcount() != static_cast<std::streamsize>(size) || stream.bad())
            fail("payload changed or read failed: " + path.string());
    }
    // Detect growth without ever reading an unbounded iterator range.
    char extra{};
    if (stream.get(extra)) fail("payload grew while reading: " + path.string());
    if (!stream.eof()) fail("payload read failed: " + path.string());
    return result;
}

std::vector<std::byte> bytes(const std::filesystem::path& path) {
    const auto content = text(path);
    const auto view = std::as_bytes(std::span(content.data(), content.size()));
    return {view.begin(), view.end()};
}

bike::io::Value json(const std::filesystem::path& path) {
    return bike::io::parse_json(text(path));
}

std::string sha256_hex(std::span<const std::byte> payload) {
    return bike::io::sha256(payload);
}

} // namespace spatial_tests
