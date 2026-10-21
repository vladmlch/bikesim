#include "support/migration_test.hpp"
#include "runtime/installed_provider.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "platform/files.hpp"

#include <atomic>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;
namespace io = bike::io;
using Value = io::Value;
constexpr std::string_view kProviderBytes = "qualified provider fixture bytes";
constexpr std::string_view kProviderName = "libscipy_openblas64_test.so";

void write(const fs::path& path, std::string_view value) {
    bike::platform::write_exclusive(path, std::as_bytes(std::span(value)));
}
struct TempDir {
    fs::path path;
    TempDir() {
        static std::atomic<unsigned> sequence{0};
        path = fs::temp_directory_path() / ("bike-installed-provider-" +
            std::to_string(static_cast<long long>(::getpid())) + "-" + std::to_string(sequence.fetch_add(1, std::memory_order_relaxed)));
        if (!fs::create_directory(path)) throw std::runtime_error("refusing an existing provider test directory");
    }
    ~TempDir() { std::error_code error; fs::remove_all(path, error); }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    TempDir(TempDir&&) = delete;
    TempDir& operator=(TempDir&&) = delete;
};
const Value& field(const Value& value, std::string_view name) {
    const auto* result = io::find_member(io::as_object(value, "test identity"), name);
    bt::require(result != nullptr, "provider identity retains expected field");
    return *result;
}
std::string policy(std::string_view original, std::string_view digest) {
    Value::Object library{
        {"path", Value{std::string(original)}}, {"role", Value{std::string("blas-lapack")}},
        {"identity_kind", Value{std::string("file")}}, {"sha256", Value{std::string(digest)}},
        {"build_id", Value{}}};
    const Value document{Value::Object{
        {"backend", Value{std::string("NUMPY_OPENBLAS")}},
        {"blas_integer_bits", Value{io::Number{"64"}}},
        {"lapack_integer_bits", Value{io::Number{"64"}}},
        {"libraries", Value{Value::Array{Value{std::move(library)}}}}}};
    return io::canonical_json(document);
}
struct Fixture {
    TempDir scratch;
    fs::path prefix = scratch.path / "installed tree";
    fs::path executable = prefix / "bin/bike-ride";
    fs::path provider = prefix / "lib" / kProviderName;
    std::vector<std::string> required{(scratch.path / "removed source/.deps/numpy/numpy.libs" / kProviderName).string()};
    std::string digest = io::sha256(std::as_bytes(std::span(kProviderBytes)));
    std::string document = policy(required.front(), digest);
    Fixture() {
        fs::create_directories(executable.parent_path());
        fs::create_directories(provider.parent_path());
        write(executable, "fixture executable identity");
        write(provider, kProviderBytes);
    }
};
template<class F> void runtime_error(F&& action, std::string_view expected = {}) {
    bool rejected = false;
    try { std::forward<F>(action)(); }
    catch (const std::runtime_error& error) {
        rejected = true;
        bt::require(std::string_view(error.what()).find(expected) != std::string_view::npos,
            "installed provider reports the concrete file/layout failure");
    }
    bt::require(rejected, "invalid installed provider must not resolve");
}
void checks_moved_install() {
    Fixture fixture;
    const auto identity = runtime::installed_provider_identity(fixture.document, fixture.required, fixture.executable);
    bt::require(identity.required_images == std::vector<std::string>{fs::canonical(fixture.provider).string()},
        "absent original .deps image resolves only to the exact adjacent provider");
    const auto decoded = io::parse_json(identity.policy_json);
    const auto& library = io::as_array(field(decoded, "libraries"), "libraries").at(0);
    bt::require(io::as_string(field(library, "path"), "path") == identity.required_images.front() &&
        io::as_string(field(library, "sha256"), "sha256") == fixture.digest &&
        io::as_string(field(library, "role"), "role") == "blas-lapack" &&
        io::as_int64(field(decoded, "blas_integer_bits"), "blas_integer_bits") == 64 &&
        io::is_null(field(library, "build_id")),
        "relocation changes only path identity, preserving the pinned digest, role and ABI");
    const auto moved = fixture.scratch.path / "renamed installation";
    fs::rename(fixture.prefix, moved);
    const auto relocated = runtime::installed_provider_identity(fixture.document, fixture.required, moved / "bin/bike-ride");
    bt::require(relocated.required_images == std::vector<std::string>{fs::canonical(moved / "lib" / kProviderName).string()},
        "the same compiled identity works after the complete install tree moves");
}
void checks_rejections() {
    Fixture fixture;
    bt::invalid([&] { (void)runtime::installed_provider_identity(fixture.document, {}, fixture.executable); });
    const std::vector<std::string> other_required{(fixture.scratch.path / "different provider.so").string()};
    bt::invalid([&] { (void)runtime::installed_provider_identity(fixture.document, other_required, fixture.executable); });
    bt::invalid([&] { (void)runtime::installed_provider_identity(policy(fixture.required.front(), "not-a-hash"), fixture.required, fixture.executable); });
    bt::invalid([&] { (void)runtime::installed_provider_identity(fixture.document, fixture.required, "bin/bike-ride"); });
    fs::remove(fixture.provider);
    write(fixture.provider, "modified provider bytes");
    runtime_error([&] { (void)runtime::installed_provider_identity(fixture.document, fixture.required, fixture.executable); },
        "SHA-256 mismatch");
    fs::remove(fixture.provider);
    // A correctly hashed same-name file elsewhere is not an installed image.
    write(fixture.scratch.path / kProviderName, kProviderBytes);
    runtime_error([&] { (void)runtime::installed_provider_identity(fixture.document, fixture.required, fixture.executable); });
    fs::create_directory(fixture.provider);
    runtime_error([&] { (void)runtime::installed_provider_identity(fixture.document, fixture.required, fixture.executable); },
        "must be a regular file");
}
void checks_symlinks() {
    Fixture fixture;
    const auto external = fixture.scratch.path / "external";
    fs::create_directory(external);
    write(external / kProviderName, kProviderBytes);
    fs::remove(fixture.provider);
    fs::create_symlink(external / kProviderName, fixture.provider);
    runtime_error([&] { (void)runtime::installed_provider_identity(fixture.document, fixture.required, fixture.executable); },
        "must not be a symlink");
    fs::remove(fixture.provider);
    fs::remove(fixture.provider.parent_path());
    fs::create_directory_symlink(external, fixture.provider.parent_path());
    runtime_error([&] { (void)runtime::installed_provider_identity(fixture.document, fixture.required, fixture.executable); },
        "lib directory is missing or is a symlink");
}
} // namespace

int main() {
    return bt::run([] { checks_moved_install(); checks_rejections(); checks_symlinks(); });
}
