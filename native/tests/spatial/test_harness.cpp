#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "platform/argv.hpp"
#include "platform/process.hpp"

#include <bit>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <variant>
#include <vector>

namespace {
class TempDirectory {
public:
    TempDirectory() {
        const auto pattern = (std::filesystem::temp_directory_path() /
                              "bike-spatial-harness-XXXXXX").string();
        std::vector<char> storage(pattern.begin(), pattern.end());
        storage.push_back('\0');
        const char* created = ::mkdtemp(storage.data());
        if (created == nullptr) throw std::runtime_error("cannot create spatial harness scratch");
        path_ = created;
    }
    ~TempDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    TempDirectory(const TempDirectory&) = delete;
    TempDirectory& operator=(const TempDirectory&) = delete;
    const std::filesystem::path& path() const noexcept { return path_; }
private:
    std::filesystem::path path_;
};
} // namespace

int main(int argc, char** argv) {
    // The shared entry-point wrapper keeps raw argv subscripting out of the
    // test under -Wunsafe-buffer-usage.
    const auto args = bike::platform::main_arguments(argc, argv);
    if (args.size() == 2 && std::string_view(args[1]) == "--zero-checks")
        return bt::run([] {});
    return bt::run([&] {
        bt::require(args.size() == 1, "unexpected spatial harness arguments");
        bt::require(std::bit_cast<std::uint64_t>(0.) !=
                    std::bit_cast<std::uint64_t>(-0.), "signed zero bit control");
        bt::invalid([] { throw std::invalid_argument("positive invalid control"); });

        const auto self = std::filesystem::absolute(args.front());
        bike::platform::ProcessOptions options;
        options.timeout_s = 10.;
        options.max_output_bytes = 4096;
        const auto child = bike::platform::run({self.string(), "--zero-checks"},
                                               self.parent_path(), options);
        bt::require(child.exit_code == 1, "zero-check child must fail normally, not crash");
        bt::require(child.err.find("zero checks") != std::string::npos,
                    "zero-check child must fail for the expected reason");

        bt::invalid([] { (void)spatial_tests::fixture(""); });
        bt::invalid([] { (void)spatial_tests::fixture("../outside.json"); });
        bt::invalid([] { (void)spatial_tests::fixture("references/../../outside.json"); });
        bt::invalid([] { (void)spatial_tests::fixture("/outside.json"); });
        bt::invalid([] { (void)spatial_tests::fixture(std::string("x\0y", 3)); });
        bt::invalid([] { (void)spatial_tests::fixture("harness/missing-payload.json"); });

        const auto control = spatial_tests::fixture("harness/control.json");
        const auto content = spatial_tests::text(control);
        const auto payload = spatial_tests::bytes(control);
        bt::require(!content.empty() && payload.size() == content.size(),
                    "fixture text and bytes preserve length");
        bt::require(std::holds_alternative<bike::io::Value::Object>(
                        spatial_tests::json(control).data),
                    "fixture JSON parses as an object");
        const std::string abc = "abc";
        bt::require(spatial_tests::sha256_hex(std::as_bytes(std::span(abc.data(), abc.size()))) ==
                    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
                    "SHA-256 standard abc vector");

        TempDirectory scratch;
        const auto oversized = scratch.path() / "oversized.bin";
        { std::ofstream file(oversized, std::ios::binary); }
        std::filesystem::resize_file(oversized, 512ULL * 1024ULL * 1024ULL + 1ULL);
        bt::invalid([&] { (void)spatial_tests::text(oversized); });
        bt::invalid([&] { (void)spatial_tests::text(scratch.path()); });
    });
}
