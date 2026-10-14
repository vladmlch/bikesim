// migration_test.cpp — implementations for the migration contract-test
// harness. Explicit little-endian binary64 decoding, length checks,
// fixture-root containment and bounded payload reads per
// migration/interface-contracts.md §8.
#include "migration_test.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <system_error>

#ifndef BIKE_FIXTURE_ROOT
#error "BIKE_FIXTURE_ROOT must be supplied as a compile definition"
#endif

namespace {
    // Bound for a single fixture payload read (interface-contracts.md §8:
    // test payloads are bounded; large trajectories are chunked into
    // declared fixture pieces).
    constexpr std::uintmax_t max_fixture_bytes = 512ULL * 1024ULL * 1024ULL;

    // require()/run() share a scoreboard so a test body that ran zero checks
    // cannot pass vacuously; thread_local keeps a later threaded helper
    // honest.
    std::size_t &require_count() {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) deliberate harness scoreboard shared by require()/run()
        static thread_local std::size_t value = 0;
        return value;
    }

    std::string hex64(std::uint64_t bits) {
        std::ostringstream out;
        out << "0x" << std::hex << std::setfill('0') << std::setw(16) << bits;
        return out.str();
    }
} // namespace

namespace bt {

int run(const std::function<void()> &test) {
    const std::size_t before = require_count();
    try {
        test();
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "FAIL non-standard test exception\n";
        return 1;
    }
    const std::size_t executed = require_count() - before;
    if (executed == 0) {
        std::cerr << "FAIL test body ran zero checks\n";
        return 1;
    }
    std::cout << "PASS " << executed << " checks\n";
    return 0;
}

void require(bool condition, std::string_view message) {
    ++require_count();
    if (!condition)
        throw std::runtime_error(std::string(message));
}

std::filesystem::path fixture(std::string_view relative) {
    const std::filesystem::path requested{relative};
    require(!requested.empty(), "fixture path must not be empty");
    require(!requested.is_absolute(), "fixture path must be relative");
    for (const auto &part: requested)
        require(part != "..",
                "fixture path must not contain a '..' component");
    const std::filesystem::path root =
        std::filesystem::weakly_canonical(BIKE_FIXTURE_ROOT);
    const std::filesystem::path full =
        std::filesystem::weakly_canonical(root / requested);
    const std::filesystem::path inside = full.lexically_relative(root);
    require(!inside.empty() && !inside.is_absolute(),
            "fixture path must stay inside the fixture root");
    for (const auto &part: inside)
        require(part != "..",
                "fixture path must not escape the fixture root");
    require(std::filesystem::exists(full),
            std::string("fixture does not exist: ") + std::string(relative));
    return full;
}

std::string text(const std::filesystem::path &path) {
    require(std::filesystem::is_regular_file(path),
            "fixture is not a regular file");
    const std::uintmax_t size = std::filesystem::file_size(path);
    require(size <= max_fixture_bytes, "fixture payload exceeds 512 MiB");
    std::ifstream stream(path, std::ios::binary);
    require(stream.is_open(), "cannot open fixture file");
    std::string content((std::istreambuf_iterator<char>(stream)),
                        std::istreambuf_iterator<char>());
    require(!stream.bad(), "fixture read failed");
    require(content.size() == size, "fixture changed while reading");
    return content;
}

std::vector<double> read_f64(const std::filesystem::path &path) {
    const std::string bytes = text(path);
    require(bytes.size() % 8 == 0,
            "truncated little-endian binary64 fixture");
    std::vector<double> values;
    values.reserve(bytes.size() / 8);
    for (std::size_t offset = 0; offset < bytes.size(); offset += 8) {
        std::uint64_t bits = 0;
        for (unsigned shift = 0; shift < 8; ++shift)
            bits |= static_cast<std::uint64_t>(
                        static_cast<unsigned char>(bytes.at(offset + shift)))
                    << (8U * shift);
        values.push_back(std::bit_cast<double>(bits));
    }
    return values;
}

void golden_text(std::string_view relative, std::string_view actual) {
    const std::string expected = text(fixture(relative));
    const bool equal = expected == actual;
    std::string message;
    if (!equal) {
        const std::size_t shared = std::min(expected.size(), actual.size());
        std::size_t first = 0;
        while (first < shared && expected.at(first) == actual.at(first))
            ++first;
        message = std::string("golden_text mismatch: ") +
                  std::string(relative) + " first_diff_byte=" +
                  std::to_string(first) + " expected_bytes=" +
                  std::to_string(expected.size()) + " actual_bytes=" +
                  std::to_string(actual.size());
    }
    require(equal, message);
}

void golden_f64(std::string_view relative, std::span<const double> actual) {
    const std::vector<double> expected = read_f64(fixture(relative));
    bool equal = expected.size() == actual.size();
    std::string message;
    if (!equal) {
        message = std::string("golden_f64 shape mismatch: ") +
                  std::string(relative) + " expected_count=" +
                  std::to_string(expected.size()) + " actual_count=" +
                  std::to_string(actual.size());
    } else {
        for (std::size_t index = 0; index < actual.size(); ++index) {
            const std::uint64_t expected_bits =
                std::bit_cast<std::uint64_t>(expected.at(index));
            const std::uint64_t actual_bits =
                std::bit_cast<std::uint64_t>(actual[index]);
            if (expected_bits != actual_bits) {
                equal = false;
                message = std::string("golden_f64 mismatch: ") +
                          std::string(relative) + " index=" +
                          std::to_string(index) + " expected=" +
                          hex64(expected_bits) + " actual=" +
                          hex64(actual_bits);
                break;
            }
        }
    }
    require(equal, message);
}

} // namespace bt
