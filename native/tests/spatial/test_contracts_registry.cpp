#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "io/value.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {
using bike::io::Value;

const Value& member(const Value& value, std::string_view key, std::string_view path) {
    const auto& object = bike::io::as_object(value, path);
    const auto* found = bike::io::find_member(object, key);
    if (found == nullptr)
        throw std::invalid_argument(std::string(path) + ": missing " + std::string(key));
    return *found;
}

bool lower_hex(std::string_view text, std::size_t size) {
    if (text.size() != size) return false;
    for (const char c : text)
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    return true;
}

void check_versions(const Value& value) {
    constexpr std::array<std::pair<std::string_view, std::int64_t>, 11> expected{{
        {"spatial_contract", 1}, {"runtime_schema", 2}, {"bootstrap_schema", 2},
        {"writer_schema", 3}, {"tyre_state", 1}, {"checkpoint", 1}, {"sensor", 2},
        {"policy_abi", 2}, {"catalog", 2}, {"container", 2}, {"replay", 4}
    }};
    const auto& versions = bike::io::as_object(value, "versions");
    bt::require(versions.size() == expected.size() + 1, "all independent schema names present");
    std::set<std::string> names;
    for (const auto& [name, unused] : versions) {
        (void)unused;
        bt::require(names.insert(name).second, "schema names are distinct");
    }
    for (const auto& [name, version] : expected)
        bt::require(bike::io::as_int64(member(value, name, "versions"), name) == version,
                    "independent schema version must match C7");
    bt::require(bike::io::as_string(member(value, "terrain", "versions"), "terrain") == "BKT3/1",
                "terrain has its own magic/version namespace");
}
} // namespace

int main() {
    return bt::run([] {
        const std::filesystem::path root{BIKE_SPATIAL_SOURCE_ROOT};
        const auto registry = spatial_tests::json(root / "migration/spatial3d/contracts.json");
        bt::require(bike::io::as_int64(member(registry, "schema_version", "registry"),
                                    "registry.schema_version") == 1, "contracts registry schema1");
        const auto& contracts = bike::io::as_array(member(registry, "contracts", "registry"), "contracts");
        bt::require(contracts.size() == 9, "C1 through C9 contracts must all be registered");
        std::set<std::string> ids, headers;
        for (const auto& contract : contracts) {
            const auto& id = bike::io::as_string(member(contract, "id", "contract"), "contract.id");
            bt::require(ids.insert(id).second, "contract ID must be unique");
            const auto& header = bike::io::as_string(member(contract, "header_path", id), id + ".header_path");
            bt::require(!header.empty() && headers.insert(header).second,
                        "contract anchor header must be nonempty and unique");
            const std::filesystem::path relative_header{header};
            bt::require(!relative_header.is_absolute(), "contract header is repository-relative");
            for (const auto& part : relative_header)
                bt::require(part != "..", "contract header cannot traverse");
            bt::require(std::filesystem::is_regular_file(root / relative_header),
                        "contract header must exist in this checkout");
            const auto& owner = bike::io::as_string(member(contract, "owner", id), id + ".owner");
            bt::require(!owner.empty(), "each contract has exactly one named owner");
            bt::require(!bike::io::as_string(member(contract, "namespace", id), id + ".namespace").empty(),
                        "contract namespace is explicit");
            const auto& consumers = bike::io::as_array(member(contract, "consumers", id), id + ".consumers");
            bt::require(!consumers.empty(), "contract consumers must not be empty");
            std::set<std::string> unique_consumers;
            for (const auto& item : consumers) {
                const auto& consumer = bike::io::as_string(item, id + ".consumers[]");
                bt::require(!consumer.empty() && unique_consumers.insert(consumer).second,
                            "consumer names are nonempty and unique");
            }
            for (const auto name : {"types", "functions", "frames", "units"})
                bt::require(!bike::io::as_array(member(contract, name, id), id + "." + name).empty(),
                            "contract type/function/frame/unit inventory must not be empty");
            bt::require(!bike::io::as_string(member(contract, "producer", id), id + ".producer").empty(),
                        "contract has a producer task");
            check_versions(member(contract, "versions", id));
        }
        for (unsigned i = 1; i <= 9; ++i)
            bt::require(ids.contains("C" + std::to_string(i)), "all C1-C9 IDs must exist");

        const auto baseline = spatial_tests::json(root / "migration/spatial3d/baseline.json");
        bt::require(lower_hex(bike::io::as_string(member(baseline, "base_sha", "baseline"),
                                                "baseline.base_sha"), 40),
                    "S00 base_sha is exactly 40 lowercase hex characters");
        const auto& wip = bike::io::as_array(member(baseline, "wip_patches", "baseline"), "baseline.wip_patches");
        for (const auto& item : wip) {
            const auto& disposition = bike::io::as_string(member(item, "disposition", "wip"), "wip.disposition");
            bt::require(disposition == "integrate" || disposition == "already_equivalent" ||
                        disposition == "retain_separate", "each WIP entry has a supported disposition");
            bt::require(!bike::io::as_string(member(item, "rationale", "wip"), "wip.rationale").empty(),
                        "each WIP disposition has a rationale");
        }
    });
}
