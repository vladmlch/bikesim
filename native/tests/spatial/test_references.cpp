// W03: independently transcribed reference data, not runtime-generated goldens.
// Execution and source/asset review belong to V01.
#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "model/benchmark_whipple.hpp"
#include "io/value.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

namespace io = bike::io;
using Parameters = bike::model::WhippleParameters;

const io::Value& member(const io::Value& value, std::string_view key,
                        std::string_view path) {
    const auto* found = io::find_member(io::as_object(value, path), key);
    if (found == nullptr)
        throw std::invalid_argument(std::string(path) + "." +
                                    std::string(key) + ": missing member");
    return *found;
}

double number(const io::Value& value, std::string_view key,
              std::string_view path) {
    return io::as_double(member(value, key, path),
                         std::string(path) + "." + std::string(key));
}

const std::string& string(const io::Value& value, std::string_view key,
                          std::string_view path) {
    return io::as_string(member(value, key, path),
                         std::string(path) + "." + std::string(key));
}

void nonempty(const io::Value& value, std::string_view key,
              std::string_view path) {
    bt::require(!string(value, key, path).empty(),
                std::string(path) + "." + std::string(key) + ": empty");
}

void close(double actual, double expected, double tolerance,
           const std::string& field) {
    bt::require(std::isfinite(actual) && std::isfinite(expected) &&
                    std::abs(actual - expected) <= tolerance,
                field + ": reference mismatch");
}

struct ParameterField {
    std::string_view name;
    double Parameters::* value;
};

constexpr std::array<ParameterField, 27> kParameterFields{{
    {"w_m", &Parameters::w_m}, {"c_m", &Parameters::c_m},
    {"lambda_rad", &Parameters::lambda_rad}, {"g_mps2", &Parameters::g_mps2},
    {"v_mps", &Parameters::v_mps}, {"rR_m", &Parameters::rR_m},
    {"mR_kg", &Parameters::mR_kg}, {"IRxx", &Parameters::IRxx},
    {"IRyy", &Parameters::IRyy}, {"xB_m", &Parameters::xB_m},
    {"zB_m", &Parameters::zB_m}, {"mB_kg", &Parameters::mB_kg},
    {"IBxx", &Parameters::IBxx}, {"IBxz", &Parameters::IBxz},
    {"IByy", &Parameters::IByy}, {"IBzz", &Parameters::IBzz},
    {"xH_m", &Parameters::xH_m}, {"zH_m", &Parameters::zH_m},
    {"mH_kg", &Parameters::mH_kg}, {"IHxx", &Parameters::IHxx},
    {"IHxz", &Parameters::IHxz}, {"IHyy", &Parameters::IHyy},
    {"IHzz", &Parameters::IHzz}, {"rF_m", &Parameters::rF_m},
    {"mF_kg", &Parameters::mF_kg}, {"IFxx", &Parameters::IFxx},
    {"IFyy", &Parameters::IFyy},
}};

void meijaard_reference() {
    const auto document = spatial_tests::json(
        spatial_tests::fixture("references/meijaard2007.json"));
    bt::require(io::as_int64(member(document, "schema_version", "meijaard"),
                            "meijaard.schema_version") == 1,
                "Meijaard reference schema");
    const auto& citation = member(document, "citation", "meijaard");
    bt::require(string(citation, "doi", "meijaard.citation") ==
                    "10.1098/rspa.2007.1857", "Meijaard source DOI");
    const auto& convention = member(document, "coordinate_convention", "meijaard");
    bt::require(string(convention, "z", "meijaard.coordinate_convention") ==
                    "down", "retain the source z-down convention");

    const auto& values = member(document, "parameters", "meijaard");
    bt::require(io::as_object(values, "meijaard.parameters").size() ==
                    kParameterFields.size(), "all Whipple parameter fields present");
    const Parameters defaults{};
    for (const auto& field : kParameterFields) {
        const auto path = "meijaard.parameters." + std::string(field.name);
        const auto& entry = member(values, field.name, "meijaard.parameters");
        close(defaults.*(field.value), number(entry, "value", path), 1e-12,
              path + ".value");
        nonempty(entry, "unit", path);
        nonempty(entry, "source_symbol", path);
        nonempty(entry, "source_location", path);
    }

    const auto& table = member(document, "table2", "meijaard");
    const auto& rows = io::as_array(member(table, "rows", "meijaard.table2"),
                                    "meijaard.table2.rows");
    bt::require(rows.size() == 2, "Table 2 must retain the v=2 and v=6 cases");
    // Published Table 2(b,c), p. 1971. None of these values is obtained
    // from the simulator or from the implementation under test.
    constexpr std::array<double, 2> speeds{2., 6.};
    constexpr std::array<std::array<std::array<double, 2>, 4>, 2> roots{{
        {{{2.68234517512745, 1.68066296590675},
          {2.68234517512745, -1.68066296590675},
          {-3.07158645641514, 0.}, {-8.67387984831735, 0.}}},
        {{{-1.52644486584142, 5.87673060598709},
          {-1.52644486584142, -5.87673060598709},
          {-0.00406690076970, 0.}, {-16.08537123098026, 0.}}},
    }};
    constexpr std::array<std::string_view, 4> modes{
        "weave_positive_imag", "weave_negative_imag", "capsize", "castering"};
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto path = "meijaard.table2.rows[" + std::to_string(i) + "]";
        close(number(rows[i], "speed_mps", path), speeds[i], 1e-12, path);
        nonempty(rows[i], "source_location", path);
        const auto& eigenvalues = io::as_array(
            member(rows[i], "eigenvalues_per_s", path), path + ".eigenvalues_per_s");
        bt::require(eigenvalues.size() == 4, path + ": four eigenvalues required");
        for (std::size_t j = 0; j < eigenvalues.size(); ++j) {
            const auto root_path = path + ".eigenvalues_per_s[" +
                                   std::to_string(j) + "]";
            bt::require(string(eigenvalues[j], "mode", root_path) == modes[j],
                        root_path + ".mode");
            close(number(eigenvalues[j], "real", root_path), roots[i][j][0],
                  1e-12, root_path + ".real");
            close(number(eigenvalues[j], "imag", root_path), roots[i][j][1],
                  1e-12, root_path + ".imag");
            nonempty(eigenvalues[j], "source_location", root_path);
        }
    }
    const auto& boundaries = member(table, "boundaries", "meijaard.table2");
    const auto& weave = member(boundaries, "weave", "meijaard.table2.boundaries");
    const auto& capsize = member(boundaries, "capsize", "meijaard.table2.boundaries");
    close(number(weave, "speed_mps", "weave"), 4.29238253634111, 1e-12,
          "Table 2(a) weave boundary");
    close(number(capsize, "speed_mps", "capsize"), 6.02426201538837, 1e-12,
          "Table 2(a) capsize boundary");
    nonempty(weave, "source_location", "weave");
    nonempty(capsize, "source_location", "capsize");
}

void tyre_lateral_reference() {
    const auto document = spatial_tests::json(
        spatial_tests::fixture("references/tyre-lateral.json"));
    bt::require(io::as_int64(member(document, "schema_version", "tyre"),
                            "tyre.schema_version") == 1,
                "tyre lateral reference schema");
    const auto& rows = io::as_array(member(document, "rows", "tyre"), "tyre.rows");
    bt::require(!rows.empty(), "empty tyre reference table must fail closed");
    std::set<std::string> ids;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto& row = rows[i];
        const auto path = "tyre.rows[" + std::to_string(i) + "]";
        nonempty(row, "id", path);
        bt::require(ids.emplace(string(row, "id", path)).second,
                    path + ".id: duplicate");
        const double load = number(row, "normal_load_n", path);
        bt::require(std::isfinite(load) && load > 0., path + ".normal_load_n");
        bt::require(number(row, "pressure_pa", path) > 0., path + ".pressure_pa");
        bt::require(number(row, "speed_mps", path) > 0., path + ".speed_mps");
        const double degrees = number(row, "slip_angle_deg", path);
        const double radians = number(row, "slip_angle_rad", path);
        bt::require(std::abs(radians - degrees * std::numbers::pi / 180.) < 1e-14,
                    path + ".slip_angle_rad: degree conversion");
        bt::require(std::abs(number(row, "camber_rad", path) -
                            number(row, "camber_deg", path) *
                                std::numbers::pi / 180.) < 1e-14,
                    path + ".camber_rad: degree conversion");
        const double stiffness = number(row, "cornering_stiffness_n_per_rad", path);
        bt::require(stiffness > 0., path + ".cornering_stiffness_n_per_rad");
        const double original = number(row, "cornering_stiffness_original_n_per_deg", path);
        close(stiffness, original * 180. / std::numbers::pi, 1e-10,
              path + ".cornering_stiffness_n_per_rad: N/deg conversion");
        for (const auto key : {"tyre", "surface", "fit_convention", "source_doi",
                               "source_location"})
            nonempty(row, key, path);
        bt::require(string(row, "applicability", path) == "cross_check_only",
                    path + ": city/cargo measurements cannot qualify MTB coefficients");
        const auto& uncertainty = member(row, "uncertainty", path);
        const double relative = number(uncertainty, "relative_fraction", path + ".uncertainty");
        bt::require(relative > 0. && relative < 1., path + ".uncertainty.relative_fraction");
        nonempty(uncertainty, "source_location", path + ".uncertainty");
        const auto& units = member(row, "units_original", path);
        bt::require(string(units, "cornering_stiffness", path + ".units_original") ==
                        "N/deg", path + ": preserve original stiffness unit");
    }
    // Raw Zenodo payloads are not consumed here. Their null identity remains
    // pending acquisition; only the attributed published table is checked.
}

} // namespace

int main() {
    return bt::run([] {
        meijaard_reference();
        tyre_lateral_reference();
    });
}
