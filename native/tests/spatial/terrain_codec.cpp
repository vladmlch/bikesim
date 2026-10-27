#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "spatial/support/terrain_fixture.hpp"
#include "terrain/asset3d.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
using namespace bike::terrain;
using bike::io::Value;

static_assert(!std::is_default_constructible_v<Grid3d>,
              "an immutable grid must pass create() validation before queries");

const Value& member(const Value& value, std::string_view key) {
    const auto* result = bike::io::find_member(bike::io::as_object(value, "fixture"), key);
    if (!result) throw std::invalid_argument("fixture missing " + std::string(key));
    return *result;
}

std::uint64_t get(std::span<const std::byte> bytes, std::size_t offset, std::size_t width = 8) {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < width; ++i)
        value |= static_cast<std::uint64_t>(std::to_integer<unsigned char>(bytes[offset + i])) << (8 * i);
    return value;
}

void put(std::span<std::byte> bytes, std::size_t offset, std::uint64_t value, std::size_t width = 8) {
    for (std::size_t i = 0; i < width; ++i) bytes[offset + i] = static_cast<std::byte>((value >> (8 * i)) & 255U);
}

void reject(const std::vector<std::byte>& bytes, std::string_view path) {
    const auto result = read_terrain3d(bytes);
    bt::require(!result.has_value(), "corrupt terrain must be rejected");
    bt::require(result.error().find(path) != std::string::npos, "corrupt terrain diagnostic must name field path");
}

template<class Edit>
std::vector<std::byte> edit_surface_json(const std::vector<std::byte>& input, Edit edit) {
    const auto offset = static_cast<std::size_t>(kTerrain3dHeaderBytes + get(input, 72) + get(input, 80));
    const auto count = static_cast<std::size_t>(get(input, 88));
    std::string text(count, '\0');
    for (std::size_t i = 0; i < count; ++i) text[i] = static_cast<char>(std::to_integer<unsigned char>(input[offset + i]));
    auto surfaces = bike::io::parse_json(text);
    edit(std::get<Value::Array>(surfaces.data));
    const auto replacement = bike::io::canonical_json(surfaces);
    std::vector<std::byte> result(input.begin(), input.begin() + static_cast<std::ptrdiff_t>(offset));
    for (char c : replacement) result.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    result.insert(result.end(), input.begin() + static_cast<std::ptrdiff_t>(offset + count), input.end());
    put(result, 88, replacement.size());
    return result;
}

Value& mutable_member(Value& value, std::string_view name) {
    for (auto& [key, child] : std::get<Value::Object>(value.data)) if (key == name) return child;
    throw std::invalid_argument("missing surface field");
}
} // namespace

int main() {
    return bt::run([] {
        const auto grid = spatial_tests::asymmetric_4x4();
        const auto encoded = write_terrain3d(grid);
        const auto decoded = read_terrain3d(encoded);
        bt::require(decoded.has_value(), "valid asymmetric terrain decodes");
        bt::require(write_terrain3d(*decoded) == encoded, "canonical BKT3 round trip is byte exact");
        bt::require(decoded->sha256_hex() == spatial_tests::sha256_hex(encoded), "identity hashes the entire canonical asset");
        bt::require(std::equal(grid.elevation().begin(), grid.elevation().end(), decoded->elevation().begin()), "elevations survive exact decode");
        bt::require(std::equal(grid.materials().begin(), grid.materials().end(), decoded->materials().begin()), "material IDs survive exact decode");
        bt::require(grid.surfaces().size() == decoded->surfaces().size(), "surface count retained");
        for (std::size_t i = 0; i < grid.surfaces().size(); ++i) {
            const auto& a = grid.surfaces()[i]; const auto& b = decoded->surfaces()[i];
            bt::require(a.id == b.id && a.name == b.name && a.mu_peak == b.mu_peak && a.mu_slide == b.mu_slide &&
                a.slip_stiffness_per_load == b.slip_stiffness_per_load && a.stribeck_speed_mps == b.stribeck_speed_mps &&
                a.description == b.description, "surface author order and fields retained exactly");
        }
        const auto fixture = spatial_tests::json(spatial_tests::fixture("terrain/asym4x4.terrain3d.json"));
        for (const auto& node : bike::io::as_array(member(fixture, "transpose_nodes"), "nodes")) {
            const auto& pair = bike::io::as_array(node, "node");
            const auto x = static_cast<std::size_t>(bike::io::as_int64(pair[0], "ix"));
            const auto y = static_cast<std::size_t>(bike::io::as_int64(pair[1], "iy"));
            const double expected = .1 * static_cast<double>(x) + .37 * static_cast<double>(y) + .01 * static_cast<double>(x * y);
            bt::require(decoded->elevation()[y * 4 + x] == expected, "iy*nx+ix transpose detector");
        }
        auto surfaces = std::vector<SurfaceSpec3d>(grid.surfaces().begin(), grid.surfaces().end());
        surfaces[0].mu_peak += .01;
        const auto changed = Grid3d::create(grid.descriptor(), {grid.elevation().begin(), grid.elevation().end()},
            {grid.materials().begin(), grid.materials().end()}, surfaces, grid.metadata());
        bt::require(changed.has_value() && changed->sha256_hex() != grid.sha256_hex(), "friction is part of physical asset identity");
        const auto matrix = spatial_tests::json(spatial_tests::fixture("terrain/corrupt_cases.json"));
        for (const auto& row : bike::io::as_array(member(matrix, "header_cases"), "cases")) {
            auto corrupt = encoded;
            const auto offset = static_cast<std::size_t>(bike::io::as_int64(member(row, "offset"), "offset"));
            const auto width = static_cast<std::size_t>(bike::io::as_int64(member(row, "width"), "width"));
            const auto& token = bike::io::as_string(member(row, "value_decimal"), "value_decimal");
            std::uint64_t value = 0;
            // to_address avoids raw pointer arithmetic (-Wunsafe-buffer-usage).
            const char* const token_end = std::to_address(token.cend());
            const auto parsed = std::from_chars(token.data(), token_end, value);
            bt::require(parsed.ec == std::errc{} && parsed.ptr == token_end, "corruption fixture u64 is exact");
            put(corrupt, offset, value, width);
            reject(corrupt, bike::io::as_string(member(row, "error"), "error"));
        }
        auto corrupt = encoded; corrupt.pop_back(); reject(corrupt, "terrain3d.header.total_bytes");
        corrupt = encoded; corrupt.push_back(std::byte{0}); reject(corrupt, "terrain3d.header.total_bytes");
        corrupt = encoded; put(corrupt, 112 + 7 * 8, std::bit_cast<std::uint64_t>(std::numeric_limits<double>::infinity()));
        reject(corrupt, "terrain3d.elevation[7]");
        corrupt = encoded; put(corrupt, 112 + 16 * 8, 999, 4); reject(corrupt, "terrain3d.material[0]");
        corrupt = edit_surface_json(encoded, [](auto& a) { mutable_member(a[1], "id") = mutable_member(a[0], "id"); });
        reject(corrupt, ".id");
        corrupt = edit_surface_json(encoded, [](auto& a) { mutable_member(a[0], "mu_peak") = Value{bike::io::Number{"-0.5"}}; });
        reject(corrupt, "terrain3d.surfaces[0].mu_peak");
        corrupt = edit_surface_json(encoded, [](auto& a) { std::get<Value::Object>(a[0].data).emplace_back("unknown", Value{true}); });
        reject(corrupt, "terrain3d.surfaces[0].unknown");
        corrupt = encoded; put(corrupt, 16, std::numeric_limits<std::uint64_t>::max()); reject(corrupt, "terrain3d.header.height_bytes");
        bt::require(!read_terrain3d(encoded, {.max_total_bytes = encoded.size() - 1, .max_nodes = 16}), "byte limit checked before allocation");
        bt::require(!read_terrain3d(encoded, {.max_total_bytes = encoded.size(), .max_nodes = 15}), "node limit checked before allocation");
        // At 2^53 the binary64 spacing is 2. A finite total domain can still
        // contain collapsed cells; with spacing 1.25 the first step moves,
        // but the second node rounds onto the first one.
        for (const double spacing : {1., 1.25}) {
            for (const bool x_axis : {true, false}) {
                auto collapsed = grid.descriptor();
                if (x_axis) { collapsed.x0_m = 0x1p53; collapsed.dx_m = spacing; }
                else { collapsed.y0_m = 0x1p53; collapsed.dy_m = spacing; }
                const auto rejected = Grid3d::create(collapsed,
                    {grid.elevation().begin(), grid.elevation().end()},
                    {grid.materials().begin(), grid.materials().end()},
                    {grid.surfaces().begin(), grid.surfaces().end()}, grid.metadata());
                const std::string_view path = x_axis ? "terrain3d.header.dx_m" : "terrain3d.header.dy_m";
                bt::require(!rejected && rejected.error().find(path) != std::string::npos,
                            "typed grid rejects collapsed world-coordinate cells");
                corrupt = encoded;
                put(corrupt, x_axis ? 32 : 40, std::bit_cast<std::uint64_t>(0x1p53));
                put(corrupt, x_axis ? 48 : 56, std::bit_cast<std::uint64_t>(spacing));
                reject(corrupt, path);
                if (x_axis) collapsed.dx_m = 2.; else collapsed.dy_m = 2.;
                bt::require(Grid3d::create(collapsed,
                    {grid.elevation().begin(), grid.elevation().end()},
                    {grid.materials().begin(), grid.materials().end()},
                    {grid.surfaces().begin(), grid.surfaces().end()}, grid.metadata()).has_value(),
                    "representable cells at the same large world offset remain valid");
            }
        }
        auto descriptor = grid.descriptor(); descriptor.dtype = 1;
        std::vector<double> f32_heights(16);
        for (std::size_t i = 0; i < f32_heights.size(); ++i) f32_heights[i] = static_cast<double>(i) * .25;
        f32_heights[0] = -0.;
        const auto f32 = Grid3d::create(descriptor, f32_heights, {grid.materials().begin(), grid.materials().end()},
            {grid.surfaces().begin(), grid.surfaces().end()}, grid.metadata());
        bt::require(f32.has_value(), "exact f32 values can be represented by dtype1");
        const auto f32_bytes = write_terrain3d(*f32);
        const auto f32_read = read_terrain3d(f32_bytes);
        bt::require(f32_read.has_value() && write_terrain3d(*f32_read) == f32_bytes, "f32 bytes round trip");
        for (std::size_t i = 0; i < f32_heights.size(); ++i)
            bt::require(std::bit_cast<std::uint64_t>(f32_read->elevation()[i]) == std::bit_cast<std::uint64_t>(f32_heights[i]), "f32 promotes exactly, including signed zero");
        auto sparse = std::vector<SurfaceSpec3d>(grid.surfaces().begin(), grid.surfaces().end());
        sparse[1].id = std::numeric_limits<std::uint32_t>::max();
        auto materials = std::vector<std::uint32_t>(grid.materials().begin(), grid.materials().end());
        for (auto& id : materials) if (id == 7) id = sparse[1].id;
        const auto sparse_grid = Grid3d::create(grid.descriptor(), {grid.elevation().begin(), grid.elevation().end()},
            std::move(materials), std::move(sparse), grid.metadata());
        bt::require(sparse_grid.has_value(), "sparse u32 surface IDs do not require max-ID-sized allocations");
    });
}
