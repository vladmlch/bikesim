#include "asset3d.hpp"

#include "io/float_format.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace bike::terrain {
namespace {

using bike::io::Value;
using Object = Value::Object;

[[noreturn]] void fail(std::string_view path, std::string_view reason) {
    throw std::invalid_argument(std::string(path) + ": " + std::string(reason));
}

std::uint64_t product(std::uint64_t a, std::uint64_t b, std::string_view path) {
    if (b != 0 && a > std::numeric_limits<std::uint64_t>::max() / b)
        fail(path, "overflow");
    return a * b;
}

std::uint64_t sum(std::uint64_t a, std::uint64_t b, std::string_view path) {
    if (a > std::numeric_limits<std::uint64_t>::max() - b) fail(path, "overflow");
    return a + b;
}

template<class UInt>
void put(std::span<std::byte> out, std::size_t offset, UInt value) noexcept {
    for (std::size_t i = 0; i < sizeof(UInt); ++i)
        out[offset + i] = static_cast<std::byte>((value >> (8 * i)) & UInt{255});
}

template<class UInt>
UInt get(std::span<const std::byte> input, std::size_t offset) noexcept {
    UInt value = 0;
    for (std::size_t i = 0; i < sizeof(UInt); ++i)
        value |= static_cast<UInt>(std::to_integer<unsigned char>(input[offset + i])) << (8 * i);
    return value;
}

Value real(double value) { return Value{bike::io::Number{bike::io::repr_binary64(value)}}; }
Value integer(std::uint32_t value) { return Value{bike::io::Number{std::to_string(value)}}; }

Value surface_value(const SurfaceSpec3d& surface) {
    Object fields{{"id", integer(surface.id)}, {"name", Value{surface.name}},
                  {"mu_peak", real(surface.mu_peak)}, {"mu_slide", real(surface.mu_slide)},
                  {"slip_stiffness_per_load", real(surface.slip_stiffness_per_load)},
                  {"stribeck_speed_mps", real(surface.stribeck_speed_mps)}};
    if (!surface.description.empty()) fields.emplace_back("description", Value{surface.description});
    return Value{std::move(fields)};
}

Value metadata_value(const Grid3dMetadata& metadata) {
    return Value{Object{{"generator_version", Value{metadata.generator_version}},
                        {"resolved_spec", metadata.resolved_spec},
                        {"seed_decimal", Value{metadata.seed_decimal}},
                        {"units", Value{metadata.units}},
                        {"height_datum_m", real(metadata.height_datum_m)},
                        {"provenance", metadata.provenance}}};
}

const Object& object(const Value& value, std::string_view path,
                     std::initializer_list<std::string_view> keys) {
    const auto& result = bike::io::as_object(value, path);
    for (const auto& [key, unused] : result) {
        (void)unused;
        if (std::ranges::find(keys, key) == keys.end())
            fail(std::string(path) + "." + key, "unknown field");
    }
    return result;
}

const Value& required(const Object& fields, std::string_view name, std::string_view path) {
    if (const auto* value = bike::io::find_member(fields, name)) return *value;
    fail(std::string(path) + "." + std::string(name), "missing field");
}

std::uint32_t surface_id(const Value& value, std::string_view path) {
    const auto result = bike::io::as_int64(value, path);
    if (result < 0 || std::cmp_greater(result, std::numeric_limits<std::uint32_t>::max()))
        fail(path, "expected u32");
    return static_cast<std::uint32_t>(result);
}

std::string byte_string(std::span<const std::byte> bytes) {
    std::string text(bytes.size(), '\0');
    for (std::size_t i = 0; i < bytes.size(); ++i)
        text[i] = static_cast<char>(std::to_integer<unsigned char>(bytes[i]));
    return text;
}

Value parse_section(std::span<const std::byte> bytes, std::string_view path) {
    try {
        return bike::io::parse_json(byte_string(bytes));
    } catch (const std::invalid_argument& error) {
        fail(path, error.what());
    }
}

} // namespace

std::vector<std::byte> write_terrain3d(const Grid3d& grid) {
    Value::Array surfaces;
    surfaces.reserve(grid.surfaces().size());
    for (const auto& surface : grid.surfaces()) surfaces.push_back(surface_value(surface));
    // The existing canonical serializer sorts object keys; surface array order
    // is the author's order and is preserved byte-for-byte by this codec.
    const auto surface_json = bike::io::canonical_json(Value{std::move(surfaces)});
    const auto metadata_json = bike::io::canonical_json(metadata_value(grid.metadata()));
    const auto& desc = grid.descriptor();
    const auto height_bytes = product(product(desc.nx, desc.ny, "terrain3d.header.height_bytes"),
                                      desc.dtype == 0 ? 8 : 4, "terrain3d.header.height_bytes");
    const auto material_bytes = product(product(desc.nx - 1, desc.ny - 1,
                                                "terrain3d.header.material_bytes"),
                                        4, "terrain3d.header.material_bytes");
    auto total = sum(kTerrain3dHeaderBytes, height_bytes, "terrain3d.header.total_bytes");
    total = sum(total, material_bytes, "terrain3d.header.total_bytes");
    total = sum(total, surface_json.size(), "terrain3d.header.total_bytes");
    total = sum(total, metadata_json.size(), "terrain3d.header.total_bytes");
    if (total > std::numeric_limits<std::size_t>::max()) fail("terrain3d.header.total_bytes", "size_t overflow");
    std::vector<std::byte> bytes(static_cast<std::size_t>(total));
    for (std::size_t i = 0; i < kTerrain3dMagic.size(); ++i)
        bytes[i] = static_cast<std::byte>(kTerrain3dMagic[i]);
    put<std::uint32_t>(bytes, 4, kTerrain3dVersion);
    put<std::uint32_t>(bytes, 8, kTerrain3dHeaderBytes);
    put<std::uint32_t>(bytes, 12, desc.dtype);
    put<std::uint64_t>(bytes, 16, desc.nx);
    put<std::uint64_t>(bytes, 24, desc.ny);
    put<std::uint64_t>(bytes, 32, std::bit_cast<std::uint64_t>(desc.x0_m));
    put<std::uint64_t>(bytes, 40, std::bit_cast<std::uint64_t>(desc.y0_m));
    put<std::uint64_t>(bytes, 48, std::bit_cast<std::uint64_t>(desc.dx_m));
    put<std::uint64_t>(bytes, 56, std::bit_cast<std::uint64_t>(desc.dy_m));
    put<std::uint64_t>(bytes, 64, grid.surfaces().size());
    put<std::uint64_t>(bytes, 72, height_bytes);
    put<std::uint64_t>(bytes, 80, material_bytes);
    put<std::uint64_t>(bytes, 88, surface_json.size());
    put<std::uint64_t>(bytes, 96, metadata_json.size());
    put<std::uint64_t>(bytes, 104, 0);
    std::size_t offset = kTerrain3dHeaderBytes;
    for (const double height : grid.elevation()) {
        if (desc.dtype == 0) {
            put<std::uint64_t>(bytes, offset, std::bit_cast<std::uint64_t>(height));
            offset += sizeof(double);
        } else {
            put<std::uint32_t>(bytes, offset, std::bit_cast<std::uint32_t>(static_cast<float>(height)));
            offset += sizeof(float);
        }
    }
    for (const auto id : grid.materials()) {
        put<std::uint32_t>(bytes, offset, id);
        offset += sizeof(std::uint32_t);
    }
    for (const char ch : surface_json) bytes[offset++] = static_cast<std::byte>(static_cast<unsigned char>(ch));
    for (const char ch : metadata_json) bytes[offset++] = static_cast<std::byte>(static_cast<unsigned char>(ch));
    return bytes;
}

std::expected<Grid3d, std::string> read_terrain3d(std::span<const std::byte> bytes,
                                               Terrain3dLoadLimits limits) {
    try {
        if (bytes.size() > limits.max_total_bytes) fail("terrain3d.header.total_bytes", "resource limit exceeded");
        if (bytes.size() < kTerrain3dHeaderBytes) fail("terrain3d.header", "truncated");
        for (std::size_t i = 0; i < kTerrain3dMagic.size(); ++i)
            if (bytes[i] != static_cast<std::byte>(kTerrain3dMagic[i])) fail("terrain3d.header.magic", "expected BKT3");
        if (get<std::uint32_t>(bytes, 4) != kTerrain3dVersion) fail("terrain3d.header.version", "unsupported version");
        if (get<std::uint32_t>(bytes, 8) != kTerrain3dHeaderBytes) fail("terrain3d.header.header_bytes", "expected 112");
        const Grid3dDescriptor desc{.nx = get<std::uint64_t>(bytes, 16), .ny = get<std::uint64_t>(bytes, 24),
            .x0_m = std::bit_cast<double>(get<std::uint64_t>(bytes, 32)),
            .y0_m = std::bit_cast<double>(get<std::uint64_t>(bytes, 40)),
            .dx_m = std::bit_cast<double>(get<std::uint64_t>(bytes, 48)),
            .dy_m = std::bit_cast<double>(get<std::uint64_t>(bytes, 56)),
            .dtype = get<std::uint32_t>(bytes, 12)};
        if (desc.dtype > 1) fail("terrain3d.header.dtype", "expected 0 or 1");
        if (desc.nx < 4) fail("terrain3d.header.nx", "must be >= 4");
        if (desc.ny < 4) fail("terrain3d.header.ny", "must be >= 4");
        if (!std::isfinite(desc.x0_m)) fail("terrain3d.header.x0_m", "non-finite");
        if (!std::isfinite(desc.y0_m)) fail("terrain3d.header.y0_m", "non-finite");
        if (!std::isfinite(desc.dx_m) || desc.dx_m <= 0.) fail("terrain3d.header.dx_m", "must be finite and > 0");
        if (!std::isfinite(desc.dy_m) || desc.dy_m <= 0.) fail("terrain3d.header.dy_m", "must be finite and > 0");
        if (get<std::uint64_t>(bytes, 104) != 0) fail("terrain3d.header.reserved", "must be zero");
        const auto nodes = product(desc.nx, desc.ny, "terrain3d.header.height_bytes");
        if (nodes > limits.max_nodes) fail("terrain3d.header.nx*ny", "node resource limit exceeded");
        const auto height_bytes = get<std::uint64_t>(bytes, 72);
        if (height_bytes != product(nodes, desc.dtype == 0 ? 8 : 4, "terrain3d.header.height_bytes"))
            fail("terrain3d.header.height_bytes", "mismatch");
        const auto cells = product(desc.nx - 1, desc.ny - 1, "terrain3d.header.material_bytes");
        const auto material_bytes = get<std::uint64_t>(bytes, 80);
        if (material_bytes != product(cells, 4, "terrain3d.header.material_bytes"))
            fail("terrain3d.header.material_bytes", "mismatch");
        const auto surface_bytes = get<std::uint64_t>(bytes, 88);
        const auto metadata_bytes = get<std::uint64_t>(bytes, 96);
        auto total = sum(kTerrain3dHeaderBytes, height_bytes, "terrain3d.header.height_bytes");
        total = sum(total, material_bytes, "terrain3d.header.material_bytes");
        total = sum(total, surface_bytes, "terrain3d.header.surface_json_bytes");
        total = sum(total, metadata_bytes, "terrain3d.header.metadata_json_bytes");
        if (surface_bytes > bytes.size()) fail("terrain3d.header.surface_json_bytes", "exceeds payload");
        if (metadata_bytes > bytes.size()) fail("terrain3d.header.metadata_json_bytes", "exceeds payload");
        if (total != bytes.size()) fail("terrain3d.header.total_bytes", "truncated payload or trailing bytes");
        if (total > limits.max_total_bytes || nodes > std::numeric_limits<std::size_t>::max() / sizeof(double) ||
            cells > std::numeric_limits<std::size_t>::max() / sizeof(std::uint32_t))
            fail("terrain3d.header.total_bytes", "allocation resource limit exceeded");
        const auto surface_offset = static_cast<std::size_t>(kTerrain3dHeaderBytes + height_bytes + material_bytes);
        const auto surface_document = parse_section(bytes.subspan(surface_offset, static_cast<std::size_t>(surface_bytes)),
                                                    "terrain3d.surfaces");
        const auto& surface_array = bike::io::as_array(surface_document, "terrain3d.surfaces");
        if (surface_array.size() != get<std::uint64_t>(bytes, 64)) fail("terrain3d.header.surface_count", "mismatch");
        std::vector<SurfaceSpec3d> surfaces;
        surfaces.reserve(surface_array.size());
        for (std::size_t i = 0; i < surface_array.size(); ++i) {
            const std::string path = "terrain3d.surfaces[" + std::to_string(i) + "]";
            const auto& fields = object(surface_array[i], path, {"id", "name", "mu_peak", "mu_slide",
                "slip_stiffness_per_load", "stribeck_speed_mps", "description"});
            SurfaceSpec3d surface{.id = surface_id(required(fields, "id", path), path + ".id"),
                .name = bike::io::as_string(required(fields, "name", path), path + ".name"),
                .mu_peak = bike::io::as_double(required(fields, "mu_peak", path), path + ".mu_peak"),
                .mu_slide = bike::io::as_double(required(fields, "mu_slide", path), path + ".mu_slide"),
                .slip_stiffness_per_load = bike::io::as_double(required(fields, "slip_stiffness_per_load", path), path + ".slip_stiffness_per_load"),
                .stribeck_speed_mps = bike::io::as_double(required(fields, "stribeck_speed_mps", path), path + ".stribeck_speed_mps"),
                .description = {}};
            if (const auto* description = bike::io::find_member(fields, "description"))
                surface.description = bike::io::as_string(*description, path + ".description");
            surfaces.push_back(std::move(surface));
        }
        const auto metadata_document = parse_section(bytes.subspan(surface_offset + static_cast<std::size_t>(surface_bytes),
            static_cast<std::size_t>(metadata_bytes)), "terrain3d.metadata");
        const auto& fields = object(metadata_document, "terrain3d.metadata", {"generator_version", "resolved_spec",
            "seed_decimal", "units", "height_datum_m", "provenance"});
        Grid3dMetadata metadata{
            .generator_version = bike::io::as_string(required(fields, "generator_version", "terrain3d.metadata"), "terrain3d.metadata.generator_version"),
            .resolved_spec = required(fields, "resolved_spec", "terrain3d.metadata"),
            .seed_decimal = bike::io::as_string(required(fields, "seed_decimal", "terrain3d.metadata"), "terrain3d.metadata.seed_decimal"),
            .units = bike::io::as_string(required(fields, "units", "terrain3d.metadata"), "terrain3d.metadata.units"),
            .height_datum_m = bike::io::as_double(required(fields, "height_datum_m", "terrain3d.metadata"), "terrain3d.metadata.height_datum_m"),
            .provenance = required(fields, "provenance", "terrain3d.metadata")};
        std::vector<double> elevation(static_cast<std::size_t>(nodes));
        std::size_t offset = kTerrain3dHeaderBytes;
        for (auto& height : elevation) {
            if (desc.dtype == 0) {
                height = std::bit_cast<double>(get<std::uint64_t>(bytes, offset));
                offset += sizeof(double);
            } else {
                height = static_cast<double>(std::bit_cast<float>(get<std::uint32_t>(bytes, offset)));
                offset += sizeof(float);
            }
        }
        std::vector<std::uint32_t> materials(static_cast<std::size_t>(cells));
        for (auto& id : materials) {
            id = get<std::uint32_t>(bytes, offset);
            offset += sizeof(std::uint32_t);
        }
        return Grid3d::create(desc, std::move(elevation), std::move(materials), std::move(surfaces), std::move(metadata));
    } catch (const std::invalid_argument& error) {
        return std::unexpected(std::string(error.what()));
    }
}

} // namespace bike::terrain
