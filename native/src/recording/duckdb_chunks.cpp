#include "encoding.hpp"
#include "duckdb.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <ranges>
#include <utility>

namespace bike::recording {
namespace {
using runtime::Piece;
[[noreturn]] void failed(std::string_view operation, const char* detail = nullptr) {
    throw std::runtime_error("recording DuckDB " + std::string(operation) + (detail ? ": " + std::string(detail) : " failed"));
}
struct Logical {
    duckdb_logical_type handle = nullptr;
    explicit Logical(duckdb_logical_type type) : handle(type) { if (!handle) failed("logical type"); }
    ~Logical() { if (handle) duckdb_destroy_logical_type(&handle); }
    Logical(Logical&& other) noexcept : handle(std::exchange(other.handle, nullptr)) {}
    Logical& operator=(Logical&& other) noexcept {
        if (handle) duckdb_destroy_logical_type(&handle);
        handle = std::exchange(other.handle, nullptr);
        return *this;
    }
    Logical(const Logical&) = delete;
    Logical& operator=(const Logical&) = delete;
};
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
Logical logical(const FieldSpec& field) {
    if (field.dtype == "double") return Logical(duckdb_create_logical_type(DUCKDB_TYPE_DOUBLE));
    if (field.dtype == "bigint") return Logical(duckdb_create_logical_type(DUCKDB_TYPE_BIGINT));
    if (field.dtype == "boolean") return Logical(duckdb_create_logical_type(DUCKDB_TYPE_BOOLEAN));
    if (field.dtype == "varchar") return Logical(duckdb_create_logical_type(DUCKDB_TYPE_VARCHAR));
    if (field.dtype == "struct" || field.dtype == "struct[]" || field.dtype == "map<struct>") {
        std::vector<Logical> owners;
        std::vector<std::string> names;
        for (const auto& child : field.children) {
            owners.push_back(logical(child)); names.push_back(child.key);
            if (child.optional) { owners.emplace_back(duckdb_create_logical_type(DUCKDB_TYPE_BOOLEAN)); names.push_back(child.key + "_present"); }
        }
        std::vector<duckdb_logical_type> types; std::vector<const char*> labels;
        types.reserve(owners.size());
for (const auto& owner : owners) types.push_back(owner.handle);
        labels.reserve(names.size());
for (const auto& name : names) labels.push_back(name.c_str());
        Logical structure(duckdb_create_struct_type(types.data(), labels.data(), static_cast<idx_t>(types.size())));
        if (field.dtype == "struct") return structure;
        if (field.dtype == "struct[]") return Logical(duckdb_create_list_type(structure.handle));
        Logical const key(duckdb_create_logical_type(DUCKDB_TYPE_VARCHAR));
        return Logical(duckdb_create_map_type(key.handle, structure.handle));
    }
    auto member = field;
    if (field.dtype.starts_with("map<")) {
        member.dtype = field.dtype.substr(4, field.dtype.size()-5);
        auto const child = logical(member); Logical const key(duckdb_create_logical_type(DUCKDB_TYPE_VARCHAR));
        return Logical(duckdb_create_map_type(key.handle, child.handle));
    }
    if (field.dtype.ends_with("[]")) { member.dtype.resize(member.dtype.size()-2); auto const child = logical(member); return Logical(duckdb_create_list_type(child.handle)); }
    throw std::invalid_argument("recording unsupported logical type: " + field.dtype);
}
struct Chunk {
    duckdb_data_chunk handle = nullptr;
    explicit Chunk(std::span<duckdb_logical_type> types) : handle(duckdb_create_data_chunk(types.data(), static_cast<idx_t>(types.size()))) { if (!handle) failed("create data chunk"); }
    ~Chunk() { if (handle) duckdb_destroy_data_chunk(&handle); }
    Chunk(const Chunk&) = delete; Chunk& operator=(const Chunk&) = delete;
    Chunk(Chunk&&) = delete; Chunk& operator=(Chunk&&) = delete;
};
struct Appender {
    duckdb_appender handle = nullptr;
    Appender(duckdb_connection connection, const std::string& table) {
        if (duckdb_appender_create(connection, "main", table.c_str(), &handle) == DuckDBError) {
            const std::string error = handle && duckdb_appender_error(handle) ? duckdb_appender_error(handle) : "appender creation failed";
            if (handle) (void)duckdb_appender_destroy(&handle);
            failed("create appender", error.c_str());
        }
    }
    ~Appender() { if (handle) (void)duckdb_appender_destroy(&handle); }
    Appender(const Appender&) = delete; Appender& operator=(const Appender&) = delete;
    Appender(Appender&&) = delete; Appender& operator=(Appender&&) = delete;
    void append(duckdb_data_chunk chunk) {
        if (duckdb_append_data_chunk(handle, chunk) == DuckDBError) failed("append data chunk", duckdb_appender_error(handle));
    }
    void finish(const std::function<void(std::string_view)>& before) {
        if (before) before("appender_flush");
        if (duckdb_appender_flush(handle) == DuckDBError) failed("flush appender", duckdb_appender_error(handle));
        if (before) before("appender_close");
        if (duckdb_appender_close(handle) == DuckDBError) failed("close appender", duckdb_appender_error(handle));
        // Copy the diagnostic before destroy invalidates its storage.
        const auto* text = duckdb_appender_error(handle);
        const std::string error = text ? text : "appender destroy failed";
        if (before) before("appender_destroy");
        if (duckdb_appender_destroy(&handle) == DuckDBError) failed("destroy appender", error.c_str());
    }
};
void validity(duckdb_vector vector, const Piece& piece, std::size_t first, std::size_t count) {
    if (!vector) failed("null vector");
    duckdb_vector_ensure_validity_writable(vector);
    auto* bits = duckdb_vector_get_validity(vector);
    if (!bits) failed("validity allocation");
    for (std::size_t row = 0; row < count; ++row) duckdb_validity_set_row_validity(bits, static_cast<idx_t>(row), piece.valid.empty() || piece.valid[first+row] != 0);
}
void strings(duckdb_vector vector, const std::vector<std::string>& values, std::size_t first, std::size_t count) {
    for (std::size_t row = 0; row < count; ++row) {
        const auto& s = values[first+row];
        duckdb_vector_assign_string_element_len(vector, static_cast<idx_t>(row), s.data(), static_cast<idx_t>(s.size()));
    }
}
void fill(duckdb_vector, const Piece&, const FieldSpec&, std::size_t, std::size_t);
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void structure(duckdb_vector vector, const Piece& p, const FieldSpec& f, std::size_t first, std::size_t count) {
    idx_t column = 0;
    for (const auto& child : f.children) {
        fill(duckdb_struct_vector_get_child(vector, column++), p.children.at(child.key), child, first, count);
        if (child.optional) {
            FieldSpec flag; flag.dtype = "boolean";
            fill(duckdb_struct_vector_get_child(vector, column++), p.children.at(child.key + "_present"), flag, first, count);
        }
    }
}
std::pair<std::size_t,std::size_t> list_extent(duckdb_vector vector, const Piece& p, std::size_t first, std::size_t count) {
    auto* data = static_cast<duckdb_list_entry*>(duckdb_vector_get_data(vector));
    if (!data && count) failed("list data allocation");
    const auto entries = std::views::counted(data, static_cast<std::ptrdiff_t>(count));
    const auto begin = static_cast<std::size_t>(p.offsets[first]);
    const auto end = static_cast<std::size_t>(p.offsets[first+count]);
    const auto elements = end - begin;
    if (duckdb_list_vector_reserve(vector, static_cast<idx_t>(elements)) == DuckDBError ||
        duckdb_list_vector_set_size(vector, static_cast<idx_t>(elements)) == DuckDBError) failed("list child allocation");
    for (std::size_t row = 0; row < count; ++row) {
        entries[row].offset = static_cast<idx_t>(static_cast<std::size_t>(p.offsets[first+row]) - begin);
        entries[row].length = static_cast<idx_t>(p.offsets[first+row+1] - p.offsets[first+row]);
    }
    return {begin, elements};
}
template<class T> void copy_values(duckdb_vector vector, const std::vector<T>& values, std::size_t first, std::size_t count) {
    auto* out = static_cast<T*>(duckdb_vector_get_data(vector));
    if (!out && count) failed("numeric data allocation");
    const auto source = std::span<const T>(values).subspan(first, count);
    const auto target = std::views::counted(out, static_cast<std::ptrdiff_t>(count));
    for (std::size_t i = 0; i < count; ++i) target[i] = source[i];
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void fill(duckdb_vector vector, const Piece& p, const FieldSpec& f, std::size_t first, std::size_t count) {
    // A valid empty list/map has no child lanes to allocate or touch. The
    // recursive piece was already validated before entering the C API seam.
    if (!count) return;
    validity(vector, p, first, count);
    switch (p.kind) {
        case Piece::Kind::scalar_d: copy_values(vector, p.d, first, count); break;
        case Piece::Kind::scalar_i: copy_values(vector, p.i, first, count); break;
        case Piece::Kind::scalar_b: {
            auto* data = static_cast<bool*>(duckdb_vector_get_data(vector));
            if (!data) failed("boolean data allocation");
            const auto values = std::views::counted(data, static_cast<std::ptrdiff_t>(count));
            for (std::size_t i = 0; i < count; ++i) values[i] = p.b[first+i] != 0;
            break;
        }
        case Piece::Kind::scalar_s: strings(vector, p.s, first, count); break;
        case Piece::Kind::list_d: case Piece::Kind::list_i: case Piece::Kind::list_s: {
            const auto [begin, elements] = list_extent(vector, p, first, count);
            const auto child = duckdb_list_vector_get_child(vector);
            duckdb_vector_ensure_validity_writable(child); auto* bits = duckdb_vector_get_validity(child);
            if (!bits && elements) failed("list child validity allocation");
            for (std::size_t i = 0; i < elements; ++i) duckdb_validity_set_row_validity(bits, static_cast<idx_t>(i), true);
            if (p.kind == Piece::Kind::list_d) copy_values(child, p.d, begin, elements);
            else if (p.kind == Piece::Kind::list_i) copy_values(child, p.i, begin, elements);
            else strings(child, p.s, begin, elements);
            break;
        }
        case Piece::Kind::structure:
            if (f.dtype == "struct[]") {
                const auto [begin, elements] = list_extent(vector, p, first, count);
                const auto child = duckdb_list_vector_get_child(vector);
                duckdb_vector_ensure_validity_writable(child); auto* bits = duckdb_vector_get_validity(child);
                if (!bits && elements) failed("struct list validity allocation");
                for (std::size_t i = 0; i < elements; ++i) duckdb_validity_set_row_validity(bits, static_cast<idx_t>(i), true);
                structure(child, p, f, begin, elements);
            } else structure(vector, p, f, first, count);
            break;
        case Piece::Kind::map: {
            const auto [begin, elements] = list_extent(vector, p, first, count);
            const auto entry = duckdb_list_vector_get_child(vector);
            duckdb_vector_ensure_validity_writable(entry); auto* bits = duckdb_vector_get_validity(entry);
            if (!bits && elements) failed("map entry validity allocation");
            for (std::size_t i = 0; i < elements; ++i) duckdb_validity_set_row_validity(bits, static_cast<idx_t>(i), true);
            const auto key = duckdb_struct_vector_get_child(entry, 0);
            duckdb_vector_ensure_validity_writable(key); bits = duckdb_vector_get_validity(key);
            if (!bits && elements) failed("map key validity allocation");
            for (std::size_t i = 0; i < elements; ++i) duckdb_validity_set_row_validity(bits, static_cast<idx_t>(i), true);
            strings(key, p.s, begin, elements);
            auto item = f; item.dtype = f.dtype.substr(4, f.dtype.size()-5);
            fill(duckdb_struct_vector_get_child(entry, 1), *p.items, item, begin, elements);
            break;
        }
    }
}
} // namespace
void append_pieces(Connection const& connection, const TableSpec& spec, const runtime::EncodedTable& batch, std::int64_t episode_id,
    const std::function<void(std::string_view)>& before) {
    validate_pieces(batch, spec);
    if (episode_id < 0) throw std::invalid_argument("negative publication episode identity");
    if (!batch.rows) return;
    std::vector<Logical> owners; std::vector<duckdb_logical_type> types;
    const auto columns = column_types(spec);
    for (const auto& [name, dtype] : columns) {
        (void)dtype; if (const auto* field = spec.field(name)) owners.push_back(logical(*field));
        else owners.emplace_back(duckdb_create_logical_type(DUCKDB_TYPE_BOOLEAN));
    }
    types.reserve(owners.size());
for (const auto& owner : owners) types.push_back(owner.handle);
    Chunk const chunk(types); Appender appender(connection.handle(), spec.name);
    const auto maximum = static_cast<std::size_t>(duckdb_vector_size());
    if (!maximum) failed("zero vector size");
    const auto rows = static_cast<std::size_t>(batch.rows);
    for (std::size_t first = 0; first < rows;) {
        duckdb_data_chunk_reset(chunk.handle);
        const auto count = std::min(maximum, rows-first);
        for (std::size_t index = 0; index < columns.size(); ++index) {
            const auto& name = columns[index].first;
            const auto vector = duckdb_data_chunk_get_vector(chunk.handle, static_cast<idx_t>(index));
            if (name == "episode_id") {
                auto* data = static_cast<std::int64_t*>(duckdb_vector_get_data(vector));
                if (!data) failed("episode identity allocation");
                std::ranges::fill(std::views::counted(data,static_cast<std::ptrdiff_t>(count)), episode_id);
            } else {
                const auto* field = spec.field(name); FieldSpec generated; generated.dtype = "boolean";
                fill(vector, batch.columns.at(name), field ? *field : generated, first, count);
            }
        }
        duckdb_data_chunk_set_size(chunk.handle, static_cast<idx_t>(count));
        if (before) before("chunk_append");
        appender.append(chunk.handle); first += count;
    }
    appender.finish(before);
}
} // namespace bike::recording
