#pragma once
#include "catalog.hpp"
#include "../runtime/encoding_values.hpp"
#include <functional>
#include <span>

namespace runtime { struct EncodePlan; }

namespace bike::recording {
[[nodiscard]] runtime::EncodePlan compile_plan(const Catalog& catalog, std::string_view table);
// Generic source adapter. Number/Tagged cells preserve the reviewed lossless
// metadata domain; optimized physical export uses encode_table_native directly.
[[nodiscard]] runtime::EncodedTable encode_rows(const TableSpec& table,
    std::span<const io::Value::Object> rows, std::int64_t episode_id);
void validate_pieces(const runtime::EncodedTable& table, const Catalog& catalog,
                     std::string_view table_name);
void validate_pieces(const runtime::EncodedTable& table, const TableSpec& spec);
[[nodiscard]] runtime::Piece slice_piece(const runtime::Piece& piece, std::size_t first, std::size_t last);
[[nodiscard]] runtime::EncodedTable slice_table(const runtime::EncodedTable&, std::size_t first, std::size_t last);
using BatchSink = std::function<void(std::string_view, runtime::EncodedTable&&)>;
void piece_fragments(std::string_view name, runtime::EncodedTable table,
                     std::size_t max_rows, const BatchSink& sink);
class Connection;
void append_pieces(Connection const& connection, const TableSpec& spec,
                   const runtime::EncodedTable& batch, std::int64_t episode_id,
                   const std::function<void(std::string_view)>& before_stage = {});
} // namespace bike::recording
