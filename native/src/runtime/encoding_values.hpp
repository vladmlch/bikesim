// Owned typed column payloads shared by the runtime projector and the
// engine-free recording analyst. No engine/model/controller headers.
#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace runtime {
// One column's raw payload; kind selects which lanes are live. The numeric
// lanes retain their original bit patterns until typed DuckDB ingestion.
struct Piece {
    enum class Kind : std::uint8_t { scalar_d, scalar_i, scalar_b, scalar_s,
        list_d, list_i, list_s, structure, map } kind = Kind::scalar_d;
    std::vector<double> d;
    std::vector<std::int64_t> i;
    std::vector<char> b;
    std::vector<std::string> s;
    std::vector<std::int32_t> offsets;
    std::vector<char> valid; // empty means all-valid
    std::map<std::string, Piece> children;
    // Runtime conversion cache: children are stable std::map nodes. A whole
    // Piece may move, but copying it would retain pointers into its source.
    struct MemberSlot { Piece* member = nullptr; Piece* present = nullptr; };
    std::vector<MemberSlot> member_slots;
    std::unique_ptr<Piece> items;
    Piece() = default;
    Piece(const Piece&) = delete;
    Piece& operator=(const Piece&) = delete;
    Piece(Piece&&) = default;
    Piece& operator=(Piece&&) = default;
    ~Piece() = default;
};
struct EncodedTable {
    std::int64_t rows = 0;
    std::map<std::string, Piece> columns;
};
} // namespace runtime
