#pragma once

#include "recording/reader.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace bike::recording {

struct ColumnDigest {
    std::string name;
    std::uint64_t rows{};
    std::string sha256;
    double min_val{};
    double max_val{};
    double l2_norm{};
};

struct IntervalDigest {
    std::int64_t interval_id{};
    std::string sha256;
};

struct TableDigest {
    std::string table;
    std::uint64_t row_count{};
    std::vector<ColumnDigest> columns;
    std::vector<IntervalDigest> intervals{};
};

struct DigestReport {
    std::int64_t schema_version = 1;
    std::vector<TableDigest> tables;
    std::string overall_sha256;
};

struct DigestDiff {
    bool equal = true;
    std::string table;
    std::string column;
    std::int64_t interval_id = -1;
    std::string detail;
};

[[nodiscard]] DigestReport compute_recording_digest(const Recording &rec);
[[nodiscard]] DigestDiff compare_recording_digests(const DigestReport &lhs,
                                                   const DigestReport &rhs);
[[nodiscard]] io::Value digest_to_json(const DigestReport &report);
[[nodiscard]] DigestReport digest_from_json(const io::Value &json);

} // namespace bike::recording
