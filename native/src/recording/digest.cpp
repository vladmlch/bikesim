#include "recording/digest.hpp"

#include "io/hash.hpp"
#include "recording/analysis_detail.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::recording {
namespace {

constexpr std::size_t kDigestBatchRows = 4096;

constexpr std::array<std::byte, 8> le_u64(std::uint64_t value) noexcept {
    std::array<std::byte, 8> out{};
    for (std::size_t i = 0; i < 8; ++i) {
        out[i] = static_cast<std::byte>((value >> (i * 8U)) & 0xffU);
    }
    return out;
}

std::string finish_hex(io::Sha256 &hasher) {
    static constexpr std::string_view kHex = "0123456789abcdef";
    const auto digest = hasher.finish();
    std::string out;
    out.reserve(64);
    for (const std::uint8_t byte : digest) {
        out.push_back(kHex[byte >> 4U]);
        out.push_back(kHex[byte & 0x0fU]);
    }
    return out;
}

void hash_length_prefixed(io::Sha256 &hasher, std::string_view text) noexcept {
    const auto len = le_u64(static_cast<std::uint64_t>(text.size()));
    hasher.update(len);
    const auto bytes = std::as_bytes(std::span<const char>{text.data(), text.size()});
    hasher.update(bytes);
}

struct ColumnAccumulator {
    std::string name;
    std::uint64_t rows = 0;
    io::Sha256 hasher;
    double min_val = 0.0;
    double max_val = 0.0;
    double sum_sq = 0.0;
    bool has_numeric = false;
    bool saw_nan = false;

    void add_double(double val, io::Sha256 *row_hasher = nullptr) {
        const auto bits = std::bit_cast<std::uint64_t>(val);
        const auto bytes = le_u64(bits);
        hasher.update(bytes);
        if (row_hasher != nullptr) {
            hash_length_prefixed(*row_hasher, name);
            row_hasher->update(bytes);
        }
        if (std::isnan(val)) {
            saw_nan = true;
            min_val = val;
            max_val = val;
        } else if (!saw_nan) {
            if (!has_numeric) {
                min_val = val;
                max_val = val;
            } else {
                min_val = std::min(min_val, val);
                max_val = std::max(max_val, val);
            }
            sum_sq += val * val;
        }
        has_numeric = true;
        ++rows;
    }

    void add_int64(std::int64_t val, io::Sha256 *row_hasher = nullptr) {
        const auto bits = static_cast<std::uint64_t>(val);
        const auto bytes = le_u64(bits);
        hasher.update(bytes);
        if (row_hasher != nullptr) {
            hash_length_prefixed(*row_hasher, name);
            row_hasher->update(bytes);
        }
        const auto as_d = static_cast<double>(val);
        if (!has_numeric) {
            min_val = as_d;
            max_val = as_d;
        } else {
            min_val = std::min(min_val, as_d);
            max_val = std::max(max_val, as_d);
        }
        sum_sq += as_d * as_d;
        has_numeric = true;
        ++rows;
    }

    void add_string(std::string_view val, io::Sha256 *row_hasher = nullptr) {
        hash_length_prefixed(hasher, val);
        if (row_hasher != nullptr) {
            hash_length_prefixed(*row_hasher, name);
            hash_length_prefixed(*row_hasher, val);
        }
        ++rows;
    }

    [[nodiscard]] ColumnDigest finish() {
        ColumnDigest out;
        out.name = name;
        out.rows = rows;
        out.sha256 = finish_hex(hasher);
        if (has_numeric) {
            out.min_val = min_val;
            out.max_val = max_val;
            out.l2_norm = saw_nan ? min_val : std::sqrt(sum_sq);
        }
        return out;
    }
};

struct CoordNames {
    std::vector<std::string> qpos;
    std::vector<std::string> qvel;
    std::vector<std::string> qfrc;
};

struct CoordSpec {
    std::string_view joint;
    std::string_view component;
    std::string_view unit;
};

std::string format_qpos_name(const CoordSpec &coord) {
    std::string base(coord.joint);
    if (!coord.component.empty()) {
        base += '.';
        base += coord.component;
    }
    if (coord.unit == "m") return base + "_m";
    if (coord.unit == "rad") return base + "_rad";
    if (coord.unit == "1") return base + "_1";
    return base + "_qpos";
}

std::string format_qvel_name(const CoordSpec &coord) {
    std::string base(coord.joint);
    if (!coord.component.empty()) {
        base += '.';
        base += coord.component;
    }
    if (coord.unit == "m/s") return base + "_mps";
    if (coord.unit == "rad/s") return base + "_rad_s";
    return base + "_qvel";
}

std::string format_qfrc_name(const CoordSpec &coord) {
    std::string base = "qfrc.";
    base += coord.joint;
    if (!coord.component.empty()) {
        base += '.';
        base += coord.component;
    }
    return base;
}

bool has_base_table(const Recording &rec, std::string_view name) {
    const std::array<io::Value, 1> params{detail::text(name)};
    const auto rows = rec.rows(
        "SELECT count(*) AS n FROM information_schema.tables "
        "WHERE table_schema='main' AND table_type='BASE TABLE' AND table_name=?",
        params);
    if (rows.empty()) return false;
    return io::as_int64(detail::field(rows.front(), "n"), "table count") > 0;
}

std::map<std::int64_t, CoordNames> load_coordinate_names(const Recording &rec) {
    std::map<std::int64_t, CoordNames> out;
    const auto rows = rec.rows(
        "SELECT episode_id, kind, coordinate_index, joint, component, unit "
        "FROM coordinate_catalog ORDER BY episode_id, kind, coordinate_index");
    for (const auto &row : rows) {
        const auto ep = io::as_int64(detail::field(row, "episode_id"), "episode_id");
        const auto &kind = io::as_string(detail::field(row, "kind"), "kind");
        const auto idx = io::as_int64(detail::field(row, "coordinate_index"), "coordinate_index");
        const auto &joint = io::as_string(detail::field(row, "joint"), "joint");
        const auto &comp = io::as_string(detail::field(row, "component"), "component");
        const auto &unit = io::as_string(detail::field(row, "unit"), "unit");
        if (idx < 0) continue;
        auto &entry = out[ep];
        const auto u_idx = static_cast<std::size_t>(idx);
        const CoordSpec spec{.joint = joint, .component = comp, .unit = unit};
        if (kind == "qpos") {
            if (entry.qpos.size() <= u_idx) entry.qpos.resize(u_idx + 1);
            entry.qpos[u_idx] = format_qpos_name(spec);
        } else if (kind == "dof") {
            if (entry.qvel.size() <= u_idx) entry.qvel.resize(u_idx + 1);
            if (entry.qfrc.size() <= u_idx) entry.qfrc.resize(u_idx + 1);
            entry.qvel[u_idx] = format_qvel_name(spec);
            entry.qfrc[u_idx] = format_qfrc_name(spec);
        }
    }
    return out;
}

const CoordNames *lookup_coords(const std::map<std::int64_t, CoordNames> &coords,
                                std::int64_t episode_id) {
    if (const auto it = coords.find(episode_id); it != coords.end()) {
        return &it->second;
    }
    if (!coords.empty()) {
        return &coords.begin()->second;
    }
    return nullptr;
}

std::string qpos_col_name(const CoordNames *coords, std::size_t idx) {
    if (coords != nullptr && idx < coords->qpos.size() && !coords->qpos[idx].empty()) {
        return coords->qpos[idx];
    }
    return "qpos[" + std::to_string(idx) + "]";
}

std::string qvel_col_name(const CoordNames *coords, std::size_t idx) {
    if (coords != nullptr && idx < coords->qvel.size() && !coords->qvel[idx].empty()) {
        return coords->qvel[idx];
    }
    return "qvel[" + std::to_string(idx) + "]";
}

std::string qfrc_col_name(const CoordNames *coords, std::size_t idx) {
    if (coords != nullptr && idx < coords->qfrc.size() && !coords->qfrc[idx].empty()) {
        return coords->qfrc[idx];
    }
    return "qfrc[" + std::to_string(idx) + "]";
}

ColumnAccumulator &get_or_create_col(std::map<std::string, ColumnAccumulator, std::less<>> &cols,
                                     const std::string &name) {
    auto [it, inserted] = cols.try_emplace(name);
    if (inserted) it->second.name = name;
    return it->second;
}

TableDigest digest_physics(const Recording &rec,
                           const std::map<std::int64_t, CoordNames> &coords) {
    TableDigest table;
    table.table = "physics";

    std::map<std::string, ColumnAccumulator, std::less<>> cols;
    for (const auto *scalar_name :
         {"dt_s", "episode_id", "interval_end_s", "interval_id", "time_s"}) {
        (void)get_or_create_col(cols, scalar_name);
    }
    if (const auto *c0 = lookup_coords(coords, 0)) {
        for (std::size_t i = 0; i < c0->qpos.size(); ++i) {
            (void)get_or_create_col(cols, qpos_col_name(c0, i));
        }
        for (std::size_t i = 0; i < c0->qvel.size(); ++i) {
            (void)get_or_create_col(cols, qvel_col_name(c0, i));
        }
    }

    auto cursor = rec.stream(
        "SELECT episode_id, interval_id, time_s, interval_end_s, dt_s, qpos, qvel "
        "FROM physics ORDER BY episode_id, interval_id");
    for (;;) {
        const auto batch = cursor.next_batch(kDigestBatchRows);
        if (batch.empty()) break;
        for (const auto &row : batch) {
            ++table.row_count;
            const auto ep = io::as_int64(detail::field(row, "episode_id"), "physics.episode_id");
            const auto interval_id =
                io::as_int64(detail::field(row, "interval_id"), "physics.interval_id");
            const double time_s = io::tagged_double(detail::field(row, "time_s"), "physics.time_s");
            const double end_s =
                io::tagged_double(detail::field(row, "interval_end_s"), "physics.interval_end_s");
            const double dt_s = io::tagged_double(detail::field(row, "dt_s"), "physics.dt_s");
            const auto &qpos = io::as_array(detail::field(row, "qpos"), "physics.qpos");
            const auto &qvel = io::as_array(detail::field(row, "qvel"), "physics.qvel");
            const auto *ep_coords = lookup_coords(coords, ep);

            io::Sha256 row_hasher;
            // Collect per-row entries in canonical sorted column order for row_hasher.
            std::vector<std::pair<std::string, double>> dbl_entries;
            dbl_entries.reserve(3 + qpos.size() + qvel.size());
            dbl_entries.emplace_back("dt_s", dt_s);
            dbl_entries.emplace_back("interval_end_s", end_s);
            dbl_entries.emplace_back("time_s", time_s);
            for (std::size_t i = 0; i < qpos.size(); ++i) {
                dbl_entries.emplace_back(
                    qpos_col_name(ep_coords, i),
                    io::tagged_double(qpos[i], "physics.qpos"));
            }
            for (std::size_t i = 0; i < qvel.size(); ++i) {
                dbl_entries.emplace_back(
                    qvel_col_name(ep_coords, i),
                    io::tagged_double(qvel[i], "physics.qvel"));
            }
            std::ranges::sort(dbl_entries, {}, &std::pair<std::string, double>::first);

            // Feed row_hasher in sorted column name order (interleaving episode_id and interval_id).
            bool fed_episode = false;
            bool fed_interval = false;
            for (const auto &[cname, val] : dbl_entries) {
                if (!fed_episode && std::string_view("episode_id") < cname) {
                    get_or_create_col(cols, "episode_id").add_int64(ep, &row_hasher);
                    fed_episode = true;
                }
                if (!fed_interval && std::string_view("interval_id") < cname) {
                    get_or_create_col(cols, "interval_id").add_int64(interval_id, &row_hasher);
                    fed_interval = true;
                }
                get_or_create_col(cols, cname).add_double(val, &row_hasher);
            }
            if (!fed_episode) {
                get_or_create_col(cols, "episode_id").add_int64(ep, &row_hasher);
            }
            if (!fed_interval) {
                get_or_create_col(cols, "interval_id").add_int64(interval_id, &row_hasher);
            }

            table.intervals.push_back(IntervalDigest{
                .interval_id = interval_id,
                .sha256 = finish_hex(row_hasher),
            });
        }
    }

    table.columns.reserve(cols.size());
    for (auto &[name, acc] : cols) {
        (void)name;
        table.columns.push_back(acc.finish());
    }
    return table;
}

TableDigest digest_forces(const Recording &rec,
                          const std::map<std::int64_t, CoordNames> &coords) {
    TableDigest table;
    table.table = "forces";

    std::map<std::string, ColumnAccumulator, std::less<>> cols;
    for (const auto *scalar_name : {"component", "episode_id", "interval_id", "power_w"}) {
        (void)get_or_create_col(cols, scalar_name);
    }
    if (const auto *c0 = lookup_coords(coords, 0)) {
        for (std::size_t i = 0; i < c0->qfrc.size(); ++i) {
            (void)get_or_create_col(cols, qfrc_col_name(c0, i));
        }
    }

    auto cursor = rec.stream(
        "SELECT episode_id, interval_id, component, qfrc, power_w "
        "FROM forces ORDER BY episode_id, interval_id, component");
    bool has_active_interval = false;
    std::int64_t active_ep = 0;
    std::int64_t active_interval = 0;
    io::Sha256 interval_hasher;

    const auto flush_interval = [&]() {
        if (!has_active_interval) return;
        table.intervals.push_back(IntervalDigest{
            .interval_id = active_interval,
            .sha256 = finish_hex(interval_hasher),
        });
        interval_hasher = io::Sha256{};
        has_active_interval = false;
    };

    for (;;) {
        const auto batch = cursor.next_batch(kDigestBatchRows);
        if (batch.empty()) break;
        for (const auto &row : batch) {
            ++table.row_count;
            const auto ep = io::as_int64(detail::field(row, "episode_id"), "forces.episode_id");
            const auto interval_id =
                io::as_int64(detail::field(row, "interval_id"), "forces.interval_id");
            const auto &component =
                io::as_string(detail::field(row, "component"), "forces.component");
            const double power_w =
                io::tagged_double(detail::field(row, "power_w"), "forces.power_w");
            const auto &qfrc = io::as_array(detail::field(row, "qfrc"), "forces.qfrc");
            const auto *ep_coords = lookup_coords(coords, ep);

            if (!has_active_interval || ep != active_ep || interval_id != active_interval) {
                flush_interval();
                has_active_interval = true;
                active_ep = ep;
                active_interval = interval_id;
            }

            std::vector<std::pair<std::string, double>> dbl_entries;
            dbl_entries.reserve(1 + qfrc.size());
            dbl_entries.emplace_back("power_w", power_w);
            for (std::size_t i = 0; i < qfrc.size(); ++i) {
                dbl_entries.emplace_back(
                    qfrc_col_name(ep_coords, i),
                    io::tagged_double(qfrc[i], "forces.qfrc"));
            }
            std::ranges::sort(dbl_entries, {}, &std::pair<std::string, double>::first);

            // Feed columns in canonical alphabetical order: component, episode_id, interval_id, power_w, qfrc.*
            get_or_create_col(cols, "component").add_string(component, &interval_hasher);
            get_or_create_col(cols, "episode_id").add_int64(ep, &interval_hasher);
            get_or_create_col(cols, "interval_id").add_int64(interval_id, &interval_hasher);
            for (const auto &[cname, val] : dbl_entries) {
                get_or_create_col(cols, cname).add_double(val, &interval_hasher);
            }
        }
    }
    flush_interval();

    table.columns.reserve(cols.size());
    for (auto &[name, acc] : cols) {
        (void)name;
        table.columns.push_back(acc.finish());
    }
    return table;
}

std::string compute_overall_sha256(std::int64_t schema_version,
                                   const std::vector<TableDigest> &tables) {
    std::vector<const TableDigest *> sorted_tables;
    sorted_tables.reserve(tables.size());
    for (const auto &tbl : tables) sorted_tables.push_back(&tbl);
    std::ranges::sort(sorted_tables, [](const TableDigest *a, const TableDigest *b) {
        return a->table < b->table;
    });

    io::Sha256 overall;
    overall.update(le_u64(static_cast<std::uint64_t>(schema_version)));
    for (const TableDigest *tbl : sorted_tables) {
        hash_length_prefixed(overall, tbl->table);
        overall.update(le_u64(tbl->row_count));
        std::vector<const ColumnDigest *> sorted_cols;
        sorted_cols.reserve(tbl->columns.size());
        for (const auto &col : tbl->columns) sorted_cols.push_back(&col);
        std::ranges::sort(sorted_cols, [](const ColumnDigest *a, const ColumnDigest *b) {
            return a->name < b->name;
        });
        for (const ColumnDigest *col : sorted_cols) {
            hash_length_prefixed(overall, col->name);
            overall.update(le_u64(col->rows));
            hash_length_prefixed(overall, col->sha256);
        }
    }
    return finish_hex(overall);
}

std::int64_t find_first_differing_interval(const TableDigest &lhs,
                                           const TableDigest &rhs) {
    const std::size_t n = std::min(lhs.intervals.size(), rhs.intervals.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (lhs.intervals[i].interval_id != rhs.intervals[i].interval_id) {
            return std::min(lhs.intervals[i].interval_id, rhs.intervals[i].interval_id);
        }
        if (lhs.intervals[i].sha256 != rhs.intervals[i].sha256) {
            return lhs.intervals[i].interval_id;
        }
    }
    if (lhs.intervals.size() > n) return lhs.intervals[n].interval_id;
    if (rhs.intervals.size() > n) return rhs.intervals[n].interval_id;
    return -1;
}

std::string find_first_differing_column(const TableDigest &lhs,
                                        const TableDigest &rhs,
                                        std::string &detail_out) {
    std::map<std::string_view, const ColumnDigest *> lhs_map;
    std::map<std::string_view, const ColumnDigest *> rhs_map;
    std::set<std::string_view> names;
    for (const auto &col : lhs.columns) {
        lhs_map.emplace(col.name, &col);
        names.insert(col.name);
    }
    for (const auto &col : rhs.columns) {
        rhs_map.emplace(col.name, &col);
        names.insert(col.name);
    }
    for (const auto name : names) {
        const auto lit = lhs_map.find(name);
        const auto rit = rhs_map.find(name);
        if (lit == lhs_map.end()) {
            detail_out = "column missing in lhs";
            return std::string(name);
        }
        if (rit == rhs_map.end()) {
            detail_out = "column missing in rhs";
            return std::string(name);
        }
        if (lit->second->rows != rit->second->rows) {
            detail_out = "row count mismatch (" + std::to_string(lit->second->rows) +
                         " vs " + std::to_string(rit->second->rows) + ")";
            return std::string(name);
        }
        if (lit->second->sha256 != rit->second->sha256) {
            detail_out = "column sha256 mismatch (" + lit->second->sha256 + " vs " +
                         rit->second->sha256 + ")";
            return std::string(name);
        }
    }
    return {};
}

double extract_json_double(const io::Value &value, std::string_view field) {
    if (const auto *obj = std::get_if<io::Value::Object>(&value.data)) {
        const auto *tag = io::find_member(*obj, io::kRecordingTag);
        const auto *val = io::find_member(*obj, "value");
        if (tag != nullptr && val != nullptr && obj->size() == 2 &&
            io::as_string(*tag, field) == "float64") {
            const auto &spelling = io::as_string(*val, field);
            if (spelling == "nan") return std::numeric_limits<double>::quiet_NaN();
            if (spelling == "+inf") return std::numeric_limits<double>::infinity();
            if (spelling == "-inf") return -std::numeric_limits<double>::infinity();
        }
    }
    return io::tagged_double(value, field);
}

std::uint64_t extract_u64(const io::Value &value, std::string_view field) {
    const auto signed_val = io::as_int64(value, field);
    if (signed_val < 0) {
        throw std::invalid_argument(std::string(field) + ": expected nonnegative integer");
    }
    return static_cast<std::uint64_t>(signed_val);
}

} // namespace

DigestReport compute_recording_digest(const Recording &rec) {
    if (rec.schema_version() != 1) {
        throw RecordingError("digest supports schema_version == 1 only");
    }
    if (!has_base_table(rec, "physics")) {
        throw RecordingError("recording is missing required table: physics");
    }
    if (!has_base_table(rec, "forces")) {
        throw RecordingError("recording is missing required table: forces");
    }
    const auto coords = load_coordinate_names(rec);
    DigestReport report;
    report.schema_version = rec.schema_version();
    report.tables.push_back(digest_physics(rec, coords));
    report.tables.push_back(digest_forces(rec, coords));
    report.overall_sha256 = compute_overall_sha256(report.schema_version, report.tables);
    return report;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (lhs, rhs) is the required public signature
DigestDiff compare_recording_digests(const DigestReport &lhs, const DigestReport &rhs) {
    if (lhs.schema_version != rhs.schema_version) {
        return DigestDiff{
            .equal = false,
            .table = {},
            .column = {},
            .interval_id = -1,
            .detail = "schema_version mismatch (" + std::to_string(lhs.schema_version) +
                      " vs " + std::to_string(rhs.schema_version) + ")",
        };
    }

    std::map<std::string_view, const TableDigest *> lhs_tables;
    std::map<std::string_view, const TableDigest *> rhs_tables;
    std::vector<std::string> ordered_names;
    std::set<std::string_view> seen;
    for (const auto *preferred : {"physics", "forces"}) {
        seen.insert(preferred);
        ordered_names.emplace_back(preferred);
    }
    for (const auto &tbl : lhs.tables) {
        lhs_tables.emplace(tbl.table, &tbl);
        if (seen.insert(tbl.table).second) ordered_names.push_back(tbl.table);
    }
    for (const auto &tbl : rhs.tables) {
        rhs_tables.emplace(tbl.table, &tbl);
        if (seen.insert(tbl.table).second) ordered_names.push_back(tbl.table);
    }
    if (ordered_names.size() > 2) {
        std::sort(ordered_names.begin() + 2, ordered_names.end());
    }

    for (const auto &tname : ordered_names) {
        const auto lit = lhs_tables.find(tname);
        const auto rit = rhs_tables.find(tname);
        if (lit == lhs_tables.end() && rit == rhs_tables.end()) {
            continue;
        }
        if (lit == lhs_tables.end() || rit == rhs_tables.end()) {
            return DigestDiff{
                .equal = false,
                .table = tname,
                .column = {},
                .interval_id = -1,
                .detail = lit == lhs_tables.end() ? "table missing in lhs" : "table missing in rhs",
            };
        }
        std::string col_detail;
        const std::string diff_col = find_first_differing_column(*lit->second, *rit->second, col_detail);
        if (!diff_col.empty()) {
            const std::int64_t diff_interval =
                find_first_differing_interval(*lit->second, *rit->second);
            return DigestDiff{
                .equal = false,
                .table = tname,
                .column = diff_col,
                .interval_id = diff_interval,
                .detail = std::move(col_detail),
            };
        }
        if (lit->second->row_count != rit->second->row_count) {
            const std::int64_t diff_interval =
                find_first_differing_interval(*lit->second, *rit->second);
            return DigestDiff{
                .equal = false,
                .table = tname,
                .column = {},
                .interval_id = diff_interval,
                .detail = "table row_count mismatch",
            };
        }
    }

    const std::string lhs_overall =
        lhs.overall_sha256.empty() ? compute_overall_sha256(lhs.schema_version, lhs.tables)
                                   : lhs.overall_sha256;
    const std::string rhs_overall =
        rhs.overall_sha256.empty() ? compute_overall_sha256(rhs.schema_version, rhs.tables)
                                   : rhs.overall_sha256;
    if (lhs_overall != rhs_overall) {
        return DigestDiff{
            .equal = false,
            .table = {},
            .column = {},
            .interval_id = -1,
            .detail = "overall_sha256 mismatch",
        };
    }

    return DigestDiff{};
}

io::Value digest_to_json(const DigestReport &report) {
    io::Value::Array tables_json;
    tables_json.reserve(report.tables.size());
    for (const auto &tbl : report.tables) {
        io::Value::Array cols_json;
        cols_json.reserve(tbl.columns.size());
        for (const auto &col : tbl.columns) {
            cols_json.emplace_back(io::Value::Object{
                {"name", detail::text(col.name)},
                {"rows", detail::integer(static_cast<std::int64_t>(col.rows))},
                {"sha256", detail::text(col.sha256)},
                {"min_val", detail::real(col.min_val)},
                {"max_val", detail::real(col.max_val)},
                {"l2_norm", detail::real(col.l2_norm)},
            });
        }
        io::Value::Array intervals_json;
        intervals_json.reserve(tbl.intervals.size());
        for (const auto &iv : tbl.intervals) {
            intervals_json.emplace_back(io::Value::Array{
                detail::integer(iv.interval_id),
                detail::text(iv.sha256),
            });
        }
        tables_json.emplace_back(io::Value::Object{
            {"table", detail::text(tbl.table)},
            {"row_count", detail::integer(static_cast<std::int64_t>(tbl.row_count))},
            {"columns", io::Value{std::move(cols_json)}},
            {"intervals", io::Value{std::move(intervals_json)}},
        });
    }
    return io::Value{io::Value::Object{
        {"schema_version", detail::integer(report.schema_version)},
        {"overall_sha256", detail::text(report.overall_sha256)},
        {"tables", io::Value{std::move(tables_json)}},
    }};
}

DigestReport digest_from_json(const io::Value &json) {
    const auto &root = io::as_object(json, "digest");
    DigestReport out;
    out.schema_version = io::as_int64(detail::field(root, "schema_version"), "digest.schema_version");
    if (out.schema_version != 1) {
        throw std::invalid_argument("digest.schema_version must be 1");
    }
    out.overall_sha256 = io::as_string(detail::field(root, "overall_sha256"), "digest.overall_sha256");
    const auto &tables_arr = io::as_array(detail::field(root, "tables"), "digest.tables");
    out.tables.reserve(tables_arr.size());
    for (const auto &tval : tables_arr) {
        const auto &tobj = io::as_object(tval, "digest.table");
        TableDigest tbl;
        tbl.table = io::as_string(detail::field(tobj, "table"), "digest.table.table");
        tbl.row_count = extract_u64(detail::field(tobj, "row_count"), "digest.table.row_count");
        const auto &cols_arr = io::as_array(detail::field(tobj, "columns"), "digest.table.columns");
        tbl.columns.reserve(cols_arr.size());
        for (const auto &cval : cols_arr) {
            const auto &cobj = io::as_object(cval, "digest.column");
            ColumnDigest col;
            col.name = io::as_string(detail::field(cobj, "name"), "digest.column.name");
            col.rows = extract_u64(detail::field(cobj, "rows"), "digest.column.rows");
            col.sha256 = io::as_string(detail::field(cobj, "sha256"), "digest.column.sha256");
            col.min_val = extract_json_double(detail::field(cobj, "min_val"), "digest.column.min_val");
            col.max_val = extract_json_double(detail::field(cobj, "max_val"), "digest.column.max_val");
            col.l2_norm = extract_json_double(detail::field(cobj, "l2_norm"), "digest.column.l2_norm");
            tbl.columns.push_back(std::move(col));
        }
        if (const auto *iv_ptr = detail::find(tobj, "intervals")) {
            const auto &iv_arr = io::as_array(*iv_ptr, "digest.table.intervals");
            tbl.intervals.reserve(iv_arr.size());
            for (const auto &ival : iv_arr) {
                const auto &pair_arr = io::as_array(ival, "digest.interval");
                if (pair_arr.size() != 2) {
                    throw std::invalid_argument("digest.interval must be [interval_id, sha256]");
                }
                tbl.intervals.push_back(IntervalDigest{
                    .interval_id = io::as_int64(pair_arr[0], "digest.interval.interval_id"),
                    .sha256 = io::as_string(pair_arr[1], "digest.interval.sha256"),
                });
            }
        }
        out.tables.push_back(std::move(tbl));
    }
    return out;
}

} // namespace bike::recording
