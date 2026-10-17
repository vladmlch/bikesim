#include "reader.hpp"
#include "analysis_detail.hpp"
#include <iterator>

namespace bike::recording {
io::Value Recording::query(std::string_view sql, std::uint64_t maximum, std::span<const io::Value> parameters) const {
    detail::limit(maximum);
    auto cursor = stream(sql, parameters);
    detail::Array columns;
    for (const auto& name : cursor.columns()) columns.push_back(detail::text(name));
    std::vector<io::Value::Object> rows;
    while (rows.size() <= maximum) {
        const auto wanted = std::min<std::uint64_t>(4096, maximum + 1 - rows.size());
        auto batch = cursor.next_batch(static_cast<std::size_t>(wanted));
        if (batch.empty()) break;
        rows.insert(rows.end(), std::make_move_iterator(batch.begin()), std::make_move_iterator(batch.end()));
    }
    auto out = detail::bounded(std::move(rows), maximum);
    detail::set(out, "columns", io::Value{std::move(columns)});
    return out;
}
io::Value query(const Recording& recording, std::string_view sql, std::uint64_t maximum) { return recording.query(sql, maximum); }
} // namespace bike::recording
