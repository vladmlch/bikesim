#include "export.hpp"
#include "analysis_detail.hpp"
#include "../platform/files.hpp"
#include <map>

namespace bike::recording {
namespace {
using namespace detail;
std::string csv_cell(const Value& value) {
    if (const auto* text_value = std::get_if<std::string>(&value.data)) return *text_value;
    if (const auto* boolean = std::get_if<bool>(&value.data)) return *boolean ? "True" : "False";
    if (const auto* number = std::get_if<io::Number>(&value.data)) return number->lexeme;
    if (const auto* bytes = std::get_if<io::Bytes>(&value.data)) {
        constexpr std::string_view hex = "0123456789abcdef";
        bool single = false, double_quote = false;
        for (auto b : bytes->value) { single = single || b == std::byte{39}; double_quote = double_quote || b == std::byte{34}; }
        const char quote_char = single && !double_quote ? '"' : '\'';
        std::string out = "b"; out += quote_char;
        for (auto b : bytes->value) {
            const auto c = std::to_integer<unsigned char>(b);
            if (c == static_cast<unsigned char>(quote_char) || c == '\\') { out += '\\'; out += static_cast<char>(c); }
            else if (c == '\n') out += "\\n";
            else if (c == '\r') out += "\\r";
            else if (c == '\t') out += "\\t";
            else if (c < 32 || c >= 127) { out += "\\x"; out += hex[c >> 4U]; out += hex[c & 15U]; }
            else out += static_cast<char>(c);
        }
        return out + quote_char;
    }
    if (const auto* tag = std::get_if<io::Tagged>(&value.data); tag && tag->kind != "float64") {
        if(tag->kind=="timedelta") {
            const auto component=[&](std::string_view key) {
                for(const auto& [name,content]:tag->fields)if(name==key)return io::as_int64(Value{io::Number{content}},key);
                throw std::invalid_argument("incomplete timedelta export value");
            };
            const auto days=component("days"),seconds=component("seconds"),micros=component("microseconds");
            if(seconds<0||seconds>=86400||micros<0||micros>=1000000)throw std::invalid_argument("timedelta must be normalized before CSV export");
            const auto padded=[](std::int64_t number,std::size_t width) {auto out=std::to_string(number);if(out.size()<width)out.insert(0,width-out.size(),'0');return out;};
            std::string out;
            if(days)out=std::to_string(days)+(days==1||days==-1?" day, ":" days, ");
            out+=std::to_string(seconds/3600)+':'+padded((seconds/60)%60,2)+':'+padded(seconds%60,2);
            if(micros)out+='.'+padded(micros,6);
            return out;
        }
        for (const auto& [name, content] : tag->fields) if (name == "value") {
            auto out = content;
            if (tag->kind == "datetime") if (const auto t = out.find('T'); t != std::string::npos) out[t] = ' ';
            return out;
        }
    }
    return io::recording_json(value);
}
std::string csv_row(const std::vector<std::string>& values) {
    std::string out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out += ',';
        const auto& value = values[i]; const bool quoted = value.find_first_of(",\"\r\n") != std::string::npos || (values.size() == 1 && value.empty());
        if (quoted) out += '"';
        for (char c : value) { out += c; if (c == '"') out += '"'; }
        if (quoted) out += '"';
    }
    return out + "\r\n";
}
void write(const platform::FileHandle& file, const std::string& text_value) { platform::write_all(file, std::as_bytes(std::span(text_value.data(), text_value.size()))); }
} // namespace
io::Value export_data(const Recording& recording, std::int64_t episode, std::string_view table,
    std::string_view format, const std::filesystem::path& output, std::optional<std::uint64_t> maximum) {
    (void)recording.episode(episode);
    const std::map<std::string_view, std::string_view> order{{"telemetry", "interval_id"}, {"legacy_telemetry", "interval_id"},
        {"forces", "interval_id,component"}, {"commands_requested", "request_id"}, {"commands_applied", "application_id"},
        {"observations", "observation_id"}, {"transitions", "transition_id"}, {"events", "event_id"}, {"metrics", "name"}};
    if (!order.contains(table) || (format != "csv" && format != "jsonl")) throw std::invalid_argument("unsupported explicit recording export");
    std::vector<Value> parameters{integer(episode)};
    auto sql = "SELECT * FROM " + quote(table) + " WHERE episode_id=? ORDER BY " + std::string(order.at(table));
    if (maximum) { limit(*maximum); sql += " LIMIT ?"; parameters.push_back(integer(static_cast<std::int64_t>(*maximum))); }
    auto cursor = recording.stream(sql, parameters);
    auto file = platform::open_exclusive(output);
    if (format == "csv") write(file, csv_row(cursor.columns()));
    std::int64_t rows = 0;
    for (;;) {
        const auto batch = cursor.next_batch(4096);
        if (batch.empty()) break;
        for (const auto& row : batch) {
            if (format == "jsonl") write(file, io::recording_json(Value{row}) + '\n');
            else {
                std::vector<std::string> values; values.reserve(cursor.columns().size());
                for (const auto& column : cursor.columns()) values.push_back(csv_cell(field(row, column)));
                write(file, csv_row(values));
            }
            if (rows == std::numeric_limits<std::int64_t>::max()) throw std::overflow_error("export row count overflow");
            ++rows;
        }
    }
    file.close();
    return Value{Object{{"output", text(output.string())}, {"format", text(format)}, {"table", text(table)}, {"rows", integer(rows)},
        {"semantics", text("explicit analytics export, not a checked-replay bundle")}}};
}
void export_table(const Recording& recording, std::int64_t episode, std::string_view table, std::string_view format,
    const std::filesystem::path& output, std::optional<std::uint64_t> limit_value) {
    (void)export_data(recording, episode, table, format, output, limit_value);
}
} // namespace bike::recording
