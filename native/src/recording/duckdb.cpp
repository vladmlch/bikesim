#include "duckdb.hpp"
#include "analysis_detail.hpp"
#include "../platform/files.hpp"
#include "../io/hash.hpp"
#include <array>
#include <chrono>
#include <cstring>
#include <iterator>
#include <limits>
#include <ranges>
#include <set>
#include <type_traits>
#include <cerrno>
#include <system_error>
#include <utility>
#include <sys/stat.h>
#include <unistd.h>

namespace bike::recording {
namespace {
using namespace detail;

[[noreturn]] void failed(std::string_view operation, const char* message) {
    throw std::invalid_argument("recording DuckDB " + std::string(operation) + ": " +
        (message ? std::string(message) : "operation failed without an engine diagnostic"));
}
struct Logical {
    duckdb_logical_type value = nullptr;
    explicit Logical(duckdb_logical_type type) : value(type) {
        if (!value) failed("type", "missing logical type");
    }
    ~Logical() { duckdb_destroy_logical_type(&value); }
    Logical(const Logical&) = delete;
    Logical& operator=(const Logical&) = delete;
    Logical(Logical&&) = delete;
    Logical& operator=(Logical&&) = delete;
};
struct Prepared {
    duckdb_prepared_statement value = nullptr;
    Prepared() = default;
    ~Prepared() { if (value) duckdb_destroy_prepare(&value); }
    Prepared(const Prepared&) = delete;
    Prepared& operator=(const Prepared&) = delete;
    Prepared(Prepared&&) = delete;
    Prepared& operator=(Prepared&&) = delete;
};
struct Extracted {
    duckdb_extracted_statements value = nullptr;
    Extracted() = default;
    ~Extracted() { if (value) duckdb_destroy_extracted(&value); }
    Extracted(const Extracted&) = delete;
    Extracted& operator=(const Extracted&) = delete;
    Extracted(Extracted&&) = delete;
    Extracted& operator=(Extracted&&) = delete;
};
struct Config {
    duckdb_config value = nullptr;
    Config() { if (duckdb_create_config(&value) == DuckDBError) failed("config", nullptr); }
    ~Config() { if (value) duckdb_destroy_config(&value); }
    void set(const char* name, const char* text_value) {
        if (duckdb_set_config(value, name, text_value) == DuckDBError) failed("config", name);
    }
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(Config&&) = delete;
};
struct EngineStringDeleter { void operator()(char* value) const noexcept { duckdb_free(value); } };
using EngineString = std::unique_ptr<char, EngineStringDeleter>;

void prepare(duckdb_connection connection, std::string_view sql, bool select_only, Prepared& out) {
    if (sql.empty() || sql.find('\0') != std::string_view::npos)
        throw std::invalid_argument("recording SQL must be nonempty and contain no NUL");
    const std::string owned(sql);
    Extracted statements;
    const auto n = duckdb_extract_statements(connection, owned.c_str(), &statements.value);
    if (n != 1) {
        const char* error = statements.value ? duckdb_extract_statements_error(statements.value) : nullptr;
        if (error && *error) failed("parse", error);
        throw std::invalid_argument("sql accepts exactly one read-only SELECT statement");
    }
    if (duckdb_prepare_extracted_statement(connection, statements.value, 0, &out.value) == DuckDBError)
        failed("prepare", out.value ? duckdb_prepare_error(out.value) : nullptr);
    if (select_only && duckdb_prepared_statement_type(out.value) != DUCKDB_STATEMENT_TYPE_SELECT)
        throw std::invalid_argument("sql accepts exactly one read-only SELECT statement");
}

void bind(duckdb_prepared_statement statement, std::span<const Value> parameters) {
    if (duckdb_nparams(statement) != parameters.size()) throw std::invalid_argument("recording SQL parameter count differs");
    for (std::size_t i = 0; i < parameters.size(); ++i) {
        const auto index = static_cast<idx_t>(i) + 1;
        const auto& value = parameters[i];
        duckdb_state state = DuckDBError;
        if (io::is_null(value)) state = duckdb_bind_null(statement, index);
        else if (const auto* boolean = std::get_if<bool>(&value.data)) state = duckdb_bind_boolean(statement, index, *boolean);
        else if (const auto* name = std::get_if<std::string>(&value.data))
            state = duckdb_bind_varchar_length(statement, index, name->data(), static_cast<idx_t>(name->size()));
        else if (const auto* number = std::get_if<io::Number>(&value.data)) {
            if (number->lexeme.find_first_of(".eE") == std::string::npos) {
                if (!number->lexeme.empty() && number->lexeme.front() == '-')
                    state = duckdb_bind_int64(statement, index, io::as_int64(value, "SQL parameter"));
                else {
                    std::uint64_t number_value = 0;
                    const auto end = std::to_address(number->lexeme.end());
                    const auto converted = std::from_chars(number->lexeme.data(), end, number_value);
                    if (converted.ec != std::errc{} || converted.ptr != end)
                        throw std::invalid_argument("SQL parameter integer exceeds UINT64_MAX");
                    state = duckdb_bind_uint64(statement, index, number_value);
                }
            } else state = duckdb_bind_double(statement, index, io::as_double(value, "SQL parameter"));
        } else if (const auto* tagged = std::get_if<io::Tagged>(&value.data); tagged && tagged->kind == "float64") {
            state = duckdb_bind_double(statement, index, io::tagged_double(value,"SQL parameter"));
        } else if (const auto* array = std::get_if<Array>(&value.data)) {
            struct Values {
                std::vector<duckdb_value> children;
                duckdb_value list = nullptr;
                Values() = default;
                ~Values() {
                    if (list) duckdb_destroy_value(&list);
                    for (auto& child : children) if (child) duckdb_destroy_value(&child);
                }
                Values(const Values&) = delete;
                Values& operator=(const Values&) = delete;
                Values(Values&&) = delete;
                Values& operator=(Values&&) = delete;
            } values;
            values.children.reserve(array->size());
            for (const auto& child : *array) {
                auto native = io::is_null(child) ? duckdb_create_null_value() : duckdb_create_double(io::tagged_double(child, "SQL double[] parameter"));
                if (!native) failed("create list child", nullptr);
                values.children.push_back(native);
            }
            Logical const type(duckdb_create_logical_type(DUCKDB_TYPE_DOUBLE));
            values.list = duckdb_create_list_value(type.value, values.children.data(), static_cast<idx_t>(values.children.size()));
            if (!values.list) failed("create double list", nullptr);
            state = duckdb_bind_value(statement, index, values.list);
        } else if (const auto* bytes = std::get_if<io::Bytes>(&value.data))
            state = duckdb_bind_blob(statement, index, bytes->value.data(), static_cast<idx_t>(bytes->value.size()));
        else throw std::invalid_argument("recording SQL parameters require scalar values");
        if (state == DuckDBError) failed("bind", duckdb_prepare_error(statement));
    }
}

template<class T> const T& item(duckdb_vector vector, idx_t row) {
    const auto* data = static_cast<const T*>(duckdb_vector_get_data(vector));
    if (!data) failed("read vector", "missing data");
    // DuckDB owns the vector and the validated parent chunk/list supplies
    // the row index. Keep pointer arithmetic inside a checked bounded view.
    if(row>=static_cast<idx_t>(std::numeric_limits<std::ptrdiff_t>::max()))
        throw std::invalid_argument("DuckDB vector index exceeds addressable extent");
    const auto values=std::views::counted(data,static_cast<std::ptrdiff_t>(row+1));
    return values[static_cast<std::size_t>(row)];
}
std::string decimal128(std::uint64_t hi, std::uint64_t lo, bool negative) {
    if (negative) { lo = ~lo + 1; hi = ~hi + (lo == 0 ? 1U : 0U); }
    std::array<std::uint32_t, 4> words{static_cast<std::uint32_t>(hi >> 32U), static_cast<std::uint32_t>(hi),
        static_cast<std::uint32_t>(lo >> 32U), static_cast<std::uint32_t>(lo)};
    std::string text_value;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-do-while) mirrors the reference control flow
    do {
        std::uint64_t remainder = 0;
        for (auto& word : words) {
            const auto dividend = (remainder << 32U) | word;
            word = static_cast<std::uint32_t>(dividend / 10U); remainder = dividend % 10U;
        }
        text_value += static_cast<char>('0' + remainder);
    } while (std::ranges::any_of(words, [](auto word) { return word != 0; }));
    if (negative) text_value += '-';
    std::ranges::reverse(text_value);
    return text_value;
}
std::string huge(duckdb_hugeint value) {
    return decimal128(static_cast<std::uint64_t>(value.upper), value.lower, value.upper < 0);
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
std::string pad(std::int64_t number, std::size_t width) {
    auto out = std::to_string(number);
    if (out.size() < width) out.insert(0, width - out.size(), '0');
    return out;
}
std::string date_text(std::int32_t days) {
    const auto day = std::chrono::sys_days{std::chrono::days{days}};
    if(day<std::chrono::sys_days{std::chrono::year{1}/1/1} || day>std::chrono::sys_days{std::chrono::year{9999}/12/31})
        throw std::invalid_argument("recording date is outside the tagged JSON date domain");
    const auto date = std::chrono::year_month_day{day};
    const int year = static_cast<int>(date.year());
    if (!date.ok() || year < 1 || year > 9999)
        throw std::invalid_argument("recording date is outside the tagged JSON date domain");
    return pad(year, 4) + '-' + pad(static_cast<unsigned>(date.month()), 2) + '-' + pad(static_cast<unsigned>(date.day()), 2);
}
std::string time_text(std::int64_t micros) {
    if (micros < 0 || micros >= 86400000000LL) throw std::invalid_argument("recording time is outside one day");
    const auto whole = micros / 1000000;
    std::string out = pad(whole / 3600, 2) + ':' + pad((whole / 60) % 60, 2) + ':' + pad(whole % 60, 2);
    if (micros % 1000000) out += '.' + pad(micros % 1000000, 6);
    return out;
}
Value timestamp(std::int64_t micros, bool timezone) {
    auto days = micros / 86400000000LL, remainder = micros % 86400000000LL;
    if (remainder < 0) { --days; remainder += 86400000000LL; }
    if (days < std::numeric_limits<std::int32_t>::min() || days > std::numeric_limits<std::int32_t>::max())
        throw std::invalid_argument("recording timestamp is out of range");
    return Value{io::Tagged{.kind="datetime", .fields={{"value", date_text(static_cast<std::int32_t>(days)) + 'T' +
        time_text(remainder) + (timezone ? "+00:00" : "")}}}};
}

// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
Value convert(duckdb_vector vector, duckdb_logical_type type, idx_t row, std::size_t depth = 0) {
    if (depth > 128) throw std::invalid_argument("recording query value exceeds nesting limit");
    auto* validity = duckdb_vector_get_validity(vector);
    if (validity && !duckdb_validity_row_is_valid(validity, row)) return Value{};
    // The SDK adds logical types independently of this codec. Unknown integer
    // tags have an explicit error path rather than suppressing enum warnings.
    const int kind = static_cast<int>(duckdb_get_type_id(type));
    switch (kind) {
    case DUCKDB_TYPE_SQLNULL: return Value{};
    case DUCKDB_TYPE_BOOLEAN: return Value{item<bool>(vector, row)};
    case DUCKDB_TYPE_TINYINT: return integer(item<std::int8_t>(vector, row));
    case DUCKDB_TYPE_SMALLINT: return integer(item<std::int16_t>(vector, row));
    case DUCKDB_TYPE_INTEGER: return integer(item<std::int32_t>(vector, row));
    case DUCKDB_TYPE_BIGINT: return integer(item<std::int64_t>(vector, row));
    case DUCKDB_TYPE_UTINYINT: return integer(item<std::uint8_t>(vector, row));
    case DUCKDB_TYPE_USMALLINT: return integer(item<std::uint16_t>(vector, row));
    case DUCKDB_TYPE_UINTEGER: return integer(item<std::uint32_t>(vector, row));
    case DUCKDB_TYPE_UBIGINT: return Value{io::Number{std::to_string(item<std::uint64_t>(vector, row))}};
    case DUCKDB_TYPE_HUGEINT: return Value{io::Number{huge(item<duckdb_hugeint>(vector, row))}};
    case DUCKDB_TYPE_UHUGEINT: {
        const auto value = item<duckdb_uhugeint>(vector, row);
        return Value{io::Number{decimal128(value.upper, value.lower, false)}};
    }
    case DUCKDB_TYPE_FLOAT: return real(static_cast<double>(item<float>(vector, row)));
    case DUCKDB_TYPE_DOUBLE: return real(item<double>(vector, row));
    case DUCKDB_TYPE_VARCHAR:
    case DUCKDB_TYPE_BLOB: {
        auto value = item<duckdb_string_t>(vector, row);
        const auto size = duckdb_string_t_length(value);
        const auto* data = duckdb_string_t_data(&value);
        if (size > 128U * 1024U * 1024U) throw std::invalid_argument("recording query cell exceeds 128 MiB");
        if (kind == DUCKDB_TYPE_VARCHAR) return Value{std::string(data, size)};
        const auto bytes = std::as_bytes(std::views::counted(data, static_cast<std::ptrdiff_t>(size)));
        return Value{io::Bytes{std::vector<std::byte>(bytes.begin(), bytes.end())}};
    }
    case DUCKDB_TYPE_DECIMAL: {
        std::string value;
        switch (static_cast<int>(duckdb_decimal_internal_type(type))) {
        case DUCKDB_TYPE_SMALLINT: value = std::to_string(item<std::int16_t>(vector, row)); break;
        case DUCKDB_TYPE_INTEGER: value = std::to_string(item<std::int32_t>(vector, row)); break;
        case DUCKDB_TYPE_BIGINT: value = std::to_string(item<std::int64_t>(vector, row)); break;
        case DUCKDB_TYPE_HUGEINT: value = huge(item<duckdb_hugeint>(vector, row)); break;
        default: throw std::invalid_argument("unsupported DuckDB decimal storage");
        }
        const auto scale = duckdb_decimal_scale(type);
        if (scale) {
            const std::size_t sign = value.front() == '-' ? 1U : 0U;
            if (value.size() - sign <= scale) value.insert(sign, static_cast<std::size_t>(scale) + 1 - (value.size() - sign), '0');
            value.insert(value.size() - scale, 1, '.');
        }
        return Value{io::Tagged{.kind="decimal", .fields={{"value", std::move(value)}}}};
    }
    case DUCKDB_TYPE_DATE: return Value{io::Tagged{.kind="date", .fields={{"value", date_text(item<duckdb_date>(vector, row).days)}}}};
    case DUCKDB_TYPE_TIME: return Value{io::Tagged{.kind="time", .fields={{"value", time_text(item<duckdb_time>(vector, row).micros)}}}};
    case DUCKDB_TYPE_TIME_TZ: {
        const auto value=duckdb_from_time_tz(item<duckdb_time_tz>(vector,row));
        const auto micros=((static_cast<std::int64_t>(value.time.hour)*60+value.time.min)*60+value.time.sec)*1000000+value.time.micros;
        const auto offset=static_cast<std::int64_t>(value.offset);
        if(offset<=-86400 || offset>=86400)throw std::invalid_argument("recording time offset is outside one day");
        const auto magnitude=offset<0?-offset:offset;
        auto suffix=std::string(offset<0?"-":"+")+pad(magnitude/3600,2)+':'+pad((magnitude/60)%60,2);
        if(magnitude%60)suffix+=':'+pad(magnitude%60,2);
        return Value{io::Tagged{.kind="time",.fields={{"value",time_text(micros)+suffix}}}};
    }
    case DUCKDB_TYPE_TIMESTAMP: return timestamp(item<duckdb_timestamp>(vector, row).micros, false);
    case DUCKDB_TYPE_TIMESTAMP_TZ: return timestamp(item<duckdb_timestamp>(vector, row).micros, true);
    case DUCKDB_TYPE_TIMESTAMP_S:
    case DUCKDB_TYPE_TIMESTAMP_MS: {
        const auto count_value = item<std::int64_t>(vector, row);
        const std::int64_t multiplier = kind == DUCKDB_TYPE_TIMESTAMP_S ? 1000000 : 1000;
        if (count_value < std::numeric_limits<std::int64_t>::min() / multiplier ||
            count_value > std::numeric_limits<std::int64_t>::max() / multiplier)
            throw std::invalid_argument("recording timestamp overflows microseconds");
        return timestamp(count_value * multiplier, false);
    }
    case DUCKDB_TYPE_TIMESTAMP_NS: return timestamp(item<std::int64_t>(vector, row) / 1000, false);
    case DUCKDB_TYPE_INTERVAL: {
        const auto value = item<duckdb_interval>(vector, row);
        auto days = static_cast<std::int64_t>(value.months) * 30 + value.days + value.micros / 86400000000LL;
        auto remainder = value.micros % 86400000000LL;
        if (remainder < 0) { --days; remainder += 86400000000LL; }
        return Value{io::Tagged{.kind="timedelta", .fields={{"days", std::to_string(days)},
            {"seconds", std::to_string(remainder / 1000000)}, {"microseconds", std::to_string(remainder % 1000000)}}}};
    }
    case DUCKDB_TYPE_UUID: {
        const auto value = item<duckdb_hugeint>(vector, row);
        const std::uint64_t high = static_cast<std::uint64_t>(value.upper) ^ (std::uint64_t{1} << 63U);
        constexpr std::string_view hex = "0123456789abcdef";
        std::string out;
        for (unsigned i = 0; i < 32; ++i) {
            if (i == 8 || i == 12 || i == 16 || i == 20) out += '-';
            const auto word = i < 16 ? high : value.lower;
            out += hex[(word >> ((15U - i % 16U) * 4U)) & 15U];
        }
        return text(out);
    }
    case DUCKDB_TYPE_ENUM: {
        std::uint64_t index = 0;
        switch (static_cast<int>(duckdb_enum_internal_type(type))) {
        case DUCKDB_TYPE_UTINYINT: index = item<std::uint8_t>(vector, row); break;
        case DUCKDB_TYPE_USMALLINT: index = item<std::uint16_t>(vector, row); break;
        case DUCKDB_TYPE_UINTEGER: index = item<std::uint32_t>(vector, row); break;
        default: throw std::invalid_argument("unsupported enum storage");
        }
        EngineString const name{duckdb_enum_dictionary_value(type, index)};
        if (!name) failed("enum", "missing dictionary entry");
        return text(name.get());
    }
    case DUCKDB_TYPE_STRUCT: {
        const auto n = duckdb_struct_type_child_count(type);
        if (n > 4096) throw std::invalid_argument("recording struct exceeds field limit");
        Object out; out.reserve(static_cast<std::size_t>(n));
        for (idx_t i = 0; i < n; ++i) {
            EngineString const key{duckdb_struct_type_child_name(type, i)};
            if (!key) failed("struct", "missing child name");
            std::string name(key.get());
            Logical const child(duckdb_struct_type_child_type(type, i));
            out.emplace_back(std::move(name), convert(duckdb_struct_vector_get_child(vector, i), child.value, row, depth + 1));
        }
        return Value{std::move(out)};
    }
    case DUCKDB_TYPE_LIST:
    case DUCKDB_TYPE_ARRAY: {
        const bool fixed = kind == DUCKDB_TYPE_ARRAY;
        const auto width = fixed ? duckdb_array_type_array_size(type) : 0;
        const auto entry = fixed ? duckdb_list_entry{.offset=row * width, .length=width} : item<duckdb_list_entry>(vector, row);
        if (entry.length > 1000000) throw std::invalid_argument("recording list exceeds one million elements");
        Logical const child(fixed ? duckdb_array_type_child_type(type) : duckdb_list_type_child_type(type));
        const auto child_vector = fixed ? duckdb_array_vector_get_child(vector) : duckdb_list_vector_get_child(vector);
        Array out; out.reserve(static_cast<std::size_t>(entry.length));
        for (idx_t i = 0; i < entry.length; ++i) out.push_back(convert(child_vector, child.value, entry.offset + i, depth + 1));
        return Value{std::move(out)};
    }
    case DUCKDB_TYPE_MAP: {
        const auto entry = item<duckdb_list_entry>(vector, row);
        if (entry.length > 1000000) throw std::invalid_argument("recording map exceeds one million entries");
        const auto pair = duckdb_list_vector_get_child(vector);
        const auto keys = duckdb_struct_vector_get_child(pair, 0), values_vector = duckdb_struct_vector_get_child(pair, 1);
        const Logical key_type(duckdb_map_type_key_type(type)), value_type(duckdb_map_type_value_type(type));
        Object out; std::set<std::string> names;
        for (idx_t i = 0; i < entry.length; ++i) {
            auto key = convert(keys, key_type.value, entry.offset + i, depth + 1);
            const auto* name = std::get_if<std::string>(&key.data);
            if (!name) throw std::invalid_argument("tagged recording JSON requires string map keys");
            if (!names.insert(*name).second) throw std::invalid_argument("recording map contains duplicate keys");
            out.emplace_back(*name, convert(values_vector, value_type.value, entry.offset + i, depth + 1));
        }
        return Value{std::move(out)};
    }
    default: throw std::invalid_argument("unsupported lossless DuckDB query type " + std::to_string(kind));
    }
}
} // namespace

struct Cursor::Impl {
    duckdb_result result{};
    duckdb_data_chunk chunk = nullptr;
    Impl() = default;
    std::vector<std::string> columns;
    std::vector<duckdb_logical_type> types;
    idx_t row = 0;
    bool finished = false;
    ~Impl() {
        if (chunk) duckdb_destroy_data_chunk(&chunk);
        for (auto& type : types) duckdb_destroy_logical_type(&type);
        duckdb_destroy_result(&result);
    }
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;
};
Cursor::Cursor(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Cursor::~Cursor() = default;
Cursor::Cursor(Cursor&&) noexcept = default;
Cursor& Cursor::operator=(Cursor&&) noexcept = default;
const std::vector<std::string>& Cursor::columns() const { return impl_->columns; }
std::vector<io::Value::Object> Cursor::next_batch(std::size_t maximum) {
    if (maximum == 0 || maximum > 4096) throw std::invalid_argument("recording fetch batch must be in 1..4096");
    std::vector<Object> out; out.reserve(maximum);
    while (out.size() < maximum && !impl_->finished) {
        if (!impl_->chunk || impl_->row == duckdb_data_chunk_get_size(impl_->chunk)) {
            if (impl_->chunk) duckdb_destroy_data_chunk(&impl_->chunk);
            impl_->chunk = duckdb_fetch_chunk(impl_->result); impl_->row = 0;
            if (!impl_->chunk) {
                if (const auto* error = duckdb_result_error(&impl_->result); error && *error) failed("fetch", error);
                impl_->finished = true; break;
            }
            if (duckdb_data_chunk_get_size(impl_->chunk) == 0) continue;
        }
        Object row; row.reserve(impl_->columns.size());
        for (std::size_t i = 0; i < impl_->columns.size(); ++i)
            row.emplace_back(impl_->columns[i], convert(duckdb_data_chunk_get_vector(impl_->chunk, i), impl_->types[i], impl_->row));
        out.push_back(std::move(row)); ++impl_->row;
    }
    return out;
}

Connection::Connection(duckdb_database database) {
    if (duckdb_connect(database, &connection_) == DuckDBError) {
        if (connection_) duckdb_disconnect(&connection_);
        failed("connect", nullptr);
    }
}
Connection::~Connection() { if (connection_) duckdb_disconnect(&connection_); }
void Connection::require_select(std::string_view sql) const { Prepared statement; prepare(connection_, sql, true, statement); }
Cursor Connection::stream(std::string_view sql, std::span<const io::Value> parameters, bool select_only) const {
    Prepared statement; prepare(connection_, sql, select_only, statement); bind(statement.value, parameters);
    auto result = std::make_unique<Cursor::Impl>();
    if (duckdb_execute_prepared_streaming(statement.value, &result->result) == DuckDBError)
        failed("query", duckdb_result_error(&result->result));
    const auto columns = duckdb_column_count(&result->result);
    if (columns > 4096) throw std::invalid_argument("recording query exceeds 4096 columns");
    std::set<std::string> names;
    result->types.reserve(static_cast<std::size_t>(columns));
    for (idx_t i = 0; i < columns; ++i) {
        const char* name = duckdb_column_name(&result->result, i);
        if (!name || !names.insert(name).second)
            throw std::invalid_argument("query output column names must be unique; use aliases");
        result->columns.emplace_back(name);
        auto type = duckdb_column_logical_type(&result->result, i);
        if (!type) failed("column type", nullptr);
        result->types.push_back(type);
    }
    return Cursor{std::move(result)};
}
void Connection::execute(std::string_view sql, std::span<const io::Value> parameters) const {
    Prepared statement; prepare(connection_, sql, false, statement); bind(statement.value, parameters);
    duckdb_result result{};
    const auto state = duckdb_execute_prepared(statement.value, &result);
    const auto* error = duckdb_result_error(&result);
    const std::string message = error ? error : "execution failed";
    duckdb_destroy_result(&result);
    if (state == DuckDBError) failed("execute", message.c_str());
}
std::vector<io::Value::Object> Connection::query(std::string_view sql, std::span<const io::Value> parameters) const {
    auto cursor = stream(sql, parameters);
    std::vector<Object> rows;
    for (;;) {
        auto batch = cursor.next_batch();
        if (batch.empty()) break;
        rows.insert(rows.end(), std::make_move_iterator(batch.begin()), std::make_move_iterator(batch.end()));
    }
    return rows;
}
Database::Database(const std::filesystem::path& path, bool read_only) {
    const auto name = path.string();
    if (name.empty() || name.find('\0') != std::string::npos) throw std::invalid_argument("database path must be nonempty");
    const std::string& engine_path = name;
    if (read_only) {
        opened_ = platform::open_regular(path);
        // DuckDB resolves descriptor pseudo-paths (/proc/self/fd, /dev/fd)
        // back to names and refuses them, so the engine must open the path
        // itself. The held descriptor still pins the validated inode for
        // opened_sha256(); the post-open inode comparison below fails if the
        // directory entry was replaced before the engine opened it.
        // Sealed recordings have no WAL sidecar: their writer checkpointed.
    }
    Config config;
    if (read_only) config.set("access_mode", "READ_ONLY");
    config.set("enable_external_access", "false");
    config.set("autoload_known_extensions", "false");
    config.set("autoinstall_known_extensions", "false");
    config.set("threads", "1");
    config.set("memory_limit", "512MB");
    // Read-only analyst queries may not turn a large sort into filesystem IO.
    if (read_only) config.set("max_temp_directory_size", "0B");
    char* error = nullptr;
    const auto state = duckdb_open_ext(engine_path.c_str(), &database_, config.value, &error);
    const EngineString owned_error{error};
    std::string message;
    try {message = owned_error ? owned_error.get() : "cannot open database";}
    catch (...) {if(database_)duckdb_close(&database_);throw;}
    if (state == DuckDBError) { if (database_) duckdb_close(&database_); failed("open", message.c_str()); }
    if (read_only) {
        // The engine opened by name: the directory entry must still name
        // the inode open_regular validated, or the hash manifest could be
        // satisfied by a different file than the one being queried.
        const auto current = platform::open_regular(path);
        struct stat held{}, latest{};
        if (::fstat(opened_.get(), &held) != 0 || ::fstat(current.get(), &latest) != 0)
            throw std::system_error(errno, std::generic_category(), "fstat recording inode");
        if (held.st_dev != latest.st_dev || held.st_ino != latest.st_ino)
            failed("open", "recording database inode changed during open");
    }
    try { connection_ = std::unique_ptr<Connection>(new Connection(database_)); }
    catch (...) { duckdb_close(&database_); throw; }
}
Database::~Database() { connection_.reset(); if (database_) duckdb_close(&database_); }
std::string Database::opened_sha256() const {
    if (!opened_.valid()) throw std::logic_error("only a read-only database owns an input inode");
    struct stat before{};
    if (::fstat(opened_.get(), &before) < 0) throw std::system_error(errno, std::generic_category(), "stat recording inode");
    if (!S_ISREG(before.st_mode) || before.st_size < 0) throw std::invalid_argument("recording inode must be a regular file");
    io::Sha256 digest; std::vector<std::byte> buffer(65536); off_t offset = 0;
    while (offset < before.st_size) {
        const auto remaining = before.st_size - offset;
        const auto wanted = std::cmp_less(remaining ,buffer.size()) ? static_cast<std::size_t>(remaining) : buffer.size();
        const auto read = ::pread(opened_.get(), buffer.data(), wanted, offset);
        if (read < 0) { if (errno == EINTR) continue; throw std::system_error(errno, std::generic_category(), "read recording inode"); }
        if (read == 0) throw std::invalid_argument("recording inode changed while hashing");
        digest.update(std::span{buffer}.first(static_cast<std::size_t>(read))); offset += read;
    }
    struct stat after{};
    if (::fstat(opened_.get(), &after) < 0) throw std::system_error(errno, std::generic_category(), "stat recording inode after hash");
    if (before.st_size != after.st_size || before.st_mtime != after.st_mtime || before.st_ctime != after.st_ctime)
        throw std::invalid_argument("recording inode changed while hashing");
    constexpr std::string_view hex = "0123456789abcdef"; std::string out; out.reserve(64);
    for (const auto byte : digest.finish()) { out += hex[byte >> 4U]; out += hex[byte & 15U]; }
    return out;
}
} // namespace bike::recording
