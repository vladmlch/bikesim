#pragma once

// Cold validation boundary. These helpers never enter the realtime loop.
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../platform/files.hpp"
#include "../numeric_sum.hpp"
#include "../recording/value_util.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace bike::validation {
using Value = io::Value;
using Object = Value::Object;
using Array = Value::Array;
namespace value = recording::value;

inline const Value* find(const Value& object, std::string_view key) {
    const auto* members = std::get_if<Object>(&object.data);
    return members ? io::find_member(*members, key) : nullptr;
}
inline const Value& get(const Value& object, std::string_view key) {
    if (const auto* item = find(object, key)) return *item;
    throw std::invalid_argument("validation: missing " + std::string(key));
}
inline Value optional(const Value& object, std::string_view key, const Value& fallback = {}) {
    const auto* item = find(object, key);
    return item ? *item : fallback;
}
inline bool true_value(const Value& item) {
    const auto* flag = std::get_if<bool>(&item.data);
    return flag && *flag;
}
inline bool false_value(const Value& item) {
    const auto* flag = std::get_if<bool>(&item.data);
    return flag && !*flag;
}
inline bool flag(const Value& object, std::string_view key) {
    const auto* item = find(object, key);
    return item && true_value(*item);
}
inline bool checks_passed(const Value& checks) {
    const auto& entries=io::as_object(checks,"checks");
    return std::ranges::all_of(entries,[](const auto& item){return true_value(item.second);});
}
// Python float(value) coercion at the JSON boundary — the oracle's
// check_bounds calls float(metrics.get(name)) and np.asarray(dtype=float)
// performs the same leaf coercion: numbers, bools and float-parseable
// strings (whitespace-padded literals, inf/infinity, nan, underscored
// digits). Nullopt where float()/np.asarray would raise.
[[nodiscard]] inline std::optional<double> python_float(const Value& v) {
    if(const auto* n=std::get_if<io::Number>(&v.data)){(void)n;return io::as_double(v,"float coercion");}
    if(const auto* b=std::get_if<bool>(&v.data))return *b?1.:0.;
    const auto* s=std::get_if<std::string>(&v.data);if(!s)return {};
    std::string_view text=*s;
    const auto blank=[](char c){return c==' '||c=='\t'||c=='\n'||c=='\r'||c=='\v'||c=='\f';};
    while(!text.empty()&&blank(text.front()))text.remove_prefix(1);
    while(!text.empty()&&blank(text.back()))text.remove_suffix(1);
    if(text.empty())return {};
    bool negative=false;
    if(text.front()=='+'||text.front()=='-'){negative=text.front()=='-';text.remove_prefix(1);if(text.empty())return {};}
    std::string lowered(text);for(char& c:lowered)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if(lowered=="inf"||lowered=="infinity")
        return negative?-std::numeric_limits<double>::infinity():std::numeric_limits<double>::infinity();
    if(lowered=="nan")return std::numeric_limits<double>::quiet_NaN();
    // Python literal grammar: digits[.digits][e[sign]digits], a single
    // underscore allowed between digits. The cleaned literal is a valid
    // C double for strtod.
    const auto digit=[](char c){return c>='0'&&c<='9';};
    std::string clean;clean.reserve(text.size());std::size_t i=0;
    const auto digits=[&]{bool any=false;while(i<text.size()){
        if(digit(text[i])){clean+=text[i++];any=true;}
        else if(text[i]=='_'&&any&&i+1<text.size()&&digit(text[i+1]))++i;
        else break;}return any;};
    const bool before=digits();bool after=false;
    if(i<text.size()&&text[i]=='.'){clean+='.';++i;after=digits();}
    if(!before&&!after)return {};
    if(i<text.size()&&(text[i]=='e'||text[i]=='E')){clean+='e';++i;
        if(i<text.size()&&(text[i]=='+'||text[i]=='-'))clean+=text[i++];
        if(!digits())return {};}
    if(i!=text.size())return {};
    try{const double out=std::stod(clean);return negative?-out:out;}
    catch(const std::exception&){return {};}
}
// Python truthiness at the JSON boundary (the oracle's
// `record.get(...) or metrics.get(...)` intervention checks):
// null/false/numeric zero/empty containers are falsy.
[[nodiscard]] inline bool python_truthy(const Value* v) {
    if(!v||io::is_null(*v))return false;
    if(const auto* b=std::get_if<bool>(&v->data))return *b;
    if(const auto* n=std::get_if<io::Number>(&v->data)){(void)n;return io::as_double(*v,"truthy value")!=0.;}
    if(const auto* s=std::get_if<std::string>(&v->data))return !s->empty();
    if(const auto* a=std::get_if<Array>(&v->data))return !a->empty();
    if(const auto* o=std::get_if<Object>(&v->data))return !o->empty();
    return false;
}
inline double real(const Value& object, std::string_view key) {
    return io::as_double(get(object, key), key);
}
inline double real(const Value& object, std::string_view key, double fallback) {
    const auto* item = find(object, key);
    return item ? io::as_double(*item, key) : fallback;
}
inline std::string text(const Value& object, std::string_view key, std::string_view fallback = {}) {
    const auto* item = find(object, key);
    return item ? io::as_string(*item, key) : std::string(fallback);
}
inline double scalar(double x, std::string_view name) {
    if (!std::isfinite(x)) throw std::invalid_argument(std::string(name) + " must be finite");
    return x;
}
inline double positive(double x, std::string_view name) {
    scalar(x, name);
    if (x <= 0.) throw std::invalid_argument(std::string(name) + " must be positive");
    return x;
}
inline double nonnegative(double x, std::string_view name) {
    scalar(x, name);
    if (x < 0.) throw std::invalid_argument(std::string(name) + " must be nonnegative");
    return x;
}
inline std::string trim(const std::string& s) {
    constexpr std::string_view whitespace = " \t\r\n\v\f";
    const auto first = s.find_first_not_of(whitespace);
    if (first == std::string::npos) return {};
    return s.substr(first, s.find_last_not_of(whitespace) - first + 1);
}
inline std::string identifier(const Value& item, std::string_view name) {
    const auto result = io::as_string(item, name);
    if (trim(result).empty()) throw std::invalid_argument(std::string(name) + " needs a nonempty identifier");
    return result;
}
inline std::set<std::string> keys(const Value& item) {
    std::set<std::string> result;
    for (const auto& [key, unused] : io::as_object(item, "validation")) {
        (void)unused;
        result.insert(key);
    }
    return result;
}
inline std::vector<double> numbers(const Value& item, std::string_view name) {
    std::vector<double> result;
    for (const auto& entry : io::as_array(item, name)) result.push_back(io::as_double(entry, name));
    return result;
}
inline std::vector<std::string> strings(const Value& item, std::string_view name) {
    std::vector<std::string> result;
    for (const auto& entry : io::as_array(item, name)) result.push_back(io::as_string(entry, name));
    return result;
}
inline std::string read_text(const std::filesystem::path& file) {
    // Reuse the opened-object/no-follow and bounded-read contract. The byte
    // reader is format-neutral; JSON/TOML validation belongs to the caller.
    const auto bytes = io::read_numpy_bytes(file);
    if (bytes.empty()) return {};
    return {reinterpret_cast<const char*>(bytes.data()), bytes.size()}; // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte storage is reinterpreted as char for the string contract
}
inline std::string hash_text(std::string_view source) {
    return io::sha256(std::as_bytes(std::span(source.data(), source.size())));
}
inline std::string file_hash(const std::filesystem::path& file) { return platform::file_sha256(file); }
inline Value read_json(const std::filesystem::path& file) { return io::parse_json(read_text(file)); }
void write_report(const std::filesystem::path& file, const Value& report);
void write_report_text(const std::filesystem::path& file, std::string_view serialized);
[[nodiscard]] std::size_t steps(double duration, double dt, double tolerance = 1e-10);
inline double mean(std::span<const double> input) {
    if (input.empty()) throw std::invalid_argument("validation: mean of empty data");
    return numeric::numpy_sum(input) / static_cast<double>(input.size());
}
inline Value pair(double first, double second) { return Value{Array{value::real(first), value::real(second)}}; }
inline Value empty_object() { return Value{Object{}}; }
} // namespace bike::validation
