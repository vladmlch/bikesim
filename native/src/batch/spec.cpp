#include "plan.hpp"
#include <cstdint>
#include "integer_detail.hpp"
#include "../io/json.hpp"
#include "../terrain/trackfile.hpp"
#include <charconv>
#include <limits>
#include <memory>

namespace bike::batch {
namespace {
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class Container : std::uint8_t { document, array, table };
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class Position : std::uint8_t { key, value, after_value };
struct Frame { Container container; Position position; };
struct Replacement { std::size_t first, last; std::string decimal; };
bool horizontal(char c) { return c == ' ' || c == '\t'; }
bool delimiter(char c) { return horizontal(c) || c == '\n' || c == '\r' || c == ',' || c == ']' || c == '}' || c == '#'; }
std::size_t quoted_end(std::string_view text, std::size_t start) {
    const char quote = text[start];
    const bool triple = text.size() - start >= 3 && text[start+1] == quote && text[start+2] == quote;
    std::size_t i = start + (triple ? 3U : 1U);
    while (i < text.size()) {
        if (quote == '"' && text[i] == '\\') { i += std::min<std::size_t>(2, text.size()-i); continue; }
        if (text[i] == quote) {
            if (!triple) return i+1;
            std::size_t end = i; while (end < text.size() && text[end] == quote) ++end;
            if (end-i >= 3) return end;
            i = end;
        } else ++i;
    }
    return text.size(); // The unchanged parser rejects the unterminated token.
}
std::size_t header_end(std::string_view text, std::size_t start) {
    for (std::size_t i = start; i < text.size();) {
        if (text[i] == '\n' || text[i] == '\r') return i;
        if (text[i] == '#') { const auto end = text.find('\n',i); return end == std::string_view::npos ? text.size() : end; }
        if (text[i] == '"' || text[i] == '\'') i = quoted_end(text,i);
        else ++i;
    }
    return text.size();
}
std::vector<Replacement> integer_values(std::string_view text) {
    std::vector<Replacement> values;
    std::vector<Frame> stack{{.container=Container::document,.position=Position::key}};
    for (std::size_t i = 0; i < text.size();) {
        const char c = text[i];
        if (c == '#') { const auto end = text.find('\n',i); i = end == std::string_view::npos ? text.size() : end; continue; }
        if (c == '\n' || c == '\r') {
            if (stack.size() == 1) stack.back().position = Position::key;
            ++i; continue;
        }
        if (horizontal(c)) { ++i; continue; }
        if (c == '"' || c == '\'') {
            if (stack.back().position == Position::value) stack.back().position = Position::after_value;
            i = quoted_end(text,i); continue;
        }
        if (stack.size() == 1 && stack.back().position == Position::key && c == '[') {
            i = header_end(text,i); continue; // numeric/dotted/quoted table keys remain keys
        }
        if (stack.back().position == Position::key) {
            if (c == '=') stack.back().position = Position::value;
            else if (c == '}' && stack.back().container == Container::table) {
                stack.pop_back(); stack.back().position = Position::after_value;
            }
            ++i; continue;
        }
        if (c == ']' && stack.back().container == Container::array) {
            stack.pop_back(); stack.back().position = Position::after_value; ++i; continue;
        }
        if (c == '}' && stack.back().container == Container::table) {
            stack.pop_back(); stack.back().position = Position::after_value; ++i; continue;
        }
        if (c == ',') {
            if (stack.back().container == Container::array) stack.back().position = Position::value;
            else if (stack.back().container == Container::table) stack.back().position = Position::key;
            ++i; continue;
        }
        if (stack.back().position == Position::value) {
            if (c == '[') { stack.push_back({.container=Container::array,.position=Position::value}); ++i; continue; }
            if (c == '{') { stack.push_back({.container=Container::table,.position=Position::key}); ++i; continue; }
            std::size_t end = i;
            while (end < text.size() && !delimiter(text[end])) ++end;
            if (const auto decimal = integer_detail::toml_token(text.substr(i,end-i))) values.push_back({.first=i,.last=end,.decimal=*decimal});
            stack.back().position = Position::after_value;
            // A mismatched closer at value position is not an atom. Leave
            // its syntax for the parser, while ensuring scanner progress.
            i = end == i ? i+1 : end; continue;
        }
        ++i;
    }
    return values;
}
std::optional<std::size_t> marker_index(std::string_view text, std::string_view prefix, std::size_t count) {
    if (!text.starts_with(prefix)) return {};
    text.remove_prefix(prefix.size());
    if (text.empty()) return {};
    std::size_t index = 0; const auto end = std::to_address(text.cend());
    const auto parsed = std::from_chars(text.data(),end,index); // NOLINT(bugprone-suspicious-stringview-data-usage) std::from_chars parses the (data, data+size) pointer range, not a C string
    if (parsed.ec != std::errc{} || parsed.ptr != end || index >= count || std::to_string(index) != text) return {};
    return index;
}
bool restore(io::Value& tree, std::string_view prefix, const std::vector<Replacement>& replacements) {
    std::vector<io::Value*> values{&tree}, marked;
    std::vector<std::size_t> counts(replacements.size());
    bool collision = false;
    while (!values.empty()) {
        auto* value = values.back(); values.pop_back();
        if (auto* object = std::get_if<io::Value::Object>(&value->data)) {
            for (auto& [key,child] : *object) { (void)key; values.push_back(&child); }
        } else if (auto* array = std::get_if<io::Value::Array>(&value->data)) {
            for (auto& child : *array) values.push_back(&child);
        } else if (const auto* text = std::get_if<std::string>(&value->data); text && text->starts_with(prefix)) {
            if (const auto index = marker_index(*text,prefix,replacements.size())) {
                if (++counts[*index] != 1) collision = true;
                marked.push_back(value);
            } else collision = true;
        }
    }
    if (std::ranges::any_of(counts,[](auto count) { return count == 0; }))
        throw std::invalid_argument("batch integer token did not survive the TOML value tree");
    // A quoted Unicode escape can decode to an otherwise absent marker.
    // Retry with a fresh prefix before changing any original value.
    if (collision) return false;
    for (auto* value : marked) {
        const auto index = marker_index(std::get<std::string>(value->data),prefix,replacements.size());
        value->data = io::Number{replacements[index.value()].decimal}; // NOLINT(bugprone-unchecked-optional-access) collision being false means marker_index engaged above
    }
    return true;
}
} // namespace
io::Value parse_spec_toml(std::string_view source, std::string_view origin) {
    if (source.size() > io::kMaxJsonBytes) throw std::invalid_argument("batch TOML exceeds the 512 MiB input limit");
    const auto replacements = integer_values(source);
    if (replacements.empty()) return terrain::parse_track_toml(source,origin);
    for (std::size_t attempt = 0;; ++attempt) {
        const auto prefix = "__bike_batch_integer_" + std::to_string(attempt) + "_";
        if (source.find(prefix) == std::string_view::npos) {
            std::string rewritten; rewritten.reserve(source.size()); std::size_t copied = 0;
            const auto append = [&](std::string_view part) {
                if (part.size() > io::kMaxJsonBytes - rewritten.size())
                    throw std::invalid_argument("batch TOML marker expansion exceeds the 512 MiB input limit");
                rewritten.append(part);
            };
            for (std::size_t i = 0; i < replacements.size(); ++i) {
                const auto& replacement = replacements[i];
                append(source.substr(copied,replacement.first-copied));
                append('"' + prefix + std::to_string(i) + '"'); copied = replacement.last;
            }
            append(source.substr(copied));
            auto tree = terrain::parse_track_toml(rewritten,origin);
            if (restore(tree,prefix,replacements)) return tree;
        }
        if (attempt == std::numeric_limits<std::size_t>::max()) throw std::overflow_error("batch TOML marker space exhausted");
    }
}
} // namespace bike::batch
