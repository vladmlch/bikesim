// native/src/physics_core/spindle_m2k2_upload.cpp — Standalone RFC-8259 JSON
// reference parser and explicit template instantiations for spindle_m2k2_upload.
#include "spindle_m2k2_upload.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::physics_core {

namespace {

constexpr std::size_t kMaxJsonDepth = 64;

enum class JsonKind : std::uint8_t {
    kNull,
    kBool,
    kNumber,
    kString,
    kArray,
    kObject,
};

struct JsonNode {
    JsonKind kind = JsonKind::kNull;
    double number = 0.0;
    std::string str;
    std::vector<std::size_t> elems;
    std::vector<std::pair<std::string, std::size_t>> members;
};

struct JsonDoc {
    std::vector<JsonNode> nodes;
    std::size_t root = 0;
};

void skip_ws(std::string_view t, std::size_t &p) noexcept {
    while (p < t.size() && (t[p] == ' ' || t[p] == '\t' || t[p] == '\n' || t[p] == '\r')) {
        ++p;
    }
}

bool is_digit(char c) noexcept {
    return c >= '0' && c <= '9';
}

bool parse_json_string(std::string_view t, std::size_t &p, std::string &out) {
    if (p >= t.size() || t[p] != '"') {
        return false;
    }
    ++p;
    out.clear();
    while (p < t.size()) {
        const auto c = static_cast<unsigned char>(t[p]);
        if (c == '"') {
            ++p;
            return true;
        }
        if (c < 0x20u) {
            return false;
        }
        if (c != '\\') {
            out.push_back(static_cast<char>(c));
            ++p;
            continue;
        }
        if (p + 1 >= t.size()) {
            return false;
        }
        const char e = t[p + 1];
        p += 2;
        char decoded = '\0';
        switch (e) {
            case '"': decoded = '"'; break;
            case '\\': decoded = '\\'; break;
            case '/': decoded = '/'; break;
            case 'b': decoded = '\b'; break;
            case 'f': decoded = '\f'; break;
            case 'n': decoded = '\n'; break;
            case 'r': decoded = '\r'; break;
            case 't': decoded = '\t'; break;
            case 'u': {
                if (p + 4 > t.size()) {
                    return false;
                }
                unsigned code = 0;
                for (std::size_t k = 0; k < 4; ++k) {
                    const char h = t[p + k];
                    unsigned v = 0;
                    if (h >= '0' && h <= '9') {
                        v = static_cast<unsigned>(h - '0');
                    } else if (h >= 'a' && h <= 'f') {
                        v = static_cast<unsigned>(h - 'a') + 10u;
                    } else if (h >= 'A' && h <= 'F') {
                        v = static_cast<unsigned>(h - 'A') + 10u;
                    } else {
                        return false;
                    }
                    code = code * 16u + v;
                }
                p += 4;
                decoded = (code < 0x80u) ? static_cast<char>(code) : '\0';
                break;
            }
            default:
                return false;
        }
        out.push_back(decoded);
    }
    return false;
}

std::optional<double> parse_json_number(std::string_view t, std::size_t &p) {
    const std::size_t start = p;
    if (p < t.size() && t[p] == '-') {
        ++p;
    }
    if (p >= t.size() || !is_digit(t[p])) {
        return std::nullopt;
    }
    if (t[p] == '0') {
        ++p;
    } else {
        while (p < t.size() && is_digit(t[p])) {
            ++p;
        }
    }
    if (p < t.size() && t[p] == '.') {
        ++p;
        if (p >= t.size() || !is_digit(t[p])) {
            return std::nullopt;
        }
        while (p < t.size() && is_digit(t[p])) {
            ++p;
        }
    }
    if (p < t.size() && (t[p] == 'e' || t[p] == 'E')) {
        ++p;
        if (p < t.size() && (t[p] == '+' || t[p] == '-')) {
            ++p;
        }
        if (p >= t.size() || !is_digit(t[p])) {
            return std::nullopt;
        }
        while (p < t.size() && is_digit(t[p])) {
            ++p;
        }
    }
    const std::string tok(t.substr(start, p - start));
    std::size_t parsed_len = 0;
    double val = 0.0;
    try {
        val = std::stod(tok, &parsed_len);
    } catch (...) {
        return std::nullopt;
    }
    if (parsed_len != tok.size() || !std::isfinite(val)) {
        return std::nullopt;
    }
    return val;
}

bool match_literal(std::string_view t, std::size_t &p, std::string_view lit) {
    if (!t.substr(p).starts_with(lit)) {
        return false;
    }
    p += lit.size();
    return true;
}

JsonDoc parse_json_doc(std::string_view text) {
    enum class Phase : std::uint8_t {
        kValue,
        kArrayFirst,
        kObjectFirst,
        kMember,
        kAfterValue,
    };
    struct Frame {
        std::size_t node_idx = 0;
        bool is_object = false;
        std::string pending_key;
        bool has_key = false;
    };

    JsonDoc doc;
    std::array<Frame, kMaxJsonDepth> stack{};
    std::size_t depth = 0;
    std::size_t p = 0;
    Phase phase = Phase::kValue;
    bool have_root = false;

    const auto attach_child = [&](std::size_t child_idx) {
        if (depth == 0) {
            doc.root = child_idx;
            have_root = true;
            return;
        }
        Frame &top = stack[depth - 1];
        if (top.is_object) {
            doc.nodes[top.node_idx].members.emplace_back(std::move(top.pending_key), child_idx);
            top.has_key = false;
        } else {
            doc.nodes[top.node_idx].elems.push_back(child_idx);
        }
    };

    for (;;) {
        switch (phase) {
            case Phase::kValue: {
                skip_ws(text, p);
                if (p >= text.size()) {
                    throw std::invalid_argument("unexpected end of JSON");
                }
                const char c = text[p];
                if (c == '{' || c == '[') {
                    if (depth >= kMaxJsonDepth) {
                        throw std::invalid_argument("JSON nesting exceeds maximum depth");
                    }
                    const bool is_obj = (c == '{');
                    const std::size_t idx = doc.nodes.size();
                    JsonNode container_node{};
                    container_node.kind = is_obj ? JsonKind::kObject : JsonKind::kArray;
                    doc.nodes.push_back(std::move(container_node));
                    attach_child(idx);
                    Frame frame{};
                    frame.node_idx = idx;
                    frame.is_object = is_obj;
                    stack[depth] = std::move(frame);
                    ++depth;
                    ++p;
                    phase = is_obj ? Phase::kObjectFirst : Phase::kArrayFirst;
                    continue;
                }
                const std::size_t idx = doc.nodes.size();
                JsonNode leaf{};
                if (c == '"') {
                    leaf.kind = JsonKind::kString;
                    if (!parse_json_string(text, p, leaf.str)) {
                        throw std::invalid_argument("invalid JSON string literal");
                    }
                } else if (c == 't') {
                    if (!match_literal(text, p, "true")) {
                        throw std::invalid_argument("invalid JSON token");
                    }
                    leaf.kind = JsonKind::kBool;
                    leaf.number = 1.0;
                } else if (c == 'f') {
                    if (!match_literal(text, p, "false")) {
                        throw std::invalid_argument("invalid JSON token");
                    }
                    leaf.kind = JsonKind::kBool;
                    leaf.number = 0.0;
                } else if (c == 'n') {
                    if (!match_literal(text, p, "null")) {
                        throw std::invalid_argument("invalid JSON token");
                    }
                    leaf.kind = JsonKind::kNull;
                } else {
                    leaf.kind = JsonKind::kNumber;
                    const std::optional<double> num = parse_json_number(text, p);
                    if (!num.has_value()) {
                        throw std::invalid_argument("invalid or non-finite JSON number");
                    }
                    leaf.number = *num;
                }
                doc.nodes.push_back(std::move(leaf));
                attach_child(idx);
                phase = Phase::kAfterValue;
                continue;
            }
            case Phase::kArrayFirst: {
                skip_ws(text, p);
                if (p >= text.size()) {
                    throw std::invalid_argument("unterminated JSON array");
                }
                if (text[p] == ']') {
                    ++p;
                    --depth;
                    phase = Phase::kAfterValue;
                    continue;
                }
                phase = Phase::kValue;
                continue;
            }
            case Phase::kObjectFirst: {
                skip_ws(text, p);
                if (p >= text.size()) {
                    throw std::invalid_argument("unterminated JSON object");
                }
                if (text[p] == '}') {
                    ++p;
                    --depth;
                    phase = Phase::kAfterValue;
                    continue;
                }
                phase = Phase::kMember;
                continue;
            }
            case Phase::kMember: {
                skip_ws(text, p);
                if (p >= text.size() || text[p] != '"') {
                    throw std::invalid_argument("expected JSON object member key");
                }
                std::string key;
                if (!parse_json_string(text, p, key)) {
                    throw std::invalid_argument("invalid JSON object member key");
                }
                skip_ws(text, p);
                if (p >= text.size() || text[p] != ':') {
                    throw std::invalid_argument("expected ':' after JSON object member key");
                }
                ++p;
                stack[depth - 1].pending_key = std::move(key);
                stack[depth - 1].has_key = true;
                phase = Phase::kValue;
                continue;
            }
            case Phase::kAfterValue: {
                if (depth == 0) {
                    if (!have_root) {
                        throw std::invalid_argument("missing JSON root value");
                    }
                    skip_ws(text, p);
                    if (p != text.size()) {
                        throw std::invalid_argument("trailing characters after JSON root value");
                    }
                    return doc;
                }
                skip_ws(text, p);
                if (p >= text.size()) {
                    throw std::invalid_argument("unterminated JSON container");
                }
                const char c = text[p];
                const Frame &top = stack[depth - 1];
                const char expected_close = top.is_object ? '}' : ']';
                if (c == expected_close) {
                    ++p;
                    --depth;
                    continue;
                }
                if (c == ',') {
                    ++p;
                    phase = top.is_object ? Phase::kMember : Phase::kValue;
                    continue;
                }
                throw std::invalid_argument("unexpected character in JSON container");
            }
        }
    }
}

const JsonNode &require_member(const JsonDoc &doc, const JsonNode &obj, std::string_view key) {
    if (obj.kind != JsonKind::kObject) {
        throw std::invalid_argument("expected JSON object");
    }
    for (const auto &[k, idx] : obj.members) {
        if (k == key) {
            return doc.nodes[idx];
        }
    }
    throw std::invalid_argument(std::string("missing JSON field: ") + std::string(key));
}

double require_number(const JsonNode &n) {
    if (n.kind != JsonKind::kNumber || !std::isfinite(n.number)) {
        throw std::invalid_argument("expected finite JSON number");
    }
    return n.number;
}

int require_direction(const JsonNode &n) {
    const double v = require_number(n);
    if (v == 1.0) {
        return 1;
    }
    if (v == -1.0) {
        return -1;
    }
    throw std::invalid_argument("JSON direction must be exactly 1 or -1");
}

std::vector<double> require_number_array(const JsonDoc &doc, const JsonNode &n) {
    if (n.kind != JsonKind::kArray) {
        throw std::invalid_argument("expected JSON array");
    }
    std::vector<double> out;
    out.reserve(n.elems.size());
    for (const std::size_t idx : n.elems) {
        out.push_back(require_number(doc.nodes[idx]));
    }
    return out;
}

TorqueCurveInput parse_curve_node(const JsonDoc &doc, const JsonNode &node) {
    return TorqueCurveInput{
        .angles = require_number_array(doc, require_member(doc, node, "angles_rad")),
        .torques = require_number_array(doc, require_member(doc, node, "torques_nm")),
        .vmax = require_number(require_member(doc, node, "vmax_rad_s")),
        .hill_c = require_number(require_member(doc, node, "hill_c")),
        .eccentric_ratio = require_number(require_member(doc, node, "eccentric_ratio")),
    };
}

std::string read_text_file(const std::filesystem::path &path) {
    const std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) {
        throw std::invalid_argument("failed to open spindle JSON file: " + path.string());
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

SpindleReferenceTables parse_spindle_reference_json(
    const SpindleReferenceJsonTexts &texts
) {
    const JsonDoc s_doc = parse_json_doc(texts.strength_json);
    const JsonDoc e_doc = parse_json_doc(texts.envelope_json);
    const JsonNode &s_joints = require_member(s_doc, s_doc.nodes[s_doc.root], "joints");
    const JsonNode &e_joints = require_member(e_doc, e_doc.nodes[e_doc.root], "joints");

    SpindleReferenceTables out{};
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumSpindleJoints); ++i) {
        const char *jname = kSpindleJointNames[i];
        const JsonNode &ej = require_member(e_doc, e_joints, jname);
        out.envelopes[i].direction = require_direction(require_member(e_doc, ej, "direction"));
        out.envelopes[i].minimum_anatomical_rad =
            require_number(require_member(e_doc, ej, "minimum_anatomical_rad"));
        out.envelopes[i].maximum_anatomical_rad =
            require_number(require_member(e_doc, ej, "maximum_anatomical_rad"));

        const JsonNode &sj = require_member(s_doc, s_joints, jname);
        const JsonNode &coord = require_member(s_doc, sj, "coordinate");
        out.strength[i].direction = require_direction(require_member(s_doc, coord, "direction"));
        const JsonNode &dirs = require_member(s_doc, sj, "directions");
        out.strength[i].positive = parse_curve_node(s_doc, require_member(s_doc, dirs, "positive"));
        out.strength[i].negative = parse_curve_node(s_doc, require_member(s_doc, dirs, "negative"));
    }
    return out;
}

SpindleReferenceTables load_spindle_reference_files(
    const std::filesystem::path &strength_json_path,
    const std::filesystem::path &envelope_json_path
) {
    const std::string strength_text = read_text_file(strength_json_path);
    const std::string envelope_text = read_text_file(envelope_json_path);
    return parse_spindle_reference_json(SpindleReferenceJsonTexts{
        .strength_json = strength_text,
        .envelope_json = envelope_text,
    });
}

template SpindleM2K2Params<float> build_spindle_m2k2_params_from_tables<float>(
    const mjModel *m,
    const mjData *d_qpos0,
    const SpindleReferenceTables &tables,
    const SpindleM2K2ScalarConfig &cfg);

template SpindleM2K2Params<double> build_spindle_m2k2_params_from_tables<double>(
    const mjModel *m,
    const mjData *d_qpos0,
    const SpindleReferenceTables &tables,
    const SpindleM2K2ScalarConfig &cfg);

template SpindleM2K2Params<float> build_spindle_m2k2_params_from_json<float>(
    const mjModel *m,
    const mjData *d_qpos0,
    const SpindleReferenceJsonTexts &texts,
    const SpindleM2K2ScalarConfig &cfg);

template SpindleM2K2Params<double> build_spindle_m2k2_params_from_json<double>(
    const mjModel *m,
    const mjData *d_qpos0,
    const SpindleReferenceJsonTexts &texts,
    const SpindleM2K2ScalarConfig &cfg);

template SpindleM2K2Params<float> build_spindle_m2k2_params_from_json_files<float>(
    const mjModel *m,
    const mjData *d_qpos0,
    const std::filesystem::path &strength_json_path,
    const std::filesystem::path &envelope_json_path,
    const SpindleM2K2ScalarConfig &cfg);

template SpindleM2K2Params<double> build_spindle_m2k2_params_from_json_files<double>(
    const mjModel *m,
    const mjData *d_qpos0,
    const std::filesystem::path &strength_json_path,
    const std::filesystem::path &envelope_json_path,
    const SpindleM2K2ScalarConfig &cfg);

} // namespace bike::physics_core
