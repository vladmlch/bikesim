// native/src/gpu/model_assets.hpp — host-only loaders for side-car model assets shared by the
// CPU and Metal batch runners: hfield_data.bin, the 1D heightfield extraction, and the captured
// bootstrap initial state. Must never be included from MSL or CUDA device code.
#pragma once

#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__)
#error "model_assets.hpp is host-only"
#endif

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "../io/value.hpp"
#include "common.hpp"

namespace bike::gpu::assets {

enum class AssetStatus : std::int8_t {
    kOk = 0,
    kMissingFile = -10,
    kSizeMismatch = -11,
    kReadError = -12,
    kParseError = -13,
    kDimMismatch = -14,
    kNoHeightfield = -15,
};

static_assert(std::endian::native == std::endian::little,
              "hfield_data.bin is little-endian binary64; add a byte swap before enabling a big-endian host");
static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8,
              "hfield_data.bin requires IEEE-754 binary64");

inline constexpr const char *kHfieldBinName = "hfield_data.bin";
inline constexpr const char *kCapturedBootstrapName = "captured_bootstrap.json";

inline const char *asset_status_name(AssetStatus s) noexcept {
    switch (s) {
        case AssetStatus::kOk: return "ok";
        case AssetStatus::kMissingFile: return "missing_file";
        case AssetStatus::kSizeMismatch: return "size_mismatch";
        case AssetStatus::kReadError: return "read_error";
        case AssetStatus::kParseError: return "parse_error";
        case AssetStatus::kDimMismatch: return "dim_mismatch";
        case AssetStatus::kNoHeightfield: return "no_heightfield";
    }
    return "unknown";
}

// Directory containing xml_path ("" or a bare file name -> ".").
inline std::string asset_dir(const std::string &xml_path) {
    const std::filesystem::path parent = std::filesystem::path(xml_path).parent_path();
    if (parent.empty()) {
        return ".";
    }
    return parent.string();
}

namespace detail {

// The single raw mjModel pointer -> std::span boundary. MuJoCo owns these arrays for
// the model's lifetime and sizes them by the adjacent model counts (nhfield, ngeom,
// nhfielddata); every access range in this file is wrapped once through this helper.
// The (pointer, count) construction itself is -Wunsafe-buffer-usage-in-container —
// suppressed here, scoped exactly like native/src/platform/argv.hpp. Register:
// docs/TESTING.md suppression table.
template <typename T>
[[nodiscard]] inline std::span<T> model_span(T *ptr, std::size_t count) noexcept {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    return std::span<T>(ptr, count);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
}

inline std::filesystem::path asset_path(const std::string &xml_path, const char *name) {
    return std::filesystem::path(asset_dir(xml_path)) / name;
}

// kOk with the byte size of an existing regular file; kMissingFile when absent; kReadError otherwise.
inline AssetStatus regular_file_size(const std::filesystem::path &p, std::uintmax_t &size) {
    std::error_code ec;
    const std::filesystem::file_status st = std::filesystem::status(p, ec);
    if (st.type() == std::filesystem::file_type::not_found) {
        return AssetStatus::kMissingFile;
    }
    if (ec || !std::filesystem::is_regular_file(st)) {
        return AssetStatus::kReadError;
    }
    size = std::filesystem::file_size(p, ec);
    return ec ? AssetStatus::kReadError : AssetStatus::kOk;
}

// Reads exactly dst.size() bytes into dst. kReadError on open failure or short read.
inline AssetStatus read_exact(const std::filesystem::path &p, std::span<char> dst) {
    std::ifstream f(p, std::ios::binary);
    if (!f.is_open()) {
        return AssetStatus::kReadError;
    }
    f.read(dst.data(), static_cast<std::streamsize>(dst.size()));
    return (f.gcount() == static_cast<std::streamsize>(dst.size())) ? AssetStatus::kOk
                                                                  : AssetStatus::kReadError;
}

// ---- Strict JSON scanner (RFC 8259 grammar). NaN/Infinity are rejected, as are non-finite numbers. ----

inline constexpr int kMaxJsonDepth = 64;

inline void skip_ws(const std::string &t, std::size_t &p) noexcept {
    while (p < t.size() && (t[p] == ' ' || t[p] == '\t' || t[p] == '\n' || t[p] == '\r')) {
        ++p;
    }
}

inline bool is_digit(char c) noexcept {
    return c >= '0' && c <= '9';
}

// Scans one string token starting at the opening quote. Decodes into *out when out != nullptr.
// Non-ASCII \u escapes decode to '\0', which never matches an ASCII key we look up.
inline bool scan_string(const std::string &t, std::size_t &p, std::string *out) {
    if (p >= t.size() || t[p] != '"') {
        return false;
    }
    ++p;
    while (p < t.size()) {
        const auto c = static_cast<unsigned char>(t[p]);
        if (c == '"') {
            ++p;
            return true;
        }
        if (c < 0x20) {
            return false;
        }
        if (c != '\\') {
            if (out != nullptr) {
                out->push_back(static_cast<char>(c));
            }
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
        if (out != nullptr) {
            out->push_back(decoded);
        }
    }
    return false;  // unterminated
}

// Scans one number token with grammar -?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?.
inline bool scan_number(const std::string &t, std::size_t &p) {
    if (p < t.size() && t[p] == '-') {
        ++p;
    }
    if (p >= t.size() || !is_digit(t[p])) {
        return false;
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
            return false;
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
            return false;
        }
        while (p < t.size() && is_digit(t[p])) {
            ++p;
        }
    }
    return true;
}

inline bool match_literal(const std::string &t, std::size_t &p, std::string_view lit) {
    if (!std::string_view{t}.substr(p).starts_with(lit)) {
        return false;
    }
    p += lit.size();
    return true;
}

// Scans one JSON value starting at p (after whitespace) and leaves p just past it.
// The grammar, the kMaxJsonDepth bound and the accepted/rejected inputs are exactly
// those of the former scan_value/scan_array/scan_object recursion; the descent is
// iterative on an explicit stack (no recursive call chain). Each call starts a
// fresh depth budget, as the recursive form did for scan_value(t, p, 0).
inline bool scan_value(const std::string &t, std::size_t &p) {
    enum class Phase : std::uint8_t {
        kValue,        // a value is required at the current position
        kArrayFirst,   // right after '[': ']' (empty) or the first element
        kObjectFirst,  // right after '{': '}' (empty) or the first member key
        kMember,       // after ',' in an object, or kObjectFirst without '}': a key and ':'
        kAfterValue,   // a value or container just closed: the frame's close or ','
    };
    struct Frame {
        char close;          // ']' or '}'
        Phase after_comma;   // kValue (array element) or kMember (object member)
    };
    std::array<Frame, kMaxJsonDepth> frames{};
    std::size_t depth = 0;
    Phase phase = Phase::kValue;
    for (;;) {
        switch (phase) {
            case Phase::kValue: {
                skip_ws(t, p);
                if (p >= t.size()) {
                    return false;
                }
                const char c = t[p];
                if (c == '{' || c == '[') {
                    if (depth >= kMaxJsonDepth) {
                        return false;
                    }
                    const bool object = (c == '{');
                    frames[depth] = Frame{.close = object ? '}' : ']',
                                          .after_comma = object ? Phase::kMember : Phase::kValue};
                    ++depth;
                    ++p;
                    phase = object ? Phase::kObjectFirst : Phase::kArrayFirst;
                    continue;
                }
                bool ok = true;
                switch (c) {
                    case '"': ok = scan_string(t, p, nullptr); break;
                    case 't': ok = match_literal(t, p, "true"); break;
                    case 'f': ok = match_literal(t, p, "false"); break;
                    case 'n': ok = match_literal(t, p, "null"); break;
                    default: ok = scan_number(t, p); break;
                }
                if (!ok) {
                    return false;
                }
                phase = Phase::kAfterValue;
                continue;
            }
            case Phase::kArrayFirst: {
                skip_ws(t, p);
                if (p < t.size() && t[p] == ']') {
                    ++p;
                    --depth;
                    phase = Phase::kAfterValue;
                    continue;
                }
                phase = Phase::kValue;
                continue;
            }
            case Phase::kObjectFirst: {
                skip_ws(t, p);
                if (p < t.size() && t[p] == '}') {
                    ++p;
                    --depth;
                    phase = Phase::kAfterValue;
                    continue;
                }
                phase = Phase::kMember;
                continue;
            }
            case Phase::kMember: {
                skip_ws(t, p);
                if (!scan_string(t, p, nullptr)) {
                    return false;
                }
                skip_ws(t, p);
                if (p >= t.size() || t[p] != ':') {
                    return false;
                }
                ++p;
                phase = Phase::kValue;
                continue;
            }
            case Phase::kAfterValue: {
                if (depth == 0) {
                    return true;  // the root value is complete
                }
                skip_ws(t, p);
                if (p >= t.size()) {
                    return false;
                }
                const Frame top = frames[depth - 1];
                if (t[p] == top.close) {
                    ++p;
                    --depth;
                    continue;  // the closed container is itself a completed value
                }
                if (t[p] != ',') {
                    return false;
                }
                ++p;
                phase = top.after_comma;
                continue;
            }
        }
    }
}

// Finds member `key` of the object at obj_pos in a document already validated by scan_value.
// value_pos receives the position of the member's value (after whitespace). kParseError when the
// value is not an object, the key is absent, or the key occurs more than once.
inline AssetStatus find_member(const std::string &t, std::size_t obj_pos, std::string_view key,
                               std::size_t &value_pos) {
    std::size_t p = obj_pos;
    skip_ws(t, p);
    if (p >= t.size() || t[p] != '{') {
        return AssetStatus::kParseError;
    }
    ++p;
    skip_ws(t, p);
    if (p < t.size() && t[p] == '}') {
        return AssetStatus::kParseError;
    }
    bool found = false;
    std::size_t found_pos = 0;
    for (;;) {
        skip_ws(t, p);
        std::string name;
        if (!scan_string(t, p, &name)) {
            return AssetStatus::kParseError;
        }
        skip_ws(t, p);
        if (p >= t.size() || t[p] != ':') {
            return AssetStatus::kParseError;
        }
        ++p;
        skip_ws(t, p);
        if (name == key) {
            if (found) {
                return AssetStatus::kParseError;
            }
            found = true;
            found_pos = p;
        }
        if (!scan_value(t, p)) {
            return AssetStatus::kParseError;
        }
        skip_ws(t, p);
        if (p >= t.size()) {
            return AssetStatus::kParseError;
        }
        if (t[p] == '}') {
            break;
        }
        if (t[p] != ',') {
            return AssetStatus::kParseError;
        }
        ++p;
    }
    if (!found) {
        return AssetStatus::kParseError;
    }
    value_pos = found_pos;
    return AssetStatus::kOk;
}

// Integer JSON token without fraction or exponent, fitting in int.
inline AssetStatus read_int_token(const std::string &t, std::size_t pos, int &out) {
    std::size_t p = pos;
    if (!scan_number(t, p)) {
        return AssetStatus::kParseError;
    }
    const std::string_view tok = std::string_view{t}.substr(pos, p - pos);
    if (tok.find_first_of(".eE") != std::string_view::npos) {
        return AssetStatus::kParseError;
    }
    long long v = 0;
    const char *const first = tok.data();
    const char *const last = std::to_address(tok.cend());
    const std::from_chars_result parsed = std::from_chars(first, last, v, 10);
    if (parsed.ec != std::errc{} || parsed.ptr != last ||
        v < static_cast<long long>(std::numeric_limits<int>::min()) ||
        v > static_cast<long long>(std::numeric_limits<int>::max())) {
        return AssetStatus::kParseError;
    }
    out = static_cast<int>(v);
    return AssetStatus::kOk;
}

// Number token parsed through the shared from_chars path: correctly rounded, with
// overflow mapping to +/-inf and underflow to a signed 0.0 — the same results strtod
// produced. Rejects non-finite results, which Python's json would accept as inf.
inline std::pair<AssetStatus, double> read_double_token(const std::string &t, std::size_t &p) {
    skip_ws(t, p);
    const std::size_t start = p;
    if (!scan_number(t, p)) {
        return {AssetStatus::kParseError, 0.0};
    }
    const std::string_view tok = std::string_view{t}.substr(start, p - start);
    bool ok = false;
    const double v = bike::io::detail::float_from_lexeme(tok, ok);
    if (!ok || !std::isfinite(v)) {
        return {AssetStatus::kParseError, 0.0};
    }
    return {AssetStatus::kOk, v};
}

}  // namespace detail

// If m->nhfielddata == 0: returns kOk and touches nothing. Otherwise reads
// asset_dir(xml_path)/hfield_data.bin, which must be exactly nhfielddata binary64 values,
// into m->hfield_data. On any failure m->hfield_data is left unmodified.
// A null model reports kNoHeightfield.
// `m` is const-qualified on purpose: no mjModel field is modified — only the
// contents of the hfield_data buffer the model already owns are filled.
inline AssetStatus load_hfield_bin(const mjModel *m, const std::string &xml_path) {
    if (m == nullptr) {
        return AssetStatus::kNoHeightfield;
    }
    if (m->nhfielddata == 0) {
        return AssetStatus::kOk;
    }
    if (m->nhfielddata < 0) {
        return AssetStatus::kSizeMismatch;
    }
    try {
        const auto count = static_cast<std::size_t>(m->nhfielddata);
        const std::uintmax_t expected_bytes = count * sizeof(double);
        const std::filesystem::path p = detail::asset_path(xml_path, kHfieldBinName);

        std::uintmax_t actual_bytes = 0;
        AssetStatus s = detail::regular_file_size(p, actual_bytes);
        if (s != AssetStatus::kOk) {
            return s;
        }
        if (actual_bytes != expected_bytes) {
            return AssetStatus::kSizeMismatch;
        }
        // ifstream fills char storage; the byte copy into the doubles buffer runs
        // span-to-span over std::byte views — no pointer reinterpretation.
        std::vector<char> raw(count * sizeof(double));
        s = detail::read_exact(p, std::span{raw});
        if (s != AssetStatus::kOk) {
            return s;
        }
        std::vector<double> buf(count);
        const std::span<const std::byte> src = std::as_bytes(std::span{raw});
        const std::span<std::byte> dst = std::as_writable_bytes(std::span{buf});
        std::ranges::copy(src, dst.begin());
        // MuJoCo stores hfield_data as float (mjtNum is double); round exactly as the runners do.
        const std::span<float> hfield = detail::model_span(m->hfield_data, count);
        for (std::size_t i = 0; i < count; ++i) {
            hfield[i] = static_cast<float>(buf[i]);
        }
        return AssetStatus::kOk;
    } catch (...) {
        return AssetStatus::kReadError;
    }
}

struct Heightfield1D {
    std::vector<float> heights;
    float x_min_m = 0.0f;
    float dx_m = 0.0f;
};

// Bit-identical to the rollout-path extraction in cpu_batch_runner.cpp (bike_cpu_run_batch):
// hfield 0, first ncol samples, height = hfield_data[i] * hfield_size[2] + z of the first
// mjGEOM_HFIELD geom; x_min = geom_x - hfield_size[0]; dx = total_len_x / max(ncol - 1, 1) with
// total_len_x = 2 * hfield_size[0], or (ncol - 1) * 0.005f when hfield_size[0] <= 0.
// kNoHeightfield if the model has none; kSizeMismatch if ncol is not in (0, nhfielddata].
// `out` is untouched on failure.
inline AssetStatus extract_heightfield_1d(const mjModel *m, Heightfield1D &out) {
    if (m == nullptr || m->nhfield <= 0 || m->nhfielddata <= 0) {
        return AssetStatus::kNoHeightfield;
    }
    try {
        const std::span<const int> hfield_ncol =
            detail::model_span(m->hfield_ncol, static_cast<std::size_t>(m->nhfield));
        const int ncol = hfield_ncol[0];
        if (ncol <= 0 || ncol > m->nhfielddata) {
            return AssetStatus::kSizeMismatch;
        }
        const std::span<const mjtNum> hfield_size =
            detail::model_span(m->hfield_size, 4 * static_cast<std::size_t>(m->nhfield));
        const float size_x_half = static_cast<float>(hfield_size[0]);
        const float scale_z = static_cast<float>(hfield_size[2]);
        float geom_x_min = 0.0f;
        float z_offset = 0.0f;
        const std::span<const int> geom_type =
            detail::model_span(m->geom_type, static_cast<std::size_t>(m->ngeom));
        const std::span<const mjtNum> geom_pos =
            detail::model_span(m->geom_pos, 3 * static_cast<std::size_t>(m->ngeom));
        for (std::size_t g = 0; std::cmp_less(g, m->ngeom); ++g) {
            if (geom_type[g] == mjGEOM_HFIELD) {
                geom_x_min = static_cast<float>(geom_pos[3 * g + 0]) - size_x_half;
                z_offset = static_cast<float>(geom_pos[3 * g + 2]);
                break;
            }
        }
        Heightfield1D result;
        result.heights.resize(static_cast<std::size_t>(ncol));
        const std::span<const float> hfield_data =
            detail::model_span(m->hfield_data, static_cast<std::size_t>(m->nhfielddata));
        for (int i = 0; i < ncol; ++i) {
            result.heights[static_cast<std::size_t>(i)] =
                static_cast<float>(hfield_data[static_cast<std::size_t>(i)]) * scale_z + z_offset;
        }
        const float total_len_x =
            (size_x_half > 0.0f) ? (2.0f * size_x_half) : (static_cast<float>(ncol - 1) * 0.005f);
        result.x_min_m = geom_x_min;
        result.dx_m = total_len_x / static_cast<float>(std::max(ncol - 1, 1));
        out = std::move(result);
        return AssetStatus::kOk;
    } catch (...) {
        return AssetStatus::kReadError;
    }
}

// Reads asset_dir(xml_path)/captured_bootstrap.json. Checks state.model_dims nq/nv equal the
// arguments (else kDimMismatch), then fills qpos (nq) from state.integration_state[1 .. 1+nq) and
// qvel (nv) from [1+nq .. 1+nq+nv). integration_state[0] (time) is not returned.
// Outputs are untouched on failure.
inline AssetStatus load_captured_initial_state(const std::string &xml_path, int nq, int nv,
                                               std::vector<double> &qpos, std::vector<double> &qvel) {
    try {
        if (nq < 0 || nv < 0) {
            return AssetStatus::kDimMismatch;
        }
        const std::filesystem::path json_path = detail::asset_path(xml_path, kCapturedBootstrapName);
        std::uintmax_t size = 0;
        AssetStatus s = detail::regular_file_size(json_path, size);
        if (s != AssetStatus::kOk) {
            return s;
        }
        std::string text(static_cast<std::size_t>(size), '\0');
        s = detail::read_exact(json_path, std::span{text});
        if (s != AssetStatus::kOk) {
            return s;
        }

        std::size_t pos = 0;
        if (!detail::scan_value(text, pos)) {
            return AssetStatus::kParseError;
        }
        detail::skip_ws(text, pos);
        if (pos != text.size()) {
            return AssetStatus::kParseError;
        }

        std::size_t state_pos = 0;
        s = detail::find_member(text, 0, "state", state_pos);
        if (s != AssetStatus::kOk) {
            return s;
        }
        std::size_t dims_pos = 0;
        s = detail::find_member(text, state_pos, "model_dims", dims_pos);
        if (s != AssetStatus::kOk) {
            return s;
        }
        std::size_t nq_pos = 0;
        std::size_t nv_pos = 0;
        s = detail::find_member(text, dims_pos, "nq", nq_pos);
        if (s != AssetStatus::kOk) {
            return s;
        }
        s = detail::find_member(text, dims_pos, "nv", nv_pos);
        if (s != AssetStatus::kOk) {
            return s;
        }
        int file_nq = 0;
        int file_nv = 0;
        s = detail::read_int_token(text, nq_pos, file_nq);
        if (s != AssetStatus::kOk) {
            return s;
        }
        s = detail::read_int_token(text, nv_pos, file_nv);
        if (s != AssetStatus::kOk) {
            return s;
        }
        if (file_nq != nq || file_nv != nv) {
            return AssetStatus::kDimMismatch;
        }

        std::size_t arr_pos = 0;
        s = detail::find_member(text, state_pos, "integration_state", arr_pos);
        if (s != AssetStatus::kOk) {
            return s;
        }
        if (text[arr_pos] != '[') {
            return AssetStatus::kParseError;
        }

        // Every element must be a number (the whole array is checked); only the needed prefix is kept.
        const std::size_t need = 1u + static_cast<std::size_t>(nq) + static_cast<std::size_t>(nv);
        std::vector<double> values;
        values.reserve(need);
        std::size_t p = arr_pos + 1;
        detail::skip_ws(text, p);
        if (p < text.size() && text[p] == ']') {
            ++p;
        } else {
            for (;;) {
                const std::pair<AssetStatus, double> token = detail::read_double_token(text, p);
                if (token.first != AssetStatus::kOk) {
                    return token.first;
                }
                if (values.size() < need) {
                    values.push_back(token.second);
                }
                detail::skip_ws(text, p);
                if (p >= text.size()) {
                    return AssetStatus::kParseError;
                }
                if (text[p] == ']') {
                    break;
                }
                if (text[p] != ',') {
                    return AssetStatus::kParseError;
                }
                ++p;
            }
        }
        if (values.size() < need) {
            return AssetStatus::kParseError;
        }

        const std::span<const double> v{values};
        const std::span<const double> qpos_range = v.subspan(1, static_cast<std::size_t>(nq));
        const std::span<const double> qvel_range =
            v.subspan(1 + static_cast<std::size_t>(nq), static_cast<std::size_t>(nv));
        qpos.assign(qpos_range.begin(), qpos_range.end());
        qvel.assign(qvel_range.begin(), qvel_range.end());
        return AssetStatus::kOk;
    } catch (...) {
        return AssetStatus::kReadError;
    }
}

namespace detail {

inline int lookup_body_id(const mjModel *m, std::span<const char *const> names, int fallback) {
    for (const char *name : names) {
        const int id = mj_name2id(m, mjOBJ_BODY, name);
        if (id >= 0) {
            return id;
        }
    }
    return fallback;
}

struct JointLookupResult {
    int jid = -1;
    int qpos_adr = -1;
    int dof_adr = -1;
};

inline JointLookupResult lookup_joint_addrs(
    const mjModel *m,
    std::span<const int> jnt_qposadr,
    std::span<const int> jnt_dofadr,
    std::span<const char *const> names,
    JointLookupResult fallback
) {
    for (const char *name : names) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, name);
        if (jid >= 0 && std::cmp_less(jid, jnt_qposadr.size()) &&
            std::cmp_less(jid, jnt_dofadr.size())) {
            return JointLookupResult{
                .jid = jid,
                .qpos_adr = jnt_qposadr[static_cast<std::size_t>(jid)],
                .dof_adr = jnt_dofadr[static_cast<std::size_t>(jid)],
            };
        }
    }
    return fallback;
}

struct ActuatorLookupTarget {
    int target_jid = -1;
    int fallback = -1;
};

inline int lookup_actuator_id(
    const mjModel *m,
    std::span<const int> actuator_trnid,
    std::span<const char *const> names,
    ActuatorLookupTarget target
) {
    int aid = -1;
    for (const char *name : names) {
        const int found = mj_name2id(m, mjOBJ_ACTUATOR, name);
        if (found >= 0) {
            aid = found;
            break;
        }
    }
    if (target.target_jid >= 0) {
        const std::size_t n_act = actuator_trnid.size() / 2;
        if (aid >= 0 && std::cmp_less(aid, n_act) &&
            actuator_trnid[2 * static_cast<std::size_t>(aid)] == target.target_jid) {
            return aid;
        }
        for (std::size_t candidate = 0; candidate < n_act; ++candidate) {
            if (actuator_trnid[2 * candidate] == target.target_jid) {
                return static_cast<int>(candidate);
            }
        }
    }
    return (aid >= 0) ? aid : target.fallback;
}

inline int lookup_tendon_id(const mjModel *m, const char *name, int fallback) {
    const int tid = mj_name2id(m, mjOBJ_TENDON, name);
    return (tid >= 0) ? tid : fallback;
}

}  // namespace detail

// Dynamically resolves GpuModelIndices from a compiled mjModel, supporting both
// nq = 27 clutch models (welded.xml) and legacy nq = 26 models (model.xml, where
// drive_shaft_spin_qpos / drive_shaft_spin_dof / crank_clutch_tendon resolve to -1).
inline bike::gpu::GpuModelIndices resolve_gpu_model_indices(const mjModel *m) {
    bike::gpu::GpuModelIndices idx{};
    if (m == nullptr) {
        return idx;
    }
    idx.nq = static_cast<int>(m->nq);
    idx.nv = static_cast<int>(m->nv);
    idx.nu = static_cast<int>(m->nu);
    idx.nbody = static_cast<int>(m->nbody);

    const std::span<const int> jnt_qposadr =
        detail::model_span(m->jnt_qposadr, static_cast<std::size_t>(std::max<mjtSize>(0, m->njnt)));
    const std::span<const int> jnt_dofadr =
        detail::model_span(m->jnt_dofadr, static_cast<std::size_t>(std::max<mjtSize>(0, m->njnt)));
    const std::span<const int> actuator_trnid =
        detail::model_span(m->actuator_trnid, 2 * static_cast<std::size_t>(std::max<mjtSize>(0, m->nu)));

    constexpr std::array<const char *, 3> kFrameNames{"frame", "main_frame", "chassis"};
    constexpr std::array<const char *, 3> kSteerNames{"steer", "fork_upper", "steering"};
    constexpr std::array<const char *, 2> kForkLowerNames{"fork_lower", "front_fork_lower"};
    constexpr std::array<const char *, 2> kFrontWheelNames{"front_wheel", "wheel_front"};
    constexpr std::array<const char *, 2> kChainstayNames{"chainstay", "rear_arm"};
    constexpr std::array<const char *, 2> kSeatstayNames{"seatstay", "rear_seatstay"};
    constexpr std::array<const char *, 2> kRearWheelNames{"rear_wheel", "wheel_rear"};

    idx.frame_body = detail::lookup_body_id(m, kFrameNames, idx.frame_body);
    idx.steer_body = detail::lookup_body_id(m, kSteerNames, idx.steer_body);
    idx.fork_lower_body = detail::lookup_body_id(m, kForkLowerNames, idx.fork_lower_body);
    idx.front_wheel_body = detail::lookup_body_id(m, kFrontWheelNames, idx.front_wheel_body);
    idx.chainstay_body = detail::lookup_body_id(m, kChainstayNames, idx.chainstay_body);
    idx.seatstay_body = detail::lookup_body_id(m, kSeatstayNames, idx.seatstay_body);
    idx.rear_wheel_body = detail::lookup_body_id(m, kRearWheelNames, idx.rear_wheel_body);

    const auto lookup1 = [&](const char *name, int def_q, int def_d) {
        const std::array<const char *, 1> names{name};
        return detail::lookup_joint_addrs(
            m, jnt_qposadr, jnt_dofadr, names,
            detail::JointLookupResult{.jid = -1, .qpos_adr = def_q, .dof_adr = def_d});
    };

    const auto rx = lookup1("root_x", idx.root_x_qpos, idx.root_x_dof);
    const auto rz = lookup1("root_z", idx.root_z_qpos, idx.root_z_dof);
    const auto rp = lookup1("root_pitch", idx.root_pitch_qpos, idx.root_pitch_dof);
    const auto cs = lookup1("crank_spin", idx.crank_spin_qpos, idx.crank_spin_dof);
    const auto ft = lookup1("fork_travel", idx.fork_travel_qpos, idx.fork_travel_dof);
    const auto fw = lookup1("front_wheel_spin", idx.front_wheel_spin_qpos, idx.front_wheel_spin_dof);
    const auto rw = lookup1("rear_wheel_spin", idx.rear_wheel_spin_qpos, idx.rear_wheel_spin_dof);
    const auto sh = lookup1("shock_stroke", idx.shock_stroke_qpos, idx.shock_stroke_dof);
    const auto ds = lookup1("drive_shaft_spin", -1, -1);
    const auto mp = lookup1("main_pivot", -1, idx.main_pivot_dof);
    const auto hp = lookup1("horst_pivot", -1, idx.horst_pivot_dof);

    idx.root_x_qpos = rx.qpos_adr; idx.root_x_dof = rx.dof_adr;
    idx.root_z_qpos = rz.qpos_adr; idx.root_z_dof = rz.dof_adr;
    idx.root_pitch_qpos = rp.qpos_adr; idx.root_pitch_dof = rp.dof_adr;
    idx.crank_spin_qpos = cs.qpos_adr; idx.crank_spin_dof = cs.dof_adr;
    idx.fork_travel_qpos = ft.qpos_adr; idx.fork_travel_dof = ft.dof_adr;
    idx.front_wheel_spin_qpos = fw.qpos_adr; idx.front_wheel_spin_dof = fw.dof_adr;
    idx.rear_wheel_spin_qpos = rw.qpos_adr; idx.rear_wheel_spin_dof = rw.dof_adr;
    idx.shock_stroke_qpos = sh.qpos_adr; idx.shock_stroke_dof = sh.dof_adr;
    idx.drive_shaft_spin_qpos = ds.qpos_adr; idx.drive_shaft_spin_dof = ds.dof_adr;
    idx.main_pivot_dof = mp.dof_adr;
    idx.horst_pivot_dof = hp.dof_adr;

    idx.freehub_tendon = detail::lookup_tendon_id(m, "ideal_mid_drive_freehub", -1);
    idx.crank_clutch_tendon = detail::lookup_tendon_id(m, "crank_clutch", -1);

    constexpr std::array<std::array<const char *, 5>, bike::gpu::kNumRiderJoints> kJointCands{{
        {{"rider_hip_front", "rider_hip", "hip_pitch", "rider_hip_pitch", "hip"}},
        {{"rider_knee_front", "rider_knee", "knee_pitch", "rider_knee_pitch", "knee"}},
        {{"rider_hip_rear", "rider_ankle_front", "rider_ankle", "ankle_pitch", "ankle"}},
        {{"rider_knee_rear", nullptr, nullptr, nullptr, nullptr}},
        {{"rider_torso_hinge", "rider_waist_pitch", "waist_pitch", "torso_pitch", "waist"}},
        {{"rider_shoulder_left", nullptr, nullptr, nullptr, nullptr}},
        {{"rider_elbow_left", nullptr, nullptr, nullptr, nullptr}},
        {{"rider_shoulder_right", "rider_shoulder_pitch", "shoulder_pitch", "shoulder", nullptr}},
        {{"rider_elbow_right", "rider_elbow", "elbow_pitch", "rider_elbow_pitch", "elbow"}},
    }};
    constexpr std::array<std::array<const char *, 5>, bike::gpu::kNumRiderJoints> kActCands{{
        {{"act_rider_hip_front", "rider_hip_motor", "act_rider_hip", "hip_motor", "hip"}},
        {{"act_rider_knee_front", "rider_knee_motor", "act_rider_knee", "knee_motor", "knee"}},
        {{"act_rider_hip_rear", "rider_ankle_motor", "act_rider_ankle", "ankle_motor", "ankle"}},
        {{"act_rider_knee_rear", nullptr, nullptr, nullptr, nullptr}},
        {{"act_rider_torso_hinge", "rider_waist_motor", "act_rider_waist_pitch", "waist_motor", "waist_pitch"}},
        {{"act_rider_shoulder_left", nullptr, nullptr, nullptr, nullptr}},
        {{"act_rider_elbow_left", nullptr, nullptr, nullptr, nullptr}},
        {{"act_rider_shoulder_right", "rider_shoulder_motor", "act_rider_shoulder_pitch", "shoulder_motor", "shoulder_pitch"}},
        {{"act_rider_elbow_right", "rider_elbow_motor", "act_rider_elbow", "elbow_motor", "elbow"}},
    }};

    const std::span<int, bike::gpu::kNumRiderJoints> r_qpos(idx.rider_qpos);
    const std::span<int, bike::gpu::kNumRiderJoints> r_dof(idx.rider_dof);
    const std::span<int, bike::gpu::kNumRiderJoints> r_ctrl(idx.rider_ctrl);
    for (std::size_t j = 0; j < static_cast<std::size_t>(bike::gpu::kNumRiderJoints); ++j) {
        std::vector<const char *> jnames;
        for (const char *s : kJointCands[j]) {
            if (s != nullptr) {
                jnames.push_back(s);
            }
        }
        std::vector<const char *> anames;
        for (const char *s : kActCands[j]) {
            if (s != nullptr) {
                anames.push_back(s);
            }
        }
        const auto res = detail::lookup_joint_addrs(
            m, jnt_qposadr, jnt_dofadr, jnames,
            detail::JointLookupResult{.jid = -1, .qpos_adr = r_qpos[j], .dof_adr = r_dof[j]});
        r_qpos[j] = res.qpos_adr;
        r_dof[j] = res.dof_adr;
        r_ctrl[j] = detail::lookup_actuator_id(
            m, actuator_trnid, anames,
            detail::ActuatorLookupTarget{.target_jid = res.jid, .fallback = r_ctrl[j]});
    }
    return idx;
}

}  // namespace bike::gpu::assets
