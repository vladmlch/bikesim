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

}  // namespace bike::gpu::assets
